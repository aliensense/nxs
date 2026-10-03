# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""A link's frames in your own program, without ROS: `nxs.cam.frames`
(this module's `frames`, exported by the package)."""

from __future__ import annotations

import dataclasses
from typing import Any, Dict, Iterator, Optional, Tuple

from nxs import host as host_layer

from . import port_state
from .contracts import Topology
from .viewers import resolve_capture_hints, source_props


@dataclasses.dataclass(frozen=True)
class Frame:
    """One frame: the image as an HxWx3 uint8 RGB array (one copy out of the
    capture stack's buffer), the pipeline's presentation stamp in
    nanoseconds (-1 when the source stamps none), and the frame's index in
    this session."""

    image: Any
    pts_ns: int
    index: int


def port_topology(port: str) -> Topology:
    """The declared port of that name."""
    from .topology import load_ports

    ports, _ = load_ports(None)
    for topology in ports.values():
        if port_state.port_name(topology) == port:
            return topology
    names = ", ".join(sorted(port_state.port_name(t) for t in ports.values())) or "none"
    raise LookupError(f"no port {port!r} (declared: {names})")


def link_pipeline(port: str, link: str,
                  count: Optional[int] = None) -> Tuple[str, Dict[str, Any]]:
    """The GStreamer description that hands a link's frames to an appsink
    named `sink` as RGBA in system memory, at the caps the port record
    carries for the link and the source properties they give
    (`source_props`), and those caps. `count` bounds the frames the source
    delivers."""
    topology = port_topology(port)
    spec = topology.link(link)
    capture_id = port_state.port_capture_id(topology, spec)
    if capture_id is None:
        raise LookupError(f"{port}/{link}: no capture id in the current port; "
                          f"bring the link up first (nxs {port} {link} on)")
    hints = (port_state.port_record(topology).get("viewers") or {}).get(spec.name)
    resolved = resolve_capture_hints(hints, link)
    host = host_layer.current()
    props = source_props(resolved, host, port=port)
    props = f" {props}" if props else ""
    limit = f" num-buffers={int(count)}" if count else ""
    desc = (f"{host.source(capture_id, resolved['sensor_mode'])}{props}{limit} ! "
            f"{host.caps(resolved['width'], resolved['height'], resolved.get('framerate'))} ! "
            f"{host.rgba_convert()} ! appsink name=sink max-buffers=4 drop=true sync=false")
    return desc, resolved


def frames(port: str, link: str, count: Optional[int] = None) -> Iterator[Frame]:
    """Yield a link's frames from its capture node as they arrive, `count`
    of them or until the caller stops iterating. Needs the host's GStreamer
    Python bindings and numpy; the link must be up (`nxs <port> <link> on`).
    A session the capture stack drops before its first frame is opened once
    more; a second failure is the caller's."""
    for attempt in (1, 2):
        delivered = False
        try:
            for frame in _session(port, link, count):
                delivered = True
                yield frame
            return
        except RuntimeError:
            if delivered or attempt == 2:
                raise


def _session(port: str, link: str, count: Optional[int]) -> Iterator[Frame]:
    """One capture session on the link's node, ended by `count`, an end of
    stream, or an error (a RuntimeError)."""
    try:
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("nxs.cam.frames needs numpy") from exc
    try:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
    except (ImportError, ValueError) as exc:
        raise RuntimeError("nxs.cam.frames needs GStreamer's Python bindings "
                           "(python3-gi and gir1.2-gstreamer-1.0)") from exc
    desc, resolved = link_pipeline(port, link, count)
    width, height = int(resolved["width"]), int(resolved["height"])
    Gst.init(None)
    pipeline = Gst.parse_launch(desc)
    sink = pipeline.get_by_name("sink")
    bus = pipeline.get_bus()
    pipeline.set_state(Gst.State.PLAYING)
    index = 0
    try:
        while count is None or index < count:
            sample = sink.emit("try-pull-sample", 2 * Gst.SECOND)
            if sample is None:
                message = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if message is None:
                    continue
                if message.type == Gst.MessageType.ERROR:
                    err, _debug = message.parse_error()
                    raise RuntimeError(f"{port}/{link}: {err.message}")
                if index == 0:
                    raise RuntimeError(f"{port}/{link}: the session ended before its first frame")
                return
            buf = sample.get_buffer()
            ok, info = buf.map(Gst.MapFlags.READ)
            if not ok:
                raise RuntimeError(f"{port}/{link}: a frame buffer did not map")
            try:
                # The converter's rows are packed for these widths; the copy
                # frees the buffer before the next sample lands.
                pixels = np.frombuffer(info.data, dtype=np.uint8, count=width * height * 4)
                image = pixels.reshape(height, width, 4)[:, :, :3].copy()
            finally:
                buf.unmap(info)
            pts = int(buf.pts) if buf.pts != Gst.CLOCK_TIME_NONE else -1
            yield Frame(image=image, pts_ns=pts, index=index)
            index += 1
    finally:
        pipeline.set_state(Gst.State.NULL)
