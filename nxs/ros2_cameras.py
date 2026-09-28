# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The camera topics the ROS 2 launch adds under `cameras:=true`: one
capture node per link the ports' records carry. The GStreamer camera node
runs the capture source the tool's own viewers use, on every host that
has it; the Isaac ROS Argus node is the alternative where that package is
installed."""

from __future__ import annotations

import dataclasses
from typing import Any, Dict, List, Optional


@dataclasses.dataclass(frozen=True)
class CameraTopic:
    """A link's capture node as the launch publishes it."""

    port: str
    link: str
    capture_id: int
    sensor_mode: int
    width: int
    height: int
    framerate: Optional[list] = None
    exposure_min_us: Optional[float] = None
    exposure_max_us: Optional[float] = None
    #: The exposure a trigger's pulse fixes, in microseconds; None when free-running.
    exposure_us: Optional[float] = None

    def namespace(self, topic_base: str = "nxs") -> str:
        """The topic namespace: `/<base>/<port>/<link>`."""
        return f"/{topic_base}/{self.port}/{self.link}"


def camera_plan() -> List[CameraTopic]:
    """Every declared link with a capture id and recorded capture caps, in
    port then link order; a link never brought up carries nothing."""
    from nxs.cam import port_state
    from nxs.cam.topology import load_ports
    from nxs.cam.viewers import resolve_capture_hints

    out: List[CameraTopic] = []
    try:
        ports, _ = load_ports(None)
    except Exception:
        return out
    for _index, topology in sorted(ports.items()):
        record = port_state.port_record(topology)
        for spec in topology.links:
            capture_id = port_state.port_capture_id(topology, spec)
            if capture_id is None:
                continue
            hints = (record.get("viewers") or {}).get(spec.name)
            if not hints:
                continue
            try:
                resolved = resolve_capture_hints(hints, spec.name)
            except SystemExit:
                continue
            out.append(CameraTopic(port_state.port_name(topology), spec.name, int(capture_id),
                                   int(resolved["sensor_mode"]), int(resolved["width"]),
                                   int(resolved["height"]), resolved.get("framerate"),
                                   resolved.get("exposure_min_us"),
                                   resolved.get("exposure_max_us"),
                                   resolved.get("exposure_us")))
    return out


#: The camera node kinds the launch offers, the first the default.
CAMERA_SOURCES = ("gstreamer", "argus")
#: Seconds between two camera nodes' starts: the capture stack opens one
#: session at a time, a session takes a few seconds to start, and two
#: sources started together both fail.
CAMERA_STAGGER_S = 8.0
#: The image encodings the camera node publishes, the first the default:
#: the hardware converter delivers `yuv422` and `mono8` at the link's rate,
#: `rgb8` needs the software converter and lags at 1080p, `jpeg` is the
#: hardware encoder's stream on `image_raw/compressed`.
CAMERA_ENCODINGS = ("yuv422", "mono8", "rgb8", "jpeg")


def gscam_pipeline(topic: CameraTopic, host=None, encoding: str = CAMERA_ENCODINGS[0]) -> str:
    """The link's capture source into system memory in the encoding, for the
    GStreamer camera node to publish: the viewer's source and caps, the
    row's exposure range included. The description ends on an element: the
    node appends its own sink and sets the encoding's caps on it."""
    from nxs import host as host_layer

    host = host or host_layer.current()
    props = ""
    if topic.exposure_us is not None:
        # A trigger's pulse fixes the exposure: pinned, so the capture
        # stack's loop drives gain alone, as the viewers do.
        props = " " + host.exposure_props(topic.exposure_us)
    elif topic.exposure_max_us is not None:
        props = " " + host.ae_props(topic.exposure_min_us, topic.exposure_max_us)
    return (f"{host.source(topic.capture_id, topic.sensor_mode)}{props} ! "
            f"{host.caps(topic.width, topic.height, topic.framerate)} ! "
            f"{host.topic_convert(encoding)}")


def gscam_node(topic: CameraTopic, topic_base: str = "nxs", host=None,
               encoding: str = CAMERA_ENCODINGS[0]) -> Dict[str, Any]:
    """The GStreamer camera node for a link (`ros-<distro>-gscam`): its
    pipeline, its parameters and the remappings onto the link's namespace.
    The frame's stamp is the capture source's own for the buffer."""
    if encoding not in CAMERA_ENCODINGS:
        raise ValueError(f"camera_encoding must be one of {', '.join(CAMERA_ENCODINGS)}, "
                         f"not {encoding!r}")
    namespace = topic.namespace(topic_base)
    frame = f"{topic.port}_{topic.link}"
    return {
        "package": "gscam",
        "executable": "gscam_node",
        "name": frame,
        "parameters": [{
            "gscam_config": gscam_pipeline(topic, host, encoding),
            "camera_name": frame,
            "frame_id": f"{frame}_optical",
            "image_encoding": encoding,
            "use_gst_timestamps": True,
            "sync_sink": False,
            # Reliable matches every subscriber; a best-effort publisher
            # never reaches the republish node's reliable one.
            "use_sensor_data_qos": False,
            "reopen_on_eof": True,
        }],
        "remappings": [("camera/image_raw", f"{namespace}/image_raw"),
                       ("camera/image_raw/compressed", f"{namespace}/image_raw/compressed"),
                       ("camera/camera_info", f"{namespace}/camera_info")],
    }


def argus_node(topic: CameraTopic, topic_base: str = "nxs") -> Dict[str, Any]:
    """The Isaac ROS Argus mono node for a link: its component, parameters
    and the remappings onto the link's namespace."""
    namespace = topic.namespace(topic_base)
    frame = f"{topic.port}_{topic.link}"
    return {
        "package": "isaac_ros_argus_camera",
        "plugin": "nvidia::isaac_ros::argus::ArgusMonoNode",
        "name": frame,
        "parameters": [{
            "camera_id": topic.capture_id,
            "module_id": 0,
            "mode": topic.sensor_mode,
            "camera_type": 0,
            "use_hw_timestamp": True,
            "camera_link_frame_name": frame,
            "optical_frame_name": f"{frame}_optical",
        }],
        "remappings": [("left/image_raw", f"{namespace}/image_raw"),
                       ("left/camera_info", f"{namespace}/camera_info")],
    }
