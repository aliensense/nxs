# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The camera topics the ROS 2 launch adds under `cameras:=true`: one
capture node per link the ports' records carry. The GStreamer camera node
runs the capture source the tool's own viewers use, on every host that
has it; the Isaac ROS Argus node is the alternative where that package is
installed, on a link that is not a synced pair's."""

from __future__ import annotations

import dataclasses
import sys
import time
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
    #: The link's part in a synced pair's one exposure and gain (`leader`,
    #: `follower`, `locked`), the pair's other link, and a locked link's
    #: gain; None where the link runs its own loop.
    ae_role: Optional[str] = None
    ae_peer: Optional[str] = None
    gain_db: Optional[float] = None

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
                                   resolved.get("exposure_us"), resolved.get("ae_role"),
                                   resolved.get("ae_peer"), resolved.get("gain_db")))
    return out


#: The camera node kinds the launch offers, the first the default.
CAMERA_SOURCES = ("gstreamer", "argus")
#: Seconds between two camera nodes' starts: the capture stack opens one
#: session at a time, a session takes a few seconds to start, and two
#: sources started together both fail.
CAMERA_STAGGER_S = 8.0
#: Seconds from a port's first camera node saying its stream started to the
#: port's output closing under it; the output opens one gate step later,
#: half a second. The capture stack hears a stream's start from about one
#: second after its session opens until it gives the session up, some four
#: seconds in.
CAMERA_OPEN_S = 1.5
#: The camera node's line once its pipeline plays.
CAMERA_STARTED = "Started stream"
#: The image encodings the camera node publishes, the first the default:
#: the hardware converter delivers `yuv422` and `mono8` at the link's rate,
#: `rgb8` needs the software converter and lags at 1080p, `jpeg` is the
#: hardware encoder's stream on `image_raw/compressed`.
CAMERA_ENCODINGS = ("yuv422", "mono8", "rgb8", "jpeg")


def port_openers(plan: List[CameraTopic]) -> Dict[str, int]:
    """The node that opens each port's first capture session, by port: the
    index of the port's first node in the plan. The capture stack locks onto
    a stream only when it sees the stream start, so that session needs the
    port's output restarted under it (`stream_start`). A session opened
    beside a delivering one needs nothing, and a restart under a delivering
    session ends it."""
    first: Dict[str, int] = {}
    for index, topic in enumerate(plan):
        first.setdefault(topic.port, index)
    return first


def stream_start(port: str, since: Optional[float] = None) -> int:
    """`nxs ros2 --stream-start <port>`: restart the port's output under the
    capture session a camera node has just opened, so the session sees the
    stream start. The output closes `CAMERA_OPEN_S` after `since`, the time
    the node said its stream started (this call's own start without one),
    and opens one gate step later. The gate is the one every choreographed
    capture switches (`verify.csi_gate`), each write under the bus lock. The
    caller runs it for a port's first session alone (`port_openers`)."""
    from nxs.cam.frame_source import port_topology
    from nxs.cam.select import _hub_for
    from nxs.cam.verbs import verify

    began = time.time() if since is None else since
    try:
        topology = port_topology(port)
    except LookupError as exc:
        print(f"nxs ros2: {exc}", file=sys.stderr)
        return 1
    hub = _hub_for(topology)
    gate = verify.csi_gate(hub, hub.flows(), topology)
    # What is left of the time, and never more than all of it: a clock set
    # back between the node's line and this call must not hold the restart.
    time.sleep(min(CAMERA_OPEN_S, max(0.0, CAMERA_OPEN_S - (time.time() - began))))
    gate(False)
    gate(True)
    print(f"{port}: the output restarted under the first camera node")
    return 0


def gscam_pipeline(topic: CameraTopic, host=None, encoding: str = CAMERA_ENCODINGS[0]) -> str:
    """The link's capture source into system memory in the encoding, for the
    GStreamer camera node to publish: the viewer's source, caps and source
    properties (`nxs.cam.viewers.source_props`). The description ends on an
    element: the node appends its own sink and sets the encoding's caps on
    it."""
    from nxs import host as host_layer
    from nxs.cam.viewers import source_props

    host = host or host_layer.current()
    props = source_props(dataclasses.asdict(topic), host, port=topic.port)
    props = f" {props}" if props else ""
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
    and the remappings onto the link's namespace.

    Raises:
        ValueError: The link is one of a synced pair's: its capture session
            runs the pair's exposure and gain, which the node's parameters
            cannot set.
    """
    if topic.ae_role:
        raise ValueError(f"{topic.port}/{topic.link}: a synced pair's link locks its capture "
                         f"session, and the Argus node takes no exposure or gain; "
                         f"camera_source:=gstreamer")
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
