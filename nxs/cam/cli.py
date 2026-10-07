# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Camera/SerDes plumbing behind the node grammar (`nxs cam1 A on`).

    nxs cam1 A on|off           bring the link up / park the port
    nxs cam1 A stream|capture   viewers with the CSI-gate choreography / a
                                headless delivery check
    nxs cam1 status|caps        presence and health / what the sensor offers
    nxs cam1 A get|set K [V]    a knob read back / changed under the laws;
                                `set sync fsync|free_run` is the port's trigger

The node decides the wires: `cam1 A`, `cam1:A`, or `--port cam1`. Ports
resolve from the manifest or from the platform itself; port verbs name their
port explicitly. Hardware knowledge comes from the hub (`nxs.cam.hubs`).
"""

from __future__ import annotations
import argparse

from nxs import term
from nxs import experimental

from .contracts import InfeasibleConfig
from . import capture, hubs, port_state, viewers
from .identity import (
    _declared_camera,
    _identity_read,
    _require_nxs_hub,
    _verify_hub_identity,
    detect_sensor,
    identity_facts,
    sensor_identity_line,
)
from .run import (
    TRAIN_ROUNDS,
    UNIT_RUN_TIMEOUT_S,
    _accepted,
    _compose_from,
    _execute,
    _execute_split,
    _mode_arg,
    _plan_links,
    _port_links,
    _rates_arg,
    _resolve_modes,
    _run_unit_program,
    _staged_values,
    _unit_personality,
)
from .select import (
    _declare,
    _descriptor_among,
    _link_descriptor,
    _owns_link,
    _hub_for,
    _per_link,
    _port_key,
    _port_name,
    _refuse,
    _remembered_sensors,
    _require_up,
    _resolve_links,
    _role_code,
    _with_sensors,
    print_stream,
    select_port_links,
)
from .verbs.caps import (
    _camera_knobs,
    _hub_fsync_rate,
    _knob_names,
    _range_text,
    caps_payload,
    cmd_caps,
)
from .verbs.on import (
    _dead_pipes,
    _port_lines,
    _relock,
    _sensors_verified,
    cmd_down,
    cmd_up,
    park_port,
)
from .verbs.set import _sensor_readings, _set_sync, cmd_get, cmd_set
from .verbs.status import (
    _unit_line,
    cmd_status,
    follow_gap,
    pair_gain,
    presence_payload,
    presence_rows,
    status_payload,
)
from .verbs.stream import (
    _selected_up_links,
    cmd_capture,
    cmd_stream,
)
from .verbs.sync import (
    _booted_index,
    _fsync_plan,
    _frames_kwarg,
    _hints_by_link,
    _port_viewer_hint,
    _rate_kwarg,
    _record_sync,
    _sync_links,
    plan_text,
    sync_text,
)
from .verbs.verify import _no_capture_id, csi_gate

__all__ = [
    "InfeasibleConfig",
    "TRAIN_ROUNDS",
    "UNIT_RUN_TIMEOUT_S",
    "_accepted",
    "_booted_index",
    "_camera_knobs",
    "_compose_from",
    "_dead_pipes",
    "_declare",
    "_declared_camera",
    "_descriptor_among",
    "_execute",
    "_execute_split",
    "_frames_kwarg",
    "_fsync_plan",
    "_hints_by_link",
    "_hub_fsync_rate",
    "_identity_read",
    "_knob_names",
    "_link_descriptor",
    "_mode_arg",
    "_no_capture_id",
    "_owns_link",
    "_hub_for",
    "_per_link",
    "_plan_links",
    "_port_key",
    "_port_links",
    "_port_lines",
    "_port_name",
    "_port_viewer_hint",
    "_range_text",
    "_rate_kwarg",
    "_rates_arg",
    "_record_sync",
    "_refuse",
    "_relock",
    "_remembered_sensors",
    "_require_nxs_hub",
    "_require_up",
    "_resolve_links",
    "_resolve_modes",
    "_role_code",
    "_run_unit_program",
    "_selected_up_links",
    "_sensor_readings",
    "_sensors_verified",
    "_set_sync",
    "_staged_values",
    "_sync_links",
    "_unit_line",
    "_unit_personality",
    "_verify_hub_identity",
    "_with_sensors",
    "add_cam_parser",
    "capture",
    "caps_payload",
    "cmd_cam",
    "cmd_caps",
    "cmd_capture",
    "cmd_down",
    "cmd_get",
    "cmd_set",
    "cmd_status",
    "cmd_stream",
    "cmd_up",
    "csi_gate",
    "detect_sensor",
    "experimental",
    "follow_gap",
    "identity_facts",
    "hubs",
    "pair_gain",
    "park_port",
    "plan_text",
    "port_state",
    "presence_payload",
    "presence_rows",
    "print_stream",
    "select_port_links",
    "sensor_identity_line",
    "status_payload",
    "sync_text",
    "term",
    "viewers",
]


# ---------------------------
# Parser
# ---------------------------

def add_cam_parser(sub) -> None:
    """Register the camera plumbing the node grammar rewrites into; `nxs cam1 A on`
    is the only public spelling, and main() refuses a literal `nxs cam`."""
    p_cam = sub.add_parser(
        "cam",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p_cam.add_argument("--_node", action="store_true",
                       help=argparse.SUPPRESS)
    p_cam.add_argument("--topology", help="port-set YAML "
                       "(default: the hub's topology)")
    p_cam.add_argument("--port", dest="port", metavar="PORT",
                       help="port name (e.g. cam0, cam1)")
    cam_sub = p_cam.add_subparsers(dest="cam_cmd", required=True)

    p = cam_sub.add_parser("on", help="bring the link up (the whole "
                                      "port's with no link named)")
    p.add_argument("links", nargs="*",
                   help="link selectors (index, name, or port:link)")
    # Declarations pair with the selected links in order: one value applies
    # to every link, repeated values go link by link.
    p.add_argument("--sensor", action="append", default=None,
                   metavar="NAME",
                   help="the sensor behind the selected link(s): a hub "
                        "sensor name or compatible; remembered by the "
                        "port, frozen by tune --freeze --ports")
    p.add_argument("--mode", action="append", default=None,
                   help="mode token (WxH, WxH-rawN, or a mode name) for "
                        "the selected link(s); caps lists them")
    p.add_argument("--fps", action="append", default=None, type=float,
                   help="free-run rate for the selected link(s), inside "
                        "the mode's lawful range (default: the declared "
                        "rate, else 30 fps inside the range)")
    p.add_argument("--dry-run", action="store_true",
                   help="the plan without touching the bus")
    p.set_defaults(cam_fn=cmd_up)

    p = cam_sub.add_parser("off", help="park the port: viewers stopped, "
                                       "sensors in standby, the CSI gate closed")
    p.add_argument("links", nargs="*")
    p.set_defaults(cam_fn=cmd_down)

    p = cam_sub.add_parser("stream", help="viewers on the local display")
    p.add_argument("links", nargs="*")
    p.add_argument("--exposure", type=int, default=None,
                   help="lock the ISP's exposure at this many ns (with --gain); "
                        "without both the ISP runs its own 3A")
    p.add_argument("--gain", type=int, default=None,
                   help="lock the ISP's gain at this value (with --exposure)")
    p.add_argument("--hud", dest="hud", action="store_true", default=True,
                   help="overlay link, mode, sync, and live fps on each "
                        "viewer (default; never on a capture or ROS path)")
    p.add_argument("--no-hud", dest="hud", action="store_false",
                   help="a clean picture, no overlay")
    p.set_defaults(cam_fn=cmd_stream)

    p = cam_sub.add_parser(
        "capture", help="headless delivery check (choreographed consumer)")
    p.add_argument("links", nargs="*")
    p.add_argument("--frames", type=int, default=4)
    p.add_argument("--timeout", type=float, default=None)
    p.add_argument("--snapshot", metavar="DIR",
                   help="also write every delivered frame there as JPEG "
                        "(frame-NNNN.jpg): the picture, headlessly")
    p.add_argument("--encoder", default=capture.DEFAULT_ENCODER,
                   help="JPEG element for --snapshot (the host's own by default; "
                        "jpegenc is the software one)")
    p.set_defaults(cam_fn=cmd_capture)

    p = cam_sub.add_parser("status", help="presence and health: the hub, each "
                                          "link's chain and unit, the sensors' "
                                          "readbacks")
    p.add_argument("links", nargs="*", help="port and/or link selectors")
    p.add_argument("--json", action="store_true",
                   help="the status surface (contract 2)")
    p.set_defaults(cam_fn=cmd_status)

    p = cam_sub.add_parser("caps", help="what the sensor offers on this port: "
                                        "every mode and its rates, the knobs")
    p.add_argument("links", nargs="*")
    p.add_argument("--json", action="store_true",
                   help="the caps surface (contract 2)")
    p.set_defaults(cam_fn=cmd_caps)

    p = cam_sub.add_parser("get", help="a knob read back: the port's sync, the "
                                       "sensor's (fps, exposure, gain), then the "
                                       "unit's parameters")
    p.add_argument("knob")
    p.add_argument("--link", dest="links", action="append", default=[])
    p.set_defaults(cam_fn=cmd_get)

    p = cam_sub.add_parser("set", help="a knob changed under the laws; `sync "
                                       "fsync|free_run` is the port's frame sync")
    p.add_argument("knob")
    p.add_argument("value")
    p.add_argument("--link")
    p.add_argument("--fps", type=float, default=None,
                   help="with sync fsync: the generator's rate (default: the "
                        "declared one)")
    p.add_argument("--exposure", type=float, default=None, metavar="US",
                   help="refused with the fact that sets the exposure: under "
                        "sync fsync the trigger pulse's low time at --fps, else "
                        "the capture stack's loop")
    p.add_argument("--dry-run", action="store_true")
    p.set_defaults(cam_fn=cmd_set)


def cmd_cam(args: argparse.Namespace) -> int:
    """Dispatch a `cam` group verb (drives its own I2C bus). A configuration
    the laws refuse is one fact and its alternatives, exit 2."""
    try:
        return args.cam_fn(args)
    except InfeasibleConfig as exc:
        term.refusal(exc.reason, *exc.alternatives)
        return 2
    except hubs.HubError as exc:
        # A sensor or a hub no installed hub or personality serves: one
        # fact and where a personality for it comes from, never a traceback.
        alternatives = ["nxs personality install ./<name>"]
        if not experimental.enabled():
            alternatives.append(f"nxs {experimental.FLAG} … (reads the hub roots "
                                f"on ${hubs.HUBS_ENV})")
        term.refusal(str(exc), *alternatives)
        return 2
