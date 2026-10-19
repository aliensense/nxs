# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""`nxs <unit> map check`: the conformance checks of Interface Description
§4 and §6.4 at the unit's two I²C doors. A smart camera has no unit door:
the address one above the one named is its camera door."""

import json
import sys

from nxs.unit_cli import _describe_target


def add_map_parser(sub) -> None:
    """Register `nxs map check`."""
    p_map = sub.add_parser('map',
                           help="The unit's NXS map over I²C: `check` proves both its doors "
                                "against the unit's own records")
    ps = p_map.add_subparsers(dest='map_cmd', required=True)
    p_check = ps.add_parser('check', help='Read both doors, run the sensor, commit each kind in '
                                          'range and out of it; exit 0 when every check holds')
    p_check.add_argument('--json', action='store_true', help='machine-readable checks')


def map_answers_alone(t, args) -> bool:
    """Whether a smart camera's door answers one above the address named,
    where no unit answers: the verb runs on the camera door alone, and
    names a layout version this tool does not speak."""
    from nxs.cam.smart_camera import answers, foreign_layout

    if args.transport != 'i2c':
        return False
    with t.door(int(args.addr) + 1) as cam_door:
        return answers(cam_door) or foreign_layout(cam_door) is not None


def _live_port(cam_door, args):
    """Why the check would race the port's own users, None when the doors
    are the check's alone: the camera streams, or nxsd holds the port, its
    followers and capture sessions committing at the camera door. The line
    names the park."""
    from nxs._libnxs_map import identity_fact
    from nxs.cam import port_state
    from nxs.daemon import BRINGING_UP, port_verdict
    from nxs.daemon.verdict import running
    from nxs.personality_cli import port_link_for

    where = port_link_for(args)
    port = port_state.port_name(where[0]) if where is not None else "<port>"
    try:
        boot = cam_door.header()
    except OSError:
        return None             # the check names a door that does not answer
    streaming = identity_fact(boot["id"], boot["version"]) is None and boot["stream"] != 0
    seen = port_verdict(port) if where is not None else None
    held = (seen is not None and running(seen)
            and (seen["verdict"] == BRINGING_UP
                 or port_state.STATE_UP in port_state.recorded_states(where[0]).values()))
    if not streaming and not held:
        return None
    fact = (f"the camera door @{cam_door.address:#04x} streams" if streaming
            else f"nxsd holds {port}")
    return f"nxs: {fact}: the check commits on a parked port\n  - nxs {port} off"


def cmd_map(t, args):
    """`map check`: the conformance checks at the unit's two I²C doors, one
    line each. A live port is refused: the camera streams, or nxsd holds
    the port."""
    from nxs._libnxs_map import unit_answers
    from nxs.cam.port_state import BusLock
    from nxs.map_check import check

    if args.transport != 'i2c':
        print("nxs: map check runs over I²C: the map's doors are the unit's I²C addresses",
              file=sys.stderr)
        return 2
    address = int(args.addr) + 1
    answered = unit_answers(t)
    unit = _describe_target(args) if answered else None
    with t.door(address) as cam_door:
        unit_door = t.door(int(args.addr)) if answered else None
        try:
            # The check's commits open windows the unit masters its own bus in:
            # the tool's other bus users wait, as they do for a port's bring-up.
            with BusLock():
                refusal = _live_port(cam_door, args)
                if refusal is not None:
                    print(refusal, file=sys.stderr)
                    return 2
                report = check(t if answered else None, cam_door, unit_door)
        finally:
            if unit_door is not None:
                unit_door.close()
    if args.json:
        from nxs.schemas import CONTRACT

        print(json.dumps({"contract": CONTRACT, "unit": unit,
                          "camera_door": address, **report.to_dict()}, indent=2))
    else:
        where = f"of {unit}" if unit else f"on i2c {args.bus}, no unit door"
        print(f"camera door @{address:#04x} {where}")
        for line in report.checks:
            print(f"{'✓' if line.ok else '✗'} {line.name}  {line.detail}")
        print("ok" if report.ok else f"{sum(not c.ok for c in report.checks)} checks failed")
    return 0 if report.ok else 1
