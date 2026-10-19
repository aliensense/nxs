# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""A smart camera: a head that programs its own sensor and serves the
capture cells of the NXS map alone, at `RBDevice::NXS_CAM` on the port's
own bus (Interface Description §6.12). It has no unit door, so the host
knows it by its camera door: the identity, and the rows and the mode
table, which are the records of the installed cam personality of the
`native` family that describes it."""

from __future__ import annotations

from typing import Optional, Sequence, Tuple

from nxs._generated_constants import NxsDevices
from nxs._generated_constants import NxsMap as Map

#: Where a smart camera's door answers on the port's own bus.
ADDRESS = int(NxsDevices.RBDevice.NXS_CAM)


def signature(records: Sequence[Tuple[int, bytes]]) -> Tuple[bytes, bytes]:
    """What a personality's records make a door serve: the PARAMS rows and
    the mode-table rows."""
    from nxs.personality.records import camera_map_mode_rows
    from nxs.personality.records_fields import PARAMS

    by_type = {int(t): bytes(p) for t, p in records}
    return by_type.get(PARAMS, b""), camera_map_mode_rows(records)


def served(door) -> Tuple[bytes, bytes]:
    """What a camera door serves: its rows as their records hold them, the
    owner bits a door writes into the flags cleared, and its mode-table rows."""
    from nxs._libnxs_map import without_owner

    count = door.u8(Map.MapReg.MODE_COUNT)
    modes = door.read(Map.MapReg.MODE_TABLE, Map.MODE_ENTRY_SIZE * count) if count else b""
    return without_owner(door.raw_rows()), modes


def answers(door) -> bool:
    """Whether a camera door this tool speaks answers at the client's
    address: its identity, in a layout version this tool knows, since a
    driver refuses another (Interface Description §2)."""
    from nxs._libnxs_map import identity_fact

    try:
        boot = door.header()
    except OSError:
        return False
    return boot["id"] == Map.MAP_ID_CAMERA and identity_fact(boot["id"], boot["version"]) is None


def serves_alone(door) -> bool:
    """Whether the camera door is a smart camera's, served with no unit door:
    it reads `FW_VERSION_MAJOR` 0, where a pod's unit reads the major
    version of the firmware it runs (Interface Description §6.12)."""
    try:
        return door.header()["fw"][0] == 0
    except OSError:
        return False


def foreign_layout(door) -> Optional[str]:
    """The line for a camera door whose identity answers in a layout version
    this tool does not speak; None for any other answer, or none."""
    try:
        boot = door.header()
    except OSError:
        return None
    if boot["id"] != Map.MAP_ID_CAMERA or boot["version"] == Map.MAP_VERSION_VALUE:
        return None
    return (f"camera door @{door.address:#04x} id {boot['id']:#04x} v{boot['version']}, a layout "
            f"this nxs does not speak (it speaks v{Map.MAP_VERSION_VALUE})")


def serves(door, records: Sequence[Tuple[int, bytes]]) -> bool:
    """Whether the door serves the rows and the mode table of these records."""
    return served(door) == signature(records)


def described_by(door) -> Optional[Tuple[str, list]]:
    """The compatible and the records of the installed `native` personality
    whose rows and mode table the door serves; None when none matches. A
    personality that does not compile, or whose records do not read,
    describes no door, and the others are still asked."""
    from nxs.cam import cam_personalities
    from nxs.image import parse_trailer
    from nxs.personality import PersonalityError, _trailer_of

    have = served(door)
    for name in cam_personalities.names():
        descriptor = cam_personalities.find(name)
        if str((descriptor.raw("meta") or {}).get("chip") or "") != "native":
            continue
        try:
            records = parse_trailer(_trailer_of(name)[1])
            serves_them = signature(records) == have
        except (PersonalityError, ValueError):
            continue
        if serves_them:
            return str(descriptor.compatible), records
    return None


def identify(door) -> Optional[str]:
    """The compatible of the installed `native` personality whose rows and
    mode table the door serves; None when none matches."""
    described = described_by(door)

    return described[0] if described else None


def map_line(door) -> str:
    """The camera door as `status` names it: its identity, its rows, its
    state, the mode in effect and the stream."""
    boot = door.header()
    state = Map.MapState._NAMES.get(boot["state"], str(boot["state"]))
    mode = "no mode" if boot["current_mode"] == 0xFF else f"mode {boot['current_mode']}"
    stream = "streaming" if boot["stream"] else "standby"
    return (f"camera door @{door.address:#04x} id {boot['id']:#04x} v{boot['version']}, "
            f"{boot['param_count']} rows, {state}, {mode}, {stream}")
