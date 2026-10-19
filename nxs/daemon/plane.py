# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The data plane's records beside the build record, for `nxs status`, a
bridge and the launch to read as the operator: why the running daemon holds
the units without one, which units it bridges, and that its camera nodes
serve the camera links."""

import logging
import time
from typing import Dict, Optional

from nxs.daemon.verdict import instance, running

log = logging.getLogger("nxsd")

#: The hold's reason when the daemon has no ROS 2 client library.
NO_ROS2 = "no ROS 2 environment"

#: Where the daemon records why it holds the units without a data plane,
#: beside the build record, for `nxs status` to read as the operator.
HOLD_FILE = "nxsd.hold"
#: Where the daemon records the units its bridge serves while the bridge
#: runs, so an operator's bridge or launch defers to it.
DATA_PLANE_FILE = "nxsd.data_plane"
#: Where the resident's camera nodes record the links they serve, holds and bring-ups included,
#: so an operator's launch adds no node beside them.
CAMERA_PLANE_FILE = "nxsd.camera_plane"


def record_hold(reason: str) -> None:
    _write(HOLD_FILE, "hold", reason=reason)


def clear_hold() -> None:
    _clear(HOLD_FILE)


def hold_reason() -> Optional[str]:
    """Why the running daemon holds the units without a data plane, None
    when it serves them or does not run (a record of a daemon that no
    longer runs says nothing)."""
    record = _read(HOLD_FILE)
    if record is None or "reason" not in record:
        return None
    return str(record["reason"])


def record_data_plane(units) -> None:
    _write(DATA_PLANE_FILE, "data-plane", units=",".join(units))


def clear_data_plane() -> None:
    _clear(DATA_PLANE_FILE)


def data_plane() -> Optional[Dict[str, object]]:
    """The unit bridge a running daemon serves: its `pid` and the `units` it
    bridges; None while it holds, or when the daemon that wrote the record
    no longer runs."""
    record = _read(DATA_PLANE_FILE)
    if record is None or "units" not in record:
        return None
    return {"pid": record["pid"], "units": _names(record["units"])}


def record_camera_plane(cameras) -> None:
    _write(CAMERA_PLANE_FILE, "camera-plane", cameras=",".join(cameras))


def clear_camera_plane() -> None:
    _clear(CAMERA_PLANE_FILE)


def camera_plane() -> Optional[Dict[str, object]]:
    """The camera nodes a running daemon runs: its `pid` and the `cameras`
    (port/link) the port records carry now, an empty list before a link is
    up; None where the daemon runs no camera node, or when the daemon that
    wrote the record no longer runs."""
    record = _read(CAMERA_PLANE_FILE)
    if record is None or "cameras" not in record:
        return None
    return {"pid": record["pid"], "cameras": _names(record["cameras"])}


def _names(value: object) -> list:
    return [name for name in str(value).split(",") if name]


def _write(name: str, what: str, **fields) -> None:
    """A record of this daemon instance by file name: when, in which boot,
    the writer (`pid`, `started`), then `fields`."""
    from nxs.cam import port_state

    pid, started = instance()
    text = "".join(f"{key}={value}\n" for key, value in [
        ("at", repr(time.time())), ("boot", port_state.boot_id() or ""), ("pid", pid),
        ("started", started), *fields.items()])
    try:
        port_state.write_store_file(port_state.state_dir() / name, text)
    except OSError as exc:
        log.warning("the %s record was not written: %s", what, exc)


def _clear(name: str) -> None:
    from nxs.cam import port_state

    try:
        (port_state.state_dir() / name).unlink()
    except OSError:
        pass


def _read(name: str) -> Optional[Dict[str, object]]:
    """A record of the running daemon by file name, None without one or
    when its writer no longer runs."""
    from nxs.cam import port_state

    try:
        text = (port_state.state_dir() / name).read_text()
    except OSError:
        return None
    record: Dict[str, object] = {"boot": None, "pid": None, "started": ""}
    for line in text.splitlines():
        key, _, value = line.partition("=")
        if key == "boot":
            record[key] = value or None
        elif key == "pid" and value.isdigit():
            record[key] = int(value)
        elif key:
            record[key] = value
    return record if running(record) else None
