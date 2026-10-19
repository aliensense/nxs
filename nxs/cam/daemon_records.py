# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The records nxsd writes beside the port record, one `key=value` file
each: the follower heartbeat of a port's synced pair and the camera node
it runs on a link. `status` and a viewer read them as the operator."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, Optional

from . import port_state


#: The heartbeat of the follower nxsd runs on a port's synced pair, one
#: `key=value` file per port beside the record.
FOLLOW_DIR = "follow"


def follow_path(port: str) -> Path:
    return port_state.state_dir() / FOLLOW_DIR / port


def write_follow(port: str, fields: Dict[str, Any]) -> None:
    """Write a port's follower heartbeat whole, a `key=value` line per field
    in order, for the owner and the store's group (nxsd writes it as root,
    `status` reads it as the operator)."""
    path = follow_path(port)
    port_state.ensure_dir(path.parent)
    staged = path.with_name(path.name + ".tmp")
    staged.unlink(missing_ok=True)
    with port_state.open_private(str(staged), os.O_EXCL) as fh:
        fh.write("".join(f"{key}={value}\n" for key, value in fields.items()))
    os.replace(staged, path)


def read_follow(port: str) -> Optional[Dict[str, str]]:
    """A port's follower heartbeat as written; None when nothing follows there."""
    try:
        text = follow_path(port).read_text()
    except OSError:
        return None
    return dict(line.split("=", 1) for line in text.splitlines() if "=" in line)


def follow_gain_db(port: str) -> Optional[float]:
    """The leader's gain a port's follower heartbeat names, in dB: the gain
    the following link's capture session starts at. None without a
    heartbeat, or before the follower takes a gain."""
    try:
        return float((read_follow(port) or {})["gain_db"])
    except (KeyError, ValueError):
        return None


def remove_follow(port: str) -> None:
    follow_path(port).unlink(missing_ok=True)


#: The record of the camera node nxsd runs on a link, a `key=value` file beside the port record:
#: `state` (running, stopped, failed), the node's `pid`, its `exits`, its last `line` and time `at`,
#: and the daemon that wrote it: the `boot`, its pid `nxsd` and its start time `nxsd_started`.
CAMERA_NODE_DIR = "cameras"


def camera_node_path(port: str, link: str) -> Path:
    return port_state.state_dir() / CAMERA_NODE_DIR / f"{port}-{link}"


def write_camera_node(port: str, link: str, fields: Dict[str, Any]) -> None:
    """Write a link's camera-node record whole, a `key=value` line per field
    in order, for the owner and the store's group. The record names the
    daemon that wrote it, as its other records do: the `boot`, its pid as
    `nxsd` and its start time as `nxsd_started`, so a reader judges a
    record of a daemon that no longer runs."""
    from nxs.daemon.verdict import instance

    pid, started = instance()
    fields = {**fields, "boot": port_state.boot_id() or "", "nxsd": pid, "nxsd_started": started}
    path = camera_node_path(port, link)
    port_state.ensure_dir(path.parent)
    staged = path.with_name(path.name + ".tmp")
    staged.unlink(missing_ok=True)
    with port_state.open_private(str(staged), os.O_EXCL) as fh:
        fh.write("".join(f"{key}={value}\n" for key, value in fields.items()))
    os.replace(staged, path)


def read_camera_node(port: str, link: str) -> Optional[Dict[str, str]]:
    """A link's camera-node record as written; None where nxsd runs none."""
    try:
        text = camera_node_path(port, link).read_text()
    except OSError:
        return None
    return dict(line.split("=", 1) for line in text.splitlines() if "=" in line)


def remove_camera_node(port: str, link: str) -> None:
    camera_node_path(port, link).unlink(missing_ok=True)
