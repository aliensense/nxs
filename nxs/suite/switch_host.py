# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The host's files for `nxs switch`: the state store and the declaration
directory, both root's and the I²C group's, the bus rules, the capture
daemon's quiet, the tab completion. The store is written by nxsd as root
and by the operator next, so every directory in it is the group's, and a
store a run as root closed is opened again here."""

from __future__ import annotations

import os
import stat
from typing import Dict, List, Optional, Tuple

from nxs import host as host_layer
from nxs.host import root

STATE_DIR = "/var/lib/aliensense"
STATE_GROUP = "i2c"


def _write(target: str, text: str) -> bool:
    """Write under root when the content differs; True when it was written."""
    if root.read_text(target) == text:
        return False
    root.write_text(target, text)
    return True


def host_step(dry_run: bool, cfg=None) -> Tuple[List[str], bool, Optional[Dict[str, str]]]:
    """The host's part: the state store, the declaration's directory on a
    host whose declaration is the system one, the bus rules, the capture
    daemon's quiet, the completion file, the nxsd unit. Returns what
    changed, whether the unit's text did, and the environment it carries."""
    from nxs import suite
    from nxs.setup_cli import COMPLETION_TARGET, completion_text

    host = host_layer.current()
    done: List[str] = []
    would = "would " if dry_run else ""
    if not _writable(STATE_DIR):
        if not dry_run:
            make_group_dir(STATE_DIR)
            make_group_dir(os.path.join(STATE_DIR, "cam"))
        done.append(f"{would}create the state store {STATE_DIR}" if dry_run
                    else f"state store {STATE_DIR}")
    elif _closed_to_group(STATE_DIR):
        if not dry_run:
            root.run(["sh", "-c", 'chmod -R g+rwX "$1" && find "$1" -type d -exec chmod g+s {} +',
                      "nxs-store", STATE_DIR])
        done.append(f"{would}open the state store to its group" if dry_run
                    else "state store opened to its group")
    declaration_dir = os.path.dirname(suite._SYSTEM_CONFIG)
    if host.keeps_system_declaration() and not _writable(declaration_dir):
        if not dry_run:
            make_group_dir(declaration_dir)
        done.append(f"{would}create the declaration directory {declaration_dir}" if dry_run
                    else f"declaration directory {declaration_dir}")
    rules = host.bus_rules()
    if rules is not None:
        text = rules.read_text()
        if root.read_text(host_layer.BUS_RULES_TARGET) != text:
            if not dry_run:
                root.write_text(host_layer.BUS_RULES_TARGET, text)
                root.run(["udevadm", "control", "--reload"])
                root.run(["udevadm", "trigger", "--subsystem-match=i2c-dev"])
            done.append(f"{would}udev rules installed")
    if not dry_run:
        changed: List[str] = []

        def write(target: str, text: str) -> bool:
            if _write(target, text):
                changed.append(target)
            return True

        host.setup(write)
        if changed:
            done.append("the capture daemon's crash reports quieted")
    completion = completion_text()
    if completion and root.read_text(COMPLETION_TARGET) != completion:
        if not dry_run:
            root.write_text(COMPLETION_TARGET, completion)
        done.append(f"{would}tab completion installed")
    from nxs.suite.switch_cam import _daemon_step
    lines, unit_changed, environment = _daemon_step(host, dry_run, cfg)
    done.extend(lines)
    return done, unit_changed, environment


def _closed_to_group(path: str) -> bool:
    """Whether another user left a directory under `path` closed to the
    group: without its write, or without the setgid bit that keeps the group
    on what lands there (nxsd writes the store as root, the operator next).
    A store file lands by a rename from a private temporary, so a file root
    wrote is replaced, never opened, and its own mode closes nothing."""
    me = os.geteuid()
    want = stat.S_IRGRP | stat.S_IWGRP | stat.S_IXGRP | stat.S_ISGID
    for parent, dirs, _files in os.walk(path):
        for name in dirs:
            try:
                st = os.lstat(os.path.join(parent, name))
            except OSError:
                continue
            if st.st_uid == me or stat.S_ISLNK(st.st_mode):
                continue
            if st.st_mode & want != want:
                return True
    return False


def make_group_dir(path: str) -> None:
    """Create `path` under root for the I²C group: mode 2775, so the group
    writes it and what lands in it keeps the group."""
    group = ["-g", STATE_GROUP] if _group_exists(STATE_GROUP) else []
    root.run(["install", "-d", "-m", "2775", *group, path])


def _writable(path: str) -> bool:
    return os.path.isdir(path) and os.access(path, os.W_OK)


def _group_exists(name: str) -> bool:
    try:
        import grp
        grp.getgrnam(name)
        return True
    except (ImportError, KeyError):
        return False
