# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The build a running daemon records beside the port record, and when: `nxs switch`
restarts a daemon whose record names another build than the tool's, or one older
than the installed package."""

from __future__ import annotations

import logging
from typing import Optional

from nxs.daemon.environment import record_environment

log = logging.getLogger("nxsd")

#: Where a running daemon records the build it runs, beside the port record.
BUILD_FILE = "nxsd.build"
def running_build() -> Optional[str]:
    """The build the running daemon recorded at start, None when none did."""
    from nxs.cam import port_state

    try:
        return (port_state.state_dir() / BUILD_FILE).read_text().strip() or None
    except OSError:
        return None


def running_since() -> Optional[float]:
    """When the running daemon recorded its build, as the record's mtime; None when none did."""
    from nxs.cam import port_state
    try:
        return (port_state.state_dir() / BUILD_FILE).stat().st_mtime
    except OSError:
        return None


def record_build() -> None:
    """Record this daemon's build and its ROS 2 environment once its ports
    are constructed, or once it holds for a manifest in tune: `nxs switch`
    restarts a daemon that a newer wheel or a rewritten unit left running
    (a reload changes the manifest only) and waits for the record to say
    so."""
    from nxs.cam import port_state
    from nxs.cli import _git_version

    try:
        port_state.write_store_file(port_state.state_dir() / BUILD_FILE, _git_version() + "\n")
    except OSError as exc:
        log.warning("the build record was not written: %s", exc)
    record_environment()
