# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The experimental switch: `nxs --experimental …` reads a hub root and the
experimental overlays ($NXS_CAM_HUBS, $NXS_CAM_PERSONALITIES, $NXS_CAM_EXPERIMENTAL) and takes
a development personality store.

Without it the tool is the product: the shipped hub and the release's
store, every mode their programs carry. The flag sets `NXS_EXPERIMENTAL=1`
so a daemon, an MCP subprocess and the hub loader in another process agree.
"""

from __future__ import annotations

import os

ENV = "NXS_EXPERIMENTAL"
FLAG = "--experimental"

_enabled = False


def enabled() -> bool:
    """Whether the experimental surface is unlocked (the flag, or the variable a
    flagged parent process left)."""
    return _enabled or os.environ.get(ENV, "") not in ("", "0")


def enable() -> None:
    """Unlock the experimental surface for this process and its children."""
    global _enabled
    _enabled = True
    os.environ[ENV] = "1"


def disable() -> None:
    """Lock it again (a test seam)."""
    global _enabled
    _enabled = False
    os.environ.pop(ENV, None)
