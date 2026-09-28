# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The experimental switch: `nxs --experimental …` unlocks three things.

Without it the tool is the product: the shipped modes and their rates,
the shipped pack. With it, the unshipped modes appear (marked), a pack
root and the experimental overlays are read ($NXS_CAM_DESCRIPTORS,
$NXS_CAM_EXPERIMENTAL). The flag sets `NXS_EXPERIMENTAL=1` so a daemon, an MCP
subprocess and the pack loader in another process agree.
"""

from __future__ import annotations

import argparse
import os
from typing import Any

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


def refusal(what: str) -> str:
    """The sentence a locked option or value is refused with."""
    return (f"{what} is experimental — run: nxs {FLAG} <port> [<link>] "
            f"<verb> …")


class _Refuse(argparse.Action):
    """A locked option: it parses (so the help stays honest about its
    existence under the flag) and refuses by name."""

    def __call__(self, parser, namespace, values, option_string=None):
        parser.error(refusal(option_string or self.dest))


def argument(parser: argparse.ArgumentParser, *flags: str, **kwargs: Any) -> None:
    """Declare an option that exists only under the flag. Unlocked it is
    the ordinary option; locked it is hidden from the help and refuses
    with the sentence naming the flag."""
    if enabled():
        parser.add_argument(*flags, **kwargs)
        return
    dest = kwargs.get("dest")
    parser.add_argument(*flags, action=_Refuse, nargs="?",
                        help=argparse.SUPPRESS, default=kwargs.get("default"),
                        **({"dest": dest} if dest else {}))
