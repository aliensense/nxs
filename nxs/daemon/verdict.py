# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The verdict of a port's `on` as the daemon records it, one file per port
beside the build record, for `nxs switch` to read as the operator: `bringing
up` from the start of the run to its end, then `up`, `reboot needed` or the
refusal it ended on."""

import logging
import os
import time
from typing import Any, Dict, List, Optional, Tuple

log = logging.getLogger("nxsd")

#: Where it records the verdict of each port's last `on`, one file per port
#: beside the build record (`nxsd.cam1`).
VERDICT_FILE = "nxsd.{port}"
#: The verdict a port carries from the start of the daemon's `on` to its end.
BRINGING_UP = "bringing up"


def port_verdict(port: str) -> Optional[Dict[str, Any]]:
    """The verdict of the last `on` the daemon ran on a port: `verdict` (`up`,
    `reboot needed`, `refused: <fact>`), the `lines` that tell it and the
    time `at` it was recorded; None when none is recorded."""
    from nxs.cam import port_state

    try:
        text = (port_state.state_dir() / VERDICT_FILE.format(port=port)).read_text()
    except OSError:
        return None
    record: Dict[str, Any] = {"lines": []}
    for line in text.splitlines():
        key, _, value = line.partition("=")
        if key == "line":
            record["lines"].append(value)
        elif key in ("verdict", "at"):
            record[key] = value
    try:
        record["at"] = float(record["at"])
    except (KeyError, ValueError):
        return None
    return record if "verdict" in record else None


def record_verdict(port: str, verdict: str, lines: List[str]) -> None:
    """Record a port's verdict whole, a `key=value` line each (`at`,
    `verdict`, a `line` per line), for `nxs switch` to read as the operator."""
    from nxs.cam import port_state

    path = port_state.state_dir() / VERDICT_FILE.format(port=port)
    staged = path.with_name(path.name + ".tmp")
    text = "".join(f"{key}={value}\n" for key, value in [
        ("at", repr(time.time())), ("verdict", verdict), *(("line", line) for line in lines)])
    try:
        # A fresh file, never one through a link left at its name: the
        # daemon writes as root into a store the operator's group shares.
        staged.unlink(missing_ok=True)
        fd = os.open(staged, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o644)
        with os.fdopen(fd, "w") as fh:
            fh.write(text)
        os.replace(staged, path)
    except OSError as exc:
        log.warning("port %s: the verdict was not recorded: %s", port, exc)


def verdict_of(rc: int, lines: List[str]) -> Tuple[str, List[str]]:
    """What a port's `on` ended on, from its status and its lines: `up`;
    `reboot needed` (status 3) with the lines under its `REBOOT NEEDED:`
    but the one that asks for the same command again; else `refused:
    <fact>` with the refusal it ended on (the fact and the `  - ` lines
    under it)."""
    if rc == 0:
        return "up", []
    if rc == 3:
        start = max((i + 1 for i, line in enumerate(lines) if line == "REBOOT NEEDED:"), default=0)
        return "reboot needed", [line for line in lines[start:] if line != "then run this command again"]
    start = len(lines)
    while start > 0 and lines[start - 1].startswith("  - "):
        start -= 1
    refusal = lines[max(start - 1, 0):] or [f"on exited {rc}"]
    return f"refused: {refusal[0]}", refusal
