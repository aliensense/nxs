# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The verdict of a port's `on` as the daemon records it, one file per port
beside the build record, for `nxs switch` to read as the operator: `bringing
up` from the start of the run to its end, then `up`, `reboot needed` or the
refusal it ended on."""

import logging
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

log = logging.getLogger("nxsd")

#: Where it records the verdict of each port's last `on`, one file per port
#: beside the build record (`nxsd.cam1`).
VERDICT_FILE = "nxsd.{port}"
#: The verdict a port carries from the start of the daemon's `on` to its end.
BRINGING_UP = "bringing up"


def port_verdict(port: str) -> Optional[Dict[str, Any]]:
    """The verdict of the last `on` the daemon ran on a port: `verdict` (`up`,
    `reboot needed`, `refused: <fact>`), the `lines` that tell it, the time
    `at` it was recorded, the `boot` it was recorded in (None where the host
    names none) and the daemon instance that wrote it (`pid`, `started`);
    None when none is recorded. A `bringing up` whose daemon no longer runs
    is absent: nobody ends that run, and a reader that waited on it would
    wait on nothing."""
    record = _read(port)
    if record is not None and record["verdict"] == BRINGING_UP and not _running(record):
        return None
    return record


def _read(port: str) -> Optional[Dict[str, Any]]:
    """The port's verdict file as recorded."""
    from nxs.cam import port_state

    try:
        text = (port_state.state_dir() / VERDICT_FILE.format(port=port)).read_text()
    except OSError:
        return None
    record: Dict[str, Any] = {"lines": [], "boot": None, "pid": None, "started": ""}
    for line in text.splitlines():
        key, _, value = line.partition("=")
        if key == "line":
            record["lines"].append(value)
        elif key in ("verdict", "at", "started"):
            record[key] = value
        elif key == "boot":
            record[key] = value or None
        elif key == "pid" and value.isdigit():
            record[key] = int(value)
    try:
        record["at"] = float(record["at"])
    except (KeyError, ValueError):
        return None
    return record if "verdict" in record else None


def _instance() -> Tuple[int, str]:
    """This process as a verdict names its writer: the pid, and the start
    time `/proc/<pid>/stat` holds (clock ticks since boot, "" without
    /proc), which together outlive the pid's reuse."""
    return os.getpid(), _start_ticks(os.getpid())


def _start_ticks(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
    except (OSError, IndexError):
        return ""


def _running(record: Dict[str, Any]) -> bool:
    """Whether the daemon instance that wrote a verdict still runs: the
    record's pid, alive in this boot, started when the record says. A
    record without a pid is an older build's, whose daemon the install
    restarted."""
    from nxs.cam import port_state

    pid = record.get("pid")
    if pid is None or record["boot"] != port_state._boot_id():
        return False
    started = _start_ticks(pid)
    if started:
        return started == record.get("started", "")
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def record_verdict(port: str, verdict: str, lines: List[str]) -> None:
    """Record a port's verdict whole, a `key=value` line each (`at`, `boot`,
    `pid` and `started` for the writer, `verdict`, a `line` per line), for
    `nxs switch` and `nxs <port> status` to read as the operator."""
    from nxs.cam import port_state

    path = port_state.state_dir() / VERDICT_FILE.format(port=port)
    staged = path.with_name(path.name + ".tmp")
    pid, started = _instance()
    text = "".join(f"{key}={value}\n" for key, value in [
        ("at", repr(time.time())), ("boot", port_state._boot_id() or ""), ("pid", pid),
        ("started", started), ("verdict", verdict), *(("line", line) for line in lines)])
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


def settle_stale() -> List[str]:
    """At the daemon's start: a port whose verdict reads `bringing up` from
    a daemon that no longer runs was cut short by that daemon's end. Each
    is recorded as a stopped bring-up, `the daemon restarted`, so the port
    status says what happened; a reader took it as absent already. The
    ports settled, by name."""
    from nxs.cam import port_state

    prefix = VERDICT_FILE.format(port="")
    try:
        names = sorted(os.listdir(port_state.state_dir()))
    except OSError:
        return []
    settled = []
    for name in names:
        if not name.startswith(prefix) or name.endswith(".tmp"):
            continue
        port = name[len(prefix):]
        seen = _read(port)
        if seen is not None and seen["verdict"] == BRINGING_UP and not _running(seen):
            record_verdict(port, *verdict_of(1, [f"{port}: the daemon restarted", "  - nxs switch"]))
            settled.append(port)
    return settled


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
