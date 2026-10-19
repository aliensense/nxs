#!/usr/bin/env python3
# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""mapspike: the pod bus under two masters at frame rate (R-10.13).

A bench tool beside `nxs`, not one of its verbs. Once a frame period it
commits each pod's exposure and gain rows at its camera door as the kernel
driver does (Interface Description §6.4), the exposure and the gain moving
inside the ranges the camera door serves. It reads the time the door
painted for an apply, `APPLY_RUN_MS`, then `STATE`, since a `COMMIT`
written while `BUSY` or `INERT` is ignored, and waits a run in flight out.
The commit is one transaction from the highest row staged through
`COMMIT`. Then it sends nothing for the painted time and reads `STATE`
every `APPLY_POLL_MS` until it leaves `BUSY`, `APPLY_POLL_MAX_MS` past the
painted time at most, before any other transaction, the other pod's commit
included. A `STATE` value outside the four counts as `BUSY`, and a terminal
state counts from two agreeing reads. A transaction the camera door does
not acknowledge is the unit mastering the pod bus: it is repeated after
`BUSY_RETRY_MS`, `BUSY_RETRIES` times at most, and a `STATE` read it cuts
counts as `BUSY`. Once a second the spike reads the pod's unit door at its
own alias, to show the unit answers throughout. The run ends with the
counts the gate judges: commits, transactions not acknowledged, commits
that needed a repeat, commits that failed or that the door was too busy to
take, applies that ended `FAULT` or stayed `BUSY` past the window, each
commit's apply latency from its write to `READY`, and the unit door's reads.
"""

from __future__ import annotations

import argparse
import dataclasses
import errno
import sys
import time
from typing import Callable, Dict, List, Optional, Tuple

from nxs import _libnxs
from nxs._generated_constants import NxsMap
from nxs._libnxs_map import STATES, Door, commit_payload, find_row
from nxs.cam import port_state

MAP = NxsMap.MapReg
STATE_READY, STATE_BUSY, STATE_FAULT = (NxsMap.MapState.READY, NxsMap.MapState.BUSY,
                                        NxsMap.MapState.FAULT)
#: The errnos a transaction the camera door did not acknowledge comes back with.
#: EREMOTEIO is Linux's; a workstation's errno module lacks it.
BUSY_ERRNOS = {getattr(errno, "EREMOTEIO", 121), errno.ENXIO, errno.EIO, errno.EAGAIN, errno.ETIMEDOUT}
#: The driver's repeat of a transaction the door NAKed, and its count.
RETRY_S = NxsMap.BUSY_RETRY_MS / 1000
RETRIES = NxsMap.BUSY_RETRIES
#: The driver's apply window past the painted time: the `STATE` poll's
#: interval, and how long a run may read BUSY.
APPLY_POLL_S = NxsMap.APPLY_POLL_MS / 1000
APPLY_POLL_MAX_S = NxsMap.APPLY_POLL_MAX_MS / 1000


@dataclasses.dataclass(frozen=True)
class Head:
    """One pod's two aliases on the port's bus, its exposure and gain rows,
    and the ranges the camera door serves for them."""

    link: str
    map_addr: int
    camera_map_addr: int
    exposure_row: int
    gain_row: int
    exposure_min_us: int
    exposure_max_us: int
    gain_min_mdb: int
    gain_max_mdb: int
    gain_step_mdb: int


@dataclasses.dataclass
class Counts:
    commits: int = 0
    naks: int = 0
    repeated: int = 0
    failed: int = 0
    faults: int = 0
    skipped: int = 0
    map_reads: int = 0
    map_errors: int = 0
    latencies_ms: List[float] = dataclasses.field(default_factory=list)


def read_head(bus, link: str, map_addr: int, camera_map_addr: int) -> Head:
    """The camera door's identity, rows and ranges, refused when no camera
    door answers there or it serves no exposure or gain row."""
    with Door(bus, camera_map_addr) as door:
        boot = door.header()
        if boot["id"] != NxsMap.MAP_ID_CAMERA:
            raise SystemExit(f"mapspike: link {link}: no camera door at {camera_map_addr:#04x} "
                             f"(id {boot['id']:#04x})")
        if boot["state"] == NxsMap.MapState.INERT:
            raise SystemExit(f"mapspike: link {link}: the camera door is INERT; bring the port up first")
        rows = door.rows()
        exposure = find_row(rows, NxsMap.ParamId.EXPOSURE_US)
        gain = find_row(rows, NxsMap.ParamId.GAIN_MDB)
        if exposure is None or gain is None:
            raise SystemExit(f"mapspike: link {link}: the camera door serves no exposure or gain row")
        cells = door.cells(rows)
    return Head(link, map_addr, camera_map_addr, exposure, gain, cells[exposure]["min"],
                cells[exposure]["max"], cells[gain]["min"], cells[gain]["max"], cells[gain]["step"])


def write(bus, head: Head, staged: Dict[int, int], counts: Counts,
          sleep: Callable[[float], None]) -> bool:
    """The commit's one transaction with the driver's repeats; False when
    every repeat failed."""
    register, payload = commit_payload(staged)
    message = register.to_bytes(2, "big") + payload
    for attempt in range(RETRIES + 1):
        try:
            bus.write(head.camera_map_addr, message)
            counts.commits += 1
            if attempt:
                counts.repeated += 1
            return True
        except OSError as exc:
            if exc.errno not in BUSY_ERRNOS:
                raise
            counts.naks += 1
            sleep(RETRY_S)
    counts.failed += 1
    return False


def read_state(bus, head: Head, counts: Counts) -> int:
    """`STATE` as the driver reads it: a NAK, a value outside the four and a
    terminal state the next read does not repeat count as BUSY."""
    pointer = int(MAP.STATE).to_bytes(2, "big")
    try:
        state = bus.read(head.camera_map_addr, pointer, 1)[0]
        if state not in STATES or state == STATE_BUSY:
            return STATE_BUSY
        return state if bus.read(head.camera_map_addr, pointer, 1)[0] == state else STATE_BUSY
    except OSError as exc:
        if exc.errno not in BUSY_ERRNOS:
            raise
        counts.naks += 1
        return STATE_BUSY


def painted_s(bus, head: Head, counts: Counts) -> float:
    """The time the door painted for an apply, `APPLY_RUN_MS`, in seconds;
    the floor the contract names when the read is cut."""
    pointer = int(MAP.APPLY_RUN_MS).to_bytes(2, "big")
    try:
        return int.from_bytes(bus.read(head.camera_map_addr, pointer, 2), "little") / 1000
    except OSError as exc:
        if exc.errno not in BUSY_ERRNOS:
            raise
        counts.naks += 1
        return NxsMap.APPLY_RUN_MS_FLOOR / 1000


def settle(bus, head: Head, counts: Counts, clock: Callable[[], float],
           sleep: Callable[[float], None], first_wait_s: float = 0.0) -> int:
    """`STATE` once it leaves BUSY: after `first_wait_s`, read every
    `APPLY_POLL_S` for `APPLY_POLL_MAX_S` past it at most, BUSY past that."""
    if first_wait_s:
        sleep(first_wait_s)
    start = clock()
    while True:
        state = read_state(bus, head, counts)
        if state != STATE_BUSY or clock() - start >= APPLY_POLL_MAX_S:
            return state
        sleep(APPLY_POLL_S)


def commit(bus, head: Head, staged: Dict[int, int], counts: Counts, clock: Callable[[], float],
           sleep: Callable[[float], None]) -> None:
    """One commit as the driver makes it: the painted apply time read, then
    `STATE` read and a run in flight waited out, the write, nothing for the
    painted time, then `STATE` polled out of BUSY before any other
    transaction. A door BUSY past the window or INERT before the write takes
    no commit and counts it failed; an apply that ends other than READY
    counts a fault. The latency is the write's to the door's READY."""
    painted = painted_s(bus, head, counts)
    if settle(bus, head, counts, clock, sleep) not in (STATE_READY, STATE_FAULT):
        counts.failed += 1
        return
    if not write(bus, head, staged, counts, sleep):
        return
    start = clock()
    state = settle(bus, head, counts, clock, sleep, first_wait_s=painted)
    counts.latencies_ms.append((clock() - start) * 1000.0)
    if state != STATE_READY:
        counts.faults += 1


def run(bus, heads: List[Head], period_s: float, seconds: float,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep) -> Dict[str, Counts]:
    """Commit to every head once a `period_s` for `seconds`, the exposure and
    the gain walking their ranges, each commit's apply waited out before the
    next transaction, and the unit door read once a second."""
    counts = {h.link: Counts() for h in heads}
    lock = port_state.BusLock()
    start = due = clock()
    frame = 0
    last_map = start
    while (now := clock()) - start < seconds:
        if not lock.acquire_now():
            for c in counts.values():
                c.skipped += 1
        else:
            try:
                sample = now - last_map >= 1.0
                for head in heads:
                    span = max(1, head.exposure_max_us - head.exposure_min_us)
                    exposure = head.exposure_min_us + (frame * 37) % span
                    step = max(1, head.gain_step_mdb)
                    steps = (head.gain_max_mdb - head.gain_min_mdb) // step + 1
                    gain = head.gain_min_mdb + ((frame * 3) % steps) * step
                    staged = {head.exposure_row: exposure, head.gain_row: gain}
                    commit(bus, head, staged, counts[head.link], clock, sleep)
                if sample:
                    last_map = now
                    for head in heads:
                        counts[head.link].map_reads += 1
                        try:
                            bus.read(head.map_addr, bytes(2), 1)
                        except OSError:
                            counts[head.link].map_errors += 1
            finally:
                lock.release()
        frame += 1
        due += period_s
        now = clock()
        if due < now:
            due += ((now - due) // period_s + 1) * period_s
        sleep(due - now)
    return counts


def verdict(c: Counts) -> Tuple[bool, str]:
    """R-10.13 on one head: repeats under 1 % of commits, no failed commit, no
    fault, every unit door read answered."""
    lat = sorted(c.latencies_ms) or [0.0]
    p50, worst = lat[len(lat) // 2], lat[-1]
    rate = (c.repeated / c.commits * 100.0) if c.commits else 0.0
    ok = c.commits > 0 and rate < 1.0 and c.failed == 0 and c.faults == 0 and c.map_errors == 0
    return ok, (f"{c.commits} commits, {c.naks} not acknowledged, {c.repeated} repeated "
                f"({rate:.2f} %), {c.failed} failed, {c.faults} faults, {c.skipped} skipped, "
                f"apply {p50:.1f} ms p50 / {worst:.1f} ms max, unit door reads {c.map_reads - c.map_errors}"
                f"/{c.map_reads}")


def _port(name: str):
    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config

    path = default_config_path()
    try:
        port = load_suite_config(path).ports.get(name)
    except ManifestError as exc:
        raise SystemExit(f"mapspike: {exc}") from exc
    if port is None:
        raise SystemExit(f"mapspike: {path} declares no port {name}")
    return port


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="mapspike",
                                     description="Commit at each pod's camera door once a frame.")
    parser.add_argument("port", help="the camera port, as the manifest names it (cam0)")
    parser.add_argument("--fps", type=float, default=50.0, help="commits per second per head (50)")
    parser.add_argument("--seconds", type=float, default=60.0, help="how long to run (60)")
    parser.add_argument("--link", action="append", help="a link to commit to (every pod link)")
    parser.add_argument("--bus", help="the port's I2C bus (the port's own)")
    args = parser.parse_args(argv)

    from nxs.cam import topology as cam_topo
    topology = cam_topo.port_topology(_port(args.port))
    links = [l for l in topology.links if l.nxs_units and (not args.link or l.name in args.link)]
    if not links:
        raise SystemExit(f"mapspike: {args.port} has no pod link to commit to")
    bus = _libnxs.Bus.open(args.bus or topology.i2c_bus)
    try:
        heads = [read_head(bus, l.name, int(l.nxs_units[0].alias_addr), int(l.nxs_units[0].camera_map_addr))
                 for l in links]
        for h in heads:
            print(f"{args.port}/{h.link}: camera door at {h.camera_map_addr:#04x}, unit door at {h.map_addr:#04x}, "
                  f"exposure {h.exposure_min_us}..{h.exposure_max_us} us, "
                  f"gain {h.gain_min_mdb}..{h.gain_max_mdb} mdB")
        print(f"committing at {args.fps:g} fps for {args.seconds:g} s")
        counts = run(bus, heads, 1.0 / args.fps, args.seconds)
    except OSError as exc:
        # A bus error names its device; a camera door's refusal is its own sentence.
        fact = f"{exc.filename}: {exc.strerror}" if exc.strerror else str(exc)
        raise SystemExit(f"mapspike: {fact}") from exc
    finally:
        bus.close()
    passed = True
    for link, c in counts.items():
        ok, line = verdict(c)
        passed = passed and ok
        print(f"{args.port}/{link}: {line}: {'pass' if ok else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
