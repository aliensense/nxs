#!/usr/bin/env python3
# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""gainpair: a synced pair's two analog gain registers, sampled once a frame.

A bench tool beside `nxs`, not one of its verbs. Once a frame period at the
port's rate it takes the bus lock every tool run shares in one attempt, reads
the leading head's gain register and the following head's at their host
addresses, and prints a line whenever either changes. A frame another run
holds the bus in is skipped. A leader value other than the last one taken is
read again and taken only when both reads agree, as nxsd's follower takes it:
the leader's driver writes the bytes under its own hold.

A sample is in step when the following head holds the leading head's gain, or
the gain the leading head held at the sample before: the follower copies the
leader's register within the frame after the leader's driver writes it, so
while the leader's loop moves its gain every frame the follower trails it by
one frame at every sample. At the end the tool prints the samples, the
leader's changes, the samples the follower trails by one frame in, and the
samples out of step, with the longest out-of-step stretch (from its first
sample to the first in-step one, or to the run's end) and the widest gap
among them in dB.

The heads, the register and its dB step are the ones nxsd's follower copies,
the pack's `build_follow` for the port: the port record's leader and follower,
else the port's first two camera links (a pair `camera.gain_db` locks).
`--leader`, `--follower` and `--reg` replace the addresses and the register.
The bus is the libnxs handle (ADR 0003).
"""

from __future__ import annotations

import argparse
import dataclasses
import errno
import math
import sys
import time
from typing import Callable, List, Optional, Tuple

from nxs import _libnxs
from nxs.cam.contracts import FollowPlan

#: The rate a port runs at when its record names none.
DEFAULT_FPS = 30.0


@dataclasses.dataclass(frozen=True)
class Sample:
    """Both heads' raw gain register values, `t` seconds into the run."""

    t: float
    slot: int
    leader: int
    follower: int


@dataclasses.dataclass(frozen=True)
class Run:
    """What a run saw: the samples it took, the frames it skipped, and the
    second it ended at."""

    samples: Tuple[Sample, ...]
    skipped: int
    ended: float


@dataclasses.dataclass(frozen=True)
class Summary:
    """A run's figures: samples, skipped frames, the leader's changes, the
    samples the follower trails the leader by one frame in, and the samples
    out of step with the longest stretch of them in ms and the widest gap
    among them in dB."""

    samples: int
    skipped: int
    changes: int
    behind: int
    out_of_step: int
    after_gaps: int
    longest_out_of_step_ms: float
    max_out_of_step_db: float


def plan_for(port) -> Tuple[FollowPlan, str, float]:
    """What nxsd's follower copies on a declared port (`port`, the manifest's
    port spec), the port's bus and the rate its record runs the leader at."""
    from nxs.cam import packs, port_state
    from nxs.cam import topology as cam_topo
    from nxs.cam.contracts import ContractError

    try:
        topology = cam_topo.port_topology(port)
        ae = (port_state.port_sync(topology) or {}).get("ae") or {}
        names = ([str(ae["leader"]), str(ae["follower"])] if ae.get("mode") == "follow"
                 else [link.name for link in topology.camera_links][:2])
        if len(names) < 2:
            raise SystemExit(f"gainpair: {port.name} runs no pair")
        pack = packs.pack_for(topology)
        build = getattr(pack.flows(), "build_follow", None)
        if build is None:
            raise SystemExit(f"gainpair: pack {pack.name} names no gain register to follow")
        plan = build(pack, topology, *names)
    except (ContractError, cam_topo.TopologyError, packs.PackError) as exc:
        raise SystemExit(f"gainpair: {exc}") from exc
    rate = port_state.running_rate(port_state.port_record(topology), names[0])
    return plan, topology.i2c_bus, float(rate or DEFAULT_FPS)


def _declared(name: str):
    """The manifest's spec of the port `name`."""
    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config

    path = default_config_path()
    try:
        port = load_suite_config(path).ports.get(name)
    except ManifestError as exc:
        raise SystemExit(f"gainpair: {exc}") from exc
    if port is None:
        raise SystemExit(f"gainpair: {path} declares no port {name}")
    return port


def _gain(bus, plan: FollowPlan, addr: int) -> int:
    reg, width, order = plan.register
    raw = bus.read(addr, reg.to_bytes(2, "big"), width)
    return int.from_bytes(raw, "little" if order == "le" else "big")


def _read_pair(bus, plan: FollowPlan, taken: Optional[int]) -> Optional[Tuple[int, int]]:
    """Both heads' gain in one hold of the bus lock; None for a frame another
    run holds the bus in, or a changed leader value two reads disagree on."""
    try:
        bus.lock(0)
    except OSError as exc:
        if exc.errno != errno.EBUSY:
            raise
        return None
    try:
        leader = _gain(bus, plan, plan.leader_addr)
        if leader != taken and _gain(bus, plan, plan.leader_addr) != leader:
            return None
        return leader, _gain(bus, plan, plan.follower_addr)
    finally:
        bus.unlock()


def change_line(taken: Sample, plan: FollowPlan) -> str:
    """`t=12.345 A=21.4dB B=21.4dB`: both heads' gain at a sample."""
    return (f"t={taken.t:.3f} {plan.leader}={taken.leader * plan.db_per_step:.1f}dB "
            f"{plan.follower}={taken.follower * plan.db_per_step:.1f}dB")


def sample(bus, plan: FollowPlan, period_s: float, seconds: float,
           clock: Callable[[], float] = time.monotonic,
           sleep: Callable[[float], None] = time.sleep,
           out: Callable[[str], None] = print) -> Run:
    """Sample both heads once a `period_s` on the monotonic `clock` for
    `seconds` (a slot a read overran is skipped, never made up), printing a
    `change_line` to `out` whenever either head's gain changes. Ctrl+C ends
    the run early. OSError: the bus's."""
    start = due = clock()
    samples: List[Sample] = []
    skipped = slot = 0
    shown: Optional[Tuple[int, int]] = None
    try:
        while (now := clock()) - start < seconds:
            got = _read_pair(bus, plan, samples[-1].leader if samples else None)
            if got is None:
                skipped += 1
            else:
                samples.append(Sample(now - start, slot, *got))
                if got != shown:
                    out(change_line(samples[-1], plan))
                    shown = got
            due += period_s
            slot += 1
            now = clock()
            if due < now:
                missed = math.ceil((now - due) / period_s)
                due += missed * period_s
                slot += missed
            sleep(due - now)
    except KeyboardInterrupt:
        pass
    return Run(tuple(samples), skipped, clock() - start)


def summarize(run: Run, plan: FollowPlan) -> Summary:
    """The run's figures. A sample is in step when the follower holds the
    leader's gain or, in the slot right after the sample before, that
    sample's leader gain (one frame behind); it is out of step when it holds
    neither and the sample before is the previous slot; after a skipped
    slot a sample that differs from its leader is not judged (the leader's
    gain of the missing frame is unknown) and counted apart. An out-of-step
    stretch lasts from its first sample to the first in-step one, or to the
    run's end."""
    changes = behind = out_of_step = after_gaps = widest = 0
    longest = 0.0
    since: Optional[float] = None
    for before, s in zip((None, *run.samples), run.samples):
        if before is not None and s.leader != before.leader:
            changes += 1
        if s.follower != s.leader:
            adjacent = before is not None and s.slot == before.slot + 1
            if adjacent and s.follower == before.leader:
                behind += 1
            elif adjacent or before is None:
                out_of_step += 1
                widest = max(widest, abs(s.leader - s.follower))
                since = s.t if since is None else since
                continue
            else:
                after_gaps += 1
                continue
        if since is not None:
            longest = max(longest, s.t - since)
            since = None
    if since is not None:
        longest = max(longest, run.ended - since)
    return Summary(samples=len(run.samples), skipped=run.skipped, changes=changes,
                   behind=behind, out_of_step=out_of_step, after_gaps=after_gaps,
                   longest_out_of_step_ms=longest * 1000.0,
                   max_out_of_step_db=widest * plan.db_per_step)


def summary_line(summary: Summary, plan: FollowPlan) -> str:
    """`summary: 1797 samples, 3 skipped, A changed 24 times, B one frame
    behind 22 times, out of step 0 samples (longest 0.0 ms, max 0.0 dB)`."""
    changed = "time" if summary.changes == 1 else "times"
    behind = "time" if summary.behind == 1 else "times"
    out = "sample" if summary.out_of_step == 1 else "samples"
    return (f"summary: {summary.samples} samples, {summary.skipped} skipped, "
            f"{plan.leader} changed {summary.changes} {changed}, "
            f"{plan.follower} one frame behind {summary.behind} {behind}, "
            f"out of step {summary.out_of_step} {out} "
            f"(longest {summary.longest_out_of_step_ms:.1f} ms, "
            f"max {summary.max_out_of_step_db:.1f} dB)"
            + (f", {summary.after_gaps} after skipped slots not judged" if summary.after_gaps else ""))


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="gainpair", description="Sample a synced pair's two gain registers once a frame.")
    parser.add_argument("port", help="the camera port, as the manifest names it (cam0)")
    parser.add_argument("--seconds", type=float, default=60.0, help="how long to sample (60)")
    parser.add_argument("--bus", help="the port's I2C bus (the port's own)")
    parser.add_argument("--leader", type=lambda s: int(s, 0),
                        help="the leading head's host address (0x1e)")
    parser.add_argument("--follower", type=lambda s: int(s, 0),
                        help="the following head's host address (0x1b)")
    parser.add_argument("--reg", type=lambda s: int(s, 0), help="the gain register (0x3514)")
    args = parser.parse_args(argv)

    plan, bus_path, fps = plan_for(_declared(args.port))
    plan = dataclasses.replace(
        plan,
        leader_addr=plan.leader_addr if args.leader is None else args.leader,
        follower_addr=plan.follower_addr if args.follower is None else args.follower,
        register=(plan.register[0] if args.reg is None else args.reg, *plan.register[1:]))
    print(f"sampling {args.port}: {plan.leader} at {plan.leader_addr:#04x}, "
          f"{plan.follower} at {plan.follower_addr:#04x}, gain register {plan.register[0]:#06x}, "
          f"{fps:g} fps for {args.seconds:g} s")
    bus = _libnxs.Bus.open(args.bus or bus_path)
    try:
        run = sample(bus, plan, 1.0 / fps, args.seconds)
    except OSError as exc:
        raise SystemExit(f"gainpair: {exc.filename}: {exc.strerror}") from exc
    finally:
        bus.close()
    print(summary_line(summarize(run, plan), plan))
    return 0


if __name__ == "__main__":
    sys.exit(main())
