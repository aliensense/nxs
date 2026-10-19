"""The delivery check: after `on`, `set sync` and `set fps` the tool counts two seconds of frames on every camera link it ran and holds each to the rate it runs; `on` finds a hub pair's line with it."""

from __future__ import annotations

import contextlib
import dataclasses
import math
import re
import time
from typing import Callable, Dict, List, Optional

from nxs import host as host_layer
from nxs import term

from nxs.cam import capture, hubs, port_state, timing, viewers
from nxs.cam import run as cam_run
from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs.cam.select import _link_descriptor, _port_name, _refuse
from nxs.cam.verbs.sync import _fsync_plan, _record_sync
from nxs.finding import parse_refusal

#: How long the check counts at each link's rate, in seconds.
COUNT_S = 2.0
#: The fewest frames the check counts on a link, whatever its rate.
MIN_FRAMES = 3
#: How far a delivered rate may sit from the rate the link runs.
RATE_TOLERANCE = 0.01
#: A near miss (this close under the rate, or any count over it, every frame delivered, no error)
#: counts again, twice as long, before the refusal: a restarted capture daemon's first count is off.
NEAR_TOLERANCE = 0.025
#: Each step of the pair-line search lengthens the line to this percentage of it.
LINE_STEP_PERCENT = 105
#: The search runs no line past this multiple of the datasheet's.
LINE_BOUND = 2


def csi_gate(hub, flows, topology: Topology):
    """The CSI output-gate toggle every choreographed capture shares, each
    gate a walk of the port's graph image under the bus lock: a capture
    holds the bus while it switches the gate and leaves it free while its
    consumers start and run. Behind a hub the walk gates the hub's output
    and touches no head; a port with no hub gates its head's own stream,
    by its start and its park or its camera door's stream row. A gate that
    fails is a control-bus fault, raised as such."""
    del flows
    composed = []

    def gate(enable: bool) -> None:
        from nxs.cam import graph

        if not composed:
            image = graph.compose_port(hub, topology, heads=False)
            if not image.steps(graph.Walk.GATE_ON):
                # No hub's output stands before the head: the head gates itself.
                image = graph.compose_port(hub, topology)
            composed.append(image)
        walk = graph.Walk.GATE_ON if enable else graph.Walk.GATE_OFF
        if not graph.run(hub, topology, walk, composed[0]).ok:
            raise _refuse("csi-gate program failed (a control-bus fault)",
                          f"nxs {_port_name(topology)} status")
    return gate


def _no_capture_id(topology: Topology, link: LinkSpec) -> str:
    """Why a link has no capture id: no node for its channel, none the host reads, or no port state."""
    try:
        ids = host_layer.current().capture_ids(topology.i2c_bus)
    except Exception:
        ids = {}
    if ids and link.csi_vc not in ids:
        port = _port_name(topology)
        return (f"link {link.name} has no capture id on this boot: its virtual "
                f"channel {link.csi_vc} has no capture node in the booted overlay "
                f"(the label carries {', '.join(f'VC{vc}' for vc in sorted(ids))} only)\n"
                f"  - nxs switch (installs {port}'s overlay), then reboot")
    if not ids:
        try:
            booted = host_layer.current().booted_modes(topology.i2c_bus)
        except Exception:
            booted = []
        if booted:
            return (f"link {link.name} has no capture id: the booted tree carries "
                    f"{len(booted)} camera modes but no capture node this host reads\n"
                    f"  - nxs host info")
    return (f"link {link.name} has no capture id in the current port\n"
            f"  - nxs {_port_name(topology)} {link.name} on")


def frames_for(rate: Optional[float], seconds: float = COUNT_S) -> int:
    """The frames the check counts on a link running `rate`: `seconds` of
    them, never fewer than MIN_FRAMES."""
    if not rate:
        return MIN_FRAMES
    return max(MIN_FRAMES, math.ceil(seconds * float(rate)))


def asked_rate(topology: Topology, link: LinkSpec) -> Optional[float]:
    """The rate the check holds a link to: the one the port record says it
    runs (the generator's under frame sync, else the free-run rate `on` or
    `set fps` programmed), else the rate its recorded capture caps ask;
    None when the record holds neither."""
    rate = port_state.running_rate(port_state.port_record(topology), link.name)
    if rate is None:
        framerate = (port_state.viewer_hints(topology, link.name) or {}).get("framerate")
        if framerate:
            rate = float(framerate[0]) / float(framerate[1])
    return rate


def count(topology: Topology, links: List[LinkSpec], hub, frames_by_link: Dict[str, int],
          timeout_s: Optional[float] = None) -> Dict[str, str]:
    """Capture every link of `links` at once through the capture stack's
    consumer at its recorded caps, `frames_by_link[link]` frames each, behind
    one gate choreography. The gate takes the bus lock for each of its
    writes, so a caller holds none. Each link's consumer output by name."""
    hints_by_id: Dict[int, dict] = {}
    names: Dict[int, str] = {}
    for link in links:
        capture_id = port_state.port_capture_id(topology, link)
        if capture_id is None:
            raise SystemExit(_no_capture_id(topology, link))
        hints_by_id[capture_id] = viewers.resolve_capture_hints(
            port_state.viewer_hints(topology, link.name), link.name)
        names[capture_id] = link.name
    gate = csi_gate(hub, hub.flows(), topology)
    frames = {capture_id: int(frames_by_link[name]) for capture_id, name in names.items()}
    outputs = capture.headless_pair(hints_by_id, gate, frames, timeout_s=timeout_s,
                                    port=_port_name(topology))
    return {name: outputs.get(capture_id, "") for capture_id, name in names.items()}


def _rate(fps: float) -> str:
    return f"{round(float(fps), 2):g}"


def _asked_words(asked: Dict[str, Optional[float]]) -> str:
    """`60 fps asked`, or per link where the links run different rates."""
    rates = set(asked.values())
    if len(rates) == 1 and None not in rates:
        return f"{_rate(rates.pop())} fps asked"
    return "asked " + ", ".join(
        f"{_rate(rate)} fps on {name}" if rate is not None else f"{frames_for(None)} frames on {name}"
        for name, rate in asked.items())


def _lawful_rates(hub, topology: Topology, links: List[LinkSpec], top: int) -> List[int]:
    """The whole rates every link of `links` runs at on the port, in order:
    under frame sync the synced rates (`fsync_rates`), else the rates
    inside every link's free-run range, up to `top` for a hub without
    the range law."""
    modes = {link.name: str(port_state.port_mode(topology, link)) for link in links}
    at = topology.with_modes(modes)
    sync = port_state.port_sync(topology) or {}
    if sync.get("source") == "fsync":
        return timing.synced_rates(hub, at, modes)
    low, high = 1, None
    for link in links:
        rates = timing.lawful_range(hub, at, link, modes[link.name])
        if rates is not None:
            lo, hi = rates.whole()
            low, high = max(low, lo), hi if high is None else min(high, hi)
    return list(range(low, (top if high is None else high) + 1))


def judge(hub, topology: Topology, links: List[LinkSpec], outputs: Dict[str, str],
          retry: Optional[str] = None) -> Dict[str, float]:
    """Hold each link's count to the rate it runs: every frame asked
    delivered, no capture-stack error, and the rate the buffers' timestamps
    give within RATE_TOLERANCE of it. The delivered rate by link.

    Raises:
        InfeasibleConfig: Naming the rate asked and each link's delivered
            rate, with `retry` (a command, `{fps}` in it) at the highest
            lawful whole rate under the rate asked and at or under the
            lowest delivered (`_lawful_rates`), or at the lowest lawful
            rate with the fact that delivery fell under it; with the
            port's status alone when the capture stack reported errors;
            for a consumer that never built its pipeline, naming the
            host's reason.
    """
    port = _port_name(topology)
    asked = {link.name: asked_rate(topology, link) for link in links}
    delivered: Dict[str, Optional[float]] = {}
    words: List[str] = []
    missed = errored = False
    near = True
    for link in links:
        output = outputs.get(link.name, "")
        broken = capture.consumer_error(output)
        if broken:
            raise InfeasibleConfig(f"{port}/{link.name}: the capture consumer did not start: {broken}",
                                   alternatives=[host_layer.current().consumer_hint()])
        rate, wanted = asked[link.name], frames_for(asked[link.name])
        got, fps = capture.count_delivered(output), capture.delivered_rate(output)
        errors = capture.count_stack_errors(output)
        delivered[link.name] = fps
        off = fps is None or (rate is not None and abs(fps - rate) > RATE_TOLERANCE * rate)
        missed = missed or off or got < wanted or errors > 0
        errored = errored or errors > 0
        near = near and got >= wanted and not errors and fps is not None and rate is not None \
            and fps >= rate * (1 - NEAR_TOLERANCE)
        if fps is None:
            value = f"{got} of {wanted} frames" if got else "none"
        else:
            value = f"{fps:.1f}"
        text = f"{value} {'delivered on' if not words else 'on'} {link.name}"
        if fps is not None and got < wanted:
            text += f" ({got} of {wanted} frames)"
        if errors:
            text += f" with {errors} capture-stack error{'s' if errors > 1 else ''}"
        words.append(text)
    if not missed:
        return {name: float(fps) for name, fps in delivered.items() if fps is not None}
    fact = f"{port}: {_asked_words(asked)}, {', '.join(words)}"
    alternatives = [f"nxs {port} status"]
    measured = [fps for fps in delivered.values() if fps]
    known = [rate for rate in asked.values() if rate]
    # A capture-stack error fails the stream at any rate, so the port's
    # status is the one next step; a lower rate is advice only for a
    # source that delivers clean and slow.
    if retry and measured and not errored:
        # A rate at or above the one asked is no way out.
        top = math.floor(min(measured))
        if known:
            top = min(top, math.ceil(min(known)) - 1)
        lawful = _lawful_rates(hub, topology, links, top)
        under = [fps for fps in lawful if fps <= top]
        if under:
            alternatives = [retry.format(fps=under[-1])]
        elif lawful:
            fact += f", under the {lawful[0]} fps floor"
            alternatives = [retry.format(fps=lawful[0])]
    refusal = InfeasibleConfig(fact, alternatives=alternatives)
    refusal.near_miss = near
    raise refusal


def check(hub, topology: Topology, links: List[LinkSpec],
          retry: Optional[str] = None) -> Dict[str, float]:
    """The delivery check of `links`: the tool's own viewers on them stopped
    (the count opens their capture sessions), each link counted for COUNT_S
    at the rate it runs and judged, and the rates that pass recorded in the
    port record and printed.

    Raises:
        InfeasibleConfig: When a link misses (`judge`), and when the count
            does not run (a link without a capture id or caps, a bus another
            run holds, a gate that fails): nothing is verified then either.
    """
    if not links:
        return {}
    stop_viewers_for_count(topology, links)
    try:
        delivered = judge(hub, topology, links, _counted(hub, topology, links), retry=retry)
    except InfeasibleConfig as first:
        if not getattr(first, "near_miss", False):
            raise
        delivered = _recount(hub, topology, links, first, retry)
    _passed(hub, topology, links, delivered)
    return delivered


def _recount(hub, topology: Topology, links: List[LinkSpec], first: InfeasibleConfig,
             retry: Optional[str]) -> Dict[str, float]:
    """One more count after a near miss, twice as long, and `judge`'s
    verdict on it: that count decides either way."""
    term.info(f"{first.reason}, counted again for {2 * COUNT_S:g} s")
    return judge(hub, topology, links, _counted(hub, topology, links, seconds=2 * COUNT_S),
                 retry=retry)


def stop_viewers_for_count(topology: Topology, links: List[LinkSpec],
                           counts: str = "the delivery check") -> None:
    """Stop the tool's own viewers on `links` before a count and say so,
    `counts` naming the count: it opens their capture sessions, and a roll
    that gets none bounces the capture daemon, which would end a running
    viewer mid-stream."""
    for stopped in viewers.stop_viewers(topology, links):
        term.info(f"viewer for link {stopped} stopped: {counts} counts its frames")


def _counted(hub, topology: Topology, links: List[LinkSpec],
             seconds: float = COUNT_S) -> Dict[str, str]:
    """`count` at each link's rate for `seconds`; a count that does not run
    (a link without a capture id or caps, a bus another run holds, a gate
    that fails) is the check's refusal."""
    frames = {link.name: frames_for(asked_rate(topology, link), seconds) for link in links}
    try:
        return count(topology, links, hub, frames)
    except SystemExit as exc:
        raise _did_not_run(topology, exc) from exc


def _did_not_run(topology: Topology, exc: BaseException) -> InfeasibleConfig:
    """The check's refusal when its count or its probe did not run: nothing
    is verified then."""
    port = _port_name(topology)
    fact, alternatives = parse_refusal(str(exc))
    return InfeasibleConfig(f"{port}: the delivery check did not run: {fact}",
                            alternatives=alternatives or [f"nxs {port} status"])


def _passed(hub, topology: Topology, links: List[LinkSpec], delivered: Dict[str, float],
            note: str = "") -> None:
    port_state.set_verified(topology, delivered, time.time())
    print(f"{_port_name(topology)}: verified {summary(topology, delivered)}{note}")
    with contextlib.suppress(OSError, RuntimeError, SystemExit):
        _read_latched(hub, topology, links)


def _read_latched(hub, topology: Topology, links: List[LinkSpec]) -> None:
    """Read each counted link's serializer status rows once, through the
    link's window as `status` reads them, under the bus lock. A stream's
    start latches flags there (the deskew pattern on each lane, a sync
    error as a head restarted) and a read clears them, so a `status` after
    the check judges what happened since; an error flag found there is
    said. The caller lets a read that does not run pass: `status` reads
    the rows itself."""
    from nxs.cam.diag import run_probes

    if not any(link.ser_compatible for link in links):
        return
    flows = hub.flows()
    i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
    with port_state.BusLock():
        i2c.open()
        try:
            for link in links:
                try:
                    serd = hub.descriptor(link.ser_compatible)
                except hubs.HubError:
                    continue
                flows.open_window(hub, i2c, topology, link)
                for row in run_probes(i2c, link.ser_addr, serd):
                    if row.ok is False:
                        term.info(f"{_port_name(topology)}/{link.name}: {row.name} {row.text} "
                                  "latched before the count")
            flows.close_windows(hub, i2c, topology)
        finally:
            i2c.close()


def _lines_text(lines: Dict[str, int]) -> str:
    """`766`, or `A 580 and B 766` where the heads' lines differ."""
    if len(set(lines.values())) == 1:
        return str(next(iter(lines.values())))
    return " and ".join(f"{name} {line}" for name, line in sorted(lines.items()))


def _datasheet_lines(hub, topology: Topology, modes: Dict[str, str]) -> Dict[str, int]:
    """The line each named link's mode runs by the laws on the port, the
    deserializer datasheet's pair line on a pair, in HMAX."""
    plain = dataclasses.replace(topology, lines=()).with_modes(modes)
    by_name = {link.name: link for link in plain.links}
    return {name: int(_link_descriptor(hub, plain, by_name[name]).modes[mode]["timing"]["hmax"])
            for name, mode in modes.items()}


def _line_note(hub, topology: Topology) -> str:
    """`, line 766` where the record holds a line the delivery check found
    past the datasheet's; empty otherwise."""
    found = port_state.found_lines(topology)
    if not found:
        return ""
    lines = {name: hmax for name, (_mode, hmax) in found.items()}
    law = _datasheet_lines(hub, topology, {name: mode for name, (mode, _hmax) in found.items()})
    return f", line {_lines_text(lines)}" if lines != law else ""


def _overflowed(hub, topology: Topology) -> bool:
    """The hub's line-memory overflow probe (the hub's `line_overflow`),
    read under the bus lock; the read clears it."""
    with port_state.BusLock():
        i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        try:
            i2c.open()
            return bool(hub.flows().line_overflow(hub, i2c, topology))
        finally:
            i2c.close()


def _wider(topology: Topology) -> List[str]:
    """What carries a longer line: the hub's four CSI lanes where the port
    runs fewer and its connector wires four."""
    port = _port_name(topology)
    if int(topology.csi_lanes) < 4 and 4 in _connector_lanes(port):
        return [f"the host's {port}_lanes: 4"]
    return [f"nxs {port} caps"]


def _connector_lanes(port: str) -> tuple:
    """The lane counts the host's row for the connector offers; none for a
    connector the host does not describe."""
    try:
        return tuple(host_layer.current().description().row(f"{port}_lanes").values)
    except Exception:        # noqa: BLE001 (a host whose block does not build offers none)
        return ()


def search(hub, topology: Topology, links: List[LinkSpec], retry: str,
           record: Callable[[Dict[str, int]], Topology]) -> Dict[str, float]:
    """The delivery check of a pair behind the hub, finding the line the
    pair needs on this rig (requirements nxs-cam §8, R-8.2). The pair
    starts at the deserializer datasheet's line. A step reads the hub's
    line-memory overflow probe to clear it, counts every link and reads
    the probe again. An overflow lengthens the line to LINE_STEP_PERCENT
    of it, no further than LINE_BOUND times the datasheet's (the bound
    itself is tested): the laws judge the rates at the new line, the pair is retimed
    there (`build_timing`, and the trigger overlay again under frame sync)
    and `record(line)` writes the port up at it. A count that misses with
    the probe clear is refused at once: the line memory carries the line,
    so the shortfall is the consumer's or the sensor's. The step that
    passes records the line and the rates.

    Raises:
        InfeasibleConfig: The line memory overflowing up to LINE_BOUND times
            the datasheet's line; a rate the next line does not carry; a
            count that missed with the probe clear (`judge`'s refusal); a
            count that does not run.
    """
    port = _port_name(topology)
    flows = hub.flows()
    modes = {link.name: str(port_state.port_mode(topology, link)) for link in links}
    rates = {link.name: rate for link in links
             if (rate := port_state.port_rate(topology, link)) is not None}
    sync = port_state.port_sync(topology) or {}
    fps = float(sync["fps"]) if sync.get("source") == "fsync" and sync.get("fps") else None
    law = _datasheet_lines(hub, topology, modes)
    bound = {name: LINE_BOUND * line for name, line in law.items()}
    lines = dict(law)

    def at_lines(current: Dict[str, int]) -> Topology:
        return topology.with_lines({name: (modes[name], line) for name, line in current.items()})

    stop_viewers_for_count(topology, links)
    while True:
        try:
            _overflowed(hub, topology)
            outputs = _counted(hub, topology, links)
            overflow = _overflowed(hub, topology)
        except (OSError, RuntimeError, SystemExit) as exc:
            raise _did_not_run(topology, exc) from exc
        missed = None
        try:
            delivered = judge(hub, at_lines(lines), links, outputs, retry=retry)
        except InfeasibleConfig as exc:
            # A consumer that never ran says nothing about the line.
            if exc.reason.startswith(f"{port}/"):
                raise
            missed = exc
        if not overflow:
            if missed is not None:
                if not getattr(missed, "near_miss", False):
                    raise missed
                try:
                    delivered = _recount(hub, at_lines(lines), links, missed, retry)
                except (OSError, RuntimeError) as exc:
                    raise _did_not_run(topology, exc) from exc
            found = _remember(hub, topology, modes, fps, record, lines)
            _passed(hub, found, links, delivered, _line_note(hub, found))
            return delivered
        # Each line LINE_STEP_PERCENT of it, clamped to its bound: the bound
        # itself is tested before the search gives up.
        step = {name: min((line * LINE_STEP_PERCENT + 99) // 100, bound[name])
                for name, line in lines.items()}
        if step == lines:
            raise InfeasibleConfig(f"{port}: the line memory overflows up to {_lines_text(lines)} clocks",
                                   alternatives=_wider(topology))
        at = at_lines(step)
        try:
            # The laws judge the rates at the new line before the bus.
            flows.build_timing(hub, at, links, modes, rates)
            if fps is not None:
                flows.build_fsync(hub, at, fps=fps, method="manual", modes=modes)
        except InfeasibleConfig as exc:
            rate = [retry.format(fps=m.group(1)) for alt in exc.alternatives
                    if (m := re.fullmatch(r"fps (\d+)", alt))]
            raise InfeasibleConfig(f"{port}: the line memory overflows up to {_lines_text(lines)} "
                                   f"clocks; at {_lines_text(step)}, {exc.reason}",
                                   alternatives=rate[:1] + _wider(topology)) from exc
        term.info(f"{port}: the line memory overflowed at {_lines_text(lines)} clocks, "
                  f"the pair runs {_lines_text(step)} now")
        # At the new line: each head's timing walk stages the line it moves to,
        # then the sync walk converts the pair again under frame sync.
        if not _retimed(hub, at, links, modes, rates, fps):
            raise InfeasibleConfig(f"{port}: the pair's timing at {_lines_text(step)} clocks did not run",
                                   alternatives=[f"nxs {port} status"])
        _remember(hub, topology, modes, fps, record, step)
        lines = step


def _retimed(hub, at: Topology, links: List[LinkSpec], modes: Dict[str, str],
             rates: Dict[str, float], fps: Optional[float]) -> bool:
    """The pair retimed at the line `at` carries: the timing walk of the
    port's graph image composed there, then its sync walk under frame sync."""
    from nxs.cam import compose, graph

    composition = compose.compose(hub, at, links, modes, rates, fsync=fps)
    walks = [graph.Walk.TIMING] + ([graph.Walk.SYNC] if fps is not None else [])
    with port_state.BusLock():
        return all(graph.walk(at, composition, walk).ok for walk in walks)


def _remember(hub, topology: Topology, modes: Dict[str, str], fps: Optional[float],
              record: Callable[[Dict[str, int]], Topology], lines: Dict[str, int]) -> Topology:
    """Record the port up at `lines` and, under frame sync, its trigger at
    them; the port at `lines`."""
    at = record(lines)
    if fps is not None:
        flows = hub.flows()
        _record_sync(hub, flows, at, "fsync", fps, plan=_fsync_plan(flows, hub, at, fps, modes))
    return at


def _age(seconds: float) -> str:
    whole = int(seconds)
    if whole < 120:
        return f"{whole} s"
    if whole < 7200:
        return f"{whole // 60} min"
    return f"{whole // 3600} h"


def summary(topology: Topology, rates: Dict[str, float],
            age_s: Optional[float] = None) -> str:
    """`30.0 fps (A 30.0, B 30.0)`: the rate the links run, then the rate
    each delivered; with `age_s`, how long ago the check ran
    (`30.0 fps 12 s ago (…)`)."""
    by_name = {link.name: link for link in topology.links}
    running = sorted({round(rate, 1) for name in rates if name in by_name
                      and (rate := asked_rate(topology, by_name[name])) is not None})
    parts = [" and ".join(f"{rate:.1f}" for rate in running) + " fps"] if running else []
    if age_s is not None:
        parts.append(f"{_age(age_s)} ago")
    parts.append("(" + ", ".join(f"{name} {float(rate):.1f}"
                                 for name, rate in sorted(rates.items())) + ")")
    return " ".join(parts)


def verified_line(topology: Topology, hub) -> Optional[str]:
    """`cam0: verified 30.0 fps 12 s ago (A 30.0, B 30.0)`, the port's last
    passing check as `status` prints it, `, line 766` after it where the
    check found a line past the datasheet's; None when the record holds
    none."""
    seen = port_state.verified(topology)
    if not seen:
        return None
    age = max(0.0, time.time() - float(seen["at"]))
    return (f"{_port_name(topology)}: verified {summary(topology, seen['rates'], age)}"
            f"{_line_note(hub, topology)}")
