# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The host's side of a port's graph image (ADR 0030): the owner's
callbacks a walk calls back (the store, the port record, the host steps,
the log) and the calls the port verbs make. `nxs.cam.compose` builds the
image from the declaration; libnxs walks it (`nxs._libnxs_graph`).
`bring_up`, `walk` and `set_param` run under the caller's bus lock, `run`
takes it."""

from __future__ import annotations

import os
import re
from typing import Callable, List, Optional

from nxs import term
from nxs._libnxs_graph import (WALKS, GraphEnv, GraphRecord, GraphReport, HostOp, StepKind, Walk)
from nxs.cam import port_state
from nxs.cam.compose import Composition
from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs.personality import records_graph as rg

#: A step's line as the walk logs it: `up 3: pod B configure`.
_STEP = re.compile(r"^(?P<walk>[a-z-]+) (?P<index>\d+): (?P<rest>.*?)(?P<soft> failed, going on)?$")


def library() -> None:
    """The host carries the executor, or InfeasibleConfig with its own fact
    and the two ways to get it."""
    from nxs import _libnxs

    try:
        _libnxs.library_path()
    except _libnxs.LibraryMissing as exc:
        raise InfeasibleConfig(
            str(exc),
            alternatives=["pipx install --system-site-packages aliensense-nxs "
                          "(the platform wheel carries the executor)",
                          f"{_libnxs.ENV}=<path to libnxs.so>"]) from exc


class Owner(GraphEnv):
    """The host as a walk's owner: the composition's images, else the
    store's of the same build; the port record as `on` keeps it (warm while
    every link the image brings up is recorded up this boot, its links
    recorded unknown when a bring-up ends cold); and one line per step in
    the words the guides capture, a pod's turn as `pod B: configure
    (<personality>)` and a commit on a camera door as `link B: trigger 1 on its
    camera door at 0x35`. A hub phase, a convergence that changes nothing
    and a host step print under `NXS_CAM_VERBOSE=1` alone. A host step
    (the capture overlay of the trigger) is answered done: the verb writes
    the overlay over the port record it writes once the walk returns."""

    def __init__(self, topology: Topology, composition: Composition,
                 say: Optional[Callable[[str], None]] = None):
        super().__init__()
        self._topology = topology
        self._c = composition
        self._say = say or term.info
        self._verbose = bool(os.environ.get("NXS_CAM_VERBOSE"))

    def find_image(self, name: str, crc: int) -> Optional[bytes]:
        image = self._c.images.get((name, crc))
        return image if image is not None else _stored(name, crc)

    def record_get(self) -> Optional[GraphRecord]:
        present = self._present()
        states = port_state.recorded_states(self._topology)
        heads = tuple(port_state.head_values(self._topology, row.name, row.head_image_crc)
                      for row in self._c.links)
        return GraphRecord(warm=bool(present) and all(states.get(l.name) == port_state.STATE_UP
                                                      for l in present), heads=heads)

    def record_set(self, record: GraphRecord) -> None:
        # A warm record is the verb's to write, with the modes and rates the image lacks. A cold
        # walk's verdict marks the links unknown; a bare head's run values stay for its next run.
        if not record.warm and record.verdict != 0:
            port_state.mark_unknown(self._topology, self._present())
        port_state.set_head_values(
            self._topology,
            {row.name: values for row, values in zip(self._c.links, record.heads) if values is not None},
            {row.name: row.head_image_crc for row in self._c.links})

    def host(self, op: int, link: Optional[str], value: int) -> int:
        del op, link, value
        return 0

    def log(self, line: str) -> None:
        text = self.line(line)
        if text:
            self._say(text)

    def line(self, line: str) -> Optional[str]:
        """`line` as the tool prints it, None for one it keeps to the bench."""
        m = _STEP.match(line)
        if m is None or m["walk"] not in WALKS:
            return line
        step = self._step(m["walk"], int(m["index"]))
        if step is None:
            return line if self._verbose else None
        soft = ": failed, going on" if m["soft"] else ""
        text = self._words(step)
        if text is None:
            return line if self._verbose else None
        return text + soft

    def _present(self) -> List[LinkSpec]:
        """The declared links the image brings up."""
        rows = {row.name for row in self._c.links if row.present}
        return [link for link in self._topology.links if link.name in rows]

    def _step(self, walk: str, index: int) -> Optional[rg.GraphStep]:
        steps = self._c.steps(WALKS.index(walk))
        return steps[index] if index < len(steps) else None

    def _words(self, step: rg.GraphStep) -> Optional[str]:
        """A pod's turn and a door's commit in the guides' words; None for a
        step the bench alone reads."""
        row = self._c.links[step.link] if step.link < len(self._c.links) else None
        if step.kind == StepKind.POD and row is not None:
            action = rg.ACTIONS[step.op] if step.op < len(rg.ACTIONS) else str(step.op)
            who = row.personality if row.pod else "the host runs the sensor image"
            return f"pod {row.name}: {action} ({who})"
        if step.kind == StepKind.MAP and row is not None:
            head = self._c.heads.get(row.name)
            name, unit = head.names.get(step.op, (f"row {step.op}", "")) if head else (f"row {step.op}", "")
            value = f"{step.value} {unit}" if unit else f"{step.value}"
            return f"link {row.name}: {name} {value} on its camera door at {row.pod_map:#04x}"
        return None


def _stored(name: str, crc: int) -> Optional[bytes]:
    """The store's image `name` when its trailer is build `crc`."""
    from nxs import personality_cli
    from nxs.image import deserialize, trailer_bytes
    from nxs.personality.records_fields import trailer_crc

    try:
        source = personality_cli.resolve(name)
        if source.kind == personality_cli.KIND_IMAGE:
            image = source.image
        else:
            image = personality_cli.compile_source(source, {})[1]
        held = trailer_crc(trailer_bytes(deserialize(image).trailer))
    except (personality_cli.ResolveError, personality_cli.CompileFailed, ValueError):
        return None
    return image if held == crc else None


def _bus(topology: Topology):
    from nxs import _libnxs

    library()
    try:
        return _libnxs.Bus.open(topology.i2c_bus)
    except OSError as exc:
        raise InfeasibleConfig(f"{topology.i2c_bus} does not open: {exc.strerror or exc}",
                               alternatives=[f"ls -l {topology.i2c_bus}"]) from exc


def bring_up(topology: Topology, composition: Composition, owner: Owner) -> GraphReport:
    """The port's bring-up by the library's policy (`nxs_graph_up`)."""
    from nxs import _libnxs_graph

    bus = _bus(topology)
    try:
        return _libnxs_graph.up(bus, composition.image, owner)
    finally:
        bus.close()


def walk(topology: Topology, composition: Composition, which: int,
         owner: Optional[Owner] = None) -> GraphReport:
    """One walk of the port's image (`nxs_graph_walk`); a walk the image does
    not carry reports `-ENOENT`."""
    from nxs import _libnxs_graph

    bus = _bus(topology)
    try:
        return _libnxs_graph.walk(bus, composition.image, which,
                                  owner if owner is not None else Owner(topology, composition))
    finally:
        bus.close()


def set_param(topology: Topology, composition: Composition, link: str, param: int, value: int,
              owner: Optional[Owner] = None) -> GraphReport:
    """One parameter set on `link` outside any walk (`nxs_graph_set`): a
    commit of the camera door's row `param` behind a pod, the bare head's
    apply with its run parameter `param` staged; `-ENOENT` for a link the
    image does not carry."""
    from nxs import _libnxs_graph

    bus = _bus(topology)
    try:
        return _libnxs_graph.set_param(bus, composition.image, link, param, value,
                                       owner if owner is not None else Owner(topology, composition))
    finally:
        bus.close()


def set_head(topology: Topology, composition: Composition, link: str, param: int,
             value: int) -> Optional[str]:
    """A bare head's run parameter `param` staged at `value` under its
    apply on the host (`set_param`), under the caller's bus lock; behind a
    hub the set opens the hub's runtime window first. The reason the set
    did not take, else None; the set prints nothing."""
    owner = Owner(topology, composition, say=lambda _line: None)
    report = set_param(topology, composition, link, param, value, owner)
    return None if report.ok else report.reason()


def compose_port(hub, topology: Topology, links=None, *, rates=None, fsync=None,
                 outputs=None, heads: bool = True) -> Composition:
    """The port's graph image as it runs: each camera link in the mode its
    last `on` ran, else the declared or the laws' mode, at the rate the
    record holds for it unless `rates` names one, at the line the delivery
    check found; `fsync` the rate of the frame sync its sync walk runs.
    Without `heads`, the walks that touch no head alone (`compose`)."""
    from nxs.cam import compose
    from nxs.cam.run import _resolve_modes

    flows = hub.flows()
    topology = port_state.with_found_lines(topology)
    links = list(links or topology.links)
    cams = [link for link in links if link.has_camera]
    recorded = {l.name: m for l in cams if (m := port_state.port_mode(topology, l))}
    modes = _resolve_modes(flows, hub, cams, recorded or None, topology) if cams else {}
    if rates is None:
        rates = {l.name: r for l in cams if (r := port_state.port_rate(topology, l)) is not None}
    return compose.compose(hub, topology, links, modes, rates, fsync=fsync, outputs=outputs,
                           heads=heads)


def run(hub, topology: Topology, which: int, composition: Optional[Composition] = None) -> GraphReport:
    """One walk of the port's graph image under the bus lock: the image
    `compose_port` builds unless the caller composed its own."""
    composition = composition if composition is not None else compose_port(hub, topology)
    with port_state.BusLock():
        return walk(topology, composition, which)


def walked_locks(hub, topology: Topology) -> Optional[dict]:
    """A warm port's link locks as its status walk reads them, both links'
    runtime window then the hub's lock bits, by link name; None for a port
    not recorded up, for one whose image carries no status walk (no link
    stands between it and its heads), and for one the walk could not read,
    whose locks the probes read instead. The caller holds the bus lock; the
    walk prints nothing."""
    if not topology.links:
        return None
    states = port_state.recorded_states(topology)
    if any(states.get(link.name) != port_state.STATE_UP for link in topology.links):
        return None
    try:
        composition = compose_port(hub, topology, heads=False)
        if not composition.steps(Walk.STATUS):
            return None
        report = walk(topology, composition, Walk.STATUS,
                      Owner(topology, composition, say=lambda _line: None))
    except (InfeasibleConfig, OSError):
        return None
    if not report.ok:
        return None
    declared = {link.name for link in topology.links}
    return {row.name: bool(report.locked & (1 << i)) for i, row in enumerate(composition.links)
            if row.name in declared}


def vlock_failed(composition: Composition, report: GraphReport) -> bool:
    """Whether a walk ended on a video lock its hub phase polled for."""
    return (report.kind == StepKind.HUB and report.run.reg is not None
            and report.run.reg in composition.walks.vlock)


__all__ = ["HostOp", "Owner", "Walk", "bring_up", "compose_port", "library", "run", "set_head",
           "set_param", "vlock_failed", "walk", "walked_locks"]
