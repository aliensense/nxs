# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The walks of a port's graph image, in the order the hub's flows bring a
port up, construct and train its links, sync it, lock its gain, park it,
gate it and retime it. A walk names the hub program's phases by their names
(its HUB_PHASES record) and stages its parameters by theirs; a link is its
row in GRAPH_LINKS, in the hub's link order. The order rules are the
flows': link B first and link A on B's warm links (`port.build_port`), the
primary's sensor parked while the addon comes up, a homogeneous pair's
start before the timing fix, a start where no rate fixed the frame, each
link's probes in the construction, a dip before every training pass but
the first, the trigger conversion a pod takes on its camera door (by its
personality's trigger action where the door serves no trigger row) and a
bare head by its personality's trigger action, the gain a pod commits on
its camera door and a bare head takes from its personality's apply. A port
with no hub walks its head alone: a pod's and a bare head's actions, a
smart camera's rows on its camera door."""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from nxs._generated_constants import NxsMap
from nxs.cam import hubimage as hi
from nxs.cam.contracts import InfeasibleConfig
from nxs.personality import records_graph as rg
from nxs.personality.records_fields import ACTIONS

ParamId = NxsMap.ParamId
Walk = rg.GraphWalk
CONFIGURE, PARK, START, TIMING, APPLY, TRIGGER = (
    ACTIONS[name] for name in ("configure", "park", "start", "timing", "apply", "trigger"))
#: The order the flows bring a pair's links up in: the primary, then the addon.
PAIR_ORDER = ("B", "A")


@dataclasses.dataclass(frozen=True)
class WalkFacts:
    """What the walks of one port are composed from: the hub program's
    phases and parameters by name, each link's row, the links the walks
    bring up (`present`, in the hub's order) and which of them carry a
    camera and a pod, each mapped link's packed address map, whether each
    camera link's frame is fixed by a rate law (the timing action, else the
    start), the homogeneous pair, port B, each camera link's parameter
    indices and fast-trigger value, each door camera's rows by `ParamId`
    (a pod's, a smart camera's), the links whose sensor takes the hub's
    trigger, whether the image carries the sync walk, the gain the gain
    walk commits on a camera door in millidecibels and stages on a bare
    head in its personality's capture units, whether the image carries
    only the walks that touch no head, whether the port has no hub, its
    smart cameras, and each camera link's mode value and rate in
    milli-frames per second, which a smart camera's door takes."""

    phases: Mapping[str, int]
    params: Mapping[str, int]
    rows: Mapping[str, int]
    present: Tuple[str, ...]
    cameras: Tuple[str, ...]
    pods: Tuple[str, ...]
    amap: Mapping[str, int]
    timed: Mapping[str, bool]
    homogeneous: bool = False
    port_b: bool = False
    indices: Mapping[str, Tuple[Optional[int], ...]] = dataclasses.field(default_factory=dict)
    trigger_free: Mapping[str, int] = dataclasses.field(default_factory=dict)
    trigger_fast: Mapping[str, Optional[int]] = dataclasses.field(default_factory=dict)
    doors: Mapping[str, Mapping[int, Tuple[int, int]]] = dataclasses.field(default_factory=dict)
    synced: Tuple[str, ...] = ()
    sync_walk: bool = False
    gain_mdb: Optional[int] = None
    gain_units: Mapping[str, int] = dataclasses.field(default_factory=dict)
    hub_only: bool = False
    direct: bool = False
    smart: Tuple[str, ...] = ()
    mode_values: Mapping[str, int] = dataclasses.field(default_factory=dict)
    rates_mfps: Mapping[str, Optional[int]] = dataclasses.field(default_factory=dict)


class _Steps:
    """The step makers over one port's facts: a phase and a parameter by
    name, a link by its name."""

    def __init__(self, facts: WalkFacts):
        self._f = facts

    def hub(self, phase: str, override: Optional[Tuple[str, int]] = None, soft: bool = False,
            again: bool = False) -> rg.GraphStep:
        if phase not in self._f.phases:
            raise InfeasibleConfig(f"the hub's program names no phase {phase}, which a port's "
                                   f"walks run", alternatives=["nxs assets install"])
        staged = None
        if override is not None:
            name, value = override
            if name not in self._f.params:
                raise InfeasibleConfig(f"the hub's program declares no parameter {name}",
                                       alternatives=["nxs assets install"])
            staged = (self._f.params[name], int(value))
        return rg.hub_step(self._f.phases[phase], staged, soft=soft, again=again)

    def pod(self, link: str, action: int, override: Optional[Tuple[int, int]] = None,
            soft: bool = False) -> rg.GraphStep:
        """A head's action. Every action past the configure stages the
        personality's `action` parameter: a personality without it runs
        its whole program alone, so no step starts, parks or retimes it."""
        if action != CONFIGURE and not self.acts(link):
            raise InfeasibleConfig(f"link {link}'s personality declares no action parameter, so "
                                   f"no step starts or parks its head",
                                   alternatives=["the action parameter, its start and park arms "
                                                 "writing the stream gate"])
        return rg.pod_step(self._f.rows[link], action, override, soft=soft)

    def door(self, link: str, pid: int, value: int, soft: bool = False) -> rg.GraphStep:
        rows = self._f.doors.get(link) or {}
        if int(pid) not in rows:
            name = ParamId._NAMES.get(int(pid), str(pid)).lower()
            raise InfeasibleConfig(f"link {link}'s personality serves no {name} row on its "
                                   f"camera door")
        row, kind = rows[int(pid)]
        return rg.map_step(self._f.rows[link], row, value, kind, soft=soft)

    def converge(self, link: str) -> rg.GraphStep:
        return rg.converge_step(self._f.rows[link])

    def host(self, op: int, link: Optional[str] = None) -> rg.GraphStep:
        return rg.host_step(op, self._f.rows[link] if link is not None else rg.NONE)

    def timing(self, link: str) -> rg.GraphStep:
        """The head's timing at the frame a rate law fixed, else its start."""
        return self.pod(link, TIMING if self._f.timed.get(link) else START)

    def acts(self, link: str) -> bool:
        """Whether a head's personality takes an action past its configure:
        a personality without the `action` parameter runs its whole program
        alone."""
        return self._f.indices.get(link, (None,) * 8)[rg.GraphParam.ACTION] is not None

    def gain(self, link: str) -> rg.GraphStep:
        """The declared gain on a head: a commit on a door camera's door in
        millidecibels, else the bare head's apply with its gain staged (the
        composer holds the gain to the head's law first)."""
        if link in self._f.doors:
            return self.door(link, ParamId.GAIN_MDB, int(self._f.gain_mdb))
        index = self._f.indices[link][rg.GraphParam.GAIN]
        return self.pod(link, APPLY, (index, int(self._f.gain_units[link])))

    def trigger(self, link: str, fast: bool, soft: bool = False) -> rg.GraphStep:
        """The trigger conversion of a bare head by its personality's
        trigger action: the fast trigger staged over the free run the row
        stages."""
        if not fast:
            return self.pod(link, TRIGGER, soft=soft)
        index = self._f.indices[link][rg.GraphParam.TRIGGER]
        value = self._f.trigger_fast.get(link)
        if index is None or value is None:
            raise InfeasibleConfig(f"link {link}'s personality offers no fast trigger")
        return self.pod(link, TRIGGER, (index, value), soft=soft)


def _shape(facts: WalkFacts) -> str:
    if len(facts.cameras) == 2:
        return "pair"
    return "pod" if not facts.cameras else "solo"


def _address_map(s: _Steps, facts: WalkFacts, link: str) -> List[rg.GraphStep]:
    if link not in facts.amap:
        return []
    return [s.hub("address_map", ("amap", facts.amap[link]))]


def _converged(s: _Steps, facts: WalkFacts, link: str, action: int) -> List[rg.GraphStep]:
    """A link's first turn: its pod brought to the declared build first."""
    steps = [s.converge(link)] if link in facts.pods else []
    return steps + [s.pod(link, action)]


def up(s: _Steps, facts: WalkFacts) -> List[rg.GraphStep]:
    """The bring-up: the port program's phases with the pods' turns between
    them (the retired walk's plan), the address maps where a link maps one,
    and port B's routing after the pair's addon rows or a solo's rows."""
    shape = _shape(facts)
    outputs = [s.hub("outputs")] if facts.port_b else []
    if shape == "pod":
        link = facts.present[0]
        return [s.hub("power"), s.hub("prologue")] + _address_map(s, facts, link)
    if shape == "solo":
        link = facts.cameras[0]
        return ([s.hub("power"), s.hub("prologue")] + _address_map(s, facts, link) + [s.hub("release")]
                + _converged(s, facts, link, CONFIGURE) + [s.hub("csi_rows")] + outputs
                + [s.timing(link), s.hub("addon_or_arm")])
    primary, addon = PAIR_ORDER
    # A pod's head answers with both links selected: the runtime window follows a phase that
    # narrows to one link before the pod's turn, and a one-serializer phase selects its own.
    both = [s.hub("runtime_window")]
    starts = ([s.pod(primary, START), s.pod(addon, START)] if facts.homogeneous else [])
    return ([s.hub("power"), s.hub("prologue")] + _address_map(s, facts, primary) + [s.hub("release")]
            + _converged(s, facts, primary, CONFIGURE)
            + [s.hub("csi_rows"), s.pod(primary, PARK), s.hub("addon_or_arm")]
            + _address_map(s, facts, addon) + both + _converged(s, facts, addon, CONFIGURE)
            + [s.hub("addon_rows")] + outputs + (both + starts if starts else [])
            + [s.hub("fix_primary")] + both + [s.timing(primary), s.hub("fix_addon")] + both
            + [s.timing(addon), s.hub("verify")])


def _in_order(facts: WalkFacts) -> List[str]:
    """The present links, the pair's primary first."""
    return [name for name in PAIR_ORDER if name in facts.present] + \
        [name for name in facts.present if name not in PAIR_ORDER]


def construct(s: _Steps, facts: WalkFacts) -> List[rg.GraphStep]:
    """A cold port's construction (`knobs.construct`): per link, the primary
    first, the probe at the hub's defaults, the serializer's program where a
    pristine serializer locked there (the phase reads the lock itself), and
    the probe at 12 Gbps. A pair ends on both links' one-shot, the one phase
    that reads both links' locks."""
    steps: List[rg.GraphStep] = []
    for link in _in_order(facts):
        value = ("link", hi.LINK_VALUE[link])
        steps += [s.hub("probe_6g", value), s.hub("construct", value), s.hub("probe_12g", value)]
    if len(facts.present) > 1:
        steps.append(s.hub("one_shot"))
    return steps


def train(s: _Steps, facts: WalkFacts) -> List[rg.GraphStep]:
    """One training pass (`training.train_links`): the heads' dip before
    every pass but the first, a pass per link, the primary first, then both
    links' one-shot, which hands the walk the links' locks."""
    return ([s.hub("dip", again=True)]
            + [s.hub("train", ("link", hi.LINK_VALUE[link])) for link in _in_order(facts)]
            + [s.hub("one_shot")])


def recover(s: _Steps, facts: WalkFacts) -> List[rg.GraphStep]:
    """The escalation before a second bring-up (`training.recover_links`):
    the long dip and the hub's reset; the policy trains after it."""
    del facts
    return [s.hub("dip_long"), s.hub("hub_reset")]


def status(s: _Steps, facts: WalkFacts) -> List[rg.GraphStep]:
    """A warm port's reachability (`knobs.reachable_steps`): both links'
    runtime window, then the lock, video and overflow bits read."""
    del facts
    return [s.hub("runtime_window"), s.hub("status")]


def _split(facts: WalkFacts, links: Sequence[str]) -> Tuple[List[str], List[str]]:
    """`links` in the hub's order, the bare heads and the pods' heads apart."""
    ordered = [name for name in facts.present if name in links]
    return ([n for n in ordered if n not in facts.pods], [n for n in ordered if n in facts.pods])


def _pod_trigger(s: _Steps, facts: WalkFacts, link: str, fast: bool,
                 soft: bool = False) -> rg.GraphStep:
    """A pod's trigger conversion: a commit on its camera door's trigger row,
    else its personality's trigger action, as a bare head converts."""
    if int(ParamId.TRIGGER) in (facts.doors.get(link) or {}):
        value = facts.trigger_fast[link] if fast else facts.trigger_free[link]
        if value is None:
            raise InfeasibleConfig(f"link {link}'s personality offers no fast trigger")
        return s.door(link, ParamId.TRIGGER, value, soft=soft)
    return s.trigger(link, fast=fast, soft=soft)


def sync(s: _Steps, facts: WalkFacts) -> List[rg.GraphStep]:
    """The declared frame sync (`fsync.build_fsync`): the generator over the
    free-running pair, each bare head's fast trigger by its personality's
    trigger action, the pipes re-armed where a bare head converted, else
    both links' window, each pod's fast trigger on its camera door, the
    video locks, then the owner's capture overlay for the trigger."""
    if not facts.sync_walk:
        return []
    bare, pods = _split(facts, facts.synced)
    return ([s.hub("fsync_on")] + [s.trigger(link, fast=True) for link in bare]
            + [s.hub("fsync_rearm" if bare else "runtime_window")]
            + [_pod_trigger(s, facts, link, fast=True) for link in pods]
            + [s.hub("video_lock"), s.host(rg.HostOp.OVERLAY)])


def free(s: _Steps, facts: WalkFacts) -> List[rg.GraphStep]:
    """The way back to free run (`fsync.build_trigger_off`): the generator
    off, each synced bare head's free run by its trigger action, each synced
    pod's on its camera door, then the owner's capture overlay."""
    if not facts.cameras:
        return []
    bare, pods = _split(facts, facts.synced)
    return ([s.hub("trigger_off")] + [s.trigger(link, fast=False) for link in bare]
            + [_pod_trigger(s, facts, link, fast=False) for link in pods]
            + [s.host(rg.HostOp.OVERLAY)])


def gain(s: _Steps, facts: WalkFacts) -> List[rg.GraphStep]:
    """The declared gain: behind the hub both links' runtime window first,
    then the gain on each camera head, committed on a door camera's door in
    millidecibels, staged under a bare head's apply in its capture units."""
    if facts.gain_mdb is None:
        return []
    window = [] if facts.direct else [s.hub("runtime_window")]
    return window + [s.gain(link) for link in facts.cameras]


def park(s: _Steps, facts: WalkFacts) -> List[rg.GraphStep]:
    """Every head in standby (the park `off` runs): free run first whatever
    the record says, the generator off and each synced head back, then the
    CSI output gated off and both links' window, then each bare head's and
    each pod's park action. Each step but the hub's park is soft: a park
    never stops at a silent head."""
    bare, pods = _split(facts, facts.synced)
    steps = ([s.hub("trigger_off", soft=True)] + [s.trigger(l, fast=False, soft=True) for l in bare]
             + [_pod_trigger(s, facts, l, fast=False, soft=True) for l in pods]
             + [s.hub("park")])
    heads, pod_heads = _split(facts, facts.cameras)
    return steps + [s.pod(link, PARK, soft=True) for link in heads + pod_heads]


def gate(s: _Steps, facts: WalkFacts, enable: bool) -> List[rg.GraphStep]:
    """Port A's gate, the capture verbs' choreography (`outputs.gate_steps`)."""
    if not facts.cameras:
        return []
    return [s.hub("gate_on" if enable else "gate_off", ("output", hi.OUTPUT_VALUE["A"]))]


def timing(s: _Steps, facts: WalkFacts) -> List[rg.GraphStep]:
    """Each head at the line the port runs (`port.build_timing`): a pair's
    timing tail, the primary first, its video locks verified; a solo's head
    and its pipe's video lock."""
    shape = _shape(facts)
    if shape == "pod":
        return []
    if shape == "solo":
        return [s.timing(facts.cameras[0]), s.hub("video_lock")]
    primary, addon = PAIR_ORDER
    both = [s.hub("runtime_window")]
    return ([s.hub("fix_primary")] + both + [s.timing(primary), s.hub("fix_addon")] + both
            + [s.timing(addon), s.hub("verify")])


def _direct(s: _Steps, facts: WalkFacts) -> Dict[int, List[rg.GraphStep]]:
    """The walks of a port with no hub, its one link's head alone: a smart
    camera's stream, mode and rate on its camera door; a pod's
    convergence, then its actions; a bare head's actions on the owner. A
    head steps its stream by its door's stream row where its door serves
    one, else by its start and its park. The port has no lock to construct
    or read, no generator to sync and no walk that touches no head."""
    if not facts.cameras or facts.hub_only:
        return {}
    link = facts.cameras[0]
    rate = facts.rates_mfps.get(link)
    if link in facts.smart:
        frame = [s.door(link, ParamId.FRAME_RATE_MFPS, rate)] if rate else []
        return {Walk.UP: [s.door(link, ParamId.STREAM, 0),
                          s.door(link, ParamId.MODE, facts.mode_values[link])] + frame
                         + [s.door(link, ParamId.STREAM, 1)],
                Walk.PARK: [s.door(link, ParamId.STREAM, 0, soft=True)],
                Walk.GATE_ON: [s.door(link, ParamId.STREAM, 1)],
                Walk.GATE_OFF: [s.door(link, ParamId.STREAM, 0)],
                Walk.TIMING: frame, Walk.GAIN: gain(s, facts)}
    up = _converged(s, facts, link, CONFIGURE) + [s.timing(link)]
    stream = int(ParamId.STREAM) in (facts.doors.get(link) or {})
    gate_on = s.door(link, ParamId.STREAM, 1) if stream else s.pod(link, START)
    gate_off = s.door(link, ParamId.STREAM, 0) if stream else s.pod(link, PARK)
    return {Walk.UP: up, Walk.PARK: [s.pod(link, PARK, soft=True)],
            Walk.GATE_ON: [gate_on], Walk.GATE_OFF: [gate_off],
            Walk.TIMING: [s.timing(link)], Walk.GAIN: gain(s, facts)}


def compose(facts: WalkFacts) -> Dict[int, Tuple[rg.GraphStep, ...]]:
    """Every walk the port's image carries, by `GraphWalk`; a walk the port
    has no step for is left out."""
    s = _Steps(facts)
    if facts.direct:
        return {walk: tuple(steps) for walk, steps in _direct(s, facts).items() if steps}
    if facts.hub_only:
        return {walk: tuple(steps) for walk, steps in (
            (Walk.CONSTRUCT, construct(s, facts)), (Walk.TRAIN, train(s, facts)),
            (Walk.RECOVER, recover(s, facts)), (Walk.STATUS, status(s, facts)),
            (Walk.GATE_ON, gate(s, facts, True)), (Walk.GATE_OFF, gate(s, facts, False)))
            if steps}
    walks = {Walk.UP: up(s, facts), Walk.CONSTRUCT: construct(s, facts),
             Walk.TRAIN: train(s, facts), Walk.RECOVER: recover(s, facts),
             Walk.STATUS: status(s, facts), Walk.SYNC: sync(s, facts), Walk.FREE: free(s, facts),
             Walk.GAIN: gain(s, facts), Walk.PARK: park(s, facts) if facts.cameras else [],
             Walk.GATE_ON: gate(s, facts, True), Walk.GATE_OFF: gate(s, facts, False),
             Walk.TIMING: timing(s, facts)}
    return {walk: tuple(steps) for walk, steps in walks.items() if steps}
