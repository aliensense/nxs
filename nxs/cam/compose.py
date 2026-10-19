# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""A port's graph image, composed from what `on` knows of its declaration
(ADR 0030): the hub's program as the owner's store holds it, under
GRAPH_SOURCE (the program and the declaration's digest), GRAPH_LINKS (each
link's addresses, its camera personality by name and CRC, its staging),
GRAPH_OUTPUTS (the hub's CSI outputs) and GRAPH_WALKS (`compose_walks`),
sealed and checked by `records_graph.graph_trailer`. A port with no hub
composes against the connector's program (`direct.program`), names no
deserializer and walks its head alone. The composer reads the program's
phases and parameters by their names and writes nothing to a bus: the
library walks the image (`nxs._libnxs_graph`)."""

from __future__ import annotations

import dataclasses
import json
import os
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from nxs._generated_constants import NxsMap
from nxs.cam import compose_walks, hubimage as hi
from nxs.cam.compose_heads import Head, gain_units, head
from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs.personality import records_graph as rg

ParamId = NxsMap.ParamId
GraphParam = rg.GraphParam
#: Training passes before the policy stops training a link, and the passes
#: a recovery runs: the flows' rounds of the dip-retry handshake.
TRAIN_ROUNDS = 6
@dataclasses.dataclass(frozen=True)
class Composition:
    """A port's graph image and what it was composed from: the port's name,
    its link rows in the hub's order, its walks, the program's phases by
    value, the images its walks name by `(name, CRC)` for the owner's store,
    the declaration text its digest is taken over, and the pods' camera
    doors by link."""

    image: bytes
    name: str
    links: Tuple[rg.GraphLink, ...]
    walks: rg.GraphWalks
    phases: Mapping[int, str]
    images: Mapping[Tuple[str, int], bytes]
    declaration: str
    heads: Mapping[str, Head] = dataclasses.field(default_factory=dict)

    def steps(self, walk: int) -> Tuple[rg.GraphStep, ...]:
        return tuple(self.walks.walks.get(int(walk), ()))

    def text(self, walk: int) -> List[str]:
        """Each step of `walk` as `nxs personality records` prints it."""
        return [rg.step_text(step, self.links, self.phases) for step in self.steps(walk)]


# ── what the owner holds ─────────────────────────────────────────────

def hub_program(hub) -> bytes:
    """The hub's program as this host holds it: the store's image, else, in
    a source tree, the hub's program class compiled; the connector's for a
    port with no hub. InfeasibleConfig names what this host lacks."""
    from nxs import personality_cli
    from nxs.cam import hubs
    from nxs.compiler import HubDevice
    from nxs.image import ImageKind, peek_format, serialize

    if isinstance(hub, hubs.Connector):
        from nxs.cam import direct

        return direct.program()
    chips = [c for c in hub.chips if hub.descriptor(c).role == "DES"]
    if len(chips) != 1:
        raise InfeasibleConfig(f"hub {hub.name} ships no hub image: it names no deserializer")
    chip = chips[0]
    stored = os.path.join(personality_cli.store_dir(), f"{chip}{personality_cli.IMAGE_SUFFIX}")
    if os.path.isfile(stored):
        try:
            source = personality_cli._image_source(stored, chip, origin="store")
        except personality_cli.ResolveError as exc:
            raise InfeasibleConfig(f"{stored}: {exc}", alternatives=["nxs assets install"]) from exc
        if peek_format(source.image)[2] != ImageKind.HUB:
            raise InfeasibleConfig(f"{stored} is not a hub image", alternatives=["nxs assets install"])
        return source.image
    # A source tree compiles the program from the hub's class; the wheel's
    # hub ships no sensors to compose from, so its program is the store's.
    path = hub.chip_source(chip) if hub.sensors() else None
    cls = personality_cli._class_in(str(path), chip) if path is not None else None
    if cls is None or not issubclass(cls, HubDevice):
        raise InfeasibleConfig(f"no hub image for {chip}: nothing at {stored}",
                               alternatives=["nxs assets install"])
    return serialize(cls().compile({}))


def address_entries(hub, spec: LinkSpec) -> List[Tuple[int, int]]:
    """The `(alias, target)` pairs a link's serializer translates. A pod
    takes both: its register map at the manifest's alias and its camera
    map, which the kernel driver writes, one above, so the host never
    addresses the sensor on that link. A head without a pod takes one, its
    capture node's address mapped to the address it straps (the hub's
    `_address_map` rule)."""
    from nxs.cam.hubs import native_sensor_address

    entries: List[Tuple[int, int]] = []
    for u in spec.nxs_units:
        if int(u.alias_addr) != int(u.target_addr):
            entries.append((int(u.alias_addr), int(u.target_addr)))
        if int(u.camera_map_addr) != int(u.map_target):
            entries.append((int(u.camera_map_addr), int(u.map_target)))
    if not spec.nxs_units:
        native = native_sensor_address(hub, spec)
        if spec.host_addr is not None and int(spec.host_addr) != native:
            entries.append((int(spec.host_addr), native))
    if len(entries) > hi.ADDRESS_MAP_MAX:
        raise InfeasibleConfig(f"link {spec.name}: the serializer has two translation slots "
                               f"and {len(entries)} devices ask for one")
    return entries


def homogeneous(hub, links: Sequence[LinkSpec], modes: Mapping[str, str]) -> bool:
    """Whether a pair runs one sensor kind in one mode at one address: the
    walk then starts both sensors before the timing fix, as the captured
    program does."""
    from nxs.cam.hubs import sensor_address

    if len(links) != 2 or not all(link.has_camera for link in links):
        return False
    a, b = links
    return (a.sensor_compatible == b.sensor_compatible and modes[a.name] == modes[b.name]
            and sensor_address(hub, a) == sensor_address(hub, b))


def _refuse_shape(hub, topology: Topology, links: Sequence[LinkSpec]) -> None:
    """The shapes a port's walks bring up: one link or the pair of links A
    and B, a pod alone on its own link, on CSI lanes the hub's output
    takes, each device at a host address of its own."""
    check = getattr(hub.flows(), "require_distinct_host_addresses", None)
    if check is not None:
        check(hub, topology)
    names = sorted(link.name for link in links)
    pods = [link.name for link in links if not link.has_camera]
    if pods and len(links) > 1:
        raise InfeasibleConfig(f"link {pods[0]} carries a pod and no camera, and one hub brings a "
                               f"pod alone up on its own",
                               alternatives=[f"link {name}" for name in names])
    if len(links) == 2 and names != ["A", "B"]:
        raise InfeasibleConfig(f"a pair is links A and B; got {names}")
    if not 1 <= len(links) <= 2:
        raise InfeasibleConfig("a walk brings up one link or the pair")
    if int(topology.csi_lanes) not in hi.CSI_LANES:
        raise InfeasibleConfig(f"the hub's CSI output takes 2 or 4 lanes, not {topology.csi_lanes}")


# ── the link rows ────────────────────────────────────────────────────

def _frame_lengths(hub, port: Topology, links: Sequence[LinkSpec], modes: Mapping[str, str],
                   rates: Mapping[str, float]) -> Dict[str, Optional[int]]:
    """The frame length each camera link's timing writes, by its family's
    frame law at the line the port runs it and the rate it runs; None
    where a sensor has no frame law (its tables set the frame) or no rate
    is known."""
    module_of = getattr(hub.flows(), "sensor_module", None)
    frames: Dict[str, Optional[int]] = {}
    for spec in links:
        frames[spec.name] = None
        if not spec.has_camera or module_of is None or spec.name not in rates:
            continue
        module = module_of(hub, spec, port)
        if hasattr(module, "vmax_for_fps"):
            hmax = int(module.descriptor().modes[modes[spec.name]]["timing"]["hmax"])
            frames[spec.name] = int(module.vmax_for_fps(float(rates[spec.name]), hmax))
    return frames


def _row(hub, spec: Optional[LinkSpec], name: str, present: bool, direct: bool) -> rg.GraphLink:
    """A link's row before its staging: the addresses, the map and the pod,
    the camera door of a pod or a smart camera; a hub link the declaration
    leaves empty is a row of its name alone. A link on the port's own bus
    names no serializer or expander and translates nothing."""
    from nxs.cam.hubs import map_address, sensor_address

    if spec is None:
        return rg.GraphLink(name=name, present=False, camera=False)
    unit = spec.nxs_units[0] if spec.nxs_units else None
    door = map_address(hub, spec) if spec.has_camera or unit else None
    return rg.GraphLink(
        name=name, head_addr=int(sensor_address(hub, spec)) if spec.has_camera else 0,
        present=present, camera=spec.has_camera, ser_addr=0 if direct else int(spec.ser_addr),
        tca_addr=0 if direct else int(spec.tca_addr),
        entries=() if direct else tuple(address_entries(hub, spec)),
        pod=unit is not None, pod_alias=int(unit.alias_addr) if unit else 0,
        pod_target=int(unit.target_addr) if unit else 0, pod_map=int(door or 0))


def _staged(row: rg.GraphLink, head: Head, facts: Mapping[str, object],
            periods: Mapping[str, int], frame_length: Optional[int],
            gain_mdb: Optional[int], exposure_us: Optional[float]) -> rg.GraphLink:
    """A camera link's row with its personality and the values a walk stages:
    a pod's personality by name, a bare head's image the owner runs, and
    neither for a smart camera, which runs no image of the host's."""
    bare = not row.pod and not row.pod_map
    return dataclasses.replace(
        row, lanes=int(facts["lanes"]), data_type=hi.DATA_TYPES[str(facts["data_type"])],
        host_csi=bool(facts["host_csi"]), frame_fixed=frame_length is not None,
        personality=head.name if row.pod else "", personality_crc=head.crc if row.pod else 0,
        head_image=head.name if bare else "", head_image_crc=head.crc if bare else 0,
        params=head.params, mode=head.mode, trigger_free=head.trigger_free,
        trigger_fast=head.trigger_fast, line_time_ns=periods["line"],
        frame_period_ns=periods["frame"], gain_mdb=int(gain_mdb or 0),
        exposure_us=int(round(exposure_us)) if exposure_us else 0)


# ── the outputs and the declaration ──────────────────────────────────

def _outputs(hub, topology: Topology, cameras: Sequence[str], outs) -> List[rg.GraphOutput]:
    """The hub's CSI outputs as the port runs them: port A to the host with
    the pipes it carries, port B to the module when the declaration puts it
    up, each with the virtual channel of each link's stream. The program's
    `streams` bits are the record's (`GRAPH_STREAM_*`)."""
    if not cameras:
        return []
    flows = hub.flows()
    des = next(c for c in hub.chips if hub.descriptor(c).role == "DES")
    rate = int(hub.descriptor(des).limits.get("csi_tx_mbps_per_lane", 0))
    vc = getattr(flows, "LINK_VC", {"A": 1, "B": 0})
    vcs = {"vc_a": int(vc["A"]) if "A" in cameras else 0,
           "vc_b": int(vc["B"]) if "B" in cameras else 0}
    mask = _stream_mask(flows, outs, cameras)
    on_a = mask & (hi.STREAM_BIT[("Y", "A")] | hi.STREAM_BIT[("Z", "A")])
    on_b = mask & (hi.STREAM_BIT[("Y", "B")] | hi.STREAM_BIT[("Z", "B")])
    b = (outs or {}).get("B")
    up_b = b is not None and bool(b.lanes)
    shared = bool((on_b >> 2) & on_a)
    mode_a = (rg.OutputMode.OFF if not on_a else rg.OutputMode.EXCLUSIVE if not up_b
              else rg.OutputMode.DUPLICATE if shared else rg.OutputMode.SPLIT)
    rows = [rg.GraphOutput(output=0, lanes=int(topology.csi_lanes), rate_mbps=rate, streams=on_a,
                           mode=mode_a, consumer=rg.Consumer.HOST, **vcs)]
    if up_b:
        mode_b = (rg.OutputMode.REPLICA if b.copy else rg.OutputMode.EXCLUSIVE if not on_a
                  else rg.OutputMode.DUPLICATE if shared else rg.OutputMode.SPLIT)
        rows.append(rg.GraphOutput(output=1, lanes=int(b.lanes), rate_mbps=rate, streams=on_b,
                                   mode=mode_b, consumer=rg.Consumer.MODULE, **vcs))
    return rows


def _pipes(flows, cameras: Sequence[str]) -> Dict[str, str]:
    """The pipe each camera link rides: a solo rides Y whatever its link."""
    if len(cameras) == 1:
        return {"Y": cameras[0]}
    link_pipe = getattr(flows, "LINK_PIPE", {"A": "Y", "B": "Z"})
    return {link_pipe[name]: name for name in cameras}


def _stream_mask(flows, outs, cameras: Sequence[str]) -> int:
    """The program's `streams`: a bit per pipe and output a stream rides."""
    pipes = _pipes(flows, cameras)
    if outs and outs.get("B") is not None and outs["B"].lanes and hasattr(flows, "stream_mask"):
        return int(flows.stream_mask(outs, pipes))
    return sum(hi.STREAM_BIT[(pipe, "A")] for pipe in pipes)


def declaration(topology: Topology, links: Sequence[LinkSpec], modes: Mapping[str, str],
                rates: Mapping[str, float], fsync: Optional[float], up_sync: bool,
                gain_db: Optional[float],
                outs) -> str:
    """The text GRAPH_SOURCE's digest is taken over: every fact the
    composition reads, in one canonical JSON document, the manifest's own
    digest of the port among them where the port is declared."""
    from nxs.cam.select import _port_name

    doc = {
        "port": _port_name(topology), "bus": topology.i2c_bus, "hub": topology.des_compatible,
        "hub_addr": int(topology.des_addr), "csi_lanes": int(topology.csi_lanes),
        "declared": topology.declared, "lines": [list(line) for line in topology.lines],
        "links": {l.name: {"sensor": l.sensor_compatible, "mode": modes.get(l.name),
                           "fps": rates.get(l.name), "host_addr": l.host_addr,
                           "units": [[int(u.alias_addr), int(u.camera_map_addr)]
                                     for u in l.nxs_units]} for l in links},
        "sync": {"fps": float(fsync), "on_up": bool(up_sync)} if fsync is not None else None,
        "gain_db": gain_db, "exposure_us": topology.camera_exposure_us,
        "outputs": {name: [int(o.lanes), list(o.streams), bool(o.copy)]
                    for name, o in sorted((outs or {}).items())},
    }
    return json.dumps(doc, sort_keys=True, separators=(",", ":"))


# ── the composition ──────────────────────────────────────────────────

def _program_facts(program, direct: bool) -> Tuple[Dict[str, int], Dict[str, int]]:
    """The program's phases and parameters by name. A hub's program hands
    the walks its lock bits in `result`; the connector's has none to hand."""
    phases_payload = next((bytes(p) for t, p in program.trailer if int(t) == rg.HUB_PHASES), None)
    if phases_payload is None:
        raise InfeasibleConfig(f"the hub image {program.name} names no phases, and a port's walks "
                               f"name the phases of its program", alternatives=["nxs assets install"])
    phases = {name: value for value, name in rg.decode_phases(phases_payload).items()}
    params = {p.name: i for i, p in enumerate(program.params)}
    for needed in ("phase",) if direct else ("phase", "result"):
        if needed not in params:
            raise InfeasibleConfig(f"the hub image {program.name} declares no {needed} parameter",
                                   alternatives=["nxs assets install"])
    return phases, params


def _base(params: Mapping[str, int], values: Mapping[str, int]) -> Tuple[Tuple[int, int], ...]:
    """The base staging: each value the program declares a parameter for."""
    return tuple((params[name], int(value)) for name, value in values.items() if name in params)


def compose(hub, topology: Topology, links: Sequence[LinkSpec], modes: Mapping[str, str],
            rates: Optional[Mapping[str, float]] = None, *, fsync: Optional[float] = None,
            up_sync: bool = False,
            gain_db: Optional[float] = None, outputs=None,
            program: Optional[bytes] = None, heads: bool = True) -> Composition:
    """The graph image of `links` brought up on `topology` in `modes` at
    `rates` (frames a second by link). `fsync` is the rate of the frame
    sync the sync walk runs, `up_sync` whether the bring-up's policy runs
    it, `gain_db` the declared gain the gain walk sets on every camera
    head, `outputs` the hub's outputs (`csi.Output` by port name, port B
    off by default), `program` the hub's image (else the owner's store's).
    Without `heads` the image carries the walks that touch no head (behind
    a hub the construction, the training, the recovery, the status and the
    gates) and no personality is looked up.

    Raises:
        InfeasibleConfig: What the composition refuses: a shape the walks
            do not bring up (`_refuse_shape`), a personality this host does
            not hold or cannot stage, one that takes no action past its
            configure, a declared gain a head's law does not take, a
            serializer or an expander the program does not reach, a record
            past its budget, with the records' own reason.
    """
    from nxs.compiler import CompileError
    from nxs.image import compose_graph, deserialize
    from nxs.cam.select import _port_name

    _refuse_shape(hub, topology, links)
    if outputs is None and topology.outputs and getattr(hub.flows(), "Output", None) is not None:
        # The outputs the declaration states: a port B feeding a module.
        output = hub.flows().Output
        outputs = {name: output(lanes, streams, copy)
                   for name, lanes, streams, copy in topology.outputs}
    direct = bool(topology.is_direct)
    rates = dict(rates or {})
    present = {l.name for l in links}
    port = (topology if len(links) == len(topology.links)
            else dataclasses.replace(topology, links=tuple(links))).with_modes(dict(modes))
    image = program if program is not None else hub_program(hub)
    prog = deserialize(image)
    phases, params = _program_facts(prog, direct)
    # A row's index is its link's bit in the lock bits the program hands the walk: a port
    # that declares link B alone carries link A as an empty row.
    declared = {l.name for l in topology.links}
    names = sorted(declared | {"A"}) if "B" in declared and not direct else sorted(declared)
    by_name = {l.name: l for l in topology.links}
    frames = _frame_lengths(hub, port, [by_name[n] for n in sorted(present)], modes, rates)
    cameras = [n for n in names if n in present and by_name[n].has_camera]
    pod_cameras = [n for n in cameras if by_name[n].nxs_units]
    gain_walk = gain_db is not None and bool(cameras) and heads
    mdb = round(float(gain_db) * 1000) if gain_walk else None
    rows: List[rg.GraphLink] = []
    found: Dict[str, Head] = {}
    for name in names:
        spec = by_name.get(name)
        row = _row(hub, spec, name, name in present, direct)
        if spec is not None and name in present and spec.has_camera and not heads:
            facts_of = hi.link_facts(hub.descriptor(spec.sensor_compatible), modes[name])
            row = dataclasses.replace(row, lanes=int(facts_of["lanes"]),
                                      data_type=hi.DATA_TYPES[str(facts_of["data_type"])],
                                      host_csi=bool(facts_of["host_csi"]))
        elif spec is not None and name in present and spec.has_camera:
            sen = hub.descriptor(spec.sensor_compatible)
            found[name], periods = head(hub, port, by_name[name], modes[name], frames.get(name))
            row = _staged(row, found[name], hi.link_facts(sen, modes[name]), periods,
                          frames.get(name), mdb, topology.camera_exposure_us)
        elif spec is not None and name in present:
            row = dataclasses.replace(row, lanes=int(hi.POD_ONLY_FACTS["lanes"]))
        rows.append(row)
    facts = _facts(hub, port, rows, found, phases, params, present, cameras, pod_cameras,
                   frames, outputs, fsync if heads else None, mdb, modes, rates, not heads,
                   gain_db)
    walks = rg.GraphWalks(
        phase_param=params["phase"], lock_param=params.get("result"),
        train_rounds=TRAIN_ROUNDS, vlock=() if direct else _vlock_regs(hub),
        walks=compose_walks.compose(facts),
        base=() if direct else _base(params, _staging(hub, prog, port, rows, cameras, present,
                                                      outputs, fsync)))
    text = declaration(topology, links, modes, rates, fsync, up_sync, gain_db, outputs)
    source = rg.GraphSource(
        program=prog.name, program_crc=rg.program_crc(prog.seal_nonce, prog.bytecode),
        digest=rg.declaration_digest(text.encode()),
        des_addr=0 if direct else int(topology.des_addr), homogeneous=facts.homogeneous,
        sync=bool(up_sync and facts.sync_walk), gain=rg.GraphWalk.GAIN in walks.walks)
    name = _port_name(topology)
    try:
        records = [(rg.GRAPH_SOURCE, rg.encode_source(source, name)),
                   (rg.GRAPH_LINKS, rg.encode_links(rows, name)),
                   (rg.GRAPH_WALKS, rg.encode_walks(walks, name))]
        out_rows = [] if direct else _outputs(hub, topology, cameras, outputs)
        if out_rows:
            records.insert(2, (rg.GRAPH_OUTPUTS, rg.encode_outputs(out_rows, name)))
        graph = compose_graph(image, records, name)
    except (rg.RecordError, CompileError) as exc:
        raise InfeasibleConfig(str(exc)) from exc
    return Composition(image=graph, name=name, links=tuple(rows), walks=walks,
                       phases={v: n for n, v in phases.items()},
                       images={(h.name, h.crc): h.image for h in found.values()},
                       declaration=text, heads=found)


def _converts(found: Head, pod: bool) -> bool:
    """Whether a head takes the trigger conversion: a pod's on its camera
    door's trigger row, any head by its personality's trigger action with
    the fast trigger staged."""
    if pod and int(ParamId.TRIGGER) in found.rows:
        return found.trigger_fast is not None
    return (found.params[GraphParam.ACTION] is not None
            and found.params[GraphParam.TRIGGER] is not None and found.trigger_fast is not None)


def _vlock_regs(hub) -> Tuple[int, ...]:
    """The hub's video-lock registers, its `vlock_<pipe>` status probes':
    the poll whose failure recovers the links."""
    des = next(c for c in hub.chips if hub.descriptor(c).role == "DES")
    d = hub.descriptor(des)
    probes = [p for p in d.raw("status") or [] if str(p.get("name", "")).startswith("vlock_")]
    return tuple(int(d.reg(str(p["reg"]))) for p in sorted(probes, key=lambda p: str(p["name"])))[:2]


def _hub_values(prog, port: Topology, outs, fsync) -> Dict[str, int]:
    """The hub's rows this bring-up runs, served values by row name: the
    program's rows at their defaults, the hub node's declared values over
    them (`Topology.hub_values`), and what the bring-up runs over those: the
    port's lanes, port B's output and the frame sync at its rate."""
    from nxs.description import from_compiled

    rows = {row.name: int(row.default) for row in from_compiled(prog).rows}
    rows.update(dict(port.hub_values))
    rows["csi_a_lanes"] = int(port.csi_lanes)
    b = (outs or {}).get("B")
    if b is not None and b.lanes:
        rows.update(csi_b_lanes=int(b.lanes), csi_b_copy=int(bool(b.copy)))
    rows["sync_source"] = hi.SYNC_FSYNC if fsync is not None else hi.SYNC_FREE
    if fsync is not None:
        rows["sync_fps"] = int(round(float(fsync) * 1000))
    return rows


def _staging(hub, prog, port: Topology, rows: Sequence[rg.GraphLink], cameras: Sequence[str],
             present, outs, fsync) -> Dict[str, int]:
    """The program's run parameters every HUB step stages: the shape, the
    lane bits, the port's lanes, the heads' supplies, the CSI-block bits, a
    solo's data type, port B's lanes, streams and copy, the address maps'
    entry count and the generator's period."""
    flows = hub.flows()
    declared = _hub_values(prog, port, outs, fsync)
    by_name = {row.name: row for row in rows}
    live = [row for row in rows if row.name in present]
    if len(cameras) == 2:
        shape = hi.SHAPE_PAIR
    else:
        shape = hi.SHAPE_SOLO_B if live and live[0].name == "B" else hi.SHAPE_SOLO_A
    values = {
        "shape": shape,
        "lanes": sum(hi.LINK_BIT[r.name] for r in live if r.camera and r.lanes == 2),
        "csi_lanes": declared["csi_a_lanes"],
        # A program that serves no supply row stages no supplies: both heads stay on.
        "heads": declared.get("link_a_power", 1) | declared.get("link_b_power", 1) << 1,
        "ser_csi": sum(hi.LINK_BIT[r.name] for r in live if r.camera and r.host_csi),
        "dt": by_name[cameras[0]].data_type if len(cameras) == 1 else 0,
        "entries": max((len(r.entries) for r in live), default=0),
    }
    b = (outs or {}).get("B")
    if b is not None and b.lanes:
        values.update(csi_b=declared["csi_b_lanes"], streams=_stream_mask(flows, outs, cameras),
                      copy=declared["csi_b_copy"])
    if declared["sync_source"] == hi.SYNC_FSYNC:
        des = next(c for c in hub.chips if hub.descriptor(c).role == "DES")
        law = getattr(hub.chip_module(des), "fsync_period", None)
        if law is None:
            raise InfeasibleConfig(f"hub {hub.name} states no generator period for a rate",
                                   alternatives=[f"{port.hub_param_path('sync_source')}: free"])
        values["fsync_period"] = int(law(declared["sync_fps"] / 1000.0))
    return values


def _gain_units(hub, port: Topology, heads: Mapping[str, Head], cameras: Sequence[str],
                doors: Sequence[str], gain_db: float) -> Dict[str, int]:
    """The declared gain in each camera head's capture units, held to the
    law its personality states; a bare head takes it under its apply, so
    its personality stages the `gain` parameter there.

    Raises:
        InfeasibleConfig: A gain a head's law does not take, or a bare head
            whose apply takes no gain, under the declaration's key.
    """
    from nxs.cam.select import _port_name

    by_link = {l.name: l for l in port.links}
    units: Dict[str, int] = {}
    for name in cameras:
        try:
            if name not in doors and heads[name].params[GraphParam.GAIN] is None:
                raise InfeasibleConfig(f"link {name}'s personality takes no gain in its apply, "
                                       f"so a declared gain does not reach the head",
                                       alternatives=["drop the key"])
            compatible = hub.descriptor(by_link[name].sensor_compatible).compatible
            units[name] = gain_units(heads[name], compatible, gain_db,
                                     place=port.hub_param_path("gain"))
        except InfeasibleConfig as exc:
            raise InfeasibleConfig(f"{port.hub_param_path('gain')}: {exc.reason}",
                                   alternatives=exc.alternatives) from exc
    return units


def _facts(hub, port: Topology, rows: Sequence[rg.GraphLink], heads: Mapping[str, Head], phases,
           params, present, cameras: Sequence[str], pod_cameras: Sequence[str],
           frames: Mapping[str, Optional[int]], outs, fsync, mdb: Optional[int],
           modes: Mapping[str, str], rates: Mapping[str, float], hub_only: bool,
           gain_db: Optional[float]) -> compose_walks.WalkFacts:
    """The facts the walks are composed from. A declared gain is held to
    each head's law here, before the bus (`_gain_units`)."""
    from nxs.cam.hubs import is_smart_camera

    flows = hub.flows()
    index = {row.name: i for i, row in enumerate(rows)}
    entries = max((len(r.entries) for r in rows if r.name in present), default=0)
    amap = {}
    for row in rows:
        if row.name in present and row.entries:
            padded = list(row.entries) + [(0, 0)] * (entries - len(row.entries))
            amap[row.name] = hi.pack_address_map(padded)
    synced: Tuple[str, ...] = ()
    plan = getattr(flows, "sync_plan", None)
    if plan is not None and cameras and not hub_only:
        taken = plan(hub, port)
        synced = tuple(n for n in cameras if taken.get(n, (False, ""))[0])
    # A head converts on its camera door's trigger row, else its personality's trigger
    # action: one that offers neither takes no pulse, which a declared sync refuses.
    stuck = [n for n in synced if not _converts(heads[n], n in pod_cameras)]
    if stuck and fsync is not None and len(cameras) == 2:
        raise InfeasibleConfig(f"link {stuck[0]}'s personality offers no trigger conversion, "
                               f"so its head takes no pulse",
                               alternatives=[f"{port.hub_param_path('sync_source')}: free"])
    synced = tuple(n for n in synced if n not in stuck)
    by_link = {l.name: l for l in port.links}
    smart = tuple(n for n in cameras if is_smart_camera(hub, by_link[n]))
    doors = tuple(n for n in cameras if n in pod_cameras or n in smart)
    units = (_gain_units(hub, port, heads, cameras, doors, float(gain_db))
             if mdb is not None else {})
    b = (outs or {}).get("B")
    return compose_walks.WalkFacts(
        phases=phases, params=params, rows=index,
        present=tuple(r.name for r in rows if r.name in present), cameras=tuple(cameras),
        pods=tuple(r.name for r in rows if r.name in present and r.pod), amap=amap,
        timed={name: frames.get(name) is not None for name in cameras},
        homogeneous=homogeneous(hub, [by_link[n] for n in cameras], modes),
        port_b=b is not None and bool(b.lanes),
        indices={name: head.params for name, head in heads.items()},
        trigger_free={name: head.trigger_free for name, head in heads.items()},
        trigger_fast={name: head.trigger_fast for name, head in heads.items()},
        doors={name: heads[name].rows for name in doors if name in heads}, synced=synced,
        sync_walk=fsync is not None and len(cameras) == 2, gain_mdb=mdb, gain_units=units,
        hub_only=hub_only, direct=bool(port.is_direct), smart=smart,
        mode_values={name: head_of.mode for name, head_of in heads.items()},
        rates_mfps={name: round(float(rates[name]) * 1000) if rates.get(name) else None
                    for name in cameras})
