"""Running a composed plan on the bus: the host segments through the engine, the unit programs on the pods."""

from __future__ import annotations

import argparse
import dataclasses
import os
import time
from typing import Any, Dict, List, Optional, Tuple


from nxs import term

from nxs.cam.descriptors import FREERUN
from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs.cam.engine import CamI2c, ConfigHandler, Manager
from nxs.cam.plan import RawConfig
from nxs.cam import unit_source
from nxs.cam.identity import _verify_hub_identity
from nxs.cam.graph import run_timing
from nxs.cam.select import _link_descriptor, _pack_for, _per_link, _port_name



#: How long a unit run may take end to end: the longest personality
#: program sleeps under two seconds and its alive probe 1.5 s; the rest
#: is register retries on a slow pod bus.
UNIT_RUN_TIMEOUT_S = 30.0

#: Link-training rounds (the dip-retry handshake) before a bring-up
#: proceeds without a pre-program lock, and the rounds a recovery runs.
TRAIN_ROUNDS = 6

def _execute(raw: Dict[str, Any], bus: str, name: str,
             quiet: bool = False, guard=None) -> bool:
    """Run a composed program on the bus; ``quiet`` for probes whose failing
    gate is an answer. ``guard`` is (pack, topology): the hub's silicon identity
    is verified through a fresh handle before the first byte."""
    if guard is not None:
        pack, topology = guard
        i2c = CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        try:
            i2c.open()
            _verify_hub_identity(pack, topology, i2c)
        finally:
            i2c.close()
    manager = Manager(ConfigHandler(raw), path_label=f"<composed:{name}>")
    try:
        ok = manager.run(bus=bus, retries=8, retry_delay_s=0.15)
    except (OSError, RuntimeError) as exc:
        # A dead device mid-program is an outcome, not a traceback.
        if not quiet:
            term.err(f"{name}: {exc}")
        ok = False
    # The polled waits that replaced the program's fixed settles: how
    # long the links really took against the budget they had.
    summary = manager.settle_summary()
    if summary and not quiet:
        term.info(f"{name}: {summary}")
    return ok


def _execute_split(cfg: RawConfig, topology: Topology, name: str,
                   guard=None, units: Optional[Dict[str, Any]] = None) -> bool:
    """Run a composed plan: every host segment through `_execute`, and at
    each unit-program marker the link's unit runs its camera personality
    with the marker's mode and trigger staged.
    ``units`` maps a link name to an open client for its unit; without one
    a client is opened at the unit's alias on the port's bus. A marker whose
    link declares no unit, or whose personality cannot be resolved, refuses
    before the first bus write."""
    markers = cfg.unit_programs
    if not markers:
        return _execute(cfg.to_dict(), topology.i2c_bus, name, guard=guard)
    pack = guard[0] if guard is not None else _pack_for(topology)
    refusal = unit_program_refusal(cfg, pack, topology)
    if refusal:
        term.err(refusal)
        return False
    # The port the plan brings up is the links its markers name: a solo
    # on a two-link port stages the solo line, not the pair's.
    together = _plan_links(topology, cfg)
    segment: List[str] = []
    part = 0
    for seq in cfg.sequence_names():
        if seq not in markers:
            segment.append(seq)
            continue
        if segment:
            part += 1
            if not _execute(cfg.subset(segment), topology.i2c_bus,
                            f"{name}:{part}", guard=guard):
                return False
            segment = []
        link, mode, trigger = markers[seq]
        if not _run_unit_program(pack, together, link, mode, trigger, name,
                                 guard, units):
            return False
    if segment:
        part += 1
        return _execute(cfg.subset(segment), topology.i2c_bus,
                        f"{name}:{part}", guard=guard)
    return True


def _plan_links(topology: Topology, cfg: RawConfig) -> Topology:
    """The port narrowed to the links whose unit programs the plan runs;
    the port itself when they are all in (or none is)."""
    names = {link for link, _mode, _trigger in cfg.unit_programs.values()}
    chosen = tuple(l for l in topology.links if l.name in names)
    if not chosen or len(chosen) == len(topology.links):
        return topology
    return dataclasses.replace(topology, links=chosen)


def unit_program_refusal(cfg: RawConfig, pack, topology: Topology,
                         links: Optional[List[LinkSpec]] = None) -> Optional[str]:
    """Why a plan's unit-program markers cannot run, or None; with `links`,
    why any of those links cannot run a program at all (a bring-up runs
    every selected link's pod). A caller that touches the bus before the
    plan (the pristine-serializer construction) calls this first: a link
    that cannot run its program refuses with the link as it was found,
    never with a serializer half-programmed."""
    markers = cfg.unit_programs
    names = ([l.name for l in links] if links is not None
             else [markers[seq][0] for seq in cfg.sequence_names() if seq in markers])
    for name in names:
        refusal = _unit_program_refusal(pack, topology, name)
        if refusal:
            return refusal
    return None


def _unit_program_refusal(pack, topology: Topology, link: str) -> Optional[str]:
    """Why a marker on `link` cannot run at all, or None: no unit on the
    link."""
    spec = topology.link(link)
    if spec.nxs_units:
        return None
    port = _port_name(topology)
    # Behind a hub the unit answers at its link's alias (A one up from
    # the strapped 0x30, B two up), on the port's own bus at its own.
    index = next((i for i, l in enumerate(topology.links) if l.name == link), 0)
    unit = ("{name: …}" if topology.is_direct
            else f"{{name: …, alias: 0x{0x30 + index + 1:02X}}}")
    return (f"{port}/{link}: link {link} declares no pod\n"
            f"  - ports.{port}.links.{link}.unit: {unit} in suite.yaml, "
            f"then nxs switch")


def _unit_personality(client, topology: Topology, spec: LinkSpec, sen):
    """The camera personality `spec`'s unit holds (read over the open
    client, the state-directory cache brought up to date), or None after
    printing the refusal: the unit holds none, or another sensor's."""
    link = spec.name
    port = _port_name(topology)
    cached = unit_source.cached_record(topology, spec) or {}
    personality = cached.get("personality") or {}
    try:
        found = unit_source.read_unit_personality(client, known_crc=personality.get("crc"))
    except unit_source.UnreadableSlot as e:
        unit_source.forget(topology, spec)
        term.refusal(f"link {link}'s unit: {e}", f"nxs {port} {link} store rm {e.slot}")
        return None
    if found is None:
        unit_source.forget(topology, spec)
        term.refusal(f"link {link}'s unit holds no camera personality",
                     f"nxs {port} {link} upload {sen.name}")
        return None
    if found.descriptor is not None:
        unit_source.cache_descriptor(topology, spec, found.descriptor,
                                     slot=found.slot, crc=found.crc,
                                     params=found.params, name=found.name)
        compatible = found.descriptor.compatible
    else:
        compatible = str(personality.get("compatible") or "")
    if compatible != sen.compatible and found.name.lower() != sen.name.lower():
        term.refusal(f"link {link}'s unit holds camera personality {found.name!r} "
                     f"({compatible or 'unknown part'}), not {sen.name}",
                     f"nxs {port} {link} upload {sen.name}")
        return None
    return found


def _staged_values(found, sen, mode: str, trigger: str,
                   frame_length: Optional[int] = None) -> Dict[int, int]:
    """The `{param index: value}` a run of `mode` and `trigger` stages, from
    the unit's own records (`ParamMap`); an image whose records name no
    param index (compiled without its descriptor) takes the pack pair's
    parameter table and the descriptor's value order instead. A
    personality that takes the line and frame periods gets the mode's
    datasheet line and its recommended frame (`frame_length` lines when
    the caller fixes the frame), so the run starts the sensor in a valid
    frame."""
    from nxs.personality.records import FRAME_PERIOD_PARAM, LINE_TIME_PARAM

    params = found.params
    mode_index, modes = params.mode_index, dict(params.modes)
    trigger_index, triggers = params.trigger_index, dict(params.triggers)
    run_indices = {name: p.index for name, p in params.run_params.items()}
    if mode_index is None:
        raise InfeasibleConfig(
            f"the unit's personality {found.name} carries no parameter records",
            alternatives=[f"nxs <port> <link> upload {sen.name} "
                          f"(the image compiled from its yaml and py pair)"])
    if mode not in modes:
        raise InfeasibleConfig(
            f"the unit's personality offers no mode {mode!r}",
            alternatives=sorted(modes))
    values = {int(mode_index): int(modes[mode])}
    if trigger_index is not None:
        if trigger not in triggers:
            raise InfeasibleConfig(
                f"the unit's personality offers no {trigger!r} conversion",
                alternatives=[f"trigger {t}" for t in triggers])
        values[int(trigger_index)] = int(triggers[trigger])
    elif trigger != FREERUN:
        raise InfeasibleConfig(
            f"{sen.name}'s personality declares no trigger param; it runs "
            f"free only")
    if LINE_TIME_PARAM in run_indices and FRAME_PERIOD_PARAM in run_indices:
        for param, value in run_timing(sen, mode, frame_length).items():
            values[run_indices[param]] = value
    return values


def _run_unit_program(pack, topology: Topology, link: str, mode: str,
                      trigger: str, name: str, guard, units,
                      action: Optional[str] = None,
                      frame_length: Optional[int] = None) -> bool:
    """One marker: the link's unit runs its personality and the verdict is
    printed. `action` names a pod action other than the whole program
    (`records.ACTIONS`); `frame_length` fixes the frame the timing writes."""
    from nxs.client import DeviceRefused, await_cam_run, cam_run_state_name
    from nxs.personality.records import ACTION_PARAM, ACTIONS

    spec = topology.link(link)
    sen = _link_descriptor(pack, topology, spec)
    client = (units or {}).get(link)
    owned = client is None
    if owned:
        from nxs.transports import open_client
        unit = spec.nxs_units[0]
        try:
            client = open_client("i2c", bus=topology.i2c_bus, address=unit.alias_addr)
        except (OSError, RuntimeError, ValueError) as exc:
            term.err(f"link {link}: the unit at {hex(unit.alias_addr)} on "
                     f"{topology.i2c_bus} does not open: {exc}")
            return False
    try:
        # The pod is brought to the declaration first: the declared
        # personality is uploaded when the pod holds another or none.
        from nxs.cam import pods
        try:
            converged = pods.converge(client, pack, topology, spec, log=term.info)
        except (InfeasibleConfig, RuntimeError) as exc:
            term.err(f"link {link}: {exc}")
            return False
        if converged:
            term.info(converged)
        found = _unit_personality(client, topology, spec, sen)
        if found is None:
            return False
        values = _staged_values(found, sen, mode, trigger, frame_length)
        if action is not None and action != "configure":
            # The action's index comes from the image this host holds, the
            # one `converge` put on the pod (its records carry no index).
            names = pods.param_names(pack, spec)
            if ACTION_PARAM not in names:
                raise InfeasibleConfig(f"{sen.name}'s personality declares no {ACTION_PARAM} "
                                       f"param; it runs the whole program only")
            values[names.index(ACTION_PARAM)] = ACTIONS[action]
        client.cam_stage_params(found.slot, values)
        client.cam_run(found.slot, timeout_s=UNIT_RUN_TIMEOUT_S)
        term.info(f"link {link}: unit runs {found.name} (slot {found.slot}) "
                  f"{action or 'configure'}, mode {mode}, trigger {trigger}")
        started = time.monotonic()
        state, _error = await_cam_run(client, UNIT_RUN_TIMEOUT_S)
        term.info(f"link {link}: unit run {cam_run_state_name(state)} in "
                  f"{time.monotonic() - started:.1f} s")
        return True
    except InfeasibleConfig as exc:
        term.err(f"link {link}: {exc}")
        return False
    except DeviceRefused as exc:
        term.err(f"link {link}: {exc} (errno {exc.code})")
        return False
    except (OSError, RuntimeError, TimeoutError) as exc:
        term.err(f"link {link}: unit run: {exc}")
        return False
    finally:
        if owned and hasattr(client, "close"):
            client.close()


def _rates_arg(args: argparse.Namespace, links: List[LinkSpec]) -> Optional[Dict[str, float]]:
    """`--fps` as the builders take it: a rate per selected link, or None."""
    rates = _per_link(getattr(args, "fps", None), links, "fps")
    return {name: float(v) for name, v in rates.items()} or None


def _compose_from(pack, topology: Topology, links: List[LinkSpec],
                  args: argparse.Namespace, modes: Optional[Dict[str, str]] = None):
    flows = pack.flows()
    mode = modes if modes else _mode_arg(args)
    extra: Dict[str, Any] = {}
    rates = _rates_arg(args, links)
    if rates is not None:
        # A pack authored before the rate law takes no fps; the rate then
        # is not a knob it offers.
        if "fps" not in _accepted(flows.build_solo if len(links) == 1 else flows.build_dual):
            raise SystemExit(f"pack {pack.name!r} offers no --fps: its flows take "
                             f"no rate")
        extra["fps"] = rates
    if len(links) == 1:
        return flows.build_solo(pack, topology, links[0].name, mode=mode, **extra)
    return flows.build_dual(pack, topology, mode=mode, **extra)


def _port_links(topology: Topology, links: List[LinkSpec]) -> Topology:
    """The port narrowed to the selected links: the topology the port's
    laws judge (a solo is one link's port, whatever the port carries),
    the same one the builders compose against."""
    import dataclasses

    if len(links) == len(topology.links):
        return topology
    return dataclasses.replace(topology, links=tuple(links))


def _resolve_modes(flows, pack, links, modes, topology):
    """flows.resolve_modes with the selected links' port when the pack
    accepts a topology."""
    import inspect

    resolve = getattr(flows, "resolve_modes", None)
    if resolve is None:
        return dict(modes)
    if "topology" in inspect.signature(resolve).parameters:
        return resolve(pack, links, modes or None,
                       topology=_port_links(topology, links))
    return resolve(pack, links, modes or None)


def _mode_arg(args: argparse.Namespace):
    """`--mode` as the builders take it: one token, or None."""
    mode = getattr(args, "mode", None)
    if isinstance(mode, list):
        return mode[0] if len(mode) == 1 else (mode or None)
    return mode

def _accepted(fn) -> set:
    """The keyword arguments a pack function takes (packs differ)."""
    import inspect
    try:
        return set(inspect.signature(fn).parameters)
    except (TypeError, ValueError):
        return set()


# ── the executor: the port's graph on the hub images ──────────────────

def hub_chips(pack) -> Optional[Tuple[str, str]]:
    """The pack's deserializer and its serializer, the two hub images the
    walk runs; None when the pack does not carry exactly one of each."""
    des = [c for c in pack.chips if pack.descriptor(c).role == "DES"]
    ser = [c for c in pack.chips if pack.descriptor(c).role == "SER"]
    if len(des) != 1 or len(ser) != 1:
        return None
    return des[0], ser[0]


def graph_images(pack) -> Tuple[bytes, bytes]:
    """The pack's hub images as this host holds them, the hub's then the
    serializer's: the store's sealed images, else the pack's own program
    classes compiled from the source tree. InfeasibleConfig names what
    this host lacks: the executor library, or an image."""
    from nxs import _libnxs, personality_cli
    from nxs.compiler import HubDevice
    from nxs.image import ImageKind, peek_format, serialize

    try:
        _libnxs.library_path()
    except _libnxs.LibraryMissing as exc:
        raise InfeasibleConfig(
            str(exc),
            alternatives=["pipx install --system-site-packages aliensense-nxs "
                          "(the platform wheel carries the executor)",
                          f"{_libnxs.ENV}=<path to libnxs.so>"]) from exc
    chips = hub_chips(pack)
    if chips is None:
        raise InfeasibleConfig(f"pack {pack.name} ships no hub image: it names no "
                               f"deserializer and serializer pair")
    found = {}
    for chip in chips:
        stored = os.path.join(personality_cli.store_dir(), f"{chip}{personality_cli.IMAGE_SUFFIX}")
        if os.path.isfile(stored):
            try:
                source = personality_cli._image_source(stored, chip, origin="store")
            except personality_cli.ResolveError as exc:
                raise InfeasibleConfig(f"{stored}: {exc}",
                                       alternatives=["nxs assets install"]) from exc
            if peek_format(source.image)[2] != ImageKind.HUB:
                raise InfeasibleConfig(f"{stored} is not a hub image",
                                       alternatives=["nxs assets install"])
            found[chip] = source.image
            continue
        # A source tree compiles the image from the pack's class; the wheel's
        # pack ships no sensors to compose from, so its images are the store's.
        path = pack.chip_source(chip) if pack.sensors() else None
        cls = personality_cli._class_in(str(path), chip) if path is not None else None
        if cls is None or not issubclass(cls, HubDevice):
            raise InfeasibleConfig(f"no hub image for {chip}: nothing at {stored}",
                                   alternatives=["nxs assets install"])
        found[chip] = serialize(cls().compile({}))
    return found[chips[0]], found[chips[1]]


def _sensor_of(params, name: str, sen, mode: str, action: Optional[int],
               frame_length: Optional[int], pod, image: Optional[bytes]):
    """The staging of one personality from its records (`ParamMap`): the
    mode's value, the free-running conversion's, and where the table takes
    them. A personality without records, or without the mode, refuses."""
    from nxs import _libnxs
    from nxs.cam import graph
    from nxs.personality.records import FRAME_PERIOD_PARAM, LINE_TIME_PARAM

    modes, triggers = dict(params.modes), dict(params.triggers)
    if params.mode_index is None:
        raise InfeasibleConfig(
            f"the personality {name} carries no parameter records",
            alternatives=[f"nxs <port> <link> upload {sen.name} "
                          f"(the image compiled from its yaml and py pair)"])
    if mode not in modes:
        raise InfeasibleConfig(f"the personality {name} offers no mode {mode!r}",
                               alternatives=sorted(modes))
    trigger = 0
    if params.trigger_index is not None:
        if FREERUN not in triggers:
            raise InfeasibleConfig(
                f"the personality {name} offers no {FREERUN!r} conversion",
                alternatives=[f"trigger {t}" for t in triggers])
        trigger = int(triggers[FREERUN])
    run_indices = {n: p.index for n, p in params.run_params.items()}
    return graph.Sensor(
        params=_libnxs.PersonalityParams(mode=int(params.mode_index),
                                         trigger=params.trigger_index, action=action,
                                         line_time=run_indices.get(LINE_TIME_PARAM),
                                         frame_period=run_indices.get(FRAME_PERIOD_PARAM)),
        mode=int(modes[mode]), trigger=trigger, frame_length=frame_length, pod=pod,
        image=image)


def _declared_image(pack, spec: LinkSpec):
    """The declared personality as this host holds it: its name and image,
    from the store else compiled from the pack's pair. None after printing
    the refusal."""
    from nxs import personality_cli
    from nxs.cam import pods

    name = pods.declared_name(pack, spec)
    try:
        source = personality_cli.resolve(name)
    except personality_cli.ResolveError as exc:
        term.refusal(f"link {spec.name}: no personality {name} on this host ({exc})",
                     "nxs assets install")
        return None
    if source.kind == personality_cli.KIND_IMAGE:
        return name, source.image
    _compiled, image = personality_cli.compile_source(source, {})
    return name, image


def _link_sensor(pack, port: Topology, spec: LinkSpec, mode: str, frame_length: Optional[int]):
    """The staging of a camera link from the host's image of its declared
    personality: the pod runs the same build (the walk brings it there),
    a link without a pod runs the image on the host. None after printing
    why not."""
    from nxs import _libnxs
    from nxs.image import deserialize
    from nxs.personality import records
    from nxs.personality.records import ACTION_PARAM

    sen = _link_descriptor(pack, port, spec)
    found = _declared_image(pack, spec)
    if found is None:
        return None
    name, image = found
    compiled = deserialize(image)
    names = [p.name for p in compiled.params]
    action = names.index(ACTION_PARAM) if ACTION_PARAM in names else None
    pod = None
    if spec.nxs_units:
        unit = spec.nxs_units[0]
        pod = _libnxs.Pod(alias=int(unit.alias_addr), target=int(unit.target_addr),
                          personality=name)
    try:
        sensor = _sensor_of(records.param_map(compiled.trailer), name, sen, mode, action,
                            frame_length, pod, image)
    except InfeasibleConfig as exc:
        term.err(f"link {spec.name}: {exc}")
        return None
    who = "the pod runs" if pod is not None else "the host runs"
    term.info(f"link {spec.name}: {who} {name}, mode {mode}, trigger {FREERUN}")
    return sensor


def run_graph(pack, topology: Topology, links: List[LinkSpec], modes: Dict[str, str],
              rates: Dict[str, float], images: Tuple[bytes, bytes], name: str) -> bool:
    """Bring `links` up on the executor: the pods brought to the
    declaration and their staging gathered, then the walk in libnxs (the
    hub images' phases on the port's bus, the pods' actions between them).
    The frame each pod's timing writes is the rate's, by the pack's law."""
    from nxs import _libnxs
    from nxs.cam import graph

    flows = pack.flows()
    port = _port_links(topology, links)
    frames: Dict[str, Optional[int]] = {}
    sensor_module = getattr(flows, "sensor_module", None)
    law = getattr(flows, "frame_law", None)
    for spec in links:
        # A pack without a frame law (its tables set the frame) stages no frame.
        if not spec.has_camera or sensor_module is None or law is None:
            frames[spec.name] = None
            continue
        module = sensor_module(pack, spec, port)
        if law(module) and spec.name in rates:
            frames[spec.name] = int(flows.FrameLaw.of(module, modes[spec.name])
                                    .vmax_for(rates[spec.name]))
        else:
            frames[spec.name] = None
    verbose = bool(os.environ.get("NXS_CAM_VERBOSE"))

    def log(line: str) -> None:
        # The pods' turns are the port's story; the hub's phases print on the bench.
        if verbose or line.startswith("pod "):
            term.info(line)

    sensors: Dict[str, graph.Sensor] = {}
    for spec in links:
        if not spec.has_camera:
            continue
        sensor = _link_sensor(pack, port, spec, modes[spec.name], frames[spec.name])
        if sensor is None:
            return False
        sensors[spec.name] = sensor
    try:
        spec = graph.port_spec(pack, topology, links, modes, images[0], images[1], sensors)
    except InfeasibleConfig as exc:
        term.err(f"{name}: {exc}")
        return False
    by_name = {l.name: l for l in links}

    def on_pod(link: str, unit) -> None:
        # The pod's first turn: the window reaches it now, so the tool
        # brings it to the declaration before the walk looks for the slot.
        from nxs.cam import pods

        converged = pods.converge(unit, pack, port, by_name[link], log=term.info)
        if converged:
            term.info(converged)

    try:
        bus = _libnxs.Bus.open(topology.i2c_bus)
    except OSError as exc:
        term.err(f"{name}: {topology.i2c_bus} does not open: {exc}")
        return False
    try:
        report = bus.port_up(spec, log=log, pod=on_pod)
    except InfeasibleConfig as exc:
        term.err(f"{name}: {exc}")
        return False
    except (OSError, RuntimeError) as exc:
        term.err(f"{name}: {exc}")
        return False
    finally:
        bus.close()
    if not report.ok:
        term.err(f"{name}: {report}")
        return False
    return True
