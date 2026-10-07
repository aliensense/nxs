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
from nxs.cam.engine import CamI2c, ConfigHandler, ExpectFailedError, Manager
from nxs.cam.plan import RawConfig
from nxs.cam import unit_source
from nxs.cam.identity import _verify_hub_identity
from nxs.cam.graph import run_timing
from nxs.cam.select import _link_descriptor, _hub_for, _per_link, _port_name



#: How long a unit run may take end to end: the longest personality
#: program sleeps under two seconds and its alive probe 1.5 s; the rest
#: is register retries on a slow pod bus.
UNIT_RUN_TIMEOUT_S = 30.0

#: Link-training rounds (the dip-retry handshake) before a bring-up
#: proceeds without a pre-program lock, and the rounds a recovery runs.
TRAIN_ROUNDS = 6

def _execute(raw: Dict[str, Any], bus: str, name: str,
             quiet: bool = False, guard=None, faults: Optional[List[str]] = None) -> bool:
    """Run a composed program on the bus; ``quiet`` for probes whose failing
    gate is an answer. ``guard`` is (hub, topology): the hub's silicon identity
    is verified through a fresh handle before the first byte. ``faults`` takes
    what a failed run stopped on, in words for the caller's refusal: the wait
    that ran out by its name, else the bus fault."""
    if guard is not None:
        hub, topology = guard
        i2c = CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        try:
            i2c.open()
            _verify_hub_identity(hub, topology, i2c)
        finally:
            i2c.close()
    manager = Manager(ConfigHandler(raw), path_label=f"<composed:{name}>")
    fault: Optional[Exception] = None
    try:
        ok = manager.run(bus=bus, retries=8, retry_delay_s=0.15)
    except (OSError, RuntimeError) as exc:
        # A dead device mid-program is an outcome, not a traceback.
        fault, ok = exc, False
    # The polled waits that replaced the program's fixed settles: how long
    # the links really took against the budget they had, on the bench or
    # when a wait ran to its deadline. The fault stands last, under them.
    summary = manager.settle_summary()
    missed = any(not settle["held"] for settle in manager.settles)
    if summary and not quiet and (missed or os.environ.get("NXS_CAM_VERBOSE")):
        term.info(f"{name}: {summary}")
    if fault is not None:
        if not quiet:
            term.err(f"{name}: {fault}")
        if faults is not None:
            faults.append(_stopped_on(fault))
    return ok


def _stopped_on(fault: Exception) -> str:
    """What a failed run stopped on: the polled wait that ran out, by its
    name and budget, also where a retry block gave up on it, else the fault
    as the engine words it."""
    wait = fault if isinstance(fault, ExpectFailedError) else fault.__cause__
    if isinstance(wait, ExpectFailedError) and wait.timeout_ms:
        return f"{wait.wait} did not hold within {wait.timeout_ms / 1000:g} s"
    return str(fault)


def _fail(faults: Optional[List[str]], text: str) -> bool:
    """A step's failure in words: printed, and handed to the caller that
    words the run's refusal. Returns False, the step's result."""
    term.err(text)
    if faults is not None:
        faults.append(text)
    return False


def _execute_split(cfg: RawConfig, topology: Topology, name: str,
                   guard=None, units: Optional[Dict[str, Any]] = None,
                   faults: Optional[List[str]] = None) -> bool:
    """Run a composed plan: every host segment through `_execute`, and at
    each unit-program marker the link's unit runs its cam personality
    with the marker's mode and trigger staged, or the host runs it when the
    link declares no unit.
    ``units`` maps a link name to an open client for its unit; without one
    a client is opened at the unit's alias on the port's bus. A marker whose
    personality cannot be resolved refuses before the first bus write.
    ``faults`` takes what a failed segment or unit program stopped on, as
    `_execute` fills it."""
    markers = cfg.unit_programs
    if not markers:
        return _execute(cfg.to_dict(), topology.i2c_bus, name, guard=guard, faults=faults)
    hub = guard[0] if guard is not None else _hub_for(topology)
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
                            f"{name}:{part}", guard=guard, faults=faults):
                return False
            segment = []
        link, mode, trigger = markers[seq]
        if not _run_unit_program(hub, together, link, mode, trigger, name,
                                 guard, units, faults=faults):
            return False
    if segment:
        part += 1
        return _execute(cfg.subset(segment), topology.i2c_bus,
                        f"{name}:{part}", guard=guard, faults=faults)
    return True


def _plan_links(topology: Topology, cfg: RawConfig) -> Topology:
    """The port narrowed to the links whose unit programs the plan runs;
    the port itself when they are all in (or none is)."""
    names = {link for link, _mode, _trigger in cfg.unit_programs.values()}
    chosen = tuple(l for l in topology.links if l.name in names)
    if not chosen or len(chosen) == len(topology.links):
        return topology
    return dataclasses.replace(topology, links=chosen)


def _run_host_program(hub, topology: Topology, spec: LinkSpec, sen, mode: str,
                      trigger: str, action: Optional[str], frame_length: Optional[int]) -> bool:
    """One marker on a link without a unit: the host runs the declared
    personality's image on the port's bus at the head's address, as the
    walk does behind a hub, and the verdict is printed."""
    from types import SimpleNamespace

    from nxs import _libnxs
    from nxs.cam import hubs
    from nxs.personality import records
    from nxs.personality.records import ACTION_PARAM, ACTIONS

    found = _declared_image(hub, spec, sen)
    if found is None:
        return False
    name, image, compiled, params = found
    held = SimpleNamespace(name=name, params=params)
    try:
        values = _staged_values(held, sen, mode, trigger, frame_length)
        if action is not None and action != "configure":
            names = [p.name for p in compiled.params]
            if ACTION_PARAM not in names:
                raise InfeasibleConfig(f"{sen.name}'s personality declares no {ACTION_PARAM} "
                                       f"param; it runs the whole program only")
            values[names.index(ACTION_PARAM)] = ACTIONS[action]
        term.info(f"link {spec.name}: the host runs {name}, {action or 'configure'}, "
                  f"mode {mode}, trigger {trigger}")
        bus = _libnxs.Bus.open(topology.i2c_bus)
        try:
            report = bus.run(image, int(hubs.sensor_address(hub, spec)), values)
        finally:
            bus.close()
    except InfeasibleConfig as exc:
        term.err(f"link {spec.name}: {exc}")
        return False
    except (OSError, RuntimeError, _libnxs.LibraryMissing) as exc:
        term.err(f"link {spec.name}: host run: {exc}")
        return False
    if report.rc != 0:
        term.err(f"link {spec.name}: host run {report.error} (pc {report.pc}, "
                 f"register {report.reg if report.reg is not None else 'none'})")
        return False
    term.info(f"link {spec.name}: host run halted, {report.reg_writes} register writes")
    return True


def _unit_personality(client, topology: Topology, spec: LinkSpec, sen):
    """The cam personality `spec`'s unit holds (read over the open
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
        term.refusal(f"link {link}'s unit holds no cam personality",
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
        term.refusal(f"link {link}'s unit holds cam personality {found.name!r} "
                     f"({compatible or 'unknown part'}), not {sen.name}",
                     f"nxs {port} {link} upload {sen.name}")
        return None
    return found


def _staged_values(found, sen, mode: str, trigger: str,
                   frame_length: Optional[int] = None) -> Dict[int, int]:
    """The `{param index: value}` a run of `mode` and `trigger` stages, from
    the unit's own records (`ParamMap`); an image whose records name no
    param index (compiled without its descriptor) takes the hub pair's
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


def _run_unit_program(hub, topology: Topology, link: str, mode: str,
                      trigger: str, name: str, guard, units,
                      action: Optional[str] = None,
                      frame_length: Optional[int] = None,
                      faults: Optional[List[str]] = None) -> bool:
    """One marker: the link's unit runs its personality and the verdict is
    printed, or the host runs it on a link without a unit. `action` names a
    pod action other than the whole program (`records.ACTIONS`);
    `frame_length` fixes the frame the timing writes; `faults` takes the
    line a failed step ends on."""
    from nxs.client import DeviceRefused, await_cam_run, cam_run_state_name
    from nxs.personality.records import ACTION_PARAM, ACTIONS

    spec = topology.link(link)
    sen = _link_descriptor(hub, topology, spec)
    if not spec.nxs_units:
        return _run_host_program(hub, topology, spec, sen, mode, trigger, action, frame_length)
    client = (units or {}).get(link)
    owned = client is None
    if owned:
        from nxs.transports import open_client
        unit = spec.nxs_units[0]
        try:
            client = open_client("i2c", bus=topology.i2c_bus, address=unit.alias_addr)
        except (OSError, RuntimeError, ValueError) as exc:
            return _fail(faults, f"link {link}: the unit at {hex(unit.alias_addr)} on "
                                 f"{topology.i2c_bus} does not open: {exc}")
    try:
        # The pod is brought to the declaration first: the declared
        # personality is uploaded when the pod holds another or none.
        from nxs.cam import pods
        try:
            converged = pods.converge(client, hub, topology, spec)
        except (pods.PodSilent, pods.PodRefused) as exc:
            return _fail(faults, str(exc))
        except (InfeasibleConfig, RuntimeError) as exc:
            return _fail(faults, f"link {link}: {exc}")
        if converged:
            term.info(converged)
        found = _unit_personality(client, topology, spec, sen)
        if found is None:
            return False
        values = _staged_values(found, sen, mode, trigger, frame_length)
        if action is not None and action != "configure":
            # The action's index comes from the image this host holds, the
            # one `converge` put on the pod (its records carry no index).
            names = pods.param_names(hub, spec)
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
        return _fail(faults, f"link {link}: {exc}")
    except DeviceRefused as exc:
        return _fail(faults, f"link {link}: {exc} (errno {exc.code})")
    except (OSError, RuntimeError, TimeoutError) as exc:
        return _fail(faults, f"link {link}: unit run: {exc}")
    finally:
        if owned and hasattr(client, "close"):
            client.close()


def park_pods(hub, topology: Topology) -> None:
    """After the host's park program: each camera link's pod runs its
    personality's park action, the walk's `pod <link>: park` turn, so the
    unit's last run is a park and its status LED stops beating for the head.
    The host's program already stood every sensor by, so a pod that does not
    park is one line and never fails the park."""
    for spec in topology.camera_links:
        why = _park_pod(hub, topology, spec)
        if why is not None:
            term.info(f"pod {spec.name}: no park run ({why})")


def _park_pod(hub, topology: Topology, spec: LinkSpec) -> Optional[str]:
    """Run `spec`'s pod park action at the pod's alias; the reason it did
    not run, else None. Only a pod whose last run brought its head up is
    parked: its probe program would fail on a head that never came up and
    leave the unit reporting a fault."""
    from nxs.cam import pods
    from nxs.client import CamRunState, DeviceRefused
    from nxs.personality.records import ACTION_PARAM, ACTIONS
    from nxs.transports import open_client

    if not spec.nxs_units:
        return "no pod"
    name = pods.declared_name(hub, spec)
    try:
        names = pods.param_names(hub, spec)
    except InfeasibleConfig as exc:
        return exc.reason
    if ACTION_PARAM not in names:
        return f"{name} has no park action"
    try:
        client = open_client("i2c", bus=topology.i2c_bus, address=spec.nxs_units[0].alias_addr)
    except (OSError, RuntimeError, ValueError):
        return "no answer"
    try:
        try:
            state, _error = client.read_cam_state()
            found = unit_source.read_unit_personality(client)
        except unit_source.UnreadableSlot as exc:
            return str(exc)
        except (OSError, RuntimeError, ValueError):
            return "no answer"
        if state != CamRunState.DONE:
            return unit_source.last_run_text(state)
        held = pods.held_instead(found, hub, spec)
        if held is not None:
            return f"holds {held}"
        term.info(f"pod {spec.name}: park ({name}, slot {found.slot})")
        try:
            client.cam_stage_params(found.slot, {names.index(ACTION_PARAM): ACTIONS["park"]})
            client.cam_run(found.slot, timeout_s=UNIT_RUN_TIMEOUT_S)
        except DeviceRefused as exc:
            term.warn(f"pod {spec.name}: park: {exc} (errno {exc.code})")
        except (OSError, RuntimeError, TimeoutError) as exc:
            term.warn(f"pod {spec.name}: park: {exc}")
        return None
    finally:
        close = getattr(client, "close", None)
        if close is not None:
            close()


def _rates_arg(args: argparse.Namespace, links: List[LinkSpec]) -> Optional[Dict[str, float]]:
    """`--fps` as the builders take it: a rate per selected link, or None."""
    rates = _per_link(getattr(args, "fps", None), links, "fps")
    return {name: float(v) for name, v in rates.items()} or None


def _compose_from(hub, topology: Topology, links: List[LinkSpec],
                  args: argparse.Namespace, modes: Optional[Dict[str, str]] = None):
    flows = hub.flows()
    mode = modes if modes else _mode_arg(args)
    extra: Dict[str, Any] = {}
    rates = _rates_arg(args, links)
    if rates is not None:
        # A hub authored before the rate law takes no fps; the rate then
        # is not a knob it offers.
        if "fps" not in _accepted(flows.build_solo if len(links) == 1 else flows.build_dual):
            raise SystemExit(f"hub {hub.name!r} offers no --fps: its flows take "
                             f"no rate")
        extra["fps"] = rates
    if len(links) == 1:
        return flows.build_solo(hub, topology, links[0].name, mode=mode, **extra)
    return flows.build_dual(hub, topology, mode=mode, **extra)


def _port_links(topology: Topology, links: List[LinkSpec]) -> Topology:
    """The port narrowed to the selected links: the topology the port's
    laws judge (a solo is one link's port, whatever the port carries),
    the same one the builders compose against."""
    import dataclasses

    if len(links) == len(topology.links):
        return topology
    return dataclasses.replace(topology, links=tuple(links))


def _resolve_modes(flows, hub, links, modes, topology):
    """flows.resolve_modes with the selected links' port when the hub
    accepts a topology."""
    import inspect

    resolve = getattr(flows, "resolve_modes", None)
    if resolve is None:
        return dict(modes)
    if "topology" in inspect.signature(resolve).parameters:
        return resolve(hub, links, modes or None,
                       topology=_port_links(topology, links))
    return resolve(hub, links, modes or None)


def _mode_arg(args: argparse.Namespace):
    """`--mode` as the builders take it: one token, or None."""
    mode = getattr(args, "mode", None)
    if isinstance(mode, list):
        return mode[0] if len(mode) == 1 else (mode or None)
    return mode

def _accepted(fn) -> set:
    """The keyword arguments a hub function takes (hubs differ)."""
    import inspect
    try:
        return set(inspect.signature(fn).parameters)
    except (TypeError, ValueError):
        return set()


# ── the executor: the port's graph on the hub images ──────────────────

def hub_chips(hub) -> Optional[Tuple[str, str]]:
    """The hub's deserializer and its serializer, the two hub images the
    walk runs; None when the hub does not carry exactly one of each."""
    des = [c for c in hub.chips if hub.descriptor(c).role == "DES"]
    ser = [c for c in hub.chips if hub.descriptor(c).role == "SER"]
    if len(des) != 1 or len(ser) != 1:
        return None
    return des[0], ser[0]


def graph_images(hub) -> Tuple[bytes, bytes]:
    """The hub's hub images as this host holds them, the hub's then the
    serializer's: the store's sealed images, else the hub's own program
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
    chips = hub_chips(hub)
    if chips is None:
        raise InfeasibleConfig(f"hub {hub.name} ships no hub image: it names no "
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
        # A source tree compiles the image from the hub's class; the wheel's
        # hub ships no sensors to compose from, so its images are the store's.
        path = hub.chip_source(chip) if hub.sensors() else None
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


def _declared_image(hub, spec: LinkSpec, sen):
    """The declared personality as this host holds it: its name, image, the
    image deserialized and its parameter map, from the store else compiled
    from the hub's pair. None after printing why not: the host holds none,
    the store's image is cut short, it is not a cam personality's, a
    trailer record is malformed, or the trailer describes another sensor
    than the declared `sen`."""
    from nxs import personality_cli
    from nxs.cam import pods
    from nxs.image import IMAGE_KIND_NAMES, ImageKind, deserialize
    from nxs.personality import records

    name = pods.declared_name(hub, spec)
    hint = _install_hint(sen)
    try:
        source = personality_cli.resolve(name)
    except personality_cli.ResolveError as exc:
        term.refusal(f"link {spec.name}: no personality {name} on this host ({exc})", hint)
        return None
    if source.kind == personality_cli.KIND_IMAGE:
        image = source.image
        try:
            compiled = deserialize(image)
        except ValueError as exc:
            term.err(f"link {spec.name}: {name}: {exc}")
            return None
    else:
        compiled, image = personality_cli.compile_source(source, {})
    if compiled.kind != ImageKind.CAMERA:
        term.refusal(f"link {spec.name}: {name} is a {IMAGE_KIND_NAMES[compiled.kind]} image, "
                     f"not a cam personality", hint)
        return None
    # The store serves the image by its file name alone: the trailer's
    # identity says which sensor's program it is, and its run records are
    # read here, before anything opens the bus.
    try:
        described = str(records.decode_trailer(compiled.trailer)["meta"]["compatible"])
        params = records.param_map(compiled.trailer)
    except records.RecordError as exc:
        term.err(f"link {spec.name}: {name}: {exc}")
        return None
    if described != sen.compatible:
        term.refusal(f"link {spec.name}: {name} describes {described}, not {sen.compatible}", hint)
        return None
    return name, image, compiled, params


def _install_hint(sen) -> str:
    """The command that puts the declared personality on this host: the
    assets for a cam personality the product ships, else the install of
    the user's own directory."""
    from nxs.cam import cam_personalities

    if cam_personalities.registry().is_shipped(sen.name):
        return "nxs assets install"
    return f"nxs personality install ./{sen.name}"

def _link_sensor(hub, port: Topology, spec: LinkSpec, mode: str, frame_length: Optional[int]):
    """The staging of a camera link from the host's image of its declared
    personality: the pod runs the same build (the walk brings it there),
    a link without a pod runs the image on the host. None after printing
    why not."""
    from nxs import _libnxs
    from nxs.personality import records
    from nxs.personality.records import ACTION_PARAM

    sen = _link_descriptor(hub, port, spec)
    found = _declared_image(hub, spec, sen)
    if found is None:
        return None
    name, image, compiled, params = found
    names = [p.name for p in compiled.params]
    action = names.index(ACTION_PARAM) if ACTION_PARAM in names else None
    pod = None
    if spec.nxs_units:
        unit = spec.nxs_units[0]
        pod = _libnxs.Pod(alias=int(unit.alias_addr), target=int(unit.target_addr),
                          personality=name)
    try:
        sensor = _sensor_of(params, name, sen, mode, action, frame_length, pod, image)
    except InfeasibleConfig as exc:
        term.err(f"link {spec.name}: {exc}")
        return None
    who = "the pod runs" if pod is not None else "the host runs"
    term.info(f"link {spec.name}: {who} {name}, mode {mode}, trigger {FREERUN}")
    return sensor


class WalkStopped(RuntimeError):
    """The port's walk stopped once it was under way: the line names the
    port and the step, and what the walk started is the caller's to park."""


def run_graph(hub, topology: Topology, links: List[LinkSpec], modes: Dict[str, str],
              rates: Dict[str, float], images: Tuple[bytes, bytes], name: str) -> bool:
    """Bring `links` up on the executor: the pods brought to the
    declaration and their staging gathered, then the walk in libnxs (the
    hub images' phases on the port's bus, the pods' actions between them).
    The frame each pod's timing writes is the rate's, by the hub's law.
    False after a refusal printed before the walk; WalkStopped when the
    walk stopped under way."""
    from nxs import _libnxs
    from nxs.cam import graph

    flows = hub.flows()
    # A pair's head runs the pair line beside the mode the partner runs.
    port = _port_links(topology, links).with_modes(modes)
    frames: Dict[str, Optional[int]] = {}
    sensor_module = getattr(flows, "sensor_module", None)
    law = getattr(flows, "frame_law", None)
    for spec in links:
        # A hub without a frame law (its tables set the frame) stages no frame.
        if not spec.has_camera or sensor_module is None or law is None:
            frames[spec.name] = None
            continue
        module = sensor_module(hub, spec, port)
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
        sensor = _link_sensor(hub, port, spec, modes[spec.name], frames[spec.name])
        if sensor is None:
            return False
        sensors[spec.name] = sensor
    try:
        spec = graph.port_spec(hub, topology, links, modes, images[0], images[1], sensors)
    except InfeasibleConfig as exc:
        term.err(f"{name}: {exc}")
        return False
    by_name = {l.name: l for l in links}

    def on_pod(link: str, unit) -> None:
        # The pod's first turn: the window reaches it now, so the tool
        # brings it to the declaration before the walk looks for the slot.
        from nxs.cam import pods

        converged = pods.converge(unit, hub, port, by_name[link])
        if converged:
            term.info(converged)

    from nxs.cam import pods

    try:
        bus = _libnxs.Bus.open(topology.i2c_bus)
    except OSError as exc:
        term.err(f"{name}: {topology.i2c_bus} does not open: {exc}")
        return False
    where = _port_name(topology)
    try:
        report = bus.port_up(spec, log=log, pod=on_pod)
    except InfeasibleConfig as exc:
        raise WalkStopped(f"{where}: {exc}") from exc
    except (pods.PodSilent, pods.PodRefused) as exc:
        # The line names the port and the link already.
        raise WalkStopped(str(exc)) from exc
    except (OSError, RuntimeError) as exc:
        raise WalkStopped(f"{where}: {exc}") from exc
    finally:
        bus.close()
    if not report.ok:
        raise WalkStopped(f"{where}: {report}")
    return True


def abort_pods(topology: Topology, links: List[LinkSpec]) -> None:
    """Abort each pod's camera run that is still live, as a walk that
    stopped on the wait for a run leaves it: a pod running holds its bus and
    takes no command but the abort. A pod with no live run is left as it is."""
    from nxs.client import DeviceRefused
    from nxs.transports import open_client

    for spec in links:
        if not spec.nxs_units:
            continue
        try:
            client = open_client("i2c", bus=topology.i2c_bus, address=spec.nxs_units[0].alias_addr)
        except (OSError, RuntimeError, ValueError):
            continue
        try:
            client.cam_abort()
            term.info(f"pod {spec.name}: its live run aborted")
        except (DeviceRefused, OSError, RuntimeError, TimeoutError):
            pass        # no run is live, or the pod does not answer: nothing to abort
        finally:
            close = getattr(client, "close", None)
            if close is not None:
                close()
