"""`set` and `get`: a knob changed under the laws or read back, the sync source included."""

from __future__ import annotations

import argparse
import sys
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple


from nxs import term
from nxs._generated_constants import NxsMap

from nxs.cam.contracts import InfeasibleConfig, Topology
from nxs.cam.engine.i2c import I2CHandlerError
from nxs.cam.plan import flatten
from nxs.cam import port_state
from nxs.cam import hubs, pods, unit_source
from nxs.cam import laws as cam_laws
from nxs.cam import run as cam_run
from nxs.cam import timing as cam_timing
from nxs.cam.identity import _require_nxs_hub
from nxs.cam.run import _accepted
from nxs.cam.select import (_named, _hub_for, _port_name, _refuse, _refuse_pod_only, _require_up,
                            print_stream, select_port_links)
from nxs.cam.verbs import verify
from nxs.cam.verbs.caps import _camera_knobs, _hub_fsync_rate
from nxs.cam.verbs.sync import (LOOP_EXPOSURE, _fsync_plan, _record_hints, _record_sync, pair_gain_fact,
                                plan_text, pulse_exposure_fact, pulse_sets_exposure, sync_text)


def cmd_set(args: argparse.Namespace) -> int:
    """`set NAME VALUE`: `sync` is the port's frame sync (`fsync` starts the
    hub's generator, `free_run` stops it); every other knob is the link's
    sensor's, composed under the hub's laws. Every refusal starts with the
    port's name, the laws' own included; the delivery check's name it
    already."""
    topology, _ = select_port_links(args)
    _require_nxs_hub(topology, "set")
    # The laws judge a knob at the line the delivery check found for the port.
    topology = port_state.with_found_lines(topology)
    hub = _hub_for(topology)
    try:
        return _set(args, topology, hub, hub.flows())
    except InfeasibleConfig as exc:
        raise InfeasibleConfig(_named(_port_name(topology), exc.reason), alternatives=exc.alternatives)


def _set(args: argparse.Namespace, topology: Topology, hub, flows) -> int:
    """`set` on the port: the sync, the mode, a link's camera knob, else the
    link's unit's parameter."""
    if args.knob == "sync":
        return _set_sync(args, topology, hub, flows)
    if args.knob == "mode":
        return _set_mode(args, topology)
    if args.knob == "exposure" and pulse_sets_exposure(topology, args.link or topology.links[0].name):
        # A synced link's exposure is its trigger pulse's; a link the trigger
        # leaves free-running keeps its own.
        port = _port_name(topology)
        fps = (port_state.port_sync(topology) or {}).get("fps")
        raise InfeasibleConfig(pulse_exposure_fact(fps),
                               alternatives=[f"nxs {port} set sync free_run"]
                               + _fastest(hub, topology, _running_modes(topology), fps))
    pair_gain = pair_gain_fact(topology) if args.knob in ("gain", "gain_db") else None
    if pair_gain:
        # A gain set by hand leaves the pair's one gain: the leader's loop or
        # the follower moves it back, or it breaks the declared lock.
        raise InfeasibleConfig(pair_gain, alternatives=[
            f"{topology.hub_param_path('gain')}: <dB> in suite.yaml, then nxs switch"])
    if args.link is None and len(topology.links) > 1:
        names = "|".join(l.name for l in topology.links)
        raise _refuse(
            f"set on {_port_name(topology)} is ambiguous (links: {names})",
            f"nxs {_port_name(topology)} {names.split('|')[0]} set {args.knob} "
            f"{args.value}")
    link_spec = (topology.link(args.link) if args.link
                 else topology.links[0])
    if not _camera_knob(hub, link_spec, topology, args.knob) and link_spec.nxs_units:
        return _set_unit_param(topology, link_spec, args)
    if not link_spec.has_camera:
        raise _refuse_pod_only(topology, link_spec, f"set {args.knob} {args.value}")
    mode = port_state.port_mode(topology, link_spec)
    if args.dry_run:
        # Inert by contract: the knob judged and named, no bus IO.
        print(_would(hub, topology, link_spec, args.knob, args.value, mode))
        return 0
    targets = [link_spec] if args.link else list(topology.links)
    _require_up(topology, targets, "set")
    ok = _set_knob(hub, topology, link_spec, args.knob, args.value, mode)
    if ok and args.knob == "fps":
        _hold_rate(args, topology, hub, flows, link_spec, mode)
    return 0 if ok else 1


def _door_commit(hub, topology: Topology, link, knob: str, value: str,
                 mode: Optional[str]) -> Optional[Tuple[Any, int]]:
    """The knob judged by the hub's laws as they plan it (`build_knob`): the
    row and the value its camera door commits, a pod's or a smart camera's;
    None for a bare head, whose apply stages the knob instead.

    Raises:
        InfeasibleConfig: The laws' refusal of the knob or its value.
        SystemExit: A knob the link's head does not have.
    """
    flows = hub.flows()
    extra: Dict[str, Any] = {"mode": mode} if "mode" in _accepted(flows.build_knob) else {}
    try:
        plan = flows.build_knob(hub, topology, knob, value, link=link.name, **extra)
    except KeyError as exc:
        raise SystemExit(f"nxs: {exc}")
    commits = [(row, staged) for _link, row, staged in plan.map_commits.values()]
    return commits[0] if commits else None


def _bare(hub, topology: Topology, link, knob: str, value: str, mode: Optional[str]):
    """A bare head's knob as its apply stages it (`compose_heads.bare_knob`):
    the port's composition, the head, the run parameter, its value in the
    parameter's unit and the value in the knob's words."""
    from nxs.cam import compose_heads, graph

    composition = graph.compose_port(hub, topology)
    head = composition.heads[link.name]
    row = next(r for r in composition.links if r.name == link.name)
    index, staged, text = compose_heads.bare_knob(hub, topology, link, head, knob, value, mode,
                                                  line_ns=row.line_time_ns)
    return composition, head, index, staged, text


def _would(hub, topology: Topology, link, knob: str, value: str, mode: Optional[str]) -> str:
    """The line a dry run of the knob prints: the commit on the link's camera
    door, else the bare head's apply on the host."""
    where = f"{_port_name(topology)}/{link.name}"
    commit = _door_commit(hub, topology, link, knob, value, mode)
    if commit is not None:
        row, staged = commit
        name = row if isinstance(row, str) else NxsMap.ParamId._NAMES[int(row)].lower()
        return f"{where}: would commit {name} {staged} on its camera door"
    _composition, head, _index, _staged, text = _bare(hub, topology, link, knob, value, mode)
    return f"{where}: would run {head.name}'s apply on the host with {knob} {text}"


def _set_knob(hub, topology: Topology, link, knob: str, value: str, mode: Optional[str]) -> bool:
    """One knob on the link's head, judged by the laws (`_door_commit`) and
    run by the library's set (`nxs_graph_set`) under the bus lock: a pod or
    a smart camera commits the row on its camera door, a bare head runs its
    apply on the host with the knob's run parameter staged. True when the
    knob took, else False after the line that says why."""
    from nxs.cam import graph, map_commit

    where = f"{_port_name(topology)}/{link.name}"
    commit = _door_commit(hub, topology, link, knob, value, mode)
    if commit is not None:
        row, staged = commit
        composition = graph.compose_port(hub, topology)
        with port_state.BusLock():
            failed = map_commit.door_set(topology, composition, link.name, row, staged)
        if failed is not None:
            term.err(failed)
        return failed is None
    composition, head, index, staged, text = _bare(hub, topology, link, knob, value, mode)
    with port_state.BusLock():
        failed = graph.set_head(topology, composition, link.name, index, staged)
    if failed is not None:
        term.err(f"{where}: {knob} {text}: {failed}")
        return False
    term.info(f"link {link.name}: {knob} {text} by {head.name}'s apply on the host")
    return True


def _set_mode(args: argparse.Namespace, topology: Topology) -> int:
    """`set mode <row>`: the chain `on --mode` walks for the link (the whole
    port with no link named): the booted table's row, the hub's pipes for
    the row's data type, the pod's run at the row, then the delivery
    check. A row the table does not boot is refused there."""
    from nxs.cam.verbs.on import cmd_up

    links = [args.link] if args.link else []
    return cmd_up(argparse.Namespace(topology=getattr(args, "topology", None), port=args.port,
                                     links=links, sensor=None, mode=[args.value], fps=None,
                                     dry_run=bool(args.dry_run), cam_cmd="on"))


def _camera_knob(hub, link, topology: Topology, knob: str) -> bool:
    """Whether a knob is the link's camera's: one of the hub's, or the
    rate and the exposure, which read back on every sensor."""
    return link.has_camera and (knob in _camera_knobs(hub, link, topology)
                                or knob in ("fps", "exposure"))


def _unit_verb(topology: Topology, link, verb, args) -> int:
    """A unit verb run on the link's unit at its alias, under the bus lock."""
    client = unit_source._unit_client(topology, link)
    try:
        with port_state.BusLock():
            return verb(client, args)
    finally:
        client.close()


def _set_unit_param(topology: Topology, link, args: argparse.Namespace) -> int:
    """A knob the camera does not have goes to the link's unit, as `get`
    reads it: its personality's parameter or a device parameter, written
    at the unit's alias, with no link isolated and the other link's video
    kept. A cam personality's run parameters are the port's bring-up's,
    not a knob. A declared unit's change reverts on `switch`, as the unit
    verb says."""
    from nxs.cam.verbs.status import _pod_names
    from nxs.cli import cmd_set as unit_set, suite_managed_note
    from nxs.personality.records_fields import (ACTION_PARAM, FRAME_LENGTH_PARAM, FRAME_PERIOD_PARAM,
                                                LINE_TIME_PARAM, TRIGGER_PARAM)

    port = _port_name(topology)
    if args.knob in (TRIGGER_PARAM, ACTION_PARAM, LINE_TIME_PARAM, FRAME_PERIOD_PARAM,
                     FRAME_LENGTH_PARAM):
        raise _refuse(f"{port}/{link.name}: {args.knob} is the cam personality's run parameter, "
                      f"which the port's bring-up sets", f"nxs {port} {link.name} on --mode <mode>")
    if args.dry_run:
        print(f"{port}/{link.name}: would set {args.knob} {args.value} on its unit")
        return 0
    name = _pod_names(topology).get(link.name)
    if name:
        print(suite_managed_note(name), file=sys.stderr)
    return _unit_verb(topology, link, unit_set, SimpleNamespace(param=args.knob, value=args.value))


def _record_rate(hub, flows, topology: Topology, link, fps: float) -> None:
    """Record the rate a link's sensor now runs and re-derive the caps the
    capture stack's sessions open with, each link's part in the exposure
    and gain the recorded sync names kept."""
    port_state.set_rates(topology, {link.name: fps})
    sync = port_state.port_sync(topology) or {}
    triggered = sync.get("source") == "fsync"
    _record_hints(hub, flows, topology, triggered,
                  sync.get("trigger_vmax") if triggered else None, ae=sync.get("ae"))


def _hold_rate(args: argparse.Namespace, topology: Topology, hub, flows, link,
               mode: Optional[str]) -> None:
    """`set fps` holds the link to the rate it asked. The rate and the caps
    are recorded before the count, since the capture stack's driver sets
    the frame from the caps' rate when a session starts; a rate the link
    does not deliver is refused with the previous rate written back the
    way the knob went and recorded."""
    port = _port_name(topology)
    previous = port_state.port_rate(topology, link)
    _record_rate(hub, flows, topology, link, float(args.value))
    at = f" {link.name}" if args.link else ""
    try:
        verify.check(hub, topology, [link], retry=f"nxs {port}{at} set fps {{fps}}")
    except InfeasibleConfig:
        if previous is not None and _set_knob(hub, topology, link, "fps", f"{previous:g}", mode):
            _record_rate(hub, flows, topology, link, previous)
            term.info(f"{port}/{link.name}: back at {previous:g} fps")
        raise


def _crop_lines(flows, topology: Topology) -> List[str]:
    """What the viewers crop under the trigger, from the recorded caps: the
    filler rows a triggered frame's capture window ends in (the hub's
    output-delay law, cited by the flows' `FAST_TRIGGER_SECTION`), one line
    for the port, per link where the links crop differently; nothing where
    no link crops."""
    hints = port_state.port_record(topology).get("viewers") or {}
    crops = {name: int(h.get("crop_bottom") or 0) for name, h in hints.items()}
    crops = {name: rows for name, rows in crops.items() if rows}
    if not crops:
        return []
    section = getattr(flows, "FAST_TRIGGER_SECTION", None)
    cited = f", datasheet {section}" if section else ""
    if len(set(crops.values())) == 1:
        rows = f"{next(iter(crops.values()))} lines"
    else:
        rows = ", ".join(f"{name} {n} lines" for name, n in sorted(crops.items()))
    return [f"viewers crop the trigger frame's filler rows ({rows}{cited})"]


def _set_sync(args: argparse.Namespace, topology: Topology, hub, flows) -> int:
    """`set sync fsync|free_run [--fps F]`: the port on the sync named
    (`apply_sync`). `--exposure` is refused with the fact that sets the
    exposure: the pulse at the asked rate, else the capture stack's loop."""
    action = str(args.value).lower()
    if action not in ("fsync", "free_run"):
        raise SystemExit(f"nxs: sync is fsync or free_run, not {args.value!r}")
    if args.link is not None:
        raise SystemExit(f"nxs: sync is {_port_name(topology)}'s, not one link's: "
                         f"nxs {_port_name(topology)} set sync {action}")
    if not args.dry_run:
        _require_up(topology, list(topology.links), f"set sync {action}")
    fps = (float(args.fps) if getattr(args, "fps", None) is not None
           else topology.synced_fps)
    port = _port_name(topology)
    if getattr(args, "exposure", None) is not None:
        if action == "free_run":
            raise InfeasibleConfig(LOOP_EXPOSURE, alternatives=[f"nxs {port} set sync free_run"])
        _cfg, plan = _sync_program(hub, flows, topology, action, fps)
        fact = (pulse_exposure_fact(fps) if plan is not None and plan.pulse_exposure
                else LOOP_EXPOSURE)
        raise InfeasibleConfig(fact, alternatives=[f"nxs {port} set sync fsync --fps {fps:g}"]
                               + _fastest(hub, topology, _running_modes(topology), fps))
    return apply_sync(topology, hub, flows, action, fps, dry_run=args.dry_run)


def apply_sync(topology: Topology, hub, flows, action: str, fps: Optional[float],
               dry_run: bool = False, keep: bool = True) -> int:
    """Put the port on `action`: the hub's generator on at `fps` under
    `fsync`, one pulse per frame, with every synced sensor on its trigger,
    or off with the sensors back in free run. A whole-port change: the
    generator paces every link, and the delivery check counts every camera
    link after it; a sync the links do not deliver is refused with the
    previous one back. `set sync` runs it, and `switch` for a declared
    rate the running port does not run yet, with `keep` False: the previous
    sync comes back after the count, and the port runs as before."""
    port = _port_name(topology)
    cfg, plan = _sync_program(hub, flows, topology, action, fps)
    if action == "fsync" and plan is not None:
        term.info(plan_text(plan))
    if dry_run:
        print_stream(flatten(cfg))
        return 0
    previous = port_state.port_sync(topology)
    ok = _run_sync(hub, topology, action, fps)
    if not ok:
        # The program stopped part way: the heads hold what it wrote so far,
        # and the record must not claim the sync it ran before.
        port_state.mark_unknown(topology, list(topology.links))
        return 1
    if ok:
        _record_sync(hub, flows, topology, action,
                     fps if action == "fsync" else None,
                     plan=plan if action == "fsync" else None)
        if action == "fsync":
            for line in _crop_lines(flows, topology):
                term.info(line)
        else:
            term.info("viewers back to full frame")
        retry = (f"nxs {port} set sync fsync --fps {{fps}}" if action == "fsync"
                 else f"nxs {port} on --fps {{fps}}")
        try:
            verify.check(hub, topology, list(topology.camera_links), retry=retry)
        except InfeasibleConfig:
            try:
                _restore_sync(hub, flows, topology, previous)
            except SyncNotRestored as exc:
                term.err(str(exc))
            raise
        if not keep:
            _restore_sync(hub, flows, topology, previous)
    return 0 if ok else 1


class SyncNotRestored(Exception):
    """The sync a port ran before a trial did not come back: the port is
    left on the trial's program with its links marked unknown, and the
    message is the fact line with its next command."""


def _run_sync(hub, topology: Topology, action: str, fps: Optional[float]) -> bool:
    """Put the port on `action`: the sync walk of its graph image composed
    at `fps`, or its free-run walk. A port whose image carries no such walk
    (no generator stands on it) runs free already, with nothing to switch.
    A walk that stops prints why."""
    from nxs.cam import graph

    composition = graph.compose_port(hub, topology, fsync=fps if action == "fsync" else None)
    which = graph.Walk.SYNC if action == "fsync" else graph.Walk.FREE
    if not composition.steps(which):
        return True
    report = graph.run(hub, topology, which, composition)
    if not report.ok:
        term.err(f"{_port_name(topology)}: {report}")
    return report.ok


def _sync_program(hub, flows, topology: Topology, action: str, fps: Optional[float]):
    """The program that puts the port on `action`, the generator at `fps`
    under fsync, and the frame-sync plan it runs (None in free run)."""
    if action == "free_run":
        return flows.build_trigger_off(hub, topology), None
    kwargs: Dict[str, Any] = {"fps": fps, "method": "manual"}
    running = _running_modes(topology)
    if running and "modes" in _accepted(flows.build_fsync):
        kwargs["modes"] = running
    return (flows.build_fsync(hub, topology, **kwargs),
            _fsync_plan(flows, hub, topology, fps, kwargs.get("modes")))


def _restore_sync(hub, flows, topology: Topology, previous: Optional[Dict[str, Any]]) -> None:
    """Put the port back on the sync the record held before a change its
    links did not deliver, free run when it held none, and say so.
    SyncNotRestored when the program does not run through."""
    port = _port_name(topology)
    fps = (previous or {}).get("fps")
    action = "fsync" if (previous or {}).get("source") == "fsync" and fps is not None else "free_run"
    rate = float(fps) if action == "fsync" else None
    _cfg, plan = _sync_program(hub, flows, topology, action, rate)
    if not _run_sync(hub, topology, action, rate):
        port_state.mark_unknown(topology, list(topology.links))
        raise SyncNotRestored(f"{port}: the sync it ran before did not come back\n  - nxs {port} on")
    _record_sync(hub, flows, topology, action, rate, plan=plan)
    term.info(f"{port}: sync back to {sync_text(port_state.port_sync(topology))}")


def _running_modes(topology: Topology) -> Dict[str, str]:
    """The mode each link's last `on` ran, where the record has one."""
    return {l.name: m for l in topology.links if (m := port_state.port_mode(topology, l))}


def _fastest(hub, topology: Topology, modes: Dict[str, str],
             fps: Optional[float]) -> List[str]:
    """The command that runs the synced pair at the highest whole rate it
    takes, the shortest exposure the pulse sets; none when the pair runs
    there already or the hub names no synced rate."""
    rates = cam_timing.synced_rates(hub, topology, modes or None)
    if not rates or (fps is not None and max(rates) <= float(fps)):
        return []
    return [f"nxs {_port_name(topology)} set sync fsync --fps {max(rates)}"]


#: The standard row each host form of a knob reads.
_FORM_ROWS = {"fps": NxsMap.ParamId.FRAME_RATE_MFPS, "gain_db": NxsMap.ParamId.GAIN_MDB,
              "gain": NxsMap.ParamId.GAIN_MDB, "exposure": NxsMap.ParamId.EXPOSURE_US,
              "mode": NxsMap.ParamId.MODE}


def _open(i2c, topology: Topology) -> None:
    """Open the port's bus for a read: a bus this user may not open goes on
    as its permission refusal, any other failure is the refusal naming the
    bus."""
    try:
        i2c.open()
    except I2CHandlerError as exc:
        if isinstance(exc.__cause__, PermissionError):
            raise
        port = _port_name(topology)
        raise _refuse(f"{port}: {topology.i2c_bus} does not open ({exc.__cause__ or exc})",
                      f"nxs {port} status") from exc


def _silent_map(hub, topology: Topology, link, exc: OSError) -> SystemExit:
    """The refusal for a camera map that does not answer a read: a pod as
    the error table names one that answers at neither its alias nor where
    it straps, else the map's own silence beside a pod that answers, or a
    smart camera's."""
    port = _port_name(topology)
    if link.nxs_units:
        silent = pods._unanswered(topology, link, exc)
        if silent is not None:
            return _refuse(silent, "check the pod")
    fact = f"{port}/{link.name}: no camera door answers at {hubs.map_address(hub, link):#04x}"
    if link.nxs_units:
        return _refuse(fact, f"nxs {port} status")
    return _refuse(fact, f"check the cable and power on link {link.name}")


def _get_map(hub, topology: Topology, link, knob: str) -> int:
    """`get` on a link whose head serves the camera map: the row's value in
    effect read from the map under the bus lock, in the row's unit, the
    rate in frames a second, the gain in decibels and the mode by its name
    for their forms. Under frame sync the port's rate is the hub
    generator's. A map that does not answer is the error table's line."""
    from nxs._libnxs_map import MapRefused, open_door

    with port_state.BusLock():
        if knob == "fps" and not topology.is_direct:
            i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
            _open(i2c, topology)
            try:
                hub_rate = _hub_fsync_rate(hub, i2c, topology)
            finally:
                i2c.close()
            if hub_rate is not None:
                print(f"{hub_rate:.2f}")
                return 0
        try:
            with open_door(topology.i2c_bus, hubs.map_address(hub, link)) as door:
                live, row = door.live(_FORM_ROWS.get(knob, knob))
        except (LookupError, MapRefused) as exc:
            raise SystemExit(f"nxs: {exc}")
        except PermissionError:
            raise
        except OSError as exc:
            raise _silent_map(hub, topology, link, exc) from exc
    if knob == "fps":
        print(f"{live / 1000:.2f}")
    elif knob == "gain_db":
        print(f"{live / 1000:g}")
    elif knob == "mode":
        # The MODE row's value is the mode's index among the personality's program modes.
        modes = hub.descriptor(link.sensor_compatible).program_modes()
        print(modes[live] if 0 <= live < len(modes) else live)
    else:
        print(f"{live} {row.unit}".rstrip())
    return 0


def cmd_get(args: argparse.Namespace) -> int:
    """`get NAME`: the camera family first (the port's sync, a knob read
    back from the sensor), then the link's unit: its personality's
    parameters and the device parameters."""
    from nxs.cam.diag import run_probes

    topology, selected = select_port_links(args)
    _require_nxs_hub(topology, "get")
    hub = _hub_for(topology)
    flows = hub.flows()
    if args.knob == "sync":
        live = port_state.port_sync(topology)
        print(sync_text(live) if live else f"{topology.sync.source} (declared)")
        return 0
    if getattr(args, "links", None):
        link = selected[0]
    else:
        ups = [l for l in topology.links
               if port_state.link_state(topology, l) == port_state.STATE_UP]
        link = ups[0] if ups else topology.links[0]
    if not link.has_camera and not link.nxs_units:
        raise _refuse_pod_only(topology, link, f"get {args.knob}")
    knobs = _camera_knobs(hub, link, topology) if link.has_camera else []
    if _camera_knob(hub, link, topology, args.knob):
        _require_up(topology, [link], "get")
        if args.knob == "exposure" and pulse_sets_exposure(topology, link.name):
            # A synced link integrates for the pulse's low time; its shutter
            # register is not what the sensor runs.
            print(pulse_exposure_fact((port_state.port_sync(topology) or {}).get("fps")))
            return 0
        if hubs.map_address(hub, link) is not None and pods.camera_map_knobs(hub, link) is not None:
            return _get_map(hub, topology, link, args.knob)
        i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        hub_rate = None
        with port_state.BusLock():
            _open(i2c, topology)
            try:
                flows.open_window(hub, i2c, topology, link)
                send = hub.descriptor(link.sensor_compatible)
                results = {r.name: r
                           for r in run_probes(
                               i2c, hubs.sensor_address(hub, link), send)}
                flows.close_windows(hub, i2c, topology)
                if args.knob == "fps":
                    hub_rate = _hub_fsync_rate(hub, i2c, topology)
            except I2CHandlerError as exc:
                # The window writes are the hub's: a hub that takes none does not answer.
                port = _port_name(topology)
                raise _refuse(f"{port}: the hub does not answer at {topology.des_addr:#04x}",
                              f"check power and the coax on {port}") from exc
            finally:
                i2c.close()
        # Under frame sync the hub's generator paces the frames, not the
        # sensor's VMAX: the port's rate is the generator's.
        if hub_rate is not None:
            print(f"{hub_rate:.2f}")
            return 0
        module = cam_laws.laws(hub.descriptor(link.sensor_compatible))
        hook = getattr(module, "knob_readback", None) if module else None
        derived = hook({k: r.raw for k, r in results.items()
                        if r.raw is not None}) if hook else {}
        if args.knob in derived:
            print(derived[args.knob])
            return 0
        if args.knob in results:
            print(results[args.knob].text)
            return 0
        raise SystemExit(f"nxs: {args.knob} has no readback on {link.sensor_compatible}")
    if link.nxs_units:
        from nxs.cli import cmd_get as unit_get

        rc = _unit_verb(topology, link, unit_get, SimpleNamespace(param=args.knob))
        if rc and knobs:
            print(f"  - camera knobs: {' '.join(knobs)}", file=sys.stderr)
        return rc
    term.refusal(f"no knob {args.knob} on link {link.name}",
                 f"knobs: {' '.join(knobs)}")
    return 1


