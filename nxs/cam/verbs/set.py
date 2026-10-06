"""`set` and `get`: a knob changed under the laws or read back, the sync source included."""

from __future__ import annotations

import argparse
import sys
from types import SimpleNamespace
from typing import Any, Dict, List, Optional


from nxs import term

from nxs.cam.contracts import InfeasibleConfig, Topology
from nxs.cam.plan import flatten
from nxs.cam import port_state
from nxs.cam import packs, unit_source
from nxs.cam import run as cam_run
from nxs.cam import timing as cam_timing
from nxs.cam.identity import _require_nxs_hub
from nxs.cam.run import _accepted
from nxs.cam.select import _pack_for, _port_name, _refuse, _refuse_pod_only, _require_up, print_stream, select_port_links
from nxs.cam.verbs import verify
from nxs.cam.verbs.caps import _camera_knobs, _hub_fsync_rate
from nxs.cam.verbs.sync import (LOOP_EXPOSURE, _fsync_plan, _record_hints, _record_sync, pair_gain_fact,
                                plan_text, pulse_exposure_fact, pulse_sets_exposure, sync_text)


def _sensor_readings(pack, topology: Topology, link) -> Dict[str, int]:
    """Live sensor status readings through the link's window ({} when the bus
    is unavailable); the vmax/hmax context knob builders need."""
    from nxs.cam.diag import run_probes

    flows = pack.flows()
    i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
    try:
        i2c.open()
    except Exception:
        return {}
    try:
        flows.open_window(pack, i2c, topology, link)
        send = pack.descriptor(link.sensor_compatible)
        results = run_probes(i2c, packs.sensor_address(pack, link), send)
        flows.close_windows(pack, i2c, topology)
        return {r.name: r.raw for r in results if r.raw is not None}
    except Exception:
        return {}
    finally:
        i2c.close()


def cmd_set(args: argparse.Namespace) -> int:
    """`set NAME VALUE`: `sync` is the port's frame sync (`fsync` starts the
    hub's generator, `free_run` stops it); every other knob is the link's
    sensor's, composed under the pack's laws. Every refusal starts with the
    port's name, the laws' own included; the delivery check's name it
    already."""
    topology, _ = select_port_links(args)
    _require_nxs_hub(topology, "set")
    # The laws judge a knob at the line the delivery check found for the port.
    topology = port_state.with_found_lines(topology)
    pack = _pack_for(topology)
    try:
        return _set(args, topology, pack, pack.flows())
    except InfeasibleConfig as exc:
        port = _port_name(topology)
        named = exc.reason.startswith((f"{port}:", f"{port}/"))
        raise InfeasibleConfig(exc.reason if named else f"{port}: {exc.reason}",
                               alternatives=exc.alternatives)


def _set(args: argparse.Namespace, topology: Topology, pack, flows) -> int:
    """`set` on the port: the sync, else a link's knob."""
    if args.knob == "sync":
        return _set_sync(args, topology, pack, flows)
    if args.knob == "exposure" and pulse_sets_exposure(topology, args.link or topology.links[0].name):
        # A synced link's exposure is its trigger pulse's; a link the trigger
        # leaves free-running keeps its own.
        port = _port_name(topology)
        fps = (port_state.port_sync(topology) or {}).get("fps")
        raise InfeasibleConfig(pulse_exposure_fact(fps),
                               alternatives=[f"nxs {port} set sync free_run"]
                               + _fastest(pack, topology, _running_modes(topology), fps))
    pair_gain = pair_gain_fact(topology) if args.knob in ("gain", "gain_db") else None
    if pair_gain:
        # A gain set by hand leaves the pair's one gain: the leader's loop or
        # the follower moves it back, or it breaks the declared lock.
        raise InfeasibleConfig(pair_gain, alternatives=[
            f"ports.{_port_name(topology)}.camera.gain_db: <dB> in suite.yaml, then nxs switch"])
    if args.link is None and len(topology.links) > 1:
        names = "|".join(l.name for l in topology.links)
        raise _refuse(
            f"set on {_port_name(topology)} is ambiguous (links: {names})",
            f"nxs {_port_name(topology)} {names.split('|')[0]} set {args.knob} "
            f"{args.value}")
    link_spec = (topology.link(args.link) if args.link
                 else topology.links[0])
    if not link_spec.has_camera:
        raise _refuse_pod_only(topology, link_spec, f"set {args.knob} {args.value}")
    import inspect

    # The running port's mode judges the knob; a pack whose knob planner
    # takes none keeps its own default.
    extra: Dict[str, Any] = {}
    if "mode" in inspect.signature(flows.build_knob).parameters:
        extra["mode"] = port_state.port_mode(topology, link_spec)

    def compose(readings, value=args.value):
        try:
            return flows.build_knob(
                pack, topology, args.knob, value,
                link=args.link, readings=readings, **extra,
            )
        except KeyError as exc:
            raise SystemExit(f"nxs: {exc}")

    if args.dry_run:
        # Inert by contract: compose from descriptor defaults, no bus IO.
        print_stream(flatten(compose({})))
        return 0
    targets = [link_spec] if args.link else list(topology.links)
    _require_up(topology, targets, "set")
    # CTRL0's low bits are link enables: isolating one link's sensor
    # powers down its live sibling (and nothing re-arms the pipes).
    others_up = [l for l in topology.links
                 if l.name != link_spec.name
                 and port_state.link_state(topology, l)
                 == port_state.STATE_UP]
    if args.link and others_up:
        names = ", ".join(l.name for l in others_up)
        raise _refuse(
            f"nxs: set to link {link_spec.name} would drop live video on {names}",
            "set before stream", f"nxs {_port_name(topology)} off (park first)")
    # One lock spans the live readings, composition, and execution so a
    # concurrent run cannot move the window between read and write.
    with port_state.BusLock():
        cfg = compose(_sensor_readings(pack, topology, link_spec))
        ok = cam_run._execute(cfg.to_dict(), topology.i2c_bus, f"set-{args.knob}",
                      guard=(pack, topology))
    if ok and args.knob == "fps":
        _hold_rate(args, topology, pack, flows, link_spec, compose)
    return 0 if ok else 1


def _record_rate(pack, flows, topology: Topology, link, fps: float) -> None:
    """Record the rate a link's sensor now runs and re-derive the caps the
    capture stack's sessions open with, each link's part in the exposure
    and gain the recorded sync names kept."""
    port_state.set_rates(topology, {link.name: fps})
    sync = port_state.port_sync(topology) or {}
    triggered = sync.get("source") == "fsync"
    _record_hints(pack, flows, topology, triggered,
                  sync.get("trigger_vmax") if triggered else None, ae=sync.get("ae"))


def _hold_rate(args: argparse.Namespace, topology: Topology, pack, flows, link,
               compose) -> None:
    """`set fps` holds the link to the rate it asked. The rate and the caps
    are recorded before the count, since the capture stack's driver sets
    the frame from the caps' rate when a session starts; a rate the link
    does not deliver is refused with the previous rate written back and
    recorded."""
    port = _port_name(topology)
    previous = port_state.port_rate(topology, link)
    _record_rate(pack, flows, topology, link, float(args.value))
    at = f" {link.name}" if args.link else ""
    try:
        verify.check(pack, topology, [link], retry=f"nxs {port}{at} set fps {{fps}}")
    except InfeasibleConfig:
        if previous is not None:
            with port_state.BusLock():
                cfg = compose(_sensor_readings(pack, topology, link), str(previous))
                back = cam_run._execute(cfg.to_dict(), topology.i2c_bus, "set-fps",
                                        guard=(pack, topology))
            if back:
                _record_rate(pack, flows, topology, link, previous)
                term.info(f"{port}/{link.name}: back at {previous:g} fps")
        raise


def _crop_lines(flows, topology: Topology) -> List[str]:
    """What the viewers crop under the trigger, from the recorded caps: the
    filler rows a triggered frame's capture window ends in (the pack's
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


def _set_sync(args: argparse.Namespace, topology: Topology, pack, flows) -> int:
    """`set sync fsync|free_run [--fps F]`: the hub's generator on at `fps`,
    one pulse per frame, with every synced sensor on its trigger, or off
    with the sensors back in free run. A whole-port change: the generator
    paces every link, and the delivery check counts every camera link after
    it; a sync the links do not deliver is refused with the previous one
    back. `--exposure` is refused with the fact that sets the exposure: the
    pulse at the asked rate, else the capture stack's loop."""
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
    asked = getattr(args, "exposure", None) is not None
    if action == "free_run" and asked:
        raise InfeasibleConfig(LOOP_EXPOSURE, alternatives=[f"nxs {port} set sync free_run"])
    cfg, plan = _sync_program(pack, flows, topology, action, fps)
    if action == "fsync":
        if asked:
            fact = (pulse_exposure_fact(fps) if plan is not None and plan.pulse_exposure
                    else LOOP_EXPOSURE)
            raise InfeasibleConfig(fact, alternatives=[f"nxs {port} set sync fsync --fps {fps:g}"]
                                   + _fastest(pack, topology, _running_modes(topology), fps))
        if plan is not None:
            term.info(plan_text(plan))
    if args.dry_run:
        print_stream(flatten(cfg))
        return 0
    previous = port_state.port_sync(topology)
    with port_state.BusLock():
        ok = cam_run._execute_split(cfg, topology, f"set-sync-{action}",
                            guard=(pack, topology))
    if ok:
        _record_sync(pack, flows, topology, action,
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
            verify.check(pack, topology, list(topology.camera_links), retry=retry)
        except InfeasibleConfig:
            _restore_sync(pack, flows, topology, previous)
            raise
    return 0 if ok else 1


def _sync_program(pack, flows, topology: Topology, action: str, fps: Optional[float]):
    """The program that puts the port on `action`, the generator at `fps`
    under fsync, and the frame-sync plan it runs (None in free run)."""
    if action == "free_run":
        return flows.build_trigger_off(pack, topology), None
    kwargs: Dict[str, Any] = {"fps": fps, "method": "manual"}
    running = _running_modes(topology)
    if running and "modes" in _accepted(flows.build_fsync):
        kwargs["modes"] = running
    return (flows.build_fsync(pack, topology, **kwargs),
            _fsync_plan(flows, pack, topology, fps, kwargs.get("modes")))


def _restore_sync(pack, flows, topology: Topology, previous: Optional[Dict[str, Any]]) -> None:
    """Put the port back on the sync the record held before a change its
    links did not deliver, free run when it held none, and say so."""
    port = _port_name(topology)
    fps = (previous or {}).get("fps")
    action = "fsync" if (previous or {}).get("source") == "fsync" and fps is not None else "free_run"
    rate = float(fps) if action == "fsync" else None
    cfg, plan = _sync_program(pack, flows, topology, action, rate)
    with port_state.BusLock():
        ok = cam_run._execute_split(cfg, topology, f"set-sync-{action}", guard=(pack, topology))
    if not ok:
        term.err(f"{port}: the sync it ran before did not come back\n  - nxs {port} on")
        return
    _record_sync(pack, flows, topology, action, rate, plan=plan)
    term.info(f"{port}: sync back to {sync_text(port_state.port_sync(topology))}")


def _running_modes(topology: Topology) -> Dict[str, str]:
    """The mode each link's last `on` ran, where the record has one."""
    return {l.name: m for l in topology.links if (m := port_state.port_mode(topology, l))}


def _fastest(pack, topology: Topology, modes: Dict[str, str],
             fps: Optional[float]) -> List[str]:
    """The command that runs the synced pair at the highest whole rate it
    takes, the shortest exposure the pulse sets; none when the pair runs
    there already or the pack names no synced rate."""
    rates = cam_timing.synced_rates(pack, topology, modes or None)
    if not rates or (fps is not None and max(rates) <= float(fps)):
        return []
    return [f"nxs {_port_name(topology)} set sync fsync --fps {max(rates)}"]


def cmd_get(args: argparse.Namespace) -> int:
    """`get NAME`: the camera family first (the port's sync, a knob read
    back from the sensor), then the link's unit: its personality's
    parameters and the device parameters."""
    from nxs.cam.diag import run_probes

    topology, selected = select_port_links(args)
    _require_nxs_hub(topology, "get")
    pack = _pack_for(topology)
    flows = pack.flows()
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
    if not link.has_camera:
        raise _refuse_pod_only(topology, link, f"get {args.knob}")
    knobs = _camera_knobs(pack, link, topology)
    # The rate and the exposure read back on every sensor (the readback
    # hook derives them from the timing registers), knob or not.
    if args.knob in knobs or args.knob in ("fps", "exposure"):
        _require_up(topology, [link], "get")
        if args.knob == "exposure" and pulse_sets_exposure(topology, link.name):
            # A synced link integrates for the pulse's low time; its shutter
            # register is not what the sensor runs.
            print(pulse_exposure_fact((port_state.port_sync(topology) or {}).get("fps")))
            return 0
        i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        hub_rate = None
        with port_state.BusLock():
            i2c.open()
            try:
                flows.open_window(pack, i2c, topology, link)
                send = pack.descriptor(link.sensor_compatible)
                results = {r.name: r
                           for r in run_probes(
                               i2c, packs.sensor_address(pack, link), send)}
                flows.close_windows(pack, i2c, topology)
                if args.knob == "fps":
                    hub_rate = _hub_fsync_rate(pack, i2c, topology)
            finally:
                i2c.close()
        # Under frame sync the hub's generator paces the frames, not the
        # sensor's VMAX: the port's rate is the generator's.
        if hub_rate is not None:
            print(f"{hub_rate:.2f}")
            return 0
        module = pack.chip_module(link.sensor_compatible)
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

        client = unit_source._unit_client(topology, link)
        try:
            with port_state.BusLock():
                rc = unit_get(client, SimpleNamespace(param=args.knob))
        finally:
            client.close()
        if rc:
            print(f"  - camera knobs: {' '.join(knobs)}", file=sys.stderr)
        return rc
    term.refusal(f"no knob {args.knob} on link {link.name}",
                 f"knobs: {' '.join(knobs)}")
    return 1


