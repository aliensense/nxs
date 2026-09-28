"""`set` and `get`: a knob changed under the laws or read back, the sync source included."""

from __future__ import annotations

import argparse
import sys
from types import SimpleNamespace
from typing import Any, Dict


from nxs import term

from nxs.cam.contracts import InfeasibleConfig, Topology
from nxs.cam.plan import flatten
from nxs.cam import port_state
from nxs.cam import packs, unit_source
from nxs.cam import run as cam_run
from nxs.cam.identity import _require_nxs_hub
from nxs.cam.run import _accepted
from nxs.cam.select import _pack_for, _port_name, _refuse, _refuse_pod_only, _require_up, print_stream, select_port_links
from nxs.cam.verbs.caps import _camera_knobs, _hub_fsync_rate
from nxs.cam.verbs.sync import _budget_warning, _fsync_plan, _record_sync, plan_text, sync_text


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
    sensor's, composed under the pack's laws."""
    topology, _ = select_port_links(args)
    _require_nxs_hub(topology, "set")
    pack = _pack_for(topology)
    flows = pack.flows()
    if args.knob == "sync":
        return _set_sync(args, topology, pack, flows)
    if args.knob == "exposure":
        sync = port_state.port_sync(topology) or {}
        if sync.get("source") == "fsync":
            port = _port_name(topology)
            raise _refuse(
                f"{port}: under frame sync the exposure is the trigger pulse's low time",
                f"nxs {port} set sync fsync --fps {sync.get('fps', '<fps>')} --exposure <us>")
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

    def compose(readings):
        try:
            return flows.build_knob(
                pack, topology, args.knob, args.value,
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
    return 0 if ok else 1


def _set_sync(args: argparse.Namespace, topology: Topology, pack, flows) -> int:
    """`set sync fsync|free_run [--fps F] [--exposure US]`: the hub's
    generator on at `fps` with every synced sensor on its trigger, or
    off with the sensors back in free run. A whole-port change: the
    generator paces every link."""
    action = str(args.value).lower()
    if action not in ("fsync", "free_run"):
        raise SystemExit(f"nxs: sync is fsync or free_run, not {args.value!r}")
    if args.link is not None:
        raise SystemExit(f"nxs: sync is {_port_name(topology)}'s, not one link's: "
                         f"nxs {_port_name(topology)} set sync {action}")
    if not args.dry_run:
        _require_up(topology, list(topology.links), f"set sync {action}")
    fps = (float(args.fps) if getattr(args, "fps", None) is not None
           else float(topology.camera_fps or topology.sync.fps or 30.0))
    plan = None
    if action == "free_run":
        cfg = flows.build_trigger_off(pack, topology)
    else:
        kwargs: Dict[str, Any] = {"fps": fps, "method": "manual"}
        if getattr(args, "exposure", None) is not None:
            if "exposure_us" not in _accepted(flows.build_fsync):
                raise InfeasibleConfig("this pack's frame sync takes no exposure")
            kwargs["exposure_us"] = float(args.exposure)
        running = {l.name: m for l in topology.links
                   if (m := port_state.port_mode(topology, l))}
        if running and "modes" in _accepted(flows.build_fsync):
            kwargs["modes"] = running
        cfg = flows.build_fsync(pack, topology, **kwargs)
        plan = _fsync_plan(flows, pack, topology, fps,
                           kwargs.get("exposure_us"), kwargs.get("modes"))
        if plan is not None:
            term.info(plan_text(plan))
        # The laws first; the host budget is advice on a lawful rate.
        _budget_warning(pack, topology, fps)
    if args.dry_run:
        print_stream(flatten(cfg))
        return 0
    with port_state.BusLock():
        ok = cam_run._execute_split(cfg, topology, f"set-sync-{action}",
                            guard=(pack, topology))
    if ok:
        _record_sync(pack, flows, topology, action,
                     fps if action == "fsync" else None,
                     plan=plan if action == "fsync" else None)
        term.info("viewers now crop the trigger filler tail"
                  if action == "fsync" else "viewers back to full frame")
    return 0 if ok else 1


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


