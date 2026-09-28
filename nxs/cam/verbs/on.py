"""`on` and `off`: bring a port's links up through the plan, verify the lock and the sensors, park the port."""

from __future__ import annotations

import argparse
from typing import Any, Callable, Dict, List


from nxs import term

from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs import host as host_layer
from nxs.cam import port_state
from nxs.cam.port_state import save_port
from nxs.cam import viewers
from nxs.cam import run as cam_run
from nxs.cam import identity as cam_identity
from nxs.cam.identity import _declared_camera, _require_nxs_hub, _verify_hub_identity, detect_sensor, sensor_identity_line
from nxs.cam.run import TRAIN_ROUNDS, _accepted, _compose_from, _mode_arg, _port_links, _rates_arg, _resolve_modes, unit_program_refusal
from nxs.cam.select import _declare, _pack_for, _port_name, _refuse, select_port_links
from nxs.cam.verbs.sync import _fsync_plan, _hints_by_link, _port_viewer_hint, _record_sync, plan_text


def _prepare_hub(pack, flows, topology, links) -> None:
    """Before the program: the hub is the declared silicon, and its links
    are constructed or trained when the port is not recorded up."""
    i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
    try:
        i2c.open()
        _verify_hub_identity(pack, topology, i2c)
        reachable = flows.links_reachable(pack, i2c, topology, links)
        construct = getattr(flows, "construct", None)
        # A port this tool records as up is warm; anything else may hide a
        # pristine serializer, and reachability is no evidence of readiness.
        warm = all(port_state.link_state(topology, l) == port_state.STATE_UP
                   for l in links)
        if construct is not None and not warm:
            # A pristine serializer must be programmed at the deserializer's
            # defaults before the program's 12 G window can lock it.
            term.info("port not recorded up -> constructing (pristine "
                      "serializers get their link program first)")
            i2c.close()
            verdict = construct(
                pack, topology, links,
                lambda cfg, label, quiet: cam_run._execute(
                    cfg.to_dict(), topology.i2c_bus, label, quiet=quiet,
                    guard=(pack, topology)))
            for name, what in verdict.items():
                term.info(f"  link {name}: {what}")
            i2c.open()
        elif not reachable:
            term.info("links down -> training (dip-retry handshake)")
            results = flows.train(pack, i2c, topology, links=links,
                                  rounds=TRAIN_ROUNDS)
            if not all(r.locked for r in results.values()):
                # Not a veto: a POR hub cannot lock before the program writes
                # its link config; the video-lock oracle after it is the gate.
                term.info("links not locked pre-program, proceeding: the "
                          "program carries the link config")
    except (OSError, RuntimeError) as exc:
        # A powered-off hub is an outcome, not a traceback.
        raise _refuse(
            f"hub at {topology.i2c_bus}:{hex(topology.des_addr)} does "
            f"not answer ({exc.__class__.__name__})",
            "check DES power and cabling", f"nxs {_port_name(topology)} status",
            "nxs generate (writes what answers on the port)")
    finally:
        i2c.close()


def cmd_up(args: argparse.Namespace) -> int:
    topology, links = select_port_links(args, require_port=True)
    _require_nxs_hub(topology, "on")
    pack = _pack_for(topology)
    topology, links, modes = _declare(pack, topology, links, args)
    modes = _declared_camera(pack, topology, links, modes, args)
    flows = pack.flows()
    host = host_layer.current()
    port = _port_name(topology)
    cameras = [l for l in links if l.has_camera]
    pods = [l for l in links if not l.has_camera]
    if not links:
        term.refusal(f"{port}: no link to bring up",
                     f"ports.{port}.links.<link>.camera: <the sensor>")
        return 1
    if pods and cameras:
        # One hub brings a pod-only link and a camera link up only as a
        # pair, and a pair is two camera programs.
        term.refusal(f"{port}: link {pods[0].name} carries a pod and no camera, link "
                     f"{cameras[0].name} a camera; one hub brings them up together "
                     f"only as a pair",
                     f"nxs {port} {cameras[0].name} on",
                     f"ports.{port}.links.{pods[0].name}.camera: <the head on link "
                     f"{pods[0].name}>")
        return 1
    # The overlay fixed the capture side's lane count at boot; a program for
    # a different count gives no video and no error. Refuse it here.
    mismatch = host.lane_mismatch(topology.i2c_bus, topology.csi_lanes)
    if mismatch:
        term.err(mismatch)
        return 2
    cfg, csi = _compose_from(pack, topology, links, args, modes)
    # The booted tree must carry every selected link's mode and a capture
    # node for the channel it rides; a gap regenerates the port's overlay
    # from the pack and asks for the reboot.
    planned = _resolve_modes(flows, pack, cameras, modes, topology)
    vcs = {link.name: int(vc.vc) for link, vc in zip(cameras, csi.virtual_channels)
           if vc is not None}
    from nxs.host.cli import capture_stack_state, reboot_rule
    reboot = reboot_rule(host, pack, topology, cameras, planned,
                         install_overlays=True, vcs=vcs)
    if reboot:
        print(f"REBOOT NEEDED:\n{reboot}")
        return 3
    # The capture stack's configuration is built for the booted table by
    # nxsd after the reboot, or by `nxs switch`; `on` never builds it. A
    # pod alone streams no video and needs none.
    state = capture_stack_state(host, port, topology) if cameras else "ready"
    if state == "preparing":
        term.refusal(f"{port}: preparing the capture stack",
                     "wait for nxsd, then run this command again")
        return 1
    if state == "missing":
        term.refusal(f"{port}: the capture stack's configuration for the "
                     f"booted table is not built", "nxs switch")
        return 1
    running = _resolve_modes(flows, pack, cameras, modes, topology)
    if args.dry_run:
        # The plan's summary: what the port would be, no write stream.
        for line in _port_lines(pack, links, running, csi):
            print(line)
        return 0
    # A direct port's sequences run on the host engine with the pod at
    # their markers, so a link without one refuses before the bus is
    # touched; behind a hub the walk runs a pod-less link's sensor itself.
    refusal = unit_program_refusal(cfg, pack, topology, links=links) if topology.is_direct else None
    if refusal:
        term.err(refusal)
        return 1

    # A hub port walks its graph on the executor; the direct pack's
    # sequences run on the host engine.
    if topology.is_direct:
        def bring_up() -> bool:
            return cam_run._execute_split(cfg, topology, "up", guard=(pack, topology))
    else:
        images = cam_run.graph_images(pack)
        rates = dict(getattr(csi, "rates", None) or {})

        def bring_up() -> bool:
            return cam_run.run_graph(pack, topology, links, running, rates, images, "up")
    with port_state.BusLock():
        if not topology.is_direct:
            _prepare_hub(pack, flows, topology, links)
        ok = bring_up()
    names = "+".join(l.name for l in links)
    if ok:
        rates = dict(getattr(csi, "rates", None) or {})
        save_port(topology, links, csi,
                   viewer=(_port_viewer_hint(pack, flows, topology, links,
                                             _mode_arg(args), rates=rates or None)
                           if cameras else None),
                   viewers=(_hints_by_link(pack, flows,
                                           _port_links(topology, links),
                                           links, running or None,
                                           rates=rates or None)
                            if cameras else None),
                   modes=running, rates=rates or None)
        port_state.set_sync("free_run", topology=topology)
        for line in _port_lines(pack, links, running, csi):
            term.info(line)
    print(f"up {names}: {'ok' if ok else 'HAD ERRORS'}")
    # A declared trigger is the port's whole-port behaviour: a solo on a
    # declared fsync port runs free.
    partial = len(links) < len(topology.links)
    declared_fsync = topology.sync.source == "fsync" and not partial and bool(cameras)
    verify = getattr(flows, "video_locked", None)
    if ok and cameras and verify is not None and not declared_fsync:
        # A byte-perfect program can land on a sensor holding stale state and
        # produce no video: follow the des lock.
        dead = _dead_pipes(pack, flows, topology, csi, links)
        if dead:
            # One escalated recovery (a power dip, a reset, retraining) and
            # one more run before refusing: a head holding stale state locks
            # after it.
            term.info(f"video did not lock on pipe {'+'.join(dead)}; recovering "
                      f"the links")
            dead = _relock(pack, flows, topology, csi, links, bring_up)
        if dead:
            port = _port_name(topology)
            silent = ([l for l, p in sorted(csi.pipes.items()) if p in dead]
                      or [l.name for l in links])
            term.refusal(", ".join(f"{port}/{l}" for l in silent) + ": video did not lock",
                         "power-cycle the hub", f"nxs {port} status")
            # The port is not up: the next `on` reconstructs rather than
            # taking the warm path, and running-stream verbs refuse.
            port_state.mark_unknown(topology, links)
            return 1
        term.info("video locked")
    if ok and cameras and not _sensors_verified(pack, flows, topology, cameras):
        port_state.mark_unknown(topology, links)
        return 1
    if ok and topology.sync.source == "fsync" and partial:
        term.warn(f"declared sync: fsync is {_port_name(topology)}'s two-camera "
                  f"port, {names} runs free\n  - nxs {_port_name(topology)} on "
                  f"(the trigger needs every link)")
    if ok and declared_fsync:
        # One rate asked of the bring-up is the generator's; else the manifest's.
        asked = _rates_arg(args, links) or {}
        if len(set(asked.values())) == 1:
            fps = float(next(iter(asked.values())))
        else:
            fps = float(topology.camera_fps or topology.sync.fps or 30.0)
        exposure = topology.camera_exposure_us
        kwargs: Dict[str, Any] = {"fps": fps, "method": "manual"}
        if exposure is not None:
            if "exposure_us" not in _accepted(flows.build_fsync):
                raise InfeasibleConfig(
                    "the declared frame sync names an exposure this pack's "
                    "frame sync does not take")
            kwargs["exposure_us"] = exposure
        overlay = flows.build_fsync(pack, topology, **kwargs)
        plan = _fsync_plan(flows, pack, topology, fps, exposure, None)
        with port_state.BusLock():
            ok = cam_run._execute_split(overlay, topology, "fsync", guard=(pack, topology))
        if ok:
            _record_sync(pack, flows, topology, "fsync", fps, plan=plan)
            print(f"declared sync: {plan_text(plan) if plan else f'fsync {fps:g} fps'}")
    if ok and cameras:
        sel = " ".join(l.name for l in links)
        print(f"next: nxs {port} {sel} stream")
    elif ok:
        print("next: nxs status")
    return 0 if ok else 1


def _dead_pipes(pack, flows, topology: Topology, csi, links: List[LinkSpec]) -> List[str]:
    """The pipes the selected links ride that carry no video lock."""
    with port_state.BusLock():
        i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        try:
            i2c.open()
            locks = flows.video_locked(pack, i2c, topology, csi, links=links)
        except (OSError, RuntimeError):
            locks = {"Y": False}
        finally:
            i2c.close()
    return [p for p, alive in sorted(locks.items()) if not alive]


def _relock(pack, flows, topology: Topology, csi, links: List[LinkSpec],
            bring_up: Callable[[], bool]) -> List[str]:
    """Recover the links (the pack's power dip, reset and retrain), bring
    them up again, and read the locks back; the pipes still dead."""
    recover = getattr(flows, "recover", None)
    if recover is None:
        return _dead_pipes(pack, flows, topology, csi, links)
    with port_state.BusLock():
        i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        try:
            i2c.open()
            recover(pack, i2c, topology, rounds=TRAIN_ROUNDS)
        except (OSError, RuntimeError) as exc:
            term.info(f"recovery did not complete: {exc}")
        finally:
            i2c.close()
        if not bring_up():
            return ["Y"]
    return _dead_pipes(pack, flows, topology, csi, links)


def _port_lines(pack, links: List[LinkSpec], running: Dict[str, str],
                 csi) -> List[str]:
    """One line per link: the sensor, the mode as a person names it, and
    the rate the sensor runs."""
    from nxs.cam.descriptors import mode_label

    rates = dict(getattr(csi, "rates", None) or {})
    lines = []
    for link in links:
        if not link.has_camera:
            pod = link.nxs_units[0] if link.nxs_units else None
            lines.append(f"link {link.name}: pod at {int(pod.alias_addr):#04x}" if pod
                         else f"link {link.name}: pod")
            continue
        text = f"link {link.name}: {link.sensor_compatible}"
        mode = running.get(link.name)
        if mode:
            try:
                text += f" {mode_label(pack.descriptor(link.sensor_compatible), mode)}"
            except Exception:
                text += f" {mode}"
        rate = rates.get(link.name)
        if rate is not None:
            text += f" @ {rate:.10g} fps" if float(rate).is_integer() else f" @ {rate:.2f} fps"
        lines.append(text)
    return lines


def _sensors_verified(pack, flows, topology: Topology,
                      links: List[LinkSpec]) -> bool:
    """After the program: an identity-bearing sensor must answer as the
    declared part through its link window; declared-only parts pass.
    What answers is recorded, so the port record remembers the sensor."""
    if not any(cam_identity.identity_facts(pack.descriptor(c)) is not None
               for c in pack.sensors()):
        return True
    open_window = getattr(flows, "open_window", None)
    if open_window is None:
        return True
    found: Dict[str, str] = {}
    bad: List[str] = []
    with port_state.BusLock():
        i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        try:
            i2c.open()
            for link in links:
                open_window(pack, i2c, topology, link)
                detected, detail = detect_sensor(pack, i2c, link)
                ok, text = sensor_identity_line(
                    link, pack.descriptor(link.sensor_compatible), detected,
                    detail)
                if detected:
                    found[link.name] = detected
                if not ok:
                    bad.append(f"link {link.name}: {text}")
            close = getattr(flows, "close_windows", None)
            if close is not None:
                close(pack, i2c, topology)
        except Exception as exc:
            # An after-the-fact check: a walk that cannot complete is reported,
            # never a failed bring-up.
            term.warn(f"sensor identity not checked: {exc}")
            return True
        finally:
            i2c.close()
    if found:
        port_state.set_sensors(topology, found)
    for line in bad:
        term.err(line)
    if bad:
        term.err(f"the program ran for the declared sensor(s); say the "
                 f"right one: nxs {_port_name(topology)} <link> on --sensor "
                 f"<name>")
    return not bad


def cmd_down(args: argparse.Namespace) -> int:
    topology, selected = select_port_links(args, require_port=True)
    _require_nxs_hub(topology, "off")
    if topology.links and len(selected) < len(topology.links):
        # The park program gates the port's capture output and stands every
        # sensor by: there is no one-link park.
        raise _refuse("off parks the whole port", f"nxs {_port_name(topology)} off")
    return park_port(topology, selected)


def park_port(topology: Topology, selected: List[LinkSpec]) -> int:
    """Stop the selected links' viewers and, for the whole port, run the pack's
    park program; takes the port itself, so a caller can park a declaration
    the manifest does not carry."""
    for stopped in viewers.stop_viewers(topology, selected):
        print(f"viewer for link {stopped} stopped")
    if len(selected) == len(topology.links) and topology.links:
        pack = _pack_for(topology)
        cfg = pack.flows().build_park(pack, topology)
        with port_state.BusLock():
            ok = cam_run._execute(cfg.to_dict(), topology.i2c_bus, "down", guard=(pack, topology))
        if ok:
            port_state.mark_parked(topology, selected)
        print("port parked (sensors in standby)"
              if ok else "park HAD ERRORS")
        return 0 if ok else 1
    return 0


