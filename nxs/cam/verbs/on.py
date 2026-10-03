"""`on` and `off`: bring a port's links up through the plan, verify the lock and the sensors, build the capture
stack's configuration the booted table lacks, verify the delivery, park the port."""

from __future__ import annotations

import argparse
import dataclasses
import functools
from typing import Callable, Dict, List, Optional


from nxs import term

from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs import host as host_layer
from nxs.cam import port_state
from nxs.cam.port_state import save_port
from nxs.cam import viewers
from nxs.cam import run as cam_run
from nxs.cam import identity as cam_identity
from nxs.cam.identity import _declared_camera, _require_nxs_hub, _verify_hub_identity, detect_sensor, sensor_identity_line
from nxs.cam.run import TRAIN_ROUNDS, _compose_from, _mode_arg, _port_links, _rates_arg, _resolve_modes, unit_program_refusal
from nxs.cam.select import _declare, _pack_for, _port_name, _refuse, select_port_links
from nxs.cam.verbs import verify
from nxs.cam.verbs.sync import (_fsync_plan, _hints_by_link, _pair_ae, _port_viewer_hint, _record_sync,
                                declared_exposure_fact, plan_text)


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
    declared_ports = _manifest_ports(topology)
    topology = _as_recorded(topology, links, args)
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
    # A mode the port's table boots no row for is refused here, before the
    # overlay is written: the table carries one pixel format, chosen by the
    # declaration, so regenerating it carries the same rows.
    from nxs.host import capture_table as tables
    partial = len(links) < len(topology.links)
    chosen = f" {' '.join(l.name for l in links)}" if partial else ""
    for link in cameras:
        if link.name in planned:
            tables.refuse_without_row(pack, topology, link, pack.descriptor(link.sensor_compatible),
                                      planned[link.name], port, f"nxs {port}{chosen} on")
    vcs = {link.name: int(vc.vc) for link, vc in zip(cameras, csi.virtual_channels)
           if vc is not None}
    from nxs.host.cli import BootTableRefused, capture_stack_state, reboot_rule
    try:
        reboot = reboot_rule(host, pack, topology, cameras, planned,
                             install_overlays=True, vcs=vcs, declared=declared_ports)
    except BootTableRefused as exc:
        # Nothing was installed: the gap, then why, and no reboot to ask for.
        term.refusal_text("\n".join(exc.gap + [str(exc)]))
        return 1
    if reboot:
        print(f"REBOOT NEEDED:\n{reboot}")
        return 3
    # The capture stack's configuration for the booted table is built after
    # the walk, once the heads stream (`_build_capture_stack`); a build in
    # another run holds the port meanwhile. A pod alone streams no video
    # and needs none.
    state = capture_stack_state(host, port, topology) if cameras else "ready"
    if state == "preparing":
        term.refusal(f"{port}: preparing the capture stack",
                     "wait for the build, then run this command again")
        return 1
    running = _resolve_modes(flows, pack, cameras, modes, topology)
    # A declared trigger is the port's whole-port behaviour: a solo on a
    # declared fsync port runs free.
    declared_fsync = topology.sync.source == "fsync" and not partial and bool(cameras)
    # A declared gain locks the camera links brought up; a solo on a declared
    # fsync port runs its own loop.
    locked = (bool(cameras) and topology.camera_gain_db is not None
              and (declared_fsync or topology.sync.source != "fsync"))
    if topology.camera_exposure_us is not None and (declared_fsync or not locked):
        # A declared exposure is refused before the bus: the pulse sets it on a
        # synced pair, the link's own loop unless a declared gain locks it.
        raise InfeasibleConfig(
            f"ports.{port}.camera.exposure_us: "
            + declared_exposure_fact(topology, declared_fsync),
            alternatives=["drop the key"])
    lock = _gain_lock(pack, flows, _port_links(topology, links), declared_fsync) if locked else None
    overlay = plan = None
    if declared_fsync:
        # The trigger overlay is composed and its rate judged before the first
        # bus write, at the one rate the bring-up asks, else the declaration's.
        asked = _rates_arg(args, links) or {}
        fps = (float(next(iter(asked.values()))) if len(set(asked.values())) == 1
               else topology.synced_fps)
        overlay = flows.build_fsync(pack, topology, fps=fps, method="manual", modes=running)
        plan = _fsync_plan(flows, pack, topology, fps, running)
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
    try:
        with port_state.BusLock():
            if not topology.is_direct:
                _prepare_hub(pack, flows, topology, links)
            ok = bring_up()
    except cam_run.WalkStopped as exc:
        return _walk_stopped(pack, topology, links, partial, str(exc))
    names = "+".join(l.name for l in links)
    record = functools.partial(_record_up, pack, flows, topology, links, csi, running, args)
    if ok:
        record()
        port_state.set_sync("free_run", topology=topology,
                            ae=_pair_ae(pack, flows, topology, "free_run"))
        for line in _port_lines(pack, links, running, csi):
            term.info(line)
    print(f"up {names}: {'ok' if ok else 'HAD ERRORS'}")
    video_locked = getattr(flows, "video_locked", None)
    if ok and cameras and video_locked is not None and not declared_fsync:
        # A byte-perfect program can land on a sensor holding stale state and
        # produce no video: follow the des lock.
        dead = _dead_pipes(pack, flows, topology, csi, links)
        if dead:
            # One escalated recovery (a power dip, a reset, retraining) and
            # one more run before refusing: a head holding stale state locks
            # after it.
            term.info(f"video did not lock on pipe {'+'.join(dead)}; recovering "
                      f"the links")
            try:
                dead = _relock(pack, flows, topology, csi, links, bring_up)
            except cam_run.WalkStopped as exc:
                return _walk_stopped(pack, topology, links, partial, str(exc))
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
        with port_state.BusLock():
            ok = cam_run._execute_split(overlay, topology, "fsync", guard=(pack, topology))
        if ok:
            _record_sync(pack, flows, topology, "fsync", fps, plan=plan)
            print(f"declared sync: {plan_text(plan) if plan else f'fsync {fps:g} fps'}")
    if ok and lock is not None:
        # Once, over the converted heads and before the count: the sessions the
        # count opens run their loops locked at the gain the heads carry.
        with port_state.BusLock():
            ok = cam_run._execute_split(lock, topology, "gain-lock", guard=(pack, topology))
        if ok:
            print(f"declared gain: {float(topology.camera_gain_db):.1f} dB on "
                  + " and ".join(l.name for l in cameras))
    if ok and cameras and state == "missing":
        # The build's sessions open each head through the kernel driver at
        # the host address the walk's translation gives it: the build
        # follows the walk, and the count follows the build.
        ok = _build_capture_stack(host, topology, links, f"nxs {port}{chosen} on")
    if ok and cameras:
        # The port is up once every camera link delivers the rate it runs; a
        # hub pair finds the line its line memory carries on the way.
        retry = f"nxs {port}{chosen} on --fps {{fps}}"
        try:
            if _finds_its_line(flows, topology, cameras):
                verify.search(pack, topology, cameras, retry, record)
            else:
                verify.check(pack, topology, cameras, retry=retry)
        except InfeasibleConfig as exc:
            port_state.mark_unknown(topology, links)
            term.refusal(exc.reason, *exc.alternatives)
            return 2
    if ok and not getattr(args, "inner", False):
        # The operator's own `on` names the next verb; `switch` and nxsd go on with theirs.
        sel = " ".join(l.name for l in links)
        print(f"next: nxs {port} {sel} stream" if cameras else "next: nxs status")
    return 0 if ok else 1


def _walk_stopped(pack, topology: Topology, links: List[LinkSpec], partial: bool,
                  line: str) -> int:
    """A walk that stopped under way parks what it started, then ends on
    its line and the port's status under it, the last lines `nxsd` reads
    its verdict from. A pod run still live is aborted, and for the whole
    port the host's park program stands every head by and each pod whose
    run ended parks its own: no sensor is left streaming, no pod left
    running. The links stay unknown, so the next `on`, the watch and a
    reload bring them up again."""
    port = _port_name(topology)
    port_state.mark_unknown(topology, links)
    cam_run.abort_pods(topology, links)
    if not partial:
        ok = _park(pack, topology)
        term.info(f"{port}: parked after the stopped bring-up" if ok else "park HAD ERRORS")
    term.refusal(line, f"nxs {port} status")
    return 1


def _manifest_ports(topology: Topology) -> Optional[List[str]]:
    """The ports the manifest declares when this port is one of them (it
    carries the declaration's digest); None for a port brought up by hand,
    whose boot entry keeps every port it names."""
    if topology.declared is None:
        return None
    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config

    try:
        return sorted(load_suite_config(default_config_path()).ports)
    except (ManifestError, OSError):
        return None


def _as_recorded(topology: Topology, links: List[LinkSpec], args: argparse.Namespace) -> Topology:
    """The port as `on` records it: carrying the manifest's declaration
    (`declared`) only when the bring-up runs it as declared. A flag that
    declares by hand (`--sensor`, `--mode`, `--fps`) or some of the port's
    links alone records none, so `switch` and a reload of nxsd bring the
    port back to its declaration."""
    by_hand = any(getattr(args, flag, None) for flag in ("sensor", "mode", "fps"))
    if by_hand or len(links) < len(topology.links):
        return dataclasses.replace(topology, declared=None)
    return topology


def _gain_lock(pack, flows, topology: Topology, synced: bool):
    """The pack's program writing the declared `camera.gain_db` to the heads of
    the port's camera links, composed and judged before the first bus write.

    Raises:
        InfeasibleConfig: The pack's refusal, or a pack that writes no gain
            lock, under the declaration's key.
    """
    gain_db = topology.camera_gain_db
    where = f"ports.{_port_name(topology)}.camera.gain_db"
    hook = getattr(flows, "build_gain_lock", None)
    if hook is None:
        raise InfeasibleConfig(f"{where}: pack {pack.name} writes no gain lock",
                               alternatives=["drop the key"])
    try:
        return hook(pack, topology, float(gain_db), synced)
    except InfeasibleConfig as exc:
        raise InfeasibleConfig(f"{where}: {exc.reason}", alternatives=exc.alternatives) from exc


def _record_up(pack, flows, topology: Topology, links: List[LinkSpec], csi,
               running: Dict[str, str], args: argparse.Namespace,
               line: Optional[Dict[str, int]] = None) -> Topology:
    """Record the port up, each camera link's capture caps derived at
    `line` (HMAX by link, the pair's line the delivery check found; the
    datasheet's when None) with its part in the free-running port's
    exposure and gain; the port at that line."""
    cameras = [l for l in links if l.has_camera]
    port = topology.with_lines({name: (running[name], hmax) for name, hmax in (line or {}).items()})
    rates = dict(getattr(csi, "rates", None) or {}) or None
    save_port(topology, links, csi,
              viewer=(_port_viewer_hint(pack, flows, port, links, _mode_arg(args), rates=rates)
                      if cameras else None),
              viewers=(_hints_by_link(pack, flows, _port_links(port, links), links,
                                      running or None, rates=rates,
                                      ae=_pair_ae(pack, flows, topology, "free_run"))
                       if cameras else None),
              modes=running, rates=rates, line=line)
    return port


def _finds_its_line(flows, topology: Topology, cameras: List[LinkSpec]) -> bool:
    """A pair behind a hub whose pack reads the line-memory overflow and
    retimes the pair: its line is found on the rig; a solo and a direct
    port run the datasheet's."""
    return (not topology.is_direct and len(cameras) == 2
            and all(hasattr(flows, hook) for hook in ("line_overflow", "build_timing")))


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


def _build_capture_stack(host, topology: Topology, links: List[LinkSpec], retry: str) -> bool:
    """The capture stack's configuration for the booted table, built (or
    installed from the store) once the walk has the heads streaming: the
    build's sessions open each head through the kernel driver at the host
    address the walk's translation gives it, and a hub that lost power
    lost the translation, so a build before the walk finds no head and
    times out. `status` reads `preparing the capture stack` meanwhile. A
    build that fails is refused with the fact and `retry`, the links left
    unknown as after a failed count. True when the stack is ready."""
    from nxs.host.cli import TuningRefused, ensure_tuning, preparing_marker

    port = _port_name(topology)
    marker = preparing_marker(port)
    try:
        marker.write_text("")
        done = ensure_tuning(host, topology, log=print)
    except TuningRefused as exc:
        port_state.mark_unknown(topology, links)
        term.refusal(f"{port}: {exc.fact}", *exc.alternatives)
        return False
    except (RuntimeError, PermissionError) as exc:
        port_state.mark_unknown(topology, links)
        term.refusal(f"{port}: the capture stack's configuration was not built ({exc})", retry)
        return False
    finally:
        marker.unlink(missing_ok=True)
    if done:
        print(done)
    return True


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
    park program, then each pod's park action; takes the port itself, so a
    caller can park a declaration the manifest does not carry."""
    for stopped in viewers.stop_viewers(topology, selected):
        print(f"viewer for link {stopped} stopped")
    if len(selected) == len(topology.links) and topology.links:
        ok = _park(_pack_for(topology), topology)
        if ok:
            port_state.mark_parked(topology, selected)
        print("port parked (sensors in standby)"
              if ok else "park HAD ERRORS")
        return 0 if ok else 1
    return 0


def _park(pack, topology: Topology) -> bool:
    """The pack's park program on the whole port, then each pod's park
    action: the host's program stood the sensors by, and each pod parks its
    own head too, so the unit's last run is a park."""
    cfg = pack.flows().build_park(pack, topology)
    with port_state.BusLock():
        ok = cam_run._execute(cfg.to_dict(), topology.i2c_bus, "down", guard=(pack, topology))
        if ok:
            cam_run.park_pods(pack, topology)
    return ok
