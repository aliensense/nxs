"""`on` and `off`: bring a port's links up on its graph image, verify the sensors, build the capture
stack's configuration the booted table lacks, verify the delivery, park the port."""

from __future__ import annotations

import argparse
import dataclasses
import errno
import functools
from typing import Dict, List, Optional

from nxs import term

from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs import host as host_layer
from nxs.cam import compose, graph, port_state
from nxs.cam.port_state import save_port
from nxs.cam import viewers
from nxs.cam import run as cam_run
from nxs.cam import identity as cam_identity
from nxs.cam.identity import _declared_camera, _require_nxs_hub, detect_sensor, sensor_identity_line
from nxs.cam.run import _compose_from, _mode_arg, _port_links, _rates_arg, _resolve_modes
from nxs.cam.select import (_declare, _hub_for, _port_name, _refuse, require_no_daemon_run,
                            select_port_links)
from nxs.cam.verbs import verify
from nxs.cam.verbs.sync import (_fsync_plan, _hints_by_link, _pair_ae, _port_viewer_hint, _record_sync,
                                declared_exposure_fact, plan_text)


def cmd_up(args: argparse.Namespace) -> int:
    topology, links = select_port_links(args, require_port=True)
    if not getattr(args, "inner", False):
        # The daemon's own `on` runs inner; an operator's waits for it.
        require_no_daemon_run(_port_name(topology), "on")
    _require_nxs_hub(topology, "on")
    hub = _hub_for(topology)
    topology, links, modes = _declare(hub, topology, links, args)
    modes = _declared_camera(hub, topology, links, modes, args)
    declared_ports = _manifest_ports(topology)
    topology = _as_recorded(topology, links, args)
    flows = hub.flows()
    host = host_layer.current()
    port = _port_name(topology)
    cameras = [l for l in links if l.has_camera]
    pods = [l for l in links if not l.has_camera]
    if not links:
        term.refusal(f"{port}: no link to bring up",
                     f"declare a camera node wired to {topology.hub_node or 'the hub'}.links.<link>")
        return 1
    if pods and cameras:
        # One hub brings a pod-only link and a camera link up only as a
        # pair, and a pair is two camera programs.
        term.refusal(f"{port}: link {pods[0].name} carries a pod and no camera, link "
                     f"{cameras[0].name} a camera; one hub brings them up together "
                     f"only as a pair",
                     f"nxs {port} {cameras[0].name} on",
                     f"declare a camera node wired to {topology.hub_node or 'the hub'}.links."
                     f"{pods[0].name}")
        return 1
    # The overlay fixed the capture side's lane count at boot; a program for
    # a different count gives no video and no error. Refuse it here.
    mismatch = host.lane_mismatch(topology.i2c_bus, topology.csi_lanes)
    if mismatch:
        term.err(mismatch)
        return 2
    # The hub's laws judge the bring-up as they plan it, and the plan states
    # the CSI contract; the walk runs the port's graph image.
    _plan, csi = _compose_from(hub, topology, links, args, modes)
    # The booted tree must carry every selected link's mode and a capture
    # node for the channel it rides; a gap regenerates the port's overlay
    # from the hub and asks for the reboot.
    planned = _resolve_modes(flows, hub, cameras, modes, topology)
    # A mode the port's table boots no row for is refused here, before the
    # overlay is written: the table carries one pixel format, chosen by the
    # declaration, so regenerating it carries the same rows.
    from nxs.host import capture_table as tables
    partial = len(links) < len(topology.links)
    chosen = f" {' '.join(l.name for l in links)}" if partial else ""
    for link in cameras:
        if link.name in planned:
            tables.refuse_without_row(hub, topology, link, hub.descriptor(link.sensor_compatible),
                                      planned[link.name], port, f"nxs {port}{chosen} on")
    vcs = {link.name: int(vc.vc) for link, vc in zip(cameras, csi.virtual_channels)
           if vc is not None}
    from nxs.host.cli import BootTableRefused, capture_stack_state, reboot_rule
    try:
        reboot = reboot_rule(host, hub, topology, cameras, planned,
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
    running = _resolve_modes(flows, hub, cameras, modes, topology)
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
            f"{topology.hub_param_path('exposure')}: "
            + declared_exposure_fact(topology, declared_fsync),
            alternatives=["drop the key"])
    if locked:
        _gain_shape(hub, flows, _port_links(topology, links), declared_fsync)
    plan = None
    fps = topology.synced_fps
    if declared_fsync:
        # The trigger overlay is composed and its rate judged before the first
        # bus write, at the one rate the bring-up asks, else the declaration's.
        asked = _rates_arg(args, links) or {}
        fps = (float(next(iter(asked.values()))) if len(set(asked.values())) == 1
               else topology.synced_fps)
        flows.build_fsync(hub, topology, fps=fps, method="manual", modes=running)
        plan = _fsync_plan(flows, hub, topology, fps, running)
    if args.dry_run:
        # The plan's summary: what the port would be, no write stream.
        for line in _port_lines(hub, links, running, csi):
            print(line)
        return 0
    names = "+".join(l.name for l in links)
    walked = _walk_up(hub, topology, links, running, csi, fps if declared_fsync else None,
                      float(topology.camera_gain_db) if locked else None)
    if isinstance(walked, int):
        return walked
    record = functools.partial(_record_up, hub, flows, topology, links, csi, running, args)
    record()
    port_state.set_sync("free_run", topology=topology,
                        ae=_pair_ae(hub, flows, topology, "free_run"))
    for line in _port_lines(hub, links, running, csi):
        term.info(line)
    print(f"up {names}: ok")
    if cameras and walked.walks.vlock and not declared_fsync:
        # The up walk's last phase waited for every pipe's video lock, and a
        # head that held stale state was recovered once on the way.
        term.info("video locked")
    if cameras and not _sensors_verified(hub, flows, topology, cameras):
        port_state.mark_unknown(topology, links)
        return 1
    if topology.sync.source == "fsync" and partial:
        term.warn(f"declared sync: fsync is {_port_name(topology)}'s two-camera "
                  f"port, {names} runs free\n  - nxs {_port_name(topology)} on "
                  f"(the trigger needs every link)")
    if declared_fsync:
        # The bring-up's policy ran the sync walk, the overlay its last step.
        _record_sync(hub, flows, topology, "fsync", fps, plan=plan)
        print(f"declared sync: {plan_text(plan) if plan else f'fsync {fps:g} fps'}")
    if locked:
        # The bring-up's policy ran the gain walk after the sync, once.
        print(f"declared gain: {float(topology.camera_gain_db):.1f} dB on "
              + " and ".join(l.name for l in cameras))
    # The build's sessions open each head through the kernel driver at the host address the
    # walk's translation gives it: the build follows the walk, the count follows the build.
    if cameras and state == "missing" and not _build_capture_stack(
            host, topology, links, f"nxs {port}{chosen} on"):
        return 1
    if cameras:
        # The port is up once every camera link delivers the rate it runs; a
        # hub pair finds the line its line memory carries on the way.
        retry = f"nxs {port}{chosen} on --fps {{fps}}"
        try:
            if _finds_its_line(flows, cameras):
                verify.search(hub, topology, cameras, retry, record)
            else:
                verify.check(hub, topology, cameras, retry=retry)
        except InfeasibleConfig as exc:
            port_state.mark_unknown(topology, links)
            term.refusal(exc.reason, *exc.alternatives)
            return 2
    if not getattr(args, "inner", False):
        # The operator's own `on` names the next verb; `switch` and nxsd go on with theirs.
        sel = " ".join(l.name for l in links)
        print(f"next: nxs {port} {sel} stream" if cameras else "next: nxs status")
    return 0


def _walk_up(hub, topology: Topology, links: List[LinkSpec], running: Dict[str, str], csi,
             fsync: Optional[float], gain_db: Optional[float]):
    """Compose the port's graph image and bring the port up on it by the
    library's policy: the composition when the port came up, else the exit
    code after the refusal. The hub must answer as the declared silicon
    first; the walk prints each pod's turn and each decision it took."""
    graph.library()
    composition = compose.compose(hub, topology, links, running,
                                  dict(getattr(csi, "rates", None) or {}), fsync=fsync,
                                  up_sync=fsync is not None, gain_db=gain_db)
    for line in _head_lines(composition, running):
        term.info(line)
    owner = graph.Owner(topology, composition)
    with port_state.BusLock():
        cam_identity.verify_hub(hub, topology)
        report = graph.bring_up(topology, composition, owner)
    if report.ok:
        return composition
    return _not_up(topology, links, composition, report, csi)


def _head_lines(composition, running: Dict[str, str]) -> List[str]:
    """What runs each camera link's personality, the mode and the trigger: the
    pod, the host for a bare head; a smart camera serves its rows itself."""
    lines = []
    for row in composition.links:
        head = composition.heads.get(row.name)
        if row.present and head is not None:
            who = ("the pod runs" if row.pod else "the camera serves" if row.pod_map
                   else "the host runs")
            lines.append(f"link {row.name}: {who} {head.name}, mode {running[row.name]}, "
                         f"trigger freerun")
    return lines


def _not_up(topology: Topology, links: List[LinkSpec], composition, report, csi) -> int:
    """A bring-up that did not end up: the links recorded unknown, so the next
    `on`, the watch and a reload construct them again, and the refusal in the
    walk's words, the port's status under it, the last lines `nxsd` reads
    its verdict from. The library aborted a pod's live run and parked the
    whole port on its way out of a stopped up walk."""
    port = _port_name(topology)
    port_state.mark_unknown(topology, links)
    if report.walk == graph.Walk.CONSTRUCT and report.rc == -errno.ENOLINK and report.link:
        answering = [l.name for l in links if l.name != report.link]
        term.refusal(f"{port}/{report.link}: nothing answers on link {report.link} (no lock at 6 "
                     f"or 12 Gbps)",
                     f"check the coax and the head's power on link {report.link}",
                     *([f"nxs {port} {' '.join(answering)} on"] if answering else []))
        return 1
    if report.walk == graph.Walk.UP and graph.vlock_failed(composition, report):
        pipes = dict(getattr(csi, "pipes", None) or {})
        dead = [name for name, pipe in sorted(pipes.items())
                if report.run.reg in _pipe_regs(composition, pipe)] or [l.name for l in links]
        term.refusal(", ".join(f"{port}/{name}" for name in dead) + ": video did not lock",
                     "power-cycle the hub", f"nxs {port} status")
        return 1
    what = {graph.Walk.SYNC: "the declared frame sync did not start",
            graph.Walk.GAIN: "the declared gain was not set"}.get(report.walk)
    if what is not None:
        term.refusal(f"{port}: {what}: {report.reason()}", f"nxs {port} status")
        return 1
    silent = _silent_pod(topology, links, report)
    term.refusal(silent or f"{port}: {report}", f"nxs {port} status")
    return 1


def _silent_pod(topology: Topology, links: List[LinkSpec], report) -> Optional[str]:
    """The line for a pod's turn that failed because no pod answers: the
    link, the addresses tried (the alias, then where the pod straps) and
    the step; None when a unit answers at the alias, so the failure was
    the unit's own."""
    import os

    from nxs.cam import pods

    from nxs.device_errors import ERRNO_ENOTSUP
    from nxs.firmware import push_fw_asset

    spec = next((l for l in links if l.name == report.link and l.nxs_units), None)
    if spec is None or report.kind not in (graph.StepKind.POD, graph.StepKind.CONVERGE):
        return None
    if report.kind == graph.StepKind.CONVERGE and report.refusal.device_errno == ERRNO_ENOTSUP:
        # The pod refused the build's upload: its firmware predates the image's format.
        unit = spec.nxs_units[0]
        return (f"{_port_name(topology)}/{spec.name}: pod @{int(unit.alias_addr):#04x}: its firmware "
                f"predates {pods.declared_name(_hub_for(topology), spec)}'s image format\n"
                f"  - nxs {pods._push_target(topology, spec, None)} push-fw {push_fw_asset()}")
    code = -report.rc if report.rc < 0 else errno.EIO
    with port_state.BusLock():
        return pods._unanswered(topology, spec, OSError(code, f"{os.strerror(code)} ({report.what})"))


def _pipe_regs(composition, pipe: str):
    """The video-lock register a pipe's lock reads: Y's first, then Z's."""
    regs = composition.walks.vlock
    index = "YZ".index(pipe) if pipe in "YZ" else -1
    return regs[index:index + 1] if 0 <= index < len(regs) else ()


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


def _gain_shape(hub, flows, topology: Topology, synced: bool) -> None:
    """The ports a declared gain (the hub's `gain`) locks, judged before
    the first bus write (ADR 0013, R-7.7): under frame sync a synced pair of one
    sensor kind, both heads taking the pulse; in free run the camera links
    together with a declared exposure, which the capture stack's loop sets
    without one. The composition holds the gain itself to each head's law.

    Raises:
        InfeasibleConfig: A port the lock does not fit, under the
            declaration's key.
    """
    where = topology.hub_param_path("gain")
    if synced:
        ae = _pair_ae(hub, flows, topology, "fsync") or {}
        if ae.get("mode") != "locked":
            raise InfeasibleConfig(f"{where}: a declared gain locks a synced pair of one sensor "
                                   f"kind, and {ae.get('reason', 'the hub states no pair')}",
                                   alternatives=["drop the key"])
    elif topology.camera_exposure_us is None:
        exposure = topology.hub_param_path("exposure")
        raise InfeasibleConfig(f"{where}: in free run a declared gain locks the camera links "
                               f"together with {exposure}, and without it the capture "
                               f"stack's loop sets the exposure",
                               alternatives=[f"{exposure}: <the exposure in us>"])


def _record_up(hub, flows, topology: Topology, links: List[LinkSpec], csi,
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
              viewer=(_port_viewer_hint(hub, flows, port, links, _mode_arg(args), rates=rates)
                      if cameras else None),
              viewers=(_hints_by_link(hub, flows, _port_links(port, links), links,
                                      running or None, rates=rates,
                                      ae=_pair_ae(hub, flows, topology, "free_run"))
                       if cameras else None),
              modes=running, rates=rates, line=line)
    return port


def _finds_its_line(flows, cameras: List[LinkSpec]) -> bool:
    """A pair whose hub reads the line-memory overflow and retimes the pair:
    its line is found on the rig; a solo runs the datasheet's."""
    return (len(cameras) == 2
            and all(hasattr(flows, hook) for hook in ("line_overflow", "build_timing")))


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
        port_state.write_store_file(marker, "")
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


def _port_lines(hub, links: List[LinkSpec], running: Dict[str, str],
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
                text += f" {mode_label(hub.descriptor(link.sensor_compatible), mode)}"
            except Exception:
                text += f" {mode}"
        rate = rates.get(link.name)
        if rate is not None:
            text += f" @ {rate:.10g} fps" if float(rate).is_integer() else f" @ {rate:.2f} fps"
        lines.append(text)
    return lines


def _sensors_verified(hub, flows, topology: Topology,
                      links: List[LinkSpec]) -> bool:
    """After the program: an identity-bearing sensor must answer as the
    declared part through its link window; declared-only parts pass.
    What answers is recorded, so the port record remembers the sensor. A
    pod's head behind a hub is its unit's: the host reads no register of it
    (R-10.2), and the walk ran the declared build on the pod."""
    links = [link for link in links if topology.is_direct or not link.nxs_units]
    if not links or not any(cam_identity.identity_facts(hub.descriptor(c)) is not None
                            for c in hub.sensors()):
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
                open_window(hub, i2c, topology, link)
                detected, detail = detect_sensor(hub, i2c, link)
                ok, text = sensor_identity_line(
                    link, hub.descriptor(link.sensor_compatible), detected,
                    detail)
                if detected:
                    found[link.name] = detected
                if not ok:
                    bad.append(f"link {link.name}: {text}")
            close = getattr(flows, "close_windows", None)
            if close is not None:
                close(hub, i2c, topology)
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
    if not getattr(args, "inner", False):
        require_no_daemon_run(_port_name(topology), "off")
    _require_nxs_hub(topology, "off")
    if topology.links and len(selected) < len(topology.links):
        # The park walk gates the port's capture output and stands every
        # sensor by: there is no one-link park.
        raise _refuse("off parks the whole port", f"nxs {_port_name(topology)} off")
    return park_port(topology, selected)


def park_port(topology: Topology, selected: List[LinkSpec]) -> int:
    """Stop the selected links' viewers and, for the whole port, park it:
    free run first, the CSI output gated, every head in standby; takes the
    port itself, so a caller can park a declaration the manifest does not
    carry."""
    for stopped in viewers.stop_viewers(topology, selected):
        print(f"viewer for link {stopped} stopped")
    if len(selected) == len(topology.links) and topology.links:
        ok = _park(_hub_for(topology), topology)
        if ok:
            port_state.mark_parked(topology, selected)
        print("port parked (sensors in standby)"
              if ok else "park HAD ERRORS")
        return 0 if ok else 1
    return 0


def _park(hub, topology: Topology) -> bool:
    """The port's park walk: behind a hub the generator off and every synced
    head back in free run, whatever the record says (a lost record leaves
    the trigger on), and the CSI output gated off; then each head's park, a
    pod's on the pod, a bare head's on the host and a smart camera's stream
    row on its camera door, so a unit's last run is a park. A park never
    stops at a silent head: a head's park is soft, and the library skips a
    pod that never came up. A port with no camera has nothing to park."""
    composition = graph.compose_port(hub, topology)
    if not composition.steps(graph.Walk.PARK):
        return True
    with port_state.BusLock():
        cam_identity.verify_hub(hub, topology)
        report = graph.walk(topology, composition, graph.Walk.PARK,
                            graph.Owner(topology, composition))
    if not report.ok:
        term.err(f"{_port_name(topology)}: {report}")
    return report.ok
