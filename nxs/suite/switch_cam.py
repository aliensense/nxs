# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The camera side of `nxs switch`: the host, each declared pod's
personality, each port's boot table, the capture stack's configuration,
then the ports themselves. Every step is idempotent and prints one line
per thing it changed; a boot table that changed ends the run with
`REBOOT NEEDED` (exit 3), because the ports come up on the booted table."""

from __future__ import annotations

import argparse
import os
import subprocess
import time
from typing import Callable, List

from nxs import host as host_layer
from nxs.cam.contracts import InfeasibleConfig
from nxs.host import root

#: The state store the daemon and the operator's shell share.
STATE_DIR = "/var/lib/aliensense"
NXSD_UNIT = "nxsd.service"
NXSD_UNIT_PATH = "/etc/systemd/system/nxsd.service"
#: How long a reloaded nxsd gets to bring the owned ports up (one port's
#: bring-up is under a minute) and how often the record is read meanwhile.
NXSD_CONVERGE_TIMEOUT_S = 120.0
NXSD_POLL_S = 2.0
#: The group that owns the I²C devices on the host, and with it the store.
STATE_GROUP = "i2c"

Log = Callable[[str], None]


class StepFailed(RuntimeError):
    """A step that did not complete; the message is the doc's error line."""


def _write(target: str, text: str) -> bool:
    """Write under root when the content differs; True when it was written."""
    if root.read_text(target) == text:
        return False
    root.write_text(target, text)
    return True


def host_step(dry_run: bool) -> List[str]:
    """The host's part: the state store, the bus rules, the capture daemon's
    quiet, the completion file, the nxsd unit. Returns what changed."""
    from nxs.setup_cli import COMPLETION_TARGET, completion_text

    host = host_layer.current()
    done: List[str] = []
    would = "would " if dry_run else ""
    if not (os.path.isdir(STATE_DIR) and os.access(STATE_DIR, os.W_OK)):
        if not dry_run:
            group = ["-g", STATE_GROUP] if _group_exists(STATE_GROUP) else []
            root.run(["install", "-d", "-m", "2775", *group, STATE_DIR])
            root.run(["install", "-d", "-m", "2775", *group, os.path.join(STATE_DIR, "cam")])
        done.append(f"{would}create the state store {STATE_DIR}" if dry_run
                    else f"state store {STATE_DIR}")
    rules = host.bus_rules()
    if rules is not None:
        text = rules.read_text()
        if root.read_text(host_layer.BUS_RULES_TARGET) != text:
            if not dry_run:
                root.write_text(host_layer.BUS_RULES_TARGET, text)
                root.run(["udevadm", "control", "--reload"])
                root.run(["udevadm", "trigger", "--subsystem-match=i2c-dev"])
            done.append(f"{would}udev rules installed")
    if not dry_run:
        changed: List[str] = []

        def write(target: str, text: str) -> bool:
            if _write(target, text):
                changed.append(target)
            return True

        host.setup(write)
        if changed:
            done.append("the capture daemon's crash reports quieted")
    completion = completion_text()
    if completion and root.read_text(COMPLETION_TARGET) != completion:
        if not dry_run:
            root.write_text(COMPLETION_TARGET, completion)
        done.append(f"{would}tab completion installed")
    done.extend(_daemon_step(host, dry_run))
    return done


def _group_exists(name: str) -> bool:
    try:
        import grp
        grp.getgrnam(name)
        return True
    except (ImportError, KeyError):
        return False


def _daemon_step(host, dry_run: bool) -> List[str]:
    from nxs.daemon import render_systemd_unit, resolve_nxsd_path

    import sys
    exec_path = resolve_nxsd_path(sys.argv[0])
    if exec_path is None:
        return []
    lines: List[str] = []
    unit = render_systemd_unit(exec_path, None, host.capture_daemon_unit())
    if root.read_text(NXSD_UNIT_PATH) != unit or not _unit_enabled():
        if dry_run:
            lines.append("would enable nxsd")
        else:
            root.write_text(NXSD_UNIT_PATH, unit)
            root.run(["systemctl", "daemon-reload"])
            root.run(["systemctl", "enable", NXSD_UNIT])
            lines.append("nxsd enabled")
    return lines


def daemon_step(cfg, dry_run: bool, log: Log) -> int:
    """A running daemon keeps the code it started with (a reload changes
    the manifest only): one of another build than this tool's restarts
    onto the installed wheel, and the step waits for the restarted daemon
    to record its build, which it does once its ports are constructed,
    since the pod steps reach a unit only through its port. 0, or 1 with
    the fact when the restarted daemon does not come back."""
    from nxs.cli import _git_version
    from nxs.daemon import running_build

    if not nxsd_active():
        return 0
    running, mine = running_build(), _git_version()
    if running == mine:
        return 0
    was = running or "an unrecorded build"
    if dry_run:
        log(f"host: would restart nxsd ({was} -> {mine})")
        return 0
    root.run(["systemctl", "restart", NXSD_UNIT])
    log(f"host: nxsd restarted ({was} -> {mine})")
    deadline = time.monotonic() + NXSD_CONVERGE_TIMEOUT_S
    while running_build() != mine:
        if time.monotonic() >= deadline:
            log(f"host: the restarted nxsd did not construct its ports in "
                f"{NXSD_CONVERGE_TIMEOUT_S:g} s\n  - journalctl -u nxsd")
            return 1
        time.sleep(NXSD_POLL_S)
    return _wait_for_nxsd(_owned_ports(cfg), log)


def _systemctl(*argv: str) -> bool:
    try:
        return subprocess.run(["systemctl", *argv], capture_output=True).returncode == 0
    except OSError:
        return False


def _unit_enabled() -> bool:
    return _systemctl("is-enabled", "--quiet", NXSD_UNIT)


def nxsd_active() -> bool:
    return _systemctl("is-active", "--quiet", NXSD_UNIT)


def _port_args(name: str) -> argparse.Namespace:
    return argparse.Namespace(topology=None, port=name, links=[], mode=None, fps=None,
                              sensor=None, dry_run=False, cam_cmd="on")


def _owned_ports(cfg) -> List[str]:
    from nxs.daemon import _owned

    return [name for name in sorted(cfg.ports) if _owned(cfg.ports[name])]


def pods_step(cfg, dry_run: bool, log: Log) -> List[str]:
    """Every declared pod holds its link's declared personality after this;
    a pod the host cannot reach yet (two pods behind one hub answer apart
    only through the aliases `on` programs) is left to `on`."""
    from nxs.cam import pods, port_state
    from nxs.cam import run as cam_run
    from nxs.cam.select import _pack_for, select_port_links
    from nxs.suite.scan import scan_bus_units
    from nxs.transports import open_client

    lines: List[str] = []
    for name in _owned_ports(cfg):
        port = cfg.ports[name]
        topology, links = select_port_links(_port_args(name), require_port=True)
        pack = _pack_for(topology)
        flows = pack.flows()
        # A pod alone holds a driver personality: the unit steps converge it.
        pod_links = [l for l in links if l.nxs_units and l.has_camera]
        if not pod_links:
            continue
        names = {l.name: (l.unit.name if l.unit else None) for l in port.links}
        with port_state.BusLock():
            i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
            try:
                i2c.open()
                for link in pod_links:
                    alias = int(link.nxs_units[0].alias_addr)
                    candidates = (alias,) if len(pod_links) > 1 else (alias, 0x30)
                    if not topology.is_direct:
                        flows.open_window(pack, i2c, topology, link)
                    hit = next(iter(scan_bus_units(topology.i2c_bus, candidates)), None)
                    if hit is None:
                        if len(pod_links) > 1 and port_state.link_state(topology, link) \
                                != port_state.STATE_UP:
                            continue        # `on` maps it, then brings it here
                        raise StepFailed(f"{name}/{link.name}: no pod answers at "
                                         f"{alias:#04x} ({names.get(link.name) or 'the pod'})")
                    client = open_client(hit.link.transport, **hit.link.client_kwargs())
                    try:
                        line = pods.converge(client, pack, topology, link,
                                             unit_name=names.get(link.name),
                                             dry_run=dry_run, log=log)
                    finally:
                        client.close()
                    if line:
                        lines.append(line)
                if not topology.is_direct:
                    flows.close_windows(pack, i2c, topology)
            except StepFailed:
                raise
            except (OSError, RuntimeError) as exc:
                raise StepFailed(f"{name}: the hub does not answer at "
                                 f"{topology.des_addr:#04x} ({exc})") from exc
            finally:
                i2c.close()
    return lines


def boot_table_step(cfg, dry_run: bool, log: Log) -> bool:
    """Every owned port's boot table carries its declared modes after this.
    Returns True when a table was (or would be) installed: a reboot is
    needed before the ports can come up."""
    from nxs.cam.identity import _declared_camera
    from nxs.cam.run import _resolve_modes
    from nxs.cam.select import _declare, _pack_for, select_port_links
    from nxs.host.cli import reboot_rule

    host = host_layer.current()
    reboot = False
    for name in _owned_ports(cfg):
        args = _port_args(name)
        topology, links = select_port_links(args, require_port=True)
        pack = _pack_for(topology)
        topology, links, modes = _declare(pack, topology, links, args)
        modes = _declared_camera(pack, topology, links, modes, args)
        cameras = [l for l in links if l.has_camera]
        if not cameras:
            continue        # a pod alone streams no video: no boot table
        planned = _resolve_modes(pack.flows(), pack, cameras, modes, topology)
        vcs = {l.name: int(l.csi_vc) for l in cameras}
        text = reboot_rule(host, pack, topology, cameras, planned,
                           install_overlays=not dry_run, vcs=vcs)
        if text:
            log(f"{name}: boot table {'would be ' if dry_run else ''}installed")
            reboot = True
    return reboot


def capture_stack_step(cfg, dry_run: bool, log: Log) -> None:
    """Every owned port's capture-stack configuration is built for the
    booted table after this; a build that fails ends the run."""
    from nxs.cam.select import select_port_links
    from nxs.host.cli import TuningRefused, ensure_tuning, preparing_marker

    host = host_layer.current()
    for name in _owned_ports(cfg):
        topology, _links = select_port_links(_port_args(name), require_port=True)
        if not topology.camera_links:
            continue        # a pod alone streams no video: no tuning
        if preparing_marker(name).is_file():
            log(f"{name}: preparing the capture stack (nxsd)")
            continue
        if dry_run:
            continue
        try:
            done = ensure_tuning(host, topology, log=log)
        except TuningRefused as exc:
            raise StepFailed(f"{name}: {exc}") from exc
        except (RuntimeError, PermissionError) as exc:
            raise StepFailed(f"{name}: the capture stack's configuration was not built "
                             f"({exc})") from exc
        if done:
            log(f"{name}: capture stack configured")


def _port_is_up(name: str) -> bool:
    """Whether every declared link of the port is recorded up with the
    declared sensor and mode: a converged port is left as it runs, a
    port whose declaration moved comes up again."""
    from nxs.cam import port_state
    from nxs.cam.select import select_port_links

    topology, links = select_port_links(_port_args(name), require_port=True)
    if not links:
        return False
    record = port_state.port_record(topology)
    sensors = record.get("sensors") or {}
    modes = record.get("modes") or {}
    for link in links:
        if port_state.link_state(topology, link) != port_state.STATE_UP:
            return False
        if sensors.get(link.name) != link.sensor_compatible:
            return False
        if link.mode and modes.get(link.name) != link.mode:
            return False
    return True


def ports_step(cfg, dry_run: bool, log: Log, again=()) -> int:
    """The owned ports come up: through the daemon when it runs (a reload),
    else here; a port whose declared links are up is left as it runs,
    unless it is named in ``again`` (a pod behind it took a new build,
    and the port runs the old program until it comes up again).
    Returns 0, or the first port's failure."""
    from nxs.cam import cli as cam_cli

    owned = [name for name in _owned_ports(cfg) if name in again or not _port_is_up(name)]
    if not owned:
        return 0
    if nxsd_active():
        if not dry_run:
            root.run(["systemctl", "kill", "-s", "HUP", NXSD_UNIT])
        log(f"{', '.join(owned)}: nxsd {'would reconverge' if dry_run else 'reconverging'}")
        return 0 if dry_run else _wait_for_nxsd(owned, log)
    rc = 0
    for name in owned:
        if dry_run:
            log(f"{name}: would come up (nxs {name} on)")
            continue
        try:
            code = cam_cli.cmd_up(_port_args(name))
        except InfeasibleConfig as exc:
            # A declaration the laws refuse: the fact and its alternatives,
            # as the verb itself prints them.
            log(f"{name}: {exc.reason}")
            for alternative in exc.alternatives:
                log(f"  - {alternative}")
            code = 1
        rc = rc or code
    return rc


def _wait_for_nxsd(owned: List[str], log: Log) -> int:
    """Wait until the daemon records every owned port up: the units behind
    a port answer only through the port it is rebuilding, and the unit
    steps follow. 1 with the fact when the deadline passes."""
    deadline = time.monotonic() + NXSD_CONVERGE_TIMEOUT_S
    pending = list(owned)
    while True:
        pending = [name for name in pending if not _port_is_up(name)]
        if not pending:
            return 0
        if time.monotonic() >= deadline:
            log(f"{', '.join(pending)}: nxsd did not bring the port up in "
                f"{NXSD_CONVERGE_TIMEOUT_S:g} s")
            log("  - journalctl -u nxsd")
            return 1
        time.sleep(NXSD_POLL_S)


def kernel_step(cfg, host) -> None:
    """A declared camera port needs the host's camera kernel package for the
    booted kernel; a rig of pods alone needs none."""
    if not any(getattr(link, "camera", None) for port in cfg.ports.values()
               for link in (getattr(port, "links", None) or [])):
        return
    missing = host.kernel_package_missing()
    if missing:
        raise StepFailed(f"{missing}\n  - {host.kernel_package_next()}")


def camera_steps(cfg, *, dry_run: bool = False, log: Log = print) -> int:
    """Run the camera steps in order; 0 when every port is up, 3 when a
    reboot is needed first, 1 when a step failed (its line printed)."""
    try:
        kernel_step(cfg, host_layer.current())
        for line in host_step(dry_run):
            log(f"host: {line}")
        if daemon_step(cfg, dry_run, log) != 0:
            return 1
        uploaded = set()
        for line in pods_step(cfg, dry_run, log):
            log(line)
            uploaded.add(line.split("/", 1)[0])
        if boot_table_step(cfg, dry_run, log):
            log("REBOOT NEEDED")
            return 3
        capture_stack_step(cfg, dry_run, log)
        return ports_step(cfg, dry_run, log, again=uploaded)
    except (StepFailed, root.RootRefused) as exc:
        log(str(exc))
        return 1

