# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The camera side of `nxs switch`: the host and its camera buses, each
declared pod's personality, each port's boot table, then the ports
themselves, whose `on` builds the capture stack's configuration the
booted table lacks. Every step is idempotent and prints one line per
thing it changed; a boot entry that changed ends the run with `REBOOT
NEEDED` (exit 3), because the ports come up on the booted table, and on
the camera buses the boot entry brings up, and so does a port whose `on`
in nxsd stopped for a reboot (the daemon's verdict)."""

from __future__ import annotations

import argparse
import os
import subprocess
import time
from typing import Callable, Dict, List, Optional

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
#: The group that owns the I²C devices on the host, and with them the store
#: and the declaration's directory.
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
    """The host's part: the state store, the declaration's directory on a
    host whose declaration is the system one, the bus rules, the capture
    daemon's quiet, the completion file, the nxsd unit. Returns what
    changed."""
    from nxs import suite
    from nxs.setup_cli import COMPLETION_TARGET, completion_text

    host = host_layer.current()
    done: List[str] = []
    would = "would " if dry_run else ""
    if not _writable(STATE_DIR):
        if not dry_run:
            make_group_dir(STATE_DIR)
            make_group_dir(os.path.join(STATE_DIR, "cam"))
        done.append(f"{would}create the state store {STATE_DIR}" if dry_run
                    else f"state store {STATE_DIR}")
    declaration_dir = os.path.dirname(suite._SYSTEM_CONFIG)
    if host.keeps_system_declaration() and not _writable(declaration_dir):
        if not dry_run:
            make_group_dir(declaration_dir)
        done.append(f"{would}create the declaration directory {declaration_dir}" if dry_run
                    else f"declaration directory {declaration_dir}")
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


def make_group_dir(path: str) -> None:
    """Create `path` under root for the I²C group: mode 2775, so the group
    writes it and what lands in it keeps the group."""
    group = ["-g", STATE_GROUP] if _group_exists(STATE_GROUP) else []
    root.run(["install", "-d", "-m", "2775", *group, path])


def _writable(path: str) -> bool:
    return os.path.isdir(path) and os.access(path, os.W_OK)


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
    since the pod steps reach a unit only through its port. 0; 3 when the
    restarted daemon's `on` of a port stops for a reboot; 1 with the fact
    when the daemon does not come back or refuses a port."""
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
    since = time.time()
    root.run(["systemctl", "restart", NXSD_UNIT])
    log(f"host: nxsd restarted ({was} -> {mine})")
    deadline = time.monotonic() + NXSD_CONVERGE_TIMEOUT_S
    while running_build() != mine:
        if time.monotonic() >= deadline:
            log(f"host: the restarted nxsd did not construct its ports in "
                f"{NXSD_CONVERGE_TIMEOUT_S:g} s\n  - journalctl -u nxsd")
            return 1
        time.sleep(NXSD_POLL_S)
    return _wait_for_nxsd(_owned_ports(cfg), log, since)


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
    """`nxs <port> on` as `switch` runs it inside its own report."""
    return argparse.Namespace(topology=None, port=name, links=[], mode=None, fps=None,
                              sensor=None, dry_run=False, cam_cmd="on", inner=True)


def _owned_ports(cfg) -> List[str]:
    from nxs.daemon import _owned

    return [name for name in sorted(cfg.ports) if _owned(cfg.ports[name])]


def laws_step(cfg) -> None:
    """The ports this tool owns judged whole by the laws `nxs status` runs,
    before a step brings a pod, the boot table or a port to them: a finding
    ends the run with its lines. The findings that hold the declaration
    against the booted tree are left out, since the boot table step writes
    that tree; a unit's own findings fail that unit alone, in its step, and
    a port the tool does not own (a kernel-driven hub's) is no step's here,
    so its findings stop nothing: `nxs status` names them."""
    from nxs.check import check_config

    owned = set(_owned_ports(cfg))
    refused = [finding for finding in check_config(cfg, booted=False)
               if _port_of(finding) in owned]
    if refused:
        raise StepFailed("\n".join(refused))


def _port_of(finding) -> Optional[str]:
    """The port a finding names (`ports.cam0.sync.fps` names cam0), else None."""
    head, _, rest = finding.where.partition(".")
    return rest.split(".", 1)[0] if head == "ports" and rest else None


def _refusal(name: str, exc: InfeasibleConfig) -> str:
    """A refusal on a port as the step's error line: the fact, named with
    the port where it is not, then its alternatives."""
    from nxs.cam.select import _named

    return "\n".join([_named(name, exc.reason), *(f"  - {alt}" for alt in exc.alternatives)])


def pods_step(cfg, dry_run: bool) -> List[str]:
    """Every declared pod holds its link's declared personality after this.
    A pod silent at its alias sits behind a hub that lost the aliases `on`
    maps (a power cycle) or never had them, whatever the port's record says:
    its port comes up again, which maps them. A pod alone behind its hub is
    looked for where it straps and brought to the declaration there; two
    pods strap one address and answer there as one through every runtime
    window, so theirs are left to the port's walk, which maps each alias
    through its own link before it reaches the pod."""
    from nxs.cam import pods, port_state
    from nxs.cam import run as cam_run
    from nxs.cam.select import _hub_for, select_port_links
    from nxs.client import DeviceRefused
    from nxs.suite.scan import scan_bus_units
    from nxs.transports import open_client

    lines: List[str] = []
    for name in _owned_ports(cfg):
        port = cfg.ports[name]
        topology, links = select_port_links(_port_args(name), require_port=True)
        hub = _hub_for(topology)
        flows = hub.flows()
        # A pod alone holds a click personality: the unit steps converge it.
        pod_links = [l for l in links if l.nxs_units and l.has_camera]
        if not pod_links:
            continue
        names = {l.name: (l.unit.name if l.unit else None) for l in port.links}
        # Two pods behind one hub strap one address: only their aliases tell them apart.
        shared = not topology.is_direct and sum(1 for l in links if l.nxs_units) > 1
        try:
            lock = port_state.BusLock()
            with port_state.held_wait():
                lock.__enter__()
        except port_state.BusHeld as exc:
            raise StepFailed(_bus_held(name)) from exc
        with lock:
            i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
            try:
                i2c.open()
                for link in pod_links:
                    unit = link.nxs_units[0]
                    alias, strapped = int(unit.alias_addr), int(unit.target_addr)
                    who = names.get(link.name) or f"@{alias:#04x}"
                    candidates = (alias,) if shared else tuple(dict.fromkeys((alias, strapped)))
                    hit = _pod_at(flows, hub, i2c, topology, link, candidates)
                    if hit is None and shared:
                        lines.append(f"{name}/{link.name}: pod {who} does not answer at its alias "
                                     f"{alias:#04x}; the port comes up to map it")
                        continue
                    if hit is None:
                        where = " or at ".join(f"{a:#04x}" for a in candidates)
                        raise StepFailed(f"{name}/{link.name}: no pod answers at {where} "
                                         f"({names.get(link.name) or 'the pod'})")
                    client = open_client(hit.link.transport, **hit.link.client_kwargs())
                    try:
                        line = pods.converge(client, hub, topology, link,
                                             unit_name=names.get(link.name), dry_run=dry_run)
                    except DeviceRefused as exc:
                        # The pod answered: its refusal is the line, never a silent hub.
                        raise StepFailed(f"{name}/{link.name}: pod {who}: {exc}") from exc
                    finally:
                        client.close()
                    if line:
                        lines.append(line)
                    if (found := int(hit.link.address)) != alias:
                        lines.append(f"{name}/{link.name}: pod {who} answers at {found:#04x}, not at "
                                     f"its alias {alias:#04x}; the port comes up to map it")
                if not topology.is_direct:
                    flows.close_windows(hub, i2c, topology)
            except StepFailed:
                raise
            except InfeasibleConfig as exc:
                raise StepFailed(_refusal(name, exc)) from exc
            except (pods.PodSilent, pods.PodRefused) as exc:
                raise StepFailed(str(exc)) from exc
            except (OSError, RuntimeError) as exc:
                raise StepFailed(f"{name}: the hub does not answer at "
                                 f"{topology.des_addr:#04x} ({exc})") from exc
            finally:
                i2c.close()
    return lines


def _pod_at(flows, hub, i2c, topology, link, candidates):
    """The pod of `link` where it answers among `candidates`, read through
    the link's window behind a hub; None when none answers."""
    from nxs.suite.scan import scan_bus_units

    if not topology.is_direct:
        flows.open_window(hub, i2c, topology, link)
    return next(iter(scan_bus_units(topology.i2c_bus, candidates)), None)


def strapped_units(cfg) -> Dict[str, List[str]]:
    """The declared units whose pod sits where it straps, not at its alias,
    on an owned port that is not up: the unit steps skip them, with the
    lines `nxs <port> status` prints for the pod. A pod at its alias is
    read whatever its port's verdict, since the alias phase ran."""
    from nxs.cam import port_state
    from nxs.cam import run as cam_run
    from nxs.cam.select import _hub_for, select_port_links

    skipped: Dict[str, List[str]] = {}
    for name in _owned_ports(cfg):
        if _port_is_up(name):
            continue
        port = cfg.ports[name]
        topology, links = select_port_links(_port_args(name), require_port=True)
        hub = _hub_for(topology)
        flows = hub.flows()
        pod_links = [l for l in links if l.nxs_units and l.has_camera]
        names = {l.name: (l.unit.name if l.unit else None) for l in port.links}
        if not any(names.get(l.name) for l in pod_links):
            continue
        try:
            lock = port_state.BusLock()
            with port_state.held_wait():
                lock.__enter__()
        except port_state.BusHeld:
            continue        # the unit steps meet the held bus themselves
        with lock:
            i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
            try:
                i2c.open()
                for link in pod_links:
                    who = names.get(link.name)
                    unit = link.nxs_units[0]
                    alias, strapped = int(unit.alias_addr), int(unit.target_addr)
                    if who is None or alias == strapped:
                        continue
                    hit = _pod_at(flows, hub, i2c, topology, link, (alias, strapped))
                    if hit is not None and int(hit.link.address) != alias:
                        skipped[who] = [f"{name}/{link.name}: pod {who} does not answer at its alias "
                                        f"{alias:#04x}; a pod answers at {int(hit.link.address):#04x}",
                                        "  - nxs switch"]
                if not topology.is_direct:
                    flows.close_windows(hub, i2c, topology)
            except (OSError, RuntimeError):
                pass        # a hub that does not answer: the unit steps say so per unit
            finally:
                i2c.close()
    return skipped


def _planned(name: str):
    """A declared port as its steps see it: the hub, the topology under the
    declaration, its camera links and the mode each runs by the laws."""
    from nxs.cam.identity import _declared_camera
    from nxs.cam.run import _resolve_modes
    from nxs.cam.select import _declare, _hub_for, select_port_links

    args = _port_args(name)
    topology, links = select_port_links(args, require_port=True)
    hub = _hub_for(topology)
    topology, links, modes = _declare(hub, topology, links, args)
    modes = _declared_camera(hub, topology, links, modes, args)
    cameras = [l for l in links if l.has_camera]
    planned = _resolve_modes(hub.flows(), hub, cameras, modes, topology) if cameras else {}
    return hub, topology, cameras, planned


def declared_sync_step(cfg, dry_run: bool, log: Log, again=()) -> None:
    """A declared frame sync at a rate the running port does not run yet is
    counted on the port before the boot table: the links deliver it and run
    on as before until the reboot, or the run ends on the refusal, and no
    table is written. A port that is not up, one named in ``again``, one
    whose modes or lanes change too, is left to the table and the bring-up.
    A dry run counts nothing: it names the count, and the lines after it
    assume the links deliver the rate."""
    from nxs.cam import port_state
    from nxs.cam.verbs.set import SyncNotRestored, apply_sync
    from nxs.host.cli import BootTableRefused, reboot_rule

    host = host_layer.current()
    for name in _owned_ports(cfg):
        try:
            hub, topology, cameras, planned = _planned(name)
        except InfeasibleConfig as exc:
            raise StepFailed(_refusal(name, exc)) from exc
        if name in again or not cameras or topology.sync.source != "fsync":
            continue
        if host.lane_mismatch(topology.i2c_bus, topology.csi_lanes):
            continue
        # The count runs under the booted table: a declaration its rows do
        # not admit (a mode, another exposure ceiling) takes the table's path.
        try:
            gap = reboot_rule(host, hub, topology, cameras, planned, install_overlays=False,
                              vcs={l.name: int(l.csi_vc) for l in cameras}, declared=sorted(cfg.ports),
                              report=lambda line: None)
        except BootTableRefused:
            gap = "refused"
        if gap:
            continue
        fps = float(topology.synced_fps)
        running = port_state.port_sync(topology) or {}
        if running.get("source") == "fsync" and running.get("fps") == fps and port_state.verified(topology):
            continue
        # The running port has to be the declared one, sensor included: the
        # count's program is the declared sensor's, as `_port_is_up` reads it.
        recorded = port_state.port_record(topology).get("sensors") or {}
        if any(port_state.link_state(topology, l) != port_state.STATE_UP
               or recorded.get(l.name) != l.sensor_compatible
               or port_state.port_mode(topology, l) != planned.get(l.name) for l in cameras):
            continue
        if dry_run:
            log(f"{name}: would count the declared frame sync at {fps:g} fps on the running "
                f"port, and the lines below assume the links deliver it")
            continue
        log(f"{name}: counting the declared frame sync at {fps:g} fps on the running port")
        try:
            with port_state.held_wait():
                ok = apply_sync(port_state.with_found_lines(topology), hub, hub.flows(), "fsync", fps,
                                keep=False) == 0
        except port_state.BusHeld as exc:
            raise StepFailed(_bus_held(name)) from exc
        except InfeasibleConfig as exc:
            raise StepFailed(_refusal(name, exc)) from exc
        except SyncNotRestored as exc:
            raise StepFailed(str(exc)) from exc
        except (OSError, RuntimeError) as exc:
            raise StepFailed(f"{name}: the hub does not answer at {topology.des_addr:#04x} ({exc})") from exc
        if not ok:
            raise StepFailed(f"{name}: the declared frame sync did not start\n  - nxs {name} status")


def boot_table_step(cfg, dry_run: bool, log: Log, fdt: Optional[str] = None) -> bool:
    """Every owned port's boot table carries its declared modes after this,
    `fdt` naming the base device tree the boot entry takes, and the boot
    entry names no overlay of a port the manifest does not declare (a line
    for each one dropped). Returns True when a table was (or would be)
    installed: a reboot is needed before the ports can come up."""
    from nxs.host.cli import BootTableRefused, reboot_rule

    host = host_layer.current()
    reboot = False
    for name in _owned_ports(cfg):
        reports: List[str] = []
        try:
            hub, topology, cameras, planned = _planned(name)
            if not cameras:
                continue        # a pod alone streams no video: no boot table
            vcs = {l.name: int(l.csi_vc) for l in cameras}
            text = reboot_rule(host, hub, topology, cameras, planned,
                               install_overlays=not dry_run, vcs=vcs, declared=sorted(cfg.ports),
                               fdt=fdt, report=reports.append)
        except InfeasibleConfig as exc:
            raise StepFailed(_refusal(name, exc)) from exc
        except BootTableRefused as exc:
            raise StepFailed(f"{name}: {exc}") from exc
        if text:
            log(f"{name}: boot table {'would be ' if dry_run else ''}installed")
            reboot = True
        for line in reports:
            log(f"{name}: {line}")
    return reboot


def capture_stack_step(cfg, log: Log) -> None:
    """The capture stack's configuration for the booted table is the port's
    to build: its `on` makes it once the bring-up walk has the heads
    streaming, so a port without one comes up in `ports_step`. This step
    says so, or that another run is building it."""
    from nxs.cam.select import select_port_links
    from nxs.host.cli import capture_stack_state, preparing_marker

    host = host_layer.current()
    for name in _owned_ports(cfg):
        topology, _links = select_port_links(_port_args(name), require_port=True)
        if not topology.camera_links:
            continue        # a pod alone streams no video: no tuning
        if preparing_marker(name).is_file():
            log(f"{name}: preparing the capture stack (nxsd)")
        elif capture_stack_state(host, name, topology) == "missing":
            log(f"{name}: the capture stack's configuration is built when the port comes up")


def _port_is_up(name: str) -> bool:
    """Whether the port runs its declaration: the record carries the
    declaration the manifest makes now (`declared`, one digest of every
    key that shapes the port), every declared link is recorded up with
    the declared sensor and mode, and the capture stack holds a
    configuration for the booted table. A converged port is left as it
    runs; a port whose declaration moved in any key comes up again, and
    so does a port whose record names no declaration or whose capture
    stack has no configuration (its `on` builds it after the walk)."""
    from nxs.cam import port_state
    from nxs.cam.select import select_port_links
    from nxs.host.cli import capture_stack_state

    topology, links = select_port_links(_port_args(name), require_port=True)
    if not links:
        return False
    record = port_state.port_record(topology)
    if record.get("declared") != topology.declared:
        return False
    if topology.camera_links and capture_stack_state(host_layer.current(), name, topology) == "missing":
        return False
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


#: `ports_step`'s status for a port whose bring-up refused: its verdict
#: stands, and the unit steps follow it, since the pods answer at their
#: aliases once the alias phase ran.
PORT_REFUSED = 4


def ports_step(cfg, dry_run: bool, log: Log, again=(), daemon: bool = True) -> int:
    """The owned ports come up: through the daemon when it runs (a reload),
    else here; a port up under its declaration is left as it runs, unless
    it is named in ``again`` (a pod behind it took a new build, and the
    port runs the old program until it comes up again, or its hub lost the
    aliases the bring-up maps). Returns 0, 3 when nxsd's `on` of a port
    stopped for a reboot, PORT_REFUSED when a port's bring-up refused, or
    the first port's other failure."""
    owned = [name for name in _owned_ports(cfg) if name in again or not _port_is_up(name)]
    return _ports_up(owned, again, dry_run, log, daemon)


def _ports_up(owned: List[str], again, dry_run: bool, log: Log, daemon: bool) -> int:
    """Bring the ports named up, through the daemon when it runs, else here.
    `ports_step`'s status."""
    from nxs.cam import cli as cam_cli

    if not owned:
        return 0
    if daemon and nxsd_active():
        since = time.time()
        if not dry_run:
            _record_down(owned, again)
            root.run(["systemctl", "kill", "-s", "HUP", NXSD_UNIT])
        log(f"{', '.join(owned)}: nxsd {'would reconverge' if dry_run else 'reconverging'}")
        return 0 if dry_run else _wait_for_nxsd(owned, log, since)
    reboot = refused = False
    for name in owned:
        if dry_run:
            log(f"{name}: would come up (nxs {name} on)")
            continue
        try:
            code = cam_cli.cmd_up(_port_args(name))
        except InfeasibleConfig as exc:
            log(_refusal(name, exc))
            code = 1
        # A reboot one port asks for ends the run, whatever another port did.
        reboot = reboot or code == 3
        refused = refused or code not in (0, 3)
    return 3 if reboot else PORT_REFUSED if refused else 0


def ports_again_step(cfg, restarted: List[str], *, dry_run: bool = False, log: Log = print,
                     config_path: Optional[str] = None, exclude=()) -> int:
    """The owned ports with a camera link whose pod is one of the units
    `restarted` names come up again, and no other port is touched: a pod
    that restarted under the unit steps (a firmware update) has run no
    camera program since, while the port's record says up. A port in
    `exclude` (one whose bring-up refused) keeps its verdict. A bring-up
    the daemon started on its own over the restart is waited out first.
    Returns `ports_step`'s status, 0 with no such port."""
    again = [name for name in _owned_ports(cfg)
             if name not in exclude
             and any(l.camera and l.unit and l.unit.name in restarted
                     for l in cfg.ports[name].links)]
    if not again:
        return 0
    daemon = daemon_reads(config_path)
    try:
        if daemon and not dry_run and (rc := bring_up_wait_step(cfg, log)) != 0:
            return rc
        return _ports_up(again, again, dry_run, log, daemon)
    except root.RootRefused as exc:
        log(str(exc))
        return 1


def _bus_held(name: str) -> str:
    """The line for a port's bus another run held past the wait: `nxsd`
    holds it for a bring-up it runs on its own, after a hub power cycle."""
    from nxs.cam import port_state

    held = port_state.held_text("nxsd brings a port up after boot and after a hub power cycle")
    return f"{name}: {held}\n  - nxs {name} status\n  - nxs switch"


def bring_up_wait_step(cfg, log: Log) -> int:
    """Wait out a bring-up the daemon runs on its own: after a hub power
    cycle its watch brings the port up again, and the port's verdict reads
    `bringing up` from the start of that `on` to its end, which holds the
    bus for the walk. The steps that read the pods follow the verdict.
    Returns 0, or the verdict's status as `_wait_for_nxsd` ends on it."""
    from nxs.daemon import BRINGING_UP, port_verdict

    if not nxsd_active():
        return 0
    pending, since = [], None
    for name in _owned_ports(cfg):
        verdict = port_verdict(name)
        if verdict is not None and verdict["verdict"] == BRINGING_UP:
            pending.append(name)
            since = verdict["at"] if since is None else min(since, verdict["at"])
    if not pending:
        return 0
    log(f"{', '.join(pending)}: nxsd is bringing the port up; waiting for it")
    return _wait_for_nxsd(pending, log, since)


def _record_down(owned: List[str], again) -> None:
    """Record every link neither up nor parked on the ports among `owned` a
    reload would leave as they are: one named in `again`, whose record says
    it runs its declaration, and one `off` parked, the operator's to the
    daemon. The reload brings such a port up, and the wait reads its record."""
    from nxs.cam import port_state
    from nxs.cam.select import select_port_links

    for name in owned:
        topology, links = select_port_links(_port_args(name), require_port=True)
        if name in again or port_state.STATE_PARKED in port_state.recorded_states(topology).values():
            port_state.mark_unknown(topology, links)


def _wait_for_nxsd(owned: List[str], log: Log, since: float) -> int:
    """Wait until the daemon's `on` of every owned port ends up: the units
    behind a port answer only through the port it is rebuilding, and the
    unit steps follow. The port's record says up when the walk ends, before
    its `on` does (the video lock, a synced pair's pod runs, the capture
    stack's build and the delivery check follow it), and a unit read then
    meets a pod bus a camera run holds: the wait ends on the daemon's
    verdict, recorded after `since`, with the record up. A port whose `on`
    stopped short ends the wait with the daemon's lines: `REBOOT NEEDED`
    and 3, or its refusal and 1. 1 with the fact when the deadline
    passes."""
    from nxs.daemon import BRINGING_UP, port_verdict

    def done(name: str) -> bool:
        verdict = port_verdict(name)
        return (verdict is not None and verdict["at"] >= since and verdict["verdict"] == "up"
                and _port_is_up(name))

    deadline = time.monotonic() + NXSD_CONVERGE_TIMEOUT_S
    pending = list(owned)
    reboot = refused = False
    while True:
        pending = [name for name in pending if not done(name)]
        # A verdict older than the restart or the reload is another run's. A
        # port that stopped is settled, and the others are waited out: the
        # daemon reconverges its ports one after another, and the unit steps
        # must not meet a bring-up still under way.
        for name in list(pending):
            verdict = port_verdict(name)
            if (verdict is None or verdict["at"] < since
                    or verdict["verdict"] in ("up", BRINGING_UP)):
                continue
            for line in verdict["lines"]:
                log(line)
            reboot = reboot or verdict["verdict"] == "reboot needed"
            refused = refused or verdict["verdict"] != "reboot needed"
            pending.remove(name)
        if not pending:
            if reboot:
                log("REBOOT NEEDED")
                return 3
            return PORT_REFUSED if refused else 0
        if time.monotonic() >= deadline:
            log(f"{', '.join(pending)}: nxsd did not bring the port up in "
                f"{NXSD_CONVERGE_TIMEOUT_S:g} s")
            log("  - journalctl -u nxsd")
            return 1
        time.sleep(NXSD_POLL_S)


def kernel_step(cfg, host) -> None:
    """The host's camera kernel package: the camera buses boot from its
    overlay, declaration or not, and a declared camera port needs its
    modules for the booted kernel. A rig of pods alone needs the overlay
    only; `cfg` is None before anything is declared."""
    missing = host.camera_bus_package_missing()
    if missing is None and cfg is not None and any(
            getattr(link, "camera", None) for port in cfg.ports.values()
            for link in (getattr(port, "links", None) or [])):
        missing = host.kernel_package_missing()
    if missing:
        raise StepFailed(f"{missing}\n  - {host.kernel_package_next()}")


def camera_bus_step(host, dry_run: bool, fdt: Optional[str], log: Log) -> bool:
    """A host whose booted tree has no camera bus gets the boot entry that
    boots them: every port rides one, and a pod alone answers only there.
    True when the entry was (or would be) written, or waits for its reboot;
    an entry that booted and brought up no bus is the host's refusal, since
    another reboot would boot the same."""
    from nxs.host.cli import boot_write_refusal

    if not host.camera_bus_missing():
        return False
    try:
        lines = host.install_camera_buses(fdt=fdt, dry_run=dry_run)
    except PermissionError as exc:
        raise StepFailed(f"host: {boot_write_refusal(exc)}") from exc
    except RuntimeError as exc:
        raise StepFailed(f"host: {exc}") from exc
    for line in lines:
        log(f"host: {line}")
    return True


def daemon_reads(config_path: Optional[str]) -> bool:
    """Whether nxsd reads the declaration at `config_path`: it runs as
    root, so it reads the system manifest (None: the caller's own)."""
    from nxs import suite

    return config_path is None or os.path.abspath(config_path) == os.path.abspath(suite._SYSTEM_CONFIG)


def camera_steps(cfg, *, dry_run: bool = False, fdt: Optional[str] = None,
                 log: Log = print, config_path: Optional[str] = None) -> int:
    """Run the camera steps in order; 0 when every port is up, 3 when a
    reboot is needed first, PORT_REFUSED when a port's bring-up refused
    (its verdict printed; the unit steps may follow), 1 when a step failed
    or the laws refuse the declaration's ports before any step brings them
    up (its lines printed).
    Without a declaration (`cfg` None) the host's steps run alone, and 0
    says the host is ready for one. `fdt` names the base device tree a
    boot entry the steps write takes. A declaration nxsd does not read
    (`config_path` outside the system manifest) comes up here, never
    through the daemon."""
    try:
        host = host_layer.current()
        kernel_step(cfg, host)
        for line in host_step(dry_run):
            log(f"host: {line}")
        if camera_bus_step(host, dry_run, fdt, log):
            log("REBOOT NEEDED")
            return 3
        if cfg is None:
            return 0
        laws_step(cfg)
        daemon = daemon_reads(config_path)
        rc = daemon_step(cfg, dry_run, log) if daemon else 0
        if rc == 0 and daemon:
            rc = bring_up_wait_step(cfg, log)
        if rc != 0:
            return rc
        again = set()
        for line in pods_step(cfg, dry_run):
            log(line)
            again.add(line.split("/", 1)[0])
        declared_sync_step(cfg, dry_run, log, again)
        if boot_table_step(cfg, dry_run, log, fdt):
            log("REBOOT NEEDED")
            return 3
        capture_stack_step(cfg, log)
        return ports_step(cfg, dry_run, log, again=again, daemon=daemon)
    except (StepFailed, root.RootRefused) as exc:
        log(str(exc))
        return 1

