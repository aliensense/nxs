"""`nxs switch`: converge every declared unit to the manifest. The
transports come from the manifest, which is the topology."""
import os
import sys
from typing import Dict, List

from nxs.suite import default_config_path, default_state_path, stray_declaration, use_config_path
from nxs.suite.schema import ManifestError, load_suite_config
from nxs.suite.state import SuiteState


def add_switch_parser(sub):
    p = sub.add_parser('switch',
                       help='Converge every declared unit to suite.yaml')
    p.add_argument('-c', '--config', default=None,
                   help='Manifest path (default: /etc/aliensense/suite.yaml on a camera '
                        'host, else ~/.config/aliensense/suite.yaml)')
    p.add_argument('--dry-run', action='store_true',
                   help='Report the actions without touching any device')
    # dest is `only_unit`, not `unit`, so it never collides with the
    # top-level `nxs --unit` (manual transport addressing).
    p.add_argument('--unit', dest='only_unit', default=None,
                   help='Reconcile a single unit by name')
    p.add_argument('--accept-new-serial', action='store_true',
                   help='Re-record a TOFU serial after a deliberate board '
                        'swap (never overrides a manifest pin)')
    p.add_argument('--fdt', default=None, metavar='PATH',
                   help="The base device tree the generated boot entry names, "
                        "where the tool cannot pick this module's own")


def manifest_present(config_path: str) -> bool:
    """Whether a declaration stands at the path; an empty file is none."""
    return os.path.exists(config_path) and os.path.getsize(config_path) > 0


def load_manifest(config_path: str):
    """The manifest, or a one-line refusal on stderr and None; a per-user
    declaration a camera host does not read is named with its move."""
    if not manifest_present(config_path):
        refusal = (stray_declaration(config_path)
                   or f"no manifest at {config_path}\n  - nxs generate")
        print(f"nxs: {refusal}", file=sys.stderr)
        return None
    try:
        return load_suite_config(config_path)
    except (ManifestError, OSError) as e:
        print(f"nxs: {e}", file=sys.stderr)
        return None


def cmd_switch(args) -> int:
    # The declaration `-c` names is the run's, for every step that reads one.
    use_config_path(getattr(args, 'config', None))
    try:
        return _switch(args)
    finally:
        use_config_path(None)


def _switch(args) -> int:
    from nxs.suite.reconcile import switch_suite
    from nxs.suite.switch_cam import (PORT_REFUSED, _owned_ports, _port_is_up, camera_steps,
                                      ports_again_step)

    config_path = default_config_path()
    # The host's steps need no declaration (a fresh host boots no camera bus, and the
    # ports answer after that reboot); a declaration that does not load stops the run.
    cfg = None
    if manifest_present(config_path):
        cfg = load_manifest(config_path)
        if cfg is None:
            return 1
    refused = False
    if args.only_unit is None:
        # The camera side first: the host, the pods, the boot table, the
        # capture stack, the ports. A reboot it asks for ends the run. A
        # port whose bring-up refused keeps its verdict, and the units
        # follow: once the alias phase ran its pods answer at their
        # aliases, and a pod still where it straps is skipped.
        rc = camera_steps(cfg, dry_run=args.dry_run, fdt=getattr(args, 'fdt', None),
                          config_path=config_path)
        if rc == PORT_REFUSED:
            refused = True
        elif rc != 0:
            return rc
    if cfg is None:
        load_manifest(config_path)        # the refusal names `nxs generate`
        return 1
    state = SuiteState.load(default_state_path())
    reports = switch_suite(cfg, state, dry_run=args.dry_run,
                           only_unit=args.only_unit,
                           accept_new_serial=args.accept_new_serial,
                           skip=strapped_units(cfg) if refused else None)
    if not reports and args.only_unit is not None:
        print(f"nxs switch: no unit named {args.only_unit!r} in the manifest",
              file=sys.stderr)
        return 1
    failed = 0
    for report in reports:
        mark = "✓" if report.ok else "✗"
        print(f"{mark} {report.name} ({report.link})")
        for action in report.actions:
            print(f"    {action}")
        for note in report.notes:
            print(f"    {note}")
        if not report.ok:
            print(f"    {report.error}")
            failed += 1
    rc = 0
    if args.only_unit is None:
        # A pod on new firmware restarted behind its port, which comes up again: a port recorded up
        # runs the old program until then, one the old firmware refused takes the declared image now.
        rc = ports_again_step(cfg, [report.name for report in reports if report.restarted],
                              dry_run=args.dry_run, config_path=config_path)
        # A refused port that came up after the units refuses the run no more.
        refused = refused and not all(_port_is_up(name) for name in _owned_ports(cfg))
    if failed:
        print(f"{failed}/{len(reports)} unit(s) failed", file=sys.stderr)
    return _final_status(rc, refused=refused, failed=bool(failed))


def _final_status(ports_rc: int, *, refused: bool, failed: bool) -> int:
    """The run's exit status: a reboot a port asked for ends it with 3
    whatever else happened, a refused port or a failed unit is 1, and the
    ports' own refusal code never leaves the tool."""
    from nxs.suite.switch_cam import PORT_REFUSED

    if ports_rc == 3:
        return 3
    if refused or failed or ports_rc == PORT_REFUSED:
        return 1
    return ports_rc


def strapped_units(cfg) -> Dict[str, List[str]]:
    """The declared units whose pod sits where it straps, not at its alias,
    on an owned port that is not up: the unit steps skip them, with the
    lines `nxs <port> status` prints for the pod. A pod at its alias is
    read whatever its port's verdict, since the alias phase ran."""
    from nxs.cam import port_state
    from nxs.cam import run as cam_run
    from nxs.cam.select import _hub_for, select_port_links
    from nxs.suite.switch_cam import _owned_ports, _pod_at, _port_args, _port_is_up

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
