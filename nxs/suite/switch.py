"""`nxs switch`: converge every declared unit to the manifest. The
transports come from the manifest, which is the topology."""
import os
import sys

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
    from nxs.suite.switch_cam import camera_steps

    config_path = default_config_path()
    # The host's steps need no declaration (a fresh host boots no camera bus, and the
    # ports answer after that reboot); a declaration that does not load stops the run.
    cfg = None
    if manifest_present(config_path):
        cfg = load_manifest(config_path)
        if cfg is None:
            return 1
    if args.only_unit is None:
        # The camera side first: the host, the pods, the boot table, the
        # capture stack, the ports. A reboot it asks for ends the run.
        rc = camera_steps(cfg, dry_run=args.dry_run, fdt=getattr(args, 'fdt', None),
                          config_path=config_path)
        if rc != 0:
            return rc
    if cfg is None:
        load_manifest(config_path)        # the refusal names `nxs generate`
        return 1
    state = SuiteState.load(default_state_path())
    reports = switch_suite(cfg, state, dry_run=args.dry_run,
                           only_unit=args.only_unit,
                           accept_new_serial=args.accept_new_serial)
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
    if failed:
        print(f"{failed}/{len(reports)} unit(s) failed", file=sys.stderr)
        return 1
    return 0
