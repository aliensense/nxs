"""`nxs switch`: converge every declared unit to the manifest. The
transports come from the manifest, which is the topology."""
import os
import sys

from nxs.suite import default_config_path, default_state_path
from nxs.suite.schema import ManifestError, load_suite_config
from nxs.suite.state import SuiteState


def add_switch_parser(sub):
    p = sub.add_parser('switch',
                       help='Converge every declared unit to suite.yaml')
    p.add_argument('-c', '--config', default=None,
                   help=f'Manifest path (default: {default_config_path()})')
    p.add_argument('--dry-run', action='store_true',
                   help='Report the actions without touching any device')
    # dest is `only_unit`, not `unit`, so it never collides with the
    # top-level `nxs --unit` (manual transport addressing).
    p.add_argument('--unit', dest='only_unit', default=None,
                   help='Reconcile a single unit by name')
    p.add_argument('--accept-new-serial', action='store_true',
                   help='Re-record a TOFU serial after a deliberate board '
                        'swap (never overrides a manifest pin)')


def load_manifest(config_path: str):
    """The manifest, or a one-line refusal on stderr and None."""
    if not (os.path.exists(config_path) and os.path.getsize(config_path) > 0):
        print(f"nxs: no manifest at {config_path}\n  - nxs generate",
              file=sys.stderr)
        return None
    try:
        return load_suite_config(config_path)
    except (ManifestError, OSError) as e:
        print(f"nxs: {e}", file=sys.stderr)
        return None


def cmd_switch(args) -> int:
    from nxs.suite.reconcile import switch_suite
    from nxs.suite.switch_cam import camera_steps

    config_path = getattr(args, 'config', None) or default_config_path()
    cfg = load_manifest(config_path)
    if cfg is None:
        return 1
    if args.only_unit is None:
        # The camera side first: the host, the pods, the boot table, the
        # capture stack, the ports. A reboot it asks for ends the run.
        rc = camera_steps(cfg, dry_run=args.dry_run)
        if rc != 0:
            return rc
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
