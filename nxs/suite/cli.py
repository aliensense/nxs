"""`nxs suite` subcommands: scan, switch, status, timesync, freeze, reset, snapshot, list-generations, rollback, collect-garbage.

Suite commands build their transports from the manifest, not from the
global -t/-b/-p flags — the manifest is the topology.
"""
import os
import sys

from nxs.suite import default_config_path, default_state_path
from nxs.suite.schema import ManifestError, load_suite_config
from nxs.suite.state import SuiteState


def add_suite_parser(sub):
    p_suite = sub.add_parser(
        'suite', help='Declarative multi-unit management (suite.yaml)')
    p_suite.add_argument('-c', '--config', default=None,
                         help=f'Manifest path (default: {default_config_path()})')
    suite_sub = p_suite.add_subparsers(dest='suite_cmd', required=True)

    p_scan = suite_sub.add_parser(
        'scan', help='Probe every plausible link and report what answered')
    p_scan.add_argument('--init', action='store_true',
                        help='Emit a suite.yaml skeleton from the scan')
    p_scan.add_argument('--diff', action='store_true',
                        help='Compare the scan against the manifest')

    p_switch = suite_sub.add_parser(
        'switch', help='Converge every declared unit to the manifest '
                       '(and record a generation when all converge)')
    p_switch.add_argument('--dry-run', action='store_true',
                         help='Report the actions without touching any device')
    # dest is `only_unit`, not `unit`, so it never collides with the
    # top-level `nxs --unit` (manual transport addressing).
    p_switch.add_argument('--unit', dest='only_unit', default=None,
                         help='Reconcile a single unit by name')
    p_switch.add_argument('--accept-new-serial', action='store_true',
                         help='Re-record a TOFU serial after a deliberate '
                              'board swap (never overrides a manifest pin)')

    suite_sub.add_parser('status', help='One health row per declared unit')

    p_ts = suite_sub.add_parser(
        'timesync', help='Keep every declared unit time-disciplined '
                         '(resident pusher)')
    p_ts.add_argument('--unit', dest='only_units', action='append',
                      default=None,
                      help='Discipline only the named unit (repeatable)')
    p_ts.add_argument('--interval', type=float, default=None,
                      help='Seconds between push rounds (default 1)')
    p_ts.add_argument('--once', action='store_true',
                      help='One round and exit')
    p_ts.add_argument('--systemd', action='store_true',
                      help='Print a systemd unit running this pusher '
                           'resident, then exit')

    p_reset = suite_sub.add_parser(
        'reset', help='Blank declared unit(s): stop driver, clear store, '
                      'forget learned state')
    reset_group = p_reset.add_mutually_exclusive_group(required=True)
    reset_group.add_argument('--unit', dest='only_unit',
                             help='Reset a single unit by name')
    reset_group.add_argument('--all', action='store_true',
                             help='Reset every declared unit (requires --yes)')
    p_reset.add_argument('--yes', action='store_true',
                         help='Confirm a suite-wide --all reset')
    p_reset.add_argument('--factory', action='store_true',
                         help='Also revert node-id and decimation to the '
                              'shipping defaults (on a shared CAN trunk this '
                              'returns every reset unit to node 125)')

    suite_sub.add_parser(
        'collect-garbage',
        help='Drop state entries for units no longer in the manifest')

    p_snapshot = suite_sub.add_parser(
        'snapshot', help='Tag the latest generation as known-good')
    p_snapshot.add_argument('label', help='Tag name, e.g. field-day-1')

    suite_sub.add_parser(
        'list-generations',
        help='Recorded manifest generations, newest first')

    p_rollback = suite_sub.add_parser(
        'rollback', help='Restore a recorded manifest and re-apply')
    p_rollback.add_argument('ref', help='A snapshot label or generation sha')

    p_freeze = suite_sub.add_parser(
        'freeze', help='Adopt live tuning into the manifest (device wins)')
    group = p_freeze.add_mutually_exclusive_group(required=True)
    group.add_argument('--unit', dest='only_unit',
                       help='Freeze a single unit by name')
    group.add_argument('--all', action='store_true',
                       help='Freeze every declared unit')
    p_freeze.add_argument('--dry-run', action='store_true',
                          help='Print the would-be manifest block(s) without '
                               'writing suite.yaml')
    p_freeze.add_argument('--pin-firmware', action='store_true',
                          help='Also pin the running firmware version where '
                               'the transport serves one')


def cmd_suite(args) -> int:
    config_path = args.config or default_config_path()
    cfg = None
    manifest_error = None
    # A zero-byte manifest is "no manifest yet", not corruption — a
    # crashed editor or an interrupted redirect leaves one behind, and
    # `scan --init` recreates it.
    if os.path.exists(config_path) and os.path.getsize(config_path) > 0:
        try:
            cfg = load_suite_config(config_path)
        except (ManifestError, OSError) as e:
            manifest_error = e

    # Scan bootstraps and grows manifests. A broken manifest must not
    # block the read-only listing (it only widens the sweeps), but
    # --init edits the file in place, so merging over a manifest that
    # cannot be parsed would risk hand-written content — refuse instead.
    # --diff has nothing to compare against and refuses too.
    if args.suite_cmd == 'scan' and not args.diff:
        if manifest_error is not None:
            if args.init:
                print(f"nxs suite: cannot merge into an invalid manifest: "
                      f"{manifest_error}", file=sys.stderr)
                return 1
            print(f"nxs suite: ignoring invalid manifest: {manifest_error}",
                  file=sys.stderr)
        return _cmd_scan(args, cfg, config_path)
    # Emitting the pusher's systemd unit is pure text generation — it
    # must work on a host that has no manifest yet.
    if args.suite_cmd == 'timesync' and args.systemd:
        return _cmd_timesync(args, cfg)
    if cfg is None:
        if manifest_error is not None:
            print(f"nxs suite: {manifest_error}", file=sys.stderr)
        else:
            print(f"nxs suite: no manifest at {config_path} — write one, or "
                  f"start from `nxs suite scan --init`",
                  file=sys.stderr)
        return 1
    if args.suite_cmd == 'scan':
        return _cmd_scan(args, cfg, config_path)

    if args.suite_cmd == 'freeze':
        return _cmd_freeze(args, cfg, config_path)
    if args.suite_cmd == 'timesync':
        return _cmd_timesync(args, cfg)
    state = SuiteState.load(default_state_path())
    if args.suite_cmd == 'switch':
        return _cmd_switch(args, cfg, state, config_path)
    if args.suite_cmd == 'reset':
        return _cmd_reset(args, cfg, state)
    if args.suite_cmd == 'collect-garbage':
        return _cmd_collect_garbage(cfg, state)
    if args.suite_cmd == 'snapshot':
        return _cmd_snapshot(args, state)
    if args.suite_cmd == 'list-generations':
        return _cmd_list_generations(state)
    if args.suite_cmd == 'rollback':
        return _cmd_rollback(args, state, config_path)
    return _cmd_status(cfg, state)


def _cmd_scan(args, cfg, config_path) -> int:
    from nxs.suite.scan import merge_init, render_diff, render_init, scan_suite

    found = scan_suite(cfg)
    if args.diff:
        sys.stdout.write(render_diff(cfg, found))
        return 0
    if args.init:
        # In-place and idempotent: no manifest creates one; an existing
        # manifest grows additively — hand-written names, configs, and
        # comments survive, and a re-run on a covered bench changes
        # nothing.
        if cfg is None:
            directory = os.path.dirname(config_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(config_path, "w", encoding="utf-8") as f:
                f.write(render_init(found))
            print(f"created {config_path} with {len(found)} link(s) "
                  f"transcribed")
            return 0
        for line in merge_init(cfg, found, config_path):
            print(line)
        return 0
    # Read-only listing; hits belonging to declared units carry their
    # names.
    named = {}
    if cfg is not None:
        named = {link.identity(): unit.name
                 for unit in cfg.units for link in unit.links}
        named.update({unit.serial: unit.name for unit in cfg.units
                      if unit.serial})
    for hit in found:
        name = named.get(hit.link.identity()) or (hit.serial and
                                                  named.get(hit.serial))
        line = f"{name}  " if name else ""
        line += hit.link.describe()
        if hit.serial:
            line += f"  serial {hit.serial}"
        if hit.fw_version:
            line += f"  fw {hit.fw_version}"
        if hit.driver:
            line += f"  driver {hit.driver}"
        print(line)
    if not found:
        print("nothing answered — check wiring and permissions")
    return 0


def _cmd_switch(args, cfg, state, config_path) -> int:
    from nxs.suite.reconcile import switch_suite

    reports = switch_suite(cfg, state, dry_run=args.dry_run,
                          only_unit=args.only_unit,
                          accept_new_serial=args.accept_new_serial)
    if not reports:
        print(f"nxs suite: no unit named {args.only_unit!r} in the manifest",
              file=sys.stderr)
        return 1
    failed = 0
    for report in reports:
        mark = "✓" if report.ok else "✗"
        print(f"{mark} {report.name} ({report.link})")
        for action in report.actions:
            print(f"    {action}")
        if not report.ok:
            print(f"    {report.error}")
            failed += 1
    if failed:
        print(f"{failed}/{len(reports)} unit(s) failed", file=sys.stderr)
        return 1
    # Only a fully-converged switch records a generation; partial
    # converges and byte-identical manifests record nothing.
    if not args.dry_run and args.only_unit is None:
        from nxs.suite.generations import GenerationsError, record_generation

        try:
            sha = record_generation(
                config_path, state.path,
                f"{len(reports)}/{len(reports)} converged")
            if sha:
                print(f"generation {sha} recorded")
        except GenerationsError as e:
            print(f"note: {e}", file=sys.stderr)
    return 0


def _cmd_timesync(args, cfg) -> int:
    import time

    from nxs.suite.timesync import SuitePusher

    from nxs.client import PUSH_INTERVAL_S, estimate_and_push

    interval = args.interval if args.interval is not None else PUSH_INTERVAL_S
    if args.systemd:
        import getpass

        from nxs.suite.timesync import render_systemd_unit, resolve_nxs_path

        print(render_systemd_unit(resolve_nxs_path(sys.argv[0]),
                                  getpass.getuser(), args.only_units,
                                  interval), end="")
        return 0
    try:
        pusher = SuitePusher(
                cfg, only_units=args.only_units,
                pusher=lambda t: estimate_and_push(t, interval_s=interval))
    except ValueError as e:
        print(f"nxs suite: {e}", file=sys.stderr)
        return 1
    count = len(args.only_units) if args.only_units else len(cfg.units)
    if not args.once:
        plural = "s" if count != 1 else ""
        print(f"disciplining {count} unit{plural} every {interval:g} s",
              flush=True)
    last = {}
    try:
        while True:
            failed = 0
            for report in pusher.round():
                ok = report.bound_us is not None
                if not ok:
                    failed += 1
                if args.once or last.get(report.name) != ok:
                    mark = "✓" if ok else "✗"
                    detail = f"±{report.bound_us} µs" if ok else report.note
                    print(f"{mark} {report.name}  {detail}", flush=True)
                last[report.name] = ok
            if args.once:
                return 1 if failed else 0
            time.sleep(interval)
    except KeyboardInterrupt:
        return 0
    finally:
        pusher.close()


def _cmd_reset(args, cfg, state) -> int:
    from nxs.suite.reset import reset_suite

    if getattr(args, 'all', False) and not args.yes:
        print("nxs suite: reset --all blanks every declared unit — re-run "
              "with --yes to confirm", file=sys.stderr)
        return 2
    reports = reset_suite(cfg, state, only_unit=args.only_unit,
                          factory=args.factory)
    if not reports:
        print(f"nxs suite: no unit named {args.only_unit!r} in the manifest",
              file=sys.stderr)
        return 1
    failed = 0
    for report in reports:
        mark = "✓" if report.ok else "✗"
        print(f"{mark} {report.name} ({report.link})")
        for action in report.actions:
            print(f"    {action}")
        if not report.ok:
            print(f"    {report.error}")
            failed += 1
    if failed:
        print(f"{failed}/{len(reports)} unit(s) failed", file=sys.stderr)
    return 1 if failed else 0


def _cmd_collect_garbage(cfg, state) -> int:
    from nxs.suite.reset import collect_garbage

    try:
        dropped = collect_garbage(cfg, state)
    except OSError as e:
        # Nothing was dropped durably — reporting names would lie.
        print(f"nxs suite: cannot write the state file ({e}) — "
              f"nothing collected", file=sys.stderr)
        return 1
    if not dropped:
        print("state matches the manifest — nothing to collect")
        return 0
    for name in dropped:
        print(f"dropped {name}")
    return 0


def _cmd_snapshot(args, state) -> int:
    from nxs.suite.generations import GenerationsError, snapshot

    try:
        sha = snapshot(state.path, args.label)
    except GenerationsError as e:
        print(f"nxs suite: {e}", file=sys.stderr)
        return 1
    print(f"tagged {args.label} at generation {sha}")
    return 0


def _cmd_list_generations(state) -> int:
    from nxs.suite.generations import GenerationsError, list_generations

    try:
        rows = list_generations(state.path)
    except GenerationsError as e:
        print(f"nxs suite: {e}", file=sys.stderr)
        return 1
    for sha, date, labels, summary in rows:
        label = f"  [{labels}]" if labels else ""
        print(f"{sha}  {date}  {summary}{label}")
    return 0


def _cmd_rollback(args, state, config_path) -> int:
    from argparse import Namespace

    from nxs.suite.generations import GenerationsError, rollback
    from nxs.suite.schema import ManifestError, load_suite_config

    try:
        rollback(state.path, args.ref, config_path)
    except GenerationsError as e:
        print(f"nxs suite: {e}", file=sys.stderr)
        return 1
    print(f"restored the manifest recorded at {args.ref}; applying")
    try:
        cfg = load_suite_config(config_path)
    except ManifestError as e:
        print(f"nxs suite: restored manifest invalid: {e}", file=sys.stderr)
        return 1
    return _cmd_switch(Namespace(dry_run=False, only_unit=None,
                                accept_new_serial=False),
                      cfg, state, config_path)


def _cmd_freeze(args, cfg, config_path) -> int:
    from nxs.suite.freeze import freeze_suite, render_frozen_block

    try:
        reports = freeze_suite(cfg, config_path,
                               only_unit=args.only_unit, dry_run=args.dry_run,
                               pin_firmware=args.pin_firmware)
    except (ManifestError, OSError) as e:
        print(f"nxs suite: cannot rewrite {config_path}: {e}", file=sys.stderr)
        return 1
    if not reports:
        print(f"nxs suite: no unit named {args.only_unit!r} in the manifest",
              file=sys.stderr)
        return 1
    failed = 0
    for report in reports:
        mark = "✓" if report.ok else "✗"
        if not report.ok:
            verb = "failed"
        elif report.changed:
            verb = "would freeze" if args.dry_run else "froze"
        else:
            verb = "unchanged"
        print(f"{mark} {report.name}: {verb}")
        for action in report.actions:
            print(f"    {action}")
        if not report.ok:
            print(f"    {report.error}")
            failed += 1
        elif report.changed and args.dry_run:
            try:
                print(render_frozen_block(config_path, report), end="")
            except ManifestError as e:
                print(f"    {e}", file=sys.stderr)
                failed += 1
    if failed:
        print(f"{failed}/{len(reports)} unit(s) failed", file=sys.stderr)
    return 1 if failed else 0


def _cmd_status(cfg, state) -> int:
    from nxs.suite.status import collect_status, render_status

    rows = collect_status(cfg, state)
    sys.stdout.write(render_status(rows))
    return 0 if all(r.up for r in rows) else 1
