#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""`nxs`, the command-line tool for the NXS sensor VM. The verbs and their
flags are the parser in `build_parser()`, which `nxs --help` prints."""

import argparse
import os
import sys

from nxs._generated_constants import CyphalDefaults, NxsDevices
from nxs import experimental

from nxs.ros2_cli import (
    _DEVICE_FLAGS,
    _allocate_can_local_ids,
    _device_flags_given,
    _ros2_lock_tokens,
    _ros2_targets,
    add_ros2_parser,
    cmd_ros2,
)
from nxs.assets_cli import add_assets_parser, cmd_assets
from nxs.setup_cli import COMPLETION_TARGET, print_pack_hint
from nxs.store_cli import (
    add_store_parser,
    cmd_cycle,
    cmd_save,
    cmd_store_clear,
    cmd_store_ls,
    cmd_store_rm,
)
from nxs.stream_cli import (
    _configure_stream,
    _display_unit,
    _every_for_hz,
    _format_sample_columns,
    _get_output_fields,
    _get_output_fields_by_name,
    _looks_like_vm_restart,
    add_stream_parser,
    cmd_stream,
)
from nxs.target_cli import (
    _apply_unit_link,
    _default_can_mtu,
    _default_local_node_id,
    _default_transport,
    _manifest_nodes,
    _manifest_unit,
    _node_rewrite,
    _open_transport,
    _pick_unit_link,
    _platform_ports,
    _resolve_default_link,
)
from nxs.unit_cli import (
    _bare_timesync,
    _describe_target,
    _not_found_hint,
    _probe_detail,
    _target_flags,
    _upload_and_run,
    add_commission_parser,
    add_timesync_parser,
    add_unit_parsers,
    cmd_caps,
    cmd_commission,
    cmd_confirm_fw,
    cmd_get,
    cmd_identify,
    cmd_probe,
    cmd_push_fw,
    cmd_reboot,
    cmd_recover,
    cmd_run,
    cmd_set,
    cmd_status,
    cmd_timesync,
    cmd_upload,
)

#: The names `nxs.cli` has always answered to, wherever they now live.
__all__ = [
    "COMPLETION_TARGET",
    "VERSION_GATE_EXEMPT",
    "_DEVICE_FLAGS",
    "_allocate_can_local_ids",
    "_apply_unit_link",
    "_bare_timesync",
    "_configure_stream",
    "_default_can_mtu",
    "_default_local_node_id",
    "_default_transport",
    "_describe_target",
    "_device_flags_given",
    "_display_unit",
    "_every_for_hz",
    "_format_sample_columns",
    "_get_output_fields",
    "_get_output_fields_by_name",
    "_is_mutating",
    "_looks_like_vm_restart",
    "_manifest_nodes",
    "_manifest_unit",
    "_node_rewrite",
    "_not_found_hint",
    "_open_transport",
    "_pick_unit_link",
    "_platform_ports",
    "_probe_detail",
    "_require_firmware",
    "_require_supported_version",
    "_resolve_default_link",
    "_ros2_lock_tokens",
    "_ros2_targets",
    "_target_flags",
    "_tool_version",
    "_upload_and_run",
    "_warn_if_suite_managed",
    "add_commission_parser",
    "add_ros2_parser",
    "add_assets_parser",
    "add_store_parser",
    "add_stream_parser",
    "add_timesync_parser",
    "add_unit_parsers",
    "build_parser",
    "cmd_caps",
    "cmd_commission",
    "cmd_confirm_fw",
    "cmd_cycle",
    "cmd_get",
    "cmd_identify",
    "cmd_probe",
    "cmd_push_fw",
    "cmd_reboot",
    "cmd_recover",
    "cmd_ros2",
    "cmd_run",
    "cmd_save",
    "cmd_set",
    "cmd_assets",
    "cmd_status",
    "cmd_store_clear",
    "cmd_store_ls",
    "cmd_store_rm",
    "cmd_stream",
    "cmd_timesync",
    "cmd_upload",
    "main",
    "print_pack_hint",
]


def _is_mutating(args) -> bool:
    """True for verbs that change device state; the read forms of dual-mode
    verbs (`commission --show`, `store ls`) stay silent."""
    if args.command in ('upload', 'set', 'run',
                        'push-fw', 'recover', 'reboot', 'confirm-fw'):
        return True
    if args.command == 'store':
        return args.store_cmd != 'ls'
    if args.command == 'commission':
        return not args.show and (args.node_id is not None
                                  or bool(args.subject) or args.save)
    return False


def _require_supported_version(t):
    """Exit if the device's register-map contract differs from this tool's.
    I2C serves PROTO_VERSION; the Cyphal transports have none and are skipped."""
    from nxs.client import contract_mismatch
    mm = contract_mismatch(t)
    if mm is not None:
        sys.exit(f"nxs: {mm} — use a matching nxs (`nxs probe` for details).")


def _require_firmware(t, args):
    """Exit if the unit runs firmware older than this tool drives; the
    line names the `push-fw` that ends the mismatch. A unit serving no
    readable identity passes, as with the contract gate."""
    from nxs.client import FirmwareTooOld, require_firmware
    try:
        require_firmware(t, target=_target_flags(args))
    except FirmwareTooOld as e:
        sys.exit(f"nxs: {e}")


def _warn_if_suite_managed(args):
    """One stderr note when a flag-addressed mutating verb targets a declared
    unit. Best-effort: a missing or broken manifest never blocks the manual path."""
    import os

    try:
        from nxs.suite import default_config_path
        from nxs.suite.schema import LinkSpec, load_suite_config

        path = default_config_path()
        if not os.path.exists(path):
            return
        cfg = load_suite_config(path)
    except Exception:
        return

    # LinkSpec.identity() resolves device aliases, so a manifest that
    # names /dev/i2c-cam1 still matches a -b /dev/i2c-9 invocation.
    if args.transport == 'i2c':
        target = LinkSpec(transport='i2c', bus=args.bus, address=args.addr)
    elif args.transport == 'cyphal-can':
        node = (args.remote_node_id if args.remote_node_id is not None
                else CyphalDefaults.DEFAULT_NODE_ID)
        target = LinkSpec(transport='cyphal-can', iface=args.port or 'can0',
                          node_id=node)
    else:
        target = LinkSpec(transport='cyphal-serial', port=args.port)
    identity = target.identity()

    for unit in cfg.units:
        if any(link.identity() == identity for link in unit.links):
            print(f"note: unit '{unit.name}' is suite-managed — this change "
                  f"reverts on `nxs switch`; keep it with "
                  f"`nxs tune --freeze --unit {unit.name}`", file=sys.stderr)
            return


# The verbs that must reach an off-contract device: `probe` reports the
# version, `push-fw` and `recover` carry the firmware that ends the mismatch.
VERSION_GATE_EXEMPT = ('probe', 'push-fw', 'recover', 'reboot', 'confirm-fw')


# The transports' subject vocabulary, in enum order.


def _git_version() -> str:
    """Commit the tool runs from: live `git describe` in a source checkout,
    else the value baked into the wheel, else empty."""
    import subprocess
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    try:
        out = subprocess.run(
            ['git', '-C', pkg_dir, 'describe', '--tags', '--match', 'v*',
             '--always', '--dirty'],
            capture_output=True, text=True, timeout=2)
        if out.returncode == 0 and out.stdout.strip():
            return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        from nxs._build_info import BUILD_GIT_VERSION
        return BUILD_GIT_VERSION
    except ImportError:
        return ''


def _package_version() -> str:
    """The installed wheel's version, `0.0.0+source` in a checkout."""
    from nxs import __version__
    return __version__


def _tool_version() -> str:
    base = _package_version()
    git = _git_version()
    return f'{base} ({git})' if git else base


class _VersionAction(argparse.Action):
    """`--version`, computed lazily: building the version string runs
    `git describe`, which must not happen at parser construction."""

    def __call__(self, parser, namespace, values, option_string=None):
        print(f"nxs {_tool_version()}")
        parser.exit()


def build_parser():
    parser = argparse.ArgumentParser(
        prog='nxs',
        description='NXS sensor VM command-line tool')
    parser.add_argument('--version', action=_VersionAction, nargs=0,
                        help="show program's version number and exit")
    parser.add_argument('--experimental', action='store_true',
                        default=experimental.enabled(),
                        help='unlock the experimental surface: the unshipped '
                             'modes (marked), a pack root and its overlays '
                             '($NXS_CAM_DESCRIPTORS, $NXS_CAM_EXPERIMENTAL)')
    parser.add_argument('-t', '--transport', choices=['i2c', 'cyphal-serial', 'cyphal-can'],
                        default=_default_transport(),
                        help='Transport ($NXS_TRANSPORT, else i2c on Linux / '
                             'cyphal-serial on macOS, Windows)')
    parser.add_argument('--local-node-id', type=int,
                        default=_default_local_node_id(),
                        help=f'Host Cyphal node-ID ($NXS_LOCAL_NODE_ID, else '
                             f'{CyphalDefaults.HOST_NODE_ID}). Vary it across '
                             f'back-to-back scripted runs: a repeated '
                             f'transfer-ID within 2 s is dropped by the '
                             f'device as a duplicate')
    parser.add_argument('-b', '--bus', default=os.environ.get('NXS_BUS'),
                        help='I2C bus device ($NXS_BUS, else the one i2c unit '
                             'suite.yaml declares, else the single unit '
                             'answering on the camera buses)')
    parser.add_argument('-p', '--port', default=os.environ.get('NXS_PORT'),
                        help='Serial device for cyphal-serial, or SocketCAN '
                             'interface (e.g. can0) for cyphal-can '
                             '($NXS_PORT, else autodetect any USB-attached '
                             'port — /dev/cu.usb* on macOS, '
                             '/dev/tty{USB,ACM}* on Linux, COM* on '
                             'Windows)')
    parser.add_argument('--baud', type=int, default=460800,
                        help='Serial baud rate (default: 460800 — matches the module lpuart1 DTS)')
    parser.add_argument('--mtu', type=int, choices=[8, 64],
                        default=_default_can_mtu(),
                        help='cyphal-can frame MTU: 64 for CAN FD (default), 8 '
                             'for a bus on a Classic profile ($NXS_CAN_MTU)')
    parser.add_argument('--remote-node-id', type=int, default=None,
                        help=f'Cyphal node-ID of the target NXS (default: '
                             f'{CyphalDefaults.DEFAULT_NODE_ID} — the '
                             f'plug-and-play factory address; override to reach '
                             f'a commissioned node)')
    # `int(x, 0)` honours the base prefix (0x/0b/decimal); the default is the
    # address every NXS straps.
    parser.add_argument('-a', '--addr', type=lambda x: int(x, 0),
                        default=NxsDevices.RBDevice.NXS,
                        help='NXS device address (any base; default: '
                             f'0x{NxsDevices.RBDevice.NXS:02x})')
    parser.add_argument('--unit', default=None,
                        help='Address a suite-managed unit by its manifest '
                             'name — resolves the transport and link from '
                             'suite.yaml, overriding -t/-b/-p/-a')
    sub = parser.add_subparsers(dest='command', required=True,
                                metavar='COMMAND')

    add_assets_parser(sub)

    p_tune = sub.add_parser('tune', help='The personality panel: edit the declared '
                                         'config from real options only')
    p_tune.add_argument('--list', action='store_true',
                        help='print the channels, their fields, and the '
                             'options each offers — no TUI')
    p_tune.add_argument('--set', action='append', default=[],
                        metavar='CHANNEL[:SECTION]:FIELD=VALUE',
                        help='set one declared value from its options '
                             '(repeatable); saves with a timestamped .bak')
    p_tune.add_argument('--schema', action='store_true',
                        help="The rig's rules as JSON Schema: the manifest schema "
                             'narrowed by the nodes on this rig, for validating a '
                             'declaration before it reaches the rig')
    p_tune.add_argument('--play', action='store_true',
                        help='validate the saved manifest and signal nxsd '
                             'to reconverge')
    p_tune.add_argument('--json', action='store_true',
                        help='machine-readable output for --list/--set/'
                             '--play (contract 2)')
    p_tune.add_argument('--freeze', action='store_true',
                        help='adopt the units\' live tuning into suite.yaml '
                             '(the device wins); with --ports the camera '
                             'ports as the booted overlay and the live '
                             'state have them')
    p_tune.add_argument('--ports', action='store_true',
                        help='with --freeze: the camera ports instead of '
                             'the units')
    p_tune.add_argument('--unit', dest='only_unit', default=None,
                        help='with --freeze: one unit by name')
    p_tune.add_argument('--dry-run', action='store_true',
                        help='with --freeze: print the would-be block, '
                             'write nothing')

    from nxs.host.cli import add_host_parser
    add_host_parser(sub)

    from nxs.suite.switch import add_switch_parser
    add_switch_parser(sub)

    p_mcp = sub.add_parser('mcp', help='Serve the nxs verbs to an AI agent '
                                       'over MCP (stdio)')
    p_mcp.add_argument('--doc-table', action='store_true',
                       help='print the Tools table of the MCP Tool Reference '
                            'and exit')

    p_gen = sub.add_parser(
        'generate',
        help='Walk the rig and write hardware.yaml; seed suite.yaml when '
             'there is none (the rig\'s nixos-generate-config)')
    p_gen.add_argument('-c', '--config', default=None,
                       help='Manifest path (default: the platform manifest)')
    p_gen.add_argument('--dry-run', action='store_true',
                       help='Walk and report; write nothing (a unit running '
                            'nothing is not tried)')
    p_gen.add_argument('--json', action='store_true',
                       help='The walk as data: every port with the nodes that '
                            'answered on it, and what was written')
    add_unit_parsers(sub)

    add_stream_parser(sub)

    add_ros2_parser(sub)

    add_store_parser(sub)

    add_timesync_parser(sub)

    from nxs.personality_cli import add_parser as add_personality_verbs
    add_personality_verbs(sub)

    from nxs.cam.cli import add_cam_parser
    add_cam_parser(sub)

    from nxs.calibrate import add_calibrate_parser
    add_calibrate_parser(sub)

    add_commission_parser(sub)

    return parser


def main():
    """The entry point. Under `--json` a refusal leaves as the `refusal`
    document, so a program reads the fact and the alternatives apart."""
    from nxs import term

    as_json = "--json" in sys.argv[1:]
    term.json_mode(as_json)
    try:
        return _main()
    except SystemExit as exc:
        if as_json and isinstance(exc.code, str):
            term.refusal_text(exc.code)
            return 1
        raise


def _main():
    # The node rewrite runs first: a leading --experimental unlocks the
    # experimental surface, and the parser is built for that surface.
    argv = _node_rewrite(sys.argv)
    parser = build_parser()
    try:
        import argcomplete
        argcomplete.autocomplete(parser)
    except ImportError:
        pass
    args = parser.parse_args(argv[1:])
    if os.environ.get("SUDO_USER") and os.geteuid() == 0:
        # The tool asks for root per step; a whole run under sudo would put
        # the state and the config where the operator's shell never looks.
        sys.exit("nxs: do not run under sudo; it asks when it needs root")
    if getattr(args, "experimental", False):
        experimental.enable()

    # The cam group is internal plumbing for the node grammar; typed directly
    # it is refused (one address space: nodes, not families).
    if (getattr(args, "cam_cmd", None)
            and not getattr(args, "_node", False)):
        raise SystemExit(
            "nxs cam is retired — address the node: nxs cam1 A "
            f"{args.cam_cmd} (or cam1:A {args.cam_cmd})")

    # `upload -o FILE` compiles to a file with no device attached, as do
    # the host verbs of `personality`; every other command needs a transport.
    if args.command == 'upload' and args.output:
        return cmd_upload(None, args)
    if args.command == 'personality':
        from nxs.personality_cli import cmd_personality
        return cmd_personality(args)

    if args.command == 'assets':
        return cmd_assets(args)

    if args.command == 'generate':
        from nxs.generate import cmd_generate
        return cmd_generate(args)

    if args.command == 'switch':
        from nxs.suite.switch import cmd_switch
        return cmd_switch(args)

    if args.command == 'timesync' and _bare_timesync(args):
        from nxs.suite.timesync import cmd_timesync_suite
        return cmd_timesync_suite(args)

    if args.command == 'tune':
        from nxs.tune import cmd_tune
        return cmd_tune(args)

    if args.command == 'mcp':
        from nxs.mcp_server import cmd_mcp
        return cmd_mcp(args)

    if args.command == 'host':
        from nxs.host.cli import cmd_host
        return cmd_host(args)

    # Bare probe/status: no bus, port, or unit addressed. Probe is
    # config-free discovery; status is the declared-vs-actual tree.
    if (args.transport == 'i2c' and args.bus is None
            and args.port is None and not getattr(args, 'unit', None)):
        if args.command == 'probe':
            from nxs.tree import render_scan
            return render_scan(as_json=args.json)
        if args.command == 'status':
            from nxs.tree import render_tree
            return render_tree(as_json=args.json)
    if args.command in ('probe', 'status') and getattr(args, 'json', False):
        sys.exit(f"nxs {args.command} --json is the bare form (discovery / "
                 f"the declared tree) or a port's: nxs cam1 {args.command} "
                 f"--json; an addressed unit renders text")

    if args.command == 'cam':
        # Cam verbs drive their own I2C bus from the port topology, not
        # the NXS transport flags.
        from nxs.cam.cli import cmd_cam
        return cmd_cam(args)

    # ros2 builds its own transports too: the whole suite by default,
    # one --unit, or an ad-hoc flag-addressed device.
    if args.command == 'ros2':
        return cmd_ros2(args)

    port_given = args.port  # None: cyphal-serial autodetects during the open
    port_was_present = bool(port_given) and os.path.exists(port_given)
    try:
        t = _open_transport(args)
    except ImportError as e:
        sys.exit(f"nxs: {e}")  # an extra is missing; no flag cures that
    except (OSError, ValueError, RuntimeError) as e:
        # A missing device or permission, a malformed bus, an unknown transport,
        # or Cyphal startup. One case is diagnosed: a serial port gone after the open.
        if (args.transport == 'cyphal-serial' and args.port
                and (port_was_present or port_given is None)
                and not os.path.exists(args.port)):
            from nxs.client import LINK_DROP_ADVICE
            sys.exit(f"nxs: {args.port} vanished while opening it — the USB "
                     f"device dropped off the bus. {LINK_DROP_ADVICE}")
        sys.exit(f"nxs: {args.transport}: {e} "
                 f"(check -t/-b/-p or the matching $NXS_* env vars)")
    except KeyboardInterrupt:
        sys.exit(130)  # Ctrl-C during a wedged open (e.g. a hung J-Link VCOM)

    # Flag-addressed mutation of a declared unit gets a breadcrumb. A device on
    # another register-map contract misbehaves silently, so refuse first; a
    # unit on firmware older than this tool drives is refused with its push.
    if args.command not in VERSION_GATE_EXEMPT:
        _require_supported_version(t)
        _require_firmware(t, args)

    if getattr(args, 'unit', None) is None and _is_mutating(args):
        _warn_if_suite_managed(args)

    if args.command == 'store':
        store_cmds = {
            'ls': cmd_store_ls,
            'save': cmd_save,
            'rm': cmd_store_rm,
            'clear': cmd_store_clear,
            'cycle': cmd_cycle,
        }
        dispatch = store_cmds[args.store_cmd]
    else:
        commands = {
            'probe': cmd_probe, 'status': cmd_status,
            'upload': cmd_upload, 'caps': cmd_caps,
            'get': cmd_get, 'set': cmd_set, 'stream': cmd_stream,
            'run': cmd_run,
            'recover': cmd_recover,
            'reboot': cmd_reboot,
            'confirm-fw': cmd_confirm_fw,
            'identify': cmd_identify,
            'push-fw': cmd_push_fw,
            'timesync': cmd_timesync,
            'commission': cmd_commission,
        }
        if args.command == 'calibrate':
            from nxs.calibrate import cmd_calibrate
            dispatch = cmd_calibrate
        else:
            dispatch = commands[args.command]

    # A command reading live state misreports an unreachable node as empty;
    # `probe`/`status` and DFU are exempt, and an upload warns and proceeds.
    NEEDS_DEVICE = {'caps', 'get', 'set', 'stream', 'run', 'commission', 'store',
                    'identify', 'calibrate'}
    RECOVERY_VERBS = {'upload'}
    command = args.command
    if command in NEEDS_DEVICE | RECOVERY_VERBS and not t.probe():
        if command in NEEDS_DEVICE:
            detail = _probe_detail(t) or (" (check wiring / power / -t/-b/-p "
                                          "or the $NXS_* env vars)")
            print(f"nxs: NXS not found on {args.transport}{detail}"
                  f"{_not_found_hint(args)}", file=sys.stderr)
            return 1
        print(f"nxs: no probe answer on {args.transport}; "
              f"attempting {args.command} anyway",
              file=sys.stderr)

    # Transport failures surface as TimeoutError / RuntimeError (no ACK,
    # non-OK code) or OSError (bus error, NACK, permissions); print one clean line.
    try:
        rc = dispatch(t, args)
    except TimeoutError as e:
        print(f"ERROR: firmware timed out: {e}", file=sys.stderr)
        rc = 1
    except RuntimeError as e:
        print(f"ERROR: firmware rejected request: {e}", file=sys.stderr)
        rc = 1
    except OSError as e:
        print(f"ERROR: transport I/O failed: {e}", file=sys.stderr)
        rc = 1
    except KeyboardInterrupt:
        print("\nnxs: interrupted", file=sys.stderr)
        rc = 130

    # A mid-session serial disconnect (a wedged/unplugged USB device) leaves the
    # port gone; report it as one clean line, not a stale "no response".
    if t.link_dropped():
        print(f"nxs: {t.disconnect_message()}", file=sys.stderr)
        return 1
    return rc


if __name__ == '__main__':
    sys.exit(main())
