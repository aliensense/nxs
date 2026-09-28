"""The verbs addressed at one NXS unit: what it is, what it runs, and what it takes."""

import sys
import time

from nxs._generated_constants import CyphalDefaults, RunnerStates, VmStates
from nxs.client import (
    ACTIVE_SLOT, op_error_name, peek_slot,
    SupportsBitTiming, SupportsCanTermination, SupportsCommissioning,
    SupportsFaultCounters, SupportsIdentify, SupportsRecovery,
    SupportsSlotPeek, SupportsTimeSync)
from nxs import params as device_params
from nxs.compiler import SEMANTIC_NAMES
from nxs.descriptor import sample_width
from nxs.image import ImageKind
from nxs.stream_cli import _get_output_fields
from nxs.term import status_line


def _describe_target(args) -> str:
    """The command's target in the link vocabulary (`i2c /dev/i2c-9@0x30`,
    `can can0 node 125`, `serial /dev/ttyUSB0`)."""
    if args.transport == 'i2c':
        return f"i2c {args.bus}@0x{args.addr:02X}"
    if args.transport == 'cyphal-can':
        node = (args.remote_node_id if args.remote_node_id is not None
                else CyphalDefaults.DEFAULT_NODE_ID)
        return f"can {args.port or 'can0'} node {node}"
    if args.transport == 'cyphal-serial':
        return f"serial {args.port}"
    return args.transport

def _target_flags(args) -> str:
    """The flags that address this command's unit, for a command line the
    refusal hands back (`--unit a`, `-t i2c -b /dev/i2c-9 -a 0x30`)."""
    unit = getattr(args, 'unit', None)
    if unit:
        return f"--unit {unit}"
    if args.transport == 'i2c':
        return f"-t i2c -b {args.bus} -a 0x{args.addr:02X}"
    if args.transport == 'cyphal-can':
        node = (args.remote_node_id if args.remote_node_id is not None
                else CyphalDefaults.DEFAULT_NODE_ID)
        return f"-t cyphal-can -p {args.port or 'can0'} --remote-node-id {node}"
    return f"-t {args.transport} -p {args.port}"

def _probe_detail(t) -> str:
    """The trailing reason suffix when the transport knows why the probe found
    nothing; empty when it does not."""
    getter = getattr(t, "probe_failure_detail", None)
    detail = getter() if getter is not None else None
    return f" — {detail}" if detail else ""

# A restarted Cyphal node reuses its transfer-IDs from zero, and the device
# drops a repeat seen inside libcanard's 2 s timeout, so two nxs runs closer
# than that on one host node-ID read as an absent node.
TID_COLLISION_HINT = (
    "If another nxs command finished less than ~2 s ago, the device may have "
    "discarded this one as a repeated transfer-ID — leave more than 2 s "
    "between commands.")

def _not_found_hint(args) -> str:
    """The transfer-ID collision note, on Cyphal/CAN only: the deduplication
    lives in libcanard's RX subscriptions, which the serial node does not
    keep, and I²C carries no transfer-IDs."""
    return (f"\n  {TID_COLLISION_HINT}"
            if getattr(args, "transport", "") == "cyphal-can" else "")

def cmd_probe(t, args):
    ok = t.probe()
    print(f"NXS @ {_describe_target(args)}: "
          f"{'found' if ok else 'NOT FOUND' + _probe_detail(t) + _not_found_hint(args)}")
    if ok:
        ver = t.interface_version()
    else:
        ver = None  # don't touch the bus after a failed probe
    if ver is not None:
        from nxs.client import SUPPORTED_PROTO_VERSION
        if ver == 0:
            print("register map: unversioned (firmware predates "
                  "PROTO_VERSION)")
        elif ver > SUPPORTED_PROTO_VERSION:
            print(f"register map: v{ver} — newer than this tool supports "
                  f"(v{SUPPORTED_PROTO_VERSION}); update nxs")
        else:
            print(f"register map: v{ver}")
    if ok:
        from nxs.client import firmware_too_old
        stale = firmware_too_old(t, target=_target_flags(args))
        if stale:
            print(f"firmware: {stale}")
        else:
            _probe_personality(t, args)
    return 0 if ok else 1

def _probe_personality(t, args) -> None:
    """One line for the camera personality a unit holds, and the addressed
    link's cache brought up to date; silent when the unit holds none or the
    store cannot be read."""
    from nxs.cam import unit_source
    from nxs.personality_cli import port_link_for

    try:
        found = unit_source.read_unit_personality(t)
    except (OSError, RuntimeError, ValueError):
        return
    if found is None:
        return
    print(f"personality: {found.summary()} in slot {found.slot}")
    where = port_link_for(args)
    if where is not None and found.descriptor is not None:
        topology, link = where
        unit_source.cache_descriptor(topology, link, found.descriptor,
                                     slot=found.slot, crc=found.crc,
                                     params=found.params, name=found.name)

def _upload_and_run(t, img, kind=ImageKind.DRIVER):
    from nxs.client import await_driver_up

    t.upload_image(img)
    if kind == ImageKind.CAMERA:
        # A camera personality is not run by the runner: it waits in a store
        # slot for the host's CAM_RUN under the bus token.
        print("Uploaded camera personality; nxs store save <slot> keeps it, "
              "and the camera verbs run it from that slot.")
        return 0
    t.vm_run()
    # Store and sensor probe finish asynchronously after RUN; the runner state
    # separates "up" from "loaded but the sensor never answered".
    state = RunnerStates.RunnerState
    settled = await_driver_up(t)
    if settled == state.MEASURING:
        print("Uploaded and running.")
        return 0
    if settled == state.PROBE_FAILED:
        print("Uploaded, but no sensor answered the personality — check the wiring "
              "and the personality's bus/address params.", file=sys.stderr)
        return 1
    print(f"Uploaded, but the personality did not come up (runner is "
          f"{state._NAMES.get(settled, settled)}) — check the device log.",
          file=sys.stderr)
    return 1

def cmd_upload(t, args):
    """`nxs upload`: compile and land a personality of either kind."""
    from nxs.personality_cli import cmd_upload as upload
    return upload(t, args)

def _format_allowed(p) -> str:
    """A param's allowed values for display: `lo..hi` for a range,
    `[a, b, c]` for an enum set."""
    vals = p['values']
    if p.get('type') == 'range' and len(vals) >= 2:
        return f"{vals[0]}..{vals[-1]}"
    return '[' + ', '.join(str(v) for v in vals) + ']'

def cmd_caps(t, args):
    name = t.read_driver_name()
    if not name:
        print("No personality loaded.")
        return 1

    print(f"Personality: {name}")
    caps = t.read_capabilities()
    if not caps:
        print("  (no parameters)")
        return 0

    # Single-value enums are compile-time fixed (e.g. `bus` on a SPI-only
    # chip); list them apart from what `caps` shows as runtime-tunable.
    tunable = [p for p in caps if len(p.get('values') or []) > 1]
    fixed   = [p for p in caps if len(p.get('values') or []) <= 1]

    if not tunable:
        print("  (no tunable parameters)")
    else:
        for p in tunable:
            live = "  [live]" if p.get('kind') == 'live' else ""
            unit = f"  unit={p['unit']}" if p['unit'] else ""
            print(f"  {p['name']}: {_format_allowed(p)}  "
                  f"current={p['current']}  default={p['default']}{unit}{live}")

    if fixed:
        summary = ', '.join(f"{p['name']}={p['current']}" for p in fixed)
        print(f"  fixed: {summary}")
    return 0

def _no_parameter(t, name: str) -> int:
    """The refusal for a name neither family carries: the personality's
    parameters and the device's, one line each."""
    print(f"no parameter {name}", file=sys.stderr)
    try:
        caps = t.read_capabilities() or []
    except (RuntimeError, OSError, TimeoutError):
        caps = []
    for p in caps:
        if len(p.get('values') or []) > 1:
            print(f"  - {p['name']}: {_format_allowed(p)}", file=sys.stderr)
    for p in device_params.PARAMS:
        print(f"  - {p.name}: {p.values}", file=sys.stderr)
    return 1

def cmd_get(t, args):
    """`nxs get <name>`: the personality's parameter, else the device's."""
    try:
        p = t.get_param(args.param)
    except KeyError:
        try:
            print(device_params.read(t, args.param))
        except device_params.ParamError as e:
            if str(e).startswith("no parameter"):
                return _no_parameter(t, args.param)
            print(f"nxs get: {e}", file=sys.stderr)
            return 1
        return 0
    print(f"{p['name']} = {p['current']}  "
          f"(valid: {_format_allowed(p)}  default: {p['default']}  "
          f"unit: {p['unit']})")
    return 0

def cmd_set(t, args):
    """`nxs set <name> <value>`: the personality's parameter, else the
    device's."""
    try:
        p = t.get_param(args.param)
    except KeyError:
        try:
            print(device_params.write(t, args.param, args.value))
        except device_params.ParamError as e:
            if str(e).startswith("no parameter"):
                return _no_parameter(t, args.param)
            print(f"nxs set: {e}", file=sys.stderr)
            return 1
        return 0

    try:
        value = int(args.value, 0)
    except ValueError:
        print(f"nxs set: {args.param} takes an integer, not {args.value!r}",
              file=sys.stderr)
        return 1
    if p.get('type') == 'range' and len(p['values']) >= 2:
        ok = p['values'][0] <= value <= p['values'][-1]
    else:
        ok = value in p['values']
    if not ok:
        print(f"nxs set: {value} is outside {args.param}'s values "
              f"{_format_allowed(p)}", file=sys.stderr)
        return 1

    old = p['current']
    t.set_param(args.param, value)
    time.sleep(0.3)
    new = t.get_param(args.param)['current']
    print(f"{args.param}: {old} → {new}")
    return 0

def _print_outputs(t, name: str) -> None:
    """The output fields the loaded personality emits, under `status`."""
    outs = t.read_outputs()
    if not outs:
        # Nothing readable on-device (firmware without the descriptor
        # window); fall back to a local compile of the Python driver.
        raw = _get_output_fields(name, t)
        outs = [{
            'idx': i,
            'name': f['name'],
            'type': f.get('type', 'int16'),
            'byte_order': f.get('byte_order', 'big'),
            'semantic': f.get('semantic', 0),
            'count': f.get('count', 0),
            'scale': f['scale'],
            'offset': f.get('offset', 0.0),
            'unit': f['unit'],
        } for i, f in enumerate(raw)]

    if not outs:
        print("  Outputs:      none")
        return
    total = sample_width(outs)
    per_sample = ("variable per sample" if total is None
                  else f"{total} byte(s) per sample")
    print(f"  Outputs:      {len(outs)} field(s), {per_sample}")
    print(f"    {'#':>2s}  {'name':<12s}  {'type':<7s}  {'order':<6s}  "
          f"{'semantic':<11s}  {'scale':>12s}  {'offset':>10s}  unit")
    for o in outs:
        sem = SEMANTIC_NAMES.get(o.get('semantic', 0),
                                 f"sem{o.get('semantic', 0)}")
        print(f"    {o['idx']:>2d}  {o['name']:<12s}  {o['type']:<7s}  "
              f"{o['byte_order']:<6s}  {sem:<11s}  {o['scale']:>12.6g}  "
              f"{o['offset']:>10.4g}  {o['unit']}")

def cmd_status(t, args):
    ok = t.probe()
    if not ok:
        print(f"NXS @ {_describe_target(args)}: NOT FOUND{_probe_detail(t)}"
              f"{_not_found_hint(args)}")
        return 1

    status = t.read_status()
    state = t.read_vm_state()
    error = t.read_error_code()
    count = t.read_sample_count()
    name = t.read_driver_name()
    store_count = t.read_store_count()
    active_slot = t.read_active_slot()
    runner_state = t.read_runner_state()
    probe_retries = t.read_probe_retries()
    serial = t.read_serial()
    fw = t.read_fw_version()

    vm_state_names = VmStates.VmState._NAMES
    runner_names = RunnerStates.RunnerState._NAMES
    flags = []
    if status & 0x80: flags.append('running')
    if status & 0x01: flags.append('sample_ready')
    if status & 0x02: flags.append('error')

    slot_str = "—" if active_slot == 0xFF else f"#{active_slot}"
    runner_str = runner_names.get(runner_state, f"?({runner_state})")

    # The latched mikroBUS I²C address rides the slot-peek capability
    # (GetDriverInfo over Cyphal, the SEL peek view over I²C).
    i2c_str = None
    session_held = False
    if isinstance(t, SupportsSlotPeek):
        info, session_held = peek_slot(t, ACTIVE_SLOT)
        if info is not None and info.i2c_addr != 0:
            i2c_str = f"0x{info.i2c_addr:02X}"

    print(f"NXS @ {_describe_target(args)}")
    if serial and any(serial):
        print(f"  Serial:       {serial.hex()}")
    if fw:
        print(f"  FW:           {fw}")
    print(f"  Personality:  {name or '(none)'}")
    print(f"  VM:           {vm_state_names.get(state, '?')} "
          f"(0x{status:02X}: {', '.join(flags) or 'idle'})")
    print(f"  Runner:       {runner_str}  "
          f"slot={slot_str}  retries={probe_retries}")
    if i2c_str:
        print(f"  mikroBUS I²C: {i2c_str}")
    if session_held:
        print("  Session:      held (calibration procedure or firmware push)")
    print(f"  Store:        {store_count} populated")
    print(f"  Samples:      {count}")
    if isinstance(t, SupportsTimeSync):
        # Firmware without the time-sync record refuses the mirror
        # read; status stays useful without the line.
        try:
            (offset_us, bound_us, rate_ppb, valid_for_us, source,
             valid) = t.read_time_sync()
        except (RuntimeError, OSError, TimeoutError):
            pass
        else:
            sync_sources = {0: 'none/stale', 1: 'host',
                            2: 'ring-master', 3: 'gnss'}
            if valid:
                src = sync_sources.get(source, f"?({source})")
                print(f"  Sync:         ±{bound_us} µs  "
                      f"(offset {offset_us:+} µs, "
                      f"rate {rate_ppb / 1000:+.0f} ppm, "
                      f"window {valid_for_us / 1_000_000:.0f} s, "
                      f"source {src})")
            else:
                print("  Sync:         -")
    if isinstance(t, SupportsFaultCounters):
        # Firmware without the fault-counter surface refuses the
        # read; status stays useful without the lines.
        overflow_read = getattr(t, "read_cmd_queue_overflow_count", None)
        try:
            io_err = t.read_io_err_count()
            probe_failed = t.read_probe_failed_count()
            drdy = t.read_drdy_coalesced_count()
            rejects = t.read_ingress_reject_count()
            overflows = overflow_read() if overflow_read else None
        except (RuntimeError, OSError, TimeoutError):
            pass
        else:
            # A healthy device stays quiet: the block appears only when a
            # counter is nonzero, so a Faults line always means something.
            if any((io_err, probe_failed, drdy, rejects, overflows)):
                print(f"  Faults:       {io_err} I/O errors absorbed, "
                      f"{probe_failed} probe failures")
                print(f"                {drdy} samples missed (VM busy at DRDY), "
                      f"{rejects} commands rejected (bus contention)")
                if overflows is not None:
                    print(f"                {overflows} writes dropped "
                          f"(device queue full)")
    if error:
        print(f"  Error:        {op_error_name(error)}")
    if name:
        _print_outputs(t, name)
    return 0






def cmd_run(t, args):
    t.vm_run()
    print("VM running.")
    return 0

def cmd_confirm_fw(t, args):
    t.confirm_fw()
    print("Running image confirmed: it survives the next reset.")
    return 0

def cmd_recover(t, args):
    if not isinstance(t, SupportsRecovery):
        print(f"recover: not supported on transport '{args.transport}'", file=sys.stderr)
        return 1
    t.recover()
    print("Device held in MCUboot serial recovery (flash via mcumgr over UART).")
    return 0

def cmd_reboot(t, args):
    if not isinstance(t, SupportsRecovery):
        print(f"reboot: not supported on transport '{args.transport}'", file=sys.stderr)
        return 1
    t.reboot()
    print("Device rebooting.")
    return 0

def cmd_identify(t, args):
    if not isinstance(t, SupportsIdentify):
        print(f"identify: not supported on transport '{args.transport}'",
              file=sys.stderr)
        return 1
    t.identify()
    print(f"Identify: status LED strobing on {_describe_target(args)} (~10 s)")
    return 0

def _bare_timesync(args) -> bool:
    """`nxs timesync` with no unit addressed disciplines the whole bench
    from the manifest (or prints its systemd unit)."""
    return bool(getattr(args, 'systemd', False)) or (
        args.transport == 'i2c' and args.bus is None and args.port is None
        and not getattr(args, 'unit', None) and not args.only_units)

def cmd_timesync(t, args):
    """Refresh the estimator and push the discipline on a fixed cadence. The
    pushed offset maps the device clock into host CLOCK_REALTIME."""
    from nxs.client import DeviceRefused, ERRNO_EBUSY, estimate_and_push

    if not isinstance(t, SupportsTimeSync):
        print("timesync: this transport serves no time surface")
        return 1
    while True:
        try:
            bound = estimate_and_push(t, interval_s=args.interval)
        except DeviceRefused as e:
            if e.code != ERRNO_EBUSY or args.once:
                print(f"timesync: {e}")
                return 1
            # A transfer session holds the mux (a driver upload or a firmware
            # push); the resident pusher skips the interval and lives.
            print("timesync: mux held (transfer in flight) — skipping")
            time.sleep(args.interval)
            continue
        except OSError as e:
            if args.once:
                print(f"timesync: {e}")
                return 1
            # The device NACKs the bus through a firmware push's staging erase
            # and across the reboot; a resident pusher rides that out.
            print(f"timesync: link unavailable ({e}) — skipping")
            time.sleep(args.interval)
            continue
        except RuntimeError as e:
            print(f"timesync: {e}")
            return 1
        if bound is None:
            if args.once:
                print("timesync: no observation (link silent?)")
                return 1
            # A round the unit did not answer: the deaf window of a firmware
            # push's erase or a reboot; a resident pusher rides that out.
            print("timesync: no observation (link silent?) — skipping")
            time.sleep(args.interval)
            continue
        print(f"pushed ±{bound} µs")
        if args.once:
            return 0
        time.sleep(args.interval)

def cmd_commission(t, args):
    if not isinstance(t, SupportsCommissioning):
        print(f"commission: not supported on transport '{args.transport}'", file=sys.stderr)
        return 1
    bitrate = None
    term = None
    try:
        if args.can_bitrate is not None:
            bitrate = device_params.parse_can_bitrate(args.can_bitrate)
        if args.can_term is not None:
            term = device_params.parse_can_term(args.can_term)
    except ValueError as e:
        print(f"commission: {e}", file=sys.stderr)
        return 1
    if args.show or (args.node_id is None and not args.subject and not args.save
                     and bitrate is None and term is None):
        ident = t.read_identity()
        print(f"node-id  {ident['node_addr']}")
        for name, sid in ident['topics'].items():
            print(f"  {name:<18} {sid}")
        if isinstance(t, SupportsBitTiming):
            print(f"  {'can-bitrate':<18} {device_params.bitrate_line(t)}")
        if isinstance(t, SupportsCanTermination):
            print(f"  {'can-term':<18} {'on' if t.read_can_term() else 'off'}")
        return 0
    topics = {}
    for spec in args.subject:
        name, sep, val = spec.partition('=')
        if not sep:
            print(f"commission: --subject expects NAME=ID, got '{spec}'", file=sys.stderr)
            return 1
        try:
            topics[name] = int(val, 0)
        except ValueError:
            print(f"commission: --subject value must be an integer, got '{val}'",
                  file=sys.stderr)
            return 1
    try:
        t.commission(node_addr=args.node_id, topics=topics or None,
                     can_bitrate=bitrate, can_term=term)
    except (ValueError, RuntimeError) as e:
        print(f"commission: {e}", file=sys.stderr)
        return 1
    if args.node_id is not None or topics or bitrate is not None:
        if term is not None:
            print("committed — termination applied live; the rest at the next reboot")
        else:
            print("committed — reboot to apply")
    elif term is not None:
        print("committed — termination applies live")
    else:
        print("saved")
    return 0

def _confirm(question: str, assume_yes: bool, flag: str = '--yes') -> bool:
    """A yes/no at the terminal, default yes; a script passes `flag`."""
    if assume_yes:
        return True
    if not sys.stdin.isatty():
        print(f"{question} — pass {flag} to answer without a terminal",
              file=sys.stderr)
        return False
    return input(f"{question} [Y/n] ").strip().lower() in ("", "y", "yes")

def _is_downgrade(running, pushed_version: str) -> bool:
    """Whether `running` names a version above the image's. An unreadable
    identity, or one that proves no version, is not a downgrade."""
    from nxs.suite.schema import parse_device_version, parse_version
    if running is None:
        return False
    have = parse_device_version(running)
    if have is None:
        return False
    try:
        want = parse_version(pushed_version.split("+", 1)[0])
    except ValueError:
        return False

    return have > want

def cmd_push_fw(t, args):
    import os
    from nxs import mcuboot_image
    from nxs._generated_constants import NxsMcuboot
    from nxs.client import push_and_verify, read_identity
    name = os.path.basename(args.firmware)
    with open(args.firmware, 'rb') as f:
        data = f.read()
    try:
        info = mcuboot_image.parse(data)
    except ValueError as e:
        print(f"  ✗ {name}: {e}", file=sys.stderr)
        return 1
    # The device refuses the first chunk past the slot bound with ENOSPC,
    # but only after a 2.4 s erase and a halted VM; refuse up front.
    if len(data) > NxsMcuboot.UPDATE_IMAGE_MAX_SIZE:
        print(f"  ✗ {name}: {len(data)} bytes is larger than the update slot "
              f"({NxsMcuboot.UPDATE_IMAGE_MAX_SIZE} bytes)", file=sys.stderr)
        return 1
    # The bootloader installs any validly signed image, so a stale wheel would
    # regress a module silently; this is the one warning.
    running = read_identity(t)
    if _is_downgrade(running, info.version):
        print(f"  ! {name} is {info.version}; the module runs {running}")
        if not _confirm("install the older image?", args.allow_downgrade,
                        '--allow-downgrade'):
            print("  push cancelled", file=sys.stderr)
            return 1

    def progress(done, total):
        frac = done / total if total else 0
        w = 24
        n = round(w * frac)
        bar = "━" * n + "─" * (w - n)
        status_line(f"  {name}  {bar} {frac * 100:3.0f}%")

    progress(0, info.total_size)
    try:
        ok, line, _ = push_and_verify(t, args.firmware, info.version,
                                      progress_cb=progress, before=running)
    except Exception as e:
        print(f"\n  ✗ {e}")
        return 1
    print()
    print(f"  {line}", file=None if ok else sys.stderr)
    return 0 if ok else 1


def add_unit_parsers(sub) -> None:
    """Register the verbs addressed at one unit, from `probe` to `set`."""
    p_probe = sub.add_parser('probe', help='Check if the module is present')
    p_probe.add_argument('--json', action='store_true',
                         help='machine-readable discovery (bare form only)')
    p_status = sub.add_parser('status',
                              help='Bare: the declaration against the rig, '
                                   'node by node; addressed: one unit\'s '
                                   'state, personality and outputs')
    p_status.add_argument('--json', action='store_true',
                          help='machine-readable tree (bare form only)')

    p_upload = sub.add_parser('upload', help='Compile and upload a personality of '
                                             'either kind by name or path')
    p_upload.add_argument('driver',
                          help='Personality name (iam20680) — the preferred form; '
                               'a compiled .nxs path is accepted but must '
                               'match this tool\'s image format')
    p_upload.add_argument('-o', '--output', metavar='FILE',
                          help='Compile to this .nxs file instead of '
                               'uploading (no device needed) — the producer '
                               'for yakut file-server fleet provisioning')
    p_upload.add_argument('--param', '--config', '-c', nargs='*',
                          metavar='KEY=VALUE', dest='config',
                          help='Driver params (e.g. sample_rate=250 accel_fs=8). '
                               '`--param` and `--config` are aliases.')
    p_upload.add_argument('--slot', type=int, default=None,
                          help='The store slot a camera personality lands in '
                               '(default: its own, else the first empty)')

    sub.add_parser('caps', help="Show the loaded personality's parameters and limits")
    sub.add_parser('run', help='Run the loaded personality again')
    sub.add_parser('recover',
                   help='Reboot into MCUboot serial recovery and hold there')
    sub.add_parser('reboot',
                   help='Cold-reset the device; a staged update swaps in, an '
                        'unconfirmed image reverts')
    sub.add_parser('confirm-fw',
                   help='Confirm the running firmware image so it survives '
                        'the next reset (supervised builds)')
    sub.add_parser('identify',
                   help='Strobe the status LED (~10 s) to physically locate '
                        'the unit')
    p_push = sub.add_parser('push-fw',
                            help='Upload firmware via the app-resident DFU '
                                 'core')
    p_push.add_argument('firmware', help='Path to signed firmware binary')
    p_push.add_argument('--allow-downgrade', action='store_true',
                        help='Install an image older than the one the module '
                             'runs without asking. The bootloader has no '
                             'downgrade check, so re-serving an older release '
                             'is the rollback; this is how a script takes it')

    p_get = sub.add_parser('get', help="Read a parameter: the personality's "
                                       "(sample_rate, accel_fs, …) or the "
                                       "device's (" + ", ".join(device_params.names()) + ")")
    p_get.add_argument('param', help='Parameter name')

    p_set = sub.add_parser('set', help="Set a parameter of either family")
    p_set.add_argument('param', help='Parameter name')
    p_set.add_argument('value', help='New value')


def add_timesync_parser(sub) -> None:
    """Register `nxs timesync`."""
    from nxs.client import PUSH_INTERVAL_S

    p_ts = sub.add_parser('timesync',
                          help='Push the host time discipline: bare, to every '
                               'unit the manifest declares (a resident '
                               'pusher); addressed, to that one unit')
    p_ts.add_argument('--interval', type=float, default=PUSH_INTERVAL_S,
                      help='Seconds between pushes (default 1)')
    p_ts.add_argument('--once', action='store_true',
                      help='Push once and exit')
    p_ts.add_argument('--unit', dest='only_units', action='append',
                      default=None,
                      help='Bare form: discipline only the named unit '
                           '(repeatable)')
    p_ts.add_argument('--systemd', action='store_true',
                      help='Print a systemd unit running the bare pusher '
                           'resident, then exit')


def add_commission_parser(sub) -> None:
    """Register `nxs commission`."""
    p_comm = sub.add_parser('commission',
                            help='Read/set the persisted node-ID and subject-IDs')
    p_comm.add_argument('--node-id', type=int, default=None,
                        help='Set the Cyphal node-ID (0-125, 255 for anonymous, or 0xFFFF '
                             'to revert to the default); applies on reboot')
    p_comm.add_argument('--subject', action='append', metavar='NAME=ID', default=[],
                        help='Set a subject-ID, e.g. acceleration=6246 (0 disables, '
                             '0xFFFF reverts to the default; repeatable)')
    p_comm.add_argument('--can-bitrate', metavar='NOM[/DATA]', default=None,
                        help='Set the CAN bit-timing profile in bit/s (bare NOM = Classic '
                             'CAN; 0 reverts to the default); applies on reboot')
    p_comm.add_argument('--can-term', metavar='on|off|default', default=None,
                        help='Set the on-board CAN split termination (default reverts '
                             'to off); applies live')
    p_comm.add_argument('--save', action='store_true',
                        help='Commit the running config (e.g. a decimation change) without changing identity')
    p_comm.add_argument('--show', action='store_true',
                        help='Print the current identity and exit')
