#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
nxs — command-line tool for NXS sensor VM.

Usage:
    nxs probe
    nxs upload <driver|file.nxs> [--param k=v ...] [-o FILE]
    nxs caps
    nxs get <param>
    nxs set <param> <value>
    nxs stream [--count N] [--raw]
    nxs ros2 [--plan] [--stamp synced|device|arrival|itow] [--launch-file]
    nxs status

Options:
    -t, --transport      i2c | cyphal-serial | cyphal-can
                         ($NXS_TRANSPORT, else i2c on Linux / cyphal-serial on macOS, Windows)
    -b, --bus BUS        I2C bus device ($NXS_BUS, else /dev/i2c-2)
    -p, --port PORT      Serial device / SocketCAN iface ($NXS_PORT, else autodetect)
    --baud BAUD          Serial baud rate (default: 460800)
    -a, --addr ADDR      NXS device address (default: 0x30 )
    --version            Print tool version
"""

import argparse
import collections
import importlib
import importlib.util
import os
import struct
import sys
import time

from nxs._generated_constants import CyphalDefaults, NxsDevices, RunnerStates
from nxs.client import (
    ACTIVE_SLOT, ERRNO_EEXIST, PUSH_INTERVAL_S, DeviceRefused,
    SupportsBitTiming, SupportsCalibration, SupportsCanTermination,
    SupportsCommissioning, SupportsEgressDecimation, SupportsFaultCounters,
    SupportsIdentify, SupportsRecovery, SupportsSlotPeek, SupportsTimeSync,
    ROTATION_NAMES, rotation_code, rotation_name)
from nxs.transports import open_client
from nxs.compiler import CompileError, SEMANTIC_NAMES
from nxs.stamp_modes import STAMP_ITOW, STAMP_MODES, STAMP_SYNCED
from nxs.image import NXS_MAJOR, NXS_MINOR, peek_format, serialize
from nxs.descriptor import (
    effective_scale_fields, is_decodable, parse_sample, sample_width)
from nxs.serial_util import autodetect_serial_port


class _RateMeter:
    """Rolling-window sample-rate meter for the `stream` command's
    `[xx.x Hz]` display. The cumulative-since-start average is wrong
    for bursty sources like NMEA GNSS — the displayed rate climbs
    during each burst and drops during idle gaps because the
    denominator keeps growing while the numerator stalls. A fixed
    1-second sliding window converges to the steady-state rate within
    one second and stays there, matching the operator's intuition of
    "samples per second" without saw-toothing.
    """

    def __init__(self, window_s: float = 1.0):
        self._window_s = window_s
        self._times: collections.deque = collections.deque()

    def tick(self, now: float, count: int = 1) -> float:
        """Record `count` samples that arrived at `now`; return the
        current rolling rate in Hz. For batched receivers (e.g. the
        I²C poll that catches `delta > 1` new samples in a poll
        interval), all `count` samples are stamped at `now` — close
        enough at the window's resolution and avoids reconstructing
        past timestamps the transport never reported."""
        for _ in range(count):
            self._times.append(now)
        while self._times and now - self._times[0] > self._window_s:
            self._times.popleft()
        return len(self._times) / self._window_s


# Human display units for the stream table, keyed by a field's declared
# canonical unit. Render-time only: the wire, the descriptors, the SI
# subjects, and every machine consumer stay SI — this converts the last
# inch for eyes on a terminal. Seeded with the one unit that actively
# fights a bench reader; a future candidate (tesla → µT) is one entry.
DISPLAY_UNITS = {
    'kelvin': ('°C', lambda v: v - 273.15),
}


def _display_unit(unit, human):
    """The unit label the stream header shows for `unit`."""
    if human and unit in DISPLAY_UNITS:
        return DISPLAY_UNITS[unit][0]
    return unit


def _display_value(v, unit, human):
    """`v` converted to its display unit; strings and SI mode pass through."""
    if human and not isinstance(v, str) and unit in DISPLAY_UNITS:
        return DISPLAY_UNITS[unit][1](v)
    return v


def _format_sample_columns(values, output_fields, human=False):
    """Format a parsed sample's fields into a single line. String fields
    print as their decoded text with trailing CR/LF stripped (so NMEA
    frames that end in `\\r\\n` don't insert a blank line per sample).
    Numeric fields print at fixed `10.4g` width — 4 significant digits
    with exponent fallback, so sub-milli SI values (tesla at Earth-field
    magnitude) stay visible instead of rounding to 0.0000. `human`
    applies the `DISPLAY_UNITS` conversions (the header converts its
    labels with the same table, so a converted value never sits under
    an SI label)."""
    parts = []
    for f in output_fields:
        v = _display_value(values.get(f['name'], 0), f.get('unit', ''), human)
        if isinstance(v, str):
            parts.append(v.rstrip('\r\n'))
        else:
            parts.append(f"{v:10.4g}")
    return '  '.join(parts)


def _default_can_mtu():
    """$NXS_CAN_MTU as the --mtu default — argparse never checks a default
    against choices, so an out-of-vocabulary value is dropped loudly here
    instead of riding silently into the CAN transport."""
    raw = os.environ.get('NXS_CAN_MTU', '64')
    try:
        value = int(raw, 0)
    except ValueError:
        value = None
    if value not in (8, 64):
        print(f"nxs: ignoring NXS_CAN_MTU={raw!r} (must be 8 or 64); using 64",
              file=sys.stderr)
        return 64
    return value


def _default_transport():
    """Transport used when -t is omitted: $NXS_TRANSPORT if set, else the
    per-platform default."""
    env = os.environ.get('NXS_TRANSPORT')
    if env:
        return env.strip().lower()
    if sys.platform == 'darwin' or sys.platform == 'win32':
        return 'cyphal-serial'
    return 'i2c'


def _manifest_unit(unit_name: str):
    """Resolve a unit from the suite manifest, for `--unit` addressing.
    Exits with a clear message on any miss."""
    import os

    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config

    path = default_config_path()
    if not os.path.exists(path):
        raise SystemExit(f"nxs: --unit needs a suite manifest; none at {path}")
    try:
        cfg = load_suite_config(path)
    except (ManifestError, OSError) as e:
        raise SystemExit(f"nxs: --unit: {e}") from None
    for unit in cfg.units:
        if unit.name == unit_name:
            return unit
    raise SystemExit(f"nxs: no unit named {unit_name!r} in {path}")


def _pick_unit_link(unit):
    """The unit's management link: the first whose device answers a
    probe, in declared order (returned with its open transport). When
    nothing answers, fall back to the first link blind with no
    transport — the dispatch-level reachability check owns the
    messaging, and recovery verbs must reach devices that don't probe."""
    for link in unit.links:
        transport = None
        try:
            transport = open_client(link.transport, **link.client_kwargs())
            if transport.probe():
                return link, transport
        except Exception:
            pass
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass
    return unit.links[0], None


def _apply_unit_link(args, link):
    """Sync the display/decision args to a manifest-resolved `--unit`
    link, so a command that prints its target (`_describe_target`)
    shows the real link, not the default flags it ignored."""
    args.transport = link.transport
    if link.transport == 'i2c':
        args.bus, args.addr = link.bus, link.address
    elif link.transport == 'cyphal-can':
        args.port, args.remote_node_id = link.iface, link.node_id
    elif link.transport == 'cyphal-serial':
        args.port = link.port
        if link.baud is not None:
            args.baud = link.baud


def _describe_target(args) -> str:
    """The command's target in the link vocabulary (`i2c /dev/i2c-9@0x30`,
    `can can0 node 125`, `serial /dev/ttyUSB0`). Printing `args.addr`
    alone would misreport the Cyphal transports, which ignore it."""
    if args.transport == 'i2c':
        return f"i2c {args.bus}@0x{args.addr:02X}"
    if args.transport == 'cyphal-can':
        node = (args.remote_node_id if args.remote_node_id is not None
                else CyphalDefaults.DEFAULT_NODE_ID)
        return f"can {args.port or 'can0'} node {node}"
    if args.transport == 'cyphal-serial':
        return f"serial {args.port}"
    return args.transport


def _open_transport(args):
    """Build a transport object from --transport and its flags — or,
    with `--unit NAME`, from the named unit's manifest link. Resolves
    a missing `--port` via `autodetect_serial_port()` on the way through
    so the daily `nxs -t cyphal-serial …` form works on any host that
    exposes exactly one USB-UART bridge."""
    if getattr(args, 'unit', None):
        link, transport = _pick_unit_link(_manifest_unit(args.unit))
        _apply_unit_link(args, link)
        if transport is not None:
            return transport
        return open_client(link.transport, **link.client_kwargs())
    # The transport→class table lives in open_client(); this assembles the
    # CLI-derived kwargs and delegates, so transports are added in one place.
    if args.transport == 'i2c':
        return open_client('i2c', bus=args.bus, address=args.addr)
    # A node-ID of None lets each Cyphal client fall back to its own default;
    # only forward an explicit --remote-node-id so the kwarg stays absent when
    # unset (i2c ignores it).
    cyphal_kwargs = {}
    if getattr(args, 'remote_node_id', None) is not None:
        cyphal_kwargs['remote_node_id'] = args.remote_node_id
    if args.transport == 'cyphal-serial':
        if not args.port:
            args.port = autodetect_serial_port()
            if not args.port:
                raise SystemExit(
                    "no serial port: autodetect found zero or many — "
                    "pass --port explicitly.")
        return open_client('cyphal-serial', port=args.port, baud=args.baud,
                           **cyphal_kwargs)
    if args.transport == 'cyphal-can':
        # --port carries the SocketCAN interface name for CAN (e.g. can0).
        return open_client('cyphal-can', can_iface=(args.port or 'can0'),
                           can_mtu=args.mtu, **cyphal_kwargs)
    raise ValueError("unknown transport")


def _is_mutating(args) -> bool:
    """True for verbs that change device state — the read forms of
    dual-mode verbs (bare `decimation`, `commission --show`, `store ls`)
    stay silent so the suite-managed note never becomes noise."""
    if args.command in ('upload', 'set', 'run', 'stop', 'reset',
                        'push-fw', 'recover'):
        return True
    if args.command == 'store':
        return args.store_cmd != 'ls'
    if args.command == 'decimation':
        return args.value is not None
    if args.command == 'commission':
        return not args.show and (args.node_id is not None
                                  or bool(args.subject) or args.save)
    return False


def _require_supported_version(t):
    """Exit loudly if the device's register-map contract differs from the one
    this tool speaks. I2C serves PROTO_VERSION; the Cyphal transports return
    None (no such register) and are skipped. A read failure is left to the
    command itself to diagnose."""
    from nxs.client import contract_mismatch
    mm = contract_mismatch(t)
    if mm is not None:
        sys.exit(f"nxs: {mm} — use a matching nxs (`nxs probe` for details).")


def _warn_if_suite_managed(args):
    """One stderr note when a flag-addressed mutating verb targets a
    declared unit. Best-effort: a missing or broken manifest never
    blocks the manual path."""
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
                  f"reverts on `nxs suite switch`; keep it with "
                  f"`nxs suite freeze --unit {unit.name}`", file=sys.stderr)
            return


# The verbs that must reach an off-contract device: `probe` reports the
# version, `push-fw` and `recover` carry the firmware that ends the mismatch.
VERSION_GATE_EXEMPT = ('probe', 'push-fw', 'recover')


def _probe_detail(t) -> str:
    """Trailing " — reason" when the transport knows why the probe found
    nothing; empty when it does not."""
    getter = getattr(t, "probe_failure_detail", None)
    detail = getter() if getter is not None else None
    return f" — {detail}" if detail else ""


def cmd_probe(t, args):
    ok = t.probe()
    print(f"NXS @ {_describe_target(args)}: "
          f"{'found' if ok else 'NOT FOUND' + _probe_detail(t)}")
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
    return 0 if ok else 1


def _upload_and_run(t, img):
    from nxs.client import await_driver_up

    t.upload_image(img)
    t.vm_run()
    # Store + sensor probe finish asynchronously after RUN. Waiting on the
    # runner state rather than on a driver name distinguishes "up" from
    # "loaded but the sensor never answered" — the name is set at load, so it
    # reports success on a driver that is about to fail its probe.
    state = RunnerStates.RunnerState
    settled = await_driver_up(t)
    if settled == state.MEASURING:
        print("Uploaded and running.")
        return 0
    if settled == state.PROBE_FAILED:
        print("Uploaded, but no sensor answered the driver — check the wiring "
              "and the driver's bus/address params.", file=sys.stderr)
        return 1
    print(f"Uploaded, but the driver did not come up (runner is "
          f"{state._NAMES.get(settled, settled)}) — check the device log.",
          file=sys.stderr)
    return 1


def cmd_upload(t, args):
    src = args.driver

    # (a) A local .py is a driver source: compile it exactly like the
    #     driver-name branch, honoring --param / -o (the Driver
    #     Development Guide's authoring flow).
    if os.path.isfile(src) and src.endswith('.py'):
        label = os.path.splitext(os.path.basename(src))[0]
        spec = importlib.util.spec_from_file_location(label, src)
        mod = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(mod)
        except Exception as e:
            from nxs.client import import_failure_detail
            print(f"error: {src} failed to import: "
                  f"{import_failure_detail(e, src)}", file=sys.stderr)
            return 1
        return _compile_from_module(t, mod, label, args)

    # (b) Any other existing file is a compiled .nxs image — upload it as
    #     is. Its params are baked in, so --param / -o don't apply.
    if os.path.isfile(src):
        if args.config:
            print("error: --param applies only when compiling a driver from "
                  "its .py; a compiled .nxs has its params baked in",
                  file=sys.stderr)
            return 1
        if args.output:
            print(f"error: {src} is already compiled; -o compiles a driver "
                  f"from its .py", file=sys.stderr)
            return 1
        with open(src, 'rb') as f:
            img = f.read()
        name = os.path.splitext(os.path.basename(src))[0]
        try:
            major, minor = peek_format(img)
        except ValueError:
            print(f"error: {src} is not an NXS image — upload the driver "
                  f"name instead: nxs upload {name}", file=sys.stderr)
            return 1
        # The tool is the compiler, so its format constants are the paired
        # reference; a stale artifact is refused with its rebuild command.
        if major != NXS_MAJOR or minor > NXS_MINOR:
            print(f"error: image format {major}.{minor}, this tool builds "
                  f"{NXS_MAJOR}.{NXS_MINOR} — rebuild it: nxs upload {name}",
                  file=sys.stderr)
            return 1
        print(f"Uploading {src}: {len(img)}B image")
        return _upload_and_run(t, img)

    # (c) Otherwise treat src as a driver name and compile drivers/<name>.py.
    driver_name = src.lower()
    try:
        mod = importlib.import_module(f"nxs.drivers.{driver_name}")
    except ModuleNotFoundError:
        print(f"No driver or file '{src}'. Generate a driver with the NXS "
              f"driver skill (writes drivers/{driver_name}.py), or pass a "
              f"path to a compiled .nxs.", file=sys.stderr)
        return 1
    return _compile_from_module(t, mod, driver_name, args)


def _compile_from_module(t, mod, label, args):
    """Compile the SensorDriver found in `mod` and upload (or, with -o,
    write) the image — the shared tail of the driver-name and local-.py
    upload branches."""
    # Find SensorDriver subclass
    from nxs.compiler import SensorDriver
    drv_cls = None
    for attr in dir(mod):
        obj = getattr(mod, attr)
        if (isinstance(obj, type) and issubclass(obj, SensorDriver)
                and obj is not SensorDriver
                and obj.__module__ == mod.__name__):
            drv_cls = obj
            break
    if drv_cls is None:
        print(f"No SensorDriver found in {label}")
        return 1

    # Build config from --config args
    config = {}
    for kv in (args.config or []):
        if '=' not in kv:
            print(f"Invalid config: {kv} (expected key=value)")
            return 1
        k, v = kv.split('=', 1)
        try:
            config[k] = int(v)
        except ValueError:
            config[k] = v

    # Capture user-supplied keys before compile (driver may mutate config).
    user_supplied = set(config.keys())

    try:
        compiled = drv_cls().compile(config)
    except CompileError as e:
        # Driver rejected the config — user-facing error, not a bug.
        # Print without the Python traceback and exit cleanly.
        print(f"Error: {e}", file=sys.stderr)
        return 1

    # Allowed keys: declared params, plus `sample_rate` and `trigger` for
    # drivers that read them without declaring (legacy / from_config loops).
    allowed_keys = {p.name for p in compiled.params}
    allowed_keys.add('sample_rate')

    measure_fn = None
    for attr_name in dir(drv_cls):
        fn = getattr(drv_cls, attr_name)
        if callable(fn) and getattr(fn, '_measure_loop', False):
            measure_fn = fn
            break
    if measure_fn is not None and getattr(measure_fn, '_trigger', '') == 'from_config':
        allowed_keys.add('trigger')

    unknown = user_supplied - allowed_keys
    if unknown:
        print(f"Unknown config keys: {', '.join(sorted(unknown))}")
        print(f"Valid: {', '.join(sorted(allowed_keys))}")
        return 1

    try:
        img = serialize(compiled)
    except CompileError as e:
        # The compiled driver exceeds an image cap (params/outputs/values/bus
        # profiles) — user-facing, print without a traceback.
        print(f"Error: {e}", file=sys.stderr)
        return 1
    print(f"Compiled {compiled.name}: {len(compiled.bytecode)}B bytecode, "
          f"{len(compiled.params)} params → {len(img)}B image")

    # -o writes the compiled image to a file instead of the device: the
    # producer for `yakut file-server`-style fleet provisioning. No device.
    if args.output:
        with open(args.output, 'wb') as f:
            f.write(img)
        print(f"Wrote {args.output}: {len(img)}B image")
        return 0

    return _upload_and_run(t, img)


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
        print("No driver loaded.")
        return 1

    print(f"Driver: {name}")
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


def cmd_get(t, args):
    try:
        p = t.get_param(args.param)
    except KeyError:
        print(f"Unknown parameter: {args.param}")
        return 1

    print(f"{p['name']} = {p['current']}  "
          f"(valid: {_format_allowed(p)}  default: {p['default']}  "
          f"unit: {p['unit']})")
    return 0


def cmd_set(t, args):
    try:
        p = t.get_param(args.param)
    except KeyError:
        print(f"Unknown parameter: {args.param}")
        return 1

    value = int(args.value)
    if p.get('type') == 'range' and len(p['values']) >= 2:
        ok = p['values'][0] <= value <= p['values'][-1]
    else:
        ok = value in p['values']
    if not ok:
        print(f"Invalid value {value} for {args.param}. Valid: {_format_allowed(p)}")
        return 1

    old = p['current']
    t.set_param(args.param, value)
    time.sleep(0.3)
    new = t.get_param(args.param)['current']
    print(f"{args.param}: {old} → {new}")
    return 0


def _looks_textual(data: bytes) -> bool:
    """True if every byte is printable ASCII or tab/CR/LF — text-shaped
    payloads (NMEA, AT lines) render readably; binary falls back to hex."""
    return all(b in (0x09, 0x0A, 0x0D) or 0x20 <= b <= 0x7E for b in data)


def cmd_stream(t, args):
    """Stream samples over any transport. The SDK yields decoded
    `Sample`s via iter_samples(); this renders them the same way
    whether they arrived by I²C poll or serial push."""
    # read_driver_name's first GetDriverInfo transfer can drop on a fresh link;
    # retry so a flaky round-trip doesn't refuse a running device.
    name = ""
    for attempt in range(3):
        name = t.read_driver_name()
        if name:
            break
        if attempt < 2:
            time.sleep(0.2)
    cal_record = None
    if isinstance(t, SupportsCalibration):
        try:
            cal_record = t.read_calibration()
        except (RuntimeError, OSError, TimeoutError):
            pass
    driver_name = name or args.driver
    if not driver_name:
        print("No driver loaded.")
        return 1
    sample_size = t.read_sample_size()
    every_nth = _configure_stream(t, args)

    # The SDK decodes from the device's own descriptors; keep a local-compiled
    # set for the column header and as a fallback when the device serves none.
    device_fields = t.read_outputs()
    local_fields = []
    if not device_fields:
        local_fields = (_get_output_fields(name, t) if name
                        else _get_output_fields_by_name(args.driver))
    fields = device_fields or local_fields
    if fields and not is_decodable(fields):
        print("warning: driver declares an output type this nxs "
              "version can't decode; streaming raw bytes — update nxs",
              file=sys.stderr)
        fields = []
    if fields:
        derived = sample_width(fields)
        # `sample_width()` is the furthest declared field end — a lower
        # bound, not an exact size. A binary-record sample may carry
        # undeclared trailing bytes (a frame checksum/reserved tail), so a
        # device SAMPLE_SIZE above `derived` is legal; only fields that
        # overflow the sample (derived > sample_size) can't be decoded. A
        # string field's count is instead a CEILING, so for those a device
        # size ABOVE the ceiling is the inconsistent case.
        has_string = any(f.get('type') == 'string' for f in fields)
        mismatch = derived is not None and (
                (sample_size > derived) if has_string else (derived > sample_size))
        if mismatch:
            cause = ("on-device descriptors are inconsistent — firmware bug, "
                     "corruption, or an unsupported proto version"
                     if device_fields else
                     "the local driver file looks stale or wrong for this image")
            print(f"warning: output descriptors sum to {derived} B/sample "
                  f"but the device reports SAMPLE_SIZE={sample_size}; "
                  f"decoded values may be wrong ({cause})", file=sys.stderr)

    human = args.units != 'si'
    # Width of the per-row `[ rate Hz] n=... ts=...` prefix, shared with the
    # header pad so the columns stay aligned: fits `[9999.0 Hz] n=65535` plus
    # a 12-digit microsecond timestamp.
    prefix_w = 36
    show_cols = bool(fields) and not args.raw and not args.quiet
    if show_cols:
        header = "  ".join(f"{f['name']:>10s}" for f in fields)
        units = "  ".join(
            f"{'(' + _display_unit(f.get('unit', ''), human) + ')':>10s}"
            for f in fields)
        print(f"Streaming {driver_name} — {len(fields)} fields, "
              f"{sample_size}B/sample, every_nth={every_nth} (ctrl-C to stop)\n")
        print(f"{'':>{prefix_w}s}  {header}")
        print(f"{'':>{prefix_w}s}  {units}")
    else:
        print(f"Streaming {driver_name} (sample_size={sample_size}, "
              f"every_nth={every_nth}, ctrl-C to stop)\n")

    count = args.count
    samples = 0
    t0 = time.time()
    meter = _RateMeter()
    prev_seq = first_seq = window_seq0 = None
    restarts = 0
    QUIET_WINDOW_S = 0.5
    next_print = t0 + QUIET_WINDOW_S
    window_start, window_samples = t0, 0

    try:
        t.start_stream(every_nth)
        for s in t.iter_samples(timeout=1.0):
            now = time.time()
            # A VM restart (slot reload after an error) resets the seq
            # counter mid-stream; reset the window stats across it.
            if _looks_like_vm_restart(prev_seq, s.count):
                restarts += 1
                first_seq = window_seq0 = s.count
                window_samples, window_start = 0, now
                if args.quiet:
                    print(f"\n(VM restart #{restarts} detected — stats reset)")
            prev_seq = s.count
            samples += 1
            if first_seq is None:
                first_seq = s.count
            rolling = meter.tick(now)

            if args.quiet:
                if window_seq0 is None:
                    window_seq0 = s.count
                window_samples += 1
                if now >= next_print:
                    win_dt = now - window_start
                    inst = window_samples / win_dt if win_dt > 0 else 0
                    sensor_rate = ((s.count - window_seq0) & 0xFFFF) / win_dt \
                        if win_dt > 0 else 0
                    elapsed = now - t0
                    avg = samples / elapsed if elapsed > 0 else 0
                    # Sensor produced this many more than we received. Under host
                    # pacing (I²C poll, --hz decimation) most is skip, not loss.
                    skipped = max(0, ((s.count - first_seq) & 0xFFFF) - samples + 1)
                    print(f"\rrx={samples:6d}  rate={inst:6.1f} Hz  "
                          f"sensor={sensor_rate:6.1f} Hz  avg={avg:6.1f} Hz  "
                          f"skipped={skipped:4d}  restarts={restarts:2d}",
                          end="", flush=True)
                    window_start, window_samples, window_seq0 = now, 0, s.count
                    next_print = now + QUIET_WINDOW_S
            else:
                ts = f" ts={s.timestamp_us}" if s.timestamp_us is not None else ""
                prefix = f"[{rolling:6.1f} Hz] n={s.count:5d}{ts}"
                if args.raw:
                    print(f"{prefix}  {' '.join(f'{b:02X}' for b in s.raw)}")
                elif not fields:
                    if _looks_textual(s.raw):
                        print(f"{prefix} len={len(s.raw)}  "
                              f"{s.raw.decode('ascii').rstrip(chr(13)+chr(10))}")
                    else:
                        print(f"{prefix} len={len(s.raw)}  "
                              f"{' '.join(f'{b:02X}' for b in s.raw)}")
                else:
                    values = s.values or (parse_sample(s.raw, local_fields,
                                                       cal_record, name)
                                          if local_fields else {})
                    print(f"{prefix:<{prefix_w}s}  "
                          f"{_format_sample_columns(values, fields, human)}")
            if count is not None and samples >= count:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if args.quiet:
            print()
        try:
            t.stop_stream()
        except Exception:
            pass

    elapsed = time.time() - t0
    avg = samples / elapsed if elapsed > 0 else 0
    print(f"\nStopped after {samples} samples in {elapsed:.1f}s ({avg:.1f} Hz avg)")
    return 0


def _looks_like_vm_restart(prev_seq, curr_seq, threshold: int = 1000) -> bool:
    """Heuristic: the VM's u16 sample seq counter is normally monotonic
    with natural wraparound at 65535 → 0 (raw masked delta = 1). A VM
    restart resets seq to a low value, which masked arithmetic instead
    sees as a near-65535 delta — `(0 - 237) & 0xFFFF = 65299`. Anything
    above the threshold is implausible as a per-sample gap (at 1 kHz
    acq, 1000 samples = 1 second of inter-sample silence the wire
    can't produce silently) and is interpreted as a restart so the
    meter resets its stats instead of reporting nonsense rates.
    Returns False when there's no previous seq to compare against."""
    if prev_seq is None:
        return False
    raw_delta = (curr_seq - prev_seq) & 0xFFFF
    return raw_delta > threshold


def _configure_stream(t, args) -> int:
    """Apply --every/--hz to the transport and return the egress
    `every_nth` (1 = none) for start_stream().

    --hz N: target output rate. On a transport that decimates on-device →
    every_nth = round(acq_rate/N); a host-paced transport (I²C) sets its
    poll rate instead and egress stays 1.
    Omitted: every sample at the transport's default cadence.
    """
    if args.hz is not None and args.hz <= 0:
        raise SystemExit("--hz must be a positive rate (Hz)")

    if args.hz is not None:
        if isinstance(t, SupportsEgressDecimation):
            return _every_for_hz(t, args.hz)
        t.set_output_rate(args.hz)  # host-paced poll rate (e.g. I²C)
        return 1

    return 1


def _every_for_hz(t, hz: int) -> int:
    """Egress decimation factor for a target rate on a device that
    decimates on-device: every_nth = round(acq_rate/hz), rounded to a
    divisor of the acquisition rate (operator sees the actual rate)."""
    try:
        p = t.get_param("sample_rate")
    except KeyError:
        raise SystemExit(
            "--hz: driver doesn't declare a 'sample_rate' param; use --every")
    current = p.get("current")
    acq_rate = int(current if current is not None else p.get("default", 0))
    if acq_rate <= 0:
        raise SystemExit("--hz: driver's sample_rate is unset; use --every")
    every = max(1, round(acq_rate / hz))
    actual = acq_rate / every
    if abs(actual - hz) > 0.5:
        print(f"--hz {hz}: driver acq_rate={acq_rate} Hz, every_nth={every}, "
              f"actual output={actual:.1f} Hz "
              f"(decimation can only deliver acq_rate / integer)")
    return every


_DEVICE_FLAGS = {'-t', '--transport', '-b', '--bus', '-p', '--port',
                 '-a', '--addr', '--baud', '--remote-node-id'}


def _device_flags_given(argv=None) -> bool:
    """True when the command line addresses a device explicitly — those
    flags select ad-hoc single-device mode even when a manifest exists."""
    argv = sys.argv[1:] if argv is None else argv
    return any(a.split('=', 1)[0] in _DEVICE_FLAGS for a in argv)


def _allocate_can_local_ids(targets):
    """Give each additional cyphal-can client on a shared iface its own
    local node-ID (every client is a Cyphal node of its own; two nodes
    claiming one ID on one bus collide). The first client on an iface
    keeps the stock HOST_NODE_ID; later ones count down from
    HOST_NODE_ID-1, skipping IDs the manifest assigns to units."""
    used = {kw.get('remote_node_id') for _, tr, kw in targets
            if tr == 'cyphal-can'}
    seen = set()
    next_id = CyphalDefaults.HOST_NODE_ID - 1
    for _, tr, kw in targets:
        if tr != 'cyphal-can':
            continue
        iface = kw.get('can_iface')
        if iface in seen:
            while next_id in used:
                next_id -= 1
            if next_id <= 0:
                raise SystemExit("nxs ros2: no free host node-IDs left "
                                 "for a shared CAN iface")
            kw['local_node_id'] = next_id
            used.add(next_id)
        seen.add(iface)
    return targets


def _ros2_targets(args, argv=None):
    """Resolve what `ros2` bridges: (suite_mode, [(name, transport,
    kwargs)]). A None transport means "open via the global flags". A
    unit is bridged over its primary (first-declared) link; a down link
    is skipped by the probe in `cmd_ros2`, not failed over here — a
    long-running publisher binds one route per unit."""
    if getattr(args, 'unit', None):
        link = _manifest_unit(args.unit).links[0]
        return False, [(args.unit, link.transport, link.client_kwargs())]
    from nxs.suite import default_config_path
    path = default_config_path()
    if os.path.exists(path) and not _device_flags_given(argv):
        from nxs.suite.schema import ManifestError, load_suite_config
        try:
            cfg = load_suite_config(path)
        except (ManifestError, OSError) as e:
            raise SystemExit(f"nxs ros2: {e}") from None
        if not cfg.units:
            raise SystemExit(f"nxs ros2: no units in {path}")
        targets = [(u.name, u.links[0].transport, u.links[0].client_kwargs())
                   for u in cfg.units]
        return True, _allocate_can_local_ids(targets)
    return False, [(None, None, None)]


def _ros2_lock_tokens(args, targets):
    """One run-lock token per bridged target. Suite and --unit targets
    key on their resolved link kwargs; an ad-hoc target keys on the
    global device flags, so runs against different devices never
    collide."""
    adhoc = [(f, getattr(args, f, None))
             for f in ('transport', 'port', 'bus', 'addr',
                       'remote_node_id')]
    return [f"{tr or 'flags'}:{sorted(kw.items()) if kw else adhoc}"
            for _, tr, kw in targets]


def cmd_ros2(args, opener=open_client, argv=None):
    """Bridge decoded samples onto ROS 2 topics: the whole suite by
    default, one unit with --unit, or an ad-hoc flag-addressed device."""
    from nxs.ros2_bridge import (
        Ros2Bridge, UnitPlan, acquire_run_lock, epoch_binding, format_plan,
        join_topic, load_map, plan_publications, run_bridge)

    if getattr(args, 'launch_file', False):
        print(os.path.join(os.path.dirname(__file__), 'ros2',
                           'bridge.launch.py'))
        return 0

    suite_mode, targets = _ros2_targets(args, argv)
    if not getattr(args, 'plan', False):
        run_lock = acquire_run_lock(  # noqa: F841 — held for process lifetime
            _ros2_lock_tokens(args, targets))
    if args.map and suite_mode:
        raise SystemExit("nxs ros2: --map applies to a single device — "
                         "use --unit or the transport flags")
    if args.frame_id and suite_mode:
        raise SystemExit("nxs ros2: suite units take their frame_id from "
                         "the manifest name; --frame-id needs --unit or "
                         "the transport flags")

    def refuse(label, reason, client=None):
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        if suite_mode:
            print(f"nxs ros2: {label}: {reason} — skipped", file=sys.stderr)
            return
        raise SystemExit(f"nxs ros2: {label}: {reason}")

    clients, plans = [], []
    for name, transport, kwargs in targets:
        label = name or 'device'
        try:
            client = _open_transport(args) if transport is None \
                else opener(transport, **kwargs)
        except (OSError, ValueError, ImportError, RuntimeError) as e:
            refuse(label, str(e))
            continue
        if not client.probe():
            refuse(label, "no probe answer", client)
            continue
        driver = ""
        for attempt in range(3):
            driver = client.read_driver_name()
            if driver:
                break
            time.sleep(0.2)
        fields = client.read_outputs()
        if not fields:
            refuse(label, "serves no output descriptors — upload and run "
                          "a driver first", client)
            continue
        if not is_decodable(fields):
            refuse(label, "declares an output type this nxs version "
                          "can't decode — update nxs", client)
            continue
        pubs = load_map(args.map) if args.map else plan_publications(fields)
        frame = args.frame_id or name or driver or 'nxs'
        plans.append(UnitPlan(name=name, frame_id=frame, publications=pubs,
                              epoch=epoch_binding(fields)))
        clients.append(client)

    if not plans:
        raise SystemExit("nxs ros2: no bridgeable units")

    if args.plan:
        print(format_plan(plans, args.topic_base))
        for client in clients:
            client.close()
        return 0

    try:
        bridge = Ros2Bridge(plans, topic_base=args.topic_base,
                            stamp_mode=args.stamp,
                            time_syncs=[c.get_time_sync() for c in clients])
    except ImportError:
        for client in clients:
            client.close()
        raise SystemExit(
            "nxs ros2: rclpy not found — run inside a sourced ROS 2 "
            "environment (source /opt/ros/<distro>/setup.bash) with nxs "
            "installed into it") from None

    for plan in plans:
        for pub in plan.publications:
            print(f"publishing {join_topic(args.topic_base, plan.name, pub.topic)}"
                  f"  [{pub.msg_type}]")
    if args.stamp in (STAMP_SYNCED, STAMP_ITOW):
        # Warm-start the estimators — itow's documented fallback is the
        # synced projection. The per-unit bound banner prints
        # from run_bridge once a unit's first samples flow — transports
        # without a time surface (I2C) observe on the sample polls, so
        # no transport has a bound to show before streaming starts.
        for client in clients:
            client.time_sync_ping()

    samples = 0
    try:
        for client in clients:
            client.start_stream(_configure_stream(client, args))
        samples = run_bridge(clients, bridge, count=args.count)
    finally:
        for client in clients:
            for step in (client.stop_stream, client.close):
                try:
                    step()
                except Exception:
                    pass
        bridge.shutdown()
    print(f"{samples} sample(s) bridged")
    return 0


def _get_output_fields_by_name(driver_name, config=None):
    """Compile the named driver locally with the given config to get
    field descriptors (scales, units, types) for pretty-printing.

    Returns [] if the driver name doesn't resolve or compile fails.
    """
    try:
        mod = importlib.import_module(f"nxs.drivers.{driver_name.lower()}")
    except ModuleNotFoundError:
        return []

    from nxs.compiler import SensorDriver
    drv_cls = None
    for attr in dir(mod):
        obj = getattr(mod, attr)
        if (isinstance(obj, type) and issubclass(obj, SensorDriver)
                and obj is not SensorDriver
                and obj.__module__ == mod.__name__):
            drv_cls = obj
            break
    if drv_cls is None:
        return []

    try:
        compiled = drv_cls().compile(config or {})
        # Fold the live param scaling in, matching what the device serves.
        return effective_scale_fields(compiled.output_fields, compiled.params)
    except Exception as e:
        import sys
        print(f"(warning: could not load output fields: {e})", file=sys.stderr)
        return []


def _get_output_fields(driver_name, t):
    """Get output field descriptors by compiling the driver locally.

    Uses the current parameter values from the device to compile with
    the right config — so scales match the actual sensor configuration.
    """
    # Map driver names to module names
    name_map = {
        'IAM20680': 'iam20680',
        'NeoM9N': 'neo_m9n',
    }
    mod_name = name_map.get(driver_name, driver_name.lower())

    try:
        mod = importlib.import_module(f"nxs.drivers.{mod_name}")
    except ModuleNotFoundError:
        return []

    from nxs.compiler import SensorDriver
    drv_cls = None
    for attr in dir(mod):
        obj = getattr(mod, attr)
        if (isinstance(obj, type) and issubclass(obj, SensorDriver)
                and obj is not SensorDriver
                and obj.__module__ == mod.__name__):
            drv_cls = obj
            break
    if drv_cls is None:
        return []

    # Read current param values from device to build matching config. A
    # failure here is NOT harmless: fields whose scale tracks a param (an
    # IMU's full-scale range, typically) would silently decode at the
    # driver's compiled default instead of the device's live value, and the
    # printed columns would look entirely ordinary while being wrong.
    config = {}
    try:
        caps = t.read_capabilities()
        for p in caps:
            config[p['name']] = p['current']
    except Exception as e:
        print(f"warning: could not read live parameters ({e}) — refusing to "
              f"decode with compiled defaults, which would print plausible "
              f"but wrong values", file=sys.stderr)
        return []

    # Compile with current config to get output fields with correct scales
    try:
        compiled = drv_cls().compile(config)
        # Fold the live param scaling in, matching what the device serves.
        return effective_scale_fields(compiled.output_fields, compiled.params)
    except Exception as e:
        import sys
        print(f"(warning: could not load output fields: {e})", file=sys.stderr)
        return []


def cmd_outputs(t, args):
    """List the output fields the currently-loaded driver emits."""
    outs = t.read_outputs()
    if not outs:
        # Nothing readable on-device (no driver, or firmware without the
        # descriptor window); fall back to a local compile of the Python driver.
        name = t.read_driver_name()
        if not name:
            print("No driver loaded.")
            return 1
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
        print("No driver loaded, or driver exposes no output fields.")
        return 1

    print(f"{'#':>2s}  {'name':<12s}  {'type':<7s}  {'order':<6s}  "
          f"{'semantic':<11s}  {'scale':>12s}  {'offset':>10s}  unit")
    for o in outs:
        sem = SEMANTIC_NAMES.get(o.get('semantic', 0),
                                 f"sem{o.get('semantic', 0)}")
        print(f"{o['idx']:>2d}  {o['name']:<12s}  {o['type']:<7s}  "
              f"{o['byte_order']:<6s}  {sem:<11s}  {o['scale']:>12.6g}  "
              f"{o['offset']:>10.4g}  {o['unit']}")
    total = sample_width(outs)
    if total is None:
        print(f"\nTotal: {len(outs)} field(s), variable per sample "
              f"(contains a variable-width field).")
    else:
        print(f"\nTotal: {len(outs)} field(s), {total} byte(s) per sample.")
    return 0


def cmd_status(t, args):
    from nxs.transports.i2c import RUNNER_STATES

    ok = t.probe()
    if not ok:
        print(f"NXS @ {_describe_target(args)}: NOT FOUND{_probe_detail(t)}")
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

    vm_state_names = {0: 'IDLE', 1: 'RUNNING', 2: 'ERROR'}
    flags = []
    if status & 0x80: flags.append('running')
    if status & 0x01: flags.append('sample_ready')
    if status & 0x02: flags.append('error')

    slot_str = "—" if active_slot == 0xFF else f"#{active_slot}"
    runner_str = RUNNER_STATES.get(runner_state, f"?({runner_state})")

    # The latched mikroBUS I²C address rides the slot-peek capability
    # (GetDriverInfo over Cyphal, the SEL peek view over I²C).
    i2c_str = None
    if isinstance(t, SupportsSlotPeek):
        info = t.read_slot_info(ACTIVE_SLOT)
        if info is not None and info.i2c_addr != 0:
            i2c_str = f"0x{info.i2c_addr:02X}"

    print(f"NXS @ {_describe_target(args)}")
    if serial and any(serial):
        print(f"  Serial:  {serial.hex()}")
    if fw:
        print(f"  FW:      {fw}")
    print(f"  Driver:  {name or '(none)'}")
    print(f"  VM:      {vm_state_names.get(state, '?')} "
          f"(0x{status:02X}: {', '.join(flags) or 'idle'})")
    print(f"  Runner:  {runner_str}  "
          f"slot={slot_str}  retries={probe_retries}")
    if i2c_str:
        print(f"  mikroBUS I²C: {i2c_str}")
    print(f"  Store:   {store_count} populated")
    print(f"  Samples: {count}")
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
                print(f"  Sync:    ±{bound_us} µs  "
                      f"(offset {offset_us:+} µs, "
                      f"rate {rate_ppb / 1000:+.0f} ppm, "
                      f"window {valid_for_us / 1_000_000:.0f} s, "
                      f"source {src})")
            else:
                print("  Sync:    -")
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
                print(f"  Faults:  {io_err} I/O errors absorbed, "
                      f"{probe_failed} probe failures")
                print(f"           {drdy} samples missed (VM busy at DRDY), "
                      f"{rejects} commands rejected (bus contention)")
                if overflows is not None:
                    print(f"           {overflows} writes dropped "
                          f"(device queue full)")
    if error:
        print(f"  Error:   {error}")
    return 0


def cmd_save(t, args):
    slot = args.slot
    if slot is None:
        # Default: next free slot (after currently-populated ones)
        slot = t.read_store_count()
    try:
        t.save_slot(slot)
    except DeviceRefused as e:
        if e.code == ERRNO_EEXIST:
            # The goal state — this image persisted — already holds.
            print("Already stored: an identical driver image occupies "
                  "another slot. Nothing to do.")
            return 0
        print(f"Save refused: {e}", file=sys.stderr)
        return 1
    except RuntimeError as e:
        print(f"Save failed: {e}", file=sys.stderr)
        return 1
    count = t.read_store_count()
    print(f"Saved to slot {slot}. Store: {count} populated.")
    return 0


def cmd_store_ls(t, args):
    count = t.read_store_count()
    active = t.read_active_slot()
    if count == 0:
        print("Store is empty.")
        return 0
    print(f"Driver store: {count}/8 populated")
    # Slot-name peek is a typed capability; a transport without it
    # (the mock) only knows the count.
    can_peek = isinstance(t, SupportsSlotPeek)
    for i in range(count):
        marker = " ← active" if i == active else ""
        name = ""
        info = None
        if can_peek:
            info = t.read_slot_info(i)
            if info is not None and info.name:
                name = info.name
        elif i == active:
            # No per-slot peek on this transport: the active slot's
            # driver is loaded, so its name is readable from the device.
            name = t.read_driver_name()
        if name:
            extras = []
            if info is not None and info.num_outputs:
                extras.append(f"{info.num_outputs} outputs")
            if info is not None and info.num_params:
                extras.append(f"{info.num_params} params")
            suffix = f"  ({', '.join(extras)})" if extras else ""
            print(f"  [{i}] {name}{suffix}{marker}")
        else:
            print(f"  [{i}]{marker}")
    return 0


def cmd_store_rm(t, args):
    try:
        t.delete_slot(args.slot)
    except DeviceRefused as e:
        print(f"Remove refused: {e}", file=sys.stderr)
        return 1
    except RuntimeError as e:
        print(f"Remove failed: {e}", file=sys.stderr)
        return 1
    print(f"Removed slot {args.slot}. Store: {t.read_store_count()} populated.")
    return 0


def cmd_store_clear(t, args):
    try:
        t.clear_store()
    except DeviceRefused as e:
        print(f"Clear refused: {e}", file=sys.stderr)
        return 1
    except RuntimeError as e:
        print(f"Clear failed: {e}", file=sys.stderr)
        return 1
    print("Driver store cleared.")
    return 0


def cmd_cycle(t, args):
    from nxs.client import await_driver_up
    from nxs.transports.i2c import RUNNER_STATES
    old_slot = t.read_active_slot()
    t.cycle()
    # The cycled slot re-loads and re-probes asynchronously; a fixed sleep
    # either reports a stale slot or waits longer than the probe needs.
    await_driver_up(t)
    new_slot = t.read_active_slot()
    runner = RUNNER_STATES.get(t.read_runner_state(), "?")
    print(f"Cycled: slot {old_slot} → {new_slot}  state={runner}")
    return 0


def cmd_run(t, args):
    t.vm_run()
    print("VM running.")
    return 0


def cmd_stop(t, args):
    t.vm_stop()
    print("VM stopped.")
    return 0


def cmd_reset(t, args):
    t.vm_reset()
    print("VM reset; driver cleared.")
    return 0


def cmd_recover(t, args):
    if not isinstance(t, SupportsRecovery):
        print(f"recover: not supported on transport '{args.transport}'", file=sys.stderr)
        return 1
    t.recover()
    print("Device held in MCUboot serial recovery (flash via mcumgr over UART).")
    return 0


def cmd_identify(t, args):
    if not isinstance(t, SupportsIdentify):
        print(f"identify: not supported on transport '{args.transport}'",
              file=sys.stderr)
        return 1
    t.identify()
    print(f"Identify: status LED strobing on {_describe_target(args)} (~10 s)")
    return 0


def cmd_set_orientation(t, args):
    if not isinstance(t, SupportsCalibration):
        print(f"set-orientation: not supported on transport '{args.transport}'",
              file=sys.stderr)
        return 1
    if args.rotation is None:
        print(f"orientation = {rotation_name(t.read_calibration().orientation)}")
        return 0
    try:
        code = rotation_code(args.rotation)
    except ValueError as e:
        print(f"set-orientation: {e}", file=sys.stderr)
        return 1
    try:
        t.set_orientation(code, persist=not args.no_persist)
    except (RuntimeError, ValueError) as e:
        print(f"set-orientation: {e}", file=sys.stderr)
        return 1
    print(f"orientation = {args.rotation.upper()}"
          f"{'' if args.no_persist else ' (persisted)'}")
    return 0


def cmd_env(args):
    import nxs
    root = os.path.join(os.path.dirname(os.path.abspath(nxs.__file__)), 'dsdl')
    print(f'export CYPHAL_PATH={root}')
    # yakut needs a node-ID for register.Access; honour any prior override.
    for key, default in (('CYPHAL_ALLOW_UNREGULATED_FIXED_PORT_ID', '1'),
                         ('UAVCAN__NODE__ID', str(CyphalDefaults.HOST_NODE_ID))):
        if not os.environ.get(key):
            print(f'export {key}={default}')
    _print_completion()
    return 0


def _print_completion():
    """Emit argcomplete's registration for the eval'd `nxs env` line.

    argcomplete completes verbs, subcommands, flags, and argument
    choices from the live parser, so the completion never drifts from
    the CLI. The snippet is bash-flavored; the zsh guard loads
    bashcompinit first. Other shells get the exports only.
    """
    try:
        from argcomplete.shell_integration import shellcode
    except ImportError:
        return
    body = shellcode(["nxs"], shell="bash").strip()
    print("""
if [ -n "${BASH_VERSION:-}${ZSH_VERSION:-}" ]; then
if [ -n "${ZSH_VERSION:-}" ]; then
    typeset -f compdef >/dev/null 2>&1 || { autoload -U compinit && compinit; }
    autoload -U +X bashcompinit && bashcompinit
fi
%s
fi""" % body)



def cmd_timesync(t, args):
    """Refresh the estimator and push the discipline on a fixed cadence.
    The pushed offset maps the device clock into host CLOCK_REALTIME —
    the star topology's mesh epoch is this host."""
    from nxs.client import DeviceRefused, XFER_EBUSY, estimate_and_push

    if not isinstance(t, SupportsTimeSync):
        print("timesync: this transport serves no time surface")
        return 1
    while True:
        try:
            bound = estimate_and_push(t, interval_s=args.interval)
        except DeviceRefused as e:
            if e.code != XFER_EBUSY or args.once:
                print(f"timesync: {e}")
                return 1
            # A transfer session holds the mux (a driver upload or a
            # firmware push). The resident pusher skips the interval and
            # lives — killing it here silently decays mesh time.
            print("timesync: mux held (transfer in flight) — skipping")
            time.sleep(args.interval)
            continue
        except OSError as e:
            if args.once:
                print(f"timesync: {e}")
                return 1
            # The device NACKs the bus outright while a firmware push
            # erases its staging slot (~2.4 s of deafness) and again across
            # the reboot that applies it. A resident pusher rides that out
            # — dying on a transient bus error decays mesh time for the
            # whole rig long after the push has finished.
            print(f"timesync: link unavailable ({e}) — skipping")
            time.sleep(args.interval)
            continue
        except RuntimeError as e:
            print(f"timesync: {e}")
            return 1
        if bound is None:
            print("timesync: no observation (link silent?)")
            return 1
        print(f"pushed ±{bound} µs")
        if args.once:
            return 0
        time.sleep(args.interval)


def cmd_decimation(t, args):
    try:
        if args.value is not None:
            t.write_decimation(args.value, args.subject)
        print(f"decimation[{args.subject or 'device'}] = {t.read_decimation(args.subject)}")
    except ValueError as e:
        print(f"decimation: {e}", file=sys.stderr)
        return 1
    return 0


def _parse_can_term(spec):
    """`on`/`off`/`default` or the raw register values 1/0/0xFFFF."""
    words = {'on': 1, 'off': 0, 'default': 0xFFFF}
    if spec.lower() in words:
        return words[spec.lower()]
    try:
        return int(spec, 0)
    except ValueError:
        raise ValueError(f"--can-term expects on|off|default, got '{spec}'")


def cmd_can_term(t, args):
    if not isinstance(t, SupportsCanTermination):
        print(f"can-term: not supported on transport '{args.transport}'", file=sys.stderr)
        return 1
    try:
        if args.value is not None:
            t.write_can_term(_parse_can_term(args.value))
        print(f"can-term = {'on' if t.read_can_term() else 'off'}")
    except (ValueError, RuntimeError) as e:
        print(f"can-term: {e}", file=sys.stderr)
        return 1
    if args.value is not None:
        print("applied — persist with 'nxs commission --save'")
    return 0


def _bitrate_line(t) -> str:
    nominal, data = t.read_can_bitrate()
    return f"{nominal}/{data} ({'classic' if data == nominal else 'fd'})"


def _parse_can_bitrate(spec):
    """`NOM[/DATA]` in any int base; a bare nominal selects Classic CAN."""
    nom, sep, data = spec.partition('/')
    try:
        nominal = int(nom, 0)
        d = int(data, 0) if sep else nominal
    except ValueError:
        raise ValueError(f"--can-bitrate expects NOMINAL[/DATA] integers, got '{spec}'")
    return nominal, d


def cmd_can_bitrate(t, args):
    if not isinstance(t, SupportsBitTiming):
        print(f"can-bitrate: not supported on transport '{args.transport}'", file=sys.stderr)
        return 1
    try:
        if args.nominal is not None:
            data = args.data if args.data is not None else args.nominal
            t.write_can_bitrate(args.nominal, data)
        print(f"can-bitrate = {_bitrate_line(t)}")
    except (ValueError, RuntimeError) as e:
        print(f"can-bitrate: {e}", file=sys.stderr)
        return 1
    if args.nominal is not None:
        if args.transport == 'i2c':
            print("committed — applies at the next reboot")
        else:
            print("staged — persist with 'nxs commission --save'; applies at the next reboot")
    return 0


def cmd_commission(t, args):
    if not isinstance(t, SupportsCommissioning):
        print(f"commission: not supported on transport '{args.transport}'", file=sys.stderr)
        return 1
    bitrate = None
    term = None
    try:
        if args.can_bitrate is not None:
            bitrate = _parse_can_bitrate(args.can_bitrate)
        if args.can_term is not None:
            term = _parse_can_term(args.can_term)
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
            print(f"  {'can-bitrate':<18} {_bitrate_line(t)}")
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


def cmd_push_fw(t, args):
    import os
    name = os.path.basename(args.firmware)
    size = os.path.getsize(args.firmware)

    def progress(done, total):
        frac = done / total if total else 0
        w = 24
        n = round(w * frac)
        bar = "━" * n + "─" * (w - n)
        print(f"\r  {name}  {bar} {frac * 100:3.0f}%", end="", flush=True)

    progress(0, size)
    try:
        t.push_image(args.firmware, progress_cb=progress)
    except Exception as e:
        print(f"\n  ✗ {e}")
        return 1
    print("\n  ✓ updated")
    return 0


def _git_version() -> str:
    """Commit the tool runs from: live `git describe` in a source checkout,
    else the value release.sh baked into the wheel, else empty."""
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


def _tool_version() -> str:
    from importlib.metadata import PackageNotFoundError, version
    try:
        base = version('nxs')
    except PackageNotFoundError:
        base = '0.0.0+source'
    git = _git_version()
    return f'{base} ({git})' if git else base


class _VersionAction(argparse.Action):
    """`--version`, computed lazily. Building the version string runs
    `git describe`; a plain `action='version'` string would execute it at
    parser construction, which is every invocation of the tool."""

    def __call__(self, parser, namespace, values, option_string=None):
        print(f"nxs {_tool_version()}")
        parser.exit()


def build_parser():
    parser = argparse.ArgumentParser(
        prog='nxs',
        description='NXS sensor VM command-line tool')
    parser.add_argument('--version', action=_VersionAction, nargs=0,
                        help="show program's version number and exit")
    parser.add_argument('-t', '--transport', choices=['i2c', 'cyphal-serial', 'cyphal-can'],
                        default=_default_transport(),
                        help='Transport ($NXS_TRANSPORT, else i2c on Linux / '
                             'cyphal-serial on macOS, Windows)')
    parser.add_argument('-b', '--bus', default=os.environ.get('NXS_BUS', '/dev/i2c-2'),
                        help='I2C bus device ($NXS_BUS, else /dev/i2c-2)')
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
    # `int(x, 0)` honours the base prefix (0x/0b/decimal). Default is the
    # firmware's RBDevice::NXS so the CLI talks to the device out of the box.
    parser.add_argument('-a', '--addr', type=lambda x: int(x, 0),
                        default=NxsDevices.RBDevice.NXS,
                        help='NXS device address (any base; default: '
                             'RBDevice::NXS)')
    parser.add_argument('--unit', default=None,
                        help='Address a suite-managed unit by its manifest '
                             'name — resolves the transport and link from '
                             'suite.yaml, overriding -t/-b/-p/-a')

    sub = parser.add_subparsers(dest='command', required=True)

    sub.add_parser('probe', help='Check if the module is present')
    sub.add_parser('status', help='Show module status')

    p_upload = sub.add_parser('upload', help='Compile and upload a driver')
    p_upload.add_argument('driver',
                          help='Driver name (iam20680) — the preferred form; '
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

    sub.add_parser('caps', help='Show driver capabilities')
    sub.add_parser('outputs', help='Show per-sample output-field layout')
    sub.add_parser('run', help='Start the VM on the loaded driver')
    sub.add_parser('stop', help='Halt the VM, keep driver loaded')
    sub.add_parser('reset', help='Clear the loaded driver entirely')
    sub.add_parser('recover',
                   help='Reboot into MCUboot serial recovery and hold there')
    sub.add_parser('identify',
                   help='Strobe the status LED (~10 s) to physically locate '
                        'the unit')

    p_push = sub.add_parser('push-fw',
                            help='Upload firmware via the app-resident DFU '
                                 'core')
    p_push.add_argument('firmware', help='Path to signed firmware binary')

    p_get = sub.add_parser('get', help='Get a parameter value')
    p_get.add_argument('param', help='Parameter name')

    p_set = sub.add_parser('set', help='Set a parameter value')
    p_set.add_argument('param', help='Parameter name')
    p_set.add_argument('value', help='New value')

    p_stream = sub.add_parser('stream', help='Stream sensor data')
    p_stream.add_argument('-n', '--count', type=int, default=None,
                          help='Number of samples (default: infinite)')
    p_stream.add_argument('--raw', action='store_true',
                          help='Show raw hex bytes')
    p_stream.add_argument('--hz', type=int, default=None,
                          help='Target output rate in Hz (no cap). I2C: host '
                               'polls the window at this rate, reading the '
                               'latest sample (size it to your bus). Cyphal: '
                               'the device streams; --hz sets the sample rate '
                               'where a sample_rate param exists. Default when '
                               'omitted: the driver\'s sample_rate, else '
                               '~100 Hz on I2C.')
    p_stream.add_argument('-d', '--driver', default=None,
                          help='Driver module name (e.g. iam20680), used to '
                               'fetch scale/unit metadata so the stream prints '
                               'named fields instead of hex. Ignored where the '
                               'device serves its own descriptors (I2C, '
                               'Cyphal).')
    p_stream.add_argument('-q', '--quiet', action='store_true',
                          help="Don't print each sample; print a rolling "
                               "rate summary ~once per second. Use this to "
                               "measure peak wire-level throughput without "
                               "terminal rendering overhead.")
    p_stream.add_argument('--units', choices=['human', 'si'],
                          default=os.environ.get('NXS_UNITS',
                                                 'human').strip().lower(),
                          help='Display units for the sample table: human '
                               'converts kelvin to °C (header follows); si '
                               'prints the canonical SI values verbatim. '
                               'Display-only — the device always publishes '
                               'SI. Default: $NXS_UNITS, else human.')

    p_ros2 = sub.add_parser('ros2', help='Bridge decoded samples onto '
                                         'ROS 2 topics')
    p_ros2.add_argument('--hz', type=int, default=None,
                        help='Target per-unit output rate in Hz (as '
                             '`stream`)')
    p_ros2.add_argument('-n', '--count', type=int, default=None,
                        help='Stop after N samples total (default: run '
                             'until interrupted)')
    p_ros2.add_argument('--topic-base', default='nxs',
                        help='Leading topic namespace (default: nxs)')
    p_ros2.add_argument('--frame-id', default=None,
                        help='header.frame_id for a single device '
                             '(default: the unit name, else the driver '
                             'name)')
    p_ros2.add_argument('--stamp', choices=list(STAMP_MODES),
                        default=STAMP_SYNCED,
                        help='header.stamp source: the device clock '
                             'projected onto host time via two-way sync '
                             '(synced, default), the raw device '
                             'microsecond clock (device), ROS time '
                             'on arrival, or the GNSS in-message epoch '
                             'mapped to UTC (itow; epoch-capable drivers '
                             'only, gated on fix validity, falls back to '
                             'synced)')
    p_ros2.add_argument('--map', metavar='FILE', default=None,
                        help='Explicit mapping YAML replacing the '
                             'semantic auto-map (single device only)')
    p_ros2.add_argument('--plan', action='store_true',
                        help='Print the topic plan and exit without '
                             'publishing (no ROS required)')
    p_ros2.add_argument('--launch-file', action='store_true',
                        help='Print the path to the shipped ROS 2 launch '
                             'file and exit: ros2 launch "$(nxs ros2 '
                             '--launch-file)"')

    p_bench = sub.add_parser('bench',
                             help='End-to-end bring-up: flash → probe → '
                                  'per-driver upload/stream/save → '
                                  'auto-bind verify')
    p_bench.add_argument('--board', required=True,
                         help='Zephyr board name of the firmware build')
    p_bench.add_argument('--app', default='nxs-v1.0',
                         help='App directory under apps/zephyr/ '
                              '(default: nxs-v1.0)')
    p_bench.add_argument('--stream-samples', type=int, default=200,
                         help='Samples to stream per sensor (default: 200)')

    p_store = sub.add_parser('store', help='Manage the driver store')
    store_sub = p_store.add_subparsers(dest='store_cmd', required=True)
    store_sub.add_parser('ls', help='List populated slots')
    p_save = store_sub.add_parser(
        'save', help='Save current RAM driver to flash store')
    p_save.add_argument('slot', type=int, nargs='?', default=None,
                        help='Slot index (default: next free)')
    p_store_rm = store_sub.add_parser('rm', help='Delete a slot')
    p_store_rm.add_argument('slot', type=int)
    store_sub.add_parser('clear', help='Wipe all slots')
    store_sub.add_parser(
        'cycle', help='Force advance to next populated slot')

    sub.add_parser('env', help='Print CYPHAL_PATH exports and shell completion: eval "$(nxs env)"')

    p_ts = sub.add_parser('timesync',
                          help='Push the host time discipline to this one '
                               'unit; `nxs suite timesync` covers the bench')
    p_ts.add_argument('--interval', type=float, default=PUSH_INTERVAL_S,
                      help='Seconds between pushes (default 1)')
    p_ts.add_argument('--once', action='store_true',
                      help='Push once and exit')

    p_decim = sub.add_parser('decimation',
                             help='Get/set device + per-subject output decimation')
    p_decim.add_argument('value', nargs='?', type=int, default=None,
                         help='New factor (1 = every sample, N = every Nth); omit to read')
    p_decim.add_argument('--subject',
                         choices=['acceleration', 'angular_velocity', 'magnetic_field',
                                  'temperature', 'pressure'],
                         help='Per-subject factor; omit for device-wide')

    from nxs.suite.cli import add_suite_parser
    add_suite_parser(sub)

    p_bitrate = sub.add_parser('can-bitrate',
                               help='Get/set the persisted CAN bit-timing profile '
                                    '(uavcan.can.bitrate); applies on reboot')
    p_bitrate.add_argument('nominal', nargs='?', type=lambda x: int(x, 0), default=None,
                           help='Arbitration bitrate in bit/s (FD: 1M/4M, 1M/2M; Classic: '
                                '1M, 500k, 250k, 125k; 0 reverts to the default); omit to read')
    p_bitrate.add_argument('data', nargs='?', type=lambda x: int(x, 0), default=None,
                           help='Data-phase bitrate in bit/s (default: NOMINAL — Classic CAN)')

    p_term = sub.add_parser('can-term',
                            help='Get/set the on-board CAN split termination '
                                 '(off by default; applies live, persists on Save)')
    p_term.add_argument('value', nargs='?', default=None,
                        help='on | off | default (revert to off); omit to read')

    from nxs.calibrate import add_calibrate_parser
    add_calibrate_parser(sub)

    p_orient = sub.add_parser(
            'set-orientation',
            help='Declare the mounting orientation (a ROTATION_* name)')
    p_orient.add_argument('rotation', nargs='?', default=None,
                          type=str.upper, choices=ROTATION_NAMES,
                          metavar='ROTATION',
                          help='one of: ' + ' '.join(ROTATION_NAMES)
                               + '. Omit to print the current orientation')
    p_orient.add_argument('--no-persist', action='store_true',
                          help='Apply to the running state only (no Save)')

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

    return parser


def main():
    parser = build_parser()
    try:
        import argcomplete
        argcomplete.autocomplete(parser)
    except ImportError:
        pass
    args = parser.parse_args()

    # `upload -o FILE` compiles to a file with no device attached; every
    # other command needs a transport.
    if args.command == 'upload' and args.output:
        return cmd_upload(None, args)

    if args.command == 'env':
        return cmd_env(args)

    # Suite commands build their transports from the manifest, not the
    # global -t/-b/-p flags.
    if args.command == 'suite':
        from nxs.suite.cli import cmd_suite
        return cmd_suite(args)

    # ros2 builds its own transports too: the whole suite by default,
    # one --unit, or an ad-hoc flag-addressed device.
    if args.command == 'ros2':
        return cmd_ros2(args)

    port_given = args.port  # None → cyphal-serial autodetects during the open
    port_was_present = bool(port_given) and os.path.exists(port_given)
    try:
        t = _open_transport(args)
    except (OSError, ValueError, ImportError, RuntimeError) as e:
        # Missing device/permission, malformed bus, unknown transport, a missing
        # optional dep (smbus2 / pycyphal extra), or Cyphal startup (e.g. no DSDL).
        # One case gets its own diagnosis: a serial port that was present (or was
        # autodetected mid-call, hence present) and is gone after the failed open
        # dropped off the bus — the settings are fine, the device died.
        if (args.transport == 'cyphal-serial' and args.port
                and (port_was_present or port_given is None)
                and not os.path.exists(args.port)):
            from nxs.transports.cyphal_control import LINK_DROP_ADVICE
            sys.exit(f"nxs: {args.port} vanished while opening it — the USB "
                     f"device dropped off the bus. {LINK_DROP_ADVICE}")
        sys.exit(f"nxs: {args.transport}: {e} "
                 f"(check -t/-b/-p or the matching $NXS_* env vars)")
    except KeyboardInterrupt:
        sys.exit(130)  # Ctrl-C during a wedged open (e.g. a hung J-Link VCOM)

    # Flag-addressed mutation of a declared unit gets a breadcrumb; with
    # --unit the operator already named the unit, so stay silent.
    # This tool speaks a single register-map contract
    # (SUPPORTED_PROTO_VERSION). A device on another contract
    # would misbehave silently — a failed push leaves its VM halted, status
    # prints fabricated counters, an upload reads as a fake "Uploaded" — so
    # refuse loudly before any session-stateful traffic. Three verbs are
    # exempt because they are how a mismatch gets diagnosed and repaired:
    # `probe` reports the version, and `push-fw` / `recover` carry the new
    # firmware that ends the mismatch. Gating those would strand a fielded
    # device on its old contract with no upgrade path but a J-Link.
    if args.command not in VERSION_GATE_EXEMPT:
        _require_supported_version(t)

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
        from nxs.bench import cmd_bench
        commands = {
            'probe': cmd_probe, 'status': cmd_status,
            'upload': cmd_upload, 'caps': cmd_caps, 'outputs': cmd_outputs,
            'get': cmd_get, 'set': cmd_set, 'stream': cmd_stream,
            'run': cmd_run, 'stop': cmd_stop, 'reset': cmd_reset,
            'recover': cmd_recover,
            'identify': cmd_identify,
            'push-fw': cmd_push_fw,
            'bench': cmd_bench,
            'decimation': cmd_decimation,
            'timesync': cmd_timesync,
            'can-bitrate': cmd_can_bitrate,
            'can-term': cmd_can_term,
            'commission': cmd_commission,
            'set-orientation': cmd_set_orientation,
        }
        if args.command == 'calibrate':
            from nxs.calibrate import cmd_calibrate
            dispatch = cmd_calibrate
        else:
            dispatch = commands[args.command]

    # A command that reads live device state misreports an unreachable node:
    # `caps` prints "No driver loaded", `get` an empty value — each interprets
    # a no-answer as a valid empty result. Fail fast with one clear line, the
    # reachability check `status`/`probe` already do for themselves. Excluded:
    # `probe`/`status` (they report reachability), and `recover`/`push-fw`
    # (DFU acts on devices in non-standard states where `probe` may not answer).
    # Recovery verbs warn and proceed instead of refusing: a driver flooding
    # the link can drop probe replies, and the cure is exactly `stop`, a
    # re-`upload`, or `reset` — a hard refusal would leave the wedge no exit.
    NEEDS_DEVICE = {'caps', 'outputs', 'get', 'set', 'stream', 'run',
                    'decimation', 'can-bitrate', 'can-term', 'commission', 'store',
                    'identify', 'calibrate', 'set-orientation'}
    RECOVERY_VERBS = {'stop', 'upload', 'reset'}
    if args.command in NEEDS_DEVICE | RECOVERY_VERBS and not t.probe():
        if args.command in NEEDS_DEVICE:
            detail = _probe_detail(t) or (" (check wiring / power / -t/-b/-p "
                                          "or the $NXS_* env vars)")
            print(f"nxs: NXS not found on {args.transport}{detail}",
                  file=sys.stderr)
            return 1
        print(f"nxs: no probe answer on {args.transport}; "
              f"attempting {args.command} anyway",
              file=sys.stderr)

    # Transport failures surface as TimeoutError / RuntimeError (no ACK,
    # non-OK code) or OSError (bus error, NACK, permissions mid-verb);
    # convert them to a clean error line, not a traceback.
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
