"""`nxs stream`: the sample table, its rate meter, and the egress decimation a target rate asks for."""

import collections
import importlib
import os
import sys
import time

from nxs.client import SupportsCalibration, SupportsEgressDecimation
from nxs.descriptor import (
    effective_scale_fields, is_decodable, parse_sample, sample_width)
from nxs.term import status_line


class _RateMeter:
    """Rolling-window sample-rate meter for the `stream` display: a fixed
    1-second window converges to the steady-state rate on bursty sources."""

    def __init__(self, window_s: float = 1.0):
        self._window_s = window_s
        self._times: collections.deque = collections.deque()

    def tick(self, now: float, count: int = 1) -> float:
        """Record `count` samples that arrived at `now` and return the rolling
        rate in Hz. A batched receiver stamps all `count` at `now`."""
        for _ in range(count):
            self._times.append(now)
        while self._times and now - self._times[0] > self._window_s:
            self._times.popleft()
        return len(self._times) / self._window_s

# Human display units for the stream table, keyed by a field's declared
# canonical unit. Render-time only: the wire and every machine consumer stay SI.
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
    """Format a parsed sample's fields into one line: decoded text with trailing
    CR/LF stripped, numbers at a fixed `10.4g`. `human` applies `DISPLAY_UNITS`."""
    parts = []
    for f in output_fields:
        v = _display_value(values.get(f['name'], 0), f.get('unit', ''), human)
        if isinstance(v, str):
            parts.append(v.rstrip('\r\n'))
        else:
            parts.append(f"{v:10.4g}")
    return '  '.join(parts)

def _looks_textual(data: bytes) -> bool:
    """True if every byte is printable ASCII or tab/CR/LF, so text-shaped
    payloads (NMEA, AT lines) render readably; binary falls back to hex."""
    return all(b in (0x09, 0x0A, 0x0D) or 0x20 <= b <= 0x7E for b in data)

def cmd_stream(t, args):
    """Stream samples over any transport, rendering the decoded `Sample`s from
    iter_samples() the same way whatever the transport."""
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
        print("No personality loaded.")
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
        print("warning: the personality declares an output type this nxs "
              "version can't decode; streaming raw bytes — update nxs",
              file=sys.stderr)
        fields = []
    if fields:
        derived = sample_width(fields)
        # `sample_width()` is a lower bound: only overflowing fields fail to
        # decode. A string field's count is a ceiling, so a larger size is inconsistent.
        has_string = any(f.get('type') == 'string' for f in fields)
        mismatch = derived is not None and (
                (sample_size > derived) if has_string else (derived > sample_size))
        if mismatch:
            cause = ("on-device descriptors are inconsistent — firmware bug, "
                     "corruption, or an unsupported proto version"
                     if device_fields else
                     "the local personality file looks stale or wrong for this image")
            print(f"warning: output descriptors sum to {derived} B/sample "
                  f"but the device reports SAMPLE_SIZE={sample_size}; "
                  f"decoded values may be wrong ({cause})", file=sys.stderr)

    human = args.units != 'si'
    # A JSON document is finite and decoded: --count samples (5 when omitted)
    # in SI, the fields named with their units, nothing else on stdout.
    as_json = getattr(args, 'json', False)
    if as_json:
        if not fields:
            print("nxs stream --json: the personality serves no output descriptors "
                  "to decode", file=sys.stderr)
            return 1
        if args.count is None:
            args.count = 5
        elif args.count < 1:
            # A finite document of nothing is not a document: the same floor
            # `nxs mcp`'s `samples` enforces.
            print(f"nxs stream --json: count is at least 1, not {args.count}",
                  file=sys.stderr)
            return 1
        human = False
        records = []
    # Width of the per-row `[ rate Hz] n=... ts=...` prefix, shared with the
    # header pad: fits `[9999.0 Hz] n=65535` plus a 12-digit microsecond timestamp.
    prefix_w = 36
    show_cols = bool(fields) and not args.raw and not args.quiet and not as_json
    if as_json:
        pass
    elif show_cols:
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
    #: A finite read gives up after this long with no sample, so it ends
    #: through its own no-data path well inside any caller's timeout.
    STREAM_SILENCE_S = 15.0
    next_print = t0 + QUIET_WINDOW_S
    window_start, window_samples = t0, 0

    # A finite read gets a silence deadline: without one a muted sensor holds
    # the loop until an outer timeout kills the process, skipping the `finally`.
    silence = STREAM_SILENCE_S if count is not None else None
    try:
        t.start_stream(every_nth)
        for s in t.iter_samples(timeout=1.0, max_silence_s=silence):
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

            if as_json:
                values = s.values or (parse_sample(s.raw, local_fields,
                                                   cal_record, name)
                                      if local_fields else {})
                records.append({"n": int(s.count), "ts_us": s.timestamp_us,
                                "values": {k: (float(v) if isinstance(v, (int, float))
                                               and not isinstance(v, bool) else v)
                                           for k, v in values.items()}})
            elif args.quiet:
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
                    # Sensor produced this many more than received. Under host
                    # pacing (I2C poll, --hz decimation) most is skip, not loss.
                    skipped = max(0, ((s.count - first_seq) & 0xFFFF) - samples + 1)
                    lost = "" if t.lost_samples is None else f"  lost={t.lost_samples:4d}"
                    status_line(f"rx={samples:6d}  rate={inst:6.1f} Hz  "
                                f"sensor={sensor_rate:6.1f} Hz  avg={avg:6.1f} Hz  "
                                f"skipped={skipped:4d}{lost}  restarts={restarts:2d}")
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
    if as_json:
        import json

        from nxs.schemas import CONTRACT
        print(json.dumps({
            "contract": CONTRACT,
            "personality": driver_name,
            "fields": [{"name": f["name"], "unit": f.get("unit", "")} for f in fields],
            "samples": records,
            "rate_hz": round(avg, 3),
        }, indent=2))
        return 0 if records else 1
    loss = "" if t.lost_samples is None else f", {t.lost_samples} lost"
    print(f"\nStopped after {samples} samples in {elapsed:.1f}s ({avg:.1f} Hz avg{loss})")
    return 0

def _looks_like_vm_restart(prev_seq, curr_seq, threshold: int = 1000) -> bool:
    """Whether the sample seq jumped backwards (a VM restart): the u16 counter
    wraps with a masked delta of 1, so a reset reads as a near-65535 delta."""
    if prev_seq is None:
        return False
    raw_delta = (curr_seq - prev_seq) & 0xFFFF
    return raw_delta > threshold

def _configure_stream(t, args) -> int:
    """Apply --every/--hz to the transport and return the egress `every_nth`
    for start_stream(), 1 for none. A transport that decimates on-device takes
    `round(acq_rate/N)`; a host-paced one sets its poll rate instead."""
    if args.hz is not None and args.hz <= 0:
        raise SystemExit("--hz must be a positive rate (Hz)")

    if args.hz is not None:
        if isinstance(t, SupportsEgressDecimation):
            return _every_for_hz(t, args.hz)
        t.set_output_rate(args.hz)  # host-paced poll rate (e.g. I²C)
        return 1

    return 1

def _every_for_hz(t, hz: int) -> int:
    """Egress decimation factor for a target rate on a device that decimates
    on-device: round(acq_rate/hz), rounded to a divisor of the acquisition rate."""
    try:
        p = t.get_param("sample_rate")
    except KeyError:
        raise SystemExit(
            "--hz: the personality doesn't declare a 'sample_rate' param; use --every")
    current = p.get("current")
    acq_rate = int(current if current is not None else p.get("default", 0))
    if acq_rate <= 0:
        raise SystemExit("--hz: the personality's sample_rate is unset; use --every")
    every = max(1, round(acq_rate / hz))
    actual = acq_rate / every
    if abs(actual - hz) > 0.5:
        print(f"--hz {hz}: personality acq_rate={acq_rate} Hz, every_nth={every}, "
              f"actual output={actual:.1f} Hz "
              f"(decimation can only deliver acq_rate / integer)")
    return every

def _get_output_fields_by_name(driver_name, config=None):
    """Compile the named driver locally with the given config for field
    descriptors; [] if the name does not resolve or the compile fails."""
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
    """Get output field descriptors by compiling the driver locally, with the
    device's current parameter values so scales match its configuration."""
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

    # Read the device's param values to build a matching config; a field whose
    # scale tracks a param would otherwise decode at the compiled default.
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


def add_stream_parser(sub) -> None:
    """Register `nxs stream`."""
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
                               'omitted: the personality\'s sample_rate, else '
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
    p_stream.add_argument('--json', action='store_true',
                          help='One JSON document (contract 2): the fields '
                               'with their units and --count decoded samples '
                               '(5 when --count is omitted). What nxs mcp '
                               'serves as samples.')
    p_stream.add_argument('--units', choices=['human', 'si'],
                          default=os.environ.get('NXS_UNITS',
                                                 'human').strip().lower(),
                          help='Display units for the sample table: human '
                               'converts kelvin to °C (header follows); si '
                               'prints the canonical SI values verbatim. '
                               'Display-only — the device always publishes '
                               'SI. Default: $NXS_UNITS, else human.')
