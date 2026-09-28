"""Calibration verbs: on-device procedure triggers (gyro, mag) and the
host-side accel wizard, which solves at the raw tier in the sensor frame. A
solve whose parameters leave their bounds uploads nothing."""
import math
import sys
import time
from collections import deque
from typing import List, Optional, Sequence, Tuple

from nxs import calsolve
from nxs._generated_constants import Calibration as CalConstants
from nxs._generated_constants import FieldSemantics, RunnerStates
from nxs.term import status_line
from nxs.client import (CALIB_ERR_REASON, ERRNO_EAGAIN, DeviceRefused,
                        SupportsCalibration, active_driver_tag, err_reason,
                        rotation_name, runner_state_name)

# Accel-wizard tuning. The stillness threshold pairs with the metric
# `is_still` computes; retune them together.
STILL_WINDOW_S = 0.5           # rolling window the stillness gate inspects
ACCEL_STILL_THRESHOLD = 0.5    # m/s^2 p2p over the window; hand tremor is >= 1.5
POSE_SAMPLES = 100             # still samples averaged per accel pose
POSE_DOMINANCE = 0.8           # |axis| >= 80% of the reading's length claims
                               # a pose; the six need only span all three axes
CHECK_MIN_COMPONENT = 0.4      # every axis >= 40% of the reading's length (and
                               # none past POSE_DOMINANCE) claims a check pose
POSE_MIN_NORM = 0.5            # of |g|: below this a reading is not gravity
ENCODER_SAMPLES = 50
STREAM_TIMEOUT_S = 1.0

# On-device procedure pacing. The mag coverage scale rendered and auto-stopped
# on is the device's own, and its gate stays authoritative.
PROGRESS_POLL_S = 0.2
GYRO_WATCH_TIMEOUT_S = 90.0
MAG_WATCH_TIMEOUT_S = 120.0
# Pause before asking the device to solve again after an EAGAIN, so the
# operator can turn it to another orientation.
MAG_RETRY_S = 2.0

_POSE_LABELS = {(0, 1): '+X', (0, -1): '-X', (1, 1): '+Y', (1, -1): '-Y',
                (2, 1): '+Z', (2, -1): '-Z'}


def is_still(window: Sequence[Tuple[float, float, float]],
             threshold: float) -> bool:
    """True when the vectors in `window` (spanning `STILL_WINDOW_S` of one
    vector quantity) show no motion; `threshold` is the matching
    *_STILL_THRESHOLD. Gates gyro-bias averaging and each accel-pose capture."""
    if not window:
        return False

    for axis in range(3):
        accel_axis_row = [v[axis] for v in window]
        p2p = max(accel_axis_row) - min(accel_axis_row)
        if p2p > threshold:
            return False

    return True

# ── Field plumbing ─────────────────────────────────────────


def _axes(fields: List[dict], bucket: int) -> Optional[List[str]]:
    """The three field names routed to a vector bucket, slot order."""
    names: List[Optional[str]] = [None, None, None]
    for f in fields:
        sem = f.get('semantic', 0)
        if FieldSemantics.FIELD_SEMANTIC_BUCKET.get(sem, 0) == bucket:
            slot = FieldSemantics.FIELD_SEMANTIC_SLOT.get(sem, 0)
            if slot < 3:
                names[slot] = f.get('name')
    if any(n is None for n in names):
        return None
    return names  # type: ignore[return-value]


def _angle_field(fields: List[dict]) -> Optional[str]:
    for f in fields:
        if f.get('semantic', 0) == FieldSemantics.FieldSemantic.ANGLE:
            return f.get('name')
    return None


def _vector(values: dict, names: Sequence[str]):
    v = tuple(values.get(n) for n in names)
    if any(x is None or not isinstance(x, (int, float)) for x in v):
        return None
    return v


def _bar(fraction: float, width: int = 10) -> str:
    filled = round(min(1.0, max(0.0, fraction)) * width)
    return '━' * filled + '─' * (width - filled)


def _bucket_index(name: str) -> int:
    return {'accel': 0, 'gyro': 1, 'mag': 2}[name]


def _driver_tag(t) -> Tuple[str, int]:
    """The running sensor's name and identity tag. The tag a solve stamps
    must be the one the guard later compares against, so both sides go
    through `_active_tag`."""
    name = t.read_driver_name() or ''
    return name, active_driver_tag(t)


def _require(t, args, bucket_names, what: str):
    """Common verb preamble: capability, driver, panel, and runner-state checks."""
    if not isinstance(t, SupportsCalibration):
        print(f"calibrate: not supported on transport '{args.transport}'",
              file=sys.stderr)
        return None, None
    fields = t.read_outputs()
    if not fields:
        print("calibrate: no driver loaded", file=sys.stderr)
        return None, None
    names = _axes(fields, bucket_names) if isinstance(bucket_names, int) \
        else bucket_names(fields)
    if not names:
        print(f"calibrate: the loaded driver has no {what}", file=sys.stderr)
        return None, None
    # A driver whose sensor never answered declares the vector but delivers
    # no samples; the device would only wait out its stillness timeout.
    runner = t.read_runner_state()
    if runner != RunnerStates.RunnerState.MEASURING:
        print(f"calibrate: the driver is not measuring (runner "
              f"{runner_state_name(runner)}) — no samples to calibrate from",
              file=sys.stderr)
        return None, None
    return fields, names


class _StillGate:
    """Rolling gate-vector window + capture accumulator for still phases."""

    def __init__(self, gate_names: Sequence[str], threshold: float):
        self._names = gate_names
        self._threshold = threshold
        self._window: deque = deque()

    def push(self, values: dict, now: float) -> bool:
        """Feed one sample; True when the window is full and still."""
        v = _vector(values, self._names)
        if v is None:
            return False
        self._window.append((now, v))
        while self._window and now - self._window[0][0] > STILL_WINDOW_S:
            self._window.popleft()
        if not self._window or now - self._window[0][0] < STILL_WINDOW_S * 0.8:
            return False
        return is_still([v for _, v in self._window], self._threshold)


# ── Verbs ──────────────────────────────────────────────────


def _finish_procedure(t, args, result: int) -> int:
    """Shared tail: report the device's verdict, persist on success."""
    if result != 0:
        print(f"✗ {err_reason(result, CALIB_ERR_REASON)} — nothing uploaded",
              file=sys.stderr)
        return 1
    if not args.no_persist:
        t.save_calibration()
    print(f"✓ applied on-device{'' if args.no_persist else ' + persisted'}")
    return 0


def _release(t) -> bool:
    """Drop a procedure still running on the device, changing nothing; returns
    whether the release landed. Catches BaseException so a second Ctrl-C does
    not replace the exception the caller is unwinding."""
    try:
        t.cal_abort()
    except BaseException:
        return False
    return True


def _release_interrupted(t) -> None:
    """The shared interrupt epilogue: release, and say what happened."""
    print()
    if _release(t):
        print("interrupted — nothing applied", file=sys.stderr)
    else:
        print("interrupted — the device may still hold the calibration "
              "session", file=sys.stderr)


def _watch_procedure(t, args, timeout_s: float, render) -> int:
    """Poll the on-device procedure until it completes; render progress.
    On a 0 verdict `render` is called once more with the idle state and
    returns its finished line."""
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            state, detail, result = t.read_cal_progress()
            if state == CalConstants.CalState.IDLE:
                if result == 0:
                    # The device goes idle between two polls, so the last
                    # bar drawn is whatever the previous poll saw.
                    status_line(render(state, detail), done=True)
                else:
                    print()
                return _finish_procedure(t, args, result)
            status_line(render(state, detail))
            time.sleep(PROGRESS_POLL_S)
    except BaseException:
        # Ctrl-C included: KeyboardInterrupt is not an Exception, and a bare
        # `except Exception` would leave the procedure running, session held.
        _release_interrupted(t)
        raise
    print()
    print("✗ device did not finish in time", file=sys.stderr)
    _release(t)
    return 1


def cmd_gyro(t, args) -> int:
    fields, _ = _require(t, args, FieldSemantics.SubjectBucket.ANGULAR_VELOCITY,
                         'gyro vector (GYRO_X/Y/Z semantics)')
    if fields is None:
        return 1
    print("hold still — the device is measuring gyro bias")
    try:
        t.cal_gyro()
    except (RuntimeError, ValueError) as e:
        print(f"✗ {e}", file=sys.stderr)
        return 1

    def render(state, detail):
        if state == CalConstants.CalState.GYRO_WAIT_STILL:
            return "waiting for stillness"
        if state == CalConstants.CalState.IDLE:
            return f"{_bar(1.0)} 100%"
        return f"{_bar(detail / 100.0)} {detail}%"

    return _watch_procedure(t, args, GYRO_WATCH_TIMEOUT_S, render)


def _pose_label(v, check) -> Optional[str]:
    """The axis label of a dominant-axis pose; `'check'` for a near-diagonal
    one, or `'flipped'` once `check` (the first check pose's mean) exists
    and this one sits in the opposite octant; None for anything between."""
    # Fractions of the reading's own length, so an uncalibrated scale or
    # offset the solve accepts cannot keep a pose from ever being labelled.
    norm = math.hypot(*v)
    if norm < POSE_MIN_NORM * calsolve.STANDARD_GRAVITY:
        return None
    axis = max(range(3), key=lambda i: abs(v[i]))
    if abs(v[axis]) >= POSE_DOMINANCE * norm:
        return _POSE_LABELS[(axis, 1 if v[axis] > 0 else -1)]
    if min(abs(x) for x in v) < CHECK_MIN_COMPONENT * norm:
        return None
    if check is None:
        return 'check'
    return 'flipped' if all(x * c < 0 for x, c in zip(v, check)) else None


def _capture_poses(t, names, labels, clock=time.monotonic) -> dict:
    """Stream until every label in `labels` has a still `POSE_SAMPLES`-sample
    mean, in any order, or the stream ends; returns what was captured."""
    gate = _StillGate(names, ACCEL_STILL_THRESHOLD)
    means: dict = {}
    run: Optional[Tuple[str, list]] = None
    for s in t.iter_samples(timeout=STREAM_TIMEOUT_S, calibrated=False,
                            max_silence_s=STREAM_TIMEOUT_S):
        v = _vector(s.values, names)
        if v is None:
            continue
        still = gate.push(s.values, clock())
        label = _pose_label(v, means.get('check')) if still else None
        if label not in labels or label in means:
            run = None
            g = calsolve.STANDARD_GRAVITY
            reading = " ".join(f"{x / g:+.2f}" for x in v)
            note = f", {label} already captured" if label in means else ""
            status_line("waiting: " + " ".join(l for l in labels if l not in means)
                    + f"   (reading {reading} g{note})")
            continue
        if run is None or run[0] != label:
            run = (label, [])
        run[1].append(v)
        status_line(f"{label} {_bar(len(run[1]) / POSE_SAMPLES)} "
                f"{len(run[1])}/{POSE_SAMPLES}")
        if len(run[1]) >= POSE_SAMPLES:
            mean = tuple(sum(x[i] for x in run[1]) / POSE_SAMPLES
                         for i in range(3))
            means[label] = mean
            status_line(f"✓ {label}  ({math.hypot(*mean) / calsolve.STANDARD_GRAVITY:.3f} g)",
                    done=True)
            run = None
            if len(means) == len(labels):
                break
    return means


def cmd_accel(t, args, clock=time.monotonic) -> int:
    fields, names = _require(
            t, args, FieldSemantics.SubjectBucket.ACCELERATION,
            'accel vector (ACCEL_X/Y/Z semantics)')
    if names is None:
        return 1
    print("accel calibration — 8 still poses, any order:\n"
          "  +X -X +Y -Y +Z -Z   each axis pointing up, then down\n"
          "  check               resting on a corner: every axis reads "
          "0.40-0.80 g\n"
          "  flipped             the same corner pointing down (every axis "
          "reversed)")
    labels = list(_POSE_LABELS.values()) + ['check', 'flipped']
    means = _capture_poses(t, names, labels, clock)
    missing = [l for l in labels if l not in means]
    if missing:
        print(f"\ncalibrate: stream ended before {' '.join(missing)}",
              file=sys.stderr)
        return 1
    checks = [means.pop('check'), means.pop('flipped')]

    try:
        sol = calsolve.solve_accel(list(means.values()))
        err = max(calsolve.verify_accel(sol, c) for c in checks)
    except calsolve.CalSolveError as e:
        print(f"✗ {e}", file=sys.stderr)
        return 1
    driver, tag = _driver_tag(t)
    record = t.read_calibration().replace_vector(
            _bucket_index('accel'), sol.m, sol.b, tag)
    t.write_calibration(record, persist=not args.no_persist)
    print(f"solve: scale ({', '.join(f'{x:.4f}' for x in sol.scale)}), "
          f"offset ({', '.join(f'{x:+.3f}' for x in sol.offset)}) m/s^2, "
          f"check poses {err * 100:.2f}% off g")
    print(f"✓ applied{'' if args.no_persist else ' + persisted'}"
          f" (tag {driver or 'unguarded'})")
    return 0


def cmd_mag(t, args) -> int:
    fields, _ = _require(t, args, FieldSemantics.SubjectBucket.MAGNETIC_FIELD,
                         'mag vector (MAG_X/Y/Z semantics)')
    if fields is None:
        return 1
    print("mag calibration — rotate the vehicle through all orientations")
    try:
        t.cal_mag_start()
    except (RuntimeError, ValueError) as e:
        print(f"✗ {e}", file=sys.stderr)
        return 1

    # Watch the device's coverage; auto-stop once it can gate a solve, or at
    # the timeout so an under-rotated run fails rather than collecting forever.
    deadline = time.monotonic() + MAG_WATCH_TIMEOUT_S
    try:
        while True:
            state = CalConstants.CalState.MAG_COLLECT
            result = 0
            while time.monotonic() < deadline:
                state, coverage, result = t.read_cal_progress()
                if state != CalConstants.CalState.MAG_COLLECT:
                    break
                status_line(f"coverage "
                        f"{_bar(coverage / CalConstants.MAG_COVERAGE_FULL)} "
                        f"{coverage}/{CalConstants.MAG_COVERAGE_FULL}")
                if coverage >= CalConstants.MAG_COVERAGE_ENOUGH:
                    break
                time.sleep(PROGRESS_POLL_S)
            print()
            if state == CalConstants.CalState.IDLE:
                # The procedure ended without this stop (the device's give-up
                # window, or another host); the progress record has the verdict.
                return _finish_procedure(t, args, result)
            try:
                t.cal_mag_stop()
            except DeviceRefused as e:
                # Coverage is decided after the fit, so the progress byte can
                # reach its stop while the solve wants more rotation; keep turning.
                if e.code == ERRNO_EAGAIN and time.monotonic() < deadline:
                    print(f"not yet — {e}. Keep rotating, especially about "
                          f"an axis you have not inverted", file=sys.stderr)
                    time.sleep(MAG_RETRY_S)
                    continue
                print(f"✗ {e} — nothing uploaded", file=sys.stderr)
                _release(t)
                return 1
            break
    except BaseException:
        # Ctrl-C included: a held collection wedges every later command until
        # the device's own window expires, and a bare `except Exception` misses it.
        _release_interrupted(t)
        raise
    return _finish_procedure(t, args, 0)


def cmd_encoder_zero(t, args) -> int:
    fields, name = _require(t, args, lambda f: _angle_field(f),
                            'angle output (ANGLE semantic)')
    if name is None:
        return 1
    sines = 0.0
    cosines = 0.0
    n = 0
    for s in t.iter_samples(timeout=STREAM_TIMEOUT_S, calibrated=False,
                             max_silence_s=STREAM_TIMEOUT_S):
        angle = s.values.get(name)
        if not isinstance(angle, (int, float)):
            continue
        sines += math.sin(angle)
        cosines += math.cos(angle)
        n += 1
        if n >= ENCODER_SAMPLES:
            break
    if n < ENCODER_SAMPLES:
        print("calibrate: stream ended before enough angle samples",
              file=sys.stderr)
        return 1
    mean = math.atan2(sines / n, cosines / n) % math.tau
    driver, tag = _driver_tag(t)
    record = t.read_calibration()
    record.encoder_zero = (-mean) % math.tau
    record.encoder_tag = tag
    t.write_calibration(record, persist=not args.no_persist)
    print(f"zero at {mean:.4f} rad → offset {record.encoder_zero:.4f}")
    print(f"✓ applied{'' if args.no_persist else ' + persisted'}"
          f" (tag {driver or 'unguarded'})")
    return 0


def _guard_label(guard: int, tag: int, driver: str) -> str:
    """One row's guard verdict for `calibrate show`, tag detail included."""
    if guard == CalConstants.BucketGuard.UNGUARDED:
        return 'unguarded'
    if guard == CalConstants.BucketGuard.BOUND:
        return f'active ({driver})' if driver else 'active'

    return (f'INACTIVE (solved for tag 0x{tag:08X}, '
            f'now {driver or "none"})')


def cmd_show(t, args) -> int:
    if not isinstance(t, SupportsCalibration):
        print(f"calibrate: not supported on transport '{args.transport}'",
              file=sys.stderr)
        return 1
    record = t.read_calibration()
    try:
        driver, active_hash = _driver_tag(t)
    except Exception:
        driver, active_hash = '', 0
    print(f"orientation  {rotation_name(record.orientation)}")
    for vec, label in enumerate(('accel', 'gyro', 'mag')):
        solved = (tuple(record.m[vec]) != (1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                           0.0, 0.0, 1.0)
                  or any(record.b[vec]))
        guard = _guard_label(record.bucket_guard(vec, active_hash),
                             record.driver_tags[vec], driver)
        print(f"{label:<6} {'solved' if solved else 'identity':<9} {guard}")
        if args.full:
            for i in range(3):
                row = record.m[vec][i * 3:i * 3 + 3]
                print(f"       [{row[0]:+9.5f} {row[1]:+9.5f} {row[2]:+9.5f}]"
                      f"  b[{i}] {record.b[vec][i]:+9.5f}")
    enc_guard = _guard_label(
            record.bucket_guard(len(record.driver_tags), active_hash),
            record.encoder_tag, driver)
    print(f"encoder zero {record.encoder_zero:+.4f} rad  {enc_guard}")
    return 0


def cmd_reset(t, args) -> int:
    if not isinstance(t, SupportsCalibration):
        print(f"calibrate: not supported on transport '{args.transport}'",
              file=sys.stderr)
        return 1
    record = t.read_calibration()
    identity = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    zero = (0.0, 0.0, 0.0)
    targets = (('accel', 'gyro', 'mag', 'encoder') if args.bucket == 'all'
               else (args.bucket,))
    for target in targets:
        if target == 'encoder':
            record.encoder_zero = 0.0
            record.encoder_tag = 0
        else:
            record = record.replace_vector(_bucket_index(target), identity,
                                           zero, 0)
    t.write_calibration(record, persist=True)
    print(f"✓ reset {args.bucket} (orientation kept: "
          f"{rotation_name(record.orientation)})")
    return 0


# ── CLI wiring ─────────────────────────────────────────────


def add_calibrate_parser(sub) -> None:
    p = sub.add_parser('calibrate',
                       help='Solve and store per-unit calibration')
    cal = p.add_subparsers(dest='cal_cmd', required=True)
    for name, help_text in (
            ('gyro', 'Average the at-rest rates into a bias (hold still)'),
            ('accel', 'Guided scale/offset solve: six poses plus two check poses'),
            ('mag', 'In-situ hard/soft-iron ellipsoid fit (mounted in the '
                    'vehicle, rotate through all orientations)'),
            ('encoder-zero', 'Declare the current encoder angle as zero')):
        c = cal.add_parser(name, help=help_text)
        c.add_argument('--no-persist', action='store_true',
                       help='Apply to the running state only (no Save)')
    s = cal.add_parser('show', help='Print the stored record + guard status')
    s.add_argument('--full', action='store_true',
                   help='Include the M/b coefficients')
    r = cal.add_parser('reset',
                       help='Restore identity calibration (orientation kept)')
    r.add_argument('bucket', nargs='?', default='all',
                   choices=['accel', 'gyro', 'mag', 'encoder', 'all'])


def cmd_calibrate(t, args) -> int:
    handlers = {'gyro': cmd_gyro, 'accel': cmd_accel, 'mag': cmd_mag,
                'encoder-zero': cmd_encoder_zero, 'show': cmd_show,
                'reset': cmd_reset}
    return handlers[args.cal_cmd](t, args)
