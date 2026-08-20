"""Calibration verbs: on-device procedure triggers plus the accel wizard.

Gyro and mag solve on the device through the `CAL_GYRO` and
`CAL_MAG_START/STOP` vendor commands, the same surface a Cyphal autopilot
drives with no tool. The verbs here trigger them and render progress.
The accel wizard acquires and solves host-side, at the raw descriptor
tier and in the sensor frame. The mounting orientation is a separate
record field the appliers compose on top. Every solve is gated by
falsifiability checks, and a failed gate uploads nothing.
"""
import math
import sys
import time
from collections import deque
from typing import List, Optional, Sequence, Tuple

from nxs import calsolve
from nxs._generated_constants import Calibration as CalConstants
from nxs._generated_constants import FieldSemantics
from nxs.client import (CALIB_ERR_REASON, ERRNO_EAGAIN, DeviceRefused,
                        SupportsCalibration, active_driver_tag, err_reason,
                        rotation_name)

# Accel-wizard tuning. The stillness threshold pairs with the metric
# `is_still` computes — retune them together.
STILL_WINDOW_S = 0.5           # rolling window the stillness gate inspects
ACCEL_STILL_THRESHOLD = 0.5    # m/s^2 p2p: 2x the ~0.25 quiet-bench floor
                               # of a full 0.5 s window at 250 Hz (IAM-20680,
                               # FS=8g); hand tremor measures >=1.5
POSE_SAMPLES = 100             # still samples averaged per accel pose
POSE_DOMINANCE = 0.8           # |axis| >= 80% of |g| claims a pose
POSE_TILT_HINT_DEG = 5.0       # past this, the capture is worth re-seating:
                               # solve_accel's reference asserts the pose is
                               # exactly axis-aligned, and |a| — the only other
                               # number shown — is rotation-invariant, so tilt
                               # is otherwise invisible until verification
ENCODER_SAMPLES = 50
STREAM_TIMEOUT_S = 1.0

# On-device procedure pacing: poll period and give-up windows. The
# coverage scale the mag verb renders and auto-stops on is the device's
# own (`Calibration.MAG_COVERAGE_*`), and its gate stays authoritative.
PROGRESS_POLL_S = 0.2
GYRO_WATCH_TIMEOUT_S = 90.0
MAG_WATCH_TIMEOUT_S = 120.0
# Pause before asking the device to solve again after an EAGAIN: the
# solve is real arithmetic on the device, and the operator needs a
# moment to turn it somewhere new for the answer to change.
MAG_RETRY_S = 2.0

_POSE_LABELS = {(0, 1): '+X', (0, -1): '-X', (1, 1): '+Y', (1, -1): '-Y',
                (2, 1): '+Z', (2, -1): '-Z'}


def is_still(window: Sequence[Tuple[float, float, float]],
             threshold: float) -> bool:
    """True when the vectors in `window` show no motion.

    `window` spans `STILL_WINDOW_S` of one vector quantity (accel in
    m/s^2, or gyro rates in rad/s when the panel has no accel);
    `threshold` is the matching *_STILL_THRESHOLD constant. Gates both
    the gyro-bias averaging and each accel-pose capture.
    """
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


def _pose_tilt_deg(mean, axis: int) -> float:
    """Angle between a captured pose and the axis it claims to be aligned with.

    `solve_accel` builds each reference as exactly ±g on this axis and zero on
    the other two, so a tilted pose hands the fit a contradiction. The captured
    magnitude cannot expose that — a rotated vector has the same length — which
    leaves tilt invisible until the calibration is verified against gravity.
    """
    norm = math.hypot(*mean)
    if norm <= 0.0:
        return 0.0
    return math.degrees(math.acos(min(1.0, abs(mean[axis]) / norm)))


def _bucket_index(name: str) -> int:
    return {'accel': 0, 'gyro': 1, 'mag': 2}[name]


def _driver_tag(t) -> Tuple[str, int]:
    """The running sensor's name and identity tag. The tag a solve stamps
    must be the one the guard later compares against, so both sides go
    through `_active_tag`."""
    name = t.read_driver_name() or ''
    return name, active_driver_tag(t)


def _require(t, args, bucket_names, what: str):
    """Common verb preamble: capability, driver, and panel checks."""
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
    """Drop a procedure still running on the device, changing nothing.

    A procedure holds the device's transfer session until it ends, and a
    held session refuses every later command — a store read, and even the
    firmware push that would recover the unit. `stop` also releases, but
    it solves and applies on the way out; this must not. Returns whether
    the release landed. Catches BaseException: a second Ctrl-C during the
    release must not replace the exception the caller is unwinding.
    """
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
    """Poll the on-device procedure until it completes; render progress."""
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            state, detail, result = t.read_cal_progress()
            if state == CalConstants.CalState.IDLE:
                print()
                return _finish_procedure(t, args, result)
            print(f"\r{render(state, detail)}", end="")
            time.sleep(PROGRESS_POLL_S)
    except BaseException:
        # Ctrl-C included, and it is the case an operator actually hits.
        # KeyboardInterrupt is not an Exception, so a bare `except
        # Exception` would walk straight past it and leave the procedure
        # running with the session held.
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
            return "waiting for stillness              "
        return f"{_bar(detail / 100.0)} {detail}%           "

    return _watch_procedure(t, args, GYRO_WATCH_TIMEOUT_S, render)


def cmd_accel(t, args) -> int:
    fields, names = _require(
            t, args, FieldSemantics.SubjectBucket.ACCELERATION,
            'accel vector (ACCEL_X/Y/Z semantics)')
    if names is None:
        return 1
    gate = _StillGate(names, ACCEL_STILL_THRESHOLD)
    g = calsolve.STANDARD_GRAVITY
    print("accel calibration — hold the unit still in 6 orientations "
          "(each axis up and down)")

    poses: dict = {}
    accum: List[Tuple[float, float, float]] = []
    current_pose = None
    while len(poses) < 6:
        got_sample = False
        for s in t.iter_samples(timeout=STREAM_TIMEOUT_S, calibrated=False,
                             max_silence_s=STREAM_TIMEOUT_S):
            got_sample = True
            now = time.monotonic()
            v = _vector(s.values, names)
            if v is None:
                continue
            still = gate.push(s.values, now)
            axis = max(range(3), key=lambda i: abs(v[i]))
            sign = 1 if v[axis] > 0 else -1
            pose = (axis, sign)
            dominant = abs(v[axis]) >= POSE_DOMINANCE * g
            if not still or not dominant or pose in poses:
                if accum:
                    accum = []
                current_pose = None
                remaining = " ".join(_POSE_LABELS[p]
                                     for p in sorted(_POSE_LABELS)
                                     if p not in poses)
                print(f"\rwaiting: {remaining}                ", end="")
                continue
            if pose != current_pose:
                current_pose = pose
                accum = []
            accum.append(v)
            print(f"\r{_POSE_LABELS[pose]} {_bar(len(accum) / POSE_SAMPLES)} "
                  f"{len(accum)}/{POSE_SAMPLES}          ", end="")
            if len(accum) >= POSE_SAMPLES:
                mean = tuple(sum(x[i] for x in accum) / len(accum)
                             for i in range(3))
                reference = [0.0, 0.0, 0.0]
                reference[pose[0]] = pose[1] * g
                poses[pose] = (mean, reference)
                mag = math.hypot(*mean) / g
                tilt = _pose_tilt_deg(mean, pose[0])
                # The pose is captured either way — it is already in `poses`
                # and the loop will not ask for it again — so this names the
                # cost rather than promising a recapture the run cannot make.
                hint = "  (drags the fit — re-run for a tighter residual)" \
                    if tilt > POSE_TILT_HINT_DEG else ""
                print(f"\r✓ {_POSE_LABELS[pose]}  ({mag:.3f} g, "
                      f"{tilt:.1f}° off axis){hint}"
                      f"                              ")
                accum = []
                current_pose = None
                break
        if not got_sample:
            print("\ncalibrate: stream ended before all 6 poses",
                  file=sys.stderr)
            return 1

    try:
        sol = calsolve.solve_accel(list(poses.values()))
    except calsolve.CalSolveError as e:
        print(f"✗ {e}", file=sys.stderr)
        return 1
    driver, tag = _driver_tag(t)
    record = t.read_calibration().replace_vector(
            _bucket_index('accel'), sol.m, sol.b, tag)
    t.write_calibration(record, persist=not args.no_persist)
    print(f"solve: max residual {sol.max_residual * 100:.1f}% of g")
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

    # Watch the device's coverage; auto-stop once it can gate a solve
    # (its own coverage gate stays authoritative), or at the timeout so an
    # under-rotated run fails loudly rather than collecting forever.
    deadline = time.monotonic() + MAG_WATCH_TIMEOUT_S
    try:
        while True:
            state = CalConstants.CalState.MAG_COLLECT
            result = 0
            while time.monotonic() < deadline:
                state, coverage, result = t.read_cal_progress()
                if state != CalConstants.CalState.MAG_COLLECT:
                    break
                print(f"\rcoverage "
                      f"{_bar(coverage / CalConstants.MAG_COVERAGE_FULL)} "
                      f"{coverage}/{CalConstants.MAG_COVERAGE_FULL}", end="")
                if coverage >= CalConstants.MAG_COVERAGE_ENOUGH:
                    break
                time.sleep(PROGRESS_POLL_S)
            print()
            if state == CalConstants.CalState.IDLE:
                # The procedure ended without our stop — the device's
                # give-up window, or another host — and the progress
                # record carries its verdict. A stop now would only
                # answer EINVAL (no collection open).
                return _finish_procedure(t, args, result)
            try:
                t.cal_mag_stop()
            except DeviceRefused as e:
                # The device decides coverage after the fit, where it can
                # divide the mount's own distortion out, so the progress
                # byte is an estimate and can reach its stopping point
                # while the solve still wants more rotation. EAGAIN keeps
                # the collection open on the device, so keep turning and
                # ask again rather than throwing the rotation away.
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
        # Ctrl-C included: the collection holds the transfer session, and
        # leaving it held wedges every later command — a store read, and
        # even the firmware push that would recover the unit — until the
        # device's own window expires. KeyboardInterrupt is not an
        # Exception, so a bare `except Exception` would miss the case an
        # operator actually hits.
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
            ('accel', 'Guided 6-pose scale/misalignment solve'),
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
