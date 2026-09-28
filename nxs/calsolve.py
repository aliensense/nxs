"""Host-side accelerometer calibration solver in the sensor frame, pure Python.
Six poses determine the six parameters exactly and two unseen check poses in
opposite octants check it; a result past a bound raises `CalSolveError`."""
import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

STANDARD_GRAVITY = 9.80665

# Bounds on the recovered parameters: six poses fit six parameters exactly,
# so a moved pose lands here, not in a residual (ArduPilot bounds the same).
ACCEL_MIN_SCALE = 0.8
ACCEL_MAX_SCALE = 1.2
ACCEL_MAX_OFFSET = 0.10      # of g, on any axis
# |a| error allowed on an unseen check pose; two check directions cannot
# validate six parameters, so a push across a pose's other axes can pass.
ACCEL_CHECK_TOLERANCE = 0.03  # of g
GN_ITERATIONS = 50
GN_STEP_TOL = 1e-9


class CalSolveError(RuntimeError):
    """A solve failed its validity gate; nothing may be uploaded."""


@dataclass
class VectorSolution:
    """One bucket's fitted per-axis parameters, with the record form derived."""
    scale: Tuple[float, ...]   # per-axis gain
    offset: Tuple[float, ...]  # per-axis zero-g offset, measured units

    @property
    def m(self) -> Tuple[float, ...]:
        """Row-major 3x3 for the record: the scales on the diagonal."""
        rows = [0.0] * 9
        for k in range(3):
            rows[k * 3 + k] = self.scale[k]
        return tuple(rows)

    @property
    def b(self) -> Tuple[float, ...]:
        """Bias for the record: `-scale * offset` per axis."""
        return tuple(-s * o for s, o in zip(self.scale, self.offset))

    def correct(self, v: Sequence[float]) -> Tuple[float, ...]:
        """Apply the fit to one measured vector: `scale * (v - offset)`."""
        return tuple(s * (x - o) for s, x, o in zip(self.scale, v, self.offset))


def solve_linear(a: List[List[float]], b: List[float]) -> List[float]:
    """Solve A·x = b by Gaussian elimination with partial pivoting."""
    n = len(b)
    m = [list(a[i]) + [b[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            raise CalSolveError("singular system (degenerate acquisition)")
        m[col], m[pivot] = m[pivot], m[col]
        for row in range(col + 1, n):
            f = m[row][col] / m[col][col]
            for c in range(col, n + 1):
                m[row][c] -= f * m[col][c]
    x = [0.0] * n
    for row in range(n - 1, -1, -1):
        s = sum(m[row][c] * x[c] for c in range(row + 1, n))
        x[row] = (m[row][n] - s) / m[row][row]
    return x


def solve_accel(poses: Sequence[Sequence[float]],
                gravity: float = STANDARD_GRAVITY) -> VectorSolution:
    """6-param fit from still poses: |diag(s)·(v − o)| = g. `poses` holds each
    pose's mean vector; the set must span all three axes (each up and down).
    Gauss-Newton from the identity; scales and offsets are bounded."""
    if len(poses) < 6:
        raise CalSolveError(f"need at least 6 poses, got {len(poses)}")
    if any(math.hypot(*v) < 1e-9 for v in poses):
        raise CalSolveError("a pose reads zero — nothing uploaded")
    o = [0.0, 0.0, 0.0]
    s = [1.0, 1.0, 1.0]
    for _ in range(GN_ITERATIONS):
        jtj = [[0.0] * 6 for _ in range(6)]
        jtr = [0.0] * 6
        for v in poses:
            c = [s[k] * (v[k] - o[k]) for k in range(3)]
            norm = math.hypot(*c)
            r = norm - gravity
            j = [0.0] * 6
            for k in range(3):
                j[k] = -c[k] * s[k] / norm
                j[3 + k] = c[k] * (v[k] - o[k]) / norm
            for p in range(6):
                jtr[p] += j[p] * r
                for q in range(6):
                    jtj[p][q] += j[p] * j[q]
        try:
            step = solve_linear(jtj, [-x for x in jtr])
        except CalSolveError:
            raise CalSolveError("poses do not span all three axes — "
                                "nothing uploaded") from None
        for k in range(3):
            o[k] += step[k]
            s[k] += step[3 + k]
        if max(abs(x) for x in step) < GN_STEP_TOL:
            break
    else:
        raise CalSolveError(f"accel fit did not converge in {GN_ITERATIONS} "
                            "iterations — nothing uploaded")

    max_offset = ACCEL_MAX_OFFSET * gravity
    for axis in range(3):
        if not ACCEL_MIN_SCALE <= s[axis] <= ACCEL_MAX_SCALE:
            raise CalSolveError(
                f"accel fit scale {s[axis]:.3f} on axis {axis} outside "
                f"{ACCEL_MIN_SCALE}..{ACCEL_MAX_SCALE} — a moved pose; "
                "nothing uploaded")
        if abs(o[axis]) > max_offset:
            raise CalSolveError(
                f"accel fit offset {o[axis]:.2f} on axis {axis} exceeds "
                f"±{max_offset:.2f} — a moved pose; nothing uploaded")
    return VectorSolution(scale=tuple(s), offset=tuple(o))


def verify_accel(sol: VectorSolution, pose: Sequence[float],
                 gravity: float = STANDARD_GRAVITY) -> float:
    """Relative |a| error of a still pose the fit did not use, as a
    fraction of g; raises past `ACCEL_CHECK_TOLERANCE`."""
    err = abs(math.hypot(*sol.correct(pose)) / gravity - 1.0)
    if err > ACCEL_CHECK_TOLERANCE:
        raise CalSolveError(
            f"check pose reads {err * 100:.1f}% off g (bound "
            f"{ACCEL_CHECK_TOLERANCE * 100:.0f}%) — a pose moved during "
            "capture; nothing uploaded")
    return err
