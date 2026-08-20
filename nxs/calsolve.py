"""Calibration solvers — pure Python, no numpy.

The device stores and applies one affine per vector bucket, and these
produce the coefficients from acquisition data. Every solver self-checks
its fit and raises `CalSolveError` on a result that would fail the
falsifiable bar, so a verb can never upload a plausible-but-bad solution.

All solves run in the sensor frame — the mounting orientation is a
separate record field composed on top by the appliers.
"""
import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

STANDARD_GRAVITY = 9.80665

# Fit-acceptance gates. Residuals are relative to the reference magnitude
# (gravity / local field), so the same bounds serve both solvers.
ACCEL_MAX_RESIDUAL = 0.05    # 5% of g on any captured pose
# Scale bounds on M's diagonal. A residual gate alone cannot catch a
# degenerate scale: a fit can reproduce every captured pose inside 5% while
# carrying a nonsense gain, because the poses only constrain the directions
# they were captured in. ArduPilot bounds its accel diagonal the same way
# (`AccelCalibrator`, 0.8..1.2) for the same reason.
ACCEL_MIN_SCALE = 0.8
ACCEL_MAX_SCALE = 1.2
MAG_MAX_SPREAD = 0.10        # 10% RMS spread of |B| after calibration
MIN_FIELD_FRACTION = 0.05    # reject a fit whose radius collapsed


class CalSolveError(RuntimeError):
    """A solve failed its validity gate; nothing may be uploaded."""


@dataclass
class VectorSolution:
    """One bucket's solved affine plus its fit diagnostics."""
    m: Tuple[float, ...]     # row-major 3x3
    b: Tuple[float, ...]
    rms_residual: float      # relative to the reference magnitude
    max_residual: float
    radius: float            # recovered reference magnitude (accel: g)


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


def eig_sym3(a: Sequence[Sequence[float]]):
    """Eigen-decompose a symmetric 3x3 by cyclic Jacobi rotations.

    Returns (eigenvalues, eigenvectors) with eigenvectors as columns:
    A = V·diag(w)·Vᵀ.
    """
    a = [list(row) for row in a]
    v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for _ in range(64):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda t: abs(a[t[0]][t[1]]))
        if abs(a[p][q]) < 1e-15:
            break
        theta = 0.5 * math.atan2(2.0 * a[p][q], a[q][q] - a[p][p])
        c = math.cos(theta)
        s = math.sin(theta)
        for k in range(3):
            akp = c * a[k][p] - s * a[k][q]
            akq = s * a[k][p] + c * a[k][q]
            a[k][p], a[k][q] = akp, akq
        for k in range(3):
            apk = c * a[p][k] - s * a[q][k]
            aqk = s * a[p][k] + c * a[q][k]
            a[p][k], a[q][k] = apk, aqk
        for k in range(3):
            vkp = c * v[k][p] - s * v[k][q]
            vkq = s * v[k][p] + c * v[k][q]
            v[k][p], v[k][q] = vkp, vkq
    return [a[0][0], a[1][1], a[2][2]], v


def _apply(m: Sequence[float], b: Sequence[float],
           v: Sequence[float]) -> List[float]:
    return [m[i * 3] * v[0] + m[i * 3 + 1] * v[1] + m[i * 3 + 2] * v[2] + b[i]
            for i in range(3)]


def solve_accel(captures: Sequence[Tuple[Sequence[float], Sequence[float]]],
                gravity: float = STANDARD_GRAVITY) -> VectorSolution:
    """12-param fit from still poses: reference = M·measured + b.

    ``captures`` pairs each pose's mean measured vector with its known
    gravity reference (±g on the dominant sensor axis). Solves each row of
    [M | b] as an independent 4-unknown least-squares over all poses, then
    verifies every pose reproduces its reference within the gate.
    """
    if len(captures) < 6:
        raise CalSolveError(f"need at least 6 poses, got {len(captures)}")
    rows = []
    bias = []
    for j in range(3):
        xtx = [[0.0] * 4 for _ in range(4)]
        xty = [0.0] * 4
        for measured, reference in captures:
            x = [measured[0], measured[1], measured[2], 1.0]
            for p in range(4):
                xty[p] += x[p] * reference[j]
                for q in range(4):
                    xtx[p][q] += x[p] * x[q]
        sol = solve_linear(xtx, xty)
        rows.extend(sol[:3])
        bias.append(sol[3])

    residuals = [max(abs(a - r) for a, r in
                     zip(_apply(rows, bias, measured), reference))
                 for measured, reference in captures]
    rms = math.sqrt(sum(r * r for r in residuals) / len(residuals)) / gravity
    worst = max(residuals) / gravity
    if worst > ACCEL_MAX_RESIDUAL:
        raise CalSolveError(
            f"accel fit residual {worst * 100:.1f}% of g exceeds "
            f"{ACCEL_MAX_RESIDUAL * 100:.0f}% — poses moved or mislabeled; "
            "nothing uploaded")
    for axis in range(3):
        scale = rows[axis * 3 + axis]
        if not ACCEL_MIN_SCALE <= scale <= ACCEL_MAX_SCALE:
            raise CalSolveError(
                f"accel fit scale {scale:.3f} on axis {axis} outside "
                f"{ACCEL_MIN_SCALE}..{ACCEL_MAX_SCALE} — a degenerate fit the "
                "residual gate cannot see; nothing uploaded")
    return VectorSolution(m=tuple(rows), b=tuple(bias), rms_residual=rms,
                          max_residual=worst, radius=gravity)

