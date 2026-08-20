"""Solver falsifiability: recover known distortions, refuse degenerate fits."""
import math
import random

import pytest

from nxs.calsolve import (CalSolveError, STANDARD_GRAVITY, eig_sym3,
                          solve_accel, solve_linear)

G = STANDARD_GRAVITY


def mat_vec(m, v):
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def apply_flat(m, b, v):
    return [m[i * 3] * v[0] + m[i * 3 + 1] * v[1] + m[i * 3 + 2] * v[2] + b[i]
            for i in range(3)]


def fibonacci_sphere(n):
    """Evenly spread unit directions — full rotation coverage."""
    golden = math.pi * (3.0 - math.sqrt(5.0))
    points = []
    for i in range(n):
        y = 1.0 - 2.0 * (i + 0.5) / n
        r = math.sqrt(1.0 - y * y)
        theta = golden * i
        points.append((r * math.cos(theta), y, r * math.sin(theta)))
    return points


def test_solve_linear_roundtrip():
    a = [[2.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 4.0]]
    x = [1.5, -2.0, 0.25]
    b = mat_vec(a, x)
    got = solve_linear(a, b)
    assert all(abs(g - e) < 1e-9 for g, e in zip(got, x))


def test_eig_sym3_reconstructs():
    a = [[4.0, 1.0, 0.5], [1.0, 3.0, 0.2], [0.5, 0.2, 2.0]]
    w, v = eig_sym3(a)
    for i in range(3):
        for j in range(3):
            recon = sum(v[i][t] * w[t] * v[j][t] for t in range(3))
            assert abs(recon - a[i][j]) < 1e-9


def _distorted_captures(distortion, offset):
    captures = []
    for axis in range(3):
        for sign in (1.0, -1.0):
            reference = [0.0, 0.0, 0.0]
            reference[axis] = sign * G
            measured = [sum(distortion[i][j] * reference[j] for j in range(3))
                        + offset[i] for i in range(3)]
            captures.append((measured, reference))
    return captures


def test_accel_recovers_known_distortion():
    distortion = [[1.02, 0.01, 0.00], [0.01, 0.97, 0.002], [0.0, 0.002, 1.05]]
    offset = [0.3, -0.2, 0.1]
    captures = _distorted_captures(distortion, offset)
    sol = solve_accel(captures)
    # The falsifiable check: every measured pose maps back onto gravity.
    for measured, reference in captures:
        got = apply_flat(sol.m, sol.b, measured)
        assert all(abs(g - r) < 1e-6 for g, r in zip(got, reference))
    assert sol.max_residual < 1e-6


def test_accel_rejects_mislabeled_pose():
    captures = _distorted_captures(
            [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]], [0.0, 0.0, 0.0])
    bad = list(captures)
    measured, reference = bad[0]
    bad[0] = (measured, [reference[1], reference[0], reference[2]])
    bad[0] = ([measured[0] + 3.0, measured[1], measured[2]], bad[0][1])
    with pytest.raises(CalSolveError):
        solve_accel(bad)


def test_accel_rejects_too_few_poses():
    with pytest.raises(CalSolveError):
        solve_accel(_distorted_captures(
                [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])[:4])


def test_accel_scale_bound_catches_what_the_residual_cannot():
    """A residual gate only constrains the directions the poses were captured
    in, so a fit can reproduce all six inside 5% of g and still carry a
    nonsense gain. ArduPilot bounds the diagonal for the same reason."""
    import math

    from nxs import calsolve

    g = calsolve.STANDARD_GRAVITY
    # A sensor reading half scale on Z: every pose is self-consistent, so the
    # solve reproduces its references, but the recovered gain is 2.0.
    captures = []
    for axis in range(3):
        for sign in (+1, -1):
            measured = [0.0, 0.0, 0.0]
            measured[axis] = sign * g * (0.5 if axis == 2 else 1.0)
            reference = [0.0, 0.0, 0.0]
            reference[axis] = sign * g
            captures.append((measured, reference))

    with pytest.raises(calsolve.CalSolveError) as excinfo:
        calsolve.solve_accel(captures)
    assert "scale" in str(excinfo.value)
    assert "axis 2" in str(excinfo.value)
