"""Tests for `_looks_like_vm_restart` — the per-iteration heuristic
that distinguishes natural u16 sample-seq wraparound (which the
masked-delta math handles correctly) from a VM restart that resets
the seq counter to a low value. Without this distinction the host
meter reports sensor rates in the kHz/MHz range when the FW restarts
the slot mid-stream."""
from nxs.cli import _looks_like_vm_restart


def test_no_prev_seq_returns_false():
    """First sample of a stream has no predecessor to compare against."""
    assert _looks_like_vm_restart(None, 0) is False
    assert _looks_like_vm_restart(None, 42) is False


def test_consecutive_samples_not_restart():
    """seq=10 → 11 → 12: per-sample delta of 1, never a restart."""
    assert _looks_like_vm_restart(10, 11) is False
    assert _looks_like_vm_restart(11, 12) is False


def test_dropped_samples_not_restart():
    """Lossy link drops some samples — seq jumps forward by a small
    amount (10s or 100s). Still not a restart."""
    assert _looks_like_vm_restart(100, 120) is False
    assert _looks_like_vm_restart(100, 500) is False
    assert _looks_like_vm_restart(100, 999) is False


def test_u16_wraparound_not_restart():
    """seq=65535 → 0 is the natural u16 wrap. Masked delta = 1, well
    under threshold. The whole reason for the threshold-based check is
    that the masked math handles this case correctly on its own."""
    assert _looks_like_vm_restart(65535, 0) is False
    assert _looks_like_vm_restart(65534, 2) is False  # wrap + 2 missed


def test_restart_to_zero_detected():
    """The canonical case from the bench: seq=237 → 0 after the runner
    reloaded the slot. Masked delta = 65299 (>>1000)."""
    assert _looks_like_vm_restart(237, 0) is True


def test_restart_to_nonzero_detected():
    """VM restart doesn't always produce seq=0 on the first sample
    received — the runner may emit a few before the first one reaches
    the host. Any low-seq value after a high-seq predecessor trips."""
    assert _looks_like_vm_restart(5000, 50) is True
    assert _looks_like_vm_restart(30000, 100) is True


def test_threshold_is_overridable():
    """For unit-test scenarios that want to simulate a tighter or
    looser detection criterion."""
    # Looser: a 2000-sample jump tolerated.
    assert _looks_like_vm_restart(100, 2050, threshold=3000) is False
    # Tighter: a 50-sample jump treated as restart.
    assert _looks_like_vm_restart(100, 200, threshold=50) is True


def test_just_below_threshold_not_restart():
    """A clean boundary check — delta of exactly 1000 must NOT trip
    (threshold is `> 1000`, not `>=`)."""
    assert _looks_like_vm_restart(100, 1100) is False
    assert _looks_like_vm_restart(100, 1101) is True
