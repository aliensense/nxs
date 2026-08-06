"""Tests for the TimeSyncEstimator estimator and its transport feeds.

Observations carry explicit host timestamps, so every filter property —
min-RTT selection, drift extrapolation, window expiry — is asserted
deterministically; no clocks are mocked.
"""

import struct
import time

import pytest

from nxs.client import TimeSyncEstimator
from nxs.transports.mock import MockTransport

US = 1000          # ns per µs
S = 1_000_000_000  # ns per s


def _observe(ts, at_ns, offset_ns, rtt_ns=100 * US):
    """One exchange at host time `at_ns` whose midpoint implies
    host = device + offset_ns."""
    device_us = (at_ns - offset_ns) // 1000
    ts.observe(at_ns - rtt_ns // 2, at_ns + rtt_ns // 2, device_us)


def test_single_observation_projects_offset():
    ts = TimeSyncEstimator()
    assert not ts.ready()
    assert ts.project_mono_ns(0) is None

    _observe(ts, at_ns=10 * S, offset_ns=3 * S)
    assert ts.ready()
    # A device stamp taken at the observation instant maps back to it.
    assert ts.project_mono_ns(7_000_000) == pytest.approx(10 * S, abs=US)


def test_min_rtt_observation_wins():
    ts = TimeSyncEstimator()
    # A tight exchange with the true offset, then loose ones whose
    # midpoints are skewed by asymmetric delay.
    _observe(ts, at_ns=10 * S, offset_ns=3 * S, rtt_ns=80 * US)
    for k in range(1, 5):
        _observe(ts, at_ns=(10 + k) * S, offset_ns=3 * S + 900 * US,
                 rtt_ns=4000 * US)
    got = ts.project_mono_ns(7_000_000)
    assert got == pytest.approx(10 * S, abs=US)
    assert ts.bound_us() == pytest.approx(40.0)


def test_drift_is_extrapolated():
    ts = TimeSyncEstimator()
    # Device runs 100 ppm fast: the host-minus-device offset shrinks by
    # 100 µs per second of host time.
    drift = -100e-6
    for k in range(11):
        at = (10 + k) * S
        _observe(ts, at_ns=at, offset_ns=int(3 * S + drift * (at - 10 * S)))
    # Project one second past the newest observation: without the drift
    # term the answer would be off by ~100 µs.
    at = 21 * S
    device_us = (at - int(3 * S + drift * (at - 10 * S))) // 1000
    assert ts.project_mono_ns(device_us) == pytest.approx(at, abs=20 * US)


def test_window_expiry_drops_stale_observations():
    ts = TimeSyncEstimator()
    _observe(ts, at_ns=10 * S, offset_ns=3 * S, rtt_ns=80 * US)
    # A full window later, only the recent (wider) observation remains,
    # so the bound reflects it.
    _observe(ts, at_ns=10 * S + int(TimeSyncEstimator.WINDOW_S * S) + S,
             offset_ns=3 * S, rtt_ns=2000 * US)
    assert ts.bound_us() == pytest.approx(1000.0)


def test_project_to_realtime_shape():
    ts = TimeSyncEstimator()
    now = time.monotonic_ns()
    _observe(ts, at_ns=now, offset_ns=5 * S)
    sec, nanosec = ts.project_to_realtime((now - 5 * S) // 1000)
    assert 0 <= nanosec < S
    # The projection lands near the wall clock "now".
    assert sec == pytest.approx(time.time_ns() / S, abs=1.0)


def test_time_sync_ping_uses_transport_surface():
    class _Timed(MockTransport):
        def read_device_time_us(self):
            return 42_000_000

    t = _Timed()
    assert t.time_sync_ping()
    assert t.get_time_sync().ready()
    # A transport keeping the base default (no time surface) pings False.
    class _Bare(MockTransport):
        def read_device_time_us(self):
            return None

    bare = _Bare()
    assert not bare.time_sync_ping()
    assert not bare.get_time_sync().ready()


def test_i2c_record_poll_feeds_the_estimator():
    from nxs.tests.test_client_conformance import _StreamingI2cBus
    from nxs.transports.i2c import NxsI2cTransport

    t = NxsI2cTransport(_bus_obj=_StreamingI2cBus())
    s = None
    deadline = time.monotonic() + 2.0
    while s is None and time.monotonic() < deadline:
        s = t.poll_sample(timeout=0.1)
    assert s is not None
    # The record carried the acquisition timestamp and the latch fed a
    # sync observation — no ping needed on I²C.
    assert s.timestamp_us is not None
    assert t.get_time_sync().ready()
    raw_val = struct.unpack('>h', s.raw)[0]
    assert raw_val == 100 + s.count


def test_high_rate_observations_are_thinned():
    # I2C observes per poll (5x the sample rate); without a spacing
    # floor the window would hold tens of thousands of entries and
    # every projection would scan them. Equal-quality exchanges inside
    # the floor are dropped.
    ts = TimeSyncEstimator()
    for k in range(10_000):
        _observe(ts, at_ns=10 * S + k * 1_000_000, offset_ns=3 * S)
    assert len(ts._obs) < 300


def test_better_round_trip_survives_thinning():
    # A tighter exchange right after a kept one must still be kept —
    # the min-RTT filter feeds on exactly those.
    ts = TimeSyncEstimator()
    _observe(ts, at_ns=10 * S, offset_ns=3 * S, rtt_ns=200 * US)
    _observe(ts, at_ns=10 * S + 1_000_000, offset_ns=3 * S, rtt_ns=80 * US)
    assert ts.bound_us() == 80 / 2


def test_estimator_learns_the_clock_rate():
    # A device clock running 400 ppm fast makes the observed offset
    # shrink at 400 µs per second of host time; the fitted rate is the
    # pushable ppb form of that slope.
    ts = TimeSyncEstimator()
    for k in range(0, 121):
        at = 10 * S + k * S
        _observe(ts, at_ns=at, offset_ns=-(at * 400) // 1_000_000)
    rate = ts.rate_ppb()
    assert -420_000 <= rate <= -380_000


def test_rate_is_zero_before_a_baseline():
    ts = TimeSyncEstimator()
    _observe(ts, at_ns=10 * S, offset_ns=3 * S)
    assert ts.rate_ppb() == 0


def test_rate_survives_thinning_with_offset_freshness():
    # The deque spans the long rate window while the bound keeps
    # serving the recent best exchange.
    ts = TimeSyncEstimator()
    for k in range(0, 121):
        at = 10 * S + k * S
        rtt = (200 * US) if k < 60 else (80 * US)
        _observe(ts, at_ns=at, offset_ns=-(at * 400) // 1_000_000,
                 rtt_ns=rtt)
    assert ts.bound_us() == 80 / 2
    assert -420_000 <= ts.rate_ppb() <= -380_000
