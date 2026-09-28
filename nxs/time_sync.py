"""Host-to-device clock discipline: the offset estimator and the push loop that keeps it fresh."""

import threading
import time
from collections import deque
from typing import Optional, Tuple

from nxs._generated_constants import RunnerStates


class TimeSyncEstimator:
    """Device-clock to host-clock offset estimator: the RFC 5905 clock filter
    over two-way observations (minimum-RTT exchange in a sliding window wins)."""

    WINDOW_S = 30.0
    RATE_WINDOW_S = 120.0
    MIN_DRIFT_BASELINE_S = 5.0
    MAX_DRIFT = 500e-6  # beyond ±500 ppm it's a glitch, not oscillator drift
    # Drift refresh cadence in observation time; the two window scans cost
    # milliseconds, so never pay them per projected sample.
    DRIFT_REFRESH_NS = 250_000_000
    # Observation spacing floor, so high-rate observers do not grow the window
    # unboundedly. One inside the floor is kept only when it improves the RTT.
    MIN_OBS_SPACING_NS = 50_000_000

    def __init__(self):
        self._lock = threading.Lock()
        self._drift_cache = 0.0
        self._drift_fit_ns = -10**18  # first observation always refits
        self._obs: deque = deque()  # (host_mid_ns, offset_ns, rtt_ns)
        # Best (minimum-RTT) exchange within WINDOW_S of the newest observation,
        # maintained on observe() so a projection never rescans the deque.
        self._recent_best = None

    def observe(self, t0_ns: int, t1_ns: int, device_us: int):
        """Record one two-way exchange: request sent at `t0_ns`, device clock
        read `device_us`, reply landed at `t1_ns`. Exchanges inside
        MIN_OBS_SPACING_NS are dropped unless they improve the round trip."""
        mid = (t0_ns + t1_ns) // 2
        entry = (mid, mid - device_us * 1000, t1_ns - t0_ns)
        horizon = mid - int(self.RATE_WINDOW_S * 1e9)
        with self._lock:
            if (self._obs
                    and mid - self._obs[-1][0] < self.MIN_OBS_SPACING_NS
                    and entry[2] >= self._obs[-1][2]):
                return
            self._obs.append(entry)
            while self._obs and self._obs[0][0] < horizon:
                self._obs.popleft()
            cut = mid - int(self.WINDOW_S * 1e9)
            self._recent_best = min(
                (o for o in self._obs if o[0] >= cut), key=lambda o: o[2])

    def project_mono_ns(self, device_us: int) -> Optional[int]:
        """Host CLOCK_MONOTONIC nanoseconds for a device timestamp, or
        None before the first observation."""
        with self._lock:
            if not self._obs:
                return None
            best = self._recent_best
            drift = self._drift_locked()
        dev_ns = device_us * 1000
        approx = dev_ns + best[1]
        return int(approx + drift * (approx - best[0]))

    def project_to_realtime(self, device_us: int) -> Optional[Tuple[int, int]]:
        """`(sec, nanosec)` of host CLOCK_REALTIME for a device timestamp, or
        None before the first observation. The realtime skew is sampled per call."""
        mono = self.project_mono_ns(device_us)
        if mono is None:
            return None
        skew = time.time_ns() - time.monotonic_ns()
        return divmod(mono + skew, 1_000_000_000)

    def ready(self) -> bool:
        with self._lock:
            return bool(self._obs)

    def bound_us(self) -> Optional[float]:
        """Half the best recent observation's round trip, the offset error bound
        in µs; None before the first observation."""
        with self._lock:
            if not self._obs:
                return None
            return self._recent_best[2] / 2000.0

    def rate_ppb(self) -> int:
        """The fitted device clock rate error in parts per billion; 0 until the
        fit's baseline exists."""
        with self._lock:
            return int(self._drift_locked() * 1e9)

    def _drift_locked(self) -> float:
        """Slope between the best exchanges of the window's oldest and newest
        thirds (0.0 until the baseline exists), refitted after DRIFT_REFRESH_NS."""
        if len(self._obs) < 4:
            return 0.0
        if self._obs[-1][0] - self._drift_fit_ns < self.DRIFT_REFRESH_NS:
            return self._drift_cache
        self._drift_fit_ns = self._obs[-1][0]
        span = self._obs[-1][0] - self._obs[0][0]
        if span < self.MIN_DRIFT_BASELINE_S * 1e9:
            self._drift_cache = 0.0
            return 0.0
        third = span // 3
        old = min((o for o in self._obs if o[0] <= self._obs[0][0] + third),
                  key=lambda o: o[2])
        new = min((o for o in self._obs if o[0] >= self._obs[-1][0] - third),
                  key=lambda o: o[2])
        if new[0] == old[0]:
            self._drift_cache = 0.0
            return 0.0
        drift = (new[1] - old[1]) / (new[0] - old[0])
        self._drift_cache = max(-self.MAX_DRIFT, min(self.MAX_DRIFT, drift))
        return self._drift_cache

PUSH_INTERVAL_S = 1.0

SYNC_LOST_PUSHES = 10

DRIVER_UP_TIMEOUT_S = 5.0

DRIVER_UP_POLL_S = 0.2

def await_driver_up(client, timeout_s: float = DRIVER_UP_TIMEOUT_S,
                    poll_s: float = DRIVER_UP_POLL_S) -> int:
    """Block until the runner settles after `vm_run()`, and return its state.
    `vm_run()` returns on the accept, not on the probe; settling is `MEASURING`
    or `PROBE_FAILED`, and a timeout returns the last state read."""
    state = RunnerStates.RunnerState
    deadline = time.monotonic() + timeout_s
    while True:
        current = client.read_runner_state()
        if current in (state.MEASURING, state.PROBE_FAILED):
            return current
        if time.monotonic() >= deadline:
            return current
        time.sleep(poll_s)

#: Budget for the time-sync mirror to catch up with a push, sized for a seed
#: issued straight after a panel deploy.
PUSH_ECHO_TIMEOUT_S = 2.0

PUSH_ECHO_POLL_S = 0.01

def estimate_and_push(client, pings: int = 8,
                      interval_s: float = PUSH_INTERVAL_S):
    """Refresh the client's estimator with a ping burst and push the result,
    valid for `SYNC_LOST_PUSHES` cycles of `interval_s`. Returns the pushed
    bound in µs, or None when the transport serves no time surface. The
    library's client runs the round on its own estimator."""
    round_ = getattr(client, "time_sync_round", None)
    if round_ is not None:
        # A round the unit did not answer raises: the caller decides whether
        # a deaf link ends it or is ridden out.
        return int(round_(pings, interval_s).bound_us)
    for _ in range(pings):
        if not client.time_sync_ping():
            return None
    estimator = client.get_time_sync()
    device_us = client.read_device_time_us()
    if device_us is None or not estimator.ready():
        return None
    projected = estimator.project_to_realtime(device_us)
    if projected is None:
        return None
    sec, nanosec = projected
    offset_us = sec * 1_000_000 + nanosec // 1000 - device_us
    bound_us = int(estimator.bound_us())
    rate_ppb = estimator.rate_ppb()
    valid_for_us = int(SYNC_LOST_PUSHES * interval_s * 1_000_000)
    client.push_time_sync(offset_us, bound_us, rate_ppb, valid_for_us)
    # The record applies on the comm thread, not in the transaction carrying
    # it, so poll the echo. Offset legitimately differs on a servo re-anchor.
    deadline = time.monotonic() + PUSH_ECHO_TIMEOUT_S
    while True:
        echoed = client.read_time_sync()
        if echoed is None:
            return bound_us
        _, echoed_bound, echoed_rate, echoed_window, _, echoed_valid = echoed
        if (echoed_valid and echoed_bound == bound_us
                and echoed_rate == rate_ppb and echoed_window == valid_for_us):
            return bound_us
        if time.monotonic() >= deadline:
            raise RuntimeError("push not applied (mirror mismatch)")
        time.sleep(PUSH_ECHO_POLL_S)
