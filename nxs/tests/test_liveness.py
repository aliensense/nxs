"""LivenessTracker state machine tests with a fake monotonic clock.

The contract these lock: WAITING/CONNECTED/DISCONNECTED transitions
fire exactly once per edge, in-state ticks are silent, and reconnect
after a firmware swap surfaces the new version in the log."""
import logging

from nxs.liveness import (
    HeartbeatSnapshot, LinkState, LivenessTracker,
)


class _FakeClock:
    def __init__(self, t0: float = 0.0):
        self.t = t0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def _hb(sender: int = 0, fw: str = "v1.0.0") -> HeartbeatSnapshot:
    return HeartbeatSnapshot(uptime_us=0, hw_id=1, sender=sender,
                             fw_version=fw, serial_hex="00" * 12)


def test_first_heartbeat_logs_up(caplog):
    clock = _FakeClock()
    t = LivenessTracker(timeout_s=2.0, now_fn=clock)
    with caplog.at_level(logging.INFO, logger="nxs.liveness"):
        t.on_heartbeat(0x10, _hb())
    assert t.state_of(0x10) == LinkState.CONNECTED
    assert "NXS 0x10 up" in caplog.text
    assert "fw=v1.0.0" in caplog.text


def test_steady_state_heartbeats_are_silent(caplog):
    clock = _FakeClock()
    t = LivenessTracker(timeout_s=2.0, now_fn=clock)
    t.on_heartbeat(0x10, _hb())
    caplog.clear()
    with caplog.at_level(logging.INFO, logger="nxs.liveness"):
        for _ in range(10):
            clock.advance(1.0)
            t.on_heartbeat(0x10, _hb())
            t.tick()
    assert caplog.text == "", f"unexpected log output: {caplog.text!r}"


def test_disconnect_after_timeout_logs_once(caplog):
    clock = _FakeClock()
    t = LivenessTracker(timeout_s=2.0, now_fn=clock)
    t.on_heartbeat(0x10, _hb())
    clock.advance(3.0)
    with caplog.at_level(logging.WARNING, logger="nxs.liveness"):
        t.tick()
        t.tick()  # second tick must not re-log
        t.tick()
    assert t.state_of(0x10) == LinkState.DISCONNECTED
    assert caplog.text.count("NXS 0x10 down") == 1


def test_reconnect_logs_with_fw_swap(caplog):
    clock = _FakeClock()
    t = LivenessTracker(timeout_s=2.0, now_fn=clock)
    t.on_heartbeat(0x10, _hb(fw="v1.0.0"))
    clock.advance(3.0)
    t.tick()  # disconnected
    caplog.clear()
    with caplog.at_level(logging.INFO, logger="nxs.liveness"):
        t.on_heartbeat(0x10, _hb(fw="v1.0.1"))
    assert t.state_of(0x10) == LinkState.CONNECTED
    assert "NXS 0x10 back" in caplog.text
    assert "v1.0.0" in caplog.text and "v1.0.1" in caplog.text


def test_reconnect_logs_with_sender_flip(caplog):
    """The clearest signal of "device rebooted into bootloader" is the
    sender field flipping app(0) → bootloader(1) on reconnect."""
    clock = _FakeClock()
    t = LivenessTracker(timeout_s=2.0, now_fn=clock)
    t.on_heartbeat(0x10, _hb(sender=0))
    clock.advance(3.0)
    t.tick()
    caplog.clear()
    with caplog.at_level(logging.INFO, logger="nxs.liveness"):
        t.on_heartbeat(0x10, _hb(sender=1))
    assert "application→bootloader" in caplog.text


def test_multi_unit_independent_state():
    clock = _FakeClock()
    t = LivenessTracker(timeout_s=2.0, now_fn=clock)
    t.on_heartbeat(0x10, _hb())
    t.on_heartbeat(0x11, _hb())
    clock.advance(3.0)
    t.on_heartbeat(0x10, _hb())  # keep 0x10 alive
    t.tick()
    assert t.state_of(0x10) == LinkState.CONNECTED
    assert t.state_of(0x11) == LinkState.DISCONNECTED


def test_is_connected_guard_for_spam_suppression():
    """Callers in the transports guard noisy logs by
    `if tracker.is_connected(src): log.warning(...)` — verify the
    guard returns False once disconnected."""
    clock = _FakeClock()
    t = LivenessTracker(timeout_s=2.0, now_fn=clock)
    assert not t.is_connected(0x10)  # WAITING
    t.on_heartbeat(0x10, _hb())
    assert t.is_connected(0x10)
    clock.advance(3.0)
    t.tick()
    assert not t.is_connected(0x10)


def test_unknown_unit_state_defaults_waiting():
    t = LivenessTracker()
    assert t.state_of(0x42) == LinkState.WAITING
    assert t.snapshot_of(0x42) is None
