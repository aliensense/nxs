"""The resident `nxs timesync` pusher's survival contract: a firmware push
holds the mux, then makes the device deaf for the erase and the reboot.
Neither may kill a daemon whose death silently decays mesh time."""
import argparse

import pytest

from nxs.cli import cmd_timesync
from nxs.client import DeviceRefused, SupportsTimeSync, XFER_EBUSY


class _Unit(SupportsTimeSync):
    """Serves the time surface; the pushes are scripted by the test."""

    def push_time_sync(self, offset_us, bound_us, rate_ppb, valid_for_us):
        pass

    def read_time_sync(self):
        return 0, 0, 0, 0, 0, False

    def read_device_time_us(self):
        return 0


def _args(once=False, interval=0.0):
    return argparse.Namespace(once=once, interval=interval)


def _run(monkeypatch, effects):
    """Drive the pusher over a scripted sequence, then stop it."""
    seen = iter(effects)

    def fake_push(t, interval_s=0.0):
        outcome = next(seen)
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome

    monkeypatch.setattr("nxs.client.estimate_and_push", fake_push)
    return cmd_timesync(_Unit(), _args())


def test_pusher_survives_the_erase_window_nack(monkeypatch, capsys):
    # Errno 121 (EREMOTEIO) is what the bus returns while the device
    # bulk-erases its staging slot; the daemon must skip, not die.
    with pytest.raises(StopIteration):
        _run(monkeypatch, [220,
                           DeviceRefused(XFER_EBUSY, "session live"),
                           OSError(121, "Remote I/O error"),
                           218])
    out = capsys.readouterr().out
    assert "mux held" in out
    assert "link unavailable" in out
    assert out.count("pushed") == 2      # it kept pushing afterwards


def test_one_shot_push_still_fails_on_a_bus_error(monkeypatch, capsys):
    def fake_push(t, interval_s=0.0):
        raise OSError(121, "Remote I/O error")

    monkeypatch.setattr("nxs.client.estimate_and_push", fake_push)
    assert cmd_timesync(_Unit(), _args(once=True)) == 1
