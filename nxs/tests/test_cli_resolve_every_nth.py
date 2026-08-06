"""Tests for `nxs stream` rate routing — `_configure_stream`
(which flag applies to which transport) and `_every_for_hz` (the
on-device egress decimation math).

The contract these lock: `--hz` sets the target output rate. On a
transport that decimates on-device it quantizes to a divisor of the
acquisition rate; on a host-paced transport (I²C) it sets the poll rate
instead.
"""
import argparse

import pytest

from nxs.cli import _configure_stream, _every_for_hz
from nxs.client import SupportsEgressDecimation


class _EgressStub(SupportsEgressDecimation):
    """A transport that decimates on-device; exposes get_param for the
    acquisition rate. Only the methods the routing touches exist."""

    def __init__(self, params: dict | None = None):
        self._params = params or {}

    def get_param(self, name: str) -> dict:
        if name not in self._params:
            raise KeyError(name)
        v = self._params[name]
        return {'name': name, 'current': v, 'default': v}


class _PollStub:
    """Mimics I²C: host-paced, no on-device egress. Records the poll rate
    set via set_output_rate so the routing can be checked."""

    def __init__(self):
        self.rate = None

    def set_output_rate(self, hz):
        self.rate = hz


def _args(hz=None):
    return argparse.Namespace(hz=hz)


# ── _every_for_hz (on-device egress math) ──────────────────────

def test_hz_divides_evenly():
    assert _every_for_hz(_EgressStub({"sample_rate": 1000}), 250) == 4


def test_hz_above_acq_rate_clamps_to_one():
    assert _every_for_hz(_EgressStub({"sample_rate": 250}), 500) == 1


def test_hz_rounds_to_nearest_divisor(capsys):
    assert _every_for_hz(_EgressStub({"sample_rate": 250}), 175) == 1
    out = capsys.readouterr().out
    assert "acq_rate=250" in out and "actual output=250.0 Hz" in out


def test_hz_without_sample_rate_param_errors():
    with pytest.raises(SystemExit) as exc:
        _every_for_hz(_EgressStub(), 100)
    assert "doesn't declare a 'sample_rate'" in str(exc.value)


# ── _configure_stream routing ──────────────────────────────────

def test_default_no_flags_every_one():
    assert _configure_stream(_PollStub(), _args()) == 1
    assert _configure_stream(_EgressStub(), _args()) == 1


def test_hz_on_egress_transport_computes_every_nth():
    assert _configure_stream(_EgressStub({"sample_rate": 1000}),
                             _args(hz=250)) == 4


def test_hz_on_host_paced_transport_sets_poll_rate():
    t = _PollStub()
    assert _configure_stream(t, _args(hz=50)) == 1  # no egress decimation
    assert t.rate == 50                             # poll rate set instead


def test_rejects_nonpositive_hz():
    # --hz 0 must never reach _every_for_hz's division or set a negative
    # I²C poll interval; the boundary rejects it on both transports.
    for bad in (0, -5):
        with pytest.raises(SystemExit) as exc:
            _configure_stream(_PollStub(), _args(hz=bad))
        assert "positive rate" in str(exc.value)
        with pytest.raises(SystemExit):
            _configure_stream(_EgressStub({"sample_rate": 1000}), _args(hz=bad))
