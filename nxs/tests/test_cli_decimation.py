"""Tests for the `nxs decimation` verb dispatch — read/write routing to
the transport and the I²C per-subject rejection.

The contract: device-wide and per-subject factors both round-trip through
`read_decimation`/`write_decimation`; a transport that can't decimate, or a
per-subject request on a host-paced transport, fails cleanly with rc=1.
"""
import argparse

from nxs.cli import cmd_decimation
from nxs.transports.mock import MockTransport


class _FakeCyphal:
    """Device-wide and per-subject factors, like the Cyphal client."""

    def __init__(self):
        self.device = 1
        self.subjects = {}

    def read_decimation(self, subject=None):
        return self.subjects.get(subject, 0) if subject else self.device

    def write_decimation(self, value, subject=None):
        if subject:
            self.subjects[subject] = value
        else:
            self.device = value


class _FakeI2c:
    """Device-wide only; per-subject raises, like the register-map transport."""

    def __init__(self):
        self.device = 1

    def read_decimation(self, subject=None):
        if subject is not None:
            raise ValueError("per-subject decimation is Cyphal-only")
        return self.device

    def write_decimation(self, value, subject=None):
        if subject is not None:
            raise ValueError("per-subject decimation is Cyphal-only")
        self.device = value


def _args(value=None, subject=None, transport='cyphal-can'):
    return argparse.Namespace(value=value, subject=subject, transport=transport)


def test_device_wide_read(capsys):
    assert cmd_decimation(_FakeCyphal(), _args()) == 0
    assert "decimation[device] = 1" in capsys.readouterr().out


def test_device_wide_write_then_read(capsys):
    t = _FakeCyphal()
    assert cmd_decimation(t, _args(value=5)) == 0
    assert t.device == 5
    assert "decimation[device] = 5" in capsys.readouterr().out


def test_per_subject_write_then_read(capsys):
    t = _FakeCyphal()
    assert cmd_decimation(t, _args(value=25, subject='temperature')) == 0
    assert t.subjects['temperature'] == 25
    assert "decimation[temperature] = 25" in capsys.readouterr().out


def test_i2c_rejects_per_subject(capsys):
    assert cmd_decimation(_FakeI2c(), _args(subject='temperature', transport='i2c')) == 1
    assert "Cyphal-only" in capsys.readouterr().err


def test_mock_round_trips_the_device_gate():
    # Decimation is part of the base NxsClient contract on every
    # transport — there is no unsupported-transport refusal to test.
    t = MockTransport()
    t.write_decimation(5)
    assert t.read_decimation() == 5
