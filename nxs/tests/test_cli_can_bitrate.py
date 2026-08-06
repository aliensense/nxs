"""Tests for the `nxs can-bitrate` verb dispatch — read/write routing to the
transport, the Classic-from-one-arg default, and the whitelist rejection.

The contract: reads print `nominal/data (fd|classic)`; one positional selects
Classic at that rate; the transport's ValueError (host-side whitelist) fails
cleanly with rc=1; a transport without bit-timing support fails cleanly.
"""
import argparse

from nxs.cli import cmd_can_bitrate
from nxs.client import SupportsBitTiming, validate_can_bitrate


class _FakeBitTiming(SupportsBitTiming):
    """Staged pair with the host-side whitelist, like the Cyphal client."""

    def __init__(self):
        self.pair = (1000000, 4000000)

    def read_can_bitrate(self):
        return self.pair

    def write_can_bitrate(self, nominal, data):
        validate_can_bitrate(nominal, data)
        self.pair = (1000000, 4000000) if nominal == 0 else (nominal, data)


class _NoBitTiming:
    """A transport that does not carry the CAN profile at all."""


def _args(nominal=None, data=None, transport='cyphal-can'):
    return argparse.Namespace(nominal=nominal, data=data, transport=transport)


def test_read_prints_fd_profile(capsys):
    assert cmd_can_bitrate(_FakeBitTiming(), _args()) == 0
    out = capsys.readouterr().out
    assert "can-bitrate = 1000000/4000000 (fd)" in out
    assert "staged" not in out                   # a read stages nothing


def test_one_arg_selects_classic(capsys):
    t = _FakeBitTiming()
    assert cmd_can_bitrate(t, _args(nominal=500000)) == 0
    assert t.pair == (500000, 500000)
    out = capsys.readouterr().out
    assert "can-bitrate = 500000/500000 (classic)" in out
    assert "staged" in out


def test_two_args_select_fd(capsys):
    t = _FakeBitTiming()
    assert cmd_can_bitrate(t, _args(nominal=1000000, data=2000000)) == 0
    assert t.pair == (1000000, 2000000)
    assert "can-bitrate = 1000000/2000000 (fd)" in capsys.readouterr().out


def test_zero_reverts_to_default(capsys):
    t = _FakeBitTiming()
    t.pair = (500000, 500000)
    assert cmd_can_bitrate(t, _args(nominal=0)) == 0
    assert t.pair == (1000000, 4000000)
    assert "can-bitrate = 1000000/4000000 (fd)" in capsys.readouterr().out


def test_off_whitelist_is_clean_error(capsys):
    t = _FakeBitTiming()
    assert cmd_can_bitrate(t, _args(nominal=300000)) == 1
    assert "unsupported CAN bitrate profile" in capsys.readouterr().err
    assert t.pair == (1000000, 4000000)          # nothing staged


def test_i2c_write_reports_committed(capsys):
    t = _FakeBitTiming()
    assert cmd_can_bitrate(t, _args(nominal=250000, transport='i2c')) == 0
    assert "committed" in capsys.readouterr().out


def test_unsupported_transport(capsys):
    assert cmd_can_bitrate(_NoBitTiming(), _args(transport='cyphal-serial')) == 1
    assert "not supported" in capsys.readouterr().err
