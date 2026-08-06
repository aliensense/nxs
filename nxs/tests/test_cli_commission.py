"""Tests for the `nxs commission` verb dispatch — `--subject NAME=ID` and
`--can-bitrate NOM[/DATA]` parsing and the commit hand-off to the transport.

The contract: a malformed `--subject` or `--can-bitrate` (missing `=`, a
non-integer ID, non-integer rates) fails cleanly with rc=1 and a user-facing
message, and never reaches the transport; a well-formed request parses values
in base-0 and hands the node, topic, and bitrate values to `commission()`.
"""
import argparse

from nxs.cli import cmd_commission
from nxs.client import (SupportsBitTiming, SupportsCanTermination,
                        SupportsCommissioning)


class _FakeTransport(SupportsCommissioning):
    """Records the commission() hand-off, like an identity-capable transport."""

    def __init__(self):
        self._calls = []

    def commission(self, node_addr=None, topics=None, can_bitrate=None,
                   can_term=None):
        self._calls.append((node_addr, topics, can_bitrate, can_term))

    def read_identity(self):
        return {"node_addr": 125, "topics": {"sample": 6144}}

    def calls(self):
        return self._calls


class _FakeBitTimingTransport(_FakeTransport, SupportsBitTiming,
                              SupportsCanTermination):
    """Commissioning transport that also serves the bit-timing pair
    and the termination selection."""

    def read_can_bitrate(self):
        return (1000000, 4000000)

    def write_can_bitrate(self, nominal, data):
        self._calls.append(("bitrate", nominal, data))

    def read_can_term(self):
        return 1

    def write_can_term(self, value):
        self._calls.append(("term", value))


def _args(node_id=None, subject=None, can_bitrate=None, can_term=None,
          show=False, save=False, transport='cyphal-can'):
    return argparse.Namespace(node_id=node_id, subject=subject or [],
                              can_bitrate=can_bitrate, can_term=can_term,
                              show=show, save=save, transport=transport)


def test_non_integer_subject_is_clean_error(capsys):
    t = _FakeTransport()
    assert cmd_commission(t, _args(subject=['sample=abc'])) == 1
    assert "must be an integer" in capsys.readouterr().err
    assert t.calls() == []                       # a parse error commits nothing


def test_subject_missing_equals_is_clean_error(capsys):
    t = _FakeTransport()
    assert cmd_commission(t, _args(subject=['sample'])) == 1
    assert "expects NAME=ID" in capsys.readouterr().err
    assert t.calls() == []


def test_valid_commission_hands_values_to_transport(capsys):
    t = _FakeTransport()
    assert cmd_commission(t, _args(node_id=10, subject=['acceleration=6246'])) == 0
    assert t.calls() == [(10, {"acceleration": 6246}, None, None)]
    assert "committed" in capsys.readouterr().out


def test_subject_id_parsed_base0(capsys):
    t = _FakeTransport()
    assert cmd_commission(t, _args(subject=['acceleration=0x1806'])) == 0
    assert t.calls() == [(None, {"acceleration": 0x1806}, None, None)]


def test_bare_can_bitrate_selects_classic(capsys):
    t = _FakeTransport()
    assert cmd_commission(t, _args(can_bitrate='500000')) == 0
    assert t.calls() == [(None, None, (500000, 500000), None)]
    assert "committed" in capsys.readouterr().out


def test_can_bitrate_pair_hands_both_rates(capsys):
    t = _FakeTransport()
    assert cmd_commission(t, _args(node_id=12, can_bitrate='1000000/4000000')) == 0
    assert t.calls() == [(12, None, (1000000, 4000000), None)]


def test_malformed_can_bitrate_is_clean_error(capsys):
    t = _FakeTransport()
    assert cmd_commission(t, _args(can_bitrate='fast/faster')) == 1
    assert "NOMINAL[/DATA]" in capsys.readouterr().err
    assert t.calls() == []


def test_show_prints_bitrate_when_supported(capsys):
    assert cmd_commission(_FakeBitTimingTransport(), _args(show=True)) == 0
    out = capsys.readouterr().out
    assert "can-bitrate" in out
    assert "1000000/4000000 (fd)" in out


def test_show_omits_bitrate_when_unsupported(capsys):
    assert cmd_commission(_FakeTransport(), _args(show=True)) == 0
    assert "can-bitrate" not in capsys.readouterr().out


def test_can_term_word_hands_value(capsys):
    t = _FakeTransport()
    assert cmd_commission(t, _args(can_term='on')) == 0
    assert t.calls() == [(None, None, None, 1)]
    assert "termination applies live" in capsys.readouterr().out


def test_malformed_can_term_is_clean_error(capsys):
    t = _FakeTransport()
    assert cmd_commission(t, _args(can_term='maybe')) == 1
    assert "on|off|default" in capsys.readouterr().err
    assert t.calls() == []


def test_show_prints_can_term_when_supported(capsys):
    assert cmd_commission(_FakeBitTimingTransport(), _args(show=True)) == 0
    assert "can-term" in capsys.readouterr().out


def test_combined_bitrate_and_term_states_both_semantics(capsys):
    t = _FakeTransport()
    assert cmd_commission(t, _args(can_bitrate='500000', can_term='on')) == 0
    out = capsys.readouterr().out
    assert "termination applied live" in out
    assert "reboot" in out
