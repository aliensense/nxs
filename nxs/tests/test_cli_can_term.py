"""Tests for the `nxs can-term` verb dispatch — word/value parsing, the
live-apply hand-off to the transport, and the vocabulary rejection.

The contract: reads print `can-term = on|off`; `on`/`off`/`default` (and the
raw register values) route to `write_can_term`; an out-of-vocabulary value
fails cleanly with rc=1 and stages nothing; a transport without termination
support fails cleanly.
"""
import argparse

from nxs.cli import cmd_can_term
from nxs.client import SupportsCanTermination, validate_can_term


class _FakeCanTerm(SupportsCanTermination):
    """Effective-state selection with the host-side vocabulary check."""

    def __init__(self):
        self.value = 0

    def read_can_term(self):
        return self.value

    def write_can_term(self, value):
        validate_can_term(value)
        self.value = 0 if value == 0xFFFF else value


class _NoCanTerm:
    """A transport that does not carry the termination selection."""


def _args(value=None, transport='cyphal-can'):
    return argparse.Namespace(value=value, transport=transport)


def test_read_prints_off(capsys):
    assert cmd_can_term(_FakeCanTerm(), _args()) == 0
    out = capsys.readouterr().out
    assert "can-term = off" in out
    assert "applied" not in out                  # a read applies nothing


def test_word_on_applies(capsys):
    t = _FakeCanTerm()
    assert cmd_can_term(t, _args(value='on')) == 0
    assert t.value == 1
    out = capsys.readouterr().out
    assert "can-term = on" in out
    assert "applied" in out


def test_raw_value_applies(capsys):
    t = _FakeCanTerm()
    t.value = 1
    assert cmd_can_term(t, _args(value='0')) == 0
    assert t.value == 0
    assert "can-term = off" in capsys.readouterr().out


def test_default_reverts(capsys):
    t = _FakeCanTerm()
    t.value = 1
    assert cmd_can_term(t, _args(value='default')) == 0
    assert t.value == 0
    assert "can-term = off" in capsys.readouterr().out


def test_out_of_vocabulary_is_clean_error(capsys):
    t = _FakeCanTerm()
    assert cmd_can_term(t, _args(value='5')) == 1
    assert "unsupported can-term" in capsys.readouterr().err
    assert t.value == 0


def test_word_garbage_is_clean_error(capsys):
    assert cmd_can_term(_FakeCanTerm(), _args(value='maybe')) == 1
    assert "on|off|default" in capsys.readouterr().err


def test_unsupported_transport(capsys):
    assert cmd_can_term(_NoCanTerm(), _args(transport='cyphal-serial')) == 1
    assert "not supported" in capsys.readouterr().err
