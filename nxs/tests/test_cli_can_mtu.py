"""Tests for the $NXS_CAN_MTU-derived --mtu default — argparse never checks a
default against choices, so `_default_can_mtu()` owns the vocabulary: 8 and 64
pass through, anything else (off-vocabulary or non-integer) falls back to 64
with a stderr note instead of riding silently into the CAN transport.
"""
from nxs.cli import _default_can_mtu


def test_unset_env_defaults_to_fd(monkeypatch, capsys):
    monkeypatch.delenv('NXS_CAN_MTU', raising=False)
    assert _default_can_mtu() == 64
    assert capsys.readouterr().err == ""


def test_classic_env_passes_through(monkeypatch, capsys):
    monkeypatch.setenv('NXS_CAN_MTU', '8')
    assert _default_can_mtu() == 8
    assert capsys.readouterr().err == ""


def test_off_vocabulary_env_falls_back_loudly(monkeypatch, capsys):
    monkeypatch.setenv('NXS_CAN_MTU', '32')
    assert _default_can_mtu() == 64
    assert "ignoring NXS_CAN_MTU" in capsys.readouterr().err


def test_garbage_env_falls_back_loudly(monkeypatch, capsys):
    monkeypatch.setenv('NXS_CAN_MTU', 'jumbo')
    assert _default_can_mtu() == 64
    assert "ignoring NXS_CAN_MTU" in capsys.readouterr().err
