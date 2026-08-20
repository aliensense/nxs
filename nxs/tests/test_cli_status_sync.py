"""`nxs status` sync line — the per-road reader the time-sync runbook's
cross-transport step relies on: bound/offset/source when disciplined, a
bare dash when stale."""
import argparse

from nxs.cli import cmd_status
from nxs.transports.mock import MockTransport


def _args():
    return argparse.Namespace(transport='i2c', bus='/dev/i2c-2', addr=0x30)


def test_status_sync_line_when_disciplined(capsys):
    t = MockTransport()
    t.push_time_sync(-123456, 220, 0, 100_000_000)
    assert cmd_status(t, _args()) == 0
    out = capsys.readouterr().out
    assert "Sync:" in out
    assert "±220 µs" in out
    assert "-123456" in out
    assert "host" in out


def test_status_sync_dash_when_never_synced(capsys):
    assert cmd_status(MockTransport(), _args()) == 0
    out = capsys.readouterr().out
    assert "Sync:    -" in out


def test_status_prints_the_build_identity(capsys):
    assert cmd_status(MockTransport(), _args()) == 0
    out = capsys.readouterr().out
    assert f"FW:      {MockTransport.FW_VERSION}" in out
