"""Tests for `nxs upload` argument polymorphism.

The one verb resolves its positional three ways: an existing `.nxs`
file is uploaded verbatim (params already baked in), a driver name is
compiled from `drivers/<name>.py` then uploaded, and an unresolvable
argument errors toward the driver skill rather than guessing. `-o`
turns the compile into an offline producer (no device) — the source of
the `.nxs` files a `yakut file-server` serves to a fleet.
"""
import argparse

from nxs.cli import cmd_upload
from nxs.drivers.iam20680 import Iam20680
from nxs.image import serialize


class _FakeTransport:
    """Records what `upload` sent to the device. Tests follow the same
    rule as production code: private members behind accessors."""

    def __init__(self):
        self._uploaded = None
        self._ran = False

    def upload_image(self, img):
        self._uploaded = bytes(img)

    def vm_run(self):
        self._ran = True

    def uploaded(self):
        return self._uploaded

    def ran(self):
        return self._ran

    def read_driver_name(self):
        # The device reports a driver name once it has come up after RUN;
        # `_upload_and_run` polls this to confirm the driver loaded.
        return "iam20680" if self._ran else ""


def _args(driver, config=None, output=None):
    return argparse.Namespace(driver=driver, config=config, output=output)


# ── (b) name → compile + upload + run ──────────────────────────

def test_name_compiles_uploads_and_runs():
    t = _FakeTransport()
    assert cmd_upload(t, _args('iam20680')) == 0
    assert t.uploaded() == serialize(Iam20680().compile({}))
    assert t.ran() is True


# ── (a) existing file → upload verbatim ────────────────────────

def test_file_uploads_bytes_verbatim(tmp_path):
    img = serialize(Iam20680().compile({}))
    nxs = tmp_path / "iam20680.nxs"
    nxs.write_bytes(img)

    t = _FakeTransport()
    assert cmd_upload(t, _args(str(nxs))) == 0
    assert t.uploaded() == img          # no recompile — bytes pass straight through
    assert t.ran() is True


def test_file_rejects_param(tmp_path, capsys):
    nxs = tmp_path / "x.nxs"
    nxs.write_bytes(b"\x00")
    t = _FakeTransport()
    assert cmd_upload(t, _args(str(nxs), config=['sample_rate=250'])) == 1
    assert t.uploaded() is None
    assert "baked in" in capsys.readouterr().err


def test_file_rejects_output(tmp_path, capsys):
    nxs = tmp_path / "x.nxs"
    nxs.write_bytes(b"\x00")
    t = _FakeTransport()
    assert cmd_upload(t, _args(str(nxs), output=str(tmp_path / "y.nxs"))) == 1
    assert t.uploaded() is None


# ── -o producer (no device) ────────────────────────────────────

def test_output_writes_image_without_device(tmp_path):
    out = tmp_path / "iam20680.nxs"
    # transport is None: -o must never reach a device.
    assert cmd_upload(None, _args('iam20680', output=str(out))) == 0
    assert out.read_bytes() == serialize(Iam20680().compile({}))


# ── (c) neither → skill-pointer error ──────────────────────────

def test_unknown_errors_toward_skill(capsys):
    assert cmd_upload(None, _args('definitely_not_a_driver')) == 1
    assert "skill" in capsys.readouterr().err.lower()


# ── reachability guard: unreachable node vs reachable-but-empty ──

class _Reachability:
    """Minimal transport whose `probe()` answer is fixed at construction;
    stands in for a node that is (un)reachable over the wire."""

    def __init__(self, reachable):
        self._reachable = reachable

    def probe(self):
        return self._reachable

    def read_driver_name(self):
        return ""            # what caps misreads as "No driver loaded"

    def read_capabilities(self):
        return []

    def link_dropped(self):
        return False

    def close(self):
        pass


def _run_cli(monkeypatch, transport, argv):
    import sys
    from nxs import cli
    monkeypatch.setattr(cli, "open_client", lambda *a, **k: transport)
    monkeypatch.setattr(sys, "argv", ["nxs", "-t", "cyphal-serial",
                                      "-p", "/dev/null"] + argv)
    return cli.main()


def test_caps_reports_unreachable_not_missing_driver(monkeypatch, capsys):
    """A no-answer node must not read as an empty one: `caps` against a
    device that fails `probe()` prints a reachability error, not the
    driver-state 'No driver loaded'."""
    rc = _run_cli(monkeypatch, _Reachability(reachable=False), ["caps"])
    assert rc == 1
    assert "NXS not found" in capsys.readouterr().err


def test_caps_still_reports_missing_driver_when_reachable(monkeypatch, capsys):
    """The legitimate empty-device path survives the guard: a reachable
    node with no driver still prints 'No driver loaded'."""
    rc = _run_cli(monkeypatch, _Reachability(reachable=True), ["caps"])
    assert rc == 1
    assert "No driver loaded" in capsys.readouterr().out
