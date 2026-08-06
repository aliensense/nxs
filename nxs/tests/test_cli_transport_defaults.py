"""Transport selection: $NXS_TRANSPORT / per-platform default, and a clean
exit (not a traceback) when the chosen device cannot be opened."""
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import pytest

from nxs import cli


def test_env_overrides_platform(monkeypatch):
    monkeypatch.setenv("NXS_TRANSPORT", "cyphal-can")
    monkeypatch.setattr(sys, "platform", "darwin")
    assert cli._default_transport() == "cyphal-can"


def test_env_transport_is_normalized(monkeypatch):
    """$NXS_TRANSPORT casing/whitespace is normalized, so 'I2C ' (or a value
    with a trailing newline) doesn't become a spurious unknown-transport."""
    monkeypatch.setenv("NXS_TRANSPORT", "  I2C\n")
    monkeypatch.setattr(sys, "platform", "linux")
    assert cli._default_transport() == "i2c"


def test_macos_defaults_to_cyphal_serial(monkeypatch):
    monkeypatch.delenv("NXS_TRANSPORT", raising=False)
    monkeypatch.setattr(sys, "platform", "darwin")
    assert cli._default_transport() == "cyphal-serial"


def test_linux_defaults_to_i2c(monkeypatch):
    monkeypatch.delenv("NXS_TRANSPORT", raising=False)
    monkeypatch.setattr(sys, "platform", "linux")
    assert cli._default_transport() == "i2c"


def test_windows_defaults_to_cyphal_serial(monkeypatch):
    monkeypatch.delenv("NXS_TRANSPORT", raising=False)
    monkeypatch.setattr(sys, "platform", "win32")
    assert cli._default_transport() == "cyphal-serial"


def test_open_failure_exits_clean(monkeypatch):
    """A device that cannot be opened (e.g. no /dev/i2c-* on a laptop) exits
    via SystemExit with a one-line message naming the transport — not an
    unhandled OSError traceback."""
    def raise_missing(args):
        raise FileNotFoundError(2, "No such file or directory", "/dev/i2c-2")

    monkeypatch.setattr(sys, "argv", ["nxs", "-t", "i2c", "probe"])
    monkeypatch.setattr(cli, "_open_transport", raise_missing)
    with pytest.raises(SystemExit) as exc:
        cli.main()
    msg = str(exc.value)
    assert "i2c" in msg and "i2c-2" in msg


def test_malformed_bus_exits_clean(monkeypatch):
    """A bus path with no /dev/i2c-N number exits clean, not on a ValueError
    traceback from the bus-number parse."""
    monkeypatch.setattr(
        sys, "argv", ["nxs", "-t", "i2c", "-b", "/dev/i2c-foo", "probe"])
    with pytest.raises(SystemExit) as exc:
        cli.main()
    assert "I2C bus" in str(exc.value)


def test_unknown_env_transport_exits_clean(monkeypatch):
    """A bad $NXS_TRANSPORT becomes the default, then exits clean at open."""
    monkeypatch.setenv("NXS_TRANSPORT", "bogus")
    monkeypatch.setattr(sys, "argv", ["nxs", "probe"])
    with pytest.raises(SystemExit) as exc:
        cli.main()
    assert "bogus" in str(exc.value)


def test_missing_transport_dep_exits_clean(monkeypatch):
    """A missing optional transport dep (e.g. smbus2) exits clean, not on an
    ImportError traceback."""
    def raise_import(args):
        raise ImportError("smbus2 is required: pip install smbus2")

    monkeypatch.setattr(sys, "argv", ["nxs", "-t", "i2c", "probe"])
    monkeypatch.setattr(cli, "_open_transport", raise_import)
    with pytest.raises(SystemExit) as exc:
        cli.main()
    assert "smbus2" in str(exc.value)


def test_cyphal_startup_error_exits_clean(monkeypatch):
    """A RuntimeError during transport open (e.g. _ensure_dsdl() finds no DSDL
    sources) exits clean, not on a traceback."""
    def raise_runtime(args):
        raise RuntimeError("no DSDL sources found — reinstall nxs[cyphal]")

    monkeypatch.setattr(sys, "argv", ["nxs", "-t", "cyphal-serial", "probe"])
    monkeypatch.setattr(cli, "_open_transport", raise_runtime)
    with pytest.raises(SystemExit) as exc:
        cli.main()
    assert "DSDL" in str(exc.value)


def test_verb_oserror_exits_clean(monkeypatch, capsys):
    """A bus error mid-verb (I2C NACK, permissions) is a clean error
    line and exit code 1, not a traceback — for every dispatched verb."""
    from nxs.client import SupportsIdentify

    class _Raises(SupportsIdentify):
        def probe(self):
            return True   # past the reachability gate; the verb itself fails

        def identify(self):
            raise OSError(121, "Remote I/O error")

        def link_dropped(self):
            return False

    monkeypatch.setattr(sys, "argv", ["nxs", "-t", "i2c", "identify"])
    monkeypatch.setattr(cli, "_open_transport", lambda args: _Raises())
    assert cli.main() == 1
    assert "transport I/O failed" in capsys.readouterr().err


def test_describe_target_i2c_names_bus_and_addr():
    args = cli.build_parser().parse_args(
        ["-t", "i2c", "-b", "/dev/i2c-9", "-a", "0x31", "probe"])
    assert cli._describe_target(args) == "i2c /dev/i2c-9@0x31"


def test_describe_target_serial_names_port_not_addr():
    """A Cyphal target is the port/node, never the (ignored) I2C address —
    `nxs -t cyphal-serial probe` used to report the default 0x30."""
    args = cli.build_parser().parse_args(
        ["-t", "cyphal-serial", "-p", "/dev/ttyUSB0", "probe"])
    assert cli._describe_target(args) == "serial /dev/ttyUSB0"


def test_describe_target_can_resolves_defaults():
    args = cli.build_parser().parse_args(["-t", "cyphal-can", "probe"])
    assert (cli._describe_target(args)
            == f"can can0 node {cli.CyphalDefaults.DEFAULT_NODE_ID}")

    args = cli.build_parser().parse_args(
        ["-t", "cyphal-can", "-p", "can1", "--remote-node-id", "10", "probe"])
    assert cli._describe_target(args) == "can can1 node 10"
