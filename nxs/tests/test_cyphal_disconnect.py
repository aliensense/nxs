"""Offline coverage for the mid-session serial-disconnect UX in
CyphalControlClient. pycyphal catches-and-logs its disconnect cascade (reader
thread + publisher ticks) through its own loggers, so the only lever is the
logger level; the drop itself is detected structurally — the port vanishes from
/dev. No bus, no DSDL, no hardware: clients use autostart=False so pycyphal is
never started.
"""

import logging
import sys

import pytest

from nxs.transports import cyphal_control as cc
from nxs.transports.cyphal_control import CyphalControlClient


@pytest.fixture
def restore_pycyphal_logging():
    lg = logging.getLogger("pycyphal")
    saved_level, saved_flag = lg.level, cc._BG_QUIETED
    yield
    lg.setLevel(saved_level)
    cc._BG_QUIETED = saved_flag


# ── logging cap ───────────────────────────────────────────

def test_quiet_background_logging_caps_pycyphal(restore_pycyphal_logging):
    cc._BG_QUIETED = False
    logging.getLogger("pycyphal").setLevel(logging.NOTSET)
    cc._quiet_background_logging()
    assert logging.getLogger("pycyphal").level == logging.CRITICAL
    # The reader-thread / publisher errors come from child loggers, which
    # inherit the cap: their ERROR records are dropped, CRITICAL still passes.
    child = logging.getLogger("pycyphal.transport.serial._serial")
    assert not child.isEnabledFor(logging.ERROR)
    assert child.isEnabledFor(logging.CRITICAL)


def test_quiet_background_logging_is_idempotent(restore_pycyphal_logging):
    cc._BG_QUIETED = False
    cc._quiet_background_logging()
    logging.getLogger("pycyphal").setLevel(logging.DEBUG)  # someone lowers it later
    cc._quiet_background_logging()                          # guarded — must not re-cap
    assert logging.getLogger("pycyphal").level == logging.DEBUG


# ── structural drop detection ─────────────────────────────

def test_link_dropped_tracks_port_existence(tmp_path):
    port = tmp_path / "cu.usbmodem-test"
    port.write_text("")                       # stand in for a present /dev node
    c = CyphalControlClient(port=str(port), autostart=False)
    assert c.link_dropped() is False          # port present → not dropped
    port.unlink()
    assert c.link_dropped() is True           # port vanished → dropped


def test_link_dropped_false_for_can_and_no_port():
    assert CyphalControlClient(can_iface="can0", autostart=False).link_dropped() is False
    assert CyphalControlClient(port=None, autostart=False).link_dropped() is False


def test_disconnect_message_names_the_cause():
    msg = CyphalControlClient(port="(x)", autostart=False).disconnect_message()
    assert "USB device disconnected" in msg
    assert "USB-UART adapter" in msg


# ── the base default is inert (i2c / mock unaffected) ─────

def test_base_client_never_reports_a_drop():
    from nxs.client import NxsClient
    assert NxsClient.link_dropped(None) is False   # base default (self unused)
    assert "dropped" in NxsClient.disconnect_message(None)


# ── one-shot serial open (the J-Link V9 VCOM wedge guard) ─

def test_serial_baud_is_set_before_open():
    from nxs.serial_util import _ConfigureOnceSerial
    s = _ConfigureOnceSerial(baudrate=460800)   # no port → constructed unopened
    assert not s.is_open
    assert s.baudrate == 460800   # programmed at open, not re-coded after


def test_serial_config_is_frozen_after_open():
    from nxs.serial_util import _ConfigureOnceSerial
    s = _ConfigureOnceSerial()
    s._configured = True          # the state open() leaves behind
    s.is_open = True              # without the freeze, the assignment below
    s.timeout = 1.0               # would reprogram a device that isn't there
    assert s.timeout == 1.0       # host-side state still updates


def test_open_serial_once_normalizes_errors():
    import serial
    from nxs.serial_util import open_serial_once
    with pytest.raises(serial.SerialException):
        open_serial_once("(no-such-device)", -1)   # ValueError → SerialException


# ── CLI construction guard: drop during open vs plain typo ─

def test_cli_reports_a_port_that_vanishes_during_open(tmp_path, monkeypatch):
    from nxs import cli
    port = tmp_path / "cu.usbmodem-test"
    port.write_text("")

    def wedge(kind, **kw):                    # the device dies as pycyphal opens it
        port.unlink()
        raise RuntimeError("SerialTransport(...) is closed")

    monkeypatch.setattr(cli, "open_client", wedge)
    monkeypatch.setattr(sys, "argv",
                        ["nxs", "-t", "cyphal-serial", "-p", str(port), "probe"])
    with pytest.raises(SystemExit) as ex:
        cli.main()
    assert "vanished while opening it" in str(ex.value.code)
    assert "USB-UART adapter" in str(ex.value.code)


def test_cli_keeps_the_generic_hint_for_a_never_present_port(monkeypatch):
    from nxs import cli

    def refuse(kind, **kw):
        raise OSError("[Errno 2] could not open port")

    monkeypatch.setattr(cli, "open_client", refuse)
    monkeypatch.setattr(sys, "argv",
                        ["nxs", "-t", "cyphal-serial", "-p", "/dev/cu.no-such", "probe"])
    with pytest.raises(SystemExit) as ex:
        cli.main()
    assert "check -t/-b/-p" in str(ex.value.code)
