"""Suite scan's CAN sweep reports why it found nothing.

A user cannot otherwise distinguish "no CAN device on this bench" from
"your adapter has no kernel driver", "you forgot `ip link set up`", or
"nxs was installed without the [cyphal] extra" — every one of those
printed the same nothing.
"""
import nxs.suite.scan as scan
from nxs.suite.schema import LinkSpec


def _no_ifaces(monkeypatch, ifaces):
    monkeypatch.setattr(scan, "_can_interfaces", lambda: ifaces)


def test_no_can_interface_is_reported_when_a_link_is_declared(monkeypatch, capsys):
    _no_ifaces(monkeypatch, [])
    declared = [LinkSpec(transport="cyphal-can", iface="can1", node_id=125)]

    assert scan._scan_can(lambda *a, **k: None, declared) == []
    err = capsys.readouterr().err
    assert "no SocketCAN interface" in err
    assert "gs_usb" in err          # names the usual cause


def test_no_can_interface_stays_quiet_when_none_is_declared(monkeypatch, capsys):
    # A bench with no CAN at all should not nag on every scan.
    _no_ifaces(monkeypatch, [])
    assert scan._scan_can(lambda *a, **k: None, []) == []
    assert capsys.readouterr().err == ""


def test_a_down_interface_is_named_not_skipped_silently(monkeypatch, capsys):
    _no_ifaces(monkeypatch, [("can1", False)])
    opened = []

    assert scan._scan_can(lambda *a, **k: opened.append(1), []) == []
    assert "can1 is down" in capsys.readouterr().err
    assert opened == []             # never even tried to open it


def test_an_open_failure_names_the_reason(monkeypatch, capsys):
    _no_ifaces(monkeypatch, [("can1", True)])

    def boom(*a, **k):
        raise RuntimeError("the device probably doesn't support CAN-FD")

    assert scan._scan_can(boom, []) == []
    err = capsys.readouterr().err
    assert "can1" in err
    assert "CAN-FD" in err          # pycyphal's own diagnosis reaches the user
