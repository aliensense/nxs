"""Offline coverage for CyphalControlClient — the verb→wire mapping, with the
pycyphal layer stubbed (autostart=False), so no bus or compiled DSDL is needed.
"""

from types import SimpleNamespace

import pytest

from nxs.client import DeviceRefused
from nxs.transports.cyphal_control import (
    CLEAR_STORE,
    CYCLE,
    DELETE_SLOT,
    RESET,
    RUN,
    SAVE,
    STOP,
    CyphalControlClient,
)


class _Fake(CyphalControlClient):
    """Records ExecuteCommand codes instead of sending them."""

    def __init__(self):
        super().__init__(port="(fake)", autostart=False)
        self.cmds = []

    def _execute(self, command, parameter=b""):
        self.cmds.append((command, bytes(parameter)))
        return True


class _Refusing(_Fake):
    """Every command replies FAILURE; cmd_error serves a preset errno."""

    def __init__(self, errno_code):
        super().__init__()
        self.errno_code = errno_code
        self.reads = []

    def _execute(self, command, parameter=b""):
        return False

    def _read_natural16(self, name):
        self.reads.append(name)
        return self.errno_code


def test_control_and_store_verbs_map_to_codes():
    c = _Fake()
    c.vm_run()
    c.vm_stop()
    c.vm_reset()
    c.save_slot(3)
    c.delete_slot(2)
    c.clear_store()
    c.cycle()
    assert c.cmds == [
        (RUN, b""),
        (STOP, b""),
        (RESET, b""),
        (SAVE, bytes([3])),       # slot rides in parameter[0]
        (DELETE_SLOT, bytes([2])),
        (CLEAR_STORE, b""),
        (CYCLE, b""),
    ]


def test_refused_save_raises_device_refused_with_code():
    c = _Refusing(17)  # EEXIST: identical image already stored
    with pytest.raises(DeviceRefused, match="already stored") as ei:
        c.save_slot(1)
    assert ei.value.code == 17
    assert c.reads == ["aliensense.nxs.cmd_error"]


def test_refused_save_unknown_errno_falls_back_to_code():
    with pytest.raises(DeviceRefused, match="device error code 99"):
        _Refusing(99).save_slot(0)


def test_refused_delete_and_clear_read_cmd_error():
    with pytest.raises(DeviceRefused, match="no such slot"):
        _Refusing(2).delete_slot(5)  # ENOENT
    with pytest.raises(DeviceRefused, match="flash"):
        _Refusing(5).clear_store()  # EIO


def test_refused_upload_names_the_pull_reason():
    # A refused LOAD_FROM_FILE surfaces the pull reason instead of
    # silently waiting for a driver that will never arrive.
    with pytest.raises(DeviceRefused, match="another file transfer"):
        _Refusing(16).upload_image(b"\x00" * 8)  # EBUSY


def test_read_getters_pull_from_driver_info():
    c = _Fake()
    info = SimpleNamespace(name=b"imu20680", sample_size=14, active_slot=2,
                           store_count=3, probe_retries=1, vm_state=1,
                           runner_state=3, error_code=0, status_byte=0x80,
                           sample_count=42)
    c._info = lambda slot=0xFF: info  # stand in for the GetDriverInfo RPC

    assert c.read_driver_name() == "imu20680"
    assert c.read_sample_size() == 14
    assert c.read_active_slot() == 2
    assert c.read_store_count() == 3
    assert c.read_probe_retries() == 1
    assert c.read_vm_state() == 1
    assert c.read_runner_state() == 3
    assert c.read_status() == 0x80
    assert c.read_sample_count() == 42


def test_read_getters_default_when_offline():
    c = _Fake()
    c._info = lambda slot=0xFF: None  # no response
    assert c.read_driver_name() == ""
    assert c.read_sample_size() == 0
    assert c.read_active_slot() == 0xFF
    assert c.probe() is False


def test_read_slot_info_peeks_named_stored_slot():
    c = _Fake()
    info = SimpleNamespace(name=b"iam20680", num_outputs=7, num_params=4,
                           i2c_addr=0x68)
    seen = {}

    def fake_info(slot=0xFF):
        seen["slot"] = slot
        return info

    c._info = fake_info
    si = c.read_slot_info(0)
    assert seen["slot"] == 0          # GetDriverInfo queried the requested slot
    assert si.name == "iam20680"      # decoded to str, not raw bytes
    assert si.num_outputs == 7
    assert si.num_params == 4
    assert si.i2c_addr == 0x68


def test_read_slot_info_none_for_empty_or_offline():
    c = _Fake()
    c._info = lambda slot=0xFF: SimpleNamespace(
        name=b"", num_outputs=0, num_params=0, i2c_addr=0)
    assert c.read_slot_info(5) is None    # empty slot
    c._info = lambda slot=0xFF: None
    assert c.read_slot_info(5) is None    # no response


class _BitrateEcho(_Fake):
    """Serves a fixed natural32[2] echo regardless of what was written."""

    def __init__(self, echo):
        super().__init__()
        self.echo = echo

    def _write_natural32_pair(self, name, values):
        return self.echo


def test_write_can_bitrate_mismatch_is_device_refused():
    c = _BitrateEcho((1000000, 4000000))
    with pytest.raises(DeviceRefused) as e:
        c.write_can_bitrate(1000000, 2000000)
    assert "rejected bitrate" in str(e.value)


class _TermEcho(_Fake):
    """Serves a fixed can_term echo regardless of what was written."""

    def __init__(self, echo):
        super().__init__()
        self.echo = echo

    def _write_natural16(self, name, value):
        pass

    def _read_natural16(self, name):
        return self.echo


def test_write_can_term_applied_passes():
    _TermEcho(1).write_can_term(1)          # echo matches: no raise


def test_write_can_term_mismatch_is_device_refused():
    with pytest.raises(DeviceRefused) as e:
        _TermEcho(0).write_can_term(1)
    assert "rejected can-term" in str(e.value)
