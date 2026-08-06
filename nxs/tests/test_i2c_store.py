"""NxsI2cTransport store commands surface the device's CMD_ERROR result.

These lock the no-false-success contract: save / delete / clear poll the
async CMD_ERROR sentinel and raise the device's reason on a non-zero
errno, and the CLI reports the failure instead of printing "Saved"."""
import types

import pytest

from nxs.cli import cmd_save, cmd_store_clear, cmd_store_rm
from nxs.client import DeviceRefused
from nxs.transports.i2c import (
    CMD_CLEAR_STORE, CMD_DELETE_SLOT, CMD_ERR_PENDING, CMD_SAVE,
    NxsI2cTransport,
    REG_CMD, REG_CMD_ERROR, REG_STORE_SELECT,
    REG_SEL_NAME_LEN, REG_SEL_NAME, REG_SEL_DRIVER_NUM_PARAMS,
    REG_SEL_DRIVER_NUM_OUTPUTS, REG_SEL_DRIVER_I2C_ADDR, CMD_PEEK_SLOT,
)


class _FakeStoreBus:
    """Emulates the firmware store surface: a store CMD arms
    CMD_ERR_PENDING, then the deferred dispatch resolves CMD_ERROR to a
    preset result (0 = OK, else +errno). The first CMD_ERROR read returns
    PENDING so the host's poll loop is exercised."""

    def __init__(self, result=0):
        self._result = result
        self._pending = False
        self.store_select = None

    def write_byte_data(self, addr, reg, val):
        if reg == REG_STORE_SELECT:
            self.store_select = val
        elif reg == REG_CMD and (val & 0x7F) in (
                CMD_SAVE, CMD_DELETE_SLOT, CMD_CLEAR_STORE):
            self._pending = True  # armed at enqueue, like the I2C target

    def read_byte_data(self, addr, reg):
        if reg == REG_CMD_ERROR:
            if self._pending:
                self._pending = False
                return CMD_ERR_PENDING  # comm thread hasn't resolved it yet
            return self._result
        return 0


class _StuckBus(_FakeStoreBus):
    """CMD_ERROR never leaves PENDING — the queued command was dropped."""

    def read_byte_data(self, addr, reg):
        if reg == REG_CMD_ERROR:
            return CMD_ERR_PENDING
        return 0


def _transport(bus):
    t = NxsI2cTransport(0, _bus_obj=bus)
    t.STORE_CMD_TIMEOUT_S = 0.2
    return t


def test_save_success_does_not_raise():
    _transport(_FakeStoreBus(result=0)).save_slot(0)


def test_save_no_driver_raises_named_reason():
    with pytest.raises(RuntimeError, match="no driver"):
        _transport(_FakeStoreBus(result=61)).save_slot(0)  # ENODATA


def test_save_duplicate_raises_device_refused_with_code():
    with pytest.raises(DeviceRefused, match="already stored") as ei:
        _transport(_FakeStoreBus(result=17)).save_slot(1)  # EEXIST
    assert ei.value.code == 17


def test_delete_invalid_slot_raises():
    with pytest.raises(RuntimeError, match="invalid slot"):
        _transport(_FakeStoreBus(result=22)).delete_slot(9)  # EINVAL


def test_clear_success_does_not_raise():
    _transport(_FakeStoreBus(result=0)).clear_store()


def test_unknown_errno_falls_back_to_code():
    with pytest.raises(RuntimeError, match="device error code 99"):
        _transport(_FakeStoreBus(result=99)).save_slot(0)


def test_store_select_is_written_before_save():
    bus = _FakeStoreBus(result=0)
    _transport(bus).save_slot(3)
    assert bus.store_select == 3


def test_save_timeout_raises():
    with pytest.raises(RuntimeError, match="timeout"):
        _transport(_StuckBus()).save_slot(0)


# ── CLI glue: real success only, never a false "Saved" ──────

class _StubTransport:
    """Minimal transport for the CLI store verbs."""

    def __init__(self, fail=False):
        self._fail = fail
        self._count = 0

    def save_slot(self, slot):
        if self._fail:
            raise RuntimeError("no driver image loaded — upload a driver first")
        self._count = slot + 1

    def delete_slot(self, slot):
        if self._fail:
            raise RuntimeError("invalid slot")
        self._count = 0

    def clear_store(self):
        if self._fail:
            raise RuntimeError("flash (NVS) write error")

    def read_store_count(self):
        return self._count


def test_cmd_save_reports_failure(capsys):
    rc = cmd_save(_StubTransport(fail=True), types.SimpleNamespace(slot=0))
    assert rc == 1
    assert "failed" in capsys.readouterr().err.lower()


def test_cmd_save_reports_success(capsys):
    rc = cmd_save(_StubTransport(fail=False), types.SimpleNamespace(slot=0))
    assert rc == 0
    assert "Saved to slot 0" in capsys.readouterr().out


def test_cmd_save_duplicate_is_a_no_op_success(capsys):
    """EEXIST means the goal state already holds — not a failure."""

    class _Dup(_StubTransport):
        def save_slot(self, slot):
            raise DeviceRefused(
                17, "an identical driver image is already stored in another slot")

    rc = cmd_save(_Dup(), types.SimpleNamespace(slot=0))
    captured = capsys.readouterr()
    assert rc == 0
    assert "Already stored" in captured.out
    assert captured.err == ""


def test_cmd_save_refusal_names_the_reason(capsys):
    class _Full(_StubTransport):
        def save_slot(self, slot):
            raise DeviceRefused(28, "the driver store is full")

    rc = cmd_save(_Full(), types.SimpleNamespace(slot=0))
    assert rc == 1
    err = capsys.readouterr().err
    assert "Save refused" in err and "store is full" in err


def test_cmd_store_rm_reports_failure(capsys):
    rc = cmd_store_rm(_StubTransport(fail=True), types.SimpleNamespace(slot=9))
    assert rc == 1
    assert "failed" in capsys.readouterr().err.lower()


def test_cmd_store_clear_reports_failure(capsys):
    rc = cmd_store_clear(_StubTransport(fail=True), types.SimpleNamespace())
    assert rc == 1
    assert "failed" in capsys.readouterr().err.lower()


def test_cmd_store_rm_and_clear_render_refusals(capsys):
    """A device refusal reads 'refused' + the reason, not a bare 'failed'."""

    class _Refusing(_StubTransport):
        def delete_slot(self, slot):
            raise DeviceRefused(2, "no such slot")

        def clear_store(self):
            raise DeviceRefused(5, "flash (NVS) write error")

    assert cmd_store_rm(_Refusing(), types.SimpleNamespace(slot=5)) == 1
    assert "Remove refused: no such slot" in capsys.readouterr().err
    assert cmd_store_clear(_Refusing(), types.SimpleNamespace()) == 1
    assert "Clear refused: flash" in capsys.readouterr().err


class _FakePeekBus(_FakeStoreBus):
    """Store fake plus the PEEK_SLOT surface: the command paints the SEL
    peek view (name, counts, latched address) before CMD_ERROR resolves."""

    def __init__(self, result=0, name=b"iam20680", counts=(4, 7), i2c_addr=0):
        super().__init__(result)
        self._name = name
        self._counts = counts
        self._i2c_addr = i2c_addr

    def write_byte_data(self, addr, reg, val):
        if reg == REG_CMD and (val & 0x7F) == CMD_PEEK_SLOT:
            self._pending = True
            return
        super().write_byte_data(addr, reg, val)

    def read_byte_data(self, addr, reg):
        if reg == REG_SEL_NAME_LEN:
            return len(self._name)
        if reg == REG_SEL_DRIVER_NUM_PARAMS:
            return self._counts[0]
        if reg == REG_SEL_DRIVER_NUM_OUTPUTS:
            return self._counts[1]
        if reg == REG_SEL_DRIVER_I2C_ADDR:
            return self._i2c_addr
        return super().read_byte_data(addr, reg)

    def read_i2c_block_data(self, addr, reg, n):
        assert reg == REG_SEL_NAME
        return list(self._name[:n])


def test_read_slot_info_serves_the_peeked_header():
    bus = _FakePeekBus(name=b"ms5611", counts=(2, 3), i2c_addr=0x77)
    info = _transport(bus).read_slot_info(1)
    assert bus.store_select == 1
    assert info.name == "ms5611"
    assert info.num_params == 2
    assert info.num_outputs == 3
    assert info.i2c_addr == 0x77


def test_read_slot_info_empty_slot_is_none():
    import errno

    info = _transport(_FakePeekBus(result=errno.ENOENT)).read_slot_info(5)
    assert info is None
