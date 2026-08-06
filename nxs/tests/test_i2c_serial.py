"""NxsI2cTransport identity reads: the 12-byte chip UID96 from the
SERIAL window and the firmware version from the FW_VERSION registers;
transports without them return None."""
from nxs.transports.i2c import (
    NxsI2cTransport, REG_SERIAL, SERIAL_LEN,
    REG_FW_VERSION_MAJOR, REG_FW_VERSION_MINOR)
from nxs.transports.mock import MockTransport


class _FakeSerialBus:
    def __init__(self, uid: bytes = b"", fw=(0, 0)):
        self._regs = bytearray(256)
        self._regs[REG_SERIAL:REG_SERIAL + len(uid)] = uid
        self._regs[REG_FW_VERSION_MAJOR] = fw[0]
        self._regs[REG_FW_VERSION_MINOR] = fw[1]

    def read_i2c_block_data(self, addr, reg, n):
        return list(self._regs[reg:reg + n])

    def read_byte_data(self, addr, reg):
        return self._regs[reg]


def test_read_serial_returns_uid96():
    uid = bytes(range(0xA0, 0xA0 + SERIAL_LEN))
    t = NxsI2cTransport(_bus_obj=_FakeSerialBus(uid))
    assert t.read_serial() == uid
    assert len(t.read_serial()) == SERIAL_LEN


def test_mock_serves_its_synthetic_identity():
    t = MockTransport()
    assert t.read_serial() == MockTransport.SERIAL
    assert len(t.read_serial()) == SERIAL_LEN
    assert t.read_fw_version() == MockTransport.FW_VERSION


def test_read_fw_version_serves_major_minor():
    t = NxsI2cTransport(_bus_obj=_FakeSerialBus(fw=(1, 4)))
    assert t.read_fw_version() == "1.4"


def test_read_fw_version_unseeded_is_none():
    """Major 0 is the sentinel for firmware predating the registers."""
    t = NxsI2cTransport(_bus_obj=_FakeSerialBus(fw=(0, 7)))
    assert t.read_fw_version() is None
