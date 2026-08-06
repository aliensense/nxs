"""I²C `write_can_term` read-back verification — an ACKed write that the
firmware ignored (the CAN_TERM window still reads 0) must raise
`DeviceRefused`, mirroring the Cyphal transport, so callers never proceed
believing the bus end is terminated.
"""
import pytest

from nxs.client import DeviceRefused
from nxs.transports.i2c import REG_CAN_TERM, NxsI2cTransport


class _Bus:
    """Byte-register stub: writes land in a dict, reads serve it."""

    def __init__(self, accept=True):
        self.regs = {}
        self.accept = accept

    def write_byte_data(self, addr, reg, value):
        if self.accept:
            self.regs[reg] = value

    def read_byte_data(self, addr, reg):
        return self.regs.get(reg, 0)


def _transport(bus):
    """The constructor opens a real SMBus device, so tests assemble the
    instance directly — write/read_can_term touch only _bus and _addr."""
    t = NxsI2cTransport.__new__(NxsI2cTransport)
    t._bus = bus
    t._addr = 0x30
    return t


def test_accepted_write_round_trips():
    t = _transport(_Bus())
    t.write_can_term(1)
    assert t.read_can_term() == 1


def test_ignored_write_is_device_refused():
    with pytest.raises(DeviceRefused) as e:
        _transport(_Bus(accept=False)).write_can_term(1)
    assert "rejected can-term" in str(e.value)
