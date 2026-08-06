"""`estimate_and_push` over I²C — the seed-at-converge path. The
transport's device-clock read rides the sample-window latch stamp, so a
`suite switch` can seed a unit before it has polled a single sample."""
import struct

import pytest

from nxs.client import TimeSyncEstimator, estimate_and_push
from nxs.transports.i2c import (REG_PROGRAM_DATA, REG_SAMPLE_DATA,
                                REG_XFER_TYPE, NxsI2cTransport)


class _Bus:
    """Sample-window stub: serves an advancing device clock from the
    latch slot and captures the pushed time-sync record."""

    def __init__(self):
        self.device_us = 5_000_000
        self.writes = {}
        self.record = b""
        self.mirror_paints = True

    def read_i2c_block_data(self, addr, reg, n):
        if reg == REG_PROGRAM_DATA:
            if not self.mirror_paints or not self.record:
                return [0xFF] * n
            return list((self.record + bytes([1, 1]))[:n])
        assert reg == REG_SAMPLE_DATA
        self.device_us += 250
        return list(struct.pack('<Q', self.device_us))[:n]

    def write_byte_data(self, addr, reg, value):
        self.writes[reg] = value

    def write_i2c_block_data(self, addr, reg, data):
        if reg == REG_PROGRAM_DATA:
            self.record = bytes(data)


def _transport(bus):
    """Assemble without the SMBus-opening constructor, per
    test_i2c_can_term."""
    t = NxsI2cTransport.__new__(NxsI2cTransport)
    t._bus = bus
    t._addr = 0x30
    t._time_sync = TimeSyncEstimator()
    return t


def test_ping_reads_the_latch_stamp():
    t = _transport(_Bus())
    assert t.time_sync_ping() is True


def test_estimate_and_push_seeds_over_i2c():
    bus = _Bus()
    t = _transport(bus)
    bound = estimate_and_push(t)
    assert bound is not None
    offset_us, pushed_bound, rate_ppb, valid_for_us = struct.unpack(
            '<qIiI', bus.record)
    assert valid_for_us == 10_000_000
    assert pushed_bound == bound
    assert REG_XFER_TYPE in bus.writes


def test_estimate_and_push_raises_when_the_mirror_stays_unpainted():
    """A push the device never applies must fail loudly, not return a
    clean bound."""
    bus = _Bus()
    bus.mirror_paints = False
    t = _transport(bus)
    with pytest.raises(RuntimeError):
        estimate_and_push(t)


class _MirrorBus:
    """Mirror-window stub: all-ones (unpainted) frames first, then the
    painted record — the repaint race read_time_sync must poll past."""

    def __init__(self, unpainted_reads):
        self.unpainted = unpainted_reads
        self.record = struct.pack('<qIiIBB', -123456, 220, -400000,
                                  50_000_000, 1, 1)

    def write_byte_data(self, addr, reg, value):
        pass

    def read_i2c_block_data(self, addr, reg, n):
        assert reg == REG_PROGRAM_DATA
        if self.unpainted > 0:
            self.unpainted -= 1
            return [0xFF] * n
        return list(self.record[:n])


def test_read_time_sync_polls_past_an_unpainted_mirror():
    t = _transport(_MirrorBus(unpainted_reads=2))
    assert t.read_time_sync() == (-123456, 220, -400000, 50_000_000, 1, True)


def test_read_time_sync_reports_invalid_when_never_painted():
    t = _transport(_MirrorBus(unpainted_reads=10 ** 9))
    assert t.read_time_sync() == (0, 0, 0, 0, 0, False)
