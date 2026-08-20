"""Calibration transfers claim the PROGRAM_DATA mux with the readback
handshake: a live session's silent refusal must fail the whole
read/write, never stream record bytes into another transfer's sink."""

import pytest

from nxs.client import DeviceRefused
from nxs.transports.i2c import (
    NxsI2cTransport,
    REG_PROGRAM_DATA, REG_XFER_TYPE,
    XFER_TYPE_DFU_IMAGE,
)


class _SessionHeldBus:
    """Mux held by a live DFU session: XFER_TYPE writes are silently
    dropped (the device's refusal contract) and every landed
    PROGRAM_DATA chunk is counted — the corruption the claim prevents."""

    def __init__(self):
        self.chunks = 0

    def write_byte_data(self, addr, reg, val):
        if reg == REG_XFER_TYPE:
            return

    def write_word_data(self, addr, reg, val):
        pass

    def write_i2c_block_data(self, addr, reg, data):
        if reg == REG_PROGRAM_DATA:
            self.chunks += 1

    def read_byte_data(self, addr, reg):
        if reg == REG_XFER_TYPE:
            return XFER_TYPE_DFU_IMAGE
        return 0

    def read_i2c_block_data(self, addr, reg, n):
        return [0] * n


def test_write_calibration_refused_streams_nothing_into_a_session():
    from nxs.client import CalibrationRecord
    bus = _SessionHeldBus()
    t = NxsI2cTransport(_bus_obj=bus)
    with pytest.raises(DeviceRefused):
        t.write_calibration(CalibrationRecord())
    assert bus.chunks == 0


def test_read_calibration_refused_rather_than_misread():
    bus = _SessionHeldBus()
    t = NxsI2cTransport(_bus_obj=bus)
    with pytest.raises(DeviceRefused):
        t.read_calibration()


class _EpochedCalibBus:
    """Serves the paged CALIB read-back with the trailing epoch byte and
    echoes register writes (no session). `flips` epochs to consume before
    stabilizing — each flip repaints the record mid-bracket."""

    def __init__(self, flips=0):
        from nxs._generated_constants import Calibration as C
        self.size = C.RECORD_SIZE
        self.record = bytes(range(256))[:self.size] * 1
        self.record = (self.record + bytes(self.size))[:self.size]
        self.epoch = 1
        self.flips = flips
        self.regs = {}
        self.reads = 0

    def _mirror(self):
        from nxs.client import CalibrationRecord
        return CalibrationRecord().pack() + bytes([self.epoch])

    def write_byte_data(self, addr, reg, val):
        self.regs[reg] = val

    def read_byte_data(self, addr, reg):
        return self.regs.get(reg, 0)

    def read_i2c_block_data(self, addr, reg, n):
        self.reads += 1
        # A pending flip lands between bracket reads: bump the epoch the
        # moment the record pages start being consumed a second time.
        if self.flips and self.reads % 3 == 0:
            self.flips -= 1
            self.epoch += 1
        page = self.regs.get(REG_STORE_SELECT_, 0)
        mirror = self._mirror()
        off = page * 32
        return list(mirror[off:off + n].ljust(n, b"\x00"))


from nxs.transports.i2c import REG_STORE_SELECT as REG_STORE_SELECT_


def test_read_calibration_epoch_bracket_accepts_stable():
    t = NxsI2cTransport(_bus_obj=_EpochedCalibBus())
    rec = t.read_calibration()
    assert rec.orientation == 0


def test_read_calibration_epoch_bracket_retries_then_succeeds():
    """One mid-read repaint moves the bracket; the retry must land."""
    bus = _EpochedCalibBus(flips=1)
    t = NxsI2cTransport(_bus_obj=bus)
    rec = t.read_calibration()
    assert rec.orientation == 0
    assert bus.epoch == 2   # the flip really happened


def test_read_calibration_gives_up_when_epoch_never_settles():
    bus = _EpochedCalibBus(flips=10_000)
    t = NxsI2cTransport(_bus_obj=bus)
    with pytest.raises(RuntimeError):
        t.read_calibration()


def test_save_calibration_issues_calib_persist():
    """The calibration save is CALIB_PERSIST, never STORE_PERSIST — the
    identity-config command whose guard refuses a bare save (EPROTO)."""
    from nxs.transports.i2c import CMD_CALIB_PERSIST, CMD_STORE_PERSIST, REG_CMD

    class _CmdRecorder(_EpochedCalibBus):
        def __init__(self):
            super().__init__()
            self.cmds = []

        def write_byte_data(self, addr, reg, val):
            if reg == REG_CMD:
                self.cmds.append(val)
                self.regs[0x1D] = 0  # CMD_ERROR: resolved OK
            super().write_byte_data(addr, reg, val)

    bus = _CmdRecorder()
    t = NxsI2cTransport(_bus_obj=bus)
    t.save_calibration()
    assert CMD_CALIB_PERSIST in bus.cmds
    assert CMD_STORE_PERSIST not in bus.cmds


def test_read_cal_epoch_restores_a_foreign_mode_and_never_writes_zero():
    """The probe runs on a streaming cadence: it must hand back a mode
    another flow owns, and a 0 write-back would clear CMD_ERROR."""
    from nxs.transports.i2c import REG_XFER_TYPE

    bus = _EpochedCalibBus()
    bus.regs[REG_XFER_TYPE] = 3          # another flow parked time-sync
    t = NxsI2cTransport(_bus_obj=bus)
    assert t.read_cal_epoch() == 1
    assert bus.regs[REG_XFER_TYPE] == 3  # restored

    bus = _EpochedCalibBus()             # mode idle at 0
    t = NxsI2cTransport(_bus_obj=bus)
    writes = []
    orig = bus.write_byte_data
    def spy(addr, reg, val):
        writes.append((reg, val))
        orig(addr, reg, val)
    bus.write_byte_data = spy
    assert t.read_cal_epoch() == 1
    assert (REG_XFER_TYPE, 0) not in writes   # 0 is never written back
