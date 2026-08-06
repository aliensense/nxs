"""NxsI2cTransport.push_image against a fake firmware DFU state machine.

The contract these lock: the host paces on the single-byte XFER_ACK
accepted-chunk counter and never runs ahead of it — a queue-dropped
chunk is resent at the stall with no gap or duplication, a rejected
write surfaces CMD_ERROR and restores the VM-bytecode consumer, and
the counter's mod-256 wrap stays unambiguous with one chunk in
flight."""
import pytest

from nxs.client import DeviceRefused
from nxs.transports.i2c import (
    NxsI2cTransport,
    CMD_DFU_BEGIN, CMD_DFU_FINISH,
    DFU_PHASE_ERROR, DFU_PHASE_FINISHING, DFU_PHASE_IDLE, DFU_PHASE_READY,
    DFU_PHASE_WRITING,
    REG_CMD, REG_PROGRAM_DATA, REG_XFER_ACK, REG_CMD_ERROR, REG_XFER_PHASE,
    REG_XFER_TYPE,
    XFER_TYPE_DFU_IMAGE, XFER_TYPE_VM_BYTECODE,
)


class _FakeI2cBus:
    """Emulates the firmware's I2C DFU surface: XFER_TYPE routing, the
    BEGIN/FINISH commands, the firmware-owned cursor, and the
    single-byte XFER_PHASE / XFER_ACK / CMD_ERROR readback.

    Failure injection: `drop_offsets` silently loses the chunk written
    at that committed byte offset once (the ISR-to-thread queue
    overflow); `reject_offsets` fails it like the DFU core would
    (cursor and counter unmoved, ERROR phase, CMD_ERROR set)."""

    EINVAL = 22

    def __init__(self, drop_offsets=(), reject_offsets=()):
        self.xfer_type = XFER_TYPE_VM_BYTECODE
        self.phase = DFU_PHASE_IDLE
        self.ack = 0
        self.committed = 0
        self.error_code = 0
        self.written = bytearray()
        self.finished = False
        self._drop = set(drop_offsets)
        self._reject = set(reject_offsets)

    def write_byte_data(self, addr, reg, val):
        if reg == REG_XFER_TYPE:
            self.xfer_type = val
            if val == XFER_TYPE_VM_BYTECODE:
                self.committed = 0
                self.ack = 0
                self.phase = DFU_PHASE_IDLE
        elif reg == REG_CMD:
            op = val & 0x7F
            if op == CMD_DFU_BEGIN:
                self.committed = 0
                self.ack = 0
                self.error_code = 0
                self.phase = DFU_PHASE_READY
            elif op == CMD_DFU_FINISH:
                self.phase = DFU_PHASE_FINISHING
                self.finished = True

    def write_i2c_block_data(self, addr, reg, data):
        if reg != REG_PROGRAM_DATA or self.xfer_type != XFER_TYPE_DFU_IMAGE:
            return
        off = self.committed
        if off in self._drop:
            self._drop.discard(off)
            return  # queue overflow: chunk lost, cursor and ack unmoved
        if off in self._reject:
            self.error_code = self.EINVAL
            self.phase = DFU_PHASE_ERROR
            return
        self.written.extend(bytes(data))
        self.committed += len(data)
        self.ack = (self.ack + 1) & 0xFF
        self.error_code = 0
        self.phase = DFU_PHASE_WRITING

    def read_byte_data(self, addr, reg):
        if reg == REG_XFER_PHASE:
            return self.phase
        if reg == REG_XFER_ACK:
            return self.ack
        if reg == REG_CMD_ERROR:
            return self.error_code
        return 0


def _transport(bus) -> NxsI2cTransport:
    t = NxsI2cTransport(0, _bus_obj=bus)
    t.DFU_BEGIN_TIMEOUT_S = 0.5
    t.DFU_ACK_TIMEOUT_S = 0.05
    return t


def _image(tmp_path, size: int) -> str:
    p = tmp_path / "fw.bin"
    p.write_bytes(bytes(i & 0xFF for i in range(size)))
    return str(p)


def test_full_push_lands_every_byte(tmp_path):
    bus = _FakeI2cBus()
    path = _image(tmp_path, 70)  # non-multiple of 32: 32 + 32 + 6
    sent = _transport(bus).push_image(path)
    assert sent == 70
    assert bytes(bus.written) == bytes(i & 0xFF for i in range(70))
    assert bus.ack == 3
    assert bus.finished is True
    # Success leaves DFU mode selected — the board is rebooting and a
    # restore write would NACK; only failures restore the VM path.
    assert bus.xfer_type == XFER_TYPE_DFU_IMAGE


def test_dropped_chunk_is_resent_at_the_stall(tmp_path):
    bus = _FakeI2cBus(drop_offsets={32})
    path = _image(tmp_path, 96)
    sent = _transport(bus).push_image(path)
    assert sent == 96
    assert bytes(bus.written) == bytes(i & 0xFF for i in range(96))
    assert bus.finished is True


def test_rejected_write_raises_and_restores_vm_path(tmp_path):
    bus = _FakeI2cBus(reject_offsets={0})
    path = _image(tmp_path, 64)
    with pytest.raises(DeviceRefused, match="dfu write rejected") as ei:
        _transport(bus).push_image(path)
    assert ei.value.code == _FakeI2cBus.EINVAL
    assert bus.finished is False
    assert bus.xfer_type == XFER_TYPE_VM_BYTECODE


def test_ack_counter_wraps_mod_256(tmp_path):
    # 300 one-byte chunks cross the counter's 255→0 wrap; with a single
    # chunk in flight the host's expected value stays unambiguous and
    # every byte lands exactly once.
    bus = _FakeI2cBus()
    path = _image(tmp_path, 300)
    sent = _transport(bus).push_image(path, chunk_size=1)
    assert sent == 300
    assert bytes(bus.written) == bytes(i & 0xFF for i in range(300))
    assert bus.ack == 300 & 0xFF
    assert bus.finished is True


def test_oversize_chunk_is_rejected(tmp_path):
    path = _image(tmp_path, 64)
    with pytest.raises(ValueError):
        _transport(_FakeI2cBus()).push_image(path, chunk_size=64)


def test_empty_image_is_rejected(tmp_path):
    path = _image(tmp_path, 0)
    with pytest.raises(RuntimeError, match="empty"):
        _transport(_FakeI2cBus()).push_image(path)
