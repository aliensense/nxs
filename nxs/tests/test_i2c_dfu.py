"""NxsI2cTransport.push_image against a fake firmware DFU state machine.

The contract these lock: the host paces on the single-byte XFER_ACK
accepted-chunk counter and never runs ahead of it — a queue-dropped
chunk is resent at the stall with no gap or duplication, a rejected
write surfaces CMD_ERROR and restores the VM-bytecode consumer, and
the counter's mod-256 wrap stays unambiguous with one chunk in
flight."""
import pytest

from nxs.client import DeviceRefused
from nxs.transports.i2c import (CMD_XFER_ABORT, 
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

    def __init__(self, drop_offsets=(), reject_offsets=(), busy=False,
                 begin_never_resolves=False, finish_fails=0,
                 finish_drops=0):
        self.xfer_type = XFER_TYPE_VM_BYTECODE
        self.phase = DFU_PHASE_IDLE
        self.ack = 0
        self.committed = 0
        self.error_code = 0
        self.written = bytearray()
        self.finished = False
        self.aborted = False
        self.busy = busy                # a foreign session holds the mux
        self.begin_never_resolves = begin_never_resolves
        self._drop = set(drop_offsets)
        self._reject = set(reject_offsets)
        self.finish_fails = finish_fails    # errno latched by a failed FINISH
        self.finish_drops = finish_drops    # FINISH writes lost at a full queue

    def write_byte_data(self, addr, reg, val):
        if reg == REG_XFER_TYPE:
            if self.busy:
                return                  # silent refusal: the mode holds
            self.xfer_type = val
            if val == XFER_TYPE_VM_BYTECODE:
                self.committed = 0
                self.ack = 0
                self.phase = DFU_PHASE_IDLE
        elif reg == REG_CMD:
            op = val & 0x7F
            if op == CMD_DFU_BEGIN:
                if self.begin_never_resolves:
                    self.error_code = 0xFF   # CMD_ERR_PENDING, forever
                    return
                if self.busy:
                    self.error_code = 16     # EBUSY from the live session
                    return
                self.committed = 0
                self.ack = 0
                self.error_code = 0
                self.phase = DFU_PHASE_READY
            elif op == CMD_DFU_FINISH:
                if self.finish_drops > 0:
                    # Queue full: the write is ACKed but dropped before
                    # dispatch — phase stays WRITING, nothing resolves.
                    self.finish_drops -= 1
                    return
                if self.finish_fails:
                    # The swap arm failed: the errno latches, the phase
                    # goes terminal ERROR, no reboot happens.
                    self.error_code = self.finish_fails
                    self.phase = DFU_PHASE_ERROR
                    return
                self.phase = DFU_PHASE_FINISHING
                self.finished = True
            elif op == CMD_XFER_ABORT:
                self.aborted = True
                self.xfer_type = XFER_TYPE_VM_BYTECODE
                self.committed = 0
                self.ack = 0
                self.phase = DFU_PHASE_IDLE
                self.error_code = 0

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
        self.phase = DFU_PHASE_WRITING

    def read_byte_data(self, addr, reg):
        if reg == REG_XFER_PHASE:
            return self.phase
        if reg == REG_XFER_ACK:
            return self.ack
        if reg == REG_CMD_ERROR:
            return self.error_code
        if reg == REG_XFER_TYPE:
            return self.xfer_type
        return 0


def _transport(bus) -> NxsI2cTransport:
    t = NxsI2cTransport(0, _bus_obj=bus)
    t.DFU_BEGIN_TIMEOUT_S = 0.5
    t.DFU_ACK_TIMEOUT_S = 0.05
    t.DFU_FINISH_POLL_S = 0.2
    t.DFU_FINISH_RETRIES = 2
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


def test_failed_finish_raises_instead_of_claiming_reboot(tmp_path):
    # A finish the device refuses latches ERROR + the errno and does NOT
    # reboot; "board resetting" on the host would be a lie. EROFS = 30.
    bus = _FakeI2cBus(finish_fails=30)
    path = _image(tmp_path, 64)
    with pytest.raises(DeviceRefused, match="dfu finish failed") as ei:
        _transport(bus).push_image(path)
    assert ei.value.code == 30
    assert bus.finished is False
    assert bus.aborted is True             # session released so the VM resumes

def test_lost_finish_is_retried_then_succeeds(tmp_path):
    # The first finish is dropped at the queue (phase stays WRITING, bus
    # alive); the host must resend rather than time out into false success.
    bus = _FakeI2cBus(finish_drops=1)
    path = _image(tmp_path, 64)
    assert _transport(bus).push_image(path) == 64
    assert bus.finished is True


def test_finish_lost_forever_raises_not_success(tmp_path):
    # A finish that never lands (bus alive, phase never terminal) must raise
    # — the old code logged "board resetting" and returned success.
    bus = _FakeI2cBus(finish_drops=99)
    path = _image(tmp_path, 64)
    with pytest.raises(RuntimeError, match="no reboot after retries"):
        _transport(bus).push_image(path)
    assert bus.finished is False
    assert bus.aborted is True             # the session is released on the way out


def test_operator_interrupt_releases_the_push(tmp_path):
    # The long-running case an operator actually can interrupt by hand.
    class _Interrupting(_FakeI2cBus):
        def write_i2c_block_data(self, addr, reg, data):
            raise KeyboardInterrupt

    bus = _Interrupting()
    path = _image(tmp_path, 96)
    with pytest.raises(KeyboardInterrupt):
        _transport(bus).push_image(path)
    assert bus.aborted is True
    assert bus.finished is False


def test_dropped_chunk_is_resent_at_the_stall(tmp_path):
    bus = _FakeI2cBus(drop_offsets={32})
    path = _image(tmp_path, 96)
    sent = _transport(bus).push_image(path)
    assert sent == 96
    assert bytes(bus.written) == bytes(i & 0xFF for i in range(96))
    assert bus.finished is True


def test_rejected_write_raises_and_aborts_the_session(tmp_path):
    bus = _FakeI2cBus(reject_offsets={0})
    path = _image(tmp_path, 64)
    with pytest.raises(DeviceRefused, match="dfu write rejected") as ei:
        _transport(bus).push_image(path)
    assert ei.value.code == _FakeI2cBus.EINVAL
    assert bus.finished is False
    # The unwind says goodbye properly: XFER_ABORT (a raw mode write would
    # be silently refused while the session lives) releases the mux.
    assert bus.aborted is True
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


def test_begin_refused_while_anothers_push_is_live(tmp_path):
    # The two-pusher race: the holder is mid-DFU, so the mode already reads
    # DFU_IMAGE and our readback passes — the begin's own CMD_ERROR edge is
    # what says no. No erase fires, the holder's progress survives.
    bus = _FakeI2cBus(busy=True)
    bus.xfer_type = XFER_TYPE_DFU_IMAGE
    path = _image(tmp_path, 64)
    with pytest.raises(DeviceRefused, match="another transfer session") as ei:
        _transport(bus).push_image(path)
    assert ei.value.code == 16
    assert bus.written == bytearray()      # never streamed a byte
    assert bus.aborted is False            # the holder's live session survives


def test_held_mux_stops_the_push_before_begin(tmp_path):
    # A live bytecode session refuses our mode write silently; the readback
    # detects it and the push stops before DFU_BEGIN ever lands.
    bus = _FakeI2cBus(busy=True)           # xfer_type stays VM_BYTECODE
    path = _image(tmp_path, 64)
    with pytest.raises(DeviceRefused, match="another transfer session"):
        _transport(bus).push_image(path)
    assert bus.phase == DFU_PHASE_IDLE     # no begin, no erase


def test_begin_timeout_names_the_transfer_state(tmp_path):
    # Ask (c) of issue #156: the timeout names XFER_PHASE / XFER_TYPE /
    # CMD_ERROR — the line that identifies a mux hijacker in one minute.
    bus = _FakeI2cBus(begin_never_resolves=True)
    path = _image(tmp_path, 64)
    t = _transport(bus)
    t.DFU_BEGIN_TIMEOUT_S = 0.2
    with pytest.raises(TimeoutError, match="XFER_PHASE=IDLE") as ei:
        t.push_image(path)
    assert "XFER_TYPE=DFU_IMAGE" in str(ei.value)
    assert "CMD_ERROR=255" in str(ei.value)
