"""The upload's session handshake: claim the bytecode consumer, read the
announce's verdict, and only then stream — nothing may reach PROGRAM_DATA
on a refusal (the streamed-into-the-wrong-sink bug of issue #156)."""

import pytest

from nxs.client import DeviceRefused
from nxs.transports.i2c import (
    CMD_ERR_PENDING, CMD_XFER_ABORT,
    NxsI2cTransport,
    REG_CMD, REG_CMD_ERROR, REG_PROGRAM_DATA, REG_PROGRAM_SIZE,
    REG_XFER_TYPE,
    XFER_TYPE_TIME_SYNC, XFER_TYPE_VM_BYTECODE,
    CMD_LOAD,
)


class _UploadBus:
    """Register-map stub for the announce protocol. `announce_result` is
    the device's verdict (0 / EBUSY 16 / EPROTO 71 / EFBIG 27), resolved
    the moment the size lands; `held_mode` pins the mux like a live
    session would — mode writes are silently ignored. `load_result` is
    CMD_LOAD's parse verdict, optionally served after `load_pending_reads`
    polls of the CMD_ERR_PENDING window."""

    def __init__(self, announce_result=0, held_mode=None, load_result=0,
                 load_pending_reads=0, mode_echo_lag=0, fail_chunk=None):
        self.held_mode = held_mode
        self.xfer_type = held_mode if held_mode is not None else 0
        self.announce_result = announce_result
        self.load_result = load_result
        self.load_pending_reads = load_pending_reads
        self.mode_echo_lag = mode_echo_lag   # reads before the echo lands
        self.fail_chunk = fail_chunk         # chunk index that raises OSError
        self.cmd_error = 0
        self.size = None
        self.chunks = []
        self.cmds = []
        self._pending_left = 0
        self._pending_mode = None

    def write_byte_data(self, addr, reg, val):
        if reg == REG_XFER_TYPE:
            if self.held_mode is None:
                if self.mode_echo_lag:
                    self._pending_mode = val   # deferred: drain lags the bus
                else:
                    self.xfer_type = val
        elif reg == REG_CMD:
            self.cmds.append(val)
            if val == CMD_LOAD:
                self._pending_left = self.load_pending_reads
                self.cmd_error = (CMD_ERR_PENDING if self._pending_left
                                  else self.load_result)

    def write_word_data(self, addr, reg, val):
        assert reg == REG_PROGRAM_SIZE
        self.size = val
        self.cmd_error = self.announce_result

    def write_i2c_block_data(self, addr, reg, data):
        assert reg == REG_PROGRAM_DATA
        if self.fail_chunk is not None and len(self.chunks) == self.fail_chunk:
            raise OSError(121, "Remote I/O error")
        self.chunks.append(bytes(data))

    def read_byte_data(self, addr, reg):
        if reg == REG_XFER_TYPE:
            if self._pending_mode is not None:
                self.mode_echo_lag -= 1
                if self.mode_echo_lag <= 0:
                    self.xfer_type = self._pending_mode
                    self._pending_mode = None
            return self.xfer_type
        if reg == REG_CMD_ERROR:
            if self._pending_left > 0:
                self._pending_left -= 1
                if self._pending_left == 0:
                    self.cmd_error = self.load_result
                return CMD_ERR_PENDING
            return self.cmd_error
        return 0


def _transport(bus) -> NxsI2cTransport:
    t = NxsI2cTransport(0, _bus_obj=bus)
    t.STORE_CMD_TIMEOUT_S = 0.2
    return t


def test_upload_streams_only_after_a_clean_verdict():
    bus = _UploadBus(announce_result=0)
    _transport(bus).upload_image(b"\x4e\x58\x53\x00" + bytes(60))
    assert bus.size == 64
    assert len(bus.chunks) == 2            # 64 B in 32-B windows
    assert bus.cmds == [CMD_LOAD]
    assert bus.xfer_type == XFER_TYPE_VM_BYTECODE


def test_upload_stops_at_a_held_mux():
    # The pusher parked the mode and a session refuses our claim: the
    # readback detects it before the size is ever announced.
    bus = _UploadBus(held_mode=XFER_TYPE_TIME_SYNC)
    with pytest.raises(DeviceRefused, match="another transfer session") as ei:
        _transport(bus).upload_image(bytes(64))
    assert ei.value.code == 16
    assert bus.size is None
    assert bus.chunks == []


@pytest.mark.parametrize("code,needle", [
    (16, "another transfer session"),
    (71, "out-of-sequence transfer op"),
    (27, "exceeds the device staging buffer"),
])
def test_upload_announce_refusals_stream_nothing(code, needle):
    bus = _UploadBus(announce_result=code)
    with pytest.raises(DeviceRefused, match=needle) as ei:
        _transport(bus).upload_image(bytes(64))
    assert ei.value.code == code
    assert bus.chunks == []                # the wrong-sink bug, dead
    assert bus.cmds == []                  # and no LOAD on a failed stage


def test_claim_tolerates_a_lagging_mode_drain():
    # The mode write commits deferred on the device; a readback beating
    # the drain must not read as a refusal — the claim polls the echo.
    bus = _UploadBus(mode_echo_lag=3)
    _transport(bus).upload_image(bytes(64))
    assert bus.cmds == [CMD_LOAD]


def test_interrupted_stream_releases_the_session():
    # The announce opened a session; a bus error mid-stream must not
    # leave it holding the mux for the whole stale window.
    bus = _UploadBus(fail_chunk=1)
    with pytest.raises(OSError):
        _transport(bus).upload_image(bytes(64))
    assert CMD_XFER_ABORT in bus.cmds
    assert CMD_LOAD not in bus.cmds


def test_operator_interrupt_releases_the_session():
    # Ctrl-C mid-stream is a deliberate give-up: KeyboardInterrupt is not an
    # Exception, so only a finally releases the mux — otherwise the next
    # upload eats EBUSY until the 2 s stale window expires.
    class _Interrupting(_UploadBus):
        def write_i2c_block_data(self, addr, reg, data):
            raise KeyboardInterrupt

    bus = _Interrupting()
    with pytest.raises(KeyboardInterrupt):
        _transport(bus).upload_image(bytes(64))
    assert CMD_XFER_ABORT in bus.cmds
    assert CMD_LOAD not in bus.cmds


def test_upload_confirms_the_parse_through_the_pending_window():
    # LOAD is async on the device; the host polls CMD_ERROR out of the
    # CMD_ERR_PENDING window before logging "Uploaded".
    bus = _UploadBus(load_pending_reads=3)
    _transport(bus).upload_image(bytes(64))
    assert bus.cmds == [CMD_LOAD]
    assert bus.cmd_error == 0


@pytest.mark.parametrize("code,needle", [
    (8, "not a valid driver"),             # ENOEXEC — issue #152's stale wheel
    (61, "short of the announced size"),   # ENODATA
    (11, "queue full"),                    # EAGAIN — the ISR dropped the op
])
def test_upload_raises_the_devices_load_verdict(code, needle):
    bus = _UploadBus(load_result=code)
    with pytest.raises(DeviceRefused, match=needle) as ei:
        _transport(bus).upload_image(bytes(64))
    assert ei.value.code == code
    assert len(bus.chunks) == 2            # the stage completed; its verdict failed
