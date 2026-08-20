"""Offline coverage for CyphalControlClient — the verb→wire mapping, with the
pycyphal layer stubbed (autostart=False), so no bus or compiled DSDL is needed.
"""

from types import SimpleNamespace

import pytest

from nxs.client import DFU_ERASE_TIMEOUT_S, DeviceRefused, STORE_CMD_TIMEOUT_S
from nxs.transports.cyphal_control import (
    CLEAR_STORE,
    COMMAND_BEGIN_SOFTWARE_UPDATE,
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

    def _execute(self, command, parameter=b"", timeout=None):
        self.cmds.append((command, bytes(parameter), timeout))
        return True


class _Refusing(_Fake):
    """Every command replies FAILURE; cmd_error serves a preset errno."""

    def __init__(self, errno_code):
        super().__init__()
        self.errno_code = errno_code
        self.reads = []

    def _execute(self, command, parameter=b"", timeout=None):
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
    # Third element is the per-call timeout: the commands that erase and
    # program flash outwait the frozen MCU, the rest take the default.
    assert c.cmds == [
        (RUN, b"", None),
        (STOP, b"", None),
        (RESET, b"", None),
        (SAVE, bytes([3]), STORE_CMD_TIMEOUT_S),   # slot rides in parameter[0]
        (DELETE_SLOT, bytes([2]), STORE_CMD_TIMEOUT_S),
        (CLEAR_STORE, b"", STORE_CMD_TIMEOUT_S),
        (CYCLE, b"", None),
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


class _LoadVerdict(_Fake):
    """The pull runs clean; `cmd_error` then serves the LOAD verdict the
    frontend mirrored out of `on_load`."""

    def __init__(self, errno_code):
        super().__init__()
        self.errno_code = errno_code

    def _read_natural16(self, name):
        return self.errno_code


class _StuckPull(_Fake):
    """cmd_error never leaves the PENDING sentinel — the pull wedged."""

    def _read_natural16(self, name):
        from nxs._generated_constants import NxsRegisters
        return NxsRegisters.CMD_ERR_PENDING


def test_upload_verdict_times_out_when_pull_wedges():
    # A pull whose verdict never latches must raise, not read the pending
    # sentinel as success — the silent-"Uploaded" class this replaced.
    with pytest.raises(TimeoutError, match="did not resolve"):
        _StuckPull()._await_pull_verdict(timeout=0.3)


def test_upload_reads_a_clean_load_verdict():
    _LoadVerdict(0).upload_image(b"\x00" * 8)   # verdict 0: no raise


class _CounterServe(_Fake):
    """Serves a fixed natural16 per register name."""

    def __init__(self, values):
        super().__init__()
        self.values = values

    def _read_natural16(self, name):
        return self.values[name]


def test_fault_counter_getters_read_their_registers():
    c = _CounterServe({
        "aliensense.nxs.drdy_coalesced_count": 7,
        "aliensense.nxs.ingress_reject_count": 2,
    })
    assert c.read_drdy_coalesced_count() == 7
    assert c.read_ingress_reject_count() == 2


def test_upload_raises_the_load_verdict():
    # The pull delivering bytes is not the driver existing: the parse
    # verdict arrives via the cmd_error mirror after the pull settles.
    with pytest.raises(DeviceRefused, match="not a valid driver") as ei:
        _LoadVerdict(8).upload_image(b"\x00" * 8)  # ENOEXEC
    assert ei.value.code == 8


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


def test_firmware_push_outwaits_the_slot_erase(tmp_path):
    """The device bulk-erases the staging slot inside the BEGIN call and is
    frozen until it finishes, so the ack lands after the default service
    timeout — which reported `device did not respond` for a working device.
    """
    img = tmp_path / "zephyr.signed.bin"
    img.write_bytes(b"\x00" * 64)

    class _Push(_Fake):
        def _serve(self, name, data):
            pass

        def _wait_for_update(self, *args, **kwargs):
            return True

    c = _Push()
    c._fileserver = SimpleNamespace(served=0)
    c.push_image(str(img))

    assert c.cmds == [(COMMAND_BEGIN_SOFTWARE_UPDATE, b"zephyr.signed.bin",
                       DFU_ERASE_TIMEOUT_S)]


def test_call_widens_pycyphals_own_response_timeout():
    """pycyphal bounds every call by the client's own response_timeout and
    returns None when it expires, so widening only the future timeout would
    change nothing. The client's bound moves for the call and back after.
    """
    import asyncio
    import threading

    granted = []

    class _Client:
        response_timeout = 1.0

        async def call(self, request):
            granted.append(self.response_timeout)
            return (SimpleNamespace(status=0), None)

    c = CyphalControlClient(port="(fake)", autostart=False)
    loop = asyncio.new_event_loop()
    threading.Thread(target=loop.run_forever, daemon=True).start()
    c._loop = loop
    client = _Client()
    try:
        assert c._call(client, None, timeout=5.0) is not None
    finally:
        loop.call_soon_threadsafe(loop.stop)

    assert granted == [5.0]                     # the call ran with the wide bound
    assert client.response_timeout == 1.0       # and it was handed back


# ── Calibration register-echo verification ────────────────────────
# These fakes answer real Access values, so they need the compiled DSDL
# namespace; environments without it skip (pycyphal is an extra).
try:
    from nxs.transports.cyphal_source import _ensure_dsdl as _dsdl
    _dsdl()
    import uavcan.register.Value_1_0  # noqa: F401 — probe only
    _HAS_DSDL = True
except Exception:
    _HAS_DSDL = False

needs_dsdl = pytest.mark.skipif(not _HAS_DSDL,
                                reason="compiled DSDL unavailable")


class _RejectingRegisterFake(_Fake):
    """Register writes answer the device's rejection contract: the echo is
    the unchanged (old) value, never the written one."""

    OLD_ORIENTATION = 0

    def __init__(self):
        super().__init__()
        self.saves = 0

    def _execute(self, command, parameter=b"", timeout=None):
        self.saves += 1
        return True

    def _access(self, name, value=None):
        import uavcan.primitive.array.Natural8_1_0 as Natural8
        import uavcan.primitive.array.Natural32_1_0 as Natural32
        import uavcan.primitive.array.Real32_1_0 as Real32
        import uavcan.register.Value_1_0 as Value
        if value is None or value.natural8 is not None:
            return Value(natural8=Natural8([self.OLD_ORIENTATION]))
        if value.natural32 is not None:
            return Value(natural32=Natural32([0, 0, 0, 0]))
        return Value(real32=Real32([0.0] * 12))


@needs_dsdl
def test_rejected_orientation_echo_raises_and_never_saves():
    """The device rejects by echoing the unchanged value; a fired-and-
    forgotten write would persist a partial record and report success."""
    t = _RejectingRegisterFake()
    with pytest.raises(DeviceRefused):
        t.set_orientation(5)
    assert t.saves == 0


@needs_dsdl
def test_rejected_coefficient_echo_raises_before_save():
    from nxs.client import CalibrationRecord
    t = _RejectingRegisterFake()
    rec = CalibrationRecord(orientation=5)
    rec.m = ((2.0,) + (0.0,) * 8,) * 3   # differs from the echoed zeros
    with pytest.raises(DeviceRefused):
        t.write_calibration(rec)
    assert t.saves == 0


@needs_dsdl
def test_float32_rounding_is_not_a_rejection():
    """A python float that is not float32-representable must compare equal
    after rounding — representability is not a device rejection."""
    import struct as _s

    class _EchoF32(_Fake):
        def _access(self, name, value=None):
            import uavcan.primitive.array.Real32_1_0 as Real32
            import uavcan.register.Value_1_0 as Value
            if value is not None and value.real32 is not None:
                rounded = [_s.unpack('<f', _s.pack('<f', v))[0]
                           for v in value.real32.value]
                return Value(real32=Real32(rounded))
            return value if value is not None else Value()

    t = _EchoF32()
    t._write_reals("aliensense.nxs.calibration.encoder_zero", [0.1])


class _Decimating(CyphalControlClient):
    """Serves the device-wide decimation register from memory."""

    def __init__(self, live: int):
        super().__init__(port="(fake)", autostart=False)
        self.live = live
        self.writes = []

    def read_decimation(self, subject=None) -> int:
        return self.live

    def write_decimation(self, value: int, subject=None) -> None:
        self.writes.append(value)
        self.live = value


def test_streaming_restores_the_device_output_gate():
    """DEC_RATE gates the device's output for *every* transport, not just
    this link. Disarming to 0 left the board mute afterwards: an `nxs stream`
    over CAN followed by one over I2C returned zero samples, and a Save taken
    later persisted the zero across a power-cycle."""
    t = _Decimating(live=4)
    t.start_stream(1)
    assert t.live == 1, "arming should throttle to the requested rate"
    t.stop_stream()
    assert t.live == 4, "teardown must put the operator's rate back"
    assert 0 not in t.writes, "0 mutes the device for every transport"


def test_streaming_restores_a_deliberate_zero():
    """A device configured with output off stays off — restoring means
    restoring, not forcing a rate the operator did not ask for."""
    t = _Decimating(live=0)
    t.start_stream(2)
    assert t.live == 2
    t.stop_stream()
    assert t.live == 0


@needs_dsdl
def test_cyphal_read_calibration_epoch_bracket():
    """The record spans seven registers; the epoch register brackets the
    read. A mid-read bump forces a retry; a never-settling epoch raises."""
    import uavcan.primitive.array.Natural16_1_0 as Natural16
    import uavcan.primitive.array.Natural32_1_0 as Natural32
    import uavcan.primitive.array.Natural8_1_0 as Natural8
    import uavcan.primitive.array.Real32_1_0 as Real32
    import uavcan.register.Value_1_0 as Value

    class _EpochedRegs(_Fake):
        def __init__(self, flips=0):
            super().__init__()
            self.epoch = 3
            self.flips = flips
            self.epoch_reads = 0

        def _access(self, name, value=None):
            if name.endswith(".epoch"):
                self.epoch_reads += 1
                # A pending flip lands between the two bracket reads.
                if self.flips and self.epoch_reads % 2 == 0:
                    self.flips -= 1
                    self.epoch += 1
                return Value(natural16=Natural16([self.epoch]))
            if name.endswith(".orientation"):
                return Value(natural8=Natural8([0]))
            if name.endswith(".driver_tags"):
                return Value(natural32=Natural32([0, 0, 0, 0]))
            return Value(real32=Real32([1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0, 0, 0, 0]))

    rec = _EpochedRegs().read_calibration()
    assert rec.orientation == 0

    bus = _EpochedRegs(flips=1)
    rec = bus.read_calibration()
    assert rec.orientation == 0
    assert bus.epoch == 4       # the retry really happened

    with pytest.raises(RuntimeError):
        _EpochedRegs(flips=10_000).read_calibration()

    class _DirtyRegs(_EpochedRegs):
        """Another host's stage stays mid-edit for `holds` probes."""

        def __init__(self, holds):
            super().__init__()
            self.holds = holds

        def _access(self, name, value=None):
            if name.endswith(".dirty"):
                if self.holds > 0:
                    self.holds -= 1
                    return Value(natural8=Natural8([1]))
                return Value(natural8=Natural8([0]))
            return super()._access(name, value)

    # A stage that clears mid-budget: the read lands on the applied record.
    rec = _DirtyRegs(holds=2).read_calibration()
    assert rec.orientation == 0

    # A stage held dirty past the budget is refused, never misreported.
    import nxs.transports.cyphal_control as cc
    with pytest.raises(RuntimeError):
        _DirtyRegs(holds=10_000).read_calibration()

    class _MidReadStager(_EpochedRegs):
        """Clean before the fields, dirty after — a host began staging
        mid-read. The bank epoch never moves, so only the second dirty
        read can catch it."""

        def __init__(self):
            super().__init__()
            self.dirty_reads = 0

        def _access(self, name, value=None):
            if name.endswith(".dirty"):
                self.dirty_reads += 1
                dirty = 1 if self.dirty_reads == 2 else 0
                return Value(natural8=Natural8([dirty]))
            return super()._access(name, value)

    stager = _MidReadStager()
    rec = stager.read_calibration()
    assert rec.orientation == 0
    assert stager.dirty_reads == 4   # pre, post(dirty) -> retry -> pre, post


@needs_dsdl
def test_descriptor_token_prefers_the_served_generation():
    """The active slot cannot express a RAM-to-RAM swap; the descriptor
    epoch register can, and old firmware falls back to the slot."""
    import uavcan.primitive.array.Natural16_1_0 as Natural16
    import uavcan.register.Value_1_0 as Value

    class _EpochRegs(_Fake):
        def __init__(self, serve):
            super().__init__()
            self.serve = serve
            self.slot_reads = 0

        def _access(self, name, value=None):
            assert name == "aliensense.nxs.descriptor.epoch"
            if self.serve is None:
                raise RuntimeError("register does not exist")
            return Value(natural16=Natural16([self.serve]))

        def read_active_slot(self):
            self.slot_reads += 1
            return 0xFF

    served = _EpochRegs(serve=42)
    assert served._descriptor_token() == 42
    assert served.slot_reads == 0

    legacy = _EpochRegs(serve=None)
    assert legacy._descriptor_token() == 0xFF
    assert legacy.slot_reads == 1


@needs_dsdl
def test_failed_staging_discards_the_stage_before_raising():
    """A write_calibration that dies mid-stage leaves a dirty partial on
    the device; the client must drop it (commit register, value 0) before
    re-raising, or the next single-field commit applies the leftovers."""
    import uavcan.primitive.array.Natural8_1_0 as Natural8
    import uavcan.primitive.array.Natural32_1_0 as Natural32
    import uavcan.primitive.array.Real32_1_0 as Real32
    import uavcan.register.Value_1_0 as Value
    from nxs.client import CalibrationRecord

    class _DyingRegs(_Fake):
        def __init__(self, die_at):
            super().__init__()
            self.writes = 0
            self.die_at = die_at
            self.discards = 0

        def _access(self, name, value=None):
            if name.endswith(".commit"):
                if (value is not None and value.natural8 is not None
                        and list(value.natural8.value[:1]) == [0]):
                    self.discards += 1
                return Value(natural8=Natural8([0]))
            if value is not None:
                self.writes += 1
                if self.writes == self.die_at:
                    raise OSError("transport dropped mid-stage")
                return value
            if name.endswith(".orientation"):
                return Value(natural8=Natural8([0]))
            if name.endswith(".driver_tags"):
                return Value(natural32=Natural32([0, 0, 0, 0]))
            return Value(real32=Real32([0.0] * 12))

    bus = _DyingRegs(die_at=3)
    with pytest.raises(OSError):
        bus.write_calibration(CalibrationRecord(), persist=False)
    assert bus.discards == 1

    # set_orientation shares the epilogue.
    bus = _DyingRegs(die_at=1)
    with pytest.raises(OSError):
        bus.set_orientation(0, persist=False)
    assert bus.discards == 1

    # A clean sequence never discards.
    bus = _DyingRegs(die_at=10_000)
    bus.write_calibration(CalibrationRecord(), persist=False)
    assert bus.discards == 0
