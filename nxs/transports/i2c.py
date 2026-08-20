"""
NXS I2C transport — register-map protocol over smbus2.

NxsI2cTransport for real hardware, plus the REG_*/CMD_* register-map
constants the host protocol uses. The board-free MockTransport lives
in nxs.transports.mock.
"""

import errno
import struct
import time
import logging
from typing import Optional

try:
    import smbus2
except ImportError:  # absent off-Linux; test seams inject the bus
    smbus2 = None

from nxs.client import (
    CALIB_ERR_REASON, CAL_READ_ATTEMPTS, CAN_TERM_UNSET, COMMISSION_ERR_REASON, COMMISSION_TOPICS, DFU_ERR_REASON,
    DFU_ERASE_TIMEOUT_S, ERRNO_ENOENT, LOAD_ERR_REASON, XFER_ERR_REASON,
    STORE_CMD_TIMEOUT_S as _STORE_CMD_TIMEOUT_S,
    CalibrationRecord, DeviceRefused, NxsClient, SupportsCalibration, SupportsBitTiming, SupportsCanTermination,
    SupportsCommissioning, SupportsFaultCounters, SupportsIdentify,
    SupportsRecovery, SupportsSlotPeek, SupportsTimeSync, XFER_EBUSY,
    err_reason,
    validate_can_bitrate, validate_can_term, validate_commission)
from nxs.image import FIELD_TYPE_NAMES
from nxs._generated_constants import (
    Calibration as CalConstants, CyphalDefaults, FieldSemantics, NxsRegisters,
    RunnerStates, NxsDevices)

# Compiled subject-ID default per commission topic (constants SSOT). Resolves an
# UNSET (0xFFFF) record field to the value the device actually runs on, so I²C
# read_identity reports effective addresses like the Cyphal path.
_TOPIC_DEFAULTS = {
    "sample": CyphalDefaults.SAMPLE_SUBJECT_ID,
    "status": CyphalDefaults.STATUS_SUBJECT_ID,
    "acceleration": CyphalDefaults.ACCEL_SUBJECT_ID,
    "angular_velocity": CyphalDefaults.GYRO_SUBJECT_ID,
    "magnetic_field": CyphalDefaults.MAGNETIC_FIELD_SUBJECT_ID,
    "temperature": CyphalDefaults.TEMPERATURE_SUBJECT_ID,
    "pressure": CyphalDefaults.PRESSURE_SUBJECT_ID,
    "gnss": CyphalDefaults.GNSS_SUBJECT_ID,
    "scalar": CyphalDefaults.SCALAR_SUBJECT_BASE,
}

# Subject token -> SubjectBucket value for the DECIMATION_SELECT
# register; the same tokens name the Cyphal decimation.<subject>
# registers. NONE (0) is not addressable — nothing publishes under it.
SUBJECT_BUCKETS = {
    name.lower(): value
    for value, name in FieldSemantics.SubjectBucket._NAMES.items()
    if name != 'NONE'
}

log = logging.getLogger(__name__)

# I2C streaming is host-paced: the host polls the SAMPLE_DATA window (which
# always holds the latest sample, so slower polling just decimates). --hz sets
# the rate; DEFAULT_POLL_HZ is the fallback when the driver declares no sample_rate.
DEFAULT_POLL_HZ = 100.0

# ── NXS register map addresses ─────────────────────────────

_Reg = NxsRegisters.Reg
_Cmd = NxsRegisters.Cmd
_DfuPhase = NxsRegisters.DfuPhase

REG_WHO_AM_I      = _Reg.WHO_AM_I
REG_STATUS        = _Reg.STATUS
REG_VM_STATE      = _Reg.VM_STATE
REG_ERROR_CODE    = _Reg.ERROR_CODE
REG_SAMPLE_COUNT  = _Reg.SAMPLE_COUNT_LO  # 16-bit LE
REG_SAMPLE_SIZE   = _Reg.SAMPLE_SIZE  # 16-bit LE (1 byte used)
REG_NUM_PARAMS    = _Reg.NUM_PARAMS
REG_NUM_OUTPUTS   = _Reg.NUM_OUTPUTS
REG_DRIVER_NAME_LEN = _Reg.DRIVER_NAME_LEN
REG_STORE_COUNT   = _Reg.STORE_COUNT
REG_ACTIVE_SLOT   = _Reg.ACTIVE_SLOT
REG_RUNNER_STATE  = _Reg.RUNNER_STATE
REG_DECIMATION    = _Reg.DECIMATION_LO  # 16-bit LE
REG_DECIMATION_SELECT = _Reg.DECIMATION_SELECT  # SubjectBucket selector, deferred echo
REG_DECIMATION_VALUE  = _Reg.DECIMATION_VALUE  # selected subject's factor, u16 LE
REG_CAN_TERM      = _Reg.CAN_TERM  # 1 byte: 0 off, 1 on, 0xFF revert
REG_PROBE_RETRIES = _Reg.PROBE_RETRIES
REG_CMD           = _Reg.CMD
REG_PROGRAM_SIZE  = _Reg.PROGRAM_SIZE_LO  # 16-bit LE
REG_PARAM_SELECT  = _Reg.PARAM_SELECT
REG_PARAM_SET_VALUE = _Reg.PARAM_SET_VALUE  # 4 bytes (u32 LE)
REG_STORE_SELECT  = _Reg.STORE_SELECT
REG_PROTO_VERSION = _Reg.PROTO_VERSION  # register-map contract version
REG_FW_VERSION_MAJOR = _Reg.FW_VERSION_MAJOR
REG_FW_VERSION_MINOR = _Reg.FW_VERSION_MINOR
REG_DESCRIPTOR_EPOCH = _Reg.DESCRIPTOR_EPOCH  # 0 = no descriptors readable; bumps per driver (re)load
REG_OUTPUT_SELECT = _Reg.OUTPUT_SELECT
REG_PROGRAM_DATA  = _Reg.PROGRAM_DATA  # 32-byte write window
REG_SAMPLE_DATA   = _Reg.SAMPLE_DATA  # 128-byte window, valid for SAMPLE_SIZE bytes

# Selected-descriptor window (0xC0..0xE9), shared by the param, output,
# and driver views; the selector written last (REG_PARAM_SELECT,
# REG_OUTPUT_SELECT, or REG_DRIVER_SELECT) decides which one it shows.
REG_SEL_NAME_LEN = _Reg.SEL_NAME_LEN
REG_SEL_NAME     = _Reg.SEL_NAME  # 16 bytes
REG_SEL_TYPE     = _Reg.SEL_TYPE  # param view: low nibble 0 enum / 1 range + bit4 kind; output view: field type code
SEL_TYPE_PARAM_TYPE_MASK = NxsRegisters.SEL_TYPE_PARAM_TYPE_MASK
SEL_TYPE_KIND_SHIFT      = NxsRegisters.SEL_TYPE_KIND_SHIFT

# Param view.
REG_SEL_PARAM_DEFAULT  = _Reg.SEL_PARAM_DEFAULT  # 4 bytes (u32 LE)
REG_SEL_PARAM_CURRENT  = _Reg.SEL_PARAM_CURRENT  # 4 bytes (u32 LE)
REG_SEL_PARAM_NUM_VALS = _Reg.SEL_PARAM_NUM_VALS

# Output view.
REG_SEL_OUTPUT_SCALE      = _Reg.SEL_OUTPUT_SCALE  # 4 bytes (f32 LE)
REG_SEL_OUTPUT_OFFSET     = _Reg.SEL_OUTPUT_OFFSET  # 4 bytes (f32 LE)
REG_SEL_OUTPUT_BYTE_ORDER = _Reg.SEL_OUTPUT_BYTE_ORDER  # 0 = big, 1 = little

REG_SEL_UNIT_LEN = _Reg.SEL_UNIT_LEN
REG_SEL_UNIT     = _Reg.SEL_UNIT  # 8 bytes

REG_SEL_VALUE_INDEX     = _Reg.SEL_VALUE_INDEX  # param view: page selector (writable)
REG_SEL_VALUE           = _Reg.SEL_VALUE  # param view: values[SEL_VALUE_INDEX], u32 LE
REG_SEL_OUTPUT_SEMANTIC = _Reg.SEL_OUTPUT_SEMANTIC
REG_SEL_OUTPUT_COUNT    = _Reg.SEL_OUTPUT_COUNT  # 2 bytes (u16 LE): string width, 0 for numeric
REG_SEL_OUTPUT_AT       = _Reg.SEL_OUTPUT_AT  # byte position within the sample

REG_DRIVER_SELECT = _Reg.DRIVER_SELECT  # DRIVER_VIEW_DIAG -> diag view; else driver name
DRIVER_VIEW_DIAG = NxsRegisters.DRIVER_VIEW_DIAG
_Diag = NxsRegisters.DiagCounter
REG_SEL_DRIVER_NUM_PARAMS = _Reg.SEL_DRIVER_NUM_PARAMS
REG_SEL_DRIVER_NUM_OUTPUTS = _Reg.SEL_DRIVER_NUM_OUTPUTS
REG_SEL_DRIVER_SLOT = _Reg.SEL_DRIVER_SLOT
REG_SEL_DRIVER_I2C_ADDR = _Reg.SEL_DRIVER_I2C_ADDR
DRIVER_VIEW_PEEK = NxsRegisters.DRIVER_VIEW_PEEK
REG_SERIAL        = _Reg.SERIAL  # 12-byte chip UID96
SERIAL_LEN        = NxsRegisters.SERIAL_LEN

# Selector writes (PARAM_SELECT / OUTPUT_SELECT) are committed by the
# device's comm thread, not in the I2C ISR, so the descriptor window
# refills asynchronously. The firmware resets the inactive selector's
# echo to SELECTOR_INACTIVE (0xFF) on each view commit, so polling the
# echo until it reads back the written index is a deterministic
# "commit landed" signal — never a stale match. Budget is generous
# (~25 ms); the commit normally lands within one extra read.
SEL_POLL_INTERVAL_S = 0.0005
SEL_POLL_ATTEMPTS = 50

# Firmware writes this to the inactive selector's echo on every view
# commit (RegisterMapFrontend.h::SELECTOR_INACTIVE). The host doesn't
# need the value to drive the handshake — it polls for its written
# index — but the constant documents the wire contract on this side.
SELECTOR_INACTIVE = NxsRegisters.SELECTOR_INACTIVE

CMD_LOAD         = _Cmd.LOAD
CMD_RUN          = _Cmd.RUN
CMD_STOP         = _Cmd.STOP
CMD_RESET        = _Cmd.RESET
CMD_SAVE         = _Cmd.SAVE
CMD_DELETE_SLOT  = _Cmd.DELETE_SLOT
CMD_CLEAR_STORE  = _Cmd.CLEAR_STORE
CMD_CYCLE        = _Cmd.CYCLE
CMD_IDENTIFY     = _Cmd.IDENTIFY
CMD_PEEK_SLOT    = _Cmd.PEEK_SLOT

RUNNER_STATES = dict(RunnerStates.RunnerState._NAMES)

WHO_AM_I_VALUE = NxsRegisters.WHO_AM_I_VALUE
UPLOAD_CHUNK_SIZE = NxsRegisters.PROGRAM_CHUNK_SIZE

# The contract capability lives in nxs.client (SUPPORTED_PROTO_VERSION,
# contract_mismatch); re-exported here for the existing import sites.

# DFU + store command registers. XFER_TYPE selects the PROGRAM_DATA
# consumer; XFER_PHASE / XFER_ACK pace a DFU push; CMD_ERROR holds the last
# command op's result (0 = OK, else errno), reading CMD_ERR_PENDING until an
# async store command resolves.

REG_XFER_TYPE   = _Reg.XFER_TYPE
REG_XFER_PHASE  = _Reg.XFER_PHASE
REG_XFER_ACK    = _Reg.XFER_ACK
REG_CMD_ERROR   = _Reg.CMD_ERROR

CMD_ERR_PENDING = NxsRegisters.CMD_ERR_PENDING

XFER_TYPE_VM_BYTECODE = NxsRegisters.XFER_TYPE_VM_BYTECODE
XFER_TYPE_DFU_IMAGE   = NxsRegisters.XFER_TYPE_DFU_IMAGE
XFER_TYPE_CONFIG      = NxsRegisters.XFER_TYPE_CONFIG
XFER_TYPE_TIME_SYNC   = NxsRegisters.XFER_TYPE_TIME_SYNC
TIME_SYNC_RECORD_SIZE = NxsRegisters.TIME_SYNC_RECORD_SIZE
XFER_TYPE_CALIB       = NxsRegisters.XFER_TYPE_CALIB
XFER_TYPE_BUILD_INFO  = NxsRegisters.XFER_TYPE_BUILD_INFO
BUILD_INFO_SIZE       = NxsRegisters.BUILD_INFO_SIZE

CMD_DFU_BEGIN  = _Cmd.DFU_BEGIN
CMD_DFU_FINISH = _Cmd.DFU_FINISH
CMD_XFER_ABORT = _Cmd.XFER_ABORT
CMD_REBOOT     = _Cmd.REBOOT
CMD_ENTER_RECOVERY = _Cmd.ENTER_RECOVERY
CMD_STORE_PERSIST = _Cmd.STORE_PERSIST
CMD_CALIB_APPLY   = _Cmd.CALIB_APPLY
CMD_CALIB_PERSIST = _Cmd.CALIB_PERSIST
CMD_CAL_GYRO      = _Cmd.CAL_GYRO
CMD_CAL_MAG_START = _Cmd.CAL_MAG_START
CMD_CAL_MAG_STOP  = _Cmd.CAL_MAG_STOP

CONFIG_RECORD_SIZE = NxsRegisters.CONFIG_RECORD_SIZE
# The u32 CAN bitrate pair sits after the u16 node address + subject addresses.
_BITRATE_OFFSET = 2 + 2 * len(COMMISSION_TOPICS)

DFU_XFER_REASONS = {**DFU_ERR_REASON, **XFER_ERR_REASON}

_PHASE_NAMES = _DfuPhase._NAMES
_MODE_NAMES = {0: "VM_BYTECODE", 1: "DFU_IMAGE", 2: "CONFIG", 3: "TIME_SYNC",
               4: "CALIB"}
# Calibration record window: staged in PROGRAM_CHUNK_SIZE chunks under
# XFER_TYPE = CALIB, read back one STORE_SELECT-selected page at a time
# (re-read the whole record and retry on a mismatch to bracket a
# concurrent change). On-device procedure progress reuses the DFU
# transfer registers: XFER_PHASE = CalState, XFER_ACK = detail, and the
# completion verdict rides CMD_ERROR.
CALIB_PAGE_SIZE = NxsRegisters.PROGRAM_CHUNK_SIZE

DFU_PHASE_IDLE      = _DfuPhase.IDLE
DFU_PHASE_ERASING   = _DfuPhase.ERASING
DFU_PHASE_READY     = _DfuPhase.READY
DFU_PHASE_WRITING   = _DfuPhase.WRITING
DFU_PHASE_FINISHING = _DfuPhase.FINISHING
DFU_PHASE_ERROR     = _DfuPhase.ERROR


class _BuildInfoBusy(Exception):
    """Internal: the build-info window is session-held; fall back."""


class NxsI2cTransport(NxsClient, SupportsCommissioning, SupportsIdentify,
                      SupportsSlotPeek, SupportsBitTiming,
                      SupportsCanTermination, SupportsTimeSync,
                      SupportsRecovery, SupportsFaultCounters,
                      SupportsCalibration):
    """I2C register-map transport for a single NXS device."""

    def __init__(self, bus=None, address: int = NxsDevices.RBDevice.NXS,
                 poll_hz=None, _bus_obj=None):
        """
        Args:
            bus: I2C bus — either an integer (bus number) or a string
                 device path (e.g., "/dev/i2c-30").
            address: NXS device address (default RBDevice::NXS = 0x30).
            poll_hz: Sample-stream poll rate in Hz. None → the driver's
                 advertised sample_rate, else DEFAULT_POLL_HZ. No cap —
                 the operator sizes this to the bus (see --hz).
            _bus_obj: Test seam — duck-typed smbus2-like object. When
                 given, `bus` is ignored and smbus2 is never imported.
        """
        super().__init__()
        self._addr = address
        self._last_probe_error: Optional[OSError] = None
        self._record_data_len = 0  # cached SAMPLE_SIZE; set at stream arm
        self._last_count = 0
        self._poll_hz = None
        self._poll_interval = 0.0
        self._sample_period = 0.0
        self._next_due = 0.0
        if poll_hz is not None:
            self.set_output_rate(poll_hz)
        if _bus_obj is not None:
            self._bus = _bus_obj
            return
        if bus is None:
            raise ValueError(
                "NxsI2cTransport needs a bus: an int bus number or a "
                "'/dev/i2c-N' path")
        if smbus2 is None:
            raise ImportError("smbus2 is required: pip install smbus2")
        if isinstance(bus, str):
            # "/dev/i2c-30" → 30
            tail = bus.rsplit("-", 1)[-1]
            if not tail.isdigit():
                raise ValueError(
                    f"I2C bus '{bus}': expected /dev/i2c-N or an integer")
            bus_number = int(tail)
        else:
            bus_number = int(bus)
        self._bus = smbus2.SMBus(bus_number)

    def probe(self) -> bool:
        """Check if the module is present at the configured address."""
        self._last_probe_error = None
        try:
            who = self._bus.read_byte_data(self._addr, REG_WHO_AM_I)
            return who == WHO_AM_I_VALUE
        except OSError as e:
            # Not the same as absent hardware: EACCES is group membership,
            # ENXIO a wrong address, ENOENT a bus that does not exist.
            self._last_probe_error = e
            return False

    def probe_failure_detail(self) -> Optional[str]:
        return str(self._last_probe_error) if self._last_probe_error else None

    def interface_version(self) -> int:
        """Register-map contract version (0 on firmware that predates it)."""
        return self._bus.read_byte_data(self._addr, REG_PROTO_VERSION)

    def read_serial(self) -> bytes:
        """Read the 12-byte chip UID96 from the SERIAL window."""
        return bytes(self._bus.read_i2c_block_data(
            self._addr, REG_SERIAL, SERIAL_LEN))

    def read_fw_version(self) -> Optional[str]:
        """The firmware build identity, verbatim `git describe`
        ("v1.0.0-4-g87fdf5b"), from the BUILD_INFO transfer. Firmware
        predating the transfer falls back to "MAJOR.MINOR" from the
        FW_VERSION registers; None when both are dark. XFER_TYPE and
        STORE_SELECT are restored around the read (never forced to 0:
        an XFER_TYPE = 0 write clears CMD_ERROR, and a status poll
        must not eat a pending command verdict)."""
        prev_xfer = self._bus.read_byte_data(self._addr, REG_XFER_TYPE)
        prev_page = self._bus.read_byte_data(self._addr, REG_STORE_SELECT)
        raw = b""
        try:
            # The mode write commits deferred: page only after the echo, or
            # the first read consumes the previous mode's window and this
            # silently degrades to the legacy pair. A refusal (live
            # transfer session) skips to that fallback — the window is
            # owned, and reading it would answer with another transfer's
            # bytes.
            try:
                self._claim_xfer_mode(XFER_TYPE_BUILD_INFO)
            except DeviceRefused:
                raise _BuildInfoBusy()
            for page in range(BUILD_INFO_SIZE // NxsRegisters.PROGRAM_CHUNK_SIZE):
                self._bus.write_byte_data(self._addr, REG_STORE_SELECT, page)
                raw += bytes(self._bus.read_i2c_block_data(
                        self._addr, REG_PROGRAM_DATA,
                        NxsRegisters.PROGRAM_CHUNK_SIZE))
                if 0 in raw:
                    break
        except _BuildInfoBusy:
            pass
        finally:
            self._bus.write_byte_data(self._addr, REG_STORE_SELECT, prev_page)
            if prev_xfer != XFER_TYPE_BUILD_INFO:
                self._bus.write_byte_data(self._addr, REG_XFER_TYPE, prev_xfer)
        text = raw.split(b"\x00", 1)[0]
        if text and all(0x20 < b < 0x7F for b in text):
            return text.decode("ascii")
        # Legacy firmware: the window served the identity-config mirror
        # (or zeros); the numeric pair is the only version the wire proves.
        major = self._bus.read_byte_data(self._addr, REG_FW_VERSION_MAJOR)
        if major == 0:
            return None
        minor = self._bus.read_byte_data(self._addr, REG_FW_VERSION_MINOR)
        return f"{major}.{minor}"

    def vm_run(self):
        """Start the VM on the module."""
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_RUN)
        log.info("VM running")

    def vm_stop(self):
        """Stop the VM on the module; the driver stays loaded."""
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_STOP)
        log.info("VM stopped")

    def vm_reset(self):
        """Unload the driver entirely."""
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_RESET)
        log.info("VM reset")

    def read_status(self) -> int:
        """Read the STATUS register."""
        return self._bus.read_byte_data(self._addr, REG_STATUS)

    def read_decimation(self, subject=None) -> int:
        """Read the device-wide output decimation factor, or a subject's
        factor via the DECIMATION_SELECT/VALUE window (the register
        mirror of `aliensense.nxs.decimation.<subject>`)."""
        if subject is None:
            return self._bus.read_word_data(self._addr, REG_DECIMATION)
        self._select_decim_subject(subject)
        return self._bus.read_word_data(self._addr, REG_DECIMATION_VALUE)

    def write_decimation(self, value: int, subject=None) -> None:
        """Set the device-wide output decimation factor, or a subject's
        factor (0 = off, 1 = every device-output sample, N = every Nth).
        Per-subject factors thin only the Cyphal SI fan-out and are
        volatile until `commission --save` persists them."""
        if subject is None:
            self._bus.write_word_data(self._addr, REG_DECIMATION, value & 0xFFFF)
            return
        self._select_decim_subject(subject)
        self._bus.write_word_data(self._addr, REG_DECIMATION_VALUE, value & 0xFFFF)

    def _select_decim_subject(self, subject: str):
        """Write the subject's SubjectBucket to DECIMATION_SELECT and poll
        the deferred echo, like every selector on this map."""
        bucket = SUBJECT_BUCKETS.get(subject)
        if bucket is None:
            raise ValueError(f"unknown subject {subject!r} "
                             f"(one of {sorted(SUBJECT_BUCKETS)})")
        self._bus.write_byte_data(self._addr, REG_DECIMATION_SELECT, bucket)
        self._await_selector(REG_DECIMATION_SELECT, bucket)

    # ── Time sync (star push) ─────────────────────────────
    def _claim_xfer_mode(self, mode: int):
        """Write `XFER_TYPE` and detect the silent refusal by readback:
        a live transfer session holds the mux and the mode does not move.
        The write commits deferred (the device queues control writes for
        its comm thread), so the readback polls a short echo budget — a
        lagging drain must not read as a refusal. Raises
        DeviceRefused(EBUSY) so callers never stream into another
        transfer's sink."""
        self._bus.write_byte_data(self._addr, REG_XFER_TYPE, mode)
        for _ in range(SEL_POLL_ATTEMPTS):
            if self._bus.read_byte_data(self._addr, REG_XFER_TYPE) == mode:
                return
            time.sleep(SEL_POLL_INTERVAL_S)
        raise DeviceRefused(XFER_EBUSY,
                            err_reason(XFER_EBUSY, XFER_ERR_REASON))

    def push_time_sync(self, offset_us: int, bound_us: int,
                       rate_ppb: int = 0,
                       valid_for_us: int = 0) -> None:
        """Stream the volatile 20-byte record; the device applies it as
        the 20th byte lands and stays in time-sync mode for the next
        push."""
        record = struct.pack('<qIiI', offset_us, bound_us, rate_ppb,
                             valid_for_us)
        # A held mux would land the record in someone else's sink; the
        # claim raises and the pusher skips this interval.
        self._claim_xfer_mode(XFER_TYPE_TIME_SYNC)
        self._bus.write_i2c_block_data(self._addr, REG_PROGRAM_DATA,
                                       list(record))

    def read_time_sync(self) -> tuple:
        """The live discipline from the record mirror:
        `(offset_us, bound_us, rate_ppb, valid_for_us, source, valid)`.
        The mirror is painted at read time once the queued mode select
        lands, so poll until the record parses in-contract (valid 0/1,
        source <= 3) — a not-yet-selected window serves other-mode
        bytes."""
        # Claim the mode with the echo-polled budget, not a single readback:
        # the write commits deferred, so a lagging drain must not read as a
        # refusal (that false negative also skipped the finally and parked
        # the mux at TIME_SYNC).
        try:
            self._claim_xfer_mode(XFER_TYPE_TIME_SYNC)
        except DeviceRefused:
            # A transfer session genuinely holds the mux (an upload or
            # firmware push in flight); the mirror is unreadable until it
            # ends. Expected texture of a status call during a push.
            log.info("time-sync mirror unavailable: transfer session live")
            return 0, 0, 0, 0, 0, False
        try:
            for _ in range(SEL_POLL_ATTEMPTS):
                raw = bytes(self._bus.read_i2c_block_data(
                        self._addr, REG_PROGRAM_DATA, TIME_SYNC_RECORD_SIZE))
                offset_us, bound_us, rate_ppb, valid_for_us, source, valid = \
                        struct.unpack('<qIiIBB', raw)
                if valid in (0, 1) and source <= 3:
                    return (offset_us, bound_us, rate_ppb, valid_for_us,
                            source, bool(valid))
                time.sleep(SEL_POLL_INTERVAL_S)
            log.warning("time-sync mirror unreadable after %d polls",
                        SEL_POLL_ATTEMPTS)
            return 0, 0, 0, 0, 0, False
        finally:
            # Leave the mux on the bytecode consumer, like read_identity —
            # safe now that a mode write never clears CMD_ERROR (a pending
            # store result survives this restore).
            try:
                self._bus.write_byte_data(self._addr, REG_XFER_TYPE,
                                          XFER_TYPE_VM_BYTECODE)
            except OSError:
                pass

    def read_device_time_us(self) -> Optional[int]:
        """The device µs clock via the sample-window latch: the target
        stamps `latch_time_us` at the first byte of any `REG_SAMPLE_DATA`
        read, sample or not. None on a zero latch."""
        buf = bytes(self._bus.read_i2c_block_data(
                self._addr, REG_SAMPLE_DATA, 8))
        latch_us = int.from_bytes(buf, 'little')
        return latch_us or None

    # ── Commissioning (identity) ──────────────────────────
    def _read_config_record(self) -> bytes:
        """Read the identity record from the config window."""
        self._claim_xfer_mode(XFER_TYPE_CONFIG)
        return bytes(self._bus.read_i2c_block_data(self._addr, REG_PROGRAM_DATA,
                                                   CONFIG_RECORD_SIZE))

    def _write_config_record(self, record: bytes) -> None:
        """Stage a full identity record (re-enters config mode to reset the offset)."""
        self._claim_xfer_mode(XFER_TYPE_CONFIG)
        self._bus.write_i2c_block_data(self._addr, REG_PROGRAM_DATA,
                                       list(record[:CONFIG_RECORD_SIZE]))

    def read_identity(self) -> dict:
        try:
            record = self._read_config_record()
        finally:
            # Leave config mode: driver uploads rely on XFER_TYPE = 0.
            self._bus.write_byte_data(self._addr, REG_XFER_TYPE, XFER_TYPE_VM_BYTECODE)
        node = struct.unpack_from("<H", record, 0)[0]
        if node == 0xFFFF:
            node = CyphalDefaults.DEFAULT_NODE_ID
        topics = {}
        for i, n in enumerate(COMMISSION_TOPICS):
            v = struct.unpack_from("<H", record, 2 + 2 * i)[0]
            topics[n] = _TOPIC_DEFAULTS[n] if v == 0xFFFF else v
        return {"node_addr": node, "topics": topics}

    def commission(self, node_addr=None, topics=None, can_bitrate=None,
                   can_term=None) -> None:
        validate_commission(node_addr, topics)
        if can_bitrate is not None:
            validate_can_bitrate(*can_bitrate)
        # Termination is a live register, not a record field: apply it first,
        # and the STORE_PERSIST below commits the staged selection with the
        # rest of the config.
        if can_term is not None:
            self.write_can_term(can_term)
        try:
            record = bytearray(self._read_config_record())
            if node_addr is not None:
                struct.pack_into("<H", record, 0, node_addr)
            for name, addr in (topics or {}).items():
                struct.pack_into("<H", record, 2 + 2 * COMMISSION_TOPICS.index(name), addr)
            if can_bitrate is not None:
                struct.pack_into("<II", record, _BITRATE_OFFSET, *can_bitrate)
            self._write_config_record(record)
            self._bus.write_byte_data(self._addr, REG_CMD, CMD_STORE_PERSIST)
            self._await_store_result(reasons=COMMISSION_ERR_REASON)
        finally:
            # Leave config mode even on failure: driver uploads rely on
            # XFER_TYPE = 0, and the write releases the firmware's config
            # window back to mirroring the live record.
            self._bus.write_byte_data(self._addr, REG_XFER_TYPE, XFER_TYPE_VM_BYTECODE)
        log.info("Committed identity config")

    # ── CAN termination ───────────────────────────────────
    def read_can_term(self) -> int:
        """Read the effective termination selection (0 = off, 1 = on)."""
        return self._bus.read_byte_data(self._addr, REG_CAN_TERM)

    def write_can_term(self, value: int) -> None:
        """Set the selection (0, 1, or 0xFFFF → byte 0xFF, revert to off).
        Applies live; persist with `Cmd::STORE_PERSIST` / commission."""
        validate_can_term(value)
        byte = 0xFF if value == CAN_TERM_UNSET else value
        self._bus.write_byte_data(self._addr, REG_CAN_TERM, byte)
        # An ACKed write proves nothing about acceptance — firmware without
        # the register ACKs and ignores it. Read back the effective state so
        # the caller fails loudly instead of proceeding unterminated.
        echo = self.read_can_term()
        expect = 0 if value == CAN_TERM_UNSET else value
        if echo != expect:
            raise DeviceRefused(errno.EINVAL,
                                f"device rejected can-term {value} (reads {echo}) — "
                                f"firmware without CAN_TERM support?")

    # ── CAN bit timing ────────────────────────────────────
    def read_can_bitrate(self) -> tuple:
        try:
            record = self._read_config_record()
        finally:
            self._bus.write_byte_data(self._addr, REG_XFER_TYPE, XFER_TYPE_VM_BYTECODE)
        nominal, data = struct.unpack_from("<II", record, _BITRATE_OFFSET)
        if nominal == 0:
            return (CyphalDefaults.CAN_BITRATE_DEFAULT,
                    CyphalDefaults.CAN_BITRATE_DATA_DEFAULT)
        return nominal, data

    def write_can_bitrate(self, nominal: int, data: int) -> None:
        """Commit the pair through the config record (the record is the only
        I²C commissioning surface, so the write persists immediately)."""
        self.commission(can_bitrate=(nominal, data))

    # ── Calibration ───────────────────────────────────────
    def read_cal_epoch(self) -> int:
        """Byte RECORD_SIZE of the CALIB read-back — the bank's change
        counter. STORE_SELECT and XFER_TYPE are restored so a probe on a
        streaming cadence leaves both where their owner put them; a mode
        of 0 is never written back (that write clears CMD_ERROR and
        would eat a pending command verdict)."""
        size = CalConstants.RECORD_SIZE
        prev_page = self._bus.read_byte_data(self._addr, REG_STORE_SELECT)
        prev_xfer = self._bus.read_byte_data(self._addr, REG_XFER_TYPE)
        self._claim_xfer_mode(XFER_TYPE_CALIB)
        try:
            self._bus.write_byte_data(self._addr, REG_STORE_SELECT,
                                      size // CALIB_PAGE_SIZE)
            page = self._bus.read_i2c_block_data(
                    self._addr, REG_PROGRAM_DATA,
                    size % CALIB_PAGE_SIZE + 1)
            return page[size % CALIB_PAGE_SIZE]
        finally:
            self._bus.write_byte_data(self._addr, REG_STORE_SELECT, prev_page)
            if prev_xfer not in (0, XFER_TYPE_CALIB):
                self._bus.write_byte_data(self._addr, REG_XFER_TYPE, prev_xfer)

    def read_calibration(self) -> CalibrationRecord:
        size = CalConstants.RECORD_SIZE
        # Byte RECORD_SIZE of the read-back is the bank's change counter;
        # one extra byte rides the final page for free.
        pages = (size + 1 + CALIB_PAGE_SIZE - 1) // CALIB_PAGE_SIZE
        epoch_page = size // CALIB_PAGE_SIZE
        epoch_off = size % CALIB_PAGE_SIZE

        def read_epoch() -> int:
            self._bus.write_byte_data(self._addr, REG_STORE_SELECT, epoch_page)
            page = self._bus.read_i2c_block_data(self._addr, REG_PROGRAM_DATA,
                                                 epoch_off + 1)
            return page[epoch_off]

        def read_record() -> bytes:
            buf = bytearray()
            for page in range(pages):
                self._bus.write_byte_data(self._addr, REG_STORE_SELECT, page)
                n = min(CALIB_PAGE_SIZE, size + 1 - page * CALIB_PAGE_SIZE)
                buf += bytes(self._bus.read_i2c_block_data(
                        self._addr, REG_PROGRAM_DATA, n))
            return bytes(buf)

        try:
            self._claim_xfer_mode(XFER_TYPE_CALIB)
            # Epoch bracket (the DESCRIPTOR_EPOCH idiom): epoch, record,
            # epoch. Equal brackets mean no repaint landed mid-read; a
            # moved bracket retries.
            for _ in range(CAL_READ_ATTEMPTS):
                before = read_epoch()
                buf = read_record()
                if buf[size] == before and read_epoch() == before:
                    return CalibrationRecord.unpack(buf[:size])
            # A host-side coherence failure (a concurrent writer kept moving
            # the record), not a device verdict — never DeviceRefused, whose
            # errnos carry calibration semantics (11 would read as EAGAIN,
            # "insufficient coverage").
            raise RuntimeError("calibration record kept changing during read "
                               "(concurrent writer?)")
        finally:
            self._bus.write_byte_data(self._addr, REG_XFER_TYPE, XFER_TYPE_VM_BYTECODE)

    def write_calibration(self, record: CalibrationRecord,
                          persist: bool = True) -> None:
        data = record.pack()
        try:
            self._claim_xfer_mode(XFER_TYPE_CALIB)
            for off in range(0, len(data), CALIB_PAGE_SIZE):
                self._bus.write_i2c_block_data(
                        self._addr, REG_PROGRAM_DATA,
                        list(data[off:off + CALIB_PAGE_SIZE]))
            self._bus.write_byte_data(self._addr, REG_CMD, CMD_CALIB_APPLY)
            self._await_store_result(reasons=CALIB_ERR_REASON)
            if persist:
                self._bus.write_byte_data(self._addr, REG_CMD, CMD_CALIB_PERSIST)
                self._await_store_result(reasons=CALIB_ERR_REASON)
        finally:
            self._bus.write_byte_data(self._addr, REG_XFER_TYPE, XFER_TYPE_VM_BYTECODE)
        log.info("Applied calibration record%s", " (persisted)" if persist else "")

    def set_orientation(self, rotation: int, persist: bool = True) -> None:
        record = self.read_calibration()
        record.orientation = rotation
        self.write_calibration(record, persist)

    def cal_gyro(self) -> None:
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_CAL_GYRO)
        self._await_store_result(reasons=CALIB_ERR_REASON)

    def cal_mag_start(self) -> None:
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_CAL_MAG_START)
        self._await_store_result(reasons=CALIB_ERR_REASON)

    def cal_mag_stop(self) -> None:
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_CAL_MAG_STOP)
        self._await_store_result(reasons=CALIB_ERR_REASON)

    def cal_abort(self) -> None:
        # XFER_ABORT is the register-map release for any open session and
        # any running procedure. Await the resolution: a fire-and-forget
        # write would report "nothing applied" to an operator whose abort
        # never landed. ENOENT (nothing was open) is success here — the
        # caller wanted nothing running, and nothing is.
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_XFER_ABORT)
        try:
            self._await_store_result(reasons=CALIB_ERR_REASON)
        except DeviceRefused as e:
            if e.code != ERRNO_ENOENT:
                raise

    def read_cal_progress(self):
        # Progress rides the reused transfer registers: XFER_PHASE = state,
        # XFER_ACK = detail, CMD_ERROR = the completion verdict (0 while the
        # procedure runs, then the errno on the idle edge).
        return (self._bus.read_byte_data(self._addr, REG_XFER_PHASE),
                self._bus.read_byte_data(self._addr, REG_XFER_ACK),
                self._bus.read_byte_data(self._addr, REG_CMD_ERROR))

    def save_calibration(self) -> None:
        # CALIB_PERSIST, not STORE_PERSIST: this save carries no identity
        # record, and the device must not have to infer which was meant.
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_CALIB_PERSIST)
        self._await_store_result(reasons=CALIB_ERR_REASON)

    def read_vm_state(self) -> int:
        """Read the VM state byte (0 idle, 1 running, 2 error)."""
        return self._bus.read_byte_data(self._addr, REG_VM_STATE)

    def read_error_code(self) -> int:
        """Read the current VM error code (0 when healthy)."""
        return self._bus.read_byte_data(self._addr, REG_ERROR_CODE)

    def read_sample_count(self) -> int:
        """Read the 16-bit sample counter."""
        return self._bus.read_word_data(self._addr, REG_SAMPLE_COUNT)

    def read_sample_size(self) -> int:
        """Read the per-sample byte count the driver advertises."""
        return self._bus.read_byte_data(self._addr, REG_SAMPLE_SIZE)

    # Mirrors RegisterMapFrontend.h::SAMPLE_DATA_WINDOW = 128. The window
    # carries one record — latch_time_us | timestamp_us | seq | data at the
    # SAMPLE_RECORD_* offsets, LE — so a poll is a single latched read that
    # returns seq, acquisition time, and data coherently, and every poll
    # doubles as a two-way time-sync observation via latch_time_us.
    SAMPLE_DATA_WINDOW = 128
    RECORD_HEADER = NxsRegisters.SAMPLE_RECORD_DATA_OFF

    def _read_record(self, data_len: int):
        """One latched read of the sample record: (seq, timestamp_us,
        data). Reads the header (+ whatever data fits the first SMBus
        chunk) first, brackets it with host stamps for the sync
        estimator, and fetches remaining data chunks from the same
        latch. `data_len` is the driver's SAMPLE_SIZE."""
        total = min(self.RECORD_HEADER + data_len, self.SAMPLE_DATA_WINDOW)
        rdwr = getattr(self._bus, 'i2c_rdwr', None)
        if rdwr is not None and smbus2 is not None:
            # One raw transfer (write reg + repeated-start read) covers
            # the whole record: the SMBus 32-byte ceiling does not apply,
            # so a sample costs one transaction and one syscall, and the
            # sync bracket tightens to that single transfer.
            wr = smbus2.i2c_msg.write(self._addr, [REG_SAMPLE_DATA])
            rd = smbus2.i2c_msg.read(self._addr, total)
            t0 = time.monotonic_ns()
            rdwr(wr, rd)
            t1 = time.monotonic_ns()
            buf = bytearray(bytes(rd))
        else:
            # Chunked SMBus fallback for duck-typed test buses. The
            # device latches at the window base and holds the snapshot
            # across continuation chunks, so the record stays coherent.
            first = min(32, total)
            t0 = time.monotonic_ns()
            buf = bytearray(self._bus.read_i2c_block_data(
                self._addr, REG_SAMPLE_DATA, first))
            t1 = time.monotonic_ns()
            offset = first
            while offset < total:
                chunk = min(32, total - offset)
                buf.extend(self._bus.read_i2c_block_data(
                    self._addr, REG_SAMPLE_DATA + offset, chunk))
                offset += chunk

        rec = NxsRegisters
        latch_us = int.from_bytes(
            buf[rec.SAMPLE_RECORD_LATCH_TIME_OFF:
                rec.SAMPLE_RECORD_LATCH_TIME_OFF + 8], 'little')
        timestamp_us = int.from_bytes(
            buf[rec.SAMPLE_RECORD_TIMESTAMP_OFF:
                rec.SAMPLE_RECORD_TIMESTAMP_OFF + 8], 'little')
        seq = int.from_bytes(
            buf[rec.SAMPLE_RECORD_SEQ_OFF:rec.SAMPLE_RECORD_SEQ_OFF + 2],
            'little')
        if latch_us:
            self._time_sync.observe(t0, t1, latch_us)
        return seq, timestamp_us, bytes(buf[self.RECORD_HEADER:total])

    def read_sample(self, size: int = None) -> bytes:
        """Read the latest sample's data bytes from the record window.

        `size` defaults to the driver-advertised SAMPLE_SIZE — the
        sample layout is whatever the loaded NXS's output-field
        descriptors say it is; nothing here assumes a sensor type.
        """
        if size is None:
            size = self.read_sample_size()
        return self._read_record(size)[2]

    # ── NXS image upload ──────────────────────────────────

    def upload_image(self, image_data: bytes, link: int = 0):
        """Upload a full NXS driver image (bytecode + capabilities).

        Claims the bytecode consumer explicitly (an inherited mode is how
        uploads used to stream into the time-sync sink), then reads the
        announce's verdict — the device arms `CMD_ERR_PENDING` and resolves
        0 / EBUSY / EPROTO / EFBIG — before streaming a single chunk.

        Args:
            image_data: Serialized NXS image from nxs.image.serialize().
        """
        size = len(image_data)
        self._claim_xfer_mode(XFER_TYPE_VM_BYTECODE)
        self._bus.write_word_data(self._addr, REG_PROGRAM_SIZE, size)
        self._await_store_result(reasons=XFER_ERR_REASON)
        # The announce opened a session; any exit before LOAD must release
        # it or the next attempt eats an unexplained EBUSY until the stale
        # window runs out. `finally` covers the operator's Ctrl-C too —
        # KeyboardInterrupt is not an Exception, and a deliberate cancel is
        # precisely what XFER_ABORT is for.
        staged = False
        try:
            for offset in range(0, size, UPLOAD_CHUNK_SIZE):
                chunk = list(image_data[offset:offset + UPLOAD_CHUNK_SIZE])
                self._bus.write_i2c_block_data(self._addr, REG_PROGRAM_DATA,
                                               chunk)
            self._bus.write_byte_data(self._addr, REG_CMD, CMD_LOAD)
            staged = True
        finally:
            if not staged:
                self._abort_transfer()
        # LOAD is async like the store commands: the device armed
        # CMD_ERR_PENDING in the ISR and resolves it with the parse's
        # verdict. Only a confirmed 0 earns the "Uploaded" log line.
        self._await_store_result(reasons=LOAD_ERR_REASON)
        log.info("Uploaded NXS image (%d bytes) to link %d", size, link)


    # ── Fault counters (SEL diag view) ─────────────────────

    def _read_diag_counter(self, index: int) -> int:
        """Page one fault counter out of the SEL diag view: select the
        view, pick the index, read the u32 window (the u16 counter
        zero-extended). Selector writes commit deferred, so each is
        awaited via its echo before the read. Leaves the SEL window in
        the diag view — every other reader re-selects its own view
        first, as this one does."""
        self._bus.write_byte_data(self._addr, REG_DRIVER_SELECT,
                                  DRIVER_VIEW_DIAG)
        got_view = self._await_selector(REG_DRIVER_SELECT, DRIVER_VIEW_DIAG)
        self._bus.write_byte_data(self._addr, REG_SEL_VALUE_INDEX, index)
        got_index = self._await_selector(REG_SEL_VALUE_INDEX, index)
        if not (got_view and got_index):
            # The selector never committed: SEL_VALUE still holds another
            # view's bytes. Fail loudly rather than return them as a counter
            # (nxs status catches this and omits the line).
            raise TimeoutError(
                "diag view did not commit — counter unreadable "
                "(device busy or a session holds the comm thread)")
        raw = self._bus.read_i2c_block_data(self._addr, REG_SEL_VALUE, 4)
        return int.from_bytes(bytes(raw), "little")

    def read_io_err_count(self) -> int:
        return self._read_diag_counter(_Diag.VM_IO_ERRORS)

    def read_probe_failed_count(self) -> int:
        return self._read_diag_counter(_Diag.PROBE_FAILURES)

    def read_drdy_coalesced_count(self) -> int:
        return self._read_diag_counter(_Diag.DRDY_COALESCED)

    def read_ingress_reject_count(self) -> int:
        return self._read_diag_counter(_Diag.INGRESS_REJECTS)

    def read_cmd_queue_overflow_count(self) -> int:
        """I2C-only: host writes dropped at a full device command queue."""
        return self._read_diag_counter(_Diag.I2C_CMD_QUEUE_OVERFLOWS)

    # ── Capabilities readback ──────────────────────────────

    def read_driver_name(self) -> str:
        """Read the loaded driver's name."""
        name_len = self._bus.read_byte_data(self._addr, REG_DRIVER_NAME_LEN)
        if name_len == 0:
            return ""
        # The name rides the SEL window's driver view: select it, then
        # read SEL_NAME.
        self._bus.write_byte_data(self._addr, REG_DRIVER_SELECT, 1)
        self._await_selector(REG_DRIVER_SELECT, 1)
        raw = self._bus.read_i2c_block_data(
            self._addr, REG_SEL_NAME, min(name_len, 16))
        return bytes(raw).decode('ascii', errors='replace')

    def read_slot_info(self, slot: int):
        """SupportsSlotPeek via `Cmd::PEEK_SLOT`: the device paints the
        peeked slot's header (name, counts, latched mikroBUS address)
        into the SEL window's peek view; None for an empty slot.
        `slot == 0xFF` peeks the active driver."""
        import errno
        from types import SimpleNamespace

        self._bus.write_byte_data(self._addr, REG_STORE_SELECT, slot)
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_PEEK_SLOT)
        try:
            self._await_store_result()
        except DeviceRefused as e:
            if e.code == errno.ENOENT:
                return None
            raise
        name_len = self._bus.read_byte_data(self._addr, REG_SEL_NAME_LEN)
        raw = self._bus.read_i2c_block_data(
            self._addr, REG_SEL_NAME, min(name_len, 16)) if name_len else []
        name = bytes(raw).decode('ascii', errors='replace')
        if not name:
            return None
        return SimpleNamespace(
            name=name,
            num_params=self._bus.read_byte_data(self._addr, REG_SEL_DRIVER_NUM_PARAMS),
            num_outputs=self._bus.read_byte_data(self._addr, REG_SEL_DRIVER_NUM_OUTPUTS),
            i2c_addr=self._bus.read_byte_data(self._addr, REG_SEL_DRIVER_I2C_ADDR))

    def read_num_params(self) -> int:
        """Read the number of configurable parameters."""
        return self._bus.read_byte_data(self._addr, REG_NUM_PARAMS)

    def _await_selector(self, reg: int, index: int) -> bool:
        """Block until a deferred selector commit lands — echo == index.

        Best-effort: returns True on commit, False after the budget so a
        wedged comm thread degrades rather than hanging. Callers that would
        otherwise serve garbage from the previous view must check the
        result. See SEL_POLL_* and SELECTOR_INACTIVE for the handshake.
        """
        for _ in range(SEL_POLL_ATTEMPTS):
            if self._bus.read_byte_data(self._addr, reg) == index:
                return True
            time.sleep(SEL_POLL_INTERVAL_S)
        log.debug("selector 0x%02X commit not observed (wrote %d)", reg, index)
        return False

    def read_param(self, index: int) -> dict:
        """Read a parameter descriptor by index.

        Returns a dict with: name, type, default, current, values, unit.
        """
        # Select the parameter, then wait for the deferred commit.
        self._bus.write_byte_data(self._addr, REG_PARAM_SELECT, index)
        self._await_selector(REG_PARAM_SELECT, index)

        name_len = self._bus.read_byte_data(self._addr, REG_SEL_NAME_LEN)
        name = ""
        if name_len > 0:
            raw = self._bus.read_i2c_block_data(
                self._addr, REG_SEL_NAME, min(name_len, 16))
            name = bytes(raw).decode('ascii', errors='replace')

        sel_type = self._bus.read_byte_data(self._addr, REG_SEL_TYPE)
        param_type = sel_type & SEL_TYPE_PARAM_TYPE_MASK
        kind = (sel_type >> SEL_TYPE_KIND_SHIFT) & 1

        default_raw = self._bus.read_i2c_block_data(
            self._addr, REG_SEL_PARAM_DEFAULT, 4)
        default_val = struct.unpack_from('<I', bytes(default_raw))[0]

        current_raw = self._bus.read_i2c_block_data(
            self._addr, REG_SEL_PARAM_CURRENT, 4)
        current_val = struct.unpack_from('<I', bytes(current_raw))[0]

        num_vals = self._bus.read_byte_data(self._addr, REG_SEL_PARAM_NUM_VALS)

        unit_len = self._bus.read_byte_data(self._addr, REG_SEL_UNIT_LEN)
        unit = ""
        if unit_len > 0:
            raw = self._bus.read_i2c_block_data(
                self._addr, REG_SEL_UNIT, min(unit_len, 8))
            unit = bytes(raw).decode('ascii', errors='replace')

        # Values are paged: write the index, read the u32. The device
        # echoes the index once the page is served, same commit edge as
        # the descriptor selectors.
        values = []
        for i in range(num_vals):
            self._bus.write_byte_data(self._addr, REG_SEL_VALUE_INDEX, i)
            self._await_selector(REG_SEL_VALUE_INDEX, i)
            v_raw = self._bus.read_i2c_block_data(
                self._addr, REG_SEL_VALUE, 4)
            values.append(struct.unpack_from('<I', bytes(v_raw))[0])

        return {
            'name': name,
            'type': 'enum' if param_type == 0 else 'range',
            'kind': 'live' if kind == 1 else 'reload',
            'default': default_val,
            'current': current_val,
            'values': values,
            'unit': unit,
        }

    def read_capabilities(self) -> list:
        """Read all parameter descriptors. Returns a list of dicts."""
        n = self.read_num_params()
        return [self.read_param(i) for i in range(n)]

    def read_num_outputs(self) -> int:
        """Read the number of output fields per sample."""
        return self._bus.read_byte_data(self._addr, REG_NUM_OUTPUTS)

    def read_descriptor_epoch(self) -> int:
        """Read the descriptor-set generation. 0 means no descriptors
        are readable: no driver loaded, or firmware without the
        descriptor window (the register then always reads 0)."""
        return self._bus.read_byte_data(self._addr, REG_DESCRIPTOR_EPOCH)

    def read_output(self, index: int) -> dict:
        """Read one output-field descriptor by index.

        Returns a dict with: idx, name, type, byte_order, semantic,
        count, scale, offset, unit — the shape the CLI printer and
        descriptor.parse_sample consume.
        """
        self._bus.write_byte_data(self._addr, REG_OUTPUT_SELECT, index)
        self._await_selector(REG_OUTPUT_SELECT, index)

        name_len = self._bus.read_byte_data(self._addr, REG_SEL_NAME_LEN)
        name = ""
        if name_len > 0:
            raw = self._bus.read_i2c_block_data(
                self._addr, REG_SEL_NAME, min(name_len, 16))
            name = bytes(raw).decode('ascii', errors='replace')

        ftype = self._bus.read_byte_data(self._addr, REG_SEL_TYPE)

        scale_raw = self._bus.read_i2c_block_data(
            self._addr, REG_SEL_OUTPUT_SCALE, 4)
        scale = struct.unpack_from('<f', bytes(scale_raw))[0]

        offset_raw = self._bus.read_i2c_block_data(
            self._addr, REG_SEL_OUTPUT_OFFSET, 4)
        offset = struct.unpack_from('<f', bytes(offset_raw))[0]

        byte_order = self._bus.read_byte_data(
            self._addr, REG_SEL_OUTPUT_BYTE_ORDER)

        unit_len = self._bus.read_byte_data(self._addr, REG_SEL_UNIT_LEN)
        unit = ""
        if unit_len > 0:
            raw = self._bus.read_i2c_block_data(
                self._addr, REG_SEL_UNIT, min(unit_len, 8))
            unit = bytes(raw).decode('ascii', errors='replace')

        semantic = self._bus.read_byte_data(
            self._addr, REG_SEL_OUTPUT_SEMANTIC)
        count_raw = self._bus.read_i2c_block_data(
            self._addr, REG_SEL_OUTPUT_COUNT, 2)
        count = struct.unpack_from('<H', bytes(count_raw))[0]
        byte_off = self._bus.read_byte_data(self._addr, REG_SEL_OUTPUT_AT)

        return {
            'idx': index,
            'name': name,
            'type': FIELD_TYPE_NAMES.get(ftype, f'type{ftype}'),
            'byte_order': 'big' if byte_order == 0 else 'little',
            'semantic': semantic,
            'byte_off': byte_off,
            'count': count,
            'scale': scale,
            'offset': offset,
            'unit': unit,
        }

    def read_outputs(self) -> Optional[list]:
        """Read the full output-descriptor set, coherently.

        Descriptor reads race the device's autonomous driver changes
        (probe-failed slot advance, re-upload), so the whole
        enumeration is validated with DESCRIPTOR_EPOCH: read it before
        and after; a mismatch means the set may mix two drivers and
        the read restarts. Returns [] when the device genuinely serves
        none (epoch 0: no driver, or firmware without the window), and
        None when the device stayed unstable past the retry budget —
        the set is transiently unreadable and the caller should keep
        its previous knowledge and retry later.
        """
        for _ in range(3):
            epoch = self.read_descriptor_epoch()
            if epoch == 0:
                return []
            outs = [self.read_output(i)
                    for i in range(self.read_num_outputs())]
            if self.read_descriptor_epoch() == epoch:
                return outs
        return None

    def write_param(self, index: int, value: int):
        """Write a new parameter value — triggers patch + VM reload.

        Args:
            index: Parameter index (0..num_params-1).
            value: New value (must be in the parameter's valid set).
        """
        self._bus.write_byte_data(self._addr, REG_PARAM_SELECT, index)
        val_bytes = struct.pack('<I', value)
        self._bus.write_i2c_block_data(
            self._addr, REG_PARAM_SET_VALUE, list(val_bytes))
        log.info("Set param[%d] = %d", index, value)

    # ── Name-based parameter access ────────────────────────
    # get_param() is inherited from NxsClient (searches capabilities).

    def set_param(self, name: str, value: int):
        """Set a parameter by name. Raises KeyError if not found."""
        caps = self.read_capabilities()
        for i, p in enumerate(caps):
            if p['name'] == name:
                self.write_param(i, value)
                return
        raise KeyError(f"No parameter named '{name}'")

    # ── Driver store (persistent flash) ──────────────────

    def read_store_count(self) -> int:
        """Number of populated slots in the persistent driver store."""
        return self._bus.read_byte_data(self._addr, REG_STORE_COUNT)

    def read_active_slot(self) -> int:
        """Current active slot (0xFF = transient RAM-only driver)."""
        return self._bus.read_byte_data(self._addr, REG_ACTIVE_SLOT)

    def read_runner_state(self) -> int:
        """Runner state enum (0=NO_DRIVER, 1=LOADING, ..., see RUNNER_STATES)."""
        return self._bus.read_byte_data(self._addr, REG_RUNNER_STATE)

    def read_probe_retries(self) -> int:
        """Current probe retry counter."""
        return self._bus.read_byte_data(self._addr, REG_PROBE_RETRIES)

    def _await_store_result(self, reasons=None):
        """Block until the store command resolves on the device; raise on
        failure. `reasons` overrides the errno→message map for commands whose
        errnos carry a different meaning (identity commit vs driver store)."""
        deadline = time.monotonic() + self.STORE_CMD_TIMEOUT_S
        while time.monotonic() < deadline:
            try:
                err = self._bus.read_byte_data(self._addr, REG_CMD_ERROR)
            except OSError:
                time.sleep(0.01)
                continue
            if err != CMD_ERR_PENDING:
                if err != 0:
                    raise DeviceRefused(err, err_reason(err, reasons))
                return
            time.sleep(0.01)
        raise RuntimeError("device did not report a result (timeout)")

    def save_slot(self, slot: int):
        """Save the currently-loaded RAM driver image into store slot `slot`.

        The host must have called `upload_image()` before this — CMD_SAVE
        commits the RAM program buffer to NVS. Raises RuntimeError if the
        device rejects the save (no driver loaded, duplicate, store full).
        """
        self._bus.write_byte_data(self._addr, REG_STORE_SELECT, slot)
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_SAVE)
        self._await_store_result()
        log.info("Saved RAM driver to store slot %d", slot)

    def delete_slot(self, slot: int):
        """Remove the driver at store slot `slot`. Raises on failure."""
        self._bus.write_byte_data(self._addr, REG_STORE_SELECT, slot)
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_DELETE_SLOT)
        self._await_store_result()
        log.info("Deleted store slot %d", slot)

    def clear_store(self):
        """Wipe all drivers from the persistent store. Raises on failure."""
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_CLEAR_STORE)
        self._await_store_result()
        log.info("Cleared driver store")

    def cycle(self):
        """Force the runner to advance to the next populated slot."""
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_CYCLE)
        log.info("Cycled to next driver slot")

    def identify(self):
        """Strobe the status LED (~10 s) to physically locate the unit."""
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_IDENTIFY)
        log.info("Identify: status LED strobing")

    def recover(self) -> int:
        """Arm MCUboot serial recovery and cold-reset into it.

        The device holds in the bootloader until an mcumgr upload
        completes — there is no timed window to catch. The reset drops
        the bus mid-transaction, so the write itself is the only
        acknowledgement; an OSError here means the command never
        landed, not that recovery failed.
        """
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_ENTER_RECOVERY)
        log.info("Rebooting into MCUboot serial recovery")

        return 0

    # ── NxsClient streaming primitives ──────────────────

    def _descriptor_token(self) -> int:
        """DESCRIPTOR_EPOCH — bumps on every driver (re)load, so the SDK
        re-fetches descriptors mid-stream when the driver changes."""
        return self.read_descriptor_epoch()

    def set_output_rate(self, hz):
        """Set the host poll rate (Hz). The register window always holds
        the latest sample, so a lower rate decimates over received
        samples. No cap. Takes effect at the next start_stream()."""
        super().set_output_rate(hz)
        self._poll_hz = hz

    def _resolve_poll_hz(self) -> float:
        """The effective poll rate: explicit `poll_hz`, else the driver's
        advertised sample_rate, else DEFAULT_POLL_HZ."""
        if self._poll_hz:
            return float(self._poll_hz)
        # GNSS-class drivers declare their cadence as `rate`, IMU-class
        # as `sample_rate`; try both so a slow part is not polled at the
        # 100 Hz default.
        rate = 0.0
        for name in ("sample_rate", "rate"):
            try:
                p = self.get_param(name)
                # 0 → keep looking: a poll cadence needs a sane value
                # (the inverse of cli._every_for_hz, which errors on 0).
                rate = float(p.get("current") or p.get("default") or 0)
            except (KeyError, OSError):
                rate = 0.0
            if rate > 0:
                break
        return rate if rate > 0 else DEFAULT_POLL_HZ

    def _arm_stream(self, every_nth: int):
        """I2C has no device-side egress to arm — samples live in the
        register window. Sync the poll cursor and fix the poll interval
        for this stream. `every_nth` is a serial-egress concept and does
        not apply to host polling (use poll_hz / --hz instead)."""
        # Arming right after vm_run can race the image load, whose
        # boundary clears the runtime window: a sample size of 0 here is
        # "no sample yet", not the driver's shape. Lazy init — _next_raw
        # refreshes a zero length when the first sample actually lands.
        self._record_data_len = self.read_sample_size()
        # Baseline the cursor from the record's own seq — the field
        # _next_raw compares. SAMPLE_COUNT clears at a load boundary
        # while the record keeps the last driver's seq; a register-read
        # baseline would emit that stale record as a fresh sample.
        self._last_count = self._read_record(0)[0]
        # The record is a single-slot latch whose appearance jitters by
        # about a third of the sample period, so flank prediction cannot
        # win: the rate-derived default polls at 5x the advertised rate,
        # the oversampling regime that captures every sample. An explicit
        # set_output_rate() stays the caller's exact cadence.
        base_hz = self._resolve_poll_hz()
        self._sample_period = 1.0 / base_hz
        self._poll_interval = (1.0 / float(self._poll_hz)) if self._poll_hz \
            else self._sample_period / 5.0
        self._next_due = 0.0  # first poll fires immediately

    def _disarm_stream(self):
        """Nothing to disarm for register polling."""

    def _next_raw(self, timeout: float):
        """Return (seq, raw, timestamp_us) when the record's sequence
        advances, else None by `timeout`. One latched record read per
        poll tick — seq, acquisition timestamp, data, and a sync
        observation in a single pass. Bus reads are gated to the poll
        interval — no matter how fast the caller spins (iter_samples or
        a host poll loop), the bus is hit at most once per interval,
        bounding occupancy to the chosen rate."""
        deadline = time.monotonic() + timeout
        while True:
            now = time.monotonic()
            if now >= self._next_due:
                self._next_due = now + self._poll_interval
                seq, timestamp_us, raw = self._read_record(
                    self._record_data_len)
                if seq != self._last_count:
                    if seq < self._last_count or self._record_data_len == 0:
                        # Restart (or 2^16 wrap): the driver may have
                        # changed shape — refresh the length, re-read. A
                        # zero length is an arm that raced the load
                        # window; the first live sample heals it here.
                        self._record_data_len = self.read_sample_size()
                        seq, timestamp_us, raw = self._read_record(
                            self._record_data_len)
                    self._last_count = seq
                    return seq, raw, timestamp_us
            if now >= deadline:
                return None
            time.sleep(max(0.0, min(self._next_due, deadline) - time.monotonic()))

    def close(self):
        """Release the smbus handle (no-op for the injected test seam)."""
        closer = getattr(self._bus, "close", None)
        if closer is not None:
            closer()

    # ── Firmware DFU push ─────────────────────────────────

    # Each chunk lands within a queue-drain tick once the comm thread picks
    # it up. The begin-erase (and each page program) stalls flash code-fetch,
    # freezing the MCU — the slave NACKs for the duration, so every wait loop
    # tolerates OSError and keeps polling.
    DFU_BEGIN_TIMEOUT_S = DFU_ERASE_TIMEOUT_S
    DFU_FINISH_POLL_S = 1.0
    DFU_FINISH_RETRIES = 3
    # Shared contract: the device's transfer-session stale window is twice
    # this ack timeout.
    DFU_ACK_TIMEOUT_S = NxsRegisters.DFU_ACK_TIMEOUT_MS / 1000.0
    DFU_WRITE_RETRIES = 4
    STORE_CMD_TIMEOUT_S = _STORE_CMD_TIMEOUT_S

    def push_image(self, bin_path: str, chunk_size: int = UPLOAD_CHUNK_SIZE,
                   progress_cb=None) -> int:
        """Drive the app-resident DFU core through one full upload.

        Sequence: select the DFU consumer (XFER_TYPE), CMD_DFU_BEGIN,
        poll XFER_PHASE to READY, stream chunks to PROGRAM_DATA paced
        on the XFER_ACK accepted-chunk counter, CMD_DFU_FINISH. The
        board then reboots into the staged image and self-confirms on
        a healthy main loop — no host confirm over I2C.

        Returns the number of bytes uploaded. Raises RuntimeError on a
        firmware-rejected write or a stalled counter, TimeoutError if
        the begin-erase never completes.
        """
        with open(bin_path, 'rb') as f:
            data = f.read()
        total = len(data)
        if total == 0:
            raise RuntimeError("image is empty")
        if chunk_size > UPLOAD_CHUNK_SIZE:
            raise ValueError(
                f"chunk_size {chunk_size} exceeds the "
                f"{UPLOAD_CHUNK_SIZE}-byte PROGRAM_DATA window")

        self._claim_xfer_mode(XFER_TYPE_DFU_IMAGE)
        # BEGIN and its verdict sit OUTSIDE the abort-unwind: a refusal here
        # (EBUSY: another host's live push) is not our session, and aborting
        # would destroy the holder's progress with the very command the
        # device's EBUSY guard exists to prevent. Only a failure after our
        # session provably opened (READY observed) may release it.
        self._bus.write_byte_data(self._addr, REG_CMD, CMD_DFU_BEGIN)
        self._dfu_wait_ready()
        # Say goodbye properly on any exit: XFER_ABORT closes OUR session
        # and hands the mux back. `finally` covers the operator's Ctrl-C
        # during a long push — KeyboardInterrupt is not an Exception. A host
        # that dies before this lands is covered by the stale window.
        streamed = False
        try:
            offset = 0
            chunk_idx = 0
            while offset < total:
                n = min(chunk_size, total - offset)
                self._dfu_send_chunk(chunk_idx, data[offset:offset + n])
                offset += n
                chunk_idx += 1
                if progress_cb is not None:
                    progress_cb(offset, total)
            streamed = True
        finally:
            if not streamed:
                self._abort_transfer()

        self._dfu_finish(total)
        return total

    def _dfu_finish(self, total: int):
        """Close the DFU session and confirm the board actually rebooted.

        Success requires positive evidence — the device latches FINISHING
        before its reboot delay, and the reboot then kills the bus. A
        finish silently dropped at a full command queue leaves the bus
        alive with the phase unchanged; that is retried, not reported as
        success (the old code's timeout-into-success bug). A latched ERROR
        phase is a failed finish (old firmware still runs): release the
        session and raise its reason.
        """
        for _ in range(self.DFU_FINISH_RETRIES):
            try:
                self._bus.write_byte_data(self._addr, REG_CMD, CMD_DFU_FINISH)
            except OSError:
                # The finish write itself NACKed: nothing was delivered
                # (a reboot only follows a processed finish), so resend.
                time.sleep(0.05)
                continue
            deadline = time.monotonic() + self.DFU_FINISH_POLL_S
            while time.monotonic() < deadline:
                try:
                    phase = self._bus.read_byte_data(self._addr, REG_XFER_PHASE)
                except OSError:
                    # Bus dead after a delivered finish: the reboot signature.
                    log.info("Pushed %d bytes; board resetting into the "
                             "new image", total)
                    return
                if phase == DFU_PHASE_ERROR:
                    err = self._bus.read_byte_data(self._addr, REG_CMD_ERROR)
                    self._abort_transfer()
                    raise DeviceRefused(err, "dfu finish failed (device "
                                        "sampling halted; session released): "
                                        + err_reason(err, DFU_XFER_REASONS))
                if phase == DFU_PHASE_FINISHING:
                    log.info("Pushed %d bytes; board resetting into the "
                             "new image", total)
                    return
                time.sleep(0.05)
            # The window elapsed with the bus alive and no terminal phase:
            # the finish was dropped before dispatch — resend.
        self._abort_transfer()
        raise RuntimeError(
            "dfu finish: no reboot after retries — the finish never "
            f"applied and the old image still runs ({self._xfer_state()})")

    def _dfu_wait_ready(self, timeout_s: float = None):
        """Wait out DFU_BEGIN by polling its own CMD_ERROR edge, then READY.

        The begin arms `CMD_ERR_PENDING` and resolves it with the verdict —
        0, or a refusal (EBUSY: someone else's session; EPROTO: wrong mode)
        — and that edge is the only thing a refused host may trust:
        `XFER_PHASE` shows the *holder's* progress and would lure it into
        streaming over a live push. The MCU NACKs through the bulk erase,
        so OSError is the expected texture of this wait.
        """
        if timeout_s is None:
            timeout_s = self.DFU_BEGIN_TIMEOUT_S
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                err = self._bus.read_byte_data(self._addr, REG_CMD_ERROR)
            except OSError:
                time.sleep(0.05)  # MCU frozen mid-erase; slave NACKs
                continue
            if err == CMD_ERR_PENDING:
                time.sleep(0.02)
                continue
            if err != 0:
                raise DeviceRefused(err, "dfu begin refused: "
                                    + err_reason(err, DFU_XFER_REASONS))
            try:
                phase = self._bus.read_byte_data(self._addr, REG_XFER_PHASE)
            except OSError:
                time.sleep(0.05)  # same erase-freeze texture as the CMD read
                continue
            if phase == DFU_PHASE_READY:
                return
            time.sleep(0.02)
        raise TimeoutError("dfu begin: no verdict within "
                           f"{timeout_s:.1f} s — {self._xfer_state()}")

    def _xfer_state(self) -> str:
        """Best-effort transfer-state triple for timeout diagnostics —
        phase=IDLE with a foreign XFER_TYPE names a mux hijacker in one
        line instead of an evening."""
        try:
            phase = self._bus.read_byte_data(self._addr, REG_XFER_PHASE)
            mode = self._bus.read_byte_data(self._addr, REG_XFER_TYPE)
            err = self._bus.read_byte_data(self._addr, REG_CMD_ERROR)
        except OSError:
            return "device not answering"
        return (f"XFER_PHASE={_PHASE_NAMES.get(phase, phase)} "
                f"XFER_TYPE={_MODE_NAMES.get(mode, mode)} "
                f"CMD_ERROR={err}")

    def _abort_transfer(self):
        """Send `Cmd::XFER_ABORT`, retrying through the erase's NACK window
        (~2.4 s of MCU deafness); both outcomes — released (0) and nothing
        to release (ENOENT) — are fine for an unwind path."""
        deadline = time.monotonic() + self.DFU_BEGIN_TIMEOUT_S
        while time.monotonic() < deadline:
            try:
                self._bus.write_byte_data(self._addr, REG_CMD,
                                          CMD_XFER_ABORT)
                return
            except OSError:
                time.sleep(0.05)

    def _dfu_ack_wait(self, expected: int, timeout_s: float = None) -> bool:
        """Poll XFER_ACK until it reads `expected` (mod 256).

        Returns False on timeout (chunk presumed lost — the caller
        re-reads the counter and resends); raises on the ERROR phase.
        """
        if timeout_s is None:
            timeout_s = self.DFU_ACK_TIMEOUT_S
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                if self._bus.read_byte_data(
                        self._addr, REG_XFER_ACK) == expected:
                    return True
                phase = self._bus.read_byte_data(self._addr, REG_XFER_PHASE)
            except OSError:
                time.sleep(0.005)
                continue
            if phase == DFU_PHASE_ERROR:
                err = self._read_cmd_error()
                raise DeviceRefused(err, "dfu write rejected: "
                                    + err_reason(err, DFU_ERR_REASON))
            time.sleep(0.002)
        return False

    def _dfu_send_chunk(self, chunk_idx: int, chunk: bytes):
        """Land chunk `chunk_idx` (0-based), resending on loss.

        The I2C target queues PROGRAM_DATA writes to the comm thread
        and drops them when the queue is full, so a chunk can vanish
        without an error. Recovery contract: re-read XFER_ACK before
        every (re)send and only send while it still reads this chunk's
        index — a lost chunk leaves the counter parked at the stall,
        an already-landed one moves it past. With a single chunk in
        flight the counter is only ever `idx` or `idx + 1`, so the
        mod-256 wrap is unambiguous.
        """
        before = chunk_idx & 0xFF
        after = (chunk_idx + 1) & 0xFF
        for _ in range(self.DFU_WRITE_RETRIES):
            try:
                ack = self._bus.read_byte_data(self._addr, REG_XFER_ACK)
                phase = self._bus.read_byte_data(self._addr, REG_XFER_PHASE)
            except OSError:
                time.sleep(0.01)
                continue
            if phase == DFU_PHASE_ERROR:
                err = self._read_cmd_error()
                raise DeviceRefused(err, f"dfu write for chunk {chunk_idx} "
                                    "rejected: "
                                    + err_reason(err, DFU_ERR_REASON))
            if ack == after:
                return  # landed; the ack poll just missed it
            if ack != before:
                raise RuntimeError(
                    f"dfu ack desync: firmware acked {ack}, host at "
                    f"chunk {chunk_idx}")
            try:
                self._bus.write_i2c_block_data(
                    self._addr, REG_PROGRAM_DATA, list(chunk))
            except OSError:
                pass  # NACKed mid-freeze; the ack re-read decides the resend
            if self._dfu_ack_wait(after):
                return
        raise RuntimeError(
            f"dfu stalled at chunk {chunk_idx}: ack stuck after "
            f"{self.DFU_WRITE_RETRIES} attempts")

    def _read_cmd_error(self) -> int:
        """Best-effort CMD_ERROR read for exception messages."""
        try:
            return self._bus.read_byte_data(self._addr, REG_CMD_ERROR)
        except OSError:
            return -1
