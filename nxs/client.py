"""Transport-independent NXS client SDK. `NxsClient` is the contract every
transport implements; transports supply the primitives, this layer the shared
procedures: sample decoding, descriptor re-sync, stream dispatch."""
import struct
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterator, List, Optional, Tuple

from nxs._generated_constants import Calibration as CalConstants
from nxs._generated_constants import FieldSemantics
from nxs._generated_constants import RunnerStates
from nxs.click_facts import is_decodable, parse_sample

from nxs.device_errors import (
    CALIB_ERR_REASON, CAM_ABORT_ERR_REASON, CAM_RUN_ERR_REASON,
    CAM_VERDICT_REASON, COMMISSION_ERR_REASON, DFU_ERR_REASON, DeviceRefused,
    ERRNO_EAGAIN, ERRNO_EBADF, ERRNO_EBUSY, ERRNO_ECANCELED, ERRNO_EEXIST, ERRNO_EINVAL,
    ERRNO_ENODATA, ERRNO_ENOENT, ERRNO_ENOTSUP, LOAD_ERR_REASON, PULL_ERR_REASON,
    SESSION_LIVE_REASON, STORE_ERR_REASON, XFER_ERR_REASON, err_reason,
    exc_detail, import_failure_detail, op_error_name,)
from nxs.capabilities import (
    ADDR_UNSET, CAL_VECTORS, CAM_RUN_TERMINAL, CAN_BITRATE_PROFILES,
    CAN_TERM_OFF, CAN_TERM_ON, CAN_TERM_UNSET, COMMISSION_TOPICS,
    CalibrationRecord, CamRunState, NODE_ID_ANONYMOUS, NODE_ID_MAX,
    ROTATION_NAMES, SCALAR_TOPIC_SPAN, SUBJECT_ID_MAX, SupportsBitTiming,
    SupportsCalibration, SupportsCameraRun, SupportsCanTermination,
    SupportsCommissioning, SupportsEgressDecimation, SupportsFaultCounters,
    SupportsIdentify, SupportsRecovery, SupportsSampleFifo, SupportsSlotPeek,
    SupportsTimeSync, _CAL_STRUCT, await_cam_run, cam_run_state_name,
    rotation_code, rotation_name, validate_can_bitrate, validate_can_term,
    validate_commission, validate_sample_fifo_depth,)
from nxs.firmware import (
    DFU_REBOOT_TIMEOUT_S, FirmwareTooOld, MIN_FIRMWARE, MIN_FIRMWARE_ASSET,
    SUPPORTED_PROTO_VERSION, await_reachable,
    contract_mismatch, firmware_too_old, parse_fw_identity, push_and_verify,
    read_firmware_version, read_identity, require_firmware, update_verdict,)
from nxs.time_sync import (
    DRIVER_UP_POLL_S, DRIVER_UP_TIMEOUT_S, PUSH_ECHO_POLL_S,
    PUSH_ECHO_TIMEOUT_S, PUSH_INTERVAL_S, SYNC_LOST_PUSHES,
    TimeSyncEstimator, await_driver_up, estimate_and_push,)

#: The names `nxs.client` has always answered to, wherever they now live.
__all__ = [
    "ACTIVE_SLOT", "ADDR_UNSET", "CALIB_ERR_REASON", "CAL_READ_ATTEMPTS",
    "CAL_REFRESH_S", "CAL_VECTORS", "CAM_ABORT_ERR_REASON",
    "CAM_RUN_ERR_REASON", "CAM_RUN_TERMINAL", "CAM_VERDICT_REASON",
    "CAN_BITRATE_PROFILES", "CAN_TERM_OFF", "CAN_TERM_ON", "CAN_TERM_UNSET",
    "COMMISSION_ERR_REASON", "COMMISSION_TOPICS", "CalibrationRecord",
    "CamRunState", "DFU_ERASE_TIMEOUT_S", "DFU_ERR_REASON",
    "DFU_REBOOT_TIMEOUT_S", "DRIVER_UP_POLL_S", "DRIVER_UP_TIMEOUT_S",
    "DeviceRefused", "ERRNO_EAGAIN", "ERRNO_EBADF", "ERRNO_EBUSY", "ERRNO_ECANCELED",
    "ERRNO_EEXIST", "ERRNO_EINVAL", "ERRNO_ENODATA", "ERRNO_ENOENT", "ERRNO_ENOTSUP",
    "FirmwareTooOld", "LOAD_ERR_REASON", "MIN_FIRMWARE",
    "MIN_FIRMWARE_ASSET", "NODE_ID_ANONYMOUS", "NODE_ID_MAX", "NxsClient",
    "PULL_ERR_REASON", "PUSH_ECHO_POLL_S", "PUSH_ECHO_TIMEOUT_S",
    "PUSH_INTERVAL_S", "RESET_SETTLE_TIMEOUT_S", "ROTATION_NAMES",
    "SCALAR_TOPIC_SPAN", "SELECTOR_INACTIVE", "SESSION_LIVE_REASON",
    "STORE_CMD_TIMEOUT_S", "STORE_ERR_REASON", "SUBJECT_BUCKETS",
    "SUBJECT_ID_MAX", "SUPPORTED_PROTO_VERSION", "SYNC_LOST_PUSHES",
    "Sample", "SupportsBitTiming", "SupportsCalibration",
    "SupportsCameraRun", "SupportsCanTermination", "SupportsCommissioning",
    "SupportsEgressDecimation", "SupportsFaultCounters", "SupportsIdentify",
    "SupportsRecovery", "SupportsSampleFifo", "SupportsSlotPeek",
    "SupportsTimeSync", "TimeSyncEstimator", "XFER_ERR_REASON",
    "_CAL_STRUCT", "active_driver_tag", "await_cam_run",
    "await_driver_unloaded", "await_driver_up", "await_reachable",
    "cam_run_state_name", "contract_mismatch", "err_reason",
    "estimate_and_push", "exc_detail", "firmware_too_old",
    "import_failure_detail", "op_error_name", "parse_fw_identity",
    "peek_slot", "push_and_verify", "read_firmware_version", "read_identity",
    "require_firmware", "rotation_code", "rotation_name",
    "runner_state_name", "update_verdict", "validate_can_bitrate",
    "validate_can_term", "validate_commission", "validate_sample_fifo_depth",
]


# RESET is published, not applied inline: the runner clears the VM and the
# staged image on its next dispatch, within two 100 ms heartbeats.
RESET_SETTLE_TIMEOUT_S = 2.0

#: The advice behind a serial link that vanished under the tool.
LINK_DROP_ADVICE = ("A J-Link VCOM can wedge on open or drop under load; prefer a "
                    "dedicated USB-UART adapter (CP210x / FT232) for the host link.")


def await_driver_unloaded(driver_gone, timeout_s=RESET_SETTLE_TIMEOUT_S,
                          poll_s=0.02):
    """Block until `driver_gone` reports the device holds no personality image,
    which is what `store save` would persist; it returns True once the image
    is gone, False while present, None when the device did not answer (never
    counted as gone). Raises TimeoutError after `timeout_s`."""
    deadline = time.monotonic() + timeout_s
    last_err = None
    while time.monotonic() < deadline:
        try:
            if driver_gone() is True:
                return
        except OSError as e:
            last_err = e            # NACK while the runner switches state
        time.sleep(poll_s)
    raise TimeoutError("device did not report the personality unloaded within "
                       f"{timeout_s:g}s" + (f" (last bus error: {last_err})" if last_err else ""))

# Wire sentinel for an inactive selector, shared by the SDK and its transports.
SELECTOR_INACTIVE = 0xFF

# Subject token -> SubjectBucket value for DECIMATION_SELECT; the same tokens
# name the Cyphal decimation.<subject> registers. NONE (0) is not addressable.
SUBJECT_BUCKETS = {
    name.lower(): value
    for value, name in FieldSemantics.SubjectBucket._NAMES.items()
    if name != 'NONE'
}

# Slot sentinel meaning "the active personality" rather than a stored slot; the
# firmware uses the same 0xFF for a transient (RAM-only) personality.
ACTIVE_SLOT = 0xFF























# DFU `begin` erases the staging slot before answering, about 2.5 s; 2x margin.
DFU_ERASE_TIMEOUT_S = 5.0

# Store commands (save, delete, clear, identity commit) program flash on the
# same frozen-MCU terms, less of it.
STORE_CMD_TIMEOUT_S = 2.0

# Retry budget for the epoch-bracketed calibration record read; each attempt
# is a full multi-register pass.
CAL_READ_ATTEMPTS = 5

CAL_REFRESH_S = 2.0
"""Streaming-side probe cadence of the calibration epoch, so a running stream
reloads its decode record when the bank moves underneath it."""




























def peek_slot(transport, slot: int):
    """`read_slot_info` that tolerates a held session: `(info, False)`, or
    `(None, True)` when the device refused the peek with EBUSY. Any other
    refusal propagates; only the I2C transport refuses this way."""
    try:
        return transport.read_slot_info(slot), False
    except DeviceRefused as e:
        if e.code != ERRNO_EBUSY:
            raise
        return None, True


@dataclass
class Sample:
    """One decoded sample. `values` maps field name to physical value, and is
    empty when the personality exposes no descriptors or declares a type this tool
    cannot decode; `raw` always holds the bytes."""
    count: int
    raw: bytes
    values: Dict[str, object] = field(default_factory=dict)
    timestamp_us: Optional[int] = None

    def __getitem__(self, name: str):
        return self.values[name]

    def get(self, name: str, default=None):
        return self.values.get(name, default)






































assert _CAL_STRUCT.size == CalConstants.RECORD_SIZE




def runner_state_name(code: int) -> str:
    """The runner state's name, or the bare code for one this SDK does not know."""
    return RunnerStates.RunnerState._NAMES.get(code, str(code))























class NxsClient(ABC):
    """The transport-independent NXS control + streaming contract."""

    def __init__(self):
        self._streaming = False
        self._spinning = False
        self._sample_handlers: List[Callable[[Sample], None]] = []
        self._poll_fields: Optional[List[dict]] = None
        self._poll_token: int = 0
        self._time_sync = TimeSyncEstimator()
        self._poll_cal: Optional[CalibrationRecord] = None
        self._poll_name: Optional[str] = None

    # ── Context management ────────────────────────────────
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        """Release transport resources. Override where needed; a no-op
        default suits poll/in-memory transports."""

    def link_dropped(self) -> bool:
        """True if a background thread saw a mid-session physical link loss (a
        USB unplug or a wedged serial device). Default False."""
        return False

    def probe_failure_detail(self) -> Optional[str]:
        """Why the last `probe()` found nothing, when the transport knows; None
        when it has nothing to add."""
        return None

    # ── Time sync ─────────────────────────────────────────
    def get_time_sync(self) -> TimeSyncEstimator:
        """The transport's device-to-host clock estimator. I2C feeds it from
        every sample-record poll; Cyphal transports via `time_sync_ping()`."""
        return self._time_sync

    def adopt_time_sync(self, sync: TimeSyncEstimator):
        """Continue an existing estimator on this client, keeping its
        observation history across a transport reopen."""
        self._time_sync = sync

    def read_device_time_us(self) -> Optional[int]:
        """The device's current µs clock, or None where the transport
        serves no time surface (transports with one override)."""
        return None

    def time_sync_ping(self) -> bool:
        """One two-way sync exchange: bracket a device-clock read with host
        stamps and feed the estimator. False when the transport has no time surface."""
        t0 = time.monotonic_ns()
        device_us = self.read_device_time_us()
        t1 = time.monotonic_ns()
        if device_us is None:
            return False
        self._time_sync.observe(t0, t1, device_us)
        return True

    def disconnect_message(self) -> str:
        """Human-readable reason for a dropped link; only meaningful when
        `link_dropped()` is True."""
        return "link dropped mid-session"

    # ── Liveness ──────────────────────────────────────────
    @abstractmethod
    def probe(self) -> bool:
        """True if the device answers on this transport."""

    def interface_version(self) -> Optional[int]:
        """Register-map contract version, or None where the transport
        has no equivalent. Default None; I2C overrides."""
        return None

    def read_serial(self) -> Optional[bytes]:
        """The board's 12-byte chip UID96, or None where the transport
        has no equivalent. Default None; I2C and Cyphal override."""
        return None

    def read_fw_version(self) -> Optional[str]:
        """The running firmware version ("MAJOR.MINOR") where the transport
        serves it, else None."""
        return None

    def read_fw_confirmed(self) -> Optional[bool]:
        """Whether the running image is confirmed (True) or still in MCUboot's
        TEST state (False); None where the transport serves no such state."""
        return None

    # ── Personality lifecycle ──────────────────────────────────
    @abstractmethod
    def upload_image(self, image: bytes):
        """Stage and load an NXS image into the VM."""

    @abstractmethod
    def vm_run(self):
        """Start the VM (probe → configure → measure)."""

    @abstractmethod
    def vm_stop(self):
        """Halt the VM; the personality stays loaded."""

    @abstractmethod
    def vm_reset(self):
        """Unload the personality entirely."""

    @abstractmethod
    def confirm_fw(self) -> None:
        """Mark the running image good so MCUboot keeps it across the next reset.
        Idempotent; raises DeviceRefused with the errno of a failed trailer write."""

    @abstractmethod
    def push_image(self, bin_path: str, chunk_size: int = 32,
                   progress_cb=None) -> int:
        """Push a signed firmware image (DFU). Returns bytes sent."""

    # ── Parameters ────────────────────────────────────────
    @abstractmethod
    def read_capabilities(self) -> List[dict]:
        """All declared parameter descriptors."""

    @abstractmethod
    def read_decimation(self, subject=None) -> int:
        """The device-wide output decimation factor, or `subject`'s
        per-SI-subject factor when given."""

    @abstractmethod
    def write_decimation(self, value: int, subject=None) -> None:
        """Set the device-wide factor (0 = off, 1 = every sample,
        N = every Nth), or `subject`'s when given."""

    @abstractmethod
    def set_param(self, name: str, value: int):
        """Set a declared parameter by name."""

    def get_param(self, name: str) -> dict:
        """One parameter descriptor by name (searches capabilities)."""
        for p in self.read_capabilities():
            if p["name"] == name:
                return p
        raise KeyError(name)

    # ── Personality store ──────────────────────────────────────
    @abstractmethod
    def save_slot(self, slot: int):
        """Persist the staged image to store slot `slot`."""

    @abstractmethod
    def delete_slot(self, slot: int):
        """Erase store slot `slot`."""

    @abstractmethod
    def clear_store(self):
        """Wipe every store slot."""

    @abstractmethod
    def cycle(self):
        """Advance to the next populated store slot."""

    # ── Status / metadata ─────────────────────────────────
    @abstractmethod
    def read_personality_name(self) -> str: ...
    @abstractmethod
    def read_sample_size(self) -> int: ...
    @abstractmethod
    def read_sample_count(self) -> int: ...
    @abstractmethod
    def read_status(self) -> int: ...
    @abstractmethod
    def read_vm_state(self) -> int: ...
    @abstractmethod
    def read_error_code(self) -> int: ...
    @abstractmethod
    def read_store_count(self) -> int: ...
    @abstractmethod
    def read_active_slot(self) -> int: ...
    @abstractmethod
    def read_runner_state(self) -> int: ...
    @abstractmethod
    def read_probe_retries(self) -> int: ...

    # ── Output descriptors ────────────────────────────────
    @abstractmethod
    def read_outputs(self) -> Optional[List[dict]]:
        """The device's output-field descriptor set. [] when the device serves
        none, where callers fall back to a local compile; None when the set is
        transiently unreadable, where callers keep what they know."""

    @abstractmethod
    def _descriptor_token(self) -> int:
        """Opaque generation of the current descriptor set; changes when the
        loaded personality changes. A transport without change detection returns a constant."""

    # ── Streaming primitives (transport-specific) ─────────
    @abstractmethod
    def _arm_stream(self, every_nth: int):
        """Begin sample delivery (push transport: arm egress; I2C: enable poll)."""

    @abstractmethod
    def _disarm_stream(self):
        """Stop sample delivery."""

    @abstractmethod
    def _next_raw(self, timeout: float) -> Optional[Tuple[int, bytes, Optional[int]]]:
        """Next (count, raw_bytes, timestamp_us), or None on timeout.
        `timestamp_us` is None where the transport has none."""

    # ── Streaming procedures (shared) ─────────────────────
    @property
    def lost_samples(self) -> Optional[int]:
        """Samples the device produced for this stream and the transport never
        delivered; None where the transport cannot tell."""
        return None

    def set_output_rate(self, hz):
        """Set a target output rate in Hz (positive, uncapped). The default
        validates then no-ops; the I2C poll transport overrides it to set its cadence."""
        if hz is None or hz <= 0:
            raise ValueError("output rate (Hz) must be positive")

    def start_stream(self, every_nth: int = 1):
        """Begin streaming with the given decimation. Idempotent: a no-op if
        already streaming (change the rate with stop_stream() then start_stream())."""
        if self._streaming:
            return
        self._arm_stream(every_nth)
        self._streaming = True

    def stop_stream(self):
        """Stop streaming. Idempotent: a no-op if not streaming."""
        if not self._streaming:
            return
        self._streaming = False
        self._disarm_stream()

    def iter_samples(self, timeout: float = 1.0,
                     calibrated: bool = True,
                     max_silence_s: Optional[float] = None) -> Iterator[Sample]:
        """Yield decoded `Sample`s until streaming is stopped. ``max_silence_s``
        returns after that long without a sample, and ``calibrated`` composes
        the served calibration record into the values."""
        if not self._streaming:
            self.start_stream()
        try:
            token, fields = self._load_fields()
            calibration, active_tag = (self._load_calibration()
                                       if calibrated else (None, 0))
            prev_seq = None
            cal_epoch = None
            next_cal_check = time.monotonic() + CAL_REFRESH_S
            last_sample = time.monotonic()
            while self._streaming:
                item = self._next_raw(timeout)
                if item is None:
                    if (max_silence_s is not None
                            and time.monotonic() - last_sample >= max_silence_s):
                        return
                    continue
                last_sample = time.monotonic()
                # The bank can move with no descriptor change, so the epoch is
                # probed on a slow clock; a refused probe waits for the next tick.
                if calibrated and time.monotonic() >= next_cal_check:
                    next_cal_check = time.monotonic() + CAL_REFRESH_S
                    try:
                        epoch = self.read_cal_epoch()
                    except Exception:
                        epoch = cal_epoch
                    if epoch is not None and epoch != cal_epoch:
                        # The first observation reloads too: the bank may have
                        # moved since the startup read.
                        cal_epoch = epoch
                        calibration, active_tag = self._load_calibration()
                # Descriptors change only on a VM restart, which resets seq, so
                # the RPC-backed token is re-checked on that backward jump.
                count = item[0]
                if token == -1 or (prev_seq is not None and count < prev_seq):
                    prev_token = token
                    token, fields = self._refreshed_fields(token, fields)
                    if calibrated and token != prev_token:
                        calibration, active_tag = self._load_calibration()
                prev_seq = count
                yield self._decode(item, fields, calibration, active_tag)
        finally:
            # Best-effort like spin()/host teardown: a disarm that raises
            # on a dead link must not supersede the loop's own exception.
            try:
                self.stop_stream()
            except Exception:
                pass

    def poll_sample(self, timeout: float = 0.0) -> Optional[Sample]:
        """Single-sample read for multiplexing several clients in one loop: a
        decoded `Sample`, or None if none arrived within `timeout`. Arms the
        stream on first call and tracks the personality like `iter_samples`."""
        if not self._streaming:
            self.start_stream()
        if self._poll_fields is None:
            self._poll_token, self._poll_fields = self._load_fields()
            self._poll_cal, self._poll_name = self._load_calibration()
            self._poll_cal_epoch = None
            self._poll_cal_check = time.monotonic() + CAL_REFRESH_S
        item = self._next_raw(timeout)
        if item is None:
            return None
        if time.monotonic() >= self._poll_cal_check:
            self._poll_cal_check = time.monotonic() + CAL_REFRESH_S
            try:
                epoch = self.read_cal_epoch()
            except Exception:
                epoch = self._poll_cal_epoch
            if epoch is not None and epoch != self._poll_cal_epoch:
                self._poll_cal_epoch = epoch
                self._poll_cal, self._poll_name = self._load_calibration()
        prev_token = self._poll_token
        self._poll_token, self._poll_fields = self._refreshed_fields(
                self._poll_token, self._poll_fields)
        if self._poll_token != prev_token:
            self._poll_cal, self._poll_name = self._load_calibration()
        return self._decode(item, self._poll_fields, self._poll_cal,
                            self._poll_name)

    def on_sample(self, handler: Callable[[Sample], None]):
        """Register a handler invoked per sample by `spin()`."""
        self._sample_handlers.append(handler)

    def spin(self, timeout: float = 1.0):
        """Block, dispatching each sample to `on_sample` handlers, until
        `stop_spin()` or KeyboardInterrupt."""
        self._spinning = True
        try:
            while self._spinning:
                sample = self.poll_sample(timeout)
                if sample is None:
                    continue
                for handler in self._sample_handlers:
                    handler(sample)
        except KeyboardInterrupt:
            pass
        finally:
            self._spinning = False
            # Disarm so a push transport stops emitting and its RX queue
            # stops growing once spin() returns; best-effort on teardown.
            try:
                self.stop_stream()
            except Exception:
                pass

    def stop_spin(self):
        """Ask a running `spin()` to return."""
        self._spinning = False

    # ── Decode helpers ────────────────────────────────────
    def _load_fields(self) -> Tuple[int, List[dict]]:
        token = self._descriptor_token()
        fields = self._fields_for(token)
        if fields is None:
            # Transiently unreadable at arm: start raw-only under a token
            # no real epoch can match, so the first sample refetches.
            return -1, []
        return token, fields

    def _refreshed_fields(self, token: int,
                          fields: List[dict]) -> Tuple[int, List[dict]]:
        """Descriptor refresh across a personality change, committing the new
        (token, fields) pair only when a complete set was read. An unresolved
        swap keeps the old token and collapses the fields to raw-only."""
        new_token = self._descriptor_token()
        if new_token == token:
            return token, fields
        new_fields = self._fields_for(new_token)
        if new_fields is None:
            return token, []
        return new_token, new_fields

    def _fields_for(self, _token: int) -> Optional[List[dict]]:
        fields = self.read_outputs()
        if fields is None:
            return None
        # An undecodable set (a field type newer than this tool) yields
        # raw-only samples rather than crashing the stream.
        return fields if is_decodable(fields) else []

    def _decode(self, item: Tuple[int, bytes, Optional[int]],
                fields: List[dict], calibration=None,
                active_tag: int = 0) -> Sample:
        count, raw, timestamp_us = item
        values = (parse_sample(raw, fields, calibration, active_tag)
                  if fields else {})
        return Sample(count=count, raw=raw, values=values,
                      timestamp_us=timestamp_us)

    def _load_calibration(self):
        """The served record and the running sensor's tag for the decode
        post-pass, or (None, 0) when no calibration surface is served or the read fails."""
        reader = getattr(self, "read_calibration", None)
        if reader is None:
            return None, 0
        try:
            record = reader()
            return record, active_driver_tag(self, record)
        except (RuntimeError, OSError, TimeoutError, ValueError,
                struct.error, DeviceRefused):
            return None, 0


def active_driver_tag(transport, record=None) -> int:
    """The running sensor's identity tag: personality name, bus, latched address.
    The bus is read, never inferred."""
    if record is not None and not any(record.personality_tags) and not record.encoder_tag:
        return 0
    from nxs.click_facts import driver_tag
    try:
        name = transport.read_personality_name() or ""
    except Exception:
        return 0
    if not name:
        return 0
    address = 0
    if isinstance(transport, SupportsSlotPeek):
        address = getattr(transport.read_slot_info(ACTIVE_SLOT), "i2c_addr", 0) or 0
    bus = 0
    for param in (transport.read_capabilities() or []):
        if param.get("name") == "bus":
            bus = int(param.get("current", 0) or 0)
            break

    return driver_tag(name, bus, address)


"""Default refresh cadence of the resident pushers, seconds."""

"""Pushes that may be lost before the pushed discipline expires."""


"""Budget for a personality to load, reset its sensor, and probe after a RUN."""

"""Register-poll spacing while waiting for the personality to come up."""






