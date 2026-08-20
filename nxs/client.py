"""
Transport-independent NXS client SDK.

`NxsClient` is the contract every transport implements — I2C register
poll and Cyphal (serial / CAN-FD). Program against it once and switch
wires by changing the constructor:

    from nxs import open_client

    client = open_client("i2c", bus="/dev/i2c-2", address=0x30)
    client.upload_image(image_bytes)
    client.vm_run()
    client.set_param("sample_rate", 250)
    for sample in client.iter_samples():     # or client.on_sample(cb); client.spin()
        print(sample.values["accel_x"])

The split mirrors the firmware: the host frontends (transports) supply
transport-specific *primitives* — register reads, frame round-trips —
and this base layer supplies the shared *procedures*: decoding a sample
from the device-served descriptors, re-syncing them when the driver
changes, and dispatching the stream as an iterator or to callbacks.

Control is synchronous request/reply on every transport. Samples may
arrive asynchronously (a push transport's background thread), but they
are consumed through the synchronous `iter_samples()` / `on_sample()`
surface — the threading lives in the transport, not in this contract.
"""
import struct
import threading
import time
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterator, List, Optional, Tuple

from nxs._generated_constants import Calibration as CalConstants
from nxs._generated_constants import FieldSemantics
from nxs._generated_constants import RunnerStates
from nxs.descriptor import IDENTITY_M, fnv1a32, is_decodable, parse_sample

# Mirrors RegisterMapFrontend.h::SELECTOR_INACTIVE; documented here so
# the SDK and its transports share one name for the wire sentinel.
SELECTOR_INACTIVE = 0xFF

# Slot sentinel meaning "the active driver" rather than a stored slot —
# the same 0xFF the firmware uses for a transient (RAM-only) driver.
ACTIVE_SLOT = 0xFF


class DeviceRefused(RuntimeError):
    """The device answered and said no — distinct from a dead link.

    Carries the device's positive errno (`code`, device libc numbering)
    so callers can branch on the reason — a duplicate store save
    (EEXIST, 17) is a benign no-op, a full store (ENOSPC, 28) is not.
    The message is the human-readable reason.
    """

    def __init__(self, code: int, reason: str):
        super().__init__(reason)
        self.code = code


# Device store-op errno → reason (device libc numbering, not the host's).
# Shared by every transport: the I2C path reads the errno from the
# CMD_ERROR register, the Cyphal path from `aliensense.nxs.cmd_error`.
STORE_ERR_REASON = {
    61: "no driver image loaded — upload a driver first",               # ENODATA
    17: "an identical driver image is already stored in another slot",  # EEXIST
    28: "the driver store is full",                                     # ENOSPC
    22: "invalid slot",                                                 # EINVAL
    2:  "no such slot",                                                 # ENOENT
    5:  "flash (NVS) write error",                                      # EIO
}

# STORE_PERSIST (identity commit) shares the error channel with the
# driver-store ops but its errnos mean different things — EINVAL is a
# rejected record, not a bad slot.
COMMISSION_ERR_REASON = {
    22: "identity record rejected (node-id or subject-id out of range)",  # EINVAL
    5:  "flash (NVS) write error",                                        # EIO
}

# Highest register-map contract version this SDK understands. A host-side
# capability, deliberately NOT sourced from the firmware's generated
# PROTO_VERSION_VALUE: bump it only when this code actually learns to speak
# a new contract version.
SUPPORTED_PROTO_VERSION = 1


def contract_mismatch(transport, *, unreadable_is_skew: bool = False) -> Optional[str]:
    """None when the device speaks this SDK's register-map contract, or when
    the transport serves no version at all (Cyphal has no such register).
    Otherwise a one-line description of the skew, for refusing mutation
    before it can misread the device.

    A transport that serves no version at all (Cyphal, or a duck-typed
    double) is always None. A read that FAILS is the caller's judgment:
    a single verb can let its own operation produce the better error, but
    anything about to mutate a device should pass `unreadable_is_skew=True`
    and refuse rather than converge blind against an unknown contract."""
    reader = getattr(transport, "interface_version", None)
    if reader is None:
        return None                 # serves no version, like Cyphal
    try:
        ver = reader()
    except NotImplementedError:
        return None
    except (OSError, RuntimeError) as e:
        if unreadable_is_skew:
            return f"could not read the device's register-map version ({e})"
        return None
    if ver is None or ver == SUPPORTED_PROTO_VERSION:
        return None
    return (f"device speaks register-map v{ver}; this nxs speaks "
            f"v{SUPPORTED_PROTO_VERSION} only")


# DFU `begin` bulk-erases the staging slot before it answers, freezing the
# MCU: measured 2.44 s for slot 1 on the STM32G491. 2x margin.
DFU_ERASE_TIMEOUT_S = 5.0

# Store commands (save, delete, clear, identity commit) program flash on the
# same frozen-MCU terms, less of it.
STORE_CMD_TIMEOUT_S = 2.0

# The silent-refusal detection code: a mode readback that does not echo the
# write means EBUSY — named here because it is raised at claim sites that
# never see a CMD_ERROR value (the device numbers, not the host's errno).
XFER_EBUSY = 16

# Retry budget for the epoch-bracketed calibration record read (both
# transports): each attempt is a full multi-page / multi-register pass, so
# a handful covers any realistic repaint storm.
CAL_READ_ATTEMPTS = 5

CAL_REFRESH_S = 2.0
"""Streaming-side probe cadence of the calibration epoch: a running
stream reloads its decode record when the bank moves underneath it (a
boot-auto solve, an on-device procedure, the other transport)."""

# Transfer-session refusals (RegisterMapFrontend's session rules). Keys are
# the DEVICE's errno numbers — Zephyr's newlib values, which match Linux for
# every entry here (EPROTO is 71 on both; macOS differs, which only matters
# to mock-bus unit tests — inject these numbers, not the host errno module's).
XFER_ERR_REASON = {
    16: "another transfer session is live (a concurrent upload or firmware "
        "push) — retry when it ends, or release it with XFER_ABORT",  # EBUSY
    71: "out-of-sequence transfer op: wrong XFER_TYPE for this operation, "
        "or a config stage trampled by an interleaved write",         # EPROTO
    27: "image exceeds the device staging buffer",                    # EFBIG
    2:  "no transfer session to release",                             # ENOENT
    11: "the device dropped the command before dispatch (queue full) — "
        "retry",                                                      # EAGAIN
}

# CMD_LOAD's verdict — the device confirms the parse before "Uploaded" means
# anything (issue #152: a stale wheel's image used to vanish silently).
LOAD_ERR_REASON = {
    61: "the staged image arrived short of the announced size — bytes were "
        "lost in transit; re-run the upload",                         # ENODATA
    8:  "the device rejected the image: not a valid driver (stale SDK "
        "wheel or corrupt content — detail in the device log)",       # ENOEXEC
    11: "the device dropped the command before dispatch (queue full) — "
        "retry",                                                      # EAGAIN
    5:  "the file pull failed on the device (server silent past the "
        "retry budget, or a refused write) — retry the upload",       # EIO
}

# Calibration apply/persist/procedure errnos. The numbers are the device's,
# not this host's: the firmware's libc diverges from glibc above 71, so
# EBADMSG, ETIMEDOUT and ECANCELED are 77/116/140 here where a Linux host
# would say 74/110/125. Taking them from the host's own `errno` module would
# silently unmap exactly those three.
CALIB_ERR_REASON = {
    22: "calibration record rejected (bad version, orientation, or non-finite values)",  # EINVAL
    19: "device has no calibration engine",                                              # ENODEV
    5:  "flash (NVS) write error",                                                       # EIO
    16: "another calibration procedure is running",                                      # EBUSY
    11: "insufficient rotation coverage or samples",                                     # EAGAIN
    34: "degenerate fit (cloud is not an ellipsoid)",                                    # ERANGE
    77: "fit failed the sphere self-check",                                              # EBADMSG
    116: "timed out waiting for stillness",                                              # ETIMEDOUT
    95: "the loaded driver has no source for this calibration",                 # EOPNOTSUPP
    140: "procedure cancelled (host abort or driver change)",                           # ECANCELED
}

# DFU begin/write/finish errnos.
DFU_ERR_REASON = {
    16: "a firmware update is already in progress",   # EBUSY
    22: "image rejected (bad size, type, or state)",  # EINVAL
    5:  "flash write error",                          # EIO
    61: "no image data staged",                       # ENODATA
}

# Cyphal file-pull start (LOAD_FROM_FILE / BEGIN_SOFTWARE_UPDATE): both
# commands share the device's single file client, so the reasons match.
# Libc-dependent errnos (ENOSYS, ENAMETOOLONG) stay on the numeric fallback.
PULL_ERR_REASON = {
    16: "another file transfer is already in progress",  # EBUSY
    22: "no valid file-server node for the pull",        # EINVAL
    5:  "flash write error",                             # EIO
}

ERRNO_EEXIST = 17
# Device libc numbering, not the host's: a mag solve answers EAGAIN when
# the cloud is not yet enough, and leaves the collection open to retry.
ERRNO_EAGAIN = 11
ERRNO_ENOENT = 2
ERRNO_ECANCELED = 140


def exc_detail(exc: BaseException) -> str:
    """`str(exc)`, falling back to the type name when it is empty.

    A bare `ImportError()` or `KeyError` stringifies to "", which reports
    as no reason at all."""
    return str(exc) or type(exc).__name__


def import_failure_detail(exc: BaseException, path: str) -> str:
    """`str(exc)` plus the file:line it happened at.

    A driver author's typo raises `NameError: name 'np' is not defined` —
    true and useless without a location. Walk the traceback to the last
    frame inside the driver file so the message points at their line."""
    import traceback
    detail = f"{type(exc).__name__}: {exc}"
    # A SyntaxError from a module the driver imports carries that module's
    # position, not the driver's — only trust it when the files match.
    line = (exc.lineno if isinstance(exc, SyntaxError) and exc.filename == path
            else None)
    if line is None:
        for frame in reversed(traceback.extract_tb(exc.__traceback__)):
            if frame.filename == path:
                line = frame.lineno
                break
    return f"{detail} (line {line})" if line else detail


def err_reason(code: int, reasons=None) -> str:
    """Human-readable reason for a device errno, defaulting to the
    store vocabulary."""
    return (reasons or STORE_ERR_REASON).get(code, f"device error code {code}")


@dataclass
class Sample:
    """One decoded sample.

    `values` maps field name → physical value (scaled float, or decoded
    string); empty when the driver exposes no descriptors or declares a
    type this tool can't decode — `raw` always holds the bytes.
    """
    count: int
    raw: bytes
    values: Dict[str, object] = field(default_factory=dict)
    timestamp_us: Optional[int] = None

    def __getitem__(self, name: str):
        return self.values[name]

    def get(self, name: str, default=None):
        return self.values.get(name, default)


class TimeSyncEstimator:
    """Device-clock → host-clock offset estimator — the RFC 5905 clock
    filter over two-way observations.

    Each observation brackets a device µs-clock reading between two host
    CLOCK_MONOTONIC stamps; the offset error is bounded by half the
    bracket width, so the minimum-RTT exchange in a sliding window wins.
    Drift (device oscillator ppm) is the slope between the best exchange
    of the window's oldest and newest thirds. Thread-safe: transport
    readers feed `observe()`, any thread projects.
    """

    WINDOW_S = 30.0
    RATE_WINDOW_S = 120.0
    MIN_DRIFT_BASELINE_S = 5.0
    MAX_DRIFT = 500e-6  # beyond ±500 ppm it's a glitch, not oscillator drift
    # Drift refresh cadence, in observation time: ppm-scale physics
    # changes over seconds, and the two window scans below cost
    # milliseconds — never pay them per projected sample.
    DRIFT_REFRESH_NS = 250_000_000
    # Observation spacing floor: high-rate observers (I2C polls run at
    # 5x the sample rate) would otherwise grow the window to tens of
    # thousands of entries and turn every projection's min() scan into
    # a hot-path cost. An observation inside the floor is kept only
    # when it improves on the last kept round trip.
    MIN_OBS_SPACING_NS = 50_000_000

    def __init__(self):
        self._lock = threading.Lock()
        self._drift_cache = 0.0
        self._drift_fit_ns = -10**18  # first observation always refits
        self._obs: deque = deque()  # (host_mid_ns, offset_ns, rtt_ns)
        # Best (minimum-RTT) exchange within WINDOW_S of the newest
        # observation, maintained on observe() so the per-sample
        # projection never rescans the deque — the deque itself spans
        # RATE_WINDOW_S to give the drift fit its baseline.
        self._recent_best = None

    def observe(self, t0_ns: int, t1_ns: int, device_us: int):
        """Record one two-way exchange: request sent at `t0_ns`, device
        clock read `device_us`, reply landed at `t1_ns` (host
        CLOCK_MONOTONIC nanoseconds). Exchanges closer than
        MIN_OBS_SPACING_NS to the last kept one are dropped unless they
        improve its round trip."""
        mid = (t0_ns + t1_ns) // 2
        entry = (mid, mid - device_us * 1000, t1_ns - t0_ns)
        horizon = mid - int(self.RATE_WINDOW_S * 1e9)
        with self._lock:
            if (self._obs
                    and mid - self._obs[-1][0] < self.MIN_OBS_SPACING_NS
                    and entry[2] >= self._obs[-1][2]):
                return
            self._obs.append(entry)
            while self._obs and self._obs[0][0] < horizon:
                self._obs.popleft()
            cut = mid - int(self.WINDOW_S * 1e9)
            self._recent_best = min(
                (o for o in self._obs if o[0] >= cut), key=lambda o: o[2])

    def project_mono_ns(self, device_us: int) -> Optional[int]:
        """Host CLOCK_MONOTONIC nanoseconds for a device timestamp, or
        None before the first observation."""
        with self._lock:
            if not self._obs:
                return None
            best = self._recent_best
            drift = self._drift_locked()
        dev_ns = device_us * 1000
        approx = dev_ns + best[1]
        return int(approx + drift * (approx - best[0]))

    def project_to_realtime(self, device_us: int) -> Optional[Tuple[int, int]]:
        """(sec, nanosec) of host CLOCK_REALTIME for a device timestamp,
        or None before the first observation. The monotonic→realtime
        skew is sampled per call, so an NTP step moves these stamps
        exactly as it moves every other host timestamp."""
        mono = self.project_mono_ns(device_us)
        if mono is None:
            return None
        skew = time.time_ns() - time.monotonic_ns()
        return divmod(mono + skew, 1_000_000_000)

    def ready(self) -> bool:
        with self._lock:
            return bool(self._obs)

    def bound_us(self) -> Optional[float]:
        """Half the best recent observation's round trip — the offset
        error bound in µs — or None before the first observation."""
        with self._lock:
            if not self._obs:
                return None
            return self._recent_best[2] / 2000.0

    def rate_ppb(self) -> int:
        """The fitted device clock rate error in parts per billion —
        the pushable form of the drift the projection already applies.
        0 until the fit's baseline exists."""
        with self._lock:
            return int(self._drift_locked() * 1e9)

    def _drift_locked(self) -> float:
        """Slope between the best exchanges of the window's oldest and
        newest thirds; 0.0 until the baseline is long enough. Cached —
        recomputed when at least DRIFT_REFRESH_NS of observation time
        has passed since the last fit."""
        if len(self._obs) < 4:
            return 0.0
        if self._obs[-1][0] - self._drift_fit_ns < self.DRIFT_REFRESH_NS:
            return self._drift_cache
        self._drift_fit_ns = self._obs[-1][0]
        span = self._obs[-1][0] - self._obs[0][0]
        if span < self.MIN_DRIFT_BASELINE_S * 1e9:
            self._drift_cache = 0.0
            return 0.0
        third = span // 3
        old = min((o for o in self._obs if o[0] <= self._obs[0][0] + third),
                  key=lambda o: o[2])
        new = min((o for o in self._obs if o[0] >= self._obs[-1][0] - third),
                  key=lambda o: o[2])
        if new[0] == old[0]:
            self._drift_cache = 0.0
            return 0.0
        drift = (new[1] - old[1]) / (new[0] - old[0])
        self._drift_cache = max(-self.MAX_DRIFT, min(self.MAX_DRIFT, drift))
        return self._drift_cache


class SupportsRecovery(ABC):
    """Transports that expose a bootloader recovery path."""

    @abstractmethod
    def recover(self) -> int:
        """Reset into the bootloader for a supervised re-flash."""


class SupportsTimeSync(ABC):
    """Transports that can push a host-computed time discipline to the
    unit and read the unit's live sync state."""

    @abstractmethod
    def push_time_sync(self, offset_us: int, bound_us: int,
                       rate_ppb: int = 0,
                       valid_for_us: int = 0) -> None:
        """Discipline the unit: synced = local + offset + elapsed·rate.
        The push is volatile and expires `valid_for_us` after it
        applies. The unit rejects a zero window."""

    @abstractmethod
    def read_time_sync(self) -> tuple:
        """The unit's live sync state as
        `(offset_us, bound_us, rate_ppb, valid_for_us, source, valid)`;
        source 0 also covers a stale discipline."""


class SupportsIdentify(ABC):
    """Transports that can strobe the device's status LED to physically
    locate the unit."""

    @abstractmethod
    def identify(self):
        """Strobe the status LED (~10 s), overriding the ambient pattern."""


class SupportsSlotPeek(ABC):
    """Transports that can read a stored slot's metadata without loading
    it (Cyphal `GetDriverInfo(slot)`; I2C `Cmd::PEEK_SLOT`)."""

    @abstractmethod
    def read_slot_info(self, slot: int):
        """Driver metadata for a stored slot, or None if empty.
        `slot == ACTIVE_SLOT` (0xFF) returns the active driver instead
        of a stored slot."""


class SupportsFaultCounters(ABC):
    """Transports that can read the runner's cumulative fault telemetry —
    the Cyphal `aliensense.nxs.*_count` registers, or the I2C diag view
    (`DRIVER_SELECT = DRIVER_VIEW_DIAG`, paged via `SEL_VALUE_INDEX`).
    All counters saturate at 65535; read deltas across a window."""

    @abstractmethod
    def read_io_err_count(self) -> int:
        """Absorbed I/O-error count since boot."""

    @abstractmethod
    def read_probe_failed_count(self) -> int:
        """Probe give-up count since boot."""

    @abstractmethod
    def read_drdy_coalesced_count(self) -> int:
        """Sample intervals missed because the VM was still busy when the
        sensor signalled data-ready."""

    @abstractmethod
    def read_ingress_reject_count(self) -> int:
        """Host commands dropped because another transport held a live
        session."""


class SupportsEgressDecimation(ABC):
    """Marker: the transport's stream is device-pushed, so `stream --hz`
    thins it by writing the device-wide decimation gate (a device-global,
    volatile effect). Transports without it (I2C) poll, and `--hz` paces
    the host-side poll via `set_output_rate` instead. The decimation
    *knobs* (`read_decimation`/`write_decimation`) are part of the base
    `NxsClient` contract on every transport."""


# Per-topic names for the commissioning surface, in the device's record order
# (ConfigStore::TopicId / the I2C config record). Each is a uavcan.pub.<name>.id.
COMMISSION_TOPICS = ("sample", "status", "acceleration", "angular_velocity",
                     "magnetic_field", "temperature", "pressure", "gnss", "scalar")

# Cyphal addressing limits. The node-ID range is 0-127, but 126 and 127 are
# conventionally reserved for diagnostic and host tooling (this tool claims
# 127), so a device commissions onto 0-125, the anonymous sentinel 255, or
# 0xFFFF (revert to the compiled default). A subject-ID is a 13-bit value
# 0-8191 (0 disables a topic, 0xFFFF reverts to the default); the scalar
# block publishes on base..base+SCALAR_TOPIC_SPAN-1, so its base keeps the
# whole span in range. Validated host-side so a typo can't silently wrap
# (-1) or be dropped by firmware and reported as a successful commission.
NODE_ID_MAX = 125
NODE_ID_ANONYMOUS = 255
SUBJECT_ID_MAX = 8191
SCALAR_TOPIC_SPAN = FieldSemantics.NUM_SCALAR_KINDS
ADDR_UNSET = 0xFFFF


def validate_commission(node_addr: Optional[int], topics: Optional[dict]) -> None:
    """Reject out-of-range commission inputs before they reach the wire."""
    if node_addr is not None and not (
            0 <= node_addr <= NODE_ID_MAX or node_addr == NODE_ID_ANONYMOUS
            or node_addr == ADDR_UNSET):
        raise ValueError(
            f"node-id {node_addr} out of range "
            f"(0-{NODE_ID_MAX}, {NODE_ID_ANONYMOUS} anonymous, "
            f"or 0xFFFF for the compiled default)")
    for name, addr in (topics or {}).items():
        if name not in COMMISSION_TOPICS:
            raise ValueError(f"unknown topic '{name}'")
        max_id = (SUBJECT_ID_MAX - (SCALAR_TOPIC_SPAN - 1)
                  if name == "scalar" else SUBJECT_ID_MAX)
        if not (0 <= addr <= max_id or addr == ADDR_UNSET):
            raise ValueError(
                f"subject-id {addr} for '{name}' out of range "
                f"(0-{max_id}, or 0xFFFF for the compiled default)")


# CAN bit-timing whitelist — mirror ConfigStore::valid_can_bitrate
# (the firmware ConfigStore record). Keep in sync. Each pair is
# (arbitration, data) in bit/s; equal rates select Classic CAN, and (0, 0)
# reverts to the compiled default profile.
CAN_BITRATE_PROFILES = (
    (0, 0),
    (1000000, 4000000),
    (1000000, 2000000),
    (1000000, 1000000),
    (500000, 500000),
    (250000, 250000),
    (125000, 125000),
)


def validate_can_bitrate(nominal: int, data: int) -> None:
    """Reject an unsupported CAN bit-timing profile before it reaches the wire."""
    if (nominal, data) not in CAN_BITRATE_PROFILES:
        raise ValueError(
            f"unsupported CAN bitrate profile {nominal}/{data} "
            f"(FD: 1M/4M, 1M/2M; Classic: 1M, 500k, 250k, 125k; 0 reverts)")


# CAN termination vocabulary — mirror ConfigStore::valid_can_term
# (the firmware ConfigStore record). 0xFFFF reverts to the
# product default: off, a module terminates only when commissioned to.
CAN_TERM_OFF = 0
CAN_TERM_ON = 1
CAN_TERM_UNSET = 0xFFFF


def validate_can_term(value: int) -> None:
    """Reject an out-of-vocabulary termination selection before the wire."""
    if value not in (CAN_TERM_OFF, CAN_TERM_ON, CAN_TERM_UNSET):
        raise ValueError(
            f"unsupported can-term value {value} (0 = off, 1 = on, "
            f"0xFFFF = revert to the default, off)")


class SupportsCanTermination(ABC):
    """The device drives its on-board CAN split termination from a persisted
    selection (aliensense.nxs.can_term), live-applied on write and committed
    by Save."""

    @abstractmethod
    def read_can_term(self) -> int:
        """Return the effective selection: 0 (off) or 1 (on)."""

    @abstractmethod
    def write_can_term(self, value: int) -> None:
        """Set the selection: 0, 1, or 0xFFFF (revert to the default, off).
        Applies live; persist with Save."""


class SupportsBitTiming(ABC):
    """The device persists a CAN bit-timing profile — the uavcan.can.bitrate
    pair (arbitration, data) in bit/s — applied at the next reboot."""

    @abstractmethod
    def read_can_bitrate(self) -> tuple:
        """Return ``(nominal, data)`` bit/s — the staged profile, or the
        compiled default when uncommissioned. ``data == nominal`` is Classic."""

    @abstractmethod
    def write_can_bitrate(self, nominal: int, data: int) -> None:
        """Set a profile; ``(0, 0)`` reverts to the compiled default. Applies
        at the next reboot (Cyphal stages until the Save command; I²C commits
        with the record)."""


# Calibration record geometry (constants/calibration.yaml): three vector
# buckets in SubjectBucket-1 order, each a row-major 3x3 M and a bias b,
# plus the encoder zero-offset and the per-solve driver-identity tags.
CAL_VECTORS = ("acceleration", "angular_velocity", "magnetic_field")

_CAL_STRUCT = struct.Struct("<BB2x27f9ff4I")
assert _CAL_STRUCT.size == CalConstants.RECORD_SIZE


# Every valid `orientation` code name, sorted — the vocabulary for CLI
# choices and shell completion.
ROTATION_NAMES = tuple(sorted(CalConstants.Rotation._NAMES.values()))


def rotation_name(code: int) -> str:
    """ROTATION_* name for a code, or the bare number if unknown."""
    return CalConstants.Rotation._NAMES.get(code, str(code))


def rotation_code(name: str) -> int:
    """ROTATION_* code for a name (case-insensitive). Raises ValueError."""
    code = getattr(CalConstants.Rotation, name.upper(), None)
    if not isinstance(code, int):
        raise ValueError(f"unknown rotation '{name}' "
                         f"(one of {', '.join(sorted(CalConstants.Rotation._NAMES.values()))})")
    return code


@dataclass
class CalibrationRecord:
    """The device's per-unit calibration record, host view.

    ``m``/``b`` are sensor-frame per-bucket affines indexed like
    ``CAL_VECTORS``; ``orientation`` is a ROTATION_* code composed on top by
    the appliers; a nonzero driver tag gates its bucket to the driver the
    solve ran against."""
    orientation: int = 0
    m: Tuple[Tuple[float, ...], ...] = (IDENTITY_M,) * 3
    b: Tuple[Tuple[float, ...], ...] = ((0.0, 0.0, 0.0),) * 3
    encoder_zero: float = 0.0
    driver_tags: Tuple[int, ...] = (0, 0, 0)
    encoder_tag: int = 0

    def bucket_guard(self, vec: int, active_tag: int) -> int:
        """How one bucket's stored solve relates to the running sensor, as a
        `Calibration.BucketGuard` code.

        `BOUND` — solved against this sensor. `UNGUARDED` — applied, but
        carrying no identity, so nothing verifies it belongs here; the
        deliberate escape hatch for a hand-written record. `STALE` — solved
        against a different sensor, so the affine must not be applied.

        The safety property of the whole feature, so it lives on the record
        and every applier asks it. Mirrors the firmware's
        `CalibrationRecord::bucket_guard`; `vec == len(driver_tags)` asks
        about the encoder.
        """
        guard = CalConstants.BucketGuard
        tag = (self.driver_tags[vec] if vec < len(self.driver_tags)
               else self.encoder_tag)
        if tag == 0:
            return guard.UNGUARDED

        return guard.BOUND if tag == active_tag else guard.STALE

    def pack(self) -> bytes:
        """Wire/flash layout, little-endian (`Calibration.RECORD_SIZE` bytes)."""
        flat_m = [v for row in self.m for v in row]
        flat_b = [v for row in self.b for v in row]
        return _CAL_STRUCT.pack(CalConstants.RECORD_VERSION, self.orientation,
                                *flat_m, *flat_b, self.encoder_zero,
                                *self.driver_tags, self.encoder_tag)

    @classmethod
    def unpack(cls, data: bytes) -> "CalibrationRecord":
        """Decode a wire record; raises ValueError on a bad size or version."""
        if len(data) < _CAL_STRUCT.size:
            raise ValueError(f"calibration record too short: {len(data)} B")
        fields = _CAL_STRUCT.unpack_from(data)
        version = fields[0]
        if version != CalConstants.RECORD_VERSION:
            raise ValueError(f"calibration record version {version} "
                             f"(expected {CalConstants.RECORD_VERSION})")
        flat_m = fields[2:29]
        flat_b = fields[29:38]
        return cls(orientation=fields[1],
                   m=tuple(tuple(flat_m[v * 9:v * 9 + 9]) for v in range(3)),
                   b=tuple(tuple(flat_b[v * 3:v * 3 + 3]) for v in range(3)),
                   encoder_zero=fields[38],
                   driver_tags=tuple(fields[39:42]),
                   encoder_tag=fields[42])

    def replace_vector(self, vec: int, m: Tuple[float, ...],
                       b: Tuple[float, ...], tag: int) -> "CalibrationRecord":
        """Copy with one bucket's affine + tag swapped."""
        ms = list(self.m)
        bs = list(self.b)
        tags = list(self.driver_tags)
        ms[vec] = tuple(m)
        bs[vec] = tuple(b)
        tags[vec] = tag
        return CalibrationRecord(orientation=self.orientation, m=tuple(ms),
                                 b=tuple(bs), encoder_zero=self.encoder_zero,
                                 driver_tags=tuple(tags),
                                 encoder_tag=self.encoder_tag)


class SupportsCalibration(ABC):
    """The device stores and applies a per-unit calibration record — one
    affine per vector bucket, the mounting orientation, and the encoder
    zero-offset — served over the wire and persisted by the unified Save."""

    @abstractmethod
    def read_calibration(self) -> CalibrationRecord:
        """The record the device currently applies (running state)."""

    @abstractmethod
    def read_cal_epoch(self) -> int:
        """The bank's change counter (wraps past 255): a one-read probe
        for "did the record move" without the record transfer."""

    @abstractmethod
    def write_calibration(self, record: CalibrationRecord,
                          persist: bool = True) -> None:
        """Apply ``record`` to the running state; ``persist`` commits it to
        the device's store so it survives a reboot."""

    @abstractmethod
    def set_orientation(self, rotation: int, persist: bool = True) -> None:
        """Set the mounting-orientation code, leaving the solved affines
        untouched."""

    @abstractmethod
    def cal_gyro(self) -> None:
        """Start the on-device gyro still-average (hold the unit still);
        completion and its verdict land in `read_cal_progress`."""

    @abstractmethod
    def cal_mag_start(self) -> None:
        """Start the on-device mag collection (rotate the vehicle)."""

    @abstractmethod
    def cal_mag_stop(self) -> None:
        """Close the mag collection: the device gates, solves, self-checks,
        and applies. Raises `DeviceRefused` on a refused fit (the previous
        calibration stays)."""

    @abstractmethod
    def cal_abort(self) -> None:
        """Drop the procedure in progress without solving, leaving the
        previous calibration in place.

        `cal_mag_stop` is the other way out of a collection, but it also
        solves and applies, so it cannot express an operator's Ctrl-C —
        and a gyro pass has no stop verb at all. Until the procedure ends
        it holds the device's transfer session, which refuses every later
        command, including the firmware push that would recover the unit.
        """

    @abstractmethod
    def read_cal_progress(self) -> Tuple[int, int, int]:
        """(state, detail, result): a `Calibration.CalState` code, the gyro
        percent / mag rotation coverage, and the last completed verdict
        (`CAL_RESULT_NONE` until one completes)."""

    @abstractmethod
    def save_calibration(self) -> None:
        """Persist the running record via the unified Save (an on-device
        procedure applies to the running state only)."""


class SupportsCommissioning(ABC):
    """The device persists a Cyphal node-ID and per-topic subject-IDs,
    commissioned over the wire and applied on the next reboot."""

    @abstractmethod
    def read_identity(self) -> dict:
        """Return ``{'node_addr': int, 'topics': {name: int}}`` — the effective
        node-ID and subject-IDs the device uses (the default when uncommissioned)."""

    @abstractmethod
    def commission(self, node_addr: Optional[int] = None,
                   topics: Optional[dict] = None,
                   can_bitrate: Optional[tuple] = None,
                   can_term: Optional[int] = None) -> None:
        """Stage ``node_addr``, per-topic subject-IDs (by name), the CAN
        bit-timing pair, and/or the termination selection, and commit them.
        ``0xFFFF`` keeps the compiled default; a subject ``0`` disables the
        topic; a bitrate of ``(0, 0)`` reverts to the default profile.
        Identity and bit timing take effect at the next reboot; termination
        applies live."""


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
        """True if a background thread saw a mid-session physical link loss
        (a USB unplug or a wedged serial device). Default False; the Cyphal
        client overrides."""
        return False

    def probe_failure_detail(self) -> Optional[str]:
        """Why the last `probe()` found nothing, when the transport knows.

        None when it has nothing to add beyond the device being silent."""
        return None

    # ── Time sync ─────────────────────────────────────────
    def get_time_sync(self) -> TimeSyncEstimator:
        """The transport's device↔host clock estimator. I²C feeds it
        from every sample-record poll; Cyphal transports feed it via
        `time_sync_ping()`."""
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
        """One two-way sync exchange: bracket a device-clock read with
        host stamps and feed the estimator. False when the transport
        serves no time surface."""
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
        """The running firmware version ("MAJOR.MINOR") where the
        transport serves it, or None where it doesn't. Default None;
        a transport overrides it when its wire serves a version."""
        return None

    # ── Driver lifecycle ──────────────────────────────────
    @abstractmethod
    def upload_image(self, image: bytes):
        """Stage and load an NXS image into the VM."""

    @abstractmethod
    def vm_run(self):
        """Start the VM (probe → configure → measure)."""

    @abstractmethod
    def vm_stop(self):
        """Halt the VM; the driver stays loaded."""

    @abstractmethod
    def vm_reset(self):
        """Unload the driver entirely."""

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

    # ── Driver store ──────────────────────────────────────
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
    def read_driver_name(self) -> str: ...
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
        """The device's output-field descriptor set (name, type, byte
        order, scale, offset, unit, semantic per field). [] when the
        device genuinely serves none — callers fall back to a local
        compile. None when the set is transiently unreadable (device
        mid-load, epoch unstable, RPC timeout) — callers keep their
        previous knowledge and retry later."""

    @abstractmethod
    def _descriptor_token(self) -> int:
        """Opaque generation of the current descriptor set. Changes when
        the loaded driver changes (I2C: DESCRIPTOR_EPOCH). A transport
        without change detection returns a constant."""

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
        `timestamp_us` is None where the transport has none (I2C polls a
        register window; a push transport pulls its receive queue)."""

    # ── Streaming procedures (shared) ─────────────────────
    def set_output_rate(self, hz):
        """Set a target output rate in Hz; must be positive (no cap — the
        operator sizes it to the bus). The default validates then no-ops:
        transports that deliver every sample (push) or have no
        host-controllable rate ignore the value; the I2C poll transport
        overrides this to set its poll cadence. Raises ValueError on a
        non-positive rate rather than silently picking a default."""
        if hz is None or hz <= 0:
            raise ValueError("output rate (Hz) must be positive")

    def start_stream(self, every_nth: int = 1):
        """Begin streaming with the given decimation. Idempotent — a
        no-op if already streaming, so it never re-sends the arm command
        (change the rate with stop_stream() then start_stream())."""
        if self._streaming:
            return
        self._arm_stream(every_nth)
        self._streaming = True

    def stop_stream(self):
        """Stop streaming. Idempotent — a no-op if not streaming, so a
        double-stop or teardown path never re-sends the disarm command."""
        if not self._streaming:
            return
        self._streaming = False
        self._disarm_stream()

    def iter_samples(self, timeout: float = 1.0,
                     calibrated: bool = True,
                     max_silence_s: Optional[float] = None) -> Iterator[Sample]:
        """Yield decoded `Sample`s until streaming is stopped.

        ``max_silence_s`` ends the stream (a plain return) after that
        long without a sample, so an acquisition loop over a muted or
        failed sensor fails through its own no-data path instead of
        hanging; None (the default) waits forever.

        ``calibrated`` composes the served calibration record into the
        decoded values — the host twin of the device's SI tier — and
        re-fetches it with the descriptors on a driver swap so the tag
        guard tracks the live driver. The calibration procedures opt out
        to acquire at the raw descriptor tier.

        Fetches the descriptor set once, decodes each sample with it,
        and re-fetches when `_descriptor_token()` changes mid-stream
        (a driver swap), so values track the live driver. Decoding adds
        no per-driver host knowledge — it uses only what the device
        serves.

        Disarms the stream when the iterator exits for any reason —
        `break`, an exception, or generator GC — so a push transport
        stops emitting and its receive queue stops growing. `stop_stream`
        is idempotent, so an explicit stop or a wrapping `spin()` is fine.
        """
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
                # The bank can move with no descriptor change — a boot-auto
                # solve, an on-device procedure, a write from the other
                # transport — so the epoch is probed on a slow clock. A
                # probe the device refuses (a live transfer session) just
                # waits for the next tick.
                if calibrated and time.monotonic() >= next_cal_check:
                    next_cal_check = time.monotonic() + CAL_REFRESH_S
                    try:
                        epoch = self.read_cal_epoch()
                    except Exception:
                        epoch = cal_epoch
                    if epoch is not None and epoch != cal_epoch:
                        # The first observation reloads too: the bank may
                        # have moved between the startup record read and
                        # this probe, and adopting the epoch without the
                        # record would pin that stale affine forever.
                        cal_epoch = epoch
                        calibration, active_tag = self._load_calibration()
                # Descriptors change only on a VM restart (which resets seq);
                # re-check the RPC-backed token on that backward jump, not per
                # sample — per-sample it collapsed the stream to the GetDriverInfo
                # round-trip rate (~1 Hz, since the response is TX-dropped under load).
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
        """Non-blocking (or briefly-blocking) single-sample read, for
        multiplexing several clients in one loop. Returns a decoded
        `Sample`, or None if none arrived within `timeout`. Arms the
        stream on first call; caches the descriptor set and re-fetches
        when the driver changes, like `iter_samples`."""
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
        `stop_spin()` or KeyboardInterrupt. The push-style counterpart
        of iterating `iter_samples()` yourself."""
        self._spinning = True
        try:
            for sample in self.iter_samples(timeout=timeout):
                if not self._spinning:
                    break
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
        """Descriptor refresh across a driver change. Commits the new
        (token, fields) pair only when a complete set was read; while the
        swap is unresolved (set transiently unreadable) the old token is
        kept — so the next sample retries — and the fields collapse to
        raw-only. Samples are never decoded with the previous driver's
        map: committing the token before the fetch used to pin a stale
        map for an entire factory row."""
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
        """The served record + the running sensor's tag for the decode
        post-pass, or (None, 0) when the transport serves no calibration
        surface or the read fails (the decode stays descriptor-tier raw)."""
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
    """The running sensor's identity tag: driver name, bus, latched address.

    The single host-side producer — `calibrate show`, the suite's CAL column
    and the decode post-pass all ask here, because a tag derived two ways is
    a guard that fails open. Mirrors the firmware's `cal::driver_tag`.

    The bus is read, never inferred: a driver with no latched I²C address may
    be on SPI *or* on a UART stream, and guessing SPI from a zero address
    yields a tag the device never wrote. `record`, when given, short-circuits
    the reads for a record that carries no tags at all — nothing to compare,
    so nothing to fetch.
    """
    if record is not None and not any(record.driver_tags) and not record.encoder_tag:
        return 0
    from nxs.descriptor import driver_tag
    try:
        name = transport.read_driver_name() or ""
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


PUSH_INTERVAL_S = 1.0
"""Default refresh cadence of the resident pushers, seconds."""

SYNC_LOST_PUSHES = 10
"""Pushes that may be lost before the pushed discipline expires."""


DRIVER_UP_TIMEOUT_S = 5.0
"""Budget for a driver to load, reset its sensor, and probe after a RUN."""

DRIVER_UP_POLL_S = 0.2
"""Register-poll spacing while waiting. One transaction per tick against a
probe measured in hundreds of milliseconds."""


def await_driver_up(client, timeout_s: float = DRIVER_UP_TIMEOUT_S,
                    poll_s: float = DRIVER_UP_POLL_S) -> int:
    """Block until the runner settles after `vm_run()`, and return its state.

    `vm_run()` returns when the device accepts the command, not when the
    driver has probed — an IAM-20680's reset-and-settle alone is ~200 ms — so
    reading any result on the next line reports a working deploy as a failed
    one. Settling means `MEASURING` (the driver is up) or `PROBE_FAILED` (it
    gave up); either ends the wait, so a dead sensor answers immediately
    instead of consuming the whole budget. A timeout returns the last state
    read, which distinguishes a driver still `PROBING` from one that never
    loaded.
    """
    state = RunnerStates.RunnerState
    deadline = time.monotonic() + timeout_s
    while True:
        current = client.read_runner_state()
        if current in (state.MEASURING, state.PROBE_FAILED):
            return current
        if time.monotonic() >= deadline:
            return current
        time.sleep(poll_s)


#: Budget for the time-sync mirror to catch up with a push. Sized for the
#: worst observed drain — a seed issued straight after a panel deploy, while
#: the comm thread still has the store writes and the driver probe ahead of it.
PUSH_ECHO_TIMEOUT_S = 2.0
PUSH_ECHO_POLL_S = 0.01


def estimate_and_push(client, pings: int = 8,
                      interval_s: float = PUSH_INTERVAL_S):
    """Refresh the client's estimator with a ping burst and push the
    result, valid for `SYNC_LOST_PUSHES` cycles of `interval_s`.
    Returns the pushed bound in µs, or None when the transport serves
    no time surface. The offset maps the device clock into host
    CLOCK_REALTIME."""
    for _ in range(pings):
        if not client.time_sync_ping():
            return None
    estimator = client.get_time_sync()
    device_us = client.read_device_time_us()
    if device_us is None or not estimator.ready():
        return None
    projected = estimator.project_to_realtime(device_us)
    if projected is None:
        return None
    sec, nanosec = projected
    offset_us = sec * 1_000_000 + nanosec // 1000 - device_us
    bound_us = int(estimator.bound_us())
    rate_ppb = estimator.rate_ppb()
    valid_for_us = int(SYNC_LOST_PUSHES * interval_s * 1_000_000)
    client.push_time_sync(offset_us, bound_us, rate_ppb, valid_for_us)
    # The record applies on the comm thread, not in the transaction that
    # carries it, so poll the echo instead of reading it once: right after a
    # panel deploy that thread is still draining flash writes and a single
    # read returns the previous record. The mirror echoes bound, rate, and
    # window verbatim; offset legitimately differs (servo re-anchor), so it
    # stays unchecked.
    deadline = time.monotonic() + PUSH_ECHO_TIMEOUT_S
    while True:
        echoed = client.read_time_sync()
        if echoed is None:
            return bound_us
        _, echoed_bound, echoed_rate, echoed_window, _, echoed_valid = echoed
        if (echoed_valid and echoed_bound == bound_us
                and echoed_rate == rate_ppb and echoed_window == valid_for_us):
            return bound_us
        if time.monotonic() >= deadline:
            raise RuntimeError("push not applied (mirror mismatch)")
        time.sleep(PUSH_ECHO_POLL_S)
