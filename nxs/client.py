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
import threading
import time
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterator, List, Optional, Tuple

from nxs._generated_constants import FieldSemantics
from nxs.descriptor import is_decodable, parse_sample

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
    """Transports that can read the runner's cumulative fault telemetry
    (Cyphal `aliensense.nxs.io_err_count` / `probe_failed_count`)."""

    @abstractmethod
    def read_io_err_count(self) -> int:
        """Absorbed I/O-error count since boot."""

    @abstractmethod
    def read_probe_failed_count(self) -> int:
        """Probe give-up count since boot."""


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

    def iter_samples(self, timeout: float = 1.0) -> Iterator[Sample]:
        """Yield decoded `Sample`s until streaming is stopped.

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
            prev_seq = None
            while self._streaming:
                item = self._next_raw(timeout)
                if item is None:
                    continue
                # Descriptors change only on a VM restart (which resets seq);
                # re-check the RPC-backed token on that backward jump, not per
                # sample — per-sample it collapsed the stream to the GetDriverInfo
                # round-trip rate (~1 Hz, since the response is TX-dropped under load).
                count = item[0]
                if token == -1 or (prev_seq is not None and count < prev_seq):
                    token, fields = self._refreshed_fields(token, fields)
                prev_seq = count
                yield self._decode(item, fields)
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
        item = self._next_raw(timeout)
        if item is None:
            return None
        self._poll_token, self._poll_fields = self._refreshed_fields(
                self._poll_token, self._poll_fields)
        return self._decode(item, self._poll_fields)

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
                fields: List[dict]) -> Sample:
        count, raw, timestamp_us = item
        values = parse_sample(raw, fields) if fields else {}
        return Sample(count=count, raw=raw, values=values,
                      timestamp_us=timestamp_us)

PUSH_INTERVAL_S = 1.0
"""Default refresh cadence of the resident pushers, seconds."""

SYNC_LOST_PUSHES = 10
"""Pushes that may be lost before the pushed discipline expires."""


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
    echoed = client.read_time_sync()
    if echoed is not None:
        # The mirror echoes bound, rate, and window verbatim; offset
        # legitimately differs (servo re-anchor), so it stays unchecked.
        _, echoed_bound, echoed_rate, echoed_window, _, echoed_valid = echoed
        if (not echoed_valid or echoed_bound != bound_us
                or echoed_rate != rate_ppb or echoed_window != valid_for_us):
            raise RuntimeError("push not applied (mirror mismatch)")
    return bound_us
