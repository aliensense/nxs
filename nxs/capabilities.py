"""The optional contracts a transport declares, with the validators and the records their calls take."""

import struct
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple

from nxs._generated_constants import Calibration as CalConstants
from nxs._generated_constants import FieldSemantics
from nxs._generated_constants import NxsRegisters
from nxs._generated_constants import Personality
from nxs.click_facts import IDENTITY_M


class SupportsRecovery(ABC):
    """Transports that expose the device's reset paths."""

    @abstractmethod
    def recover(self) -> int:
        """Reset into the bootloader for a supervised re-flash."""

    @abstractmethod
    def reboot(self) -> None:
        """Cold-reset the device from the application, mid-session too:
        a staged update swaps in, an unconfirmed image reverts."""

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
    it (Cyphal `GetDriverInfo(slot)`; I2C the slot's `STORE_DIRECTORY` entry)."""

    @abstractmethod
    def read_slot_info(self, slot: int):
        """Personality metadata for a stored slot, or None if empty.
        `slot == ACTIVE_SLOT` (0xFF) returns the active personality instead
        of a stored slot. Its `kind` is the image kind's name, None where
        the transport serves no kind (Cyphal)."""

class SupportsFaultCounters(ABC):
    """Transports that read the runner's fault counters (the Cyphal
    `aliensense.nxs.*_count` registers, or the I2C `DIAG` region); each saturates at 65535."""

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

    @abstractmethod
    def read_io_errno(self) -> int:
        """The positive errno of the latest absorbed I/O error, 0 before the first."""

    @abstractmethod
    def read_bus_heal_count(self) -> int:
        """Resets of the host-bus peripheral after a transfer the bus cut; 0 on a unit without a pod bus."""

    @abstractmethod
    def read_bus_heal_errno(self) -> int:
        """The positive errno of the failed transfer that preceded the latest heal, 0 before the first."""

class SupportsEgressDecimation(ABC):
    """Marker: the transport's stream is device-pushed, so `stream --hz` writes the
    device-wide decimation gate; transports without it pace a host-side poll."""

# Per-topic names for the commissioning surface, in the device's record order.
# Each is a uavcan.pub.<name>.id.
COMMISSION_TOPICS = ("sample", "status", "acceleration", "angular_velocity",
                     "magnetic_field", "temperature", "pressure", "gnss", "scalar")

# Cyphal addressing limits: a device commissions onto node-ID 0-125 (126 and
# 127 are host tooling), the sentinel 255, or 0xFFFF; a subject-ID is 0-8191.
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

# CAN bit-timing whitelist as (arbitration, data) in bit/s; equal rates select
# Classic CAN, and (0, 0) reverts to the compiled default profile.
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

# CAN termination vocabulary. 0xFFFF reverts to the product default: off, a
# module terminates only when commissioned to.
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
    selection (aliensense.nxs.can_term), live-applied on write, committed by Save."""

    @abstractmethod
    def read_can_term(self) -> int:
        """Return the effective selection: 0 (off) or 1 (on)."""

    @abstractmethod
    def write_can_term(self, value: int) -> None:
        """Set the selection: 0, 1, or 0xFFFF (revert to the default, off).
        Applies live; persist with Save."""

def validate_sample_fifo_depth(records: int) -> None:
    """Reject a depth the device would ignore before the wire."""
    if not 0 <= records <= NxsRegisters.SAMPLE_FIFO_DEPTH_MAX:
        raise ValueError(
            f"unsupported fifo depth {records} "
            f"(0..{NxsRegisters.SAMPLE_FIFO_DEPTH_MAX} records, 0 = all the storage holds)")

class SupportsSampleFifo(ABC):
    """The device queues samples for the I2C burst read. The depth
    (aliensense.nxs.sample_fifo.depth) is live-applied on write, committed by Save."""

    @abstractmethod
    def read_sample_fifo_depth(self) -> int:
        """Return the depth setting in records; 0 = all the storage holds."""

    @abstractmethod
    def write_sample_fifo_depth(self, records: int) -> None:
        """Set the depth in records. Applies live; persist with Save."""

class SupportsBitTiming(ABC):
    """The device persists a CAN bit-timing profile, the uavcan.can.bitrate
    pair (arbitration, data) in bit/s, applied at the next reboot."""

    @abstractmethod
    def read_can_bitrate(self) -> tuple:
        """Return ``(nominal, data)`` bit/s: the staged profile, or the compiled
        default when uncommissioned. ``data == nominal`` is Classic."""

    @abstractmethod
    def write_can_bitrate(self, nominal: int, data: int) -> None:
        """Set a profile; ``(0, 0)`` reverts to the compiled default. Applies at
        the next reboot; Cyphal stages until Save, I2C commits with the record."""

# Calibration record geometry: three vector buckets in SubjectBucket-1 order,
# each a row-major 3x3 M and a bias b, plus the encoder zero-offset and tags.
CAL_VECTORS = ("acceleration", "angular_velocity", "magnetic_field")

# The record's blocks: the header, then one per vector bucket.
_CAL_HEAD = struct.Struct("<BB2xfI")
_CAL_BUCKET = struct.Struct("<9f3fI")
assert CalConstants.RECORD_SIZE == 4 * CalConstants.RECORD_BLOCK_SIZE
assert _CAL_BUCKET.size <= CalConstants.RECORD_BLOCK_SIZE

# Every valid `orientation` code name, sorted: the vocabulary for CLI choices
# and shell completion.
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
    """The device's per-unit calibration record, host view. ``m``/``b`` are
    sensor-frame affines indexed like ``CAL_VECTORS``, and a personality tag
    gates its bucket. ``orientation`` is the ROTATION_* code the running IMU
    personality's `orientation` row holds, composed on top: the record
    carries no rotation of its own."""
    orientation: int = 0
    m: Tuple[Tuple[float, ...], ...] = (IDENTITY_M,) * 3
    b: Tuple[Tuple[float, ...], ...] = ((0.0, 0.0, 0.0),) * 3
    encoder_zero: float = 0.0
    personality_tags: Tuple[int, ...] = (0, 0, 0)
    encoder_tag: int = 0

    def bucket_guard(self, vec: int, active_tag: int) -> int:
        """How one bucket's stored solve relates to the running sensor, as a
        `Calibration.BucketGuard` code: `BOUND` solved against this sensor,
        `UNGUARDED` carrying no identity, `STALE` never applied."""
        guard = CalConstants.BucketGuard
        tag = (self.personality_tags[vec] if vec < len(self.personality_tags)
               else self.encoder_tag)
        if tag == 0:
            return guard.UNGUARDED

        return guard.BOUND if tag == active_tag else guard.STALE

    def pack(self) -> bytes:
        """The wire record, `Calibration.RECORD_SIZE` bytes of little-endian blocks."""
        raw = bytearray(CalConstants.RECORD_SIZE)
        # Byte 1 is reserved: the mount rotation is the personality's row.
        _CAL_HEAD.pack_into(raw, 0, CalConstants.RECORD_VERSION, 0,
                            self.encoder_zero, self.encoder_tag)
        for v in range(3):
            _CAL_BUCKET.pack_into(raw, CalConstants.RECORD_BLOCK_SIZE * (v + 1),
                                  *self.m[v], *self.b[v], self.personality_tags[v])
        return bytes(raw)

    @classmethod
    def unpack(cls, data: bytes) -> "CalibrationRecord":
        """Decode a wire record; raises ValueError on a bad size or version."""
        if len(data) < CalConstants.RECORD_SIZE:
            raise ValueError(f"calibration record too short: {len(data)} B")
        version, _reserved, encoder_zero, encoder_tag = _CAL_HEAD.unpack_from(data, 0)
        if version != CalConstants.RECORD_VERSION:
            raise ValueError(f"calibration record version {version} "
                             f"(expected {CalConstants.RECORD_VERSION})")
        buckets = [_CAL_BUCKET.unpack_from(data, CalConstants.RECORD_BLOCK_SIZE * (v + 1))
                   for v in range(3)]
        return cls(orientation=0,
                   m=tuple(tuple(bucket[0:9]) for bucket in buckets),
                   b=tuple(tuple(bucket[9:12]) for bucket in buckets),
                   encoder_zero=encoder_zero,
                   personality_tags=tuple(bucket[12] for bucket in buckets),
                   encoder_tag=encoder_tag)

    def replace_vector(self, vec: int, m: Tuple[float, ...],
                       b: Tuple[float, ...], tag: int) -> "CalibrationRecord":
        """Copy with one bucket's affine + tag swapped."""
        ms = list(self.m)
        bs = list(self.b)
        tags = list(self.personality_tags)
        ms[vec] = tuple(m)
        bs[vec] = tuple(b)
        tags[vec] = tag
        return CalibrationRecord(orientation=self.orientation, m=tuple(ms),
                                 b=tuple(bs), encoder_zero=self.encoder_zero,
                                 personality_tags=tuple(tags),
                                 encoder_tag=self.encoder_tag)

class SupportsCalibration(ABC):
    """The device stores and applies a per-unit calibration record: one affine per
    vector bucket and the encoder zero-offset. The mounting orientation it
    composes on top is the IMU personality's `orientation` row."""

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
    def set_orientation(self, rotation: int) -> None:
        """Commit the mounting-orientation code to the running IMU
        personality's `orientation` row, leaving the solved affines
        untouched. The row is live until a save of the personality's slot
        keeps it, or until `nxs switch` converges the declared rotation."""

    @abstractmethod
    def cal_gyro(self) -> None:
        """Start the on-device gyro still-average (hold the unit still);
        completion and its verdict land in `read_cal_progress`."""

    @abstractmethod
    def cal_mag_start(self) -> None:
        """Start the on-device mag collection (rotate the vehicle)."""

    @abstractmethod
    def cal_mag_stop(self) -> None:
        """Close the mag collection: the device gates, solves, self-checks, and
        applies. Raises `DeviceRefused` on a refused fit (the previous stays)."""

    @abstractmethod
    def cal_abort(self) -> None:
        """Drop the procedure in progress without solving, leaving the previous
        calibration in place."""

    @abstractmethod
    def read_cal_progress(self) -> Tuple[int, int, int]:
        """(state, detail, result): a `Calibration.CalState` code, the gyro
        percent / mag rotation coverage, and the last completed verdict
        (`CAL_RESULT_NONE` until one completes)."""

    @abstractmethod
    def save_calibration(self) -> None:
        """Persist the running record via the unified Save (an on-device
        procedure applies to the running state only)."""

CamRunState = Personality.CamRunState

class SupportsCameraRun(ABC):
    """The unit runs a stored cam personality on its pod-side bus by the
    commits of its rows through the unit door, while the host stays off
    the sensor (a port's graph image walks it, `nxs.cam.graph`), and serves
    the personality's descriptor trailer back."""

    @abstractmethod
    def read_personality_info(self, slot: int) -> bytes:
        """The descriptor trailer of the personality in store slot `slot`,
        count byte included (`nxs.image.parse_trailer` splits it); a slot
        holding no descriptors serves the one-byte empty trailer."""

class SupportsCommissioning(ABC):
    """The device persists a Cyphal node-ID and per-topic subject-IDs,
    commissioned over the wire and applied on the next reboot."""

    @abstractmethod
    def read_identity(self) -> dict:
        """Return ``{'node_addr': int, 'topics': {name: int}}``: the effective
        node-ID and subject-IDs (the default when uncommissioned)."""

    @abstractmethod
    def commission(self, node_addr: Optional[int] = None,
                   topics: Optional[dict] = None,
                   can_bitrate: Optional[tuple] = None,
                   can_term: Optional[int] = None) -> None:
        """Stage ``node_addr``, per-topic subject-IDs, the CAN bit-timing pair
        and the termination selection, and commit them. ``0xFFFF`` keeps the
        compiled default, ``0`` disables a topic, ``(0, 0)`` reverts."""
