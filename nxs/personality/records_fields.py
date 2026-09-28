"""The trailer's shared vocabulary: the record ids, the struct layouts, and the primitives every record is written and read through."""

from __future__ import annotations

import struct
import zlib
from typing import Any, Dict, List, Tuple

from nxs._generated_constants import NxsDriverImage
from nxs.cam.contracts import ContractError
from nxs.cam.descriptors import Descriptor, to_int

TrailerRecord = NxsDriverImage.TrailerRecord
MAX_TRAILER_SIZE = NxsDriverImage.MAX_TRAILER_SIZE

IDENTITY = TrailerRecord.IDENTITY
PROGRAM = TrailerRecord.PROGRAM
TRIGGERS = TrailerRecord.TRIGGERS
RUN_PARAMS = TrailerRecord.RUN_PARAMS

#: The enum params a camera personality dispatches on, by the name its
#: behaviour class declares them under; the wire carries their indices.
MODE_PARAM = "mode"
TRIGGER_PARAM = "trigger"
#: The run parameters a camera personality stages in physical units and
#: the values it records for the host, by name: the line period and the
#: frame period a run starts the sensor at, the frame length in lines it
#: achieved, and the action a run performs.
ACTION_PARAM = "action"
LINE_TIME_PARAM = "line_time"          # ns
FRAME_PERIOD_PARAM = "frame_period"    # ns
FRAME_LENGTH_PARAM = "frame_length"    # lines, what the run achieved
#: What a run does: the whole program (the tables, the timing, the start),
#: a standby, a start from standby, or the timing program alone (the
#: staged line and frame periods written under standby, then the start).
ACTIONS: Dict[str, int] = {"configure": 0, "park": 1, "start": 2, "timing": 3}

#: Law families by wire id; a descriptor naming no family is generic.
FAMILY_IDS: Dict[str, int] = {"generic": 1, "sony_imx": 2}
_FAMILY_NAMES = {v: k for k, v in FAMILY_IDS.items()}

#: The kernel control vocabulary: every row a law family emits, by wire
#: id, with the register name a decoded descriptor gives the row's
#: register. The table the kernel driver interprets is fixed to this set.
CONTROL_IDS: Dict[str, int] = {
    "group-hold": 1, "gain": 2, "exposure": 3, "frame-length": 4,
    "line-length": 5, "standby": 6, "start": 7, "trigger": 8, "vint-en": 9,
    "sync-sel": 10, "test-pattern": 11, "black-level": 12,
}
_CONTROL_NAMES = {v: k for k, v in CONTROL_IDS.items()}
_CONTROL_REGISTERS: Dict[str, str] = {
    "group-hold": "REGHOLD", "gain": "GAIN", "exposure": "SHS",
    "frame-length": "VMAX", "line-length": "HMAX", "standby": "STANDBY",
    "start": "XMSTA", "trigger": "TRIGMODE", "vint-en": "VINT_EN",
    "sync-sel": "SYNCSEL", "test-pattern": "TESTPAT_EN",
    "black-level": "BLKLEVEL",
}
#: The register the test-pattern row's third parameter addresses.
_TEST_PATTERN_SELECT = "TESTPAT_SEL"
#: The register a decoded generic descriptor polls for the alive check
#: when the identity register is not it.
_ALIVE_REGISTER = "ALIVE"

PIXEL_PHASES = ("rggb", "grbg", "gbrg", "bggr")
_SYNC_ROLES = ("master", "slave")
_TRIGGER_SLOTS = ("freerun", "fast", "sequential")

#: Payload caps the records inherit from their length fields.
MAX_NAME = 32
MAX_COMPATIBLE = 48
_NONE_U16 = 0xFFFF
_NONE_U8 = 0xFF

_MODE_FLAG_DEFAULT = 0x01
_MODE_FLAG_TRIGGERABLE = 0x02
_MODE_FLAG_SERIALIZER_CSI = 0x04
_IDENTITY_FLAG_TAKES_TRIGGER = 0x01
_PROGRAM_FLAGS = ("timing_start", "trigger_switch", "fast_trigger",
                  "sync_switch", "restart", "start", "stop_ms", "hmax_live")
#: A mode's timing facts, in presence-bit order, with the struct format of
#: each ("s" is a length-prefixed string, "q" a x1000 fixed-point number,
#: "c" a num/den pair).
_TIMING_FIELDS = (("hmax", "H"), ("vmax", "I"), ("vmax_clean", "I"),
                  ("vmax_jump_threshold", "I"), ("trigger_vmax", "I"),
                  ("min_frame_length", "I"), ("framerate_cap", "c"),
                  ("exposure_us", "I"), ("gain", "H"), ("fps", "q"),
                  ("delta_kind", "s"), ("vint_mode", "B"))
#: The experimental operating points a mode's timing once carried: refused by
#: the encoder (a personality is compiled from the shipped descriptor),
#: read past and dropped by the decoder.
_RETIRED_TIMING = frozenset({"vmax_clean", "vmax_jump_threshold", "trigger_vmax",
                             "framerate_cap", "exposure_us", "gain"})
#: The law families' `limits`, in presence-bit order: scalars with their
#: struct format, "n" for a number carried as nanoseconds, "g" for the gain
#: law, "w" for the wait table, "i" for the floor's wait indices, "d" for
#: a delta that is one value or a table, "t" for a table by readout kind.
_LAW_FIELDS = (("inck_hz", "I"), ("integration_offset_us", "n"),
               ("min_integration_lines", "H"), ("gain_max", "I"),
               ("gain_reg_per_db", "g"), ("gain_hcg_reg_min", "I"),
               ("min_fps", "q"), ("captured_waits", "w"),
               ("shs_floor_regs", "i"), ("min_frame_length_delta", "d"),
               ("frame_length_delta_const", "t"), ("shs_floor", "H"))
_DELTA_SCALAR = 1
_DELTA_TABLE = 2

_IDENTITY_HEAD = struct.Struct("<BBBBHBHHB")
_MODES_HEAD = struct.Struct("<BB")
_MODE_HEAD = struct.Struct("<BHHBBHBBBBH")
_TRIGGERS_HEAD = struct.Struct("<BB")
_RUN_PARAMS_HEAD = struct.Struct("<B")
_RUN_PARAM = struct.Struct("<BIII")
_CONTROL_ROW = struct.Struct("<BHBBBIIII")
_LAWS_HEAD = struct.Struct("<BH")
_SHIPPED_POINT = struct.Struct("<BBBIIHI")
_CAMERA_COUNTS = (1, 2)
_CAPTURE_HEAD = struct.Struct("<IIBIiiIiHHIIIIIIIB")
#: A capture row: width u16, height u16, bit depth u8, pixel phase u8, line
#: length u32, max fps x1000 u32, min exposure us u32, embedded-data lines
#: u8. A trailer written before the last field carried 15-byte rows; the
#: decoder takes the row size from the record's length.
_CAPTURE_ENTRY = struct.Struct("<HHBBIIIB")
_CAPTURE_ENTRY_V1 = struct.Struct("<HHBBIII")
_CAPTURE_FLAG_CLOCK_NONCONTINUOUS = 0x01
_PROGRAM_HEAD = struct.Struct("<BHHHBB")
_PROGRAM_REPAIR = struct.Struct("<HBBI")
_PROGRAM_TAIL = struct.Struct("<HHHHHHHHHHHHHHHH")
#: The AD-depth block that may follow the tail: the ADBIT_MONOSEL register
#: (addr u16, width u8, order u8), the chromacity u8, the init table's
#: INCK u32, the code count u8, then (bit depth u8, code u8) per output
#: depth. Its presence is the record's length.
_PROGRAM_ADBIT = struct.Struct("<HBBBIB")
_PROGRAM_ADBIT_CODE = struct.Struct("<BB")
#: The black-level block that may follow the AD-depth block: the BLKLEVEL
#: register (addr u16, width u8, order u8), the value count u8, then (bit
#: depth u8, value u32) per output depth. Its presence is the record's
#: length too.
_PROGRAM_BLKLEVEL = struct.Struct("<HBBB")
_PROGRAM_BLKLEVEL_VALUE = struct.Struct("<BI")
_CHROMACITIES = ("color", "mono")
#: VINT_EN's two interrupt bits (the Sony family's `VINT_BITS`).
_VINT_BITS = 0x03


class RecordError(ContractError):
    """A descriptor fact the trailer cannot carry, or a trailer this codec
    cannot read; the message names which."""

def trailer_crc(data: bytes) -> int:
    """The CRC-32 of a trailer's bytes: what a cache compares to know
    whether the unit's personality changed."""
    return zlib.crc32(bytes(data)) & 0xFFFFFFFF

def _u(value: Any, bits: int, what: str) -> int:
    v = int(value)
    if not 0 <= v < (1 << bits):
        raise RecordError(f"{what}: {v} does not fit {bits} bits")
    return v

def _text(value: str, limit: int, what: str) -> bytes:
    raw = str(value).encode("ascii", errors="strict")
    if len(raw) > limit:
        raise RecordError(f"{what}: {value!r} is longer than {limit} characters")
    return struct.pack("<B", len(raw)) + raw

def _read_text(data: bytes, pos: int) -> Tuple[str, int]:
    if pos >= len(data):
        raise RecordError("trailer record truncated before a string")
    n = data[pos]
    end = pos + 1 + n
    if end > len(data):
        raise RecordError("trailer record truncated inside a string")
    return data[pos + 1:end].decode("ascii"), end

def _unpack(fmt: str, data: bytes, pos: int, what: str) -> Tuple[Any, int]:
    """One little-endian value of struct format `fmt` at `pos`."""
    st = struct.Struct("<" + fmt)
    if pos + st.size > len(data):
        raise RecordError(f"{what} record truncated")
    return st.unpack_from(data, pos)[0], pos + st.size

def _fixed(value: Any, what: str) -> int:
    """A number as x1000 fixed point."""
    return _u(round(float(value) * 1000), 32, what)

def _number(fixed: int):
    """A x1000 fixed-point value back to the number the yaml carried: an
    integer when it was one."""
    return fixed // 1000 if fixed % 1000 == 0 else fixed / 1000

def _table(entries, what: str) -> bytes:
    """A small table by name: count u8, then len u8 + name and value u16."""
    items = list(entries)
    buf = bytearray(struct.pack("<B", _u(len(items), 8, f"{what} entries")))
    for name, value in items:
        buf += _text(str(name), MAX_NAME, f"{what} key")
        buf += struct.pack("<H", _u(value, 16, f"{what} {name}"))
    return bytes(buf)

def _read_table(data: bytes, pos: int, what: str) -> Tuple[Dict[str, int], int]:
    if pos >= len(data):
        raise RecordError(f"{what} record truncated")
    count = data[pos]
    pos += 1
    table: Dict[str, int] = {}
    for _ in range(count):
        name, pos = _read_text(data, pos)
        table[name], pos = _unpack("H", data, pos, what)
    return table, pos

def _family_name(descriptor: Descriptor) -> str:
    from nxs.cam import chips
    return chips.family_for(descriptor)

def _surface(descriptor: Descriptor):
    from nxs.cam import chips
    return chips.bind(descriptor)

def _register_spec(descriptor: Descriptor, name: str) -> Tuple[int, int, int]:
    spec = descriptor.registers[name]
    order = 1 if str(spec.get("order", "le")) == "be" else 0
    return to_int(spec["addr"]), int(spec.get("width", 1)), order

def _unit_modes(descriptor: Descriptor) -> List[str]:
    """The modes the unit's program offers, in descriptor order (the
    descriptor's own rule: every mode not flagged `host_only`). Their
    indices are the `mode` param values."""
    return descriptor.program_modes()

def _register(addr: int, width: int, order: int) -> Dict[str, Any]:
    spec: Dict[str, Any] = {"addr": int(addr), "width": int(width)}
    if order:
        spec["order"] = "be"
    return spec
