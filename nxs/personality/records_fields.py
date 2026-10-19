"""The trailer's shared vocabulary: the record ids, the struct layouts, and the primitives every record is written and read through."""

from __future__ import annotations

import struct
import zlib
from typing import Any, Dict, List, Tuple

from nxs._generated_constants import NxsMap, NxsDriverImage
from nxs.cam.contracts import ContractError
from nxs.cam.descriptors import Descriptor, to_int

TrailerRecord = NxsDriverImage.TrailerRecord
MAX_TRAILER_SIZE = NxsDriverImage.MAX_TRAILER_SIZE
#: The record types no compiler writes and every decoder skips (SHIPPED,
#: the shipped points); their numbers stay reserved.
RETIRED_RECORDS = frozenset(TrailerRecord.RETIRED)

IDENTITY = TrailerRecord.IDENTITY
PROGRAM = TrailerRecord.PROGRAM
TRIGGERS = TrailerRecord.TRIGGERS
CAMERA_MAP = TrailerRecord.CAMERA_MAP
PARAMS = TrailerRecord.PARAMS
OUTPUTS = TrailerRecord.OUTPUTS
PORTS = TrailerRecord.PORTS
PRESENTATION = TrailerRecord.PRESENTATION

#: The enum params a cam personality dispatches on, by the name its
#: behaviour class declares them under; the wire carries their indices.
MODE_PARAM = "mode"
TRIGGER_PARAM = "trigger"
#: The run parameters a cam personality stages in physical units and
#: the values it records for the host, by name: the line period and the
#: frame period a run starts the sensor at, the frame length in lines it
#: achieved, and the action a run performs.
ACTION_PARAM = "action"
LINE_TIME_PARAM = "line_time"          # ns
FRAME_PERIOD_PARAM = "frame_period"    # ns
FRAME_LENGTH_PARAM = "frame_length"    # lines, what the run achieved
#: The run parameters the camera map stages for an apply, written under the group hold with
#: the staged frame length; the gain is in the capture stack's unit, `GAIN_FACTOR` units per decibel.
EXPOSURE_PARAM = "exposure"            # lines
GAIN_PARAM = "gain"                    # the capture stack's gain unit
#: What a run does, numbered as `CamAction`: the whole program, a standby, a start from standby,
#: the timing program under standby, the apply under the group hold, or the trigger conversion.
ACTIONS: Dict[str, int] = {"configure": 0, "park": 1, "start": 2, "timing": 3,
                           "apply": 4, "trigger": 5}

#: Law families by wire id; a descriptor naming no family is generic.
FAMILY_IDS: Dict[str, int] = {"generic": 1, "sony_imx": 2, "native": 3}
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

#: The payload cap a record's names inherit from their length fields.
MAX_NAME = 32
_NONE_U16 = 0xFFFF
_NONE_U8 = 0xFF

_MODE_FLAG_DEFAULT = 0x01
_MODE_FLAG_TRIGGERABLE = 0x02
_MODE_FLAG_SERIALIZER_CSI = 0x04
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
#: a delta that is one value or a table, "t" for a table by readout kind,
#: "f" for the fast-trigger chart (the output delay as ns u32 and lines u8,
#: then its wait indices and the exposure's), "m" for the modes' own waits
#: (count u8, then per mode its index u8 and a wait table). A decoder reads
#: the fields it knows and never looks past them, so a field appended here
#: reaches an older tool as bytes it ignores; a field inserted would not.
_LAW_FIELDS = (("inck_hz", "I"), ("integration_offset_us", "n"),
               ("min_integration_lines", "H"), ("gain_max", "I"),
               ("gain_reg_per_db", "g"), ("gain_hcg_reg_min", "I"),
               ("min_fps", "q"), ("captured_waits", "w"),
               ("shs_floor_regs", "i"), ("min_frame_length_delta", "d"),
               ("frame_length_delta_const", "t"), ("shs_floor", "H"),
               ("fast_trigger", "f"), ("mode_waits", "m"))
#: The chart's two register lists, in wire order.
_CHART_REG_LISTS = ("output_delay_regs", "exposure_regs")
_DELTA_SCALAR = 1
_DELTA_TABLE = 2

_MODES_HEAD = struct.Struct("<BB")
_MODE_HEAD = struct.Struct("<BHHBBHBBBBH")
_TRIGGERS_HEAD = struct.Struct("<BB")
_CONTROL_ROW = struct.Struct("<BHBBBIIII")
_LAWS_HEAD = struct.Struct("<BH")
_CAPTURE_HEAD = struct.Struct("<IIBIiiIiHHIIIIIIIB")
#: A capture row: width u16, height u16, bit depth u8, pixel phase u8, line
#: length u32, max fps x1000 u32, min exposure us u32, embedded-data lines
#: u8. A trailer written before the last field carried 15-byte rows; the
#: decoder takes the row size from the record's length.
_CAPTURE_ENTRY = struct.Struct("<HHBBIIIB")
_CAPTURE_ENTRY_V1 = struct.Struct("<HHBBIII")
#: The CAMERA_MAP record, the camera law a pod converts its PARAMS rows
#: with: mode count u8, gain factor u16 (capture units per decibel), the
#: run-value indices in `_CAMERA_MAP_PARAM_NAMES` order (0xFF absent), the
#: law head (gain min / max / step in capture units u16, frame length max
#: u32, exposure min u32, exposure margin u16, in lines), one law row per
#: unit mode (line time ns u32, frame length floor u32), then one
#: `ModeEntry` row per unit mode (width u16, height u16, bit depth u8,
#: lanes u8, rate u16, data type u8, flags u8, max fps x1000 u32, mode
#: value u8, pixel phase u8: the `PIXEL_PHASES` index, 0xFF none).
_CAMERA_MAP_HEAD = struct.Struct("<BH")
_CAMERA_MAP_PARAMS = struct.Struct("<11B")
_CAMERA_MAP_LAW = struct.Struct("<HHHIIH")
_CAMERA_MAP_LAW_MODE = struct.Struct("<II")
_CAMERA_MAP_MODE = struct.Struct("<HHBBHBBIBB")
_MODE_ENTRY_FIELDS = ("width", "height", "bit_depth", "lanes", "rate_mbps", "data_type", "flags",
                      "max_fps_milli", "value", "phase")


def mode_entries(rows: bytes) -> List[Dict[str, int]]:
    """The `ModeEntry` rows of a CAMERA_MAP record decoded, one dict per
    mode in the record's order: `width`, `height`, `bit_depth`, `lanes`,
    `rate_mbps`, `data_type`, `flags`, `max_fps_milli`, `value`, `phase`."""
    size = _CAMERA_MAP_MODE.size
    return [dict(zip(_MODE_ENTRY_FIELDS, _CAMERA_MAP_MODE.unpack_from(rows, at)))
            for at in range(0, len(rows) - len(rows) % size, size)]
_CAMERA_MAP_PARAM_NAMES = (ACTION_PARAM, EXPOSURE_PARAM, FRAME_LENGTH_PARAM, GAIN_PARAM,
                           TRIGGER_PARAM, "test_pattern", LINE_TIME_PARAM, MODE_PARAM,
                           FRAME_PERIOD_PARAM, "black_level", "flips")
assert _CAMERA_MAP_PARAMS.size == len(_CAMERA_MAP_PARAM_NAMES)
assert _CAMERA_MAP_MODE.size == NxsMap.MODE_ENTRY_SIZE
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
    from nxs.cam import laws
    return laws.family_for(descriptor)

def _surface(descriptor: Descriptor):
    from nxs.cam import laws
    return laws.bind(descriptor)

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
