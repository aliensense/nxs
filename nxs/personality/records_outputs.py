"""The OUTPUTS record: a personality's output fields as fixed rows, what one
sample record carries. The unit serves the rows as the record carries them at
`MapReg::OUTPUT_ROWS`. The row layout is `OutputField` in
constants/nxs_map.yaml."""

from __future__ import annotations

import dataclasses
import struct
from typing import Any, Dict, List, Sequence

from nxs._generated_constants import NxsDriverImage, NxsMap
from nxs.dsl.fields import FIELD_TYPES
from nxs.personality.records_fields import RecordError, _u

#: One row: name, unit, semantic, type, flags, count, position, two reserved
#: bytes, scale, offset, then the reserved tail.
ROW = struct.Struct("<16s6sHBBHH2xdd16x")
assert ROW.size == NxsMap.ROW_SIZE
_STRING = "string"


@dataclasses.dataclass(frozen=True)
class OutputRow:
    """One output field as its row describes it. `type` is the field type's
    code, `count` the elements of the field or a string's width in bytes,
    and `at` its byte position in the sample record's data. `scale` is the
    effective factor: the base factor times the value of the parameter the
    field links."""

    name: str
    unit: str
    semantic: int
    type: int
    count: int
    at: int
    scale: float
    offset: float = 0.0
    is_big_endian: bool = False
    is_string: bool = False


def field_rows(fields: Sequence[Dict[str, Any]], params: Sequence[Any]) -> List[OutputRow]:
    """The rows of a personality's compiled output fields, in declaration
    order. A field whose scale links a parameter (`scale_param`) carries the
    base scale times that parameter's compiled value, the factor the unit
    serves until the parameter moves."""
    values = {p.name: int(p.current) for p in params}
    rows = []
    for field in fields:
        type_name = field.get("type", "int16")
        scale = float(field.get("scale", 1.0))
        if field.get("scale_param") is not None:
            scale *= values[field["scale_param"]]
        rows.append(OutputRow(
            name=field["name"], unit=field.get("unit") or "",
            semantic=int(field.get("semantic", 0)), type=FIELD_TYPES.index(type_name),
            count=int(field.get("count", 0)) if type_name == _STRING else 1,
            at=int(field["byte_off"]), scale=scale, offset=float(field.get("offset", 0.0)),
            is_big_endian=field.get("byte_order", "big") == "big",
            is_string=type_name == _STRING))
    return rows


def encode_outputs(rows: Sequence[OutputRow], what: str) -> bytes:
    """The OUTPUTS payload: the rows back to back, `MAX_OUTPUT_ROWS` at most."""
    if len(rows) > NxsDriverImage.MAX_OUTPUT_ROWS:
        raise RecordError(f"{what}: {len(rows)} output fields, a personality declares "
                          f"{NxsDriverImage.MAX_OUTPUT_ROWS} at most")
    return b"".join(_encode_row(row, what) for row in rows)


def decode_outputs(payload: bytes) -> List[OutputRow]:
    """The OUTPUTS payload back to its rows."""
    if len(payload) % ROW.size:
        raise RecordError(f"OUTPUTS record of {len(payload)} B is not whole "
                          f"{ROW.size}-byte rows")
    return [_decode_row(payload[pos:pos + ROW.size])
            for pos in range(0, len(payload), ROW.size)]


def _encode_row(row: OutputRow, what: str) -> bytes:
    where = f"{what} {row.name}"
    try:
        name = row.name.encode("ascii")
        unit = row.unit.encode("ascii")
    except UnicodeEncodeError:
        raise RecordError(f"{where}: an output name and its unit are ASCII") from None
    if not name or len(name) >= NxsMap.ROW_NAME_LEN:
        raise RecordError(f"{where}: an output name takes 1 to "
                          f"{NxsMap.ROW_NAME_LEN - 1} characters")
    if len(unit) > NxsMap.ROW_UNIT_LEN:
        raise RecordError(f"{where}: unit {row.unit!r} is longer than "
                          f"{NxsMap.ROW_UNIT_LEN} characters")
    if not 0 <= row.type < len(FIELD_TYPES):
        raise RecordError(f"{where}: field type code {row.type} names no field type")
    flags = ((NxsMap.OUTPUT_FLAG_BIG_ENDIAN if row.is_big_endian else 0)
             | (NxsMap.OUTPUT_FLAG_STRING if row.is_string else 0))
    return ROW.pack(name, unit, _u(row.semantic, 16, f"{where} semantic"), row.type, flags,
                    _u(row.count, 16, f"{where} count"), _u(row.at, 16, f"{where} position"),
                    float(row.scale), float(row.offset))


def _decode_row(raw: bytes) -> OutputRow:
    """One row back, RecordError for a name or unit past ASCII."""
    name, unit, semantic, type_code, flags, count, at, scale, offset = ROW.unpack(raw)
    label = name.rstrip(b"\0")
    try:
        text_name, text_unit = label.decode("ascii"), unit.rstrip(b"\0").decode("ascii")
    except UnicodeDecodeError:
        raise RecordError(f"OUTPUTS row {label!r} carries a byte past ASCII in its name or "
                          f"unit") from None
    return OutputRow(name=text_name, unit=text_unit, semantic=semantic, type=type_code,
                     count=count, at=at, scale=scale, offset=offset,
                     is_big_endian=bool(flags & NxsMap.OUTPUT_FLAG_BIG_ENDIAN),
                     is_string=bool(flags & NxsMap.OUTPUT_FLAG_STRING))
