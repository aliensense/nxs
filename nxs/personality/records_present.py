"""The PRESENTATION record: how a block's PARAMS rows read, one
`PRESENT_ROW_SIZE` entry per row in row order, then the names of the rows
that carry names. A host prints a row's value from this record alone: a
shown value is the served value times ten to the entry's exponent, in the
entry's unit, and a named row shows the name of its value. The entry
layout is `PresentField` in constants/nxs_map.yaml."""

from __future__ import annotations

import dataclasses
import struct
from typing import List, Optional, Sequence, Tuple

from nxs._generated_constants import NxsDriverImage, NxsMap
from nxs.personality.records_fields import RecordError

#: One entry: the display unit, the decimal exponent, the flags.
ENTRY = struct.Struct("<6sbB")
assert ENTRY.size == NxsMap.PRESENT_ROW_SIZE
#: Bytes of the unit field; a unit may fill it.
_UNIT_LEN = NxsMap.PresentField.EXPONENT - NxsMap.PresentField.UNIT
_EXPONENT_MIN, _EXPONENT_MAX = -128, 127
#: Values a row names at most: the tail's count byte.
_LABELS_MAX = 0xFF


@dataclasses.dataclass(frozen=True)
class RowPresentation:
    """How one row reads: the unit a value shows in, the power of ten a
    shown value is the served value times, and the name of each value of an
    enum in its order or of a range from its min by its step. `labels` is
    None for a row whose values carry no names."""

    unit: str
    exponent: int
    labels: Optional[Tuple[str, ...]] = None


def encode_presentation(rows: Sequence[RowPresentation], what: str = "PRESENTATION") -> bytes:
    """The PRESENTATION payload: an entry per row in row order, `MAX_PARAMS`
    at most, then the names of every named row in the same order, each a
    count byte and a length-prefixed name per value."""
    if len(rows) > NxsDriverImage.MAX_PARAMS:
        raise RecordError(f"{what}: {len(rows)} entries, an image declares "
                          f"{NxsDriverImage.MAX_PARAMS} rows at most")
    entries = bytearray()
    tail = bytearray()
    for i, row in enumerate(rows):
        where = f"{what} row {i}"
        entries += ENTRY.pack(_unit(row.unit, where), _exponent(row.exponent, where),
                              NxsMap.PRESENT_FLAG_LABELS if row.labels is not None else 0)
        if row.labels is not None:
            tail += _encode_labels(row.labels, where)
    return bytes(entries + tail)


def decode_presentation(payload: bytes, row_count: int) -> List[RowPresentation]:
    """The PRESENTATION payload of an image with `row_count` PARAMS rows
    back to its entries. RecordError for a record that does not hold an
    entry per row, a name cut short or past ASCII, or bytes past the last
    name."""
    fixed = row_count * ENTRY.size
    if len(payload) < fixed:
        raise RecordError(f"PRESENTATION record of {len(payload)} B holds no entry for each "
                          f"of {row_count} rows")
    heads = [ENTRY.unpack_from(payload, i * ENTRY.size) for i in range(row_count)]
    pos = fixed
    rows = []
    for i, (unit, exponent, flags) in enumerate(heads):
        labels = None
        if flags & NxsMap.PRESENT_FLAG_LABELS:
            labels, pos = _decode_labels(payload, pos, f"PRESENTATION row {i}")
        rows.append(RowPresentation(unit=_text(unit.rstrip(b"\0"), f"PRESENTATION row {i} unit"),
                                    exponent=exponent, labels=labels))
    if pos != len(payload):
        raise RecordError(f"PRESENTATION record carries {len(payload) - pos} B past the names "
                          f"of its {row_count} rows")
    return rows


def _unit(unit: str, where: str) -> bytes:
    raw = _ascii(unit, f"{where} unit")
    if len(raw) > _UNIT_LEN:
        raise RecordError(f"{where}: unit {unit!r} is longer than {_UNIT_LEN} characters")
    return raw


def _exponent(exponent: int, where: str) -> int:
    value = int(exponent)
    if not _EXPONENT_MIN <= value <= _EXPONENT_MAX:
        raise RecordError(f"{where}: exponent {value} does not fit a signed byte")
    return value


def _encode_labels(labels: Sequence[str], where: str) -> bytes:
    if not 1 <= len(labels) <= _LABELS_MAX:
        raise RecordError(f"{where}: {len(labels)} names, a row names 1 to {_LABELS_MAX} values")
    twice = sorted({label for label in labels if list(labels).count(label) > 1})
    if twice:
        raise RecordError(f"{where}: {', '.join(twice)} names two values, and a name is "
                          f"one value's")
    buf = bytearray(struct.pack("<B", len(labels)))
    for label in labels:
        raw = _ascii(label, f"{where} name")
        if not 1 <= len(raw) <= NxsMap.LABEL_CHARS_MAX:
            raise RecordError(f"{where}: name {label!r} takes 1 to {NxsMap.LABEL_CHARS_MAX} "
                              f"characters")
        buf += struct.pack("<B", len(raw)) + raw
    return bytes(buf)


def _decode_labels(payload: bytes, pos: int, where: str) -> Tuple[Tuple[str, ...], int]:
    if pos >= len(payload):
        raise RecordError(f"{where}: the record ends before its names")
    count = payload[pos]
    pos += 1
    labels = []
    for _ in range(count):
        if pos >= len(payload):
            raise RecordError(f"{where}: the record ends inside its names")
        end = pos + 1 + payload[pos]
        if end > len(payload):
            raise RecordError(f"{where}: the record ends inside a name")
        labels.append(_text(payload[pos + 1:end], f"{where} name"))
        pos = end
    return tuple(labels), pos


def _ascii(text: str, what: str) -> bytes:
    try:
        raw = str(text).encode("ascii")
    except UnicodeEncodeError:
        raise RecordError(f"{what}: {text!r} carries a character past ASCII") from None
    if any(c < 0x20 or c > 0x7E for c in raw):
        raise RecordError(f"{what}: {text!r} is not printable ASCII")
    return raw


def _text(raw: bytes, what: str) -> str:
    try:
        return bytes(raw).decode("ascii")
    except UnicodeDecodeError:
        raise RecordError(f"{what} carries a byte past ASCII") from None
