"""The PARAMS record: a personality's parameters as fixed rows, one format
for a click personality and a cam personality alike. The unit serves the
rows as the record carries them; the camera map serves a camera's rows at
`MapReg::ROWS`, and a smart camera serves the same bytes from its memory.
The row layout is `RowField` in constants/nxs_map.yaml. Each row carries
how it reads, which the PRESENTATION record beside it holds
(`records_present`)."""

from __future__ import annotations

import dataclasses
import math
import struct
from typing import Any, Dict, List, Optional, Sequence, Tuple

from nxs._generated_constants import NxsMap, NxsDriverImage
from nxs.dsl.fields import STREAM_LABELS
from nxs.personality.records_fields import LINE_TIME_PARAM, RecordError, _u
from nxs.personality.records_present import RowPresentation, encode_presentation

ParamId = NxsMap.ParamId
ParamKind = NxsDriverImage.ParamKind
#: A compiled parameter's kind by the name the DSL gives it: `ParamKind` in lower case.
_KINDS = {name.lower(): number for number, name in ParamKind._NAMES.items()}

#: One row: name, unit, id, type, kind, flags, count, default, values.
ROW = struct.Struct("<16s6sHBBBBI32s")
assert ROW.size == NxsMap.ROW_SIZE
_ENUM_VALUE = struct.Struct("<H")
_WIDE_VALUE = struct.Struct("<I")
_RANGE = struct.Struct("<III")
#: Sign-extend a u32 cell of a signed row.
_SIGN_BIT = 1 << 31


@dataclasses.dataclass(frozen=True)
class ParamRow:
    """One parameter as the row describes it. An enum row lists its values;
    a range row's `values` are its min, max and step. `present` is how the
    row reads, its own unit at exponent 0 when None; a decoded row has
    none, the PRESENTATION record holds it."""

    name: str
    unit: str
    id: int
    kind: int
    values: Tuple[int, ...]
    default: int
    is_range: bool
    is_signed: bool = False
    is_read_only: bool = False
    is_set_by_trigger: bool = False
    is_graph: bool = False
    present: Optional[RowPresentation] = dataclasses.field(default=None, compare=False)


def encode_rows(rows: Sequence[ParamRow], what: str) -> bytes:
    """The PARAMS payload: the rows back to back, `MAX_PARAMS` at most, the
    rows a unit serves; the camera map's layout holds `MAX_ROWS`. A name
    names one row, and a standard id is one row's: a host sets a row by its
    name and maps an id to its one control."""
    if len(rows) > NxsDriverImage.MAX_PARAMS:
        raise RecordError(f"{what}: {len(rows)} parameters, a personality declares "
                          f"{NxsDriverImage.MAX_PARAMS} at most")
    for label, keys in (("rows named", [row.name for row in rows]),
                        ("rows of id", [ParamId._NAMES.get(row.id, row.id) for row in rows
                                        if row.id != ParamId.CUSTOM])):
        twice = sorted({str(key) for key in keys if keys.count(key) > 1})
        if twice:
            raise RecordError(f"{what}: two {label} {', '.join(twice)}")
    return b"".join(_encode_row(row, what) for row in rows)


def decode_rows(payload: bytes) -> List[ParamRow]:
    """The PARAMS payload back to its rows."""
    if len(payload) % ROW.size:
        raise RecordError(f"PARAMS record of {len(payload)} B is not whole "
                          f"{ROW.size}-byte rows")
    return [_decode_row(payload[pos:pos + ROW.size])
            for pos in range(0, len(payload), ROW.size)]


def click_rows(params: Sequence[Any], what: str) -> List[ParamRow]:
    """The rows of compiled parameters in table order: a sampling
    personality's whole table, so row i is parameter i, or a hub or host
    program's public parameters. A row carries the standard id its parameter
    declares, and the one GATE row, the stream row the compiler adds, is
    `ParamId.STREAM`. A row reads in its parameter's `show`, else in its own
    unit at exponent 0, with its parameter's value names."""
    rows = []
    for p in params:
        is_range = p.param_type == "range"
        values = [int(v) for v in p.values]
        if is_range:
            values = values[:2] + [values[2] if len(values) > 2 else 1]
        kind = _KINDS.get(p.kind, ParamKind.RUN)
        pid = ParamId.STREAM if kind == ParamKind.GATE else int(p.id)
        present = RowPresentation(unit=p.show.unit if p.show else (p.unit or ""),
                                  exponent=p.show.exponent if p.show else 0, labels=p.labels)
        rows.append(ParamRow(name=p.name, unit=p.unit or "", id=pid, kind=kind,
                             values=tuple(values), default=int(p.default), is_range=is_range,
                             present=present))
    return rows


def encode_presentation_rows(rows: Sequence[ParamRow], what: str) -> bytes:
    """The PRESENTATION payload of `rows`, an entry per row in row order:
    each row's presentation, else its own unit at exponent 0. RecordError
    for a row whose names do not count its values."""
    entries = []
    for row in rows:
        present = row.present or RowPresentation(unit=row.unit, exponent=0)
        if present.labels is not None and len(present.labels) != _value_count(row):
            raise RecordError(f"{what} {row.name}: {len(present.labels)} names for "
                              f"{_value_count(row)} values")
        entries.append(present)
    return encode_presentation(entries, f"{what} presentation")


def camera_rows(descriptor, law, staging: Dict[str, Optional[int]], triggers: Sequence[int],
                free_run: int, default_mode: int, mode_count: int,
                line_span: Optional[Sequence[int]] = None,
                trigger_names: Optional[Dict[int, str]] = None,
                mode_names: Optional[Sequence[str]] = None,
                inck_hz: Optional[int] = None) -> List[ParamRow]:
    """The public rows of a cam personality, from its apply law: the
    exposure, the rate and the gain always; the black level and the flips
    when the law names their register or the head serves them; the
    trigger when the sensor takes one and the program converts it; the
    stream gate when the law has one; the mode when the program stages a
    mode switch, and the graph's line with it when the program stages a
    line time, over `line_span`, the declared line's `[min, max]` in
    nanoseconds; the temperature when the law reads it. A range row's
    span is the widest over the modes: the device paints each mode's own
    span in `MIN` and `MAX`. A native head's exposure and the frame
    periods its rate spans are the microseconds its facts state, which no
    line rounds. `free_run` is the trigger value that stops a conversion.
    Every row carries how it reads: the exposure in microseconds, the rate
    in frames per second, the gain in decibels, the temperature in
    degrees, the trigger by the conversion `trigger_names` names for each
    value, the mode by `mode_names` in table order and the stream by
    `STREAM_LABELS`. `inck_hz`, the clock the head runs from, is a
    read-only row in hertz shown in megahertz."""
    what = f"{descriptor.name} parameters"
    lines = law.line_times_ns
    floors = (law.frame_length_floors if len(law.frame_length_floors) == len(lines)
              else (law.frame_length_min,) * len(lines))
    capture = descriptor.raw("capture") or {}
    sync = descriptor.raw("sync") or {}
    pulse = ((descriptor.raw("trigger") or {}).get("timing") or {}).get("pulse_width_is_exposure")
    takes_trigger = bool(sync.get("takes_trigger")) and bool(triggers) and \
        staging["trigger"] is not None
    rows: List[ParamRow] = []

    if law.frame_periods_min_us:
        period_min = min(law.frame_periods_min_us)
        period_max = law.frame_period_max_us
        period_default = law.frame_periods_min_us[default_mode]
        exposure_min, margin_us = law.exposure_min_us, law.exposure_margin_us
        exposure_max = max(0, period_max - margin_us)
        frame_exposure = period_default - margin_us
    else:
        # The default mode's floor is a minimum, so it rounds up as the span's does.
        periods = [floor * line for floor, line in zip(floors, lines)]
        period_min = math.ceil(min(periods) / 1000)
        period_max = _u32(law.frame_length_max * max(lines) // 1000)
        period_default = math.ceil(periods[default_mode] / 1000)
        exposure_min = math.ceil(law.exposure_min * min(lines) / 1000)
        exposure_max = _u32(max(0, law.frame_length_max - law.exposure_margin) * max(lines) // 1000)
        # The device's rounding: the frame's lines less the margin, rounded down.
        frame_exposure = (max(0, floors[default_mode] - law.exposure_margin)
                          * lines[default_mode] // 1000)
    # The default is the declared one held to the exposure the default row's shortest frame
    # (the one a configure runs) leaves past the margin, so the configure applies it as it is.
    default_us = int((capture.get("exposure") or {}).get("default_us", exposure_min))
    default_us = max(exposure_min, min(default_us, frame_exposure))
    rows.append(_range("exposure", "us", ParamId.EXPOSURE_US, ParamKind.APPLY,
                       exposure_min, exposure_max, 1, default_us, what,
                       set_by_trigger=bool(pulse) and takes_trigger))
    # The rate row: the longest frame is the lowest rate, in milli-frames per second.
    rows.append(_range("fps", "mfps", ParamId.FRAME_RATE_MFPS, ParamKind.APPLY,
                       _mfps(period_max), _mfps(period_min), 1, _mfps(period_default), what,
                       present=RowPresentation("fps", -3)))

    def mdb(units: int) -> int:
        return round(int(units) * 1000 / law.gain_factor)

    gain_default = max(0, int((capture.get("gain") or {}).get("default", law.gain_min)))
    rows.append(_range("gain", "mdB", ParamId.GAIN_MDB, ParamKind.APPLY,
                       mdb(law.gain_min), mdb(law.gain_max), mdb(law.gain_step),
                       mdb(gain_default), what, present=RowPresentation("dB", -3)))

    if ((law.black_level is not None and staging["black_level"] is not None)
            or "black_level" in law.serves):
        rows.append(_range("black_level", "DN", ParamId.BLACK_LEVEL_DN, ParamKind.APPLY,
                           law.black_level_min, law.black_level_max, 1, law.black_level_min,
                           what))
    if (law.flips is not None and staging["flips"] is not None) or "flips" in law.serves:
        flips = NxsMap.FLIP_X | NxsMap.FLIP_Y
        rows.append(_enum("flips", ParamId.FLIPS, ParamKind.APPLY, range(flips + 1), 0, what))
    if takes_trigger:
        names = tuple((trigger_names or {}).get(int(v), str(v)) for v in triggers)
        rows.append(_enum("trigger", ParamId.TRIGGER, ParamKind.TRIGGER, triggers, free_run,
                          what, labels=names))
    if law.stream_gate:
        rows.append(_enum("stream", ParamId.STREAM, ParamKind.GATE, (0, 1), 0, what,
                          labels=STREAM_LABELS))
    if staging["mode"] is not None and staging["frame_period"] is not None:
        rows.append(_enum("mode", ParamId.MODE, ParamKind.RUN, range(mode_count), default_mode,
                          what, labels=None if mode_names is None else tuple(mode_names)))
        # The graph's line: a pair's longest line, which a configure stages
        # over the mode's own. The graph's owner sets it from the rig's facts.
        if staging["line_time"] is not None and line_span:
            graph = _range(LINE_TIME_PARAM, "ns", ParamId.CUSTOM, ParamKind.RUN,
                           int(line_span[0]), int(line_span[-1]), 1, lines[default_mode], what)
            rows.append(dataclasses.replace(graph, is_graph=True))
    if law.temperature is not None or "temperature" in law.serves:
        rows.append(ParamRow(name="temperature", unit="dC", id=ParamId.TEMPERATURE_DC,
                             kind=ParamKind.APPLY, values=(-32768, 32767, 1),
                             default=_signed(NxsMap.TEMPERATURE_NONE, True), is_range=True,
                             is_signed=True, is_read_only=True,
                             present=RowPresentation("C", -1)))
    if inck_hz is not None:
        rows.append(ParamRow(name="inck_hz", unit="Hz", id=ParamId.CUSTOM, kind=ParamKind.RUN,
                             values=(int(inck_hz),), default=int(inck_hz), is_range=False,
                             is_read_only=True, present=RowPresentation("MHz", -6)))
    return rows


def _range(name: str, unit: str, pid: int, kind: int, lo: int, hi: int, step: int,
           default: int, what: str, set_by_trigger: bool = False,
           present: Optional[RowPresentation] = None) -> ParamRow:
    if not lo <= hi:
        raise RecordError(f"{what}: {name} spans {lo}..{hi}, an empty range")
    if not lo <= default <= hi:
        raise RecordError(f"{what}: {name} defaults to {default} {unit}, outside "
                          f"{lo}..{hi} {unit}")
    return ParamRow(name=name, unit=unit, id=pid, kind=kind, values=(lo, hi, max(1, step)),
                    default=default, is_range=True, is_set_by_trigger=set_by_trigger,
                    present=present or RowPresentation(unit, 0))


def _enum(name: str, pid: int, kind: int, values, default: int, what: str,
          labels: Optional[Tuple[str, ...]] = None) -> ParamRow:
    values = tuple(int(v) for v in values)
    if default not in values:
        raise RecordError(f"{what}: {name} defaults to {default}, not one of {list(values)}")
    return ParamRow(name=name, unit="", id=pid, kind=kind, values=values, default=default,
                    is_range=False, present=RowPresentation("", 0, labels))


def _u32(value: int) -> int:
    return min(int(value), 0xFFFFFFFF)


def _mfps(period_us: int) -> int:
    """The frame rate in milli-frames per second of a frame period in microseconds, rounded."""
    return round(1_000_000_000 / int(period_us))


def _value_count(row: ParamRow) -> int:
    """The values a row takes: an enum's count, a range's from its min to
    its max by its step."""
    if row.is_range:
        lo, hi, step = row.values
        return (hi - lo) // max(1, step) + 1
    return len(row.values)


def _encode_row(row: ParamRow, what: str) -> bytes:
    where = f"{what} {row.name}"
    try:
        name = row.name.encode("ascii")
        unit = row.unit.encode("ascii")
    except UnicodeEncodeError:
        raise RecordError(f"{where}: a parameter name and its unit are ASCII")
    if any(c < 0x20 or c > 0x7E for c in name + unit):
        raise RecordError(f"{where}: a parameter name and its unit are printable ASCII")
    # The unit map's parameter view keeps 15 characters and the NUL, so a
    # name both doors serve whole leaves the row's last byte NUL.
    if not name or len(name) >= NxsMap.ROW_NAME_LEN:
        raise RecordError(f"{where}: a parameter name takes 1 to "
                          f"{NxsMap.ROW_NAME_LEN - 1} characters")
    if len(unit) > NxsMap.ROW_UNIT_LEN:
        raise RecordError(f"{where}: unit {row.unit!r} is longer than "
                          f"{NxsMap.ROW_UNIT_LEN} characters")
    row_type = ((NxsMap.ROW_TYPE_RANGE if row.is_range else 0)
                | (NxsMap.ROW_TYPE_SIGNED if row.is_signed else 0))
    flags = ((NxsMap.ROW_FLAG_READ_ONLY if row.is_read_only else 0)
             | (NxsMap.ROW_FLAG_SET_BY_TRIGGER if row.is_set_by_trigger else 0)
             | (NxsMap.ROW_FLAG_GRAPH if row.is_graph else 0))
    if row.is_range:
        lo, hi, step = (_cell(v, row.is_signed, where) for v in row.values)
        values = _RANGE.pack(lo, hi, step)
        count = len(row.values)
    else:
        # Values past 16 bits, a baud rate, take the wide form: half as many.
        wide = any(int(v) > 0xFFFF for v in row.values)
        limit = NxsMap.ROW_WIDE_VALUES_MAX if wide else NxsMap.ROW_VALUES_MAX
        if len(row.values) > limit:
            raise RecordError(f"{where}: {len(row.values)} values, a row lists {limit} at most"
                              + (" when one passes 16 bits" if wide else ""))
        packer = _WIDE_VALUE if wide else _ENUM_VALUE
        values = b"".join(packer.pack(_u(v, 32, f"{where} value")) for v in row.values)
        row_type |= NxsMap.ROW_TYPE_WIDE if wide else 0
        count = len(row.values)
    return ROW.pack(name, unit, _u(row.id, 16, f"{where} id"), row_type, int(row.kind), flags,
                    count, _cell(row.default, row.is_signed, where), values)


def _decode_row(raw: bytes) -> ParamRow:
    """One row back, RecordError for a row the format cannot hold: a name or
    unit past ASCII, a range that does not list 3 values, an enum past its
    value count."""
    name, unit, pid, row_type, kind, flags, count, default, values = ROW.unpack(raw)
    label = name.rstrip(b"\0")
    try:
        text_name, text_unit = label.decode("ascii"), unit.rstrip(b"\0").decode("ascii")
    except UnicodeDecodeError:
        raise RecordError(f"PARAMS row {label!r} carries a byte past ASCII in its name or "
                          f"unit") from None
    is_range = bool(row_type & NxsMap.ROW_TYPE_RANGE)
    is_signed = bool(row_type & NxsMap.ROW_TYPE_SIGNED)
    if is_range:
        if count != 3:
            raise RecordError(f"PARAMS row {label!r} counts {count} values, a range lists 3")
        decoded = tuple(_signed(v, is_signed) for v in _RANGE.unpack_from(values))
    else:
        wide = bool(row_type & NxsMap.ROW_TYPE_WIDE)
        limit = NxsMap.ROW_WIDE_VALUES_MAX if wide else NxsMap.ROW_VALUES_MAX
        if count > limit:
            raise RecordError(f"PARAMS row lists {count} values, past {limit}")
        packer = _WIDE_VALUE if wide else _ENUM_VALUE
        decoded = tuple(packer.unpack_from(values, packer.size * i)[0] for i in range(count))
    return ParamRow(name=text_name, unit=text_unit, id=pid, kind=kind,
                    values=decoded, default=_signed(default, is_signed), is_range=is_range,
                    is_signed=is_signed,
                    is_read_only=bool(flags & NxsMap.ROW_FLAG_READ_ONLY),
                    is_set_by_trigger=bool(flags & NxsMap.ROW_FLAG_SET_BY_TRIGGER),
                    is_graph=bool(flags & NxsMap.ROW_FLAG_GRAPH))


def _cell(value: int, signed: bool, what: str) -> int:
    v = int(value)
    if signed:
        if not -_SIGN_BIT <= v < _SIGN_BIT:
            raise RecordError(f"{what}: {v} does not fit a signed 32-bit cell")
        return v & 0xFFFFFFFF
    return _u(v, 32, what)


def _signed(value: int, signed: bool) -> int:
    return value - (1 << 32) if signed and value & _SIGN_BIT else value
