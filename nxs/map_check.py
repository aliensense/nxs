# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The conformance checks `nxs <unit> map check` runs: a unit's map at both
its doors against the unit's own records, and a smart camera's capture
cells at its camera door against the installed personality whose rows it
serves (Interface Description §4, §6.4, §9)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from nxs._generated_constants import NxsMap
from nxs._libnxs_map import (COMMIT_KINDS, ERRNO_EINVAL, OWNER_SHIFT, Door, MapRefused, MapReg,
                             MapState, ParamId, ParamKind, camera_door_serves, identity_fact,
                             without_owner)
from nxs.personality.records_fields import RecordError
from nxs.personality.records_params import ParamRow, decode_rows

#: The bytes of the chip's serial at `MapReg::SERIAL`, the UID96.
_SERIAL_BYTES = 12


@dataclass
class Check:
    """One conformance check: its name, whether it held, and the fact read."""
    name: str
    ok: bool
    detail: str = ""


@dataclass
class CheckReport:
    """What `map check` found, in order."""
    checks: List[Check] = field(default_factory=list)
    naks: int = 0

    def add(self, name: str, ok: bool, detail: str = "") -> bool:
        self.checks.append(Check(name, bool(ok), detail))
        return bool(ok)

    @property
    def ok(self) -> bool:
        return all(c.ok for c in self.checks)

    def to_dict(self) -> Dict[str, Any]:
        return {"ok": self.ok, "naks": self.naks,
                "checks": [{"name": c.name, "ok": c.ok, "detail": c.detail} for c in self.checks]}


def _camera_rows_at_unit_door(unit_door: Door) -> Tuple[List[int], bytes]:
    """The unit door's indices of the camera personality's rows, the owner
    of its mode row's, and those rows' bytes as the door serves them."""
    raw = unit_door.raw_rows()
    rows = decode_rows(raw)
    mode = next((i for i, r in enumerate(rows) if r.id == ParamId.MODE), None)
    if mode is None:
        return [], b""
    flags = NxsMap.RowField.FLAGS
    owner = raw[mode * NxsMap.ROW_SIZE + flags] >> OWNER_SHIFT
    indices = [i for i in range(len(rows)) if raw[i * NxsMap.ROW_SIZE + flags] >> OWNER_SHIFT == owner]
    return indices, b"".join(raw[i * NxsMap.ROW_SIZE:(i + 1) * NxsMap.ROW_SIZE] for i in indices)


def _in_range(row: ParamRow, cell: Dict[str, int]) -> int:
    """A value the row takes in the mode in effect: an enum's default, a
    range's default held inside the range on its step grid, and the
    rate's fastest, the shortest frame, so the check never asks for a long
    frame."""
    if not row.is_range:
        return row.default
    low, high, step = cell["min"], cell["max"], max(1, cell["step"])
    if row.id == ParamId.FRAME_RATE_MFPS:
        return high
    value = min(max(row.default, low), high)

    return low + (value - low) // step * step


def _tolerance(row: ParamRow, cell: Dict[str, int], line_us: int) -> int:
    """How far `LIVE` may read from a staged value: a line for the
    exposure, which the sensor counts in lines, the step for another range,
    nothing for an enum."""
    if row.id == ParamId.EXPOSURE_US:
        return line_us
    return max(1, cell["step"]) if row.is_range else 0


def _reads_back(row: ParamRow, cell: Dict[str, int], value: int, live: int, line_us: int) -> bool:
    """Whether `LIVE` reads a staged value back: the rate within a line and
    a microsecond of the frame period it asks for, since the sensor counts
    the frame in lines; another row within its tolerance."""
    if row.id == ParamId.FRAME_RATE_MFPS:
        return value > 0 and live > 0 and abs(1e9 / live - 1e9 / value) <= line_us + 1
    return abs(live - value) <= _tolerance(row, cell, line_us)


def _index(rows: Sequence[ParamRow], pid: int) -> Optional[int]:
    return next((i for i, r in enumerate(rows) if r.id == pid), None)


def _valued(value: int, row: ParamRow) -> str:
    """A value with the row's unit, as a check line names it."""
    return f"{value} {row.unit}" if row.unit else str(value)


def _named(state: int) -> str:
    """A `STATE` value by its name, as a check line names it."""
    return MapState._NAMES.get(int(state), str(state))


def check(unit, cam_door: Door, unit_door: Optional[Door] = None) -> CheckReport:
    """The conformance checks over a unit's open client and its two doors:
    the unit door's identity, the records of the camera slot the doors
    serve, the camera door's identity, rows and mode table against those
    records and the unit door's camera rows against the camera door's, one
    RUN of the default row through the unit door, then every row the camera
    door commits: each in range, each range past its maximum, the refusals
    the contract names, the stream gate and the mode switch where the rows
    offer them, the run times those commits painted, the camera door's
    values as the unit door reads them, and the camera door's zeros where
    the unit door alone serves. `unit` is None for a smart camera, which
    answers no unit door: its records are those of the installed
    personality whose rows and mode table its door serves, and its run is
    the default row committed from standby at that door. The checks stop at
    a map whose identity or layout this tool does not speak, at rows that
    do not decode, at a run that does not end READY or a default row the
    records do not carry, and at a commit the map refuses."""
    from nxs.cam import smart_camera
    from nxs.image import parse_trailer
    from nxs.personality.records import camera_map_info, camera_map_mode_rows
    from nxs.personality.records_fields import MODE_PARAM, PARAMS

    report = CheckReport()
    if unit is not None:
        report.add("identity", True, "the unit answers at its unit door")
        version = unit.interface_version()
        report.add("proto_version", version == NxsMap.PROTO_VERSION_VALUE, f"PROTO_VERSION {version}")
        slot, line = _served_slot(unit)
        if not report.add("records", slot is not None, line):
            return report
        try:
            records = parse_trailer(unit.read_personality_info(slot))
        except ValueError as exc:
            report.add("camera_map_record", False, f"slot {slot}: {exc}")
            return report
    else:
        answered = smart_camera.answers(cam_door)
        fact = None if answered else (smart_camera.foreign_layout(cam_door)
                                      or f"no camera door answers at {cam_door.address:#04x}")
        if not report.add("identity", answered,
                          "no unit door, the camera door answers: a smart camera" if answered
                          else f"no unit door, and {fact}"):
            return report
        described = smart_camera.described_by(cam_door)
        if not report.add("records", described is not None,
                          f"the rows of {described[0]}" if described
                          else "no installed cam personality serves its rows"):
            return report
        records = described[1]
    by_type = {int(t): bytes(p) for t, p in records}
    try:
        info = camera_map_info(records)
    except RecordError as exc:
        report.add("camera_map_record", False, str(exc))
        return report
    if not report.add("camera_map_record", info is not None and PARAMS in by_type,
                      "the personality carries the PARAMS rows and the CAMERA_MAP law"):
        return report
    boot = cam_door.header()
    if not report.add("map_id", boot["id"] == NxsMap.MAP_ID_CAMERA
                      and identity_fact(boot["id"], boot["version"]) is None,
                      f"id {boot['id']:#04x} version {boot['version']}"):
        return report
    served = cam_door.raw_rows()
    try:
        rows = decode_rows(served)
    except RecordError as exc:
        report.add("rows", False, f"{boot['param_count']} rows served, which do not decode: {exc}")
        return report
    report.add("rows", without_owner(served) == by_type[PARAMS],
               f"{boot['param_count']} rows served, the records carry {len(info['rows'])}")
    unit_rows: List[int] = []
    if unit_door is not None:
        unit_rows, at_unit = _camera_rows_at_unit_door(unit_door)
        report.add("unit_door_rows", at_unit == served,
                   f"the unit door serves the camera's {len(unit_rows)} rows at "
                   f"{unit_rows[0] if unit_rows else '-'}..{unit_rows[-1] if unit_rows else '-'}")
    report.add("mode_count", boot["mode_count"] == info["mode_count"],
               f"served {boot['mode_count']}, the records say {info['mode_count']}")
    n = info["mode_count"]
    served_modes = cam_door.read(MapReg.MODE_TABLE, NxsMap.MODE_ENTRY_SIZE * n) if n else b""
    report.add("mode_table", served_modes == camera_map_mode_rows(records), "the rows are the records'")
    table = cam_door.mode_table()
    default_row = next((r for r in table if r["flags"] & NxsMap.MODE_FLAG_DEFAULT), table[0] if table else None)
    if default_row is None:
        report.add("run", False, "no mode to run: the map serves no mode table")
        return report
    default_index = table.index(default_row)
    if default_index >= len(info["law_modes"]):
        report.add("run", False, f"the default row is mode-table row {default_index}, past the "
                                 f"{len(info['law_modes'])} modes the records carry")
        return report
    law = info["law_modes"][default_index]
    finished = False
    try:
        if unit_door is None:
            ran = _run_default(report, cam_door, rows, default_row)
        elif info["params"].get(MODE_PARAM) is None:
            # A pod's camera runs by a RUN commit of its mode row alone.
            ran = report.add("run", False, "the camera personality names no mode parameter to run")
        else:
            ran = _run_through_unit_door(report, cam_door, unit_door, rows, unit_rows, default_row)
        if not ran:
            return report
        boot = cam_door.header()
        report.add("ready", boot["state"] == MapState.READY and boot["current_mode"] == default_row["value"],
                   f"STATE {boot['state']}, CURRENT_MODE {boot['current_mode']}")
        line_us = max(1, round(law["line_time_ns"] / 1000))
        _check_rows(report, cam_door, rows, line_us)
        _check_refusals(report, cam_door, rows, default_row["value"])
        _check_gate(report, cam_door, rows, table, default_row)
        _check_run_times(report, cam_door)
        if unit_door is not None:
            _check_both_doors(report, cam_door, unit_door, rows, unit_rows)
        finished = True
    except MapRefused as exc:
        report.add("commit", False, str(exc))
    finally:
        report.naks = cam_door.naks + (unit_door.naks if unit_door is not None else 0)
    if finished:
        report.add("busy_naks", True, f"{report.naks} NAKs answered by the driver's repeat")
    return report


def _served_slot(unit) -> Tuple[Optional[int], str]:
    """The store slot the camera door serves, the first of kind CAMERA as
    the unit runs it, with the line that names it; None with the fact when
    the store holds no cam personality the unit describes."""
    from nxs.cam import unit_source
    from nxs.client import DeviceRefused

    stale = ""
    for slot in range(unit_source.MAX_SLOTS):
        try:
            kind = unit_source.slot_kind(unit, slot)
        except DeviceRefused as exc:
            if exc.code not in unit_source.STALE_PEEKS:
                raise
            stale = stale or f"; slot {slot} holds an image {unit_source.STALE_PEEKS[exc.code]}"
            continue
        if kind == "cam":
            name = getattr(unit.read_slot_info(slot), "name", "") or "a cam personality"
            return slot, f"slot {slot}: {name}"
    return None, f"no cam personality served{stale}"


def _run_through_unit_door(report: CheckReport, cam_door: Door, unit_door: Door,
                           rows: Sequence[ParamRow], unit_rows: Sequence[int],
                           default_row: Dict[str, Any]) -> bool:
    """A pod's run: the stream parked where the camera door takes it, then a
    RUN commit of the default row through the unit door, as the owner of a
    hub's bus brings a pod up. Whether the run ended READY."""
    stream = _index(rows, ParamId.STREAM)
    mode = _index(rows, ParamId.MODE)
    if stream is not None and cam_door.u8(MapReg.STATE) != MapState.INERT and cam_door.u8(MapReg.STREAM):
        cam_door.commit({stream: 0}, ParamKind.GATE)
    if mode is None or not unit_rows:
        return report.add("run", False, "the camera personality serves no mode row to run")
    state, error = unit_door.commit({unit_rows[mode]: default_row["value"]}, ParamKind.RUN)
    return report.add("run", state == MapState.READY,
                      f"row {default_row['value']} through the unit door ended {_named(state)} error {error}")


def _check_run_times(report: CheckReport, cam_door: Door) -> None:
    """The run times the door painted, read after the commits: every kind a
    commit of the check ran reads its run's time, more than 0 ms."""
    boot = cam_door.header()
    painted = {ParamKind.RUN: boot["mode_run_ms"], ParamKind.GATE: boot["gate_run_ms"],
               ParamKind.TRIGGER: boot["trigger_run_ms"]}
    report.add("run_times", all(painted[kind] > 0 for kind in painted if kind in cam_door.kinds_applied),
               f"mode {boot['mode_run_ms']} ms, gate {boot['gate_run_ms']} ms, "
               f"trigger {boot['trigger_run_ms']} ms painted")


def _run_default(report: CheckReport, cam_door: Door, rows: Sequence[ParamRow],
                 default_row: Dict[str, Any]) -> bool:
    """A smart camera's run: the stream parked, then the default row
    committed, as a driver's probe starts it. Whether the run ended READY."""
    stream = _index(rows, ParamId.STREAM)
    mode = _index(rows, ParamId.MODE)
    if stream is not None:
        cam_door.commit({stream: 0}, ParamKind.GATE)
    if mode is None:
        state, error = cam_door.u8(MapReg.STATE), cam_door.u8(MapReg.ERROR)
    else:
        state, error = cam_door.commit({mode: default_row["value"]}, ParamKind.RUN)
    return report.add("run", state == MapState.READY,
                      f"row {default_row['value']} ended {_named(state)} error {error}")


def _check_rows(report: CheckReport, cam_door: Door, rows: Sequence[ParamRow], line_us: int) -> None:
    """Every row the door applies takes a value in range and reads it back
    within its tolerance, and every range but the rate clamps a value past
    its maximum."""
    for i, row in enumerate(rows):
        if row.kind != ParamKind.APPLY or row.is_read_only:
            continue
        cell = cam_door.cells(rows)[i]
        value = _in_range(row, cell)
        before = cam_door.u8(MapReg.APPLY_COUNT)
        state, error = cam_door.commit({i: value}, ParamKind.APPLY)
        live = cam_door.cells(rows)[i]["live"]
        report.add(f"apply_{row.name}", state == MapState.READY
                   and cam_door.u8(MapReg.APPLY_COUNT) == (before + 1) & 0xFF
                   and _reads_back(row, cell, value, live, line_us),
                   f"{_valued(value, row)} read {live}, ended {state} error {error}")
        if not row.is_range or row.is_signed or row.id == ParamId.FRAME_RATE_MFPS:
            continue
        cell = cam_door.cells(rows)[i]
        # Past the tolerance too, so an unclamped value reads as one.
        past = cell["max"] + max(cell["step"], 2 * _tolerance(row, cell, line_us))
        if past > 0xFFFFFFFF:
            continue
        state, error = cam_door.commit({i: past}, ParamKind.APPLY)
        live = cam_door.cells(rows)[i]["live"]
        report.add(f"clamp_{row.name}", state == MapState.READY
                   and live <= cell["max"] + _tolerance(row, cell, line_us),
                   f"{_valued(past, row)} read {live}, served max {cell['max']}, ended {state}")


def _check_refusals(report: CheckReport, cam_door: Door, rows: Sequence[ParamRow],
                    mode_value: int) -> None:
    """The commits the contract refuses with `EINVAL`: two kinds in one mask,
    a read-only row, an enum value off its list, a row past `PARAM_COUNT`,
    and a mode switch while streaming, the stream started for it from
    standby."""
    def refused(name: str, staged: Mapping[int, int], kind: int) -> None:
        state, error = cam_door.commit(staged, kind)
        report.add(name, state == MapState.FAULT and error == ERRNO_EINVAL, f"ended {state} error {error}")

    by_kind: Dict[int, int] = {}
    for i, row in enumerate(rows):
        if row.kind in COMMIT_KINDS and not row.is_read_only:
            by_kind.setdefault(int(row.kind), i)
    stream = _index(rows, ParamId.STREAM)
    if ParamKind.APPLY in by_kind and stream is not None:
        apply_row = by_kind[ParamKind.APPLY]
        refused("mixed_mask", {apply_row: rows[apply_row].default, stream: 0}, ParamKind.APPLY)
    read_only = next((i for i, r in enumerate(rows) if r.is_read_only), None)
    if read_only is not None:
        refused("read_only_row", {read_only: rows[read_only].default}, rows[read_only].kind)
    enum = next((i for i, r in enumerate(rows) if not r.is_range and r.kind in COMMIT_KINDS
                 and not r.is_read_only), None)
    if enum is not None:
        refused("enum_off_list", {enum: max(rows[enum].values) + 1}, rows[enum].kind)
    if len(rows) < NxsMap.MAX_ROWS:
        refused("unserved_row", {len(rows): 0}, ParamKind.APPLY)
    mode = _index(rows, ParamId.MODE)
    if mode is not None and stream is not None:
        if not cam_door.u8(MapReg.STREAM):
            cam_door.commit({stream: 1}, ParamKind.GATE)
        refused("mode_while_streaming", {mode: mode_value}, ParamKind.RUN)


def _check_gate(report: CheckReport, cam_door: Door, rows: Sequence[ParamRow],
                table: Sequence[Dict[str, Any]], default_row: Dict[str, Any]) -> None:
    """The stream gate parks and starts, and the mode switch takes another
    row in standby and comes back, where the rows offer them."""
    stream = _index(rows, ParamId.STREAM)
    mode = _index(rows, ParamId.MODE)
    if stream is None:
        return
    state, _error = cam_door.commit({stream: 0}, ParamKind.GATE)
    report.add("stream_off", state == MapState.READY and cam_door.u8(MapReg.STREAM) == 0,
               f"ended {state} in {cam_door.last_commit_ms} ms, STREAM {cam_door.u8(MapReg.STREAM)}")
    if mode is not None and len(table) > 1:
        other = next(r for r in table if r is not default_row)
        state, error = cam_door.commit({mode: other["value"]}, ParamKind.RUN)
        report.add("mode_switch", state == MapState.READY and cam_door.u8(MapReg.CURRENT_MODE) == other["value"],
                   f"row {other['value']}: ended {state} error {error} in {cam_door.last_commit_ms} ms, "
                   f"CURRENT_MODE {cam_door.u8(MapReg.CURRENT_MODE)}")
        # A configure may end streaming: the switch back starts from standby.
        if cam_door.u8(MapReg.STREAM):
            cam_door.commit({stream: 0}, ParamKind.GATE)
        state, error = cam_door.commit({mode: default_row["value"]}, ParamKind.RUN)
        report.add("mode_back", state == MapState.READY and cam_door.u8(MapReg.CURRENT_MODE) == default_row["value"],
                   f"row {default_row['value']}: ended {state} error {error} in {cam_door.last_commit_ms} ms")
    state, _error = cam_door.commit({stream: 1}, ParamKind.GATE)
    on = cam_door.u8(MapReg.STREAM)
    state, _error = cam_door.commit({stream: 0}, ParamKind.GATE)
    report.add("stream_on_off", on == 1 and state == MapState.READY and cam_door.u8(MapReg.STREAM) == 0,
               f"STREAM {on} then {cam_door.u8(MapReg.STREAM)}")


def _check_both_doors(report: CheckReport, cam_door: Door, unit_door: Door,
                      rows: Sequence[ParamRow], unit_rows: Sequence[int]) -> None:
    """The two doors are one map: the unit door reads the camera door's
    values in effect at its own indices of the camera's rows, and the
    camera door reads 0 where the unit door alone serves."""
    at_camera = [cell["live"] for cell in cam_door.cells(rows)]
    unit_cells = unit_door.cells(unit_door.rows())
    at_unit = [unit_cells[i]["live"] for i in unit_rows] if len(unit_cells) > max(unit_rows, default=-1) else []
    report.add("both_doors", at_unit == at_camera,
               f"LIVE {at_camera} at the camera door, {at_unit} at the unit door")
    # The header and the chip's serial: cells a unit always paints.
    header = cam_door.read(MapReg.MAP_ID, MapReg.LIVE)
    serial = cam_door.read(MapReg.SERIAL, _SERIAL_BYTES)
    served = [(at, byte) for at, byte in enumerate(header) if byte and not camera_door_serves(at)]
    served += [(MapReg.SERIAL + i, byte) for i, byte in enumerate(serial) if byte]
    report.add("camera_door_filter", not served,
               "the camera door reads 0 in the unit's cells" if not served
               else f"the camera door serves {len(served)} bytes of the unit's cells, "
                    f"{served[0][1]:#04x} at {served[0][0]:#06x}")
