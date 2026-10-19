# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The descriptor trailer of a cam personality: the records a datasheet
descriptor compiles into, and the descriptor a unit's trailer decodes back
to. The unit stores the trailer opaquely and serves it page by page; the
host reads modes, controls, laws, and capture facts from it and never
needs the hub on the robot.

Record layouts (all integers little-endian; `TrailerRecord` numbers the
types, `IDENTITY` and `PROGRAM` extend that vocabulary above the
firmware's reserved `INSTANCE_CAL` and below `VENDOR_BASE`):

`IDENTITY` (6): the `IDENTITY_SIZE` head every image kind carries, the
`IdentityField` layout (`nxs.personality.records_identity`): the address
u8 (`meta.i2c_addr`), flags u8 (`IDENTITY_FLAG_TAKES_TRIGGER`,
`IDENTITY_FLAG_PULSE_EXPOSURE`: under fast trigger the pulse's low time is
the exposure), the firmware version and the serial (0 and empty in an
image), the compatible and the product. A cam personality appends the
`IDENTITY_CAM_SIZE` tail, the `IdentityCamField` layout: reg_bits u8,
val_bits u8, device_id_reg u16 (0xFFFF none), device_id_width u8,
device_id u16, alive_reg u16 (0xFFFF none), the default mode index u8, six
reserved bytes. A decoded descriptor's name is the model its compatible
names after the vendor.

`MODES` (1): mode_param_index u8 (the index of the image's `mode` enum
param in its parameter table, the VM's parameter index; 0xFF when the
personality dispatches no mode, a single-mode part), count u8, then per
mode in descriptor order: param value u8 (the enum value that selects it,
its index in that order), width u16, height u16, bit_depth u8, lanes u8,
rate_mbps u16, data type u8 (the RAW bit width), flags u8 (bit0 default,
bit1 triggerable, bit2 the personality writes the serializer's CSI block),
embedded_lines u8, sensor_mode u8 (0xFF: no mode declares a capture
index, the host derives it from the table it generates), then
the mode's declared timing facts behind a presence mask u16
(`_TIMING_FIELDS`, in bit order: hmax u16, vmax u32, vmax_clean u32,
vmax_jump_threshold u32, trigger_vmax u32, min_frame_length u32,
framerate_cap num u16 + den u16, exposure_us u32, gain u16, fps x1000
u32, delta_kind len u8 + bytes, vint_mode u8), then name len u8 + bytes
and trigger input len u8 + bytes. A decoded mode's `timing` is the
declared one. The experimental operating points (`_RETIRED_TIMING`: vmax_clean,
vmax_jump_threshold, trigger_vmax, framerate_cap, exposure_us, gain) keep
their bits for the images that carried them: the encoder refuses them (a
personality is compiled from the shipped descriptor), the decoder reads
them past and drops them.

`TRIGGERS` (8): trigger_param_index u8 (the `trigger` enum param's index,
0xFF when the personality declares none), count u8, then per conversion the
personality implements: param value u8 (the `TriggerCode` registry's value
for a preset named there, else the next value past the registry in
declaration order) and the preset name, len u8 + bytes. The records carry no
param names: the tool reads the `mode` and `trigger` semantics from these
indices.

`CONTROLS` (2): count u8, then per kernel row: control id u8
(`CONTROL_IDS`), addr u16, width u8, order u8 (0 little, 1 big), form u8,
p0..p3 u32.

`LAWS` (3): family id u8 (`FAMILY_IDS`), then the family's parameters as
the descriptor's `limits` declare them, behind a presence mask u16
(`_LAW_FIELDS`, in bit order): inck_hz u32, integration_offset_us as ns
u32, min_integration_lines u16, gain_max u32, gain_reg_per_db (form u8:
0 a scalar, 1 a num/den pair; num u32, den u32), gain_hcg_reg_min u32,
min_fps x1000 u32, captured_waits (count u8, then per wait register len
u8 + name and value u16), shs_floor_regs (count u8, then indices u8 into
captured_waits), min_frame_length_delta (form u8: 1 one value u16, 2 a
table by readout kind: count u8, then len u8 + name and value u16),
frame_length_delta_const (a table by readout kind, the same shape),
fast_trigger (the chart: the output delay as ns u32 and lines u8, then
its wait registers and the exposure's as index lists into
captured_waits, count u8 + indices u8 each), mode_waits (count u8, then
per mode its index u8 and a wait table by name, the shape above). A
decoder reads the fields it knows and never looks past them, so a field
appended behind the known bits reaches an older tool as bytes it
ignores. A decoded descriptor's `limits` are the declared ones, a mode's
own waits land back under its `timing`; the generic family declares
inck_hz and min_fps at most, and a native head none: its bounds ride the
CAMERA_MAP record, so its LAWS record names the family alone.

`CAPTURE` (4): mclk_khz u32, pix_clk_hz u32, lane_polarity u8 (0xFF
none), gain (factor u32, min i32, max i32, step u32, default i32), hdr
ratio (min u16, max u16), exposure (factor u32, max_us u32, step u32,
default_us u32), framerate (factor u32, min_fps x1000 u32, step u32),
count u8, then per table entry: width u16, height u16, bit_depth u8,
pixel_phase u8 (`PIXEL_PHASES`), line_length u32, max_fps x1000 u32,
min_exp_us u32, embedded lines u8. A flags u8 may follow the table (bit0:
the sensor gates its clock lane between bursts, `clock_noncontinuous`);
a record without it declares none.

`PROGRAM` (7): the standby-wrapped program settles the family composes
with on the host: flags u8 (bit0 timing_start, bit1 trigger_switch, bit2
fast_trigger, bit3 sync_switch, bit4 restart, bit5 start, bit6 stop_ms,
bit7 hmax_live), timing_start (standby_ms u16, release_ms u16, start_ms
u16, syncsel u8: 0 none, 1 master, 2 slave, repair count u8, then per
repair addr u16, width u8, order u8, value u32; a decoded repair names
its register `REPAIR<n>`, the bytes it writes are the declared ones),
three sleep sets of (standby_ms, release_ms, start_ms) u16, restart
(standby_ms, stop_ms, release_ms, start_ms) u16, start (release_ms,
start_ms) u16, stop_ms u16; a zero settle decodes as absent. A record
longer than that carries the AD-depth block: the ADBIT_MONOSEL register
(addr u16, width u8, order u8), the chromacity u8 (0 colour, 1
monochrome), the INCK the init table is set for u32 (0 undeclared), a
code count u8, then (bit depth u8, ADBIT code u8) per output depth
(`program.adbit`, `program.init_inck_hz`, `meta.chromacity`); longer
still, the black-level block: the BLKLEVEL register (addr u16, width u8,
order u8), a count u8, then (bit depth u8, value u32) per output depth
(`program.blklevel`).

`CAMERA_MAP` (12): the camera law a pod converts its PARAMS rows with
(`constants/nxs_map.yaml`): mode count u8, gain factor u16 (capture
units per decibel), the run-value indices a commit stages (action,
exposure, frame_length, gain, trigger, test_pattern, line_time, mode,
frame_period, black_level, flips; 0xFF absent), the law head (gain min /
max / step in capture units u16, frame length max u32, exposure min u32,
exposure margin u16, in lines), one law row per unit mode (line time ns
u32, frame length floor u32), then one mode-table row per unit mode
(width u16, height u16, bit depth u8, lanes u8, rate u16, data type u8,
flags u8: bit0 default, bit1 triggerable, max fps x1000 u32, the `mode`
value u8, pixel phase u8: the `PIXEL_PHASES` index, 0xFF none). Absent
when the family derives no apply law or the program stages no apply,
which leaves the camera map INERT. A native head's record names no apply
value (exposure, frame length, gain): the head converts a commit in its
own logic.

`PARAMS` (13): a personality's parameters as `ROW_SIZE`-byte rows, the
`RowField` layout (`nxs.personality.records_params`): a click
personality's are its VM parameters in table order, a cam personality's
the public rows the camera map serves, built from the apply law, a hub or
host program's its public parameters. A click image carries it whatever its
trailer; a cam image beside CAMERA_MAP; a hub image only when it declares a
public parameter.

`OUTPUTS` (14): a personality's output fields as `ROW_SIZE`-byte rows, the
`OutputField` layout (`nxs.personality.records_outputs`): name, unit,
semantic, type, flags, count, position, the effective scale and the offset.
A personality that declares output fields carries it; a cam personality
declares none.

`PORTS` (15): the ports a personality's device offers as
`PORT_ROW_SIZE`-byte rows, the `PortField` layout
(`nxs.personality.records_ports`): name, type and direction. The compiler
writes it for every image, a cam image's after the records above.

`GRAPH_SOURCE` (16), `GRAPH_LINKS` (17), `GRAPH_OUTPUTS` (18) and
`GRAPH_WALKS` (19) are a graph image's records, and `HUB_PHASES` (20) names
a hub program's phases (`nxs.personality.records_graph`).

`PRESENTATION` (21): how the PARAMS rows read, one `PRESENT_ROW_SIZE` entry
per row in row order, the `PresentField` layout
(`nxs.personality.records_present`): the display unit, the decimal exponent
a shown value is the served value times, and the flags. Then, for every row
flagged `PRESENT_FLAG_LABELS` in row order, a count u8 and a name per value
as len u8 + ASCII, `LABEL_CHARS_MAX` characters at most: an enum's in its
value order, a range's from its min by its step. Every image that carries
PARAMS carries it beside them.

`SHIPPED` (9) is retired (`RETIRED_RECORDS`): no compiler writes it, the
decoder skips it whatever it carries, and its number stays reserved.
`RUN_PARAMS` (11) is written by no compiler and read by no tool: a run
stages its values through the graph image's walks and the camera door's
commits, and an image that carries one keeps it as a record skipped.

`STATUS` (10): the status probes `status` reads and the frame oracle
judges. A count u8, then per probe: name len u8 + bytes, addr u16, width
u8, order u8, flags u8 (bit0 the value prints as an integer, bit1 a
non-zero value warns, bit2 a mask follows, bit3 an expected value
follows), mask u32 and expect u32 as flagged, a decode count u8, then per
entry value u32 and text len u8 + bytes. A probe's `desc` stays in the
hub.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

from nxs.cam.descriptors import Descriptor
from nxs.dsl.fields import Port
from nxs.personality.records_fields import (
    ACTIONS, ACTION_PARAM, EXPOSURE_PARAM, CAMERA_MAP, FRAME_LENGTH_PARAM, FRAME_PERIOD_PARAM,
    GAIN_PARAM, IDENTITY, LINE_TIME_PARAM, MAX_TRAILER_SIZE, MODE_PARAM, OUTPUTS, PARAMS, PORTS,
    PRESENTATION, PROGRAM, RETIRED_RECORDS, TRIGGERS, TrailerRecord, RecordError, _ALIVE_REGISTER,
    _CONTROL_REGISTERS, _FAMILY_NAMES, _RETIRED_TIMING, trailer_crc, _read_text, _register,
    _unit_modes)
from nxs.personality.records_outputs import OutputRow, decode_outputs
from nxs.personality.records_ports import decode_ports
from nxs.personality.records_sensor import (
    ParamMap, check_params, mode_values, trigger_values,
    _decode_controls, _decode_identity, _decode_modes,
    _decode_status, _decode_triggers, _encode_controls, _encode_identity,
    _encode_modes, _encode_status, _encode_triggers,
    _mode_entry)
from nxs.personality.records_params import ParamRow, decode_rows
from nxs.personality.records_present import RowPresentation, decode_presentation
from nxs.personality.records_family import (
    _decode_capture, _decode_camera_map, _decode_generic, _decode_laws, _decode_program,
    _decode_sony, _encode_capture, _encode_camera, _encode_laws, _encode_program)

#: The names `nxs.personality.records` has always answered to.
__all__ = [
    "ACTIONS", "ACTION_PARAM", "EXPOSURE_PARAM", "CAMERA_MAP", "FRAME_LENGTH_PARAM",
    "FRAME_PERIOD_PARAM", "GAIN_PARAM", "IDENTITY", "LINE_TIME_PARAM", "MAX_TRAILER_SIZE",
    "MODE_PARAM", "OUTPUTS", "PARAMS", "PORTS", "PRESENTATION", "PROGRAM", "RETIRED_RECORDS",
    "TRIGGERS", "TrailerRecord", "ParamMap", "RecordError", "check_params",
    "decode_trailer", "descriptor_from_trailer", "encode_trailer", "camera_map_info",
    "camera_map_mode_rows",
    "law_family", "mode_values", "output_rows", "param_map", "param_rows", "port_rows",
    "presentation", "trailer_crc", "trigger_values", "_read_text",
]


# ── helpers ──────────────────────────────────────────────────────────


# ── encode ───────────────────────────────────────────────────────────

def encode_trailer(descriptor: Descriptor, params=None, hub=None) -> List[Tuple[int, bytes]]:
    """The `(type, bytes)` records of a sensor descriptor: IDENTITY, MODES,
    TRIGGERS, CONTROLS, LAWS, PROGRAM, CAMERA_MAP, PARAMS and PRESENTATION
    when the family derives an apply law, CAPTURE when the descriptor
    carries the capture facts, and STATUS when it declares probes. `params` is the compiled image's
    parameter table (`CompiledDriver.params`), the source of the `mode` and
    `trigger` param indices; without it the
    records say the personality stages no param. ``hub`` is the hub the
    descriptor belongs to, whose serializer tail the capture rows' top rate
    counts; without it the rows carry the sensor's own bound. RecordError
    names a fact the records cannot carry."""
    if descriptor.role != "SEN":
        raise RecordError(f"{descriptor.name}: a cam personality is a "
                          f"sensor descriptor, not {descriptor.role}")
    modes = _unit_modes(descriptor)
    if not modes:
        raise RecordError(f"{descriptor.name}: no mode the unit can run "
                          f"(every mode is host_only)")
    if params is not None:
        check_params(descriptor, params)
    _refuse_experimental_facts(descriptor)
    records = [(IDENTITY, _encode_identity(descriptor, modes)),
               (TrailerRecord.MODES, _encode_modes(descriptor, modes, params)),
               (TRIGGERS, _encode_triggers(descriptor, params)),
               (TrailerRecord.LAWS, _encode_laws(descriptor)),
               (TrailerRecord.CONTROLS, _encode_controls(descriptor)),
               (PROGRAM, _encode_program(descriptor))]
    camera = _encode_camera(descriptor, params)
    if camera is not None:
        records.append((CAMERA_MAP, camera[0]))
        records.append((PARAMS, camera[1]))
        records.append((PRESENTATION, camera[2]))
    if descriptor.raw("capture"):
        records.append((TrailerRecord.CAPTURE, _encode_capture(descriptor, hub)))
    status = _encode_status(descriptor)
    if status is not None:
        records.append((TrailerRecord.STATUS, status))
    return records


def _refuse_experimental_facts(descriptor: Descriptor) -> None:
    """A personality is compiled from the shipped descriptor: a bench
    overlay's operating points and measured limits never reach a unit."""
    found: List[str] = []
    for name, mode in descriptor.modes.items():
        timing = mode.get("timing") or {}
        found += [f"{name}.timing.{k}" for k in sorted(_RETIRED_TIMING & set(timing))]
    found += [f"limits.{k}" for k, v in sorted(descriptor.limits_source.items())
              if v == "experimental"]
    if found:
        raise RecordError(
            f"{descriptor.name}: a personality is compiled from the shipped "
            f"descriptor; the experimental facts {found} stay in the experimental overlay")


# ── decode ───────────────────────────────────────────────────────────

def decode_trailer(records: Sequence[Tuple[int, bytes]]) -> Dict[str, Any]:
    """A descriptor mapping (the shape `cam-personality.schema.json` pins)
    from a personality's trailer records: identity, the registers the
    control rows imply, the modes without blobs, the laws' limits, the
    trigger and sync facts, the program settles, and the capture table.
    RecordError when the IDENTITY record is missing or a record is
    malformed; unknown and retired record types are skipped."""
    by_type: Dict[int, bytes] = {}
    for rec_type, payload in records:
        if int(rec_type) not in RETIRED_RECORDS:
            by_type.setdefault(int(rec_type), bytes(payload))
    if IDENTITY not in by_type:
        raise RecordError("the trailer carries no IDENTITY record: not a "
                          "cam personality this tool reads")
    identity = _decode_identity(by_type[IDENTITY])
    family_id, limits, mode_waits = _decode_laws(by_type.get(TrailerRecord.LAWS, b""))
    family = _FAMILY_NAMES.get(family_id)
    if family is None:
        raise RecordError(f"the trailer names chip family id {family_id}, "
                          f"which this tool does not know")
    _index, modes = _decode_modes(by_type.get(TrailerRecord.MODES, b"\xff\x00"))
    if not modes:
        raise RecordError("the trailer carries no modes")
    rows = _decode_controls(by_type.get(TrailerRecord.CONTROLS, b"\x00"))

    meta: Dict[str, Any] = {
        "compatible": identity["compatible"], "role": "SEN", "kind": "cam",
        "chip": family, "i2c_addr": identity["i2c_addr"],
        "reg_bits": identity["reg_bits"], "val_bits": identity["val_bits"],
    }
    if identity["device_id_reg"] is not None:
        meta["device_id_reg"] = identity["device_id_reg"]
        meta["device_id"] = identity["device_id"]
        meta["device_id_width"] = identity["device_id_width"]
    registers: Dict[str, Dict[str, Any]] = {}
    for name, addr, width, order, _form, _params in rows:
        registers[_CONTROL_REGISTERS[name]] = _register(addr, width, order)
    doc: Dict[str, Any] = {"meta": meta, "registers": registers}
    default_index = identity["default_mode"]
    if default_index >= len(modes):
        raise RecordError(f"default mode index {default_index} is past the "
                          f"{len(modes)} modes")
    doc["default_mode"] = modes[default_index]["name"]
    doc["modes"] = {m["name"]: _mode_entry(m) for m in modes}
    for index, waits in mode_waits:
        if index >= len(modes):
            raise RecordError(f"LAWS names mode index {index}, past the {len(modes)} modes")
        doc["modes"][modes[index]["name"]].setdefault("timing", {})["waits"] = waits
    if family == "sony_imx":
        _decode_sony(doc, rows)
    else:
        _decode_generic(doc, rows)
    if limits:
        doc["limits"] = limits
    sync: Dict[str, Any] = {"takes_trigger": identity["takes_trigger"]}
    sync["text"] = ("takes the frame-sync trigger" if identity["takes_trigger"]
                    else "no trigger input declared")
    if "sync" in doc:
        sync.update(doc["sync"])
    doc["sync"] = sync
    if identity["pulse_width_is_exposure"]:
        trigger = doc.setdefault("trigger", {})
        trigger.setdefault("timing", {})["pulse_width_is_exposure"] = True
    alive = identity["alive_reg"]
    if (alive is not None and family != "sony_imx"
            and alive != identity["device_id_reg"]):
        registers[_ALIVE_REGISTER] = _register(alive, 1, 0)
        doc.setdefault("program", {})["alive_reg"] = _ALIVE_REGISTER
    program = _decode_program(by_type.get(PROGRAM, b""), registers, meta)
    if program:
        doc.setdefault("program", {}).update(program)
    if TrailerRecord.CAPTURE in by_type:
        doc["capture"] = _decode_capture(by_type[TrailerRecord.CAPTURE])
    if TrailerRecord.STATUS in by_type:
        doc["status"] = _decode_status(by_type[TrailerRecord.STATUS], registers)
    return doc


def param_map(records: Sequence[Tuple[int, bytes]]) -> ParamMap:
    """The staging facts of a trailer: the `mode` and `trigger` params'
    indices in the image's parameter table with the value behind every
    mode and conversion name."""
    by_type = {int(t): bytes(p) for t, p in records}
    mode_index, modes = _decode_modes(by_type.get(TrailerRecord.MODES, b"\xff\x00"))
    trigger_index, triggers = _decode_triggers(by_type.get(TRIGGERS, b"\xff\x00"))
    return ParamMap(mode_index=mode_index,
                    modes={m["name"]: m["value"] for m in modes},
                    trigger_index=trigger_index, triggers=triggers)


def camera_map_info(records: Sequence[Tuple[int, bytes]]) -> Optional[Dict[str, Any]]:
    """The camera a trailer declares: the CAMERA_MAP record decoded
    (`_decode_camera_map`) with the PARAMS rows under `rows`; None when the
    personality carries no CAMERA_MAP record."""
    by_type = {int(t): bytes(p) for t, p in records}
    if CAMERA_MAP not in by_type:
        return None
    info = _decode_camera_map(by_type[CAMERA_MAP])
    info["rows"] = decode_rows(by_type.get(PARAMS, b""))
    return info


def camera_map_mode_rows(records: Sequence[Tuple[int, bytes]]) -> bytes:
    """The mode-table rows a trailer's CAMERA_MAP record closes on: the
    bytes a camera door serves at `MODE_TABLE`, empty without the record."""
    from nxs.personality.records_fields import (_CAMERA_MAP_HEAD, _CAMERA_MAP_LAW,
                                                _CAMERA_MAP_LAW_MODE, _CAMERA_MAP_MODE,
                                                _CAMERA_MAP_PARAMS)

    by_type = {int(t): bytes(p) for t, p in records}
    payload = by_type.get(CAMERA_MAP)
    if not payload:
        return b""
    count = payload[0]
    start = (_CAMERA_MAP_HEAD.size + _CAMERA_MAP_PARAMS.size + _CAMERA_MAP_LAW.size
             + count * _CAMERA_MAP_LAW_MODE.size)
    return payload[start:start + count * _CAMERA_MAP_MODE.size]


def law_family(records: Sequence[Tuple[int, bytes]]) -> Optional[str]:
    """The law family a trailer's LAWS record names by its first byte, None
    when the trailer carries no LAWS record or names a family this tool
    does not know."""
    by_type = {int(t): bytes(p) for t, p in records}
    payload = by_type.get(TrailerRecord.LAWS)

    return _FAMILY_NAMES.get(payload[0]) if payload else None


def param_rows(records: Sequence[Tuple[int, bytes]]) -> List[ParamRow]:
    """The PARAMS rows a trailer carries, none when it carries no record."""
    by_type = {int(t): bytes(p) for t, p in records}
    return decode_rows(by_type.get(PARAMS, b""))


def presentation(records: Sequence[Tuple[int, bytes]]) -> List[RowPresentation]:
    """How a trailer's PARAMS rows read, one entry per row, none when it
    carries no PRESENTATION record."""
    by_type = {int(t): bytes(p) for t, p in records}
    if PRESENTATION not in by_type:
        return []
    return decode_presentation(by_type[PRESENTATION], len(decode_rows(by_type.get(PARAMS, b""))))


def output_rows(records: Sequence[Tuple[int, bytes]]) -> List[OutputRow]:
    """The OUTPUTS rows a trailer carries, none when it carries no record."""
    by_type = {int(t): bytes(p) for t, p in records}
    return decode_outputs(by_type.get(OUTPUTS, b""))


def port_rows(records: Sequence[Tuple[int, bytes]]) -> List[Port]:
    """The ports a trailer's PORTS record declares, none when it carries no
    record."""
    by_type = {int(t): bytes(p) for t, p in records}
    return decode_ports(by_type.get(PORTS, b""))


def descriptor_from_trailer(data: bytes, name: Optional[str] = None) -> Descriptor:
    """The `Descriptor` a unit's trailer bytes describe (count byte
    included, as `read_personality_info` serves it); `name` overrides the
    descriptor name the IDENTITY record carries."""
    from nxs.image import parse_trailer
    records = parse_trailer(bytes(data))
    doc = decode_trailer(records)
    identity = _decode_identity(dict((int(t), bytes(p)) for t, p in records)[IDENTITY])
    return Descriptor.from_data(doc, name or identity["name"])


