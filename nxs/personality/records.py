# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The descriptor trailer of a camera personality: the records a datasheet
descriptor compiles into, and the descriptor a unit's trailer decodes back
to. The unit stores the trailer opaquely and serves it page by page; the
host reads modes, controls, laws, and capture facts from it and never
needs the descriptor pack on the robot.

Record layouts (all integers little-endian; `TrailerRecord` numbers the
types, `IDENTITY` and `PROGRAM` extend that vocabulary above the
firmware's reserved `INSTANCE_CAL` and below `VENDOR_BASE`):

`IDENTITY` (6): i2c_addr u8, reg_bits u8, val_bits u8, flags u8 (bit0
takes_trigger), device_id_reg u16 (0xFFFF none), device_id_width u8,
device_id u16, alive_reg u16 (0xFFFF none), default mode index u8, then
the descriptor name and the compatible string, each as len u8 + bytes.

`MODES` (1): mode_param_index u8 (the index of the image's `mode` enum
param in its parameter table, what `PARAM_SELECT` stages; 0xFF when the
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
0xFF when the personality declares none), count u8, then per conversion
the personality implements: param value u8 (its index among the
descriptor's trigger presets in declaration order) and the preset name,
len u8 + bytes. The records carry no param names: the tool reads the
`mode` and `trigger` semantics from these indices.

`RUN_PARAMS` (11): count u8, then per range parameter of the image, the
values a host stages in physical units: index u8 (what `PARAM_SELECT`
stages), min u32, max u32, default u32, then the name (the quantity,
`line_time` or `exposure`) and its unit, each len u8 + bytes.

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
frame_length_delta_const (a table by readout kind, the same shape). A
decoded descriptor's `limits` are the declared ones; the generic family
declares inck_hz and min_fps at most.

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

`SHIPPED` (9): the points a mode ships. A count u8, then per point: mode
index u8 (the MODES order), cameras u8 (1 or 2), csi_lanes u8, fps floor
x1000 u32, fps ceiling x1000 u32, hmax u16 (the line length the port runs
at), trigger_vmax u32 (the fast-trigger frame the pair syncs at, 0 for a
point that ships free-running only). How a point was proven rides no
image.

`STATUS` (10): the status probes `status` reads and the frame oracle
judges. A count u8, then per probe: name len u8 + bytes, addr u16, width
u8, order u8, flags u8 (bit0 the value prints as an integer, bit1 a
non-zero value warns, bit2 a mask follows, bit3 an expected value
follows), mask u32 and expect u32 as flagged, a decode count u8, then per
entry value u32 and text len u8 + bytes. A probe's `desc` stays in the
pack.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

from nxs.cam.descriptors import Descriptor
from nxs.personality.records_fields import (
    ACTIONS, ACTION_PARAM, FRAME_LENGTH_PARAM, FRAME_PERIOD_PARAM, IDENTITY,
    LINE_TIME_PARAM, MAX_TRAILER_SIZE, MODE_PARAM, PROGRAM, RUN_PARAMS,
    TRIGGERS, TrailerRecord, RecordError, _ALIVE_REGISTER, _CONTROL_REGISTERS,
    _FAMILY_NAMES, _RETIRED_TIMING, trailer_crc, _read_text, _register,
    _unit_modes)
from nxs.personality.records_sensor import (
    ParamMap, RunParam, check_params, mode_values, trigger_values,
    _decode_controls, _decode_identity, _decode_modes, _decode_run_params,
    _decode_shipped, _decode_status, _decode_triggers, _encode_controls,
    _encode_identity, _encode_modes, _encode_run_params, _encode_shipped,
    _encode_status, _encode_triggers, _mode_entry)
from nxs.personality.records_family import (
    _decode_capture, _decode_generic, _decode_laws, _decode_program,
    _decode_sony, _encode_capture, _encode_laws, _encode_program)

#: The names `nxs.personality.records` has always answered to.
__all__ = [
    "ACTIONS", "ACTION_PARAM", "FRAME_LENGTH_PARAM", "FRAME_PERIOD_PARAM",
    "IDENTITY", "LINE_TIME_PARAM", "MAX_TRAILER_SIZE", "MODE_PARAM", "PROGRAM",
    "RUN_PARAMS", "TRIGGERS", "TrailerRecord", "ParamMap", "RecordError",
    "RunParam", "check_params", "decode_trailer", "descriptor_from_trailer",
    "encode_trailer", "mode_values", "param_map", "trailer_crc",
    "trigger_values", "_read_text",
]


# ── helpers ──────────────────────────────────────────────────────────


# ── encode ───────────────────────────────────────────────────────────

def encode_trailer(descriptor: Descriptor, params=None, pack=None) -> List[Tuple[int, bytes]]:
    """The `(type, bytes)` records of a sensor descriptor: IDENTITY, MODES,
    TRIGGERS, RUN_PARAMS, CONTROLS, LAWS, PROGRAM, and CAPTURE when the
    descriptor carries the capture facts. `params` is the compiled image's
    parameter table (`CompiledDriver.params`), the source of the `mode` and
    `trigger` param indices and of the run parameters; without it the
    records say the personality stages no param. ``pack`` is the pack the
    descriptor belongs to, whose serializer tail the capture rows' top rate
    counts; without it the rows carry the sensor's own bound. RecordError
    names a fact the records cannot carry."""
    if descriptor.role != "SEN":
        raise RecordError(f"{descriptor.name}: a camera personality is a "
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
               (RUN_PARAMS, _encode_run_params(params)),
               (TrailerRecord.LAWS, _encode_laws(descriptor)),
               (TrailerRecord.CONTROLS, _encode_controls(descriptor)),
               (PROGRAM, _encode_program(descriptor))]
    if descriptor.raw("capture"):
        records.append((TrailerRecord.CAPTURE, _encode_capture(descriptor, pack)))
    shipped = _encode_shipped(descriptor, modes)
    if shipped is not None:
        records.append((TrailerRecord.SHIPPED, shipped))
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
    """A descriptor mapping (the shape `cam-descriptor.schema.json` pins)
    from a personality's trailer records: identity, the registers the
    control rows imply, the modes without blobs, the laws' limits, the
    trigger and sync facts, the program settles, and the capture table.
    RecordError when the IDENTITY record is missing or a record is
    malformed; unknown record types are skipped."""
    by_type: Dict[int, bytes] = {}
    for rec_type, payload in records:
        by_type.setdefault(int(rec_type), bytes(payload))
    if IDENTITY not in by_type:
        raise RecordError("the trailer carries no IDENTITY record: not a "
                          "camera personality this tool reads")
    identity = _decode_identity(by_type[IDENTITY])
    family_id, limits = _decode_laws(by_type.get(TrailerRecord.LAWS, b""))
    family = _FAMILY_NAMES.get(family_id)
    if family is None:
        raise RecordError(f"the trailer names chip family id {family_id}, "
                          f"which this tool does not know")
    _index, modes = _decode_modes(by_type.get(TrailerRecord.MODES, b"\xff\x00"))
    if not modes:
        raise RecordError("the trailer carries no modes")
    rows = _decode_controls(by_type.get(TrailerRecord.CONTROLS, b"\x00"))

    meta: Dict[str, Any] = {
        "compatible": identity["compatible"], "role": "SEN", "kind": "camera",
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
    if TrailerRecord.SHIPPED in by_type:
        shipped = _decode_shipped(by_type[TrailerRecord.SHIPPED], modes)
        if shipped:
            doc["shipped"] = shipped
    if TrailerRecord.STATUS in by_type:
        doc["status"] = _decode_status(by_type[TrailerRecord.STATUS], registers)
    return doc


def param_map(records: Sequence[Tuple[int, bytes]]) -> ParamMap:
    """The staging facts of a trailer: the `mode` and `trigger` params'
    indices in the image's parameter table with the value behind every
    mode and conversion name, and the run parameters by name."""
    by_type = {int(t): bytes(p) for t, p in records}
    mode_index, modes = _decode_modes(by_type.get(TrailerRecord.MODES, b"\xff\x00"))
    trigger_index, triggers = _decode_triggers(by_type.get(TRIGGERS, b"\xff\x00"))
    return ParamMap(mode_index=mode_index,
                    modes={m["name"]: m["value"] for m in modes},
                    trigger_index=trigger_index, triggers=triggers,
                    run_params=_decode_run_params(by_type.get(RUN_PARAMS, b"\x00")))


def descriptor_from_trailer(data: bytes, name: Optional[str] = None) -> Descriptor:
    """The `Descriptor` a unit's trailer bytes describe (count byte
    included, as `read_personality_info` serves it); `name` overrides the
    descriptor name the IDENTITY record carries."""
    from nxs.image import parse_trailer
    records = parse_trailer(bytes(data))
    doc = decode_trailer(records)
    identity = _decode_identity(dict((int(t), bytes(p)) for t, p in records)[IDENTITY])
    return Descriptor.from_data(doc, name or identity["name"])


