"""The records a law family contributes: its laws, the standby-wrapped program it composes, and the capture table the kernel driver reads."""

from __future__ import annotations

import struct
from typing import Any, Dict, List, Optional, Tuple

from nxs._generated_constants import NxsMap
from nxs.cam.descriptors import Descriptor
from nxs.personality.records_fields import (
    ACTION_PARAM, EXPOSURE_PARAM, FAMILY_IDS, FRAME_LENGTH_PARAM, FRAME_PERIOD_PARAM, GAIN_PARAM,
    LINE_TIME_PARAM, MODE_PARAM, PIXEL_PHASES, RecordError, TRIGGER_PARAM, _CAPTURE_ENTRY,
    _CAPTURE_ENTRY_V1, _CAPTURE_FLAG_CLOCK_NONCONTINUOUS, _CAPTURE_HEAD, _CHROMACITIES,
    _CONTROL_REGISTERS, _DELTA_SCALAR, _DELTA_TABLE, _CAMERA_MAP_HEAD, _CAMERA_MAP_LAW,
    _CAMERA_MAP_LAW_MODE, _CAMERA_MAP_MODE, _CAMERA_MAP_PARAMS, _CAMERA_MAP_PARAM_NAMES,
    _CHART_REG_LISTS, _LAWS_HEAD, _LAW_FIELDS, _NONE_U8,
    _PROGRAM_ADBIT, _PROGRAM_ADBIT_CODE, _PROGRAM_BLKLEVEL, _PROGRAM_BLKLEVEL_VALUE,
    _PROGRAM_FLAGS, _PROGRAM_HEAD, _PROGRAM_REPAIR, _PROGRAM_TAIL,
    _SYNC_ROLES, _TEST_PATTERN_SELECT, _TRIGGER_SLOTS, _VINT_BITS,
    _family_name, _fixed, _number, _read_table, _register, _register_spec,
    _table, _u, _unit_modes, _unpack)
from nxs.personality.records_params import camera_rows, encode_presentation_rows, encode_rows
from nxs.personality.records_sensor import _param_index, _param_values, trigger_values

#: The run values a pod's apply writes; a program that stages one stages
#: the others, with the action and the line time beside them.
_APPLY_VALUES = (EXPOSURE_PARAM, FRAME_LENGTH_PARAM, GAIN_PARAM)
#: The pixel streams the camera map's mode table carries, its `DATA_TYPE`
#: the RAW sample depth (Interface Description §13.3).
_CAMERA_MAP_DATA_TYPES = ("RAW10", "RAW12")


def _encode_laws(descriptor: Descriptor) -> bytes:
    family = _family_name(descriptor)
    if family not in FAMILY_IDS:
        raise RecordError(f"{descriptor.name}: unknown chip family {family!r}; "
                          f"have {sorted(FAMILY_IDS)}")
    if family == "native":
        # A native head's limits are the camera map's physical bounds and
        # ride the CAMERA_MAP record; the LAWS record names the family alone.
        return _LAWS_HEAD.pack(FAMILY_IDS[family], 0)
    limits = descriptor.limits
    present = 0
    body = bytearray()
    waits = list((limits.get("captured_waits") or {}).items())
    names = [k for k, _ in waits]
    # A mode's own waits ride the LAWS record by the mode's index, so an
    # older tool's MODES decoder never meets them.
    mode_waits = []
    for index, name in enumerate(_unit_modes(descriptor)):
        own = (descriptor.modes[name].get("timing") or {}).get("waits")
        if isinstance(own, dict):
            mode_waits.append((index, own))
    for bit, (key, fmt) in enumerate(_LAW_FIELDS):
        value = mode_waits if key == "mode_waits" else limits.get(key)
        if value is None or (key == "mode_waits" and not value):
            continue
        present |= 1 << bit
        what = f"{descriptor.name} limits.{key}"
        if fmt == "n":
            body += struct.pack("<I", _u(round(float(value) * 1000), 32, what))
        elif fmt == "q":
            body += struct.pack("<I", _fixed(value, what))
        elif fmt == "g":
            pair = isinstance(value, (list, tuple))
            num, den = (value[0], value[1]) if pair else (value, 1)
            body += struct.pack("<BII", 1 if pair else 0, _u(num, 32, what), _u(den, 32, what))
        elif fmt == "w":
            body += _table(((k, v) for k, v in waits), what)
        elif fmt == "i":
            body += _wait_indices(value, names, what)
        elif fmt == "f":
            chart = dict(value)
            body += struct.pack("<IB", _u(round(float(chart["output_delay_us"]) * 1000), 32, what),
                                _u(chart["output_delay_lines"], 8, what))
            for regs_key in _CHART_REG_LISTS:
                body += _wait_indices(chart.get(regs_key) or [], names, what)
        elif fmt == "m":
            body += struct.pack("<B", _u(len(value), 8, what))
            for index, table in value:
                body += struct.pack("<B", _u(index, 8, what)) + _table(table.items(), what)
        elif fmt == "d":
            if isinstance(value, dict):
                body += struct.pack("<B", _DELTA_TABLE) + _table(value.items(), what)
            else:
                body += struct.pack("<BH", _DELTA_SCALAR, _u(value, 16, what))
        elif fmt == "t":
            body += _table(dict(value).items(), what)
        else:
            body += struct.pack("<" + fmt, _u(value, {"H": 16, "I": 32}[fmt], what))
    return _LAWS_HEAD.pack(FAMILY_IDS[family], present) + bytes(body)

def _wait_indices(regs, names: List[str], what: str) -> bytes:
    """A register list as indices into the captured waits: count u8, then
    one index u8 per register."""
    buf = bytearray(struct.pack("<B", _u(len(regs), 8, what)))
    for reg in regs:
        if str(reg) not in names:
            raise RecordError(f"{what}: {reg!r} is not a captured wait register")
        buf += struct.pack("<B", names.index(str(reg)))
    return bytes(buf)

def _read_wait_indices(payload: bytes, pos: int, names: List[str]) -> Tuple[List[str], int]:
    count, pos = _unpack("B", payload, pos, "LAWS")
    regs = []
    for _ in range(count):
        i, pos = _unpack("B", payload, pos, "LAWS")
        if i >= len(names):
            raise RecordError("LAWS names a wait register it does not carry")
        regs.append(names[i])
    return regs, pos

def _encode_program(descriptor: Descriptor) -> bytes:
    program = descriptor.raw("program") or {}
    flags = 0
    for bit, key in enumerate(_PROGRAM_FLAGS):
        if key in program and program[key] is not None:
            flags |= 1 << bit
    timing = program.get("timing_start") or {}
    role = str(timing.get("syncsel") or "")
    if role and role not in _SYNC_ROLES:
        raise RecordError(f"{descriptor.name}: timing_start.syncsel {role!r} is "
                          f"not one of {', '.join(_SYNC_ROLES)}")
    repairs = list((timing.get("repairs") or {}).items())
    buf = bytearray(_PROGRAM_HEAD.pack(
        flags,
        _u(timing.get("standby_ms", 0), 16, "timing_start.standby_ms"),
        _u(timing.get("release_ms", 0), 16, "timing_start.release_ms"),
        _u(timing.get("start_ms", 0), 16, "timing_start.start_ms"),
        _SYNC_ROLES.index(role) + 1 if role else 0,
        _u(len(repairs), 8, "timing_start.repairs")))
    for reg, value in repairs:
        addr, width, order = _register_spec(descriptor, str(reg))
        buf += _PROGRAM_REPAIR.pack(addr, width, order, _u(value, 32, f"repair {reg}"))
    tail: List[int] = []
    for key in ("trigger_switch", "fast_trigger", "sync_switch"):
        sleep_set = program.get(key) or {}
        tail += [int(sleep_set.get(k, 0)) for k in ("standby_ms", "release_ms", "start_ms")]
    restart = program.get("restart") or {}
    tail += [int(restart.get(k, 0)) for k in ("standby_ms", "stop_ms", "release_ms", "start_ms")]
    start = program.get("start") or {}
    tail += [int(start.get(k, 0)) for k in ("release_ms", "start_ms")]
    tail.append(int(program.get("stop_ms", 0)))
    buf += _PROGRAM_TAIL.pack(*(_u(v, 16, "program settle") for v in tail))
    adbit = program.get("adbit") or {}
    init_inck = program.get("init_inck_hz")
    blklevel = program.get("blklevel") or {}
    if adbit or init_inck is not None or blklevel:
        if "ADBIT_MONOSEL" not in descriptor.registers:
            raise RecordError(f"{descriptor.name}: program.adbit without an "
                              f"ADBIT_MONOSEL register")
        addr, width, order = _register_spec(descriptor, "ADBIT_MONOSEL")
        chromacity = str((descriptor.raw("meta") or {}).get("chromacity", "color"))
        if chromacity not in _CHROMACITIES:
            raise RecordError(f"{descriptor.name}: meta.chromacity {chromacity!r} "
                              f"is not one of {', '.join(_CHROMACITIES)}")
        codes = sorted((_u(bits, 8, f"adbit depth {bits}"), _u(code, 8, f"adbit {bits}"))
                       for bits, code in adbit.items())
        buf += _PROGRAM_ADBIT.pack(addr, width, order, _CHROMACITIES.index(chromacity),
                                   _u(init_inck or 0, 32, "init_inck_hz"),
                                   _u(len(codes), 8, "adbit"))
        for bits, code in codes:
            buf += _PROGRAM_ADBIT_CODE.pack(bits, code)
    if blklevel:
        if "BLKLEVEL" not in descriptor.registers:
            raise RecordError(f"{descriptor.name}: program.blklevel without a "
                              f"BLKLEVEL register")
        addr, width, order = _register_spec(descriptor, "BLKLEVEL")
        values = sorted((_u(bits, 8, f"blklevel depth {bits}"),
                         _u(value, 32, f"blklevel {bits}"))
                        for bits, value in blklevel.items())
        buf += _PROGRAM_BLKLEVEL.pack(addr, width, order, _u(len(values), 8, "blklevel"))
        for bits, value in values:
            buf += _PROGRAM_BLKLEVEL_VALUE.pack(bits, value)
    return bytes(buf)

def _encode_capture(descriptor: Descriptor, hub=None) -> bytes:
    from nxs.cam.capture_facts import derived_rows

    cap = descriptor.raw("capture")
    gain, hdr, exposure, framerate = (cap["gain"], cap["hdr_ratio"],
                                      cap["exposure"], cap["framerate"])
    polarity = cap.get("lane_polarity")
    # A row the unit program offers derives its line length and top rate
    # from the mode's timing, the hub's serializer tail counted when the
    # hub is in hand: the rows a unit serves are then the rows a host
    # with the hub derives.
    table = derived_rows(hub, descriptor)
    buf = bytearray(_CAPTURE_HEAD.pack(
        _u(cap["mclk_khz"], 32, "capture.mclk_khz"),
        _u(cap["pix_clk_hz"], 32, "capture.pix_clk_hz"),
        _NONE_U8 if polarity is None else _u(polarity, 8, "capture.lane_polarity"),
        _u(gain["factor"], 32, "capture.gain.factor"), int(gain["min"]),
        int(gain["max"]), _u(gain["step"], 32, "capture.gain.step"),
        int(gain["default"]),
        _u(hdr["min"], 16, "capture.hdr_ratio.min"),
        _u(hdr["max"], 16, "capture.hdr_ratio.max"),
        _u(exposure["factor"], 32, "capture.exposure.factor"),
        _u(exposure["max_us"], 32, "capture.exposure.max_us"),
        _u(exposure["step"], 32, "capture.exposure.step"),
        _u(exposure["default_us"], 32, "capture.exposure.default_us"),
        _u(framerate["factor"], 32, "capture.framerate.factor"),
        _u(round(float(framerate["min_fps"]) * 1000), 32, "capture.framerate.min_fps"),
        _u(framerate["step"], 32, "capture.framerate.step"),
        _u(len(table), 8, "capture.table")))
    for entry in table:
        phase = str(entry["pixel_phase"])
        if phase not in PIXEL_PHASES:
            raise RecordError(f"{descriptor.name}: capture pixel_phase {phase!r}")
        buf += _CAPTURE_ENTRY.pack(
            _u(entry["width"], 16, "capture width"),
            _u(entry["height"], 16, "capture height"),
            _u(entry["bit_depth"], 8, "capture bit_depth"),
            PIXEL_PHASES.index(phase),
            _u(entry["line_length"], 32, "capture line_length"),
            _u(round(float(entry["max_fps"]) * 1000), 32, "capture max_fps"),
            _u(entry["min_exp_us"], 32, "capture min_exp_us"),
            _u(entry.get("embedded_lines", 0) or 0, 8, "capture embedded_lines"))
    # The flags byte rides only when a flag is set: a record without one
    # keeps the bytes it always had.
    if cap.get("clock_noncontinuous"):
        buf += struct.pack("<B", _CAPTURE_FLAG_CLOCK_NONCONTINUOUS)
    return bytes(buf)

def _decode_laws(payload: bytes) -> Tuple[int, Dict[str, Any], List[Tuple[int, Dict[str, int]]]]:
    """`(family id, limits, mode waits)`: the declared limits the LAWS record
    carries, and the own waits of each mode that declares them, by the
    mode's index."""
    if not payload:
        return FAMILY_IDS["generic"], {}, []
    if len(payload) < _LAWS_HEAD.size:
        raise RecordError("LAWS record truncated")
    family, present = _LAWS_HEAD.unpack_from(payload)
    pos = _LAWS_HEAD.size
    limits: Dict[str, Any] = {}
    waits: Dict[str, int] = {}
    mode_waits: List[Tuple[int, Dict[str, int]]] = []
    for bit, (key, fmt) in enumerate(_LAW_FIELDS):
        if not present & (1 << bit):
            continue
        if fmt == "n":
            ns, pos = _unpack("I", payload, pos, "LAWS")
            limits[key] = _number(ns)
        elif fmt == "q":
            fixed, pos = _unpack("I", payload, pos, "LAWS")
            limits[key] = _number(fixed)
        elif fmt == "g":
            form, pos = _unpack("B", payload, pos, "LAWS")
            num, pos = _unpack("I", payload, pos, "LAWS")
            den, pos = _unpack("I", payload, pos, "LAWS")
            limits[key] = [num, den] if form else num
        elif fmt == "w":
            waits, pos = _read_table(payload, pos, "LAWS")
            limits[key] = dict(waits)
        elif fmt == "i":
            limits[key], pos = _read_wait_indices(payload, pos, list(waits))
        elif fmt == "f":
            ns, pos = _unpack("I", payload, pos, "LAWS")
            lines, pos = _unpack("B", payload, pos, "LAWS")
            chart: Dict[str, Any] = {"output_delay_us": _number(ns),
                                     "output_delay_lines": lines}
            for regs_key in _CHART_REG_LISTS:
                regs, pos = _read_wait_indices(payload, pos, list(waits))
                if regs or regs_key == _CHART_REG_LISTS[0]:
                    chart[regs_key] = regs
            limits[key] = chart
        elif fmt == "m":
            count, pos = _unpack("B", payload, pos, "LAWS")
            for _ in range(count):
                index, pos = _unpack("B", payload, pos, "LAWS")
                table, pos = _read_table(payload, pos, "LAWS")
                mode_waits.append((index, table))
        elif fmt == "d":
            form, pos = _unpack("B", payload, pos, "LAWS")
            if form == _DELTA_TABLE:
                limits[key], pos = _read_table(payload, pos, "LAWS")
            else:
                limits[key], pos = _unpack("H", payload, pos, "LAWS")
        elif fmt == "t":
            limits[key], pos = _read_table(payload, pos, "LAWS")
        else:
            limits[key], pos = _unpack(fmt, payload, pos, "LAWS")
    return family, limits, mode_waits

def _decode_sony(doc: Dict[str, Any], rows) -> None:
    """The Sony family re-derives its rows from registers and limits: the
    limits ride the LAWS record; the presets, roles, and test pattern come
    back from the trigger, sync, and test-pattern rows."""
    by_name = {name: (addr, width, order, form, params)
               for name, addr, width, order, form, params in rows}
    if "trigger" in by_name and "vint-en" in by_name:
        trigmodes = by_name["trigger"][4]
        vints = by_name["vint-en"][4]
        # The kernel row carries whole registers: the default mode's
        # VINT_EN field over each preset's two interrupt bits. A default
        # mode that declares its field gives the bits back.
        default = doc["modes"][doc["default_mode"]]
        if (default.get("timing") or {}).get("vint_mode") is not None:
            vints = [v & _VINT_BITS for v in vints]
        doc["trigger"] = {slot: {"trigmode": trigmodes[i], "vint_en": vints[i]}
                          for i, slot in enumerate(_TRIGGER_SLOTS)}
    if "sync-sel" in by_name:
        params = by_name["sync-sel"][4]
        doc["sync"] = {"syncsel": {"master": params[0], "slave": params[1]}}
    if "test-pattern" in by_name:
        _addr, _width, _order, _form, params = by_name["test-pattern"]
        reg = _CONTROL_REGISTERS["test-pattern"]
        doc["registers"][_TEST_PATTERN_SELECT] = _register(params[2], 1, 0)
        doc["test_pattern"] = {"enable": {reg: params[1]},
                               "disable": {reg: params[0]},
                               "select": _TEST_PATTERN_SELECT, "codes": {}}

def _decode_generic(doc: Dict[str, Any], rows) -> None:
    """The generic family reads a `controls:` table: one entry per row, gain
    with its dB law, the stream gate with its two values, the rest clamped
    to the register's full range."""
    controls: Dict[str, Any] = {}
    for name, _addr, width, _order, _form, params in rows:
        reg = _CONTROL_REGISTERS[name]
        key = name.replace("-", "_")
        if name == "gain":
            controls[key] = {"reg": reg, "min": 0, "max": params[2],
                             "num": params[0], "den": params[1]}
        elif name == "standby":
            controls[key] = {"reg": reg, "min": min(params[:2]), "max": max(params[:2]),
                             "standby": params[0], "streaming": params[1]}
        else:
            controls[key] = {"reg": reg, "min": 0, "max": (1 << (8 * width)) - 1}
    if controls:
        doc["controls"] = controls

def _decode_program(payload: bytes, registers: Dict[str, Any],
                    meta: Dict[str, Any]) -> Dict[str, Any]:
    if not payload:
        return {}
    if len(payload) < _PROGRAM_HEAD.size:
        raise RecordError("PROGRAM record truncated")
    (flags, standby_ms, release_ms, start_ms, role,
     repair_count) = _PROGRAM_HEAD.unpack_from(payload)
    pos = _PROGRAM_HEAD.size
    repairs: Dict[str, int] = {}
    for i in range(repair_count):
        if pos + _PROGRAM_REPAIR.size > len(payload):
            raise RecordError("PROGRAM record truncated inside its repairs")
        addr, width, order, value = _PROGRAM_REPAIR.unpack_from(payload, pos)
        pos += _PROGRAM_REPAIR.size
        name = f"REPAIR{i}"
        registers[name] = _register(addr, width, order)
        repairs[name] = value
    if pos + _PROGRAM_TAIL.size > len(payload):
        raise RecordError("PROGRAM record truncated")
    tail = list(_PROGRAM_TAIL.unpack_from(payload, pos))
    pos += _PROGRAM_TAIL.size
    program: Dict[str, Any] = {}
    if pos < len(payload):
        if pos + _PROGRAM_ADBIT.size > len(payload):
            raise RecordError("PROGRAM record truncated inside its AD-depth block")
        (addr, width, order, chromacity, init_inck,
         count) = _PROGRAM_ADBIT.unpack_from(payload, pos)
        pos += _PROGRAM_ADBIT.size
        if pos + count * _PROGRAM_ADBIT_CODE.size > len(payload):
            raise RecordError("PROGRAM record truncated inside its AD-depth codes")
        if chromacity >= len(_CHROMACITIES):
            raise RecordError(f"PROGRAM chromacity code {chromacity}")
        registers["ADBIT_MONOSEL"] = _register(addr, width, order)
        meta["chromacity"] = _CHROMACITIES[chromacity]
        if init_inck:
            program["init_inck_hz"] = int(init_inck)
        adbit: Dict[int, int] = {}
        for _ in range(count):
            bits, code = _PROGRAM_ADBIT_CODE.unpack_from(payload, pos)
            pos += _PROGRAM_ADBIT_CODE.size
            adbit[int(bits)] = int(code)
        if adbit:
            program["adbit"] = adbit
    if pos < len(payload):
        if pos + _PROGRAM_BLKLEVEL.size > len(payload):
            raise RecordError("PROGRAM record truncated inside its black-level block")
        addr, width, order, count = _PROGRAM_BLKLEVEL.unpack_from(payload, pos)
        pos += _PROGRAM_BLKLEVEL.size
        if pos + count * _PROGRAM_BLKLEVEL_VALUE.size > len(payload):
            raise RecordError("PROGRAM record truncated inside its black-level values")
        registers["BLKLEVEL"] = _register(addr, width, order)
        blklevel: Dict[int, int] = {}
        for _ in range(count):
            bits, value = _PROGRAM_BLKLEVEL_VALUE.unpack_from(payload, pos)
            pos += _PROGRAM_BLKLEVEL_VALUE.size
            blklevel[int(bits)] = int(value)
        if blklevel:
            program["blklevel"] = blklevel
    present = {key for bit, key in enumerate(_PROGRAM_FLAGS) if flags & (1 << bit)}

    def settles(keys, values):
        # A zero settle is an absent one: the family sleeps 0 either way.
        return {k: v for k, v in zip(keys, values) if v}

    if "timing_start" in present:
        timing: Dict[str, Any] = settles(("standby_ms", "release_ms", "start_ms"),
                                         (standby_ms, release_ms, start_ms))
        if repairs:
            timing["repairs"] = repairs
        if role:
            timing["syncsel"] = _SYNC_ROLES[role - 1]
        program["timing_start"] = timing
    for i, key in enumerate(("trigger_switch", "fast_trigger", "sync_switch")):
        if key in present:
            program[key] = settles(("standby_ms", "release_ms", "start_ms"),
                                   tail[3 * i:3 * i + 3])
    if "restart" in present:
        program["restart"] = settles(("standby_ms", "stop_ms", "release_ms", "start_ms"),
                                     tail[9:13])
    if "start" in present:
        program["start"] = settles(("release_ms", "start_ms"), tail[13:15])
    if "stop_ms" in present:
        program["stop_ms"] = tail[15]
    if "hmax_live" in present:
        program["hmax_live"] = True
    return program

def _decode_capture(payload: bytes) -> Dict[str, Any]:
    if len(payload) < _CAPTURE_HEAD.size:
        raise RecordError("CAPTURE record truncated")
    (mclk, pix, polarity, g_factor, g_min, g_max, g_step, g_default, h_min,
     h_max, e_factor, e_max, e_step, e_default, f_factor, f_min, f_step,
     count) = _CAPTURE_HEAD.unpack_from(payload)
    cap: Dict[str, Any] = {
        "mclk_khz": mclk, "pix_clk_hz": pix,
        "gain": {"factor": g_factor, "min": g_min, "max": g_max,
                 "step": g_step, "default": g_default},
        "hdr_ratio": {"min": h_min, "max": h_max},
        "exposure": {"factor": e_factor, "max_us": e_max, "step": e_step,
                     "default_us": e_default},
        "framerate": {"factor": f_factor, "min_fps": f_min / 1000, "step": f_step},
        "table": []}
    if polarity != _NONE_U8:
        cap["lane_polarity"] = polarity
    pos = _CAPTURE_HEAD.size
    # The row size is the record's: a trailer written before the
    # embedded-lines field carries 15-byte rows.
    body = len(payload) - pos
    entry = _CAPTURE_ENTRY
    if count and body == count * _CAPTURE_ENTRY_V1.size:
        entry = _CAPTURE_ENTRY_V1
    for _ in range(count):
        if pos + entry.size > len(payload):
            raise RecordError("CAPTURE record truncated inside its table")
        fields = entry.unpack_from(payload, pos)
        (width, height, bits, phase, line_length, fps1000, min_exp) = fields[:7]
        embedded = fields[7] if len(fields) > 7 else 0
        pos += entry.size
        if phase >= len(PIXEL_PHASES):
            raise RecordError(f"CAPTURE pixel phase code {phase}")
        row = {"width": width, "height": height,
               "bit_depth": bits, "pixel_phase": PIXEL_PHASES[phase],
               "line_length": line_length,
               "max_fps": fps1000 / 1000, "min_exp_us": min_exp}
        if embedded:
            row["embedded_lines"] = int(embedded)
        cap["table"].append(row)
    if pos < len(payload) and payload[pos] & _CAPTURE_FLAG_CLOCK_NONCONTINUOUS:
        cap["clock_noncontinuous"] = True
    return cap

def _encode_camera(descriptor: Descriptor, params=None) -> Optional[Tuple[bytes, bytes, bytes]]:
    """The CAMERA_MAP, PARAMS and PRESENTATION payloads of a cam personality:
    the law the unit converts a commit with and the mode table it serves,
    the public rows the camera map serves, and how they read. None when the
    family derives no apply law for the descriptor, or a pod's program
    stages no apply value (no `exposure`, `frame_length` or `gain`
    parameter), and no camera map is served. A program that stages one and lacks another run value of
    the apply (`action`, `line_time`, `exposure`, `frame_length`, `gain`)
    is refused by name: the unit converts a commit at the run's line time
    and runs it by the action, so its camera map would stay INERT. A
    native head's law names no register: the head converts in its own
    logic, so its record names no apply value. The trigger conversions
    served are the ones the program's `trigger` parameter declares, as the
    TRIGGERS record offers them."""
    from nxs.cam import laws
    from nxs.cam.descriptors import FREERUN

    surface = laws.bind(descriptor)
    if not hasattr(surface, "apply_law"):
        return None
    indices = [_param_index(params, name) for name in _CAMERA_MAP_PARAM_NAMES]
    staging = dict(zip(_CAMERA_MAP_PARAM_NAMES, indices))
    law = surface.apply_law()
    what = f"{descriptor.name} camera map"
    if law.frame_length is not None:
        staged = [name for name in _APPLY_VALUES if staging[name] is not None]
        if not staged:
            return None
        missing = [name for name in (ACTION_PARAM, LINE_TIME_PARAM, *_APPLY_VALUES)
                   if staging[name] is None]
        if missing:
            raise RecordError(
                f"{what}: the program stages {', '.join(staged)} and declares no "
                f"{', '.join(missing)}; the unit converts a commit at the run's line time "
                f"and runs the apply by the action, so declare_param() each of them")
    modes = _unit_modes(descriptor)
    if len(modes) > NxsMap.MODE_TABLE_MAX:
        raise RecordError(f"{what}: {len(modes)} modes, the camera map serves "
                          f"{NxsMap.MODE_TABLE_MAX}")
    if len(law.line_times_ns) != len(modes) or not all(law.line_times_ns):
        raise RecordError(f"{what}: the law states line times {law.line_times_ns} for "
                          f"{len(modes)} modes; every mode needs one")
    if law.gain_factor < 1:
        raise RecordError(f"{what}: gain factor {law.gain_factor}; the map counts capture "
                          f"units per decibel")
    presets = trigger_values(descriptor)
    declared = set(_param_values(params, TRIGGER_PARAM) or [])
    triggers = sorted(value for value in presets.values() if value in declared)
    default = str(descriptor.raw("default_mode"))
    rows = camera_rows(descriptor, law, staging, triggers,
                       presets.get(FREERUN, min(triggers) if triggers else 0),
                       modes.index(default), len(modes),
                       line_span=_param_values(params, LINE_TIME_PARAM),
                       trigger_names={value: name for name, value in presets.items()},
                       mode_names=modes,
                       inck_hz=(descriptor.raw("limits") or {}).get("inck_hz"))
    sync = descriptor.raw("sync") or {}
    capture = descriptor.raw("capture") or {}
    buf = bytearray(_CAMERA_MAP_HEAD.pack(len(modes),
                                          _u(law.gain_factor, 16, f"{what} gain factor")))
    buf += _CAMERA_MAP_PARAMS.pack(*(_NONE_U8 if i is None else _u(i, 8, f"{what} param index")
                                     for i in indices))
    buf += _CAMERA_MAP_LAW.pack(
        _u(law.gain_min, 16, f"{what} gain min"), _u(law.gain_max, 16, f"{what} gain max"),
        _u(law.gain_step, 16, f"{what} gain step"),
        _u(law.frame_length_max, 32, f"{what} frame length max"),
        _u(law.exposure_min, 32, f"{what} exposure min"),
        _u(law.exposure_margin, 16, f"{what} exposure margin"))
    # A law with one floor per mode keeps each; any other shape is the
    # one floor the apply bytecode clamps every mode to.
    floors = (law.frame_length_floors if len(law.frame_length_floors) == len(modes)
              else (law.frame_length_min,) * len(modes))
    for line_time_ns, floor in zip(law.line_times_ns, floors):
        buf += _CAMERA_MAP_LAW_MODE.pack(_u(line_time_ns, 32, f"{what} line time"),
                                         _u(floor, 32, f"{what} frame length floor"))
    # The Bayer order of each capture row, by its geometry and depth: a
    # host without a boot-time table reads the pixel format from the camera map.
    phases = {(int(row["width"]), int(row["height"]), int(row["bit_depth"])):
              PIXEL_PHASES.index(str(row["pixel_phase"]))
              for row in (capture.get("table") or [])
              if str(row.get("pixel_phase")) in PIXEL_PHASES}
    for value, name in enumerate(modes):
        mode = descriptor.modes[name]
        geo, mipi = mode["geometry"], mode["mipi"]
        data_type = str(mipi["data_type"])
        if data_type not in _CAMERA_MAP_DATA_TYPES:
            raise RecordError(f"{what}: {name} streams {data_type}, and the camera map's mode table "
                              f"carries {' or '.join(_CAMERA_MAP_DATA_TYPES)}")
        phase = phases.get((int(geo["width"]), int(geo["height"]), int(geo["bit_depth"])), 0xFF)
        # A row takes the trigger conversions when the sensor takes a
        # trigger on that mode's input and the program offers a conversion.
        triggerable = bool(mipi.get("trigger_input")) and bool(sync.get("takes_trigger")) and bool(triggers)
        mode_flags = ((NxsMap.MODE_FLAG_DEFAULT if name == default else 0)
                      | (NxsMap.MODE_FLAG_TRIGGERABLE if triggerable else 0))
        buf += _CAMERA_MAP_MODE.pack(
            _u(geo["width"], 16, f"{name} width"), _u(geo["height"], 16, f"{name} height"),
            _u(geo["bit_depth"], 8, f"{name} bit_depth"), _u(geo["lanes"], 8, f"{name} lanes"),
            _u(geo["rate_mbps"], 16, f"{name} rate_mbps"), int(data_type[3:]), mode_flags,
            _u(round(float(surface.fps_ceiling(name)) * 1000), 32, f"{name} max fps"),
            value, phase)
    what = f"{descriptor.name} parameters"
    return bytes(buf), encode_rows(rows, what), encode_presentation_rows(rows, what)

def _decode_camera_map(payload: bytes) -> Dict[str, Any]:
    """The CAMERA_MAP record back: the mode count and the gain factor, the
    run-value indices by parameter name (None when absent), the law head,
    the law row of each mode and the mode rows."""
    head = _CAMERA_MAP_HEAD.size + _CAMERA_MAP_PARAMS.size + _CAMERA_MAP_LAW.size
    if len(payload) < head:
        raise RecordError("CAMERA_MAP record truncated")
    mode_count, g_factor = _CAMERA_MAP_HEAD.unpack_from(payload)
    pos = _CAMERA_MAP_HEAD.size
    indices = _CAMERA_MAP_PARAMS.unpack_from(payload, pos)
    pos += _CAMERA_MAP_PARAMS.size
    g_min, g_max, g_step, fl_max, e_min, e_margin = _CAMERA_MAP_LAW.unpack_from(payload, pos)
    pos += _CAMERA_MAP_LAW.size
    if pos + mode_count * (_CAMERA_MAP_LAW_MODE.size + _CAMERA_MAP_MODE.size) != len(payload):
        raise RecordError(f"CAMERA_MAP record of {len(payload)} B does not hold "
                          f"{mode_count} modes")
    law_modes: List[Dict[str, int]] = []
    for _ in range(mode_count):
        line_time_ns, floor = _CAMERA_MAP_LAW_MODE.unpack_from(payload, pos)
        pos += _CAMERA_MAP_LAW_MODE.size
        law_modes.append({"line_time_ns": line_time_ns, "frame_length_floor": floor})
    modes: List[Dict[str, Any]] = []
    for _ in range(mode_count):
        (width, height, bits, lanes, rate, data_type, mode_flags, fps1000, value,
         phase) = _CAMERA_MAP_MODE.unpack_from(payload, pos)
        pos += _CAMERA_MAP_MODE.size
        modes.append({"width": width, "height": height, "bit_depth": bits, "lanes": lanes,
                      "rate_mbps": rate, "data_type": data_type,
                      "default": bool(mode_flags & NxsMap.MODE_FLAG_DEFAULT),
                      "triggerable": bool(mode_flags & NxsMap.MODE_FLAG_TRIGGERABLE),
                      "max_fps": fps1000 / 1000, "value": value,
                      "pixel_phase": PIXEL_PHASES[phase] if phase < len(PIXEL_PHASES) else None})
    return {"mode_count": mode_count, "gain_factor": g_factor,
            "gain": {"min": g_min, "max": g_max, "step": g_step},
            "exposure": {"min": e_min, "margin": e_margin},
            "frame_length": {"max": fl_max},
            "params": {name: (None if i == _NONE_U8 else i)
                       for name, i in zip(_CAMERA_MAP_PARAM_NAMES, indices)},
            "law_modes": law_modes, "modes": modes}
