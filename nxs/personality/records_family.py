"""The records a law family contributes: its laws, the standby-wrapped program it composes, and the capture table the kernel driver reads."""

from __future__ import annotations

import struct
from typing import Any, Dict, List, Tuple

from nxs.cam.descriptors import Descriptor
from nxs.personality.records_fields import (
    FAMILY_IDS, PIXEL_PHASES, RecordError, _CAPTURE_ENTRY, _CAPTURE_ENTRY_V1,
    _CAPTURE_FLAG_CLOCK_NONCONTINUOUS, _CAPTURE_HEAD, _CHROMACITIES, _CONTROL_REGISTERS, _DELTA_SCALAR,
    _DELTA_TABLE, _LAWS_HEAD, _LAW_FIELDS, _NONE_U8, _PROGRAM_ADBIT,
    _PROGRAM_ADBIT_CODE, _PROGRAM_BLKLEVEL, _PROGRAM_BLKLEVEL_VALUE,
    _PROGRAM_FLAGS, _PROGRAM_HEAD, _PROGRAM_REPAIR, _PROGRAM_TAIL,
    _SYNC_ROLES, _TEST_PATTERN_SELECT, _TRIGGER_SLOTS, _VINT_BITS,
    _family_name, _fixed, _number, _read_table, _register, _register_spec,
    _table, _u, _unpack)


def _encode_laws(descriptor: Descriptor) -> bytes:
    family = _family_name(descriptor)
    if family not in FAMILY_IDS:
        raise RecordError(f"{descriptor.name}: unknown chip family {family!r}; "
                          f"have {sorted(FAMILY_IDS)}")
    limits = descriptor.limits
    present = 0
    body = bytearray()
    waits = list((limits.get("captured_waits") or {}).items())
    for bit, (key, fmt) in enumerate(_LAW_FIELDS):
        value = limits.get(key)
        if value is None:
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
            names = [k for k, _ in waits]
            body += struct.pack("<B", _u(len(value), 8, what))
            for reg in value:
                if str(reg) not in names:
                    raise RecordError(f"{what}: {reg!r} is not a captured wait register")
                body += struct.pack("<B", names.index(str(reg)))
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

def _decode_laws(payload: bytes) -> Tuple[int, Dict[str, Any]]:
    """`(family id, limits)`: the declared limits the LAWS record carries."""
    if not payload:
        return FAMILY_IDS["generic"], {}
    if len(payload) < _LAWS_HEAD.size:
        raise RecordError("LAWS record truncated")
    family, present = _LAWS_HEAD.unpack_from(payload)
    pos = _LAWS_HEAD.size
    limits: Dict[str, Any] = {}
    waits: Dict[str, int] = {}
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
            count, pos = _unpack("B", payload, pos, "LAWS")
            names = list(waits)
            regs = []
            for _ in range(count):
                i, pos = _unpack("B", payload, pos, "LAWS")
                if i >= len(names):
                    raise RecordError("LAWS names a wait register it does not carry")
                regs.append(names[i])
            limits[key] = regs
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
    return family, limits

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
