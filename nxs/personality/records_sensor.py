"""The records a sensor declares about itself: its identity, its modes, its triggers, run parameters, status probes and control rows."""

from __future__ import annotations

import dataclasses
import struct
from typing import Any, Dict, List, Optional, Tuple

from nxs._generated_constants import NxsMap
from nxs.cam.descriptors import Descriptor, to_int
from nxs.personality.records_fields import (
    CONTROL_IDS, MAX_NAME, MODE_PARAM, TRIGGER_PARAM,
    RecordError, _CONTROL_NAMES, _CONTROL_ROW, _MODES_HEAD,
    _MODE_FLAG_DEFAULT, _MODE_FLAG_SERIALIZER_CSI, _MODE_FLAG_TRIGGERABLE,
    _MODE_HEAD, _NONE_U16, _NONE_U8, _RETIRED_TIMING,
    _TIMING_FIELDS, _TRIGGERS_HEAD,
    _family_name, _fixed, _number, _read_text, _register, _register_spec,
    _text, _u, _unit_modes, _unpack)
from nxs.personality.records_identity import (IdentityHead, decode_identity_head,
                                              encode_identity_head)
from nxs.cam.contracts import ContractError

_STATUS_FLAG_INT = 0x01
_STATUS_FLAG_WARN_NONZERO = 0x02
_STATUS_FLAG_MASK = 0x04
_STATUS_FLAG_EXPECT = 0x08
#: The tail a cam personality appends to the IDENTITY head, the `IdentityCamField` layout: register
#: and value widths, the identity register, its width and value, the alive register, default mode.
_IDENTITY_CAM = struct.Struct("<BBHBHHB6x")
assert _IDENTITY_CAM.size == NxsMap.IDENTITY_CAM_SIZE


def mode_values(descriptor: Descriptor) -> Dict[str, int]:
    """Mode name -> the `mode` enum value the image dispatches on."""
    return {name: i for i, name in enumerate(_unit_modes(descriptor))}

def trigger_values(descriptor: Descriptor) -> Dict[str, int]:
    """Trigger preset name -> the `trigger` enum value that selects its
    conversion. The presets are the `trigger:` entries carrying `trigmode`.
    A preset of the registry (`TriggerCode`: freerun, fast, sequential)
    takes its code, so a driver names the conversion from the row alone;
    any other takes the next value past the registry, in declaration
    order."""
    trig = descriptor.raw("trigger") or {}
    presets = [str(k) for k, v in trig.items()
               if isinstance(v, dict) and "trigmode" in v]
    registry = {name.lower(): int(code) for code, name in NxsMap.TriggerCode._NAMES.items()}
    values: Dict[str, int] = {}
    free = len(registry)
    for name in presets:
        if name in registry:
            values[name] = registry[name]
        else:
            values[name] = free
            free += 1
    return values

@dataclasses.dataclass(frozen=True)
class ParamMap:
    """How a personality's params are staged: the `mode` and `trigger`
    params' indices in the image's parameter table (None when the
    personality declares none) with the value behind each mode or
    conversion name."""

    mode_index: Optional[int]
    modes: Dict[str, int]
    trigger_index: Optional[int]
    triggers: Dict[str, int]

def _param_index(params, name: str) -> Optional[int]:
    """The index of the param called `name` in a compiled parameter table
    (`CompiledDriver.params`, or a list of names), None when absent."""
    for i, param in enumerate(params or []):
        if (param if isinstance(param, str) else param.name) == name:
            return i
    return None

def _param_values(params, name: str) -> Optional[List[int]]:
    for param in params or []:
        if not isinstance(param, str) and param.name == name:
            return [int(v) for v in param.values]
    return None

def check_params(descriptor: Descriptor, params) -> None:
    """Refuse a compiled parameter table whose `mode` values do not cover
    the descriptor's unit modes exactly, or whose `trigger` values name a
    preset the descriptor lacks; a single-mode part may declare no `mode`."""
    modes = mode_values(descriptor)
    declared = _param_values(params, MODE_PARAM)
    if declared is None and _param_index(params, MODE_PARAM) is not None:
        return                      # names only: indices are known, values are not
    if declared is None:
        if len(modes) != 1:
            raise RecordError(
                f"{descriptor.name}: the personality declares no {MODE_PARAM!r} "
                f"param but the descriptor offers {len(modes)} modes; "
                f"declare_param({MODE_PARAM!r}, values={sorted(modes.values())}) "
                f"and select() over them")
    elif sorted(declared) != sorted(modes.values()):
        wanted = ", ".join(f"{v}={n}" for n, v in modes.items())
        raise RecordError(
            f"{descriptor.name}: {MODE_PARAM!r} values {sorted(declared)} must "
            f"be the descriptor's modes in order ({wanted})")
    triggers = trigger_values(descriptor)
    declared = _param_values(params, TRIGGER_PARAM)
    if declared is not None:
        unknown = sorted(set(declared) - set(triggers.values()))
        if unknown:
            wanted = ", ".join(f"{v}={n}" for n, v in triggers.items())
            raise RecordError(
                f"{descriptor.name}: {TRIGGER_PARAM!r} values {unknown} name no "
                f"trigger preset (presets in order: {wanted or 'none'})")

def _encode_status(descriptor: Descriptor) -> Optional[bytes]:
    """The STATUS record, None when the descriptor declares no probe."""
    probes = descriptor.raw("status") or []
    if not probes:
        return None
    buf = bytearray(struct.pack("<B", _u(len(probes), 8, "status probes")))
    for probe in probes:
        name = str(probe["name"])
        try:
            addr, width, order = _register_spec(descriptor, str(probe["reg"]))
        except KeyError:
            raise RecordError(f"{descriptor.name}: probe {name!r} reads an undeclared "
                              f"register {probe['reg']!r}") from None
        flags = ((_STATUS_FLAG_INT if str(probe.get("format", "")) == "int" else 0)
                 | (_STATUS_FLAG_WARN_NONZERO if probe.get("warn_nonzero") else 0)
                 | (_STATUS_FLAG_MASK if "mask" in probe else 0)
                 | (_STATUS_FLAG_EXPECT if "expect" in probe else 0))
        buf += _text(name, MAX_NAME, f"probe {name}")
        buf += struct.pack("<HBBB", _u(addr, 16, f"{name} addr"),
                           _u(width, 8, f"{name} width"), order, flags)
        if "mask" in probe:
            buf += struct.pack("<I", _u(to_int(probe["mask"]), 32, f"{name} mask"))
        if "expect" in probe:
            buf += struct.pack("<I", _u(to_int(probe["expect"]), 32, f"{name} expect"))
        decode = probe.get("decode") or {}
        buf += struct.pack("<B", _u(len(decode), 8, f"{name} decode entries"))
        for value, text in decode.items():
            buf += struct.pack("<I", _u(to_int(value), 32, f"{name} decode value"))
            buf += _text(str(text), MAX_NAME, f"{name} decode text")
    return bytes(buf)

def _alive_register(descriptor: Descriptor) -> Optional[int]:
    """The register the alive poll reads: the family's alive register for a
    generic part, STANDBY for a Sony one, None when neither is declared."""
    meta = descriptor.raw("meta") or {}
    program = descriptor.raw("program") or {}
    if _family_name(descriptor) == "sony_imx":
        return descriptor.reg("STANDBY") if "STANDBY" in descriptor.registers else None
    if program.get("alive_reg"):
        return descriptor.reg(str(program["alive_reg"]))
    if meta.get("device_id_reg") is not None:
        return to_int(meta["device_id_reg"])
    return None

def _encode_identity(descriptor: Descriptor, modes: List[str]) -> bytes:
    """The IDENTITY record of a cam personality: the head every image
    carries, the head's address and trigger facts under the descriptor's
    compatible, then the tail of how the head answers."""
    meta = descriptor.raw("meta") or {}
    sync = descriptor.raw("sync") or {}
    timing = (descriptor.raw("trigger") or {}).get("timing") or {}
    default = str(descriptor.raw("default_mode"))
    if default not in modes:
        raise RecordError(f"{descriptor.name}: default_mode {default!r} is not "
                          f"a mode the unit runs")
    id_reg = meta.get("device_id_reg")
    alive = _alive_register(descriptor)
    flags = 0
    if sync.get("takes_trigger"):
        flags |= NxsMap.IDENTITY_FLAG_TAKES_TRIGGER
    if timing.get("pulse_width_is_exposure"):
        flags |= NxsMap.IDENTITY_FLAG_PULSE_EXPOSURE
    head = IdentityHead(address=_u(meta.get("i2c_addr", 0x1A), 7, "meta.i2c_addr"),
                        flags=flags, compatible=descriptor.compatible)
    tail = _IDENTITY_CAM.pack(
        _u(meta.get("reg_bits", 16), 8, "meta.reg_bits"),
        _u(meta.get("val_bits", 8), 8, "meta.val_bits"),
        _NONE_U16 if id_reg is None else _u(id_reg, 16, "meta.device_id_reg"),
        _u(meta.get("device_id_width", 1), 8, "meta.device_id_width"),
        0 if id_reg is None else _u(meta.get("device_id", 0), 16, "meta.device_id"),
        _NONE_U16 if alive is None else _u(alive, 16, "alive register"),
        modes.index(default))
    return encode_identity_head(head, f"{descriptor.name} identity") + tail

def _encode_modes(descriptor: Descriptor, modes: List[str], params=None) -> bytes:
    default = str(descriptor.raw("default_mode"))
    index = _param_index(params, MODE_PARAM)
    buf = bytearray(_MODES_HEAD.pack(
        _NONE_U8 if index is None else _u(index, 8, "mode param index"),
        _u(len(modes), 8, "mode count")))
    for value, name in enumerate(modes):
        mode = descriptor.modes[name]
        geo = mode["geometry"]
        mipi = mode["mipi"]
        timing = mode.get("timing") or {}
        trigger_input = mipi.get("trigger_input")
        flags = ((_MODE_FLAG_DEFAULT if name == default else 0)
                 | (_MODE_FLAG_TRIGGERABLE if trigger_input else 0)
                 | (_MODE_FLAG_SERIALIZER_CSI if mode.get("serializer_csi") else 0))
        sensor_mode = mode.get("sensor_mode")
        present = 0
        body = bytearray()
        for bit, (key, fmt) in enumerate(_TIMING_FIELDS):
            if timing.get(key) is None:
                continue
            present |= 1 << bit
            body += _encode_timing_field(fmt, timing[key], f"{name} timing.{key}")
        buf += _MODE_HEAD.pack(
            value,
            _u(geo["width"], 16, f"{name} width"),
            _u(geo["height"], 16, f"{name} height"),
            _u(geo["bit_depth"], 8, f"{name} bit_depth"),
            _u(geo["lanes"], 8, f"{name} lanes"),
            _u(geo["rate_mbps"], 16, f"{name} rate_mbps"),
            _u(int(str(mipi["data_type"])[3:]), 8, f"{name} data_type"),
            flags,
            _u(mipi.get("embedded_lines", 0), 8, f"{name} embedded_lines"),
            _NONE_U8 if sensor_mode is None else _u(sensor_mode, 8, f"{name} sensor_mode"),
            present)
        buf += body
        buf += _text(name, MAX_NAME, "mode name")
        buf += _text(trigger_input or "", MAX_NAME, f"{name} trigger_input")
    return bytes(buf)

def _encode_timing_field(fmt: str, value: Any, what: str) -> bytes:
    if fmt == "s":
        return _text(str(value), MAX_NAME, what)
    if fmt == "q":
        return struct.pack("<I", _fixed(value, what))
    if fmt == "c":
        num, den = value
        return struct.pack("<HH", _u(num, 16, f"{what} num"), _u(den, 16, f"{what} den"))
    bits = {"B": 8, "H": 16, "I": 32}[fmt]
    return struct.pack("<" + fmt, _u(value, bits, what))

def _decode_timing_field(fmt: str, data: bytes, pos: int, what: str) -> Tuple[Any, int]:
    if fmt == "s":
        return _read_text(data, pos)
    if fmt == "q":
        fixed, pos = _unpack("I", data, pos, what)
        return _number(fixed), pos
    if fmt == "c":
        num, pos = _unpack("H", data, pos, what)
        den, pos = _unpack("H", data, pos, what)
        return [num, den], pos
    return _unpack(fmt, data, pos, what)

def _encode_triggers(descriptor: Descriptor, params=None) -> bytes:
    index = _param_index(params, TRIGGER_PARAM)
    declared = _param_values(params, TRIGGER_PARAM)
    presets = trigger_values(descriptor)
    if index is None:
        offered: List[Tuple[str, int]] = []
    else:
        offered = [(name, value) for name, value in presets.items()
                   if value in set(declared or [])]
    buf = bytearray(_TRIGGERS_HEAD.pack(
        _NONE_U8 if index is None else _u(index, 8, "trigger param index"),
        _u(len(offered), 8, "trigger count")))
    for name, value in offered:
        buf += struct.pack("<B", value) + _text(name, MAX_NAME, "trigger preset")
    return bytes(buf)

def _encode_controls(descriptor: Descriptor) -> bytes:
    from nxs.cam import laws
    try:
        rows = laws.control_rows(descriptor)
    except (KeyError, ContractError) as e:
        raise RecordError(f"{descriptor.name}: the {_family_name(descriptor)} family "
                          f"cannot derive its control rows: {e}") from None
    buf = bytearray(struct.pack("<B", _u(len(rows), 8, "control count")))
    for name, reg, form, params in rows:
        if name not in CONTROL_IDS:
            raise RecordError(
                f"{descriptor.name}: control {name!r} is outside the kernel "
                f"control vocabulary ({', '.join(CONTROL_IDS)})")
        if len(params) > 4:
            raise RecordError(f"{descriptor.name}: control {name!r} carries "
                              f"{len(params)} parameters, the row holds 4")
        addr, width, order = _register_spec(descriptor, reg)
        padded = [int(p) for p in params] + [0] * (4 - len(params))
        buf += _CONTROL_ROW.pack(CONTROL_IDS[name], _u(addr, 16, f"{reg} addr"),
                                 _u(width, 8, f"{reg} width"), order,
                                 _u(form, 8, f"{name} form"),
                                 *(_u(p, 32, f"{name} parameter") for p in padded))
    return bytes(buf)

def _decode_identity(payload: bytes) -> Dict[str, Any]:
    """A cam personality's IDENTITY record, its head and its tail, as the
    descriptor's facts. RecordError for a record that is no camera's: a
    head alone, or a length of another layout."""
    head = decode_identity_head(payload)
    if len(payload) != NxsMap.IDENTITY_SIZE + _IDENTITY_CAM.size:
        raise RecordError(f"IDENTITY record of {len(payload)} B carries no camera's tail")
    (reg_bits, val_bits, id_reg, id_width, device_id, alive,
     default) = _IDENTITY_CAM.unpack_from(payload, NxsMap.IDENTITY_SIZE)
    return {"i2c_addr": head.address, "reg_bits": reg_bits, "val_bits": val_bits,
            "takes_trigger": bool(head.flags & NxsMap.IDENTITY_FLAG_TAKES_TRIGGER),
            "pulse_width_is_exposure": bool(head.flags & NxsMap.IDENTITY_FLAG_PULSE_EXPOSURE),
            "device_id_reg": None if id_reg == _NONE_U16 else id_reg,
            "device_id_width": id_width, "device_id": device_id,
            "alive_reg": None if alive == _NONE_U16 else alive,
            "default_mode": default, "name": head.name, "compatible": head.compatible,
            "product": head.product}

def _decode_triggers(payload: bytes) -> Tuple[Optional[int], Dict[str, int]]:
    if len(payload) < _TRIGGERS_HEAD.size:
        raise RecordError("TRIGGERS record truncated")
    index, count = _TRIGGERS_HEAD.unpack_from(payload)
    pos = _TRIGGERS_HEAD.size
    triggers: Dict[str, int] = {}
    for _ in range(count):
        if pos >= len(payload):
            raise RecordError("TRIGGERS record truncated")
        value = payload[pos]
        name, pos = _read_text(payload, pos + 1)
        triggers[name] = value
    return (None if index == _NONE_U8 else index), triggers

def _decode_modes(payload: bytes) -> Tuple[Optional[int], List[Dict[str, Any]]]:
    if len(payload) < _MODES_HEAD.size:
        raise RecordError("MODES record truncated")
    index, count = _MODES_HEAD.unpack_from(payload)
    pos = _MODES_HEAD.size
    modes = []
    for _ in range(count):
        if pos + _MODE_HEAD.size > len(payload):
            raise RecordError("MODES record truncated")
        (value, width, height, bits, lanes, rate, dtype, flags, embedded,
         sensor_mode, present) = _MODE_HEAD.unpack_from(payload, pos)
        pos += _MODE_HEAD.size
        timing: Dict[str, Any] = {}
        for bit, (key, fmt) in enumerate(_TIMING_FIELDS):
            if present & (1 << bit):
                timing[key], pos = _decode_timing_field(fmt, payload, pos, "MODES")
        name, pos = _read_text(payload, pos)
        trigger_input, pos = _read_text(payload, pos)
        # An image written before the operating points moved to the bench
        # overlay carries them; the shipped facts are the rest.
        for key in _RETIRED_TIMING:
            timing.pop(key, None)
        modes.append({"value": value, "name": name, "width": width,
                      "height": height, "bit_depth": bits, "lanes": lanes,
                      "rate_mbps": rate, "timing": timing,
                      "data_type": f"RAW{dtype}",
                      "default": bool(flags & _MODE_FLAG_DEFAULT),
                      "triggerable": bool(flags & _MODE_FLAG_TRIGGERABLE),
                      "serializer_csi": bool(flags & _MODE_FLAG_SERIALIZER_CSI),
                      "embedded_lines": embedded,
                      "sensor_mode": None if sensor_mode == _NONE_U8 else sensor_mode,
                      "trigger_input": trigger_input or None})
    return (None if index == _NONE_U8 else index), modes

def _mode_entry(m: Dict[str, Any]) -> Dict[str, Any]:
    mipi: Dict[str, Any] = {"data_type": m["data_type"],
                            "embedded_lines": m["embedded_lines"]}
    if m["trigger_input"]:
        mipi["trigger_input"] = m["trigger_input"]
    entry: Dict[str, Any] = {
        "geometry": {"width": m["width"], "height": m["height"],
                     "bit_depth": m["bit_depth"], "lanes": m["lanes"],
                     "rate_mbps": m["rate_mbps"]},
        "mipi": mipi}
    if m["timing"]:
        entry["timing"] = dict(m["timing"])
    if m["sensor_mode"] is not None:
        entry["sensor_mode"] = m["sensor_mode"]
    if m.get("serializer_csi"):
        entry["serializer_csi"] = True
    return entry

def _decode_status(payload: bytes, registers: Dict[str, Any]) -> List[Dict[str, Any]]:
    """The status probes back in the yaml's shape. A probe's register joins
    `registers` under the name already holding that address, width and
    order, else as `STATUS_<NAME>`."""
    if not payload:
        raise RecordError("STATUS record empty")
    count = payload[0]
    pos = 1
    probes: List[Dict[str, Any]] = []
    for _ in range(count):
        name, pos = _read_text(payload, pos)
        addr, pos = _unpack("H", payload, pos, "STATUS")
        width, pos = _unpack("B", payload, pos, "STATUS")
        order, pos = _unpack("B", payload, pos, "STATUS")
        flags, pos = _unpack("B", payload, pos, "STATUS")
        spec = _register(addr, width, order)
        reg = next((known for known, held in registers.items() if held == spec), None)
        if reg is None:
            reg = f"STATUS_{name.upper()}"
            registers[reg] = spec
        probe: Dict[str, Any] = {"name": name, "reg": reg}
        if flags & _STATUS_FLAG_MASK:
            probe["mask"], pos = _unpack("I", payload, pos, "STATUS")
        if flags & _STATUS_FLAG_EXPECT:
            probe["expect"], pos = _unpack("I", payload, pos, "STATUS")
        if flags & _STATUS_FLAG_INT:
            probe["format"] = "int"
        if flags & _STATUS_FLAG_WARN_NONZERO:
            probe["warn_nonzero"] = True
        entries, pos = _unpack("B", payload, pos, "STATUS")
        if entries:
            decode: Dict[int, str] = {}
            for _ in range(entries):
                value, pos = _unpack("I", payload, pos, "STATUS")
                decode[int(value)], pos = _read_text(payload, pos)
            probe["decode"] = decode
        probes.append(probe)
    return probes

def _decode_controls(payload: bytes):
    if not payload:
        raise RecordError("CONTROLS record empty")
    count = payload[0]
    pos = 1
    rows = []
    for _ in range(count):
        if pos + _CONTROL_ROW.size > len(payload):
            raise RecordError("CONTROLS record truncated")
        cid, addr, width, order, form, p0, p1, p2, p3 = _CONTROL_ROW.unpack_from(payload, pos)
        pos += _CONTROL_ROW.size
        name = _CONTROL_NAMES.get(cid)
        if name is None:
            raise RecordError(f"CONTROLS names control id {cid}, which this tool "
                              f"does not know")
        rows.append((name, addr, width, order, form, (p0, p1, p2, p3)))
    return rows
