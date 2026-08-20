"""
Sensor descriptor utilities — driver loading, sample parsing, config loading.
"""

import importlib
import logging
import math
import struct
from typing import Any, Dict, List, Optional, Type

from nxs._generated_constants import Calibration as CalConstants
from nxs._generated_constants import FieldSemantics
from nxs.compiler import SensorDriver

# Identity 3x3, row-major — the no-op affine for one vector bucket.
IDENTITY_M = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

log = logging.getLogger("nxs.descriptor")

# Unknown-rotation codes already warned about (once per code, not per sample).
_WARNED_ROTATIONS: set = set()


FNV_PRIME = 0x01000193


def fnv1a32(name: str) -> int:
    """FNV-1a 32-bit hash of a driver name. Must match the firmware's
    `cal::fnv1a32` so both appliers derive one verdict."""
    h = 0x811C9DC5
    for byte in name.encode():
        h ^= byte
        h = (h * FNV_PRIME) & 0xFFFFFFFF
    return h


def driver_tag(name: Optional[str], bus: int = 0, address: int = 0) -> int:
    """The calibration record's sensor-identity tag: driver name plus the
    attachment point it is bound to.

    The name alone cannot separate two identical parts on one panel — both
    would hash the same and the guard could not refuse a swap between them.
    Mirrors the firmware's `cal::driver_tag`, continuing the same FNV-1a walk
    over the bus kind and the latched address. `None` is the unguarded 0.
    """
    if name is None or name in ("", "-"):
        return 0
    h = fnv1a32(name)
    h = ((h ^ (bus & 0xFF)) * FNV_PRIME) & 0xFFFFFFFF
    h = ((h ^ (address & 0xFF)) * FNV_PRIME) & 0xFFFFFFFF
    return h


# ── Type sizes for sample parsing ───────────────────────────

_TYPE_INFO = {
    'int8':    {'size': 1, 'struct_le': '<b', 'struct_be': '>b'},
    'uint8':   {'size': 1, 'struct_le': '<B', 'struct_be': '>B'},
    'int16':   {'size': 2, 'struct_le': '<h', 'struct_be': '>h'},
    'uint16':  {'size': 2, 'struct_le': '<H', 'struct_be': '>H'},
    'int32':   {'size': 4, 'struct_le': '<i', 'struct_be': '>i'},
    'uint32':  {'size': 4, 'struct_le': '<I', 'struct_be': '>I'},
    'float32': {'size': 4, 'struct_le': '<f', 'struct_be': '>f'},
    'float64': {'size': 8, 'struct_le': '<d', 'struct_be': '>d'},
    'string':  {'size': 0},  # variable — uses 'count' field
}


def field_size(field: dict) -> int:
    """Byte width of one output field in a sample. Numeric widths come
    from the type; string fields use their driver-declared 'count'."""
    if field.get('type') == 'string':
        return int(field.get('count', 0))
    return _TYPE_INFO.get(field.get('type', 'int16'), {}).get('size', 0)


def is_decodable(fields: List[dict]) -> bool:
    """True if every field's type is one `parse_sample` can handle. A
    device may serve a field-type code newer than this tool knows; such
    a set isn't decodable here (the local driver file may still be), so
    callers fall back rather than crash."""
    return all(f.get('type', 'int16') in _TYPE_INFO for f in fields)


def sample_width(fields: List[dict]):
    """Sample extent derived from the descriptors — the furthest field
    end (offsets may gap; a field without one packs after the previous)
    — or None if a field's width is indeterminate: a string with
    unknown count (0/absent), which by convention means 'consume the
    rest of the buffer'. Callers use None to mean 'no fixed total
    derivable' rather than silently undercounting a variable-width
    field as 0 bytes."""
    end = 0
    offset = 0
    for f in fields:
        if f.get('type') == 'string' and not f.get('count'):
            return None
        offset = int(f.get('byte_off', offset))
        offset += field_size(f)
        end = max(end, offset)
    return end


def load_driver(name: str) -> Type[SensorDriver]:
    """Load a driver class by name from nxs.drivers.

    Returns the SensorDriver subclass *defined in* the loaded module
    (skipping base classes like RegisterDriver / I2cCommandDriver /
    StreamDriver that the driver imports for inheritance). For
    modules with multiple driver classes, returns the alphabetically
    first one — which is unique in current shipped drivers.

    Usage:
        cls = load_driver("iam20680")
        drv = cls()
        result = drv.compile(config)
    """
    module = importlib.import_module(f"nxs.drivers.{name}")
    for attr_name in sorted(dir(module)):
        obj = getattr(module, attr_name)
        if (isinstance(obj, type) and
                issubclass(obj, SensorDriver) and
                obj is not SensorDriver and
                obj.__module__ == module.__name__):
            return obj
    raise ImportError(
        f"No SensorDriver subclass defined in "
        f"nxs.drivers.{name}")


def effective_scale_fields(output_fields: List[dict], params: list) -> List[dict]:
    """Fold each field's live-param scaling into its scale.

    A field that names a `scale_param` (or carries a `scale_param_index`)
    has a *base* scale; its effective scale is `base * param.current`. This
    is the host mirror of the firmware's `effective_scale`: a device serves
    descriptors with the param already folded in (the I²C window and
    GetOutputInfo both expose the effective scale), so a host holding the
    raw NXS descriptors must fold the same way to decode identical SI.
    Returns new field dicts; fields with no linked param are copied as-is.
    """
    by_name = {p.name: p for p in params}
    out = []
    for f in output_fields:
        g = dict(f)
        p = None
        if f.get('scale_param'):
            p = by_name.get(f['scale_param'])
        elif f.get('scale_param_index') not in (None, 0xFF):
            idx = f['scale_param_index']
            if idx < len(params):
                p = params[idx]
        if p is not None:
            g['scale'] = f.get('scale', 1.0) * p.current
        out.append(g)
    return out


def apply_calibration(values: Dict[str, Any], output_fields: List[dict],
                      calibration, active_tag: int = 0) -> Dict[str, Any]:
    """Apply the device's per-vector affine calibration to decoded values.

    The host twin of the firmware's SI-tier stage: groups the decoded
    fields by semantic bucket and applies ``R·(M·v + b)`` per vector plus
    the encoder zero-offset on the angle scalar, so an I2C/RawSample
    consumer gets the same physical values the Cyphal SI subjects carry.
    ``calibration`` is a ``CalibrationRecord``-shaped object;
    ``driver_name`` drives the tag guard exactly like the firmware (a
    tagged bucket solved for another driver contributes only the mounting
    rotation). Fields with no vector semantic pass through untouched.

    A record whose orientation code postdates this tool (the rotation
    vocabulary is append-only) disables the whole post-pass: the mount
    composes into every bucket, so no part of the record can be applied
    correctly, and raw values are honest where wrong-rotation values are
    not. Warned once per unknown code.
    """
    rot = CalConstants.Rotation.MATRIX.get(calibration.orientation)
    if rot is None:
        if calibration.orientation not in _WARNED_ROTATIONS:
            _WARNED_ROTATIONS.add(calibration.orientation)
            log.warning(
                "calibration record carries unknown rotation code %d "
                "(newer device?) — decoding uncalibrated; update nxs",
                calibration.orientation)
        return dict(values)
    active_hash = active_tag
    out = dict(values)

    groups: Dict[int, Dict[int, str]] = {}
    angle_names = []
    for f in output_fields:
        name = f.get('name')
        if name not in values or not isinstance(values[name], (int, float)):
            continue
        sem = f.get('semantic', 0)
        bucket = FieldSemantics.FIELD_SEMANTIC_BUCKET.get(sem, 0)
        if FieldSemantics.SubjectBucket.ACCELERATION <= bucket \
                <= FieldSemantics.SubjectBucket.MAGNETIC_FIELD:
            slot = FieldSemantics.FIELD_SEMANTIC_SLOT.get(sem, 0)
            groups.setdefault(bucket - 1, {})[slot] = name
        elif sem == FieldSemantics.FieldSemantic.ANGLE:
            angle_names.append(name)

    for vec, slots in groups.items():
        if set(slots) != {0, 1, 2}:
            # A rotation or bias applied to a partial vector would mix the
            # present components with fabricated zeros — an incomplete
            # vector passes through raw, mirroring the firmware applier.
            continue
        active = (calibration.bucket_guard(vec, active_hash)
                  != CalConstants.BucketGuard.STALE)
        m = calibration.m[vec] if active else IDENTITY_M
        b = calibration.b[vec] if active else (0.0, 0.0, 0.0)
        v = [values[slots[i]] for i in range(3)]
        affine = [m[i * 3] * v[0] + m[i * 3 + 1] * v[1] + m[i * 3 + 2] * v[2]
                  + b[i] for i in range(3)]
        rotated = [rot[i * 3] * affine[0] + rot[i * 3 + 1] * affine[1]
                   + rot[i * 3 + 2] * affine[2] for i in range(3)]
        for i, name in slots.items():
            out[name] = rotated[i]

    encoder_active = (calibration.bucket_guard(
            len(calibration.driver_tags), active_hash)
            != CalConstants.BucketGuard.STALE)
    if encoder_active and calibration.encoder_zero:
        for name in angle_names:
            out[name] = (values[name] + calibration.encoder_zero) % math.tau

    return out


def parse_sample(raw: bytes, output_fields: List[dict],
                 calibration=None,
                 active_tag: int = 0) -> Dict[str, Any]:
    """Parse a raw sample using the output field descriptors.

    Returns a dict of {field_name: value}. Numeric fields are scaled
    floats. String fields are decoded ASCII strings (0xFF trimmed). The
    field `scale` is taken as-is: device-served descriptors already carry
    the effective (param-folded) scale; for raw NXS descriptors, fold
    first with `effective_scale_fields`. A ``calibration`` record applies
    the per-vector affine post-pass (`apply_calibration`); None keeps
    the descriptor-tier decode unchanged.
    """
    result = {}
    offset = 0
    for field in output_fields:
        ftype = field.get('type', 'int16')
        byte_order = field.get('byte_order', 'big')
        # Descriptors carry an explicit byte position per field (gaps
        # are legal); one without it packs after the previous field.
        offset = int(field.get('byte_off', offset))

        if ftype == 'string':
            # count == 0 (or absent) means "unknown" (e.g. firmware
            # predating the count field) — consume the rest of the buffer.
            # max(0, …) guards a byte_off past the sample end: a negative
            # count would walk offset backwards and corrupt later fields.
            count = field.get('count') or max(0, len(raw) - offset)
            chunk = raw[offset:offset + count]
            # Strip 0xFF (uninitialized) and 0x00 (null) padding from the
            # tail only — filtering all interior occurrences would silently
            # drop legitimate payload bytes in binary-text protocols.
            text = chunk.rstrip(b'\xff\x00')
            result[field['name']] = text.decode('ascii', errors='replace')
            offset += count
        else:
            info = _TYPE_INFO.get(ftype)
            if info is None:
                raise ValueError(
                    f"unsupported output field type {ftype!r}; "
                    f"update nxs or re-upload the driver")
            size = info['size']
            if offset + size > len(raw):
                offset += size   # advance so a later field can't decode at a stale offset
                continue
            fmt = info['struct_be'] if byte_order == 'big' else info['struct_le']
            raw_val = struct.unpack_from(fmt, raw, offset)[0]
            scale = field.get('scale', 1.0)
            field_offset = field.get('offset', 0.0)
            result[field['name']] = raw_val * scale + field_offset
            offset += size

    if calibration is not None:
        result = apply_calibration(result, output_fields, calibration, active_tag)

    return result


def parse_sample_raw(raw: bytes, output_fields: List[dict]) -> Dict[str, Any]:
    """Parse a raw sample, returning unscaled values.

    Numeric fields return integers. String fields return bytes.
    """
    result = {}
    offset = 0
    for field in output_fields:
        ftype = field.get('type', 'int16')
        byte_order = field.get('byte_order', 'big')
        offset = int(field.get('byte_off', offset))

        if ftype == 'string':
            count = field.get('count') or max(0, len(raw) - offset)
            result[field['name']] = raw[offset:offset + count]
            offset += count
        else:
            info = _TYPE_INFO.get(ftype)
            if info is None:
                raise ValueError(
                    f"unsupported output field type {ftype!r}; "
                    f"update nxs or re-upload the driver")
            size = info['size']
            if offset + size > len(raw):
                offset += size   # advance so a later field can't decode at a stale offset
                continue
            fmt = info['struct_be'] if byte_order == 'big' else info['struct_le']
            result[field['name']] = struct.unpack_from(fmt, raw, offset)[0]
            offset += size

    return result


def sample_size_from_fields(output_fields: List[dict]) -> int:
    """Sample extent in bytes from output field descriptors — the furthest
    field end. Fields carry explicit byte offsets (gaps are legal); one
    without an offset packs after the previous field."""
    end = 0
    offset = 0
    for field in output_fields:
        ftype = field.get('type', 'int16')
        offset = int(field.get('byte_off', offset))
        width = field.get('count', 0) if ftype == 'string' \
            else _TYPE_INFO[ftype]['size']
        offset += width
        end = max(end, offset)
    return end
