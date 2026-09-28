"""Sensor descriptor utilities: driver loading, sample parsing, config loading."""

import importlib
import logging
import math
import struct
from typing import Any, Dict, List, Optional, Type

from nxs._generated_constants import Calibration as CalConstants
from nxs._generated_constants import FieldSemantics
from nxs.compiler import SensorDriver

# Identity 3x3, row-major: the no-op affine for one vector bucket.
IDENTITY_M = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

log = logging.getLogger("nxs.descriptor")

# Unknown-rotation codes already warned about (once per code, not per sample).
_WARNED_ROTATIONS: set = set()


FNV_PRIME = 0x01000193


def fnv1a32(name: str) -> int:
    """FNV-1a 32-bit hash of a driver name; the same hash the device computes."""
    h = 0x811C9DC5
    for byte in name.encode():
        h ^= byte
        h = (h * FNV_PRIME) & 0xFFFFFFFF
    return h


def driver_tag(name: Optional[str], bus: int = 0, address: int = 0) -> int:
    """The calibration record's sensor-identity tag: the FNV-1a walk over the
    driver name, then the bus kind and the latched address, as the device
    computes it. None gives the unguarded 0."""
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
    'string':  {'size': 0},  # variable; uses 'count' field
}


def field_size(field: dict) -> int:
    """Byte width of one output field in a sample. Numeric widths come
    from the type; string fields use their driver-declared 'count'."""
    if field.get('type') == 'string':
        return int(field.get('count', 0))
    return _TYPE_INFO.get(field.get('type', 'int16'), {}).get('size', 0)


def is_decodable(fields: List[dict]) -> bool:
    """True if every field's type is one `parse_sample` can handle; a device may
    serve a newer field-type code, and callers fall back."""
    return all(f.get('type', 'int16') in _TYPE_INFO for f in fields)


def sample_width(fields: List[dict]):
    """Sample extent from the descriptors, the furthest field end, or None when
    a string field has no count (it consumes the rest of the buffer)."""
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
    """Load the SensorDriver subclass defined in ``nxs.drivers.<name>`` (base
    classes it imports are skipped; the alphabetically first when several)."""
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
    """Fold each field's live-param scaling into its scale: a field naming a
    `scale_param` (or `scale_param_index`) gets `base * param.current`, the
    same fold a device applies before serving descriptors."""
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
    """Apply the device's per-vector affine calibration ``R·(M·v + b)`` and the
    encoder zero-offset to decoded values, as the device's SI stage does.
    An unknown orientation code disables the pass (warned once per code)."""
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
            # A rotation or bias applied to a partial vector would mix real
            # components with zeros; an incomplete vector passes through raw.
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
    """Parse a raw sample into {field_name: value}: numeric fields scaled
    floats, strings ASCII with 0xFF/0x00 tail trimmed. ``scale`` is taken as-is;
    a ``calibration`` record applies `apply_calibration`."""
    result = {}
    offset = 0
    for field in output_fields:
        ftype = field.get('type', 'int16')
        byte_order = field.get('byte_order', 'big')
        # Descriptors carry an explicit byte position per field (gaps
        # are legal); one without it packs after the previous field.
        offset = int(field.get('byte_off', offset))

        if ftype == 'string':
            # count == 0 (or absent) means unknown: consume the rest of the buffer.
            # max(0, ...) guards a byte_off past the sample end.
            count = field.get('count') or max(0, len(raw) - offset)
            chunk = raw[offset:offset + count]
            # Strip 0xFF and 0x00 padding from the tail only; interior bytes
            # are payload.
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
    """Parse a raw sample into unscaled values: integers for numeric fields,
    bytes for strings."""
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
    """Sample extent in bytes from output field descriptors: the furthest field
    end. A field without an explicit byte offset packs after the previous."""
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


def driver_descriptor(name: str, drivers_dir: Optional[str] = None) -> dict:
    """The personality's YAML fact sheet (meta + params) by name: the personality store
    first, then the built-in package. No driver code is imported."""
    import importlib.util
    import os

    import yaml

    from nxs.suite import personality_file

    # A personality is a .py + .yaml pair: the store's descriptor counts only
    # beside a store driver, so `tune` offers what gets compiled.
    store_py = personality_file(name, "py", drivers_dir)
    if store_py is not None:
        # The store's code wins, so its descriptor must answer for it; the
        # packaged one describes other parameters.
        beside = os.path.splitext(store_py)[0] + ".yaml"
        if not os.path.exists(beside):
            raise FileNotFoundError(
                f"no parameter descriptor beside {store_py} — a personality is a "
                f".py and a .yaml; install both")
        return _load_descriptor_file(beside)
    spec = importlib.util.find_spec(f"nxs.drivers.{name}")
    if spec and spec.origin:
        packaged = os.path.splitext(spec.origin)[0] + ".yaml"
        if os.path.exists(packaged):
            return _load_descriptor_file(packaged)
    raise FileNotFoundError(f"no descriptor for driver {name!r}")


def _load_descriptor_file(path: str) -> dict:
    """A driver descriptor read and validated against the unit-driver schema;
    a malformed file fails here, naming the file."""
    import yaml

    from nxs import schemas

    with open(path, encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    problems = schemas.findings(doc, schemas.UNIT_DRIVER, where=path)
    # A law the schema cannot say: parameter names are the wire's indices,
    # so a duplicate would compile as one parameter while tune offered two.
    seen = set()
    params = doc.get("params") if isinstance(doc, dict) else None
    for entry in params if isinstance(params, list) else []:
        name = entry.get("name") if isinstance(entry, dict) else None
        if name in seen:
            problems.append(f"{path}.params: duplicate parameter {name!r}")
        seen.add(name)
        if isinstance(entry, dict):
            problems.extend(f"{path}.params[{name}]: {p}" for p in _param_laws(entry))
    if problems:
        raise ValueError("; ".join(problems))
    return doc


def _param_laws(entry: dict) -> list:
    """The value laws the compiler applies to a parameter, judged on the
    descriptor alone: an enum default is one of its values, a range is
    `[min, max]` with the default inside it."""
    values = entry.get("values")
    default = entry.get("default")
    if not isinstance(values, list) or isinstance(default, bool) or not isinstance(default, int):
        return []                          # shape problems: the schema's
    if entry.get("type", "enum") == "range":
        if len(values) != 2 or not all(isinstance(v, int) for v in values):
            return []
        lo, hi = values
        if lo > hi:
            return [f"range [{lo}, {hi}] is reversed (min must not exceed max)"]
        if not lo <= default <= hi:
            return [f"default {default} is outside its range [{lo}, {hi}]"]
        return []
    if default not in values:
        return [f"default {default} is not one of its values {values}"]
    return []


def driver_params(cls) -> list:
    """The params table for a driver class: the YAML sibling of the class's
    defining file, else the built-in descriptor matching the class name."""
    import importlib.util
    import os
    import sys

    import yaml

    candidates = []
    for klass in cls.__mro__:
        module = sys.modules.get(klass.__module__)
        src = getattr(module, "__file__", None)
        if not src:
            # A class whose module is not registered (a host-local
            # driver loaded by path) still knows its own source file.
            try:
                import inspect
                src = inspect.getsourcefile(klass)
            except (TypeError, OSError):
                src = None
        if src:
            candidates.append(os.path.splitext(src)[0] + ".yaml")
        spec = None
        try:
            spec = importlib.util.find_spec(
                f"nxs.drivers.{klass.__name__.lower()}")
        except (ImportError, ValueError):
            pass
        if spec and spec.origin:
            candidates.append(os.path.splitext(spec.origin)[0] + ".yaml")
    for path in candidates:
        if os.path.exists(path):
            return _load_descriptor_file(path).get("params") or []
    raise FileNotFoundError(
        f"no parameter descriptor for {cls.__name__} (looked beside "
        f"{', '.join(dict.fromkeys(candidates)) or 'nothing resolvable'})")
