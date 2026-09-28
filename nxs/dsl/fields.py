"""Output-field semantics, the image limits, and the parameter and patch descriptors."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from nxs._generated_constants import FieldSemantics, NxsDriverImage
from nxs.dsl.errors import CompileError


# ── Output-field semantics ─────────────────────────────────────

# Semantic codes carried per output field in the NXS image and exposed through
# both host interfaces. Code 0 (generic) is the fallback for an unknown name.
FIELD_SEMANTICS = {name.lower(): code
                   for code, name in FieldSemantics.FieldSemantic._NAMES.items()}
SEMANTIC_NAMES = {v: k for k, v in FIELD_SEMANTICS.items()}

# Field-name spellings that drivers use for a canonical semantic.
_SEMANTIC_ALIASES = {
    'temp': 'temperature',
    'freq': 'frequency',
    'weight': 'mass',
    'range': 'distance',
    'flow_rate': 'flow',
    'itow': 'time_of_week',
}


def infer_semantic(field_name: str) -> int:
    """Semantic code for an output-field name, 0 (generic) when unknown.
    Matching is exact, case-insensitive, after alias folding."""
    key = field_name.lower()
    return FIELD_SEMANTICS.get(_SEMANTIC_ALIASES.get(key, key), 0)


def field_width(field: dict) -> int:
    """Byte width of one output field: type width for numerics, the
    declared `count` for strings."""
    sizes = {'int8': 1, 'uint8': 1, 'int16': 2, 'uint16': 2,
             'int32': 4, 'uint32': 4, 'float32': 4, 'float64': 8}
    t = field.get('type', 'int16')

    return int(field.get('count', 0)) if t == 'string' else sizes.get(t, 2)


def resolve_field_offsets(fields: list) -> list:
    """Stamp each field's `byte_off`: sequential after the previous field
    unless set explicitly (the `at=` path). Idempotent."""
    off = 0
    for field in fields:
        if 'byte_off' not in field:
            field['byte_off'] = off
        off = int(field['byte_off']) + field_width(field)

    return fields


# Descriptor caps shared with the firmware image parser.
MAX_OUTPUTS = NxsDriverImage.MAX_OUTPUTS
MAX_PATCH_SITES = NxsDriverImage.MAX_PATCH_SITES
MAX_PARAMS = NxsDriverImage.MAX_PARAMS
VM_MAX_PROGRAM_SIZE = NxsDriverImage.VM_MAX_PROGRAM_SIZE
VM_HOST_PROGRAM_SIZE = NxsDriverImage.VM_HOST_PROGRAM_SIZE
MAX_DRIVER_IMAGE_SIZE = NxsDriverImage.MAX_DRIVER_IMAGE_SIZE
MAX_TRAILER_SIZE = NxsDriverImage.MAX_TRAILER_SIZE

# Register-address ceilings by I²C address width; the emitter refuses a
# register past the profile's.
REG_ADDR_MAX_8 = 0xFF
REG_ADDR_MAX_16 = 0xFFFF


def validate_field_layout(fields: list, buf_size: int):
    """Reject a resolved field set the firmware can't serve: too many fields,
    a field past the sample buffer, or two fields overlapping."""
    if len(fields) > MAX_OUTPUTS:
        raise CompileError(
            f"set_output declares {len(fields)} fields; the firmware "
            f"descriptor table holds {MAX_OUTPUTS}.")
    spans = []
    for f in fields:
        start = int(f['byte_off'])
        end = start + field_width(f)
        if start < 0 or end > buf_size:
            raise CompileError(
                f"set_output field {f['name']!r} spans sample bytes "
                f"[{start}, {end}), outside the {buf_size}-byte sample "
                f"buffer.")
        spans.append((start, end, f['name']))
    spans.sort()
    for (s0, e0, n0), (s1, e1, n1) in zip(spans, spans[1:]):
        if s1 < e0:
            raise CompileError(
                f"set_output fields {n0!r} and {n1!r} overlap: "
                f"[{s0}, {e0}) and [{s1}, {e1}). Each sample byte belongs "
                f"to one field.")


# PWM drive bounds; the firmware validates each `set pwm_freq` against them.
PWM_FREQ_MIN_HZ = 500
PWM_FREQ_MAX_HZ = 25000


# ── Parameter descriptors ──────────────────────────────────────

@dataclass
class ParamDescriptor:
    """Describes a configurable parameter with valid values."""
    name: str
    param_type: str          # "enum" or "range"
    values: list             # enum: [2,4,8,16]; range: [min,max,step]
    default: Any
    current: Any
    unit: str = ""
    kind: str = "reload"     # "reload" = patch + VM restart; "live" = applied in place


@dataclass
class PatchEntry:
    """Maps a bytecode offset to a config parameter."""
    offset: int              # byte offset in bytecode
    param_name: str          # which parameter
    value_map: Dict          # config_value → byte_value(s) to write
    reg: Optional[int] = None  # target register (register writes only)
    size: int = 1            # patched width in bytes (1, 2, or 4)


