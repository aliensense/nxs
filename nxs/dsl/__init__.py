"""The personality DSL compiler as a package: the field and parameter descriptors, the
bytecode emitter, the measure-loop compiler, the personality base classes, and the
register, cam and stream personalities. `nxs.compiler` re-exports every name."""

from nxs.dsl.errors import CompileError
from nxs.dsl.fields import (
    FIELD_SEMANTICS,
    SEMANTIC_NAMES,
    infer_semantic,
    field_width,
    resolve_field_offsets,
    MAX_OUTPUTS,
    MAX_PATCH_SITES,
    MAX_PARAMS,
    VM_MAX_PROGRAM_SIZE,
    MAX_DRIVER_IMAGE_SIZE,
    MAX_TRAILER_SIZE,
    REG_ADDR_MAX_8,
    REG_ADDR_MAX_16,
    validate_field_layout,
    PWM_FREQ_MIN_HZ,
    PWM_FREQ_MAX_HZ,
    ParamDescriptor,
    PatchEntry,
)
from nxs.dsl.values import U32_MAX, RunValue
from nxs.dsl.compiled import ImageKind, CompiledDriver, compile_report, Sample
from nxs.dsl.emit import (
    _RegAlloc,
    WideRef,
    _Emitter,
    BUS_REGISTER,
    BUS_STREAM,
    OpErrorCode,
    TracedSlice,
)
from nxs.dsl.loop import _ASTCompiler
from nxs.dsl.base import ClickPersonality
from nxs.dsl.register import RegisterClickPersonality, SLEEP_ROW, _source_dir, load_table
from nxs.dsl.camera import CamPersonality
from nxs.dsl.hub import HubDevice
from nxs.dsl.stream import (
    I2cCommandClickPersonality,
    StreamClickPersonality,
    ChecksumDescriptor,
    ChecksumFletcher,
)

__all__ = [
    "CompileError",
    "FIELD_SEMANTICS",
    "SEMANTIC_NAMES",
    "infer_semantic",
    "field_width",
    "resolve_field_offsets",
    "MAX_OUTPUTS",
    "MAX_PATCH_SITES",
    "MAX_PARAMS",
    "VM_MAX_PROGRAM_SIZE",
    "MAX_DRIVER_IMAGE_SIZE",
    "MAX_TRAILER_SIZE",
    "REG_ADDR_MAX_8",
    "REG_ADDR_MAX_16",
    "validate_field_layout",
    "PWM_FREQ_MIN_HZ",
    "PWM_FREQ_MAX_HZ",
    "ParamDescriptor",
    "PatchEntry",
    "U32_MAX",
    "RunValue",
    "ImageKind",
    "CompiledDriver",
    "compile_report",
    "Sample",
    "WideRef",
    "BUS_REGISTER",
    "BUS_STREAM",
    "OpErrorCode",
    "TracedSlice",
    "ClickPersonality",
    "RegisterClickPersonality",
    "SLEEP_ROW",
    "load_table",
    "CamPersonality",
    "HubDevice",
    "I2cCommandClickPersonality",
    "StreamClickPersonality",
    "ChecksumDescriptor",
    "ChecksumFletcher",
]
__all__ += ["_ASTCompiler", "_Emitter", "_RegAlloc", "_source_dir"]
