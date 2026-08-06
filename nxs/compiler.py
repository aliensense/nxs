"""
Hybrid trace + AST compiler for NXS sensor driver bytecode.

- probe() and configure() are traced: executed with a recording register proxy.
- measure() is AST-compiled: parsed and pattern-matched into opcodes.

The driver class is a "datasheet in code" — it knows all register
addresses, modes, and scale factors. The YAML config selects the mode;
compile(config) produces bytecode + output field descriptors with the
correct scales.
"""

from __future__ import annotations

import ast
import inspect
import struct
import textwrap
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from nxs._generated_constants import FieldSemantics, NxsDriverImage
from nxs.opcodes import Op
from nxs.profiles import I2cProfile, SpiProfile, UartProfile


# ── Output-field semantics ─────────────────────────────────────

# Semantic codes carried per output field in the NXS image and exposed
# through both host interfaces (the Cyphal GetOutputInfo service, the I2C
# descriptor window). They let a consumer categorize fields — "this is the
# accelerometer X axis" — without parsing field names. Code 0 (generic) is
# the fallback for anything unrecognized.
#
# Single source of truth: constants/field_semantics.yaml, generated into a
# FieldSemantic enum (C++) and FieldSemantics (Python) from one source, so the
# two bindings can't drift. Append only — never renumber.
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
}


def infer_semantic(field_name: str) -> int:
    """Semantic code for an output-field name; 0 (generic) when the
    name doesn't match a known semantic. Inference is by exact name
    (case-insensitive, after alias folding) so a rename in a driver
    can't silently shift a field to a wrong category."""
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
    """Stamp each field's `byte_off` — its byte position within the
    sample. Fields without one pack sequentially after the previous
    field; an explicit `byte_off` (the `at=` path) is kept as-is.
    Idempotent: re-resolving an already-stamped list is a no-op."""
    off = 0
    for field in fields:
        if 'byte_off' not in field:
            field['byte_off'] = off
        off = int(field['byte_off']) + field_width(field)

    return fields


# Shared with the firmware parser via the SSOT (constants/driver_image.yaml);
# the generator emits it to both this module's source and DriverImage.h, so it
# can't drift.
MAX_OUTPUTS = NxsDriverImage.MAX_OUTPUTS
MAX_PATCH_SITES = NxsDriverImage.MAX_PATCH_SITES


def validate_field_layout(fields: list, buf_size: int):
    """Reject a resolved field set the firmware can't serve: more fields
    than the descriptor table holds, a field past the sample buffer, or
    two fields overlapping (each byte belongs to one field)."""
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


# PWM drive bounds. The TIM1 prescaler in the device timer configuration fixes the
# achievable frequency window; pwm_freq's declared range is the safety
# bound the firmware validates each `set pwm_freq` against.
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


# ── Compiled output ─────────────────────────────────────────

@dataclass
class CompiledDriver:
    bytecode: bytes
    sample_size: int
    name: str
    config: dict = field(default_factory=dict)
    output_fields: List[dict] = field(default_factory=list)
    params: List[ParamDescriptor] = field(default_factory=list)
    patch_map: List[PatchEntry] = field(default_factory=list)
    # Probe metadata — pulled from the driver's class attributes
    # (`WHO_AM_I_REG`, `WHO_AM_I_VALUES`, `I2C_ADDRS`). Lets NXS
    # auto-detect a sensor's I²C address on load and sanity-check
    # that the right IC is on the bus. Empty for StreamDrivers and
    # drivers that don't declare these attrs.
    who_am_i_reg: int = 0
    who_am_i_values: List[int] = field(default_factory=list)
    i2c_addrs: List[int] = field(default_factory=list)
    # Human-readable justification for opting out of the WHO_AM_I
    # probe (i.e. for `WHO_AM_I_VALUES = []`). Populated from the
    # driver class's `WHO_AM_I_SKIP_REASON` attribute, which the
    # compiler requires whenever WHO_AM_I_VALUES is empty. Surfaces
    # the audit trail so Claude-authored opt-outs aren't silent.
    who_am_i_skip_reason: Optional[str] = None
    # Optional bus-config trailer for the NXS image: one register-access
    # profile per bus the chip supports, built at compile time from the
    # driver's SPI_PROFILE / I2C_PROFILE descriptors or conventional
    # wire-shape defaults. Firmware applies the profile whose kind matches
    # the selected `bus`. None means no trailer at all — a stream driver
    # with no UART profile, or a legacy image; a register driver always
    # emits one profile per bus, so it never leaves this None.
    bus_config: Optional[list] = None


class Sample:
    """Marker returned by measure() to signal 'commit sample buffer'."""

    def __init__(self, raw=None):
        self.raw = raw


# ── Register allocator ──────────────────────────────────────

class _RegAlloc:
    """Maps named variables to VM register indices (r0..r7).

    Scratch names would otherwise accumulate in `_map` for the
    driver's whole lifetime: a driver that chains `read_n + .expect
    + compute_checksum(Fletcher) + read_until` allocates ≥11 names —
    past the firmware's `VM_NUM_REGS = 8` — even though the live
    count at any single point is at most ~5. `scope()` solves this
    by giving each helper a `with`-block whose contents are released
    on exit.

    Three binding lifetimes:

    - **Pinned** (`pin(name)`) — loop-carried registers declared by
      `persistent()`. Allocated top-down from r7, survive `reset()`,
      so the configure()-time init and the measure() body resolve
      the name to the same register regardless of what either
      section allocates.
    - **Section scratch** (`get(name)` outside a scope) — lasts until
      the next `reset()` at the configure→measure boundary.
    - **Scoped scratch** (`get(name)` inside `scope()`) — released at
      scope exit.
    """

    # Mirrors SensorDriverVM.h:VM_NUM_REGS, the firmware register-file size;
    # pinned to the header by test_vm_layout_constants_match_firmware_header.
    VM_NUM_REGS = 8

    def __init__(self):
        self._map: Dict[str, int] = {}
        self._next = 0
        self._pinned: Dict[str, int] = {}
        self._pin_next = self.VM_NUM_REGS - 1

    def pin(self, name: str) -> int:
        """Bind `name` to a register that survives `reset()` — the
        loop-carried slots `persistent()` declares.

        Pinned names allocate from the top of the register file (r7
        down) while section scratch allocates from the bottom (r0 up),
        so the binding holds across the configure() → measure()
        boundary regardless of what either section allocates.
        """
        if name in self._pinned:
            return self._pinned[name]
        if name in self._map:
            raise CompileError(f"'{name}' is already in use as a scratch "
                               f"variable — declare persistent() before "
                               f"first use")
        if self._pin_next < self._next:
            raise CompileError(f"Out of VM registers (max {self.VM_NUM_REGS}), "
                               f"cannot pin '{name}'")
        self._pinned[name] = self._pin_next
        self._pin_next -= 1
        return self._pinned[name]

    def get(self, name: str) -> int:
        """Allocate (or look up) a register for `name`.

        Pinned names resolve to their reserved slot. Otherwise, inside
        a `scope()` block the binding is released at scope exit;
        outside any scope it lasts until the next `reset()`.
        """
        if name in self._pinned:
            return self._pinned[name]
        if name not in self._map:
            if self._next > self._pin_next:
                raise CompileError(f"Out of VM registers (max {self.VM_NUM_REGS}), "
                                   f"cannot allocate '{name}'")
            self._map[name] = self._next
            self._next += 1
        return self._map[name]

    @contextmanager
    def scope(self):
        """Open a scratch-register scope.

        Names allocated via `get(name)` inside the block are
        released at block exit, freeing their register slots for
        the next `scope()`. Wrap each framing helper's body in a
        `with self._regs.scope():` so its scratch names
        (`__cursor`, `__byte_tmp`, `__ck_a`, …) don't leak into
        the next helper's pool.

        Nests cleanly: a helper that internally invokes another
        helper picks up a fresh inner scope, and the inner names
        are released before control returns to the outer scope.

        DO NOT REMOVE the `with self._regs.scope():` wrappers around
        helper bodies — they are load-bearing. Without them, a
        driver that chains read_n + .expect + compute_checksum +
        read_until exhausts VM_NUM_REGS=8 with names whose values
        were dead the moment each helper returned.
        """
        mark = len(self._map)
        try:
            yield self
        finally:
            # Drop everything allocated since the mark. Python
            # dicts preserve insertion order (3.7+), so the names
            # added inside the scope are the tail of the map.
            for k in list(self._map.keys())[mark:]:
                del self._map[k]
            # Recompute _next so freed slots become available
            # again. `default=-1` handles "scope was the whole
            # map" cleanly.
            self._next = max(self._map.values(), default=-1) + 1

    def reset(self):
        """Drop all scratch bindings at a section boundary (the
        configure() → measure() switch in compile()). Pinned names
        deliberately survive — they are the loop-carried registers
        whose configure()-time init must stay addressable from the
        measure body."""
        self._map.clear()
        self._next = 0


# ── Work-buffer allocator (64-bit slots) ────────────────────

class WideRef:
    """A 64-bit value living at a work-buffer offset.

    Returned by `configure()`-time coefficient reads and stored on the
    driver as `self.cN`; the measure-loop AST compiler resolves `self.cN`
    to its offset. Also tags each wide intermediate the width-tracking
    lowering produces.
    """

    def __init__(self, off: int):
        self._off = off

    @property
    def off(self) -> int:
        return self._off


class _WorkAlloc:
    """Allocates 8-byte slots in the VM's 64-bit work buffer.

    A free-list of byte offsets. Persistent values (coefficients bound to
    `self.<attr>` in `configure()`, named wide measure-locals) are
    allocated and held; anonymous intermediates are freed the moment their
    consuming op runs, so a compensation polynomial reuses a handful of
    slots rather than burning one per subexpression.
    """

    SLOT = 8

    def __init__(self, size: int):
        self._free = list(range(0, size - self.SLOT + 1, self.SLOT))

    def alloc(self) -> int:
        if not self._free:
            raise CompileError(
                "out of VM work-buffer slots — the compensation formula needs "
                "more 64-bit scratch than the work buffer holds")
        return self._free.pop(0)

    def free(self, off: int) -> None:
        if off not in self._free:
            self._free.append(off)
            self._free.sort()


# ── Bytecode emitter ────────────────────────────────────────

class _Emitter:
    """Accumulates instructions as (offset, bytes) pairs with label support."""

    # Opcodes that return VM_YIELD from the firmware VM, giving the
    # runner a chance to observe a STOP request between iterations.
    # A backward branch whose source-predecessor or target is one of
    # these already provides STOP-checkability for that loop.
    _YIELDABLE_OPCODES = frozenset({Op.YIELD, Op.SLEEP_MS, Op.SLEEP_US})

    def __init__(self):
        self._instructions: list = []
        self._labels: Dict[str, int] = {}
        self._fixups: list = []
        # Opcode of the most-recently-emitted instruction (or None at
        # program start). Used to decide whether a backward branch
        # already has a yieldable source-predecessor.
        self._last_op: Optional[int] = None
        # Labels placed but not yet bound to a target instruction.
        # Resolved on the next emit so backward-branch lookups can ask
        # "what opcode does this label point at?"
        self._pending_labels: list = []
        # label name → opcode of the first instruction at the label.
        self._label_op: Dict[str, int] = {}

    def label(self, name: str):
        self._labels[name] = self._current_offset()
        self._pending_labels.append(name)

    def emit(self, *args: int):
        self._on_emit(args[0])
        self._instructions.append((None, bytes(args)))

    def emit_u16(self, opcode: int, val: int):
        self._on_emit(opcode)
        self._instructions.append((None, struct.pack("<BH", opcode, val)))

    def emit_reg(self, opcode: int, reg: int, *tail: int):
        """Emit a register opcode: 8-bit opcode + 16-bit little-endian reg
        operand + trailing single-byte operands. The operand is a uniform
        16-bit field, but the bus devices frame 8-bit addresses only, so a
        register above 0xFF is rejected here — the compiler is the one layer
        that knows no wider-addressed part is supported, and catching it now
        beats a runtime -EINVAL that reads as a probe failure."""
        if not 0 <= reg <= 0xFF:
            raise CompileError(
                f"register 0x{reg:X} exceeds the 8-bit address limit; "
                f"register addresses wider than 8 bits are not yet supported")
        self._on_emit(opcode)
        self._instructions.append(
            (None, struct.pack("<BH", opcode, reg) + bytes(tail)))

    def emit_u32(self, opcode: int, reg: int, val: int):
        self._on_emit(opcode)
        self._instructions.append((None, struct.pack("<BBL", opcode, reg, val)))

    def emit_cmp(self, opcode: int, reg: int, imm: int, dst: int):
        self._on_emit(opcode)
        self._instructions.append(
            (None, struct.pack("<BBLB", opcode, reg, imm, dst)))

    def emit_jmp(self, opcode: int, label: str, reg: Optional[int] = None):
        """Emit a jump with a fixup for the label offset.

        For backward branches (label already defined), insert an
        implicit OP_YIELD beforehand if neither the source-predecessor
        nor the branch target is a yieldable opcode. This bounds STOP
        latency to one loop tick. Current shipped drivers always have
        YIELD or SLEEP_MS at their loop heads, so the insertion is a
        no-op today; the rule kicks in for future inner loops whose
        body lacks an explicit yield point.
        """
        if label in self._labels:
            target_op = self._label_op.get(label)
            if (self._last_op not in self._YIELDABLE_OPCODES and
                    target_op not in self._YIELDABLE_OPCODES):
                # Insert a cooperative yield so the runner can observe
                # a STOP request between iterations. SLEEP_US(0) is the
                # right primitive: returns VM_YIELD to the runner, does
                # NOT block on DRDY (which a stream-driver inner loop
                # has no semantic for; using OP_YIELD here would hit
                # the 1 s DRDY-timeout error path on every iteration).
                # IMU/DRDY drivers don't trigger this branch — their
                # measure-loop top is already OP_YIELD.
                self.emit_u16(Op.SLEEP_US, 0)

        self._on_emit(opcode)
        offset = self._current_offset()
        if reg is not None:
            placeholder = struct.pack("<BBh", opcode, reg, 0)
            fixup_pos = offset + 2
        else:
            placeholder = struct.pack("<Bh", opcode, 0)
            fixup_pos = offset + 1
        self._fixups.append((fixup_pos, label, offset, len(placeholder)))
        self._instructions.append((None, placeholder))

    def _on_emit(self, opcode: int):
        for name in self._pending_labels:
            self._label_op[name] = opcode
        self._pending_labels.clear()
        self._last_op = opcode

    def build(self) -> bytes:
        raw = b"".join(data for _, data in self._instructions)
        buf = bytearray(raw)
        for fixup_pos, label, instr_start, _instr_len in self._fixups:
            if label not in self._labels:
                raise CompileError(f"Undefined label: '{label}'")
            target = self._labels[label]
            offset = target - instr_start
            struct.pack_into("<h", buf, fixup_pos, offset)
        return bytes(buf)

    def _current_offset(self) -> int:
        return sum(len(data) for _, data in self._instructions)


class CompileError(Exception):
    pass


class _TraceReadValue:
    """Sentinel returned by a width-1 read during configure() tracing.

    configure() runs at compile time against mocked reads, so a value read
    there is not the live register — a runtime conditional or read-modify-write
    can't be expressed in configure(). Any use of this value raises; the fix is
    to move the logic to measure(), where reads execute on-device."""
    __slots__ = ("_reg",)

    def __init__(self, reg):
        self._reg = reg

    def _fail(self, *_args):
        raise CompileError(
            f"a register read in configure() returns a compile-time mock, not "
            f"the live value (register 0x{self._reg:02X}); a runtime "
            f"conditional or read-modify-write belongs in measure().")

    __index__ = __bool__ = _fail
    __eq__ = __ne__ = __lt__ = __le__ = __gt__ = __ge__ = _fail
    __add__ = __radd__ = __sub__ = __rsub__ = __mul__ = __rmul__ = _fail
    __and__ = __rand__ = __or__ = __ror__ = __xor__ = __rxor__ = _fail
    __lshift__ = __rshift__ = __invert__ = _fail
    __hash__ = None
    __str__ = __repr__ = __format__ = _fail


# Bus kind tags — the AST compiler uses these to pick the right opcode
# family when the user writes `self.read(...)` in a measure loop.
BUS_REGISTER = "register"
BUS_STREAM = "stream"

# Physical bus name → integer code stored in the NXS `bus` param and
# used by firmware to bind hal::RegDeviceI. Single source of truth for
# the bus-name-to-code mapping; referenced by the auto-inject in
# compile() and by the config-canonicalisation at the top of compile().
_BUS_NAME_TO_CODE = {'i2c': 0, 'spi': 1}
_BUS_CODE_TO_NAME = {v: k for k, v in _BUS_NAME_TO_CODE.items()}

# Error code emitted by the auto-generated WHO_AM_I prologue when the
# read value matches none of the declared WHO_AM_I_VALUES. Lands in the
# VM's error_code field and is surfaced by `nxs status`.
WHO_AM_I_MISMATCH_CODE = 0xC0

# Same check, failed on a companion die (I2C_COMPANIONS): distinct code
# so the bench can tell "wrong primary" from "companion missing".
COMPANION_MISMATCH_CODE = 0xC1

# OP_ERROR operand values that the firmware maps to specific VmErr
# return codes (see SensorDriverVM.cpp OP_ERROR arm). These let
# compiler-emitted timeout/mismatch errors surface as VM_TIMEOUT /
# VM_MISMATCH instead of the generic VM_USER_ERROR.
ERR_CODE_TIMEOUT  = 17
ERR_CODE_MISMATCH = 18


@dataclass
class TracedSlice:
    """Trace-side handle on a slice of `sample_buf` produced by
    `read_n` / `read_until`. Carries the buffer offset and length
    so `.expect(expected_bytes)` can emit byte-by-byte LOAD_U8 +
    CMP_EQ comparisons at the right offsets.

    Not constructed directly by drivers — the framing helpers
    return one. The slice is purely a trace-time object; at
    runtime the bytes live in the VM's sample_buf.
    """
    driver: Any
    buf_off: int
    length: int

    def expect(self, expected: bytes) -> None:
        """Emit bytecode that compares `expected[i]` against
        `sample_buf[self.buf_off + i]` for each i, jumping to
        `OP_ERROR ERR_CODE_MISMATCH` on the first mismatch. The
        VM transitions to ERROR state and step() returns
        `VmErr::MISMATCH` (-418); the runner converts that to
        PROBE_FAILED + slot advance.

        Raises CompileError if `expected` is longer than the slice
        or empty.
        """
        if isinstance(expected, (bytes, bytearray)):
            expected_bytes = bytes(expected)
        else:
            raise CompileError(
                f"TracedSlice.expect: expected must be bytes, got "
                f"{type(expected).__name__}")
        if len(expected_bytes) == 0:
            raise CompileError("TracedSlice.expect: expected cannot be empty")
        if len(expected_bytes) > self.length:
            raise CompileError(
                f"TracedSlice.expect: expected ({len(expected_bytes)} B) "
                f"longer than slice ({self.length} B)")

        em = self.driver._emitter
        L_FAIL = self.driver._fresh_label("ex_fail")
        L_DONE = self.driver._fresh_label("ex_done")

        # Scratch scope: __byte_tmp / __match_tmp are dead the moment
        # the LOAD_U8/CMP_EQ chain finishes — releasing them keeps
        # the next helper's pool unsaturated. See `_RegAlloc.scope`.
        with self.driver._regs.scope():
            byte_r = self.driver._regs.get("__byte_tmp")
            match_r = self.driver._regs.get("__match_tmp")
            for i, want in enumerate(expected_bytes):
                em.emit(Op.LOAD_U8, byte_r, self.buf_off + i)
                em.emit_cmp(Op.CMP_EQ, byte_r, want, match_r)
                em.emit_jmp(Op.JZ, L_FAIL, match_r)
            em.emit_jmp(Op.JMP, L_DONE)
            em.label(L_FAIL)
            em.emit(Op.ERROR, ERR_CODE_MISMATCH)
            em.label(L_DONE)


# ── AST compiler for measure_loop ───────────────────────────

def _load_spec_byte(width: int, signed: bool, endian: str) -> int:
    """Compose an `OP_LOAD` spec byte: low 3 bits = width (1-4), bit 3 =
    little-endian, bit 4 = sign-extend. Mirrors `SensorDriverVM.cpp` OP_LOAD.
    Defaults (unsigned, big-endian) leave the byte equal to `width`."""
    if endian not in ("big", "little"):
        raise CompileError(
            f"read(endian={endian!r}): must be 'big' or 'little'")
    return width | (0x08 if endian == "little" else 0) | (0x10 if signed else 0)


class _ASTCompiler(ast.NodeVisitor):
    """Compiles a measure() function body into bytecode via AST walking."""

    # Mirror SensorDriverVM.h; pinned to it by
    # test_vm_layout_constants_match_firmware_header.
    # Burst codegen places its rotating TX/RX slot at the tail of the sample
    # buffer, past the output region.
    VM_SAMPLE_BUF_SIZE = 128
    # Scalar value-reads (`read(reg, width)`, `read_analog`) stage their bytes
    # at the sample-buffer tail before loading them into a register, so they
    # can never alias the sample data placed at [0, sample_size). The widest
    # scalar read is 4 bytes (the LOAD width cap), so the slot is the last 4
    # bytes. Data-placing verbs (`read_burst`/`read_words`) still land at
    # offset 0 — those bytes ARE the committed sample.
    SCALAR_SCRATCH_OFF = VM_SAMPLE_BUF_SIZE - 4   # 124
    # 64-bit compute scratch the int64 ops address (32 × 8-byte slots; 256 B
    # is the u8-offset ceiling).
    VM_WORK_BUF_SIZE = 256

    def __init__(self, emitter: _Emitter, regs: _RegAlloc,
                 trigger: str, sample_rate: int, bus_kind: str,
                 frame=None, driver=None, drdy_base_hz: int = 0):
        self._em = emitter
        self._regs = regs
        self._trigger = trigger
        self._sample_rate = sample_rate
        self._drdy_base_hz = drdy_base_hz
        self._bus_kind = bus_kind
        self._frame = frame
        # The live driver instance — used as a fallback for methods
        # not in the AST dispatch table. Lets the driver author add a
        # new helper (read_until, store_sample_n, …) on the base class
        # without touching the AST compiler; calling it from inside
        # measure() works because the helper emits to the shared
        # emitter directly.
        self._driver = driver
        self._loop_label = "__measure_loop"
        # measure-local name → work-buffer offset for 64-bit (wide) locals.
        # Narrow locals stay in _regs; this tracks the ones promoted to the
        # work buffer by a multiply or a wide operand.
        self._wide_locals: Dict[str, int] = {}

    def _divider_for(self, rate: int) -> int:
        """Sync divider for `rate` on a fixed-sync part; exact divisors only —
        a non-divisor would deliver a rate other than the one declared."""
        base = self._drdy_base_hz
        if rate <= 0 or base % rate != 0:
            raise CompileError(
                f"sample_rate {rate} does not divide the {base} Hz hardware "
                f"sync; a fixed-sync part's rates must be exact divisors "
                f"(base/N) so the delivered spacing matches the declared "
                f"value.")
        return base // rate

    def compile_function(self, func):
        source = inspect.getsource(func)
        source = textwrap.dedent(source)
        tree = ast.parse(source)
        func_def = tree.body[0]
        if not isinstance(func_def, ast.FunctionDef):
            raise CompileError("Expected a function definition")

        func_def.decorator_list = []

        # Emit loop top. DRDY mode emits only OP_YIELD — the hardware
        # interrupt already gates timing at the sensor's ODR, so an
        # extra SLEEP_MS would only subtract from the achievable rate.
        # Downsampling below the hardware rate goes through the sensor's
        # ODR register (a tagged write) or, on a fixed-sync part, through
        # OP_EVENT_DIV (drdy_base_hz) — never by sleeping here. Poll mode
        # has no hardware timing source and uses SLEEP_MS as its sole
        # throttle.
        # Per-body ledger of sample-buffer regions claimed by data-placing
        # reads; overlapping claims are rejected in _claim_data_span.
        self._data_spans = []

        if self._trigger == "drdy" and self._drdy_base_hz > 0:
            # Fixed-sync part: pace by dividing the hardware sync at the
            # source. Emitted once, before the loop; the u16 divider operand
            # is sample_rate's patch site.
            div = self._divider_for(self._sample_rate)
            div_off = self._em._current_offset() + 1
            self._em.emit_u16(Op.EVENT_DIV, div)
            if self._driver is not None:
                self._driver._patch_drdy_div(div_off, self._drdy_base_hz)

        self._em.label(self._loop_label)
        if self._trigger == "drdy":
            self._em.emit(Op.YIELD)
        elif self._trigger == "poll":
            interval = max(1, 1000 // self._sample_rate)
            sleep_off = self._em._current_offset() + 1
            self._em.emit_u16(Op.SLEEP_MS, interval)
            if self._driver is not None:
                self._driver._patch_poll_rate(sleep_off)

        for stmt in func_def.body:
            self._compile_stmt(stmt)

        self._em.emit_jmp(Op.JMP, self._loop_label)

    def _compile_stmt(self, node):
        if isinstance(node, ast.Assign):
            self._compile_assign(node)
        elif isinstance(node, ast.Expr):
            self._compile_expr_stmt(node)
        elif isinstance(node, ast.If):
            self._compile_if(node)
        elif isinstance(node, ast.Return):
            self._compile_return(node)
        else:
            raise CompileError(
                f"Unsupported statement: {type(node).__name__} "
                f"(line {node.lineno})")

    def _compile_assign(self, node):
        if len(node.targets) != 1:
            raise CompileError("Multiple assignment targets not supported")
        target = node.targets[0]
        if not isinstance(target, ast.Name):
            raise CompileError("Only simple variable assignment supported")
        var_name = target.id
        self._compile_value_into_reg(node.value, var_name)

    def _read_kwargs(self, keywords):
        """Extract (signed, endian, dev) from a `read()` call's keywords,
        rejecting unknown keys. Defaults keep the read unsigned/big-endian on
        the primary — byte-for-byte identical to a plain `read(reg, width)`."""
        signed, endian, dev = False, "big", None
        for kw in keywords:
            if kw.arg == "signed":
                signed = bool(self._eval_const(kw.value))
            elif kw.arg == "endian":
                endian = self._eval_const(kw.value)
            elif kw.arg == "dev":
                dev = self._eval_const(kw.value)
            else:
                raise CompileError(
                    f"read(): unknown keyword {kw.arg!r}; supported: "
                    f"signed=, endian=, dev=")
        return signed, endian, dev

    def _into_kwarg(self, keywords, lineno):
        """Extract (into, dev) from a data-placing read's keywords — the
        destination offset (default 0) and an optional companion target —
        rejecting unknown keys."""
        into, dev = 0, None
        for kw in keywords:
            if kw.arg == "into":
                into = self._eval_const(kw.value)
            elif kw.arg == "dev":
                dev = self._eval_const(kw.value)
            else:
                raise CompileError(
                    f"unknown keyword {kw.arg!r} on a data read; supported: "
                    f"into=, dev= (line {lineno})")
        return into, dev

    def _dev_open(self, dev, lineno):
        """Open a companion bracket: resolve `dev` and emit the retarget.
        Returns True when a matching `_dev_close(True)` must follow."""
        if dev is None:
            return False
        if self._bus_kind != BUS_REGISTER or self._driver is None:
            raise CompileError(
                f"dev= targets an I2C companion; this driver kind has none "
                f"(line {lineno}).")
        addr = self._driver._companion_addr(dev, lineno)
        self._em.emit(Op.I2C_TARGET, addr)
        return True

    def _dev_close(self, opened):
        if opened:
            self._em.emit(Op.I2C_TARGET, 0)

    def _scalar_scratch_off(self, verb: str) -> int:
        """Byte offset a scalar value-read stages at before it LOADs.

        Parks at the sample-buffer tail so the read can't overwrite the
        sample data at [0, sample_size). A driver whose sample reaches into
        that slot is rejected here rather than shipping a field the read
        would corrupt.
        """
        size = self._driver._sample_size
        if size > self.SCALAR_SCRATCH_OFF:
            raise CompileError(
                f"{verb} stages through sample_buf["
                f"{self.SCALAR_SCRATCH_OFF}..{self.VM_SAMPLE_BUF_SIZE}), but "
                f"sample_size={size} extends into that scratch slot; the read "
                f"would corrupt the sample. Keep sample_size <= "
                f"{self.SCALAR_SCRATCH_OFF}.")
        return self.SCALAR_SCRATCH_OFF

    def _claim_data_span(self, off: int, length: int, verb: str, lineno: int):
        """Reserve sample_buf[off .. off+length) for a data-placing read and
        reject a silent overwrite or an out-of-range span. Two distinct verbs
        writing overlapping regions clobber each other. A data read may fill the
        whole buffer; the scalar-scratch tail isn't reserved here — a scalar
        read guards its own slot in _scalar_scratch_off (which rejects when
        sample_size reaches the tail), so a pure-burst sample can use the
        full buffer."""
        end = off + length
        if off < 0 or end > self.VM_SAMPLE_BUF_SIZE:
            raise CompileError(
                f"{verb} writes sample_buf[{off}..{end}), outside the "
                f"{self.VM_SAMPLE_BUF_SIZE}-byte sample buffer. Adjust the "
                f"count or into= (line {lineno}).")
        for (o, e, v, ln) in self._data_spans:
            if off < e and o < end:
                raise CompileError(
                    f"{verb} at sample_buf[{off}..{end}) (line {lineno}) "
                    f"overlaps {v} at [{o}..{e}) (line {ln}); give one an "
                    f"into= offset so they don't clobber.")
        self._data_spans.append((off, end, verb, lineno))

    def _compile_value_into_reg(self, value, var_name: str):
        # 64-bit (wide) assignment: a multiply, or any expression touching a
        # wide operand (a coefficient `self.cN` or a wide local), lands in the
        # work buffer rather than a register. Evaluate first (so the RHS sees
        # the name's current binding), then rebind.
        if self._is_wide(value):
            off, owned = self._eval_wide(value)
            existing = self._wide_locals.get(var_name)
            if existing is not None:
                # In-place reassignment. Writing into the local's *existing*
                # slot is control-flow-safe: a conditional reassign (e.g. the
                # second-order `off = off - off2` inside `if temp < 2000`)
                # leaves that one address holding original-or-updated, so a
                # later read sees the right value whether or not the branch ran.
                if off != existing:
                    self._em.emit(Op.SHL64, off, 0, existing)  # copy result in
                    if owned:
                        self._driver._work.free(off)
                return
            # First binding. A bare `y = self.cN` / `y = <wide local>` (not
            # owned) is copied into a fresh slot so the local owns its storage.
            if not owned:
                named = self._driver._work.alloc()
                self._em.emit(Op.SHL64, off, 0, named)
                off = named
            self._wide_locals[var_name] = off
            return

        # Narrow assignment. If `var_name` was previously wide, release its slot.
        old = self._wide_locals.pop(var_name, None)
        if old is not None:
            self._driver._work.free(old)
        dst = self._regs.get(var_name)

        # Constant assignment (`flag = 0`) -> LOAD_IMM.
        if isinstance(value, ast.Constant) and isinstance(value.value, int):
            self._em.emit_u32(Op.LOAD_IMM, dst, value.value & 0xFFFFFFFF)
            return

        # Variable copy (`prev = raw`) -> MOV.
        if isinstance(value, ast.Name):
            self._em.emit(Op.MOV, dst, self._regs.get(value.id))
            return

        # Arithmetic assignment: y = x <op> const, or y = x <op> var.
        if isinstance(value, ast.BinOp):
            self._compile_arith_binop(value, dst)
            return

        if not (isinstance(value, ast.Call) and
                self._is_self_method(value.func)):
            raise CompileError(
                f"Unsupported assignment value: {ast.dump(value)} "
                f"(line {value.lineno})")

        method = value.func.attr

        if self._bus_kind == BUS_REGISTER:
            if method == "read":
                reg = self._eval_const(value.args[0])
                if self._frame is not None:
                    # FRAME driver: clock the composed wire frame, not a plain
                    # REG_READ (which would clock unframed garbage on the bus).
                    dw = self._frame.data_byte_width()
                    if len(value.args) > 1 and self._eval_const(value.args[1]) != dw:
                        raise CompileError(
                            f"a FRAME read returns the {dw}-byte data field; "
                            f"drop the width or pass {dw} (line {value.lineno}).")
                    signed, endian, dev = self._read_kwargs(value.keywords)
                    if dev is not None:
                        raise CompileError(
                            f"dev= targets an I2C companion; a FRAME driver "
                            f"is SPI and declares none (line {value.lineno}).")
                    self._driver._emit_frame_read(reg, dst, signed, endian)
                    return
                if len(value.args) > 1:
                    # read(reg, width[, signed=, endian=, dev=]): burst `width`
                    # bytes then LOAD into the register, honouring sign and
                    # byte order (defaults: unsigned big-endian).
                    width = self._eval_const(value.args[1])
                    if not 1 <= width <= 4:
                        raise CompileError(
                            f"read(reg, width) supports width 1..4 (OP_LOAD), "
                            f"got {width} (line {value.lineno}).")
                    signed, endian, dev = self._read_kwargs(value.keywords)
                    off = self._scalar_scratch_off("read(reg, width)")
                    opened = self._dev_open(dev, value.lineno)
                    self._em.emit_reg(Op.REG_READ_BURST, reg, width, off)
                    self._dev_close(opened)
                    self._em.emit(Op.LOAD, dst, off,
                                  _load_spec_byte(width, signed, endian))
                    return
                signed, endian, dev = self._read_kwargs(value.keywords)
                if signed or endian != "big":
                    raise CompileError(
                        f"read(reg, signed=/endian=) needs an explicit width — "
                        f"e.g. read(reg, 2, signed=True) (line {value.lineno}).")
                opened = self._dev_open(dev, value.lineno)
                self._em.emit_reg(Op.REG_READ, reg, dst)
                self._dev_close(opened)
                return
            if method == "read_burst":
                reg = self._eval_const(value.args[0])
                count = self._eval_const(value.args[1])
                into, dev = self._into_kwarg(value.keywords, value.lineno)
                self._claim_data_span(into, count, "read_burst", value.lineno)
                opened = self._dev_open(dev, value.lineno)
                self._em.emit_reg(Op.REG_READ_BURST, reg, count, into)
                self._dev_close(opened)
                return
            if method == "read_words":
                if self._frame is None:
                    raise CompileError(
                        f"self.read_words(...) requires a FRAME schema on "
                        f"the driver class (line {value.lineno}). Plain-"
                        f"register drivers should use self.read_burst(...).")
                start_reg = self._eval_const(value.args[0])
                num_words = self._eval_const(value.args[1])
                # read_words keeps its own output-vs-frame-slot bound check
                # inside _emit_frame_burst (the FRAME slot floats with byte
                # width, unlike the fixed scalar tail), so it is not tracked in
                # the read_burst span ledger.
                self._emit_frame_burst(start_reg, num_words)
                return
            if method == "xfer":
                # `x = self.xfer(word, width=2)`: clock a literal full-duplex
                # SPI word (staged at the scalar-scratch tail by the driver
                # verb) and load the response — unsigned, MSB-first, the whole
                # `width` bytes. Mask/shift the value before publishing.
                word = self._eval_const(value.args[0])
                width = 2
                if len(value.args) > 1:
                    width = self._eval_const(value.args[1])
                for kw in value.keywords:
                    if kw.arg != "width":
                        raise CompileError(
                            f"unknown keyword {kw.arg!r} on xfer; the only "
                            f"supported keyword is width= (line {value.lineno})")
                    width = self._eval_const(kw.value)
                off = self._driver._emit_xfer(word, width)
                if width == 1:
                    self._em.emit(Op.LOAD_U8, dst, off)
                elif width == 2:
                    self._em.emit(Op.LOAD_U16_BE, dst, off)
                else:
                    self._em.emit(Op.LOAD, dst, off,
                                  _load_spec_byte(width, False, "big"))
                return

        if self._bus_kind == BUS_STREAM:
            if method == "available":
                self._em.emit(Op.UART_AVAIL, dst)
                return
            if method == "read":
                count = self._eval_const(value.args[0])
                into, dev = self._into_kwarg(value.keywords, value.lineno)
                if dev is not None:
                    raise CompileError(
                        f"dev= targets an I2C companion; a stream driver "
                        f"has none (line {value.lineno}).")
                self._claim_data_span(into, count, "read", value.lineno)
                self._em.emit(Op.UART_READ, count, into)
                return

        # Analog input is orthogonal to the data bus — available on any
        # driver kind. read_analog samples the AN pad and hands back the raw
        # 16-bit count as a value in `dst`; it stages at the scalar-scratch
        # tail (not offset 0), so publish it through a named field —
        # `return Sample(voltage=raw)`, not a positional `Sample(raw)`.
        if method == "read_analog":
            ch = self._eval_const(value.args[0])
            off = self._scalar_scratch_off("read_analog")
            self._em.emit(Op.ADC_READ, ch, off)
            self._em.emit(Op.LOAD_U16_BE, dst, off)   # expose the count for arithmetic / a named field
            return

        # `m = self.match(b0, b1, ...)`: mismatch count of sample_buf[0:N] vs the
        # expected bytes (0 = exact match = pass). Bus-agnostic.
        if method == "match":
            wants = [self._eval_const(a) & 0xFF for a in value.args]
            self._em.emit_u32(Op.LOAD_IMM, dst, 0)
            with self._regs.scope():
                b = self._regs.get("__match_byte")
                eq = self._regs.get("__match_eq")
                for i, want in enumerate(wants):
                    self._em.emit(Op.LOAD_U8, b, i)
                    self._em.emit_cmp(Op.CMP_EQ, b, want, eq)
                    self._em.emit_cmp(Op.XOR, eq, 1, eq)
                    self._em.emit(Op.ADD_REG, dst, eq, dst)
            return

        # `bad = self.verify_checksum(ChecksumFletcher(), start_off, length,
        # ck_off)`: recompute the checksum over sample_buf[start_off ..
        # start_off+length) and count mismatches against the received bytes
        # at ck_off (0 = frame intact). The RX counterpart of
        # compute_checksum — that one writes, this one compares. Fletcher
        # only: it is the one algorithm with an RX consumer.
        if method == "verify_checksum":
            self._compile_verify_checksum(value, dst)
            return

        raise CompileError(
            f"Unsupported assignment value: self.{method}(...) "
            f"on {self._bus_kind} driver (line {value.lineno})")

    def _compile_verify_checksum(self, value, dst: int):
        """Emit the Fletcher-verify loop: two-byte Fletcher-8 over the
        buffered span, XOR-compared against the two received checksum
        bytes; `dst` accumulates the mismatch count. Uses 4 scoped
        scratch registers and leaves `__cursor` untouched, so
        `store_sample_n()` after a passing verify commits the frame."""
        args = list(value.args)
        if not (args and isinstance(args[0], ast.Call)
                and isinstance(args[0].func, ast.Name)
                and args[0].func.id == "ChecksumFletcher"):
            raise CompileError(
                f"verify_checksum: the spec must be ChecksumFletcher() — "
                f"the one checksum with an RX consumer (line "
                f"{value.lineno}).")
        params = {}
        for name, node in zip(("start_off", "length", "ck_off"), args[1:]):
            params[name] = self._eval_const(node)
        for kw in value.keywords:
            params[kw.arg] = self._eval_const(kw.value)
        missing = {"start_off", "length", "ck_off"} - set(params)
        if missing:
            raise CompileError(
                f"verify_checksum: missing {sorted(missing)} (line "
                f"{value.lineno}).")
        start_off = int(params["start_off"])
        length = int(params["length"])
        ck_off = int(params["ck_off"])
        buf = self.VM_SAMPLE_BUF_SIZE
        if not (0 <= start_off and 0 < length
                and start_off + length <= buf and 0 <= ck_off
                and ck_off + 2 <= buf):
            raise CompileError(
                f"verify_checksum: span [{start_off}, {start_off + length}) "
                f"or checksum bytes [{ck_off}, {ck_off + 2}) fall outside "
                f"the {buf}-byte sample buffer (line {value.lineno}).")

        L_TOP = f"__vck_top_{id(value)}"
        self._em.emit_u32(Op.LOAD_IMM, dst, 0)
        with self._regs.scope():
            ck_a = self._regs.get("__vck_a")
            ck_b = self._regs.get("__vck_b")
            cursor = self._regs.get("__vck_cursor")
            byte_r = self._regs.get("__vck_byte")

            self._em.emit_u32(Op.LOAD_IMM, ck_a, 0)
            self._em.emit_u32(Op.LOAD_IMM, ck_b, 0)
            self._em.emit_u32(Op.LOAD_IMM, cursor, start_off)
            self._em.label(L_TOP)
            self._em.emit(Op.LOAD_U8_REG, byte_r, cursor)
            self._em.emit(Op.ADD_REG, ck_a, byte_r, ck_a)
            self._em.emit_cmp(Op.AND, ck_a, 0xFF, ck_a)
            self._em.emit(Op.ADD_REG, ck_b, ck_a, ck_b)
            self._em.emit_cmp(Op.AND, ck_b, 0xFF, ck_b)
            self._em.emit_cmp(Op.ADD, cursor, 1, cursor)
            # byte_r doubles as the loop-exit flag: its byte value is
            # consumed by the ADDs above, and reusing it keeps the verb
            # at 4 scratch registers (see VM_NUM_REGS).
            self._em.emit_cmp(Op.CMP_EQ, cursor, start_off + length, byte_r)
            self._em.emit_jmp(Op.JZ, L_TOP, byte_r)
            # Compare each computed byte against the received one:
            # XOR == 0 means equal; fold the inequality into dst.
            for reg, off in ((ck_a, ck_off), (ck_b, ck_off + 1)):
                self._em.emit(Op.LOAD_U8, byte_r, off)
                self._em.emit(Op.XOR_REG, byte_r, reg, byte_r)
                self._em.emit_cmp(Op.CMP_EQ, byte_r, 0, byte_r)
                self._em.emit_cmp(Op.XOR, byte_r, 1, byte_r)
                self._em.emit(Op.ADD_REG, dst, byte_r, dst)

    def _compile_arith_binop(self, value, dst: int):
        """Compile `y = x <op> rhs`: rhs is a measure()-local variable
        (register-register, only + - ^) or a compile-time constant (immediate)."""
        left_name = self._get_name(value.left)
        if not left_name:
            raise CompileError(
                f"left operand of a 32-bit arithmetic expression must be a "
                f"simple variable, not {type(value.left).__name__} "
                f"(line {value.lineno}).")
        src_reg = self._regs.get(left_name)
        op = value.op
        if isinstance(value.right, ast.Name):
            src_b = self._regs.get(value.right.id)
            regreg = {ast.Add: Op.ADD_REG, ast.Sub: Op.SUB_REG, ast.BitXor: Op.XOR_REG}
            opc = regreg.get(type(op))
            if opc is None:
                raise CompileError(
                    f"Register-register {type(op).__name__} is unsupported "
                    f"(only +, -, ^); use a constant operand (line {value.lineno}).")
            self._em.emit(opc, src_reg, src_b, dst)
            return
        rhs = self._eval_const(value.right)
        imm_ops = {ast.Add: Op.ADD, ast.Sub: Op.SUB, ast.BitXor: Op.XOR,
                   ast.BitAnd: Op.AND, ast.BitOr: Op.OR}
        if type(op) in imm_ops:
            self._em.emit_cmp(imm_ops[type(op)], src_reg, rhs & 0xFFFFFFFF, dst)
            return
        if isinstance(op, (ast.LShift, ast.RShift)):
            if not 0 <= rhs <= 31:
                raise CompileError(
                    f"32-bit shift count must be 0..31, got {rhs} "
                    f"(line {value.lineno}).")
            opc = Op.SHL if isinstance(op, ast.LShift) else Op.SHR
            self._em.emit(opc, src_reg, rhs, dst)
            return
        raise CompileError(
            f"Unsupported arithmetic operator {type(op).__name__} "
            f"(line {value.lineno}).")

    # ── 64-bit width-tracking evaluator ─────────────────────
    # A multiply always yields a 64-bit value (a 32x32 product can overflow
    # 32 bits); that width propagates through +/-/// . Wide values live in
    # work-buffer slots; narrow ones in registers. No author annotation — the
    # structure decides, so `ms5611.py` is plain Python integer arithmetic.

    def _const_pow2(self, node):
        """Exponent k if `node` is a compile-time 2**k (k>=0), else None."""
        try:
            v = self._eval_const(node)
        except CompileError:
            return None
        if isinstance(v, int) and v > 0 and (v & (v - 1)) == 0:
            return v.bit_length() - 1
        return None

    def _is_wide(self, node) -> bool:
        """True if `node` evaluates to a 64-bit value."""
        if isinstance(node, ast.BinOp):
            if isinstance(node.op, ast.Mult):
                return True
            if isinstance(node.op, (ast.Add, ast.Sub, ast.FloorDiv,
                                    ast.LShift, ast.RShift)):
                return self._is_wide(node.left) or self._is_wide(node.right)
            return False
        if isinstance(node, ast.Name):
            return node.id in self._wide_locals
        if self._is_self_method(node):  # bare `self.<attr>` → a coefficient?
            return isinstance(getattr(self._driver, node.attr, None), WideRef)
        return False

    def _eval_narrow_reg(self, node) -> int:
        """Compile a narrow (32-bit) leaf into a register; return its index."""
        if isinstance(node, ast.Name):
            if node.id in self._wide_locals:
                raise CompileError(
                    f"64-bit value '{node.id}' used in a 32-bit context "
                    f"(line {getattr(node, 'lineno', '?')}).")
            return self._regs.get(node.id)
        if isinstance(node, (ast.Constant, ast.UnaryOp, ast.BinOp)):
            r = self._regs.get("__wide_scratch")
            self._em.emit_u32(Op.LOAD_IMM, r, self._eval_const(node) & 0xFFFFFFFF)
            return r
        raise CompileError(
            f"Unsupported operand in a 64-bit expression: {ast.dump(node)} "
            f"(line {getattr(node, 'lineno', '?')}).")

    def _to_wide(self, node):
        """`node` → (work_off, owned), widening a narrow node with `CVT64`; use
        when the node may be either width (`_eval_wide` assumes already-wide).
        owned=True is a fresh temp the caller may free; owned=False is a
        persistent/named slot."""
        if self._is_wide(node):
            return self._eval_wide(node)
        reg = self._eval_narrow_reg(node)
        off = self._driver._work.alloc()
        self._em.emit(Op.CVT64, reg, off)
        return off, True

    def _wide_result_unary(self, a_off, a_owned):
        """Destination for a unary wide op: reuse an owned operand (handlers
        read before write), else a fresh slot."""
        return a_off if a_owned else self._driver._work.alloc()

    def _wide_result_binary(self, a_off, a_owned, b_off, b_owned):
        """Destination for a binary wide op: reuse an owned operand, freeing
        the other owned temp; else a fresh slot."""
        if a_owned:
            if b_owned:
                self._driver._work.free(b_off)
            return a_off
        if b_owned:
            return b_off
        return self._driver._work.alloc()

    def _eval_wide(self, node):
        """Compile an already-wide `node` → (work_off, owned); caller ensures
        `_is_wide(node)`. Use `_to_wide` to widen a possibly-narrow node."""
        if isinstance(node, ast.Name):
            return self._wide_locals[node.id], False
        if self._is_self_method(node):
            return getattr(self._driver, node.attr).off, False
        if isinstance(node, ast.BinOp):
            op = node.op
            if isinstance(op, ast.Mult):
                # `value * 2**k` (either side) lowers to SHL64; else MUL64.
                for lhs, rhs in ((node.left, node.right), (node.right, node.left)):
                    k = self._const_pow2(rhs)
                    if k is not None:
                        a_off, a_owned = self._to_wide(lhs)
                        dst = self._wide_result_unary(a_off, a_owned)
                        self._em.emit(Op.SHL64, a_off, k, dst)
                        return dst, True
                a_off, a_owned = self._to_wide(node.left)
                b_off, b_owned = self._to_wide(node.right)
                dst = self._wide_result_binary(a_off, a_owned, b_off, b_owned)
                self._em.emit(Op.MUL64, a_off, b_off, dst)
                return dst, True
            if isinstance(op, ast.FloorDiv):
                k = self._const_pow2(node.right)
                if k is None:
                    raise CompileError(
                        f"64-bit // must divide by a power of two "
                        f"(line {node.lineno}).")
                a_off, a_owned = self._to_wide(node.left)
                dst = self._wide_result_unary(a_off, a_owned)
                self._em.emit(Op.SHR64, a_off, k, dst)
                return dst, True
            if isinstance(op, (ast.LShift, ast.RShift)):
                n = self._eval_const(node.right)
                if not 0 <= n <= 63:
                    raise CompileError(
                        f"64-bit shift count must be 0..63, got {n} "
                        f"(line {getattr(node, 'lineno', '?')}).")
                a_off, a_owned = self._to_wide(node.left)
                dst = self._wide_result_unary(a_off, a_owned)
                opc = Op.SHL64 if isinstance(op, ast.LShift) else Op.SHR64
                self._em.emit(opc, a_off, n, dst)
                return dst, True
            if isinstance(op, (ast.Add, ast.Sub)):
                a_off, a_owned = self._to_wide(node.left)
                b_off, b_owned = self._to_wide(node.right)
                dst = self._wide_result_binary(a_off, a_owned, b_off, b_owned)
                opc = Op.ADD64 if isinstance(op, ast.Add) else Op.SUB64
                self._em.emit(opc, a_off, b_off, dst)
                return dst, True
        raise CompileError(
            f"Unsupported 64-bit expression: {ast.dump(node)} "
            f"(line {getattr(node, 'lineno', '?')}).")

    def _value_to_reg(self, node) -> int:
        """Compile any value into a register: a wide value is TRUNC64'd to its
        low 32 bits (its temp slot freed); a narrow value goes via the register
        path. Used where a register is required — output stores and compares."""
        if self._is_wide(node):
            off, owned = self._eval_wide(node)
            r = self._regs.get("__narrow_scratch")
            self._em.emit(Op.TRUNC64, off, r)
            if owned:
                self._driver._work.free(off)
            return r
        return self._eval_narrow_reg(node)

    def _emit_frame_burst(self, start_reg: int, num_words: int):
        """Emit a FRAME-aware pipelined read of N consecutive registers.

        Layout inside the VM's sample buffer (``VM_SAMPLE_BUF_SIZE`` B):

            [0 .. num_words * dw)              → packed output data
            [SLOT .. SLOT + frame.byte_width)  → rotating TX/RX slot

        where ``SLOT = VM_SAMPLE_BUF_SIZE - frame.byte_width``. Each iteration stages
        one TX frame into the slot, issues an XFER, then — for
        iterations ``k >= pipeline`` — MEMCPYs the data bytes of the
        just-arrived RX (which corresponds to request ``k - pipeline``)
        into the output region *before* the next iteration's MEMCPY_IMM
        overwrites the slot. That ordering constraint is what lets a
        single slot serve the whole burst, saving buffer footprint
        with no bytecode overhead.
        """
        frame = self._frame
        fw = frame.byte_width
        dw = frame.data_byte_width()
        doff = frame.data_byte_offset()
        pipeline = max(0, int(frame.read_pipeline))

        if dw != 2:
            raise CompileError(
                f"read_words: FRAME data_byte_width={dw} B not "
                f"supported (only 16-bit data fields).")
        if num_words <= 0:
            raise CompileError(
                f"read_words: num_words must be positive, got {num_words}")

        slot = self.VM_SAMPLE_BUF_SIZE - fw
        out_bytes = num_words * dw
        if out_bytes > slot:
            raise CompileError(
                f"read_words: {num_words} words × {dw} B = {out_bytes} B "
                f"output would overflow the sample buffer (TX/RX slot "
                f"starts at offset {slot}). Split into smaller bursts.")

        num_tx = num_words + pipeline
        dummy_bytes = frame.compose(rw=0, addr=0, data=0)
        # Total inter-frame settle in µs; emitted as SLEEP_US below 1 ms (a
        # staging tied to a fast internal update tick — e.g. an 8 kHz datapath
        # stages within 125 µs), SLEEP_MS at whole milliseconds.
        gap_us = (int(frame.inter_frame_sleep_ms) * 1000
                  + int(frame.inter_frame_sleep_us))
        if gap_us > 0xFFFF:
            raise CompileError(
                f"inter-frame settle {gap_us} µs exceeds the 16-bit sleep "
                f"operand; use a smaller gap")

        # A FRAME that declares integrity fields (crc / status_ok) gets each
        # harvested response verified on-device while it still sits in the
        # slot: recompute the CRC over the covered window and compare it to
        # the received CRC byte, then mask-compare the status field. Either
        # mismatch drops the tick — a backward jump to the loop head, the
        # same mechanism as `return None` — so a corrupted or unprepared
        # frame never reaches STORE_SAMPLE; a systematic failure starves the
        # sample watchdog, which escalates loudly. Byte geometry comes from
        # the frame declaration; a window the byte-granular engine cannot
        # check is a CompileError, never a silently skipped check.
        verify = frame.crc is not None or frame.status_ok is not None
        crc_style = cov_off = cov_len = crc_off = 0
        st_off = st_mask = st_expect = 0
        if verify:
            try:
                if frame.crc is not None:
                    if frame.crc.compute_fn is not None:
                        raise ValueError(
                            "a compute_fn CRC is host-only; the on-device "
                            "check needs poly/init/xor_out")
                    crc_style = {"standard": 0,
                                 "input-lsb": 1}[frame.crc.feedback_style]
                    cov_off, cov_len = frame.crc_cover_window()
                    crc_off = frame.crc_byte_offset()
                if frame.status_ok is not None:
                    st_off, st_mask, st_expect = frame.status_byte()
            except (ValueError, KeyError) as e:
                raise CompileError(
                    f"{type(self._driver).__name__}: FRAME verification is "
                    f"not expressible on-device: {e}") from None

        with self._regs.scope():
            r_a = self._regs.get("__fchk_crc") if verify else 0
            r_b = self._regs.get("__fchk_tmp") if verify else 0
            for k in range(num_tx):
                if k < num_words:
                    tx_bytes = frame.compose(rw=0, addr=start_reg + k, data=0)
                else:
                    tx_bytes = dummy_bytes
                self._em.emit(Op.MEMCPY_IMM, slot, fw, *tx_bytes)
                self._em.emit(Op.REG_XFER, slot, slot, fw)
                if k >= pipeline:
                    out_idx = k - pipeline
                    self._em.emit(Op.MEMCPY, out_idx * dw, slot + doff, dw)
                    if frame.crc is not None:
                        self._em.emit(Op.CRC8, slot + cov_off, cov_len,
                                      frame.crc.poly, frame.crc.init,
                                      frame.crc.xor_out, r_a, crc_style)
                        self._em.emit(Op.LOAD_U8, r_b, slot + crc_off)
                        self._em.emit(Op.XOR_REG, r_a, r_b, r_a)
                        self._em.emit_jmp(Op.JNZ, self._loop_label, r_a)
                    if frame.status_ok is not None:
                        self._em.emit(Op.LOAD_U8, r_b, slot + st_off)
                        self._em.emit_cmp(Op.AND, r_b, st_mask, r_b)
                        self._em.emit_cmp(Op.CMP_EQ, r_b, st_expect, r_b)
                        self._em.emit_jmp(Op.JZ, self._loop_label, r_b)
                # Some parts (e.g. IIM-20670) require a settle delay between
                # a read-request frame and the frame that clocks the response
                # out. Constants such as FIXED_VALUE/WHO_AM_I are ready
                # immediately on MISO, which is why single-register probe
                # reads succeed without this pause, but sampled data
                # registers return stale/zero without it. FRAME schemas
                # that don't need the gap (e.g. BMI270 plain SPI) leave
                # both sleep fields at 0 and pay no overhead.
                if gap_us > 0 and k < num_tx - 1:
                    if gap_us % 1000 == 0:
                        self._em.emit_u16(Op.SLEEP_MS, gap_us // 1000)
                    else:
                        self._em.emit_u16(Op.SLEEP_US, gap_us)

    def _compile_expr_stmt(self, node):
        call = node.value
        if not isinstance(call, ast.Call):
            # A docstring is the one legal non-call expression statement.
            # A BoolOp (`ready and self.write(...)`) or bare comparison emits
            # no bytecode, so reject it rather than drop it.
            if isinstance(call, ast.Constant) and isinstance(call.value, str):
                return
            raise CompileError(
                f"Expression statement has no effect and cannot be "
                f"compiled: {ast.dump(call)} (line {node.value.lineno}). "
                f"Use an `if:` for conditionals; assign a read's result "
                f"or call a self.* verb directly.")

        if not self._is_self_method(call.func):
            raise CompileError(
                f"Unsupported expression: {ast.dump(call)} "
                f"(line {call.lineno})")

        method = call.func.attr

        if self._bus_kind == BUS_REGISTER:
            if method == "write":
                reg = self._eval_const(call.args[0])
                val = self._eval_const(call.args[1])
                dev = None
                for kw in call.keywords:
                    if kw.arg != "dev":
                        raise CompileError(
                            f"write(): unknown keyword {kw.arg!r}; the only "
                            f"measure()-side keyword is dev= (line "
                            f"{call.lineno})")
                    dev = self._eval_const(kw.value)
                opened = self._dev_open(dev, call.lineno)
                self._em.emit_reg(Op.REG_WRITE, reg, val)
                self._dev_close(opened)
                return
            if method == "read_burst":
                reg = self._eval_const(call.args[0])
                count = self._eval_const(call.args[1])
                into, dev = self._into_kwarg(call.keywords, call.lineno)
                self._claim_data_span(into, count, "read_burst", call.lineno)
                opened = self._dev_open(dev, call.lineno)
                self._em.emit_reg(Op.REG_READ_BURST, reg, count, into)
                self._dev_close(opened)
                return
            if method == "read":
                # Bare read in statement position (read-to-clear a latch): emit
                # the assignment form's transaction and discard the value.
                # Handled here so it doesn't route to the coefficient trace
                # path, which stages at offset 0 and allocates a work slot.
                dev = None
                for kw in call.keywords:
                    if kw.arg != "dev":
                        raise CompileError(
                            f"a bare read discards its value; signed=/endian= "
                            f"have no effect (line {call.lineno}).")
                    dev = self._eval_const(kw.value)
                reg = self._eval_const(call.args[0])
                if self._frame is not None:
                    if dev is not None:
                        raise CompileError(
                            f"dev= targets an I2C companion; a FRAME driver "
                            f"is SPI and declares none (line {call.lineno}).")
                    # FRAME driver: a bare read-to-clear must still clock the
                    # composed frame. Discard the loaded value into a scratch reg.
                    with self._regs.scope():
                        self._driver._emit_frame_read(reg,
                                                      self._regs.get("__bare_read"))
                    return
                opened = self._dev_open(dev, call.lineno)
                if len(call.args) > 1:
                    width = self._eval_const(call.args[1])
                    if not 1 <= width <= 4:
                        raise CompileError(
                            f"read(reg, width) supports width 1..4 (OP_LOAD), "
                            f"got {width} (line {call.lineno}).")
                    off = self._scalar_scratch_off("read(reg, width)")
                    self._em.emit_reg(Op.REG_READ_BURST, reg, width, off)
                else:
                    with self._regs.scope():
                        self._em.emit_reg(Op.REG_READ, reg,
                                          self._regs.get("__bare_read"))
                self._dev_close(opened)
                return

        if self._bus_kind == BUS_STREAM:
            if method == "write":
                val = self._eval_const(call.args[0])
                self._em.emit(Op.UART_WRITE, val)
                return

        # Fallback: if the driver instance defines this method, call
        # it with the evaluated arguments. The method emits bytecode
        # to the shared emitter directly. This is how higher-level
        # helpers like `read_until`, `read_n`, `store_sample_n`, and
        # `parse_u16_le` participate in measure() without each one
        # needing a hand-coded AST dispatch case.
        if self._driver is not None and hasattr(self._driver, method):
            fn = getattr(self._driver, method)
            args = [self._eval_const(a) for a in call.args]
            kwargs = {}
            for kw in call.keywords:
                kwargs[kw.arg] = self._eval_const(kw.value)
            fn(*args, **kwargs)
            return

        raise CompileError(
            f"Unsupported expression: self.{method}(...) "
            f"on {self._bus_kind} driver (line {call.lineno})")

    def _compile_if(self, node):
        skip_label = f"__if_skip_{id(node)}"

        test = node.test
        negated = False
        if isinstance(test, ast.UnaryOp) and isinstance(test.op, ast.Not):
            test = test.operand
            negated = True

        tmp_reg = self._regs.get("__if_tmp")   # dead after the branch; shared across all ifs

        # Each arm emits a comparison into tmp_reg and sets `skip_op` to the
        # branch that SKIPS the body (so the body runs when the test is true).
        if isinstance(test, ast.BinOp) and isinstance(test.op, ast.BitAnd):
            # if (var & mask): — run the body when masked bits are nonzero.
            src_reg = self._value_to_reg(test.left)
            mask = self._eval_const(test.right)
            self._em.emit_cmp(Op.AND, src_reg, mask & 0xFFFFFFFF, tmp_reg)
            skip_op = Op.JZ
        elif isinstance(test, ast.Compare) and len(test.ops) == 1:
            # if (var <op> const): signed comparison via CMP_EQ / CMP_LT.
            # <= / > use `var < rhs+1`; >= / > invert the < result.
            src_reg = self._value_to_reg(test.left)
            rhs = self._eval_const(test.comparators[0])
            op = test.ops[0]
            if isinstance(op, ast.Eq):
                self._em.emit_cmp(Op.CMP_EQ, src_reg, rhs & 0xFFFFFFFF, tmp_reg)
                skip_op = Op.JZ
            elif isinstance(op, ast.NotEq):
                self._em.emit_cmp(Op.CMP_EQ, src_reg, rhs & 0xFFFFFFFF, tmp_reg)
                skip_op = Op.JNZ
            elif isinstance(op, ast.Lt):
                self._em.emit_cmp(Op.CMP_LT, src_reg, rhs & 0xFFFFFFFF, tmp_reg)
                skip_op = Op.JZ
            elif isinstance(op, ast.GtE):
                self._em.emit_cmp(Op.CMP_LT, src_reg, rhs & 0xFFFFFFFF, tmp_reg)
                skip_op = Op.JNZ
            elif isinstance(op, ast.LtE):
                if rhs >= 0x7FFFFFFF:
                    raise CompileError(
                        f"'<=' against INT32_MAX is degenerate (always true); "
                        f"the `rhs + 1` bound would overflow the signed range. "
                        f"Rewrite the condition (line {node.lineno}).")
                self._em.emit_cmp(Op.CMP_LT, src_reg, (rhs + 1) & 0xFFFFFFFF, tmp_reg)
                skip_op = Op.JZ
            elif isinstance(op, ast.Gt):
                if rhs >= 0x7FFFFFFF:
                    raise CompileError(
                        f"'>' against INT32_MAX is degenerate (always false); "
                        f"the `rhs + 1` bound would overflow the signed range. "
                        f"Rewrite the condition (line {node.lineno}).")
                self._em.emit_cmp(Op.CMP_LT, src_reg, (rhs + 1) & 0xFFFFFFFF, tmp_reg)
                skip_op = Op.JNZ
            else:
                raise CompileError(
                    f"Unsupported comparison operator {type(op).__name__} "
                    f"(line {node.lineno}).")
        else:
            raise CompileError(
                f"Unsupported if condition: {ast.dump(node.test)} "
                f"(line {node.lineno}). Supported: if (var & mask): or "
                f"if var <op> const: with op in == != < > <= >=.")

        if negated:
            skip_op = Op.JNZ if skip_op == Op.JZ else Op.JZ

        self._em.emit_jmp(skip_op, skip_label, tmp_reg)
        for stmt in node.body:
            self._compile_stmt(stmt)
        if node.orelse:
            # The true path must jump over the else body — without this the
            # body falls through and BOTH branches execute. A body ending in
            # `return` makes this JMP dead; harmless, same as the loop-back
            # JMP after a terminal return. `elif` is a nested If in orelse
            # and recurses through this same path.
            end_label = f"__if_end_{id(node)}"
            self._em.emit_jmp(Op.JMP, end_label)
            self._em.label(skip_label)
            for stmt in node.orelse:
                self._compile_stmt(stmt)
            self._em.label(end_label)
        else:
            self._em.label(skip_label)

    def _compile_return(self, node):
        if node.value is None or (isinstance(node.value, ast.Constant) and
                                  node.value.value is None):
            self._em.emit_jmp(Op.JMP, self._loop_label)
            return

        if (isinstance(node.value, ast.Call) and
                self._get_name(node.value.func) == "Sample"):
            call = node.value
            if call.keywords:
                # return Sample(field=value, ...): write each computed value to
                # its declared field, then commit.
                self._emit_sample_fields(call.keywords, node.lineno)
            else:
                # return Sample(raw): raw bytes already staged in the buffer.
                self._em.emit(Op.STORE_SAMPLE)
            # A committed sample terminates the iteration: jump to the loop top
            # so a mid-body `return Sample(...)` (e.g. a FIFO gate's clean-read
            # path) doesn't fall through into the statements after it. Mirrors
            # `return None`; a terminal return's jump duplicates the loop-back
            # JMP `compile_function` appends, which is dead but harmless.
            self._em.emit_jmp(Op.JMP, self._loop_label)
            return

        raise CompileError(
            f"Unsupported return: {ast.dump(node.value)} "
            f"(line {node.lineno}). Supported: return None, return Sample(raw), "
            f"return Sample(field=value, ...)")

    @staticmethod
    def _field_width_bytes(f) -> int:
        """Bytes a declared output field occupies in the sample buffer."""
        return field_width(f)

    def _emit_sample_fields(self, keywords, lineno):
        """Write each `Sample(field=value)` kwarg into its declared output
        field, set the sample size, and commit. The layout (offset, width, byte
        order) comes from `set_output()`. A kwarg naming no field — or a field
        with no kwarg — is a hard error (the one drift that corrupts the wire)."""
        int_types = {'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32'}
        layout = {}
        for f in self._driver._output_fields:
            t = f.get('type', 'int16')
            if t not in int_types:
                raise CompileError(
                    f"Sample(field=value) writes a computed integer; output "
                    f"field {f['name']!r} has type {t!r} (line {lineno}). "
                    f"float/string fields are unsupported on the computed path.")
            w = self._field_width_bytes(f)
            layout[f['name']] = (int(f['byte_off']), w, f.get('byte_order', 'big'))

        given = [kw.arg for kw in keywords]
        if set(given) != set(layout):
            raise CompileError(
                f"Sample({', '.join(given)}) must assign exactly the declared "
                f"output fields {list(layout)} (line {lineno}).")

        for kw in keywords:
            field_off, width, border = layout[kw.arg]
            reg = self._value_to_reg(kw.value)
            self._emit_store_bytes(reg, field_off, width, border)
        # The sample extent is the furthest field end (offsets may gap).
        total = max(o + w for (o, w, _) in layout.values())
        self._em.emit(Op.SET_SAMPLE_SIZE, total)
        self._em.emit(Op.STORE_SAMPLE)

    def _emit_store_bytes(self, reg, off, width, byte_order):
        """Store `width` low bytes of r[reg] into sample_buf[off..], MSB-first
        for big-endian. STORE_U8 writes the low byte, so each higher byte is
        shifted down into a scratch register first."""
        tmp = self._regs.get("__store_scratch")
        for i in range(width):
            shift = 8 * (width - 1 - i) if byte_order == 'big' else 8 * i
            if shift == 0:
                self._em.emit(Op.STORE_U8, off + i, reg)
            else:
                self._em.emit(Op.SHR, reg, shift, tmp)
                self._em.emit(Op.STORE_U8, off + i, tmp)

    @staticmethod
    def _is_self_method(node) -> bool:
        """True for AST nodes of shape `self.<something>`."""
        return (isinstance(node, ast.Attribute) and
                isinstance(node.value, ast.Name) and
                node.value.id == "self")

    @staticmethod
    def _get_name(node) -> str:
        if isinstance(node, ast.Name):
            return node.id
        if isinstance(node, ast.Attribute):
            return node.attr
        return ""

    def _eval_const(self, node) -> int:
        if isinstance(node, ast.Constant):
            # bytes literals (e.g. `b'\n'`) pass through unchanged so
            # helpers like `read_until` can accept them as-is. Integer
            # promotion still happens for everything numeric.
            if isinstance(node.value, (bytes, bytearray, str)):
                return node.value
            return int(node.value)
        if (isinstance(node, ast.Attribute) and
                isinstance(node.value, ast.Name) and node.value.id == "self"):
            # `self.CMD_WORD`: an UPPER_CASE class-level integer constant —
            # the datasheet-as-code home for computed wire words (parity
            # bits, rw flags folded in by plain Python at class definition).
            # Resolved on the class, not the instance, so trace-time state
            # (coefficients, counters) can't leak into bytecode.
            if (self._driver is not None and node.attr.isupper()):
                val = getattr(type(self._driver), node.attr, None)
                if isinstance(val, int) and not isinstance(val, bool):
                    return val
            raise CompileError(
                f"self.{node.attr} is not a compile-time constant; only "
                f"UPPER_CASE class-level integer attributes resolve in "
                f"measure() (line {node.lineno})")
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
            return -self._eval_const(node.operand)
        if isinstance(node, ast.BinOp):
            left = self._eval_const(node.left)
            right = self._eval_const(node.right)
            if isinstance(node.op, ast.Add): return left + right
            if isinstance(node.op, ast.Sub): return left - right
            if isinstance(node.op, ast.Mult): return left * right
            if isinstance(node.op, ast.FloorDiv): return left // right
            if isinstance(node.op, ast.Pow): return left ** right
            if isinstance(node.op, ast.LShift): return left << right
            if isinstance(node.op, ast.RShift): return left >> right
            if isinstance(node.op, ast.BitOr): return left | right
            if isinstance(node.op, ast.BitAnd): return left & right
        raise CompileError(
            f"Cannot evaluate constant: {ast.dump(node)} "
            f"(line {node.lineno})")


# ── SensorDriver base class ────────────────────────────────

class SensorDriver:
    """
    Base class for NXS sensor drivers.

    A driver is a "datasheet in code" — it knows all register addresses,
    supported modes, and scale factors. The YAML config selects the mode;
    compile(config) produces bytecode + output field descriptors.
    """

    # Subclass-declared bus family. RegisterDriver → BUS_REGISTER,
    # StreamDriver → BUS_STREAM. Plain SensorDriver is abstract;
    # compile() rejects it.
    BUS_KIND: Optional[str] = None

    # mikroBUS reset active level ('low' default / 'high'). Firmware pulses the
    # shared mkbus_rst at bind for every driver kind; set 'high' for active-high
    # parts (e.g. FXOS8700).
    RESET_ACTIVE = 'low'

    def __init__(self):
        self._emitter = _Emitter()
        self._regs = _RegAlloc()
        # 64-bit work-buffer slots. Persists across configure() → measure()
        # (unlike _regs, which compile() resets between them), so coefficients
        # read in configure() stay addressable in the measure loop.
        self._work = _WorkAlloc(_ASTCompiler.VM_WORK_BUF_SIZE)
        self._sample_size = 14
        self._read_responses: Dict[int, list] = {}
        self._output_fields: List[dict] = []
        self._config: dict = {}
        self._params: Dict[str, ParamDescriptor] = {}
        self._patch_entries: List[PatchEntry] = []
        self._patch_accum: Dict[str, Dict] = {}  # param_name → {config_val → byte_val}
        # Monotonic counter for synthesising unique labels inside
        # multi-instruction helpers (read_until, read_n, etc.). Each
        # helper bumps this and prefixes its labels so two calls in
        # the same configure()/measure() don't collide.
        self._label_counter = 0
        # Swapped by _finalize_patches to capture per-parameter bytes;
        # normal use points at _record_patch.
        self._patch_recorder_fn = self._record_patch

    def _check_param_value(self, name, value, values, param_type, unit):
        """Raise CompileError if `value` is illegal for this param — outside
        [min, max] for a range param, or not in the allowed set for an enum.

        The single predicate every value (declared default, config override,
        runtime set) is checked against, so validation can't drift between
        entry points.
        """
        unit_str = f" {unit}" if unit else ""
        # Params serialize as u32; a non-integer override (e.g. a YAML string)
        # is always invalid. Reject it with a clear error rather than letting
        # the range comparison raise a bare TypeError. bool is an int subclass,
        # so exclude it explicitly.
        if not isinstance(value, int) or isinstance(value, bool):
            raise CompileError(
                f"Parameter {name!r} = {value!r} must be an integer for "
                f"{type(self).__name__}.")
        if param_type == "range":
            if not (values[0] <= value <= values[1]):
                raise CompileError(
                    f"Parameter {name!r} = {value!r} out of range for "
                    f"{type(self).__name__}. "
                    f"Valid range: {values[0]}-{values[1]}{unit_str}")
        elif value not in values:
            raise CompileError(
                f"Parameter {name!r} = {value!r} not supported by "
                f"{type(self).__name__}. "
                f"Valid values: {values}{unit_str}")

    def declare_param(self, name: str, values: list, default: Any,
                      param_type: str = "enum", unit: str = "",
                      kind: str = "reload"):
        """Declare a configurable parameter with valid values.

        Call in configure() before using the parameter. Records the
        parameter metadata for the capabilities descriptor AND validates
        the values baked into the image: the range/enum shape, the declared
        ``default``, and any config override. `configure()` fails fast at
        declaration time rather than storing a bad value that'd be applied
        at load (e.g. an out-of-range initial PWM drive).
        """
        if param_type not in ("enum", "range"):
            raise CompileError(
                f"declare_param({name!r}): param_type must be 'enum' or "
                f"'range', got {param_type!r}")
        if param_type == "range" and (len(values) != 2 or values[0] > values[1]):
            raise CompileError(
                f"declare_param({name!r}): a range param must declare exactly "
                f"[min, max] with min <= max, got {values!r}")
        # The descriptor wire carries parameter values as int32: a
        # fractional value would truncate silently (0.25 → 0, colliding
        # with 0.5 → 0) and ship a corrupt allowed set. Reject here, not
        # on the wire.
        for v in values:
            if not isinstance(v, int) or isinstance(v, bool):
                raise CompileError(
                    f"declare_param({name!r}): value {v!r} is not an "
                    f"integer — the descriptor wire carries int32. Encode "
                    f"fractional physical values in a smaller integer unit "
                    f"(mHz, mV), or drop the fractional rows with a stated "
                    f"exclusion, or make the setting a compile-time config "
                    f"key.")
        # The declared default must be legal; a config override, if present,
        # must be too. Both reach the image — default as the fallback,
        # current as the value applied at load.
        self._check_param_value(name, default, values, param_type, unit)
        if name in self._config:
            current = self._config[name]
            self._check_param_value(name, current, values, param_type, unit)
        else:
            current = default
        if kind not in ("reload", "live"):
            raise CompileError(
                f"declare_param({name!r}): kind must be 'reload' or 'live', "
                f"got {kind!r}")
        # A range param has no bytecode patch site, so the firmware only
        # accepts it as live (PARAM_RANGE_NOT_LIVE otherwise). Require live
        # here so a reload range can't compile host-side yet never load.
        if param_type == "range" and kind != "live":
            raise CompileError(
                f"declare_param({name!r}): a range param must be kind='live' "
                f"(the firmware rejects a non-live range).")
        self._params[name] = ParamDescriptor(
            name=name, param_type=param_type, values=values,
            default=default, current=current, unit=unit, kind=kind,
        )
        self._patch_accum[name] = {}

    def drive_pwm(self, freq: int = 1000, duty: int = 50):
        """Drive the mikroBUS PWM pin via two live parameters.

        Call in configure(). ``pwm_freq`` (Hz) and ``pwm_duty`` (%) are
        ``kind="live"``: the host retunes them at runtime with no VM reload,
        and the runner re-derives period/pulse and updates the line. Their
        presence is how the firmware knows the driver drives PWM. ``freq`` and
        ``duty`` are the initial drive, applied when the driver loads.
        """
        if not (PWM_FREQ_MIN_HZ <= freq <= PWM_FREQ_MAX_HZ):
            raise CompileError(
                f"drive_pwm(freq={freq}): must be "
                f"{PWM_FREQ_MIN_HZ}-{PWM_FREQ_MAX_HZ} Hz")
        if not (0 <= duty <= 100):
            raise CompileError(f"drive_pwm(duty={duty}): must be 0-100 %")
        self.declare_param("pwm_freq", values=[PWM_FREQ_MIN_HZ, PWM_FREQ_MAX_HZ],
                           default=freq, param_type="range", unit="Hz",
                           kind="live")
        self.declare_param("pwm_duty", values=[0, 100], default=duty,
                           param_type="range", unit="%", kind="live")

    def persistent(self, name: str, init: int = 0):
        """Declare a loop-carried integer, initialized once in the configure()
        prologue and preserved across measure() iterations (a verdict
        accumulator, edge counter, or carried previous reading). Call in
        configure().
        """
        reg = self._regs.pin(name)
        self._emitter.emit_u32(Op.LOAD_IMM, reg, init & 0xFFFFFFFF)

    def get_param(self, name: str) -> Any:
        """Read a declared parameter's value from config, validating it
        against the allowed set.

        Use this instead of ``config.get(name, default)`` inside
        ``configure()`` — it uses the ``declare_param`` metadata as the
        single source of truth for what's valid and raises
        ``CompileError`` with a helpful message when the user passes an
        unsupported value. The CLI catches that and prints it without a
        traceback, so the end-user sees "Valid values: [2, 4, 16, 32] g"
        rather than ``KeyError: 8``.

        Must be called *after* the matching ``declare_param(name, ...)``.
        """
        if name not in self._params:
            raise CompileError(
                f"get_param({name!r}): parameter not declared — "
                f"call declare_param({name!r}, ...) first")
        p = self._params[name]
        if name not in self._config:
            return p.default
        val = self._config[name]
        self._check_param_value(name, val, p.values, p.param_type, p.unit)
        return val

    def _record_patch(self, param_name: str, config_value: Any,
                      byte_value: int, offset: int, size: int = 1,
                      reg: Optional[int] = None):
        """Record a patchable byte in the bytecode stream."""
        # declare_param seeds _patch_accum; a tag without it has no value set.
        param = self._params.get(param_name)
        if param_name not in self._patch_accum or param is None:
            raise CompileError(
                f"param=({param_name!r}, ...) tagged on a write before "
                f"declare_param({param_name!r}, ...); declare the parameter "
                f"first so its value set is known and the write patches.")
        # The value_map is built from the declared set; the tagged value must
        # be in it.
        self._check_param_value(param_name, config_value, param.values,
                                param.param_type, param.unit)
        self._patch_accum[param_name][config_value] = byte_value
        self._patch_entries.append(PatchEntry(
            offset=offset, param_name=param_name,
            value_map={},  # filled in _finalize_patches
            reg=reg, size=size,
        ))

    def _patch_poll_rate(self, sleep_operand_offset: int):
        """Patch a poll loop's SLEEP_MS interval from a declared sample_rate.

        The interval is a pure function of the rate, so the value_map is built
        directly and carried through _finalize_patches without a re-trace."""
        p = self._params.get("sample_rate")
        if p is None or p.param_type != "enum":
            return
        # A tagged register write already owns the rate; don't add a second
        # site on the loop tick.
        if any(pe.param_name == "sample_rate" for pe in self._patch_entries):
            return
        value_map = {rate: max(1, 1000 // int(rate)) for rate in p.values}
        self._patch_entries.append(PatchEntry(
            offset=sleep_operand_offset, param_name="sample_rate",
            value_map=value_map, size=2,
        ))

    def _patch_drdy_div(self, div_operand_offset: int, base_hz: int):
        """Patch a drdy loop's OP_EVENT_DIV divider from a declared
        sample_rate on a fixed-sync part (`drdy_base_hz`).

        Mirrors `_patch_poll_rate`: the divider is a pure function of the
        rate, so the value_map is built directly. Every declared rate must
        divide the sync exactly — a non-divisor would silently deliver a
        different rate than the one the host set."""
        p = self._params.get("sample_rate")
        if p is None or p.param_type != "enum":
            return
        # drdy_base_hz asserts "no rate register" — a part with one (the
        # ordinary ODR-divider IMU) tags that register and must NOT divide
        # the sync too: the two would double-pace. Loud contradiction, not
        # a silent pick.
        if any(pe.param_name == "sample_rate" for pe in self._patch_entries):
            raise CompileError(
                "drdy_base_hz declared, but sample_rate already patches a "
                "register write — a part with a rate register paces through "
                "it; drop drdy_base_hz (or the param= tag, if the register "
                "is not the rate).")
        for rate in p.values:
            if rate <= 0 or base_hz % int(rate) != 0:
                raise CompileError(
                    f"sample_rate value {rate} does not divide the {base_hz} "
                    f"Hz hardware sync; declare exact divisors (base/N) so "
                    f"every settable rate is delivered exactly.")
        value_map = {rate: base_hz // int(rate) for rate in p.values}
        self._patch_entries.append(PatchEntry(
            offset=div_operand_offset, param_name="sample_rate",
            value_map=value_map, size=2,
        ))

    # Auto-chunk threshold for sleep_ms(). Above this, the call expands
    # into a sequence of OP_SLEEP_MS instructions, each of which returns
    # VM_YIELD from the firmware VM and gives the runner thread a chance
    # to observe a STOP request. Bounds STOP latency to one chunk.
    SLEEP_CHUNK_MS = 50

    # Hard cap on auto-chunked sleeps. Pure unrolling has linear bytecode
    # cost, and the VM's 2 KB program size leaves no room for sleeps
    # measured in seconds. A real runtime chunked loop would need an
    # OP_SUB/OP_DEC opcode the VM does not currently provide; that is a
    # firmware-side change tracked separately. Until then, sleeps longer
    # than this are almost always bugs (typical cause: the author meant
    # sleep_us). Cap covers every realistic cold-boot or calibration
    # delay that has appeared in shipped drivers.
    SLEEP_MAX_UNROLLED_MS = 1000

    def sleep_ms(self, ms: int):
        if ms <= self.SLEEP_CHUNK_MS:
            self._emitter.emit_u16(Op.SLEEP_MS, ms)
            return
        if ms > self.SLEEP_MAX_UNROLLED_MS:
            raise CompileError(
                f"sleep_ms({ms}) exceeds the {self.SLEEP_MAX_UNROLLED_MS} "
                f"ms unrolled cap. The VM lacks a runtime decrement "
                f"opcode, so sleeps must unroll into "
                f"OP_SLEEP_MS({self.SLEEP_CHUNK_MS}) chunks at compile "
                f"time. If this is a microsecond delay, use sleep_us(). "
                f"For genuine long waits, factor the driver to use "
                f"trigger=drdy or split the wait across measure-loop "
                f"iterations.")
        full = ms // self.SLEEP_CHUNK_MS
        remainder = ms % self.SLEEP_CHUNK_MS
        for _ in range(full):
            self._emitter.emit_u16(Op.SLEEP_MS, self.SLEEP_CHUNK_MS)
        if remainder > 0:
            self._emitter.emit_u16(Op.SLEEP_MS, remainder)

    def sleep_us(self, us: int):
        self._emitter.emit_u16(Op.SLEEP_US, us)

    def read(self, reg, width=1, signed=False, endian="big", dev=None):
        """Read a register/command word. `read(reg)` returns one byte;
        `read(reg, width)` reads `width` bytes — big-endian unsigned by
        default, or `signed=True` / `endian="little"` for a signed or
        little-endian field. `dev=` targets a declared I2C companion for
        this access (bracketed: the bus returns to the primary after).
        When used as
        `self.cN = self.read(reg, width, signed=True)` in `configure()` it
        returns a `WideRef` — the bytes are CVT64'd into a persistent
        work-buffer slot so the measure loop reads them back as a 64-bit
        coefficient (signed coefficients sign-extend correctly). In
        `configure()` a width-1 value-read returns a compile-time mock that
        raises if used in logic; runtime conditionals and read-modify-write
        belong in `measure()`. Shared by every driver family (command-response
        and register); `RegisterDriver` overrides only the custom-`FRAME`
        path."""
        if not 1 <= width <= 4:
            raise CompileError(
                f"read(reg, width) supports width 1..4 (OP_LOAD), got {width}.")
        if width == 1 and (signed or endian != "big"):
            raise CompileError(
                "read(reg, signed=/endian=) needs width > 1 (a single byte "
                "has no byte order; use width 2+ for a signed field).")
        addr = self._companion_addr(dev) if dev is not None else None
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, addr)
        if width > 1:
            # Stage the burst through ONE reused scratch register: the value is
            # dead the moment CVT64 copies it into the work buffer, so a fresh
            # per-coefficient register name would permanently burn a slot and
            # exhaust the 8-register file on parts with many coefficients.
            self._emitter.emit_reg(Op.REG_READ_BURST, reg, width, 0)
            if addr is not None:
                self._emitter.emit(Op.I2C_TARGET, 0)
            r = self._regs.get("__coef_scratch")
            self._emitter.emit(Op.LOAD, r, 0, _load_spec_byte(width, signed, endian))
            off = self._work.alloc()
            self._emitter.emit(Op.CVT64, r, off)
            return WideRef(off)
        dst = self._regs.get(f"__reg_read_{reg}")
        self._emitter.emit_reg(Op.REG_READ, reg, dst)
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, 0)
        responses = self._read_responses.get((dev, reg) if dev else reg)
        val = responses.pop(0) if responses else 0
        # In configure() the value is a compile-time mock; hand back a sentinel
        # so any use of it fails loudly. probe() keeps the real mock for its
        # WHO_AM_I assert.
        if self._trace_phase == "configure":
            return _TraceReadValue(reg)
        return val

    def set_sample_size(self, size: int):
        if not (0 < size <= _ASTCompiler.VM_SAMPLE_BUF_SIZE):
            raise CompileError(
                f"set_sample_size({size}): the sample buffer holds "
                f"{_ASTCompiler.VM_SAMPLE_BUF_SIZE} bytes.")
        self._sample_size = size
        self._emitter.emit(Op.SET_SAMPLE_SIZE, size)

    def set_output(self, fields: List[dict]):
        """Declare the output vector fields with scales and units.

        Called from configure() after register setup. The fields describe
        how the raw sample bytes map to physical values.

        Each field dict: {name, scale, unit?, offset? (default 0),
        scale_param? (default None)}. A field whose name infers a known SI
        semantic inherits that semantic's canonical unit from the constants
        registry when `unit` is omitted, and declaring a conflicting unit
        is a compile error — the descriptor always carries the canonical SI
        unit (temperature is kelvin). GENERIC and non-SI fields (humidity,
        NMEA) have no canonical unit to inherit, so they must declare
        `unit` — `''` is the explicit unitless declaration; omitting the
        key is a compile error. When `scale_param` names a declared
        parameter, `scale` is the *base* (per-unit-of-param) factor and
        The device forms the effective SI scale on-board as `scale ×
        param.current_value` — so the output tracks a live full-scale
        range without a re-upload. The param's values must be the linear
        coefficient (the ROS-unit magnitude, e.g. accel_fs ∈ {2,4,8,16}).
        """
        self._output_fields = []
        for f in fields:
            semantic = infer_semantic(f['name'])
            canonical = FieldSemantics.FIELD_SEMANTIC_UNIT.get(semantic)
            unit = f.get('unit')
            if canonical is not None:
                if unit is not None and unit != canonical:
                    raise CompileError(
                        f"set_output field {f['name']!r}: unit {unit!r} "
                        f"contradicts the canonical SI unit {canonical!r} "
                        f"for its semantic — emit SI at the source (omit "
                        f"`unit` to inherit it).")
                unit = canonical
            elif unit is None:
                raise CompileError(
                    f"set_output field {f['name']!r}: no canonical SI unit "
                    f"to inherit for its semantic — declare 'unit' "
                    f"explicitly ('' for unitless).")
            entry = {
                'name': f['name'],
                'type': f.get('type', 'int16'),
                'byte_order': f.get('byte_order', 'big'),
                'scale': float(f['scale']),
                'offset': float(f.get('offset', 0.0)),
                'unit': unit,
                'semantic': semantic,
            }
            # A scale_param links the field's scale to a live parameter;
            # it must already be declared so the NXS can resolve it to a
            # param index the firmware multiplies by at SI-projection time.
            scale_param = f.get('scale_param')
            if scale_param is not None and scale_param not in self._params:
                raise CompileError(
                    f"set_output field {f['name']!r}: scale_param "
                    f"{scale_param!r} is not a declared parameter — call "
                    f"declare_param({scale_param!r}, ...) before set_output().")
            # effective_scale = scale × current_value, so the value is the
            # multiplier; a 0 in the set zeroes the field. Register-code value
            # sets (0, 1, 2, …) are the common mistake here.
            if scale_param is not None and 0 in self._params[scale_param].values:
                raise CompileError(
                    f"set_output field {f['name']!r}: scale_param "
                    f"{scale_param!r} has 0 in its values; the value multiplies "
                    f"the scale, so 0 zeroes the field. Its values must be "
                    f"physical magnitudes, not register codes.")
            entry['scale_param'] = scale_param
            # Strings carry an explicit width; numeric types' width is
            # implied by `type` so `count` isn't meaningful for them.
            if entry['type'] == 'string':
                entry['count'] = int(f.get('count', 0))
            # `at=` places the field at an explicit sample-buffer byte —
            # the binary-record idiom, where fields map onto scattered
            # offsets inside a captured frame.
            if 'at' in f:
                entry['byte_off'] = int(f['at'])
            self._output_fields.append(entry)

        # Explicit placement is all-or-none: mixing `at=` fields with
        # pack-sequential ones makes the implicit offsets depend on
        # declaration order in a way that reads as a bug.
        n_at = sum(1 for e in self._output_fields if 'byte_off' in e)
        if 0 < n_at < len(self._output_fields):
            raise CompileError(
                f"set_output mixes fields with and without 'at': declare "
                f"an explicit byte offset on every field or on none.")
        resolve_field_offsets(self._output_fields)
        validate_field_layout(self._output_fields,
                              _ASTCompiler.VM_SAMPLE_BUF_SIZE)

    @staticmethod
    def measure_loop(trigger: str = "drdy", sample_rate: int = 250,
                     drdy_base_hz: int = 0,
                     when: Optional[tuple] = None, default: bool = False):
        """Decorator marking a method for AST compilation as the measure loop.

        trigger="from_config" reads the trigger mode from the config dict.

        `drdy_base_hz` declares the hardware sync rate of a fixed-rate part
        (no ODR divider register; the DRDY pin runs at this rate). With a
        declared `sample_rate` param whose values are exact divisors of the
        base, the drdy loop is paced by dividing the sync at the source
        (`OP_EVENT_DIV`): the delivered spacing is `base/rate` sync periods,
        crystal-exact, and `set sample_rate` patches the divider. Parts with
        a real ODR register leave this 0 and tag that register instead.

        `when=(key, value)` declares a config-selected variant: the driver
        may carry several decorated methods, all keyed on the same config
        key, and `compile(config)` picks the one whose value matches — the
        idiom for parts whose protocols need different framing bytecode
        (which a runtime param can't patch). Mark exactly one variant
        `default=True` to be chosen when the config omits the key.
        """
        def decorator(func):
            func._measure_loop = True
            func._trigger = trigger
            func._sample_rate = sample_rate
            func._drdy_base_hz = drdy_base_hz
            func._when = when
            func._when_default = default
            return func
        return decorator

    def _select_measure_fn(self, config: dict):
        """Resolve the measure method this compile targets.

        A driver has either one undecorated-selector measure loop, or a
        set of `when=(key, value)` variants keyed on one config key — the
        config picks the variant, the `default=True` one standing in when
        the key is absent. Every ambiguity is a CompileError: two bare
        loops, mixed bare/`when` loops, split keys, zero or two defaults,
        or a config value no variant claims."""
        variants = []
        for name in dir(type(self)):
            fn = getattr(type(self), name)
            if callable(fn) and getattr(fn, "_measure_loop", False):
                variants.append(fn)
        if not variants:
            return None

        tagged = [fn for fn in variants if getattr(fn, "_when", None)]
        if not tagged:
            if len(variants) > 1:
                names = sorted(fn.__name__ for fn in variants)
                raise CompileError(
                    f"{len(variants)} measure loops ({', '.join(names)}) "
                    f"but no `when=` selectors — tag each variant with "
                    f"when=(key, value), or keep a single measure loop.")
            return variants[0]
        if len(tagged) != len(variants):
            bare = sorted(fn.__name__ for fn in variants
                          if not getattr(fn, "_when", None))
            raise CompileError(
                f"measure loops {', '.join(bare)} carry no `when=` while "
                f"others do — tag every variant or none.")

        keys = {fn._when[0] for fn in tagged}
        if len(keys) != 1:
            raise CompileError(
                f"measure-loop `when=` selectors use different config "
                f"keys {sorted(keys)} — all variants key on one.")
        key = keys.pop()
        values = [fn._when[1] for fn in tagged]
        if len(set(values)) != len(values):
            raise CompileError(
                f"two measure loops claim the same {key!r} value — "
                f"each variant's `when` value must be distinct.")

        chosen = config.get(key)
        if chosen is None:
            defaults = [fn for fn in tagged
                        if getattr(fn, "_when_default", False)]
            if len(defaults) != 1:
                raise CompileError(
                    f"config omits {key!r} and {len(defaults)} variants "
                    f"are marked default=True — exactly one must be.")
            fn = defaults[0]
            config[key] = fn._when[1]
            return fn
        for fn in tagged:
            if fn._when[1] == chosen:
                return fn
        raise CompileError(
            f"config {key}={chosen!r} matches no measure variant; "
            f"declared values: {sorted(map(str, values))}.")

    def compile(self, config: Optional[dict] = None) -> CompiledDriver:
        """Compile the driver into VM bytecode.

        Args:
            config: Sensor configuration dict (from YAML). Passed to
                    configure(). If None, configure() is called with
                    an empty dict.
        """
        if self.BUS_KIND is None:
            raise CompileError(
                f"{type(self).__name__} inherits SensorDriver directly; "
                "use RegisterDriver or StreamDriver as the base class.")

        if config is None:
            config = {}
        # Canonicalise known enum-string params to their integer codes
        # before anything else touches `config`. Downstream validation
        # (in declare_param and the BUSES auto-inject) compares against
        # the declared int value set, so `bus=spi` on the CLI needs to
        # become `bus=1` here or the comparison will silently mismatch.
        if isinstance(config.get('bus'), str):
            config['bus'] = _BUS_NAME_TO_CODE.get(
                config['bus'].lower().strip(), config['bus'])
        self._config = config

        self._emitter = _Emitter()
        self._regs = _RegAlloc()
        self._work = _WorkAlloc(_ASTCompiler.VM_WORK_BUF_SIZE)
        self._output_fields = []
        self._params = {}
        self._patch_entries = []
        self._trace_phase = None
        self._patch_accum = {}

        # Snapshot the driver's declared `_read_responses` to refill
        # the queue before each re-trace in _finalize_patches. Tracing
        # pops from the lists; without a template the probe()/configure()
        # re-trace cannot repeat cleanly.
        self._read_responses_template = {
            reg: list(vals) for reg, vals in self._read_responses.items()
        }

        # Phase 0: Auto-emit the WHO_AM_I check from class attributes
        # before any user-defined probe() body. The Python `assert who
        # == X` lines in driver probe() methods don't compile to runtime
        # bytecode (the AST compiler doesn't model `ast.Assert`), so
        # without this prologue the VM has no way to detect "wrong
        # driver loaded for this hardware." See design-rationale §F.
        self._validate_companions()
        self._emit_who_am_i_prologue()

        # Phase 1: Trace probe()
        if hasattr(self, "probe"):
            self._trace_phase = "probe"
            self.probe()

        # Phase 2: Trace configure()
        # Select the measure variant BEFORE configure() runs: selection
        # stamps the chosen value into `config`, and configure() branches
        # on the same key (chip-config writes differ per variant).
        measure_fn = self._select_measure_fn(config)

        if hasattr(self, "configure"):
            self._trace_phase = "configure"
            self.configure(config)
        self._trace_phase = None

        # Phase 3: AST-compile measure()
        if measure_fn is not None:
            self._regs.reset()
            trigger = getattr(measure_fn, "_trigger", "drdy")
            sample_rate = getattr(measure_fn, "_sample_rate", 250)

            # "from_config" reads trigger and sample_rate from config dict
            if trigger == "from_config":
                trigger = config.get("trigger", "drdy")
            # Always allow config to override sample_rate
            sample_rate = config.get("sample_rate", sample_rate)

            ast_compiler = _ASTCompiler(
                self._emitter, self._regs, trigger, sample_rate, self.BUS_KIND,
                frame=getattr(self, 'FRAME', None),
                driver=self,
                drdy_base_hz=getattr(measure_fn, "_drdy_base_hz", 0))
            ast_compiler.compile_function(measure_fn)
        elif isinstance(self, I2cCommandDriver):
            # Command-response I²C drivers use procedural code-gen
            # instead of the AST path — their measure loop is a fixed
            # pattern (command → wait → read → CRC-verify → pack →
            # store). See I2cCommandDriver._emit_measure_loop.
            self._emit_measure_loop()

        bytecode = self._emitter.build()

        # Finalize patch map: build full value_map for each patch entry
        # by compiling the driver with each valid parameter value
        finalized_patches = self._finalize_patches(config)

        # Auto-declare the runtime `bus` param on every register driver.
        # The set of allowed values comes from the driver's `BUSES`
        # class attribute — a chip-level fact declared at the top of
        # the driver class (`BUSES = ('spi',)` for an IIM-20670-style
        # SPI-only part, `BUSES = ('i2c',)` for a Sensirion-style
        # I²C-only part, `BUSES = ('i2c', 'spi')` for a part that
        # truly supports both). The first entry in the tuple is the
        # default when the user doesn't pass `bus=...`. Firmware
        # rebinds hal::RegDeviceI based on the selected value;
        # patch_offset stays NONE (runtime side-effect, no bytecode
        # patching).
        if self.BUS_KIND == BUS_REGISTER and 'bus' not in self._params:
            buses = getattr(self, 'BUSES', None)
            if not buses:
                raise CompileError(
                    f"{type(self).__name__}: BUSES class attribute is "
                    f"empty or missing. Declare which physical buses "
                    f"the sensor supports, e.g. BUSES = ('spi',) for a "
                    f"SPI-only part.")
            bus_values = []
            for name in buses:
                code = _BUS_NAME_TO_CODE.get(name.lower())
                if code is None:
                    raise CompileError(
                        f"{type(self).__name__}: BUSES contains unknown "
                        f"entry {name!r}. Valid entries: "
                        f"{sorted(_BUS_NAME_TO_CODE.keys())}")
                if code not in bus_values:
                    bus_values.append(code)
            default_bus = bus_values[0]
            # A `BUS = '<name>'` class attribute (the one-line Communication
            # Profile selector) overrides the BUSES-order default.
            declared = getattr(self, 'BUS', None)
            if declared is not None:
                code = _BUS_NAME_TO_CODE.get(str(declared).lower().strip())
                if code is None or code not in bus_values:
                    raise CompileError(
                        f"{type(self).__name__}: BUS = {declared!r} must name "
                        f"one of BUSES {buses}")
                default_bus = code
            current_bus = default_bus
            if 'bus' in config:
                raw = config['bus']
                if isinstance(raw, str):
                    raw = _BUS_NAME_TO_CODE.get(raw.lower().strip(), raw)
                if raw not in bus_values:
                    bus_labels = [_BUS_CODE_TO_NAME[c] for c in bus_values]
                    raise CompileError(
                        f"Parameter 'bus' = {config['bus']!r} not "
                        f"supported by {type(self).__name__}. "
                        f"Valid values: {bus_labels}")
                current_bus = raw
            self._params['bus'] = ParamDescriptor(
                name='bus', param_type='enum',
                values=bus_values,
                default=default_bus,
                current=current_bus,
                unit='',
            )

        # Auto-inject a runtime `reset_active` param when the driver overrides the
        # mikroBUS reset polarity. Firmware pulses mkbus_rst at bind for every
        # driver kind, so this is not gated on the bus family.
        reset_active = str(getattr(self, 'RESET_ACTIVE', 'low')).lower().strip()
        if reset_active not in ('low', 'high'):
            raise CompileError(
                f"{type(self).__name__}: RESET_ACTIVE must be 'low' or "
                f"'high', got {getattr(self, 'RESET_ACTIVE')!r}")
        if reset_active == 'high' and 'reset_active' not in self._params:
            self._params['reset_active'] = ParamDescriptor(
                name='reset_active', param_type='enum',
                values=[0, 1], default=1, current=1, unit='')

        cls = type(self)
        who_am_i_values = [
            int(v) & 0xFF
            for v in getattr(cls, 'WHO_AM_I_VALUES', []) or []
        ][:16]
        skip_reason = getattr(cls, 'WHO_AM_I_SKIP_REASON', None)
        if skip_reason is not None:
            skip_reason = str(skip_reason).strip() or None
        # Only enforce the WHO_AM_I three-tier rule when the driver
        # class explicitly declares WHO_AM_I_VALUES somewhere in its
        # MRO. Drivers that don't declare it at all (typical for test
        # stubs that exercise isolated compiler paths) are exempt —
        # they aren't deployed and the audit-trail concern is moot.
        # Stream drivers (UART) are also exempt, since the WHO_AM_I
        # probe has no meaning for a byte-stream bus.
        who_am_i_declared = any(
            'WHO_AM_I_VALUES' in c.__dict__ for c in cls.__mro__
        )
        if self.BUS_KIND == BUS_REGISTER and who_am_i_declared:
            if not who_am_i_values and not skip_reason:
                raise CompileError(
                    f"{cls.__name__}: WHO_AM_I_VALUES = [] requires a "
                    f"non-empty WHO_AM_I_SKIP_REASON attribute explaining "
                    f"why this driver opts out of the WHO_AM_I probe. "
                    f"Either declare a real WHO_AM_I check (`WHO_AM_I_REG`, "
                    f"`WHO_AM_I_VALUES`), define a synthetic `probe()` "
                    f"that reads any register and asserts a known "
                    f"reset-state value, or document the opt-out:\n"
                    f"  WHO_AM_I_SKIP_REASON = \"...\"")
            if who_am_i_values and skip_reason:
                raise CompileError(
                    f"{cls.__name__}: WHO_AM_I_SKIP_REASON is set but "
                    f"WHO_AM_I_VALUES is non-empty. The skip reason "
                    f"only applies when opting out "
                    f"(WHO_AM_I_VALUES = []). Remove "
                    f"WHO_AM_I_SKIP_REASON or empty WHO_AM_I_VALUES.")
        # A declared reload/enum param that patches no bytecode has no runtime
        # effect: a `set` validates and reloads identical bytecode. Injected
        # runtime params (bus, reset_active) bypass declare_param, so they are
        # absent from _patch_accum and exempt.
        patched = {pe.param_name for pe in finalized_patches}
        for name, p in self._params.items():
            if (name in self._patch_accum and p.kind == "reload"
                    and p.param_type == "enum" and name not in patched):
                raise CompileError(
                    f"param {name!r} is declared reload/enum but patches no "
                    f"bytecode; a `set {name}` would reload identical bytecode. "
                    f"Tag a write with param=({name!r}, ...), or remove the "
                    f"declaration.")

        # A live param takes effect only if a runtime consumer reads its
        # current_value: the PWM pair, or a param named as an output field's
        # scale_param. Any other live param is a no-op set.
        live_consumers = {"pwm_freq", "pwm_duty"}
        scale_refs = {f.get("scale_param") for f in self._output_fields
                      if f.get("scale_param")}
        for name, p in self._params.items():
            if (p.kind == "live" and name not in live_consumers
                    and name not in scale_refs):
                raise CompileError(
                    f"param {name!r} is kind='live' but nothing reads it at "
                    f"runtime (not PWM, not an output scale_param); a "
                    f"`set {name}` would be a no-op. Use kind='reload', or "
                    f"reference it as a scale_param.")

        return CompiledDriver(
            bytecode=bytecode,
            sample_size=self._sample_size,
            name=cls.__name__,
            config=config,
            output_fields=resolve_field_offsets(list(self._output_fields)),
            params=list(self._params.values()),
            patch_map=finalized_patches,
            who_am_i_reg=int(getattr(cls, 'WHO_AM_I_REG', 0) or 0),
            who_am_i_values=who_am_i_values,
            i2c_addrs=[
                int(a) & 0x7F
                for a in getattr(cls, 'I2C_ADDRS', []) or []
            ][:8],
            who_am_i_skip_reason=skip_reason,
            bus_config=self._build_bus_config(cls, config),
        )

    def _build_bus_config(self, cls, config: dict) -> Optional[list]:
        """Assemble the NXS bus_config trailer: one register-access profile
        per bus a register driver supports.

        Register drivers emit a profile for every entry in ``BUSES`` so the
        runtime ``bus`` switch picks the active one with no recompile. Each
        profile comes from the driver's ``SPI_PROFILE`` / ``I2C_PROFILE``
        descriptor, or conventional wire-shape defaults when the driver
        declares none. Switches the firmware does not yet apply (SMBus PEC,
        no-auto-increment, UART framing) are rejected rather than baked into
        the image and dropped at runtime. Returns None only for a stream
        driver (no trailer); a register driver always returns at least the
        default profile per bus.
        """
        if self.BUS_KIND == BUS_REGISTER:
            profiles = []
            seen = set()
            for name in getattr(cls, 'BUSES', ('i2c', 'spi')):
                key = str(name).lower()
                if key in seen:
                    continue
                seen.add(key)
                if key == 'i2c':
                    profiles.append(self._i2c_profile_dict(cls))
                elif key == 'spi':
                    profiles.append(self._spi_profile_dict(cls))
            return profiles or None
        if self.BUS_KIND == BUS_STREAM:
            if isinstance(getattr(cls, 'UART_PROFILE', None), UartProfile):
                raise CompileError(
                    "UART_PROFILE is not yet applied by firmware; set the "
                    "peripheral baud with set_baud() in probe()")
            return None
        return None

    @staticmethod
    def _spi_profile_dict(cls) -> dict:
        """One SPI register-access profile dict for the NXS trailer, from the
        driver's ``SPI_PROFILE`` descriptor (conventional wire-shape defaults
        when the driver declares none)."""
        prof = getattr(cls, 'SPI_PROFILE', None)
        if not isinstance(prof, SpiProfile):
            prof = SpiProfile()
        if prof.auto_inc == 'none':
            raise CompileError(
                "SpiProfile(auto_inc='none'): parts without auto-increment "
                "are not supported; use 'implicit' or 'msb'")
        max_hz = int(prof.max_hz) if prof.max_hz else 0
        spi_mode = (prof.mode & 0x03) | (0x04 if prof.bit_order == 'lsb' else 0)
        return {
            'kind': 'spi', 'max_hz': max_hz, 'spi_mode': spi_mode,
            'addr_bytes': prof.addr_bytes, 'rw_read_level': prof.rw_read_level,
            'dummy_bytes': prof.dummy_bytes, 'auto_inc': prof.auto_inc,
        }

    @staticmethod
    def _i2c_profile_dict(cls) -> dict:
        """One I²C register-access profile dict for the NXS trailer, from the
        driver's ``I2C_PROFILE`` descriptor (conventional defaults when the
        driver declares none)."""
        prof = getattr(cls, 'I2C_PROFILE', None)
        if not isinstance(prof, I2cProfile):
            prof = I2cProfile()
        if prof.pec == 'crc8':
            raise CompileError(
                "I2cProfile(pec='crc8'): SMBus PEC is not yet applied by "
                "firmware; a part needing it would run with no error "
                "checking, so it is rejected rather than silently dropped")
        if prof.auto_inc == 'none':
            raise CompileError(
                "I2cProfile(auto_inc='none'): parts without auto-increment "
                "are not supported; use 'implicit' or 'msb'")
        max_hz = int(prof.max_hz) if prof.max_hz else 0
        return {'kind': 'i2c', 'max_hz': max_hz,
                'auto_inc': prof.auto_inc, 'pec': prof.pec}

    def _companion_addr(self, dev, lineno=None):
        """Resolve a `dev=` name to its declared companion address.

        Loud on an unknown name: the driver must declare every
        co-resident slave in ``I2C_COMPANIONS`` before targeting it.
        """
        companions = getattr(type(self), 'I2C_COMPANIONS', None) or {}
        spec = companions.get(dev)
        if spec is None:
            where = f" (line {lineno})" if lineno else ""
            raise CompileError(
                f"dev={dev!r} names no declared companion{where}; "
                f"I2C_COMPANIONS declares {sorted(companions) or 'none'}.")
        return int(spec['addr'])

    def _validate_companions(self):
        """Structural checks on ``I2C_COMPANIONS`` — a driver reaching a
        second I2C slave must be I2C-only (an SPI bind has no address to
        retarget), and each companion needs its own identity anchor or a
        documented skip, mirroring the primary's contract."""
        companions = getattr(type(self), 'I2C_COMPANIONS', None) or {}
        if not companions:
            return
        name = type(self).__name__
        if tuple(getattr(self, 'BUSES', ())) != ('i2c',):
            raise CompileError(
                f"{name}: I2C_COMPANIONS requires BUSES = ('i2c',) — a "
                f"non-I2C bind cannot retarget a slave address.")
        primaries = set(getattr(type(self), 'I2C_ADDRS', None) or [])
        for dev, spec in companions.items():
            if not isinstance(dev, str) or not dev.isidentifier():
                raise CompileError(
                    f"{name}: companion name {dev!r} must be an identifier.")
            if not isinstance(spec, dict):
                raise CompileError(
                    f"{name}: I2C_COMPANIONS[{dev!r}] must be a dict with "
                    f"'addr' and identity fields.")
            addr = spec.get('addr')
            if not isinstance(addr, int) or not 1 <= addr <= 0x7F:
                raise CompileError(
                    f"{name}: companion {dev!r} addr must be a 7-bit I2C "
                    f"address (1..0x7F), got {addr!r}.")
            if addr in primaries:
                raise CompileError(
                    f"{name}: companion {dev!r} addr 0x{addr:02X} is also a "
                    f"primary strap candidate in I2C_ADDRS — a companion is "
                    f"a different die, not a strap alternative.")
            values = list(spec.get('who_am_i_values') or [])
            if values:
                wai_reg = spec.get('who_am_i_reg')
                if not isinstance(wai_reg, int) or not 0 <= wai_reg <= 0xFF:
                    raise CompileError(
                        f"{name}: companion {dev!r} who_am_i_reg must be "
                        f"0..0xFF, got {wai_reg!r}.")
                for v in values:
                    if not isinstance(v, int) or not 0 <= v <= 0xFF:
                        raise CompileError(
                            f"{name}: companion {dev!r} who_am_i value {v!r} "
                            f"doesn't fit in a byte (OP_REG_READ is 8-bit).")
            elif not spec.get('who_am_i_skip_reason'):
                raise CompileError(
                    f"{name}: companion {dev!r} declares no who_am_i_values "
                    f"and no who_am_i_skip_reason — same audit contract as "
                    f"the primary's WHO_AM_I_SKIP_REASON.")

    def _emit_who_am_i_prologue(self):
        """Auto-emit runtime identity checks from class attributes.

        Reads ``WHO_AM_I_REG`` and OP_ERRORs with code
        ``WHO_AM_I_MISMATCH_CODE`` if the value isn't in ``WHO_AM_I_VALUES``.
        Skipped for stream drivers (UART has no WHO_AM_I concept) and for
        register drivers that opted out via ``WHO_AM_I_SKIP_REASON``.
        Each ``I2C_COMPANIONS`` entry with an identity anchor then gets
        its own bracketed check (retarget, read, compare, restore) that
        fails with ``COMPANION_MISMATCH_CODE`` — the primary opt-out does
        not skip companion checks.

        A plain register part reads through the 8-bit OP_REG_READ; a FRAME part
        reads through its wire frame (``_emit_frame_read``), so a CRC-wrapped
        SPI part gets the same on-device identity check as an I2C part.
        """
        if self.BUS_KIND != BUS_REGISTER:
            return
        values = list(getattr(type(self), 'WHO_AM_I_VALUES', None) or [])
        if values:
            self._emit_primary_who_am_i(values)
        self._emit_companion_checks()

    def _emit_primary_who_am_i(self, values):
        reg = int(getattr(type(self), 'WHO_AM_I_REG', 0) or 0)
        is_frame = getattr(self, 'FRAME', None) is not None

        SCRATCH_READ = 6
        SCRATCH_MATCH = 7
        em = self._emitter

        if is_frame:
            # The value is compared against the frame's data field; it must fit
            # that field's width.
            width = self.FRAME.data_byte_width()
            limit = (1 << (8 * width)) - 1
            for v in values:
                if not isinstance(v, int) or v < 0 or v > limit:
                    raise CompileError(
                        f"{type(self).__name__}: WHO_AM_I_VALUES entry {v!r} "
                        f"doesn't fit the {width}-byte frame data field.")
            self._emit_frame_read(reg, SCRATCH_READ)
        else:
            # OP_REG_READ is an 8-bit read; values must fit a byte.
            for v in values:
                if not isinstance(v, int) or v < 0 or v > 0xFF:
                    raise CompileError(
                        f"{type(self).__name__}: WHO_AM_I_VALUES entry {v!r} "
                        f"doesn't fit in a byte (OP_REG_READ is 8-bit).")
            em.emit_reg(Op.REG_READ, reg, SCRATCH_READ)

        pass_label = "__whoami_ok"
        for v in values:
            em.emit_cmp(Op.CMP_EQ, SCRATCH_READ, v, SCRATCH_MATCH)
            em.emit_jmp(Op.JNZ, pass_label, SCRATCH_MATCH)
        em.emit(Op.ERROR, WHO_AM_I_MISMATCH_CODE)
        em.label(pass_label)

    def _emit_companion_checks(self):
        """Bracketed identity check per companion die: retarget, read its
        WHO_AM_I, compare, restore the primary. The ERROR path needs no
        restore — the VM halts there, and the next RUN re-binds home."""
        companions = getattr(type(self), 'I2C_COMPANIONS', None) or {}
        if not companions:
            return
        SCRATCH_READ = 6
        SCRATCH_MATCH = 7
        em = self._emitter
        for dev in sorted(companions):
            spec = companions[dev]
            values = list(spec.get('who_am_i_values') or [])
            if not values:
                continue  # documented opt-out (who_am_i_skip_reason)
            em.emit(Op.I2C_TARGET, int(spec['addr']))
            em.emit_reg(Op.REG_READ, int(spec['who_am_i_reg']), SCRATCH_READ)
            pass_label = f"__whoami_ok_{dev}"
            for v in values:
                em.emit_cmp(Op.CMP_EQ, SCRATCH_READ, v, SCRATCH_MATCH)
                em.emit_jmp(Op.JNZ, pass_label, SCRATCH_MATCH)
            em.emit(Op.ERROR, COMPANION_MISMATCH_CODE)
            em.label(pass_label)
            em.emit(Op.I2C_TARGET, 0)

    def _finalize_patches(self, config: dict) -> List[PatchEntry]:
        """Build complete value maps by compiling with each param value."""
        if not self._patch_entries:
            return []

        # Group patch sites per param. A param may own several sites (a
        # write_modify field split across two registers); each distinct
        # bytecode offset is one site. A direct REG_WRITE clobbers, so two
        # params can't share one register (each value_map would bake the
        # other's compile-time bits) — but RMW sites pass reg=None and skip
        # that check, since read-modify-write preserves each other's bits.
        sites_by_param: Dict[str, List[PatchEntry]] = {}
        reg_owner: Dict[int, str] = {}
        for entry in self._patch_entries:
            lst = sites_by_param.setdefault(entry.param_name, [])
            if any(s.offset == entry.offset for s in lst):
                continue  # same site re-recorded
            if entry.reg is not None:
                owner = reg_owner.get(entry.reg)
                if owner is not None and owner != entry.param_name:
                    raise CompileError(
                        f"params {owner!r} and {entry.param_name!r} both patch "
                        f"register 0x{entry.reg:02X}; a shared register can't be "
                        f"driven by two params — compose it from one param.")
                reg_owner[entry.reg] = entry.param_name
            lst.append(PatchEntry(
                offset=entry.offset,
                param_name=entry.param_name,
                value_map=dict(entry.value_map),  # keep a pre-filled map (poll rate)
                reg=entry.reg,
                size=entry.size,
            ))

        for param_name, lst in sites_by_param.items():
            if len(lst) > MAX_PATCH_SITES:
                raise CompileError(
                    f"param {param_name!r} patches {len(lst)} bytecode sites; "
                    f"the image carries at most {MAX_PATCH_SITES} per param.")

        # For each declared param, compile with each valid value to get the
        # patch byte at every one of the param's sites.
        for param_name, param in self._params.items():
            if param_name not in sites_by_param:
                continue
            if param.param_type != "enum":
                continue

            sites = sites_by_param[param_name]
            if all(s.value_map for s in sites):
                continue  # already built directly (poll-rate SLEEP_MS patch)
            # Sites in emission order (by their main-trace offset). The
            # re-trace runs from a fresh emitter (no WHO_AM_I prologue), so its
            # offsets differ in absolute value but preserve this order — map
            # re-trace records to sites by rank, not absolute offset.
            ordered_sites = sorted(sites, key=lambda s: s.offset)
            ref_offsets = None
            for val in param.values:
                alt_config = dict(config)
                alt_config[param_name] = val
                # Mini-compile: re-trace configure() with this value,
                # using a fresh emitter and a recorder that only keeps
                # patches for *this* parameter.
                alt_emitter = _Emitter()
                alt_regs = _RegAlloc()
                alt_patches: List[tuple] = []
                alt_records_target = param_name

                def _alt_recorder(pn, cv, bv, off, size=1, reg=None,
                                  _target=alt_records_target):
                    if pn == _target:
                        alt_patches.append((cv, bv, off))

                # Swap emitter/regs and the patch recorder pointer.
                orig_em = self._emitter
                orig_regs_obj = self._regs
                orig_params = self._params
                orig_accum = self._patch_accum
                orig_fields = self._output_fields
                orig_recorder = self._patch_recorder_fn
                orig_read_responses = self._read_responses

                self._emitter = alt_emitter
                self._regs = alt_regs
                self._params = {}
                self._patch_accum = {}
                self._output_fields = []
                self._patch_recorder_fn = _alt_recorder
                # Refill read-response queue from the driver's template
                # so probe() reads the declared WHO_AM_I etc. again.
                self._read_responses = {
                    reg: list(vals)
                    for reg, vals in self._read_responses_template.items()
                }

                if hasattr(self, "probe"):
                    self._trace_phase = "probe"
                    self.probe()
                if hasattr(self, "configure"):
                    self._trace_phase = "configure"
                    self.configure(alt_config)
                self._trace_phase = None

                # Restore
                self._emitter = orig_em
                self._regs = orig_regs_obj
                self._params = orig_params
                self._patch_accum = orig_accum
                self._output_fields = orig_fields
                self._patch_recorder_fn = orig_recorder
                self._read_responses = orig_read_responses

                # Records for this value, in emission order (re-trace offset).
                recs = sorted(((bv, off) for cv, bv, off in alt_patches
                               if cv == val), key=lambda r: r[1])
                # One record per site — a mismatch means the tagged write is
                # conditional on the value (the image writer would default the
                # missing byte to 0 and `set` would patch the wrong bytes).
                if len(recs) != len(ordered_sites):
                    raise CompileError(
                        f"param {param_name!r} value {val!r} produces "
                        f"{len(recs)} patch record(s) but the param has "
                        f"{len(ordered_sites)} site(s); tag a write that runs "
                        f"for every value.")
                # The re-trace offsets must be identical across values — a
                # moving shape means configure() emits value-dependent bytecode
                # the fixed sites can't track.
                these_offsets = tuple(off for _bv, off in recs)
                if ref_offsets is None:
                    ref_offsets = these_offsets
                elif these_offsets != ref_offsets:
                    raise CompileError(
                        f"param {param_name!r} patch sites move between values "
                        f"({list(ref_offsets)} vs {list(these_offsets)}); "
                        f"configure() must emit the same bytecode shape for "
                        f"every value of a patched param.")
                for site, (bv, _off) in zip(ordered_sites, recs):
                    site.value_map[val] = bv

        # Flatten: sites ordered by offset within each param (matches the
        # deterministic wire order in image._write_param).
        result: List[PatchEntry] = []
        for lst in sites_by_param.values():
            result.extend(sorted(lst, key=lambda s: s.offset))
        return result


class RegisterDriver(SensorDriver):
    """Base class for sensors addressed on a register bus (I²C, SPI).

    Default path (``FRAME`` is ``None``): ``self.write``/``self.read``
    emit the conventional ``OP_REG_WRITE`` / ``OP_REG_READ`` opcodes.
    The VM's bound ``hal::RegDeviceI`` chooses the wire format (I²C or
    plain SPI with MSB-set = read).

    Custom-framing path: subclass declares a ``FRAME = SpiFrame(...)``
    schema (see ``nxs.framing``). Reads and writes then emit
    ``OP_MEMCPY_IMM`` + ``OP_REG_XFER`` sequences carrying the exact
    on-wire bytes — CRC, reserved bits, read-pipeline dummies. The VM
    stays bus-agnostic; the wire framing lives entirely in the NXS
    image.
    """

    BUS_KIND = BUS_REGISTER

    # Physical buses the chip supports. Chip-level fact, not board-
    # level — the driver describes what the silicon can do. Defaults
    # to both I²C and SPI because most register-addressed sensor ICs
    # (BMI270, LSM6, BMP series, IAM-20680 family …) expose both
    # interfaces via strap pins. Narrow this to a single-element
    # tuple for chips that physically support only one:
    #   BUSES = ('spi',)   # IIM-20670 and other SPI-only industrial IMUs
    #   BUSES = ('i2c',)   # rare; most pure-I²C parts live in StreamDriver-
    #                      # or I2cCommandDriver-land instead
    # The first entry is the default when the user doesn't pass
    # `bus=...` at upload time.
    BUSES = ('i2c', 'spi')

    # Override in a subclass to declare non-standard SPI framing.
    FRAME = None

    # Co-resident I2C slaves on the same bus (multi-die packages): name →
    # {'addr': 0x0C, 'who_am_i_reg': 0x0F, 'who_am_i_values': [0x49]}.
    # The primary keeps the I2C_ADDRS strap scan; companions are fixed
    # addresses reached per access via the dev= kwarg on read/write/
    # read_burst/write_burst — each access is bracketed (target, access,
    # restore primary), so the bus always rests at the primary. Each
    # companion carries its own identity anchor, checked in the probe
    # prologue, or a who_am_i_skip_reason. Requires BUSES = ('i2c',).
    I2C_COMPANIONS = {}

    def write(self, reg: int, val: int, param: Optional[tuple] = None,
              dev=None):
        if self.FRAME is None:
            # Simple path: REG_WRITE is 4 bytes [opcode, reg_lo, reg_hi,
            # val]; the value byte is at offset +3, patchable byte-by-byte.
            # A dev= write brackets itself with the companion retarget; the
            # patch offset is computed after the bracket opens, so patching
            # is unaffected, and the owner key carries the device so two
            # dies' same-numbered registers don't false-collide.
            addr = self._companion_addr(dev) if dev is not None else None
            if addr is not None:
                self._emitter.emit(Op.I2C_TARGET, addr)
            if param is not None:
                offset = self._emitter._current_offset() + 3
                self._patch_recorder_fn(param[0], param[1], val, offset,
                                        reg=(dev, reg) if dev else reg)
            self._emitter.emit_reg(Op.REG_WRITE, reg, val)
            if addr is not None:
                self._emitter.emit(Op.I2C_TARGET, 0)
            return
        if dev is not None:
            raise CompileError(
                "dev= targets an I2C companion; a FRAME driver is SPI and "
                "declares no companions.")

        # FRAME path: build the full wire bytes (rw=1, addr, data, CRC)
        # at trace time, emit MEMCPY_IMM to stage them in sample_buf,
        # then REG_XFER to clock them out.
        frame_bytes = self.FRAME.compose(rw=1, addr=reg, data=val)
        # The frame's first byte lands at sample_buf[0]. Offset of that
        # byte inside the bytecode = current offset + 3 (MEMCPY_IMM header).
        frame_byte_off = self._emitter._current_offset() + 3
        self._emitter.emit(Op.MEMCPY_IMM, 0, len(frame_bytes), *frame_bytes)
        self._emitter.emit(Op.REG_XFER, 0, 0, len(frame_bytes))
        if param is not None:
            # Patch the entire wire frame as a single unit (CRC and all).
            # Stored as a little-endian integer so patch_driver_param's
            # "bytecode[0]=LSB, bytecode[3]=MSB" write pattern lays the
            # bytes back down in the original MSB-first wire order.
            frame_u32 = int.from_bytes(frame_bytes, "little")
            self._patch_recorder_fn(
                param[0], param[1], frame_u32, frame_byte_off, size=4, reg=reg)

    def write_modify(self, reg: int, set_bits: int = 0, clear_bits: int = 0,
                     param: Optional[tuple] = None):
        """On-device read-modify-write of a FRAME register: read `reg`,
        AND out `clear_bits`, OR in `set_bits`, write the result back
        with the frame CRC recomputed on-device.

        The pattern for registers whose reserved bits carry undocumented
        factory state (the datasheet's "read the whole register first,
        change the desired bits only"): the modified value exists only
        in the device's registers at load time, so no unit's factory
        bits ever get baked into the image. Emits a bit-serial RISC CRC
        loop — a once-at-load write does not earn an opcode, and it
        covers CRC feedback variants no opcode implements.

        `param=(name, value)` makes the field runtime-tunable: the OR
        `set_bits` immediate becomes a patch site (`nxs set` rewrites it and
        reloads, re-running the RMW with the new field code; reserved bits are
        still read fresh each load). Pass `clear_bits` as the *whole* field
        mask (constant across values) so only the OR immediate varies — a
        single stable site per register. A field split across two registers
        tags each `write_modify`; both are sites of the one param."""
        if self.FRAME is None:
            raise CompileError(
                f"{type(self).__name__}: write_modify requires a FRAME "
                f"schema — a plain REG_WRITE carries an immediate value "
                f"and cannot write a runtime-computed one")
        if self._trace_phase not in ("probe", "configure"):
            raise CompileError(
                "write_modify is a configure()/probe() verb; in measure() "
                "its staging would overwrite the sample region")
        if not 0 <= set_bits <= 0xFFFF or not 0 <= clear_bits <= 0xFFFF:
            raise CompileError(
                f"write_modify: set_bits/clear_bits must fit the 16-bit "
                f"data field, got set=0x{set_bits:X} clear=0x{clear_bits:X}")
        if set_bits == 0 and clear_bits == 0:
            raise CompileError(
                "write_modify with neither set_bits nor clear_bits is a "
                "no-op; drop the call")
        # AND ~clear, then OR set: a bit in both is cleared then set, so
        # overlap is well-defined and intended for a param= write (clear the
        # WHOLE field, OR the value's code). Without param=, disjoint masks are
        # the convention, so overlap there flags a driver mistake.
        if param is None and (set_bits & clear_bits):
            raise CompileError(
                f"write_modify: set_bits and clear_bits overlap on "
                f"0x{set_bits & clear_bits:04X}; a bit cannot be both")
        # A param= write patches only the OR set_bits, so clear_bits must be
        # the whole field (a superset of every value's set) — else a set bit
        # outside clear would stick across a range change.
        if param is not None and (set_bits & ~clear_bits):
            raise CompileError(
                f"write_modify(param={param[0]!r}): set_bits 0x{set_bits:X} has "
                f"bits outside clear_bits 0x{clear_bits:X}; pass clear_bits as "
                f"the whole field mask so the field fully clears before the set.")

        frame = self.FRAME
        crc = frame.crc
        if (crc is None or crc.width != 8 or crc.compute_fn is not None
                or frame.fields[-1].name != 'crc'
                or tuple(crc.covers) != tuple(
                    f.name for f in frame.fields[:-1])):
            raise CompileError(
                f"{type(self).__name__}: write_modify supports frames "
                f"whose trailing 8-bit CRC covers all preceding fields "
                f"in order; this FRAME does not")
        if crc.feedback_style not in ("standard", "input-lsb"):
            raise CompileError(
                f"write_modify: unknown CRC feedback_style "
                f"{crc.feedback_style!r}")
        if frame.data_byte_width() != 2:
            raise CompileError(
                f"write_modify: FRAME data_byte_width="
                f"{frame.data_byte_width()} not supported (only 16-bit)")

        em = self._emitter
        fw = frame.byte_width
        doff = frame.data_byte_offset()
        # Constant skeleton: header bits composed at trace time, data
        # zeroed; the data bytes and the CRC byte are filled on-device.
        skeleton = bytearray(frame.compose(rw=1, addr=reg, data=0))
        skeleton[fw - 1] = 0

        with self._regs.scope():
            val = self._regs.get("__wm_val")
            tmp = self._regs.get("__wm_tmp")
            self._emit_frame_read(reg, val)
            if param is not None:
                # Runtime-tunable: emit a fixed AND (constant full-field clear)
                # then OR (the value's field code), always both, so the shape —
                # and thus the OR immediate's offset — is identical for every
                # value. The OR immediate (4-byte L in emit_cmp's <BBLB, at
                # byte +2) is the patch site. reg is not registered as an owner:
                # RMW writes preserve each other's bits, so two params sharing a
                # register don't clobber (unlike a direct REG_WRITE), and banked
                # parts reuse a register number across banks.
                em.emit_cmp(Op.AND, val, (~clear_bits) & 0xFFFF, val)
                or_offset = em._current_offset() + 2
                self._patch_recorder_fn(param[0], param[1], set_bits, or_offset,
                                        size=4)
                em.emit_cmp(Op.OR, val, set_bits, val)
            else:
                if clear_bits:
                    em.emit_cmp(Op.AND, val, (~clear_bits) & 0xFFFF, val)
                if set_bits:
                    em.emit_cmp(Op.OR, val, set_bits, val)
            em.emit(Op.MEMCPY_IMM, 0, fw, *skeleton)
            em.emit(Op.SHR, val, 8, tmp)
            em.emit(Op.STORE_U8, doff, tmp)
            em.emit(Op.STORE_U8, doff + 1, val)
            # CRC over the staged bytes [0, fw-1); `val` is dead after
            # the stores and doubles as the CRC accumulator.
            self._emit_crc_bitserial(0, fw - 1, val, tmp)
            em.emit(Op.STORE_U8, fw - 1, val)
            em.emit(Op.REG_XFER, 0, 0, fw)

    def _emit_crc_bitserial(self, start: int, length: int,
                            crc_reg: int, tmp: int):
        """Emit a runtime bit-serial CRC over sample_buf[start ..
        start+length) into r[crc_reg], following FRAME.crc's init /
        poly / xor_out / feedback_style. MSB-first per byte, mirroring
        `Crc.compute`."""
        crc = self.FRAME.crc
        em = self._emitter
        mask = (1 << crc.width) - 1
        uid = em._current_offset()
        l_byte = f"__wm_crc_byte_{uid}"
        l_bit = f"__wm_crc_bit_{uid}"
        l_nopoly = f"__wm_crc_nopoly_{uid}"

        with self._regs.scope():
            cursor = self._regs.get("__wm_crc_cursor")
            work = self._regs.get("__wm_crc_work")
            bits = self._regs.get("__wm_crc_bits")

            em.emit_u32(Op.LOAD_IMM, crc_reg, crc.init)
            em.emit_u32(Op.LOAD_IMM, cursor, start)
            em.label(l_byte)
            em.emit(Op.LOAD_U8_REG, work, cursor)
            em.emit_u32(Op.LOAD_IMM, bits, 8)
            em.label(l_bit)
            if crc.feedback_style == "standard":
                # feedback = MSB(crc) XOR MSB(work), tested after shift.
                em.emit(Op.SHR, crc_reg, crc.width - 1, tmp)
                with self._regs.scope():
                    t2 = self._regs.get("__wm_crc_t2")
                    em.emit(Op.SHR, work, 7, t2)
                    em.emit(Op.XOR_REG, tmp, t2, tmp)
            else:
                # input-lsb: feedback = MSB(crc) only.
                em.emit(Op.SHR, crc_reg, crc.width - 1, tmp)
            em.emit(Op.SHL, crc_reg, 1, crc_reg)
            em.emit_cmp(Op.AND, crc_reg, mask, crc_reg)
            em.emit_jmp(Op.JZ, l_nopoly, tmp)
            em.emit_cmp(Op.XOR, crc_reg, crc.poly, crc_reg)
            em.label(l_nopoly)
            if crc.feedback_style == "input-lsb":
                # ... then the input bit lands in the LSB.
                em.emit(Op.SHR, work, 7, tmp)
                em.emit(Op.XOR_REG, crc_reg, tmp, crc_reg)
            em.emit(Op.SHL, work, 1, work)
            em.emit_cmp(Op.AND, work, 0xFF, work)
            em.emit_cmp(Op.SUB, bits, 1, bits)
            em.emit_jmp(Op.JNZ, l_bit, bits)
            em.emit_cmp(Op.ADD, cursor, 1, cursor)
            em.emit_cmp(Op.CMP_EQ, cursor, start + length, tmp)
            em.emit_jmp(Op.JZ, l_byte, tmp)
            if crc.xor_out:
                em.emit_cmp(Op.XOR, crc_reg, crc.xor_out, crc_reg)

    def _emit_frame_read(self, reg, dst, signed=False, endian="big"):
        """Emit a custom-FRAME register read into `dst`: stage the request
        frame, XFER it (plus read-pipeline dummies), then load the data field.

        Shared by the trace path (`read`) and the measure-body AST compiler,
        so a FRAME part's `self.read(reg)` in `measure()` clocks the real wire
        frame instead of an unframed `REG_READ`. Default (unsigned big-endian)
        keeps the `LOAD_U16_BE` emission byte-identical; `signed`/`endian`
        switch to the general `LOAD` spec."""
        req = self.FRAME.compose(rw=0, addr=reg, data=0x0000)
        dummy = self.FRAME.compose(rw=0, addr=0, data=0x0000)
        fw = len(req)
        pipeline = max(0, int(self.FRAME.read_pipeline))

        # Stage + issue the initial request (RX into buf[0..fw)).
        self._emitter.emit(Op.MEMCPY_IMM, 0, fw, *req)
        self._emitter.emit(Op.REG_XFER, 0, 0, fw)
        # Pipeline dummies (each one clocks the next response out). No
        # inter-frame gap here: single-register reads target always-ready
        # constants (FIXED_VALUE/WHO_AM_I on MISO immediately); the response-
        # staging gap that sampled data registers need lives in the measure-
        # loop burst path, gated by the FRAME's inter_frame_sleep_ms.
        for _ in range(pipeline):
            self._emitter.emit(Op.MEMCPY_IMM, 0, fw, *dummy)
            self._emitter.emit(Op.REG_XFER, 0, 0, fw)

        # The data field occupies `data_byte_width` bytes starting at
        # `data_byte_offset` within a received frame. Extract into `dst`.
        if self.FRAME.data_byte_width() != 2:
            raise CompileError(
                f"FRAME data_byte_width={self.FRAME.data_byte_width()} "
                f"not yet supported (only 2 bytes / 16-bit)")
        data_off = self.FRAME.data_byte_offset()
        if not signed and endian == "big":
            self._emitter.emit(Op.LOAD_U16_BE, dst, data_off)
        else:
            self._emitter.emit(Op.LOAD, dst, data_off,
                               _load_spec_byte(2, signed, endian))

    def read(self, reg, width=1, signed=False, endian="big", dev=None):
        # Simple single-byte reads and multi-byte coefficient reads (no custom
        # framing) are the base SensorDriver.read; RegisterDriver adds only the
        # custom-FRAME path below.
        if self.FRAME is None:
            return super().read(reg, width, signed, endian, dev=dev)
        if dev is not None:
            raise CompileError(
                "dev= targets an I2C companion; a FRAME driver is SPI and "
                "declares no companions.")

        # FRAME path with optional pipelining (read_pipeline=N means the
        # response to request K arrives during request K+N). For a
        # two-frame-handshake part (N=1): the request frame, then a dummy
        # that returns `reg`'s data.
        self._emit_frame_read(reg, self._regs.get(f"__reg_read_{reg}"),
                              signed, endian)

        responses = self._read_responses.get(reg)
        val = responses.pop(0) if responses else 0
        if self._trace_phase == "configure":
            return _TraceReadValue(reg)
        return val

    def xfer(self, word: int, width: int = 2):
        """Clock a literal full-duplex SPI word; the response replaces it
        in the staging slot.

        The escape hatch for parts whose wire words carry computed bits a
        FRAME schema cannot express (parity, embedded rw flags): the driver
        computes `word` in plain Python at trace time and xfer moves it on
        the wire verbatim, one CS assertion per word. In assignment position
        inside measure() the response loads as an unsigned MSB-first value
        of the full `width`; in statement position (pipeline priming) it is
        discarded."""
        self._emit_xfer(word, width)
        return 0

    def _emit_xfer(self, word: int, width: int) -> int:
        """Stage `width` literal bytes (MSB-first) at the scalar-scratch
        tail and REG_XFER them; the response lands in the same slot.
        Returns the slot offset for the caller's response LOAD."""
        if tuple(self.BUSES) != ('spi',):
            raise CompileError(
                f"xfer clocks a literal full-duplex SPI word, so the driver "
                f"must declare BUSES = ('spi',) — {type(self).__name__} "
                f"declares {self.BUSES}, and a non-SPI binding has no "
                f"defined wire behaviour for it")
        if not 1 <= width <= 4:
            raise CompileError(
                f"xfer(word, width) supports width 1..4, got {width}")
        if not 0 <= word < (1 << (8 * width)):
            raise CompileError(
                f"xfer word 0x{word:X} does not fit in {width} byte(s)")
        slot = _ASTCompiler.SCALAR_SCRATCH_OFF
        if self._sample_size > slot:
            raise CompileError(
                f"xfer stages through sample_buf[{slot}.."
                f"{_ASTCompiler.VM_SAMPLE_BUF_SIZE}), but sample_size="
                f"{self._sample_size} extends into that scratch slot; keep "
                f"sample_size <= {slot}.")
        tx = word.to_bytes(width, "big")
        self._emitter.emit(Op.MEMCPY_IMM, slot, width, *tx)
        self._emitter.emit(Op.REG_XFER, slot, slot, width)
        return slot

    def read_burst(self, reg: int, count: int, dev=None) -> bytes:
        if self.FRAME is not None:
            raise CompileError(
                "read_burst with FRAME schema not yet implemented — "
                "use individual self.read(...) calls for now")
        addr = self._companion_addr(dev) if dev is not None else None
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, addr)
        self._emitter.emit_reg(Op.REG_READ_BURST, reg, count, 0)
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, 0)
        return bytes(count)

    def write_burst(self, reg: int, data, dev=None) -> None:
        """Burst-write `data` to consecutive registers starting at `reg`.

        Stages the payload into sample_buf, then `OP_REG_WRITE_BURST` drives
        the HAL `write_regs` — the register-framed multi-byte write behind
        RTC set-time, EEPROM stores, and IMU config blocks. `data` is a
        bytes/list of byte values; 16-bit-address parts use raw writes."""
        if self.FRAME is not None:
            raise CompileError(
                "write_burst with a FRAME schema is not supported — stage "
                "the framed bytes and use self.write(...) per register")
        payload = bytes(data)
        if len(payload) > _ASTCompiler.VM_SAMPLE_BUF_SIZE:
            raise CompileError(
                f"write_burst payload {len(payload)} B exceeds the "
                f"{_ASTCompiler.VM_SAMPLE_BUF_SIZE}-byte sample buffer — "
                f"split into chunks")
        addr = self._companion_addr(dev) if dev is not None else None
        self._emitter.emit(Op.MEMCPY_IMM, 0, len(payload), *payload)
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, addr)
        self._emitter.emit_reg(Op.REG_WRITE_BURST, reg, 0, len(payload))
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, 0)


class I2cCommandDriver(SensorDriver):
    """Base class for command-response I²C sensors (Sensirion, Melexis).

    Unlike ``RegisterDriver`` (which addresses N registers via
    (reg, val) pairs), this covers the pattern:

        host sends an N-byte command →
        waits a datasheet-prescribed delay →
        reads M chunks of (data_bytes, crc_bytes) →
        verifies each CRC, skips the sample if any fails,
        packs the data contiguously and stores.

    The measure loop is generated procedurally — not AST-compiled —
    because the pattern is fixed. Subclasses declare:

        FRAME: an ``I2cFrame`` describing chunk layout + CRC.
        MEASURE_COMMAND: bytes of the measurement command (1 or 2 bytes).
        MEASURE_DELAY_MS: post-command delay before read (datasheet).
        MEASURE_NUM_WORDS: how many (data, crc) chunks the response has.

    ``configure(config)`` sets up ``set_output`` and ``set_sample_size``
    as usual; ``compile`` auto-emits the measure loop afterwards.
    """

    BUS_KIND = BUS_REGISTER

    # I2cCommandDriver uses `write_raw`/`read_raw`, which have no
    # meaningful SPI equivalent (the protocol shape is 'command, STOP,
    # wait, read' — full-duplex SPI doesn't express that idiomatically).
    # So I²C is the only legal transport; subclasses don't get to
    # override this.
    BUSES = ('i2c',)

    # Subclass overrides
    FRAME = None
    MEASURE_COMMAND = (0xFD,)
    MEASURE_DELAY_MS = 10
    MEASURE_NUM_WORDS = 1

    def send_command(self, cmd_bytes):
        """Emit MEMCPY_IMM + BUS_WRITE_RAW for a raw I²C command — a single
        command byte (`send_command(0x1E)`) or a bytes-like sequence."""
        cmd = bytes([cmd_bytes]) if isinstance(cmd_bytes, int) else bytes(cmd_bytes)
        self._emitter.emit(Op.MEMCPY_IMM, 0, len(cmd), *cmd)
        self._emitter.emit(Op.BUS_WRITE_RAW, 0, len(cmd))

    def _emit_measure_loop(self):
        """Procedural code-gen for the fixed Sensirion-style pattern."""
        if self.FRAME is None or self.FRAME.crc is None:
            raise CompileError(
                f"{type(self).__name__}: FRAME with CRC must be declared")
        if self.FRAME.crc.feedback_style != "standard":
            raise CompileError(
                f"{type(self).__name__}: I2cCommandDriver CRC verification "
                f"relies on the standard feedback style (CRC of msg+crc=0 "
                f"property). Got feedback_style="
                f"{self.FRAME.crc.feedback_style!r}.")
        if self.FRAME.crc.xor_out != 0:
            raise CompileError(
                f"{type(self).__name__}: CRC verify-via-zero requires "
                f"xor_out=0. Got xor_out=0x{self.FRAME.crc.xor_out:02X}.")

        em = self._emitter
        frame = self.FRAME
        num_words = int(self.MEASURE_NUM_WORDS)
        chunk = frame.chunk_bytes

        # Enforce sample_buf limits.
        total = frame.total_bytes(num_words)
        if total > _ASTCompiler.VM_SAMPLE_BUF_SIZE:
            raise CompileError(
                f"{type(self).__name__}: response {total} B exceeds "
                f"the {_ASTCompiler.VM_SAMPLE_BUF_SIZE}-byte sample buffer")

        # Pick the loop tick interval from the sample_rate config.
        sample_rate = max(1, int(self._config.get("sample_rate", 1)))
        interval_ms = max(1, 1000 // sample_rate)

        em.label("__measure_loop")
        sleep_off = em._current_offset() + 1
        em.emit_u16(Op.SLEEP_MS, interval_ms)
        self._patch_poll_rate(sleep_off)

        # Send command bytes.
        self.send_command(self.MEASURE_COMMAND)

        # Wait for the measurement to complete.
        em.emit_u16(Op.SLEEP_MS, int(self.MEASURE_DELAY_MS))

        # Read the response into buf[0..total).
        em.emit(Op.BUS_READ_RAW, 0, total)

        # Per-word CRC verification via the "CRC(data||crc)==0"
        # property (standard CRC with xor_out=0). If any word fails,
        # JZ back to the loop top — no STORE_SAMPLE for this cycle.
        poly = frame.crc.poly
        init = frame.crc.init
        xor_out = frame.crc.xor_out
        SCRATCH_RESULT = 7
        SCRATCH_VALID = 6
        for k in range(num_words):
            off = k * chunk
            em.emit(Op.CRC8, off, chunk, poly, init, xor_out,
                    SCRATCH_RESULT, 0)
            em.emit_cmp(Op.CMP_EQ, SCRATCH_RESULT, 0, SCRATCH_VALID)
            em.emit_jmp(Op.JZ, "__measure_loop", SCRATCH_VALID)

        # All CRCs passed — pack data bytes contiguously. Word 0's
        # data is already at buf[0..data_bytes); each subsequent word
        # needs to be moved from buf[k*chunk..) to buf[k*data_bytes..).
        for k in range(1, num_words):
            dst = k * frame.data_bytes
            src = k * chunk
            em.emit(Op.MEMCPY, dst, src, frame.data_bytes)

        em.emit(Op.STORE_SAMPLE)
        em.emit_jmp(Op.JMP, "__measure_loop")


class StreamDriver(SensorDriver):
    """Base class for sensors on a byte-stream bus (UART)."""

    BUS_KIND = BUS_STREAM

    def set_baud(self, baud: int = 38400, param: Optional[tuple] = None):
        # UART_CONFIGURE is 5 bytes: [opcode, b0..b3]. Value at offset +1.
        if param is not None:
            offset = self._emitter._current_offset() + 1
            self._patch_recorder_fn(param[0], param[1], baud, offset, size=4)
        self._emitter.emit(Op.UART_CONFIGURE,
                           baud & 0xFF,
                           (baud >> 8) & 0xFF,
                           (baud >> 16) & 0xFF,
                           (baud >> 24) & 0xFF)

    def write(self, val):
        """Write to UART TX.

        - `write(int)` clocks a single byte via OP_UART_WRITE.
        - `write(bytes | bytearray)` stages the buffer at sample_buf[0]
          via OP_MEMCPY_IMM and clocks it out as one frame via
          OP_UART_WRITE_RAW. Use for multi-byte commands (UBX-CFG-VALSET,
          SBF text commands, AT-command strings).

        Note: the bytes overload clobbers sample_buf[0..len(val)) at
        runtime. Any subsequent read_until / read_n / TracedSlice.expect
        will overwrite it again, so this is only a concern if the driver
        is interleaving writes with reads that need to preserve earlier
        bytes — uncommon.
        """
        if isinstance(val, int):
            if not (0 <= val <= 0xFF):
                raise CompileError(f"write(int): byte must be 0..255, got {val}")
            self._emitter.emit(Op.UART_WRITE, val)
            return
        if isinstance(val, (bytes, bytearray)):
            if len(val) == 0:
                return
            if len(val) > 0xFF:
                raise CompileError(
                    f"write(bytes): max 255 bytes per frame, got {len(val)}. "
                    f"Split into multiple write() calls.")
            self._emitter.emit(Op.MEMCPY_IMM, 0, len(val), *val)
            self._emitter.emit(Op.UART_WRITE_RAW, 0, len(val))
            return
        raise CompileError(
            f"write(): expected int or bytes, got {type(val).__name__}")

    def read(self, count: int) -> bytes:
        self._emitter.emit(Op.UART_READ, count, 0)
        return bytes(count)

    def available(self) -> int:
        dst = self._regs.get("__uart_avail")
        self._emitter.emit(Op.UART_AVAIL, dst)
        return 0

    # ── Staged-frame helpers ─────────────────────────────────
    #
    # When a TX frame contains both param-driven bytes and a
    # checksum that covers them, the driver must:
    #   1. Stage the frame in sample_buf (MEMCPY_IMM 0 len bytes…)
    #   2. Record a patch site at the param-driven byte offset
    #   3. Compute the checksum at runtime over the staged bytes
    #   4. Clock the staged frame out (UART_WRITE_RAW 0 len)
    # The patchable rate stays patchable AND the checksum stays
    # correct after each `set <param>`.
    #
    # `stage()` and `send_staged()` are the public verbs for steps
    # 1+2 and step 4. Compose them with `compute_checksum()` for
    # step 3. Drivers that don't need patchable bytes can use the
    # `write(bytes)` overload, which does stage+send in one call
    # but doesn't expose the staging period for checksum patching.

    def stage(self, frame: bytes,
              patch: Optional[tuple] = None) -> None:
        """Stage `frame` bytes at sample_buf[0..len(frame)] via
        OP_MEMCPY_IMM.

        `patch` (optional): record a patch site for `set <param>`
        in-place updates. Tuple shape:
        `(name, requested_value, encoded_value, frame_offset, size)`
        where `frame_offset` is the byte index inside `frame` of
        the first patchable byte, and `size` is the patch field
        width in bytes (typically 1, 2, or 4). The patch site's
        absolute bytecode offset is computed here (= current
        emitter offset + MEMCPY_IMM header (3 B) + frame_offset).

        Follow with `compute_checksum()` to recompute checksum
        bytes that depend on the patched value, then
        `send_staged(len(frame))` to clock the frame out.
        """
        if not (0 < len(frame) <= 0xFF):
            raise CompileError(
                f"stage: frame length must be 1..255, got {len(frame)}")
        if patch is not None:
            name, requested, encoded, frame_offset, size = patch
            # MEMCPY_IMM header is 3 bytes (opcode + dst_off + len);
            # the first inline data byte lands at current + 3.
            patch_offset = self._emitter._current_offset() + 3 + frame_offset
            self._patch_recorder_fn(name, requested, encoded,
                                    patch_offset, size=size)
        self._emitter.emit(Op.MEMCPY_IMM, 0, len(frame), *frame)

    def send_staged(self, length: int) -> None:
        """Clock `length` bytes from sample_buf[0..length) out via
        OP_UART_WRITE_RAW. Pairs with `stage()` (and an optional
        intervening `compute_checksum()`)."""
        if not (0 < length <= 0xFF):
            raise CompileError(
                f"send_staged: length must be 1..255, got {length}")
        self._emitter.emit(Op.UART_WRITE_RAW, 0, length)

    # ── Framing helpers ──────────────────────────────────────
    #
    # These compile to bytecode loops over the RISC primitives — no
    # protocol knowledge in firmware. NMEA-style drivers call
    # `read_until(0x0A)`; SBF/UBX-style drivers call `read_n(count)`
    # after locating a sync prefix. The compiled loop tracks the byte
    # cursor in a named scratch register and ends with `store_sample_n()`
    # which commits exactly that many bytes per record.

    def _fresh_label(self, base: str) -> str:
        """Generate a unique label name; safe to call multiple times
        per helper invocation."""
        self._label_counter += 1
        return f"__{base}_{self._label_counter}"

    def read_until(self, delim, max: int = _ASTCompiler.VM_SAMPLE_BUF_SIZE,
                   timeout_ms: Optional[int] = None) -> None:
        """Compile: read UART bytes one-by-one into sample_buf until
        the `delim` sequence is seen at the tail (delim bytes included
        in the captured prefix) or the `max` cap is reached. The total
        byte count lands in the `__cursor` scratch register; follow with
        `store_sample_n()` to publish exactly those bytes.

        `delim` may be:
        - an `int` (0..255), e.g. `0x0A` for LF
        - a single-byte `bytes` literal, e.g. `b'\\n'`
        - a multi-byte `bytes` literal, e.g. `b'\\xb5\\x62'` (UBX sync)
          or `b'$@'` (SBF sync)

        Multi-byte delims use a progress-counter scheme: the compiled
        loop tracks how many of the delim bytes have matched at the
        tail of sample_buf. On match-byte received, progress advances.
        On mismatch, progress resets but a possible new-match-on-this-
        byte is also checked (so delims with no self-similar prefix
        like `b'\\xb5\\x62'`, `b'$@'`, `b'OK\\r\\n'` work correctly).
        Self-similar prefixes (full KMP) are not supported — flagged
        at compile time.

        `timeout_ms` (optional): bound the wait via an iteration
        counter on the idle (no-bytes-available) branch. Each idle
        pass does one `SLEEP_MS 1`, so the counter approximates wall
        time within a few percent. On expiry, the VM emits
        `OP_ERROR ERR_CODE_TIMEOUT` and step() returns
        `VmErr::TIMEOUT` (-417). Must be in 1..65535.
        """
        # Normalise: int → bytes of length 1.
        if isinstance(delim, int):
            if not (0 <= delim <= 0xFF):
                raise CompileError(
                    f"read_until: delim int must be 0..255, got {delim!r}")
            delim_bytes = bytes([delim])
        elif isinstance(delim, (bytes, bytearray)):
            delim_bytes = bytes(delim)
        else:
            raise CompileError(
                f"read_until: delim must be int or bytes, got {type(delim).__name__}")

        if len(delim_bytes) == 0:
            raise CompileError("read_until: delim cannot be empty")
        if not (0 < max <= _ASTCompiler.VM_SAMPLE_BUF_SIZE):
            raise CompileError(
                f"read_until: max must be 1..{_ASTCompiler.VM_SAMPLE_BUF_SIZE} "
                f"(the sample buffer), got {max!r}")
        if len(delim_bytes) > max:
            raise CompileError(
                f"read_until: delim length ({len(delim_bytes)}) exceeds max ({max})")

        # Reject delims with self-similar prefixes — they need KMP-style
        # backtracking which we don't emit. Detect: any proper prefix of
        # delim that also matches the corresponding suffix.
        for k in range(1, len(delim_bytes)):
            if delim_bytes[:k] == delim_bytes[-k:]:
                raise CompileError(
                    f"read_until: delim {delim_bytes!r} has a self-similar "
                    f"prefix/suffix of length {k} — needs KMP backtracking "
                    f"not supported by the compiler. Pick a different "
                    f"delimiter or split the read.")

        if timeout_ms is not None and not (0 < timeout_ms <= 0xFFFF):
            raise CompileError(
                f"read_until: timeout_ms must be 1..65535, got {timeout_ms!r}")

        em = self._emitter
        L_TOP = self._fresh_label("ru_top")
        L_IDLE = self._fresh_label("ru_idle")
        L_AFTER = self._fresh_label("ru_after_match")
        L_FOUND = self._fresh_label("ru_done")
        L_TIMEOUT = self._fresh_label("ru_timeout") if timeout_ms is not None else None

        # Scratch scope: every `__*` register allocated below is dead
        # once L_FOUND is reached. The `with self._regs.scope():`
        # wrapper releases them. See `_RegAlloc.scope`.
        with self._regs.scope():
            cursor_r = self._regs.get("__cursor")
            avail_r = self._regs.get("__uart_avail_tmp")
            byte_r = self._regs.get("__byte_tmp")
            match_r = self._regs.get("__match_tmp")
            timer_r = self._regs.get("__timer_tmp") if timeout_ms is not None else None

            em.emit_u32(Op.LOAD_IMM, cursor_r, 0)
            if timer_r is not None:
                em.emit_u32(Op.LOAD_IMM, timer_r, 0)
            if len(delim_bytes) == 1:
                # Fast path: single-byte delim, no progress counter needed.
                em.label(L_TOP)
                em.emit(Op.UART_AVAIL, avail_r)
                em.emit_jmp(Op.JZ, L_IDLE, avail_r)
                em.emit(Op.UART_READ_REG, 1, cursor_r)
                em.emit(Op.LOAD_U8_REG, byte_r, cursor_r)
                em.emit_cmp(Op.ADD, cursor_r, 1, cursor_r)
                em.emit_cmp(Op.CMP_EQ, byte_r, delim_bytes[0], match_r)
                em.emit_jmp(Op.JNZ, L_FOUND, match_r)
                em.emit_cmp(Op.CMP_EQ, cursor_r, max, match_r)
                em.emit_jmp(Op.JNZ, L_FOUND, match_r)
                em.emit_jmp(Op.JMP, L_TOP)
            else:
                # Multi-byte delim. `pg` (progress) tracks how many delim
                # bytes have matched at the tail. Each new byte advances
                # pg on match against delim[pg], or resets pg on mismatch
                # (with a check for restart-on-this-byte against delim[0]).
                pg_r = self._regs.get("__match_pg")
                em.emit_u32(Op.LOAD_IMM, pg_r, 0)

                em.label(L_TOP)
                em.emit(Op.UART_AVAIL, avail_r)
                em.emit_jmp(Op.JZ, L_IDLE, avail_r)
                em.emit(Op.UART_READ_REG, 1, cursor_r)
                em.emit(Op.LOAD_U8_REG, byte_r, cursor_r)
                em.emit_cmp(Op.ADD, cursor_r, 1, cursor_r)

                # Dispatch: jump to the case-block for the current pg.
                # pg is always in [0, len(delim_bytes)) by construction so
                # the trailing JMP is defensive only.
                case_labels = [self._fresh_label(f"ru_case_{i}")
                               for i in range(len(delim_bytes))]
                for i, lbl in enumerate(case_labels):
                    if i == 0:
                        em.emit_jmp(Op.JZ, lbl, pg_r)
                    else:
                        em.emit_cmp(Op.CMP_EQ, pg_r, i, match_r)
                        em.emit_jmp(Op.JNZ, lbl, match_r)
                em.emit_jmp(Op.JMP, L_AFTER)

                # Per-case bodies. Each handles one value of pg, ending
                # with JMP L_AFTER (or JNZ L_FOUND on full match).
                for i, lbl in enumerate(case_labels):
                    em.label(lbl)
                    em.emit_cmp(Op.CMP_EQ, byte_r, delim_bytes[i], match_r)
                    is_last = (i + 1 == len(delim_bytes))
                    if is_last:
                        # Full delim matched.
                        em.emit_jmp(Op.JNZ, L_FOUND, match_r)
                    else:
                        # Partial match → advance pg.
                        L_advance = self._fresh_label(f"ru_advance_{i}")
                        em.emit_jmp(Op.JNZ, L_advance, match_r)
                    # Fall through: this byte did not match delim[i].
                    if i == 0:
                        # pg was 0; stay at 0.
                        em.emit_jmp(Op.JMP, L_AFTER)
                    else:
                        # pg was > 0; reset, but the current byte itself
                        # might start a fresh match against delim[0].
                        em.emit_cmp(Op.CMP_EQ, byte_r, delim_bytes[0], match_r)
                        L_reset_zero = self._fresh_label(f"ru_reset_{i}")
                        em.emit_jmp(Op.JZ, L_reset_zero, match_r)
                        em.emit_u32(Op.LOAD_IMM, pg_r, 1)
                        em.emit_jmp(Op.JMP, L_AFTER)
                        em.label(L_reset_zero)
                        em.emit_u32(Op.LOAD_IMM, pg_r, 0)
                        em.emit_jmp(Op.JMP, L_AFTER)
                    if not is_last:
                        em.label(L_advance)
                        em.emit_u32(Op.LOAD_IMM, pg_r, i + 1)
                        em.emit_jmp(Op.JMP, L_AFTER)

                em.label(L_AFTER)
                em.emit_cmp(Op.CMP_EQ, cursor_r, max, match_r)
                em.emit_jmp(Op.JNZ, L_FOUND, match_r)
                em.emit_jmp(Op.JMP, L_TOP)

            em.label(L_IDLE)
            if timer_r is not None:
                em.emit_cmp(Op.ADD, timer_r, 1, timer_r)
                em.emit_cmp(Op.CMP_EQ, timer_r, timeout_ms, match_r)
                em.emit_jmp(Op.JNZ, L_TIMEOUT, match_r)
            em.emit_u16(Op.SLEEP_MS, 1)                  # tight poll cadence
            em.emit_jmp(Op.JMP, L_TOP)
            if timer_r is not None:
                em.label(L_TIMEOUT)
                em.emit(Op.ERROR, ERR_CODE_TIMEOUT)
            em.label(L_FOUND)

    def read_n(self, count: int,
               timeout_ms: Optional[int] = None) -> TracedSlice:
        """Compile: read exactly `count` bytes from UART into
        sample_buf[0..count). Blocks (yielding via SLEEP_MS) until all
        bytes have arrived. The frame lands at `sample_buf[0]`, so a
        fixed-length record commits with `store_sample()` (which
        publishes the compile-time `set_sample_size`). `store_sample_n()`
        is for the variable-length `read_until` path, not a fixed
        `read_n`.

        `timeout_ms` (optional): bound the total wait via an iteration
        counter on the idle branch. On expiry the VM emits
        `OP_ERROR ERR_CODE_TIMEOUT` and step() returns
        `VmErr::TIMEOUT` (-417). Must be in 1..65535.

        Returns a `TracedSlice` over `sample_buf[0..count)` so callers
        can chain `.expect(expected_bytes)` to verify a structured
        response (e.g. a UBX-ACK or SBF `$R+...` header).
        """
        if not (0 < count <= _ASTCompiler.VM_SAMPLE_BUF_SIZE):
            raise CompileError(
                f"read_n: count must be 1..{_ASTCompiler.VM_SAMPLE_BUF_SIZE} "
                f"(the sample buffer), got {count!r}")
        if timeout_ms is not None and not (0 < timeout_ms <= 0xFFFF):
            raise CompileError(
                f"read_n: timeout_ms must be 1..65535, got {timeout_ms!r}")

        em = self._emitter
        L_TOP = self._fresh_label("rn_top")
        L_IDLE = self._fresh_label("rn_idle")
        L_DONE = self._fresh_label("rn_done")
        L_TIMEOUT = self._fresh_label("rn_timeout") if timeout_ms is not None else None

        # Scratch scope: see `_RegAlloc.scope`. The `__cursor`,
        # `__uart_avail_tmp`, `__match_tmp`, `__timer_tmp` slots
        # are dead at L_DONE — releasing them lets the next helper
        # (TracedSlice.expect emitted on the returned slice, or a
        # later read_until / compute_checksum) reuse the same
        # register indices. Without this, a probe of the form
        # `read_n(N, timeout_ms=T) → .expect(bytes)` followed by
        # a configure with compute_checksum trips VM_NUM_REGS=8.
        with self._regs.scope():
            cursor_r = self._regs.get("__cursor")
            avail_r = self._regs.get("__uart_avail_tmp")
            match_r = self._regs.get("__match_tmp")
            timer_r = self._regs.get("__timer_tmp") if timeout_ms is not None else None

            em.emit_u32(Op.LOAD_IMM, cursor_r, 0)
            if timer_r is not None:
                em.emit_u32(Op.LOAD_IMM, timer_r, 0)
            em.label(L_TOP)
            em.emit(Op.UART_AVAIL, avail_r)
            em.emit_jmp(Op.JZ, L_IDLE, avail_r)
            em.emit(Op.UART_READ_REG, 1, cursor_r)
            em.emit_cmp(Op.ADD, cursor_r, 1, cursor_r)
            em.emit_cmp(Op.CMP_EQ, cursor_r, count, match_r)
            em.emit_jmp(Op.JNZ, L_DONE, match_r)
            em.emit_jmp(Op.JMP, L_TOP)
            em.label(L_IDLE)
            if timer_r is not None:
                em.emit_cmp(Op.ADD, timer_r, 1, timer_r)
                em.emit_cmp(Op.CMP_EQ, timer_r, timeout_ms, match_r)
                em.emit_jmp(Op.JNZ, L_TIMEOUT, match_r)
            em.emit_u16(Op.SLEEP_MS, 1)
            em.emit_jmp(Op.JMP, L_TOP)
            if timer_r is not None:
                em.label(L_TIMEOUT)
                em.emit(Op.ERROR, ERR_CODE_TIMEOUT)
            em.label(L_DONE)
        return TracedSlice(driver=self, buf_off=0, length=count)

    def store_sample(self) -> None:
        """Compile: commit one fixed-length sample to the ring. The width
        is the compile-time `set_sample_size` value, so no runtime size
        register is read. This is the correct pairing for a fixed-length
        record read with `read_n`: the frame lands at `sample_buf[0]` and
        its length is known at compile time. Use `store_sample_n()` only
        for a variable-length record captured by `read_until`."""
        self._emitter.emit(Op.STORE_SAMPLE)

    def store_sample_n(self) -> None:
        """Compile: commit `__cursor` bytes of sample_buf to the ring as
        one sample — the variable-length width captured by an immediately
        preceding `read_until`. For a fixed-length record read with
        `read_n`, use `store_sample()` instead: `__cursor` is a scratch
        register that intervening `match`/`verify_checksum` ops may
        overwrite, and a fixed record's width is already known from
        `set_sample_size`."""
        cursor_r = self._regs.get("__cursor")
        self._emitter.emit(Op.STORE_SAMPLE_N, cursor_r)

    def parse_u16_le(self, offset: int) -> int:
        """Compile: load 16-bit little-endian word from sample_buf at
        the given immediate offset. Returns the placeholder int 0
        (the actual value lives in the `__parsed_u16` runtime
        register). Used by length-prefixed binary protocols to read
        the length field after a sync match."""
        if not (0 <= offset <= 0xFF):
            raise CompileError(f"parse_u16_le: offset must be 0..255, got {offset!r}")
        dst = self._regs.get("__parsed_u16")
        self._emitter.emit(Op.LOAD_U16_LE, dst, offset)
        return 0

    def compute_checksum(self, spec: "ChecksumDescriptor",
                         start_off: int, length: int, dst_off: int) -> None:
        """Compile a checksum-over-buffer loop into RISC bytecode.

        Iterates `sample_buf[start_off:start_off+length]` with the
        algorithm in `spec` and stores the resulting bytes at
        `sample_buf[dst_off:]`. `spec` is one of `ChecksumFletcher`,
        `ChecksumXorFold`, or `ChecksumPolynomial` — vendor-agnostic
        descriptors that cover every checksum the project has needed
        so far. The bytecode is the same shape regardless of vendor;
        no protocol-specific firmware op is needed.

        `start_off`, `length`, and `dst_off` are immediate (compile-
        time) values. Runtime-variable `length` (for length-prefixed
        binary protocols where the length comes from a parsed header)
        is a follow-up extension.
        """
        if not isinstance(spec, ChecksumDescriptor):
            raise CompileError(
                f"compute_checksum: spec must be a ChecksumDescriptor "
                f"(Fletcher / XorFold / Polynomial), got {type(spec).__name__}")
        if not (0 <= start_off <= 0xFF):
            raise CompileError(
                f"compute_checksum: start_off out of range: {start_off!r}")
        if not (0 < length <= 0xFF):
            raise CompileError(
                f"compute_checksum: length must be 1..255, got {length!r}")
        if not (0 <= dst_off <= 0xFF):
            raise CompileError(
                f"compute_checksum: dst_off out of range: {dst_off!r}")
        spec._emit(self, start_off, length, dst_off)


# ── Checksum descriptors ────────────────────────────────────
#
# Vendor-agnostic algorithm specs. Each compiles to a RISC-bytecode
# loop that runs on the VM at execution time, so changing a
# patched param (e.g. UBX `meas_rate_ms`) automatically updates the
# checksum on the next reload without any per-vendor opcode.

class ChecksumDescriptor:
    """Abstract base. Subclasses implement `_emit(driver, start_off,
    length, dst_off)` to inject the checksum-compute bytecode into the
    driver's emitter using the new RISC primitives."""

    def _emit(self, driver, start_off: int, length: int, dst_off: int) -> None:
        raise NotImplementedError


class ChecksumFletcher(ChecksumDescriptor):
    """Two-byte Fletcher-8 checksum:

        CK_A = sum_i(byte_i) & 0xFF
        CK_B = sum_i(CK_A_after_byte_i) & 0xFF

    Used by UBX (u-blox binary), TCP/IP, NTP. Writes 2 bytes:
    `[CK_A, CK_B]` starting at `dst_off`."""

    def _emit(self, driver, start_off: int, length: int, dst_off: int) -> None:
        em = driver._emitter
        L_TOP = driver._fresh_label("ck_fl_top")
        L_DONE = driver._fresh_label("ck_fl_done")

        # Scratch scope: Fletcher uses 5 scratch registers
        # (ck_a, ck_b, cursor, byte_r, match_r). Without
        # `with self._regs.scope():`, these would persist for the
        # whole driver lifetime — and a probe that already allocated
        # scratch names via read_n + .expect would push the total
        # past VM_NUM_REGS=8 here. See `_RegAlloc.scope`.
        with driver._regs.scope():
            ck_a = driver._regs.get("__ck_a")
            ck_b = driver._regs.get("__ck_b")
            cursor = driver._regs.get("__ck_cursor")
            byte_r = driver._regs.get("__ck_byte")
            match_r = driver._regs.get("__ck_match")

            em.emit_u32(Op.LOAD_IMM, ck_a, 0)
            em.emit_u32(Op.LOAD_IMM, ck_b, 0)
            em.emit_u32(Op.LOAD_IMM, cursor, start_off)
            em.label(L_TOP)
            em.emit(Op.LOAD_U8_REG, byte_r, cursor)
            em.emit(Op.ADD_REG, ck_a, byte_r, ck_a)
            em.emit_cmp(Op.AND, ck_a, 0xFF, ck_a)
            em.emit(Op.ADD_REG, ck_b, ck_a, ck_b)
            em.emit_cmp(Op.AND, ck_b, 0xFF, ck_b)
            em.emit_cmp(Op.ADD, cursor, 1, cursor)
            em.emit_cmp(Op.CMP_EQ, cursor, start_off + length, match_r)
            em.emit_jmp(Op.JZ, L_TOP, match_r)
            em.label(L_DONE)
            em.emit(Op.STORE_U8, dst_off, ck_a)
            em.emit(Op.STORE_U8, dst_off + 1, ck_b)


class ChecksumXorFold(ChecksumDescriptor):
    """Single-byte XOR-fold:

        ck = byte_0 ^ byte_1 ^ … ^ byte_(length-1)

    Used by NMEA (1 byte XOR over `$…*` payload then encoded as 2
    hex digits), Modbus-ASCII LRC variants. Writes 1 byte at
    `dst_off`."""

    def _emit(self, driver, start_off: int, length: int, dst_off: int) -> None:
        em = driver._emitter
        L_TOP = driver._fresh_label("ck_xor_top")
        L_DONE = driver._fresh_label("ck_xor_done")

        # Scratch scope — see ChecksumFletcher._emit and
        # `_RegAlloc.scope`. The ck / cursor / byte_r / match_r
        # slots are released at scope exit.
        with driver._regs.scope():
            ck = driver._regs.get("__ck_xor")
            cursor = driver._regs.get("__ck_cursor")
            byte_r = driver._regs.get("__ck_byte")
            match_r = driver._regs.get("__ck_match")

            em.emit_u32(Op.LOAD_IMM, ck, 0)
            em.emit_u32(Op.LOAD_IMM, cursor, start_off)
            em.label(L_TOP)
            em.emit(Op.LOAD_U8_REG, byte_r, cursor)
            em.emit(Op.XOR_REG, ck, byte_r, ck)
            em.emit_cmp(Op.ADD, cursor, 1, cursor)
            em.emit_cmp(Op.CMP_EQ, cursor, start_off + length, match_r)
            em.emit_jmp(Op.JZ, L_TOP, match_r)
            em.label(L_DONE)
            em.emit(Op.STORE_U8, dst_off, ck)


class ChecksumPolynomial(ChecksumDescriptor):
    """Polynomial CRC-8 with configurable `poly`, `init`, `xor_out`.
    Compiles to the dedicated `OP_CRC8` opcode (no loop in bytecode
    — the firmware op covers the inner double-loop in tight C).
    Covers Sensirion 0x31, SMBus PEC 0x07, IIM-20670 0x1D, and any
    other CRC-8 family. Writes 1 byte at `dst_off`."""

    def __init__(self, poly: int, init: int = 0, xor_out: int = 0):
        if not (0 <= poly <= 0xFF):
            raise CompileError(f"ChecksumPolynomial: poly out of range: {poly!r}")
        if not (0 <= init <= 0xFF):
            raise CompileError(f"ChecksumPolynomial: init out of range: {init!r}")
        if not (0 <= xor_out <= 0xFF):
            raise CompileError(f"ChecksumPolynomial: xor_out out of range: {xor_out!r}")
        self.poly = poly
        self.init = init
        self.xor_out = xor_out

    def _emit(self, driver, start_off: int, length: int, dst_off: int) -> None:
        em = driver._emitter
        # Scratch scope — see `_RegAlloc.scope`. Only one register
        # needed (the CRC result), but the scope is still required:
        # without it, every call site of a Polynomial checksum
        # leaks `__ck_crc` into the persistent pool, and a driver
        # chaining multiple checksums or another helper afterwards
        # eats into the VM_NUM_REGS=8 budget needlessly.
        with driver._regs.scope():
            result_r = driver._regs.get("__ck_crc")
            em.emit(Op.CRC8, start_off, length, self.poly, self.init,
                    self.xor_out, result_r, 0)
            em.emit(Op.STORE_U8, dst_off, result_r)
