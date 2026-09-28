"""The bytecode emitter and its allocators: registers, the 64-bit work buffer, labels, traced reads."""

from __future__ import annotations

import struct
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Dict, Optional

from nxs._generated_constants import OpErrors
from nxs.opcodes import Op
from nxs.dsl.errors import CompileError
from nxs.dsl.fields import REG_ADDR_MAX_8


# ── Register allocator ──────────────────────────────────────

class _RegAlloc:
    """Maps named variables to VM register indices (r0..r7). Pinned names
    allocate from r7 down and survive `reset()`; scratch names allocate from
    r0 up and are released at `reset()` or at the exit of their `scope()`."""

    # Firmware register-file size.
    VM_NUM_REGS = 8

    def __init__(self):
        self._map: Dict[str, int] = {}
        self._next = 0
        self._pinned: Dict[str, int] = {}
        self._pin_next = self.VM_NUM_REGS - 1

    def pin(self, name: str) -> int:
        """Bind `name` to a register that survives `reset()`. Pinned names
        allocate from r7 down, scratch from r0 up, so the binding holds across
        the configure() to measure() boundary."""
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
        """Allocate or look up a register for `name`. Inside a `scope()` the
        binding is released at scope exit; outside, it lasts until `reset()`."""
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
        """Open a scratch-register scope: names allocated via `get` inside the
        block are released at exit. Nests. Every framing helper body must run
        inside one, or chained helpers exhaust the 8-register file."""
        mark = len(self._map)
        try:
            yield self
        finally:
            # Names added inside the scope are the tail of the insertion-ordered
            # map; drop everything since the mark.
            for k in list(self._map.keys())[mark:]:
                del self._map[k]
            # Recompute _next so the freed slots become available again.
            self._next = max(self._map.values(), default=-1) + 1

    def reset(self):
        """Drop all scratch bindings at the configure() to measure() boundary.
        Pinned names survive."""
        self._map.clear()
        self._next = 0


# ── Work-buffer allocator (64-bit slots) ────────────────────

class WideRef:
    """A 64-bit value living at a work-buffer offset, returned by configure()
    coefficient reads (`self.cN`) and produced by wide intermediates."""

    def __init__(self, off: int):
        self._off = off

    @property
    def off(self) -> int:
        return self._off


class _WorkAlloc:
    """Free-list allocator of 8-byte slots in the VM's 64-bit work buffer.
    Persistent values are held; anonymous intermediates are freed once their
    consuming op runs."""

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

    # Opcodes that return VM_YIELD, letting the runner observe a STOP request;
    # a backward branch next to one of these needs no inserted yield.
    _YIELDABLE_OPCODES = frozenset({Op.YIELD, Op.SLEEP_MS, Op.SLEEP_US})

    def __init__(self, reg_addr_max: int = REG_ADDR_MAX_8):
        self._instructions: list = []
        self._labels: Dict[str, int] = {}
        self._fixups: list = []
        # Opcode of the most recently emitted instruction (None at start).
        self._last_op: Optional[int] = None
        # Labels placed but unbound; resolved on the next emit.
        self._pending_labels: list = []
        # label name → opcode of the first instruction at the label.
        self._label_op: Dict[str, int] = {}
        # Highest register address the bound bus frames (the I²C profile's
        # addr_bytes decides).
        self._reg_addr_max = reg_addr_max

    def label(self, name: str):
        self._labels[name] = self._current_offset()
        self._pending_labels.append(name)

    def emit(self, *args: int):
        self._on_emit(args[0])
        self._instructions.append((None, bytes(args)))

    def emit_u16(self, opcode: int, val: int):
        self._on_emit(opcode)
        self._instructions.append((None, struct.pack("<BH", opcode, val)))

    def check_reg(self, reg: int) -> int:
        """`reg` when the bound bus frames it; a register past the profile's
        address width is a CompileError."""
        if not 0 <= reg <= self._reg_addr_max:
            if self._reg_addr_max == REG_ADDR_MAX_8:
                raise CompileError(
                    f"register 0x{reg:X} exceeds the 8-bit address limit; a "
                    f"16-bit register map declares "
                    f"I2C_PROFILE = I2cProfile(addr_bytes=2)")
            raise CompileError(
                f"register 0x{reg:X} exceeds the 16-bit address limit")
        return reg

    def emit_reg(self, opcode: int, reg: int, *tail: int):
        """Emit a register opcode: 8-bit opcode, 16-bit little-endian register
        operand, then single-byte operands. The register must fit the bound
        bus's address width (8 bits unless the I²C profile widens it)."""
        self.check_reg(reg)
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
        """Emit a jump with a fixup for the label offset. A backward branch with
        no yieldable opcode at its source or target gets an implicit yield
        first, bounding STOP latency to one loop tick."""
        if label in self._labels:
            target_op = self._label_op.get(label)
            if (self._last_op not in self._YIELDABLE_OPCODES and
                    target_op not in self._YIELDABLE_OPCODES):
                # SLEEP_US(0) yields to the runner without blocking on DRDY,
                # which a stream-driver inner loop has no semantic for.
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



class _TraceReadValue:
    """Sentinel returned by a width-1 read during configure() tracing. Any use
    raises: configure() reads are compile-time mocks, so runtime conditionals
    and read-modify-write belong in measure()."""
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


# Bus kind tags. The AST compiler picks the opcode family by these when the
# driver writes `self.read(...)` in a measure loop.
BUS_REGISTER = "register"
BUS_STREAM = "stream"

# Physical bus name to the integer code stored in the NXS `bus` param.
_BUS_NAME_TO_CODE = {'i2c': 0, 'spi': 1}
_BUS_CODE_TO_NAME = {v: k for k, v in _BUS_NAME_TO_CODE.items()}

# `OP_ERROR` operand codes: the byte lands in the VM's error field, `ERROR_CODE`
# mirrors it, and `nxs status` names it.
OpErrorCode = OpErrors.OpErrorCode


@dataclass
class TracedSlice:
    """Trace-side handle on a `sample_buf` slice produced by `read_n` /
    `read_until`; carries the buffer offset and length for `.expect()`."""
    driver: Any
    buf_off: int
    length: int

    def expect(self, expected: bytes) -> None:
        """Emit a byte-by-byte compare of `expected` against the slice, jumping
        to `OP_ERROR OpErrorCode.MISMATCH` on the first mismatch. Raises
        CompileError if `expected` is empty or longer than the slice."""
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

        # Scoped scratch: the LOAD_U8/CMP_EQ temporaries die with the chain.
        with self.driver._regs.scope():
            byte_r = self.driver._regs.get("__byte_tmp")
            match_r = self.driver._regs.get("__match_tmp")
            for i, want in enumerate(expected_bytes):
                em.emit(Op.LOAD_U8, byte_r, self.buf_off + i)
                em.emit_cmp(Op.CMP_EQ, byte_r, want, match_r)
                em.emit_jmp(Op.JZ, L_FAIL, match_r)
            em.emit_jmp(Op.JMP, L_DONE)
            em.label(L_FAIL)
            em.emit(Op.ERROR, OpErrorCode.MISMATCH)
            em.label(L_DONE)


# ── AST compiler for measure_loop ───────────────────────────

def _load_spec_byte(width: int, signed: bool, endian: str) -> int:
    """Compose an `OP_LOAD` spec byte: low 3 bits = width (1-4), bit 3 =
    little-endian, bit 4 = sign-extend. Unsigned big-endian equals `width`."""
    if endian not in ("big", "little"):
        raise CompileError(
            f"read(endian={endian!r}): must be 'big' or 'little'")
    return width | (0x08 if endian == "little" else 0) | (0x10 if signed else 0)


def _emit_store_bytes(em: _Emitter, regs: _RegAlloc, reg: int, off: int,
                      width: int, byte_order: str) -> None:
    """Store `width` low bytes of r[reg] into sample_buf[off..], MSB-first
    for big-endian. STORE_U8 writes the low byte, so each higher byte is
    shifted down into a scratch register first."""
    tmp = regs.get("__store_scratch")
    for i in range(width):
        shift = 8 * (width - 1 - i) if byte_order == 'big' else 8 * i
        if shift == 0:
            em.emit(Op.STORE_U8, off + i, reg)
        else:
            em.emit(Op.SHR, reg, shift, tmp)
            em.emit(Op.STORE_U8, off + i, tmp)


