"""Run values: the unsigned 32-bit expressions a camera program computes at run time."""

from __future__ import annotations

from typing import Any, Callable

from nxs.opcodes import Op
from nxs.dsl.emit import _Emitter, _RegAlloc
from nxs.dsl.errors import CompileError


# ── Run values ─────────────────────────────────────────────

U32_MAX = 0xFFFFFFFF


class RunValue:
    """An unsigned 32-bit value a camera program computes at run time from
    its staged parameters: `param()` reads one, `+ - * //` with another
    value or an integer build an expression, `>>` and `&` with an integer
    take a field out of a packed value, and a verb (`write_wide`,
    `store_param`) emits it. Every intermediate carries the interval the
    declared ranges allow, and an operation whose result can leave
    0..2^32-1, or divide by zero, is refused at compile time."""

    _REG_OPS = {"+": Op.ADD_REG, "-": Op.SUB_REG, "*": Op.MUL_REG, "//": Op.DIVU_REG}

    def __init__(self, lo: int, hi: int, kind: str, args: tuple, text: str):
        self._lo = lo
        self._hi = hi
        self._kind = kind        # "param" | "const" | "binop" | "field"
        self._args = args
        self._text = text

    @property
    def lo(self) -> int:
        """The lowest value the expression can take."""
        return self._lo

    @property
    def hi(self) -> int:
        """The highest value the expression can take."""
        return self._hi

    @classmethod
    def of_param(cls, index: int, name: str, lo: int, hi: int) -> "RunValue":
        return cls(lo, hi, "param", (index,), f"param({name!r})")

    @classmethod
    def of_const(cls, value: Any, context: str) -> "RunValue":
        if not isinstance(value, int) or isinstance(value, bool):
            raise CompileError(
                f"{context}: {value!r} is not an integer; a run value is "
                f"unsigned 32-bit arithmetic on staged parameters")
        if not 0 <= value <= U32_MAX:
            raise CompileError(
                f"{context}: {value} is outside 0..{U32_MAX}")
        return cls(value, value, "const", (value,), str(value))

    def emit(self, em: "_Emitter", regs: "_RegAlloc",
             fresh: Callable[[str], str]) -> int:
        """Evaluate into a scratch register of the current scope and return
        it. A right operand takes a nested scope, so an expression holds
        one register per level of its right operands."""
        if self._kind == "param":
            reg = regs.get(fresh("rv"))
            em.emit(Op.PARAM_LOAD, reg, self._args[0])
            return reg
        if self._kind == "const":
            reg = regs.get(fresh("rv"))
            em.emit_u32(Op.LOAD_IMM, reg, self._args[0])
            return reg
        if self._kind == "field":
            op, a, n = self._args
            reg = a.emit(em, regs, fresh)
            if op == ">>":
                em.emit(Op.SHR, reg, n, reg)
            else:
                em.emit_cmp(Op.AND, reg, n, reg)
            return reg
        op, a, b = self._args
        reg = a.emit(em, regs, fresh)
        with regs.scope():
            rb = b.emit(em, regs, fresh)
            em.emit(self._REG_OPS[op], reg, rb, reg)
        return reg

    def _binop(self, op: str, other: Any, reflected: bool) -> "RunValue":
        if not isinstance(other, RunValue):
            other = RunValue.of_const(other, f"{self._text} {op} {other!r}")
        a, b = (other, self) if reflected else (self, other)
        text = f"({a._text} {op} {b._text})"
        if op == "+":
            lo, hi = a._lo + b._lo, a._hi + b._hi
        elif op == "-":
            lo, hi = a._lo - b._hi, a._hi - b._lo
            if lo < 0:
                raise CompileError(
                    f"{text} can go below 0 ({a._text} spans {a._lo}..{a._hi}, "
                    f"{b._text} spans {b._lo}..{b._hi}); the VM subtracts "
                    f"unsigned")
        elif op == "*":
            lo, hi = a._lo * b._lo, a._hi * b._hi
        else:
            if b._lo == 0:
                raise CompileError(
                    f"{text}: the divisor can be 0 ({b._text} spans "
                    f"{b._lo}..{b._hi}); declare its range from 1")
            lo, hi = a._lo // b._hi, a._hi // b._lo
        if hi > U32_MAX:
            raise CompileError(
                f"{text} can reach {hi}, past {U32_MAX} ({a._text} spans "
                f"{a._lo}..{a._hi}, {b._text} spans {b._lo}..{b._hi}); the VM "
                f"keeps the low 32 bits")
        return RunValue(lo, hi, "binop", (op, a, b), text)

    def __add__(self, other):
        return self._binop("+", other, False)

    def __radd__(self, other):
        return self._binop("+", other, True)

    def __sub__(self, other):
        return self._binop("-", other, False)

    def __rsub__(self, other):
        return self._binop("-", other, True)

    def __mul__(self, other):
        return self._binop("*", other, False)

    def __rmul__(self, other):
        return self._binop("*", other, True)

    def __floordiv__(self, other):
        return self._binop("//", other, False)

    def __rfloordiv__(self, other):
        return self._binop("//", other, True)

    def _field(self, op: str, n: Any) -> "RunValue":
        """`>>` or `&` with an integer the compile knows: `SHR` takes its
        shift and `AND` its mask as immediates, and the VM has no form of
        either with a run value on the right."""
        what, name = ("shift", "SHR") if op == ">>" else ("mask", "AND")
        if isinstance(n, bool) or not isinstance(n, int):
            raise CompileError(
                f"{self._text} {op} {n!r}: the {what} must be an integer the compile "
                f"knows, since the VM's {name} takes it as an immediate")
        if op == ">>":
            if not 0 <= n <= 31:
                raise CompileError(f"{self._text} >> {n}: a shift is 0 to 31 bits")
            lo, hi, text = self._lo >> n, self._hi >> n, f"({self._text} >> {n})"
        else:
            if not 0 <= n <= U32_MAX:
                raise CompileError(f"{self._text} & {n}: a mask is 0..{U32_MAX}")
            # x & n is at most x and at most n, and exact for a constant x.
            exact = self._lo == self._hi
            lo = self._lo & n if exact else 0
            hi = self._lo & n if exact else min(self._hi, n)
            text = f"({self._text} & 0x{n:X})"
        return RunValue(lo, hi, "field", (op, self, n), text)

    def __rshift__(self, other):
        return self._field(">>", other)

    def __rrshift__(self, other):
        raise CompileError(
            f"{other!r} >> {self._text}: the shift must be an integer the compile knows, "
            f"since the VM's SHR takes it as an immediate")

    def __and__(self, other):
        return self._field("&", other)

    __rand__ = __and__

    def __truediv__(self, other):
        raise CompileError(
            f"{self._text} / {other!r}: the VM divides unsigned integers; "
            f"write //")

    __rtruediv__ = __truediv__

    def __bool__(self):
        raise CompileError(
            f"{self._text} has no truth at compile time; select() dispatches "
            f"on a parameter's value")

    def __repr__(self) -> str:
        return self._text


