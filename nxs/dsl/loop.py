"""The measure loop's AST compiler: statements, control flow, and the sample commit."""

from __future__ import annotations

import ast
import inspect
import textwrap
from typing import Dict

from nxs.opcodes import Op
from nxs.dsl.emit import BUS_REGISTER, BUS_STREAM, _Emitter, _RegAlloc, _emit_store_bytes
from nxs.dsl.errors import CompileError
from nxs.dsl.fields import field_width
from nxs.dsl.loop_values import _ValueCompiler


class _ASTCompiler(_ValueCompiler, ast.NodeVisitor):
    """Compiles a measure() function body into bytecode via AST walking."""

    # Firmware sample-buffer size; burst codegen keeps its rotating TX/RX slot
    # at the tail, past the output region.
    VM_SAMPLE_BUF_SIZE = 128
    # Scalar value-reads stage at the last 4 bytes (the LOAD width cap), so they
    # never alias sample data at [0, sample_size). Data reads land at offset 0.
    SCALAR_SCRATCH_OFF = VM_SAMPLE_BUF_SIZE - 4   # 124
    # 64-bit work buffer: 32 slots of 8 bytes; 256 B is the u8-offset ceiling.
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
        # Fallback target for methods absent from the AST dispatch table: a base
        # class helper emits to the shared emitter directly.
        self._driver = driver
        self._loop_label = "__measure_loop"
        # measure-local name to work-buffer offset for 64-bit locals; narrow
        # locals stay in _regs.
        self._wide_locals: Dict[str, int] = {}

    def _divider_for(self, rate: int) -> int:
        """Sync divider for `rate` on a fixed-sync part. Only exact divisors
        deliver the declared rate, so anything else is rejected."""
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

        # Ledger of sample-buffer regions claimed by data-placing reads;
        # overlapping claims are rejected in _claim_data_span.
        self._data_spans = []

        if self._trigger == "drdy" and self._drdy_base_hz > 0:
            # Fixed-sync part: divide the hardware sync once, before the loop;
            # the u16 divider operand is sample_rate's patch site.
            div = self._divider_for(self._sample_rate)
            div_off = self._em._current_offset() + 1
            self._em.emit_u16(Op.EVENT_DIV, div)
            if self._driver is not None:
                self._driver._patch_drdy_div(div_off, self._drdy_base_hz)

        # Loop top: DRDY mode yields on the hardware event and never sleeps
        # (rate reduction goes through the ODR register); poll mode sleeps.
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
        rejecting unknown keys. Defaults: unsigned, big-endian, primary."""
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
        """Extract (into, dev) from a data-placing read's keywords: the
        destination offset (default 0) and an optional companion target."""
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
        """Byte offset a scalar value-read stages at before it LOADs: the
        sample-buffer tail. Rejects a sample_size that reaches into it."""
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
        """Reserve sample_buf[off .. off+length) for a data-placing read;
        overlapping or out-of-range spans are rejected. The scalar-scratch tail
        is guarded by _scalar_scratch_off, so a pure-burst sample may fill it."""
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

    def _compile_expr_stmt(self, node):
        call = node.value
        if not isinstance(call, ast.Call):
            # A docstring is the one legal non-call expression statement; a
            # BoolOp or bare comparison emits no bytecode, so reject it.
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
                # the transaction and discard the value.
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

        # Fallback: a driver method (read_until, read_n, store_sample_n, ...)
        # called with evaluated arguments emits to the shared emitter directly.
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
            # if (var & mask): run the body when the masked bits are nonzero.
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
            # The true path jumps over the else body; `elif` is a nested If in
            # orelse and recurses through this path.
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
            # A committed sample ends the iteration: jump to the loop top so a
            # mid-body `return Sample(...)` doesn't fall through.
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
        field, set the sample size, and commit. The kwarg set must equal the
        declared field set."""
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
            _emit_store_bytes(self._em, self._regs, reg, field_off, width, border)
        # The sample extent is the furthest field end (offsets may gap).
        total = max(o + w for (o, w, _) in layout.values())
        self._em.emit(Op.SET_SAMPLE_SIZE, total)
        self._em.emit(Op.STORE_SAMPLE)

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
            # bytes literals pass through unchanged for helpers like
            # `read_until`; everything numeric is promoted to int.
            if isinstance(node.value, (bytes, bytearray, str)):
                return node.value
            return int(node.value)
        if (isinstance(node, ast.Attribute) and
                isinstance(node.value, ast.Name) and node.value.id == "self"):
            # `self.CMD_WORD`: an UPPER_CASE class-level integer constant,
            # resolved on the class so trace-time state can't leak into bytecode.
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


