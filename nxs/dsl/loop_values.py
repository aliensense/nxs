"""The measure loop's value compiler: expressions, wide arithmetic, checksum verification, frame bursts."""

from __future__ import annotations

import ast

from nxs.opcodes import Op
from nxs.dsl.emit import BUS_REGISTER, BUS_STREAM, WideRef, _load_spec_byte
from nxs.dsl.errors import CompileError


class _ValueCompiler:
    """The expression half of `_ASTCompiler`: how a value lands in a register, the
    work buffer, or the sample buffer."""

    def _compile_value_into_reg(self, value, var_name: str):
        # A wide assignment (a multiply, or any wide operand) lands in the work
        # buffer. Evaluate first so the RHS sees the name's current binding.
        if self._is_wide(value):
            off, owned = self._eval_wide(value)
            existing = self._wide_locals.get(var_name)
            if existing is not None:
                # Reassign in place: the local's one slot holds original-or-
                # updated, so a conditional reassign reads right either way.
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

        # Narrow assignment. If `var_name` was wide before, release its slot.
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
                    # bytes, then LOAD honouring sign and byte order.
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
                # read_words bounds its output against the floating FRAME slot
                # inside _emit_frame_burst, so it is not in the span ledger.
                self._emit_frame_burst(start_reg, num_words)
                return
            if method == "xfer":
                # `x = self.xfer(word, width=2)`: clock a literal full-duplex SPI
                # word and load the response, unsigned MSB-first, `width` bytes.
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

        # read_analog is bus-agnostic: it stages the raw 16-bit count at the
        # scalar-scratch tail, so publish it through a named Sample field.
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
        # ck_off)`: recompute over the span and count mismatches (0 = intact).
        if method == "verify_checksum":
            self._compile_verify_checksum(value, dst)
            return

        raise CompileError(
            f"Unsupported assignment value: self.{method}(...) "
            f"on {self._bus_kind} driver (line {value.lineno})")

    def _compile_verify_checksum(self, value, dst: int):
        """Emit the Fletcher-verify loop; `dst` accumulates the mismatch count.
        Uses 4 scoped scratch registers and leaves `__cursor` untouched, so a
        following `store_sample_n()` commits the frame."""
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
            # byte_r doubles as the loop-exit flag, keeping the verb at 4
            # scratch registers.
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
    # A multiply yields a 64-bit value; that width propagates through + - // and
    # shifts. Wide values live in work-buffer slots, narrow ones in registers.

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
        """`node` to (work_off, owned), widening a narrow node with `CVT64`.
        owned=True is a fresh temp the caller may free."""
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
        """Compile an already-wide `node` to (work_off, owned); the caller
        ensures `_is_wide(node)`. `_to_wide` widens a possibly-narrow node."""
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
        low 32 bits (temp slot freed), a narrow value goes via the register path."""
        if self._is_wide(node):
            off, owned = self._eval_wide(node)
            r = self._regs.get("__narrow_scratch")
            self._em.emit(Op.TRUNC64, off, r)
            if owned:
                self._driver._work.free(off)
            return r
        return self._eval_narrow_reg(node)

    def _emit_frame_burst(self, start_reg: int, num_words: int):
        """Emit a FRAME-aware pipelined read of N consecutive registers. Output
        packs at [0, num_words * dw); one rotating TX/RX slot sits at the buffer
        tail, and each RX is MEMCPY'd out before the next TX overwrites it."""
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
        # Inter-frame settle: SLEEP_US below 1 ms, SLEEP_MS at whole
        # milliseconds.
        gap_us = (int(frame.inter_frame_sleep_ms) * 1000
                  + int(frame.inter_frame_sleep_us))
        if gap_us > 0xFFFF:
            raise CompileError(
                f"inter-frame settle {gap_us} µs exceeds the 16-bit sleep "
                f"operand; use a smaller gap")

        # A FRAME declaring crc / status_ok gets each response verified on-device
        # in the slot; a mismatch drops the tick by jumping back to the loop head.
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
                # Some parts need a settle between a read-request frame and the
                # frame that clocks the response out; ROM constants do not.
                if gap_us > 0 and k < num_tx - 1:
                    if gap_us % 1000 == 0:
                        self._em.emit_u16(Op.SLEEP_MS, gap_us // 1000)
                    else:
                        self._em.emit_u16(Op.SLEEP_US, gap_us)

