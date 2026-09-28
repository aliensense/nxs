"""Command-response I²C drivers, byte-stream (UART) drivers, and the checksum descriptors."""

from __future__ import annotations

from typing import Optional

from nxs.opcodes import Op
from nxs.dsl.base import SensorDriver
from nxs.dsl.emit import BUS_REGISTER, BUS_STREAM, OpErrorCode, TracedSlice
from nxs.dsl.errors import CompileError
from nxs.dsl.loop import _ASTCompiler


class I2cCommandDriver(SensorDriver):
    """Base class for command-response I²C sensors (a barometer's CONVERT
    and ADC READ): `send_command` writes a raw command, `sleep_ms` waits
    the conversion, `read` fetches the result behind its read opcode, and
    the measure loop compiles as a register driver's does."""

    BUS_KIND = BUS_REGISTER

    # A raw command (bytes, STOP, wait, read) has no SPI equivalent, so I²C
    # is the only legal transport.
    BUSES = ('i2c',)

    def send_command(self, cmd_bytes):
        """Emit MEMCPY_IMM + BUS_WRITE_RAW for a raw I²C command: a single
        command byte (`send_command(0x1E)`) or a bytes-like sequence."""
        cmd = bytes([cmd_bytes]) if isinstance(cmd_bytes, int) else bytes(cmd_bytes)
        self._emitter.emit(Op.MEMCPY_IMM, 0, len(cmd), *cmd)
        self._emitter.emit(Op.BUS_WRITE_RAW, 0, len(cmd))

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
        """Write to UART TX: `write(int)` clocks one byte via OP_UART_WRITE;
        `write(bytes)` stages the buffer at sample_buf[0] and clocks it as one
        frame via OP_UART_WRITE_RAW, clobbering sample_buf[0..len) at runtime."""
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

    # ── Staged-frame helpers ─────────────────────────────────
    # `stage()` stages a TX frame (recording a patch site), `compute_checksum()`
    # recomputes its checksum at runtime, `send_staged()` clocks it out.

    def stage(self, frame: bytes,
              patch: Optional[tuple] = None) -> None:
        """Stage `frame` at sample_buf[0..len) via OP_MEMCPY_IMM. `patch` =
        `(name, requested_value, encoded_value, frame_offset, size)` records a
        patch site at the frame byte `frame_offset`, `size` bytes wide."""
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
    # Each compiles to a bytecode loop over the RISC primitives; the byte cursor
    # lives in a scratch register and `store_sample_n()` commits that many bytes.

    def read_until(self, delim, max: int = _ASTCompiler.VM_SAMPLE_BUF_SIZE,
                   timeout_ms: Optional[int] = None) -> None:
        """Read UART bytes into sample_buf until the `delim` (int or bytes, no
        self-similar prefix) or `max` bytes; `__cursor` holds the count for
        `store_sample_n()`. `timeout_ms` (1..65535) raises OP_ERROR TIMEOUT."""
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

        # Delims with a self-similar prefix need KMP-style backtracking, which
        # the compiled loop does not do.
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

        # Scoped scratch: every `__*` register below is dead at L_FOUND.
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
                # Multi-byte delim: `pg` counts delim bytes matched at the tail;
                # a mismatch resets it, checking a restart against delim[0].
                pg_r = self._regs.get("__match_pg")
                em.emit_u32(Op.LOAD_IMM, pg_r, 0)

                em.label(L_TOP)
                em.emit(Op.UART_AVAIL, avail_r)
                em.emit_jmp(Op.JZ, L_IDLE, avail_r)
                em.emit(Op.UART_READ_REG, 1, cursor_r)
                em.emit(Op.LOAD_U8_REG, byte_r, cursor_r)
                em.emit_cmp(Op.ADD, cursor_r, 1, cursor_r)

                # Dispatch on pg; it is always in [0, len(delim_bytes)) so the
                # trailing JMP is defensive only.
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
                em.emit(Op.ERROR, OpErrorCode.TIMEOUT)
            em.label(L_FOUND)

    def stamp_frame(self) -> None:
        """Set the pass's acquisition bound to the RX backlog's first-byte
        arrival. Place at the frame-sync point; the bound equals the frame's
        first byte only while the loop drains the backlog every pass."""
        self._emitter.emit(Op.ACQ_FRAME)

    def read_n(self, count: int,
               timeout_ms: Optional[int] = None) -> TracedSlice:
        """Read exactly `count` bytes from UART into sample_buf[0..count),
        yielding via SLEEP_MS until they arrive; `timeout_ms` (1..65535) raises
        OP_ERROR TIMEOUT. Returns a `TracedSlice` for `.expect(...)`."""
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

        # Scoped scratch: the cursor, avail, match, and timer slots are dead at
        # L_DONE, so the next helper reuses the same register indices.
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
                em.emit(Op.ERROR, OpErrorCode.TIMEOUT)
            em.label(L_DONE)
        return TracedSlice(driver=self, buf_off=0, length=count)

    def store_sample(self) -> None:
        """Commit one fixed-length sample (the compile-time `set_sample_size`
        width); the pairing for a `read_n` record."""
        self._emitter.emit(Op.STORE_SAMPLE)

    def store_sample_n(self) -> None:
        """Commit `__cursor` bytes of sample_buf as one sample: the width an
        immediately preceding `read_until` captured. Intervening `match` /
        `verify_checksum` ops may overwrite `__cursor`."""
        cursor_r = self._regs.get("__cursor")
        self._emitter.emit(Op.STORE_SAMPLE_N, cursor_r)

    def compute_checksum(self, spec: "ChecksumDescriptor",
                         start_off: int, length: int, dst_off: int) -> None:
        """Compile a checksum loop over sample_buf[start_off:start_off+length]
        per `spec` (`ChecksumFletcher`)
        and store the result bytes at sample_buf[dst_off:]. All args immediate."""
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
# Each compiles to a bytecode loop that runs on the VM, so a patched param's
# checksum is right again on the next reload with no per-vendor opcode.

class ChecksumDescriptor:
    """Abstract base; subclasses implement `_emit(driver, start_off, length,
    dst_off)` to inject the checksum bytecode into the driver's emitter."""

    def _emit(self, driver, start_off: int, length: int, dst_off: int) -> None:
        raise NotImplementedError


class ChecksumFletcher(ChecksumDescriptor):
    """Two-byte Fletcher-8: CK_A = sum(byte_i) & 0xFF, CK_B = sum(CK_A after
    each byte) & 0xFF (UBX, TCP/IP, NTP). Writes `[CK_A, CK_B]` at `dst_off`."""

    def _emit(self, driver, start_off: int, length: int, dst_off: int) -> None:
        em = driver._emitter
        L_TOP = driver._fresh_label("ck_fl_top")
        L_DONE = driver._fresh_label("ck_fl_done")

        # Scoped scratch: five registers, dead once the loop finishes; without
        # the scope a probe's read_n + .expect names would push past 8.
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

