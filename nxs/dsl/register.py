"""Register-bus drivers (I²C, SPI) and the register tables they load."""

from __future__ import annotations

import csv
import inspect
import os
import struct
import sys
from typing import List, Optional, Tuple

from nxs._generated_constants import NxsDriverImage
from nxs.opcodes import Op
from nxs.dsl.base import SensorDriver
from nxs.dsl.emit import BUS_REGISTER, _TraceReadValue, _load_spec_byte
from nxs.dsl.errors import CompileError
from nxs.dsl.loop import _ASTCompiler


class RegisterDriver(SensorDriver):
    """Base class for sensors on a register bus (I²C, SPI). Without ``FRAME``,
    reads and writes emit ``OP_REG_READ`` / ``OP_REG_WRITE``; with a ``SpiFrame``
    schema they emit the exact on-wire bytes via MEMCPY_IMM + REG_XFER."""

    BUS_KIND = BUS_REGISTER

    # Physical buses the silicon supports; the first entry is the default when
    # no `bus=` is passed at upload. Narrow to one entry for single-bus parts.
    BUSES = ('i2c', 'spi')

    # Override in a subclass to declare non-standard SPI framing.
    FRAME = None

    # Co-resident I2C slaves: name to {'addr', 'who_am_i_reg', 'who_am_i_values'}
    # or a 'who_am_i_skip_reason'. Reached via dev=; requires BUSES = ('i2c',).
    I2C_COMPANIONS = {}

    def write(self, reg: int, val: int, param: Optional[tuple] = None,
              dev=None):
        if self.FRAME is None:
            # REG_WRITE is [opcode, reg_lo, reg_hi, val]: the value byte at +3 is
            # the patch site. The owner key carries the device for a dev= write.
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

        # FRAME path: compose the full wire bytes at trace time, MEMCPY_IMM
        # them into sample_buf, then REG_XFER to clock them out.
        frame_bytes = self.FRAME.compose(rw=1, addr=reg, data=val)
        # The frame's first byte lands at sample_buf[0]. Offset of that
        # byte inside the bytecode = current offset + 3 (MEMCPY_IMM header).
        frame_byte_off = self._emitter._current_offset() + 3
        self._emitter.emit(Op.MEMCPY_IMM, 0, len(frame_bytes), *frame_bytes)
        self._emitter.emit(Op.REG_XFER, 0, 0, len(frame_bytes))
        if param is not None:
            # The whole frame (CRC and all) is one little-endian u32 patch
            # site, so the firmware lays the bytes back down MSB-first.
            frame_u32 = int.from_bytes(frame_bytes, "little")
            self._patch_recorder_fn(
                param[0], param[1], frame_u32, frame_byte_off, size=4, reg=reg)

    def write_modify(self, reg: int, set_bits: int = 0, clear_bits: int = 0,
                     param: Optional[tuple] = None):
        """On-device read-modify-write of a FRAME register: AND out
        `clear_bits`, OR in `set_bits`, write back with the CRC recomputed.
        With `param=`, `clear_bits` must be the whole field mask."""
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
        # AND ~clear then OR set, so overlap is intended for a param= write;
        # without param= disjoint masks are the convention.
        if param is None and (set_bits & clear_bits):
            raise CompileError(
                f"write_modify: set_bits and clear_bits overlap on "
                f"0x{set_bits & clear_bits:04X}; a bit cannot be both")
        # A param= write patches only the OR immediate, so clear_bits must
        # cover every value's set bits.
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
                # Fixed AND then OR, always both, so the OR immediate (byte +2 of
                # emit_cmp's <BBLB) sits at the same offset for every value.
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
        """Emit a runtime bit-serial CRC over sample_buf[start .. start+length)
        into r[crc_reg], per FRAME.crc; MSB-first per byte, as `Crc.compute`."""
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
        """Emit a FRAME register read into `dst`: stage the request, XFER it
        plus the read-pipeline dummies, then load the data field. Shared by the
        trace path and the measure-body compiler."""
        req = self.FRAME.compose(rw=0, addr=reg, data=0x0000)
        dummy = self.FRAME.compose(rw=0, addr=0, data=0x0000)
        fw = len(req)
        pipeline = max(0, int(self.FRAME.read_pipeline))

        # Stage + issue the initial request (RX into buf[0..fw)).
        self._emitter.emit(Op.MEMCPY_IMM, 0, fw, *req)
        self._emitter.emit(Op.REG_XFER, 0, 0, fw)
        # Pipeline dummies clock each response out. No inter-frame gap: single
        # reads target always-ready constants; the burst path gates its own.
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
        # Plain reads are the base SensorDriver.read; RegisterDriver adds only
        # the custom-FRAME path below.
        if self.FRAME is None:
            return super().read(reg, width, signed, endian, dev=dev)
        if dev is not None:
            raise CompileError(
                "dev= targets an I2C companion; a FRAME driver is SPI and "
                "declares no companions.")

        # FRAME path with optional pipelining: read_pipeline=N means the
        # response to request K arrives during request K+N.
        self._emit_frame_read(reg, self._regs.get(f"__reg_read_{reg}"),
                              signed, endian)

        responses = self._read_responses.get(reg)
        val = responses.pop(0) if responses else 0
        if self._trace_phase == "configure":
            return _TraceReadValue(reg)
        return val

    def xfer(self, word: int, width: int = 2):
        """Clock a literal full-duplex SPI word, one CS assertion per word; the
        response replaces it in the staging slot. In assignment position the
        response loads as an unsigned MSB-first value of the full `width`."""
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
                "read_burst with a FRAME schema is not supported; "
                "use individual self.read(...) calls")
        addr = self._companion_addr(dev) if dev is not None else None
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, addr)
        self._emitter.emit_reg(Op.REG_READ_BURST, reg, count, 0)
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, 0)
        return bytes(count)

    def _write_burst(self, reg: int, payload: bytes, dev=None) -> None:
        """Burst-write `payload` to consecutive registers from `reg`, staged
        through sample_buf then OP_REG_WRITE_BURST (`write_table`'s runs)."""
        if self.FRAME is not None:
            raise CompileError(
                "write_table with a FRAME schema is not supported; write the "
                "framed registers one by one with self.write(...)")
        if len(payload) > _ASTCompiler.VM_SAMPLE_BUF_SIZE:
            raise CompileError(
                f"write_table run of {len(payload)} B exceeds the "
                f"{_ASTCompiler.VM_SAMPLE_BUF_SIZE}-byte sample buffer")
        addr = self._companion_addr(dev) if dev is not None else None
        self._emitter.emit(Op.MEMCPY_IMM, 0, len(payload), *payload)
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, addr)
        self._emitter.emit_reg(Op.REG_WRITE_BURST, reg, 0, len(payload))
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, 0)

    def poll(self, reg: int, mask: int, value: int, timeout_ms: int,
             poll_ms: int = 20, ne: bool = False, soft: bool = False,
             dev=None) -> None:
        """Read `reg` every `poll_ms` until `(read & mask) == value` (`ne`:
        until it differs) or `timeout_ms` passes; a NAK counts as not yet.
        A timeout faults the run, or with `soft` raises the run's soft-miss
        flag and continues. The value width is the I²C profile's
        `data_width`. `dev` names a declared companion to poll instead of
        the primary. Compiles to one `POLL_REG`, bracketed by the retarget
        for a companion."""
        if self.FRAME is not None:
            raise CompileError(
                "poll with a FRAME schema is not supported; POLL_REG reads "
                "with the plain register framing")
        width_bits = 8 * self._i2c_profile().data_width
        limit = (1 << width_bits) - 1
        for name, operand in (("mask", mask), ("value", value)):
            if not isinstance(operand, int) or not 0 <= operand <= limit:
                raise CompileError(
                    f"poll: {name} 0x{operand:X} does not fit the "
                    f"{width_bits}-bit register value the profile declares")
        if not 1 <= timeout_ms <= 0xFFFF:
            raise CompileError(
                f"poll: timeout_ms must be 1..65535, got {timeout_ms!r}")
        if not 1 <= poll_ms <= 0xFFFF:
            raise CompileError(
                f"poll: poll_ms must be 1..65535, got {poll_ms!r}")
        if poll_ms > timeout_ms:
            raise CompileError(
                f"poll: poll_ms {poll_ms} exceeds timeout_ms {timeout_ms}; "
                f"the first read would already be the last")
        self._emitter.check_reg(reg)
        flags = ((NxsDriverImage.POLL_FLAG_NE if ne else 0)
                 | (NxsDriverImage.POLL_FLAG_SOFT if soft else 0))
        addr = self._companion_addr(dev) if dev is not None else None
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, addr)
        self._emitter.emit(*struct.pack("<BHIIHHB", Op.POLL_REG, reg, mask,
                                        value, timeout_ms, poll_ms, flags))
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, 0)

    def load_table(self, path: str) -> List[Tuple[int, int]]:
        """The `(reg, value)` rows of a register table beside the driver's
        source file (an absolute path is used as is): a YAML list of
        `[reg, value]` pairs or `{reg, value}` mappings, a YAML `reg: value`
        mapping, or a CSV of `reg,value` rows (a header row and `#` lines
        are skipped; hex and decimal both read). Compile-time only."""
        return load_table(path, relative_to=_source_dir(type(self)))

    def write_table(self, rows, dev=None) -> None:
        """Write `(reg, value)` rows in order: a run of consecutive registers
        becomes one `REG_WRITE_BURST` (chunked to the sample buffer), a lone
        8-bit value a `REG_WRITE`; values wider than a byte (the profile's
        `data_width`) always go by burst, packed in the profile's byte
        order. A `(SLEEP_ROW, ms)` row (a table's `{sleep_ms: N}` marker)
        ends the run and sleeps."""
        rows = [(r if r == SLEEP_ROW else int(r), int(v)) for r, v in rows]
        prof = self._i2c_profile()
        width = prof.data_width
        limit = (1 << (8 * width)) - 1
        for reg, val in rows:
            if reg == SLEEP_ROW:
                continue
            self._emitter.check_reg(reg)
            if not 0 <= val <= limit:
                raise CompileError(
                    f"write_table: value 0x{val:X} at register 0x{reg:X} "
                    f"does not fit the {8 * width}-bit register value the "
                    f"profile declares")
        max_regs = _ASTCompiler.VM_SAMPLE_BUF_SIZE // width
        run: List[Tuple[int, int]] = []

        def flush():
            if not run:
                return
            start = run[0][0]
            if len(run) == 1 and width == 1:
                self.write(start, run[0][1], dev=dev)
            else:
                payload = b"".join(v.to_bytes(width, prof.byte_order)
                                   for _, v in run)
                self._write_burst(start, payload, dev=dev)
            run.clear()

        for reg, val in rows:
            if reg == SLEEP_ROW:
                flush()
                self.sleep_ms(val)
                continue
            if run and (reg != run[-1][0] + 1 or len(run) >= max_regs):
                flush()
            run.append((reg, val))
        flush()


#: The register slot of a table row that is a settle, not a write: a
#: `{sleep_ms: N}` marker in a register table.
SLEEP_ROW = "sleep_ms"


def _source_dir(cls) -> Optional[str]:
    """Directory of the file defining `cls`, or None for a class with no
    source file (built with exec)."""
    module = sys.modules.get(cls.__module__)
    src = getattr(module, "__file__", None)
    if not src:
        try:
            src = inspect.getsourcefile(cls)
        except (TypeError, OSError):
            src = None
    return os.path.dirname(os.path.abspath(src)) if src else None


def _table_int(text, where: str) -> int:
    """An int from a YAML scalar or a CSV field (`0x..` hex or decimal)."""
    if isinstance(text, bool) or text is None:
        raise CompileError(f"{where}: {text!r} is not a register or value")
    if isinstance(text, int):
        value = text
    else:
        try:
            value = int(str(text).strip(), 0)
        except ValueError:
            raise CompileError(
                f"{where}: {text!r} is not a register or value") from None
    if value < 0:
        raise CompileError(f"{where}: {value} is negative")
    return value


def load_table(path: str, relative_to: Optional[str] = None) -> List[Tuple[int, int]]:
    """The `(reg, value)` rows of a register table file (see
    `RegisterDriver.load_table`); a relative `path` resolves against
    `relative_to`. A YAML `{sleep_ms: N}` row is a settle between writes
    and loads as `(SLEEP_ROW, N)`. Every row is validated with the file
    and row named."""
    if not os.path.isabs(path):
        if relative_to is None:
            raise CompileError(
                f"load_table({path!r}): the driver has no source file to "
                f"resolve a relative path against; pass an absolute path")
        path = os.path.join(relative_to, path)
    if not os.path.exists(path):
        raise CompileError(f"load_table: {path} does not exist")
    ext = os.path.splitext(path)[1].lower()
    rows: List[Tuple[int, int]] = []
    if ext in (".yaml", ".yml"):
        import yaml
        try:
            with open(path, encoding="utf-8") as fh:
                doc = yaml.safe_load(fh)
        except yaml.YAMLError as e:
            raise CompileError(f"load_table: {path}: not valid YAML: {e}") from None
        if doc is None:
            doc = []                    # an empty file: no rows
        if isinstance(doc, dict):
            entries = list(doc.items())
        elif isinstance(doc, list):
            entries = []
            for i, row in enumerate(doc):
                where = f"{path} row {i + 1}"
                if isinstance(row, dict) and set(row) == {SLEEP_ROW}:
                    ms = _table_int(row[SLEEP_ROW], where)
                    if ms == 0:
                        raise CompileError(f"{where}: a sleep_ms row sleeps 0 ms")
                    entries.append((SLEEP_ROW, ms))
                elif isinstance(row, dict) and {"reg", "value"} <= set(row):
                    entries.append((row["reg"], row["value"]))
                elif isinstance(row, (list, tuple)) and len(row) == 2:
                    entries.append((row[0], row[1]))
                else:
                    raise CompileError(
                        f"{where}: expected [reg, value], {{reg, value}} or "
                        f"{{sleep_ms: N}}, got {row!r}")
        else:
            raise CompileError(
                f"load_table: {path}: expected a list of [reg, value] rows or "
                f"a reg: value mapping")
        for i, (reg, val) in enumerate(entries):
            where = f"{path} row {i + 1}"
            if reg == SLEEP_ROW:
                rows.append((SLEEP_ROW, val))
                continue
            rows.append((_table_int(reg, where), _table_int(val, where)))
    elif ext == ".csv":
        with open(path, encoding="utf-8", newline="") as fh:
            for i, fields in enumerate(csv.reader(fh)):
                where = f"{path} line {i + 1}"
                fields = [f.strip() for f in fields]
                if not fields or not fields[0] or fields[0].startswith("#"):
                    continue
                if len(fields) != 2:
                    raise CompileError(
                        f"{where}: expected reg,value; got {len(fields)} fields")
                try:
                    row = (_table_int(fields[0], where), _table_int(fields[1], where))
                except CompileError:
                    # The one non-numeric row allowed is a header before
                    # any data row.
                    if rows:
                        raise
                    continue
                rows.append(row)
    else:
        raise CompileError(
            f"load_table: {path}: a register table is a .yaml, .yml or .csv file")
    if not rows:
        raise CompileError(f"load_table: {path} holds no rows")
    return rows


