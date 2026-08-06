"""Tests for the nxs compiler, descriptor, and driver."""

import sys
import os
import struct
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from nxs.opcodes import Op
from nxs.compiler import (
    RegisterDriver, Sample, SensorDriver, StreamDriver, CompileError,
    ChecksumFletcher, ChecksumXorFold, ChecksumPolynomial,
    TracedSlice, ERR_CODE_TIMEOUT, ERR_CODE_MISMATCH,
)
from nxs.disassembler import disassemble
from nxs.descriptor import load_driver, parse_sample, parse_sample_raw


# ── Basic opcode tests ──────────────────────────────────────

def test_spi_write_emits_correct_bytes():
    class WriteDriver(RegisterDriver):
        def probe(self):
            self.write(0x6B, 0x80)
    drv = WriteDriver()
    result = drv.compile()
    # REG_WRITE carries a 16-bit little-endian reg operand: [op, reg_lo, reg_hi, val].
    assert result.bytecode[:4] == bytes([Op.REG_WRITE, 0x6B, 0x00, 0x80])


def test_spi_read_emits_correct_bytes():
    class ReadDriver(RegisterDriver):
        def probe(self):
            self.read(0x75)
    drv = ReadDriver()
    result = drv.compile()
    assert result.bytecode[0] == Op.REG_READ
    assert result.bytecode[1] == 0x75


def test_sleep_ms_emits_le_u16():
    class SleepDriver(RegisterDriver):
        def configure(self, config):
            self.sleep_ms(40)  # below chunk threshold, single SLEEP_MS
    drv = SleepDriver()
    result = drv.compile()
    assert result.bytecode[0] == Op.SLEEP_MS
    assert struct.unpack_from("<H", result.bytecode, 1)[0] == 40


def test_set_sample_size():
    class SizedDriver(RegisterDriver):
        def configure(self, config):
            self.set_sample_size(20)
    drv = SizedDriver()
    result = drv.compile()
    assert bytes([Op.SET_SAMPLE_SIZE, 20]) in result.bytecode
    assert result.sample_size == 20


# ── Comparison + arithmetic grammar (measure loop) ──────────

def _disasm(bytecode):
    return "\n".join(disassemble(bytecode, print_fn=lambda *_: None))


def test_measure_eq_emits_cmp_eq():
    class C(RegisterDriver):
        def configure(self, config):
            self.set_sample_size(1)

        @SensorDriver.measure_loop(trigger="drdy")
        def measure(self):
            st = self.read(0x00)
            if st == 0xAB:
                return None
            return Sample(st)
    text = _disasm(C().compile().bytecode)
    assert "CMP_EQ" in text


def test_measure_lt_and_gt_lower_to_cmp_lt():
    class C(RegisterDriver):
        def configure(self, config):
            self.set_sample_size(1)

        @SensorDriver.measure_loop(trigger="drdy")
        def measure(self):
            x = self.read(0x00)
            if x < 100:
                return None
            if x > 200:  # > lowers to CMP_LT against rhs+1
                return None
            return Sample(x)
    text = _disasm(C().compile().bytecode)
    assert text.count("CMP_LT") >= 2


def test_measure_arithmetic_assignment_emits_alu():
    class C(RegisterDriver):
        def configure(self, config):
            self.set_sample_size(1)

        @SensorDriver.measure_loop(trigger="drdy")
        def measure(self):
            x = self.read(0x00)
            y = x - 5      # SUB immediate
            z = y & 0x0F   # AND immediate
            return Sample(z)
    text = _disasm(C().compile().bytecode)
    assert "SUB" in text and "AND" in text


# ── Canonical SI units (semantic SSOT) ───────────────────────

def _compile_single_field(field: dict):
    class One(RegisterDriver):
        def configure(self, config):
            self.set_output([field])

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            x = self.read(0x00)
            return Sample(x)
    return One().compile()


def test_si_semantic_inherits_canonical_unit():
    cd = _compile_single_field({'name': 'temp', 'scale': 0.01})
    assert cd.output_fields[0]['unit'] == 'kelvin'


def test_si_semantic_rejects_conflicting_unit():
    import pytest
    from nxs.compiler import CompileError
    with pytest.raises(CompileError, match="canonical SI unit"):
        _compile_single_field({'name': 'temp', 'scale': 0.01,
                               'unit': 'celsius'})


def test_si_semantic_accepts_matching_unit():
    cd = _compile_single_field({'name': 'temp', 'scale': 0.01,
                                'unit': 'kelvin'})
    assert cd.output_fields[0]['unit'] == 'kelvin'


def test_generic_field_keeps_authored_unit():
    cd = _compile_single_field({'name': 'my_custom_state', 'scale': 1.0,
                                'unit': 'furlongs'})
    assert cd.output_fields[0]['semantic'] == 0  # GENERIC
    assert cd.output_fields[0]['unit'] == 'furlongs'


def test_generic_field_requires_declared_unit():
    import pytest
    from nxs.compiler import CompileError
    with pytest.raises(CompileError, match="declare 'unit'"):
        _compile_single_field({'name': 'my_custom_state', 'scale': 1.0})


def test_generic_field_accepts_explicit_unitless():
    cd = _compile_single_field({'name': 'my_custom_state', 'scale': 1.0,
                                'unit': ''})
    assert cd.output_fields[0]['unit'] == ''


def test_non_si_semantic_keeps_authored_unit():
    cd = _compile_single_field({'name': 'humidity', 'scale': 0.01,
                                'unit': '%RH'})
    assert cd.output_fields[0]['unit'] == '%RH'


def test_range_param_bad_shape_rejected():
    """A range param carries exactly [min, max]; any other shape must fail
    at compile, not truncate silently into a bogus bound."""
    import pytest
    from nxs.compiler import CompileError

    class Ranged(RegisterDriver):
        def configure(self, config):
            self.declare_param("trim", values=[0, 100, 1],
                               default=0, param_type="range", kind="live")
            self.set_sample_size(1)

        @SensorDriver.measure_loop(trigger="drdy")
        def measure(self):
            x = self.read(0x00)
            return Sample(x)

    with pytest.raises(CompileError, match="exactly"):
        Ranged().compile()


def test_param_type_typo_rejected_plainly():
    """A typo'd param_type gets a must-be-enum error, not 'reserved'."""
    import pytest
    from nxs.compiler import CompileError

    class Typoed(RegisterDriver):
        def configure(self, config):
            self.declare_param("trim", values=[0, 1],
                               default=0, param_type="enun")
            self.set_sample_size(1)

        @SensorDriver.measure_loop(trigger="drdy")
        def measure(self):
            x = self.read(0x00)
            return Sample(x)

    with pytest.raises(CompileError, match="must be 'enum'"):
        Typoed().compile()


def test_wide_compensation_lowers_to_int64_ops():
    # A compensation-style driver: a 16-bit coefficient read in configure()
    # binds a wide work-buffer value; the measure loop multiplies it by a raw
    # field and scales — plain Python, no width markers. The compiler must lower
    # the multiply to int64 ops and write the computed int32 via Sample(field=).
    class Baro(RegisterDriver):
        def configure(self, config):
            self.k = self.read(0xA2, 2)   # coefficient → WideRef in the work buffer
            self.set_output([
                {'name': 'pressure', 'type': 'int32', 'scale': 1.0},
            ])

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            d = self.read(0x00, 3)        # 24-bit raw → narrow register
            p = self.k * d // 2**4        # wide: MUL64 then SHR64(>>4)
            return Sample(pressure=p)

    text = _disasm(Baro().compile().bytecode)
    for op in ("CVT64", "MUL64", "SHR64", "TRUNC64", "STORE_SAMPLE"):
        assert op in text, f"{op} missing from:\n{text}"
    assert "SET_SAMPLE_SIZE" in text  # int32 field → size 4


def test_sample_field_mismatch_is_a_hard_error():
    class Baro(RegisterDriver):
        def configure(self, config):
            self.set_output([
                {'name': 'pressure', 'type': 'int32', 'scale': 1.0},
            ])

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            d = self.read(0x00, 3)
            return Sample(temperature=d)  # names no declared field

    try:
        Baro().compile()
        assert False, "expected CompileError for unknown Sample field"
    except CompileError:
        pass


def test_second_order_branch_with_wide_compare_and_inplace_update():
    # The MS5611 second-order shape: a computed (wide) value in the branch test
    # (`if temp < 2000`), a wide square (`f*f`), and an *in-place* reassignment
    # of a wide local inside the branch (`off = off - f`). All plain Python,
    # lowered by the general width-tracking + _compile_if — no new opcode.
    class C(RegisterDriver):
        def configure(self, config):
            self.k = self.read(0xA2, 2)
            self.set_output([{'name': 'out', 'type': 'int32', 'scale': 1.0, 'unit': ''}])

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            d = self.read(0x00, 3)
            off = self.k * d // 2**7
            temp = self.k * d // 2**23
            if temp < 2000:
                f = (temp - 2000) * (temp - 2000)   # wide square
                off = off - f                        # in-place reassign in branch
            return Sample(out=off)

    text = _disasm(C().compile().bytecode)
    assert "TRUNC64" in text  # wide `temp` truncated for the compare
    assert "CMP_LT" in text   # the branch test
    assert "MUL64" in text    # the square


# ── Measure loop tests ──────────────────────────────────────

class SimpleMeasureDriver(RegisterDriver):
    def configure(self, config):
        self.set_sample_size(4)

    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        raw = self.read_burst(0x3B, 4)
        return Sample(raw)


def test_measure_loop_drdy_emits_yield():
    drv = SimpleMeasureDriver()
    result = drv.compile()
    assert Op.YIELD in result.bytecode
    assert Op.STORE_SAMPLE in result.bytecode


class PollMeasureDriver(RegisterDriver):
    def configure(self, config):
        self.set_sample_size(4)

    @SensorDriver.measure_loop(trigger="poll", sample_rate=200)
    def measure(self):
        raw = self.read_burst(0x3B, 4)
        return Sample(raw)


def test_measure_loop_poll_emits_sleep():
    drv = PollMeasureDriver()
    result = drv.compile()
    assert Op.YIELD not in result.bytecode
    assert Op.SLEEP_MS in result.bytecode


# ── Conditional branch tests ────────────────────────────────

class ConditionalDriver(RegisterDriver):
    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        status = self.read(0x3A)
        if not (status & 0x01):
            return None
        raw = self.read_burst(0x3B, 14)
        return Sample(raw)


def test_conditional_return_none_emits_jmp_to_yield():
    drv = ConditionalDriver()
    result = drv.compile()
    assert Op.AND in result.bytecode
    assert Op.JNZ in result.bytecode


# ── Folder-discovered compile sweep ─────────────────────────

def _discover_driver_names():
    """Every driver module under `nxs/drivers`, so the sweep below picks
    up a new driver the moment its file lands and drops one the moment
    it's deleted — no hand-maintained list to fall out of sync."""
    import pkgutil
    import nxs.drivers
    return sorted(info.name
                  for info in pkgutil.iter_modules(nxs.drivers.__path__)
                  if not info.name.startswith("_"))


def test_every_driver_compiles():
    """Each shipped driver compiles on its defaults and on every bus it
    declares. Discovered from the folder, so adding or removing a driver
    needs no edit here; a driver that stops compiling names itself in the
    failure."""
    names = _discover_driver_names()
    assert names, "no drivers discovered under nxs.drivers"
    for name in names:
        cls = load_driver(name)
        try:
            cls().compile({})
        except Exception as e:
            raise AssertionError(
                f"driver {name!r} ({cls.__name__}) failed to compile: {e}") from e
        for bus in (getattr(cls, "BUSES", None) or ()):
            try:
                cls().compile({"bus": bus})
            except Exception as e:
                raise AssertionError(
                    f"driver {name!r} ({cls.__name__}) failed to compile "
                    f"for bus {bus!r}: {e}") from e


# ── Config-driven compilation ───────────────────────────────

def test_iam20680_compiles_with_config():
    from nxs.drivers.iam20680 import Iam20680
    config = {'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'drdy'}
    drv = Iam20680()
    result = drv.compile(config)
    assert len(result.bytecode) > 0
    # Compact sanity bound (hard limit is 2048); the tiered FIFO gate is
    # ~200 B — well clear, but larger than the old strict-flush body.
    assert len(result.bytecode) < 256
    assert result.sample_size == 14
    assert result.name == "Iam20680"
    assert result.config == config


def test_iam20680_output_fields():
    from nxs.drivers.iam20680 import Iam20680
    config = {'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'drdy'}
    result = Iam20680().compile(config)

    assert len(result.output_fields) == 7
    assert result.output_fields[0]['name'] == 'accel_x'
    assert result.output_fields[0]['unit'] == 'm/s^2'
    assert result.output_fields[3]['name'] == 'temp'
    assert result.output_fields[3]['offset'] == 298.15
    assert result.output_fields[4]['name'] == 'gyro_x'
    assert result.output_fields[4]['unit'] == 'rad/s'


def test_iam20680_scales_vary_with_config():
    from nxs.drivers.iam20680 import Iam20680
    from nxs.descriptor import effective_scale_fields

    r8 = Iam20680().compile({'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'drdy'})
    r16 = Iam20680().compile({'sample_rate': 250, 'accel_fs': 16, 'gyro_fs': 2000, 'trigger': 'drdy'})

    # The BASE descriptor scale is range-independent — the range lives in
    # the live param, not the baked scale — and accel_x links to accel_fs.
    assert r8.output_fields[0]['scale'] == r16.output_fields[0]['scale']
    assert r8.output_fields[0]['scale_param'] == 'accel_fs'

    # The EFFECTIVE scale (base × current accel_fs) is what doubles 8g→16g.
    eff8 = effective_scale_fields(r8.output_fields, r8.params)
    eff16 = effective_scale_fields(r16.output_fields, r16.params)
    assert abs(eff16[0]['scale'] / eff8[0]['scale'] - 2.0) < 0.01

    # Gyro is the same range (2000 dps) → identical effective scale.
    assert eff8[4]['scale'] == eff16[4]['scale']

    # Bytecodes should differ (different accel FS register value)
    assert r8.bytecode != r16.bytecode


def test_iam20680_trigger_from_config():
    from nxs.drivers.iam20680 import Iam20680

    drdy = Iam20680().compile({'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'drdy'})
    poll = Iam20680().compile({'sample_rate': 100, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'poll'})

    # Check for the opcode at an instruction boundary, not a raw byte: a
    # YIELD's 0x02 also appears as an operand (e.g. a 2-byte FIFO-count read),
    # so disassemble and look for the mnemonic.
    drdy_ops = "\n".join(disassemble(drdy.bytecode, print_fn=lambda _x: None))
    poll_ops = "\n".join(disassemble(poll.bytecode, print_fn=lambda _x: None))
    assert "YIELD" in drdy_ops
    assert "YIELD" not in poll_ops
    assert "SLEEP_MS" in poll_ops


def test_fractional_enum_values_rejected_at_declaration():
    """Parameter values ride the descriptor wire as int32: a fractional
    enum value would truncate silently (0.25 -> 0, colliding with
    0.5 -> 0) and ship a corrupt allowed set. declare_param rejects it
    loudly instead."""
    class Fractional(RegisterDriver):
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test fixture"

        def configure(self, config):
            self.declare_param("sample_rate", values=[0.5, 1, 2],
                               default=1, unit="Hz")

    try:
        Fractional().compile({})
    except CompileError as e:
        assert "int32" in str(e)
    else:
        raise AssertionError("expected CompileError for a fractional enum")


def test_from_config_registerless_sample_rate_poll_branch_only():
    """A part with no rate register declares `sample_rate` only when the
    requested trigger is poll: the param exists solely as the poll loop's
    SLEEP_MS patch, and a drdy image would have no patch site for it.
    configure() gates on `config.get('trigger')` (absent = drdy default)."""
    class FixedRateSync(RegisterDriver):
        BUSES = ('spi',)
        PINS = {'drdy': 'mkbus_int'}
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test fixture"

        def configure(self, config):
            if config.get('trigger') == 'poll':
                self.declare_param("sample_rate", values=[10, 25, 50, 100],
                                   default=100, unit="Hz")
            self.set_output([{'name': 'raw', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="from_config")
        def measure(self):
            v = self.read(0x00, 2)
            return Sample(raw=v)

    drdy = FixedRateSync().compile({})
    assert "sample_rate" not in [p.name for p in drdy.params]
    assert "YIELD" in _disasm(drdy.bytecode)

    poll = FixedRateSync().compile({'trigger': 'poll'})
    assert "sample_rate" in [p.name for p in poll.params]
    entries = [e for e in poll.patch_map if e.param_name == "sample_rate"]
    assert len(entries) == 1
    assert entries[0].value_map == {10: 100, 25: 40, 50: 20, 100: 10}


def test_iam20680_disassembles():
    from nxs.drivers.iam20680 import Iam20680
    config = {'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'drdy'}
    result = Iam20680().compile(config)
    lines = disassemble(result.bytecode, print_fn=lambda x: None)
    assert len(lines) > 10

    text = "\n".join(lines)
    assert "REG_READ" in text
    assert "REG_WRITE" in text
    assert "STORE_SAMPLE" in text
    assert "YIELD" in text


def test_frame_driver_emits_who_am_i_prologue():
    # A FRAME part gets the same on-device identity check as a register part:
    # a frame read, a compare against the full identity word, then ERROR.
    from nxs.drivers.iim20670 import Iim20670
    ops = _disasm(Iim20670().compile({}).bytecode)
    assert "0xAA55" in ops                       # the full 16-bit identity word
    assert "ERROR" in ops and "code=192" in ops  # WHO_AM_I_MISMATCH_CODE (0xC0)
    # The check runs before the measure loop's first tick (drdy YIELD head).
    assert ops.index("CMP_EQ") < ops.index("YIELD")


def test_disassemble_reg_operands():
    # The reg operand is 16-bit little-endian; the disassembler must combine
    # both bytes and read the value from the shifted offset. Regression: a
    # write of 0x80 printed "= 0x00" when only the low reg byte was read.
    import struct
    from nxs.opcodes import Op
    reg_write = struct.pack("<BH", Op.REG_WRITE, 0x6B) + bytes([0x80])
    text = "\n".join(disassemble(reg_write, print_fn=lambda _x: None))
    assert "[0x6B] = 0x80" in text


# ── Sample parsing ──────────────────────────────────────────

def test_parse_sample():
    from nxs.drivers.iam20680 import Iam20680
    from nxs.descriptor import effective_scale_fields
    config = {'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'drdy'}
    result = Iam20680().compile(config)

    # Simulate a raw sample: accel=[4096, 0, 0] temp=8170 gyro=[0, 0, 0]
    raw = struct.pack('>3hh3h', 4096, 0, 0, 8170, 0, 0, 0)
    # Fold the live accel_fs/gyro_fs into the scale, as the device serves it.
    fields = effective_scale_fields(result.output_fields, result.params)
    values = parse_sample(raw, fields)

    # accel_x at 8g: 4096 * (9.80665/4096) ≈ 9.80665 m/s^2 (1g)
    assert abs(values['accel_x'] - 9.80665) < 0.01

    # temp: 8170 * (1/326.8) + 298.15 ≈ 323.15 K (50 C)
    assert abs(values['temp'] - 323.15) < 0.1

    # gyro should be 0
    assert abs(values['gyro_x']) < 0.001


def test_parse_sample_raw():
    from nxs.drivers.iam20680 import Iam20680
    config = {'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'drdy'}
    result = Iam20680().compile(config)

    raw = struct.pack('>3hh3h', 1234, -567, 890, 4000, -100, 200, -300)
    values = parse_sample_raw(raw, result.output_fields)

    assert values['accel_x'] == 1234
    assert values['accel_y'] == -567
    assert values['gyro_z'] == -300


# ── Driver loading ──────────────────────────────────────────

def test_load_driver():
    cls = load_driver("iam20680")
    assert cls.__name__ == "Iam20680"
    drv = cls()
    result = drv.compile({'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'drdy'})
    assert len(result.bytecode) > 0


# ── Error cases ─────────────────────────────────────────────

def test_bytecode_size_limit():
    from nxs.drivers.iam20680 import Iam20680
    config = {'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'drdy'}
    result = Iam20680().compile(config)
    assert len(result.bytecode) <= 2048


# ── Implicit YIELD before backward branch ──────────────────

from nxs.compiler import _Emitter
from nxs.opcodes import INSTRUCTION_SIZE


def _walk_bc(bytecode: bytes):
    """Yield (offset, opcode, instr_size) for each instruction.
    Handles OP_MEMCPY_IMM's variable-length payload."""
    pos = 0
    while pos < len(bytecode):
        op = bytecode[pos]
        size = INSTRUCTION_SIZE.get(op)
        if size is None:
            raise ValueError(f"unknown opcode 0x{op:02X} at +{pos}")
        if op == Op.MEMCPY_IMM:
            size = 3 + bytecode[pos + 2]
        yield pos, op, size
        pos += size


def _backward_branches(bytecode: bytes):
    """Yield (source_offset, opcode, target_offset) for each backward
    branch (negative relative offset)."""
    for off, op, _size in _walk_bc(bytecode):
        if op == Op.JMP:
            rel = struct.unpack_from("<h", bytecode, off + 1)[0]
        elif op in (Op.JNZ, Op.JZ):
            rel = struct.unpack_from("<h", bytecode, off + 2)[0]
        else:
            continue
        target = off + rel
        if target < off:
            yield off, op, target


def test_emitter_inserts_yield_before_backward_branch_when_neither_yieldable():
    """Source-pred and target both not yieldable → emitter inserts a
    cooperative `SLEEP_US 0`. (Was OP_YIELD originally; switched to
    SLEEP_US because YIELD blocks on DRDY and stream-driver inner
    loops have no DRDY semantic — using YIELD there hits the 1 s
    DRDY-timeout error path on every iteration.)"""
    em = _Emitter()
    em.label("loop")
    em.emit(Op.NOP)              # target opcode = NOP
    em.emit(Op.NOP)              # source-predecessor = NOP
    em.emit_jmp(Op.JMP, "loop")  # backward
    bc = em.build()
    # NOP (1) + NOP (1) + SLEEP_US 0 (3, inserted) + JMP (3) = 8 bytes
    assert len(bc) == 8
    assert bc[0] == Op.NOP
    assert bc[1] == Op.NOP
    assert bc[2] == Op.SLEEP_US
    # next 2 bytes are the u16 LE 0
    assert bc[3] == 0 and bc[4] == 0
    assert bc[5] == Op.JMP


def test_emitter_no_insert_when_target_is_yieldable():
    """Target's first instr is YIELD → next iteration crosses STOP-check
    at the loop top; no redundant YIELD before the JMP. Preserves
    DRDY-mode timing for shipped drivers like IIM-20670."""
    em = _Emitter()
    em.label("loop")
    em.emit(Op.YIELD)            # target = YIELD, yieldable
    em.emit(Op.NOP)
    em.emit_jmp(Op.JMP, "loop")
    bc = em.build()
    # YIELD (1) + NOP (1) + JMP (3) = 5 bytes, no insert.
    assert len(bc) == 5
    assert bc[0] == Op.YIELD
    assert bc[1] == Op.NOP
    assert bc[2] == Op.JMP


def test_emitter_no_insert_when_source_predecessor_is_yieldable():
    """Immediately-preceding instr is SLEEP_MS → branch already has
    STOP-check at its source side; no redundant YIELD."""
    em = _Emitter()
    em.label("loop")
    em.emit(Op.NOP)              # target = NOP, not yieldable
    em.emit_u16(Op.SLEEP_MS, 5)  # source-pred = SLEEP_MS, yieldable
    em.emit_jmp(Op.JMP, "loop")
    bc = em.build()
    # NOP (1) + SLEEP_MS (3) + JMP (3) = 7 bytes, no insert.
    assert len(bc) == 7
    assert bc[0] == Op.NOP
    assert bc[1] == Op.SLEEP_MS
    assert bc[4] == Op.JMP


def test_emitter_forward_branch_does_not_trigger_insert():
    """Forward branches don't loop back; no YIELD insertion regardless
    of context."""
    em = _Emitter()
    em.emit(Op.NOP)
    em.emit_jmp(Op.JMP, "later")
    em.emit(Op.NOP)
    em.label("later")
    em.emit(Op.HALT)
    bc = em.build()
    # NOP (1) + JMP (3) + NOP (1) + HALT (1) = 6 bytes, no YIELD.
    assert len(bc) == 6
    assert Op.YIELD not in bc


def test_every_backward_branch_in_shipped_drivers_has_yield_at_src_or_tgt():
    """Bytecode-level invariant: every backward branch in compiled
    driver bytecode must reach a yieldable opcode (YIELD/SLEEP_MS/
    SLEEP_US) at either its immediate predecessor or its branch target.
    Bounds STOP latency to one tick of the loop. Currently a no-op
    insertion (all shipped drivers have YIELD or SLEEP_MS at the loop
    head); this test fails the moment a future change introduces a
    backward branch without that property."""
    from nxs.drivers.iim20670 import Iim20670
    from nxs.drivers.iam20680 import Iam20680
    from nxs.drivers.mc6470 import Mc6470

    yieldable = {Op.YIELD, Op.SLEEP_MS, Op.SLEEP_US}

    cases = [
        (Iim20670, {'trigger': 'drdy'}),
        (Iim20670, {'trigger': 'poll'}),
        (Iam20680, {'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000,
                    'trigger': 'drdy'}),
        (Mc6470, {}),
    ]

    for cls, config in cases:
        bc = cls().compile(config).bytecode
        instrs = list(_walk_bc(bc))
        op_at_offset = {off: op for off, op, _ in instrs}
        pred = {}
        prev = None
        for off, op, _ in instrs:
            pred[off] = prev
            prev = op

        for src_off, src_op, tgt_off in _backward_branches(bc):
            tgt_op = op_at_offset.get(tgt_off)
            src_pred = pred.get(src_off)
            ok = (src_pred in yieldable) or (tgt_op in yieldable)
            label = f"{cls.__name__} (config={config})"
            assert ok, (
                f"{label}: backward branch at +{src_off} "
                f"(op=0x{src_op:02X}) → target +{tgt_off} "
                f"(op=0x{tgt_op or 0:02X}); predecessor "
                f"0x{src_pred or 0:02X}; neither is yieldable")


# ── Auto-chunked sleep_ms ──────────────────────────────────


def _decode_sleep_ms_sequence(bc: bytes):
    """Walk a bytecode buffer and return the list of (offset, ms) for
    each OP_SLEEP_MS instruction. Other opcodes are skipped via the
    INSTRUCTION_SIZE table."""
    sleeps = []
    pos = 0
    while pos < len(bc):
        op = bc[pos]
        size = INSTRUCTION_SIZE.get(op)
        if size is None:
            break
        if op == Op.MEMCPY_IMM:
            size = 3 + bc[pos + 2]
        if op == Op.SLEEP_MS:
            ms = struct.unpack_from("<H", bc, pos + 1)[0]
            sleeps.append((pos, ms))
        pos += size
    return sleeps


def test_sleep_ms_at_chunk_threshold_does_not_chunk():
    """sleep_ms(N) for N <= SLEEP_CHUNK_MS emits exactly one
    OP_SLEEP_MS(N) — no chunking overhead at small values."""
    class D(RegisterDriver):
        def configure(self, config):
            self.sleep_ms(50)  # exactly at threshold
    bc = D().compile().bytecode
    sleeps = _decode_sleep_ms_sequence(bc)
    assert len(sleeps) == 1
    assert sleeps[0][1] == 50


def test_sleep_ms_above_threshold_chunks_to_50ms_pieces():
    """sleep_ms(N) for SLEEP_CHUNK_MS < N <= SLEEP_MAX_UNROLLED_MS
    expands to multiple OP_SLEEP_MS(50) instructions, each of which
    returns VM_YIELD and bounds STOP latency to one chunk."""
    class D(RegisterDriver):
        def configure(self, config):
            self.sleep_ms(150)  # 50 + 50 + 50, exact multiple
    bc = D().compile().bytecode
    sleeps = _decode_sleep_ms_sequence(bc)
    assert len(sleeps) == 3
    assert all(ms == 50 for _, ms in sleeps)


def test_sleep_ms_handles_non_multiple_via_remainder_chunk():
    """sleep_ms(N) where N is not a multiple of SLEEP_CHUNK_MS emits
    full chunks plus one remainder chunk so the total wait equals N."""
    class D(RegisterDriver):
        def configure(self, config):
            self.sleep_ms(170)  # 50 + 50 + 50 + 20
    bc = D().compile().bytecode
    sleeps = _decode_sleep_ms_sequence(bc)
    assert len(sleeps) == 4
    assert [ms for _, ms in sleeps] == [50, 50, 50, 20]
    assert sum(ms for _, ms in sleeps) == 170


def test_sleep_ms_at_unrolled_cap_succeeds():
    """sleep_ms(SLEEP_MAX_UNROLLED_MS) is the upper edge of the
    auto-chunked range; it expands to (max/chunk) full chunks."""
    class D(RegisterDriver):
        def configure(self, config):
            self.sleep_ms(1000)  # SLEEP_MAX_UNROLLED_MS
    bc = D().compile().bytecode
    sleeps = _decode_sleep_ms_sequence(bc)
    assert len(sleeps) == 20
    assert all(ms == 50 for _, ms in sleeps)


def test_sleep_ms_above_cap_raises_compile_error():
    """sleep_ms beyond SLEEP_MAX_UNROLLED_MS rejects with a clear
    diagnostic, since pure unrolling would exceed VM_MAX_PROGRAM_SIZE
    and the VM lacks a runtime-decrement opcode for a true chunked
    loop. The error message points the author at the likely cause
    (mistaken unit) and at the design constraint."""
    class D(RegisterDriver):
        def configure(self, config):
            self.sleep_ms(60_000)
    try:
        D().compile()
    except CompileError as e:
        msg = str(e).lower()
        assert "exceeds" in msg
        assert "1000" in str(e)  # the cap value
    else:
        raise AssertionError(
            "expected CompileError for sleep_ms(60_000), got no error")


def test_sleep_us_does_not_chunk():
    """sleep_us is bounded at u16 (~65 ms) by opcode design; STOP
    latency is already small enough that chunking is unnecessary."""
    class D(RegisterDriver):
        def configure(self, config):
            self.sleep_us(60_000)  # 60 ms in microseconds
    bc = D().compile().bytecode
    # Single SLEEP_US instruction — no chunking applies.
    assert bc[0] == Op.SLEEP_US
    assert struct.unpack_from("<H", bc, 1)[0] == 60_000


# ── WHO_AM_I_SKIP_REASON enforcement ───────────────────────


def test_explicit_empty_who_am_i_without_skip_reason_raises():
    """A register driver that explicitly declares WHO_AM_I_VALUES = []
    without WHO_AM_I_SKIP_REASON is rejected — leaves no audit trail
    for why the probe was skipped."""
    class NoReason(RegisterDriver):
        WHO_AM_I_REG = 0x00
        WHO_AM_I_VALUES = []  # explicit opt-out, but no reason
    try:
        NoReason().compile()
    except CompileError as e:
        assert "WHO_AM_I_SKIP_REASON" in str(e)
    else:
        raise AssertionError(
            "expected CompileError for WHO_AM_I_VALUES = [] without "
            "WHO_AM_I_SKIP_REASON")


def test_explicit_empty_who_am_i_with_skip_reason_compiles():
    """WHO_AM_I_VALUES = [] plus a non-empty WHO_AM_I_SKIP_REASON is
    the documented opt-out path. Reason is captured in CompiledDriver
    for inspection by nxs tooling."""
    class WithReason(RegisterDriver):
        WHO_AM_I_REG = 0x00
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "No WHO_AM_I register on this IC."
    cd = WithReason().compile()
    assert cd.who_am_i_values == []
    assert cd.who_am_i_skip_reason == "No WHO_AM_I register on this IC."


def test_skip_reason_with_non_empty_values_raises():
    """A driver that sets BOTH WHO_AM_I_VALUES (non-empty) AND
    WHO_AM_I_SKIP_REASON is contradictory — the skip reason only
    applies when actually opting out."""
    class Both(RegisterDriver):
        WHO_AM_I_REG = 0x75
        WHO_AM_I_VALUES = [0xFA]
        WHO_AM_I_SKIP_REASON = "Should not be set"
    try:
        Both().compile()
    except CompileError as e:
        assert "WHO_AM_I_SKIP_REASON is set but WHO_AM_I_VALUES is non-empty" in str(e)
    else:
        raise AssertionError(
            "expected CompileError for both WHO_AM_I_VALUES and "
            "WHO_AM_I_SKIP_REASON set")


def test_blank_skip_reason_treated_as_missing():
    """An empty or whitespace-only WHO_AM_I_SKIP_REASON does not
    satisfy the rule — the reason must carry actual audit content."""
    class BlankReason(RegisterDriver):
        WHO_AM_I_REG = 0x00
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "   "  # whitespace only
    try:
        BlankReason().compile()
    except CompileError as e:
        assert "WHO_AM_I_SKIP_REASON" in str(e)
    else:
        raise AssertionError(
            "expected CompileError for whitespace-only "
            "WHO_AM_I_SKIP_REASON")


def test_undeclared_who_am_i_does_not_trigger_rule():
    """Driver classes that don't declare WHO_AM_I_VALUES anywhere in
    their MRO are exempt from the rule — typical for compiler test
    stubs that exercise isolated paths and aren't real drivers."""
    class Stub(RegisterDriver):
        def probe(self):
            self.write(0x6B, 0x80)
    cd = Stub().compile()
    assert cd.who_am_i_values == []
    assert cd.who_am_i_skip_reason is None


def test_stream_driver_exempt_from_who_am_i_rule():
    """StreamDriver (UART) drivers don't have a WHO_AM_I-style probe;
    the rule does not apply to them regardless of attribute state."""
    class UartStub(StreamDriver):
        WHO_AM_I_VALUES = []  # would trip the rule on a register driver
    cd = UartStub().compile()
    assert cd.who_am_i_skip_reason is None


# ── Patch operand-slot validation ──────────────────────────


def _instruction_bounds_at(bc: bytes, target_offset: int):
    """Find the instruction whose bytes contain `target_offset`.
    Returns (instr_start, opcode, instr_size) or None if target_offset
    is past end-of-program."""
    for off, op, size in _walk_bc(bc):
        if off <= target_offset < off + size:
            return off, op, size
    return None


def _assert_patches_land_in_operand_bytes(cd):
    """Walk every patch entry in a CompiledDriver and assert it lands
    strictly inside the operand region of an emitted instruction
    (i.e., not on the opcode byte and not past the instruction)."""
    bc = cd.bytecode
    for entry in cd.patch_map:
        for value, byte_or_bytes in entry.value_map.items():
            patch_offset = entry.offset
            patch_size = (len(byte_or_bytes)
                          if isinstance(byte_or_bytes, (bytes, list))
                          else 1)
            bounds = _instruction_bounds_at(bc, patch_offset)
            assert bounds is not None, (
                f"{cd.name}: patch entry param={entry.param_name} "
                f"value={value} offset={patch_offset} is past end of "
                f"bytecode ({len(bc)} bytes)")
            instr_start, opcode, instr_size = bounds
            # Patch must NOT touch the opcode byte.
            assert patch_offset > instr_start, (
                f"{cd.name}: patch for param={entry.param_name} value={value} "
                f"at offset {patch_offset} lands ON the opcode byte "
                f"of instruction at +{instr_start} (op=0x{opcode:02X}). "
                f"Patches must hit operand bytes only.")
            # Patch must not extend past the instruction bounds.
            patch_end = patch_offset + patch_size
            instr_end = instr_start + instr_size
            assert patch_end <= instr_end, (
                f"{cd.name}: patch for param={entry.param_name} value={value} "
                f"at offset {patch_offset} size {patch_size} extends "
                f"past instruction at +{instr_start} (op=0x{opcode:02X}, "
                f"size={instr_size}). Patch end {patch_end} > instr "
                f"end {instr_end}.")


def test_iam20680_patches_land_in_reg_write_operand_bytes():
    """RegisterDriver simple-path: every parameter patch (sample_rate,
    accel_fs, gyro_fs) lands on the value byte of an OP_REG_WRITE
    instruction (offset=3, size=1 within the 4-byte instruction — the reg
    operand is a 16-bit little-endian field)."""
    from nxs.drivers.iam20680 import Iam20680
    config = {'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000,
              'trigger': 'drdy'}
    cd = Iam20680().compile(config)
    assert len(cd.patch_map) > 0, "expected param patches"
    _assert_patches_land_in_operand_bytes(cd)
    # Verify all patches land specifically on REG_WRITE operands (offset
    # = instr_start + 2, size = 1) — confirms the simple-path contract
    # rather than just "somewhere in the operand region."
    for entry in cd.patch_map:
        bounds = _instruction_bounds_at(cd.bytecode, entry.offset)
        assert bounds is not None
        instr_start, opcode, _ = bounds
        if entry.param_name == 'bus':
            continue  # `bus` patch is special — handled differently
        assert opcode == Op.REG_WRITE, (
            f"expected REG_WRITE for {entry.param_name}, got 0x{opcode:02X}")
        assert entry.offset == instr_start + 3, (
            f"expected REG_WRITE val byte (start+3), got offset "
            f"{entry.offset} vs instr_start {instr_start}")


def test_baud_patch_lands_in_uart_configure_operand_bytes():
    """StreamDriver path: set_baud() with a param emits UART_CONFIGURE
    (5 bytes: opcode + 4 baud bytes); the patch covers all 4 baud
    bytes (offset = instr_start + 1, size = 4)."""
    class BaudDriver(StreamDriver):
        def configure(self, config):
            self.declare_param("baud", values=[9600, 38400, 115200],
                               default=38400, unit="baud")
            baud = config.get('baud', 38400)
            self.set_baud(baud=baud, param=("baud", baud))
            self.set_output([{'name': 'data', 'type': 'string',
                              'count': 32, 'scale': 1.0, 'unit': ''}])
            self.set_sample_size(32)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            self.read_until(b'\n', max=32)
            self.store_sample_n()

    cd = BaudDriver().compile({})
    baud_entries = [e for e in cd.patch_map if e.param_name == 'baud']
    assert baud_entries, "expected a baud patch entry"
    _assert_patches_land_in_operand_bytes(cd)
    for entry in baud_entries:
        bounds = _instruction_bounds_at(cd.bytecode, entry.offset)
        assert bounds is not None
        instr_start, opcode, _ = bounds
        assert opcode == Op.UART_CONFIGURE, (
            f"expected UART_CONFIGURE for baud, got 0x{opcode:02X}")
        assert entry.offset == instr_start + 1, (
            f"expected UART_CONFIGURE baud bytes (start+1), got offset "
            f"{entry.offset} vs instr_start {instr_start}")


def test_frame_path_patch_lands_in_memcpy_imm_payload():
    """RegisterDriver FRAME path: write(reg, val, param=...) with a
    FRAME schema emits OP_MEMCPY_IMM whose payload is the wire frame;
    the patch covers the 4-byte frame within the payload."""
    from nxs.framing import SpiFrame, Crc

    class FrameDriver(RegisterDriver):
        WHO_AM_I_REG = 0x0B
        WHO_AM_I_VALUES = [0xAA]
        FRAME = SpiFrame(
            width=32,
            fields=[('rw', 1), ('addr', 5), ('rs', 2),
                    ('data', 16), ('crc', 8)],
            crc=Crc(width=8, poly=0x1D, init=0xFF, xor_out=0xFF,
                    covers=('rw', 'addr', 'rs', 'data'),
                    feedback_style='input-lsb'),
            read_pipeline=1,
            inter_frame_sleep_ms=1,
        )

        def __init__(self):
            super().__init__()
            self._read_responses = {self.WHO_AM_I_REG: [0xAA55]}

        def probe(self):
            who = self.read(self.WHO_AM_I_REG)
            assert who == 0xAA55

        def configure(self, config):
            self.declare_param("mode", values=[0x01, 0x02, 0x03],
                               default=0x01, unit="")
            mode = config.get('mode', 0x01)
            self.write(0x10, mode, param=("mode", mode))
            self.set_sample_size(2)
            self.set_output([{'name': 'val',
                              'type': 'int16',
                              'byte_order': 'big',
                              'scale': 1.0,
                              'unit': 'raw'}])

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_words(0x00, 1)
            return Sample(raw)

    cd = FrameDriver().compile({})
    mode_entries = [e for e in cd.patch_map if e.param_name == 'mode']
    assert mode_entries, "expected a mode patch entry on FrameDriver"
    _assert_patches_land_in_operand_bytes(cd)
    for entry in mode_entries:
        bounds = _instruction_bounds_at(cd.bytecode, entry.offset)
        assert bounds is not None
        instr_start, opcode, _ = bounds
        assert opcode == Op.MEMCPY_IMM, (
            f"expected MEMCPY_IMM for FRAME-path patch, "
            f"got 0x{opcode:02X}")
        # MEMCPY_IMM is [opcode, dst_off, len, data...]; payload starts
        # at instr_start + 3.
        assert entry.offset == instr_start + 3, (
            f"expected MEMCPY_IMM payload start (instr+3), got offset "
            f"{entry.offset} vs instr_start {instr_start}")


# ── Stream-driver framing helpers ───────────────────────────

def test_read_until_emits_byte_loop():
    """read_until should produce a loop of UART_READ_REG + LOAD_U8_REG +
    CMP_EQ + JNZ that scans for the delimiter byte-by-byte."""
    class NmeaSentenceDriver(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            self.read_until(b'\n', max=96)
            self.store_sample_n()
            return None

    cd = NmeaSentenceDriver().compile()
    bc = cd.bytecode
    # All five primitives the helper relies on must be present.
    assert Op.UART_AVAIL in bc
    assert Op.UART_READ_REG in bc
    assert Op.LOAD_U8_REG in bc
    assert Op.CMP_EQ in bc
    assert Op.STORE_SAMPLE_N in bc
    # And the delimiter byte 0x0A appears as the CMP_EQ immediate.
    # (Just check that 0x0A is in the bytecode — looser but stable
    # against bytecode-layout tweaks.)
    assert 0x0A in bc


def test_read_n_emits_fixed_count_loop():
    """read_n should produce a loop that reads exactly N bytes."""
    class FrameHeaderDriver(StreamDriver):
        def configure(self, config):
            self.set_baud(115200)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            self.read_n(6)
            self.store_sample_n()
            return None

    cd = FrameHeaderDriver().compile()
    bc = cd.bytecode
    assert Op.UART_READ_REG in bc
    assert Op.STORE_SAMPLE_N in bc
    # The target count 6 appears as a CMP_EQ immediate.
    assert 6 in bc


def test_parse_u16_le_emits_load_op():
    """parse_u16_le should emit LOAD_U16_LE with the requested offset."""
    class LengthPrefixDriver(StreamDriver):
        def configure(self, config):
            self.set_baud(115200)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            self.read_n(6)
            self.parse_u16_le(4)
            self.store_sample_n()
            return None

    cd = LengthPrefixDriver().compile()
    bc = cd.bytecode
    assert Op.LOAD_U16_LE in bc
    # Find the LOAD_U16_LE opcode and verify its offset operand is 4.
    idx = bc.index(Op.LOAD_U16_LE)
    # LOAD_U16_LE is [opcode, dst_reg, buf_off]. buf_off is at idx+2.
    assert bc[idx + 2] == 4


def test_read_until_rejects_self_similar_delim():
    """Multi-byte delim with a self-similar prefix/suffix needs
    KMP-style backtracking, which the compiler doesn't emit. Reject
    at compile time with a helpful message."""
    class BadDriver(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            # `abab` has prefix `ab` matching suffix `ab` (length 2).
            self.read_until(b'abab')
            return None

    try:
        BadDriver().compile()
        assert False, "expected CompileError for self-similar delim"
    except CompileError as e:
        assert "self-similar" in str(e)


# ── Checksum descriptors ────────────────────────────────────

def test_checksum_fletcher_emits_risc_loop():
    """ChecksumFletcher compiles to an arithmetic loop over the new
    RISC primitives — no protocol-specific opcode anywhere."""
    class UbxValsetDriver(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            # Stage a fake UBX frame in the buffer then compute its
            # Fletcher checksum into the trailing 2 bytes — same shape
            # NeoM9n uses to seal a UBX-CFG-VALSET payload.
            self.compute_checksum(ChecksumFletcher(),
                                  start_off=2, length=6, dst_off=8)
    cd = UbxValsetDriver().compile()
    bc = cd.bytecode
    assert Op.ADD_REG in bc
    assert Op.AND in bc
    assert Op.LOAD_U8_REG in bc
    assert Op.STORE_U8 in bc
    # Fletcher loop should NOT use OP_CRC8.
    assert Op.CRC8 not in bc


def test_checksum_xorfold_emits_xor_loop():
    class XorDriver(StreamDriver):
        def configure(self, config):
            self.set_baud(9600)
            self.compute_checksum(ChecksumXorFold(),
                                  start_off=1, length=10, dst_off=12)
    cd = XorDriver().compile()
    bc = cd.bytecode
    assert Op.XOR_REG in bc
    assert Op.LOAD_U8_REG in bc
    assert Op.STORE_U8 in bc


def test_checksum_polynomial_uses_existing_op_crc8():
    """ChecksumPolynomial delegates to the parameter-driven OP_CRC8
    opcode (proven hot-path algorithmic-family op). The compiler
    should NOT emit a RISC-bytecode loop for this case."""
    class CrcDriver(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.compute_checksum(
                ChecksumPolynomial(poly=0x31, init=0xFF, xor_out=0x00),
                start_off=0, length=8, dst_off=8)
    cd = CrcDriver().compile()
    bc = cd.bytecode
    assert Op.CRC8 in bc
    assert Op.STORE_U8 in bc
    # Polynomial path should NOT touch the RISC arithmetic primitives.
    assert Op.ADD_REG not in bc
    assert Op.XOR_REG not in bc


def test_checksum_invalid_spec_raises():
    class BadDriver(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.compute_checksum("not-a-descriptor", 0, 4, 4)  # type: ignore
    try:
        BadDriver().compile()
        assert False, "expected CompileError"
    except CompileError:
        pass


# ── New: write(bytes), timeouts, TracedSlice.expect ────────


def _opcode_counts(bytecode: bytes) -> dict:
    """Count occurrences of each opcode in `bytecode`."""
    counts = {}
    for _off, op, _size in _walk_bc(bytecode):
        counts[op] = counts.get(op, 0) + 1
    return counts


def test_write_int_still_emits_single_uart_write():
    """`write(int)` regression: must still compile to a single
    OP_UART_WRITE, not the MEMCPY_IMM + UART_WRITE_RAW path."""
    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.write(0xB5)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            return None

    bc = Drv().compile().bytecode
    counts = _opcode_counts(bc)
    assert counts.get(Op.UART_WRITE, 0) == 1, \
        f"expected one OP_UART_WRITE, counts={counts}"
    assert counts.get(Op.MEMCPY_IMM, 0) == 0
    assert counts.get(Op.UART_WRITE_RAW, 0) == 0


def test_write_bytes_emits_memcpy_then_write_raw():
    """`write(bytes)` stages via MEMCPY_IMM, then clocks out via
    UART_WRITE_RAW. The staged bytes appear inline in the
    MEMCPY_IMM payload."""
    frame = bytes([0xB5, 0x62, 0x06, 0x8A])

    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.write(frame)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            return None

    bc = Drv().compile().bytecode
    counts = _opcode_counts(bc)
    assert counts.get(Op.MEMCPY_IMM, 0) == 1
    assert counts.get(Op.UART_WRITE_RAW, 0) == 1
    # The frame bytes appear in the MEMCPY_IMM payload.
    assert frame in bc, f"frame {frame!r} not found in bytecode"


def test_write_bytes_empty_is_noop():
    """`write(b'')` emits nothing."""
    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            before = self._emitter._current_offset()
            self.write(b'')
            after = self._emitter._current_offset()
            assert before == after

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            return None

    Drv().compile()


def test_write_rejects_non_int_non_bytes():
    """`write("string")` is a programming error — fail at compile time."""
    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.write("AT")  # str, not bytes

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            return None

    try:
        Drv().compile()
        assert False, "expected CompileError for write(str)"
    except CompileError as e:
        assert "int or bytes" in str(e)


def test_read_until_multi_byte_emits_pg_counter_and_cases():
    """Multi-byte delim emits one LOAD_IMM per pg-init plus the
    dispatch chain. For a 2-byte delim, both case blocks are
    present and the final byte's match jumps to the done label."""
    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            self.read_until(b'\xb5\x62', max=64)
            self.store_sample_n()

    bc = Drv().compile().bytecode
    counts = _opcode_counts(bc)
    # The multi-byte loop uses CMP_EQ per case + max-check + per-byte
    # body. Bytecode must reference both 0xB5 and 0x62 as immediate
    # operands somewhere.
    assert 0xB5 in bc and 0x62 in bc
    # Dispatch + case bodies push CMP_EQ count above the single-byte
    # case (which used 2: byte-vs-delim, cursor-vs-max).
    assert counts.get(Op.CMP_EQ, 0) >= 4


def test_read_until_self_similar_delim_rejected():
    """Self-similar prefix (would need KMP) — compile-time error."""
    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            self.read_until(b'\xaa\xaa')  # prefix `\xaa` == suffix `\xaa`
            self.store_sample_n()

    try:
        Drv().compile()
        assert False, "expected CompileError"
    except CompileError as e:
        assert "self-similar" in str(e)


def test_read_n_with_timeout_emits_timer_branch():
    """`read_n(N, timeout_ms=T)` adds an iteration counter on the
    SLEEP_MS idle branch, ending in OP_ERROR ERR_CODE_TIMEOUT."""
    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            self.read_n(10, timeout_ms=2000)
            self.store_sample_n()

    bc = Drv().compile().bytecode
    counts = _opcode_counts(bc)
    assert counts.get(Op.ERROR, 0) >= 1, \
        f"expected at least one OP_ERROR (for timeout), counts={counts}"
    # The 2000 timeout iterations immediate appears in a CMP_EQ.
    # 2000 = 0xD0 0x07 little-endian; verify the LE bytes are in the
    # CMP_EQ immediate stream.
    assert b'\xd0\x07' in bc


def test_read_n_without_timeout_omits_timer():
    """`read_n(N)` without timeout omits the counter — no OP_ERROR
    emitted by the read itself."""
    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            self.read_n(10)
            self.store_sample_n()

    bc = Drv().compile().bytecode
    counts = _opcode_counts(bc)
    assert counts.get(Op.ERROR, 0) == 0, \
        f"unexpected OP_ERROR with no timeout, counts={counts}"


def test_read_n_returns_traced_slice():
    """`read_n` returns a TracedSlice with the right offset and length."""
    captured = {}

    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            slc = self.read_n(10, timeout_ms=2000)
            captured['slc'] = slc

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            return None

    Drv().compile()
    slc = captured['slc']
    assert isinstance(slc, TracedSlice)
    assert slc.buf_off == 0
    assert slc.length == 10


def test_traced_slice_expect_emits_per_byte_compares():
    """`.expect(bytes)` emits one LOAD_U8 + CMP_EQ + JZ per byte,
    plus a single OP_ERROR ERR_CODE_MISMATCH at the failure label."""
    expected = bytes([0xB5, 0x62, 0x05, 0x01])

    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            slc = self.read_n(10, timeout_ms=2000)
            slc.expect(expected)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            return None

    bc = Drv().compile().bytecode
    counts = _opcode_counts(bc)
    # One LOAD_U8 per expected byte.
    assert counts.get(Op.LOAD_U8, 0) >= len(expected)
    # One OP_ERROR for timeout + one for mismatch = at least 2.
    assert counts.get(Op.ERROR, 0) >= 2
    # The expected bytes appear inline in CMP_EQ immediates.
    for want in expected:
        assert want in bc


def test_traced_slice_expect_rejects_longer_than_slice():
    """`.expect()` longer than the slice is a compile error."""
    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            slc = self.read_n(4)
            slc.expect(b'\x01\x02\x03\x04\x05')  # 5 > 4

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            return None

    try:
        Drv().compile()
        assert False, "expected CompileError"
    except CompileError as e:
        assert "longer than slice" in str(e)


def test_traced_slice_expect_rejects_empty():
    """`.expect(b'')` is a programming error."""
    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            slc = self.read_n(4)
            slc.expect(b'')

        @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            return None

    try:
        Drv().compile()
        assert False, "expected CompileError"
    except CompileError as e:
        assert "empty" in str(e)


def test_stage_emits_memcpy_imm_with_frame_inline():
    """stage(bytes) emits OP_MEMCPY_IMM at sample_buf[0..len(bytes))
    with the frame bytes inline in the payload."""
    frame = bytes([0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03])

    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.stage(frame)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            return None

    bc = Drv().compile().bytecode
    counts = _opcode_counts(bc)
    assert counts.get(Op.MEMCPY_IMM, 0) == 1
    # Frame bytes appear inline in the MEMCPY_IMM payload.
    assert frame in bc


def test_stage_records_patch_site_at_correct_offset():
    """stage(frame, patch=(name, req, enc, frame_offset, size))
    records the patch site at the absolute bytecode offset
    corresponding to sample_buf[frame_offset], i.e.
    current_emit_offset + 3-byte MEMCPY_IMM header + frame_offset."""
    captured_patches = []

    class Drv(StreamDriver):
        def configure(self, config):
            # Hook the patch recorder to capture call args.
            orig = self._patch_recorder_fn

            def capture(*args, **kwargs):
                captured_patches.append((args, kwargs))
                return orig(*args, **kwargs)
            self._patch_recorder_fn = capture

            self.declare_param("rate", values=[1, 2, 4], default=1)
            self.set_baud(38400)
            stage_offset = self._emitter._current_offset()
            frame = bytes([0xAA, 0xBB, 0xCC, 0xDD])
            rate = config.get("rate", 1)
            # The staged value tracks the config value, so every declared
            # value maps to a patch (a hardcoded value would leave the others
            # patching 0).
            self.stage(frame,
                       patch=("rate", rate, 0xCC00 | rate, 2, 2))
            # MEMCPY_IMM header = 3 bytes; patch byte at frame_offset=2.
            expected_patch_offset = stage_offset + 3 + 2
            assert captured_patches, "patch recorder not called"
            args, _ = captured_patches[0]
            assert args[0] == "rate"
            assert args[3] == expected_patch_offset

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            return None

    Drv().compile()


def test_send_staged_emits_uart_write_raw():
    """send_staged(N) emits OP_UART_WRITE_RAW 0 N."""
    class Drv(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.stage(bytes([1, 2, 3, 4]))
            self.send_staged(4)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            return None

    bc = Drv().compile().bytecode
    counts = _opcode_counts(bc)
    assert counts.get(Op.UART_WRITE_RAW, 0) == 1


def test_reg_alloc_scope_releases_names():
    """Names allocated inside a scope are dropped on exit; subsequent
    persistent gets reuse the freed register slots."""
    from nxs.compiler import _RegAlloc
    alloc = _RegAlloc()
    persistent_a = alloc.get("persistent_a")
    assert persistent_a == 0
    with alloc.scope():
        s1 = alloc.get("__scratch_1")
        s2 = alloc.get("__scratch_2")
        assert s1 == 1 and s2 == 2
    # After scope: scratches released, next allocation reuses slot 1.
    persistent_b = alloc.get("persistent_b")
    assert persistent_b == 1


def test_reg_alloc_scope_nests():
    """Nested scopes release independently; inner-scope names are
    invisible to outer scope after inner exit."""
    from nxs.compiler import _RegAlloc
    alloc = _RegAlloc()
    with alloc.scope():
        outer = alloc.get("__outer")
        assert outer == 0
        with alloc.scope():
            inner = alloc.get("__inner")
            assert inner == 1
        # Inner scope exited: slot 1 is free again.
        outer_b = alloc.get("__outer_b")
        assert outer_b == 1


def test_reg_alloc_pin_survives_reset():
    """Pinned names allocate top-down, survive reset(), and bound the
    scratch pool; scratch names never land on a pinned slot."""
    from nxs.compiler import _RegAlloc
    alloc = _RegAlloc()
    scratch = alloc.get("__config_scratch")
    assert scratch == 0
    pinned = alloc.pin("acc")
    assert pinned == _RegAlloc.VM_NUM_REGS - 1
    alloc.reset()
    # Post-reset: the pin resolves to the same slot; fresh scratch
    # restarts at r0 and cannot reach the pinned slot.
    assert alloc.get("acc") == pinned
    for i in range(_RegAlloc.VM_NUM_REGS - 1):
        assert alloc.get(f"__s{i}") == i
    try:
        alloc.get("__overflow")
        assert False, "expected CompileError"
    except CompileError:
        pass


def test_persistent_register_stable_across_sections():
    """A persistent()'s configure()-time init must target the same
    register the measure() body reads and writes — even when measure
    touches other names first (the allocation orders differ, so any
    order-dependent binding drifts apart)."""
    class Drv(RegisterDriver):
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.persistent("acc", 7777)
            self.set_output([{'name': 'verdict', 'type': 'uint8',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(1)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            decoy = self.read(0x10)
            if decoy == 0: acc = 4242
            return Sample(verdict=acc)

    bc = Drv().compile({}).bytecode

    def load_imm_reg(imm):
        for off, op, _size in _walk_bc(bc):
            if op == Op.LOAD_IMM and struct.unpack_from('<I', bc, off + 2)[0] == imm:
                return bc[off + 1]
        raise AssertionError(f"no LOAD_IMM {imm} in bytecode")

    assert load_imm_reg(7777) == load_imm_reg(4242)


def test_error_code_constants_match_firmware_mapping():
    """ERR_CODE_TIMEOUT / ERR_CODE_MISMATCH must match the values
    the firmware's OP_ERROR arm switches on (src/vm/SensorDriverVM.cpp).
    Lock them in so a careless renumber on either side raises here."""
    assert ERR_CODE_TIMEOUT == 17
    assert ERR_CODE_MISMATCH == 18


def test_many_coefficient_reads_fit_register_file():
    # Each `self.cN = self.read(reg, 2)` stages the burst through a scratch
    # register before CVT64 parks it in the work buffer. That register must be
    # REUSED across coefficients — a distinct name per coefficient would exhaust
    # the 8-register file. A part with a dozen coefficients (BME680-class) must
    # still compile.
    class ManyCoef(RegisterDriver):
        def configure(self, config):
            for i in range(12):
                setattr(self, f"c{i}", self.read(0xA2 + 2 * i, 2))
            self.set_output(
                [{'name': 'out', 'type': 'int32', 'scale': 1.0, 'unit': ''}])

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            d = self.read(0x00, 3)
            p = self.c0 * d + self.c11 * d   # uses the first and last coefficient
            return Sample(out=p)

    result = ManyCoef().compile()   # must not raise "Out of VM registers"
    assert len(result.bytecode) > 0


def test_arith_lhs_must_be_a_simple_name():
    # A non-Name left operand (`(x & 0x0F) - 1`) used to allocate a register for
    # the empty string and miscompile silently. It must now fail loudly.
    class C(RegisterDriver):
        def configure(self, config):
            self.set_sample_size(1)

        @SensorDriver.measure_loop(trigger="drdy")
        def measure(self):
            x = self.read(0x00)
            y = (x & 0x0F) - 1   # left operand of '-' is a BinOp, not a name
            return Sample(y)

    try:
        C().compile()
        assert False, "expected CompileError for a non-name arithmetic LHS"
    except CompileError:
        pass


def test_compare_against_int32_max_is_rejected():
    # `x <= INT32_MAX` lowers to `x < rhs+1`; rhs+1 overflows the signed range,
    # so the comparison is degenerate and must be rejected, not miscompiled.
    class C(RegisterDriver):
        def configure(self, config):
            self.set_sample_size(1)

        @SensorDriver.measure_loop(trigger="drdy")
        def measure(self):
            x = self.read(0x00)
            if x <= 0x7FFFFFFF:
                return None
            return Sample(x)

    try:
        C().compile()
        assert False, "expected CompileError for '<= INT32_MAX'"
    except CompileError:
        pass


def test_read_width_out_of_range_is_rejected():
    # OP_LOAD only handles 1..4 bytes; a wider read must fail at compile time,
    # not silently emit a spec byte the firmware rejects at runtime.
    class C(RegisterDriver):
        def configure(self, config):
            self.set_sample_size(5)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            d = self.read(0x00, 5)   # 5 > 4: unsupported by OP_LOAD
            return Sample(d)

    try:
        C().compile()
        assert False, "expected CompileError for read width 5"
    except CompileError:
        pass


def test_computed_sample_rejects_non_integer_field():
    # The computed-Sample path stores raw integer bytes; a float/string output
    # field would be silently corrupted, so it must be rejected.
    class C(RegisterDriver):
        def configure(self, config):
            self.k = self.read(0xA2, 2)
            self.set_output(
                [{'name': 'val', 'type': 'float32', 'scale': 1.0, 'unit': ''}])

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            d = self.read(0x00, 3)
            p = self.k * d
            return Sample(val=p)

    try:
        C().compile()
        assert False, "expected CompileError for a float32 computed field"
    except CompileError:
        pass


def test_out_of_range_32bit_shift_is_rejected():
    # A 32-bit shift count outside 0..31 used to be masked (& 0xFF) and emit a
    # different shift than Python would — it must fail at compile time instead.
    class C(RegisterDriver):
        def configure(self, config):
            self.set_sample_size(1)

        @SensorDriver.measure_loop(trigger="drdy")
        def measure(self):
            x = self.read(0x00)
            y = x << 40            # > 31
            return Sample(y)

    try:
        C().compile()
        assert False, "expected CompileError for a 32-bit shift of 40"
    except CompileError:
        pass


def test_out_of_range_64bit_shift_is_rejected():
    # A 64-bit (wide) shift count outside 0..63 was masked (& 0x3F); the VM
    # gates n < 64, so the compiler must reject it rather than wrap.
    class C(RegisterDriver):
        def configure(self, config):
            self.k = self.read(0xA2, 2)
            self.set_output([{'name': 'out', 'type': 'int32', 'scale': 1.0, 'unit': ''}])

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            d = self.read(0x00, 3)
            p = self.k * d          # wide
            q = p << 100            # > 63
            return Sample(out=q)

    try:
        C().compile()
        assert False, "expected CompileError for a 64-bit shift of 100"
    except CompileError:
        pass


def test_vm_layout_constants_match_firmware_header():
    # The compiler mirrors the VM's RAM layout — sample buffer, work buffer,
    # register count — which live in SensorDriverVM.h. Drift silently
    # mis-sizes or mis-rejects drivers, so pin the mirror to the header.
    # The header sits in the firmware repo beside the SDK tree; a standalone
    # SDK checkout has nothing to pin against, so the check skips there.
    import re
    from pathlib import Path

    import pytest

    from nxs.compiler import _ASTCompiler, _RegAlloc

    header_path = (Path(__file__).resolve().parents[3]
                   / "include" / "redbrain" / "vm" / "SensorDriverVM.h")
    if not header_path.exists():
        pytest.skip("firmware header lives in the firmware repo")
    header = header_path.read_text()

    def hdr(name):
        m = re.search(rf"\b{name}\s*=\s*(\d+)", header)
        assert m, f"{name} not found in SensorDriverVM.h"
        return int(m.group(1))

    assert _ASTCompiler.VM_SAMPLE_BUF_SIZE == hdr("VM_SAMPLE_BUF_SIZE")
    assert _ASTCompiler.VM_WORK_BUF_SIZE == hdr("VM_WORK_BUF_SIZE")
    assert _RegAlloc.VM_NUM_REGS == hdr("VM_NUM_REGS")

    # The descriptor caps (MAX_OUTPUTS/MAX_PARAMS/MAX_PARAM_VALUES) are no
    # longer hand-mirrored — they come from the SSOT (constants/driver_
    # image.yaml), generated to both this module and DriverImage.h, so the
    # compiler's value equals the generated constant by construction.
    from nxs.compiler import MAX_OUTPUTS
    from nxs._generated_constants import NxsDriverImage
    assert MAX_OUTPUTS == NxsDriverImage.MAX_OUTPUTS


# ── Analog (ADC) tests ──────────────────────────────────────

class AnalogMeasureDriver(RegisterDriver):
    BUSES = ("i2c",)
    WHO_AM_I_VALUES = []
    WHO_AM_I_SKIP_REASON = "analog-only test double"

    def configure(self, config):
        # `voltage` inherits its canonical SI unit (V) — omit `unit`.
        self.set_output([{'name': 'voltage', 'type': 'uint16', 'scale': 1.0}])
        self.set_sample_size(2)

    @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
    def measure(self):
        raw = self.read_analog(0)
        # Named placement: read_analog stages off the sample buffer, so the
        # value reaches the output only through the declared field.
        return Sample(voltage=raw)


def test_read_analog_stages_off_the_sample_buffer():
    # ch=0, buf_off=SCALAR_SCRATCH_OFF (92) — the count is staged past the
    # sample window, then LOADed and stored into the `voltage` field. Staging
    # at 0 would let a later scalar read (or the field store) alias it.
    from nxs.compiler import _ASTCompiler
    off = _ASTCompiler.SCALAR_SCRATCH_OFF
    result = AnalogMeasureDriver().compile()
    assert bytes([Op.ADC_READ, 0, off]) in result.bytecode
    assert Op.STORE_SAMPLE in result.bytecode


def test_read_analog_disassembles():
    result = AnalogMeasureDriver().compile()
    listing = "\n".join(disassemble(result.bytecode, print_fn=lambda *_: None))
    assert "ADC_READ" in listing
    assert "adc(ch=0)" in listing


def test_scalar_read_stages_off_sample_data():
    # A scalar read(reg, width) that follows a data burst must not stage over
    # the burst's bytes — this is the accel_x=0 FIFO bug in miniature.
    from nxs.compiler import _ASTCompiler
    off = _ASTCompiler.SCALAR_SCRATCH_OFF

    class BurstThenScalar(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x10, 2)   # sample data -> buf[0..1]
            n = self.read(0x72, 2)           # scalar count -> buf[92], not buf[0]
            if n == 0:
                return None
            return Sample(raw)

    bc = BurstThenScalar().compile().bytecode
    # data burst lands at 0; the scalar count lands at the scratch tail
    assert bytes([Op.REG_READ_BURST, 0x10, 0x00, 0x02, 0x00]) in bc
    assert bytes([Op.REG_READ_BURST, 0x72, 0x00, 0x02, off]) in bc


def test_scalar_read_sample_size_guard():
    # A sample that reaches into the scalar-scratch slot is a compile error,
    # not silent corruption.
    class BigSample(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint8',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(125)  # 125 > 124: overlaps the scratch slot

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            n = self.read(0x00, 2)
            return Sample(a=n)

    try:
        BigSample().compile()
        assert False, "expected CompileError for a sample reaching the scratch slot"
    except CompileError:
        pass


def test_bare_read_statement_stages_off_sample():
    # A bare `self.read(...)` (read-to-clear a latched flag) must run the
    # transaction but never stage over sample data. Width>1 stages at the
    # scratch tail; width==1 lands in a scratch register.
    from nxs.compiler import _ASTCompiler
    off = _ASTCompiler.SCALAR_SCRATCH_OFF

    class BareRead(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="drdy")
        def measure(self):
            raw = self.read_burst(0x28, 2)   # sample data -> buf[0..1]
            self.read(0x1E, 2)               # read-to-clear INT_STATUS (bare)
            self.read(0x1F)                  # single-byte bare read
            return Sample(raw)

    bc = BareRead().compile().bytecode
    # data burst at 0; the bare wide read stages at the tail, not buf[0]
    assert bytes([Op.REG_READ_BURST, 0x28, 0x00, 0x02, 0x00]) in bc
    assert bytes([Op.REG_READ_BURST, 0x1E, 0x00, 0x02, off]) in bc
    # the single-byte bare read is a REG_READ (transaction happens)
    assert bytes([Op.REG_READ, 0x1E]) not in bc  # sanity: 0x1E was the wide one
    assert Op.REG_READ in bc


def _scalar_load_specs(bc):
    # spec bytes of every OP_LOAD that reads from the scalar-scratch tail (92).
    from nxs.compiler import _ASTCompiler
    off = _ASTCompiler.SCALAR_SCRATCH_OFF
    return [bc[i + 3] for i in range(len(bc) - 3)
            if bc[i] == Op.LOAD and bc[i + 2] == off]


def test_read_defaults_unsigned_big_endian():
    class Plain(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            v = self.read(0x28, 2)
            return Sample(a=v)

    assert _scalar_load_specs(Plain().compile().bytecode) == [0x02]  # width 2, BE, unsigned


def test_read_signed_little_endian_spec():
    class SignedLE(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            v = self.read(0x28, 2, signed=True, endian="little")
            return Sample(a=v)

    # width 2 | little-endian (0x08) | sign-extend (0x10) = 0x1A
    assert _scalar_load_specs(SignedLE().compile().bytecode) == [0x1A]


def test_read_bad_endian_rejected():
    class BadEndian(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            v = self.read(0x28, 2, endian="middle")
            return Sample(a=v)

    try:
        BadEndian().compile()
        assert False, "expected CompileError for bad endian"
    except CompileError:
        pass


def test_read_kwargs_require_explicit_width():
    class NoWidth(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            v = self.read(0x28, signed=True)   # no width
            return Sample(a=v)

    try:
        NoWidth().compile()
        assert False, "expected CompileError for kwargs without explicit width"
    except CompileError as e:
        assert "width" in str(e)


def test_read_burst_into_places_distinct_banks():
    # Two bursts into distinct regions build a split-bank sample.
    class SplitBank(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16', 'scale': 1.0, 'unit': ''},
                             {'name': 'b', 'type': 'uint16', 'scale': 1.0, 'unit': ''}])
            self.set_sample_size(12)

        @RegisterDriver.measure_loop(trigger="drdy")
        def measure(self):
            raw = self.read_burst(0x02, 6)        # bank A -> [0, 6)
            self.read_burst(0x3B, 6, into=6)      # bank B -> [6, 12)
            return Sample(raw)

    bc = SplitBank().compile().bytecode
    assert bytes([Op.REG_READ_BURST, 0x02, 0x00, 6, 0]) in bc
    assert bytes([Op.REG_READ_BURST, 0x3B, 0x00, 6, 6]) in bc


def test_read_burst_overlap_rejected():
    class Overlap(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16', 'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="drdy")
        def measure(self):
            raw = self.read_burst(0x02, 6)   # [0, 6)
            self.read_burst(0x3B, 6)         # [0, 6) — clobbers bank A
            return Sample(raw)

    try:
        Overlap().compile()
        assert False, "expected CompileError for overlapping bursts"
    except CompileError as e:
        assert "overlap" in str(e)


def test_read_burst_past_sample_buffer_rejected():
    class PastBuf(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16', 'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="drdy")
        def measure(self):
            raw = self.read_burst(0x00, 4, into=126)  # [126, 130) — past the 128-byte buffer
            return Sample(raw)

    try:
        PastBuf().compile()
        assert False, "expected CompileError for a burst past the sample buffer"
    except CompileError:
        pass


def test_pure_burst_fills_full_sample_buffer():
    # No scalar read uses the tail scratch, so a burst may fill all 128 bytes.
    class FullBurst(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16', 'scale': 1.0, 'unit': ''}])
            self.set_sample_size(128)

        @RegisterDriver.measure_loop(trigger="drdy")
        def measure(self):
            raw = self.read_burst(0x00, 128)          # [0, 128) — the whole buffer
            return Sample(raw)

    FullBurst().compile()   # must not raise


def test_frame_driver_measure_read_is_framed():
    # A FRAME part's `self.read(reg)` in measure() must clock the composed
    # wire frame (MEMCPY_IMM + REG_XFER), not an unframed REG_READ.
    from nxs.framing import SpiFrame

    class Framed(RegisterDriver):
        BUSES = ("spi",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"
        FRAME = SpiFrame(width=32,
                         fields=[('rw', 1), ('addr', 7), ('data', 16), ('pad', 8)],
                         read_pipeline=1)

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            x = self.read(0x05)      # scalar FRAME read
            return Sample(a=x)

    bc = Framed().compile().bytecode
    assert Op.MEMCPY_IMM in bc           # request frame staged
    assert Op.REG_XFER in bc             # frame clocked out
    assert Op.REG_READ not in bc         # no unframed read on a FRAME part


def _wm_frame():
    from nxs.framing import SpiFrame, Crc
    return SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, poly=0x1D, init=0xFF, xor_out=0xFF,
                covers=('rw', 'addr', 'rs', 'data'), feedback_style='input-lsb'),
        read_pipeline=1)


def test_write_modify_param_is_patchable():
    # A param= write_modify makes its field runtime-tunable: the OR set-bits
    # immediate is a patch site whose value differs per range.
    frame = _wm_frame()

    class WM(RegisterDriver):
        BUSES = ("spi",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"
        FRAME = frame

        def configure(self, config):
            self.declare_param("rng", values=[1, 2, 4], default=1)
            rng = config.get("rng", 1)
            code = {1: 0b001, 2: 0b010, 4: 0b100}[rng]
            # clear the whole 3-bit field (constant), OR the code (varies).
            self.write_modify(0x14, set_bits=code, clear_bits=0b111,
                              param=("rng", rng))
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            x = self.read(0x00)
            return Sample(a=x)

    cd = WM().compile()
    sites = [pe for pe in cd.patch_map if pe.param_name == "rng"]
    assert len(sites) == 1
    assert sorted(sites[0].value_map.values()) == [0b001, 0b010, 0b100]


def test_write_modify_param_set_outside_clear_rejected():
    # param= patches only the OR set; clear must be the whole field, else a set
    # bit outside clear would stick across a range change.
    frame = _wm_frame()

    class BadClear(RegisterDriver):
        BUSES = ("spi",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"
        FRAME = frame

        def configure(self, config):
            self.declare_param("rng", values=[1, 2], default=1)
            rng = config.get("rng", 1)
            # set bit 2 but clear only bits [1:0] — bit 2 is outside clear.
            self.write_modify(0x14, set_bits=0b100, clear_bits=0b011,
                              param=("rng", rng))
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            x = self.read(0x00)
            return Sample(a=x)

    try:
        BadClear().compile()
        assert False, "expected CompileError for set_bits outside clear_bits"
    except CompileError as e:
        assert "outside clear_bits" in str(e)


def _sync_driver(rates, base_hz=8000):
    class FixedSync(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.declare_param("sample_rate", values=rates, default=rates[0],
                               unit="Hz")
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="drdy", sample_rate=rates[0],
                                     drdy_base_hz=base_hz)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    return FixedSync


def test_drdy_base_emits_patchable_event_divider():
    # A fixed-sync part paces drdy by dividing the sync: EVENT_DIV carries
    # the divider as sample_rate's patch site, one exact divider per rate.
    cd = _sync_driver([10, 100, 500])().compile()
    entries = [e for e in cd.patch_map if e.param_name == "sample_rate"]
    assert len(entries) == 1
    assert cd.bytecode[entries[0].offset - 1] == Op.EVENT_DIV
    assert entries[0].size == 2
    assert entries[0].value_map == {10: 800, 100: 80, 500: 16}


def test_drdy_base_rejects_non_divisor_rate():
    # 300 doesn't divide 8000 — delivering it would silently run at a rate
    # other than the one set. Loud compile error, not approximation.
    try:
        _sync_driver([100, 300])().compile()
        assert False, "expected CompileError for a non-divisor sample_rate"
    except CompileError as e:
        assert "does not divide" in str(e)


def test_drdy_base_with_rate_register_rejected():
    # A part with a real ODR register (the ordinary IMU) paces through the
    # tagged register write; declaring drdy_base_hz too would double-pace.
    class Both(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.declare_param("sample_rate", values=[100, 200], default=100,
                               unit="Hz")
            rate = config.get("sample_rate", 100)
            self.write(0x19, {100: 9, 200: 4}[rate],
                       param=("sample_rate", rate))
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="drdy", sample_rate=100,
                                     drdy_base_hz=8000)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    try:
        Both().compile()
        assert False, "expected CompileError for drdy_base_hz + rate register"
    except CompileError as e:
        assert "already patches a register write" in str(e)


def test_drdy_without_base_emits_no_divider():
    # A drdy part with a real ODR register declares no drdy_base_hz and gets
    # no EVENT_DIV — its rate knob is the tagged register write.
    class PlainDrdy(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="drdy")
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    bc = PlainDrdy().compile().bytecode
    assert Op.EVENT_DIV not in bc


def test_tag_before_declare_rejected():
    # A param= tag on a write before its declare_param used to be dropped
    # silently — the byte then never patched at runtime.
    class TagFirst(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.write(0x20, 1, param=("mode", 1))   # tagged before declare
            self.declare_param("mode", values=[1, 2], default=1)
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    try:
        TagFirst().compile()
        assert False, "expected CompileError for tag-before-declare"
    except CompileError as e:
        assert "declare" in str(e).lower()


def test_tagged_value_not_in_declared_set_rejected():
    class BadTag(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.declare_param("mode", values=[1, 2], default=1)
            self.write(0x20, 3, param=("mode", 3))   # 3 not in [1, 2]
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    try:
        BadTag().compile()
        assert False, "expected CompileError for an out-of-set tagged value"
    except CompileError:
        pass


def test_param_two_sites_allowed_over_cap_rejected():
    # A param may patch up to MAX_PATCH_SITES bytecode sites (a setting written
    # to two registers). Two compiles; a third exceeds the cap and is rejected.
    def _driver(n_sites):
        class MultiSite(RegisterDriver):
            BUSES = ("i2c",)
            WHO_AM_I_VALUES = []
            WHO_AM_I_SKIP_REASON = "test double"

            def configure(self, config):
                self.declare_param("mode", values=[1, 2], default=1)
                mode = config.get("mode", 1)
                for i in range(n_sites):
                    self.write(0x20 + i, mode, param=("mode", mode))
                self.set_output([{'name': 'a', 'type': 'uint16',
                                  'scale': 1.0, 'unit': ''}])
                self.set_sample_size(2)

            @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
            def measure(self):
                raw = self.read_burst(0x00, 2)
                return Sample(raw)

        return MultiSite

    # Two sites: compiles, and both sites carry the param.
    cd = _driver(2)().compile()
    assert sum(1 for pe in cd.patch_map if pe.param_name == "mode") == 2

    # Three sites: over MAX_PATCH_SITES (2), rejected.
    try:
        _driver(3)().compile()
        assert False, "expected CompileError for a param over MAX_PATCH_SITES"
    except CompileError as e:
        assert "bytecode sites" in str(e) and "at most" in str(e)


def test_two_params_one_register_rejected():
    # The packed-register trap: sample_rate and range both driving one CTRL byte.
    class Packed(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.declare_param("odr", values=[1, 2], default=1)
            self.declare_param("fs", values=[1, 2], default=1)
            self.write(0x10, 1, param=("odr", 1))
            self.write(0x10, 1, param=("fs", 1))     # same register, other param
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    try:
        Packed().compile()
        assert False, "expected CompileError for two params on one register"
    except CompileError as e:
        assert "register 0x10" in str(e)


def test_conditional_tagged_write_missing_value_rejected():
    # The tagged write runs for the default value (patch recorded, so the
    # untagged-param rule passes) but a conditional skips it for another value —
    # that value would patch nothing and default to 0. Must fail loudly.
    class Conditional(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.declare_param("mode", values=[1, 2], default=2)
            mode = config.get("mode", 2)
            if mode == 2:                    # skipped when mode == 1
                self.write(0x20, mode, param=("mode", mode))
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    try:
        Conditional().compile()
        assert False, "expected CompileError for a conditional tagged write"
    except CompileError as e:
        assert "patch record(s) but the param has" in str(e)


def test_value_dependent_patch_layout_rejected():
    class Drift(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.declare_param("mode", values=[1, 2], default=1)
            m = config.get("mode", 1)
            if m == 2:
                self.write(0x05, 0)          # extra write shifts the patch site
            self.write(0x10, m, param=("mode", m))
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    try:
        Drift().compile()
        assert False, "expected CompileError for a value-dependent patch layout"
    except CompileError as e:
        assert "move between values" in str(e)


def test_configure_read_result_rejected():
    # A runtime conditional on a read inside configure() reads a compile-time
    # mock, not the live register — it must fail loudly, not bake the mock branch.
    class RmwConfig(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            if self.read(0x0F) & 0x08:      # value is a mock at trace time
                self.write(0x20, 1)
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    try:
        RmwConfig().compile()
        assert False, "expected CompileError for a runtime read in configure()"
    except CompileError as e:
        assert "measure()" in str(e)


def test_probe_read_result_still_usable():
    # probe() reads must keep returning a real (mock) int for its assert.
    class Probed(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_REG = 0x75
        WHO_AM_I_VALUES = [0xAA]

        def __init__(self):
            super().__init__()
            self._read_responses = {0x75: [0xAA]}

        def probe(self):
            who = self.read(0x75)
            assert who == 0xAA          # real int, not a sentinel

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    Probed().compile()   # must not raise


def test_live_param_without_consumer_rejected():
    class OrphanLive(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.declare_param("threshold", values=[1, 2, 3], default=1,
                               kind="live")
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    try:
        OrphanLive().compile()
        assert False, "expected CompileError for a live param with no consumer"
    except CompileError as e:
        assert "no-op" in str(e)


def test_scale_param_with_zero_value_rejected():
    class ZeroScale(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.declare_param("fs", values=[0, 2, 4, 8], default=2)   # 0 in set
            self.set_output([{'name': 'accel_x', 'scale': 1.0,
                              'scale_param': 'fs'}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    try:
        ZeroScale().compile()
        assert False, "expected CompileError for a scale_param with a 0 value"
    except CompileError as e:
        assert "zeroes the field" in str(e)


def test_poll_rate_patches_sleep_interval():
    # A poll driver's declared sample_rate patches the loop SLEEP_MS interval.
    class Polled(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.declare_param("sample_rate", values=[1, 10, 100],
                               default=10, unit="Hz")
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    cd = Polled().compile()
    sr = [p for p in cd.patch_map if p.param_name == "sample_rate"]
    assert sr, "sample_rate should patch the poll interval"
    assert sr[0].value_map == {1: 1000, 10: 100, 100: 10}
    assert sr[0].size == 2


def test_untagged_reload_enum_param_rejected():
    class Untagged(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.declare_param("mode", values=[1, 2], default=1)   # never tagged
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_burst(0x00, 2)
            return Sample(raw)

    try:
        Untagged().compile()
        assert False, "expected CompileError for an untagged reload-enum param"
    except CompileError as e:
        assert "patches no bytecode" in str(e)


def test_non_call_expression_statement_rejected():
    # `n and self.write(...)` used to be silently dropped — the write never
    # happened. It must be a loud CompileError; a docstring stays legal.
    class BoolOpDriver(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            """Docstring stays legal."""
            n = self.read(0x00)
            n and self.write(0x01, 0xAA)   # BoolOp — silently dropped before the fix
            return Sample(a=n)

    try:
        BoolOpDriver().compile()
        assert False, "expected CompileError for a no-effect expression statement"
    except CompileError as e:
        assert "no effect" in str(e)


def test_if_else_branches_are_exclusive():
    # The true path must jump over the else body. Without the end-label JMP
    # both branches execute and the else value silently wins.
    class ElseDriver(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            n = self.read(0x00)
            if n == 0:
                flags = 1
            else:
                flags = 2
            return Sample(a=flags)

    lines = disassemble(ElseDriver().compile().bytecode,
                        print_fn=lambda *_: None)
    i_true = next(i for i, l in enumerate(lines) if "= 0x00000001" in l)
    i_else = next(i for i, l in enumerate(lines) if "= 0x00000002" in l)
    assert i_else == i_true + 2, "else body must directly follow the jump"
    jmp = lines[i_true + 1]
    assert "JMP" in jmp, "true path must jump over the else body"
    # The jump must land AFTER the else body's LOAD_IMM (forward, past it).
    target = int(jmp.split("-> 0x")[1].split()[0], 16)
    else_addr = int(lines[i_else].split()[0], 16)
    assert target > else_addr


def test_early_return_sample_terminates_iteration():
    # A mid-body `return Sample(...)` must jump to the loop top, not fall
    # through into the statements after it (the FIFO tiered gate depends on
    # this). Structural check: every STORE_SAMPLE is followed by a JMP.
    class EarlyReturn(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"

        def configure(self, config):
            self.set_output([{'name': 'a', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            n = self.read(0x00, 2)
            if n == 0:
                return Sample(a=n)      # early return — must jump to loop top
            self.write(0x01, 0xAA)      # only reachable when n != 0
            return None

    lines = disassemble(EarlyReturn().compile().bytecode,
                        print_fn=lambda *_: None)
    stores = [i for i, l in enumerate(lines) if "STORE_SAMPLE" in l]
    assert stores, "expected a STORE_SAMPLE"
    for i in stores:
        assert "JMP" in lines[i + 1], "return Sample must jump to the loop top"


def test_voltage_semantic_registered():
    from nxs.compiler import FIELD_SEMANTICS, infer_semantic
    assert FIELD_SEMANTICS['voltage'] == 24
    assert infer_semantic('voltage') == 24


# ── PWM (drive_pwm) tests ───────────────────────────────────

class PwmDriver(RegisterDriver):
    def configure(self, config):
        self.set_sample_size(2)
        self.drive_pwm(freq=2000, duty=25)

    @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
    def measure(self):
        raw = self.read_analog(0)
        return Sample(raw)


def test_drive_pwm_declares_live_range_params():
    cd = PwmDriver().compile()
    by_name = {p.name: p for p in cd.params}
    freq, duty = by_name["pwm_freq"], by_name["pwm_duty"]
    assert freq.param_type == "range" and freq.kind == "live"
    assert freq.values == [500, 25000] and freq.default == 2000
    assert duty.param_type == "range" and duty.kind == "live"
    assert duty.values == [0, 100] and duty.default == 25


def test_drive_pwm_rejects_out_of_range_freq():
    class BadFreq(RegisterDriver):
        def configure(self, config):
            self.drive_pwm(freq=100)  # below the 500 Hz floor

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_analog(0)
            return Sample(raw)
    try:
        BadFreq().compile()
        assert False, "expected CompileError for out-of-range freq"
    except CompileError:
        pass


def test_drive_pwm_rejects_out_of_range_config_override():
    # drive_pwm()'s args are valid, but a YAML config override must be
    # range-checked at declare time too — not left to the firmware set() path.
    try:
        PwmDriver().compile({"pwm_freq": 999999})  # past the 25 kHz ceiling
        assert False, "expected CompileError for out-of-range pwm_freq override"
    except CompileError:
        pass


def test_declare_range_param_rejects_bad_shape():
    # A range param must carry exactly [min, max]; a stepped/short list would
    # IndexError or emit an NXS shape the firmware parser rejects.
    class BadShape(RegisterDriver):
        def configure(self, config):
            self.declare_param("foo", values=[1, 2, 3], default=2,
                               param_type="range", kind="live")

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_analog(0)
            return Sample(raw)
    try:
        BadShape().compile()
        assert False, "expected CompileError for malformed range values"
    except CompileError:
        pass


def test_get_param_handles_range_params():
    # get_param() must accept an in-range override for a range param (its
    # values are [min, max], not an enum set) and reject an out-of-range one.
    class RangeGet(RegisterDriver):
        def configure(self, config):
            # A live range needs a runtime consumer; reference it as a
            # scale_param (values are magnitudes, so no 0).
            self.declare_param("gain", values=[1, 100], default=10,
                               param_type="range", kind="live")
            self.set_output([{'name': 'x', 'type': 'uint16', 'scale': 1.0,
                              'unit': '', 'scale_param': 'gain'}])
            self.set_sample_size(2)
            self._seen = self.get_param("gain")

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_analog(0)
            return Sample(x=raw)
    d = RangeGet()
    d.compile({"gain": 50})            # in-range override must be accepted
    assert d._seen == 50
    try:
        RangeGet().compile({"gain": 999})   # out of range must be rejected
        assert False, "expected CompileError for out-of-range range get_param"
    except CompileError:
        pass


def test_declare_param_rejects_out_of_range_default():
    # The declared default itself must be legal, even with no config override —
    # it becomes current and is applied at load.
    class BadRangeDefault(RegisterDriver):
        def configure(self, config):
            self.declare_param("gain", values=[0, 100], default=999,
                               param_type="range", kind="live")

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_analog(0)
            return Sample(raw)
    try:
        BadRangeDefault().compile()
        assert False, "expected CompileError for out-of-range range default"
    except CompileError:
        pass


def test_declare_param_rejects_out_of_set_enum_default():
    class BadEnumDefault(RegisterDriver):
        def configure(self, config):
            self.set_sample_size(2)
            self.declare_param("mode", values=[1, 2, 4], default=3,
                               param_type="enum")

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_analog(0)
            return Sample(raw)
    try:
        BadEnumDefault().compile()
        assert False, "expected CompileError for out-of-set enum default"
    except CompileError:
        pass


def test_param_value_rejects_non_integer_override():
    # A non-integer override (e.g. a YAML string) must be a clean CompileError,
    # not a TypeError from the range comparison.
    try:
        PwmDriver().compile({"pwm_freq": "fast"})
        assert False, "expected CompileError for non-integer override"
    except CompileError:
        pass


def test_declare_range_param_requires_live():
    # The firmware rejects non-live range params; the host must too. kind
    # defaults to "reload", so this also guards the forgotten-kind='live' case.
    class ReloadRange(RegisterDriver):
        def configure(self, config):
            self.declare_param("gain", values=[0, 100], default=50,
                               param_type="range", kind="reload")

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_analog(0)
            return Sample(raw)
    try:
        ReloadRange().compile()
        assert False, "expected CompileError for non-live range param"
    except CompileError:
        pass


def test_declare_param_rejects_unknown_param_type():
    class BadType(RegisterDriver):
        def configure(self, config):
            self.declare_param("x", values=[1, 2], default=1, param_type="rnage")

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_analog(0)
            return Sample(raw)
    try:
        BadType().compile()
        assert False, "expected CompileError for unknown param_type"
    except CompileError:
        pass


# ── 'at' field placement + layout guards ───────────────────

def _at_driver(fields, sample_size):
    class AtDriver(RegisterDriver):
        BUSES = ("i2c",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "at-placement test double"

        def configure(self, config):
            self.set_output(fields)
            self.set_sample_size(sample_size)

        @RegisterDriver.measure_loop(trigger="drdy")
        def measure(self):
            raw = self.read_burst(0x00, 14)
            return Sample(raw)
    return AtDriver


def test_at_offsets_round_trip():
    # Scattered offsets survive compile → serialize → deserialize.
    cd = _at_driver([
        {'name': 'fix', 'type': 'uint8', 'at': 24, 'scale': 1.0, 'unit': ''},
        {'name': 'latitude', 'type': 'int32', 'byte_order': 'little',
         'at': 32, 'scale': 1e-9},
    ], 98)().compile({})
    offs = {f['name']: f['byte_off'] for f in cd.output_fields}
    assert offs == {'fix': 24, 'latitude': 32}
    from nxs.image import serialize, deserialize
    rt = deserialize(serialize(cd))
    assert {f['name']: f['byte_off'] for f in rt.output_fields} == offs
    assert rt.sample_size == 36  # furthest field end: 32 + 4


def test_sequential_fields_stamped():
    cd = _at_driver([
        {'name': 'a', 'type': 'int16', 'scale': 1.0, 'unit': ''},
        {'name': 'b', 'type': 'int32', 'scale': 1.0, 'unit': ''},
        {'name': 'c', 'type': 'uint8', 'scale': 1.0, 'unit': ''},
    ], 7)().compile({})
    assert [f['byte_off'] for f in cd.output_fields] == [0, 2, 6]


def test_at_mixing_rejected():
    import pytest
    with pytest.raises(CompileError, match="mixes fields"):
        _at_driver([
            {'name': 'a', 'type': 'int16', 'at': 0, 'scale': 1.0, 'unit': ''},
            {'name': 'b', 'type': 'int16', 'scale': 1.0, 'unit': ''},
        ], 4)().compile({})


def test_at_overlap_rejected():
    import pytest
    with pytest.raises(CompileError, match="overlap"):
        _at_driver([
            {'name': 'a', 'type': 'int32', 'at': 0, 'scale': 1.0, 'unit': ''},
            {'name': 'b', 'type': 'int16', 'at': 2, 'scale': 1.0, 'unit': ''},
        ], 6)().compile({})


def test_field_past_buffer_rejected():
    import pytest
    with pytest.raises(CompileError, match="outside"):
        _at_driver([
            {'name': 'a', 'type': 'int32', 'at': 126, 'scale': 1.0,
             'unit': ''},
        ], 96)().compile({})


def test_too_many_output_fields_rejected():
    fields = [{'name': f'g{i}', 'type': 'uint8', 'scale': 1.0, 'unit': ''}
              for i in range(17)]
    import pytest
    with pytest.raises(CompileError, match="descriptor table"):
        _at_driver(fields, 17)().compile({})


def test_set_sample_size_bounded():
    import pytest
    with pytest.raises(CompileError, match="sample buffer holds"):
        _at_driver([
            {'name': 'a', 'type': 'uint8', 'scale': 1.0, 'unit': ''},
        ], 129)().compile({})


# ── Stream read bounds ──────────────────────────────────────

def _stream_driver(measure_fn):
    class Bounds(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.set_output([{'name': 'data', 'type': 'string', 'count': 96,
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(96)
    Bounds.measure = measure_fn
    return Bounds


def test_read_n_bounded_by_sample_buffer():
    @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
    def measure(self):
        self.read_n(129)
        self.store_sample_n()
    import pytest
    with pytest.raises(CompileError, match="read_n"):
        _stream_driver(measure)().compile()


def test_read_until_max_bounded_by_sample_buffer():
    @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
    def measure(self):
        self.read_until(b'\n', max=129)
        self.store_sample_n()
    import pytest
    with pytest.raises(CompileError, match="read_until"):
        _stream_driver(measure)().compile()


# ── verify_checksum (RX Fletcher verify) ────────────────────

class UbxVerifyDriver(StreamDriver):
    def configure(self, config):
        self.set_baud(38400)
        self.set_output([{'name': 'data', 'type': 'string', 'count': 98,
                          'scale': 1.0, 'unit': ''}])
        self.set_sample_size(98)

    @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
    def measure(self):
        self.read_until(b'\xb5\x62')
        self.read_n(98)
        m = self.match(0x01, 0x07, 0x5C, 0x00)
        if m != 0:
            return None
        bad = self.verify_checksum(ChecksumFletcher(), 0, 96, 96)
        if bad != 0:
            return None
        self.store_sample_n()


def test_verify_checksum_emits_fletcher_compare():
    cd = UbxVerifyDriver().compile()
    bc = cd.bytecode
    # The Fletcher loop + the received-byte compares.
    assert Op.LOAD_U8_REG in bc     # loop byte fetch
    assert Op.ADD_REG in bc         # ck accumulation
    assert Op.XOR_REG in bc         # computed-vs-received compare
    assert Op.STORE_SAMPLE_N in bc  # cursor-count commit still follows


def test_verify_checksum_rejects_non_fletcher():
    class Bad(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.set_output([{'name': 'data', 'type': 'string', 'count': 8,
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(8)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            bad = self.verify_checksum(ChecksumXorFold(), 0, 6, 6)
            if bad != 0:
                return None
            self.store_sample_n()
    import pytest
    with pytest.raises(CompileError, match="ChecksumFletcher"):
        Bad().compile()


def test_verify_checksum_bounds_checked():
    class Past(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.set_output([{'name': 'data', 'type': 'string', 'count': 8,
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(8)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            bad = self.verify_checksum(ChecksumFletcher(), 0, 96, 127)
            if bad != 0:
                return None
            self.store_sample_n()
    import pytest
    with pytest.raises(CompileError, match="outside"):
        Past().compile()


# ── Config-selected measure variants ────────────────────────

class DualProtocolDriver(StreamDriver):
    def configure(self, config):
        self.set_baud(38400)
        self.set_output([{'name': 'data', 'type': 'string', 'count': 96,
                          'scale': 1.0, 'unit': ''}])
        self.set_sample_size(96)

    @SensorDriver.measure_loop(trigger="poll", sample_rate=100,
                               when=("protocol", "ubx"), default=True)
    def measure_ubx(self):
        self.read_n(7)
        self.store_sample_n()

    @SensorDriver.measure_loop(trigger="poll", sample_rate=100,
                               when=("protocol", "nmea"))
    def measure_nmea(self):
        self.read_n(9)
        self.store_sample_n()


def test_variant_selected_by_config():
    ubx = DualProtocolDriver().compile({'protocol': 'ubx'})
    nmea = DualProtocolDriver().compile({'protocol': 'nmea'})
    assert ubx.bytecode != nmea.bytecode
    assert ubx.config['protocol'] == 'ubx'
    assert nmea.config['protocol'] == 'nmea'


def test_variant_default_when_key_absent():
    cd = DualProtocolDriver().compile({})
    # The default variant is chosen AND the config records the choice.
    assert cd.config['protocol'] == 'ubx'
    assert cd.bytecode == DualProtocolDriver().compile(
        {'protocol': 'ubx'}).bytecode


def test_variant_unknown_value_rejected():
    import pytest
    with pytest.raises(CompileError, match="matches no measure variant"):
        DualProtocolDriver().compile({'protocol': 'sbf'})


def test_two_bare_measure_loops_rejected():
    class TwoBare(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.set_output([{'name': 'data', 'type': 'string', 'count': 8,
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(8)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure_a(self):
            self.read_n(4)
            self.store_sample_n()

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure_b(self):
            self.read_n(5)
            self.store_sample_n()
    import pytest
    with pytest.raises(CompileError, match="no `when=` selectors"):
        TwoBare().compile()


def test_mixed_bare_and_when_rejected():
    class Mixed(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.set_output([{'name': 'data', 'type': 'string', 'count': 8,
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(8)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10,
                                   when=("protocol", "a"), default=True)
        def measure_a(self):
            self.read_n(4)
            self.store_sample_n()

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure_b(self):
            self.read_n(5)
            self.store_sample_n()
    import pytest
    with pytest.raises(CompileError, match="carry no `when=`"):
        Mixed().compile()


def test_variants_without_default_need_the_key():
    class NoDefault(StreamDriver):
        def configure(self, config):
            self.set_baud(38400)
            self.set_output([{'name': 'data', 'type': 'string', 'count': 8,
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(8)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10,
                                   when=("protocol", "a"))
        def measure_a(self):
            self.read_n(4)
            self.store_sample_n()

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10,
                                   when=("protocol", "b"))
        def measure_b(self):
            self.read_n(5)
            self.store_sample_n()
    import pytest
    with pytest.raises(CompileError, match="default=True"):
        NoDefault().compile()
    # With the key given, selection works without a default.
    assert NoDefault().compile({'protocol': 'b'}).config['protocol'] == 'b'


# ── Run all tests ───────────────────────────────────────────

if __name__ == "__main__":
    test_funcs = [v for k, v in sorted(globals().items())
                  if k.startswith("test_") and callable(v)]
    for fn in test_funcs:
        fn()
        print(f"  PASS  {fn.__name__}")
    print(f"\nnxs compiler: all {len(test_funcs)} tests passed")


# ── FRAME RX verification (compiler-composed CRC + status checks) ──


def _instr_walk(bc):
    from nxs.opcodes import INSTRUCTION_SIZE
    pos = 0
    while pos < len(bc):
        op = bc[pos]
        size = INSTRUCTION_SIZE.get(op) or 1
        if op == Op.MEMCPY_IMM:
            size = 3 + bc[pos + 2]
        yield pos, op, size
        pos += size


def _verified_driver(frame, words=2):
    # The measure body needs a literal word count (the AST compiler
    # evaluates constants, not closures), so pick a fixed-body class.
    class Verified2(RegisterDriver):
        BUSES = ("spi",)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test double"
        FRAME = frame

        def configure(self, config):
            self.set_output([
                {'name': f'f{i}', 'type': 'uint16', 'scale': 1.0, 'unit': ''}
                for i in range(2)])
            self.set_sample_size(4)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_words(0x00, 2)
            return Sample(raw)

    class Verified3(Verified2):
        def configure(self, config):
            self.set_output([
                {'name': f'f{i}', 'type': 'uint16', 'scale': 1.0, 'unit': ''}
                for i in range(3)])
            self.set_sample_size(6)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_words(0x00, 3)
            return Sample(raw)

    return {2: Verified2, 3: Verified3}[words]


def test_frame_crc_and_status_emit_checks_per_harvested_word():
    """Declaring crc + status_ok on the FRAME makes read_words verify
    every harvested response: CRC8 (declared style) + XOR against the
    received CRC byte + backward JNZ, then mask/compare of the status
    field + backward JZ. Priming frames stay unchecked (their RX is
    not a response to anything)."""
    from nxs.framing import SpiFrame, Crc
    frame = SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, poly=0x1D, init=0xFF, xor_out=0xFF,
                covers=('rw', 'addr', 'rs', 'data'),
                feedback_style='input-lsb'),
        read_pipeline=1,
        status_ok=('rs', 0b01),
    )
    bc = _verified_driver(frame, words=3)().compile().bytecode
    crc8s = [(pos, op) for pos, op, _ in _instr_walk(bc) if op == Op.CRC8]
    assert len(crc8s) == 3, "one CRC8 per harvested word, none for priming"
    for pos, _ in crc8s:
        assert bc[pos + 3] == 0x1D    # poly
        assert bc[pos + 7] == 1       # style = input-lsb
    # Status checks ride along: one AND + CMP_EQ + JZ per word.
    ands = [op for _, op, _ in _instr_walk(bc) if op == Op.AND]
    assert len(ands) >= 3


def test_frame_crc_only_emits_no_status_check():
    from nxs.framing import SpiFrame, Crc
    frame = SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, poly=0x1D, init=0xFF, xor_out=0xFF,
                covers=('rw', 'addr', 'rs', 'data'),
                feedback_style='input-lsb'),
        read_pipeline=1,
    )
    bc = _verified_driver(frame, words=2)().compile().bytecode
    assert len([1 for _, op, _ in _instr_walk(bc) if op == Op.CRC8]) == 2
    # No status projection → no AND/CMP_EQ pairs from the checker (the
    # poll loop itself emits no AND).
    assert not [1 for _, op, _ in _instr_walk(bc) if op == Op.AND]


def test_frame_status_only_emits_no_crc():
    from nxs.framing import SpiFrame
    frame = SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('pad', 8)],
        read_pipeline=1,
        status_ok=('rs', 0b01),
    )
    bc = _verified_driver(frame, words=2)().compile().bytecode
    assert not [1 for _, op, _ in _instr_walk(bc) if op == Op.CRC8]
    assert len([1 for _, op, _ in _instr_walk(bc) if op == Op.AND]) == 2


def test_frame_without_integrity_fields_emits_no_checks():
    from nxs.framing import SpiFrame
    frame = SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 7), ('data', 16), ('pad', 8)],
        read_pipeline=1,
    )
    bc = _verified_driver(frame, words=2)().compile().bytecode
    for _, op, _ in _instr_walk(bc):
        assert op not in (Op.CRC8, Op.XOR_REG), (
            "no verification ops without declared integrity fields")


def test_frame_misaligned_cover_window_is_compile_error():
    """A CRC whose covered window ends mid-byte cannot be checked by the
    whole-byte engine — loud CompileError, never a silently skipped
    check."""
    import pytest
    from nxs.framing import SpiFrame, Crc
    frame = SpiFrame(
        width=32,
        fields=[('a', 10), ('data', 16), ('pad', 2), ('crc', 4)],
        crc=Crc(width=4, poly=0x3, covers=('a', 'data')),
        read_pipeline=1,
    )
    with pytest.raises(CompileError, match="not expressible on-device"):
        _verified_driver(frame, words=2)().compile()


def test_frame_compute_fn_crc_is_compile_error():
    import pytest
    from nxs.framing import SpiFrame, Crc
    frame = SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 7), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, compute_fn=lambda d, w: 0x42,
                covers=('rw', 'addr', 'data')),
        read_pipeline=1,
    )
    with pytest.raises(CompileError, match="not expressible on-device"):
        _verified_driver(frame, words=2)().compile()
