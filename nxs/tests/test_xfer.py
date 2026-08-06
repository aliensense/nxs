"""xfer: the literal full-duplex SPI verb for parity-framed parts.

A part whose wire words carry computed bits (parity over rw+address,
embedded flags) cannot use a FRAME schema — the words are not
composable from fixed-layout fields. xfer keeps the computation in
plain Python instead: the driver derives each command word at class
definition time (datasheet-as-code), stores it as an UPPER_CASE class
constant, and measure() clocks it verbatim — MEMCPY_IMM stages the
literal bytes at the scalar-scratch tail, REG_XFER transceives them
with one CS assertion per word, and in assignment position the response
loads back as an unsigned MSB-first value.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from nxs.opcodes import INSTRUCTION_SIZE, Op
from nxs.compiler import RegisterDriver, Sample, CompileError, _ASTCompiler

SLOT = _ASTCompiler.SCALAR_SCRATCH_OFF


def _walk(bytecode: bytes):
    """Yield (offset, opcode, instr_size) per instruction, handling
    OP_MEMCPY_IMM's variable-length payload. Raises on unknown opcodes
    so a codegen regression fails loudly instead of mis-counting."""
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


def _xfer_sites(bytecode: bytes):
    """Byte offsets of every MEMCPY_IMM staging into the scratch slot —
    one per xfer call."""
    return [off for off, op, _ in _walk(bytecode)
            if op == Op.MEMCPY_IMM and bytecode[off + 1] == SLOT]


def _read_word(addr):
    """Read command: parity(15) | rw=1(14) | addr(13:0), even parity
    over bits 14:0 — computed by plain Python at class definition."""
    word = (1 << 14) | addr
    return word | ((bin(word).count("1") & 1) << 15)


class ParityEncoder(RegisterDriver):
    """Fictional parity-framed magnetic encoder: 16-bit words, read
    responses pipelined one word behind, bit 14 of a response flags an
    on-chip error, and both directions carry even parity in bit 15."""
    BUSES = ('spi',)
    WHO_AM_I_VALUES = []
    WHO_AM_I_SKIP_REASON = "no identity register on this part"

    CMD_POSITION = _read_word(0x3FFE)   # 0x7FFE — even popcount, parity clear
    CMD_DIAG = _read_word(0x0000)       # 0xC000 — odd popcount, parity set

    def configure(self, config):
        self.set_output([
            {'name': 'angle', 'type': 'uint16', 'scale': 1.0},
        ])
        self.set_sample_size(2)

    @RegisterDriver.measure_loop(trigger="poll", sample_rate=100)
    def measure(self):
        self.xfer(self.CMD_POSITION)       # prime the read pipeline
        a = self.xfer(self.CMD_POSITION)   # response to the previous word
        if a & 0x4000:                      # error flag from the part
            return None
        t = a >> 8                          # fold even parity into bit 0
        p = a ^ t
        t = p >> 4
        p = p ^ t
        t = p >> 2
        p = p ^ t
        t = p >> 1
        p = p ^ t
        if p & 1:
            return None
        angle = a & 0x3FFF
        return Sample(angle=angle)


def test_parity_word_vectors():
    """The plain-Python parity helper: even parity over rw+addr lands in
    bit 15, so a word with an odd popcount gets the bit and one with an
    even popcount doesn't."""
    assert ParityEncoder.CMD_POSITION == 0x7FFE
    assert ParityEncoder.CMD_DIAG == 0xC000


def test_xfer_statement_then_assignment_shapes():
    """Statement-position xfer (pipeline priming) emits stage + XFER and
    discards the response; assignment-position adds the LOAD. Both stage
    the literal command bytes MSB-first at the scratch slot."""
    bc = ParityEncoder().compile({}).bytecode
    sites = _xfer_sites(bc)
    assert len(sites) == 2

    for off in sites:
        assert bc[off + 2] == 2                      # width bytes staged
        assert bc[off + 3: off + 5] == bytes([0x7F, 0xFE])
        xfer_off = off + 3 + 2
        assert bc[xfer_off] == Op.REG_XFER
        assert bc[xfer_off + 1] == SLOT              # tx_off
        assert bc[xfer_off + 2] == SLOT              # rx_off
        assert bc[xfer_off + 3] == 2                 # len

    prime_after = sites[0] + 5 + 4                   # past MEMCPY_IMM + XFER
    assert bc[prime_after] == Op.MEMCPY_IMM          # no LOAD: discarded
    read_after = sites[1] + 5 + 4
    assert bc[read_after] == Op.LOAD_U16_BE
    assert bc[read_after + 2] == SLOT


def test_xfer_gates_compile_to_branches():
    """The response gates (error flag, parity fold) lower to the existing
    shift/mask/xor arithmetic and conditional drops — no new opcodes."""
    from nxs.disassembler import disassemble
    text = "\n".join(disassemble(ParityEncoder().compile({}).bytecode))
    for op in ("XOR_REG", "SHR", "AND", "JZ"):
        assert op in text, f"{op} missing from:\n{text}"


def test_xfer_width_one_loads_u8():
    class W1(RegisterDriver):
        BUSES = ('spi',)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test fixture"

        def configure(self, config):
            self.set_output([
                {'name': 'raw', 'type': 'uint8', 'scale': 1.0, 'unit': ''},
            ])
            self.set_sample_size(1)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            v = self.xfer(0xA5, 1)
            return Sample(raw=v)

    bc = W1().compile({}).bytecode
    off = _xfer_sites(bc)[0]
    assert bc[off + 2] == 1
    assert bc[off + 3] == 0xA5
    load_off = off + 3 + 1 + 4
    assert bc[load_off] == Op.LOAD_U8
    assert bc[load_off + 2] == SLOT


def test_xfer_width_four_loads_be_spec():
    class W4(RegisterDriver):
        BUSES = ('spi',)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test fixture"

        def configure(self, config):
            self.set_output([
                {'name': 'raw', 'type': 'uint32', 'scale': 1.0, 'unit': ''},
            ])
            self.set_sample_size(4)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            v = self.xfer(0xDEADBEEF, width=4)
            return Sample(raw=v)

    bc = W4().compile({}).bytecode
    off = _xfer_sites(bc)[0]
    assert bc[off + 2] == 4
    assert bc[off + 3: off + 7] == bytes([0xDE, 0xAD, 0xBE, 0xEF])
    load_off = off + 3 + 4 + 4
    assert bc[load_off] == Op.LOAD
    assert bc[load_off + 2] == SLOT
    assert bc[load_off + 3] == 4        # spec: width 4, unsigned big-endian


def test_xfer_word_too_wide_raises():
    class Bad(ParityEncoder):
        @ParityEncoder.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            v = self.xfer(0x1FFFF)      # 17 bits into a 2-byte word
            return Sample(angle=v)

    try:
        Bad().compile({})
    except CompileError as e:
        assert "fit" in str(e)
    else:
        raise AssertionError("expected CompileError for an oversized word")


def test_xfer_width_out_of_range_raises():
    class Bad(ParityEncoder):
        @ParityEncoder.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            v = self.xfer(0x00, 5)
            return Sample(angle=v)

    try:
        Bad().compile({})
    except CompileError as e:
        assert "width 1..4" in str(e)
    else:
        raise AssertionError("expected CompileError for width 5")


def test_xfer_requires_spi_only_buses():
    """A driver offering an I²C binding can't clock literal SPI words —
    the image would have no defined wire behaviour on that bus."""
    class DualBus(RegisterDriver):
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test fixture"

        def configure(self, config):
            self.set_output([
                {'name': 'raw', 'type': 'uint16', 'scale': 1.0, 'unit': ''},
            ])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            v = self.xfer(0x7FFE)
            return Sample(raw=v)

    try:
        DualBus().compile({})
    except CompileError as e:
        assert "BUSES = ('spi',)" in str(e)
    else:
        raise AssertionError("expected CompileError for dual-bus xfer")


def test_xfer_rejects_sample_reaching_scratch_slot():
    """A sample that extends into the scratch tail would be corrupted by
    the staged word — reject at compile time."""
    class Big(ParityEncoder):
        def configure(self, config):
            self.set_output([
                {'name': 'angle', 'type': 'uint16', 'scale': 1.0},
            ])
            self.set_sample_size(126)

    try:
        Big().compile({})
    except CompileError as e:
        assert "scratch" in str(e)
    else:
        raise AssertionError(
            "expected CompileError for sample_size into the scratch slot")


def test_xfer_usable_in_probe():
    """probe() is traced, so the verb emits there too — a part with no
    identity register can still prime its pipeline before configure."""
    class ProbeXfer(ParityEncoder):
        def probe(self):
            self.xfer(self.CMD_DIAG)

    bc = ProbeXfer().compile({}).bytecode
    off = _xfer_sites(bc)[0]
    assert bc[off + 3: off + 5] == bytes([0xC0, 0x00])


def test_lowercase_attribute_is_not_a_constant():
    """Only UPPER_CASE class attributes resolve — trace-time instance
    state must not leak into bytecode as a stale constant."""
    class Bad(ParityEncoder):
        cmd_position = 0x7FFE

        @ParityEncoder.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            v = self.xfer(self.cmd_position)
            return Sample(angle=v)

    try:
        Bad().compile({})
    except CompileError as e:
        assert "UPPER_CASE" in str(e)
    else:
        raise AssertionError(
            "expected CompileError for a lowercase attribute constant")


def test_sleep_us_emits_inside_measure_loop():
    """sleep_us in measure() routes through the driver-verb fallback and
    lands a SLEEP_US between the wire words (inter-word CS gap)."""
    class Gapped(ParityEncoder):
        @ParityEncoder.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            self.xfer(self.CMD_POSITION)
            self.sleep_us(1)
            a = self.xfer(self.CMD_POSITION)
            angle = a & 0x3FFF
            return Sample(angle=angle)

    bc = Gapped().compile({}).bytecode
    sites = _xfer_sites(bc)
    assert len(sites) == 2
    gap_off = sites[0] + 5 + 4
    assert bc[gap_off] == Op.SLEEP_US
    assert bc[gap_off + 1] == 1
