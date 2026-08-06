"""write_modify: on-device read-modify-write for FRAME registers.

The verb exists for registers whose reserved bits carry undocumented
factory state: the datasheet mandates "read the whole register first,
change the desired bits only", and a trace-time read cannot satisfy
that (it is a mock). write_modify reads on-device, masks/sets, and
writes back with the frame CRC recomputed on-device by a bit-serial
RISC loop.

The core test executes the emitted bytecode in a small interpreter
against a scripted bus and asserts the written frame equals
``FRAME.compose(rw=1, addr, data=expected)`` — compose computes the
CRC through ``framing.Crc`` (the datasheet-verified reference), so one
byte-equality assert proves bit preservation AND that the RISC loop
implements the exact CRC algorithm, for both feedback styles.
"""

import sys
import os
import struct

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from nxs.opcodes import INSTRUCTION_SIZE, Op
from nxs.compiler import RegisterDriver, Sample, CompileError
from nxs.framing import Crc, SpiFrame


def _lsb_frame():
    """The industrial 32-bit shape: trailing CRC-8, input-lsb feedback."""
    return SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, poly=0x1D, init=0xFF, xor_out=0xFF,
                covers=('rw', 'addr', 'rs', 'data'),
                feedback_style='input-lsb'),
        read_pipeline=1,
        inter_frame_sleep_ms=1,
    )


def _std_frame():
    """Same layout with textbook (standard) CRC feedback."""
    return SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, poly=0x31, init=0x00, xor_out=0x00,
                covers=('rw', 'addr', 'rs', 'data'),
                feedback_style='standard'),
        read_pipeline=1,
    )


def _emit_write_modify(frame, reg, **kwargs) -> bytes:
    """Emit one write_modify in isolation on a bare driver instance."""
    class D(RegisterDriver):
        BUSES = ('spi',)
        FRAME = frame

    d = D()
    d._trace_phase = "configure"
    d.write_modify(reg, **kwargs)
    return d._emitter.build()


class _Vm:
    """Interpreter for the opcode subset write_modify emits. Mirrors
    the firmware semantics: 32-bit registers, 128-byte sample buffer,
    jump offsets relative to the jump instruction's start."""

    def __init__(self, rx_script):
        self._buf = bytearray(128)
        self._regs = [0] * 8
        self._xfers = []          # recorded tx bytes per REG_XFER
        self._rx_script = list(rx_script)

    def run(self, bc: bytes):
        pc = 0
        steps = 0
        while pc < len(bc):
            steps += 1
            assert steps < 10_000, "runaway program"
            op = bc[pc]
            if op == Op.MEMCPY_IMM:
                dst, n = bc[pc + 1], bc[pc + 2]
                self._buf[dst:dst + n] = bc[pc + 3:pc + 3 + n]
                pc += 3 + n
                continue
            size = INSTRUCTION_SIZE[op]
            a = bc[pc + 1:pc + size]
            if op == Op.REG_XFER:
                tx_off, rx_off, n = a[0], a[1], a[2]
                self._xfers.append(bytes(self._buf[tx_off:tx_off + n]))
                rx = self._rx_script.pop(0) if self._rx_script else bytes(n)
                self._buf[rx_off:rx_off + n] = rx[:n]
            elif op == Op.LOAD_U16_BE:
                self._regs[a[0]] = (self._buf[a[1]] << 8) | self._buf[a[1] + 1]
            elif op == Op.LOAD_IMM:
                self._regs[a[0]] = struct.unpack("<L", a[1:5])[0]
            elif op == Op.LOAD_U8_REG:
                self._regs[a[0]] = self._buf[self._regs[a[1]]]
            elif op == Op.STORE_U8:
                self._buf[a[0]] = self._regs[a[1]] & 0xFF
            elif op in (Op.AND, Op.OR, Op.XOR, Op.ADD, Op.SUB, Op.CMP_EQ):
                reg, imm, dst = a[0], struct.unpack("<L", a[1:5])[0], a[5]
                v = self._regs[reg]
                if op == Op.AND:
                    r = v & imm
                elif op == Op.OR:
                    r = v | imm
                elif op == Op.XOR:
                    r = v ^ imm
                elif op == Op.ADD:
                    r = (v + imm) & 0xFFFFFFFF
                elif op == Op.SUB:
                    r = (v - imm) & 0xFFFFFFFF
                else:
                    r = 1 if v == imm else 0
                self._regs[dst] = r
            elif op == Op.SHR:
                self._regs[a[2]] = self._regs[a[0]] >> a[1]
            elif op == Op.SHL:
                self._regs[a[2]] = (self._regs[a[0]] << a[1]) & 0xFFFFFFFF
            elif op == Op.XOR_REG:
                self._regs[a[2]] = self._regs[a[0]] ^ self._regs[a[1]]
            elif op in (Op.JZ, Op.JNZ):
                off = struct.unpack("<h", a[1:3])[0]
                taken = ((self._regs[a[0]] == 0) if op == Op.JZ
                         else (self._regs[a[0]] != 0))
                if taken:
                    pc += off
                    continue
            elif op in (Op.SLEEP_US, Op.SLEEP_MS):
                pass
            else:
                raise AssertionError(f"unexpected opcode 0x{op:02X} at {pc}")
            pc += size

    @property
    def xfers(self):
        return self._xfers


def _run_rmw(frame, reg, reg_value, **kwargs):
    """Emit + execute one write_modify against a device whose register
    reads back `reg_value`; return the recorded write frame."""
    bc = _emit_write_modify(frame, reg, **kwargs)
    # The pipelined read issues two XFERs; the response to the request
    # arrives in the second one. The write is the third XFER.
    response = frame.compose(rw=0, addr=reg, data=reg_value)
    vm = _Vm(rx_script=[bytes(4), response])
    vm.run(bc)
    assert len(vm.xfers) == 3, vm.xfers
    return vm.xfers[2]


def test_input_lsb_rmw_preserves_reserved_bits():
    """The wedge-killer: reserved bits (0x0A07) read from the device
    survive the write verbatim; only the set bit changes. Byte equality
    against compose() proves the on-device CRC loop matches the
    datasheet-verified reference algorithm."""
    frame = _lsb_frame()
    wrote = _run_rmw(frame, 0x17, reg_value=0x0A07, set_bits=0x1000)
    assert wrote == frame.compose(rw=1, addr=0x17, data=0x1A07)


def test_input_lsb_rmw_clear_bits():
    frame = _lsb_frame()
    wrote = _run_rmw(frame, 0x08, reg_value=0xBEEF,
                     set_bits=0x0001, clear_bits=0x00F0)
    assert wrote == frame.compose(rw=1, addr=0x08, data=0xBE0F)


def test_standard_feedback_rmw():
    """The same emission parameterized to textbook feedback."""
    frame = _std_frame()
    wrote = _run_rmw(frame, 0x11, reg_value=0x8421, set_bits=0x0100)
    assert wrote == frame.compose(rw=1, addr=0x11, data=0x8521)


def test_rmw_value_is_never_a_compile_time_constant():
    """The written data bytes must come from the device, not the image:
    two runs with different device values produce different frames from
    the same bytecode."""
    frame = _lsb_frame()
    a = _run_rmw(frame, 0x17, reg_value=0x0007, set_bits=0x1000)
    b = _run_rmw(frame, 0x17, reg_value=0x0A07, set_bits=0x1000)
    assert a != b
    assert a == frame.compose(rw=1, addr=0x17, data=0x1007)
    assert b == frame.compose(rw=1, addr=0x17, data=0x1A07)


def test_write_modify_requires_frame():
    class Plain(RegisterDriver):
        BUSES = ('spi',)

    d = Plain()
    d._trace_phase = "configure"
    try:
        d.write_modify(0x17, set_bits=0x1000)
    except CompileError as e:
        assert "FRAME" in str(e)
    else:
        raise AssertionError("expected CompileError without a FRAME")


def test_write_modify_rejected_in_measure():
    class D(RegisterDriver):
        BUSES = ('spi',)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test fixture"
        FRAME = _lsb_frame()

        def configure(self, config):
            self.set_output([{'name': 'raw', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            self.write_modify(0x17, set_bits=0x1000)
            raw = self.read_words(0x00, 1)
            return Sample(raw)

    try:
        D().compile({})
    except CompileError as e:
        assert "configure" in str(e)
    else:
        raise AssertionError("expected CompileError for measure() use")


def test_write_modify_rejects_noop_and_overlap():
    frame = _lsb_frame()
    for kwargs, needle in (
            (dict(), "no-op"),
            (dict(set_bits=0x0100, clear_bits=0x0100), "overlap"),
            (dict(set_bits=0x10000), "16-bit")):
        try:
            _emit_write_modify(frame, 0x17, **kwargs)
        except CompileError as e:
            assert needle in str(e)
        else:
            raise AssertionError(f"expected CompileError for {kwargs}")


def test_write_modify_rejects_unsupported_frame_shapes():
    # CRC not the trailing field / not covering every preceding field.
    bad = SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 7), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, poly=0x1D, init=0xFF, xor_out=0xFF,
                covers=('data',),        # partial coverage
                feedback_style='input-lsb'),
    )
    try:
        _emit_write_modify(bad, 0x17, set_bits=0x1000)
    except CompileError as e:
        assert "covers" in str(e)
    else:
        raise AssertionError("expected CompileError for partial CRC coverage")


def test_full_driver_with_write_modify_compiles():
    """Integration: a FRAME driver enabling a pin via write_modify in
    configure() compiles; the image carries the RISC CRC loop."""
    class Enabler(RegisterDriver):
        BUSES = ('spi',)
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "test fixture"
        FRAME = _lsb_frame()

        def configure(self, config):
            self.write(0x19, 0x0055)             # plain unlock word
            self.write_modify(0x17, set_bits=0x1000)
            self.set_output([{'name': 'raw', 'type': 'uint16',
                              'scale': 1.0, 'unit': ''}])
            self.set_sample_size(2)

        @RegisterDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_words(0x00, 1)
            return Sample(raw)

    cd = Enabler().compile({})
    assert Op.LOAD_U8_REG in cd.bytecode
    assert Op.JNZ in cd.bytecode
