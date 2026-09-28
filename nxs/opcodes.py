# AUTO-GENERATED from constants/*.yaml. DO NOT EDIT BY HAND.
# CI regenerates this on every build via
# `python3 scripts/generate-constants.py constants/`.

"""Sensor-VM opcodes: the bytecode contract between the host compiler
and the firmware interpreter. An 8-bit opcode plus fixed-length
little-endian operands."""
# ruff: noqa: E501

from enum import IntEnum
from typing import Dict


class Op(IntEnum):
    NOP = 0x00  # No operation; advances PC.
    HALT = 0x01  # Stop the VM; transition to IDLE.
    YIELD = 0x02  # Block until the next data-ready (DRDY) event.
    ERROR = 0x03  # Signal an error to the host; transition to ERROR state.
    LOAD_IMM = 0x10  # Load a 32-bit immediate into r[dst].
    MOV = 0x11  # Copy r[src] into r[dst].
    LOAD_U16_BE = 0x12  # Zero-extend a 16-bit big-endian value from sample_buf[buf_off] into r[dst].
    LOAD_U8 = 0x13  # Zero-extend a byte from sample_buf[buf_off] into r[dst].
    LOAD_U16_LE = 0x14  # Zero-extend a 16-bit little-endian value from sample_buf[buf_off] into r[dst].
    LOAD_U8_REG = 0x15  # Load sample_buf[r[off_reg]] into r[dst] (runtime cursor index).
    LOAD = 0x16  # Load a sensor field from sample_buf[buf_off] into r[dst]. spec packs width (low 3 bits, 1-4 bytes), byte order (bit 3: 0=BE, 1=LE), and sign (bit 4: 1=sign-extend). One op covers u8/i8..u32/i32, so no sensor width needs a dedicated load.
    PARAM_LOAD = 0x17  # Load run parameter `index` into r[dst]: the value the loader seeded for this run.
    PARAM_STORE = 0x18  # Store r[src] as run parameter `index`: the value the loader reads back once the run ends.
    CMP_EQ = 0x20  # r[dst] = (r[reg] == imm) ? 1 : 0.
    AND = 0x21  # r[dst] = r[reg] & imm.
    OR = 0x22  # r[dst] = r[reg] | imm.
    CRC8 = 0x23  # Left-shift CRC-8 over sample_buf[src_off..+len]; result zero-extended into r[dst]. style selects the shift-register structure: 0 = standard (feedback = MSB XOR input; Sensirion 0x31, SMBus PEC 0x07), 1 = input-lsb (feedback = MSB only, input bit XOR'd into the LSB after the shift; industrial framed-SPI parts). Parametric poly/init/xor_out/style covers the whole CRC-8 family.
    ADD = 0x24  # r[dst] = r[src] + imm.
    SUB = 0x25  # r[dst] = r[src] - imm.
    XOR = 0x26  # r[dst] = r[src] ^ imm.
    SHL = 0x27  # r[dst] = r[src] << n (logical, n < 32).
    SHR = 0x28  # r[dst] = r[src] >> n (logical, n < 32).
    ADD_REG = 0x29  # r[dst] = r[src_a] + r[src_b].
    SUB_REG = 0x2A  # r[dst] = r[src_a] - r[src_b].
    XOR_REG = 0x2B  # r[dst] = r[src_a] ^ r[src_b].
    CMP_LT = 0x2C  # r[dst] = ((int32_t)r[reg] < (int32_t)imm) ? 1 : 0 (signed). The compiler derives >/<=/>= via operand swap + branch inversion, and != from CMP_EQ. Reserved sibling CMP_LT_U (0x2D) covers unsigned.
    MUL_REG = 0x2E  # r[dst] = r[src_a] * r[src_b], the low 32 bits of the product.
    DIVU_REG = 0x2F  # r[dst] = r[src_a] / r[src_b], unsigned and truncating. A zero divisor faults with DIV_BY_ZERO.
    JMP = 0x30  # Unconditional relative jump: PC = instruction_start + offset.
    JNZ = 0x31  # Jump if r[reg] != 0.
    JZ = 0x32  # Jump if r[reg] == 0.
    REG_WRITE = 0x40  # Write a byte to a sensor register.
    REG_READ = 0x41  # Read a byte from a sensor register into r[dst].
    REG_READ_BURST = 0x42  # Burst-read count bytes from reg into sample_buf[buf_off..].
    REG_XFER = 0x43  # Full-duplex bus transfer: len bytes out of sample_buf[tx_off..] / into sample_buf[rx_off..] (SPI framing).
    UART_WRITE = 0x44  # Write a byte to the UART TX.
    UART_CONFIGURE = 0x45  # Set the UART baud rate.
    UART_READ = 0x46  # Read up to count bytes from UART RX into sample_buf[buf_off..]; no register written.
    UART_AVAIL = 0x47  # Available UART RX byte count into r[dst].
    BUS_WRITE_RAW = 0x48  # Raw bus write of len bytes from sample_buf[src_off..] (command-based I2C; no reg prefix).
    BUS_READ_RAW = 0x49  # Raw bus read of len bytes into sample_buf[dst_off..] (no reg prefix).
    UART_READ_REG = 0x4A  # Like UART_READ but the destination offset is r[off_reg] (cursor-growing delimiter loops).
    UART_WRITE_RAW = 0x4B  # Clock len bytes from sample_buf[src_off..] to the UART TX (staged frame with runtime checksum).
    REG_WRITE_BURST = 0x4C  # Register-prefixed burst write: drive HAL write_regs(reg, sample_buf[buf_off..+len]) — the mirror of REG_READ_BURST. Covers multi-byte register payloads (RTC set-time, EEPROM, IMU config blocks) with proper I2C/SPI register framing.
    ADC_READ = 0x4D  # Sample ADC channel `ch`; write the raw count as a big-endian u16 into sample_buf[buf_off..+2]. Compiled from self.read_analog(ch).
    I2C_TARGET = 0x4E  # Retarget the bound register bus to I2C slave address `addr`; addr 0 restores the bind-latched primary. Compiled from the dev= kwarg on register verbs (companion dies on one bus); fails on non-I2C binds.
    EVENT_DIV = 0x4F  # Configure the DRDY event backend to wake YIELD on every div-th hardware edge (0/1 = every edge). Backend-config like UART_CONFIGURE; emitted once before the measure loop for a fixed-sync part whose sample_rate divides the sync. load_program resets the divider to 1.
    SLEEP_MS = 0x50  # Sleep for ms milliseconds.
    SLEEP_US = 0x51  # Sleep for us microseconds.
    ACQ_FRAME = 0x52  # Set the pass's acquisition bound to the RX backlog's first-byte arrival; now when the stream serves no stamp. Compiled from self.stamp_frame().
    ACQ_BIAS = 0x53  # Declare the driver's acquisition latency: every committed stamp is biased this many microseconds earlier. Held for a driver that declares its latency; no driver verb emits it.
    POLL_REG = 0x54  # Read register `reg` every `poll_ms` ms until `(value & mask) == val` (flags bit0: != instead) or `timeout_ms` has passed; a NAK counts as not yet. On timeout, flags bit1 (soft) continues and raises the soft-miss status flag, else the VM faults with ERR_VM_POLL_TIMEOUT. The value width follows the bus profile's data width. Compiled from self.poll(); a camera personality's alive gate.
    STORE_SAMPLE = 0x60  # Commit sample_buf to the ring slot; bump the sample counter.
    SET_SAMPLE_SIZE = 0x61  # Set the published sample size in bytes.
    MEMCPY_IMM = 0x62  # Copy `len` inline program bytes into sample_buf[dst_off..]. The data follows the header; instruction_size() returns the 3-byte header and callers add len.
    MEMCPY = 0x63  # Overlap-safe intra-sample_buf copy (memmove); strips interleaved CRC bytes before STORE_SAMPLE.
    STORE_SAMPLE_N = 0x64  # Commit sample_buf[0..r[size_reg]) with a runtime size (variable-length records: NMEA/SBF/UBX).
    STORE_U8 = 0x65  # Store the low byte of r[src] into sample_buf[buf_off].
    STORE_U8_REG = 0x66  # Store the low byte of r[src] into sample_buf[r[off_reg]] (runtime cursor).
    CVT64 = 0x70  # Sign-extend r[reg] (i32) into the 64-bit slot at work[dst_off..+8].
    MUL64 = 0x71  # i64(work[a_off]) * i64(work[b_off]) -> low 64 bits into work[dst_off..+8] (signed; covers 32x32 after CVT64).
    ADD64 = 0x72  # i64(work[a_off]) + i64(work[b_off]) -> work[dst_off..+8].
    SUB64 = 0x73  # i64(work[a_off]) - i64(work[b_off]) -> work[dst_off..+8].
    SHR64 = 0x74  # Arithmetic (signed) i64(work[src_off]) >> n -> work[dst_off..+8] (n < 64).
    SHL64 = 0x75  # i64(work[src_off]) << n -> work[dst_off..+8] (n < 64).
    TRUNC64 = 0x76  # Low 32 bits of i64(work[src_off]) -> r[dst] (final compensated result back to a register).


INSTRUCTION_SIZE: Dict[int, int] = {
    Op.NOP: 1,
    Op.HALT: 1,
    Op.YIELD: 1,
    Op.ERROR: 2,
    Op.LOAD_IMM: 6,
    Op.MOV: 3,
    Op.LOAD_U16_BE: 3,
    Op.LOAD_U8: 3,
    Op.LOAD_U16_LE: 3,
    Op.LOAD_U8_REG: 3,
    Op.LOAD: 4,
    Op.PARAM_LOAD: 3,
    Op.PARAM_STORE: 3,
    Op.CMP_EQ: 7,
    Op.AND: 7,
    Op.OR: 7,
    Op.CRC8: 8,
    Op.ADD: 7,
    Op.SUB: 7,
    Op.XOR: 7,
    Op.SHL: 4,
    Op.SHR: 4,
    Op.ADD_REG: 4,
    Op.SUB_REG: 4,
    Op.XOR_REG: 4,
    Op.CMP_LT: 7,
    Op.MUL_REG: 4,
    Op.DIVU_REG: 4,
    Op.JMP: 3,
    Op.JNZ: 4,
    Op.JZ: 4,
    Op.REG_WRITE: 4,
    Op.REG_READ: 4,
    Op.REG_READ_BURST: 5,
    Op.REG_XFER: 4,
    Op.UART_WRITE: 2,
    Op.UART_CONFIGURE: 5,
    Op.UART_READ: 3,
    Op.UART_AVAIL: 2,
    Op.BUS_WRITE_RAW: 3,
    Op.BUS_READ_RAW: 3,
    Op.UART_READ_REG: 3,
    Op.UART_WRITE_RAW: 3,
    Op.REG_WRITE_BURST: 5,
    Op.ADC_READ: 3,
    Op.I2C_TARGET: 2,
    Op.EVENT_DIV: 3,
    Op.SLEEP_MS: 3,
    Op.SLEEP_US: 3,
    Op.ACQ_FRAME: 1,
    Op.ACQ_BIAS: 5,
    Op.POLL_REG: 16,
    Op.STORE_SAMPLE: 1,
    Op.SET_SAMPLE_SIZE: 2,
    Op.MEMCPY_IMM: 3,  # header only; + len inline bytes
    Op.MEMCPY: 4,
    Op.STORE_SAMPLE_N: 2,
    Op.STORE_U8: 3,
    Op.STORE_U8_REG: 3,
    Op.CVT64: 3,
    Op.MUL64: 4,
    Op.ADD64: 4,
    Op.SUB64: 4,
    Op.SHR64: 4,
    Op.SHL64: 4,
    Op.TRUNC64: 3,
}

# NXS minor version that introduced each opcode; the compiler
# stamps an image's required-minor as the max over emitted opcodes.
OPCODE_SINCE_MINOR: Dict[int, int] = {
    Op.NOP: 0,
    Op.HALT: 0,
    Op.YIELD: 0,
    Op.ERROR: 0,
    Op.LOAD_IMM: 0,
    Op.MOV: 0,
    Op.LOAD_U16_BE: 0,
    Op.LOAD_U8: 0,
    Op.LOAD_U16_LE: 0,
    Op.LOAD_U8_REG: 0,
    Op.LOAD: 0,
    Op.PARAM_LOAD: 2,
    Op.PARAM_STORE: 2,
    Op.CMP_EQ: 0,
    Op.AND: 0,
    Op.OR: 0,
    Op.CRC8: 0,
    Op.ADD: 0,
    Op.SUB: 0,
    Op.XOR: 0,
    Op.SHL: 0,
    Op.SHR: 0,
    Op.ADD_REG: 0,
    Op.SUB_REG: 0,
    Op.XOR_REG: 0,
    Op.CMP_LT: 0,
    Op.MUL_REG: 2,
    Op.DIVU_REG: 2,
    Op.JMP: 0,
    Op.JNZ: 0,
    Op.JZ: 0,
    Op.REG_WRITE: 0,
    Op.REG_READ: 0,
    Op.REG_READ_BURST: 0,
    Op.REG_XFER: 0,
    Op.UART_WRITE: 0,
    Op.UART_CONFIGURE: 0,
    Op.UART_READ: 0,
    Op.UART_AVAIL: 0,
    Op.BUS_WRITE_RAW: 0,
    Op.BUS_READ_RAW: 0,
    Op.UART_READ_REG: 0,
    Op.UART_WRITE_RAW: 0,
    Op.REG_WRITE_BURST: 0,
    Op.ADC_READ: 0,
    Op.I2C_TARGET: 0,
    Op.EVENT_DIV: 0,
    Op.SLEEP_MS: 0,
    Op.SLEEP_US: 0,
    Op.ACQ_FRAME: 0,
    Op.ACQ_BIAS: 0,
    Op.POLL_REG: 0,
    Op.STORE_SAMPLE: 0,
    Op.SET_SAMPLE_SIZE: 0,
    Op.MEMCPY_IMM: 0,
    Op.MEMCPY: 0,
    Op.STORE_SAMPLE_N: 0,
    Op.STORE_U8: 0,
    Op.STORE_U8_REG: 0,
    Op.CVT64: 0,
    Op.MUL64: 0,
    Op.ADD64: 0,
    Op.SUB64: 0,
    Op.SHR64: 0,
    Op.SHL64: 0,
    Op.TRUNC64: 0,
}
