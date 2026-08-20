"""
Bytecode disassembler for NXS VM programs.

Reads compiled bytecode and prints human-readable instruction listing.
"""

import struct
from nxs.opcodes import Op, INSTRUCTION_SIZE


def disassemble(bytecode: bytes, *, print_fn=print) -> list[str]:
    """Disassemble bytecode into human-readable lines.

    Returns list of formatted strings. Also prints via print_fn.
    """
    lines = []
    pc = 0
    while pc < len(bytecode):
        addr = pc
        opcode = bytecode[pc]
        size = INSTRUCTION_SIZE.get(opcode, 0)

        if size == 0:
            line = f"  {addr:04X}  ??? (0x{opcode:02X})"
            lines.append(line)
            print_fn(line)
            pc += 1
            continue

        # Variable-length opcodes: MEMCPY_IMM has `len` inline bytes
        # after the 3-byte header. Peek at byte[pc+2] to find the
        # full instruction size.
        if opcode == Op.MEMCPY_IMM and pc + 3 <= len(bytecode):
            size = 3 + bytecode[pc + 2]

        instr = bytecode[pc:pc + size]
        text = _format_instruction(addr, opcode, instr)
        lines.append(text)
        print_fn(text)
        pc += size

    return lines


def _fmt_jmp(addr, instr):
    offset = struct.unpack_from("<h", instr, 1)[0]
    return f"-> 0x{addr + offset:04X} (offset {offset:+d})"

def _fmt_jcond(addr, instr):
    reg = instr[1]
    offset = struct.unpack_from("<h", instr, 2)[0]
    cond = "!= 0" if Op(instr[0]) == Op.JNZ else "== 0"
    return f"r{reg} {cond} -> 0x{addr + offset:04X}"

def _fmt_alu(addr, instr):
    reg, imm, dst = instr[1], struct.unpack_from("<L", instr, 2)[0], instr[6]
    op = Op(instr[0])
    if op == Op.CMP_EQ:
        return f"r{dst} = (r{reg} == 0x{imm:X})"
    elif op == Op.AND:
        return f"r{dst} = r{reg} & 0x{imm:X}"
    else:
        return f"r{dst} = r{reg} | 0x{imm:X}"

def _fmt_crc8(addr, instr):
    src_off, length, poly, init, xor_out, dst, style = instr[1:8]
    style_name = "input-lsb" if style else "standard"
    return (f"r{dst} = crc8(buf[{src_off}..+{length}], "
            f"poly=0x{poly:02X}, init=0x{init:02X}, xor_out=0x{xor_out:02X}, "
            f"style={style_name})")

_FORMATTERS = {
    Op.NOP: lambda a, i: "",
    Op.HALT: lambda a, i: "",
    Op.YIELD: lambda a, i: "",
    Op.STORE_SAMPLE: lambda a, i: "",
    Op.ERROR: lambda a, i: f"code={i[1]}",
    Op.LOAD_IMM: lambda a, i: f"r{i[1]} = 0x{struct.unpack_from('<L', i, 2)[0]:08X}",
    Op.MOV: lambda a, i: f"r{i[1]} = r{i[2]}",
    Op.CMP_EQ: _fmt_alu,
    Op.AND: _fmt_alu,
    Op.OR: _fmt_alu,
    Op.JMP: _fmt_jmp,
    Op.JNZ: _fmt_jcond,
    Op.JZ: _fmt_jcond,
    Op.REG_WRITE: lambda a, i: f"[0x{i[1] | (i[2] << 8):02X}] = 0x{i[3]:02X}",
    Op.REG_READ: lambda a, i: f"r{i[3]} = [0x{i[1] | (i[2] << 8):02X}]",
    Op.REG_READ_BURST: lambda a, i: f"buf[{i[4]}..+{i[3]}] = [0x{i[1] | (i[2] << 8):02X}]",
    Op.REG_XFER: lambda a, i: f"xfer tx=buf[{i[1]}] rx=buf[{i[2]}] len={i[3]}",
    Op.CRC8: _fmt_crc8,
    Op.BUS_WRITE_RAW: lambda a, i: f"bus.write(buf[{i[1]}..+{i[2]}])",
    Op.BUS_READ_RAW: lambda a, i: f"buf[{i[1]}..+{i[2]}] = bus.read()",
    Op.ADC_READ: lambda a, i: f"buf[{i[2]}..+2] = adc(ch={i[1]})",
    Op.I2C_TARGET: lambda a, i: (
        "i2c_target home" if i[1] == 0 else f"i2c_target 0x{i[1]:02X}"),
    Op.MEMCPY_IMM: lambda a, i: (
        f"buf[{i[1]}..+{i[2]}] = "
        + " ".join(f"{b:02X}" for b in i[3:3 + i[2]])
    ),
    Op.MEMCPY: lambda a, i: f"buf[{i[1]}..+{i[3]}] = buf[{i[2]}..+{i[3]}]",
    Op.UART_WRITE: lambda a, i: f"tx 0x{i[1]:02X}",
    Op.UART_CONFIGURE: lambda a, i: f"baud={struct.unpack_from('<L', i, 1)[0]}",
    Op.UART_READ: lambda a, i: f"buf[{i[2]}..+{i[1]}] = uart_rx",
    Op.UART_AVAIL: lambda a, i: f"r{i[1]} = uart_avail",
    Op.SLEEP_MS: lambda a, i: f"{struct.unpack_from('<H', i, 1)[0]} ms",
    Op.SLEEP_US: lambda a, i: f"{struct.unpack_from('<H', i, 1)[0]} us",
    Op.ACQ_FRAME: lambda a, i: "acq = rx_backlog_start",
    Op.ACQ_BIAS: lambda a, i: f"acq_bias = {struct.unpack_from('<L', i, 1)[0]} us",
    Op.SET_SAMPLE_SIZE: lambda a, i: f"size={i[1]}",
    Op.REG_WRITE_BURST: lambda a, i: f"[0x{i[1] | (i[2] << 8):02X}] = buf[{i[3]}..+{i[4]}]",
    Op.CVT64: lambda a, i: f"work[{i[2]}] = (i64) r{i[1]}",
    Op.MUL64: lambda a, i: f"work[{i[3]}] = work[{i[1]}] * work[{i[2]}]",
    Op.ADD64: lambda a, i: f"work[{i[3]}] = work[{i[1]}] + work[{i[2]}]",
    Op.SUB64: lambda a, i: f"work[{i[3]}] = work[{i[1]}] - work[{i[2]}]",
    Op.SHR64: lambda a, i: f"work[{i[3]}] = work[{i[1]}] >> {i[2]}",
    Op.SHL64: lambda a, i: f"work[{i[3]}] = work[{i[1]}] << {i[2]}",
    Op.TRUNC64: lambda a, i: f"r{i[2]} = (i32) work[{i[1]}]",
}


def _format_instruction(addr: int, opcode: int, instr: bytes) -> str:
    """Format a single instruction as a readable string."""
    hex_bytes = " ".join(f"{b:02X}" for b in instr)
    op = Op(opcode)
    name = op.name

    detail = _FORMATTERS.get(op, lambda _a, _i: "")(addr, instr)

    return f"  {addr:04X}  {hex_bytes:<20s} {name:<18s} {detail}"
