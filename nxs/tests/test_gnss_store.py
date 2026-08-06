"""Adoption contract for the GNSS store primitive.

A fixed-length UBX-NAV-PVT frame commits with STORE_SAMPLE (the
compile-time set_sample_size); the variable-length NMEA sentence commits
with STORE_SAMPLE_N (the __cursor byte count from read_until). Using
store_sample_n() after read_n publishes a stale __cursor that an
intervening match / verify_checksum overwrites — the SAMPLE_SIZE-churn
bug found on the SerDes bench (device reported 22/36/128 instead of 98).
A regen of either u-blox driver must keep this split.
"""

from nxs.drivers.neo_m9n import NeoM9n
from nxs.drivers.zed_f9p import ZedF9p
from nxs.opcodes import INSTRUCTION_SIZE, Op


def _opcodes(bytecode: bytes) -> set:
    """The opcodes actually executed, skipping operand bytes so an
    immediate equal to STORE_SAMPLE's value is not a false hit."""
    ops = set()
    pos = 0
    while pos < len(bytecode):
        op = bytecode[pos]
        size = INSTRUCTION_SIZE.get(op)
        if size is None:
            raise ValueError(f"unknown opcode 0x{op:02X} at +{pos}")
        if op == Op.MEMCPY_IMM:
            size = 3 + bytecode[pos + 2]
        ops.add(op)
        pos += size
    return ops


def _assert_store_split(cls, binary_size: int):
    binary = cls().compile({})                    # default protocol = binary
    ops = _opcodes(binary.bytecode)
    assert Op.STORE_SAMPLE in ops, "fixed UBX frame must commit with store_sample()"
    assert Op.STORE_SAMPLE_N not in ops, \
        "fixed frame must not use store_sample_n() (stale __cursor)"
    assert binary.sample_size == binary_size

    nmea = cls().compile({"protocol": "nmea"})
    assert Op.STORE_SAMPLE_N in _opcodes(nmea.bytecode), \
        "variable NMEA sentence keeps store_sample_n()"


def test_neo_m9n_store_split():
    _assert_store_split(NeoM9n, 98)


def test_zed_f9p_store_split():
    _assert_store_split(ZedF9p, 98)


if __name__ == "__main__":
    test_neo_m9n_store_split()
    test_zed_f9p_store_split()
    print("GNSS store-primitive adoption: OK")
