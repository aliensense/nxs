"""Adoption contract for the fused MC6470 eCompass driver — the first
two-die companion driver: accelerometer primary (strap 0x4C/0x6C) plus
magnetometer companion at fixed 0x0C, one 12-byte fused sample."""

from nxs.drivers.mc6470 import Mc6470
from nxs.image import serialize, deserialize
from nxs.opcodes import INSTRUCTION_SIZE, Op
from nxs.compiler import WHO_AM_I_MISMATCH_CODE, COMPANION_MISMATCH_CODE


def _walk(bytecode: bytes):
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


def test_mc6470_compiles_and_shapes():
    cd = Mc6470().compile({})
    assert cd.name == "Mc6470"
    assert cd.sample_size == 12
    assert [f['name'] for f in cd.output_fields] == [
        'accel_x', 'accel_y', 'accel_z', 'mag_x', 'mag_y', 'mag_z']
    assert cd.i2c_addrs == [0x4C, 0x6C]
    assert sorted(p.name for p in cd.params) == [
        'accel_fs', 'bus', 'mag_odr', 'mag_res', 'sample_rate']


def test_mc6470_identity_is_the_silicon_pcode_family():
    """Hardware-validated identity: silicon reads PCODE 0x10 at reg 0x3B
    (bit 4 set), contradicting the datasheet's fixed-zero claim on bits
    [7:4] — so the accepted set is the 0x1_ family, and it survives
    serialization. A regen that reverts to the datasheet-derived 0x0_
    family (which also contains 0x00, matching an empty read) reverses a
    validated decision and must not be adopted."""
    cd = Mc6470().compile({})
    d = deserialize(serialize(cd))
    assert d.who_am_i_values == [0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E]
    assert 0x00 not in d.who_am_i_values


def test_mc6470_dual_anchor_prologue():
    """Both dies are identity-checked before measuring: the accel PCODE
    set first (0xC0 on mismatch), then the bracketed mag WHO_AM_I
    (0xC1) — 'wrong primary' and 'companion missing' stay separable."""
    bc = Mc6470().compile({}).bytecode
    codes = [bc[off + 1] for off, op, _ in _walk(bc) if op == Op.ERROR]
    assert WHO_AM_I_MISMATCH_CODE in codes
    assert COMPANION_MISMATCH_CODE in codes
    assert codes.index(WHO_AM_I_MISMATCH_CODE) < \
        codes.index(COMPANION_MISMATCH_CODE)


def test_mc6470_bus_always_rests_at_primary():
    """Every companion access is bracketed: the I2C_TARGET operand
    stream alternates 0x0C / home across the whole image (prologue,
    configure writes, and the per-pass mag burst)."""
    bc = Mc6470().compile({}).bytecode
    t = [bc[off + 1] for off, op, _ in _walk(bc) if op == Op.I2C_TARGET]
    assert len(t) >= 10
    assert all(a == 0x0C for a in t[0::2])
    assert all(a == 0x00 for a in t[1::2])


def test_mc6470_param_maps():
    cd = Mc6470().compile({})
    m = {e.param_name: e.value_map for e in cd.patch_map}
    assert m['sample_rate'] == {1: 0x05, 2: 0x04, 4: 0x03, 8: 0x02,
                                16: 0x01, 32: 0x00, 64: 0x08,
                                128: 0x09, 256: 0x0A}
    # OUTCFG = range | resolution; default accel_res=14 -> 0x05 folded in.
    assert m['accel_fs'] == {2: 0x05, 4: 0x15, 8: 0x25, 16: 0x35}
    # Companion registers patch through their own (dev, reg) owner keys.
    assert m['mag_odr'] == {10: 0x88, 20: 0x90, 100: 0x98}
    assert m['mag_res'] == {14: 0x80, 15: 0x90}


def test_mc6470_integer_enums_survive_roundtrip():
    """The sub-hertz datasheet rows are excluded (integer wire); what is
    declared must round-trip exactly — no truncation, no duplicates."""
    cd = Mc6470().compile({})
    cd2 = deserialize(serialize(cd))
    sr = [p for p in cd2.params if p.name == "sample_rate"][0]
    assert sr.values == [1, 2, 4, 8, 16, 32, 64, 128, 256]
    mo = [p for p in cd2.params if p.name == "mag_odr"][0]
    assert mo.values == [10, 20, 100]


def test_mc6470_measure_is_gated_fused_read():
    """drdy loop: YIELD head, SR gate (read clears ACQ_INT), accel burst
    to [0..6), bracketed mag burst to [6..12), one STORE_SAMPLE."""
    bc = Mc6470().compile({}).bytecode
    ops = [op for _, op, _ in _walk(bc)]
    assert Op.YIELD in ops
    bursts = [(bc[off + 4], bc[off + 3], bc[off + 1] | (bc[off + 2] << 8))
              for off, op, _ in _walk(bc) if op == Op.REG_READ_BURST]
    # (into, count, reg) pairs for the fused read.
    assert (0, 6, 0x0D) in bursts    # accel XOUT_EX..ZOUT_EX
    assert (6, 6, 0x10) in bursts    # mag OUTX..OUTZ (companion)


def test_mc6470_scales():
    cd = Mc6470().compile({})
    fields = {f['name']: f for f in cd.output_fields}
    # Accel base at 14-bit: g0 / 2^13, scaled live by accel_fs.
    assert abs(fields['accel_x']['scale'] - 9.80665 / 8192.0) < 1e-12
    assert fields['accel_x']['scale_param'] == 'accel_fs'
    # Mag scale is NEGATED: datasheet Figure 3 defines the mag axes
    # anti-parallel to the accel axes; one body frame for both vectors.
    assert fields['mag_x']['scale'] == -0.15e-6
    assert 'scale_param' not in fields['mag_x'] or \
        not fields['mag_x'].get('scale_param')
