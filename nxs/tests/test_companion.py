"""Companion I2C devices: one driver, two co-resident slaves on one bus.

Multi-die packages expose a second I2C slave at its own fixed address,
with a register map that may overlap the primary's. The driver declares
it in ``I2C_COMPANIONS`` and reaches it per access with ``dev=`` — each
access is bracketed (``I2C_TARGET addr`` … access … ``I2C_TARGET 0``),
so the bus always rests at the strap-scanned primary and no control
path (drop-and-loop, probe retry) can start on the wrong die.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from nxs.opcodes import INSTRUCTION_SIZE, Op
from nxs.compiler import (
    RegisterDriver, Sample, CompileError,
    WHO_AM_I_MISMATCH_CODE, COMPANION_MISMATCH_CODE,
)


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


def _targets(bytecode: bytes):
    """Every I2C_TARGET operand, in emission order."""
    return [bytecode[off + 1] for off, op, _ in _walk(bytecode)
            if op == Op.I2C_TARGET]


class TwoDieCombo(RegisterDriver):
    """Fictional two-die motion combo: a primary die on a strap-scanned
    address plus an auxiliary die at a fixed address, register maps
    overlapping. The aux die runs single-shot conversions triggered per
    pass and read back the next pass."""
    BUSES = ('i2c',)
    PINS = {'drdy': 'mkbus_int'}

    WHO_AM_I_REG = 0x00
    WHO_AM_I_VALUES = [0x21]
    I2C_ADDRS = [0x30, 0x31]

    I2C_COMPANIONS = {
        'aux': {'addr': 0x0D, 'who_am_i_reg': 0x0F,
                'who_am_i_values': [0x33]},
    }

    def __init__(self):
        super().__init__()
        self._read_responses = {
            self.WHO_AM_I_REG: [0x21],
            ('aux', 0x0F): [0x33],
        }

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)
        assert who == 0x21
        aux = self.read(0x0F, dev='aux')
        assert aux == 0x33

    def configure(self, config):
        self.declare_param("sample_rate", values=[10, 50, 100],
                           default=100, unit="Hz")
        sr = config.get('sample_rate', 100)
        self.write(0x08, {10: 0x05, 50: 0x03, 100: 0x01}[sr],
                   param=("sample_rate", sr))
        # Companion init: same register NUMBER as the primary's rate
        # register — a different die, so no patch-owner collision.
        self.write(0x08, 0x90, dev='aux')
        self.write(0x1B, 0x82, dev='aux')
        self.set_output([
            {'name': 'accel_x', 'scale': 1.0},
            {'name': 'accel_y', 'scale': 1.0},
            {'name': 'accel_z', 'scale': 1.0},
            {'name': 'mag_x', 'scale': 1.0},
            {'name': 'mag_y', 'scale': 1.0},
            {'name': 'mag_z', 'scale': 1.0},
        ])
        self.set_sample_size(12)

    @RegisterDriver.measure_loop(trigger="from_config")
    def measure(self):
        status = self.read(0x03)
        if not (status & 0x80):
            return None
        raw = self.read_burst(0x0D, 6)
        stat = self.read(0x18, dev='aux')
        if stat & 0x40:
            self.read_burst(0x10, 6, into=6, dev='aux')
        self.write(0x1D, 0x40, dev='aux')   # trigger the next conversion
        return Sample(raw)


def test_prologue_checks_both_dies_and_rests_at_home():
    """Primary WHO_AM_I first (code 0xC0 on mismatch), then the bracketed
    companion check (code 0xC1), ending with the home restore — so a
    probe retry that reloads without re-binding starts on the primary."""
    cd = TwoDieCombo().compile({})
    bc = cd.bytecode

    err_codes = [bc[off + 1] for off, op, _ in _walk(bc) if op == Op.ERROR]
    assert WHO_AM_I_MISMATCH_CODE in err_codes
    assert COMPANION_MISMATCH_CODE in err_codes
    assert err_codes.index(WHO_AM_I_MISMATCH_CODE) < \
        err_codes.index(COMPANION_MISMATCH_CODE)

    # The first target pair is the companion check: 0x0D then home.
    t = _targets(bc)
    assert t[0] == 0x0D and t[1] == 0x00


def test_every_companion_access_is_bracketed():
    """Every I2C_TARGET to the companion is followed (after its access)
    by a home restore: the operand stream alternates 0x0D, 0x00, so no
    path leaves the bus on the companion."""
    cd = TwoDieCombo().compile({})
    t = _targets(cd.bytecode)
    assert len(t) >= 10   # probe pair + 2 config writes + 3 measure accesses
    assert all(a == 0x0D for a in t[0::2])
    assert all(a == 0x00 for a in t[1::2])


def test_dev_write_patch_map_survives_bracketing():
    """The tagged primary write patches normally even though a companion
    write to the same register NUMBER exists — the owner key carries the
    device, so the dies don't collide."""
    cd = TwoDieCombo().compile({})
    entries = [e for e in cd.patch_map if e.param_name == "sample_rate"]
    assert len(entries) == 1
    assert entries[0].value_map == {10: 0x05, 50: 0x03, 100: 0x01}
    # The patched byte is the primary's rate code at the recorded offset.
    assert cd.bytecode[entries[0].offset] == 0x01


def test_unknown_dev_is_a_compile_error():
    class Bad(TwoDieCombo):
        @TwoDieCombo.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            v = self.read(0x00, dev='gyro')
            return Sample(raw=v)

    try:
        Bad().compile({})
    except CompileError as e:
        assert "aux" in str(e)   # the error lists what IS declared
    else:
        raise AssertionError("expected CompileError for an unknown dev")


def test_companions_require_i2c_only_buses():
    class DualBus(TwoDieCombo):
        BUSES = ('i2c', 'spi')

    try:
        DualBus().compile({})
    except CompileError as e:
        assert "BUSES = ('i2c',)" in str(e)
    else:
        raise AssertionError("expected CompileError for dual-bus companions")


def test_companion_addr_colliding_with_primary_strap_rejected():
    class Collide(TwoDieCombo):
        I2C_COMPANIONS = {
            'aux': {'addr': 0x30, 'who_am_i_reg': 0x0F,
                    'who_am_i_values': [0x33]},
        }

    try:
        Collide().compile({})
    except CompileError as e:
        assert "strap candidate" in str(e)
    else:
        raise AssertionError("expected CompileError for a strap collision")


def test_companion_without_identity_needs_skip_reason():
    class NoAnchor(TwoDieCombo):
        I2C_COMPANIONS = {
            'aux': {'addr': 0x0D},
        }

    try:
        NoAnchor().compile({})
    except CompileError as e:
        assert "who_am_i_skip_reason" in str(e)
    else:
        raise AssertionError("expected CompileError for a missing anchor")


def test_companion_skip_reason_skips_the_check():
    class Skipped(TwoDieCombo):
        I2C_COMPANIONS = {
            'aux': {'addr': 0x0D,
                    'who_am_i_skip_reason': "no identity register"},
        }

        def probe(self):
            who = self.read(self.WHO_AM_I_REG)
            assert who == 0x21

    cd = Skipped().compile({})
    err_codes = [cd.bytecode[off + 1]
                 for off, op, _ in _walk(cd.bytecode) if op == Op.ERROR]
    assert COMPANION_MISMATCH_CODE not in err_codes


def test_primary_opt_out_still_checks_companion():
    """A primary without an identity register (WHO_AM_I_SKIP_REASON)
    still gets the companion's prologue check."""
    class NoPrimaryId(TwoDieCombo):
        WHO_AM_I_VALUES = []
        WHO_AM_I_SKIP_REASON = "no identity register on the primary die"

        def probe(self):
            aux = self.read(0x0F, dev='aux')
            assert aux == 0x33

    cd = NoPrimaryId().compile({})
    err_codes = [cd.bytecode[off + 1]
                 for off, op, _ in _walk(cd.bytecode) if op == Op.ERROR]
    assert WHO_AM_I_MISMATCH_CODE not in err_codes
    assert COMPANION_MISMATCH_CODE in err_codes


def test_single_die_drivers_emit_no_retarget():
    """The adopted fleet is companion-free and must stay byte-free of
    I2C_TARGET — the bracket machinery costs nothing unless declared."""
    from nxs.drivers.iam20680 import Iam20680
    from nxs.drivers.fxos8700 import Fxos8700
    from nxs.drivers.iim20670 import Iim20670

    for cls, cfg in ((Iam20680, {}), (Fxos8700, {}), (Iim20670, {})):
        bc = cls().compile(dict(cfg)).bytecode
        ops = {op for _, op, _ in _walk(bc)}
        assert Op.I2C_TARGET not in ops, f"{cls.__name__} emits I2C_TARGET"


def test_companion_roundtrip_through_image():
    """The companion machinery is bytecode-only: the image serializes and
    round-trips with the standard format, no descriptor change."""
    from nxs.image import serialize, deserialize
    cd = TwoDieCombo().compile({})
    cd2 = deserialize(serialize(cd))
    assert cd2.bytecode == cd.bytecode
    assert cd2.i2c_addrs == [0x30, 0x31]   # primary strap set only
