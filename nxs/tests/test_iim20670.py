"""End-to-end test: compile the IIM-20670 reference driver and verify
the emitted bytecode carries the expected FRAME-based opcodes.

This exercises the full path:
  Driver class (FRAME declared)
    → compiler.py (detects FRAME, emits MEMCPY_IMM + REG_XFER)
    → framing.py (composes bytes, computes CRC)
    → measure loop's read_words → FRAME-aware pipelined burst
"""

from nxs.drivers.iim20670 import Iim20670
from nxs.image import serialize, deserialize
from nxs.opcodes import INSTRUCTION_SIZE, Op


def _walk(bytecode: bytes):
    """Yield (offset, opcode, instr_size) for each instruction in
    `bytecode`. Handles OP_MEMCPY_IMM's variable-length payload.
    Raises on unknown opcodes so a codegen regression surfaces as a
    clear test failure rather than a silent mis-count."""
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


def _find_burst_start(bytecode: bytes, slot: int) -> int:
    """Byte offset of the measure loop's FRAME burst: the first
    MEMCPY_IMM staging into the rotating TX/RX slot at the buffer tail.
    probe() frames stage at offset 0, so the slot address is unique to
    the burst."""
    for off, op, _ in _walk(bytecode):
        if op == Op.MEMCPY_IMM and bytecode[off + 1] == slot:
            return off
    raise AssertionError(f"no MEMCPY_IMM staging at slot {slot} found")


def test_iim20670_compiles_without_errors():
    cd = Iim20670().compile({})
    assert cd.name == "Iim20670"
    assert cd.sample_size == 14
    assert len(cd.output_fields) == 7
    # probe + tcode unlock (6 writes) + 10 on-device RMWs (2 FS, 6 ODR
    # routing, 2 filter — each with a bit-serial CRC loop) + 4 bank
    # selects + the 8-XFER measure burst carrying a CRC + status check
    # block per harvested frame. Loose bounds guard against runaway
    # codegen without binding to one byte count; the ceiling is the VM's
    # hard program limit (SSOT constants/driver_image.yaml).
    from nxs.image import VM_MAX_PROGRAM_SIZE
    assert 1500 < len(cd.bytecode) <= VM_MAX_PROGRAM_SIZE


def test_iim20670_bytecode_uses_frame_opcodes():
    """FRAME-based drivers emit MEMCPY_IMM + REG_XFER, not the plain
    REG_READ / REG_WRITE path."""
    cd = Iim20670().compile({})
    assert Op.MEMCPY_IMM in cd.bytecode
    assert Op.REG_XFER in cd.bytecode
    # probe() reads FIXED_VALUE via the FRAME path, which extracts the
    # 16-bit data field via OP_LOAD_U16_BE.
    assert Op.LOAD_U16_BE in cd.bytecode


def test_iim20670_unlock_sequence_pinned():
    """The section-4.11 bank unlock is pinned byte-exact: six pre-CRC'd
    write frames to the MODE register, in order, before the first bank
    select. A regen that drops, reorders, or re-values a word fails
    here rather than on hardware (where the symptom is an ODR pin that
    never pulses)."""
    cd = Iim20670().compile({})
    bc = cd.bytecode
    frames = [bytes.fromhex(h) for h in
              ("E4000288", "E400018B", "E400048E",
               "E40300AD", "E4018017", "E4028030")]
    pos = 0
    for f in frames:
        idx = bc.find(f, pos)
        assert idx != -1, f"unlock frame {f.hex()} missing or out of order"
        pos = idx + len(f)
    # The first bank select (bank 6, accel full-scale) follows the walk.
    bank6 = Iim20670.FRAME.compose(rw=1, addr=0x1F, data=0x0006)
    assert bc.find(bytes(bank6), pos) != -1


def test_iim20670_odr_routing_is_read_modify_write():
    """Every bank-3 ODR routing register carries undocumented factory
    bits, so the enable must be on-device RMW (write_modify emits a
    pipelined read then a runtime-CRC'd write), never a composed
    write frame with hardcoded reserved bits."""
    cd = Iim20670().compile({})
    bc = cd.bytecode
    # A hardcoded ODR_Config_6 write frame (rw=1, addr=0x17, any data)
    # must NOT exist: byte0 would be 0x80 | 0x17<<2 = 0xDC.
    for off, op, _ in _walk(bc):
        if op == Op.MEMCPY_IMM and bc[off + 2] == 4:
            b0 = bc[off + 3]
            data = (bc[off + 4] << 8) | bc[off + 5]
            assert not (b0 == 0xDC and data != 0x0000), (
                f"hardcoded write to ODR_Config_6 at +{off}: "
                f"data=0x{data:04X} would clobber factory bits")
    # The RMW machinery is present: runtime CRC loops (LOAD_U8_REG +
    # backward JNZ) only exist in write_modify emissions.
    assert Op.LOAD_U8_REG in bc
    assert Op.JNZ in bc


def test_iim20670_sample_rate_paces_both_modes():
    """`sample_rate` has no chip register (fixed 8 kHz internal rate), so
    the knob is loop pacing in both modes. In drdy mode the compiler
    divides the ODR sync at the source: the patch site is an EVENT_DIV
    operand whose per-value map is the exact divider 8000/rate —
    crystal-derived spacing a regen must not revert to free-running or
    to a poll-only param. In poll mode the same values patch the loop's
    SLEEP_MS interval."""
    RATES = [10, 25, 50, 100, 200, 250]

    drdy = Iim20670().compile({})
    p = {pp.name: pp for pp in drdy.params}["sample_rate"]
    assert p.values == RATES and p.kind == "reload"
    entries = [e for e in drdy.patch_map if e.param_name == "sample_rate"]
    assert len(entries) == 1
    # Exact dividers of the 8 kHz sync, one per declared rate.
    assert entries[0].value_map == {r: 8000 // r for r in RATES}
    # The site is the EVENT_DIV u16 operand: the opcode byte precedes it.
    assert drdy.bytecode[entries[0].offset - 1] == Op.EVENT_DIV
    assert entries[0].size == 2

    poll = Iim20670().compile({'trigger': 'poll'})
    entries = [e for e in poll.patch_map if e.param_name == "sample_rate"]
    assert len(entries) == 1
    assert entries[0].value_map == {r: max(1, 1000 // r) for r in RATES}


def test_iim20670_drdy_loop_is_edge_paced_with_staging_gaps():
    """The drdy measure loop: ``OP_YIELD`` head (the ODR edge paces the
    loop — no sleep throttle), the datasheet's 5 µs post-edge settle as
    one ``OP_SLEEP_US``, then the FRAME burst expanding to:

    - ``N + read_pipeline`` interleaved ``OP_MEMCPY_IMM`` + ``OP_REG_XFER``
      pairs using a rotating 4-byte TX/RX slot at the tail of the
      sample buffer,
    - a ``OP_MEMCPY`` extract after every XFER whose RX carries real
      data (``k >= pipeline``),
    - a CRC + status verification block after every extract, while the
      response still sits in the slot: ``OP_CRC8`` (input-lsb style)
      over the covered bytes vs the received CRC byte via ``XOR_REG`` +
      backward ``JNZ`` to the loop head, then the RS field mask-compared
      (``AND 0x03``, ``CMP_EQ 0x01``) with a backward ``JZ``. A mismatch
      drops the tick before ``STORE_SAMPLE`` — a corrupted or unprepared
      (RS = 10) frame never publishes,
    - a 250 µs ``OP_SLEEP_US`` staging gap after every frame but the last:
      a sampled register stages its response on the part's next 8 kHz
      internal update (125 µs period), so the gap is one tick with 2×
      margin. Bench-swept: 125 µs reads clean, 63 µs returns garbage,
      back-to-back reads stale/idle at any SPI clock. This is the
      adoption contract's teeth twice over: a regen once dropped the
      settle entirely (justified by the datasheet's self-test tables —
      a different transaction) and read zeros on hardware, and the
      original 1 ms value was the ms-granular knob's minimum, ~8× the
      real floor. A regen that drops the gap or regresses it to
      milliseconds reverses a bench-validated decision,
    - closing ``OP_STORE_SAMPLE`` + ``OP_JMP`` back to the loop top.
    """
    cd = Iim20670().compile({})
    bc = cd.bytecode

    FW = 4
    DW = 2
    DOFF = 1
    NUM_WORDS = 7
    PIPELINE = 1
    NUM_TX = NUM_WORDS + PIPELINE
    # Burst codegen places its rotating slot at the buffer tail; track
    # the firmware constant so the test follows VM_SAMPLE_BUF_SIZE bumps.
    from nxs.compiler import _ASTCompiler
    SLOT = _ASTCompiler.VM_SAMPLE_BUF_SIZE - FW

    pos = _find_burst_start(bc, SLOT)

    # Loop head immediately precedes the burst: YIELD, then SLEEP_US 5.
    head = bc[pos - 4: pos]
    assert head[0] == Op.YIELD, f"expected YIELD before the burst, got {head!r}"
    assert head[1] == Op.SLEEP_US
    assert int.from_bytes(head[2:4], "little") == 5

    for k in range(NUM_TX):
        # OP_MEMCPY_IMM dst=SLOT len=FW <tx_bytes...>
        assert bc[pos] == Op.MEMCPY_IMM, (
            f"iteration {k}: expected MEMCPY_IMM at +{pos}, "
            f"got 0x{bc[pos]:02X}")
        assert bc[pos + 1] == SLOT
        assert bc[pos + 2] == FW
        pos += 3 + FW
        # OP_REG_XFER tx_off=SLOT rx_off=SLOT len=FW
        assert bc[pos] == Op.REG_XFER, (
            f"iteration {k}: expected REG_XFER at +{pos}, "
            f"got 0x{bc[pos]:02X}")
        assert bc[pos + 1] == SLOT
        assert bc[pos + 2] == SLOT
        assert bc[pos + 3] == FW
        pos += 4
        if k >= PIPELINE:
            out_idx = k - PIPELINE
            assert bc[pos] == Op.MEMCPY, (
                f"iteration {k}: expected MEMCPY extract at +{pos}, "
                f"got 0x{bc[pos]:02X}")
            assert bc[pos + 1] == out_idx * DW
            assert bc[pos + 2] == SLOT + DOFF
            assert bc[pos + 3] == DW
            pos += 4
            # CRC check: recompute over the covered bytes (rw|addr|rs|data
            # = slot bytes 0..2, input-lsb style) and XOR against the
            # received CRC byte at slot+3; non-zero → drop the tick.
            assert bc[pos] == Op.CRC8, (
                f"iteration {k}: expected CRC8 at +{pos}, got 0x{bc[pos]:02X}")
            assert bc[pos + 1] == SLOT       # covered window offset
            assert bc[pos + 2] == 3          # covered window length
            assert bc[pos + 3] == 0x1D       # poly
            assert bc[pos + 4] == 0xFF       # init
            assert bc[pos + 5] == 0xFF       # xor_out
            crc_reg = bc[pos + 6]
            assert bc[pos + 7] == 1          # style = input-lsb
            pos += 8
            assert bc[pos] == Op.LOAD_U8
            tmp_reg = bc[pos + 1]
            assert bc[pos + 2] == SLOT + 3   # received CRC byte
            pos += 3
            assert bc[pos] == Op.XOR_REG
            assert (bc[pos + 1], bc[pos + 2], bc[pos + 3]) == (
                crc_reg, tmp_reg, crc_reg)
            pos += 4
            assert bc[pos] == Op.JNZ
            assert bc[pos + 1] == crc_reg
            assert int.from_bytes(bc[pos + 2:pos + 4], "little",
                                  signed=True) < 0, "drop jump is backward"
            pos += 4
            # Status check: RS bits (slot byte 0, mask 0x03) must read 01
            # — "successful register read/write"; 10 is the unprepared
            # staged response.
            assert bc[pos] == Op.LOAD_U8
            assert bc[pos + 1] == tmp_reg
            assert bc[pos + 2] == SLOT       # status byte
            pos += 3
            assert bc[pos] == Op.AND
            assert bc[pos + 1] == tmp_reg
            assert int.from_bytes(bc[pos + 2:pos + 6], "little") == 0x03
            assert bc[pos + 6] == tmp_reg
            pos += 7
            assert bc[pos] == Op.CMP_EQ
            assert bc[pos + 1] == tmp_reg
            assert int.from_bytes(bc[pos + 2:pos + 6], "little") == 0x01
            assert bc[pos + 6] == tmp_reg
            pos += 7
            assert bc[pos] == Op.JZ
            assert bc[pos + 1] == tmp_reg
            assert int.from_bytes(bc[pos + 2:pos + 4], "little",
                                  signed=True) < 0, "drop jump is backward"
            pos += 4
        if k < NUM_TX - 1:
            # Response-staging gap between frames (none after the last):
            # 250 µs = one 8 kHz internal update with 2× margin.
            assert bc[pos] == Op.SLEEP_US, (
                f"iteration {k}: expected staging SLEEP_US at +{pos}, "
                f"got 0x{bc[pos]:02X}")
            assert int.from_bytes(bc[pos + 1:pos + 3], "little") == 250
            pos += 3
        else:
            assert bc[pos] not in (Op.SLEEP_US, Op.SLEEP_MS), (
                f"trailing staging sleep after the last frame at +{pos}")

    assert bc[pos] == Op.STORE_SAMPLE
    pos += 1
    assert bc[pos] == Op.JMP


def test_iim20670_read_words_respects_buffer_cap():
    """Asking for more words than the sample buffer can hold must fail
    at compile time, not produce bytecode that'll overrun the buffer
    at runtime. The output region ends at the rotating TX/RX slot
    (124 bytes for a 4-byte FRAME), 2 bytes per word → 62 words max."""
    from nxs.compiler import CompileError, Sample

    class HugeBurst(Iim20670):
        @Iim20670.measure_loop(trigger="poll", sample_rate=100)
        def measure(self):
            raw = self.read_words(0x00, 64)
            return Sample(raw)

    try:
        HugeBurst().compile({})
    except CompileError as e:
        assert "overflow" in str(e).lower() or "sample buffer" in str(e).lower()
    else:
        raise AssertionError(
            "expected CompileError for read_words burst that overflows the "
            "sample buffer, got no error")


def test_iim20670_nxs_roundtrip():
    """Compile, serialise to NXS, deserialise, check round-trip."""
    cd = Iim20670().compile({})
    blob = serialize(cd)
    cd2 = deserialize(blob)
    assert cd2.name == cd.name
    assert cd2.sample_size == cd.sample_size
    assert cd2.bytecode == cd.bytecode
    got = sorted((p.name, p.current) for p in cd2.params)
    want = sorted((p.name, p.current) for p in cd.params)
    assert got == want


# ── Adoption contract: full-scale / filter are runtime reload params ──
#
# The part's config registers are read-modify-write over undocumented reserved
# bits, which once forced these to compile-time keys. They are now runtime
# params tagged on the RMW writes (write_modify param=). A regen that reverts
# them to compile-time keys, or drops the multi-site filter, reverses a
# hardware-validated decision and must lose to these tests.

def test_iim20670_fs_and_filter_are_runtime_params():
    cd = Iim20670().compile({})
    by_name = {p.name: p for p in cd.params}
    assert set(by_name) >= {"accel_fs", "gyro_fs", "filter_hz"}
    # Clean datasheet-label value sets; all reload (a range switch rewrites a
    # register, so the VM must restart configure()).
    assert by_name["accel_fs"].values == [2, 4, 16, 32]
    assert by_name["gyro_fs"].values == [41, 61, 82, 123, 164, 218, 246, 328,
                                         437, 492, 655, 874, 1311, 1966]
    assert by_name["filter_hz"].values == [10, 46, 60]
    for name in ("accel_fs", "gyro_fs", "filter_hz"):
        assert by_name[name].kind == "reload"


def test_iim20670_patch_site_counts():
    """accel_fs/gyro_fs write one register each (1 site); filter_hz writes two
    (0x0C + 0x0E), so it owns two sites at distinct offsets."""
    cd = Iim20670().compile({})
    sites = {}
    for pe in cd.patch_map:
        sites.setdefault(pe.param_name, []).append(pe)
    assert len(sites["accel_fs"]) == 1
    assert len(sites["gyro_fs"]) == 1
    assert len(sites["filter_hz"]) == 2
    # The two filter sites are distinct bytecode offsets.
    assert len({pe.offset for pe in sites["filter_hz"]}) == 2


def test_iim20670_scale_tracks_range_exactly_for_accel():
    """accel scale is base x the range label and is EXACT (the labels' true
    ranges are a constant 1.024x, folded into the base). Each accel output
    links scale_param='accel_fs'."""
    cd = Iim20670().compile({})
    accel = [f for f in cd.output_fields if f['name'].startswith('accel_')]
    assert accel and all(f.get('scale_param') == 'accel_fs' for f in accel)
    base = accel[0]['scale']
    # effective_scale = base x current_value; check every range is exact.
    for label, true_g in [(2, 2.048), (4, 4.096), (16, 16.384), (32, 32.768)]:
        assert base * label == 9.80665 * true_g / 32768.0
    gyro = [f for f in cd.output_fields if f['name'].startswith('gyro_')]
    assert gyro and all(f.get('scale_param') == 'gyro_fs' for f in gyro)
