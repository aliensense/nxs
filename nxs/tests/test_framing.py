"""Unit tests for the SpiFrame / Crc declarative framing module."""

import pytest

from nxs.framing import Crc, SpiField, SpiFrame


# ── Crc ──────────────────────────────────────────────────────


def test_iim20670_crc_datasheet_examples():
    """Datasheet §5.2: CRC(0xA0CA85)=0x2F, CRC(0x3B007C)=0xC2.

    Uses the IIM-20670-specific feedback variant (input XOR'd at LSB
    after shift). This is distinct from the standard textbook CRC
    that Sensirion uses.
    """
    c = Crc(width=8, poly=0x1D, init=0xFF, xor_out=0xFF,
            feedback_style='input-lsb')
    assert c.compute(0xA0CA85, 24) == 0x2F
    assert c.compute(0x3B007C, 24) == 0xC2


def test_standard_crc_sensirion_datasheet_example():
    """Sensirion app-note AN_CRC_Checksum_Calculation: CRC of
    `0xBEEF` is `0x92` with poly=0x31, init=0xFF, xor_out=0x00.
    This is the standard CRC variant (default feedback_style).
    """
    c = Crc(width=8, poly=0x31, init=0xFF, xor_out=0x00)
    assert c.compute(0xBEEF, 16) == 0x92


def test_standard_crc_feed_back_in_property():
    """For standard CRC with xor_out=0: CRC(msg || CRC(msg)) == 0.
    This is the linearity property the firmware CRC-verification
    pattern relies on — check CRC over the message + its CRC byte
    equals 0 rather than extracting and comparing the CRC byte.
    """
    c = Crc(width=8, poly=0x31, init=0xFF, xor_out=0x00)
    for msg in [0xBEEF, 0x1234, 0x0000, 0xFFFF, 0xDEAD]:
        crc_of_msg = c.compute(msg, 16)
        # Append the CRC as a 3rd byte and recompute over all 24 bits.
        full = (msg << 8) | crc_of_msg
        assert c.compute(full, 24) == 0


def test_crc_custom_compute_fn():
    """Custom `compute_fn` overrides the polynomial path."""
    c = Crc(width=8, compute_fn=lambda data, w: (data + w) & 0xFF)
    assert c.compute(0x10, 8) == 0x18


# ── SpiFrame structural ──────────────────────────────────────


def test_spiframe_validation_field_sum_mismatch():
    with pytest.raises(ValueError, match="sum to"):
        SpiFrame(width=32, fields=[('a', 16), ('b', 16), ('c', 1)])


def test_spiframe_validation_non_byte_aligned():
    with pytest.raises(ValueError, match="byte-aligned"):
        SpiFrame(width=12, fields=[('a', 4), ('b', 8)])


def test_spiframe_validation_crc_without_field():
    with pytest.raises(ValueError, match="no 'crc' field"):
        SpiFrame(
            width=16,
            fields=[('a', 16)],
            crc=Crc(width=8, poly=0x07, covers=('a',)),
        )


# ── SpiFrame compose / extract ──────────────────────────────


def _iim20670_frame() -> SpiFrame:
    return SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, poly=0x1D, init=0xFF, xor_out=0xFF,
                covers=('rw', 'addr', 'rs', 'data'),
                feedback_style='input-lsb'),
        read_pipeline=1,
    )


def test_iim20670_unlock_sequence_compose():
    """The 6-frame FS-unlock sequence from §4.11 must match bit-for-bit."""
    f = _iim20670_frame()
    EXPECTED = [
        (0x0002, 0xE4000288),
        (0x0001, 0xE400018B),
        (0x0004, 0xE400048E),
        (0x0300, 0xE40300AD),
        (0x0180, 0xE4018017),
        (0x0280, 0xE4028030),
    ]
    for data, expected_u32 in EXPECTED:
        frame = f.compose(rw=1, addr=0x19, rs=0, data=data)
        got = int.from_bytes(frame, 'big')
        assert got == expected_u32, (
            f"compose(data=0x{data:04X}) = 0x{got:08X}, "
            f"expected 0x{expected_u32:08X}")


def test_iim20670_read_frame_shape():
    """A read request for reg 0x0B (fixed_value) with default data."""
    f = _iim20670_frame()
    frame = f.compose(rw=0, addr=0x0B, data=0x0000)
    # byte 3 = 0b0 01011 00 = 0x2C
    assert frame[0] == 0x2C
    assert frame[1] == 0x00
    assert frame[2] == 0x00
    # CRC over 24 high bits (rw|addr|rs|data) = 0x2C0000
    expected_crc = Crc(
        width=8, poly=0x1D, init=0xFF, xor_out=0xFF,
        feedback_style='input-lsb',
    ).compute(0x2C0000, 24)
    assert frame[3] == expected_crc


def test_extract_data_field():
    f = _iim20670_frame()
    # Build a simulated response: rw=0, addr=0x0B, rs=0b01, data=0xAA55
    resp = f.compose(rw=0, addr=0x0B, rs=0b01, data=0xAA55)
    assert f.extract(resp, 'data') == 0xAA55
    assert f.extract(resp, 'rs') == 0b01
    assert f.extract(resp, 'addr') == 0x0B


def test_data_byte_offset_and_width():
    """For a 32-bit frame with data at bits 23..8, data occupies bytes
    1..2 (MSB-first) and is 2 bytes wide."""
    f = _iim20670_frame()
    assert f.data_byte_offset() == 1
    assert f.data_byte_width() == 2


# ── RX-verification geometry (status_ok + byte-window helpers) ──


def test_status_ok_validation():
    fields = [('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)]
    # Unknown field name.
    with pytest.raises(KeyError):
        SpiFrame(width=32, fields=list(fields), status_ok=('nope', 0))
    # Value outside the field width.
    with pytest.raises(ValueError, match="does not fit"):
        SpiFrame(width=32, fields=list(fields), status_ok=('rs', 0b100))
    # A field spanning a byte boundary cannot be checked with one load.
    with pytest.raises(ValueError, match="spans bytes"):
        SpiFrame(width=32,
                 fields=[('a', 4), ('st', 8), ('data', 16), ('pad', 4)],
                 status_ok=('st', 0x00))


def test_status_byte_projection():
    """rs occupies bits 25:24 → byte 0 (MSB-first), in-byte mask 0x03,
    expected value unshifted (field starts at a byte boundary)."""
    f = SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)],
        status_ok=('rs', 0b01),
    )
    assert f.status_byte() == (0, 0x03, 0b01)


def test_status_byte_projection_shifted_field():
    """A status field not at the byte's LSB gets its mask and expected
    value shifted into byte position."""
    f = SpiFrame(
        width=16,
        fields=[('st', 2), ('data', 14)],
        status_ok=('st', 0b10),
    )
    # st occupies bits 15:14 → byte 0, shift 6 within that byte.
    assert f.status_byte() == (0, 0xC0, 0b10 << 6)


def test_crc_cover_window_and_byte_offset():
    """The iim-style frame covers rw|addr|rs|data = bits 31..8 → bytes
    0..2; the CRC byte itself is byte 3."""
    f = _iim20670_frame()
    assert f.crc_cover_window() == (0, 3)
    assert f.crc_byte_offset() == 3


def test_crc_cover_window_rejects_non_byte_aligned():
    """A covered window that ends mid-byte is not checkable by the
    whole-byte on-device engine."""
    f = SpiFrame(
        width=32,
        fields=[('a', 10), ('data', 16), ('pad', 2), ('crc', 4)],
        crc=Crc(width=4, poly=0x3, covers=('a', 'data')),
    )
    with pytest.raises(ValueError, match="byte-aligned"):
        f.crc_cover_window()


def test_crc_cover_window_rejects_non_contiguous():
    f = SpiFrame(
        width=32,
        fields=[('a', 8), ('skip', 8), ('data', 8), ('crc', 8)],
        crc=Crc(width=8, poly=0x1D, covers=('a', 'data')),
    )
    with pytest.raises(ValueError, match="contiguous"):
        f.crc_cover_window()


def test_crc_covers_typo_rejected_at_construction():
    """A misspelled cover name must fail loudly at construction — not
    silently drop from the CRC window (wrong bytes) or crash later with
    an IndexError."""
    with pytest.raises(ValueError, match="unknown field"):
        SpiFrame(
            width=32,
            fields=[('rw', 1), ('addr', 7), ('data', 16), ('crc', 8)],
            crc=Crc(width=8, poly=0x1D, covers=('rw', 'addr', 'dat')),  # typo
        )


def test_crc_empty_covers_rejected_at_construction():
    with pytest.raises(ValueError, match="covers no fields"):
        SpiFrame(
            width=16,
            fields=[('data', 8), ('crc', 8)],
            crc=Crc(width=8, poly=0x1D, covers=()),
        )
