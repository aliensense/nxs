"""Declarative SPI frame schemas. A personality declares its framing (CRC, reserved
bits, pipelined reads) as a `SpiFrame`; the compiler emits the exact on-wire
bytes per access. Fields are MSB-first and `width` is a multiple of 8."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple


@dataclass
class Crc:
    """CRC spec for an in-frame integrity byte: `poly` without its x^width term;
    `feedback_style` 'standard' (feedback = MSB XOR input) or 'input-lsb'
    (feedback = MSB, input XOR'd into the LSB); or a custom `compute_fn`."""

    width: int
    poly: int = 0
    init: int = 0
    xor_out: int = 0
    covers: Tuple[str, ...] = ()
    compute_fn: Optional[Callable[[int, int], int]] = None
    feedback_style: str = "standard"

    def compute(self, data_bits: int, input_width_bits: int) -> int:
        if self.compute_fn is not None:
            return self.compute_fn(data_bits, input_width_bits)
        crc = self.init
        mask = (1 << self.width) - 1
        top_bit = 1 << (self.width - 1)
        if self.feedback_style == "standard":
            # Textbook left-shift CRC: feedback = MSB XOR input.
            for i in range(input_width_bits - 1, -1, -1):
                b = (data_bits >> i) & 1
                top = (crc & top_bit) != 0
                feedback = top ^ bool(b)
                crc = (crc << 1) & mask
                if feedback:
                    crc ^= self.poly
            return crc ^ self.xor_out
        elif self.feedback_style == "input-lsb":
            # input-lsb variant: feedback = MSB only; input bit is XOR'd
            # into the LSB after shift + polynomial-XOR.
            for i in range(input_width_bits - 1, -1, -1):
                b = (data_bits >> i) & 1
                top = (crc & top_bit) != 0
                crc = (crc << 1) & mask
                if top:
                    crc ^= self.poly
                crc ^= b
            return crc ^ self.xor_out
        else:
            raise ValueError(
                f"Unknown feedback_style: {self.feedback_style!r}. "
                f"Expected 'standard' or 'input-lsb'.")


@dataclass
class SpiField:
    """One field in a framed SPI transaction."""
    name: str
    bits: int


@dataclass
class SpiFrame:
    """Declarative SPI frame: named fields MSB-first summing to `width` bits (a
    multiple of 8). Declaring `crc` or `status_ok` makes the compiler verify
    every harvested response on-device; a failing frame drops the tick."""

    width: int
    fields: List[SpiField]
    crc: Optional[Crc] = None                    # needs a field named 'crc'
    read_pipeline: int = 0                       # frames between a read request and its data
    inter_frame_sleep_ms: int = 0                # settle between burst frames
    inter_frame_sleep_us: int = 0                # sub-ms settle, added to the ms value
    status_ok: Optional[Tuple[str, int]] = None  # (field, expected); field within one byte

    def __post_init__(self):
        # Tuples like `('rw', 1)` are shorthand; convert them to SpiField on
        # construction.
        self.fields = [
            f if isinstance(f, SpiField) else SpiField(*f) for f in self.fields
        ]
        total = sum(f.bits for f in self.fields)
        if total != self.width:
            raise ValueError(
                f"Frame fields sum to {total} bits, expected {self.width}")
        if self.width % 8 != 0:
            raise ValueError(
                f"Frame width must be byte-aligned, got {self.width}")
        if self.crc is not None:
            if not any(f.name == 'crc' for f in self.fields):
                raise ValueError("CRC spec given but no 'crc' field in frame")
            # An unknown or empty `covers` fails at construction: a typo would
            # silently drop from the CRC window in both compose() and on-device.
            field_names = {f.name for f in self.fields}
            unknown = [n for n in self.crc.covers if n not in field_names]
            if unknown:
                raise ValueError(
                    f"crc covers unknown field(s) {unknown}; every covered "
                    f"name must be a declared frame field")
            if not self.crc.covers:
                raise ValueError(
                    "crc covers no fields; a CRC must protect at least one")
        if self.status_ok is not None:
            name, expect = self.status_ok
            bits = self._field_bits(name)   # KeyError if absent
            if not 0 <= expect < (1 << bits):
                raise ValueError(
                    f"status_ok value 0x{expect:X} does not fit the "
                    f"{bits}-bit field {name!r}")
            off = self._field_offset(name)
            if off // 8 != (off + bits - 1) // 8:
                raise ValueError(
                    f"status_ok field {name!r} spans bytes (bits "
                    f"{off + bits - 1}..{off}); the on-device check "
                    f"loads one byte")

    @property
    def byte_width(self) -> int:
        return self.width // 8

    # ── Field position lookup ──────────────────────────────────

    def _field_offset(self, name: str) -> int:
        """Bit offset of `name` from the LSB."""
        off = self.width
        for f in self.fields:
            off -= f.bits
            if f.name == name:
                return off
        raise KeyError(f"No field named {name!r} in frame")

    def _field_bits(self, name: str) -> int:
        for f in self.fields:
            if f.name == name:
                return f.bits
        raise KeyError(f"No field named {name!r} in frame")

    # ── Building and parsing frames ────────────────────────────

    def compose(self, **values) -> bytes:
        """Build a frame from field values, MSB-first bytes. Unspecified fields
        are 0; the CRC is computed over `covers` and placed in `crc`."""
        frame = 0
        for f in self.fields:
            if f.name == 'crc' and self.crc is not None:
                continue
            val = values.get(f.name, 0)
            mask = (1 << f.bits) - 1
            frame |= (val & mask) << self._field_offset(f.name)

        if self.crc is not None:
            covered_bits = 0
            covered_width = 0
            for f in self.fields:
                if f.name in self.crc.covers:
                    off = self._field_offset(f.name)
                    mask = (1 << f.bits) - 1
                    val = (frame >> off) & mask
                    covered_bits = (covered_bits << f.bits) | val
                    covered_width += f.bits
            crc_val = self.crc.compute(covered_bits, covered_width)
            crc_off = self._field_offset('crc')
            crc_width = self._field_bits('crc')
            frame |= (crc_val & ((1 << crc_width) - 1)) << crc_off

        return frame.to_bytes(self.byte_width, byteorder='big')

    def extract(self, frame_bytes: bytes, field_name: str) -> int:
        """Pull one field's value out of received bytes."""
        if len(frame_bytes) != self.byte_width:
            raise ValueError(
                f"Expected {self.byte_width} bytes, got {len(frame_bytes)}")
        frame = int.from_bytes(frame_bytes, byteorder='big')
        off = self._field_offset(field_name)
        width = self._field_bits(field_name)
        return (frame >> off) & ((1 << width) - 1)

    def data_byte_offset(self) -> int:
        """Byte index (MSB-first) of the ``data`` field's MSB within the frame."""
        off_bits = self._field_offset('data')
        width_bits = self._field_bits('data')
        # MSB of data is at (off_bits + width_bits - 1). Convert to byte
        # index counting from the left (MSB-first).
        msb_bit_from_left = self.width - 1 - (off_bits + width_bits - 1)
        return msb_bit_from_left // 8

    def data_byte_width(self) -> int:
        return self._field_bits('data') // 8

    # ── RX-verification geometry (compiler helpers) ────────────
    # The on-device checker is byte-granular; these map bit-level fields onto
    # byte windows and raise ValueError for frames it cannot check.

    def crc_byte_offset(self) -> int:
        """Byte index (MSB-first) of the CRC field, which must be whole bytes on
        a byte boundary."""
        off = self._field_offset('crc')
        bits = self._field_bits('crc')
        if bits % 8 != 0 or off % 8 != 0:
            raise ValueError(
                f"crc field (bits {off + bits - 1}..{off}) is not "
                f"byte-aligned; the on-device check compares whole bytes")
        return (self.width - off - bits) // 8

    def crc_cover_window(self) -> Tuple[int, int]:
        """(byte_offset, byte_length) of the CRC-covered span: contiguous fields
        in declaration order, starting and ending on byte boundaries."""
        covered = [f for f in self.fields if f.name in self.crc.covers]
        names = [f.name for f in self.fields]
        idx = [names.index(f.name) for f in covered]
        if idx != list(range(idx[0], idx[0] + len(idx))):
            raise ValueError(
                f"crc covers {self.crc.covers!r} are not contiguous "
                f"fields; the on-device check runs over one byte window")
        hi = self._field_offset(covered[0].name) + covered[0].bits
        lo = self._field_offset(covered[-1].name)
        if hi % 8 != 0 or lo % 8 != 0:
            raise ValueError(
                f"crc covered window (bits {hi - 1}..{lo}) is not "
                f"byte-aligned; the on-device check runs over whole bytes")
        return (self.width - hi) // 8, (hi - lo) // 8

    def status_byte(self) -> Tuple[int, int, int]:
        """(byte_offset, mask, expected) for the ``status_ok`` check:
        ``rx[byte_offset] & mask == expected``."""
        name, expect = self.status_ok
        off = self._field_offset(name)
        bits = self._field_bits(name)
        shift = off % 8
        mask = ((1 << bits) - 1) << shift
        return (self.width - 1 - (off + bits - 1)) // 8, mask, expect << shift

