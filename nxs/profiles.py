"""
Declarative register-bus communication profiles for the NXS VM.

A register-addressed sensor's wire protocol varies along a small set of
orthogonal switches: how the address is framed, the R/W bit polarity,
dummy bytes, auto-increment behaviour, clock, CRC. A driver declares one
profile per bus it supports; the compiler bakes every profile into the
image and the firmware applies the active one — selected by the runtime
``bus`` param — to the bus device before ``probe()`` runs. Switching bus
is a reload, never a recompile.

A profile is the *bus interface* section of the datasheet rendered as
data: no vendor names, just the switches. Omitted switches fall back to
the conventional defaults (a standard InvenSense/ST-style register bus,
identical to the firmware's built-in behaviour), so a typical part
declares nothing and an exotic part lists only its deviations.

Switch axes (one per bus-profile switch):

    SPI: addr_bytes (S1), rw_read_level (S2), dummy_bytes (S3),
         mode (S4), bit_order (S5), max_hz (S6), auto_inc (S8)
    I2C: auto_inc (I4), pec (I6), max_hz (I8)

The register opcode operand is a uniform 16-bit register address, so an
8-bit-addressed part just uses values below 256. Framing a register
address wider than 8 bits on the wire is a separate device-side concern
that is not yet implemented — the bus devices reject a register above
0xFF — so only 8-bit register addresses are supported end to end today.

Example — NXP FXOS8700CQ: 2-byte SPI address framing with the R/W bit
cleared for a read, standard over I²C:

    SPI_PROFILE = SpiProfile(addr_bytes=2, rw_read_level=0, max_hz=1_000_000)
    I2C_PROFILE = I2cProfile(max_hz=400_000)
    BUS = 'spi'
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class SpiProfile:
    """SPI register-access switch set.

    Defaults describe the conventional InvenSense/ST register bus: a
    single address byte with the R/W bit in bit 7 (set for a read), no
    dummy bytes, SPI mode 0, MSB-first, implicit auto-increment. This is
    byte-for-byte what the firmware does when no profile is declared.
    """

    max_hz: Optional[int] = None     # S6: clock ceiling in Hz; None = DTS default
    mode: int = 0                    # S4: CPOL/CPHA combined, 0..3
    bit_order: str = 'msb'           # S5: 'msb' | 'lsb'
    addr_bytes: int = 1              # S1: address bytes on the wire (FXOS8700 = 2)
    rw_read_level: int = 1           # S2: R/W bit (bit 7) level for a read; write inverts
    dummy_bytes: int = 0             # S3: dummy bytes after the address, before read data
    auto_inc: str = 'implicit'       # S8: 'implicit' | 'msb' | 'none'

    def __post_init__(self):
        if not 0 <= self.mode <= 3:
            raise ValueError(f"SpiProfile.mode must be 0..3, got {self.mode}")
        if self.bit_order not in ('msb', 'lsb'):
            raise ValueError(
                f"SpiProfile.bit_order must be 'msb' or 'lsb', got "
                f"{self.bit_order!r}")
        if self.addr_bytes not in (1, 2):
            raise ValueError(
                f"SpiProfile.addr_bytes must be 1 or 2, got {self.addr_bytes}")
        if self.rw_read_level not in (0, 1):
            raise ValueError(
                f"SpiProfile.rw_read_level must be 0 or 1, got "
                f"{self.rw_read_level}")
        if not 0 <= self.dummy_bytes <= 4:
            raise ValueError(
                f"SpiProfile.dummy_bytes must be 0..4, got {self.dummy_bytes}")
        if self.auto_inc not in ('implicit', 'msb', 'none'):
            raise ValueError(
                f"SpiProfile.auto_inc must be 'implicit', 'msb' or 'none', "
                f"got {self.auto_inc!r}")
        if self.addr_bytes == 2 and self.auto_inc == 'msb':
            raise ValueError(
                "SpiProfile: addr_bytes=2 with auto_inc='msb' is "
                "contradictory — the burst MS bit (bit 6 of the first "
                "address byte) would corrupt the address")


@dataclass
class I2cProfile:
    """I²C register-access switch set.

    Defaults describe the conventional 8-bit-sub-address register bus
    with implicit auto-increment and no packet error checking — what the
    firmware does when no profile is declared. ``auto_inc='msb'`` covers
    ST LIS/LSM parts that require the sub-address MSB set to read
    multiple bytes; ``pec='crc8'`` covers SMBus PEC parts (Melexis).
    """

    max_hz: Optional[int] = None     # I8: clock ceiling in Hz; None = DTS default
    auto_inc: str = 'implicit'       # I4: 'implicit' | 'msb' | 'none'
    pec: str = 'none'                # I6: 'none' | 'crc8' (SMBus PEC)

    def __post_init__(self):
        if self.auto_inc not in ('implicit', 'msb', 'none'):
            raise ValueError(
                f"I2cProfile.auto_inc must be 'implicit', 'msb' or 'none', "
                f"got {self.auto_inc!r}")
        if self.pec not in ('none', 'crc8'):
            raise ValueError(
                f"I2cProfile.pec must be 'none' or 'crc8', got {self.pec!r}")


@dataclass
class UartProfile:
    """UART framing for stream sensors.

    The byte-stream analogue of the register profiles: the same uniform
    Communication Profile layer, expressed as line framing. Defaults are
    8N1 at 38400 baud.
    """

    baud: Optional[int] = 38400      # peripheral baud; None = DTS current-speed
    parity: int = 0                  # 0 = none, 1 = even, 2 = odd
    stop_bits: int = 1               # 1 or 2
    data_bits: int = 8               # 7 or 8

    def __post_init__(self):
        if self.parity not in (0, 1, 2):
            raise ValueError(
                f"UartProfile.parity must be 0 (none), 1 (even) or 2 (odd), "
                f"got {self.parity}")
        if self.stop_bits not in (1, 2):
            raise ValueError(
                f"UartProfile.stop_bits must be 1 or 2, got {self.stop_bits}")
        if self.data_bits not in (7, 8):
            raise ValueError(
                f"UartProfile.data_bits must be 7 or 8, got {self.data_bits}")
