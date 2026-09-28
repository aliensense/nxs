"""Register-bus communication profiles: one per bus a driver supports, baked
into the image, applied by the firmware per the runtime ``bus`` param. Omitted
switches take the conventional defaults: 8-bit register addresses and 8-bit
values; an I²C profile widens both for CCI-style parts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class SpiProfile:
    """SPI register-access switch set. Defaults: one address byte with the R/W
    bit in bit 7 (set for a read), no dummy bytes, mode 0, MSB-first, implicit
    auto-increment; identical to the firmware's behaviour with no profile."""

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
    """I²C register-access switch set. Defaults: 8-bit sub-address, 8-bit
    values, implicit auto-increment, no PEC. ``auto_inc='msb'`` sets the
    sub-address MSB for a multi-byte read (ST LIS/LSM); ``pec='crc8'`` is
    SMBus PEC. ``addr_bytes=2`` frames a 16-bit register map (CCI) and lifts
    the compiler's 8-bit register cap; ``data_width`` and ``byte_order`` frame
    the value a register holds."""

    max_hz: Optional[int] = None     # I8: clock ceiling in Hz; None = DTS default
    auto_inc: str = 'implicit'       # I4: 'implicit' | 'msb' | 'none'
    pec: str = 'none'                # I6: 'none' | 'crc8' (SMBus PEC)
    addr_bytes: int = 1              # I1: register-address bytes on the wire, 1 | 2
    data_width: int = 1              # I2: value bytes per register, 1 | 2 | 4
    byte_order: str = 'big'          # I3: 'big' | 'little' for multi-byte values

    def __post_init__(self):
        if self.auto_inc not in ('implicit', 'msb', 'none'):
            raise ValueError(
                f"I2cProfile.auto_inc must be 'implicit', 'msb' or 'none', "
                f"got {self.auto_inc!r}")
        if self.pec not in ('none', 'crc8'):
            raise ValueError(
                f"I2cProfile.pec must be 'none' or 'crc8', got {self.pec!r}")
        if self.addr_bytes not in (1, 2):
            raise ValueError(
                f"I2cProfile.addr_bytes must be 1 or 2, got {self.addr_bytes}")
        if self.data_width not in (1, 2, 4):
            raise ValueError(
                f"I2cProfile.data_width must be 1, 2 or 4, got {self.data_width}")
        if self.byte_order not in ('big', 'little'):
            raise ValueError(
                f"I2cProfile.byte_order must be 'big' or 'little', got "
                f"{self.byte_order!r}")
        if self.addr_bytes == 2 and self.auto_inc == 'msb':
            raise ValueError(
                "I2cProfile: addr_bytes=2 with auto_inc='msb' is "
                "contradictory — the sub-address MSB is an address bit of a "
                "16-bit register map")

