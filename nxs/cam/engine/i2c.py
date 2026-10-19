# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Register-oriented I2C access for the camera engine, over libnxs's bus
handle: one combined transaction per read, one write per write, the errno
the bus answered with on a refusal."""

from __future__ import annotations

from typing import Optional

from nxs import _libnxs


class I2CHandlerError(RuntimeError):
    """I2C transport error raised by CamI2c."""


class CamI2c:
    """Register-oriented I2C handler with per-call address override."""

    def __init__(
        self,
        addr: str,
        bus: str = "/dev/i2c-1",
        bus_obj=None,
    ) -> None:
        """``addr`` is the default device address ("0x6a"); ``bus`` a number, a
        device path, or a udev alias path, ignored when ``bus_obj`` is given."""
        self.bus = bus
        self.addr = addr
        self._bus_obj = bus_obj
        self._handle: Optional[_libnxs.Bus] = None
        self._is_open = bus_obj is not None

    def _addr_int(self, addr: Optional[str] = None) -> int:
        value = (addr if addr is not None else self.addr).strip().lower()
        return int(value, 16) if value.startswith("0x") else int(value)

    def _bus_path(self) -> str:
        bus = self.bus.strip()
        if bus.isdigit():
            return f"/dev/i2c-{bus}"
        return bus

    def open(self) -> None:
        """Open the bus through libnxs; I2CHandlerError when the open fails."""
        if self._is_open:
            return
        try:
            self._handle = _libnxs.Bus.open(self._bus_path())
        except OSError as exc:
            self._handle = None
            raise I2CHandlerError(
                f"failed to open I2C bus {self._bus_path()}"
            ) from exc
        self._is_open = True

    def close(self) -> None:
        """Close the bus (an injected bus_obj is left to its owner)."""
        if self._handle is not None:
            self._handle.close()
            self._handle = None
        self._is_open = self._bus_obj is not None

    # -- low-level transfers ------------------------------------------

    def _do_write(self, addr_int: int, payload: list[int]) -> None:
        if self._bus_obj is not None:
            try:
                self._bus_obj.write_bytes(addr_int, payload)
            except OSError as exc:
                raise I2CHandlerError("I2C transfer failed") from exc
            return
        if not self._is_open:
            self.open()
        try:
            self._handle.write(addr_int, bytes(payload))
        except OSError as exc:
            raise I2CHandlerError("I2C transfer failed") from exc

    def _do_write_read(
        self, addr_int: int, payload: list[int], read_len: int
    ) -> bytes:
        if self._bus_obj is not None:
            try:
                return bytes(
                    self._bus_obj.write_read(addr_int, payload, read_len)
                )
            except OSError as exc:
                raise I2CHandlerError("I2C transfer failed") from exc
        if not self._is_open:
            self.open()
        try:
            # One combined transaction: the register, then the read.
            return self._handle.read(addr_int, bytes(payload), read_len)
        except OSError as exc:
            raise I2CHandlerError("I2C transfer failed") from exc

    # -- register API --------------------------------------------------

    @staticmethod
    def _reg_bytes(reg: int, reg_width: int) -> list[int]:
        if reg_width == 8:
            return [reg & 0xFF]
        if reg_width == 16:
            return [(reg >> 8) & 0xFF, reg & 0xFF]
        raise ValueError("reg_width must be 8 or 16")

    def write_reg(
        self,
        reg: int,
        value: int | list[int],
        reg_width: int = 16,
        data_width: int = 8,
        addr: Optional[str] = None,
    ) -> None:
        """Write one or more values to sequential registers; ``reg_width`` and
        ``data_width`` are 8 or 16 bits (big-endian words). I2CHandlerError on
        transfer failure."""
        values = value if isinstance(value, list) else [value]
        payload = self._reg_bytes(reg, reg_width)
        if data_width == 8:
            payload.extend(v & 0xFF for v in values)
        elif data_width == 16:
            for v in values:
                payload.extend([(v >> 8) & 0xFF, v & 0xFF])
        else:
            raise ValueError("data_width must be 8 or 16")
        self._do_write(self._addr_int(addr), payload)

    def write_raw(self, data: list[int], addr: Optional[str] = None) -> None:
        """Write ``data`` as one transaction with no register framing (a part
        whose first byte is its register); I2CHandlerError on transfer failure."""
        self._do_write(self._addr_int(addr), [v & 0xFF for v in data])

    def read_reg(
        self,
        reg: int,
        length: int = 1,
        reg_width: int = 16,
        data_width: int = 8,
        addr: Optional[str] = None,
    ) -> bytes:
        """Write the register address, then read ``length`` values as raw bytes;
        ``reg_width`` and ``data_width`` are 8 or 16 bits."""
        addr_bytes = self._reg_bytes(reg, reg_width)
        if data_width == 8:
            read_len = length
        elif data_width == 16:
            read_len = length * 2
        else:
            raise ValueError("data_width must be 8 or 16")
        return self._do_write_read(self._addr_int(addr), addr_bytes, read_len)
