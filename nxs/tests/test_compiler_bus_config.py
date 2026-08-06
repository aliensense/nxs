"""Compiler emission tests for the NXS per-bus bus_config trailer.

Locks the contract between a driver's Communication Profile layer
(``SPI_PROFILE`` / ``I2C_PROFILE`` / ``UART_PROFILE``) and the per-bus
profile list the compiler emits. Register drivers emit one profile per
``BUSES`` entry so the runtime ``bus`` switch picks the active one with
no recompile; the on-wire round-trip is covered by
``test_image_bus_config.py``.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from nxs.compiler import (
    CompileError, RegisterDriver, Sample, SensorDriver, StreamDriver)
from nxs.profiles import I2cProfile, SpiProfile, UartProfile


def _spi(**over):
    d = {'kind': 'spi', 'max_hz': 0, 'spi_mode': 0, 'addr_bytes': 1,
         'rw_read_level': 1, 'dummy_bytes': 0, 'auto_inc': 'implicit'}
    d.update(over)
    return d


def _i2c(**over):
    d = {'kind': 'i2c', 'max_hz': 0, 'auto_inc': 'implicit', 'pec': 'none'}
    d.update(over)
    return d


# ── Bare register driver: a default profile per supported bus ─────────

class _BareRegister(RegisterDriver):
    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


def test_bare_register_emits_default_profile_per_bus():
    # BUSES defaults to ('i2c', 'spi') → one default profile each. Defaults
    # are byte-for-byte the firmware's built-in wire shape.
    assert _BareRegister().compile().bus_config == [_i2c(), _spi()]


# ── SPI-only driver: one default profile, no I²C ──────────────────────

class _SpiOnlyBare(RegisterDriver):
    BUSES = ('spi',)

    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


def test_spi_only_emits_single_default_profile():
    assert _SpiOnlyBare().compile(config={'bus': 1}).bus_config == [_spi()]


# ── New Communication Profile descriptors ─────────────────────────────

class _FxosLike(RegisterDriver):
    """Fxos8700-style: 2-byte SPI address framing, R/W cleared for a read,
    standard I²C; SPI is the default bus."""
    SPI_PROFILE = SpiProfile(addr_bytes=2, rw_read_level=0, max_hz=1_100_000)
    I2C_PROFILE = I2cProfile(max_hz=400_000)
    BUS = 'spi'

    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


def test_fxos_like_profiles_and_bus_selector():
    cd = _FxosLike().compile()
    assert cd.bus_config == [
        _i2c(max_hz=400_000),
        _spi(max_hz=1_100_000, addr_bytes=2, rw_read_level=0),
    ]
    # BUS = 'spi' makes spi (code 1) the compile-time default.
    bus = [p for p in cd.params if p.name == 'bus'][0]
    assert bus.default == 1 and bus.current == 1


def test_bus_override_keeps_both_profiles():
    # Forcing i2c flips the param but still bakes both profiles — switching
    # back to spi later is a reload, not a recompile.
    cd = _FxosLike().compile(config={'bus': 'i2c'})
    bus = [p for p in cd.params if p.name == 'bus'][0]
    assert bus.current == 0
    assert [p['kind'] for p in cd.bus_config] == ['i2c', 'spi']


class _StAutoInc(RegisterDriver):
    """ST LIS/LSM-style: sub-address MSB enables I²C auto-increment."""
    BUSES = ('i2c',)
    I2C_PROFILE = I2cProfile(auto_inc='msb', pec='none')

    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


def test_i2c_auto_inc_msb():
    assert _StAutoInc().compile().bus_config == [_i2c(auto_inc='msb')]


class _SmbusPec(RegisterDriver):
    """SMBus PEC part."""
    BUSES = ('i2c',)
    I2C_PROFILE = I2cProfile(pec='crc8', max_hz=100_000)

    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


def test_i2c_pec_crc8_rejected():
    # Firmware doesn't apply SMBus PEC yet; a part needing it must not
    # compile into an image that silently runs with no error checking.
    with pytest.raises(CompileError):
        _SmbusPec().compile()


class _SpiLsbDummy(RegisterDriver):
    """SPI mode 3, LSB-first, one dummy byte after the address (Bosch-style)."""
    BUSES = ('spi',)
    SPI_PROFILE = SpiProfile(mode=3, bit_order='lsb', dummy_bytes=1)

    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


def test_spi_mode_bit_order_dummy():
    # spi_mode packs CPOL/CPHA (mode 3 = 0b11) | LSB-first (bit 2) = 0b111.
    assert _SpiLsbDummy().compile(config={'bus': 1}).bus_config == [
        _spi(spi_mode=0b111, dummy_bytes=1),
    ]


# ── StreamDriver / UART path ──────────────────────────────────────────

class _UartProfileDriver(StreamDriver):
    UART_PROFILE = UartProfile(baud=38_400, parity=2, stop_bits=2)

    @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
    def measure(self):
        return None


def test_uart_profile_rejected():
    # Firmware doesn't apply UART framing yet; declaring a profile is a
    # compile error, not a silently-dropped image field.
    with pytest.raises(CompileError):
        _UartProfileDriver().compile()


class _BareStream(StreamDriver):
    @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
    def measure(self):
        return None


def test_stream_without_baud_emits_none():
    assert _BareStream().compile().bus_config is None


def test_spi_profile_rejects_addr2_autoinc_msb():
    # addr_bytes=2 frames the address across two bytes; auto_inc='msb' would
    # set bit 6 of the first byte on a burst, corrupting the address.
    with pytest.raises(ValueError):
        SpiProfile(addr_bytes=2, auto_inc='msb')


class _AutoIncNone(RegisterDriver):
    BUSES = ('i2c',)
    I2C_PROFILE = I2cProfile(auto_inc='none')

    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


def test_auto_inc_none_rejected():
    # Firmware can't honor "no auto-increment", so a burst would misread;
    # reject the declaration rather than let it behave like 'implicit'.
    with pytest.raises(CompileError):
        _AutoIncNone().compile()


class _WideReg(RegisterDriver):
    BUSES = ('i2c',)

    def configure(self, config):
        self.write(0x0100, 0x01)   # 16-bit register address — unsupported
        self.set_output([{'name': 'x', 'scale': 1.0, 'unit': ''}])
        self.set_sample_size(2)

    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


def test_register_above_8bit_rejected():
    # The devices frame 8-bit addresses; the compiler rejects a wider reg
    # instead of letting it truncate to -EINVAL at runtime.
    with pytest.raises(CompileError):
        _WideReg().compile()
