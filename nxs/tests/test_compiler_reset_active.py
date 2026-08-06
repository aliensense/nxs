"""Compiler emission of the runtime `reset_active` param from RESET_ACTIVE.

A register driver whose mikroBUS reset is active-high declares
`RESET_ACTIVE = 'high'`; the compiler auto-injects a runtime-only
`reset_active` param the firmware reads at bind. The default ('low')
emits nothing — firmware then drives the active-low sense.
"""
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import pytest

from nxs.compiler import (
    RegisterDriver, Sample, SensorDriver, StreamDriver, CompileError)


def _param(compiled, name):
    return next((p for p in compiled.params if p.name == name), None)


class _ActiveHigh(RegisterDriver):
    RESET_ACTIVE = 'high'

    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


class _ActiveLow(RegisterDriver):
    RESET_ACTIVE = 'low'

    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


class _DefaultPolarity(RegisterDriver):
    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


class _BadPolarity(RegisterDriver):
    RESET_ACTIVE = 'rising'

    @SensorDriver.measure_loop(trigger="drdy")
    def measure(self):
        return Sample(self.read_burst(0x00, 2))


def test_active_high_emits_reset_active_param():
    p = _param(_ActiveHigh().compile(), 'reset_active')
    assert p is not None
    assert p.default == 1


def test_active_low_emits_no_param():
    assert _param(_ActiveLow().compile(), 'reset_active') is None


def test_default_polarity_emits_no_param():
    assert _param(_DefaultPolarity().compile(), 'reset_active') is None


def test_invalid_polarity_rejected():
    with pytest.raises(CompileError, match="RESET_ACTIVE"):
        _BadPolarity().compile()


class _StreamActiveHigh(StreamDriver):
    RESET_ACTIVE = 'high'
    DEFAULT_BAUD = 38400

    def probe(self):
        self.set_baud(self.DEFAULT_BAUD)

    def configure(self, config):
        self.set_output([
            {'name': 'data', 'type': 'string', 'count': 16, 'scale': 1.0, 'unit': ''},
        ])
        self.set_sample_size(16)

    @SensorDriver.measure_loop(trigger="poll", sample_rate=1)
    def measure(self):
        self.read_until(b'\n', max=16)
        self.store_sample_n()


def test_stream_driver_also_emits_reset_active():
    # The firmware pulses mkbus_rst for every driver kind, so a UART stream
    # driver must be able to declare an active-high reset too — not register-only.
    p = _param(_StreamActiveHigh().compile(), 'reset_active')
    assert p is not None
    assert p.default == 1
