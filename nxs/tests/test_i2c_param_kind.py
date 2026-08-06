"""NxsI2cTransport parameter-descriptor reads against a fake SEL window.

The contract these lock: SEL_TYPE carries two independent fields, and
read_param() decodes both. The low nibble selects enum or range; bit 4
selects the parameter's kind, and a live parameter (PWM frequency and
duty) retunes without a driver reload while a reload parameter does
not. A host that loses the kind bit would silently reload the driver on
every set."""
import struct

import pytest

from nxs.transports.i2c import (
    NxsI2cTransport,
    REG_NUM_PARAMS, REG_PARAM_SELECT, REG_SEL_NAME, REG_SEL_NAME_LEN,
    REG_SEL_PARAM_CURRENT, REG_SEL_PARAM_DEFAULT, REG_SEL_PARAM_NUM_VALS,
    REG_SEL_TYPE, REG_SEL_UNIT, REG_SEL_UNIT_LEN, REG_SEL_VALUE,
    REG_SEL_VALUE_INDEX, SEL_TYPE_KIND_SHIFT,
)


class _FakeParamBus:
    """Emulates the firmware's parameter surface: NUM_PARAMS and the
    PARAM_SELECT-driven SEL window refill, including the paged values
    the descriptor serves one u32 at a time."""

    def __init__(self, params):
        self._regs = bytearray(256)
        self._params = list(params)
        self._regs[REG_NUM_PARAMS] = len(self._params)
        self._selected = 0

    def write_byte_data(self, addr, reg, val):
        if reg == REG_PARAM_SELECT:
            self._selected = val
            self._regs[REG_PARAM_SELECT] = val
            self._fill_window(val)
        elif reg == REG_SEL_VALUE_INDEX:
            self._regs[REG_SEL_VALUE_INDEX] = val
            self._fill_value(val)

    def read_byte_data(self, addr, reg):
        return self._regs[reg]

    def read_i2c_block_data(self, addr, reg, n):
        return list(self._regs[reg:reg + n])

    def _fill_window(self, idx):
        if idx >= len(self._params):
            return
        p = self._params[idx]
        name = p['name'].encode()
        self._regs[REG_SEL_NAME_LEN] = len(name)
        self._regs[REG_SEL_NAME:REG_SEL_NAME + len(name)] = name
        kind_bit = 1 if p['kind'] == 'live' else 0
        self._regs[REG_SEL_TYPE] = p['type_code'] | (kind_bit << SEL_TYPE_KIND_SHIFT)
        struct.pack_into('<I', self._regs, REG_SEL_PARAM_DEFAULT, p['default'])
        struct.pack_into('<I', self._regs, REG_SEL_PARAM_CURRENT, p['current'])
        self._regs[REG_SEL_PARAM_NUM_VALS] = len(p['values'])
        unit = p['unit'].encode()
        self._regs[REG_SEL_UNIT_LEN] = len(unit)
        self._regs[REG_SEL_UNIT:REG_SEL_UNIT + len(unit)] = unit
        self._fill_value(0)

    def _fill_value(self, page):
        p = self._params[self._selected]
        value = p['values'][page] if page < len(p['values']) else 0
        struct.pack_into('<I', self._regs, REG_SEL_VALUE, value)


def _param(name, kind, type_code=1, values=(500, 25000), unit="Hz"):
    return {'name': name, 'kind': kind, 'type_code': type_code,
            'default': 1000, 'current': 2000, 'values': list(values),
            'unit': unit}


@pytest.mark.parametrize("kind, bit", [("live", 1), ("reload", 0)])
def test_read_param_decodes_kind_from_sel_type_bit4(kind, bit):
    bus = _FakeParamBus([_param("pwm_freq", kind)])
    t = NxsI2cTransport(0, _bus_obj=bus)

    got = t.read_param(0)

    assert got['kind'] == kind
    assert (bus.read_byte_data(0, REG_SEL_TYPE) >> SEL_TYPE_KIND_SHIFT) & 1 == bit


def test_kind_bit_does_not_disturb_the_param_type_nibble():
    """The two fields share one register, so a live range parameter must
    still read as a range rather than folding bit 4 into the type."""
    bus = _FakeParamBus([_param("pwm_duty", "live", type_code=1),
                         _param("sample_rate", "reload", type_code=0)])
    t = NxsI2cTransport(0, _bus_obj=bus)

    live_range = t.read_param(0)
    reload_enum = t.read_param(1)

    assert live_range['type'] == 'range' and live_range['kind'] == 'live'
    assert reload_enum['type'] == 'enum' and reload_enum['kind'] == 'reload'


def test_descriptor_round_trips_alongside_the_kind():
    bus = _FakeParamBus([_param("pwm_freq", "live")])
    t = NxsI2cTransport(0, _bus_obj=bus)

    got = t.read_param(0)

    assert got['name'] == "pwm_freq"
    assert got['default'] == 1000
    assert got['current'] == 2000
    assert got['values'] == [500, 25000]
    assert got['unit'] == "Hz"
