"""NxsI2cTransport output-descriptor reads against a fake descriptor
window.

The contract these lock: the SEL window's output view round-trips
name/type/scale/offset/byte order/semantic/count by register address,
and read_outputs() only ever returns a set validated by an unchanged
DESCRIPTOR_EPOCH — a driver swap mid-enumeration discards the mixed
set and converges on the new driver's descriptors."""
import struct

import pytest

from nxs.transports.i2c import (
    NxsI2cTransport,
    REG_DESCRIPTOR_EPOCH, REG_NUM_OUTPUTS, REG_OUTPUT_SELECT,
    REG_SEL_NAME, REG_SEL_NAME_LEN, REG_SEL_OUTPUT_AT,
    REG_SEL_OUTPUT_BYTE_ORDER, REG_SEL_OUTPUT_COUNT, REG_SEL_OUTPUT_OFFSET,
    REG_SEL_OUTPUT_SCALE, REG_SEL_OUTPUT_SEMANTIC, REG_SEL_TYPE,
    REG_SEL_UNIT, REG_SEL_UNIT_LEN,
)


class _FakeDescriptorBus:
    """Emulates the firmware's descriptor surface: NUM_OUTPUTS,
    DESCRIPTOR_EPOCH, and the OUTPUT_SELECT-driven SEL window refill.

    `select_hook` runs before each OUTPUT_SELECT commit — tests use it
    to swap the driver mid-enumeration the way the runner's
    probe-failed auto-advance would."""

    def __init__(self, fields, epoch=1):
        self.select_hook = None
        self._regs = bytearray(256)
        self._fields = []
        self.set_driver(fields, epoch)

    def set_driver(self, fields, epoch):
        self._fields = list(fields)
        self._regs[REG_NUM_OUTPUTS] = len(self._fields)
        self._regs[REG_DESCRIPTOR_EPOCH] = epoch

    def write_byte_data(self, addr, reg, val):
        if reg == REG_OUTPUT_SELECT:
            if self.select_hook is not None:
                self.select_hook()
            self._regs[REG_OUTPUT_SELECT] = val   # echo, as firmware sets post-commit
            self._fill_window(val)

    def read_byte_data(self, addr, reg):
        return self._regs[reg]

    def read_i2c_block_data(self, addr, reg, n):
        return list(self._regs[reg:reg + n])

    def _fill_window(self, idx):
        self._regs[0xC0:0xE9] = bytes(0xE9 - 0xC0)
        if idx >= len(self._fields):
            return
        f = self._fields[idx]
        name = f['name'].encode()
        self._regs[REG_SEL_NAME_LEN] = len(name)
        self._regs[REG_SEL_NAME:REG_SEL_NAME + len(name)] = name
        self._regs[REG_SEL_TYPE] = f['ftype']
        self._regs[REG_SEL_OUTPUT_SCALE:REG_SEL_OUTPUT_SCALE + 4] = \
            struct.pack('<f', f.get('scale', 1.0))
        self._regs[REG_SEL_OUTPUT_OFFSET:REG_SEL_OUTPUT_OFFSET + 4] = \
            struct.pack('<f', f.get('offset', 0.0))
        self._regs[REG_SEL_OUTPUT_BYTE_ORDER] = f.get('byte_order', 0)
        unit = f.get('unit', '').encode()
        self._regs[REG_SEL_UNIT_LEN] = len(unit)
        self._regs[REG_SEL_UNIT:REG_SEL_UNIT + len(unit)] = unit
        self._regs[REG_SEL_OUTPUT_SEMANTIC] = f.get('semantic', 0)
        self._regs[REG_SEL_OUTPUT_COUNT:REG_SEL_OUTPUT_COUNT + 2] = \
            struct.pack('<H', f.get('count', 0))
        self._regs[REG_SEL_OUTPUT_AT] = f.get('byte_off', 0)


IMU_FIELDS = [
    {'name': 'accel_x', 'ftype': 2, 'byte_order': 0, 'semantic': 1,
     'unit': 'm/s^2', 'scale': 0.0024, 'offset': 0.5},
    {'name': 'nmea', 'ftype': 8, 'semantic': 13, 'count': 96},
]
GPS_FIELDS = [
    {'name': 'nmea', 'ftype': 8, 'semantic': 13, 'count': 96},
]


def _transport(bus):
    return NxsI2cTransport(0, _bus_obj=bus)


def test_read_output_round_trips_window_fields():
    t = _transport(_FakeDescriptorBus(IMU_FIELDS))

    o = t.read_output(0)
    assert o == {
        'idx': 0, 'name': 'accel_x', 'type': 'int16',
        'byte_order': 'big', 'semantic': 1, 'byte_off': 0, 'count': 0,
        'scale': pytest.approx(0.0024), 'offset': pytest.approx(0.5),
        'unit': 'm/s^2',
    }

    s = t.read_output(1)
    assert s['type'] == 'string'
    assert s['semantic'] == 13
    assert s['count'] == 96


def test_read_output_out_of_range_reads_cleared_window():
    t = _transport(_FakeDescriptorBus(IMU_FIELDS))
    o = t.read_output(5)
    assert o['name'] == ''
    assert o['count'] == 0


def test_read_outputs_empty_on_epoch_zero():
    """Epoch 0 covers both 'no driver loaded' and firmware without the
    descriptor window — neither has descriptors to read."""
    t = _transport(_FakeDescriptorBus(IMU_FIELDS, epoch=0))
    assert t.read_outputs() == []


def test_read_outputs_returns_coherent_set():
    t = _transport(_FakeDescriptorBus(IMU_FIELDS, epoch=7))
    outs = t.read_outputs()
    assert [o['name'] for o in outs] == ['accel_x', 'nmea']
    assert [o['idx'] for o in outs] == [0, 1]


def test_read_outputs_retries_when_driver_swaps_mid_enumeration():
    """The probe-failed auto-advance can replace the driver between
    two descriptor reads; the epoch mismatch must discard the mixed
    set and re-enumerate the new driver."""
    bus = _FakeDescriptorBus(IMU_FIELDS, epoch=7)
    t = _transport(bus)

    calls = {'n': 0}

    def swap_once():
        calls['n'] += 1
        if calls['n'] == 2:  # between descriptor 0 and descriptor 1
            bus.set_driver(GPS_FIELDS, epoch=8)
            bus.select_hook = None

    bus.select_hook = swap_once

    outs = t.read_outputs()
    assert [o['name'] for o in outs] == ['nmea']  # the new set, never a mix


def test_read_outputs_unstable_epoch_returns_none():
    """An epoch that never holds still across the retry budget means the
    set is transiently unreadable: None (caller keeps its previous
    knowledge), never a truncated or empty set presented as truth."""
    bus = _FakeDescriptorBus(IMU_FIELDS, epoch=1)
    state = {"epoch": 1}

    def churn():
        state["epoch"] += 1
        bus.set_driver(IMU_FIELDS, state["epoch"])

    bus.select_hook = churn
    t = _transport(bus)
    assert t.read_outputs() is None
