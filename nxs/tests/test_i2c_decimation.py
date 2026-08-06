"""Per-subject decimation over the I2C DECIMATION_SELECT/VALUE window
mirrors the Cyphal decimation.<subject> registers: select a SubjectBucket,
poll the deferred echo, read or write the u16 factor."""
import pytest

from nxs.transports.i2c import (
    NxsI2cTransport, SUBJECT_BUCKETS,
    REG_DECIMATION, REG_DECIMATION_SELECT, REG_DECIMATION_VALUE)


class _FakeDecimBus:
    """Serves the select window with the firmware's deferred-echo
    behavior: the echo lags one read behind the write."""

    def __init__(self, factors):
        self._factors = dict(factors)
        self._select = None
        self._echo_pending = False
        self._device_wide = 1

    def write_byte_data(self, addr, reg, val):
        assert reg == REG_DECIMATION_SELECT
        self._select = val
        self._echo_pending = True

    def read_byte_data(self, addr, reg):
        assert reg == REG_DECIMATION_SELECT
        if self._echo_pending:
            self._echo_pending = False
            return 0xFF
        return self._select

    def write_word_data(self, addr, reg, val):
        if reg == REG_DECIMATION:
            self._device_wide = val
        else:
            assert reg == REG_DECIMATION_VALUE
            self._factors[self._select] = val

    def read_word_data(self, addr, reg):
        if reg == REG_DECIMATION:
            return self._device_wide
        assert reg == REG_DECIMATION_VALUE
        return self._factors[self._select]


def _transport(bus):
    return NxsI2cTransport(0, _bus_obj=bus)


def test_subject_factor_round_trips():
    bus = _FakeDecimBus({SUBJECT_BUCKETS['temperature']: 25})
    t = _transport(bus)
    assert t.read_decimation(subject='temperature') == 25
    t.write_decimation(40, subject='temperature')
    assert t.read_decimation(subject='temperature') == 40


def test_device_wide_path_is_untouched():
    t = _transport(_FakeDecimBus({}))
    t.write_decimation(5)
    assert t.read_decimation() == 5


def test_unknown_subject_raises():
    with pytest.raises(ValueError, match='unknown subject'):
        _transport(_FakeDecimBus({})).write_decimation(2, subject='gravity')


def test_subject_tokens_match_the_semantic_buckets():
    assert SUBJECT_BUCKETS == {'acceleration': 1, 'angular_velocity': 2,
                               'magnetic_field': 3, 'temperature': 4,
                               'pressure': 5, 'scalar': 6}
