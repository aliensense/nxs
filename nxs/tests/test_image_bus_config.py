"""NXS per-bus bus_config trailer round-trip + golden-byte tests.

Locks the wire-format contract for the driver-declared bus configuration:
the trailer is a count byte followed by one register-access / UART profile
block per supported bus. Every kind round-trips losslessly, an absent
trailer survives as ``bus_config = None`` (firmware uses DTS defaults), and
the SPI block's exact bytes are pinned so the firmware parser can be
written against them.
"""
import pytest

from nxs.compiler import CompiledDriver
from nxs.image import serialize, deserialize, _read_bus_config, MAX_BUS_PROFILES


def _empty_driver(name: str = "TestDriver") -> CompiledDriver:
    """Minimal CompiledDriver with no bytecode or fields — leaves the
    bus_config trailer as the only meaningful section to roundtrip."""
    return CompiledDriver(
        bytecode=b"",
        sample_size=0,
        name=name,
        config={},
        output_fields=[],
        params=[],
        patch_map=[],
    )


def test_no_bus_config_roundtrip():
    cd = _empty_driver()
    cd.bus_config = None
    assert deserialize(serialize(cd)).bus_config is None


def test_per_bus_list_roundtrip():
    cd = _empty_driver()
    cd.bus_config = [
        {'kind': 'i2c', 'max_hz': 400_000, 'auto_inc': 'implicit', 'pec': 'none'},
        {'kind': 'spi', 'max_hz': 1_100_000, 'spi_mode': 0, 'addr_bytes': 2,
         'rw_read_level': 0, 'dummy_bytes': 0, 'auto_inc': 'implicit'},
    ]
    assert deserialize(serialize(cd)).bus_config == cd.bus_config


def test_spi_switches_roundtrip():
    cd = _empty_driver()
    cd.bus_config = [
        {'kind': 'spi', 'max_hz': 8_000_000, 'spi_mode': 0b111, 'addr_bytes': 2,
         'rw_read_level': 0, 'dummy_bytes': 1, 'auto_inc': 'msb'},
    ]
    assert deserialize(serialize(cd)).bus_config == cd.bus_config


def test_i2c_switches_roundtrip():
    cd = _empty_driver()
    cd.bus_config = [
        {'kind': 'i2c', 'max_hz': 100_000, 'auto_inc': 'msb', 'pec': 'crc8'},
    ]
    assert deserialize(serialize(cd)).bus_config == cd.bus_config


def test_uart_roundtrip():
    cd = _empty_driver()
    cd.bus_config = [
        {'kind': 'uart', 'max_hz': 38_400, 'uart_parity': 1,
         'uart_stop_bits': 2, 'uart_data_bits': 8},
    ]
    assert deserialize(serialize(cd)).bus_config == cd.bus_config


def test_spi_profile_golden_bytes():
    """The serialized trailer for one FXOS-style SPI profile is pinned to
    its exact wire bytes — the format the firmware parser must mirror.

    num_profiles=1 | kind=SPI(0x01) | max_hz=1_100_000 LE | spi_mode=0 |
    addr_bytes=2 | rw_read_level=0 | dummy_bytes=0 | auto_inc=IMPLICIT(0)
    """
    cd = _empty_driver()
    cd.bus_config = [
        {'kind': 'spi', 'max_hz': 1_100_000, 'spi_mode': 0, 'addr_bytes': 2,
         'rw_read_level': 0, 'dummy_bytes': 0, 'auto_inc': 'implicit'},
    ]
    expected = bytes([
        0x01,                    # num_profiles
        0x01,                    # kind = SPI
        0xE0, 0xC8, 0x10, 0x00,  # max_hz = 1_100_000, little-endian
        0x00,                    # spi_mode
        0x02,                    # addr_bytes
        0x00,                    # rw_read_level (read clears bit 7)
        0x00,                    # dummy_bytes
        0x00,                    # auto_inc = IMPLICIT
    ])
    assert serialize(cd).endswith(expected)


def test_unknown_kind_name_rejected():
    cd = _empty_driver()
    cd.bus_config = [{'kind': 'can', 'max_hz': 1_000_000}]
    with pytest.raises(ValueError) as exc:
        serialize(cd)
    assert "unknown bus kind" in str(exc.value)


def test_deserialize_count_over_max_rejected():
    # A trailer count above MAX_BUS_PROFILES is rejected on read, mirroring
    # the firmware's BUS_CONFIG_TOO_MANY, instead of over-reading.
    with pytest.raises(ValueError):
        _read_bus_config(bytes([MAX_BUS_PROFILES + 1]), 0)


def test_deserialize_unknown_switch_code_rejected():
    # count=1, kind=I2C, max_hz=0, auto_inc=0xEE (unknown) — fail fast rather
    # than silently decode to a default the firmware would reject.
    data = bytes([1, 0x00, 0, 0, 0, 0, 0xEE, 0x00])
    with pytest.raises(ValueError):
        _read_bus_config(data, 0)
