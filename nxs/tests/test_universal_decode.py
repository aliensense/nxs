"""Universal decode — the test that keeps the "driverless, universal"
promise honest.

The claim under test: a host with no copy of a sensor's driver can
read raw sample bytes off the device and turn them into correct
physical values, using only the descriptors the device serves over
its I2C window.

Anti-trick rules every case here obeys:

  1. Descriptors are recovered from the *NXS image* —
     `deserialize(serialize(compile(driver)))` — i.e. exactly the
     bytes the device parses and re-exposes, not the live compiled
     object the host happens to hold.
  2. The decode is driven through the I2C host path
     (`NxsI2cTransport.read_outputs` → `descriptor.parse_sample`)
     fed by a fake register window. The source driver class is never
     imported or compiled inside the decode — a path that secretly
     leaned on the local driver file would round-trip but is
     structurally excluded here.
  3. Physical-value expectations are recomputed from the datasheet
     formula in the test, not read back from the descriptor and
     multiplied through (which would be tautological).

Scope honesty: this proves the pipeline faithfully *carries and
applies* whatever a driver declared — field order, type, byte order,
scale, offset, string width, semantic. It cannot verify a driver's
scale is physically right; that is the datasheet's responsibility,
and the authoring AI's. Universality holds to the extent the
datasheet was encoded correctly.
"""
import math
import pkgutil
import struct

import pytest

import nxs.drivers as drivers_pkg
from nxs.descriptor import (
    effective_scale_fields, field_size, is_decodable, load_driver,
    parse_sample, sample_width)
from nxs.image import _FIELD_TYPE_MAP, deserialize, serialize
from nxs.transports.i2c import NxsI2cTransport
from nxs.tests.test_i2c_outputs import _FakeDescriptorBus


# Every shipped driver except the *_reference teaching copies.
ALL_DRIVERS = sorted(
    m.name for m in pkgutil.iter_modules(drivers_pkg.__path__)
    if not m.name.endswith('_reference'))


def _image_fields(name, config=None):
    """Descriptors as the *device serves them* — recovered from the NXS
    image (not the live compiled object), with each field's live-param
    scaling folded in. The firmware folds `base * param.current` into the
    scale it exposes on the I2C window / GetOutputInfo, so a faithful
    device simulation must too — otherwise a ranged field would decode at
    its base scale, not its effective one."""
    compiled = load_driver(name)().compile(config or {})
    restored = deserialize(serialize(compiled))
    fields = effective_scale_fields(restored.output_fields, restored.params)
    return fields, compiled


def _window_fields(image_fields):
    """Map image output_fields onto the byte-level shape the fake I2C
    descriptor window is filled from (numeric type code, byte-order
    bit), so the read goes through the same registers real hardware
    serves."""
    return [{
        'name': f['name'],
        'ftype': _FIELD_TYPE_MAP[f['type']],
        'byte_order': 0 if f['byte_order'] == 'big' else 1,
        'semantic': f.get('semantic', 0),
        'byte_off': f.get('byte_off', 0),
        'count': f.get('count', 0),
        'scale': f.get('scale', 1.0),
        'offset': f.get('offset', 0.0),
        'unit': f.get('unit', ''),
    } for f in image_fields]


def _decode_over_i2c(image_fields, raw):
    """Read descriptors back over the (emulated) I2C window and decode
    `raw` with them — the full host path, zero driver import."""
    bus = _FakeDescriptorBus(_window_fields(image_fields), epoch=1)
    device_fields = NxsI2cTransport(0, _bus_obj=bus).read_outputs()
    return device_fields, parse_sample(raw, device_fields)


# ── Universal: every driver round-trips and drives a decode ────────

@pytest.mark.parametrize('name', ALL_DRIVERS)
def test_descriptor_set_round_trips(name):
    fields, compiled = _image_fields(name)

    # One descriptor per declared output, in declaration order.
    assert [f['name'] for f in fields] == \
        [f['name'] for f in compiled.output_fields]

    # Every field lies within the declared sample — the invariant the
    # CLI tripwire enforces against a stale local driver file. Offsets
    # may gap (binary-record drivers), so the bound is each field's
    # end, not a width sum.
    for f in fields:
        assert f['byte_off'] + field_size(f) <= compiled.sample_size

    # Semantic codes stay in the defined range.
    from nxs._generated_constants import FieldSemantics
    max_semantic = max(FieldSemantics.FieldSemantic._NAMES)
    for f in fields:
        assert 0 <= f['semantic'] <= max_semantic


@pytest.mark.parametrize('name', ALL_DRIVERS)
def test_decode_over_i2c_window_without_driver_file(name):
    fields, compiled = _image_fields(name)

    # A deterministic synthetic sample of the declared width.
    raw = bytes((i * 7 + 3) & 0xFF for i in range(compiled.sample_size))
    device_fields, values = _decode_over_i2c(fields, raw)

    # The window served the same fields, and decode produced exactly
    # one value per field — no field dropped, none invented.
    assert [d['name'] for d in device_fields] == [f['name'] for f in fields]
    assert set(values) == {f['name'] for f in fields}


# ── Honest physical values across three structural shapes ──────────

def test_iam20680_imu_decodes_to_si_units():
    """int16 big-endian, scaled — accel m/s^2, gyro rad/s, temp degC."""
    fields, _ = _image_fields('iam20680', {
        'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000,
        'trigger': 'drdy'})

    # Datasheet, recomputed here independently of the descriptor:
    g = 9.80665
    accel_scale = g / 4096.0              # ±8 g, int16
    gyro_scale = 2000.0 / 32768.0 * math.pi / 180.0  # ±2000 dps
    temp_scale, temp_offset = 1.0 / 326.8, 298.15  # kelvin at the source

    # 7 × int16 BE: accel xyz, temp, gyro xyz.
    raw = struct.pack('>7h', 4096, -4096, 2048, 0, 1000, -1000, 500)
    _, v = _decode_over_i2c(fields, raw)

    assert v['accel_x'] == pytest.approx(4096 * accel_scale)   # ~ +1 g
    assert v['accel_y'] == pytest.approx(-4096 * accel_scale)
    assert v['accel_z'] == pytest.approx(2048 * accel_scale)
    assert v['temp'] == pytest.approx(0 * temp_scale + temp_offset)
    assert v['gyro_x'] == pytest.approx(1000 * gyro_scale)
    assert v['gyro_z'] == pytest.approx(500 * gyro_scale)


def test_neo_m9n_ubx_decodes_geodetic_si():
    """Binary-record path: scattered little-endian int32 geodetic fields
    decode to SI through the full I2C window path. Lat/lon are recomputed
    here from the datasheet formula (raw 1e-7 deg → rad), and the two
    altitudes pin the MSL-vs-ellipsoid datum split — altitude = MSL at
    offset 40, alt_ellipsoid at 36 — independent of the descriptor's
    declared scale, so a swapped datum or a wrong factor fails here."""
    fields, compiled = _image_fields('neo_m9n')   # default protocol: ubx
    raw = bytearray(compiled.sample_size)
    struct.pack_into('<i', raw, 28, 85409900)      # longitude 8.54099 deg
    struct.pack_into('<i', raw, 32, 473977400)     # latitude 47.39774 deg
    struct.pack_into('<i', raw, 36, 410000)        # alt_ellipsoid 410.0 m
    struct.pack_into('<i', raw, 40, 408000)        # altitude (MSL) 408.0 m
    _, v = _decode_over_i2c(fields, bytes(raw))

    deg = math.pi / 180.0
    assert v['longitude'] == pytest.approx(85409900 * 1e-7 * deg, abs=1e-7)
    assert v['latitude'] == pytest.approx(473977400 * 1e-7 * deg, abs=1e-7)
    assert v['alt_ellipsoid'] == pytest.approx(410.0, abs=1e-3)
    assert v['altitude'] == pytest.approx(408.0, abs=1e-3)


def test_unknown_field_type_is_rejected_not_crashed():
    """A device serving a field-type code newer than this tool knows
    must be detectable (is_decodable False) and, if decoded anyway,
    raise a clear error — never a cryptic KeyError mid-stream."""
    assert is_decodable([{'type': 'int16'}, {'type': 'string', 'count': 4}])
    assert not is_decodable([{'type': 'int16'}, {'type': 'type9'}])

    with pytest.raises(ValueError):
        parse_sample(b'\x00\x00', [{'name': 'x', 'type': 'type9'}])


def test_sample_width_is_none_for_unknown_string_width():
    """A string field with unknown count (0/absent) makes the total
    indeterminate — None, not a misleading undercount — so the CLI
    tripwire and the per-sample total don't cry wolf on older firmware."""
    known = [{'type': 'int16'}, {'type': 'string', 'count': 96}]
    assert sample_width(known) == 2 + 96

    assert sample_width([{'type': 'int16'}, {'type': 'string', 'count': 0}]) is None
    assert sample_width([{'type': 'int16'}, {'type': 'string'}]) is None


def test_string_field_decodes_when_count_unknown():
    """Firmware predating the descriptor count field sends no count, so
    it defaults to 0. A string field must then consume the rest of the
    buffer, not zero bytes — the backward-compat path."""
    sentence = b"$GPGGA,123519,4807.038,N*47"
    raw = sentence + b'\xff' * 20

    # count key absent (older host) and count == 0 (older firmware tail)
    # must both fall back to "rest of buffer".
    for fields in ([{'name': 'nmea', 'type': 'string', 'byte_order': 'big'}],
                   [{'name': 'nmea', 'type': 'string', 'count': 0}]):
        assert parse_sample(raw, fields)['nmea'] == sentence.decode('ascii')


def test_string_field_with_known_count_decodes_over_i2c():
    """A known-count string field survives the full host path — I2C
    descriptor window → read_outputs() → parse_sample() — not just the
    count==0 backward-compat fallback. neo_m9n in NMEA mode is the
    shipping string-output driver; its declared count must ride the
    window and bound the decode."""
    fields, compiled = _image_fields('neo_m9n', {'protocol': 'nmea'})
    sentence = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M*47"
    raw = sentence + b'\xff' * (compiled.sample_size - len(sentence))
    device_fields, values = _decode_over_i2c(fields, raw)

    # The declared count (not 0) rode the window and bounded the decode.
    nmea = next(d for d in device_fields if d['name'] == 'nmea')
    assert nmea['count'] == 82
    assert values['nmea'] == sentence.decode('ascii')


# ── The trick we refuse to play ────────────────────────────────────

def test_decode_path_does_not_consult_the_driver_module():
    """A host that decoded against its own driver copy would still
    pass the round-trip tests above — so prove the negative directly:
    decoding succeeds when the source module is made unimportable.

    Renders the universality claim falsifiable: if any decode step
    reached for `nxs.drivers.iam20680`, this raises instead of
    asserting."""
    import builtins

    fields, _ = _image_fields('iam20680', {
        'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000,
        'trigger': 'drdy'})                      # capture descriptors first
    raw = struct.pack('>7h', 4096, 0, 0, 0, 0, 0, 0)  # accel_x = +1 g

    real_import = builtins.__import__

    def _ban_driver(name, *args, **kwargs):
        if 'drivers.iam20680' in name:
            raise AssertionError(
                "decode path imported the source driver module")
        return real_import(name, *args, **kwargs)

    builtins.__import__ = _ban_driver
    try:
        _, v = _decode_over_i2c(fields, raw)
    finally:
        builtins.__import__ = real_import

    assert v['accel_x'] == pytest.approx(4096 * 9.80665 / 4096.0)  # ~ +1 g
