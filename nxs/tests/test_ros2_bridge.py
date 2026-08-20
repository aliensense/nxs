"""Tests for the descriptor-driven ROS 2 bridge.

Everything here runs without ROS: fillers are asserted as dotted-path
dicts, the runtime is exercised through injected stub message classes
and a recording fake runtime, and the CLI verb through a fake opener.
The one test that needs a real rclpy (`test_real_rclpy_smoke`) skips
when no ROS 2 environment is sourced — CI has none.

The end-to-end tests decode raw bytes through the same device-served
descriptor path the CLI uses (`_decode_over_i2c`), then assert the ROS
message values equal that decode — the falsifiable bridge claim.
"""

import argparse
import logging
import os
import subprocess
import sys as _sys
import math
import re
import struct
import time
import sys
import types

import pytest

from nxs import ros2_bridge as rb
from nxs._generated_constants import FieldSemantics
from nxs.client import Sample
from nxs.tests.test_universal_decode import _decode_over_i2c, _image_fields

Sem = FieldSemantics.FieldSemantic


def _f(name, semantic=Sem.GENERIC, ftype='int16', **kw):
    d = {'name': name, 'type': ftype, 'byte_order': 'big',
         'semantic': int(semantic), 'count': 0, 'scale': 1.0,
         'offset': 0.0, 'unit': ''}
    d.update(kw)
    return d


IMU_FIELDS = [
    _f('accel_x', Sem.ACCEL_X), _f('accel_y', Sem.ACCEL_Y),
    _f('accel_z', Sem.ACCEL_Z), _f('temp', Sem.TEMPERATURE),
    _f('gyro_x', Sem.GYRO_X), _f('gyro_y', Sem.GYRO_Y),
    _f('gyro_z', Sem.GYRO_Z),
]

GNSS_FIELDS = [
    _f('fix_type', Sem.FIX_TYPE, 'uint8'), _f('num_sv', ftype='uint8'),
    _f('longitude', Sem.LONGITUDE, 'int32'),
    _f('latitude', Sem.LATITUDE, 'int32'),
    _f('alt_ellipsoid', ftype='int32'),
    _f('altitude', Sem.ALTITUDE, 'int32'),
    _f('pos_h_acc', Sem.POS_H_ACC, 'uint32'),
    _f('pos_v_acc', Sem.POS_V_ACC, 'uint32'),
    _f('vel_north', Sem.VEL_NORTH, 'int32'),
    _f('vel_east', Sem.VEL_EAST, 'int32'),
    _f('vel_down', Sem.VEL_DOWN, 'int32'),
    _f('speed', Sem.SPEED, 'int32'),
    _f('heading', ftype='int32'),
    _f('vel_s_acc', Sem.VEL_S_ACC, 'uint32'),
    _f('pdop', ftype='uint16'),
]


def _by_key(pubs):
    return {p.key: p for p in pubs}


# ── Planner ─────────────────────────────────────────────────────────

def test_plan_imu_and_temperature_from_imu_semantics():
    pubs = _by_key(rb.plan_publications(IMU_FIELDS))
    assert set(pubs) == {'imu', 'temperature'}
    imu = pubs['imu']
    assert imu.msg_type == 'sensor_msgs/msg/Imu'
    assert imu.topic == 'imu'
    assert imu.bindings == {
        'accel_x': 'accel_x', 'accel_y': 'accel_y', 'accel_z': 'accel_z',
        'gyro_x': 'gyro_x', 'gyro_y': 'gyro_y', 'gyro_z': 'gyro_z'}
    assert pubs['temperature'].bindings == {'value': 'temp'}


def test_plan_accel_only_imu_has_no_gyro_bindings():
    fields = [_f('accel_x', Sem.ACCEL_X), _f('accel_y', Sem.ACCEL_Y),
              _f('accel_z', Sem.ACCEL_Z)]
    pubs = _by_key(rb.plan_publications(fields))
    assert set(pubs) == {'imu'}
    assert 'gyro_x' not in pubs['imu'].bindings


def test_plan_partial_vector_demotes_to_per_field_topics():
    fields = [_f('accel_x', Sem.ACCEL_X), _f('accel_y', Sem.ACCEL_Y)]
    pubs = rb.plan_publications(fields)
    assert [p.kind for p in pubs] == ['scalar', 'scalar']
    assert [p.topic for p in pubs] == ['accel_x', 'accel_y']


def test_plan_mag_pressure_and_humidity():
    fields = [_f('mag_x', Sem.MAG_X), _f('mag_y', Sem.MAG_Y),
              _f('mag_z', Sem.MAG_Z), _f('pressure', Sem.PRESSURE),
              _f('humidity', Sem.HUMIDITY)]
    pubs = _by_key(rb.plan_publications(fields))
    assert pubs['mag'].msg_type == 'sensor_msgs/msg/MagneticField'
    assert pubs['pressure'].msg_type == 'sensor_msgs/msg/FluidPressure'
    # No canonical humidity unit -> per-field fallback, not RelativeHumidity.
    assert pubs['field:humidity'].msg_type == 'std_msgs/msg/Float64'


def test_plan_geodetic_group_consumes_quality_fields():
    pubs = _by_key(rb.plan_publications(GNSS_FIELDS))
    fix = pubs['fix']
    assert fix.msg_type == 'sensor_msgs/msg/NavSatFix'
    assert fix.bindings == {
        'lat': 'latitude', 'lon': 'longitude', 'alt': 'altitude',
        'h_acc': 'pos_h_acc', 'v_acc': 'pos_v_acc',
        'fix_type': 'fix_type'}
    vel = pubs['vel']
    assert vel.msg_type == 'geometry_msgs/msg/TwistStamped'
    assert vel.bindings == {'north': 'vel_north', 'east': 'vel_east',
                            'down': 'vel_down', 'fix_type': 'fix_type'}
    # Consumed by fix/vel: no per-field fallback topic.
    for consumed in ('latitude', 'pos_h_acc', 'pos_v_acc', 'fix_type'):
        assert f'field:{consumed}' not in pubs
    # Not consumed: generic quality fields and vel_s_acc (TwistStamped
    # carries no covariance) stay visible per-field.
    for kept in ('num_sv', 'pdop', 'heading', 'alt_ellipsoid', 'speed',
                 'vel_s_acc'):
        assert pubs[f'field:{kept}'].msg_type == 'std_msgs/msg/Float64'


def test_plan_incomplete_position_triple_falls_back():
    fields = [_f('latitude', Sem.LATITUDE, 'int32'),
              _f('longitude', Sem.LONGITUDE, 'int32')]
    pubs = _by_key(rb.plan_publications(fields))
    assert 'fix' not in pubs
    assert set(pubs) == {'field:latitude', 'field:longitude'}


def test_plan_partial_ned_velocity_falls_back():
    fields = [_f('vel_north', Sem.VEL_NORTH, 'int32'),
              _f('vel_east', Sem.VEL_EAST, 'int32')]
    pubs = _by_key(rb.plan_publications(fields))
    assert 'vel' not in pubs
    assert set(pubs) == {'field:vel_north', 'field:vel_east'}


def test_plan_string_fields_route_to_string():
    pubs = rb.plan_publications([_f('nmea', Sem.NMEA, 'string', count=82)])
    assert pubs[0].msg_type == 'std_msgs/msg/String'
    assert pubs[0].topic == 'nmea'


def test_plan_duplicate_semantic_first_claims_slot():
    fields = [_f('temp_a', Sem.TEMPERATURE), _f('temp_b', Sem.TEMPERATURE)]
    pubs = _by_key(rb.plan_publications(fields))
    assert pubs['temperature'].bindings == {'value': 'temp_a'}
    assert pubs['field:temp_b'].msg_type == 'std_msgs/msg/Float64'


def test_navsat_status_policy_strict_and_fail_closed():
    for ft in (2, 3, 4):
        assert rb.navsat_status_from_fix_type(ft) == rb.NAVSAT_STATUS_FIX
    # No fix, dead-reckoning-only, time-only: gated.
    for ft in (0, 1, 5):
        assert rb.navsat_status_from_fix_type(ft) == rb.NAVSAT_STATUS_NO_FIX
    # Unknown vocabulary: fail-closed, never published as a position.
    for ft in (6, 42, 255, -1):
        assert rb.navsat_status_from_fix_type(ft) == rb.NAVSAT_STATUS_NO_FIX


def test_plan_field_named_like_standard_topic_gets_prefixed():
    fields = [_f('accel_x', Sem.ACCEL_X), _f('accel_y', Sem.ACCEL_Y),
              _f('accel_z', Sem.ACCEL_Z), _f('imu')]
    pubs = _by_key(rb.plan_publications(fields))
    assert pubs['imu'].topic == 'imu'
    assert pubs['field:imu'].topic == 'field_imu'


def test_sanitize_ros_name():
    # Suite roles are dashed; ROS names are [A-Za-z_][A-Za-z0-9_]*.
    assert rb.sanitize_ros_name('unit-i2c-10-30') == 'unit_i2c_10_30'
    assert rb.sanitize_ros_name('imu-mast') == 'imu_mast'
    assert rb.sanitize_ros_name('alt.ellipsoid') == 'alt_ellipsoid'
    # A leading digit is illegal as the first char.
    assert rb.sanitize_ros_name('3d') == '_3d'
    # Already-valid names pass through; the map is idempotent.
    assert rb.sanitize_ros_name('accel_x') == 'accel_x'
    assert rb.sanitize_ros_name(rb.sanitize_ros_name('unit-i2c-10-30')) \
        == 'unit_i2c_10_30'


def test_join_topic_sanitizes_every_token():
    # A dashed unit name would raise InvalidTopicNameException at
    # create_publisher; join_topic must produce a valid ROS name.
    assert rb.join_topic('nxs', 'unit-i2c-10-30', 'imu') \
        == 'nxs/unit_i2c_10_30/imu'
    assert rb.join_topic('nxs', None, 'imu') == 'nxs/imu'
    assert re.fullmatch(r'[A-Za-z_][A-Za-z0-9_/]*',
                        rb.join_topic('nxs', 'imu-mast', 'fix'))


def test_plan_every_field_lands_at_least_once():
    fields = IMU_FIELDS + GNSS_FIELDS + [
        _f('angle', Sem.ANGLE), _f('note', ftype='string', count=8)]
    pubs = rb.plan_publications(fields)
    bound = set()
    for p in pubs:
        bound.update(p.bindings.values())
    assert bound == {f['name'] for f in fields}


# ── Fillers ─────────────────────────────────────────────────────────

def test_stamp_from_us():
    assert rb.stamp_from_us(0) == (0, 0)
    assert rb.stamp_from_us(123_456_789) == (123, 456_789_000)
    two_days = 2 * 24 * 3600 * 1_000_000 + 42
    assert rb.stamp_from_us(two_days) == (172_800, 42_000)


def test_fill_imu_covariance_conventions():
    pubs = _by_key(rb.plan_publications(IMU_FIELDS))
    out = rb.fill_imu(pubs['imu'], {
        'accel_x': 0.1, 'accel_y': -0.2, 'accel_z': 9.81,
        'gyro_x': 0.01, 'gyro_y': 0.02, 'gyro_z': -0.03})
    assert out['linear_acceleration.z'] == pytest.approx(9.81)
    assert out['angular_velocity.y'] == pytest.approx(0.02)
    # No orientation estimate -> leading -1; measured axes -> zeros
    # ("covariance unknown" per the Imu message convention).
    assert out['orientation_covariance'][0] == -1.0
    assert out['linear_acceleration_covariance'] == [0.0] * 9
    assert out['angular_velocity_covariance'] == [0.0] * 9
    # A bound value missing from the sample suppresses the message.
    assert rb.fill_imu(pubs['imu'], {'accel_x': 0.1}) is None


def test_fill_imu_missing_triad_marks_no_estimate():
    fields = [_f('accel_x', Sem.ACCEL_X), _f('accel_y', Sem.ACCEL_Y),
              _f('accel_z', Sem.ACCEL_Z)]
    pub = rb.plan_publications(fields)[0]
    out = rb.fill_imu(pub, {'accel_x': 1.0, 'accel_y': 2.0, 'accel_z': 3.0})
    assert out['linear_acceleration_covariance'] == [0.0] * 9
    assert out['angular_velocity_covariance'][0] == -1.0
    assert 'angular_velocity.x' not in out


def test_fill_temperature_kelvin_to_celsius():
    pub = rb.plan_publications([_f('temp', Sem.TEMPERATURE)])[0]
    out = rb.fill_temperature(pub, {'temp': 298.15})
    assert out['temperature'] == pytest.approx(25.0)
    assert out['variance'] == 0.0


def test_fill_pressure_and_mag_passthrough():
    pubs = _by_key(rb.plan_publications([
        _f('mag_x', Sem.MAG_X), _f('mag_y', Sem.MAG_Y),
        _f('mag_z', Sem.MAG_Z), _f('pressure', Sem.PRESSURE)]))
    out = rb.fill_pressure(pubs['pressure'], {'pressure': 101325.0})
    assert out == {'fluid_pressure': 101325.0, 'variance': 0.0}
    out = rb.fill_mag(pubs['mag'],
                      {'mag_x': 1e-5, 'mag_y': -2e-5, 'mag_z': 5e-5})
    assert out['magnetic_field.y'] == pytest.approx(-2e-5)
    assert out['magnetic_field_covariance'] == [0.0] * 9


def _gnss_pubs(monkeypatch, status=rb.NAVSAT_STATUS_FIX):
    monkeypatch.setattr(rb, 'navsat_status_from_fix_type',
                        lambda ft: status)
    return _by_key(rb.plan_publications(GNSS_FIELDS))


GNSS_VALUES = {
    'fix_type': 3.0, 'latitude': math.radians(47.39774),
    'longitude': math.radians(8.54099), 'altitude': 408.0,
    'pos_h_acc': 1.5, 'pos_v_acc': 2.5,
    'vel_north': 1.0, 'vel_east': 2.0, 'vel_down': 0.5,
}


def test_fill_navsat_degrees_and_diagonal_covariance(monkeypatch):
    pubs = _gnss_pubs(monkeypatch)
    out = rb.fill_navsat(pubs['fix'], GNSS_VALUES)
    assert out['latitude'] == pytest.approx(47.39774)
    assert out['longitude'] == pytest.approx(8.54099)
    assert out['altitude'] == pytest.approx(408.0)
    assert out['status.status'] == rb.NAVSAT_STATUS_FIX
    assert out['status.service'] == rb.NAVSAT_SERVICE_ALL
    cov = out['position_covariance']
    assert cov[0] == pytest.approx(1.5 ** 2)
    assert cov[4] == pytest.approx(1.5 ** 2)
    assert cov[8] == pytest.approx(2.5 ** 2)
    assert out['position_covariance_type'] == \
        rb.COVARIANCE_TYPE_DIAGONAL_KNOWN


def test_fill_navsat_missing_accuracy_uses_sentinel(monkeypatch):
    pubs = _gnss_pubs(monkeypatch)
    values = {k: v for k, v in GNSS_VALUES.items() if k != 'pos_v_acc'}
    out = rb.fill_navsat(pubs['fix'], values)
    assert out['position_covariance'][8] == rb.UNKNOWN_VARIANCE
    assert out['position_covariance_type'] == \
        rb.COVARIANCE_TYPE_DIAGONAL_KNOWN

    values = {k: v for k, v in GNSS_VALUES.items()
              if k not in ('pos_h_acc', 'pos_v_acc')}
    out = rb.fill_navsat(pubs['fix'], values)
    assert out['position_covariance'] == [0.0] * 9
    assert out['position_covariance_type'] == rb.COVARIANCE_TYPE_UNKNOWN


def test_fill_navsat_no_fix_is_nan_not_zeros(monkeypatch):
    pubs = _gnss_pubs(monkeypatch, status=rb.NAVSAT_STATUS_NO_FIX)
    out = rb.fill_navsat(pubs['fix'], GNSS_VALUES)
    assert out['status.status'] == rb.NAVSAT_STATUS_NO_FIX
    assert math.isnan(out['latitude'])
    assert math.isnan(out['longitude'])
    assert math.isnan(out['altitude'])
    assert out['position_covariance_type'] == rb.COVARIANCE_TYPE_UNKNOWN


def test_fill_navsat_suppressed_when_position_absent(monkeypatch):
    pubs = _gnss_pubs(monkeypatch)
    values = {k: v for k, v in GNSS_VALUES.items() if k != 'latitude'}
    assert rb.fill_navsat(pubs['fix'], values) is None


def test_fill_navsat_without_fix_type_binding_trusts_driver():
    fields = [_f('latitude', Sem.LATITUDE, 'int32'),
              _f('longitude', Sem.LONGITUDE, 'int32'),
              _f('altitude', Sem.ALTITUDE, 'int32')]
    pub = rb.plan_publications(fields)[0]
    out = rb.fill_navsat(pub, {'latitude': 0.1, 'longitude': 0.2,
                               'altitude': 10.0})
    assert out['status.status'] == rb.NAVSAT_STATUS_FIX


def test_fill_twist_ned_to_enu(monkeypatch):
    pubs = _gnss_pubs(monkeypatch)
    out = rb.fill_twist(pubs['vel'], GNSS_VALUES)
    assert out['twist.linear.x'] == pytest.approx(2.0)    # east
    assert out['twist.linear.y'] == pytest.approx(1.0)    # north
    assert out['twist.linear.z'] == pytest.approx(-0.5)   # -down
    values = {k: v for k, v in GNSS_VALUES.items() if k != 'vel_down'}
    assert rb.fill_twist(pubs['vel'], values) is None


def test_fill_twist_suppressed_without_fix(monkeypatch):
    pubs = _gnss_pubs(monkeypatch, status=rb.NAVSAT_STATUS_NO_FIX)
    assert rb.fill_twist(pubs['vel'], GNSS_VALUES) is None


def test_fill_scalar_and_text():
    scalar = rb.plan_publications([_f('angle', Sem.ANGLE)])[0]
    assert rb.fill_scalar(scalar, {'angle': 1.57}) == {'data': 1.57}
    assert rb.fill_scalar(scalar, {}) is None
    text = rb.plan_publications([_f('nmea', Sem.NMEA, 'string')])[0]
    assert rb.fill_text(text, {'nmea': '$GPGGA,x*47\r\n'}) == \
        {'data': '$GPGGA,x*47'}


# ── Falsifiable end-to-end: bridge values == the CLI's decode ───────

def test_iam20680_bridge_equals_cli_decode():
    fields, _ = _image_fields('iam20680', {
        'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000,
        'trigger': 'drdy'})
    raw = struct.pack('>7h', 4096, -4096, 2048, 0, 1000, -1000, 500)
    device_fields, values = _decode_over_i2c(fields, raw)

    pubs = _by_key(rb.plan_publications(device_fields))
    out = rb.fill_imu(pubs['imu'], values)
    assert out['linear_acceleration.x'] == pytest.approx(values['accel_x'])
    assert out['linear_acceleration.z'] == pytest.approx(values['accel_z'])
    assert out['angular_velocity.x'] == pytest.approx(values['gyro_x'])
    # Sanity against physics, not just self-consistency: +1 g on x.
    assert out['linear_acceleration.x'] == pytest.approx(9.80665, rel=1e-3)

    temp = rb.fill_temperature(pubs['temperature'], values)
    assert temp['temperature'] == pytest.approx(values['temp'] - 273.15)


def test_neo_m9n_navsat_equals_cli_decode(monkeypatch):
    monkeypatch.setattr(rb, 'navsat_status_from_fix_type',
                        lambda ft: rb.NAVSAT_STATUS_FIX)
    fields, compiled = _image_fields('neo_m9n')
    raw = bytearray(compiled.sample_size)
    raw[24] = 3                                     # fix_type: 3D
    struct.pack_into('<i', raw, 28, 85409900)       # lon 8.54099 deg
    struct.pack_into('<i', raw, 32, 473977400)      # lat 47.39774 deg
    struct.pack_into('<i', raw, 40, 408000)         # hMSL 408.0 m
    struct.pack_into('<I', raw, 44, 1500)           # hAcc 1.5 m
    struct.pack_into('<I', raw, 48, 2500)           # vAcc 2.5 m
    struct.pack_into('<i', raw, 52, 1000)           # velN 1.0 m/s
    struct.pack_into('<i', raw, 56, 2000)           # velE 2.0 m/s
    struct.pack_into('<i', raw, 60, 500)            # velD 0.5 m/s
    device_fields, values = _decode_over_i2c(fields, bytes(raw))

    pubs = _by_key(rb.plan_publications(device_fields))
    fix = rb.fill_navsat(pubs['fix'], values)
    assert fix['latitude'] == pytest.approx(math.degrees(values['latitude']))
    # The I2C window serves scale as float32, so the geodetic scale
    # quantizes; 1e-5 deg (~1 m) bounds that honestly.
    assert fix['latitude'] == pytest.approx(47.39774, abs=1e-5)
    assert fix['altitude'] == pytest.approx(values['altitude'])
    assert fix['position_covariance'][0] == \
        pytest.approx(values['pos_h_acc'] ** 2)
    assert fix['position_covariance'][8] == \
        pytest.approx(values['pos_v_acc'] ** 2)

    vel = rb.fill_twist(pubs['vel'], values)
    assert vel['twist.linear.x'] == pytest.approx(values['vel_east'])
    assert vel['twist.linear.y'] == pytest.approx(values['vel_north'])
    assert vel['twist.linear.z'] == pytest.approx(-values['vel_down'])


# ── Runtime (stub messages, fake runtime — no rclpy) ────────────────

def _ns(**kw):
    return types.SimpleNamespace(**kw)


def _header():
    return _ns(stamp=_ns(sec=0, nanosec=0), frame_id='')


def _vec3():
    return _ns(x=0.0, y=0.0, z=0.0)


class _StubImu:
    def __init__(self):
        self.header = _header()
        self.orientation_covariance = [0.0] * 9
        self.linear_acceleration = _vec3()
        self.linear_acceleration_covariance = [0.0] * 9
        self.angular_velocity = _vec3()
        self.angular_velocity_covariance = [0.0] * 9


class _StubTemperature:
    def __init__(self):
        self.header = _header()
        self.temperature = 0.0
        self.variance = 0.0


class _StubFloat64:
    def __init__(self):
        self.data = 0.0


class _StubString:
    def __init__(self):
        self.data = ''


_STUBS = {
    'sensor_msgs/msg/Imu': _StubImu,
    'sensor_msgs/msg/Temperature': _StubTemperature,
    'std_msgs/msg/Float64': _StubFloat64,
    'std_msgs/msg/String': _StubString,
}


def _stub_resolver(msg_type):
    try:
        return _STUBS[msg_type]
    except KeyError:
        raise ImportError(f"no stub for {msg_type}")


class _FakePublisher:
    def __init__(self, msg_cls, topic):
        self.msg_cls = msg_cls
        self.topic = topic
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _FakeNode:
    def __init__(self, name, namespace):
        self.name = name
        self.namespace = namespace


class _FakeRuntime:
    def __init__(self, now=(111, 222)):
        self.pubs = {}          # full topic -> _FakePublisher
        self.nodes = []         # _FakeNode per unit
        self.now = now
        self.down = False

    def create_node(self, name, namespace):
        node = _FakeNode(name, namespace)
        self.nodes.append(node)
        return node

    def create_publisher(self, node, msg_cls, topic):
        # Resolve the relative topic under the node namespace, as rclpy does.
        full = f"{node.namespace}/{topic}".lstrip('/')
        pub = _FakePublisher(msg_cls, full)
        self.pubs[full] = pub
        return pub

    def now_stamp(self):
        return self.now

    def shutdown(self):
        self.down = True


def _imu_bridge(runtime, units=('imu-mast', 'fl-knee'), **kw):
    plans = [rb.UnitPlan(name=u, frame_id=u,
                         publications=rb.plan_publications(IMU_FIELDS))
             for u in units]
    return rb.Ros2Bridge(plans, runtime=runtime, resolver=_stub_resolver,
                         **kw)


IMU_VALUES = {'accel_x': 0.1, 'accel_y': 0.2, 'accel_z': 9.81,
              'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
              'temp': 300.15}


def test_bridge_publishes_per_unit_namespaced_topics():
    runtime = _FakeRuntime()
    bridge = _imu_bridge(runtime, stamp_mode=rb.STAMP_DEVICE)
    # One ROS node per unit (name + namespace sanitized), so `ros2 node
    # list` shows each unit as its own sensor node.
    assert [(n.name, n.namespace) for n in runtime.nodes] == [
        ('imu_mast', '/nxs/imu_mast'), ('fl_knee', '/nxs/fl_knee')]
    assert set(runtime.pubs) == {
        'nxs/imu_mast/imu', 'nxs/imu_mast/temperature',
        'nxs/fl_knee/imu', 'nxs/fl_knee/temperature'}

    n = bridge.publish(0, Sample(count=1, raw=b'', values=IMU_VALUES,
                                 timestamp_us=2_500_000))
    assert n == 2
    imu = runtime.pubs['nxs/imu_mast/imu'].published[0]
    assert imu.header.frame_id == 'imu_mast'
    assert imu.header.stamp.sec == 2
    assert imu.header.stamp.nanosec == 500_000_000
    assert imu.linear_acceleration.z == pytest.approx(9.81)
    temp = runtime.pubs['nxs/imu_mast/temperature'].published[0]
    assert temp.temperature == pytest.approx(27.0)
    # The other unit saw nothing.
    assert runtime.pubs['nxs/fl_knee/imu'].published == []


def test_bridge_flat_topics_for_adhoc_device():
    runtime = _FakeRuntime()
    plans = [rb.UnitPlan(name=None, frame_id='iam20680',
                         publications=rb.plan_publications(IMU_FIELDS))]
    rb.Ros2Bridge(plans, runtime=runtime, resolver=_stub_resolver)
    assert set(runtime.pubs) == {'nxs/imu', 'nxs/temperature'}


def test_bridge_suppressed_fill_publishes_nothing():
    runtime = _FakeRuntime()
    bridge = _imu_bridge(runtime, units=('one',))
    n = bridge.publish(0, Sample(count=1, raw=b'',
                                 values={'accel_x': 0.1}))
    # accel triad incomplete -> Imu suppressed; temp absent -> suppressed.
    assert n == 0
    assert runtime.pubs['nxs/one/imu'].published == []


def test_bridge_stamp_modes(caplog):
    runtime = _FakeRuntime(now=(555, 666))
    bridge = _imu_bridge(runtime, units=('one',), stamp_mode=rb.STAMP_DEVICE)
    with caplog.at_level(logging.WARNING, logger='nxs.ros2_bridge'):
        bridge.publish(0, Sample(count=1, raw=b'', values=IMU_VALUES,
                                 timestamp_us=None))
        bridge.publish(0, Sample(count=2, raw=b'', values=IMU_VALUES,
                                 timestamp_us=None))
    imu = runtime.pubs['nxs/one/imu'].published[0]
    assert (imu.header.stamp.sec, imu.header.stamp.nanosec) == (555, 666)
    # The no-wire-timestamp fallback warns once, not per sample.
    warnings = [r for r in caplog.records if 'no wire timestamp' in r.message]
    assert len(warnings) == 1

    arrival = rb.Ros2Bridge(
        [rb.UnitPlan('one', 'one', rb.plan_publications(IMU_FIELDS))],
        runtime=_FakeRuntime(now=(9, 9)), resolver=_stub_resolver,
        stamp_mode=rb.STAMP_ARRIVAL)
    arrival.publish(0, Sample(count=1, raw=b'', values=IMU_VALUES,
                              timestamp_us=2_500_000))


class _FakeTimeSync:
    def __init__(self, stamp=(777, 888), bound=None):
        self._stamp = stamp
        self._bound = bound
        self.projected = []

    def project_to_realtime(self, device_us):
        self.projected.append(device_us)
        return self._stamp

    def bound_us(self):
        return self._bound


def test_bridge_synced_mode_projects_device_time():
    runtime = _FakeRuntime(now=(111, 222))
    sync = _FakeTimeSync(stamp=(777, 888))
    plans = [rb.UnitPlan('one', 'one', rb.plan_publications(IMU_FIELDS))]
    bridge = rb.Ros2Bridge(plans, runtime=runtime, resolver=_stub_resolver,
                           time_syncs=[sync])  # stamp_mode default: synced
    bridge.publish(0, Sample(count=1, raw=b'', values=IMU_VALUES,
                             timestamp_us=2_500_000))
    imu = runtime.pubs['nxs/one/imu'].published[0]
    assert (imu.header.stamp.sec, imu.header.stamp.nanosec) == (777, 888)
    assert sync.projected == [2_500_000]


def test_bridge_synced_mode_falls_back_before_first_observation(caplog):
    runtime = _FakeRuntime(now=(111, 222))
    sync = _FakeTimeSync(stamp=None)  # estimator not ready
    plans = [rb.UnitPlan('one', 'one', rb.plan_publications(IMU_FIELDS))]
    bridge = rb.Ros2Bridge(plans, runtime=runtime, resolver=_stub_resolver,
                           time_syncs=[sync])
    with caplog.at_level(logging.WARNING, logger='nxs.ros2_bridge'):
        bridge.publish(0, Sample(count=1, raw=b'', values=IMU_VALUES,
                                 timestamp_us=2_500_000))
    imu = runtime.pubs['nxs/one/imu'].published[0]
    assert (imu.header.stamp.sec, imu.header.stamp.nanosec) == (111, 222)
    assert any('no time-sync observation' in r.message
               for r in caplog.records)


def test_bridge_arrival_mode_ignores_wire_stamp():
    runtime = _FakeRuntime(now=(9, 9))
    bridge = _imu_bridge(runtime, units=('one',),
                         stamp_mode=rb.STAMP_ARRIVAL)
    bridge.publish(0, Sample(count=1, raw=b'', values=IMU_VALUES,
                             timestamp_us=2_500_000))
    imu = runtime.pubs['nxs/one/imu'].published[0]
    assert (imu.header.stamp.sec, imu.header.stamp.nanosec) == (9, 9)


def test_bridge_unresolvable_message_drops_publication():
    runtime = _FakeRuntime()

    def no_imu(msg_type):
        if msg_type.endswith('Imu'):
            raise ImportError('sensor_msgs absent')
        return _stub_resolver(msg_type)

    plans = [rb.UnitPlan('one', 'one', rb.plan_publications(IMU_FIELDS))]
    rb.Ros2Bridge(plans, runtime=runtime, resolver=no_imu)
    assert set(runtime.pubs) == {'nxs/one/temperature'}

    with pytest.raises(SystemExit, match='no publishable topics'):
        rb.Ros2Bridge(plans, runtime=_FakeRuntime(),
                      resolver=lambda t: (_ for _ in ()).throw(ImportError(t)))


def test_resolve_msg_type_and_set_nested_attr():
    assert rb.resolve_msg_type('collections/abc/Sequence') is not None
    obj = _ns(a=_ns(b=_ns(c=0)))
    rb.set_nested_attr(obj, 'a.b.c', 7)
    assert obj.a.b.c == 7


# ── run_bridge fan-in ───────────────────────────────────────────────

class _FakeClient:
    def __init__(self, samples, fields=None, driver='iam20680',
                 up=True, fail_open=False):
        self._samples = samples
        self._fields = IMU_FIELDS if fields is None else fields
        self._driver = driver
        self._up = up
        self._time_sync = _FakeTimeSync()
        self.started = None
        self.stopped = False
        self.closed = False
        self.pings = 0

    def get_time_sync(self):
        return self._time_sync

    def time_sync_ping(self):
        self.pings += 1
        return False

    def probe(self):
        return self._up

    def read_driver_name(self):
        return self._driver

    def read_outputs(self):
        return self._fields

    def start_stream(self, every_nth=1):
        self.started = every_nth

    def stop_stream(self):
        self.stopped = True

    def iter_samples(self, timeout=1.0):
        yield from self._samples

    def close(self):
        self.closed = True


def _samples(n, unit_tag=0):
    return [Sample(count=i, raw=b'', values=IMU_VALUES,
                   timestamp_us=1_000_000 * (i + 1) + unit_tag)
            for i in range(n)]


def test_run_bridge_fans_in_all_units_and_stops_streams():
    runtime = _FakeRuntime()
    bridge = _imu_bridge(runtime)
    clients = [_FakeClient(_samples(3)), _FakeClient(_samples(2))]
    total = rb.run_bridge(clients, bridge)
    assert total == 5
    assert len(runtime.pubs['nxs/imu_mast/imu'].published) == 3
    assert len(runtime.pubs['nxs/fl_knee/imu'].published) == 2
    assert all(c.stopped for c in clients)


def test_acquire_run_lock_is_reentrant_in_process_and_exclusive_across():
    fd = rb.acquire_run_lock(["tok:a", "tok:b"])
    # Same process, same key (any order) — the held lock, not a refusal.
    assert rb.acquire_run_lock(["tok:b", "tok:a"]) == fd
    # Another process must be refused while this one holds the lock.
    pkg_root = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))
    probe = ("from nxs.ros2_bridge import acquire_run_lock; "
             "acquire_run_lock(['tok:a', 'tok:b'])")
    r = subprocess.run([_sys.executable, "-c", probe],
                       capture_output=True, text=True, cwd=pkg_root)
    assert r.returncode != 0
    assert "already serving" in r.stderr


class _FlakyClient(_FakeClient):
    """First iter_samples call dies mid-stream (a unit rebooting under
    the poll); the next call streams normally. The reader must retry
    after the error, never die with it."""

    def __init__(self, samples):
        super().__init__(samples)
        self._calls = 0

    def iter_samples(self, timeout=1.0):
        self._calls += 1
        if self._calls == 1:
            raise OSError(121, "Remote I/O error")
        yield from self._samples


def test_run_bridge_prints_sync_bound_once_available(capsys):
    runtime = _FakeRuntime()
    bridge = _imu_bridge(runtime, units=('one',))
    client = _FakeClient(_samples(3))
    client._time_sync = _FakeTimeSync(bound=97.0)
    rb.run_bridge([client], bridge, count=3)
    out = capsys.readouterr().out
    assert out.count("time sync") == 1
    assert "\u00b10.10 ms" in out


def test_run_bridge_reader_retries_after_stream_error(monkeypatch):
    monkeypatch.setattr(rb, "READ_RETRY_BACKOFF_S", 0.05)
    runtime = _FakeRuntime()
    bridge = _imu_bridge(runtime, units=('one',))
    client = _FlakyClient(_samples(3))
    total = rb.run_bridge([client], bridge, count=3)
    assert total == 3
    assert client._calls == 2


def test_run_bridge_respects_count():
    runtime = _FakeRuntime()
    bridge = _imu_bridge(runtime, units=('one',))
    total = rb.run_bridge([_FakeClient(_samples(50))], bridge, count=7)
    assert total == 7


def test_run_bridge_count_includes_suppressed_samples():
    # `count` paces on samples bridged, not messages published — a
    # sample whose fillers all suppress still counts (stream --count
    # semantics).
    runtime = _FakeRuntime()
    bridge = _imu_bridge(runtime, units=('one',))
    empty = [Sample(count=i, raw=b'', values={}) for i in range(5)]
    total = rb.run_bridge([_FakeClient(empty)], bridge, count=3)
    assert total == 3
    assert runtime.pubs['nxs/one/imu'].published == []


# ── The CLI verb ────────────────────────────────────────────────────

MANIFEST = """\
units:
  - name: imu-mast
    module: nxs
    links: [{transport: i2c, bus: /dev/i2c-9, address: 0x30}]
    sensors: []
  - name: fl-knee
    module: nxs
    links: [{transport: i2c, bus: /dev/i2c-9, address: 0x31}]
    sensors: []
"""


def _install_manifest(tmp_path, monkeypatch, text=MANIFEST):
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    (tmp_path / 'aliensense').mkdir(exist_ok=True)
    (tmp_path / 'aliensense' / 'suite.yaml').write_text(text)


def _args(**kw):
    base = dict(unit=None, transport='i2c', bus='/dev/i2c-9', addr=0x30,
                port=None, baud=460800, remote_node_id=None,
                hz=None, count=None, topic_base='nxs', frame_id=None,
                stamp='synced', map=None, plan=False, launch_file=False)
    base.update(kw)
    return argparse.Namespace(**base)


def test_ros2_lock_tokens_key_adhoc_on_device_flags():
    # An ad-hoc target resolves to (None, None, None); its lock token
    # must come from the device flags so two runs against different
    # devices never collide, and two runs against the same one do.
    from nxs.cli import _ros2_lock_tokens
    adhoc = [(None, None, None)]
    a = _ros2_lock_tokens(_args(port='/dev/ttyUSB0'), adhoc)
    b = _ros2_lock_tokens(_args(port='/dev/ttyUSB1'), adhoc)
    c = _ros2_lock_tokens(_args(port='/dev/ttyUSB0'), adhoc)
    assert a != b
    assert a == c
    # Suite targets keep keying on their resolved link kwargs.
    suite = [('u', 'i2c', {'bus': '/dev/i2c-9', 'address': 0x30})]
    assert _ros2_lock_tokens(_args(), suite) == \
        _ros2_lock_tokens(_args(port='/dev/ttyUSB1'), suite)


def test_run_bridge_restores_switch_interval():
    import sys
    from nxs.ros2_bridge import run_bridge
    prev = sys.getswitchinterval()
    run_bridge([], object(), count=0)
    assert sys.getswitchinterval() == prev


def test_device_flags_given():
    from nxs.cli import _device_flags_given
    assert _device_flags_given(['-t', 'i2c', 'ros2'])
    assert _device_flags_given(['--port=/dev/ttyUSB0', 'ros2'])
    assert not _device_flags_given(['ros2', '--plan'])


def test_allocate_can_local_ids_on_shared_iface():
    from nxs.cli import _allocate_can_local_ids
    targets = [
        ('a', 'cyphal-can', {'can_iface': 'can0', 'remote_node_id': 10}),
        ('b', 'cyphal-can', {'can_iface': 'can0', 'remote_node_id': 11}),
        ('c', 'cyphal-can', {'can_iface': 'can1', 'remote_node_id': 10}),
        ('d', 'i2c', {'bus': '/dev/i2c-9', 'address': 0x30}),
    ]
    _allocate_can_local_ids(targets)
    # First client on each iface keeps the stock host ID (no override).
    assert 'local_node_id' not in targets[0][2]
    assert 'local_node_id' not in targets[2][2]
    assert 'local_node_id' not in targets[3][2]
    # The second client on can0 gets its own, below HOST_NODE_ID.
    from nxs._generated_constants import CyphalDefaults
    assert targets[1][2]['local_node_id'] == CyphalDefaults.HOST_NODE_ID - 1


def test_allocate_can_local_ids_skips_manifest_ids():
    from nxs._generated_constants import CyphalDefaults
    from nxs.cli import _allocate_can_local_ids
    taken = CyphalDefaults.HOST_NODE_ID - 1
    targets = [
        ('a', 'cyphal-can', {'can_iface': 'can0', 'remote_node_id': 10}),
        ('b', 'cyphal-can', {'can_iface': 'can0', 'remote_node_id': taken}),
    ]
    _allocate_can_local_ids(targets)
    assert targets[1][2]['local_node_id'] == taken - 1


def test_ros2_targets_modes(tmp_path, monkeypatch):
    from nxs.cli import _ros2_targets
    _install_manifest(tmp_path, monkeypatch)

    suite_mode, targets = _ros2_targets(_args(), argv=['ros2'])
    assert suite_mode
    assert [t[0] for t in targets] == ['imu-mast', 'fl-knee']
    assert targets[0][1] == 'i2c'
    assert targets[0][2] == {'bus': '/dev/i2c-9', 'address': 0x30}

    suite_mode, targets = _ros2_targets(_args(unit='fl-knee'),
                                        argv=['ros2'])
    assert not suite_mode
    assert targets == [('fl-knee', 'i2c',
                        {'bus': '/dev/i2c-9', 'address': 0x31})]

    # Explicit device flags select ad-hoc mode despite the manifest.
    suite_mode, targets = _ros2_targets(_args(),
                                        argv=['-t', 'i2c', 'ros2'])
    assert not suite_mode
    assert targets == [(None, None, None)]


def test_cmd_ros2_launch_file_prints_existing_path(capsys):
    import os
    from nxs.cli import cmd_ros2
    rc = cmd_ros2(_args(launch_file=True), argv=['ros2', '--launch-file'])
    assert rc == 0
    path = capsys.readouterr().out.strip()
    assert path.endswith('ros2/bridge.launch.py')
    assert os.path.isfile(path)  # shipped in the wheel via package-data


def test_launch_file_is_valid_python():
    import os
    import py_compile
    from nxs import cli
    launch = os.path.join(os.path.dirname(cli.__file__), 'ros2',
                          'bridge.launch.py')
    py_compile.compile(launch, doraise=True)


def test_launch_description_builds():
    # generate_launch_description() needs the ROS launch packages; skips
    # where no ROS 2 environment is sourced (CI has none).
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    import importlib.util
    import os
    from nxs import cli
    path = os.path.join(os.path.dirname(cli.__file__), 'ros2',
                        'bridge.launch.py')
    spec = importlib.util.spec_from_file_location('nxs_bridge_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    from launch import LaunchDescription
    assert isinstance(mod.generate_launch_description(), LaunchDescription)


def test_build_viz_rviz_config_per_unit_displays():
    # The shipped base config carries no Imu display (unit names only
    # exist in the manifest); the generator appends one per given unit,
    # wired to the per-unit topic the bridge actually publishes.
    import os

    import yaml

    from nxs.ros2_bridge import build_viz_rviz_config
    path = build_viz_rviz_config(
        [('imu-mast', 'imu_mast'), ('imu-tail', 'imu_tail')], 'nxs')
    try:
        with open(path) as f:
            cfg = yaml.safe_load(f)
    finally:
        os.unlink(path)
    displays = cfg['Visualization Manager']['Displays']
    classes = [d['Class'] for d in displays]
    assert 'rviz_default_plugins/Grid' in classes
    assert 'rviz_default_plugins/TF' in classes
    imus = [d for d in displays if d['Class'] == 'rviz_imu_plugin/Imu']
    assert [d['Topic']['Value'] for d in imus] == [
        '/nxs/imu_mast/imu', '/nxs/imu_tail/imu']
    assert all(d['Acceleration properties']['Enable acceleration']
               for d in imus)


def test_sensor_plans_imu_by_compiled_semantics():
    # The filter compiles the shipped driver offline and asks the same
    # planner the bridge uses — inertial drivers plan an imu topic, the
    # GNSS driver does not, and an unresolvable driver fails open.
    from nxs.ros2_bridge import sensor_plans_imu
    assert sensor_plans_imu('iam20680', {'sample_rate': 200}) is True
    assert sensor_plans_imu('fxos8700',
                            {'bus': 1, 'sample_rate': 200,
                             'accel_fs': 4}) is True
    assert sensor_plans_imu('neo_m9n') is False
    assert sensor_plans_imu('no_such_driver') is True


def test_build_viz_rviz_config_cleans_up_at_exit(monkeypatch):
    # The generated config is temporary: the generator registers an
    # exit hook that removes it, so repeated viz launches do not leak.
    import os

    import nxs.ros2_bridge as rb
    registered = []
    monkeypatch.setattr(rb.atexit, 'register',
                        lambda fn, *a: registered.append((fn, a)))
    path = rb.build_viz_rviz_config([('u-1', 'u_1')], 'nxs')
    assert os.path.exists(path)
    assert registered
    fn, a = registered[-1]
    fn(*a)
    assert not os.path.exists(path)


def test_launch_viz_actions_build_rviz_and_transforms(monkeypatch):
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    import importlib.util
    import os
    from types import SimpleNamespace
    from nxs import cli
    path = os.path.join(os.path.dirname(cli.__file__), 'ros2',
                        'bridge.launch.py')
    spec = importlib.util.spec_from_file_location('nxs_bridge_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    imu_spec = [SimpleNamespace(driver='imu_drv', config={})]
    gnss_spec = [SimpleNamespace(driver='gnss_drv', config={})]
    monkeypatch.setattr(mod, '_manifest_units',
                        lambda: [('unit-a', 'unit_a', imu_spec),
                                 ('unit-b', 'unit_b', gnss_spec)])
    import nxs.ros2_bridge as rb
    monkeypatch.setattr(rb, 'sensor_plans_imu',
                        lambda driver, config=None: driver == 'imu_drv')
    seen = {}

    def fake_config(units, topic_base):
        seen['units'] = units
        return '/tmp/fake.rviz'

    monkeypatch.setattr(rb, 'build_viz_rviz_config', fake_config)
    from launch import LaunchContext
    ctx = LaunchContext()
    ctx.launch_configurations.update({'viz': 'true', 'topic_base': 'nxs'})
    actions = mod._viz_actions(ctx)
    # RViz plus one static transform per unit — but Imu displays only
    # for the unit whose driver plans an imu topic.
    assert len(actions) == 3
    assert seen['units'] == [('unit-a', 'unit_a')]

    ctx_off = LaunchContext()
    ctx_off.launch_configurations.update({'viz': 'false',
                                          'topic_base': 'nxs'})
    assert mod._viz_actions(ctx_off) == []


def test_cmd_ros2_plan_prints_topics_without_ros(tmp_path, monkeypatch,
                                                 capsys):
    from nxs.cli import cmd_ros2
    _install_manifest(tmp_path, monkeypatch)
    clients = {0x30: _FakeClient([]), 0x31: _FakeClient([])}

    def opener(transport, **kw):
        return clients[kw['address']]

    rc = cmd_ros2(_args(plan=True), opener=opener, argv=['ros2', '--plan'])
    assert rc == 0
    out = capsys.readouterr().out
    assert 'nxs/imu_mast/imu' in out
    assert 'nxs/fl_knee/temperature' in out
    # The unit header previews the sanitized name the runtime uses, not
    # the raw dashed role.
    assert 'imu_mast:' in out and 'imu-mast:' not in out
    assert all(c.closed for c in clients.values())


def test_cmd_ros2_clean_error_without_rclpy(tmp_path, monkeypatch):
    from nxs.cli import cmd_ros2
    _install_manifest(tmp_path, monkeypatch)
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    client = _FakeClient([])
    with pytest.raises(SystemExit, match='sourced ROS 2 environment'):
        cmd_ros2(_args(unit='imu-mast'),
                 opener=lambda transport, **kw: client, argv=['ros2'])
    assert client.closed


def test_cmd_ros2_requires_descriptors(tmp_path, monkeypatch):
    from nxs.cli import cmd_ros2
    _install_manifest(tmp_path, monkeypatch)
    client = _FakeClient([], fields=[])
    with pytest.raises(SystemExit, match='upload and run a driver'):
        cmd_ros2(_args(unit='imu-mast'),
                 opener=lambda transport, **kw: client, argv=['ros2'])


def test_cmd_ros2_suite_skips_down_unit(tmp_path, monkeypatch, capsys):
    from nxs.cli import cmd_ros2
    _install_manifest(tmp_path, monkeypatch)
    good = _FakeClient([])

    def opener(transport, **kw):
        if kw['address'] == 0x30:
            raise OSError('bus fell over')
        return good

    rc = cmd_ros2(_args(plan=True), opener=opener, argv=['ros2', '--plan'])
    assert rc == 0
    captured = capsys.readouterr()
    assert 'imu-mast: bus fell over — skipped' in captured.err
    assert 'nxs/fl_knee/imu' in captured.out
    assert 'imu_mast/imu' not in captured.out


def test_cmd_ros2_streams_publishes_and_stops(tmp_path, monkeypatch,
                                              capsys):
    from nxs.cli import cmd_ros2
    _install_manifest(tmp_path, monkeypatch)
    clients = {0x30: _FakeClient(_samples(3)),
               0x31: _FakeClient(_samples(2))}

    created = []

    class _RecordingBridge:
        def __init__(self, plans, **kw):
            self.plans = plans
            self.kw = kw
            self.published = []
            self.down = False
            created.append(self)

        def publish(self, unit_idx, sample):
            self.published.append((unit_idx, sample.count))
            return 1

        def unit_frame(self, unit_idx):
            return self.plans[unit_idx].frame_id

        def shutdown(self):
            self.down = True

    monkeypatch.setattr('nxs.ros2_bridge.Ros2Bridge', _RecordingBridge)
    rc = cmd_ros2(_args(), opener=lambda transport, **kw: clients[kw['address']],
                  argv=['ros2'])
    assert rc == 0
    bridge = created[0]
    assert bridge.kw['topic_base'] == 'nxs'
    assert bridge.kw['stamp_mode'] == 'synced'  # the parser default
    assert len(bridge.kw['time_syncs']) == 2
    assert len(bridge.published) == 5
    assert all(c.pings >= 1 for c in clients.values())  # warm-start ping
    assert bridge.down
    assert all(c.stopped and c.closed for c in clients.values())
    out = capsys.readouterr().out
    assert 'publishing nxs/imu_mast/imu' in out
    assert '5 sample(s) bridged' in out


def test_cmd_ros2_map_replaces_auto_plan(tmp_path, monkeypatch, capsys):
    from nxs.cli import cmd_ros2
    _install_manifest(tmp_path, monkeypatch)
    map_file = tmp_path / 'map.yaml'
    map_file.write_text(
        "- message: std_msgs/msg/Float64\n"
        "  topic: custom\n"
        "  mapping: {data: accel_z}\n")
    client = _FakeClient([])
    rc = cmd_ros2(_args(unit='imu-mast', map=str(map_file), plan=True),
                  opener=lambda transport, **kw: client, argv=['ros2'])
    assert rc == 0
    out = capsys.readouterr().out
    assert 'nxs/imu_mast/custom' in out
    assert 'nxs/imu_mast/imu' not in out


def test_load_map_rejects_incomplete_blocks(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text("- topic: custom\n")
    with pytest.raises(SystemExit, match="block 0 needs 'message'"):
        rb.load_map(str(bad))
    # A block without a mapping could only ever publish default-valued
    # messages — refused at load, not silently published.
    for block in ("- {message: std_msgs/msg/Float64, topic: t}\n",
                  "- {message: std_msgs/msg/Float64, topic: t, mapping: {}}\n",
                  "- message: sensor_msgs/msg/Imu\n"
                  "  topic: imu\n"
                  "  constants: {orientation_covariance: [1,0,0,0,0,0,0,0,0]}\n"):
        bad.write_text(block)
        with pytest.raises(SystemExit, match="non-empty 'mapping'"):
            rb.load_map(str(bad))


def test_fill_mapped_applies_mapping_and_constants():
    pub = rb.Publication(
        key='mapped:0', kind='mapped', msg_type='sensor_msgs/msg/Imu',
        topic='imu', bindings={},
        extras={'mapping': {'linear_acceleration.z': 'accel_z'},
                'constants': {'linear_acceleration_covariance':
                              [0.01] + [0.0] * 8}})
    out = rb.fill_mapped(pub, {'accel_z': 9.81, 'ignored': 1.0})
    assert out['linear_acceleration.z'] == pytest.approx(9.81)
    assert out['linear_acceleration_covariance'][0] == pytest.approx(0.01)
    # No mapped field in the sample: suppress — never a default-valued
    # message, and constants alone must not resurrect it.
    assert rb.fill_mapped(pub, {'ignored': 1.0}) is None


def test_fill_mapped_passes_string_fields_through():
    pub = rb.Publication(
        key='mapped:0', kind='mapped', msg_type='std_msgs/msg/String',
        topic='nmea', bindings={},
        extras={'mapping': {'data': 'nmea'}, 'constants': {}})
    out = rb.fill_mapped(pub, {'nmea': '$GPGGA,x*47'})
    assert out == {'data': '$GPGGA,x*47'}


# ── Real rclpy smoke (skips without a sourced ROS 2 environment) ────

def test_real_rclpy_smoke():
    pytest.importorskip('rclpy')
    sensor_msgs = pytest.importorskip('sensor_msgs.msg')

    # The hardcoded wire constants must match the real message classes.
    assert rb.NAVSAT_STATUS_NO_FIX == sensor_msgs.NavSatStatus.STATUS_NO_FIX
    assert rb.NAVSAT_STATUS_FIX == sensor_msgs.NavSatStatus.STATUS_FIX
    assert rb.COVARIANCE_TYPE_UNKNOWN == \
        sensor_msgs.NavSatFix.COVARIANCE_TYPE_UNKNOWN
    assert rb.COVARIANCE_TYPE_DIAGONAL_KNOWN == \
        sensor_msgs.NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    plans = [rb.UnitPlan('smoke', 'smoke',
                         rb.plan_publications(IMU_FIELDS))]
    bridge = rb.Ros2Bridge(plans)
    try:
        n = bridge.publish(0, Sample(count=1, raw=b'', values=IMU_VALUES,
                                     timestamp_us=1_000_000))
        assert n == 2
    finally:
        bridge.shutdown()


def test_utc_from_itow_resolves_week_and_leap():
    # Mid-week: host sits 1000 s after the epoch described by itow.
    now = rb.GpsTime.GPS_EPOCH_UNIX_S - rb.GpsTime.GPS_UTC_LEAP_S \
        + 2400 * rb.GpsTime.GPS_WEEK_S + 300_000 + 1000
    utc = rb.utc_from_itow(300_000.0, now)
    assert utc == now - 1000

    # Week rollover: itow restarted near zero while the host clock still
    # sits at the tail of the previous week — the next week wins.
    now = rb.GpsTime.GPS_EPOCH_UNIX_S - rb.GpsTime.GPS_UTC_LEAP_S \
        + 2400 * rb.GpsTime.GPS_WEEK_S + rb.GpsTime.GPS_WEEK_S - 30
    utc = rb.utc_from_itow(5.0, now)
    assert utc == now + 35


def test_utc_from_itow_refuses_implausible_host_clock():
    # A cold host (clock near the Unix epoch) cannot resolve the GPS
    # week — refuse rather than confidently emit a wrong one.
    assert rb.utc_from_itow(300_000.0, 1_000_000.0) is None
    assert rb.utc_from_itow(300_000.0,
                            rb.HOST_CLOCK_FLOOR_UNIX_S - 1) is None


def test_epoch_binding_requires_both_semantics():
    tow = _f('itow', Sem.TIME_OF_WEEK, 'uint32')
    fix = _f('fix_type', Sem.FIX_TYPE, 'uint8')
    other = _f('pressure', Sem.PRESSURE, 'uint32')
    assert rb.epoch_binding([tow, fix, other]) == ('itow', 'fix_type')
    assert rb.epoch_binding([tow, other]) is None
    assert rb.epoch_binding([other]) is None


# ── itow stamp mode, composed ─────────────────────────────

# The stamp path is per unit, not per message, so these ride an IMU unit
# (header-bearing and stubbed) carrying the two epoch fields.
EPOCH_FIELDS = IMU_FIELDS + [_f('itow', Sem.TIME_OF_WEEK, 'uint32'),
                             _f('fix_type', Sem.FIX_TYPE, 'uint8')]


def _epoch_bridge(runtime, **kw):
    plan = rb.UnitPlan(name='gnss', frame_id='gnss',
                       publications=rb.plan_publications(EPOCH_FIELDS),
                       epoch=rb.epoch_binding(EPOCH_FIELDS))
    return rb.Ros2Bridge([plan], runtime=runtime, resolver=_stub_resolver,
                         **kw)


def _epoch_sample(itow_s, fix_type, count=1):
    values = dict(IMU_VALUES, itow=itow_s, fix_type=fix_type)
    return Sample(count=count, raw=b'', values=values,
                  timestamp_us=2_500_000)


def test_itow_mode_stamps_from_the_receiver_epoch():
    # A time-solved fix stamps with the resolver's UTC — not the sync
    # projection, not arrival.
    runtime = _FakeRuntime(now=(9, 9))
    sync = _FakeTimeSync(stamp=(777, 888))
    bridge = _epoch_bridge(runtime, stamp_mode=rb.STAMP_ITOW,
                           time_syncs=[sync])
    itow_s = 300_000.0
    bridge.publish(0, _epoch_sample(itow_s, 3))
    imu = runtime.pubs['nxs/gnss/imu'].published[0]
    expected = rb.utc_from_itow(itow_s, time.time())
    assert imu.header.stamp.sec == int(expected)
    assert (imu.header.stamp.sec, imu.header.stamp.nanosec) != (777, 888)
    assert (imu.header.stamp.sec, imu.header.stamp.nanosec) != (9, 9)


def test_itow_mode_falls_back_to_synced_once_without_a_time_fix(caplog):
    # fix_type 1 is dead-reckoning only: no time solution, so the mode
    # degrades to the synced projection and says so exactly once.
    runtime = _FakeRuntime(now=(9, 9))
    sync = _FakeTimeSync(stamp=(777, 888))
    bridge = _epoch_bridge(runtime, stamp_mode=rb.STAMP_ITOW,
                           time_syncs=[sync])
    with caplog.at_level(logging.WARNING, logger='nxs.ros2_bridge'):
        bridge.publish(0, _epoch_sample(300_000.0, 1, count=1))
        bridge.publish(0, _epoch_sample(300_001.0, 1, count=2))
    published = runtime.pubs['nxs/gnss/imu'].published
    assert len(published) == 2
    for msg in published:
        assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (777, 888)
    warnings = [r for r in caplog.records
                if 'no valid GNSS epoch' in r.message]
    assert len(warnings) == 1


def test_format_plan_marks_epoch_capable_units():
    epoch_plan = rb.UnitPlan('gnss', 'gnss',
                             rb.plan_publications(EPOCH_FIELDS),
                             epoch=rb.epoch_binding(EPOCH_FIELDS))
    imu_plan = rb.UnitPlan('imu', 'imu', rb.plan_publications(IMU_FIELDS),
                           epoch=rb.epoch_binding(IMU_FIELDS))
    out = rb.format_plan([epoch_plan, imu_plan], 'nxs')
    assert 'gnss:  [epoch-capable]' in out
    assert 'imu:' in out
    assert 'imu:  [epoch-capable]' not in out
