# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""What a field's semantic publishes as: the topic plan a personality's outputs produce, and the message each publication fills."""

import importlib
import logging
import math
import re
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import yaml

from nxs._generated_constants import FieldSemantics, GpsTime

log = logging.getLogger(__name__)


Sem = FieldSemantics.FieldSemantic

# Frozen sensor_msgs wire constants, hardcoded so the planning core never
# imports ROS packages (the rclpy smoke test cross-checks them).
NAVSAT_STATUS_NO_FIX = -1

NAVSAT_STATUS_FIX = 0

NAVSAT_SERVICE_ALL = 0b1111  # GPS|GLONASS|COMPASS|GALILEO; the wire names no constellation

COVARIANCE_TYPE_UNKNOWN = 0

COVARIANCE_TYPE_DIAGONAL_KNOWN = 2

# Variance for a missing accuracy estimate: the device's own sentinel for
# "unknown" on its geodetic subject.
UNKNOWN_VARIANCE = 1.0e6

ACCEL_AXES = (Sem.ACCEL_X, Sem.ACCEL_Y, Sem.ACCEL_Z)

GYRO_AXES = (Sem.GYRO_X, Sem.GYRO_Y, Sem.GYRO_Z)

MAG_AXES = (Sem.MAG_X, Sem.MAG_Y, Sem.MAG_Z)

GEO_POSITION = (Sem.LATITUDE, Sem.LONGITUDE, Sem.ALTITUDE)

GEO_VELOCITY = (Sem.VEL_NORTH, Sem.VEL_EAST, Sem.VEL_DOWN)

def resolve_msg_type(type_string: str):
    """Resolve a ROS 2 message type string ("sensor_msgs/msg/Imu") to its class."""
    parts = type_string.replace("/", ".")
    module_path, _, class_name = parts.rpartition(".")
    module = importlib.import_module(module_path)
    return getattr(module, class_name)

def set_nested_attr(obj, path: str, value):
    """Set a nested attribute by dotted path ("linear_acceleration.x")."""
    parts = path.split(".")
    for part in parts[:-1]:
        obj = getattr(obj, part)
    setattr(obj, parts[-1], value)

@dataclass(frozen=True)
class Publication:
    """One planned ROS publisher: which message, on which relative topic,
    fed by which sample fields (`bindings` maps a filler role to a field
    name; `extras` carries the --map mapping/constants payload)."""
    key: str
    kind: str
    msg_type: str
    topic: str
    bindings: Dict[str, str]
    extras: dict = field(default_factory=dict)

@dataclass
class UnitPlan:
    """One bridged unit: its manifest name (None for an ad-hoc device),
    the frame_id its messages carry, its planned publications, and the
    epoch-capable binding (`--stamp itow`'s field pair) when present."""
    name: Optional[str]
    frame_id: str
    publications: List[Publication]
    epoch: Optional[Tuple[str, str]] = None

def fields_by_semantic(fields: List[dict]) -> Dict[int, dict]:
    """First field claiming each non-generic semantic (strings excluded); the
    one resolver behind publication planning and epoch binding."""
    by_sem: Dict[int, dict] = {}
    for f in fields:
        sem = int(f.get('semantic', Sem.GENERIC))
        if sem != Sem.GENERIC and sem not in by_sem \
                and f.get('type') != 'string':
            by_sem[sem] = f

    return by_sem

def epoch_binding(fields: List[dict]) -> Optional[Tuple[str, str]]:
    """(time-of-week, fix-type) field names when the descriptors carry both
    registry semantics; the epoch-capable contract `--stamp itow` keys on."""
    by_sem = fields_by_semantic(fields)
    tow = by_sem.get(Sem.TIME_OF_WEEK)
    fix = by_sem.get(Sem.FIX_TYPE)
    if tow is None or fix is None:
        return None

    return tow['name'], fix['name']

def plan_publications(fields: List[dict]) -> List[Publication]:
    """Group device-served descriptors into ROS publications by semantic: vector
    groups and geodetic triples form their standard message only when complete,
    every other field lands on its own per-field topic. Nothing is dropped."""
    by_sem = fields_by_semantic(fields)

    consumed = set()
    pubs = []

    def claim(sems) -> List[str]:
        names = [by_sem[s]['name'] for s in sems]
        consumed.update(names)
        return names

    accel_ok = all(s in by_sem for s in ACCEL_AXES)
    gyro_ok = all(s in by_sem for s in GYRO_AXES)
    if accel_ok or gyro_ok:
        bindings = {}
        if accel_ok:
            bindings.update(zip(('accel_x', 'accel_y', 'accel_z'),
                                claim(ACCEL_AXES)))
        if gyro_ok:
            bindings.update(zip(('gyro_x', 'gyro_y', 'gyro_z'),
                                claim(GYRO_AXES)))
        pubs.append(Publication('imu', 'imu', 'sensor_msgs/msg/Imu',
                                'imu', bindings))

    if all(s in by_sem for s in MAG_AXES):
        bindings = dict(zip(('x', 'y', 'z'), claim(MAG_AXES)))
        pubs.append(Publication('mag', 'mag',
                                'sensor_msgs/msg/MagneticField',
                                'mag', bindings))

    if Sem.TEMPERATURE in by_sem:
        name = claim((Sem.TEMPERATURE,))[0]
        pubs.append(Publication('temperature', 'temperature',
                                'sensor_msgs/msg/Temperature',
                                'temperature', {'value': name}))

    if Sem.PRESSURE in by_sem:
        name = claim((Sem.PRESSURE,))[0]
        pubs.append(Publication('pressure', 'pressure',
                                'sensor_msgs/msg/FluidPressure',
                                'pressure', {'value': name}))

    fix = by_sem.get(Sem.FIX_TYPE)
    fix_field = fix['name'] if fix is not None else None

    if all(s in by_sem for s in GEO_POSITION):
        bindings = dict(zip(('lat', 'lon', 'alt'), claim(GEO_POSITION)))
        if Sem.POS_H_ACC in by_sem:
            bindings['h_acc'] = claim((Sem.POS_H_ACC,))[0]
        if Sem.POS_V_ACC in by_sem:
            bindings['v_acc'] = claim((Sem.POS_V_ACC,))[0]
        if fix_field:
            bindings['fix_type'] = fix_field
            consumed.add(fix_field)
        pubs.append(Publication('fix', 'fix', 'sensor_msgs/msg/NavSatFix',
                                'fix', bindings))

    if all(s in by_sem for s in GEO_VELOCITY):
        bindings = dict(zip(('north', 'east', 'down'), claim(GEO_VELOCITY)))
        if fix_field:
            bindings['fix_type'] = fix_field
            consumed.add(fix_field)
        pubs.append(Publication('vel', 'vel',
                                'geometry_msgs/msg/TwistStamped',
                                'vel', bindings))

    standard_topics = {p.topic for p in pubs}
    for f in fields:
        name = f['name']
        if name in consumed:
            continue
        topic = name
        if topic in standard_topics:
            # A field named like a standard topic would publish a second
            # message type on it; keep the plan deterministic instead.
            topic = f"field_{name}"
            log.warning("field %r collides with a standard topic; "
                        "publishing on %r", name, topic)
        if f.get('type') == 'string':
            pubs.append(Publication(f"field:{name}", 'text',
                                    'std_msgs/msg/String', topic,
                                    {'value': name}))
        else:
            pubs.append(Publication(f"field:{name}", 'scalar',
                                    'std_msgs/msg/Float64', topic,
                                    {'value': name}))
    return pubs

# u-blox NAV-PVT fix types carrying a GNSS-derived position: 2D, 3D,
# GNSS+dead-reckoning (0 no fix, 1 DR-only, 5 time-only are not).
FIX_TYPES_WITH_FIX = (2, 3, 4)

# Fix types whose GPS time is solved: the positions above plus 5 time-only,
# which stamps from iTOW while NavSatFix reports NO_FIX.
FIX_TYPES_WITH_TIME = (2, 3, 4, 5)

def navsat_status_from_fix_type(fix_type: int) -> int:
    """NavSatStatus.status for the personality's `fix_type`: GNSS-derived positions
    pass as NAVSAT_STATUS_FIX, everything else (unknown vocabulary included)
    gates as NO_FIX."""
    return NAVSAT_STATUS_FIX if fix_type in FIX_TYPES_WITH_FIX \
        else NAVSAT_STATUS_NO_FIX

# Below this the host clock is implausible (cold boot, no NTP) and the
# nearest-week resolver would pick a wrong week. 2020-01-01 UTC.
HOST_CLOCK_FLOOR_UNIX_S = 1_577_836_800

def utc_from_itow(itow_s: float, now_unix_s: float) -> Optional[float]:
    """UTC seconds for a GPS time-of-week, or None when the host clock cannot
    resolve the week: the week is the nearest to the host clock (which must be
    within ±3.5 days), GPS to UTC via the maintained leap constant."""
    if now_unix_s < HOST_CLOCK_FLOOR_UNIX_S:
        return None
    now_gps = now_unix_s - GpsTime.GPS_EPOCH_UNIX_S + GpsTime.GPS_UTC_LEAP_S
    gps = float(itow_s) + GpsTime.GPS_WEEK_S * round((now_gps - float(itow_s))
                                                     / GpsTime.GPS_WEEK_S)
    return gps + GpsTime.GPS_EPOCH_UNIX_S - GpsTime.GPS_UTC_LEAP_S

def stamp_from_us(us: int) -> Tuple[int, int]:
    """(sec, nanosec) for a device microsecond timestamp."""
    return int(us // 1_000_000), int(us % 1_000_000) * 1000

def fill_imu(pub: Publication, values: Dict[str, object]) -> Optional[dict]:
    out = {'orientation_covariance': [-1.0] + [0.0] * 8}
    for group, target in (('accel', 'linear_acceleration'),
                          ('gyro', 'angular_velocity')):
        keys = [f"{group}_{ax}" for ax in 'xyz']
        if keys[0] in pub.bindings:
            vals = [values.get(pub.bindings[k]) for k in keys]
            if any(v is None for v in vals):
                return None
            for ax, v in zip('xyz', vals):
                out[f"{target}.{ax}"] = float(v)
            out[f"{target}_covariance"] = [0.0] * 9
        else:
            out[f"{target}_covariance"] = [-1.0] + [0.0] * 8
    return out

def fill_mag(pub: Publication, values: Dict[str, object]) -> Optional[dict]:
    vals = [values.get(pub.bindings[ax]) for ax in 'xyz']
    if any(v is None for v in vals):
        return None
    out = {f"magnetic_field.{ax}": float(v) for ax, v in zip('xyz', vals)}
    out['magnetic_field_covariance'] = [0.0] * 9
    return out

def fill_temperature(pub: Publication,
                     values: Dict[str, object]) -> Optional[dict]:
    v = values.get(pub.bindings['value'])
    if v is None:
        return None
    # Wire temperature is kelvin; sensor_msgs/Temperature carries °C.
    return {'temperature': float(v) - 273.15, 'variance': 0.0}

def fill_pressure(pub: Publication,
                  values: Dict[str, object]) -> Optional[dict]:
    v = values.get(pub.bindings['value'])
    if v is None:
        return None
    return {'fluid_pressure': float(v), 'variance': 0.0}

def fill_navsat(pub: Publication,
                values: Dict[str, object]) -> Optional[dict]:
    b = pub.bindings
    pos = [values.get(b[k]) for k in ('lat', 'lon', 'alt')]
    if any(v is None for v in pos):
        return None

    status = NAVSAT_STATUS_FIX
    if 'fix_type' in b:
        ft = values.get(b['fix_type'])
        if ft is None:
            return None
        status = navsat_status_from_fix_type(int(ft))

    out = {'status.status': status, 'status.service': NAVSAT_SERVICE_ALL}
    if status < NAVSAT_STATUS_FIX:
        # No fix: never a plausible-looking zero position.
        out.update({'latitude': math.nan, 'longitude': math.nan,
                    'altitude': math.nan,
                    'position_covariance': [0.0] * 9,
                    'position_covariance_type': COVARIANCE_TYPE_UNKNOWN})
        return out

    lat, lon, alt = (float(v) for v in pos)
    # Wire position is radians; NavSatFix wants degrees. Altitude is the
    # device's MSL value (an ellipsoidal height rides its own topic).
    out.update({'latitude': math.degrees(lat),
                'longitude': math.degrees(lon),
                'altitude': alt})

    h = values.get(b['h_acc']) if 'h_acc' in b else None
    v = values.get(b['v_acc']) if 'v_acc' in b else None
    if h is None and v is None:
        out['position_covariance'] = [0.0] * 9
        out['position_covariance_type'] = COVARIANCE_TYPE_UNKNOWN
    else:
        hv = float(h) ** 2 if h is not None else UNKNOWN_VARIANCE
        vv = float(v) ** 2 if v is not None else UNKNOWN_VARIANCE
        out['position_covariance'] = [hv, 0.0, 0.0,
                                      0.0, hv, 0.0,
                                      0.0, 0.0, vv]
        out['position_covariance_type'] = COVARIANCE_TYPE_DIAGONAL_KNOWN
    return out

def fill_twist(pub: Publication,
               values: Dict[str, object]) -> Optional[dict]:
    b = pub.bindings
    if 'fix_type' in b:
        ft = values.get(b['fix_type'])
        if ft is None or navsat_status_from_fix_type(int(ft)) \
                < NAVSAT_STATUS_FIX:
            return None
    ned = [values.get(b[k]) for k in ('north', 'east', 'down')]
    if any(v is None for v in ned):
        return None
    n, e, d = (float(v) for v in ned)
    # Wire velocity is NED; ROS linear velocity is ENU (REP-103).
    return {'twist.linear.x': e, 'twist.linear.y': n, 'twist.linear.z': -d}

def fill_scalar(pub: Publication,
                values: Dict[str, object]) -> Optional[dict]:
    v = values.get(pub.bindings['value'])
    if v is None:
        return None
    return {'data': float(v)}

def fill_text(pub: Publication,
              values: Dict[str, object]) -> Optional[dict]:
    v = values.get(pub.bindings['value'])
    if v is None:
        return None
    return {'data': str(v).rstrip('\r\n')}

def fill_mapped(pub: Publication,
                values: Dict[str, object]) -> Optional[dict]:
    mapping = pub.extras.get('mapping', {})
    out = {}
    for msg_field, sensor_field in mapping.items():
        if sensor_field in values:
            v = values[sensor_field]
            out[msg_field] = v if isinstance(v, str) else float(v)
    if mapping and not out:
        # No mapped field in this sample: suppress rather than publish
        # a default-valued (all-zeros) message.
        return None
    for path, value in pub.extras.get('constants', {}).items():
        out[path] = [float(x) for x in value] if isinstance(value, list) \
            else value
    return out

_FILLERS: Dict[str, Callable[[Publication, Dict[str, object]],
                             Optional[dict]]] = {
    'imu': fill_imu,
    'mag': fill_mag,
    'temperature': fill_temperature,
    'pressure': fill_pressure,
    'fix': fill_navsat,
    'vel': fill_twist,
    'scalar': fill_scalar,
    'text': fill_text,
    'mapped': fill_mapped,
}

def fill_publication(pub: Publication,
                     values: Dict[str, object]) -> Optional[dict]:
    """Dotted-path attribute dict for one publication from one sample's
    decoded values, or None to suppress this cycle."""
    return _FILLERS[pub.kind](pub, values)

def load_map(path: str) -> List[Publication]:
    """Parse a --map YAML (a list of {message, topic, mapping, constants} blocks)
    into publications; the explicit map replaces the auto plan entirely."""
    with open(path) as fh:
        doc = yaml.safe_load(fh)
    if not isinstance(doc, list):
        raise SystemExit(f"nxs ros2: --map {path}: expected a list of "
                         f"{{message, topic, mapping}} blocks")
    pubs = []
    for i, blk in enumerate(doc):
        if not isinstance(blk, dict) or 'message' not in blk \
                or 'topic' not in blk:
            raise SystemExit(f"nxs ros2: --map {path}: block {i} needs "
                             f"'message' and 'topic'")
        mapping = blk.get('mapping')
        if not isinstance(mapping, dict) or not mapping:
            # Without a mapping every sample would publish a
            # default-valued message; refuse the config up front.
            raise SystemExit(f"nxs ros2: --map {path}: block {i} needs "
                             f"a non-empty 'mapping'")
        pubs.append(Publication(
            key=f"mapped:{i}", kind='mapped', msg_type=str(blk['message']),
            topic=str(blk['topic']), bindings={},
            extras={'mapping': mapping,
                    'constants': blk.get('constants') or {}}))
    return pubs

def sanitize_ros_name(name: str) -> str:
    """A valid ROS 2 name token (`[A-Za-z_][A-Za-z0-9_]*`) from an arbitrary
    label: illegal characters collapse to `_`, a leading digit gains an `_`
    prefix (`unit-i2c-10-30` to `unit_i2c_10_30`). Idempotent."""
    token = re.sub(r'[^A-Za-z0-9_]', '_', name)
    if token and token[0].isdigit():
        token = '_' + token
    return token or '_'

def join_topic(base: str, unit_name: Optional[str], rel: str) -> str:
    """Topic name `<base>/<unit>/<rel>` (suite) or `<base>/<rel>` (ad-hoc),
    relative to the node namespace; every token is sanitized."""
    parts = (sanitize_ros_name(p) for p in (base, unit_name, rel) if p)
    return "/".join(parts)

def sensor_plans_imu(personality: str,
                     config: Optional[Dict[str, object]] = None) -> bool:
    """True when the named shipped click personality's compiled output plans an `imu`
    publication; a resolution or compile failure also returns True."""
    try:
        from nxs.suite.reconcile import load_click_personality
        img = load_click_personality(personality)().compile(dict(config or {}))
        return any(p.topic == 'imu'
                   for p in plan_publications(img.output_fields))
    except Exception:
        return True
