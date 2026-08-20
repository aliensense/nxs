"""Descriptor-driven ROS 2 bridge — the engine behind `nxs ros2`.

The device serves per-field descriptors (name, unit, semantic, scale,
offset); `plan_publications` groups them onto standard ROS 2 messages
by semantic — the same `FieldSemantics` routing the device uses for its
own SI subjects, reframed as sensor_msgs — so any driver appears in ROS
with zero per-sensor config. Fillers are pure functions from a
`Publication` plus decoded sample values to a dotted-path dict;
`Ros2Bridge` is the only rclpy surface (imported lazily, so the tool
runs without ROS everywhere else). `run_bridge` fans in any number of
units: one reader thread per transport feeds a queue, and the thread
that owns the node drains it.
"""

import atexit
import fcntl
import hashlib
import importlib
import logging
import math
import os
import queue

from nxs.client import PUSH_INTERVAL_S, SupportsTimeSync, estimate_and_push
import re
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import yaml

from nxs._generated_constants import FieldSemantics, GpsTime

log = logging.getLogger(__name__)

Sem = FieldSemantics.FieldSemantic

# Frozen sensor_msgs wire constants, hardcoded so the planning core never
# imports ROS packages (the rclpy smoke test cross-asserts them against
# the real message classes).
NAVSAT_STATUS_NO_FIX = -1
NAVSAT_STATUS_FIX = 0
NAVSAT_SERVICE_ALL = 0b1111  # GPS|GLONASS|COMPASS|GALILEO — the wire carries no constellation identity
COVARIANCE_TYPE_UNKNOWN = 0
COVARIANCE_TYPE_DIAGONAL_KNOWN = 2

# Variance for a missing accuracy estimate — the device's own sentinel
# for "unknown" on its geodetic subject (host interface, SI projection).
UNKNOWN_VARIANCE = 1.0e6

ACCEL_AXES = (Sem.ACCEL_X, Sem.ACCEL_Y, Sem.ACCEL_Z)
GYRO_AXES = (Sem.GYRO_X, Sem.GYRO_Y, Sem.GYRO_Z)
MAG_AXES = (Sem.MAG_X, Sem.MAG_Y, Sem.MAG_Z)
GEO_POSITION = (Sem.LATITUDE, Sem.LONGITUDE, Sem.ALTITUDE)
GEO_VELOCITY = (Sem.VEL_NORTH, Sem.VEL_EAST, Sem.VEL_DOWN)

# Re-exported so bridge consumers keep one import site; the names live in
# a leaf module the CLI and launch surfaces can read without the bridge.
from nxs.stamp_modes import (STAMP_ARRIVAL, STAMP_DEVICE, STAMP_ITOW,
                             STAMP_MODES, STAMP_SYNCED)


def resolve_msg_type(type_string: str):
    """Resolve a ROS 2 message type string to the actual Python class.

    Example: "sensor_msgs/msg/Imu" → sensor_msgs.msg.Imu
    """
    parts = type_string.replace("/", ".")
    module_path, _, class_name = parts.rpartition(".")
    module = importlib.import_module(module_path)
    return getattr(module, class_name)


def set_nested_attr(obj, path: str, value):
    """Set a nested attribute using dot notation.

    Example: set_nested_attr(msg, "linear_acceleration.x", 9.81)
    """
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
    """First field claiming each non-generic semantic (strings excluded) —
    the one resolver behind publication planning and epoch binding."""
    by_sem: Dict[int, dict] = {}
    for f in fields:
        sem = int(f.get('semantic', Sem.GENERIC))
        if sem != Sem.GENERIC and sem not in by_sem \
                and f.get('type') != 'string':
            by_sem[sem] = f

    return by_sem


def epoch_binding(fields: List[dict]) -> Optional[Tuple[str, str]]:
    """(time-of-week, fix-type) field names when the descriptors carry
    both registry semantics — the epoch-capable contract `--stamp itow`
    keys on. Resolved once at plan time so stamping never guesses names."""
    by_sem = fields_by_semantic(fields)
    tow = by_sem.get(Sem.TIME_OF_WEEK)
    fix = by_sem.get(Sem.FIX_TYPE)
    if tow is None or fix is None:
        return None

    return tow['name'], fix['name']


def plan_publications(fields: List[dict]) -> List[Publication]:
    """Group device-served descriptors into ROS publications by semantic.

    Vector groups (accel, gyro, mag) and the geodetic position/velocity
    triples form their standard message only when every member is
    present; otherwise the members fall back to per-field topics. The
    first field carrying a semantic claims it — a duplicate demotes to
    its own per-field topic. Every field lands in at least one
    publication; nothing is dropped.
    """
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
# GNSS+dead-reckoning. Not listed: 0 no fix, 1 DR-only (unbounded
# drift), 5 time-only.
FIX_TYPES_WITH_FIX = (2, 3, 4)

# Fix types whose GPS time is solved: the positions above plus 5
# time-only. Time validity is a weaker bar than position validity — a
# time-only fix stamps from iTOW while NavSatFix reports NO_FIX.
FIX_TYPES_WITH_TIME = (2, 3, 4, 5)


def navsat_status_from_fix_type(fix_type: int) -> int:
    """Map the driver's `fix_type` quality field onto a
    NavSatStatus.status value. GNSS-derived positions pass as
    NAVSAT_STATUS_FIX; everything else — including vocabulary this
    bridge doesn't know — gates as NO_FIX (fail-closed: only positions
    the bridge can vouch for publish; a driver serving no fix_type at
    all is trusted instead). Any value below NAVSAT_STATUS_FIX
    publishes a NO_FIX NavSatFix with NaN position and suppresses the
    velocity twist.
    """
    return NAVSAT_STATUS_FIX if fix_type in FIX_TYPES_WITH_FIX \
        else NAVSAT_STATUS_NO_FIX


# Below this the host clock is implausible (cold boot, no NTP) and the
# nearest-week resolver would confidently pick a wrong week — refuse
# instead, as RTKLIB clamps and gpsd warns. 2020-01-01 UTC.
HOST_CLOCK_FLOOR_UNIX_S = 1_577_836_800


def utc_from_itow(itow_s: float, now_unix_s: float) -> Optional[float]:
    """UTC seconds for a GPS time-of-week, or None when the host clock
    is too implausible to resolve the week. The week number is resolved
    against the host clock (nearest of the adjacent weeks, so a rollover
    boundary can't misplace the epoch; the host only has to be within
    ±3.5 days), GPS→UTC via the maintained leap constant. A distinct
    timescale from the host-projected device stamps — mixing the two in
    one consumer needs care."""
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
    # device's MSL value (the ellipsoidal height, when the driver serves
    # one, rides its own per-field topic).
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
    """Parse a --map YAML — a list of {message, topic, mapping,
    constants} blocks — into publications. The explicit map replaces the
    auto plan entirely."""
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
    """A valid ROS 2 name token from an arbitrary label. ROS node,
    namespace, and topic tokens are `[A-Za-z_][A-Za-z0-9_]*`; suite unit
    names are dashed roles (`imu-mast`, `unit-i2c-10-30`) that rclpy
    rejects verbatim. Illegal characters collapse to `_` and a leading
    digit gains an `_` prefix, so `unit-i2c-10-30` → `unit_i2c_10_30`.
    Idempotent."""
    token = re.sub(r'[^A-Za-z0-9_]', '_', name)
    if token and token[0].isdigit():
        token = '_' + token
    return token or '_'


def join_topic(base: str, unit_name: Optional[str], rel: str) -> str:
    """Topic name `<base>/<unit>/<rel>` (suite) or `<base>/<rel>`
    (ad-hoc) — relative, which ROS resolves under the node namespace to
    `/<base>/...`. Every token is sanitized to a valid ROS name."""
    parts = (sanitize_ros_name(p) for p in (base, unit_name, rel) if p)
    return "/".join(parts)


def sensor_plans_imu(driver: str,
                     config: Optional[Dict[str, object]] = None) -> bool:
    """True when the named shipped driver's compiled output plans an
    `imu` publication — the launch keeps RViz Imu displays to the units
    that publish one. Resolution or compile failure (a driver outside
    the wheel) returns True: an idle display over a missing one."""
    try:
        from nxs.descriptor import load_driver
        img = load_driver(driver)().compile(dict(config or {}))
        return any(p.topic == 'imu'
                   for p in plan_publications(img.output_fields))
    except Exception:
        return True


def build_viz_rviz_config(units: List[Tuple[str, str]],
                          topic_base: str) -> str:
    """Write a temporary RViz config for the launch's `viz:=true` path:
    the shipped base (Grid + TF) plus one `rviz_imu_plugin/Imu` display
    per given unit, wired to `/<base>/<frame>/imu`. Returns the file's
    path. Unit names only exist in the manifest, so the shipped file
    cannot carry the displays statically; the launch passes the units
    whose driver plans an imu topic (`sensor_plans_imu`)."""
    base_path = os.path.join(os.path.dirname(__file__), "ros2",
                             "nxs_bridge.rviz")
    with open(base_path) as f:
        cfg = yaml.safe_load(f)
    displays = cfg["Visualization Manager"]["Displays"]
    base = sanitize_ros_name(topic_base)
    for _, frame in units:
        displays.append({
            "Class": "rviz_imu_plugin/Imu",
            "Name": f"Imu {frame}",
            "Enabled": True,
            "Topic": {
                "Value": f"/{base}/{frame}/imu",
                "Depth": 10,
                "Reliability Policy": "Best Effort",
            },
            "Box properties": {"Enable box": True},
            "Axes properties": {"Enable axes": True},
            "Acceleration properties": {"Enable acceleration": True},
        })
    out = tempfile.NamedTemporaryFile(mode="w", suffix=".rviz",
                                      prefix="nxs_bridge_", delete=False)
    with out:
        yaml.safe_dump(cfg, out, sort_keys=False)
    # The launch process outlives RViz's read of the config, so its
    # exit is the earliest safe point to remove the file.
    atexit.register(_unlink_quiet, out.name)
    return out.name


def _unlink_quiet(path: str):
    try:
        os.unlink(path)
    except OSError:
        pass


def format_plan(plans: List[UnitPlan], topic_base: str) -> str:
    """The --plan table: one block per unit, one line per topic."""
    rows = []
    for plan in plans:
        rows.append((None, plan))
        for pub in plan.publications:
            topic = join_topic(topic_base, plan.name, pub.topic)
            fields = " ".join(dict.fromkeys(pub.bindings.values()))
            rows.append(((topic, pub.msg_type, fields), plan))
    width_t = max((len(r[0][0]) for r in rows if r[0]), default=0)
    width_m = max((len(r[0][1]) for r in rows if r[0]), default=0)
    lines = []
    for row, plan in rows:
        if row is None:
            # The sanitized name is what the runtime uses for the node,
            # namespace, and frame_id — preview the real ROS surface.
            epoch = "  [epoch-capable]" if plan.epoch else ""
            lines.append(f"{sanitize_ros_name(plan.frame_id)}:{epoch}")
        else:
            topic, msg_type, fields = row
            lines.append(f"  {topic:<{width_t}}  {msg_type:<{width_m}}"
                         f"  {fields}".rstrip())
    return "\n".join(lines)


class _RclpyRuntime:
    """The one place rclpy is imported. Owns rclpy init/shutdown once per
    process and a node per unit — publishers land on their unit's node,
    so `ros2 node list` shows each unit as its own sensor node."""

    def __init__(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        self._rclpy = rclpy
        rclpy.init()
        self._qos = qos_profile_sensor_data
        self._nodes = []

    def create_node(self, name: str, namespace: str):
        node = self._rclpy.create_node(name, namespace=namespace)
        self._nodes.append(node)
        return node

    def create_publisher(self, node, msg_cls, topic: str):
        return node.create_publisher(msg_cls, topic, self._qos)

    def now_stamp(self) -> Tuple[int, int]:
        # Every node shares the process clock; the first one answers.
        sec, nanosec = self._nodes[0].get_clock().now().seconds_nanoseconds()
        return sec, nanosec

    def shutdown(self):
        if self._rclpy is None:
            return
        for node in self._nodes:
            node.destroy_node()
        self._nodes = []
        # Under `ros2 launch`, rclpy's SIGINT handler shuts the context
        # down before this runs; a second shutdown raises RCLError.
        if self._rclpy.ok():
            self._rclpy.shutdown()
        self._rclpy = None


@dataclass
class _Channel:
    publication: Publication
    msg_cls: type
    publisher: object
    has_header: bool
    # Publish-path caches: one reused message instance per channel (a
    # fresh construction per cycle dominates the publish cost), and the
    # dotted paths resolved once to (parent, leaf) pairs — stable
    # because the message object is.
    msg: object = None
    setters: Dict[str, tuple] = field(default_factory=dict)
    last_vals: Dict[str, object] = field(default_factory=dict)


class Ros2Bridge:
    """Publishes decoded samples for a set of unit plans.

    The runtime seam (`runtime`) defaults to the real rclpy wrapper —
    constructing without a sourced ROS 2 environment raises ImportError
    for the CLI to translate. `resolver` turns a message type string
    into a class; a message package that fails to resolve drops that
    publication with an error, and a bridge with zero publishable
    topics refuses to start.
    """

    def __init__(self, units: List[UnitPlan], *, topic_base: str = "nxs",
                 stamp_mode: str = STAMP_SYNCED,
                 time_syncs: Optional[list] = None, runtime=None,
                 resolver: Callable = resolve_msg_type):
        self._units = units
        self._stamp_mode = stamp_mode
        # itow's documented fallback is the synced projection, so both
        # modes take the sync path on samples without a usable epoch.
        self._sync_stamps = stamp_mode in (STAMP_SYNCED, STAMP_ITOW)
        self._time_syncs = time_syncs
        self._runtime = runtime if runtime is not None else _RclpyRuntime()
        self._channels: List[List[_Channel]] = []
        self._frames = [sanitize_ros_name(u.frame_id) for u in units]
        self._epochs = [u.epoch for u in units]
        if stamp_mode == STAMP_ITOW:
            # Epoch-less units warn once here and never take the itow path.
            for unit, epoch in zip(units, self._epochs):
                if epoch is None:
                    log.warning("%s: no epoch binding (TIME_OF_WEEK + "
                                "FIX_TYPE semantics); stamping on the "
                                "synced projection", unit.frame_id)
        # One warned flag per unit per fallback kind.
        self._stamp_warned: Dict[str, List[bool]] = {}
        total = 0
        for unit in units:
            resolved = []
            for pub in unit.publications:
                try:
                    resolved.append((pub, resolver(pub.msg_type)))
                except (ImportError, AttributeError) as e:
                    log.error("%s: cannot resolve %s (%s) — topic dropped",
                              unit.frame_id, pub.msg_type, e)
            if not resolved:
                self._channels.append([])
                continue
            # One node per unit, in its own namespace; publishers use
            # relative topics resolved under it (`/<base>/<unit>/imu`).
            namespace = "/" + "/".join(
                sanitize_ros_name(p) for p in (topic_base, unit.name) if p)
            node = self._runtime.create_node(
                sanitize_ros_name(unit.name or unit.frame_id), namespace)
            channels = [_Channel(
                publication=pub, msg_cls=msg_cls,
                publisher=self._runtime.create_publisher(
                    node, msg_cls, sanitize_ros_name(pub.topic)),
                has_header=hasattr(msg_cls(), 'header'))
                for pub, msg_cls in resolved]
            self._channels.append(channels)
            total += len(channels)
        if total == 0:
            self._runtime.shutdown()
            raise SystemExit("nxs ros2: no publishable topics — are the "
                             "ROS message packages installed?")

    def publish(self, unit_idx: int, sample) -> int:
        """Map one decoded sample onto the unit's topics. Returns the
        number of messages published (fillers may suppress)."""
        frame = self._frames[unit_idx]
        stamp = self._stamp(unit_idx, sample)
        published = 0
        for ch in self._channels[unit_idx]:
            out = fill_publication(ch.publication, sample.values)
            if out is None:
                continue
            if ch.msg is None:
                ch.msg = ch.msg_cls()
            msg = ch.msg
            if ch.has_header:
                msg.header.frame_id = frame
                msg.header.stamp.sec = stamp[0]
                msg.header.stamp.nanosec = stamp[1]
            for path, value in out.items():
                # Skip unchanged values on the reused message: constant
                # fields (covariance arrays) cost a numpy-backed convert
                # per assignment, which dominates the publish path.
                if ch.last_vals.get(path) == value:
                    continue
                ch.last_vals[path] = value
                cached = ch.setters.get(path)
                if cached is None:
                    parent = msg
                    parts = path.split('.')
                    for part in parts[:-1]:
                        parent = getattr(parent, part)
                    cached = (parent, parts[-1])
                    ch.setters[path] = cached
                setattr(cached[0], cached[1], value)
            ch.publisher.publish(msg)
            published += 1
        return published

    def shutdown(self):
        self._runtime.shutdown()

    def unit_frame(self, unit_idx: int) -> str:
        return self._units[unit_idx].frame_id

    def _stamp(self, unit_idx: int, sample) -> Tuple[int, int]:
        if self._stamp_mode == STAMP_ITOW \
                and (epoch := self._epochs[unit_idx]) is not None:
            stamp = self._stamp_from_itow(epoch, sample)
            if stamp is not None:
                return stamp
            self._warn_stamp_fallback(unit_idx, "no valid GNSS epoch in "
                                                "the stream",
                                      "the synced projection")
        if self._sync_stamps and sample.timestamp_us is not None:
            sync = self._time_syncs[unit_idx] if self._time_syncs else None
            if sync is not None:
                stamp = sync.project_to_realtime(sample.timestamp_us)
                if stamp is not None:
                    return stamp
            self._warn_arrival(unit_idx, "no time-sync observation yet")
        elif self._stamp_mode == STAMP_DEVICE \
                and sample.timestamp_us is not None:
            return stamp_from_us(sample.timestamp_us)
        elif self._stamp_mode != STAMP_ARRIVAL:
            self._warn_arrival(unit_idx, "transport carries no wire "
                                         "timestamp")
        return self._runtime.now_stamp()

    def _stamp_from_itow(self, epoch: Tuple[str, str],
                         sample) -> Optional[Tuple[int, int]]:
        """UTC (sec, nanosec) from the message's own GNSS epoch, or None
        when the fix isn't time-solved (FIX_TYPES_WITH_TIME) or the host
        clock can't resolve the GPS week."""
        tow_name, fix_name = epoch
        values = sample.values or {}
        itow_s = values.get(tow_name)
        fix = values.get(fix_name)
        if itow_s is None or fix is None \
                or int(fix) not in FIX_TYPES_WITH_TIME:
            return None
        utc = utc_from_itow(float(itow_s), time.time())
        if utc is None:
            return None

        return stamp_from_us(round(utc * 1_000_000))

    def _warn_arrival(self, unit_idx: int, reason: str):
        self._warn_stamp_fallback(unit_idx, reason, "arrival")

    def _warn_stamp_fallback(self, unit_idx: int, reason: str, to: str):
        """One warning per unit per fallback kind — a mode that degrades
        two ways reports both."""
        warned = self._stamp_warned.get(to)
        if warned is None:
            warned = [False] * len(self._units)
            self._stamp_warned[to] = warned
        if not warned[unit_idx]:
            warned[unit_idx] = True
            log.warning("%s: %s; stamping on %s",
                        self._units[unit_idx].frame_id, reason, to)


SILENT_UNIT_WARN_S = 5.0
_held_run_locks: Dict[str, int] = {}


def acquire_run_lock(tokens):
    """Take the single-instance lock for a streaming bridge run.

    One streaming bridge per unit set per host: concurrent instances
    duplicate every ROS node name and contend on the device buses. The
    lock is an exclusive `flock` on a file keyed by the resolved target
    set, so the kernel releases it whenever the process ends — no stale
    lockfiles after a crash. Returns the held file descriptor; the
    caller keeps it for the process lifetime; re-acquiring the same
    key in the same process returns the held lock (one process is
    one instance). Raises SystemExit with the holder's pid when
    another process already serves these units.
    `--plan` runs take no lock — descriptor reads coexist with a live
    bridge.
    """
    key = hashlib.sha1("\n".join(sorted(tokens)).encode()).hexdigest()[:12]
    if key in _held_run_locks:
        return _held_run_locks[key]
    path = os.path.join(tempfile.gettempdir(), f"nxs-ros2-{key}.lock")
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        try:
            holder = os.read(fd, 32).decode(errors="replace").strip()
        except OSError:
            holder = ""
        os.close(fd)
        who = f" (pid {holder})" if holder else ""
        raise SystemExit(
            f"nxs ros2: another instance{who} is already serving these "
            f"units — stop it first; concurrent bridges duplicate every "
            f"ROS node and contend on the device buses") from None
    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode())
    _held_run_locks[key] = fd
    return fd


READ_RETRY_BACKOFF_S = 2.0
SYNC_PING_INTERVAL_S = 1.0


def run_bridge(clients: list, bridge: Ros2Bridge,
               count: Optional[int] = None,
               queue_size: int = 1024) -> int:
    """Fan samples from every client into the bridge until `count`
    samples have been bridged, every source ends (finite test fakes),
    or the caller interrupts. A sample counts when it is dequeued and
    offered to the fillers — like `stream --count`, and whether or not
    any message publishes (fillers may suppress). One reader thread per
    client feeds a bounded queue; the calling thread does all the
    publishing, and re-pings each client's time sync on a fixed cadence
    (a no-op for transports whose observations ride the sample poll).
    The caller arms the streams beforehand and closes the clients
    afterwards; stopping a client's stream is what unblocks its reader.
    """
    # Three reader threads and the publisher share one interpreter; the
    # default 5 ms GIL switch interval quantizes every handoff to ~5 ms
    # and caps the per-unit pipeline near 130 Hz regardless of per-op
    # cost. A sub-millisecond interval lets the IO-bound threads
    # interleave at sample cadence.
    prev_switch = sys.getswitchinterval()
    sys.setswitchinterval(0.0005)
    q: queue.Queue = queue.Queue(maxsize=queue_size)
    stop = threading.Event()

    def read(idx: int, client):
        # A reader outlives transient transport faults: a unit that
        # reboots mid-poll (reflash, power-cycle) errors one transaction,
        # the iterator disarms itself, and the retry re-arms the stream
        # once the unit answers again. A clean iterator end (a finite
        # test fake, or a stopped stream) ends the reader instead.
        while not stop.is_set():
            try:
                for sample in client.iter_samples(timeout=0.5):
                    if stop.is_set():
                        break
                    while not stop.is_set():
                        try:
                            q.put((idx, sample), timeout=0.2)
                            break
                        except queue.Full:
                            continue
                if not stop.is_set():
                    return
            except Exception as e:
                log.warning("%s: sample stream error: %s — retrying in %g s",
                            bridge.unit_frame(idx), e, READ_RETRY_BACKOFF_S)
                stop.wait(READ_RETRY_BACKOFF_S)

    threads = [threading.Thread(target=read, args=(i, c), daemon=True,
                                name=f"nxs-ros2-{i}")
               for i, c in enumerate(clients)]
    for th in threads:
        th.start()

    samples = 0
    last_seen = [time.monotonic()] * len(clients)
    warned = [False] * len(clients)
    bound_shown = [False] * len(clients)
    last_ping = time.monotonic()
    last_push = time.monotonic()
    # A degrading time discipline is invisible in the samples themselves —
    # they keep flowing, stamped from an estimator that is quietly
    # extrapolating past its last fit. Warn once per unit per fault so a
    # recording that will be wrong says so while it is still running.
    ping_warned = [False] * len(clients)
    push_warned = [False] * len(clients)
    try:
        while count is None or samples < count:
            now = time.monotonic()
            if now - last_ping >= SYNC_PING_INTERVAL_S:
                last_ping = now
                for i, client in enumerate(clients):
                    try:
                        client.time_sync_ping()
                        ping_warned[i] = False
                    except Exception as e:
                        # The silent-unit warning only fires when SAMPLES
                        # stop; an RPC path that dies while the sample
                        # subject keeps flowing would never trip it, and the
                        # stamps drift at the device's uncorrected rate.
                        if not ping_warned[i]:
                            ping_warned[i] = True
                            log.warning("%s: time-sync ping failed (%s) — "
                                        "timestamps will drift until it "
                                        "recovers", bridge.unit_frame(i), e)
            # Star-topology discipline: push each unit's offset down on
            # the shared cadence so devices stay synced while it runs.
            if now - last_push >= PUSH_INTERVAL_S:
                last_push = now
                for i, client in enumerate(clients):
                    if not isinstance(client, SupportsTimeSync):
                        continue
                    try:
                        estimate_and_push(client, pings=0)
                        push_warned[i] = False
                    except Exception as e:
                        # "Staleness is bounded" only holds if a later push
                        # lands. Against a standing refusal the device's
                        # validity window lapses and discipline is simply
                        # gone, so say it once rather than never.
                        if not push_warned[i]:
                            push_warned[i] = True
                            log.warning("%s: time-sync push failed (%s) — "
                                        "device discipline lapses if this "
                                        "persists", bridge.unit_frame(i), e)
            try:
                idx, sample = q.get(timeout=0.5)
            except queue.Empty:
                if not any(th.is_alive() for th in threads) and q.empty():
                    break
                now = time.monotonic()
                for i, seen in enumerate(last_seen):
                    if not warned[i] and now - seen > SILENT_UNIT_WARN_S:
                        warned[i] = True
                        log.warning("%s: no samples for %.0f s",
                                    bridge.unit_frame(i), now - seen)
                continue
            bridge.publish(idx, sample)
            samples += 1
            last_seen[idx] = time.monotonic()
            warned[idx] = False
            # One-shot sync banner, deferred until the estimator has a
            # bound: transports without a time surface (I2C) observe on
            # the sample polls, so the launch-time banner has nothing to
            # print and the first bound arrives with the first samples.
            if not bound_shown[idx]:
                if bound := clients[idx].get_time_sync().bound_us():
                    bound_shown[idx] = True
                    print(f"time sync {bridge.unit_frame(idx)}: "
                          f"±{bound / 1000.0:.2f} ms", flush=True)
    finally:
        sys.setswitchinterval(prev_switch)
        stop.set()
        for client in clients:
            try:
                client.stop_stream()
            except Exception:
                pass
        for th in threads:
            th.join(timeout=2.0)
    return samples
