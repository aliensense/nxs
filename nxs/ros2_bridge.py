"""Descriptor-driven ROS 2 bridge, the engine behind `nxs ros2`: `plan_publications`
groups device-served field descriptors onto standard ROS 2 messages by semantic,
fillers map decoded values onto them, and `Ros2Bridge` is the only rclpy surface."""

import atexit
import fcntl
import hashlib
import logging
import os
import queue
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from nxs.firmware import SUPPORTED_PROTO_VERSION

import yaml

from nxs.client import PUSH_INTERVAL_S, SupportsTimeSync, estimate_and_push
from nxs._generated_constants import GpsTime
from nxs.ros2_semantics import (
    ACCEL_AXES, COVARIANCE_TYPE_DIAGONAL_KNOWN, COVARIANCE_TYPE_UNKNOWN,
    FIX_TYPES_WITH_FIX, FIX_TYPES_WITH_TIME, GEO_POSITION, GEO_VELOCITY,
    GYRO_AXES, HOST_CLOCK_FLOOR_UNIX_S, MAG_AXES, NAVSAT_SERVICE_ALL,
    NAVSAT_STATUS_FIX, NAVSAT_STATUS_NO_FIX, Publication, Sem,
    UNKNOWN_VARIANCE, UnitPlan, epoch_binding, fields_by_semantic, fill_imu,
    fill_mag, fill_mapped, fill_navsat, fill_pressure, fill_publication,
    fill_scalar, fill_temperature, fill_text, fill_twist, join_topic,
    load_map, navsat_status_from_fix_type, plan_publications,
    resolve_msg_type, sanitize_ros_name, sensor_plans_imu, set_nested_attr,
    stamp_from_us, utc_from_itow)

# Re-exported so bridge consumers keep one import site; the names live in
# a leaf module the CLI and launch surfaces can read without the bridge.
from nxs.stamp_modes import (STAMP_ARRIVAL, STAMP_DEVICE, STAMP_ITOW,
                             STAMP_MODES, STAMP_SYNCED)

#: The names `nxs.ros2_bridge` has always answered to, wherever they now live.
__all__ = [
    "ACCEL_AXES", "COVARIANCE_TYPE_DIAGONAL_KNOWN", "COVARIANCE_TYPE_UNKNOWN",
    "FIX_TYPES_WITH_FIX", "FIX_TYPES_WITH_TIME", "GEO_POSITION",
    "GEO_VELOCITY", "GYRO_AXES", "GpsTime", "HOST_CLOCK_FLOOR_UNIX_S",
    "MAG_AXES", "NAVSAT_SERVICE_ALL", "NAVSAT_STATUS_FIX",
    "NAVSAT_STATUS_NO_FIX", "Publication", "READ_RETRY_BACKOFF_S",
    "Ros2Bridge", "STAMP_ARRIVAL", "STAMP_DEVICE", "STAMP_ITOW",
    "STAMP_MODES", "STAMP_SYNCED", "Sem", "UNKNOWN_VARIANCE", "UnitPlan",
    "acquire_run_lock", "build_viz_rviz_config", "epoch_binding",
    "fields_by_semantic", "fill_imu", "fill_mag", "fill_mapped",
    "fill_navsat", "fill_pressure", "fill_publication", "fill_scalar",
    "fill_temperature", "fill_text", "fill_twist", "format_plan",
    "join_topic", "load_map", "navsat_status_from_fix_type",
    "plan_publications", "resolve_msg_type", "run_bridge",
    "sanitize_ros_name", "sensor_plans_imu", "set_nested_attr",
    "stamp_from_us", "utc_from_itow",
]

log = logging.getLogger(__name__)


def foreign_interface(client) -> Optional[int]:
    """The interface version a unit serves when it is not this SDK's, None
    when it is, when the transport serves none, or when the read fails."""
    reader = getattr(client, "interface_version", None)
    if reader is None:
        return None
    try:
        version = reader()
    except (OSError, RuntimeError, NotImplementedError):
        return None
    return None if version in (None, SUPPORTED_PROTO_VERSION) else int(version)


def build_viz_rviz_config(units: List[Tuple[str, str]],
                          topic_base: str) -> str:
    """Write a temporary RViz config for `viz:=true`: the shipped base plus one
    `rviz_imu_plugin/Imu` display per given unit at `/<base>/<frame>/imu`.
    Returns the file's path."""
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
            # The sanitized name is what the runtime uses for the node, namespace,
            # and frame_id: preview the real ROS surface.
            epoch = "  [epoch-capable]" if plan.epoch else ""
            lines.append(f"{sanitize_ros_name(plan.frame_id)}:{epoch}")
        else:
            topic, msg_type, fields = row
            lines.append(f"  {topic:<{width_t}}  {msg_type:<{width_m}}"
                         f"  {fields}".rstrip())
    return "\n".join(lines)


class _RclpyRuntime:
    """The one place rclpy is imported: owns rclpy init/shutdown once per
    process and a node per unit, so each unit shows as its own node."""

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
    # Publish-path caches: one reused message instance per channel, and
    # the dotted paths resolved once to (parent, leaf) pairs.
    msg: object = None
    setters: Dict[str, tuple] = field(default_factory=dict)
    last_vals: Dict[str, object] = field(default_factory=dict)


class _StreamHealth:
    """One unit's sample continuity as the bridge publishes it: the samples
    its sequence numbers say the stream lost, a device stamp that jumped
    over one sample, and a synced stamp that moved against the device
    clock. Each event is logged as it happens, and `summary()` counts them."""

    def __init__(self, frame: str) -> None:
        self._frame = frame
        self._seq: Optional[int] = None
        self._device_us: Optional[int] = None
        self._stamp_ns: Optional[int] = None
        self._period_us: Optional[float] = None
        self._lost = 0
        self._late = 0
        self._steps = 0

    def observe(self, sample, stamp: Tuple[int, int], synced: bool) -> None:
        """Account for `sample`, published at `stamp`, `synced` when the stamp
        is the device clock's projection rather than the arrival time."""
        seq = int(sample.count)
        device_us = sample.timestamp_us
        stamp_ns = stamp[0] * 1_000_000_000 + stamp[1]
        if self._seq is not None:
            missing = (seq - self._seq - 1) & 0xFFFF
            # A backward jump is the VM restarting its count, not a loss.
            if 0 < missing < 0x8000:
                self._lost += missing
                log.warning("%s: %d samples lost (seq %d to %d)", self._frame, missing, self._seq, seq)
            elif missing == 0 and device_us is not None and self._device_us is not None:
                self._observe_step(device_us - self._device_us,
                                   stamp_ns - self._stamp_ns if synced and self._stamp_ns is not None else None)
        self._seq, self._device_us, self._stamp_ns = seq, device_us, stamp_ns

    def _observe_step(self, device_step_us: int, stamp_step_ns: Optional[int]) -> None:
        """One consecutive pair: the device's cadence learned from it, a jump
        past two periods logged, and the projection's move against it."""
        if device_step_us <= 0:
            return
        if self._period_us is None:
            self._period_us = float(device_step_us)
        elif device_step_us > 2 * self._period_us:
            self._late += 1
            log.warning("%s: the device stamp jumped %.1f ms over one sample (its period is %.2f ms)",
                        self._frame, device_step_us / 1000, self._period_us / 1000)
        else:
            self._period_us += (device_step_us - self._period_us) / 64
        if stamp_step_ns is not None and abs(stamp_step_ns / 1000 - device_step_us) > STAMP_STEP_US:
            self._steps += 1
            log.warning("%s: the synced stamp moved %.2f ms against the device clock",
                        self._frame, (stamp_step_ns / 1000 - device_step_us) / 1000)

    def summary(self) -> Optional[str]:
        """The counts as one line, or None when the stream stayed whole."""
        if not (self._lost or self._late or self._steps):
            return None
        return (f"{self._frame}: {self._lost} samples lost, {self._late} device stamps jumped, "
                f"{self._steps} synced stamps moved")


class Ros2Bridge:
    """Publishes decoded samples for a set of unit plans. `runtime` defaults to
    the rclpy wrapper (ImportError without ROS 2); `resolver` turns a message
    type string into a class, and a bridge with no publishable topic refuses."""

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
        self._health = [_StreamHealth(u.frame_id) for u in units]
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
        stamp, synced = self._stamp(unit_idx, sample)
        self._health[unit_idx].observe(sample, stamp, synced)
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
                # Skip unchanged values on the reused message: constant fields
                # cost a numpy-backed convert per assignment.
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
        for health in self._health:
            if (line := health.summary()) is not None:
                log.warning("%s", line)
        self._runtime.shutdown()

    def unit_frame(self, unit_idx: int) -> str:
        return self._units[unit_idx].frame_id

    def _stamp(self, unit_idx: int, sample) -> Tuple[Tuple[int, int], bool]:
        """The sample's stamp, and whether it follows the device clock (a
        synced projection, the device time or a GNSS epoch) rather than the
        arrival time."""
        if self._stamp_mode == STAMP_ITOW \
                and (epoch := self._epochs[unit_idx]) is not None:
            stamp = self._stamp_from_itow(epoch, sample)
            if stamp is not None:
                return stamp, True
            self._warn_stamp_fallback(unit_idx, "no valid GNSS epoch in "
                                                "the stream",
                                      "the synced projection")
        if self._sync_stamps and sample.timestamp_us is not None:
            sync = self._time_syncs[unit_idx] if self._time_syncs else None
            if sync is not None:
                stamp = sync.project_to_realtime(sample.timestamp_us)
                if stamp is not None:
                    return stamp, True
            self._warn_arrival(unit_idx, "no time-sync observation yet")
        elif self._stamp_mode == STAMP_DEVICE \
                and sample.timestamp_us is not None:
            return stamp_from_us(sample.timestamp_us), True
        elif self._stamp_mode != STAMP_ARRIVAL:
            self._warn_arrival(unit_idx, "transport carries no wire "
                                         "timestamp")
        return self._runtime.now_stamp(), False

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
        """One warning per unit per fallback kind; a mode that degrades two ways
        reports both."""
        warned = self._stamp_warned.get(to)
        if warned is None:
            warned = [False] * len(self._units)
            self._stamp_warned[to] = warned
        if not warned[unit_idx]:
            warned[unit_idx] = True
            log.warning("%s: %s; stamping on %s",
                        self._units[unit_idx].frame_id, reason, to)


SILENT_UNIT_WARN_S = 5.0
#: A synced stamp that moves this far against the device clock between two consecutive samples
#: is a re-anchor of the projection, not the sensor's cadence.
STAMP_STEP_US = 1000
_held_run_locks: Dict[str, int] = {}


def acquire_run_lock(tokens, names=None):
    """Take the single-instance locks for a streaming bridge run: one
    exclusive `flock` file per bridged unit, keyed by its link, released by
    the kernel when the process ends, so a suite bridge and a `--unit`
    bridge of the same unit meet on one file. Returns the held fds, in
    token order; SystemExit names the holder: nxsd, which serves the unit
    from its data plane, or another bridge."""
    names = list(names) if names is not None else [None] * len(tokens)
    held, taken = [], []
    for token, name in sorted(zip(tokens, names), key=lambda pair: pair[0]):
        key = hashlib.sha1(token.encode()).hexdigest()[:12]
        if key in _held_run_locks:
            held.append(_held_run_locks[key])
            continue
        path = _run_lock_path(key)
        try:
            fd = _open_run_lock(path)
        except OSError as exc:
            for key_taken in taken:
                os.close(_held_run_locks.pop(key_taken))
            raise SystemExit(f"nxs ros2: the bridge lock {path} does not open: "
                             f"{exc.strerror or exc}\n  - ls -l {path}") from None
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            try:
                holder = os.read(fd, 32).decode(errors="replace").strip()
            except OSError:
                holder = ""
            os.close(fd)
            # The locks this call took go back: a refused run holds nothing.
            for key_taken in taken:
                os.close(_held_run_locks.pop(key_taken))
            raise SystemExit(_lock_refusal(holder, name)) from None
        os.ftruncate(fd, 0)
        os.write(fd, str(os.getpid()).encode())
        _held_run_locks[key] = fd
        taken.append(key)
        held.append(fd)
    return held


def _run_lock_path(key: str) -> str:
    """A unit's run lock, in the state store the daemon and the operator
    share: a world-writable sticky directory refuses root another user's
    file (`fs.protected_regular`), and the store's group directory does not."""
    from nxs.cam import port_state

    locks = port_state.ensure_dir(port_state.state_dir() / "ros2")
    return str(locks / f"{key}.lock")


def _open_run_lock(path: str) -> int:
    """The lock file's descriptor, created for the owner and the store's
    group; a file another user made keeps its mode, since only its owner
    may change it."""
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o660)
    if os.fstat(fd).st_uid == os.getuid():
        os.fchmod(fd, 0o660)
    return fd


def _lock_refusal(holder: str, name) -> str:
    """The refusal for a unit another process bridges: the daemon, named with
    every unit its data plane serves, or another bridge."""
    from nxs.daemon import data_plane

    unit = name or "these units"
    plane = data_plane()
    if plane is not None and holder.isdigit() and int(holder) == plane["pid"]:
        served = ", ".join(plane["units"]) or unit
        return (f"nxs ros2: nxsd (pid {holder}) serves {served} on this host\n"
                f"  - sudo systemctl stop nxsd, for a bridge of your own")
    who = f" (pid {holder})" if holder else ""

    return f"nxs ros2: another bridge{who} serves {unit}\n  - stop it first"


READ_RETRY_BACKOFF_S = 2.0
SYNC_PING_INTERVAL_S = 1.0


def run_bridge(clients: list, bridge: Ros2Bridge,
               count: Optional[int] = None,
               queue_size: int = 1024) -> int:
    """Fan samples from every client into the bridge until `count` samples are
    bridged, every source ends, or the caller interrupts. One reader thread per
    client feeds a bounded queue; the calling thread publishes and re-pings sync."""
    # The default 5 ms GIL switch interval quantizes every handoff between
    # the reader threads and the publisher; a sub-millisecond one interleaves them.
    prev_switch = sys.getswitchinterval()
    sys.setswitchinterval(0.0005)
    q: queue.Queue = queue.Queue(maxsize=queue_size)
    stop = threading.Event()
    # A unit back on another interface (a firmware it rebooted into) is left alone from then on:
    # its bytes are another map, and a time-sync push would write at an address of that map.
    detached = [False] * len(clients)

    def read(idx: int, client):
        # A reader outlives transient transport faults: an error disarms the
        # iterator and the retry re-arms the stream; a clean end ends the reader.
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
                if (foreign := foreign_interface(client)) is not None:
                    detached[idx] = True
                    log.warning("%s: speaks interface version %d now, this nxs speaks %d: "
                                "detached, nxs switch upgrades it", bridge.unit_frame(idx),
                                foreign, SUPPORTED_PROTO_VERSION)
                    return
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
    # A degrading time discipline is invisible in the samples themselves;
    # warn once per unit per fault while the recording is still running.
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
                        # The silent-unit warning only fires when samples stop; an RPC
                        # path that dies while samples keep flowing would never trip it.
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
                    if not isinstance(client, SupportsTimeSync) or detached[i]:
                        continue
                    try:
                        estimate_and_push(client, pings=0)
                        push_warned[i] = False
                    except Exception as e:
                        # Against a standing refusal the device's validity window lapses
                        # and discipline is gone, so say it once.
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
            # One-shot sync banner, deferred until the estimator has a bound;
            # transports without a time surface observe on the sample polls.
            if not bound_shown[idx]:
                if bound := clients[idx].get_time_sync().bound_us():
                    bound_shown[idx] = True
                    print(f"time sync {bridge.unit_frame(idx)}: "
                          f"±{bound / 1000.0:.2f} ms", flush=True)
    finally:
        sys.setswitchinterval(prev_switch)
        stop.set()
        for i, client in enumerate(clients):
            # The unit overwrites what the host leaves unread past its queue.
            if lost := getattr(client, "lost_samples", None):
                log.warning("%s: the unit overwrote %d samples before the host read them",
                            bridge.unit_frame(i), lost)
            try:
                client.stop_stream()
            except Exception:
                pass
        for th in threads:
            th.join(timeout=2.0)
    return samples
