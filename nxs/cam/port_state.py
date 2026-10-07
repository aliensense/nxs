# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Port bookkeeping: the bus lock and the per-link state machine. `on`
records each link's state, capture id, and viewer caps, and the port's
declaration, in a YAML under `state_dir()`; `off` records `parked`; an
absent record reads as `unknown`.
The delivery check that last passed on a port rides its section until the
port changes under it."""

from __future__ import annotations

import contextlib
import contextvars
import fcntl
import functools
import os
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from .contracts import CsiContract, LinkSpec, Topology

#: Overrides for the record and the bus lock (tests point them at a
#: temporary directory); None means the operator's state directory.
LOCK_PATH: Optional[str] = None
STATE_PATH: Optional[str] = None


#: The provisioned host's state store, shared by the daemon and the
#: operator's shell (`nxs switch` creates it, group-writable).
SYSTEM_STATE_DIR = "/var/lib/aliensense"


def state_dir() -> Path:
    """The state store: `/var/lib/aliensense/cam` on a provisioned host (the
    directory exists and is writable), else the operator's own
    `$XDG_STATE_HOME/nxs` (default `~/.local/state/nxs`), created
    owner-only. The port record, the bus lock, the viewer logs and the
    realized derivations live here."""
    system = Path(SYSTEM_STATE_DIR)
    if system.is_dir() and os.access(system, os.W_OK):
        path = system / "cam"
        path.mkdir(parents=True, exist_ok=True)
        return path
    base = os.environ.get("XDG_STATE_HOME") or os.path.join(os.path.expanduser("~"), ".local", "state")
    path = Path(base) / "nxs"
    path.mkdir(parents=True, exist_ok=True)
    os.chmod(path, 0o700)
    return path


def state_path() -> str:
    return STATE_PATH or str(state_dir() / "cam-state.yaml")


def lock_path() -> str:
    return LOCK_PATH or str(state_dir() / "cam-i2c.lock")


#: The capture facts of each link that is up, one `key=value` file per link
#: beside the record: what `libnxs` opens a frame session with.
CAPTURE_DIR = "capture"


def capture_path(port: str, link: str) -> Path:
    return state_dir() / CAPTURE_DIR / f"{port}-{link}"


def _write_capture(port: str, ids: Dict[str, Any], viewers: Dict[str, Dict[str, Any]],
                   topology: Topology) -> None:
    """Write the capture facts of every link the viewer caps cover; a link
    without a capture id or a mode index gets none. A synced pair's link
    carries its part in the pair's exposure and gain (`ae_role`): the
    loop's lock, with the port whose follower heartbeat names the gain for
    a follower (`follow=`, read as the session opens) and the declared gain
    for a locked link, and the ISP left alone for every part."""
    declared = {link.name: link.capture_id for link in topology.links}
    for link, hints in viewers.items():
        capture_id = ids.get(link)
        if capture_id is None:
            capture_id = declared.get(link)
        if capture_id is None or hints.get("sensor_mode") is None:
            continue
        lines = [f"camera_id={int(capture_id)}", f"sensor_mode={int(hints['sensor_mode'])}",
                 f"width={int(hints['width'])}", f"height={int(hints['height'])}"]
        rate = hints.get("framerate")
        if rate:
            lines += [f"fps_num={int(rate[0])}", f"fps_den={int(rate[1])}"]
        pinned = hints.get("exposure_us")
        if pinned is not None and hints.get("exposure_max_us") is None:
            # A trigger's pulse fixes the exposure: the session pins both ends.
            lines += [f"exposure_min_us={float(pinned):g}", f"exposure_max_us={float(pinned):g}"]
        else:
            for key in ("exposure_min_us", "exposure_max_us"):
                if hints.get(key) is not None:
                    lines.append(f"{key}={float(hints[key]):g}")
        role = hints.get("ae_role")
        if role == "follower":
            lines += ["ae_lock=1", f"follow={port}"]
        elif role == "locked":
            lines += ["ae_lock=1", f"gain_db={float(hints.get('gain_db') or 0.0):g}"]
        if role:
            lines += ["isp_gain_unity=1", "isp_filters_off=1"]
        path = capture_path(port, link)
        path.parent.mkdir(parents=True, exist_ok=True)
        staged = path.with_name(path.name + ".tmp")
        staged.write_text("\n".join(lines) + "\n")
        os.replace(staged, path)


def _remove_capture(port: str, links: List[str]) -> None:
    for link in links:
        capture_path(port, link).unlink(missing_ok=True)


#: The heartbeat of the follower nxsd runs on a port's synced pair, one
#: `key=value` file per port beside the record.
FOLLOW_DIR = "follow"


def follow_path(port: str) -> Path:
    return state_dir() / FOLLOW_DIR / port


def write_follow(port: str, fields: Dict[str, Any]) -> None:
    """Write a port's follower heartbeat whole, a `key=value` line per field
    in order, for the owner and the store's group (nxsd writes it as root,
    `status` reads it as the operator)."""
    path = follow_path(port)
    path.parent.mkdir(parents=True, exist_ok=True)
    staged = path.with_name(path.name + ".tmp")
    staged.unlink(missing_ok=True)
    with _open_private(str(staged), os.O_EXCL) as fh:
        fh.write("".join(f"{key}={value}\n" for key, value in fields.items()))
    os.replace(staged, path)


def read_follow(port: str) -> Optional[Dict[str, str]]:
    """A port's follower heartbeat as written; None when nothing follows there."""
    try:
        text = follow_path(port).read_text()
    except OSError:
        return None
    return dict(line.split("=", 1) for line in text.splitlines() if "=" in line)


def follow_gain_db(port: str) -> Optional[float]:
    """The leader's gain a port's follower heartbeat names, in dB: the gain
    the following link's capture session starts at. None without a
    heartbeat, or before the follower takes a gain."""
    try:
        return float((read_follow(port) or {})["gain_db"])
    except (KeyError, ValueError):
        return None


def remove_follow(port: str) -> None:
    follow_path(port).unlink(missing_ok=True)


def _open_private(path: str, flags: int = 0):
    """A file of the tool's own, opened without following a symlink and
    created for the owner and the store's group (the daemon writes the
    record as root, the operator reads it); a planted link at the name is
    refused."""
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW | flags, 0o660)
    os.fchmod(fd, 0o660)
    return os.fdopen(fd, "w")

STATE_UP = "up"
STATE_PARKED = "parked"
STATE_UNKNOWN = "unknown"

#: The port section's key for the delivery check that last passed on it.
VERIFIED = "verified"

#: How long a command waits for another's hold on the bus before it
#: refuses: longer than a gate write holds the lock (half a second behind
#: the hub), shorter than a bring-up.
BUS_LOCK_WAIT_S = 1.0
#: How often a waiting command tries the lock again.
BUS_LOCK_POLL_S = 0.01


class BusHeld(SystemExit):
    """The bus lock is another run's: the exit a verb ends with unprepared,
    and its own type for the one that reports a held bus instead."""


#: The wait `waiting` sets, (seconds, poll seconds), per context so one thread's policy
#: never reaches another's; None waits BUS_LOCK_WAIT_S, polling every BUS_LOCK_POLL_S.
_WAIT: contextvars.ContextVar = contextvars.ContextVar("bus_wait", default=None)


@contextlib.contextmanager
def waiting(seconds: float, poll_s: float = 0.5):
    """Every `BusLock` taken inside waits up to `seconds` for a held lock,
    trying every `poll_s`, so a read that takes the lock on its own waits
    the way the verb's own acquisitions do."""
    token = _WAIT.set((seconds, poll_s))
    try:
        yield
    finally:
        _WAIT.reset(token)


#: A run's wait for a bus another run holds, and its poll: nxsd holds the lock per step
#: while it brings the ports up after boot, each step well under the wait.
HELD_WAIT_S = 20.0
HELD_POLL_S = 0.5


def held_wait():
    """The bounded wait for a held bus: every `BusLock` taken inside waits
    up to HELD_WAIT_S, trying every HELD_POLL_S."""
    return waiting(HELD_WAIT_S, HELD_POLL_S)


class BusLock:
    """Exclusive lock so concurrent runs can't interleave I2C. A hold that
    ends within the wait (BUS_LOCK_WAIT_S, or the policy `waiting` sets) is
    waited out, a longer one refused with BusHeld; the lock is a `flock` on
    a fresh descriptor, so a second one in the same process waits on the
    first like any other run's."""

    def __init__(self) -> None:
        self._fh = _open_private(lock_path())

    def __enter__(self) -> "BusLock":
        wait_s, poll_s = _WAIT.get() or (BUS_LOCK_WAIT_S, BUS_LOCK_POLL_S)
        deadline = time.monotonic() + wait_s
        while True:
            try:
                fcntl.flock(self._fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return self
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    self._fh.close()
                    raise BusHeld(f"{bus_holder('another nxs cam run')} holds {lock_path()} — wait for it")
                time.sleep(poll_s)

    def __exit__(self, *exc: Any) -> None:
        fcntl.flock(self._fh, fcntl.LOCK_UN)
        self._fh.close()


def bus_holder(unnamed: Optional[str] = None) -> Optional[str]:
    """Who holds the bus lock, as the kernel's lock table and the holder's
    command line say (`nxsd (pid 9590)`, `nxs cam1 on (pid 123)`), else
    `unnamed`: no holder, this process itself, or one the host does not let
    this run read. Asked after a wait met the lock held."""
    try:
        held = os.stat(lock_path())
        inode = f"{os.major(held.st_dev):02x}:{os.minor(held.st_dev):02x}:{held.st_ino}"
        with open("/proc/locks") as table:
            pid = next(int(row[-4]) for row in map(str.split, table)
                       if row[-3] == inode and "FLOCK" in row and row[1] != "->")
        argv = Path(f"/proc/{pid}/cmdline").read_bytes().decode(errors="replace").split("\0")[:-1]
    except (OSError, StopIteration, ValueError):
        return unnamed
    if argv and os.path.basename(argv[0]).startswith("python"):
        argv = argv[1:]
    if pid == os.getpid() or not argv:
        return unnamed
    return f"{' '.join([os.path.basename(argv[0]), *argv[1:4]])} (pid {pid})"


def held_text(hint: str) -> str:
    """The line for a bus another run holds past HELD_WAIT_S: the holder the
    kernel's lock table names, else `another nxs run` with `hint`, what
    holds a bus that long."""
    holder = bus_holder()
    held = f"holds the bus after {HELD_WAIT_S:g} s of waiting"
    return f"{holder} {held}" if holder else f"another nxs run {held} ({hint})"


def port_name(topology: Topology) -> str:
    """A port's short name: the carrier suffix (pixelmate/cam1 -> cam1)."""
    return topology.carrier.split("/")[-1].lower()


def _load() -> Dict[str, Any]:
    path = Path(state_path())
    if not path.exists():
        # The record's earlier file name, read until this tool writes one.
        earlier = path.with_name("cam-world.yaml")
        if earlier.exists():
            path = earlier
    try:
        record = yaml.safe_load(path.read_text()) or {}
    except FileNotFoundError:
        return {}
    # The states and every delivery check hold for one boot: after a reboot
    # the hub is unprogrammed on every port at once, whatever the record said.
    if record.get("boot") != _boot_id():
        record.pop("states", None)
        for section in (record.get("ports") or {}).values():
            section.pop(VERIFIED, None)
    return record


def _store(record: Dict[str, Any]) -> None:
    """Write the record whole: a temporary file beside it (created fresh,
    never through a link left at its name) and an atomic replace, so a
    reader never sees a truncated document."""
    record["boot"] = _boot_id()
    target = Path(state_path())
    tmp = target.with_name(target.name + ".tmp")
    tmp.unlink(missing_ok=True)
    with _open_private(str(tmp), os.O_EXCL) as fh:
        fh.write(yaml.safe_dump(record))
    os.replace(tmp, target)


BOOT_ID_PATH = "/proc/sys/kernel/random/boot_id"


def _boot_id() -> Optional[str]:
    """The kernel's id for this boot; None where the kernel publishes none."""
    try:
        return Path(BOOT_ID_PATH).read_text().strip() or None
    except OSError:
        return None


_guard = threading.RLock()
_depth = 0
_lock_fh = None


class _StateLock:
    """Serializes a read-modify-write of the record across processes (one
    flock per process) and re-entrantly within one (an RLock)."""

    def __enter__(self) -> "_StateLock":
        global _depth, _lock_fh
        _guard.acquire()
        if _depth == 0:
            _lock_fh = _open_private(state_path() + ".lock")
            fcntl.flock(_lock_fh, fcntl.LOCK_EX)
        _depth += 1
        return self

    def __exit__(self, *exc: Any) -> None:
        global _depth, _lock_fh
        _depth -= 1
        if _depth == 0 and _lock_fh is not None:
            fcntl.flock(_lock_fh, fcntl.LOCK_UN)
            _lock_fh.close()
            _lock_fh = None
        _guard.release()


def _locked(fn):
    """Run a record mutator under the state lock."""
    @functools.wraps(fn)
    def wrapper(*args: Any, **kwargs: Any):
        with _StateLock():
            return fn(*args, **kwargs)
    return wrapper


@_locked
def save_port(
    topology: Topology,
    links: List[LinkSpec],
    csi: CsiContract,
    viewer: Optional[Dict[str, Any]] = None,
    viewers: Optional[Dict[str, Dict[str, Any]]] = None,
    modes: Optional[Dict[str, str]] = None,
    rates: Optional[Dict[str, float]] = None,
    line: Optional[Dict[str, int]] = None,
) -> None:
    """Record an applied port: selected links go `up`, port siblings go `parked`.
    ``viewer`` is one set of capture caps, ``viewers`` the per-link caps of a
    mixed hub, ``modes`` the mode each link runs, ``rates`` the free-run
    rate each link's sensor was programmed for, ``line`` the line in HMAX
    each head of a pair runs where the delivery check found it (a port
    recorded without one runs the datasheet's). The port's declaration
    rides as `declared` when the topology carries one (`nxs switch` brings
    a port up again once the manifest's differs)."""
    vc_to_capture = {
        link.csi_vc: link.capture_id
        for link in topology.links
        if link.capture_id is not None
    }
    record = _load()
    ids = {
        link.name: vc_to_capture.get(vc.vc)
        for link, vc in zip(links, csi.virtual_channels)
    }
    # The last port stays at the top level; every port also keeps its own
    # section, so two ports up at once each find their ids, caps, and sync.
    record["carrier"] = topology.carrier
    record["links"] = ids
    port = record.setdefault("ports", {}).setdefault(port_name(topology), {})
    port.update({"carrier": topology.carrier, "links": ids,
                 "pipes": dict(getattr(csi, "pipes", None) or {})})
    if topology.declared is not None:
        port["declared"] = topology.declared
    else:
        port.pop("declared", None)
    # The port's section describes the links this port declares and no
    # other: a link a previous declaration carried is forgotten with it.
    selected = {link.name for link in links}
    record.setdefault("states", {})[port_name(topology)] = {
        link.name: STATE_UP if link.name in selected else STATE_PARKED
        for link in topology.links}
    declared = {link.name for link in topology.links}
    for key in ("sensors", "modes", "rates"):
        port[key] = {name: value for name, value in (port.get(key) or {}).items()
                     if name in declared}
    sensors = port["sensors"]
    for link in links:
        sensors[link.name] = link.sensor_compatible
    if modes:
        port.setdefault("modes", {}).update(
            {str(k): str(v) for k, v in modes.items()})
    if rates:
        port.setdefault("rates", {}).update(
            {str(k): float(v) for k, v in rates.items()})
    if line:
        port["line"] = {str(k): int(v) for k, v in line.items()}
    else:
        port.pop("line", None)
    if viewers:
        port["viewers"] = {str(k): dict(v) for k, v in viewers.items()}
        first = next(iter(viewers.values()))
        viewer = viewer or first
    elif viewer:
        port.pop("viewers", None)
    if viewer:
        record["viewer"] = viewer
        port["viewer"] = viewer
    _store(record)
    _remove_capture(port_name(topology), [link.name for link in topology.links])
    if viewers:
        _write_capture(port_name(topology), ids, viewers, topology)


@_locked
def set_sensors(topology: Topology, sensors: Dict[str, str]) -> None:
    """Record the sensor found (or declared) behind each link, before any
    port is up."""
    record = _load()
    port = record.setdefault("ports", {}).setdefault(port_name(topology), {})
    port.setdefault("sensors", {}).update(
        {str(k): str(v) for k, v in sensors.items()})
    _store(record)


def port_sensor(topology: Topology, link: LinkSpec) -> Optional[str]:
    """The sensor the port's record names for a link (detected by a
    probe or applied by `on`), if any."""
    sensors = _port(_load(), topology).get("sensors") or {}
    value = sensors.get(link.name)
    return str(value) if value else None


def port_mode(topology: Topology, link: LinkSpec) -> Optional[str]:
    """The mode a link's last `on` ran, if recorded."""
    modes = _port(_load(), topology).get("modes") or {}
    value = modes.get(link.name)
    return str(value) if value else None


def found_lines(topology: Topology) -> Dict[str, Tuple[str, int]]:
    """The line each head of the port's pair runs as the delivery check
    found it on the rig, (mode, HMAX) by link; {} when the record holds
    none."""
    section = _port(_load(), topology)
    modes = section.get("modes") or {}
    return {str(name): (str(modes[name]), int(hmax))
            for name, hmax in (section.get("line") or {}).items() if modes.get(name)}


def with_found_lines(topology: Topology) -> Topology:
    """The port at the lines the delivery check found for it
    (`found_lines`), the laws judging its rates there; the port itself
    where the record holds none."""
    found = found_lines(topology)
    return topology.with_lines(found) if found else topology


def port_rate(topology: Topology, link: LinkSpec) -> Optional[float]:
    """The free-run rate a link's last `on` programmed, if recorded."""
    rates = _port(_load(), topology).get("rates") or {}
    value = rates.get(link.name)
    return float(value) if value is not None else None


def running_rate(record: Dict[str, Any], link: str) -> Optional[float]:
    """The rate a link of a port's record runs at: the generator's while the
    recorded frame sync paces it, else the free-run rate its last `on`
    programmed; None when the record holds neither. A sync record that
    names no links paces every link."""
    sync = record.get("sync") or {}
    if (sync.get("source") == "fsync" and sync.get("fps") is not None
            and str((sync.get("links") or {}).get(link, "fsync")).startswith("fsync")):
        return float(sync["fps"])
    rate = (record.get("rates") or {}).get(link)
    return float(rate) if rate is not None else None


def port_record(topology: Topology) -> Dict[str, Any]:
    """A port's own section of the record; empty when that port never had a
    port (no fallback to the last port)."""
    return dict(_port(_load(), topology))


def _port(record: Dict[str, Any], topology: Optional[Topology]) -> Dict[str, Any]:
    """A port's own section of the record (empty when unknown), or the
    top level when no port is named."""
    if topology is None:
        return record
    return (record.get("ports") or {}).get(port_name(topology)) or {}


@_locked
def mark_unknown(topology: Topology, links: List[LinkSpec]) -> None:
    """Record links whose bring-up did not verify: neither up nor parked, so
    the next `on` reconstructs them."""
    record = _load()
    states = record.setdefault("states", {}).setdefault(
        port_name(topology), {})
    for link in links:
        states[link.name] = STATE_UNKNOWN
    _port(record, topology).pop(VERIFIED, None)
    _store(record)
    _remove_capture(port_name(topology), [link.name for link in links])


@_locked
def mark_parked(topology: Topology, links: List[LinkSpec]) -> None:
    """Record links as parked (after `off`)."""
    record = _load()
    states = record.setdefault("states", {}).setdefault(
        port_name(topology), {})
    for link in links:
        states[link.name] = STATE_PARKED
    _port(record, topology).pop(VERIFIED, None)
    _store(record)
    _remove_capture(port_name(topology), [link.name for link in links])


@_locked
def set_rates(topology: Topology, rates: Dict[str, float]) -> None:
    """Record the free-run rate `set fps` programs on each named link's
    sensor, and clear the port's last delivery check."""
    record = _load()
    port = record.setdefault("ports", {}).setdefault(port_name(topology), {})
    port.setdefault("rates", {}).update({str(k): float(v) for k, v in rates.items()})
    port.pop(VERIFIED, None)
    _store(record)


@_locked
def set_verified(topology: Topology, rates: Dict[str, float], when: float) -> None:
    """Record the delivery check that passed on the port: the rate each link
    delivered and `when` it ran, in seconds since the epoch. A new boot, a
    sync or rate change, and a link marked unknown or parked clear it."""
    record = _load()
    port = record.setdefault("ports", {}).setdefault(port_name(topology), {})
    port[VERIFIED] = {"at": float(when),
                      "rates": {str(k): round(float(v), 3) for k, v in rates.items()}}
    _store(record)


def verified(topology: Topology) -> Optional[Dict[str, Any]]:
    """The port's last passing delivery check, `{at, rates}`; None when the
    record holds none."""
    seen = _port(_load(), topology).get(VERIFIED)
    return {"at": float(seen["at"]), "rates": dict(seen["rates"])} if seen else None


def recorded_states(topology: Topology) -> Dict[str, str]:
    """The states a step recorded this boot for the port's links, by link
    name; a link no step recorded is absent."""
    record = _load()
    return dict((record.get("states") or {}).get(port_name(topology)) or {})


def link_state(topology: Topology, link: LinkSpec) -> str:
    """The recorded state of one link (`up` / `parked` / `unknown`); a
    record written under another boot carries no states, so it reads as
    `unknown`."""
    record = _load()
    states = (record.get("states") or {}).get(port_name(topology)) or {}
    return str(states.get(link.name, STATE_UNKNOWN))


def port_capture_id(topology: Topology, link: LinkSpec) -> Optional[int]:
    """Capture id for a link in the port's CURRENT port (port fallback)."""
    record = _load()
    section = _port(record, topology)
    ids = section.get("links") or (record.get("links") if not section else {}) or {}
    if link.name in ids and ids[link.name] is not None:
        return int(ids[link.name])
    return link.capture_id


def port_pipes(topology: Topology) -> Dict[str, str]:
    """The deserializer pipe each link rides in the port's current port
    (link name -> pipe); {} when the builder did not say."""
    record = _load()
    section = _port(record, topology)
    return dict((section or {}).get("pipes") or {})


@_locked
def set_viewer_hints(hints: Dict[str, Any],
                     topology: Optional[Topology] = None,
                     viewers: Optional[Dict[str, Dict[str, Any]]] = None) -> None:
    """Replace the recorded viewer caps (a trigger change re-derives
    them); ``viewers`` replaces the per-link caps of a mixed hub."""
    record = _load()
    if viewers and not hints:
        hints = next(iter(viewers.values()))
    record["viewer"] = hints
    if topology is not None:
        port = record.setdefault("ports", {}).setdefault(port_name(topology), {})
        port["viewer"] = hints
        if viewers:
            port["viewers"] = {str(k): dict(v) for k, v in viewers.items()}
        else:
            port.pop("viewers", None)
    _store(record)
    if topology is not None:
        _remove_capture(port_name(topology), [link.name for link in topology.links])
        if viewers:
            _write_capture(port_name(topology), dict(port.get("links") or {}), viewers, topology)


def viewer_hints(topology: Optional[Topology] = None,
                 link: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """Viewer caps recorded at the port's `on` (the last port's when no
    port is named), if any; with ``link`` that link's own caps on a
    mixed hub (falling back to the port's)."""
    record = _load()
    section = _port(record, topology)
    if link is not None:
        per_link = section.get("viewers") or {}
        if link in per_link:
            return per_link[link]
    return section.get("viewer") or (record.get("viewer") if not section else None)


@_locked
def set_sync(source: str, fps: Optional[float] = None,
             topology: Optional[Topology] = None,
             links: Optional[Dict[str, str]] = None,
             pulse_exposure: bool = False,
             trigger_vmax: Optional[Dict[str, int]] = None,
             ae: Optional[Dict[str, Any]] = None) -> None:
    """Record the live synced pair: `free_run`, or `fsync` at fps.
    ``links`` names what each link does under it (`fsync`, or
    `free_run` with the reason) when a mixed hub syncs only some;
    ``pulse_exposure`` and ``trigger_vmax`` carry the generator plan
    (whether the pulse sets the synced links' exposure, the frame each
    link runs); ``ae`` who sets the camera links' exposure and gain (the
    hub's `pair_ae`: `follow`, `locked` or `per_link`)."""
    record = _load()
    sync: Dict[str, Any] = {"source": source, "fps": fps}
    if links:
        sync["links"] = {str(k): str(v) for k, v in links.items()}
    if pulse_exposure:
        sync["pulse_exposure"] = True
    if trigger_vmax:
        sync["trigger_vmax"] = {str(k): int(v) for k, v in trigger_vmax.items()}
    if ae:
        sync["ae"] = dict(ae)
    record["sync"] = sync
    if topology is not None:
        port = record.setdefault("ports", {}).setdefault(port_name(topology), {})
        port["sync"] = dict(sync)
        port.pop(VERIFIED, None)
    _store(record)


def port_sync(topology: Optional[Topology] = None) -> Optional[Dict[str, Any]]:
    """The live sync recorded by the port's last `on` or `set sync` (the
    last port's when no port is named), if any."""
    record = _load()
    section = _port(record, topology)
    return section.get("sync") or (record.get("sync") if not section else None)
