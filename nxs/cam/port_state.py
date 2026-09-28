# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Port bookkeeping: the bus lock and the per-link state machine. `on`
records each link's state, capture id, and viewer caps in a YAML under
`state_dir()`; `off` records `parked`; an absent record reads as `unknown`."""

from __future__ import annotations

import fcntl
import functools
import os
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

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
    without a capture id or a mode index gets none."""
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
        path = capture_path(port, link)
        path.parent.mkdir(parents=True, exist_ok=True)
        staged = path.with_name(path.name + ".tmp")
        staged.write_text("\n".join(lines) + "\n")
        os.replace(staged, path)


def _remove_capture(port: str, links: List[str]) -> None:
    for link in links:
        capture_path(port, link).unlink(missing_ok=True)


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


class BusLock:
    """Exclusive lock so concurrent runs can't interleave I2C."""

    def __init__(self) -> None:
        self._fh = _open_private(lock_path())

    def __enter__(self) -> "BusLock":
        try:
            fcntl.flock(self._fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit(
                f"another nxs cam run holds {lock_path()} — wait for it"
            )
        return self

    def __exit__(self, *exc: Any) -> None:
        fcntl.flock(self._fh, fcntl.LOCK_UN)
        self._fh.close()


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
    # The states hold for one boot: the hub is unprogrammed after a reboot,
    # whatever the record said before it, on every port at once.
    if record.get("boot") != _boot_id():
        record.pop("states", None)
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
) -> None:
    """Record an applied port: selected links go `up`, port siblings go `parked`.
    ``viewer`` is one set of capture caps, ``viewers`` the per-link caps of a
    mixed hub, ``modes`` the mode each link runs, ``rates`` the free-run
    rate each link's sensor was programmed for."""
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


def port_rate(topology: Topology, link: LinkSpec) -> Optional[float]:
    """The free-run rate a link's last `on` programmed, if recorded."""
    rates = _port(_load(), topology).get("rates") or {}
    value = rates.get(link.name)
    return float(value) if value is not None else None


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
    _store(record)
    _remove_capture(port_name(topology), [link.name for link in links])


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
             pulses_per_frame: Optional[int] = None,
             exposure_us: Optional[float] = None,
             trigger_vmax: Optional[Dict[str, int]] = None) -> None:
    """Record the live synced pair: `free_run`, or `fsync` at fps.
    ``links`` names what each link does under it (`fsync`, or
    `free_run` with the reason) when a mixed hub syncs only some;
    ``pulses_per_frame``, ``exposure_us`` and ``trigger_vmax`` carry the
    generator plan (the pulse multiple, the exposure it sets, the frame
    each link runs)."""
    record = _load()
    sync: Dict[str, Any] = {"source": source, "fps": fps}
    if links:
        sync["links"] = {str(k): str(v) for k, v in links.items()}
    if pulses_per_frame is not None:
        sync["pulses_per_frame"] = int(pulses_per_frame)
    if exposure_us is not None:
        sync["exposure_us"] = float(exposure_us)
    if trigger_vmax:
        sync["trigger_vmax"] = {str(k): int(v) for k, v in trigger_vmax.items()}
    record["sync"] = sync
    if topology is not None:
        record.setdefault("ports", {}).setdefault(port_name(topology), {})["sync"] = dict(sync)
    _store(record)


def port_sync(topology: Optional[Topology] = None) -> Optional[Dict[str, Any]]:
    """The live sync recorded by the port's last `on` or `set sync` (the
    last port's when no port is named), if any."""
    record = _load()
    section = _port(record, topology)
    return section.get("sync") or (record.get("sync") if not section else None)
