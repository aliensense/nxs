# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0
"""The manifest's camera ports: a port's declaration, its links and the units
riding them, and the generated wiring laid under the hand-written intent."""

import hashlib
import math
import os
from dataclasses import dataclass, field
from typing import List, Optional

from nxs.suite.schema_base import ManifestError, _parse_int, _require_keys

HUB_DRIVERS = ("nxs", "kernel")

_PORT_KEYS = {"bus", "hub", "csi_lanes", "sync", "links", "camera"}
_PORT_CAMERA_KEYS = {"mode", "sync", "sensor", "exposure_us", "gain_db"}
# A link's `camera` is the sensor compatible (string) or a mapping
# {sensor, mode}; the port's `camera.sensor` is the default.
_LINK_CAMERA_KEYS = {"sensor", "mode", "fps"}
_HUB_KEYS = {"compatible", "addr", "driver"}
_PORT_LINK_KEYS = {"camera", "ser", "des_window", "csi_vc", "ser_addr",
                   "sensor_addr", "tca_addr", "capture_id", "unit", "inck_hz"}
# What only a link behind a hub carries.
_HUB_LINK_KEYS = ("ser", "des_window", "ser_addr", "tca_addr")
_PORT_UNIT_KEYS = {"name", "alias", "target"}
_PORT_SYNC_KEYS = {"source", "fps"}
#: The frame-sync sources a port declares; anything else is a typo.
SYNC_SOURCES = ("free_run", "fsync")


@dataclass
class PortUnitRef:
    """The NXS unit riding a port link (translated host address)."""
    name: str
    alias: int = 0x30
    target: int = 0x30


@dataclass
class PortLinkSpec:
    name: str
    camera: Optional[str]             # the sensor compatible; None until declared
    ser: Optional[str]                # the link's serializer, behind a hub
    des_window: Optional[int]
    csi_vc: Optional[int]             # None: the hub's channel for the link's name
    ser_addr: int = 0x42
    sensor_addr: Optional[int] = None  # None: the descriptor's own address
    tca_addr: int = 0x20
    capture_id: Optional[int] = None
    unit: Optional[PortUnitRef] = None
    camera_mode: Optional[str] = None  # this link's declared mode token
    camera_fps: Optional[float] = None  # this link's declared free-run rate
    inck_hz: Optional[int] = None      # the clock the pod feeds the sensor


@dataclass
class PortSpec:
    """One carrier connector: its bus and what is connected to it, a hub
    with its GMSL links or the one sensor on the port's own bus."""
    name: str
    bus: Optional[str] = None          # None: the host's bus for the port's name
    hub_compatible: Optional[str] = None
    #: The generated wiring file, when the hub is its word and not the intent's.
    hub_source: Optional[str] = None
    hub_addr: int = 0x6A
    hub_driver: str = "nxs"
    csi_lanes: int = 4
    csi_lanes_declared: bool = False   # False: the booted overlay decides
    sync_source: str = "free_run"
    sync_fps: Optional[float] = None
    ## Declared camera behavior: a mode token (a descriptor mode name) and the
    ## fps that context interprets (free_run timing, or the fsync rate).
    camera_mode: Optional[str] = None
    camera_fps: Optional[float] = None
    ## A declared exposure, microseconds: `check` and `on` refuse it with
    ## what sets the exposure (the trigger pulse under frame sync, the
    ## capture stack's loop in free run without `camera_gain_db`).
    camera_exposure_us: Optional[float] = None
    ## A declared analog gain, dB: the camera links locked at it, a synced
    ## pair, or free-running links with `camera_exposure_us`.
    camera_gain_db: Optional[float] = None
    ## The port's default sensor (compatible) for links that name none.
    camera_sensor: Optional[str] = None
    links: List[PortLinkSpec] = field(default_factory=list)


def port_signature(port: PortSpec) -> str:
    """Every declared field that shapes the port's construction, as one
    digest: nxsd reconverges a port on a reload when it changed, and `nxs
    switch` brings a port up again when its record carries another."""
    fields = (port.bus, port.hub_compatible, port.hub_addr, port.hub_driver,
              port.csi_lanes, port.csi_lanes_declared,
              port.camera_mode, port.camera_fps, port.camera_sensor,
              port.camera_exposure_us, port.camera_gain_db, port.sync_source, port.sync_fps,
              tuple((l.name, l.camera, l.camera_mode, l.camera_fps, l.inck_hz, l.ser,
                     l.ser_addr, l.sensor_addr, l.tca_addr, l.des_window, l.csi_vc,
                     l.capture_id,
                     (l.unit.name, l.unit.alias, l.unit.target) if l.unit else None)
                    for l in port.links))
    return hashlib.sha256(repr(fields).encode()).hexdigest()


def _parse_link_camera(raw, port_default: Optional[str], where: str):
    """A link's camera declaration, `vendor,sensor` or `{sensor, mode, fps}`,
    with the port's default sensor for a link that names none. Returns
    (sensor, mode, fps)."""
    if raw is None:
        # A link with no sensor named yet: `check` says what to declare,
        # and the port's flows leave the link out until it is.
        return port_default, None, None
    if isinstance(raw, str):
        if not raw.strip():
            raise ManifestError(f"{where}.camera: empty")
        return raw.strip(), None, None
    if not isinstance(raw, dict):
        raise ManifestError(
            f"{where}.camera: a sensor compatible or a mapping "
            f"{{sensor, mode, fps}}")
    _require_keys(raw, _LINK_CAMERA_KEYS, f"{where}.camera")
    sensor = raw.get("sensor", port_default)
    if sensor is None:
        raise ManifestError(
            f"{where}.camera: needs a 'sensor' (or a port-level "
            f"camera.sensor default)")
    mode = raw.get("mode")
    if mode is not None and not str(mode).strip():
        raise ManifestError(f"{where}.camera.mode: empty")
    fps = raw.get("fps")
    if fps is not None:
        try:
            fps = float(fps)
        except (TypeError, ValueError):
            raise ManifestError(f"{where}.camera.fps: {fps!r} is not a number") from None
        if not math.isfinite(fps) or fps <= 0:
            raise ManifestError(f"{where}.camera.fps: must be finite and positive, got {fps}")
    return (str(sensor).strip(), (str(mode).strip() if mode is not None else None), fps)


#: The alternative every wiring disagreement names.
WIRING_ALTERNATIVE = "nxs generate (writes what answers on the port)"


def _no_sync_generator(where: str, source: str) -> ManifestError:
    return ManifestError(
        f"{where}: {source!r} needs a frame-sync generator, and the sensor on "
        f"the port's own bus has none", [f"{where}: free_run"])


def _require_separable_pods(port: PortSpec, where: str) -> None:
    """Two pods behind one hub answer apart only through their aliases: a
    pod at its strapped address answers on every link the hub merges, and
    two pods at one alias merge the same way."""
    pods = [(index, link) for index, link in enumerate(port.links)
            if link.unit is not None]
    if len(pods) < 2:
        return
    alternatives = [f"{where}.links.{link.name}.unit.alias: {0x31 + index:#04x}"
                    for index, link in pods]
    claimed = {}
    for _index, link in pods:
        lwhere = f"{where}.links.{link.name}.unit"
        if link.unit.alias == link.unit.target:
            raise ManifestError(
                f"{lwhere}: a pod at its strapped address {link.unit.target:#04x} "
                f"answers on every link, and {len(pods)} pods ride this hub",
                alternatives)
        if link.unit.alias in claimed:
            raise ManifestError(
                f"{lwhere}.alias: {link.unit.alias:#04x} is also link "
                f"{claimed[link.unit.alias]}'s, and two pods at one alias "
                f"merge", alternatives)
        claimed[link.unit.alias] = link.name


def _manifest_name(where: str) -> str:
    """The manifest file's name out of a parser location (`<path>.ports.<name>`)."""
    head = where.split(":", 1)[0]
    for marker in (".ports.", "/ports."):
        if marker in head:
            head = head.split(marker, 1)[0]
    return os.path.basename(head) or "suite.yaml"


def _parse_port(name, raw, where: str) -> PortSpec:
    _require_keys(raw, _PORT_KEYS, where)
    # The bus is the host's fact for a named port; a declared one stands.
    port = PortSpec(name=str(name), bus=(str(raw["bus"]) if raw.get("bus") else None))
    hub = raw.get("hub")
    if hub is not None:
        if isinstance(hub, str):
            if not hub.strip():
                raise ManifestError(f"{where}.hub: empty")
            hub = {"compatible": hub.strip()}
        _require_keys(hub, _HUB_KEYS, f"{where}.hub")
        if "compatible" not in hub:
            raise ManifestError(f"{where}.hub: needs a 'compatible'")
        port.hub_compatible = str(hub["compatible"])
        port.hub_source = _manifest_name(where)
        port.hub_addr = _parse_int(hub.get("addr", 0x6A), f"{where}.hub.addr")
        port.hub_driver = str(hub.get("driver", "nxs"))
        if port.hub_driver not in HUB_DRIVERS:
            raise ManifestError(
                f"{where}.hub.driver: {port.hub_driver!r} "
                f"(one of {list(HUB_DRIVERS)})")
    if "csi_lanes" in raw:
        port.csi_lanes = _parse_int(raw["csi_lanes"], f"{where}.csi_lanes")
        if not 1 <= port.csi_lanes <= 4:
            raise ManifestError(f"{where}.csi_lanes: {port.csi_lanes} (1 to 4)")
        port.csi_lanes_declared = True
    sync = raw.get("sync") or {}
    _require_keys(sync, _PORT_SYNC_KEYS, f"{where}.sync")
    port.sync_source = str(sync.get("source", "free_run"))
    if port.sync_source not in SYNC_SOURCES:
        raise ManifestError(
            f"{where}.sync.source: {port.sync_source!r} "
            f"(one of {', '.join(SYNC_SOURCES)})")
    fps = sync.get("fps")
    if fps is not None:
        try:
            fps = float(fps)
        except (TypeError, ValueError):
            raise ManifestError(f"{where}.sync.fps: {fps!r} is not a number") from None
        if not math.isfinite(fps) or fps <= 0:
            raise ManifestError(f"{where}.sync.fps: must be finite and positive, got {fps}")
    port.sync_fps = fps
    # With no hub the port serves the one sensor on its own bus.
    direct = port.hub_compatible is None
    if direct and port.sync_source != "free_run":
        raise _no_sync_generator(f"{where}.sync.source", port.sync_source)
    camera = raw.get("camera")
    if camera is not None:
        if direct and not raw.get("links"):
            raise ManifestError(
                f"{where}.camera: nothing is declared on this port to apply "
                f"camera config to",
                [f"{where}.links.A.camera: <sensor>", WIRING_ALTERNATIVE])
        _require_keys(camera, _PORT_CAMERA_KEYS, f"{where}.camera")
        if not camera:
            raise ManifestError(
                f"{where}.camera: needs a 'mode', 'sensor' or 'sync'")
        token = str(camera.get("mode", "") or "").strip()
        if "mode" in camera and not token:
            raise ManifestError(f"{where}.camera.mode: empty")
        if token and "@" in token:
            token, fps_part = token.rsplit("@", 1)
            try:
                fps = float(fps_part)
            except ValueError:
                raise ManifestError(
                    f"{where}.camera.mode: fps {fps_part!r} is not a "
                    f"number") from None
            if not math.isfinite(fps) or fps <= 0:
                raise ManifestError(
                    f"{where}.camera.mode: fps must be finite and "
                    f"positive, got {fps_part}")
            port.camera_fps = fps
        port.camera_mode = token or None
        if camera.get("exposure_us") is not None:
            try:
                exposure = float(camera["exposure_us"])
            except (TypeError, ValueError):
                raise ManifestError(
                    f"{where}.camera.exposure_us: {camera['exposure_us']!r} is not "
                    f"a number") from None
            if not (math.isfinite(exposure) and exposure > 0):
                raise ManifestError(f"{where}.camera.exposure_us: must be a finite "
                                    f"positive number")
            port.camera_exposure_us = exposure
        if camera.get("gain_db") is not None:
            try:
                gain = float(camera["gain_db"])
            except (TypeError, ValueError):
                raise ManifestError(
                    f"{where}.camera.gain_db: {camera['gain_db']!r} is not a number") from None
            if not (math.isfinite(gain) and gain >= 0):
                raise ManifestError(f"{where}.camera.gain_db: must be a finite number of dB, "
                                    f"0 or more")
            port.camera_gain_db = gain
        if "sync" in camera:
            # camera.sync supersedes the port-level sync key.
            port.sync_source = str(camera["sync"])
            if port.sync_source not in SYNC_SOURCES:
                raise ManifestError(
                    f"{where}.camera.sync: {port.sync_source!r} "
                    f"(one of {', '.join(SYNC_SOURCES)})")
            if direct and port.sync_source != "free_run":
                raise _no_sync_generator(f"{where}.camera.sync", port.sync_source)
        if camera.get("sensor") is not None:
            port.camera_sensor = str(camera["sensor"])
    if direct and len(raw.get("links") or {}) > 1:
        # One CSI receiver takes one sensor's lanes; a hub multiplexes two.
        names = ", ".join(str(n) for n in raw["links"])
        raise ManifestError(
            f"{where}.links: the port's receiver takes one sensor's lanes, "
            f"got {len(raw['links'])} links ({names})",
            ["one link", WIRING_ALTERNATIVE])
    for lname, lraw in (raw.get("links") or {}).items():
        lwhere = f"{where}.links.{lname}"
        _require_keys(lraw, _PORT_LINK_KEYS, lwhere)
        if direct:
            stray = sorted(k for k in _HUB_LINK_KEYS if k in lraw)
            if stray:
                raise ManifestError(
                    f"{lwhere}: {stray} describe a link behind a hub, and "
                    f"{where} names no hub", [WIRING_ALTERNATIVE, "drop the keys"])
            lraw = dict(lraw, csi_vc=lraw.get("csi_vc", 0))
            if _parse_int(lraw["csi_vc"], f"{lwhere}.csi_vc") != 0:
                raise ManifestError(
                    f"{lwhere}.csi_vc: {lraw['csi_vc']} on the sensor's own lanes, which "
                    f"carry virtual channel 0", [f"{lwhere}.csi_vc: 0", WIRING_ALTERNATIVE])
        link_sensor, link_mode, link_fps = _parse_link_camera(
            lraw.get("camera"), port.camera_sensor, lwhere)
        unit = None
        if lraw.get("unit") is not None:
            uraw = lraw["unit"]
            if isinstance(uraw, str):
                uraw = {"name": uraw}
            _require_keys(uraw, _PORT_UNIT_KEYS, f"{lwhere}.unit")
            if "name" not in uraw:
                raise ManifestError(f"{lwhere}.unit: needs a 'name'")
            if direct and "alias" in uraw:
                # Nothing translates on the port's own bus: the unit is
                # reached at its own address, `target`.
                raise ManifestError(
                    f"{lwhere}.unit.alias: an alias is a hub's translation, and "
                    f"{where} names no hub",
                    [f"{lwhere}.unit.target: {uraw['alias']}", WIRING_ALTERNATIVE])
            target = _parse_int(uraw.get("target", 0x30),
                                f"{lwhere}.unit.target")
            if (direct and lraw.get("sensor_addr") is not None
                    and _parse_int(lraw["sensor_addr"], f"{lwhere}.sensor_addr") == target):
                # Nothing translates on the port's own bus: two devices at
                # one address take each other's writes.
                raise ManifestError(
                    f"{lwhere}.unit.target: {target:#04x} is the sensor's address too, "
                    f"and nothing on the port's own bus tells the two apart",
                    [f"{lwhere}.unit.target: the address the unit straps",
                     f"{lwhere}.sensor_addr: the address the sensor straps"])
            unit = PortUnitRef(
                name=str(uraw["name"]),
                alias=_parse_int(uraw.get("alias", target),
                                 f"{lwhere}.unit.alias"),
                target=target,
            )
        # Behind a hub the serializer, the window and the channel are the
        # hub's rules for the link's name unless the link names them.
        port.links.append(PortLinkSpec(
            name=str(lname),
            camera=link_sensor,
            ser=(str(lraw["ser"]) if not direct and lraw.get("ser") else None),
            des_window=(_parse_int(lraw["des_window"], f"{lwhere}.des_window")
                        if not direct and lraw.get("des_window") is not None else None),
            csi_vc=(_parse_int(lraw["csi_vc"], f"{lwhere}.csi_vc")
                    if lraw.get("csi_vc") is not None else (0 if direct else None)),
            ser_addr=_parse_int(lraw.get("ser_addr", 0x42),
                                f"{lwhere}.ser_addr"),
            sensor_addr=(_parse_int(lraw["sensor_addr"],
                                    f"{lwhere}.sensor_addr")
                         if lraw.get("sensor_addr") is not None else None),
            tca_addr=_parse_int(lraw.get("tca_addr", 0x20),
                                f"{lwhere}.tca_addr"),
            capture_id=(_parse_int(lraw["capture_id"], f"{lwhere}.capture_id")
                      if lraw.get("capture_id") is not None else None),
            unit=unit,
            camera_mode=link_mode,
            camera_fps=link_fps,
            inck_hz=(_parse_int(lraw["inck_hz"], f"{lwhere}.inck_hz")
                     if lraw.get("inck_hz") is not None else None),
        ))
    if not direct:
        _require_separable_pods(port, where)
    return port


#: What the report (`hardware.yaml`) writes per port and per link: what
#: answered, never read back as a declaration.
