# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""What the camera laws offer the panel's knobs: the hubs and the cam
personalities installed, a link's modes and its lawful free-run rates, a
port's synced rates and its gain range, and the refusal `on` would print
for a port's cameras as declared."""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

#: The option that declares nothing.
NONE = "(none)"
#: The HUB of a camera on the host's connector: a port without a hub.
CONNECTOR = "connector"


def installed_hubs() -> List[str]:
    """The compatibles of every deserializer the installed hubs serve."""
    from nxs.cam import hubs

    out: List[str] = []
    try:
        found = hubs.discover()
    except Exception:        # noqa: BLE001 (no hub installed: nothing to offer)
        return out
    for hub in found:
        for chip in hub.chips:
            try:
                d = hub.descriptor(chip)
            except Exception:        # noqa: BLE001 (a chip that does not load is status's finding)
                continue
            if d.role == "DES" and d.compatible not in out:
                out.append(d.compatible)
    return out


def installed_sensors() -> List[str]:
    """The compatibles of every installed cam personality: a head is its own
    node, and any hub's link or the connector may carry it."""
    from nxs.cam import cam_personalities

    try:
        registry = cam_personalities.registry()
        return list(dict.fromkeys(registry.find(name).compatible for name in registry.names()))
    except Exception:        # noqa: BLE001 (a store that does not read offers nothing)
        return []


def hub_for(topology):
    """The hub, or the connector's flows, serving `topology`; None when nothing
    installed does, or without a topology."""
    from nxs.cam import hubs

    if topology is None:
        return None
    try:
        return hubs.for_topology(topology)
    except Exception:        # noqa: BLE001 (the finding is `status`'s)
        return None


def mode_options(hub, sensor: str) -> List[Tuple[str, str]]:
    """(name, label) for every mode the sensor's unit program offers."""
    try:
        sen = hub.descriptor(sensor)
        names = sen.program_modes()
    except Exception:        # noqa: BLE001 (a sensor the hub cannot describe offers no mode)
        return []
    out = []
    for name in names:
        mode = sen.modes.get(name) or {}
        geo = mode.get("geometry") or {}
        data_type = (mode.get("mipi") or {}).get("data_type", "")
        out.append((name, f"{name} · {geo.get('width')}x{geo.get('height')} {data_type}".rstrip()))
    return out


def default_mode(hub, topology, link) -> Optional[str]:
    """The mode the laws run `link` at when the declaration names none: the
    hub's feasible mode on this port, else the sensor's default."""
    flows = hub.flows()
    for call in (lambda: flows.feasible_mode(hub, topology, link),
                 lambda: flows.default_mode(hub, link, topology),
                 lambda: flows.default_mode(hub, link)):
        try:
            return str(call())
        except Exception:        # noqa: BLE001 (a law the flows lack, or a mode the port refuses)
            continue
    return None


def mode_name(hub, sensor: str, token: Optional[str]) -> Optional[str]:
    """The mode `token` names (a mode name, or a geometry the sensor offers
    once), None for a token the sensor does not resolve."""
    from nxs.cam.descriptors import resolve_mode

    if not token:
        return None
    try:
        return resolve_mode(hub.descriptor(sensor), str(token))
    except Exception:        # noqa: BLE001 (several modes share the geometry, or none)
        return None


def free_run_rates(hub, topology, link, mode: str) -> List[int]:
    """Every whole rate from the mode's floor to its ceiling that the
    free-run laws admit on `link`; [] for a mode of one fixed rate."""
    from nxs.cam import timing as cam_timing

    try:
        module = hub.chip_module(link.sensor_compatible.split(",")[-1])
        if module is None:
            return []
        rates = cam_timing.lawful_range(hub, topology, link, mode)
        ceiling = rates.ceiling if rates is not None else module.fps_ceiling(mode)
        floor = (rates.floor if rates is not None and rates.floor
                 else getattr(module, "fps_floor", lambda: 1.0)())
    except Exception:        # noqa: BLE001 (a law the module lacks: no rate knob)
        return []
    if not ceiling:
        return []
    lo, hi = max(int(math.ceil(float(floor))), 1), int(ceiling)
    return [f for f in range(lo, hi + 1)
            if cam_timing.free_run_refusal(hub, link, mode, float(f), topology=topology) is None]


def synced_rates(hub, topology, modes: Dict[str, str]) -> List[int]:
    """The whole rates the trigger laws leave the port's cameras under frame sync."""
    from nxs.cam import timing as cam_timing

    try:
        return list(cam_timing.synced_rates(hub, topology, modes))
    except Exception:        # noqa: BLE001 (a hub without the trigger overlay)
        return []


def gain_range(hub, topology) -> Optional[Tuple[int, int]]:
    """(lo, hi) in whole dB that every camera's gain law admits on the port;
    None when a camera carries no gain law."""
    hi: Optional[int] = None
    for link in topology.camera_links:
        try:
            module = hub.chip_module(link.sensor_compatible.split(",")[-1])
            law = getattr(module, "gain_db_max", None)
            top = law() if law is not None else None
        except Exception:        # noqa: BLE001 (a sensor nothing describes, facts without the law)
            return None
        if top is None:
            return None
        hi = int(top) if hi is None else min(hi, int(top))
    return (0, hi) if hi is not None else None


def camera_refusal(hub, topology, modes: Dict[str, str], rates: Dict[str, float],
                   sync: Optional[dict]) -> Optional[str]:
    """Why `on` would refuse the port's cameras as declared, or None: under
    frame sync the trigger laws at the port's rate, free the timing law of
    each link at its own rate."""
    from nxs.cam import timing as cam_timing

    if sync and sync.get("source") == "fsync":
        if sync.get("fps") is None:
            return None
        try:
            hub.flows().build_fsync(hub, topology, float(sync["fps"]), modes=modes)
        except Exception as exc:        # noqa: BLE001 (InfeasibleConfig, or no trigger overlay)
            return str(exc)
        return None
    for link in topology.camera_links:
        fps, mode = rates.get(link.name), modes.get(link.name)
        if fps is None or mode is None:
            continue
        refusal = cam_timing.free_run_refusal(hub, link, mode, float(fps), topology=topology)
        if refusal:
            return f"link {link.name}: {refusal}"
    return None
