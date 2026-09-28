# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The free-run timing law a declared fps is judged by, shared by `on`,
`check`, and `tune`."""

from __future__ import annotations

import inspect
from typing import Optional

from .contracts import InfeasibleConfig, LinkSpec, RateRange, Topology


def lawful_range(pack, topology: Topology, link: LinkSpec,
                 mode: str) -> Optional[RateRange]:
    """The rates `mode` may run at on `link` with the port's camera count
    and lane count, from the
    pack's laws and shipped points (`fps_range`); None for a pack
    without the range law."""
    hook = getattr(pack.flows(), "fps_range", None)
    if hook is None:
        return None
    return hook(pack, topology, link, mode)


def synced_ceiling(pack, topology: Topology, modes=None) -> Optional[float]:
    """The highest fsync generator rate the pack's readout law admits for
    the selected modes (`fsync_fps_ceiling`); None when the pack has no
    trigger overlay or no link takes the pulse."""
    ceiling = getattr(pack.flows(), "fsync_fps_ceiling", None)
    if ceiling is None:
        return None
    try:
        value = ceiling(pack, topology, modes)
    except InfeasibleConfig:
        return None
    return float(value) if value else None


def free_run_refusal(pack, link: LinkSpec, mode: str, fps: float,
                     topology: Optional[Topology] = None) -> Optional[str]:
    """None when `mode` runs free at `fps`, else the refusal `on` would
    print: the rate outside the mode's lawful range on this port (with
    the topology in hand), then the sensor's frame-length law at that
    rate's VMAX. A sensor module without `validate_vmax` has no law."""
    if topology is not None:
        try:
            rates = lawful_range(pack, topology, link, mode)
        except InfeasibleConfig as exc:
            # A shipped point the laws no longer admit is a finding, not a crash.
            return str(exc)
        if rates is not None and not rates.contains(float(fps)):
            from .descriptors import mode_label
            label = mode_label(pack.descriptor(link.sensor_compatible), mode)
            law = "shipped" if rates.shipped else f"the {rates.binds} law's"
            return (f"{fps:g} fps is outside {label}'s {rates.text()} "
                    f"({law} range)")
    module = pack.chip_module(link.sensor_compatible.split(",")[-1])
    validate = getattr(module, "validate_vmax", None)
    vmax_for_fps = getattr(module, "vmax_for_fps", None)
    if validate is None or vmax_for_fps is None:
        return None
    timing = pack.descriptor(link.sensor_compatible).modes[mode]["timing"]
    try:
        vmax = int(vmax_for_fps(float(fps), int(timing["hmax"])))
        # A family whose frame law knows the pixel path's tail takes the
        # transport; one whose law is the datasheet's alone takes none.
        if "transport" in inspect.signature(validate).parameters:
            validate(vmax, mode, transport="pixel")
        else:
            validate(vmax, mode)
    except InfeasibleConfig as exc:
        return str(exc)
    return None
