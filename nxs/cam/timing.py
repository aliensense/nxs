# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The timing laws a declared fps is judged by, shared by `on`, `check`,
`set` and `tune`: the free-run range and frame law, the rates frame sync
takes."""

from __future__ import annotations

from typing import List, Optional

from .contracts import InfeasibleConfig, LinkSpec, RateRange, Topology


def lawful_range(pack, topology: Topology, link: LinkSpec,
                 mode: str) -> Optional[RateRange]:
    """The rates `mode` may run at on `link` with the port's camera count
    and lane count, from the pack's laws (`fps_range`); None for a pack
    without the range law."""
    hook = getattr(pack.flows(), "fps_range", None)
    if hook is None:
        return None
    return hook(pack, topology, link, mode)


def synced_rates(pack, topology: Topology, modes=None) -> List[int]:
    """The whole rates the port runs under frame sync for the selected
    modes (`fsync_rates`); empty when the pack has no trigger overlay or
    the port takes no pulse."""
    rates = getattr(pack.flows(), "fsync_rates", None)
    if rates is None:
        return []
    try:
        return [int(fps) for fps in rates(pack, topology, modes)]
    except InfeasibleConfig:
        return []


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
            # A mode the laws refuse on the port returns their refusal.
            return str(exc)
        if rates is not None and not rates.contains(float(fps)):
            from .descriptors import mode_label
            label = mode_label(pack.descriptor(link.sensor_compatible), mode)
            return (f"{fps:g} fps is outside {label}'s {rates.text()} "
                    f"(the {rates.binds} law's range)")
    module = pack.chip_module(link.sensor_compatible.split(",")[-1])
    validate = getattr(module, "validate_vmax", None)
    vmax_for_fps = getattr(module, "vmax_for_fps", None)
    if validate is None or vmax_for_fps is None:
        return None
    timing = pack.descriptor(link.sensor_compatible).modes[mode]["timing"]
    try:
        vmax = int(vmax_for_fps(float(fps), int(timing["hmax"])))
        validate(vmax, mode)
    except InfeasibleConfig as exc:
        return str(exc)
    return None
