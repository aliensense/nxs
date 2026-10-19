"""The composition helpers the port verbs share: the hub's plan of a
bring-up (`_compose_from`), which its laws judge and which states the CSI
contract, the modes and the rates the verbs take, the port narrowed to the
links they select, and the bus handler the verbs read probes through
(`CamI2c`). Every port comes up, syncs, parks, gates and takes a knob on
its graph image (`nxs.cam.graph`)."""

from __future__ import annotations

import argparse
from typing import Any, Dict, List, Optional

from nxs.cam.contracts import LinkSpec, Topology
from nxs.cam.engine import CamI2c
from nxs.cam.select import _per_link

__all__ = ["CamI2c"]


def _rates_arg(args: argparse.Namespace, links: List[LinkSpec]) -> Optional[Dict[str, float]]:
    """`--fps` as the builders take it: a rate per selected link, or None."""
    rates = _per_link(getattr(args, "fps", None), links, "fps")
    return {name: float(v) for name, v in rates.items()} or None


def _compose_from(hub, topology: Topology, links: List[LinkSpec],
                  args: argparse.Namespace, modes: Optional[Dict[str, str]] = None):
    flows = hub.flows()
    mode = modes if modes else _mode_arg(args)
    extra: Dict[str, Any] = {}
    rates = _rates_arg(args, links)
    if rates is not None:
        # A hub authored before the rate law takes no fps; the rate then
        # is not a knob it offers.
        if "fps" not in _accepted(flows.build_solo if len(links) == 1 else flows.build_dual):
            raise SystemExit(f"hub {hub.name!r} offers no --fps: its flows take "
                             f"no rate")
        extra["fps"] = rates
    if len(links) == 1:
        return flows.build_solo(hub, topology, links[0].name, mode=mode, **extra)
    return flows.build_dual(hub, topology, mode=mode, **extra)


def _port_links(topology: Topology, links: List[LinkSpec]) -> Topology:
    """The port narrowed to the selected links: the topology the port's
    laws judge (a solo is one link's port, whatever the port carries),
    the same one the builders compose against."""
    import dataclasses

    if len(links) == len(topology.links):
        return topology
    return dataclasses.replace(topology, links=tuple(links))


def _resolve_modes(flows, hub, links, modes, topology):
    """flows.resolve_modes with the selected links' port when the hub
    accepts a topology."""
    import inspect

    resolve = getattr(flows, "resolve_modes", None)
    if resolve is None:
        return dict(modes)
    if "topology" in inspect.signature(resolve).parameters:
        return resolve(hub, links, modes or None,
                       topology=_port_links(topology, links))
    return resolve(hub, links, modes or None)


def _mode_arg(args: argparse.Namespace):
    """`--mode` as the builders take it: one token, or None."""
    mode = getattr(args, "mode", None)
    if isinstance(mode, list):
        return mode[0] if len(mode) == 1 else (mode or None)
    return mode

def _accepted(fn) -> set:
    """The keyword arguments a hub function takes (hubs differ)."""
    import inspect
    try:
        return set(inspect.signature(fn).parameters)
    except (TypeError, ValueError):
        return set()
