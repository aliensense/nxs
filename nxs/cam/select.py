"""Selecting a port and its links from the node grammar, the declarations that ride with them, and the descriptor a link runs."""

from __future__ import annotations

import argparse
import dataclasses
from typing import Any, Dict, List, Tuple



from nxs.cam.contracts import LinkSpec, Topology
from nxs.cam import port_state
from nxs.cam import packs, topology as topo_mod, unit_source


def _refuse(fact: str, *alternatives: str) -> SystemExit:
    """A refusal to raise: the fact, then one `  - ` line per alternative."""
    return SystemExit("\n".join([fact, *(f"  - {alt}" for alt in alternatives)]))


def _role_code(device: int) -> str:
    return {0x6A: "DES", 0x42: "SER", 0x1A: "SEN"}.get(device, f"0x{device:02X}")


def print_stream(stream: List[tuple]) -> None:
    """Print a normalized plan stream with device codenames."""
    for step in stream:
        kind = step[0]
        if kind == "w":
            _, device, reg16, values = step
            vals = ",".join(f"{v:02X}" for v in values)
            print(f"  {_role_code(device)} 0x{reg16:04X} = {vals}")
        elif kind == "r":
            _, device, reg16, length = step
            print(f"  {_role_code(device)} 0x{reg16:04X} ? read{length}")
        elif kind == "e":
            _, device, reg16, mask, op, value = step
            print(f"  {_role_code(device)} 0x{reg16:04X} expect "
                  f"&0x{mask:02X} {op} 0x{value:02X}")


# ---------------------------
# Selection
# ---------------------------

def _resolve_links(topology: Topology, tokens: List[str]) -> List[LinkSpec]:
    if not tokens:
        return list(topology.links)
    picked: List[LinkSpec] = []
    for token in tokens:
        if token.isdigit():
            index = int(token)
            if index >= len(topology.links):
                raise SystemExit(
                    f"link index {index} out of range "
                    f"(port has {len(topology.links)} links)"
                )
            link = topology.links[index]
        else:
            for link in topology.links:
                if link.name.upper() == token.upper():
                    break
            else:
                raise SystemExit(f"no link named {token!r} on this port")
        if link in picked:
            raise SystemExit(f"link {link.name} selected twice")
        picked.append(link)
    # Topology order, not user order: downstream zips (save_port with a
    # dual flow's CSI channels) assume it.
    return [l for l in topology.links if l in picked]


def _port_name(topology: Topology) -> str:
    """A port's short name: the carrier suffix (pixelmate/cam1 -> cam1)."""
    return topology.carrier.split("/")[-1].lower()


def _port_key(ports: Dict[int, Topology], token: str) -> int:
    """Resolve a port selector: an index or a port name."""
    t = str(token).strip().lower()
    if t.lstrip("-").isdigit():
        index = int(t)
        if index in ports:
            return index
    else:
        for index, topo in ports.items():
            if _port_name(topo) == t:
                return index
    have = ", ".join(f"{_port_name(t)} ({i})" for i, t in sorted(ports.items()))
    raise SystemExit(f"no port {token!r}; have {have}")


def select_port_links(
    args: argparse.Namespace,
    require_port: bool = False,
) -> Tuple[Topology, List[LinkSpec]]:
    """Resolve (port, links) from selectors: `--port cam1`, a bare port-name
    token, or `port:link` (`cam1:B`, `1:0`); bare digits and letter names are
    link selectors. With `require_port` the port must be named explicitly."""
    try:
        ports, default_port = topo_mod.load_ports(
            getattr(args, "topology", None)
        )
    except (topo_mod.TopologyError, packs.PackError) as exc:
        raise SystemExit(f"nxs: {exc}")
    port = getattr(args, "port", None)
    if port is not None:
        port = _port_key(ports, str(port))

    def claim(explicit: int) -> None:
        nonlocal port
        if port is not None and explicit != port:
            raise SystemExit("one command addresses one port")
        port = explicit

    names = {_port_name(t) for t in ports.values()}
    tokens: List[str] = []
    for token in getattr(args, "links", None) or []:
        if ":" in token:
            port_part, link_part = token.split(":", 1)
            claim(_port_key(ports, port_part))
            if link_part:
                tokens.append(link_part)
        elif not token.isdigit() and token.lower() in names:
            claim(_port_key(ports, token))
        else:
            tokens.append(token)
    if port is None:
        if require_port:
            have = ", ".join(
                _port_name(t) for _, t in sorted(ports.items()))
            hint = tokens[0] if tokens else "A"
            verb = getattr(args, "cam_cmd", None) or "on"
            raise _refuse(f"this verb needs a port (ports: {have})",
                          f"nxs {_port_name(ports[default_port])} {hint} {verb}")
        if tokens:
            # A bare link selector must name exactly one port's link.
            owners = [i for i, topo in sorted(ports.items())
                      if all(_owns_link(topo, t) for t in tokens)]
            if len(owners) == 1:
                port = owners[0]
            elif owners:
                have = ", ".join(_port_name(ports[i]) for i in owners)
                raise _refuse(
                    f"link selector {' '.join(tokens)!r} is ambiguous ({have})",
                    f"nxs {_port_name(ports[owners[0]])} {tokens[0]} …")
            else:
                port = default_port
        else:
            port = default_port
    topology = _remembered_sensors(ports[port])
    # Verbs that behave differently for `stream` vs `stream B` need to
    # know whether any token picked a link (ports don't count).
    args._link_tokens = list(tokens)
    return topology, _resolve_links(topology, tokens)


def _owns_link(topology: Topology, token: str) -> bool:
    if token.isdigit():
        return int(token) < len(topology.links)
    return any(l.name.upper() == token.upper() for l in topology.links)


def _refuse_pod_only(topology: Topology, link: LinkSpec, verb: str) -> SystemExit:
    """A pod alone carries no camera: the camera verbs refuse it and name
    the unit verb that reaches the pod."""
    from nxs.cam.verbs.status import _pod_names

    port = _port_name(topology)
    who = _pod_names(topology).get(link.name) or "<the pod's name>"
    return _refuse(f"{port}/{link.name}: link {link.name} carries a pod and no camera",
                   f"nxs --unit {who} {verb}")


def _require_up(topology: Topology, links: List[LinkSpec],
                verb: str) -> None:
    """Refuse a running-stream verb on links the port record says are not up."""
    port = _port_name(topology)
    down = [(l, port_state.link_state(topology, l)) for l in links]
    down = [(l, s) for l, s in down if s != port_state.STATE_UP]
    if down:
        detail = ", ".join(f"{l.name} is {s}" for l, s in down)
        need = " ".join(l.name for l, _ in down)
        raise _refuse(f"nxs: {verb} needs the link up: {detail} on {port}",
                      f"nxs {port} {need} on")


def _pack_for(topology: Topology):
    try:
        return packs.pack_for(topology)
    except packs.PackError as exc:
        raise SystemExit(f"nxs: {exc}")


def _per_link(values, links: List[LinkSpec], what: str) -> Dict[str, Any]:
    """Pair repeated flag values with the selected links in order: one
    value for every link, or exactly one per link."""
    if not values:
        return {}
    if isinstance(values, str):
        values = [values]
    if len(values) == 1:
        return {l.name: values[0] for l in links}
    if len(values) != len(links):
        names = " ".join(l.name for l in links) or "(none)"
        raise _refuse(
            f"nxs: {len(values)} --{what} values for {len(links)} selected "
            f"link(s) ({names})",
            "one value for every link, or one per link in order")
    return {l.name: v for l, v in zip(links, values)}


def _with_sensors(pack, topology: Topology, links: List[LinkSpec],
                  sensors: Dict[str, str]) -> Tuple[Topology, List[LinkSpec]]:
    """The topology with the named sensors on the selected links (a pack
    sensor name or compatible, checked against the pack)."""
    if not sensors:
        return topology, links
    resolved: Dict[str, str] = {}
    for name, token in sensors.items():
        try:
            resolved[name] = pack.descriptor(str(token)).compatible
        except packs.PackError:
            have = ", ".join(pack.sensors())
            raise SystemExit(
                f"nxs: no sensor {token!r} in pack {pack.name!r} "
                f"(sensors: {have})")
    new_links = tuple(
        dataclasses.replace(l, sensor_compatible=resolved[l.name])
        if l.name in resolved else l
        for l in topology.links
    )
    topology = dataclasses.replace(topology, links=new_links)
    return topology, [topology.link(l.name) for l in links]


def _remembered_sensors(topology: Topology) -> Topology:
    """The topology with the sensors remembered for links the manifest
    leaves undeclared: the personality the link's unit served (its cached
    descriptor), else the port record; a manifest-declared sensor always
    wins."""
    changed = False
    new_links = []
    for link in topology.links:
        known = None
        if not link.sensor_declared:
            cached = (unit_source.cached_descriptor(topology, link)
                      if link.nxs_units else None)
            known = (cached.compatible if cached is not None
                     else port_state.port_sensor(topology, link))
        if known and known != link.sensor_compatible:
            new_links.append(dataclasses.replace(link, sensor_compatible=known))
            changed = True
        else:
            new_links.append(link)
    if not changed:
        return topology
    return dataclasses.replace(topology, links=tuple(new_links))


def _declare(pack, topology: Topology, links: List[LinkSpec], args
             ) -> Tuple[Topology, List[LinkSpec], Dict[str, str]]:
    """Apply the command's declarations: `--sensor` per selected link
    (else what the port record remembers), `--mode` per selected link. Returns
    the topology, the selected links, and the per-link mode tokens."""
    sensors = _per_link(getattr(args, "sensor", None), links, "sensor")
    topology, links = _with_sensors(pack, topology, links, sensors)
    modes = _per_link(getattr(args, "mode", None), links, "mode")
    return topology, links, modes

def _link_descriptor(pack, topology: Topology, link: LinkSpec):
    """A link's sensor descriptor as this port runs it: the pack's own
    binding when its flows offer one (the line a pair runs), the plain
    descriptor otherwise."""
    bind = getattr(pack.flows(), "sensor_descriptor", None)
    if bind is not None:
        return bind(pack, link, topology)
    return pack.descriptor(link.sensor_compatible)


def _descriptor_among(pack, topology: Topology, link: LinkSpec, port=None):
    """A link's sensor descriptor as its port runs it (``port``: the names
    of the links up together, the port's when None)."""
    import dataclasses

    if port is not None:
        chosen = tuple(l for l in topology.links if l.name in set(port))
        if len(chosen) != len(topology.links):
            topology = dataclasses.replace(topology, links=chosen)
    return _link_descriptor(pack, topology, link)


