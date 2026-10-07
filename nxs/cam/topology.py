# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Port-set topology loading. A port is one host CSI receiver: its I2C bus,
its deserializer, and the GMSL links behind it; a port that names no
deserializer is direct, its one link the sensor wired to the host. Files
are strict-parsed; without a file the manifest, the platform, then the
hub default apply."""

from __future__ import annotations

import dataclasses

from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

from nxs.suite.schema_ports import port_signature

from .contracts import LinkSpec, NxsUnitSpec, SyncSpec, Topology
from .descriptors import to_int
from . import hubs

_CARD_KEYS = {"carrier", "i2c_bus", "des_addr", "des_compatible",
              "csi_lanes", "sync", "links", "node_addrs"}
_LINK_KEYS = {"name", "compatible", "ser_compatible", "des_window",
              "csi_vc", "ser_addr", "sensor_addr", "tca_addr", "capture_id",
              "nxs_units", "mode", "inck_hz"}
_SYNC_KEYS = {"source", "fps"}
#: The frame-sync sources the tool composes; anything else is a typo.
_SYNC_SOURCES = ("free_run", "fsync")
_UNIT_KEYS = {"alias_addr", "target_addr"}


class TopologyError(ValueError):
    """A port-set file is malformed."""


def _require(condition: bool, where: str, message: str) -> None:
    if not condition:
        raise TopologyError(f"{where}: {message}")


#: What only a link behind a deserializer carries.
_SERDES_LINK_KEYS = ("ser_compatible", "des_window", "ser_addr", "tca_addr")


def _parse_link(raw: Dict[str, Any], where: str, direct: bool = False) -> LinkSpec:
    unknown = set(raw) - _LINK_KEYS
    _require(not unknown, where, f"unknown keys {sorted(unknown)}")
    if direct:
        stray = sorted(k for k in _SERDES_LINK_KEYS if k in raw)
        _require(not stray, where,
                 f"{stray} describe a link behind a deserializer, and the port "
                 f"names no des_compatible")
        raw = dict(raw, csi_vc=raw.get("csi_vc", 0))
        # The sensor's own lanes carry virtual channel 0, and the overlay
        # describes that channel.
        _require(to_int(raw["csi_vc"]) == 0, where,
                 f"csi_vc {raw['csi_vc']} on the sensor's own lanes, which carry "
                 f"virtual channel 0")
        required = ("name", "compatible")
    else:
        required = ("name", "compatible", "ser_compatible", "des_window", "csi_vc")
    for key in required:
        _require(key in raw, where, f"missing {key!r}")
    units = []
    for i, unit in enumerate(raw.get("nxs_units") or []):
        uwhere = f"{where}.nxs_units[{i}]"
        unknown = set(unit) - _UNIT_KEYS
        _require(not unknown, uwhere, f"unknown keys {sorted(unknown)}")
        _require("alias_addr" in unit, uwhere, "missing alias_addr")
        if direct:
            # Nothing translates on the port's own bus: the alias is the address.
            target = to_int(unit.get("target_addr", unit["alias_addr"]))
            _require(to_int(unit["alias_addr"]) == target, uwhere,
                     f"alias_addr {unit['alias_addr']} differs from target_addr "
                     f"{target:#x}, and the port names no des_compatible to "
                     f"translate it")
        if direct and raw.get("sensor_addr") is not None:
            # Two devices at one address on the port's own bus take each
            # other's writes.
            _require(to_int(unit["alias_addr"]) != to_int(raw["sensor_addr"]), uwhere,
                     f"alias_addr {to_int(unit['alias_addr']):#04x} is the sensor's "
                     f"address too, and nothing on the port's own bus tells the two "
                     f"apart")
        units.append(NxsUnitSpec(
            alias_addr=to_int(unit["alias_addr"]),
            # On the port's own bus the unit's address is its own.
            target_addr=to_int(unit.get("target_addr",
                                        unit["alias_addr"] if direct else 0x30)),
        ))
    # Integers as the schema admits them: a number, or a string in any
    # base-0 spelling ("0x21").
    return LinkSpec(
        name=str(raw["name"]),
        des_window=None if direct else to_int(raw["des_window"]),
        csi_vc=to_int(raw["csi_vc"]),
        sensor_compatible=str(raw["compatible"]),
        ser_compatible=None if direct else str(raw["ser_compatible"]),
        ser_addr=to_int(raw.get("ser_addr", 0x42)),
        sensor_addr=(to_int(raw["sensor_addr"]) if raw.get("sensor_addr")
                     is not None else None),
        tca_addr=to_int(raw.get("tca_addr", 0x20)),
        capture_id=(to_int(raw["capture_id"]) if raw.get("capture_id") is not None
                  else None),
        nxs_units=tuple(units),
        mode=(str(raw["mode"]) if raw.get("mode") else None),
        inck_hz=(to_int(raw["inck_hz"]) if raw.get("inck_hz") is not None
                 else None),
    )


def _require_separable_units(keyed, where: str) -> None:
    """Two units behind one hub answer apart only through their aliases: a
    unit at its strapped address answers on every link the hub merges, and
    two units at one alias merge the same way."""
    units = [(f"{where}.links.{key}.nxs_units[{i}]", unit)
             for key, link in keyed for i, unit in enumerate(link.nxs_units)]
    if len(units) < 2:
        return
    claimed = {}
    for uwhere, unit in units:
        _require(unit.alias_addr != unit.target_addr, uwhere,
                 f"alias_addr {unit.alias_addr:#04x} is the address the unit "
                 f"straps, which answers on every link, and {len(units)} units "
                 f"ride this hub")
        _require(unit.alias_addr not in claimed, uwhere,
                 f"alias_addr {unit.alias_addr:#04x} is also "
                 f"{claimed.get(unit.alias_addr)}'s, and two units at one alias "
                 f"merge")
        claimed[unit.alias_addr] = uwhere


def _parse_port(raw: Dict[str, Any], where: str) -> Topology:
    unknown = set(raw) - _CARD_KEYS
    _require(not unknown, where, f"unknown keys {sorted(unknown)}")
    for key in ("carrier", "i2c_bus"):
        _require(key in raw, where, f"missing {key!r}")
    direct = raw.get("des_compatible") is None
    if direct:
        _require("des_addr" not in raw, where,
                 "des_addr is a deserializer's address, and the port names no "
                 "des_compatible")
    links = []
    keyed = []
    for key, spec in sorted(
        (raw.get("links") or {}).items(), key=lambda kv: int(kv[0])
    ):
        links.append(_parse_link(spec, f"{where}.links.{key}", direct=direct))
        keyed.append((key, links[-1]))
    # One CSI receiver takes one sensor's lanes; only a hub multiplexes two.
    _require(not direct or len(links) == 1, where,
             f"the port's receiver takes one sensor's lanes, got {len(links)} links")
    if not direct:
        _require_separable_units(keyed, where)
    sync_raw = raw.get("sync") or {}
    unknown = set(sync_raw) - _SYNC_KEYS
    _require(not unknown, f"{where}.sync", f"unknown keys {sorted(unknown)}")
    source = str(sync_raw.get("source", "free_run"))
    _require(source in _SYNC_SOURCES, f"{where}.sync.source",
             f"{source!r} (one of {', '.join(_SYNC_SOURCES)})")
    if sync_raw.get("fps") is not None:
        _require(isinstance(sync_raw["fps"], (int, float)) and not isinstance(sync_raw["fps"], bool)
                 and float(sync_raw["fps"]) > 0,
                 f"{where}.sync.fps", f"{sync_raw['fps']!r} is not a positive rate")
    node_addrs = _node_addr_pairs(raw.get("node_addrs"))
    if not direct:
        links = [_at_node_address(link, node_addrs) for link in links]
    return Topology(
        carrier=str(raw["carrier"]),
        i2c_bus=str(raw["i2c_bus"]),
        links=tuple(links),
        des_compatible=None if direct else str(raw["des_compatible"]),
        des_addr=to_int(raw.get("des_addr", 0x6A)),
        csi_lanes=to_int(raw.get("csi_lanes", 4)),
        sync=SyncSpec(
            source=str(sync_raw.get("source", "free_run")),
            fps=sync_raw.get("fps"),
        ),
        node_addrs=node_addrs,
    )


def _node_addr_pairs(addrs) -> Tuple[Tuple[int, int], ...]:
    """The booted tree's {vc: node address} as the topology carries it."""
    return tuple(sorted((to_int(vc), to_int(addr))
                        for vc, addr in (addrs or {}).items()))


def _at_node_address(link: LinkSpec, node_addrs: Tuple[Tuple[int, int], ...]) -> LinkSpec:
    """The link with the host address of its sensor: its capture node's,
    for the channel it rides; none when the tree names no node for it."""
    for vc, addr in node_addrs:
        if int(vc) == int(link.csi_vc):
            return dataclasses.replace(link, host_addr=int(addr))
    return link


def _ports_from_suite() -> Optional[Tuple[Dict[int, Topology], int]]:
    """Ports from suite.yaml `ports:` (the camera ports: a hub's, and the
    direct ones, which name no hub and carry their one link); None when the
    manifest is absent or declares none. A manifest that does not parse
    raises TopologyError, never falls back."""
    import os

    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config

    path = default_config_path()
    if not os.path.exists(path):
        return None
    try:
        cfg = load_suite_config(path)
    except ManifestError as exc:
        raise TopologyError(f"{path}: {exc}") from exc
    cameras = {n: p for n, p in cfg.ports.items() if p.hub_compatible or p.links}
    if not cameras:
        return None
    from nxs import host as host_layer
    host = host_layer.current()
    try:
        return {index: port_topology(cameras[name], host)
                for index, name in enumerate(sorted(cameras))}, 0
    except TopologyError as exc:
        raise TopologyError(f"{path}: {exc}") from exc


#: The channel each hub link rides and its window when no hub states
#: them; the serializer is the hub's alone.
_LINK_VC = {"A": 1, "B": 0}
_LINK_WINDOW = {"A": 0x21, "B": 0x22}


def _hub_rules(hub_compatible: str) -> Tuple[Optional[str], Dict[str, int], Dict[str, int]]:
    """The hub's rules for the links behind `hub_compatible`: its
    serializer chip, the window per link and the channel per link; the
    defaults above when no hub serves the hub."""
    try:
        found = hubs.discover()
    except hubs.HubError:
        found = []
    for hub in found:
        try:
            desd = hub.descriptor(hub_compatible)
        except Exception:        # noqa: BLE001 (another hub's chip)
            continue
        if desd.role != "DES":
            continue
        ser = next((hub.descriptor(c).compatible for c in hub.chips
                    if hub.descriptor(c).role == "SER"), None)
        windows = {str(k): to_int(v) for k, v in (desd.raw("windows") or {}).items()}
        try:
            vcs = {str(k): int(v) for k, v in dict(hub.flows().LINK_VC).items()}
        except Exception:        # noqa: BLE001 (a hub whose flows name none)
            vcs = dict(_LINK_VC)
        return ser, windows or dict(_LINK_WINDOW), vcs
    return None, dict(_LINK_WINDOW), dict(_LINK_VC)


def port_topology(port, host=None) -> Topology:
    """The port a manifest port declares, as the flows take it, carrying the
    declaration's digest (`declared`); a port spec stands alone and is never
    re-resolved by name. What the declaration leaves out is the host's fact
    (the bus) or the hub's rule (the serializer, the window and the channel
    of a link behind the hub)."""
    if host is None:
        from nxs import host as host_layer
        host = host_layer.current()
    bus = port.bus or host.camera_buses().get(port.name)
    if not bus:
        # A host that boots no camera bus gets them from `nxs switch`.
        raise TopologyError(
            f"ports.{port.name}: no bus on this host answers to that name "
            f"(the host's ports: {', '.join(sorted(host.camera_buses())) or 'none'})"
            + ("\n  - nxs switch" if host.camera_bus_missing() else ""))
    ser_rule, windows, vcs = (_hub_rules(port.hub_compatible) if port.hub_compatible
                              else (None, {}, {}))
    if port.hub_compatible and ser_rule is None:
        # No hub names the serializer: a link that names its own stands in
        # for the others, else the hub default the hub's flows are built on.
        ser_rule = next((l.ser for l in port.links if l.ser), None)
    for l in port.links:
        if port.hub_compatible and l.des_window is None and l.name not in windows:
            raise TopologyError(f"ports.{port.name}.links.{l.name}: no window rule for "
                                f"a link named {l.name!r} (the hub knows "
                                f"{', '.join(sorted(windows))}); declare des_window")
        if l.csi_vc is None and l.name not in vcs:
            raise TopologyError(f"ports.{port.name}.links.{l.name}: no channel rule for "
                                f"a link named {l.name!r}; declare csi_vc")
    # A declared value is the operator's word (check compares it with the booted
    # tree); an undeclared one follows the tree, then the connector's wiring.
    booted_ids = host.capture_ids(bus)
    node_addrs = _booted_node_addrs(host, bus)
    lanes = port.csi_lanes
    if not port.csi_lanes_declared:
        lanes = host.booted_lanes(bus) or _connector_lanes(host, port.name) or port.csi_lanes
    links = tuple(
        LinkSpec(
            name=l.name,
            des_window=(l.des_window if l.des_window is not None
                        else (windows.get(l.name) if port.hub_compatible else None)),
            csi_vc=(l.csi_vc if l.csi_vc is not None else vcs.get(l.name, 0)),
            sensor_compatible=l.camera,
            sensor_declared=True,
            ser_compatible=(l.ser or ser_rule) if port.hub_compatible else None,
            ser_addr=l.ser_addr,
            sensor_addr=l.sensor_addr,
            tca_addr=l.tca_addr,
            # Capture ids are the booted tree's to give: a frozen id is a
            # snapshot of some earlier boot, and the node order moves with
            # the label. A declared id only stands where the tree is silent.
            capture_id=(booted_ids.get(int(l.csi_vc if l.csi_vc is not None
                                           else vcs.get(l.name, 0)))
                      if booted_ids else l.capture_id),
            # Nothing translates a direct port's unit: it answers at its own address.
            nxs_units=(NxsUnitSpec(alias_addr=(l.unit.alias if port.hub_compatible
                                               else l.unit.target),
                                   target_addr=l.unit.target),)
            if l.unit else (),
            mode=l.camera_mode,
            fps=l.camera_fps,
            inck_hz=l.inck_hz,
        )
        for l in port.links if l.camera or (l.unit and port.hub_compatible)
    )
    pairs = _node_addr_pairs(node_addrs)
    if port.hub_compatible:
        links = tuple(_at_node_address(link, pairs) for link in links)
    return Topology(
        carrier=f"ports/{port.name}",
        i2c_bus=bus,
        links=links,
        des_compatible=port.hub_compatible,
        des_addr=port.hub_addr,
        csi_lanes=lanes,
        hub_driver=port.hub_driver,
        camera_mode=port.camera_mode,
        camera_fps=port.camera_fps,
        camera_exposure_us=port.camera_exposure_us,
        camera_gain_db=port.camera_gain_db,
        sync=SyncSpec(source=port.sync_source, fps=port.sync_fps),
        node_addrs=pairs,
        declared=port_signature(port),
    )


def _connector_lanes(host, port: str) -> Optional[int]:
    """The lane count the port's connector wires; None from a host that
    does not say."""
    read = getattr(host, "connector_lanes", None)
    return read(port) if read is not None else None


def _booted_node_addrs(host, bus: str) -> Dict[int, int]:
    """The booted tree's capture node address per virtual channel; {} from
    a host that does not read the tree."""
    read = getattr(host, "node_addrs", None)
    return dict(read(bus)) if read is not None else {}


def _ports_from_platform() -> Optional[Tuple[Dict[int, Topology], int]]:
    """One port per platform camera port, shaped by the hub's default port: the
    port name resolves the bus, the hub supplies the chip family, and the hub
    is verified by silicon id at first contact."""
    from nxs import host as host_layer

    buses = host_layer.current().camera_buses()
    if not buses:
        return None
    for hub in hubs.discover():
        default = hub.topology_path()
        if default is not None:
            break
    else:
        return None
    raw = _load_topology_document(str(default))
    if "ports" in raw:
        base = (raw.get("ports") or {}).get(
            sorted(raw["ports"])[0]) if raw.get("ports") else None
        if base is None:
            return None
        pack_ports = list((raw.get("ports") or {}).values())
    else:
        base = raw
        pack_ports = [raw]
    # A port keeps its own hub port (links, capture ids, lanes differ per
    # port); the first port only shapes port names the hub does not carry.
    by_name = {
        str(c.get("carrier", "")).rsplit("/", 1)[-1]: c for c in pack_ports
    }
    ports: Dict[int, Topology] = {}
    for index, (name, bus) in enumerate(sorted(buses.items())):
        shaped = dict(by_name.get(name, base))
        shaped["carrier"] = f"platform/{name}"
        shaped["i2c_bus"] = bus
        _follow_booted_tree(shaped, bus)
        ports[index] = _parse_port(shaped, f"platform:{name}")
    return ports, 0


def _follow_booted_tree(shaped: Dict[str, Any], bus: str) -> None:
    """The booted overlay decides the CSI lane count and the capture ids; where
    the tree is silent the connector's lane count and the port's other values
    stand, where it speaks a link whose virtual channel has no capture node
    gets none."""
    from nxs import host as host_layer

    host = host_layer.current()
    port = str(shaped.get("carrier", "")).rsplit("/", 1)[-1]
    lanes = host.booted_lanes(bus) or _connector_lanes(host, port)
    if lanes is not None:
        shaped["csi_lanes"] = lanes
    node_addrs = _booted_node_addrs(host, bus)
    if node_addrs:
        shaped["node_addrs"] = node_addrs
    ids = host.capture_ids(bus)
    if ids:
        links = {}
        for key, spec in (shaped.get("links") or {}).items():
            spec = dict(spec)
            vc = spec.get("csi_vc")
            if vc is not None:
                spec["capture_id"] = ids.get(to_int(vc))
            links[key] = spec
        shaped["links"] = links


def discover_ports() -> Tuple[Dict[int, Topology], int]:
    """The ports as the platform and the hubs have them, never the manifest:
    the booted tree on the platform's buses, else the hub's default topology.
    Raises hubs.HubError when no hub provides a topology either."""
    ports = _ports_from_platform()
    if ports is not None:
        return ports
    for hub in hubs.discover():
        default = hub.topology_path()
        if default is not None:
            return load_ports(str(default))
    searched = ", ".join(str(p) for p in hubs.search_paths())
    raise hubs.HubError(
        f"no camera ports found on this platform and no hub "
        f"provides a default topology; searched: {searched}")


#: `cards`/`default_card` are the hub api 1 spellings of `ports`/`default_port`;
#: still accepted, renamed at load.
_CARD_ALIASES = {"cards": "ports", "default_card": "default_port"}


def _accept_card_spelling(raw: Dict[str, Any], path: str) -> Dict[str, Any]:
    """The api-1 `cards`/`default_card` keys under their current names."""
    if not any(old in raw for old in _CARD_ALIASES):
        return raw
    renamed = dict(raw)
    for old, new in _CARD_ALIASES.items():
        if old not in renamed:
            continue
        _require(new not in renamed, f"{path}:{old}",
                 f"file carries both {old} and {new}; keep {new}")
        renamed[new] = renamed.pop(old)
    return renamed


def _load_topology_document(path: str) -> Dict[str, Any]:
    """A topology file as a mapping the shipped contract accepts, the one door
    for every loader; a malformed file is a TopologyError naming file and key."""
    raw = yaml.safe_load(Path(path).read_text())
    _require(isinstance(raw, dict), path,
             "expected a mapping (a port set under `ports:` or one port), "
             f"got {type(raw).__name__ if raw is not None else 'an empty file'}")
    raw = _accept_card_spelling(raw, path)
    from nxs import schemas
    problems = schemas.findings(raw, schemas.TOPOLOGY, where=path)
    _require(not problems, path, "; ".join(problems))
    return raw


def load_ports(path: Optional[str]) -> Tuple[Dict[int, Topology], int]:
    """Load a port set from ``path``, or without one from the manifest, the
    platform, then the hub default. Returns (ports by index, default port
    index); raises TopologyError on a malformed file, hubs.HubError with none."""
    if not path:
        ports = _ports_from_suite()
        if ports is not None:
            return ports
        ports = _ports_from_platform()
        if ports is not None:
            return ports
        for hub in hubs.discover():
            default = hub.topology_path()
            if default is not None:
                path = str(default)
                break
        else:
            searched = ", ".join(str(p) for p in hubs.search_paths())
            raise hubs.HubError(
                "no topology given and no hub provides a "
                f"default; searched: {searched}. Install a hub or set "
                f"${hubs.HUBS_ENV}."
            )
    raw = _load_topology_document(path)
    if "ports" in raw:
        ports = {
            int(key): _parse_port(value, f"{path}:ports.{key}")
            for key, value in (raw.get("ports") or {}).items()
        }
        _require(bool(ports), path, "ports is empty")
        default = to_int(raw.get("default_port", sorted(ports)[0]))
        _require(default in ports, f"{path}:default_port",
                 f"{default} names no port (ports: {sorted(ports)})")
        return ports, default
    return {0: _parse_port(raw, path)}, 0
