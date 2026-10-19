# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The port view of a declaration: each camera connector of the host with
its hub, its links and the pods riding them, and each unit with its routes
and its panel. It is the 1.0 form's spelling and the shape the panel's model
writes a port and a unit in. `nodes_from_view` turns a view into the nodes
and connections it declares in api 3, through the converter api-2 files
take (`convert.api3`), and `merge` lays them over a declaration's file,
which keeps every node and connection the view does not describe (a
module, a hub's second output), each node's rows and the comments of what
did not move."""

from __future__ import annotations

import math
import os
import re
from typing import Dict, List, Optional, Tuple

from nxs.suite.nodes import API, Graph, end_text
from nxs.suite.schema_base import ManifestError

#: The host node's name where the declaration names none.
HOST_NAME = "host"
#: The kinds the view describes; a module is the declaration's alone.
_VIEW_KINDS = ("host", "hub", "camera", "unit", "click")


def view_of(cfg) -> dict:
    """A parsed declaration in the port view: what the panel's model reads
    and writes, the bus and the lanes only where declared."""
    view: dict = {}
    if cfg.name:
        view["suite"] = {"name": cfg.name}
    ports = {name: port_entry(spec) for name, spec in sorted(cfg.ports.items())}
    if ports:
        view["ports"] = ports
    if cfg.units:
        view["units"] = [unit_entry(unit) for unit in cfg.units]
    return view


def port_entry(spec) -> dict:
    """One port's declaration in the view."""
    from nxs.suite.freeze import hex_address

    entry: dict = {}
    if spec.bus is not None:
        # A hub's control bus on an I²C adapter of its own; a connector's bus is the host's.
        entry["bus"] = spec.bus
    if spec.hub_compatible:
        extras = {}
        if spec.hub_addr != 0x6A:
            extras["addr"] = hex_address(spec.hub_addr)
        if spec.hub_driver != "nxs":
            extras["driver"] = spec.hub_driver
        entry["hub"] = {"compatible": spec.hub_compatible, **extras} if extras else spec.hub_compatible
    if spec.csi_lanes_declared:
        entry["csi_lanes"] = int(spec.csi_lanes)
    if spec.sync_source != "free_run" or spec.sync_fps is not None:
        entry["sync"] = {"source": spec.sync_source}
        if spec.sync_fps is not None:
            entry["sync"]["fps"] = _number(spec.sync_fps)
    camera = {k: _number(v) for k, v in (("exposure_us", spec.camera_exposure_us),
                                          ("gain_db", spec.camera_gain_db)) if v is not None}
    if camera:
        entry["camera"] = camera
    links = {}
    for link in spec.links:
        out: dict = {}
        if link.camera:
            cam = {"sensor": link.camera}
            if link.camera_mode:
                cam["mode"] = link.camera_mode
            if link.camera_fps is not None:
                cam["fps"] = _number(link.camera_fps)
            out["camera"] = cam
        if link.unit is not None:
            ref = {"name": link.unit.name}
            if spec.hub_compatible:
                ref["alias"] = hex_address(link.unit.alias)
            if link.unit.target != 0x30:
                ref["target"] = hex_address(link.unit.target)
            out["unit"] = ref
        for key in ("ser", "des_window", "csi_vc", "ser_addr", "tca_addr", "sensor_addr", "inck_hz",
                    "capture_id"):
            value = getattr(link, key)
            # A head on the connector's own lanes rides virtual channel 0, which nothing states.
            if key == "csi_vc" and not spec.hub_compatible:
                continue
            if value is not None and value != _WIRING_DEFAULTS.get(key):
                out[key] = value
        links[link.name] = out
    if links:
        entry["links"] = links
    return entry


#: A link's wiring values the node types derive when nothing states them.
_WIRING_DEFAULTS = {"ser_addr": 0x42, "tca_addr": 0x20}


def unit_entry(unit) -> dict:
    """One unit's declaration in the view."""
    from nxs.suite.freeze import hex_address

    routes = []
    for link in unit.links:
        route: dict = {"transport": link.transport}
        if link.link_ref:
            route["link"] = link.link_ref
        else:
            if link.connector:
                route["connector"] = link.connector
            for key in ("bus", "address", "iface", "node_id", "port", "baud"):
                value = getattr(link, key, None)
                if value is not None and not (key == "bus" and link.connector):
                    route[key] = hex_address(value) if key == "address" else value
        routes.append(route)
    entry: dict = {"name": unit.name, "module": unit.module, "links": routes}
    for key in ("serial", "firmware"):
        if getattr(unit, key) is not None:
            entry[key] = getattr(unit, key)
    if unit.sensors:
        entry["sensors"] = [{"personality": s.personality, **({"config": dict(s.config)}
                                                               if s.config else {})}
                            for s in unit.sensors]
    egress: dict = {}
    if unit.rows.get("decimation") is not None:
        egress["decimation"] = unit.rows["decimation"]
    subjects = {subject: unit.rows[row] for subject, row in _DECIMATION_ROWS.items()
                if row in unit.rows}
    if subjects:
        egress["subjects"] = subjects
    if egress:
        entry["egress"] = egress
    return entry


#: The unit rows of the subject factors, by the subject token the view spells.
_DECIMATION_ROWS = {"acceleration": "decim.accel", "angular_velocity": "decim.gyro",
                    "magnetic_field": "decim.mag", "temperature": "decim.temp",
                    "pressure": "decim.press", "scalar": "decim.scalar"}


def _number(value):
    """A rate or a level as a declaration spells it: whole when it is one,
    and as written when it is no number, which the loader then judges."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return value
    if math.isnan(number) or math.isinf(number):
        return value
    return int(number) if number.is_integer() else round(number, 3)


# ── the view to nodes ────────────────────────────────────────────────

class Names:
    """The names the emitted nodes take: the previous declaration's for the
    same role (the hub that feeds a connector, the camera on a link, the
    click on a unit), else the derived name."""

    def __init__(self, previous: Optional[Graph], host_type: str):
        self._previous = previous
        found = previous.host() if previous is not None else None
        self._host_type = found.type if found is not None else host_type
        self._host = found.name if found is not None else HOST_NAME

    @property
    def host(self) -> str:
        """The host node's name."""
        return self._host

    @property
    def host_type(self) -> str:
        """The host node's type, its plugin."""
        return self._host_type

    def _feed(self, node: str, port: str) -> Optional[str]:
        if self._previous is None:
            return None
        found = self._previous.source(node, port)
        return found[0] if found is not None else None

    def _kind(self, name: Optional[str], kind: str) -> Optional[str]:
        if name is None or self._previous is None:
            return None
        return name if self._previous.nodes[name].kind == kind else None

    def hub(self, port: str) -> str:
        return self._kind(self._feed(self._host, port), "hub") or f"hub-{port}"

    def camera(self, port: str, link: str, carrier: Optional[str]) -> str:
        """The camera on a link: the one feeding the link's pod or the link."""
        if carrier is not None:
            found = self._kind(self._feed(carrier, "head"), "camera")
        else:
            hub = self._kind(self._feed(self._host, port), "hub")
            found = self._kind(self._feed(hub, f"links.{link}") if hub else
                               self._feed(self._host, port), "camera")
        return found or f"camera-{port}-{link.lower()}"

    def click(self, unit: str, personality: str, taken) -> str:
        """The click of a personality on a unit: the previous declaration's
        of that type on the unit, else `click-<unit>`, numbered past the
        first; never a name in `taken`."""
        if self._previous is not None and unit in self._previous.nodes:
            for node, _port in self._previous.sources(unit, "click"):
                prev = self._previous.nodes.get(node)
                if (prev is not None and prev.kind == "click" and node not in taken
                        and prev.type.replace("-", "_") == personality.replace("-", "_")):
                    return node
        name, n = f"click-{unit}", 2
        while name in taken:
            name, n = f"click-{unit}-{n}", n + 1
        return name

    def owner(self, hub: str) -> Optional[Tuple[str, str]]:
        """The source wired into a hub's control input, as the previous
        declaration has it."""
        if self._previous is None or hub not in self._previous.nodes:
            return None
        return self._previous.source(hub, "i2c")


def nodes_from_view(view: dict, previous: Optional[Graph] = None, host_type: Optional[str] = None,
                    buses: Optional[Dict[str, str]] = None, where: str = "suite.yaml") -> dict:
    """The api-3 declaration a view states: `{api, suite, nodes,
    connections}`, through the converter (`convert.api3`). `previous` lends
    its names, `buses` maps the host's camera connectors to their bus, so a
    unit on a connector's bus rides it. ManifestError names a value of the
    view api 3 has no place for."""
    return nodes_and_roles(view, previous, host_type, buses, where)[0]


def nodes_and_roles(view: dict, previous: Optional[Graph] = None, host_type: Optional[str] = None,
                    buses: Optional[Dict[str, str]] = None,
                    where: str = "suite.yaml") -> Tuple[dict, Dict[tuple, str]]:
    """`nodes_from_view`'s declaration and the name each role took: `("hub",
    port)`, `("host", port)`, `("camera", port, link)`, `("unit", unit)`,
    `("click", unit, index)`."""
    from nxs.suite.convert import api3

    doc, roles = _api2_from_view(view, previous, host_type, buses)
    return api3(doc, where), roles


def _api2_from_view(view: dict, previous: Optional[Graph], host_type: Optional[str],
                    buses: Optional[Dict[str, str]]) -> Tuple[dict, Dict[tuple, str]]:
    """The api-2 shape of a view, the converter's input, and the roles."""
    if host_type is None:
        from nxs import host as host_layer
        host_type = host_layer.current().name
    names = Names(previous, host_type)
    emit = _Emitter(names, dict(buses or {}))
    ports, units = view.get("ports") or {}, view.get("units") or []
    if not isinstance(ports, dict) or not isinstance(units, list):
        raise ManifestError("ports: a mapping of connectors, units: a list of units",
                            ["nxs generate (walks the rig and writes the node form)"])
    for port, entry in ports.items():
        if not isinstance(entry or {}, dict):
            raise ManifestError(f"ports.{port}: not a mapping")
        emit.port(str(port), entry or {})
    for i, unit in enumerate(units):
        if not isinstance(unit, dict) or not unit.get("name"):
            raise ManifestError(f"units[{i}]: a unit is a mapping that names itself (name:)")
        emit.unit(unit)
    doc: dict = {"api": 2}
    if view.get("suite"):
        doc["suite"] = dict(view["suite"])
    if view.get("defaults"):
        doc["defaults"] = dict(view["defaults"])
    for node in emit.nodes.values():
        for key in ("parameters", "ports"):
            if key in node and not node[key]:
                del node[key]
    doc["nodes"] = emit.nodes
    doc["connections"] = emit.connections()
    return doc, emit.roles


class _Emitter:
    def __init__(self, names: Names, buses: Dict[str, str]):
        self._names = names
        self._nodes: Dict[str, dict] = {names.host: {"kind": "host", "type": names.host_type}}
        self._edges: List[Tuple[str, str]] = []
        self._buses = buses
        #: The node each role took, by role.
        self.roles: Dict[tuple, str] = {}

    @property
    def nodes(self) -> Dict[str, dict]:
        """The node entries emitted so far, by name."""
        return self._nodes

    # the ports

    def port(self, name: str, entry: dict) -> None:
        self.roles[("host", name)] = self._names.host
        if entry.get("bus"):
            self._host_fact(f"bus_{name}", "bus", entry["bus"])
            self._buses[name] = str(entry["bus"])
        if entry.get("csi_lanes") is not None:
            self._host_fact(name, "lanes", entry["csi_lanes"])
        hub = entry.get("hub")
        if isinstance(hub, str):
            hub = {"compatible": hub}
        camera = dict(entry.get("camera") or {})
        sensor_default = camera.get("sensor")
        mode_default, fps_default = _mode_token(camera.get("mode"))
        sync = dict(entry.get("sync") or {})
        if camera.get("sync"):
            sync["source"] = camera["sync"]
        synced = sync.get("source") == "fsync"
        if synced and fps_default is not None and sync.get("fps") is None:
            sync["fps"] = fps_default
        links = entry.get("links") or {}
        if hub:
            hub_name = self._hub(name, hub, sync, camera)
            for letter, link in links.items():
                self._hub_link(name, hub_name, str(letter), link or {}, sensor_default,
                               mode_default, None if synced else fps_default)
            return
        for letter, link in links.items():
            self._direct_link(name, str(letter), link or {}, sensor_default, mode_default,
                              fps_default, camera)

    def _hub(self, port: str, hub: dict, sync: dict, camera: dict) -> str:
        name = self._names.hub(port)
        self.roles[("hub", port)] = name
        params: dict = {}
        if hub.get("addr") is not None and _int(hub["addr"]) != 0x6A:
            params["address"] = hub["addr"]
        driver = hub.get("driver", "nxs")
        if driver == "kernel":
            params["driver"] = "kernel"
        if sync.get("source", "free_run") != "free_run" or sync.get("fps") is not None:
            params["sync"] = {k: (_number(v) if k == "fps" else v) for k, v in sync.items()
                              if v is not None}
        for key in ("gain_db", "exposure_us"):
            if camera.get(key) is not None:
                params[key] = _number(camera[key])
        self._nodes[name] = _node("hub", hub["compatible"], params)
        self._edge(f"{name}.csi.A", f"{self._names.host}.{port}")
        if driver == "module":
            owner = self._names.owner(name)
            if owner is not None:
                self._edge(end_text(owner), f"{name}.i2c")
        else:
            self._edge(f"{self._names.host}.bus_{port}", f"{name}.i2c")
        return name

    def _hub_link(self, port: str, hub: str, letter: str, link: dict, sensor_default,
                  mode_default, fps_default) -> None:
        from nxs.generate_seed import pod_alias

        wiring = {k: link[k] for k in ("ser", "des_window", "csi_vc", "ser_addr", "tca_addr",
                                       "capture_id") if link.get(k) is not None}
        if wiring:
            self._nodes[hub].setdefault("ports", {})[f"links.{letter}"] = wiring
        unit = _unit_ref(link.get("unit"))
        carrier = None
        if unit is not None:
            carrier = unit["name"]
            facts = {}
            if unit.get("alias") is not None and _int(unit["alias"]) != pod_alias(letter):
                facts["alias"] = unit["alias"]
            if unit.get("target") is not None and _int(unit["target"]) != 0x30:
                facts["target"] = unit["target"]
            self._unit_node(carrier, facts)
            self._edge(f"{carrier}.out", f"{hub}.links.{letter}")
        camera = self._camera(port, letter, link, carrier, sensor_default, mode_default,
                              fps_default)
        if camera is not None:
            self._edge(f"{camera}.out", f"{carrier}.head" if carrier else f"{hub}.links.{letter}")

    def _direct_link(self, port: str, letter: str, link: dict, sensor_default, mode_default,
                     fps_default, camera_entry: dict) -> None:
        if link.get("capture_id") is not None:
            self._host_fact(port, "capture_id", link["capture_id"])
        # A hub's wiring on a head of the connector's own lanes is the 1.0
        # form's mistake: carried onto the connector, whose loader refuses it.
        for key in ("ser", "des_window", "ser_addr", "tca_addr", "csi_vc"):
            if link.get(key) is not None and not (key == "csi_vc" and _int(link[key]) == 0):
                self._host_fact(port, key, link[key])
        unit = _unit_ref(link.get("unit"))
        carrier = None
        if unit is not None:
            carrier = unit["name"]
            target = unit.get("target", unit.get("alias"))
            self._unit_node(carrier, {"target": target}
                            if target is not None and _int(target) != 0x30 else {})
            self._edge(f"{carrier}.out", f"{self._names.host}.{port}")
        camera = self._camera(port, letter, link, carrier, sensor_default, mode_default,
                              fps_default)
        if camera is None:
            return
        for key in ("gain_db", "exposure_us"):
            if camera_entry.get(key) is not None:
                self._nodes[camera]["parameters"][key] = _number(camera_entry[key])
        self._edge(f"{camera}.out", f"{carrier}.head" if carrier else f"{self._names.host}.{port}")

    def _camera(self, port, letter, link, carrier, sensor_default, mode_default,
                fps_default) -> Optional[str]:
        raw = link.get("camera")
        sensor, mode, fps = sensor_default, mode_default, fps_default
        if isinstance(raw, str):
            sensor = raw
        elif isinstance(raw, dict):
            sensor = raw.get("sensor", sensor_default)
            mode = raw.get("mode", mode_default)
            fps = raw.get("fps", fps_default)
        if not sensor:
            return None
        name = self._names.camera(port, letter, carrier)
        self.roles[("camera", port, letter)] = name
        params: dict = {}
        if mode:
            params["mode"] = str(mode)
        if fps is not None:
            params["fps"] = _number(fps)
        if link.get("inck_hz") is not None:
            params["inck_hz"] = link["inck_hz"]
        if link.get("sensor_addr") is not None:
            params["address"] = link["sensor_addr"]
        self._nodes[name] = {"kind": "camera", "type": str(sensor), "parameters": params}
        return name

    # the units

    def unit(self, entry: dict) -> None:
        name = str(entry["name"])
        self.roles[("unit", name)] = name
        node = self._unit_node(name, None)
        node["type"] = str(entry.get("module", "nxs"))
        params = node["parameters"]
        for key in ("serial", "firmware", "orientation", "egress"):
            if entry.get(key) is not None:
                params[key] = entry[key]
        sensors = entry.get("sensors")
        if sensors is not None and not sensors:
            params["click"] = "none"
        for index, sensor in enumerate(sensors or []):
            personality = str(sensor.get("personality"))
            click = self._names.click(name, personality, self._nodes)
            self.roles[("click", name, index)] = click
            self._nodes[click] = {"kind": "click", "type": personality,
                                 "parameters": dict(sensor.get("config") or {})}
            self._edge(f"{click}.out", f"{name}.click")
        for route in entry.get("links") or []:
            self._route(name, route)

    def _unit_node(self, name: str, out_facts: Optional[dict]) -> dict:
        node = self._nodes.setdefault(name, {"kind": "unit", "type": "nxs", "parameters": {}})
        if out_facts:
            node.setdefault("ports", {})["out"] = dict(out_facts)
        return node

    def _route(self, unit: str, route: dict) -> None:
        transport = route.get("transport")
        host = self._names.host
        if transport == "i2c":
            if route.get("link"):
                return      # the pod's camera chain is its route
            if self._source(unit, "i2c") is not None:
                raise ManifestError(
                    f"units.{unit}: a second I²C route ({route.get('bus')}), and a unit takes "
                    f"one I²C bus", [f"keep one of {unit}'s I²C routes"])
            port = route.get("connector") or self._connector_of(route.get("bus"))
            if port is not None:
                source = f"bus_{port}"
                # A route that names the connector's bus by its path keeps it.
                if route.get("bus") and port not in self._buses:
                    self._host_fact(source, "bus", route["bus"])
                    self._buses[port] = str(route["bus"])
            else:
                bus = str(route.get("bus"))
                source = _bus_port_name(bus, "i2c")
                self._host_port(source, {"type": "i2c", "bus": bus})
            if route.get("address") is not None and _int(route["address"]) != 0x30:
                self._nodes[unit].setdefault("ports", {})["i2c"] = {"address": route["address"]}
            self._edge(f"{host}.{source}", f"{unit}.i2c")
        elif transport == "cyphal-can":
            iface = str(route.get("iface"))
            self._host_port(iface, {"type": "cyphal-can"})
            self._nodes[unit].setdefault("ports", {})["can"] = {"node_id": route.get("node_id")}
            self._edge(f"{host}.{iface}", f"{unit}.can")
        elif transport == "cyphal-serial":
            device = str(route.get("port"))
            source = _bus_port_name(device, "serial")
            facts = {"type": "cyphal-serial", "port": device}
            if route.get("baud") is not None:
                facts["baud"] = route["baud"]
            self._host_port(source, facts)
            self._edge(f"{host}.{source}", f"{unit}.serial")

    def _connector_of(self, bus) -> Optional[str]:
        """The camera connector whose bus a route names, by the path or the
        node behind it."""
        if not bus:
            return None
        real = os.path.realpath(str(bus))
        for port, path in self._buses.items():
            if path and (path == bus or os.path.realpath(path) == real):
                return port
        found = re.match(r"^/dev/i2c-(cam\d+)$", str(bus))
        return found.group(1) if found else None

    # the host and the connections

    def _host_fact(self, port: str, key: str, value) -> None:
        self._nodes[self._names.host].setdefault("ports", {}).setdefault(port, {})[key] = value

    def _host_port(self, port: str, facts: dict) -> None:
        self._nodes[self._names.host].setdefault("ports", {}).setdefault(port, dict(facts))

    def _source(self, node: str, port: str) -> Optional[str]:
        return next((s for s, t in self._edges if t == f"{node}.{port}"), None)

    def _edge(self, source: str, target: str) -> None:
        if (source, target) not in self._edges:
            self._edges.append((source, target))

    def connections(self) -> dict:
        out: Dict[str, list] = {}
        for source, target in self._edges:
            out.setdefault(source, []).append(target)
        return out


def _node(kind: str, node_type: str, params: dict) -> dict:
    node: dict = {"kind": kind, "type": node_type}
    if params:
        node["parameters"] = params
    return node


def _int(value) -> Optional[int]:
    """An integer as the 1.0 form spelled it, in any base or quoted; None
    for what is none, which the node form then states as written and its
    loader judges."""
    try:
        return int(value, 0) if isinstance(value, str) else int(value)
    except (TypeError, ValueError):
        return None


def _unit_ref(raw) -> Optional[dict]:
    if raw is None:
        return None
    return {"name": raw} if isinstance(raw, str) else dict(raw)


def _mode_token(raw) -> Tuple[Optional[str], object]:
    """A port's `camera.mode` of the 1.0 form: a mode name or a geometry,
    with an optional `@fps`."""
    token = str(raw or "").strip()
    if not token:
        return None, None
    if "@" in token:
        token, fps = token.rsplit("@", 1)
        return token or None, _number(fps)
    return token, None


def _bus_port_name(path: str, prefix: str) -> str:
    """A host port's name for a bus device: its node's name, `i2c-7` or
    `ttyACM0`, the characters a port's name takes."""
    base = os.path.basename(path.rstrip("/")) or prefix
    return re.sub(r"[^A-Za-z0-9_-]+", "-", base)



# ── the nodes into the file ──────────────────────────────────────────

def _connectors(graph: Graph) -> set:
    """The host's camera connectors the graph's host offers, by name."""
    from nxs.suite.schema_nodes import _is_connector

    host = graph.host()
    if host is None:
        return set()
    return {port.name for port in host.description.ports if _is_connector(host, port.name)}


def chain_scope(graph: Optional[Graph], connectors=None) -> set:
    """The host and every node on a camera chain into one of its
    connectors (those of `connectors` alone where it names some): what a
    port freeze writes."""
    if graph is None:
        return set()
    host = graph.host()
    scope = {host.name} if host is not None else set()
    offered = _connectors(graph)
    frontier = [e.source for e in graph.edges
                if host is not None and e.target[0] == host.name and e.target[1] in offered
                and (connectors is None or e.target[1] in connectors)]
    while frontier:
        node, _port = frontier.pop()
        if node in scope or graph.nodes[node].kind not in _VIEW_KINDS:
            continue
        scope.add(node)
        frontier += [e.source for e in graph.edges
                     if e.target[0] == node and _CHAIN_INPUT.match(e.target[1])]
    return scope


#: The inputs a camera chain runs through: a hub's links and a pod's head.
_CHAIN_INPUT = re.compile(r"^(links\.[AB]|head)$")


def view_scope(graph: Optional[Graph]) -> set:
    """The nodes of a declaration the view describes: the camera chains
    into the host's connectors, every unit and the clicks on it. A module,
    and anything on a chain that reaches no connector of the host, is the
    declaration's alone."""
    if graph is None:
        return set()
    scope = chain_scope(graph)
    for node in graph.nodes.values():
        if node.kind == "unit":
            scope.add(node.name)
            scope.update(click for click, _port in graph.sources(node.name, "click"))
    return scope


def kept_parameters(emitted: dict, previous: Optional[Graph], owned=()) -> None:
    """Each emitted node the previous declaration also names keeps the
    parameters the file gives it, in place: the view spells a few of a
    block's rows, and a node is the file's to keep but where the caller
    owns it (`owned`, the nodes whose rows a panel or a freeze sets)."""
    if previous is None:
        return
    for name, entry in emitted["nodes"].items():
        node = previous.nodes.get(name)
        if node is None or name in owned or node.type != entry.get("type"):
            continue
        if node.parameters:
            entry["parameters"] = dict(node.parameters)
        else:
            entry.pop("parameters", None)


def merge(doc, emitted: dict, previous: Optional[Graph]) -> None:
    """Lay the emitted declaration over the file's document in place. A node
    the view describes takes the emitted entry, key by key, so a value that
    did not move keeps its comment; one the view no longer names goes.
    Every node and connection outside the view stays."""
    scope = view_scope(previous)
    doc["api"] = API
    doc.pop("defaults", None)
    if emitted.get("suite"):
        doc["suite"] = emitted["suite"]
    else:
        doc.pop("suite", None)
    nodes = doc.setdefault("nodes", {})
    for name in [n for n in nodes if n in scope and n not in emitted["nodes"]]:
        del nodes[name]
    for name, entry in emitted["nodes"].items():
        current = nodes.get(name)
        if isinstance(current, dict):
            _assign(current, entry)
        else:
            nodes[name] = entry
    kept = []
    for source, targets in (doc.get("connections") or {}).items():
        for target in targets or []:
            ends = (str(source).split(".", 1)[0], str(target).split(".", 1)[0])
            if all(end in scope for end in ends):
                continue
            if all(end in nodes for end in ends):
                kept.append((str(source), str(target)))
    connections: Dict[str, list] = {}
    for source, targets in emitted["connections"].items():
        connections[source] = list(targets)
    for source, target in kept:
        if target not in connections.setdefault(source, []):
            connections[source].append(target)
    doc["connections"] = connections


def _assign(current, new: dict) -> None:
    for key in [k for k in current if k not in new]:
        del current[key]
    for key, value in new.items():
        if isinstance(current.get(key), dict) and isinstance(value, dict):
            _assign(current[key], value)
        elif current.get(key) != value or type(current.get(key)) is not type(value):
            current[key] = value


__all__ = ["HOST_NAME", "Names", "chain_scope", "kept_parameters", "merge", "nodes_from_view",
           "port_entry", "unit_entry", "view_of", "view_scope"]
