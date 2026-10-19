# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""What the graph derives: each camera connector of the host with what
feeds it, a hub with its links or the one head on the connector's own bus,
and each unit with the routes this host reaches it by. A port's values are
its nodes' rows: the hub's frame sync, gain, exposure and lanes, the host's
lanes and driver for the connector, each camera's mode and rate. What no
row states, a link's window and channel, a pod's alias, is the hub's rule
or the link's letter."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from nxs.description import ADDRESS, display, PortDirection, PortType
from nxs.suite.nodes import End, Graph, Node, end_text
from nxs.suite.schema_base import ManifestError

#: The hub's address when its node pins none.
HUB_ADDR_DEFAULT = 0x6A
#: The address every NXS straps for its unit door.
POD_TARGET_DEFAULT = 0x30
#: Who runs a hub from this host: the tool (`nxs`), a kernel driver (`kernel`), or nobody here
#: because a module masters the hub's bus (`module`, a passive host).
HUB_DRIVERS = ("nxs", "kernel", "module")
#: The frame-sync sources a port runs, as the camera layer names them.
SYNC_SOURCES = ("free_run", "fsync")
#: A hub's link inputs, by the link's name.
_LINKS = ("A", "B")
#: The rows of a hub and of a camera the port's construction reads.
SYNC_SOURCE, SYNC_FPS, GAIN, EXPOSURE = "sync_source", "sync_fps", "gain", "exposure"
MODE, RATE = "mode", "fps"
#: The hub's frame sync by its `sync_source` row's value: 0 free run, 1 the generator.
_SYNC_BY_VALUE = {0: "free_run", 1: "fsync"}
#: The alternative every wiring disagreement names.
WIRING_ALTERNATIVE = "nxs generate (writes what answers on the port)"


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
    ser: Optional[str] = None         # the link's serializer: the hub's rule
    des_window: Optional[int] = None  # the hub's rule for the link's name
    csi_vc: Optional[int] = None      # None: the hub's channel for the link's name
    ser_addr: int = 0x42
    sensor_addr: Optional[int] = None  # None: the descriptor's own address
    tca_addr: int = 0x20
    capture_id: Optional[int] = None
    unit: Optional[PortUnitRef] = None
    camera_mode: Optional[str] = None  # this link's declared mode name
    camera_fps: Optional[float] = None  # this link's declared free-run rate
    inck_hz: Optional[int] = None      # the clock the pod feeds the sensor
    camera_node: Optional[str] = None  # the camera node that declares the head
    #: The camera node's declared rows, served values by row name.
    camera_rows: Dict[str, int] = field(default_factory=dict)


@dataclass
class PortSpec:
    """One camera connector of the host: its bus and what feeds it, a hub
    with its GMSL links or the one head on the port's own bus."""
    name: str
    bus: Optional[str] = None          # None: the host's bus for the port's name
    hub_compatible: Optional[str] = None
    #: The declaration's file name, which a finding about the hub names.
    hub_source: Optional[str] = None
    hub_node: Optional[str] = None     # the hub node's name
    hub_addr: int = HUB_ADDR_DEFAULT
    hub_driver: str = "nxs"
    #: The node that masters the hub's bus when it is not this host.
    owner: Optional[str] = None
    csi_lanes: int = 4
    csi_lanes_declared: bool = False   # False: the booted overlay decides
    sync_source: str = "free_run"
    sync_fps: Optional[float] = None
    ## A declared exposure, microseconds: `check` and `on` refuse it with what sets the exposure:
    ## the trigger pulse under frame sync, else the capture stack's loop without `camera_gain_db`.
    camera_exposure_us: Optional[float] = None
    ## A declared analog gain, dB: the camera links locked at it, a synced
    ## pair, or free-running links with `camera_exposure_us`.
    camera_gain_db: Optional[float] = None
    links: List[PortLinkSpec] = field(default_factory=list)
    #: The hub's outputs the declaration states, `(lanes, streams, copy)`
    #: by port name: port B when its csi.B feeds a module.
    outputs: Dict[str, Tuple[int, Tuple[str, ...], bool]] = field(default_factory=dict)
    #: The hub node's rows, its declared values over the rows' defaults, by row name.
    hub_values: Dict[str, int] = field(default_factory=dict)
    #: Each node that shapes the port, with the digest of what shapes it.
    nodes: Dict[str, str] = field(default_factory=dict)


def port_signature(port: PortSpec) -> str:
    """Every declared field that shapes the port's construction, and the
    digest of each node that declares them, as one digest: nxsd reconverges
    a port on a reload when it changed, and `nxs switch` brings a port up
    again when its record carries another."""
    fields = (port.bus, port.hub_compatible, port.hub_addr, port.hub_driver,
              port.csi_lanes, port.csi_lanes_declared,
              port.camera_exposure_us, port.camera_gain_db, port.sync_source, port.sync_fps,
              tuple((l.name, l.camera, l.camera_mode, l.camera_fps, l.inck_hz, l.ser,
                     l.ser_addr, l.sensor_addr, l.tca_addr, l.des_window, l.csi_vc,
                     l.capture_id, tuple(sorted(l.camera_rows.items())),
                     (l.unit.name, l.unit.alias, l.unit.target) if l.unit else None)
                    for l in port.links),
              tuple(sorted(port.outputs.items())), tuple(sorted(port.hub_values.items())),
              tuple(sorted(port.nodes.items())))
    return hashlib.sha256(repr(fields).encode()).hexdigest()


def changed_nodes(before: PortSpec, after: PortSpec) -> List[str]:
    """The nodes of a port whose declaration moved between two loads, the
    ones that came and went included, by name."""
    names = set(before.nodes) | set(after.nodes)
    return sorted(n for n in names if before.nodes.get(n) != after.nodes.get(n))


def derive(graph: Graph, where: str, manifest: str) -> Tuple[Dict[str, PortSpec], list]:
    """The host's camera ports and the units it reaches, from the graph.
    `manifest` is the declaration's file name, which a finding names."""
    host = graph.host()
    ports: Dict[str, PortSpec] = {}
    if host is not None:
        for edge in graph.edges:
            node, name = edge.target
            if node == host.name and _is_connector(host, name):
                ports[name] = _port(graph, host, name, edge.source, where, manifest)
    units = [unit for unit in (_unit(graph, node, host, where)
                               for node in graph.nodes.values() if node.kind == "unit")
             if unit is not None]
    return ports, units


def _is_connector(host: Node, port: str) -> bool:
    """Whether a host port is a camera connector: a CSI stream input."""
    found = host.port(port)
    return found is not None and found.type == PortType.CSI and found.direction == PortDirection.INPUT


def connector_bus(port: str) -> str:
    """The host's port that masters a camera connector's I²C, beside it."""
    return f"bus_{port}"


def value(node: Node, row: str) -> Optional[int]:
    """A row's served value on a node: declared, else the row's default;
    None for a row the block lacks."""
    if row in node.values:
        return node.values[row]
    try:
        return int(node.description.row(row).default)
    except KeyError:
        return None


def _shown(node: Node, row: str) -> str:
    """A row's value on a node in its display form, for a refusal."""
    return display(node.description.row(row), value(node, row))


# ── the camera ports ─────────────────────────────────────────────────

def _port(graph: Graph, host: Node, name: str, source: End, where: str,
          manifest: str) -> PortSpec:
    src = graph.nodes[source[0]]
    port = PortSpec(name=name)
    lanes_row = f"{name}_lanes"
    if src.kind == "hub":
        _hub_port(graph, host, port, src, source, where, manifest)
        stated = lanes_row in host.values or "csi_a_lanes" in src.values
        lanes = value(src, "csi_a_lanes") if "csi_a_lanes" in src.values else value(host, lanes_row)
    elif src.kind in ("camera", "unit"):
        _direct_port(graph, port, src, where)
        stated = lanes_row in host.values
        lanes = value(host, lanes_row)
    else:
        raise ManifestError(f"{where}.connections.{end_text(source)}: a {src.kind} feeds no "
                            f"camera connector", ["a hub's csi.A, a camera's out or a pod's out"])
    if lanes is not None:
        port.csi_lanes = int(lanes)
        port.csi_lanes_declared = stated
    if "A" in port.outputs:
        # Port A runs the connector's lanes, known once both ends agree.
        port.outputs["A"] = (port.csi_lanes,) + tuple(port.outputs["A"][1:])
    port.nodes = _port_nodes(graph, host, port)
    return port


def _hub_port(graph: Graph, host: Node, port: PortSpec, hub: Node, source: End, where: str,
              manifest: str) -> None:
    hwhere = f"{where}.nodes.{hub.name}"
    if source[1] != "csi.A":
        raise ManifestError(
            f"{where}.connections.{end_text(source)}: the hub's port B is a module's, and a host "
            f"takes the hub's csi.A", [f"connections.{hub.name}.csi.A: [{host.name}.{port.name}]"])
    port.hub_compatible = hub.type
    port.hub_source = manifest
    port.hub_node = hub.name
    port.hub_addr = int(hub.pins.get(ADDRESS) or hub.description.identity.address or HUB_ADDR_DEFAULT)
    port.hub_values = {row.name: int(hub.values.get(row.name, row.default))
                     for row in hub.description.rows}
    owner = graph.source(hub.name, "i2c")
    if owner is None:
        raise ManifestError(
            f"{hwhere}: nothing feeds {hub.name}.i2c, so nothing masters the hub's bus",
            [f"connections.{host.name}.{connector_bus(port.name)}: [{hub.name}.i2c]"])
    driver_row = f"{port.name}_driver"
    if graph.kind_of(owner) == "host":
        if owner[1].startswith("bus_") and owner[1] != connector_bus(port.name):
            raise ManifestError(
                f"{where}.connections.{end_text(owner)}: {hub.name}'s csi.A feeds "
                f"{host.name}.{port.name}, and a hub's control bus is its connector's",
                [f"connections.{host.name}.{connector_bus(port.name)}: [{hub.name}.i2c]"])
        if not owner[1].startswith("bus_"):
            from nxs.host import buses
            port.bus = str(buses.route(owner[1]).get("bus"))
        port.hub_driver = _shown(host, driver_row) if value(host, driver_row) is not None else "nxs"
    else:
        if driver_row in host.values:
            raise ManifestError(
                f"{where}.nodes.{host.name}.parameters.{driver_row}: names who drives the hub this "
                f"host masters, and {owner[0]} masters {hub.name}'s bus", ["drop the key"])
        port.hub_driver = "module"
        port.owner = owner[0]
    port.sync_source = _SYNC_BY_VALUE.get(value(hub, SYNC_SOURCE) or 0, "free_run")
    if SYNC_FPS in hub.values:
        port.sync_fps = _scaled(hub, SYNC_FPS)
    if GAIN in hub.values:
        port.camera_gain_db = _scaled(hub, GAIN)
    if hub.values.get(EXPOSURE):
        port.camera_exposure_us = _scaled(hub, EXPOSURE)
    for letter in _LINKS:
        feed = graph.source(hub.name, f"links.{letter}")
        if feed is not None:
            port.links.append(_hub_link(graph, hub, letter, feed, where))
    port.outputs = _outputs(graph, hub, port)
    _require_separable_pods(port, where)


def _scaled(node: Node, row: str) -> float:
    """A row's declared value as the number its display form shows."""
    from decimal import Decimal

    from nxs.description import present

    shown = present(node.description.row(row))
    return float(Decimal(int(node.values[row])).scaleb(shown.exponent))


def _hub_link(graph: Graph, hub: Node, letter: str, feed: End, where: str) -> PortLinkSpec:
    """The link behind a hub's `links.<letter>`: the head that feeds it, or
    the pod and the head that feeds the pod."""
    link = PortLinkSpec(name=letter, camera=None)
    node = graph.nodes[feed[0]]
    camera = node if node.kind == "camera" else None
    if node.kind == "unit":
        link.unit = _pod_ref(node, letter, hub=True)
        head = graph.source(node.name, "head")
        camera = graph.nodes[head[0]] if head is not None else None
    if camera is not None:
        _camera_onto(link, camera)
    _refuse_smart_camera_off_its_port(graph, camera, link.unit, hub.name, where)
    return link


def _pod_ref(node: Node, letter: str, hub: bool) -> PortUnitRef:
    """A pod's addresses on its link: behind a hub the alias its link's
    letter takes (A 0x32, B 0x34); on the connector's own bus the address
    it straps, which nothing translates. Its `address` pin is the address
    it straps."""
    from nxs.generate_seed import pod_alias

    target = int(node.pins.get(ADDRESS, POD_TARGET_DEFAULT))
    alias = pod_alias(letter) if hub else target
    return PortUnitRef(name=node.name, alias=alias, target=target)


def _camera_onto(link: PortLinkSpec, camera: Node) -> None:
    """A camera node's sensor and rows onto the link it rides: its mode by
    name, its rate in frames per second where it declares one, its address
    pin."""
    link.camera = camera.type
    link.camera_node = camera.name
    link.camera_rows = dict(camera.values)
    if MODE in camera.values:
        link.camera_mode = _shown(camera, MODE)
    if RATE in camera.values:
        link.camera_fps = camera.values[RATE] / 1000
    if ADDRESS in camera.pins:
        link.sensor_addr = int(camera.pins[ADDRESS])


def _direct_port(graph: Graph, port: PortSpec, src: Node, where: str) -> None:
    """A connector port: its one head on the port's own bus, or the pod that
    carries it there."""
    link = PortLinkSpec(name="A", camera=None, csi_vc=0)
    camera = src if src.kind == "camera" else None
    if src.kind == "unit":
        link.unit = _pod_ref(src, "A", hub=False)
        head = graph.source(src.name, "head")
        camera = graph.nodes[head[0]] if head is not None else None
        if camera is None:
            raise ManifestError(
                f"{where}.nodes.{src.name}: a pod on a camera connector carries a head, and "
                f"nothing feeds {src.name}.head", [f"connections.<camera>.out: [{src.name}.head]"])
    if camera is not None:
        _camera_onto(link, camera)
        if GAIN in camera.values:
            port.camera_gain_db = _scaled(camera, GAIN)
        if camera.values.get(EXPOSURE):
            port.camera_exposure_us = _scaled(camera, EXPOSURE)
        if (link.unit is not None and link.sensor_addr is not None
                and link.sensor_addr == link.unit.target):
            raise ManifestError(
                f"{where}.nodes.{src.name}.parameters.address: {link.unit.target:#04x} is the "
                f"sensor's address too, and nothing on the port's own bus tells the two apart",
                [f"nodes.{src.name}.parameters.address: the address the unit straps",
                 f"nodes.{camera.name}.parameters.address: the address the sensor straps"])
    _refuse_smart_camera_off_its_port(graph, camera, link.unit, None, where)
    port.links.append(link)


def _outputs(graph: Graph, hub: Node, port: PortSpec) -> Dict[str, tuple]:
    """The hub's outputs beside port A, `(lanes, streams, copy)` by port
    name: port B when its csi.B feeds a module, at the hub's `csi_b_lanes`
    and `csi_b_copy`, carrying every camera link's stream."""
    if not graph.targets(hub.name, "csi.B"):
        return {}
    cameras = tuple(l.name for l in port.links if l.camera)
    lanes = value(hub, "csi_b_lanes")
    return {"B": (int(lanes) if lanes is not None else 2, cameras,
                  bool(value(hub, "csi_b_copy") or 0))}


# ── the laws a port's addresses keep ─────────────────────────────────

def _require_separable_pods(port: PortSpec, where: str) -> None:
    """Two pods behind one hub answer apart only through their aliases. A
    pod's own link translates its alias to the pod's unit door and the next
    address to its camera door. Every pod's strapped pair, the unit door at
    `target` and the camera door one above, answers untranslated on every
    link the hub merges. A pod is refused when its alias or the address
    above it reaches a second pod. Of two links whose pairs overlap, the
    later one is refused."""
    pods = [link for link in port.links if link.unit is not None]
    if len(pods) < 2:
        return
    for index, link in enumerate(pods):
        for addr, what in ((link.unit.alias, "alias"), (link.unit.alias + 1, "camera door")):
            fact = _merge_fact(link, addr, what, pods, pods[:index],
                               f"{where}.nodes.{link.unit.name}.parameters.address")
            if fact is not None:
                raise ManifestError(fact, [f"nodes.{pod.unit.name}.parameters.address: the address "
                                           f"{pod.unit.name} straps" for pod in pods])


def _merge_fact(link, addr: int, what: str, pods, earlier, lwhere: str) -> Optional[str]:
    """Why `addr`, the host address of the pod's `what`, reaches a second
    pod: another pod's strapped door answers there, or an earlier link's
    alias pair holds it. None when it reaches this pod alone."""
    alias, target = link.unit.alias, link.unit.target
    for part, offset in (("unit door", 0), ("camera door", 1)):
        others = [p for p in pods if p is not link and p.unit.target + offset == addr]
        if not others:
            continue
        if what == "alias" and offset == 0 and alias == target:
            return (f"{lwhere}: a pod at its strapped address {target:#04x} answers on every "
                    f"link, and {len(pods)} pods ride this hub")
        every = all(p.unit.target + offset == addr for p in pods)
        if what == "alias" and offset == 1 and every:
            return (f"{lwhere}: link {link.name}'s alias {alias:#04x} is every pod's camera door, "
                    f"strapped, which answers on every link, and {len(pods)} pods ride this hub")
        owner = (f"every pod's strapped {part}" if every
                 else f"{others[0].unit.name}'s strapped {part}")
        return (f"{lwhere}: link {link.name}'s alias {alias:#04x} puts the pod's {what} at "
                f"{addr:#04x}, {owner}, and two pods at one address merge")
    for other in earlier:
        for part, offset in (("alias", 0), ("camera door", 1)):
            if other.unit.alias + offset != addr:
                continue
            if what == "alias" and offset == 0:
                return (f"{lwhere}: link {link.name}'s alias {alias:#04x} is also "
                        f"{other.unit.name}'s, and two pods at one alias merge")
            return (f"{lwhere}: link {link.name}'s alias {alias:#04x} puts the pod's {what} at "
                    f"{addr:#04x}, {other.unit.name}'s {part}, and two pods at one address merge")
    return None


def _refuse_smart_camera_off_its_port(graph, camera, unit, hub: Optional[str], where: str) -> None:
    """A smart camera, a cam personality of the `native` family, serves its
    camera door at 0x31 alone on a connector's own bus. Behind a hub 0x31
    is every pod's strapped camera door, and a pod carrying it answers its
    own camera door there: both are refused at the declaration."""
    if camera is None or (hub is None and unit is None):
        return
    from nxs.cam.hubs import declares_smart_camera

    try:
        smart = declares_smart_camera(camera.type)
    except (RuntimeError, OSError):
        return          # a registry this host cannot read describes no smart camera here
    if not smart:
        return
    host = graph.host()
    connector = f"connections.{camera.name}.out: [{host.name if host else '<host>'}.cam<N>]"
    if hub is not None:
        raise ManifestError(
            f"{where}.nodes.{camera.name}.type: {camera.type} is a smart camera, which serves its "
            f"camera door at 0x31 on a connector's own bus, and behind a hub 0x31 is every pod's "
            f"strapped camera door", [connector, f"nodes.{camera.name}.type: a sensor the hub serves"])
    raise ManifestError(
        f"{where}.nodes.{unit.name}: {camera.type} is a smart camera, which serves its camera door "
        f"at 0x31 alone on the connector's bus, and a pod carrying it answers its own camera door "
        f"there", [connector])


# ── the units ────────────────────────────────────────────────────────

def _unit(graph: Graph, node: Node, host: Optional[Node], where: str):
    """A unit node as the unit step drives it: its routes from this host in
    the declaration's order, its panel, its pins and its rows; None on a rig
    with no host, whose units are a module's to reach. A socket no click
    feeds declares an empty panel."""
    from nxs.suite.schema import SensorSpec, UnitSpec

    uwhere = f"{where}.nodes.{node.name}"
    if host is None:
        return None
    routes = _routes(graph, node, host, where)
    if not routes and _behind_another_owner(graph, node, host):
        # The hub's owner reaches the unit; this host takes the stream alone.
        return None
    if not routes:
        raise ManifestError(
            f"{uwhere}: nothing on {host.name} reaches the unit",
            [f"connections.{node.name}.out: [<hub>.links.A]",
             f"connections.{host.name}.<bus>: [{node.name}.i2c]",
             f"connections.{host.name}.<can bus>: [{node.name}.can]"])
    clicks = graph.sources(node.name, "click")
    sensors = [SensorSpec(personality=graph.nodes[c[0]].type.replace("-", "_"),
                          config=dict(graph.nodes[c[0]].values)) for c in clicks]
    # Drift and freeze match the running personality by name, so a panel
    # names each once.
    for i, sensor in enumerate(sensors):
        first = next(j for j, s in enumerate(sensors) if s.personality == sensor.personality)
        if first != i:
            raise ManifestError(
                f"{where}.nodes.{clicks[i][0]}.type: {sensor.personality} is also "
                f"{clicks[first][0]}'s, and {node.name}'s panel names a personality once",
                [f"drop {clicks[i][0]}"])
    return UnitSpec(name=node.name, module=node.type, links=routes, sensors=sensors,
                    serial=node.pins.get("serial"), firmware=node.pins.get("firmware"),
                    rows=dict(node.values))


def _routes(graph: Graph, node: Node, host: Node, where: str) -> list:
    """The routes from the host to a unit, the order the tool tries them in
    whatever the file's layout: the camera chain its `out` rides, then the
    host's buses wired into its `i2c`, `can` and `serial`."""
    from nxs.suite.schema import LinkSpec

    routes = []
    for edge in graph.edges:
        if edge.source == (node.name, "out"):
            route = _chain_route(graph, node, edge.target, host)
            if route is not None:
                routes.append((0, LinkSpec(transport="i2c", link_ref=route)))
            continue
        ends = {edge.source, edge.target}
        mine = next((e for e in ends if e[0] == node.name), None)
        other = next((e for e in ends if e[0] == host.name), None)
        if mine is None or other is None or mine[1] not in _ROUTE_PORTS:
            continue
        routes.append((_ROUTE_PORTS.index(mine[1]) + 1,
                       _bus_route(node, mine[1], host, other[1], where)))
    return [route for _rank, route in sorted(routes, key=lambda r: r[0])]


def _behind_another_owner(graph: Graph, node: Node, host: Node) -> bool:
    """Whether a unit's camera chain reaches this host through a hub whose
    bus another node masters: the owner reaches the unit, this host takes
    the hub's stream and transfers nothing on its bus."""
    for edge in graph.edges:
        if edge.source != (node.name, "out") or graph.kind_of(edge.target) != "hub":
            continue
        owner = graph.source(edge.target[0], "i2c")
        reaches = any(out[0] == host.name for out in graph.targets(edge.target[0], "csi.A"))
        if reaches and (owner is None or owner[0] != host.name):
            return True
    return False


#: A unit's ports a host bus reaches it on, in the order they are tried.
_ROUTE_PORTS = ("i2c", "can", "serial")
#: The unit's row that names its node on a Cyphal/CAN bus.
CAN_NODE_ID = "can_node_id"


def _chain_route(graph: Graph, pod: Node, target: End, host: Node) -> Optional[str]:
    """`<connector>/<link>` of the camera chain a pod's `out` rides to this
    host: a hub's link whose csi.A feeds one of the host's connectors, or
    the connector itself; None when the chain reaches another host, or a
    hub whose bus another node masters."""
    if target[0] == host.name and _is_connector(host, target[1]):
        return f"{target[1]}/A"
    if graph.kind_of(target) != "hub" or not target[1].startswith("links."):
        return None
    owner = graph.source(target[0], "i2c")
    if owner is None or owner[0] != host.name:
        return None
    for out in graph.targets(target[0], "csi.A"):
        if out[0] == host.name and _is_connector(host, out[1]):
            return f"{out[1]}/{target[1].split('.', 1)[1]}"
    return None


def _bus_route(node: Node, mine: str, host: Node, port: str, where: str):
    """A route over one of the host's buses: a connector's I²C, an I²C
    adapter, a Cyphal/CAN bus or a Cyphal/serial port, each named for its
    device (`nxs.host.buses`)."""
    from nxs.host import buses
    from nxs.suite.schema import parse_node_id, parse_route

    pwhere = f"{where}.nodes.{node.name}"
    if mine == "i2c":
        address = node.pins.get(ADDRESS, POD_TARGET_DEFAULT)
        if port.startswith("bus_"):
            link = parse_route({"transport": "i2c", "bus": "", "address": address}, pwhere)
            link.bus, link.connector = None, port[len("bus_"):]
            return link
        return parse_route({**buses.route(port), "address": address}, pwhere)
    if mine == "can":
        if CAN_NODE_ID not in node.values:
            raise ManifestError(f"{pwhere}.parameters: a unit on a Cyphal/CAN bus names its node id",
                                [f"{pwhere}.parameters.{CAN_NODE_ID}: <0..125>"])
        node_id = parse_node_id(node.values[CAN_NODE_ID], f"{pwhere}.parameters.{CAN_NODE_ID}")
        return parse_route({**buses.route(port), "node_id": node_id}, pwhere)
    return parse_route(buses.route(port, host.values), pwhere)


# ── the signatures ───────────────────────────────────────────────────

def _port_nodes(graph: Graph, host: Node, port: PortSpec) -> Dict[str, str]:
    """Each node that shapes the port, by name, with the digest of what it
    declares that shapes it: the host's connector rows, the hub and the
    wiring into it, each head and each pod's address. A unit's panel, its
    other pins and its rows are the unit step's and shape no port."""
    rows = {k: v for k, v in host.values.items() if k.startswith(f"{port.name}_")}
    sigs = {f"{host.name}.{port.name}": _digest(("host", rows))}
    feed = graph.source(host.name, port.name)
    names: List[str] = [feed[0]] if feed is not None else []
    hub = graph.nodes.get(port.hub_node) if port.hub_node else None
    if hub is not None:
        names += [graph.source(hub.name, f"links.{l}")[0] for l in _LINKS
                  if graph.source(hub.name, f"links.{l}") is not None]
    for name in list(names):
        if graph.nodes[name].kind == "unit":
            head = graph.source(name, "head")
            if head is not None:
                names.append(head[0])
    for name in dict.fromkeys(names):
        sigs[name] = node_signature(graph, name)
    return sigs


def node_signature(graph: Graph, name: str) -> str:
    """The digest of what a node declares that shapes a camera port: its
    kind and type, its rows and pins (a unit's address alone) and the
    sources wired into it."""
    node = graph.nodes[name]
    if node.kind == "unit":
        values, pins = {}, {k: v for k, v in node.pins.items() if k == ADDRESS}
    else:
        values, pins = node.values, node.pins
    feeds = sorted((e.target[1], end_text(e.source)) for e in graph.edges
                   if e.target[0] == name and (node.kind != "unit" or e.target[1] == "head"))
    return _digest((node.kind, node.type, values, pins, feeds))


def _digest(value) -> str:
    return hashlib.sha256(repr(_canonical(value)).encode()).hexdigest()[:16]


def _canonical(value):
    """A value with its mappings in key order, so equal declarations digest alike."""
    if isinstance(value, dict):
        return tuple(sorted((str(k), _canonical(v)) for k, v in value.items()))
    if isinstance(value, (list, tuple)):
        return tuple(_canonical(v) for v in value)
    return value


__all__ = ["CAN_NODE_ID", "HUB_ADDR_DEFAULT", "HUB_DRIVERS", "POD_TARGET_DEFAULT", "PortLinkSpec",
           "PortSpec", "PortUnitRef", "SYNC_SOURCES", "WIRING_ALTERNATIVE", "changed_nodes",
           "connector_bus", "derive", "node_signature", "port_signature", "value"]
