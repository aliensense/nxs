# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The declaration as a graph: each node a block of a type, and the
connections keyed by the source port (ADR 0029). A node's type resolves to
its description (`nxs.description.of_type`), which fixes its kind, its
ports and the rows its parameters name. A parameter is a row of the block
in the row's display form, or an identity pin. A connection joins ports
that exist, from a port that feeds to one that takes, of types that meet
(`PortType.FEEDS`), whose same-named attributes agree; an input takes one
source, an output that is no bus feeds one port, and one host describes
the rig."""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Tuple

from nxs import description as desc
from nxs.description import ADDRESS, FIRMWARE, SERIAL, Description, PortDirection, PortType
from nxs.suite.schema_base import ManifestError, _parse_int, _require_keys

#: The declaration language this tool reads, the suite's `api`.
API = 3
#: The port types whose outputs fan out; an output of any other type feeds one port.
BUS_TYPES = (PortType.I2C, PortType.CYPHAL_CAN)

_TOP_KEYS = {"api", "suite", "nodes", "connections"}
_NODE_KEYS = {"type", "parameters"}
NODE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


@dataclass
class Node:
    """A declared node: its type, the description the type resolves to, its
    parameters as written, the served value of each row it declares by row
    name, and its identity pins (`address` an int, `serial` and `firmware`
    text)."""

    name: str
    type: str
    description: Description
    parameters: dict = field(default_factory=dict)
    values: Dict[str, int] = field(default_factory=dict)
    pins: Dict[str, Any] = field(default_factory=dict)

    @property
    def kind(self) -> str:
        """The block's kind word: host, hub, unit, camera, click or module."""
        return self.description.kind

    def port(self, name: str):
        """The node's port `name`, None for one its block does not offer."""
        return self.description.port(name)


#: One end of a connection: a node's name and its port's.
End = Tuple[str, str]


@dataclass(frozen=True)
class Edge:
    """A connection from a source port to a port it feeds."""

    source: End
    target: End


@dataclass
class Graph:
    """The declared nodes and their connections, in the file's order."""

    name: str = ""
    nodes: Dict[str, Node] = field(default_factory=dict)
    edges: List[Edge] = field(default_factory=list)

    def host(self) -> Optional[Node]:
        """The host the declaration describes, None for a hostless rig."""
        return next((n for n in self.nodes.values() if n.kind == "host"), None)

    def source(self, node: str, port: str) -> Optional[End]:
        """The one source wired into an input, None when nothing feeds it."""
        return next((e.source for e in self.edges if e.target == (node, port)), None)

    def sources(self, node: str, port: str) -> List[End]:
        """Every source wired into a port, in the declaration's order."""
        return [e.source for e in self.edges if e.target == (node, port)]

    def targets(self, node: str, port: str) -> List[End]:
        """The ports a source feeds, in the declaration's order."""
        return [e.target for e in self.edges if e.source == (node, port)]

    def kind_of(self, end: Optional[End]) -> Optional[str]:
        """The kind of the node an end names, None for no end or no node."""
        return self.nodes[end[0]].kind if end is not None and end[0] in self.nodes else None


def end_text(end: End) -> str:
    """An end as the declaration writes it, `<node>.<port>`."""
    return f"{end[0]}.{end[1]}"


def parse_end(text, where: str) -> End:
    """`node.port` as its two names; the node's name holds no dot."""
    if not isinstance(text, str) or "." not in text:
        raise ManifestError(f"{where}: {text!r} names no port; a port is written <node>.<port>")
    node, port = text.split(".", 1)
    if not node or not port:
        raise ManifestError(f"{where}: {text!r} names no port; a port is written <node>.<port>")
    return node, port


def is_1_0_form(raw: dict) -> bool:
    """Whether a declaration is written in the 1.0 form, `ports` and `units`."""
    return isinstance(raw, dict) and "api" not in raw and ("ports" in raw or "units" in raw)


#: The next command for a declaration of an earlier language.
WRITES_NODE_FORM = f"nxs generate (writes it in api {API}, the earlier file kept beside it)"


def parse_graph(raw: dict, where: str = "suite.yaml", lenient: bool = False) -> Graph:
    """The graph a declaration states, its shape and its laws held: every
    type resolves to a description, every parameter names a row or a pin
    and takes a value the row offers, every connection joins ports that
    meet, an input takes one source and one host describes the rig.
    `lenient` reads a draft the laws refuse, as the panel judges it while
    the operator steps toward a lawful one: a node or a value that does not
    parse is left out, and a connection keeps its edge where its ports
    disagree."""
    if is_1_0_form(raw):
        raise ManifestError(f"{where}: declares the 1.0 form (ports and units), and this nxs "
                            f"reads api {API}", [WRITES_NODE_FORM])
    if isinstance(raw, dict) and isinstance(raw.get("api"), int) and raw["api"] < API:
        raise ManifestError(f"{where}.api: {raw['api']}, an earlier declaration language, and "
                            f"this nxs reads api {API}", [WRITES_NODE_FORM])
    _require_keys(raw, _TOP_KEYS, where)
    if raw.get("api") != API:
        raise ManifestError(f"{where}.api: {raw.get('api')!r}, and this nxs reads api {API}",
                            [f"api: {API}"])
    suite = raw.get("suite") or {}
    _require_keys(suite, {"name"}, f"{where}.suite")
    nodes_raw = raw.get("nodes")
    if not isinstance(nodes_raw, dict) or not nodes_raw:
        raise ManifestError(f"{where}.nodes: a declaration names its nodes, at least one")
    connections = raw.get("connections") or {}
    if not isinstance(connections, dict):
        raise ManifestError(f"{where}.connections: expected a mapping of a source port to the "
                            f"ports it feeds")
    wired = _wired_ports(connections)
    graph = Graph(name=str(suite.get("name", "")))
    for name, spec in nodes_raw.items():
        try:
            graph.nodes[str(name)] = _parse_node(str(name), spec, wired.get(str(name), ()),
                                                 f"{where}.nodes.{name}", lenient)
        except ManifestError:
            if not lenient:
                raise
    hosts = [n.name for n in graph.nodes.values() if n.kind == "host"]
    if len(hosts) > 1:
        raise ManifestError(f"{where}.nodes: {' and '.join(hosts)} are hosts, and a declaration "
                            f"describes the rig of one host", [f"drop {hosts[1]}"])
    for key, targets in connections.items():
        try:
            _parse_connection(graph, key, targets, f"{where}.connections.{key}", lenient)
        except ManifestError:
            if not lenient:
                raise
    return graph


def _wired_ports(connections: Mapping) -> Dict[str, List[str]]:
    """The ports each node's connections name, by node: a block that offers
    ports by name (a host's buses) offers these."""
    out: Dict[str, List[str]] = {}
    for key, targets in connections.items():
        for text in [key, *(targets if isinstance(targets, list) else [])]:
            if isinstance(text, str) and "." in text:
                node, port = text.split(".", 1)
                out.setdefault(node, []).append(port)
    return out


def _parse_node(name: str, spec, wired, where: str, lenient: bool = False) -> Node:
    if not NODE_NAME.match(name):
        raise ManifestError(f"{where}: {name!r} is no node name (letters, digits, '-' and '_', "
                            f"no dot)")
    _require_keys(spec, _NODE_KEYS, where)
    node_type = spec.get("type")
    if not isinstance(node_type, str) or not node_type.strip():
        raise ManifestError(f"{where}.type: a node names its type: a host plugin, a hub's "
                            f"deserializer compatible, a unit module, a cam or click personality",
                            ["nxs describe <type>"])
    node_type = node_type.strip()
    parameters = spec.get("parameters") or {}
    if not isinstance(parameters, dict):
        raise ManifestError(f"{where}.parameters: expected a mapping of a row or a pin to its value")
    block = describe_type(node_type, f"{where}.type").with_buses(wired)
    node = Node(name=name, type=node_type, description=block, parameters=dict(parameters))
    for key, value in parameters.items():
        try:
            _parse_parameter(node, str(key), value, f"{where}.parameters.{key}")
        except ManifestError:
            if not lenient:
                raise
    return node


def describe_type(node_type: str, where: str) -> Description:
    """The description a node's type resolves to. ManifestError names the
    type where nothing installed is of it, or where its block does not
    build."""
    from nxs import experimental

    try:
        return desc.of_type(node_type)
    except desc.UnknownType:
        alternatives = [f"nxs describe {node_type}", "nxs describe nxs (a unit)",
                        "nxs personality install <directory> (a personality of your own)"]
        if not experimental.enabled():
            alternatives.append(f"nxs {experimental.FLAG} … (reads the hub and personality "
                                f"roots a development tree adds)")
        raise ManifestError(f"{where}: {node_type!r} names no host plugin, hub, unit module or "
                            f"personality installed here", alternatives) from None
    except desc.DescriptionError as exc:
        raise ManifestError(f"{where}: {exc}", ["nxs personality check <directory>"]) from None


def _parse_parameter(node: Node, key: str, value, where: str) -> None:
    """One parameter: an identity pin the block takes, else a row of the
    block in its display form, a value the row offers."""
    from nxs.suite.options import BlockView, judge, nearest, static_options

    block = node.description
    if key in block.pins:
        node.pins[key] = parse_pin(key, value, where)
        return
    try:
        row = block.row(key)
    except KeyError:
        if not block.live:
            # The unit's rows are its own, read from the unit: a declared one is
            # judged where the unit answers, and holds a number until then.
            node.values[key] = _parse_int(value, where)
            return
        # The alternatives are what a declaration sets: no read-only row, no run gate.
        settable = static_options(BlockView(block))
        names = [r.name for r in block.rows if settable[r.name].applicable] + list(block.pins)
        raise ManifestError(f"{where}: {block.type} has no row or pin {key}",
                            [f"nodes.{node.name}.parameters.{name}" for name in names]
                            or ["drop the key"])
    view = BlockView(block)
    options = static_options(view)
    try:
        raw = desc.parse(row, value)
    except desc.DescriptionError as exc:
        raise ManifestError(f"{where}: {exc}", nearest(row, options.get(key), row.default)) from None
    refusal = judge(view, options, key, raw)
    if refusal is not None:
        raise ManifestError(f"{where}: {refusal}", nearest(row, options.get(key), raw))
    node.values[key] = raw


def parse_pin(key: str, value, where: str) -> Any:
    """An identity pin as the graph holds it: `address` a 7-bit address,
    `serial` the 24 hex digits of the UID96, `firmware` a version."""
    from nxs.suite.schema import normalize_serial, parse_version

    if key == ADDRESS:
        address = _parse_int(value, where)
        if not 0x08 <= address <= 0x77:
            raise ManifestError(f"{where}: {address:#04x} is not a usable 7-bit I²C address "
                                f"(0x08-0x77)")
        return address
    if key == SERIAL:
        return normalize_serial(value, where)
    if key == FIRMWARE:
        text = str(value)
        try:
            parse_version(text)
        except ValueError as e:
            raise ManifestError(f"{where}: {e}") from None
        return text
    raise ManifestError(f"{where}: {key} is no identity pin")


def _parse_connection(graph: Graph, key, targets, where: str, lenient: bool = False) -> None:
    source = parse_end(key, where)
    src_node, src_port = _port_of(graph, source, where)
    if src_port.direction == PortDirection.INPUT:
        raise ManifestError(f"{where}: {end_text(source)} is an input, and a connection is keyed "
                            f"by the source port", [f"key the connection by the port that feeds "
                                                    f"{end_text(source)}"])
    if not isinstance(targets, list) or not targets:
        raise ManifestError(f"{where}: expected a list of the ports {end_text(source)} feeds")
    fans_out = src_port.direction == PortDirection.BUS or src_port.type in BUS_TYPES
    if not fans_out and len(targets) > 1:
        raise ManifestError(
            f"{where}: {end_text(source)} is a {desc.port_type_word(src_port.type)} output, which "
            f"feeds one port, and {len(targets)} are listed",
            [f"connections.{end_text(source)}: [{targets[0]}]"])
    for i, text in enumerate(targets):
        target = parse_end(text, f"{where}[{i}]")
        if target == source:
            raise ManifestError(f"{where}[{i}]: a port does not feed itself")
        dst_node, dst_port = _port_of(graph, target, f"{where}[{i}]")
        findings = desc.connect(src_node.description, source[1], dst_node.description, target[1],
                                src_node.values, dst_node.values,
                                a_name=src_node.name, b_name=dst_node.name)
        if findings and not lenient:
            raise ManifestError(f"{where}[{i}]: {end_text(source)} does not feed "
                                f"{end_text(target)}: {'; '.join(findings)}",
                                _connection_alternatives(src_node, source[1], dst_node, target[1]))
        edge = Edge(source, target)
        if edge in graph.edges:
            raise ManifestError(f"{where}[{i}]: {end_text(target)} is listed twice")
        if dst_port.direction == PortDirection.INPUT:
            taken = graph.source(*target)
            if taken is not None:
                raise ManifestError(
                    f"{where}[{i}]: {end_text(target)} is an input and takes one source, and "
                    f"{end_text(taken)} already feeds it",
                    [f"drop {end_text(target)} from connections.{end_text(source)}",
                     f"drop {end_text(target)} from connections.{end_text(taken)}"])
        graph.edges.append(edge)


def _connection_alternatives(a: Node, a_port: str, b: Node, b_port: str) -> List[str]:
    """What makes a refused connection lawful: for a disagreeing attribute,
    each value the other end's row takes that this end's row offers; for a
    port of another type, the ports of the target that take the source."""
    out: List[str] = []
    source, target = a.port(a_port), b.port(b_port)
    if source is None or target is None:
        return out
    b_rows = b.description.attributes(b_port)
    for attribute, row_a in a.description.attributes(a_port).items():
        row_b = b_rows.get(attribute)
        if row_b is None:
            continue
        value_a = a.values.get(row_a.name, row_a.default)
        value_b = b.values.get(row_b.name, row_b.default)
        if value_a == value_b:
            continue
        if not row_a.is_range and value_b in row_a.values:
            out.append(f"nodes.{a.name}.parameters.{row_a.name}: {desc.display(row_a, value_b)}")
        if not row_b.is_range and value_a in row_b.values:
            out.append(f"nodes.{b.name}.parameters.{row_b.name}: {desc.display(row_b, value_a)}")
    if not out:
        fed = PortType.FEEDS.get(source.type, (source.type,))
        takes = [p.name for p in b.description.ports
                 if p.type in fed and p.direction != PortDirection.OUTPUT]
        out = [f"connections.{a.name}.{a_port}: [{b.name}.{name}]" for name in takes]
    return out


def _port_of(graph: Graph, end: End, where: str):
    node = graph.nodes.get(end[0])
    if node is None:
        raise ManifestError(f"{where}: no node named {end[0]!r}",
                            [f"{name}" for name in sorted(graph.nodes)])
    port = node.port(end[1])
    if port is None:
        ports = [p.name for p in node.description.ports]
        extra = ([f"{node.name}.<bus>: a bus by its device's name (i2c-<N>, can<N>, tty<name>)"]
                 if node.description.buses is not None else [])
        raise ManifestError(f"{where}: {node.type} has no port {end[1]}",
                            [f"{node.name}.{p}" for p in ports] + extra)
    return node, port


__all__ = ["API", "BUS_TYPES", "Edge", "End", "Graph", "Node", "WRITES_NODE_FORM", "describe_type",
           "end_text", "is_1_0_form", "parse_end", "parse_graph", "parse_pin"]
