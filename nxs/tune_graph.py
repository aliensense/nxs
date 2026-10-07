# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The rig as a tree: a port carries its hub's links or the devices on its own bus, a link its pod or a bare head, a pod its personalities; every node names the knobs the model holds for it."""

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from nxs.tune_model import _token, load_model
from nxs.tune_sweep import sweep as _sweep
from nxs.tune_fields import (NONE, _platform_port_buses,
                             refresh)

STACK_WORDS = {"ready": "capture stack ready",
               "preparing": "preparing the capture stack",
               "missing": "capture stack not configured"}


@dataclass
class Knob:
    """One field of the model, with the channel and section it steps through."""
    channel: Any
    section: Any
    field: Any

    @property
    def name(self) -> str:
        return str(self.field.name)

    @property
    def value(self) -> str:
        return str(self.field.render(self.field.value))


@dataclass
class Node:
    kind: str                       # port | link | unit | personality
    key: tuple
    parts: List[str]                # joined with " · "
    suffix: str = ""                # "" | absent | new
    state: Optional[str] = None     # up | parked, a link's capture state
    knobs: List[Knob] = field(default_factory=list)
    children: List["Node"] = field(default_factory=list)
    unit: Optional[str] = None      # the unit identify strobes and freeze --unit takes
    route: Any = None               # the I2C route of a unit the manifest does not name
    port: Optional[str] = None      # the port freeze --ports takes
    freeze: str = "ports"           # which freeze the node takes: ports | unit
    note: str = ""

    def walk(self, depth: int = 0):
        yield self, depth
        for child in self.children:
            yield from child.walk(depth + 1)


@dataclass
class Facts:
    """What the host says beside the declaration, read once per load or refresh."""
    buses: Dict[str, str] = field(default_factory=dict)
    sweep: Dict[str, Any] = field(default_factory=dict)
    records: Dict[str, dict] = field(default_factory=dict)
    states: Dict[Tuple[str, str], str] = field(default_factory=dict)
    stacks: Dict[str, Optional[str]] = field(default_factory=dict)
    labels: Dict[Tuple[str, str], str] = field(default_factory=dict)


def load():
    """(path, cfg, channels, facts): the model and the facts on one bus sweep,
    the declared pod aliases probed beside the standard addresses."""
    from nxs.suite import default_config_path
    from nxs.suite.schema import load_suite_config

    path = default_config_path()
    declared = load_suite_config(path) if os.path.exists(path) else None
    buses = _platform_port_buses()
    sweep = _sweep(buses, declared) if buses else {}
    path, cfg, channels = load_model(sweep=sweep)
    return path, cfg, channels, gather_facts(cfg, buses, sweep)


def gather_facts(cfg, buses, sweep) -> Facts:
    """The port records, capture states and mode labels of the declared ports; a
    host that cannot say leaves the entry out."""
    facts = Facts(buses=dict(buses), sweep=dict(sweep))
    for name, port in sorted(cfg.ports.items()):
        topology = _topology(port)
        if topology is None:
            continue
        facts.records[name] = _record(topology)
        facts.stacks[name] = _stack(name, topology)
        for link in port.links:
            facts.states[(name, link.name)] = _state(topology, link.name)
            token = _mode_token(facts.records[name], port, link)
            if token and link.camera:
                facts.labels[(name, link.name)] = _label(topology, link.camera, token)
    return facts


def _topology(port):
    from nxs.cam.topology import port_topology
    try:
        return port_topology(port)
    except Exception:  # noqa: BLE001 (a port the host cannot shape)
        return None


def _record(topology) -> dict:
    from nxs.cam import port_state
    try:
        return port_state.port_record(topology)
    except Exception:  # noqa: BLE001
        return {}


def _state(topology, link_name) -> Optional[str]:
    from nxs.cam import port_state
    link = next((l for l in topology.links if l.name == link_name), None)
    if link is None:
        return None
    try:
        state = port_state.link_state(topology, link)
    except Exception:  # noqa: BLE001
        return None
    return state if state in (port_state.STATE_UP, port_state.STATE_PARKED) else None


def _stack(name, topology) -> Optional[str]:
    from nxs import host as host_layer
    from nxs.host.cli import capture_stack_state
    try:
        return capture_stack_state(host_layer.current(), name, topology)
    except Exception:  # noqa: BLE001
        return None


def _mode_token(record, port, link) -> Optional[str]:
    return (record.get("modes") or {}).get(link.name) or link.camera_mode or port.camera_mode


def _rate(facts, name, port, link) -> Optional[float]:
    from nxs.cam import port_state
    rate = port_state.running_rate(facts.records.get(name) or {}, link.name)
    if rate is None:
        rate = link.camera_fps or port.camera_fps
    return float(rate) if rate is not None else None


def _label(topology, compatible, token) -> str:
    from nxs.cam import hubs
    from nxs.cam.descriptors import mode_label, resolve_mode
    try:
        descriptor = hubs.for_topology(topology).descriptor(compatible)
        name = token if token in descriptor.modes else resolve_mode(descriptor, token)
        return mode_label(descriptor, name)
    except Exception:  # noqa: BLE001 (a token the hub no longer names)
        return str(token)


def route_text(link, buses) -> str:
    """A unit's route as the tree prints it: `i2c cam1@0x30`, `can can0 node 7`."""
    if link.transport != "i2c":
        return link.describe()
    port = next((p for p, b in buses.items() if b == link.bus), None)
    return f"i2c {port or os.path.basename(str(link.bus))}@{link.address:#04x}"


def build_tree(cfg, channels, facts: Facts) -> List[Node]:
    """The nodes in reading order: every port sorted by name, declared or not,
    with its links and its own-bus units, then the units no camera port carries."""
    by_name = {ch.name: ch for ch in channels}
    by_unit = {u.name: u for u in cfg.units}
    seen = set()
    tree = []
    for channel in sorted((ch for ch in channels if ch.kind == "port"), key=lambda ch: ch.name):
        tree.append(_port_node(channel, cfg, by_name, by_unit, facts, seen))
    for channel in channels:
        if channel.kind == "unit" and channel.name not in seen:
            tree.append(_unit_node(channel, by_unit.get(channel.name), facts, seen))
    return tree


def _port_node(channel, cfg, by_name, by_unit, facts, seen) -> Node:
    name = channel.name
    port = cfg.ports.get(name) if channel.declared else None
    parts = [name]
    if port is not None and port.hub_compatible:
        parts.append(f"NXS Hub {port.hub_compatible}")
    if STACK_WORDS.get(facts.stacks.get(name)):
        parts.append(STACK_WORDS[facts.stacks[name]])
    bus = facts.buses.get(name) or (port.bus if port is not None else None)
    swept = facts.sweep.get(bus)
    suffix = "absent" if port is not None and port.hub_compatible and swept and not swept["hub"] else ""
    node = Node("port", ("port", name), parts, suffix=suffix, knobs=_knobs(channel), port=name,
                note=channel.note)
    hub = channel.knobs().get("HUB", NONE)
    found = (swept or {}).get("links", {})
    for link in channel.family.links:
        # A declared port draws its declared links and the ones something
        # answered on; an undeclared one every letter once a hub is chosen.
        declared = port is not None and any(l.name == link.letter for l in port.links)
        told = found.get(link.letter) or {}
        answered = told.get("sensor") or told.get("head") or told.get("pod")
        if declared or answered or (port is None and hub != NONE and link.letter in channel.family.letters()):
            node.children.append(_link_node(link, port, bus, by_name, by_unit, facts, seen))
    node.children.extend(_bus_units(bus, cfg, by_name, by_unit, facts, seen))
    if not channel.declared and swept:
        heads = any(l["sensor"] or l["head"] for l in found.values())
        fresh = any(n.suffix == "new" for child in node.children for n, _d in child.walk())
        if swept["hub"] or heads or fresh:
            node.suffix = "new"
    return node


def _link_node(link, port, bus, by_name, by_unit, facts, seen) -> Node:
    name, letter = link.port_name, link.letter
    key = ("link", name, letter)
    spec = next((l for l in port.links if l.name == letter), None) if port is not None else None
    state = facts.states.get((name, letter))
    told = (facts.sweep.get(bus) or {}).get("links", {}).get(letter) or {}
    figure = _camera_figure(facts, name, port, spec) if spec is not None else []
    sensor = spec.camera if spec is not None else told.get("sensor")
    pod_token = _token(link.knobs().get("POD", NONE))
    pod = by_name.get(pod_token) if pod_token != NONE else None
    if pod is None:
        # A bare link: the head on the line, its knobs with the link's. A pod
        # that answered on it undeclared hangs under it, new.
        head = ([sensor] if sensor else
                [f"head @{told['head_addr']:#04x}"] if told.get("head") and told.get("head_addr") is not None
                else [])
        node = Node("link", key, [letter] + head + figure, state=state, port=name, knobs=_knobs(link))
        found = by_name.get(_pod_name(name, letter))
        if found is not None and not found.declared:
            node.children.append(_unit_node(found, None, facts, seen))
        return node
    seen.add(pod.name)
    alias = (spec.unit.alias if spec is not None and spec.unit is not None
             else pod.template[0].address if pod.template else None)
    suffix = ("new" if not pod.declared else _pod_suffix(facts, bus, letter))
    node = Node("link", key, [letter, f"{pod.name} @{alias:#04x}" if alias is not None else pod.name],
                state=state, suffix=suffix, unit=pod.name, port=name,
                knobs=_knobs(link, kinds=("declare",)) + _knobs(pod, kinds=("declare", "mount", "egress", "firmware")),
                route=pod.template[0] if not pod.declared and pod.template else None, note=pod.note)
    if sensor:
        node.children.append(Node("personality", key + ("camera",), [sensor] + figure,
                                  knobs=_knobs(link, kinds=("camera",)), unit=pod.name, port=name))
    node.children.extend(_personalities(key, pod, unit=pod.name, port=name))
    return node


def _camera_figure(facts, name, port, link) -> List[str]:
    label = facts.labels.get((name, link.name))
    rate = _rate(facts, name, port, link)
    if label is None and rate is None:
        return []
    words = [w for w in (label, f"{rate:g} fps" if rate is not None else None) if w]
    return [" ".join(words)]


def _pod_suffix(facts, bus, letter) -> str:
    """absent for a declared pod whose link the hub walked and found none on."""
    swept = facts.sweep.get(bus)
    link = (swept["links"].get(letter) if swept and swept["hub"] else None)
    if link is None or not link["walked"]:
        return ""
    return "" if link["pod"] else "absent"


def _pod_name(port, letter) -> str:
    from nxs.generate_seed import _unit_name
    return _unit_name(port, letter)


def _bus_units(bus, cfg, by_name, by_unit, facts, seen) -> List[Node]:
    """The units reached over the port's own bus, declared or answering."""
    nodes = []
    if not bus:
        return nodes
    for unit in cfg.units:
        if unit.name in seen or unit.name not in by_name:
            continue
        route = next((l for l in unit.links
                      if l.transport == "i2c" and l.link_ref is None and l.bus == bus), None)
        if route is not None:
            nodes.append(_unit_node(by_name[unit.name], unit, facts, seen, route))
    for channel in by_name.values():
        if channel.kind != "unit" or channel.declared or channel.name in seen or channel.link_ref:
            continue
        link, _serial = channel.template
        if link.bus == bus:
            nodes.append(_unit_node(channel, None, facts, seen))
    return nodes


def _unit_node(channel, spec, facts, seen, route=None) -> Node:
    """A unit's node: under a port, `name @addr` on the route that reaches it; at
    the top level, its name and its first route. Its knobs are the pod's or the
    unit's own; its personalities hang under it."""
    seen.add(channel.name)
    key = ("unit", channel.name)
    knobs = _knobs(channel, kinds=("declare", "mount", "egress", "firmware"))
    if not channel.declared:
        link, _serial = channel.template
        parts = ([f"{channel.name} @{link.address:#04x}"] if link.transport == "i2c"
                 else [channel.name, route_text(link, facts.buses)])
        return Node("unit", key, parts, suffix="new", knobs=knobs,
                    unit=channel.name, route=link, freeze="unit", note=channel.note)
    if route is not None:
        parts = [f"{channel.name} @{route.address:#04x}"]
    else:
        first = spec.links[0] if spec is not None and spec.links else None
        parts = [channel.name] + ([route_text(first, facts.buses)] if first is not None else [])
    node = Node("unit", key, parts, suffix="absent" if channel.presence == "absent" else "",
                knobs=knobs, unit=channel.name, freeze="unit", note=channel.note)
    node.children = _personalities(key, channel, unit=channel.name, port=None)
    return node


def _personalities(key, channel, unit, port) -> List[Node]:
    """One node per click personality the unit declares, its parameters as
    knobs; a click personality freezes with its unit."""
    nodes = []
    for section in channel.sections:
        if section.kind != "sensor":
            continue
        knobs = _knobs(channel, labels=(section.label,))
        personality = section.label.rsplit("/", 1)[-1]
        parts = [personality] + ([knobs[0].value] if knobs else [])
        nodes.append(Node("personality", key + (section.sensor_index,), parts, knobs=knobs,
                          unit=unit, port=port, freeze="unit"))
    return nodes


def _knobs(channel, kinds=None, labels=None) -> List[Knob]:
    """The channel's knobs, every section's or those of the kinds or labels named."""
    out = []
    for section in channel.sections:
        if kinds is not None and section.kind not in kinds:
            continue
        if labels is not None and section.label not in labels:
            continue
        out.extend(Knob(channel, section, f) for f in section.fields)
    return out


def token(option):
    """The addressable value of an option (a PRESET carries a label)."""
    return option[0] if isinstance(option, tuple) else option


def apply_edits(channels, edits, keep=None) -> dict:
    """Replay unsaved edits, `(channel, section label, field) -> token`, onto a
    freshly loaded model in their order, each followed by its section's rebuild;
    an edit whose knob or value is gone, or that `keep(channel, section, field)`
    refuses, is dropped. Returns the edits that hold."""
    kept = {}
    by_name = {ch.name: ch for ch in channels}
    for (name, label, field_name), value in edits.items():
        channel = by_name.get(name)
        section = next((s for s in channel.sections if s.label == label), None) if channel else None
        field = next((f for f in section.fields if f.name == field_name), None) if section else None
        if field is None or (keep is not None and not keep(channel, section, field_name)):
            continue
        tokens = [token(o) for o in field.options]
        if value not in tokens:
            continue
        field.index = tokens.index(value)
        refresh(section)
        kept[(name, label, field_name)] = value
    return kept


def counts(tree: List[Node]) -> Tuple[int, int]:
    """(ports, units) in the tree: a pod on a link counts as a unit."""
    ports = units = 0
    for root in tree:
        for node, _depth in root.walk():
            ports += node.kind == "port"
            units += node.kind == "unit" or (node.kind == "link" and node.unit is not None)
    return ports, units


def find(tree: List[Node], key) -> Optional[Node]:
    for root in tree:
        for node, _depth in root.walk():
            if node.key == key:
                return node
    return None


__all__ = ["Facts", "Knob", "Node", "NONE", "apply_edits", "build_tree", "counts", "find",
           "gather_facts", "load", "route_text", "token"]
