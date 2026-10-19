# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The declaration against the rig as its graph: every declared node with
what it declares and what the rig answers for it, and every connection, as
the `graph` of bare `nxs status --json`. It is a read-only view, the one an
editor of the declaration renders; the port and unit sections beside it
carry the full readings."""

from __future__ import annotations

from typing import Any, Dict, List

#: What the rig answers for a node: as declared, nothing, or not read (a bus another run held, a
#: node this host does not reach).
PRESENT, ABSENT, UNKNOWN = "present", "absent", "unknown"


def graph_view(cfg, ports: List[dict], units: List[dict], by_node: Dict[str, list]) -> dict:
    """The `graph` document: `nodes` in the declaration's order and
    `connections` as `{from, to}`, each node with its `state` and the
    findings of the port or the unit it belongs to."""
    from nxs.check import _belongs
    from nxs.finding import as_data

    graph = cfg.graph
    port_entries = {e["name"]: e for e in ports}
    unit_entries = {e["name"]: e for e in units}
    nodes = []
    for node in graph.nodes.values():
        group = _belongs(graph, node.name)
        entry: Dict[str, Any] = {"name": node.name, "kind": node.kind, "type": node.type,
                                 "parameters": _plain(node.parameters),
                                 "state": _state(cfg, node, port_entries, unit_entries)}
        if group is not None and group.startswith("ports."):
            entry["port"] = group.split(".", 1)[1]
        mine = [f for f in by_node.get(group, []) if _names(f, node.name)] if group else []
        entry["findings"] = as_data(mine)
        nodes.append(entry)
    connections = [{"from": f"{e.source[0]}.{e.source[1]}", "to": f"{e.target[0]}.{e.target[1]}"}
                   for e in graph.edges]
    return {"nodes": nodes, "connections": connections}


def _names(finding, name: str) -> bool:
    where = getattr(finding, "where", "") or ""
    return where == f"nodes.{name}" or where.startswith(f"nodes.{name}.")


def _state(cfg, node, ports: Dict[str, dict], units: Dict[str, dict]) -> str:
    """What the rig answers for one node, from the readings the port and
    unit sections took."""
    if node.kind == "host":
        return PRESENT
    if node.kind == "module":
        return UNKNOWN
    if node.kind == "unit":
        entry = units.get(node.name)
        if entry is None or entry.get("held"):
            return UNKNOWN
        return PRESENT if entry.get("ok") else ABSENT
    if node.kind == "click":
        unit = next((t[0] for t in cfg.graph.targets(node.name, "out")), None)
        entry = units.get(unit) if unit else None
        if entry is None or not entry.get("ok"):
            return UNKNOWN
        running = str(entry.get("personality") or "").lower()
        return PRESENT if running in _click_names(node.type) else ABSENT
    port, link = _port_of(cfg, node)
    presence = (ports.get(port) or {}).get("presence") if port else None
    if not presence or presence.get("held") or presence.get("error"):
        return UNKNOWN
    if node.kind == "hub":
        hub = presence.get("hub")
        return UNKNOWN if hub is None else (PRESENT if hub.get("present") else ABSENT)
    found = next((l for l in presence.get("links") or [] if l.get("name") == link), None)
    if found is None:
        return UNKNOWN
    return {"present": PRESENT, "no ACK": ABSENT}.get(found.get("sen"), UNKNOWN)


def _port_of(cfg, node) -> tuple:
    """The port and the link a hub or a camera serves, by the derived ports."""
    for name, port in cfg.ports.items():
        if port.hub_node == node.name:
            return name, None
        for link in port.links:
            if link.camera_node == node.name:
                return name, link.name
    return None, None


def _click_names(personality: str) -> set:
    """The names a click personality runs under, lowercased: its key and the
    class name its image carries."""
    from nxs.tree import _declared_personality_names

    return _declared_personality_names([personality])


def _plain(value):
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_plain(v) for v in value]
    return value


__all__ = ["ABSENT", "PRESENT", "UNKNOWN", "graph_view"]
