# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The rig's views: each node of a declaration as the options hook sees it,
its description read once, its declared values, the peers wired to each of
its ports, the other nodes through the rig, and the camera port its chain
shapes. `nxs status`, the panel and `nxs tune --schema` judge a declared
value through these views and nothing else."""

from __future__ import annotations

import functools
from typing import Callable, Dict, List, Optional, Tuple

from nxs.finding import Finding
from nxs.suite.options import BlockView, Peer, judge, nearest, options_for


def views(cfg) -> Dict[str, BlockView]:
    """Every node of a parsed declaration (`SuiteConfig`) by name, as a
    view: its peers by its own port, the rig's other views by name, and the
    camera port its chain shapes where it rides one."""
    graph = cfg.graph
    out: Dict[str, BlockView] = {}
    if graph is None:
        return out
    ports = _ports_by_node(cfg)
    for node in graph.nodes.values():
        peers: Dict[str, List[Peer]] = {}
        for edge in graph.edges:
            for mine, other in ((edge.source, edge.target), (edge.target, edge.source)):
                if mine[0] != node.name:
                    continue
                peer = graph.nodes[other[0]]
                peers.setdefault(mine[1], []).append(
                    Peer(node=peer.name, port=other[1], description=peer.description,
                         declared=dict(peer.values)))
        out[node.name] = BlockView(description=node.description, declared=dict(node.values),
                                   peers={port: tuple(found) for port, found in peers.items()},
                                   node=node.name, rig=out.get,
                                   topology=_topology_of(ports.get(node.name)))
    return out


def draft(doc: dict, where: str) -> Tuple[Optional[object], Dict[str, BlockView]]:
    """`(graph, views)` of a draft the laws refuse, read leniently
    (`parse_graph(lenient=True)`): each node with the values that parse and
    its peers by every connection, and a camera port where the draft still
    derives one. `(None, {})` where nothing reads."""
    import os
    from types import SimpleNamespace

    from nxs.suite.nodes import parse_graph
    from nxs.suite.schema_base import ManifestError
    from nxs.suite.schema_nodes import derive

    try:
        graph = parse_graph(doc, where, lenient=True)
    except ManifestError:
        return None, {}
    try:
        ports, _units = derive(graph, where, os.path.basename(where) or "suite.yaml")
    except ManifestError:
        ports = {}
    return graph, views(SimpleNamespace(graph=graph, ports=ports))


def _ports_by_node(cfg) -> Dict[str, object]:
    """The camera port each hub and each camera node shapes, by node name."""
    out: Dict[str, object] = {}
    for port in (cfg.ports or {}).values():
        if port.hub_node:
            out[port.hub_node] = port
        for link in port.links:
            if link.camera_node:
                out[link.camera_node] = port
    return out


def _topology_of(port) -> Optional[Callable[[], object]]:
    """The camera laws' view of a declared port, built once on first use;
    None where the node shapes no port. A port this host cannot shape (no
    bus answers to its name) reads None."""
    if port is None:
        return None

    @functools.lru_cache(maxsize=1)
    def topology():
        from nxs.cam.topology import port_topology
        try:
            return port_topology(port)
        except Exception:        # noqa: BLE001 (a port the host cannot shape is status's finding)
            return None
    return topology


def findings(cfg) -> List[Finding]:
    """A finding for every declared value a node's options refuse here,
    each at its node path with the values nearest it."""
    out: List[Finding] = []
    for name, view in views(cfg).items():
        if not view.declared:
            continue
        options = options_for(view)
        for row_name, raw in view.declared.items():
            try:
                row = view.description.row(row_name)
            except KeyError:
                continue        # a unit's own row, judged where the unit answers
            refusal = judge(view, options, row_name, raw)
            if refusal is not None:
                out.append(Finding(f"nodes.{name}.parameters.{row_name}", refusal,
                                   nearest(row, options.get(row_name), raw)))
    return out


#: What the rig's schema says of itself.
_NOTE = ("The rules of this rig: the declaration schema with each node's parameters narrowed "
         "to the rows its block offers here and the values each takes. Necessary, not "
         "sufficient: the connections' laws are the loader's, and `nxs status` judges the rig.")


def schema(cfg) -> Dict[str, object]:
    """The declaration schema narrowed by the rig `cfg` declares: each
    node's type pinned and its parameters the JSON Schema its description
    and its options give here (`description.json_schema`)."""
    import copy

    from nxs import schemas
    from nxs.description import json_schema

    out = copy.deepcopy(schemas.load(schemas.SUITE))
    out["$id"] = "urn:aliensense:nxs:rig-schema"
    out["title"] = "suite.yaml on this rig"
    out["description"] = _NOTE
    named = {}
    for name, view in views(cfg).items():
        node = cfg.graph.nodes[name]
        parameters = json_schema(view.description, options_for(view))
        if not view.description.live:
            # A unit's own rows are read from the unit: a declared one holds a number.
            parameters["additionalProperties"] = {"type": "integer"}
        named[name] = {"allOf": [{"$ref": "#/$defs/node"},
                                 {"properties": {"type": {"const": node.type},
                                                 "parameters": parameters}}]}
    out["properties"]["nodes"] = dict(out["properties"]["nodes"])
    out["properties"]["nodes"]["properties"] = named
    return out


__all__ = ["draft", "findings", "schema", "views"]
