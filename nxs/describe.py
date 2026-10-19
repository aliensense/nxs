# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""`nxs describe <type>`: what a node of a type offers a declaration, from
its description alone: its kind, its ports, the identity pins it takes and
its rows with the values each admits, in the form a declaration writes."""

from __future__ import annotations

from typing import Any, Dict, Mapping

from nxs._generated_constants import NxsDriverImage, NxsMap
from nxs.description import (Description, DescriptionError, UnknownType, direction_word, display,
                             named_values, number, of_type, port_type_word, unit_of)
from nxs.suite.options import BlockView, Option, options_for

ParamId = NxsMap.ParamId
PortDirection = NxsMap.PortDirection
PortType = NxsMap.PortType
#: A row's kind as the document names it.
KIND_WORDS = {value: name.lower() for value, name in NxsDriverImage.ParamKind._NAMES.items()}


def describe(node_type: str) -> Dict[str, Any]:
    """The `describe` document of a node type (`description.of_type`), its
    rows' options those of the block alone."""
    description = of_type(node_type)
    return document(description, options_for(BlockView(description)))


def document(description: Description, options: Mapping[str, Option]) -> Dict[str, Any]:
    """The document of a description: the surface `describe` of the
    contract."""
    from nxs.schemas import CONTRACT

    doc = {"contract": CONTRACT, "type": description.type, "kind": description.kind,
           "name": description.name, "product": description.identity.product,
           "live": description.live, "pins": list(description.pins),
           "ports": [_port(description, port) for port in description.ports],
           "parameters": [_parameter(row, options.get(row.name) or Option())
                          for row in description.rows]}
    forms = getattr(description.buses, "forms", ()) if description.buses is not None else ()
    if forms:
        doc["buses"] = [{"name": name, "type": port_type_word(kind), "direction": direction_word(way),
                         **({"attributes": {attr: f"<port>_{attr}" for attr in attrs}} if attrs else {})}
                        for name, kind, way, attrs in forms]
    return doc


def _port(description: Description, port) -> Dict[str, Any]:
    entry: Dict[str, Any] = {"name": port.name, "type": port_type_word(port.type),
                             "direction": direction_word(port.direction)}
    fed = PortType.FEEDS.get(port.type, ())
    if port.direction != PortDirection.INPUT and len(fed) > 1:
        entry["feeds"] = [port_type_word(t) for t in fed]
    attributes = description.attributes(port.name)
    if attributes:
        entry["attributes"] = {attr: row.name for attr, row in attributes.items()}
    return entry


def _parameter(row, option: Option) -> Dict[str, Any]:
    entry: Dict[str, Any] = {"name": row.name, "kind": KIND_WORDS.get(int(row.kind), str(row.kind)),
                             "default": display(row, row.default), "applicable": option.applicable}
    if int(row.id) != ParamId.CUSTOM:
        entry["id"] = ParamId._NAMES.get(int(row.id), str(row.id))
    unit = unit_of(row)
    if unit and row.present.labels is None:
        entry["unit"] = unit
    values = option.values if option.values is not None else named_values(row, option.span)
    if values is None and not row.is_range and option.span is None:
        values = row.values
    span = option.span if option.span is not None else (
        tuple(row.values) if row.is_range and values is None else None)
    if values is not None:
        entry["values"] = [display(row, v) for v in values]
    elif span is not None:
        lo, hi, step = span
        entry["minimum"], entry["maximum"] = display(row, lo), display(row, hi)
        if step > 1:
            entry["step"] = number(row, step)
    if row.is_read_only:
        entry["read_only"] = True
    if option.finding:
        entry["finding"] = option.finding
    return entry


def render(doc: Dict[str, Any]) -> str:
    """The document as lines a person reads: the kind, each port, each pin,
    each parameter with what it admits."""
    product = f", {doc['product']}" if doc.get("product") else ""
    lines = [f"{doc['type']}: a {doc['kind']}{product}"]
    for port in doc["ports"]:
        feeds = f", feeds {' or '.join(port['feeds'])}" if port.get("feeds") else ""
        attributes = port.get("attributes") or {}
        stated = (f" ({', '.join(f'{attr} in {row}' for attr, row in attributes.items())})"
                  if attributes else "")
        lines.append(f"  port {port['name']}: {port['type']} {port['direction']}{feeds}{stated}")
    for bus in doc.get("buses") or ():
        attributes = bus.get("attributes") or {}
        stated = (f", {', '.join(f'{attr} in {row}' for attr, row in attributes.items())}"
                  if attributes else "")
        lines.append(f"  port {bus['name']}: {bus['type']} {bus['direction']}, a bus by its "
                     f"device's name{stated}")
    for pin in doc["pins"]:
        lines.append(f"  pin {pin}")
    for param in doc["parameters"]:
        unit = f" {param['unit']}" if param.get("unit") else ""
        if not param["applicable"]:
            lines.append(f"  parameter {param['name']}: {param.get('finding', 'not declared')} "
                         f"({param['kind']})")
            continue
        if "values" in param:
            admits = ", ".join(param["values"])
        elif "minimum" in param:
            admits = f"{param['minimum']}..{param['maximum']}"
            if param.get("step"):
                admits += f" by {param['step']}"
        else:
            admits = "any value"
        note = ", read-only" if param.get("read_only") else ""
        lines.append(f"  parameter {param['name']}: {admits}{unit}, default {param['default']} "
                     f"({param['kind']}{note})")
    if not doc.get("live", True):
        lines.append("  rows: the unit's own, read from a live unit")
    return "\n".join(lines) + "\n"


def add_parser(sub) -> None:
    p = sub.add_parser('describe', help="A node type's ports and parameters: a host plugin, "
                                        "a hub, the unit module nxs, a cam or click personality")
    p.add_argument('type', help="The node type, as a declaration names it: a host plugin, a hub's "
                                "deserializer compatible, nxs, a cam or click personality")
    p.add_argument('--json', action='store_true', help='The describe surface (contract 2)')


def cmd_describe(args) -> int:
    import json

    from nxs import host as host_layer
    from nxs import term

    try:
        doc = describe(args.type)
    except UnknownType:
        host = host_layer.current()
        here = [f"nxs describe {host.name} (this host)"] if host.description() is not None else []
        term.refusal(f"no node type {args.type}: no host plugin, hub, unit module or personality "
                     f"of that name is installed",
                     *here, "nxs describe nxs (a unit)",
                     "nxs personality install <directory> (a personality of your own)")
        return 1
    except DescriptionError as exc:
        term.refusal(str(exc), "nxs personality check <directory>")
        return 1
    if args.json:
        print(json.dumps(doc, indent=2))
    else:
        print(render(doc), end="")
    return 0


__all__ = ["UnknownType", "add_parser", "cmd_describe", "describe", "document", "render"]
