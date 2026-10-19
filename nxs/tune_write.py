# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The one writer of the declaration: the DECLARE knobs write what the rig
holds in the port view (`nxs.suite.port_view`), its nodes and connections
in api 3, and every block's rows section writes that node's parameters;
then the nodes are laid over the file's. `nxs tune`, its batch, the MCP
tools and `nxs generate` all write through it, and a save judges every
value it writes by the block's options there (`nxs.suite.views`)."""

from __future__ import annotations

import copy
import os
import time
from typing import Dict, Tuple

from nxs.tune_fields import NONE, Channel
from nxs.tune_model import CONNECTOR, ROWS, spelled


def read_document(path: str) -> dict:
    """The declaration's file as plain mappings; {} for none or an empty one."""
    import yaml

    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return {}
    with open(path, encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)
    return doc if isinstance(doc, dict) else {}


def draft(doc: dict, previous, channels, only=None) -> Tuple[dict, Dict[tuple, str]]:
    """The declaration the knobs make, laid over a copy of the file's
    document, and the node each role took (`port_view.nodes_and_roles`).
    `only` (a set of sections) narrows what the knobs write to the nodes
    they belong to."""
    from nxs.suite.port_view import merge, nodes_and_roles

    previous_view = declared_view(doc)[1] if doc else {}
    view = copy.deepcopy(previous_view)
    write_model(view, channels, only=only, strict=False)
    for key in ("ports", "units"):
        if key in view and not view[key]:
            del view[key]
    emitted, roles = nodes_and_roles(view, previous=previous, buses=_camera_buses())
    _overlay(emitted, roles, channels, previous, only)
    out = copy.deepcopy(doc) if doc else {}
    merge(out, emitted, previous)
    return out, roles


def save_model(path, channels, only=None):
    """Patch the declaration in place (timestamped .bak beside it): the
    model's nodes laid over the file's, every value the knobs write judged
    by its block's options first; `only` narrows the write to a set of
    sections."""
    from nxs.suite.freeze import _round_trip_yaml, _write_atomic
    from nxs.suite.port_view import merge, nodes_and_roles

    yaml_rt = _round_trip_yaml()
    existed = os.path.exists(path)
    if existed:
        with open(path, encoding="utf-8") as fh:
            doc = yaml_rt.load(fh) or {}
    else:
        doc = {}
    previous, view = declared_view(doc)
    had = bool(view.get("ports") or view.get("units"))
    write_model(view, channels, only=only)
    # A removal that empties the file is a save; a file with nothing to declare is not.
    if not had and not view.get("ports") and not view.get("units"):
        raise SystemExit("nxs tune: nothing declared to save — set a HUB and a link's CAM, "
                         "or a unit's CLICK, first")
    for key in ("ports", "units"):
        if key in view and not view[key]:
            del view[key]
    emitted, roles = nodes_and_roles(view, previous=previous, buses=_camera_buses())
    owned = _overlay(emitted, roles, channels, previous, only)
    _judge(doc, emitted, previous, owned, path)
    merge(doc, emitted, previous)
    backup = None
    if existed:
        backup = f"{path}.bak-{time.strftime('%Y%m%d-%H%M%S')}"
        with open(path, encoding="utf-8") as fh:
            original = fh.read()
        _write_atomic(backup, lambda f: f.write(original))
    # An emptied declaration is an empty file, the shape the loader reads as none.
    empty = not view.get("ports") and not view.get("units") and all(
        _is_host(n) for n in (doc.get("nodes") or {}).values())
    _write_atomic(path, (lambda f: f.write("")) if empty else (lambda f: yaml_rt.dump(doc, f)))
    return backup


def declaration(channels) -> dict:
    """The declaration the knobs make from an empty file, as `nxs generate`
    seeds it."""
    doc, _roles = draft({}, None, channels)
    return _plain(doc)


def _is_host(node) -> bool:
    from nxs import description

    if not isinstance(node, dict):
        return False
    try:
        return description.of_type(str(node.get("type"))).kind == "host"
    except (description.UnknownType, description.DescriptionError):
        return False


def declared_view(doc):
    """(the declared graph, its port view) of a declaration's document; (None,
    {}) for an empty one."""
    from nxs.suite.port_view import view_of
    from nxs.suite.schema import parse_suite_config

    if not doc:
        return None, {}
    cfg = parse_suite_config(_plain(doc))
    return cfg.graph, view_of(cfg)


def _camera_buses():
    """The host's camera connectors by name, with their buses; {} where the
    host names none."""
    from nxs import host as host_layer
    try:
        return dict(host_layer.current().camera_buses())
    except Exception:        # noqa: BLE001 (a host without port rules names no bus)
        return {}


# ── the rows into the nodes ──────────────────────────────────────────

def _overlay(emitted: dict, roles: Dict[tuple, str], channels, previous, only) -> set:
    """Each emitted node's parameters: a node a written rows section speaks
    for takes the section's knobs, which the save spells, beside the pins
    and the rows no section speaks for, a unit's node id on its CAN route
    among them, as the file gives them or else as the view wrote them; any
    other node the file names keeps its parameters; a new one keeps what
    the view wrote. Returns the nodes the sections wrote."""
    from nxs import description

    sections: Dict[str, list] = {}
    for channel in channels:
        if not _written(channel, only):
            continue
        for section in channel.sections:
            if section.kind == ROWS and roles.get(section.role):
                sections.setdefault(roles[section.role], []).append(section)
    for name, entry in emitted["nodes"].items():
        node = previous.nodes.get(name) if previous is not None else None
        kept = dict(node.parameters) if node is not None and node.type == entry.get("type") else None
        if name not in sections:
            if kept is not None:
                _set_parameters(entry, kept)
            continue
        try:
            block = description.of_type(str(entry.get("type")))
        except (description.UnknownType, description.DescriptionError):
            continue
        converted = dict(entry.get("parameters") or {})
        owned = set().union(*(_section_rows(block, section) for section in sections[name]))
        params = {k: v for k, v in converted.items() if k not in owned and k not in block.pins}
        params.update({k: v for k, v in (kept or {}).items() if k not in owned})
        params.update({k: v for k, v in converted.items() if k in block.pins and k not in owned})
        for section in sections[name]:
            shown = {f.name for f in section.fields}
            # A knob the operator leaves alone keeps what the structure implies,
            # a hub's lanes into a connector of two.
            params.update({k: v for k, v in converted.items() if k in shown and k in owned})
            params.update({k: _quoted(v) for k, v in spelled(section).items()})
        params = {k: v for k, v in params.items() if v != NONE}
        _set_parameters(entry, params)
    return set(sections)


#: The words YAML 1.1 reads as a boolean unless quoted.
_BOOLEAN_WORDS = frozenset({"on", "off", "yes", "no", "true", "false", "y", "n"})


def _quoted(value):
    """A value name the file writes quoted where an unquoted one would read
    back as a boolean."""
    from ruamel.yaml.scalarstring import SingleQuotedScalarString

    if isinstance(value, str) and value.lower() in _BOOLEAN_WORDS:
        return SingleQuotedScalarString(value)
    return value


def _section_rows(block, section) -> set:
    """The parameters a rows section writes: every row of its block, those
    named for its connector of a host's, a unit's firmware pin."""
    if section.role[0] == "host":
        return {row.name for row in block.rows if row.name.startswith(f"{section.role[1]}_")}
    if section.role[0] == "unit":
        return {"firmware"}
    return {row.name for row in block.rows}


def _set_parameters(entry: dict, params: dict) -> None:
    if params:
        entry["parameters"] = params
    else:
        entry.pop("parameters", None)


def _written(channel, only) -> bool:
    """Whether a save writes a channel's rows: every channel without `only`;
    with it, a channel of a port family any of whose channels it touches, or
    a unit it touches."""
    if only is None:
        return True
    family = getattr(channel, "family", None)
    members = [family.port, *family.links] if family is not None else [channel]
    return any(section in only for member in members for section in member.sections)


def _judge(doc, emitted: dict, previous, owned: set, path: str) -> None:
    """Refuse a save whose declaration does not load, or one in which a
    value a rows section writes is not offered there: the fact, then the
    values offered."""
    from nxs.suite import views
    from nxs.suite.port_view import merge
    from nxs.suite.schema import ManifestError, parse_suite_config

    candidate = copy.deepcopy(_plain(doc)) if doc else {}
    merge(candidate, copy.deepcopy(emitted), previous)
    try:
        cfg = parse_suite_config(candidate, os.path.basename(path))
    except ManifestError as exc:
        raise SystemExit(f"nxs tune: {exc}") from None
    refused = [f for f in views.findings(cfg)
               if str(f.where).split(".")[1:2] and str(f.where).split(".")[1] in owned]
    if refused:
        first = refused[0]
        lines = [f"nxs tune: {first.where}: {first.fact}", *(f"  - {a}" for a in first.alternatives)]
        raise SystemExit("\n".join(lines))


# ── the structure into the port view ─────────────────────────────────

def write_model(raw, channels, only=None, strict=True):
    """What the DECLARE knobs declare, in the port view: every port entry
    from its family's knobs, every unit entry from its channel's. `only` (a
    set of sections) narrows the write to the nodes they belong to; a pod
    declared through its link's POD knob or its own CLICK is written with
    its port. A node the knobs leave incomplete (a link's camera before its
    port's hub) is refused with the knob to set, or with `strict` False
    left out, as the model's rebuild drafts it."""
    families = {ch.family.name: ch.family for ch in channels if ch.kind == "port"}
    units = [ch for ch in channels if ch.kind == "unit"]

    def touched(ch):
        return only is None or any(sec in only for sec in ch.sections)

    riders = _pods_to_write(families, units)
    written, removed = set(), set()
    for family in families.values():
        if (touched(family.port) or any(touched(l) for l in family.links)
                or any(touched(ch) for (port, _letter), ch in riders.items() if port == family.name)):
            if _write_port(raw, family, riders, strict):
                removed.add(family.name)
            else:
                written.add(family.name)
    for ch in units:
        if ch.link_ref and ch.link_ref.split("/", 1)[0] in removed:
            # The pod went with its port.
            continue
        # A found pod the POD knob declares is written with its port, and a
        # declared pod its POD knob dropped goes with it; a declared unit is
        # otherwise written only when its own knobs are touched.
        rider = any(c is ch for c in riders.values())
        dropped = ch.declared and ch.link_ref and not rider and ch.link_ref.split("/", 1)[0] in written
        if touched(ch) or (not ch.declared and rider) or dropped:
            _write_unit(raw, ch, families, riders, explicit=only is not None and touched(ch),
                        strict=strict)


def _pods_to_write(families, units) -> Dict[tuple, Channel]:
    """(port, letter) -> the pod channel riding it that the save declares: a
    declared pod its link's POD knob keeps, or a found pod the POD knob or
    its own CLICK names."""
    out = {}
    for ch in units:
        if not ch.link_ref:
            continue
        port, letter = ch.link_ref.split("/", 1)
        family = families.get(port)
        link = family.link(letter) if family is not None else None
        pod = link.knobs().get("POD", NONE) if link is not None else NONE
        named = any(f.value != NONE for f in ch.declare().fields)
        if pod == ch.name or (not ch.declared and named):
            out[(port, letter)] = ch
    return out


def _write_port(raw, family, riders, strict=True):
    """Write the port's entry; True when the save removed it instead."""
    from nxs.suite.freeze import port_block

    ports = raw.setdefault("ports", {})
    entry = ports.get(family.name)
    hub = family.hub_value()
    if hub == NONE:
        if family.spec is not None and (family.spec.hub_compatible or family.spec.links):
            # (none) on a declared port: the port goes, with the pods riding it.
            _remove_port(raw, family.name)
            return True
        for link in family.links:
            if link.knobs().get("CAM", NONE) != NONE and strict:
                raise SystemExit(f"nxs tune: {link.name} declares a sensor, and {family.name} no hub "
                                 f"— set {family.name}:HUB first, in the same --set batch or before")
        return
    topology = family.topology()
    if hub == CONNECTOR and (topology is None or not topology.camera_links):
        # A connector port is its camera: (none) on the sensor removes a
        # declared port, and an entry without a camera declares nothing.
        if family.spec is not None and (family.spec.hub_compatible or family.spec.links):
            _remove_port(raw, family.name)
            return True
        letter = family.links[0].letter if family.links else "A"
        if not strict:
            return
        raise SystemExit(f"nxs tune: {family.name}: a camera on the connector declares with its "
                         f"sensor — set {family.name}/{letter}:CAM in the same --set batch")
    if topology is None:
        if entry is None:
            entry = ports[family.name] = {}
        entry["hub"] = hub
        return
    declared = copy.deepcopy(_plain(entry)) if isinstance(entry, dict) else (
        {"bus": family.pinned_bus} if family.pinned_bus else {})
    units = {}
    for link in family.links:
        pod = riders.get((family.name, link.letter))
        old = ((declared.get("links") or {}).get(link.letter) or {}) if declared else {}
        if pod is None:
            if isinstance(old, dict):
                old.pop("unit", None)
            continue
        if pod.declared:
            # The declared ref stands: the file's, else the parsed declaration's.
            units[link.letter] = (old["unit"] if isinstance(old.get("unit"), dict)
                                  else _declared_ref(family, link.letter, pod.name, hub == CONNECTOR))
        elif hub == CONNECTOR:
            route, _serial = pod.template
            units[link.letter] = {"name": pod.name} | (
                {"target": route.address} if route.address not in (None, 0x30) else {})
        else:
            from nxs.generate_seed import pod_alias
            units[link.letter] = {"name": pod.name, "alias": pod_alias(link.letter)}
    block = port_block(topology, units=units, declared=declared)
    if entry is None:
        ports[family.name] = block
    else:
        _assign(entry, block)


def _declared_ref(family, letter, name, connector):
    """A declared pod's `unit` ref from the parsed declaration: its alias
    behind a hub, its own address on the connector where it is not the one
    every NXS straps."""
    link = next((l for l in family.spec.links if l.name == letter), None) if family.spec else None
    unit = link.unit if link is not None else None
    if unit is None:
        return {"name": name}
    if connector:
        return {"name": unit.name} | ({"target": unit.target} if unit.target != 0x30 else {})
    return {"name": unit.name, "alias": unit.alias} | (
        {"target": unit.target} if unit.target != 0x30 else {})


def _write_unit(raw, ch, families, riders, explicit=False, strict=True):
    """Write the unit's entry; `explicit` says the batch named one of its
    knobs, so a unit nothing declares is refused rather than skipped, and
    `strict` False skips a unit the knobs leave incomplete."""
    names = [f.value for f in ch.declare().fields]
    personalities = [(i, n) for i, n in enumerate(names) if n != NONE]
    if ch.link_ref:
        port_name, _letter = ch.link_ref.split("/", 1)
        if not any(c is ch for c in riders.values()):
            if ch.declared:
                _remove_unit(raw, ch.name)
            elif explicit and strict:
                raise SystemExit(f"nxs tune: {ch.name} is not declared — set {ch.link_ref}:POD={ch.name} "
                                 f"or {ch.name}:CLICK in the same --set batch")
            return
        family = families.get(port_name)
        if family is None or family.hub_value() == NONE or port_name not in (raw.get("ports") or {}):
            if not strict:
                return
            raise SystemExit(f"nxs tune: {ch.name} rides {ch.link_ref}, and {port_name} declares no "
                             f"hub — set {port_name}:HUB first, in the same --set batch or before")
    elif not ch.declared and not personalities and not ch.adopt:
        if explicit and strict:
            raise SystemExit(f"nxs tune: {ch.name} is not declared — set {ch.name}:CLICK "
                             f"in the same --set batch")
        return
    units = raw.setdefault("units", [])
    entry = next((u for u in units if isinstance(u, dict) and u.get("name") == ch.name), None)
    if entry is None:
        if ch.template is not None:
            route, serial = ch.template
            routes = _routes(type("Spec", (), {"links": [route]})())
        else:
            # A declared unit written into a fresh document: its routes as declared.
            routes, serial = _routes(ch.unit_spec), getattr(ch.unit_spec, "serial", None)
        entry = {"name": ch.name, "module": "nxs", "links": routes}
        if _serial(serial):
            entry["serial"] = _serial(serial)
        units.append(entry)
    old = {i: s for i, s in enumerate(entry.get("sensors") or [])}
    sensors = []
    for i, personality in personalities:
        spec = old.get(i) if isinstance(old.get(i), dict) and old[i].get("personality") == personality else {}
        sensors.append({"personality": personality, **({"config": spec["config"]}
                                                       if spec.get("config") else {})})
    if sensors or "sensors" in entry:
        entry["sensors"] = sensors


def _routes(spec) -> list:
    """A declared unit's routes as the file spells them."""
    from nxs.suite.freeze import hex_address

    out = []
    for link in getattr(spec, "links", None) or []:
        route = {"transport": link.transport}
        if link.link_ref:
            route["link"] = link.link_ref
        else:
            if getattr(link, "connector", None):
                route["connector"] = link.connector
            for key in ("bus", "address", "iface", "node_id", "port", "baud"):
                value = getattr(link, key, None)
                if value is not None and not (key == "bus" and getattr(link, "connector", None)):
                    route[key] = hex_address(value) if key == "address" else value
        out.append(route)
    return out


def _serial(serial):
    """A serial the manifest parser takes back (the 12-byte UID96 as 24 hex
    digits), else None: a saved file that does not parse would wedge the
    bootstrap it exists to start."""
    from nxs.suite.schema import ManifestError, normalize_serial

    if not serial or serial == "?":
        return None
    try:
        return normalize_serial(serial, "save")
    except ManifestError:
        return None


def _remove_port(raw, name):
    """The port entry goes, with the units riding its links."""
    (raw.get("ports") or {}).pop(name, None)
    units = raw.get("units") or []
    for unit in list(units):
        routes = unit.get("links") or [] if isinstance(unit, dict) else []
        if any(isinstance(r, dict) and str(r.get("link", "")).split("/")[0] == name for r in routes):
            units.remove(unit)


def _remove_unit(raw, name):
    units = raw.get("units") or []
    for unit in list(units):
        if isinstance(unit, dict) and unit.get("name") == name:
            units.remove(unit)


def _plain(node):
    """A plain-dict copy of a round-trip YAML node."""
    if isinstance(node, dict):
        return {str(k): _plain(v) for k, v in node.items()}
    if isinstance(node, list):
        return [_plain(v) for v in node]
    return node


def _assign(entry, block):
    """`entry` becomes `block`, key by key: a key the block drops goes, a
    value that did not move keeps its node (its comment and its quoting),
    and a nested link mapping is treated the same way."""
    for key in [k for k in entry if k not in block]:
        del entry[key]
    for key, value in block.items():
        current = entry.get(key)
        if key == "links" and isinstance(current, dict) and isinstance(value, dict):
            for name in [n for n in current if n not in value]:
                del current[name]
            for name, link in value.items():
                if isinstance(current.get(name), dict) and isinstance(link, dict):
                    _assign(current[name], link)
                else:
                    current[name] = link
            continue
        if isinstance(current, dict) and isinstance(value, dict):
            _assign(current, value)
        elif current != value or type(current) is not type(value) and not isinstance(value, dict):
            entry[key] = value


__all__ = ["declaration", "declared_view", "draft", "read_document", "save_model", "write_model"]
