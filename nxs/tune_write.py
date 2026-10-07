# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The one writer of the declaration: every port entry from its family's
knobs through `freeze.port_block`, every unit entry from its channel's;
`nxs tune`, its batch, the MCP tools and `nxs generate` all save through it."""

from __future__ import annotations

import copy
import os
import time
from typing import Dict

from nxs import tune_camera as laws
from nxs.tune_fields import NONE, Channel
from nxs.tune_model import CONNECTOR, _token


def save_model(path, channels, only=None):
    """Patch the manifest in place (timestamped .bak beside it); `only`
    narrows the write to a set of sections."""
    from nxs.suite.freeze import _round_trip_yaml, _write_atomic

    yaml_rt = _round_trip_yaml()
    existed = os.path.exists(path)
    if existed:
        with open(path, encoding="utf-8") as fh:
            raw = yaml_rt.load(fh) or {}
    else:
        raw = {}
    had = bool(raw.get("ports") or raw.get("units"))
    write_model(raw, channels, only=only)
    # A removal that empties the file is a save; a file with nothing to declare is not.
    if not had and not raw.get("ports") and not raw.get("units"):
        raise SystemExit("nxs tune: nothing declared to save — set a HUB and a link's SENSOR, "
                         "or a unit's PERSONALITY, first")
    for key in ("ports", "units"):
        if key in raw and not raw[key]:
            del raw[key]
    backup = None
    if existed:
        backup = f"{path}.bak-{time.strftime('%Y%m%d-%H%M%S')}"
        with open(path, encoding="utf-8") as fh:
            original = fh.read()
        _write_atomic(backup, lambda f: f.write(original))
    # An emptied declaration is an empty file, the shape the loader reads as none.
    _write_atomic(path, (lambda f: yaml_rt.dump(raw, f)) if raw else (lambda f: f.write("")))
    return backup


def write_model(raw, channels, only=None):
    """The one writer of the declaration: every port entry from its family's
    knobs through `freeze.port_block`, every unit entry from its channel's.
    `only` (a set of sections) narrows the write to the nodes they belong
    to; a pod declared through its link's POD knob or its own PERSONALITY
    is written with its port."""
    families = {ch.family.name: ch.family for ch in channels if ch.kind == "port"}
    units = [ch for ch in channels if ch.kind == "unit"]

    def touched(ch):
        return only is None or any(sec in only for sec in ch.sections)

    riders = _pods_to_write(families, units)
    written, removed = set(), set()
    for family in families.values():
        if (touched(family.port) or any(touched(l) for l in family.links)
                or any(touched(ch) for (port, _letter), ch in riders.items() if port == family.name)):
            if _write_port(raw, family, riders):
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
            _write_unit(raw, ch, families, riders, explicit=only is not None and touched(ch))


def _pods_to_write(families, units) -> Dict[tuple, Channel]:
    """(port, letter) -> the pod channel riding it that the save declares: a
    declared pod its link's POD knob keeps, or a found pod the POD knob or
    its own PERSONALITY names."""
    out = {}
    for ch in units:
        if not ch.link_ref:
            continue
        port, letter = ch.link_ref.split("/", 1)
        family = families.get(port)
        link = family.link(letter) if family is not None else None
        pod = _token(link.knobs().get("POD", NONE)) if link is not None else NONE
        named = any(f.value != NONE for f in ch.declare().fields)
        if pod == ch.name or (not ch.declared and named):
            out[(port, letter)] = ch
    return out


def _write_port(raw, family, riders):
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
            if link.knobs().get("SENSOR", NONE) != NONE:
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
        raise SystemExit(f"nxs tune: {family.name}: a camera on the connector declares with its "
                         f"sensor — set {family.name}/{letter}:SENSOR in the same --set batch")
    if topology is None:
        if entry is None:
            entry = ports[family.name] = {}
        entry["hub"] = hub
        return
    hub_obj = laws.hub_for(topology)
    if hub_obj is not None:
        refusal = laws.camera_refusal(hub_obj, topology, family.modes(), family.rates(), family.sync())
        if refusal:
            raise SystemExit(f"nxs tune: {family.name}: {refusal}")
    # What the file spells out: the declared values and the knobs stepped off
    # the laws' defaults; a default left alone stays the laws'.
    modes, rates, sync = family.modes(written=True), family.rates(written=True), family.sync(written=True)
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
    gain = family.gain()
    if gain is None and isinstance(declared.get("camera"), dict):
        declared["camera"].pop("gain_db", None)
    block = port_block(topology, sync=sync, modes=modes, rates=rates, gain_db=gain, units=units,
                       declared=declared)
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


def _write_unit(raw, ch, families, riders, explicit=False):
    """Write the unit's entry; `explicit` says the batch named one of its
    knobs, so a unit nothing declares is refused rather than skipped."""
    names = [f.value for f in ch.declare().fields]
    personalities = [(i, n) for i, n in enumerate(names) if n != NONE]
    if ch.link_ref:
        port_name, _letter = ch.link_ref.split("/", 1)
        if not any(c is ch for c in riders.values()):
            if ch.declared:
                _remove_unit(raw, ch.name)
            elif explicit:
                raise SystemExit(f"nxs tune: {ch.name} is not declared — set {ch.link_ref}:POD={ch.name} "
                                 f"or {ch.name}:PERSONALITY in the same --set batch")
            return
        family = families.get(port_name)
        if family is None or family.hub_value() == NONE or port_name not in (raw.get("ports") or {}):
            raise SystemExit(f"nxs tune: {ch.name} rides {ch.link_ref}, and {port_name} declares no "
                             f"hub — set {port_name}:HUB first, in the same --set batch or before")
    elif not ch.declared and not personalities and not ch.adopt:
        if explicit:
            raise SystemExit(f"nxs tune: {ch.name} is not declared — set {ch.name}:PERSONALITY "
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
    sections = {s.sensor_index: s for s in ch.sections if s.kind == "sensor"}
    old = {i: s for i, s in enumerate(entry.get("sensors") or [])}
    sensors = []
    for i, personality in personalities:
        spec = old.get(i) if isinstance(old.get(i), dict) and old[i].get("personality") == personality else {}
        config = dict(spec.get("config") or {})
        section = sections.get(i)
        for f in (section.fields if section is not None else []):
            # A value at its compiled default stays implicit unless the file names it.
            if f.name in config or f.value != f.default:
                config[f.name] = f.value
        sensors.append({"personality": personality, "config": config} if config else {"personality": personality})
    if sensors or "sensors" in entry:
        entry["sensors"] = sensors
    _set_or_drop(entry, "orientation", _single(ch, "mount"))
    decimation = _single(ch, "egress")
    egress = entry.get("egress") if isinstance(entry.get("egress"), dict) else {}
    _set_or_drop(egress, "decimation", decimation)
    if egress:
        entry["egress"] = egress
    else:
        entry.pop("egress", None)
    _set_or_drop(entry, "firmware", _single(ch, "firmware"))


def _routes(spec) -> list:
    """A declared unit's routes as the file spells them."""
    from nxs.suite.freeze import hex_address

    out = []
    for link in getattr(spec, "links", None) or []:
        route = {"transport": link.transport}
        if link.link_ref:
            route["link"] = link.link_ref
        else:
            for key in ("bus", "address", "iface", "node_id", "port", "baud"):
                value = getattr(link, key, None)
                if value is not None:
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


def _single(ch, kind):
    section = next((s for s in ch.sections if s.kind == kind), None)
    if section is None:
        return None
    value = section.fields[0].value
    return None if value == NONE else value


def _set_or_drop(mapping, key, value):
    if value is None:
        mapping.pop(key, None)
    else:
        mapping[key] = value


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
