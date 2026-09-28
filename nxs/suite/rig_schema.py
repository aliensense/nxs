# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The rig's rules as one JSON Schema: the manifest schema narrowed by the
nodes that are on this rig. Each node brings its own rules, selected by its
`compatible` the way a devicetree binding is: a hub brings its links and its
frame sync, a sensor its modes and the rates each ships at, a unit's
personality the configuration keys it takes. A key no node on the rig brings
is not in the schema, and `additionalProperties: false` refuses it.

The schema is necessary, not sufficient. It judges names, values, and the
combinations inside one link, from the same laws and shipped points the tool
composes with. Arithmetic across nodes (two cameras on one line, lanes
against the hub's output) stays with those laws, and `nxs status` on the rig
is the judge."""

from __future__ import annotations

import copy
from typing import Any, Dict, List, Optional, Tuple

from nxs import experimental, schemas

#: What a hub's link carries and a sensor on the port's own bus does not.
_HUB_LINK_KEYS = ("ser", "des_window", "ser_addr", "tca_addr")

_NOTE = ("The rules of this rig, from the nodes on it. Necessary, not sufficient: "
         "names, values and the combinations inside one link are judged here; "
         "arithmetic across nodes is the laws', and `nxs status` on the rig is the judge.")


def _rate(flows, pack, topology, link, mode: str) -> Optional[Dict[str, Any]]:
    """The `fps` rule of one mode on a link: a range, or the one rate a
    table-only part runs it at; None where the laws refuse the mode here."""
    from nxs.cam.contracts import InfeasibleConfig

    try:
        rates = flows.fps_range(pack, topology, link, mode)
    except (InfeasibleConfig, KeyError, AttributeError):
        return None
    if rates.floor == rates.ceiling:
        return {"const": rates.ceiling}
    return {"minimum": rates.floor, "maximum": rates.ceiling}


def _mode_tokens(sen, names: List[str]) -> Dict[str, str]:
    """Token -> mode name: every mode name, and a `WxH` that names one mode."""
    tokens = {name: name for name in names}
    sizes: Dict[str, List[str]] = {}
    for name in names:
        geo = sen.modes[name].get("geometry") or {}
        sizes.setdefault(f"{geo.get('width')}x{geo.get('height')}", []).append(name)
    tokens.update({size: found[0] for size, found in sizes.items() if len(found) == 1})
    return tokens


def _offered_modes(sen, cameras: int, lanes: int) -> List[str]:
    from nxs.cam import shipped

    proven = list(shipped.shipped_modes(sen, cameras, lanes))
    if experimental.enabled():
        proven += [n for n in sen.program_modes() if n not in proven]
    return proven


def _default_mode(flows, pack, topology, link) -> Optional[str]:
    try:
        return str(flows.default_mode(pack, link, topology))
    except Exception:
        return None


def _camera_rule(pack, flows, topology, link) -> Dict[str, Any]:
    """A link's `camera`: a sensor the port's pack serves, or the mapping
    whose mode belongs to that sensor and whose rate belongs to that mode.
    Every served sensor gets its own rules, judged as if the link carried
    it; a mapping that names no mode is judged at the sensor's default."""
    import dataclasses

    from nxs.cam import shipped

    cameras, lanes = shipped.cameras(topology.links), int(topology.csi_lanes)
    served = sorted(pack.descriptor(chip).compatible for chip in pack.sensors())
    rules = []
    for compatible in served:
        sen = pack.descriptor(compatible)
        as_if = dataclasses.replace(link, sensor_compatible=compatible)
        tokens = _mode_tokens(sen, _offered_modes(sen, cameras, lanes))
        per_mode = []
        for token, mode in sorted(tokens.items()):
            rate = _rate(flows, pack, topology, as_if, mode)
            if rate is not None:
                per_mode.append({"if": {"properties": {"mode": {"const": token}},
                                        "required": ["mode"]},
                                 "then": {"properties": {"fps": rate}}})
        default = _default_mode(flows, pack, topology, as_if)
        rate = _rate(flows, pack, topology, as_if, default) if default else None
        if rate is not None:
            per_mode.append({"if": {"not": {"required": ["mode"]}},
                             "then": {"properties": {"fps": rate}}})
        rule = {"properties": {"mode": {"enum": sorted(tokens)}},
                **({"allOf": per_mode} if per_mode else {})}
        rules.append({"if": {"properties": {"sensor": {"const": compatible}},
                             "required": ["sensor"]}, "then": rule})
        if compatible == link.sensor_compatible:
            # A mapping that names no sensor runs the link's: the port's
            # default, else the sensor the wiring found there.
            rules.append({"if": {"not": {"required": ["sensor"]}}, "then": rule})
    mapping = {"type": "object", "additionalProperties": False, "minProperties": 1,
               "properties": {"sensor": {"enum": served}, "mode": {"type": "string"},
                              "fps": {"type": "number", "exclusiveMinimum": 0}},
               "allOf": rules}
    # `if/then/else`, never `anyOf`: a violation then names the key it broke
    # and the values that key admits.
    return {"if": {"type": "object"}, "then": mapping, "else": {"enum": served}}


def _direct_link_rules(base: Dict[str, Any]) -> Dict[str, Any]:
    """The base schema's per-link rules for a port without a hub, keyed by
    link property: the branch of the port's `allOf` that applies when no
    hub is declared."""
    for clause in base["$defs"]["port"]["allOf"]:
        if clause.get("if", {}).get("required") == ["hub"] and "else" in clause:
            return clause["else"]["properties"]["links"]["additionalProperties"]["properties"]
    raise KeyError("suite.schema.json: no hub-less link rules in the port's allOf")


def _port_rule(base: Dict[str, Any], port) -> Dict[str, Any]:
    from nxs.cam import packs
    from nxs.cam import topology as cam_topo
    from nxs.suite.schema import SYNC_SOURCES

    rule = copy.deepcopy(base["$defs"]["port"])
    # The wiring file states the bus and each link's wiring: the intent may
    # leave them out, and may not contradict them.
    rule.pop("required", None)
    rule.pop("allOf", None)
    props = rule["properties"]
    if port.bus:
        props["bus"] = {"const": port.bus}
    props["csi_lanes"] = {"const": int(port.csi_lanes)}
    if port.hub_compatible is None:
        del props["hub"]
        sources = ["free_run"]
    else:
        props["hub"] = {"anyOf": [{"const": port.hub_compatible}, {
            "type": "object", "properties": {"compatible": {"const": port.hub_compatible}}}]}
        sources = list(SYNC_SOURCES)
    props["sync"] = copy.deepcopy(props["sync"])
    props["sync"]["properties"]["source"] = {"enum": sources}
    camera = copy.deepcopy(base["$defs"]["camera"])
    if "sync" in camera.get("properties", {}):
        camera["properties"]["sync"] = {"enum": sources}
    props["camera"] = camera
    try:
        topology = cam_topo.port_topology(port)
        pack = packs.pack_for(topology)
        flows = pack.flows()
    except Exception:
        return rule          # a pack that does not load is `nxs status`'s finding
    resolved = {link.name: link for link in topology.links}
    links: Dict[str, Any] = {}
    for declared in port.links:
        entry = copy.deepcopy(base["$defs"]["port_link"])
        if port.hub_compatible is None:
            for key in _HUB_LINK_KEYS:
                entry["properties"].pop(key, None)
            # The parser's rules for a link on the sensor's own lanes (virtual
            # channel 0, a unit at its own address), as the base states them.
            for key, narrowed in _direct_link_rules(base).items():
                entry["properties"][key] = {"allOf": [entry["properties"][key], narrowed]}
        link = resolved.get(declared.name)
        if link is None:
            # A link that names no camera yet: the key stays, and the schema
            # asks for the camera the way `check` does.
            entry["required"] = ["camera"]
        else:
            entry["properties"]["camera"] = _camera_rule(pack, flows, topology, link)
        links[declared.name] = entry
    # The rig's links and no other, whatever node brings them.
    props["links"] = {"type": "object", "properties": links, "additionalProperties": False}
    if port.hub_compatible is None:
        props["links"]["maxProperties"] = 1
    return rule


def _unit_rules(cfg) -> Tuple[List[str], List[Dict[str, Any]]]:
    """The personalities a unit may name (every one the tool knows, as the
    manifest spells them) and, per personality, the configuration keys it
    takes, applied to the `sensors[]` entry that names it. The rig's own
    personalities are judged at their declared configuration."""
    from nxs.check import sensor_allowed_keys
    from nxs.suite.reconcile import DriverNotFound, known_driver_modules, load_unit_driver

    declared = {spec.driver: dict(spec.config) for unit in cfg.units
                for spec in (unit.sensors or [])}
    admitted: List[str] = []
    rules = []
    for name in sorted(set(known_driver_modules()) | set(declared)):
        try:
            driver = load_unit_driver(name)
            allowed = sorted(sensor_allowed_keys(driver, driver().compile(declared.get(name, {}))))
        except (DriverNotFound, Exception):
            continue     # a personality that does not load is `nxs status`'s finding
        names = sorted({name, name.replace("_", "-")})
        admitted.extend(names)
        rules.append({
            "if": {"properties": {"personality": {"enum": names}},
                   "required": ["personality"]},
            "then": {"properties": {"config": {"propertyNames": {"enum": allowed}}}}})
    return sorted(admitted), rules


def rig_schema(cfg) -> Dict[str, Any]:
    """The manifest schema narrowed by the rig `cfg` declares (its wiring
    laid under its intent)."""
    base = copy.deepcopy(schemas.load(schemas.SUITE))
    base["$id"] = "urn:aliensense:nxs:rig-schema"
    base["title"] = "suite.yaml on this rig"
    base["description"] = _NOTE
    base["properties"]["ports"]["properties"] = {
        name: _port_rule(base, port) for name, port in sorted(cfg.ports.items())}
    # The rig's ports and no other.
    base["properties"]["ports"]["additionalProperties"] = False
    admitted, rules = _unit_rules(cfg)
    if rules:
        base["properties"]["units"] = {"allOf": [
            base["properties"]["units"],
            {"items": {"properties": {"sensors": {"items": {
                "properties": {"personality": {"enum": admitted}}, "allOf": rules}}}}}]}
    return base
