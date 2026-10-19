# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The panel's model: one channel per node of the rig (a port, each of its
links, a pod or a unit). A channel's DECLARE knobs choose what the rig
holds, a port's hub, a link's camera and pod, a unit's clicks; each block
the declaration then names opens one rows section, each knob a row its
description offers at that point of the graph with the values its options
admit there (`nxs.suite.views`). Every step re-asks every block's options
on the declaration the knobs make, so a step on one block narrows or hides
another's rows. The one writer of the declaration is `nxs.tune_write`."""

from __future__ import annotations

import dataclasses
import os
from decimal import Decimal
from typing import Any, Dict, List, Optional, Tuple

from nxs.tune_fields import (NONE, Channel, Field, Section, _EmptyConfig, _platform_port_buses, token,
                             _platform_ports, _unit_name)
from nxs.tune_sweep import ROUTES, pods as swept_pods, presence, sweep as sweep_buses

#: The HUB of a camera on the host's connector: a port without a hub.
CONNECTOR = "connector"
#: A block's section: its rows as knobs.
ROWS = "rows"
#: The steps a range row's knob offers between its ends; a batch takes any value inside.
_LADDER = 50


# ---------------------------
# What is installed
# ---------------------------

def installed_hubs() -> List[str]:
    """The compatibles of every deserializer the installed hubs serve."""
    from nxs.cam import hubs

    out: List[str] = []
    try:
        found = hubs.discover()
    except Exception:        # noqa: BLE001 (no hub installed: nothing to offer)
        return out
    for hub in found:
        for chip in hub.chips:
            try:
                d = hub.descriptor(chip)
            except Exception:        # noqa: BLE001 (a chip that does not load is status's finding)
                continue
            if d.role == "DES" and d.compatible not in out:
                out.append(d.compatible)
    return out


def hub_product(compatible: str) -> Optional[str]:
    """The product name the installed hub prints for a deserializer
    compatible; None where no hub names one."""
    from nxs.cam import hubs

    try:
        return hubs.product_name(compatible)
    except Exception:        # noqa: BLE001 (no hub installed: the compatible stands)
        return None


def installed_sensors() -> List[str]:
    """The compatibles of every installed cam personality: a head is its own
    node, and any hub's link or the connector may carry it."""
    from nxs.cam import cam_personalities

    try:
        registry = cam_personalities.registry()
        return list(dict.fromkeys(registry.find(name).compatible for name in registry.names()))
    except Exception:        # noqa: BLE001 (a store that does not read offers nothing)
        return []


def hub_for(topology):
    """The hub, or the connector's flows, serving `topology`; None when nothing
    installed does, or without a topology."""
    from nxs.cam import hubs

    if topology is None:
        return None
    try:
        return hubs.for_topology(topology)
    except Exception:        # noqa: BLE001 (the finding is `status`'s)
        return None


# ---------------------------
# A block's rows as knobs
# ---------------------------

def row_token(row, raw: int):
    """A served value as the declaration writes it and a knob offers it:
    the name where the row names its values, else the number in the row's
    display form, whole where it is one."""
    from nxs.description import display, present

    text = display(row, int(raw))
    if present(row).labels is not None:
        return text
    number = Decimal(text)
    return int(number) if number == number.to_integral_value() else float(number)


def _ladder(lo: int, hi: int, step: int, current: Optional[int]) -> List[int]:
    """A range's ends and the values a knob steps through between them."""
    step = max(1, step)
    count = (hi - lo) // step
    stride = max(1, count // _LADDER) * step
    values = set(range(lo, hi + 1, stride)) | {lo, hi}
    if current is not None and lo <= current <= hi and (current - lo) % step == 0:
        values.add(current)
    return sorted(values)


def row_field(row, option, raw: int, declared: bool) -> Field:
    """A row's knob: the values its option admits in display form, opened on
    `raw` where admitted, else on the admitted value nearest it. A range
    row's knob steps a ladder and takes any admitted value from a batch.
    `declared` says the file names the value, which a save then keeps."""
    from nxs.description import named_values, unit_of

    span = option.span if option.span is not None else (
        tuple(row.values) if row.is_range and option.values is None else None)
    if option.values is not None:
        raws = list(option.values)
    elif named_values(row, span) is not None:
        raws = list(named_values(row, span))
    elif span is not None:
        raws = _ladder(span[0], span[1], span[2], raw)
    else:
        raws = list(row.values)
    pick = raw if option.admits(raw) and (raw in raws or span is not None) else _nearest(raws, raw)
    if pick not in raws:
        raws = sorted(set(raws) | {pick})
    unit = unit_of(row) if (row.present is None or row.present.labels is None) else ""
    field = Field(row.name, [row_token(row, v) for v in raws], raws.index(pick),
                  render=lambda value, u=unit: f"{value} {u}".rstrip(), unit=unit or None)
    field.row, field.option, field.raws = row, option, raws
    field.default = None if declared else row_token(row, row.default)
    if span is not None and option.values is None:
        field.bounds = (span[0], span[1])
    return field


def field_raw(field) -> int:
    """The served value a row knob holds."""
    return field.raws[field.index]


def admit(field, text: str) -> Optional[int]:
    """The index a row knob takes for `text` from a batch: an offered token,
    else any value its option admits, added to the knob; None for neither."""
    from nxs.description import DescriptionError, parse

    for i, option in enumerate(field.options):
        if str(token(option)).lower() == text.lower():
            return i
    row = getattr(field, "row", None)
    if row is None:
        return None
    try:
        raw = parse(row, text)
    except DescriptionError:
        return None
    if not field.option.admits(raw):
        return None
    raws = sorted(set(field.raws) | {raw})
    field.raws, field.options = raws, [row_token(row, v) for v in raws]
    return raws.index(raw)


def _nearest(values: List[int], wanted: int) -> int:
    return min(values, key=lambda v: (abs(v - wanted), v)) if values else wanted


def _spelled(field) -> bool:
    """Whether a save spells the knob out: a value the file declares or one
    the operator set. A knob left alone stays the block's to default, the
    value it opened on included where its row's default is not offered."""
    return field.default is None


def spelled(section) -> Dict[str, Any]:
    """A rows section's knobs a save writes, row name to token, a pin set
    to (none) among them."""
    return {f.name: token(f.value) for f in section.fields if _spelled(f)}


# ---------------------------
# The model
# ---------------------------

class Model:
    """The channels and what their rows rebuild from: the declaration's
    file as loaded and its graph. Every rows section's rebuild is the
    model's (`rebuild`)."""

    def __init__(self, path: str, doc: dict, previous, channels: List[Channel]):
        self.path, self.doc, self.previous, self.channels = path, doc, previous, channels

    def rebuild(self) -> Tuple[Optional[str], bool]:
        """Re-ask every block's options on the declaration the knobs make and
        rebuild each rows section: a knob keeps its value where its row
        still admits it, else snaps to the nearest admitted value, and a row
        the block no longer offers here leaves. Returns (note, snapped):
        an operator line, and whether a value the operator set moved."""
        from nxs.suite import views
        from nxs.suite.schema import ManifestError, parse_suite_config
        from nxs.tune_write import draft

        try:
            doc, roles = draft(self.doc, self.previous, self.channels)
        except (ManifestError, SystemExit):
            # The knobs make no declaration yet (a link's camera before its
            # port's hub): the rows wait, and the save names the refusal.
            return None, False
        try:
            cfg = parse_suite_config(doc, self.path)
            graph, seen = cfg.graph, views.views(cfg)
        except ManifestError:
            # A draft the laws refuse is judged as it reads, every node with
            # its peers, so a step toward a lawful one moves nothing else.
            cfg = None
            graph, seen = views.draft(doc, self.path)
        notes, snapped = [], False
        for channel in self.channels:
            for role, label, unit in _roles_of(channel):
                note, moved = _rebuild_section(channel, role, label, unit, doc, roles, graph, seen)
                notes += [note] if note else []
                snapped |= moved
        if cfg is None:
            refusal = self.refusal()
            notes += [refusal] if refusal else []
        return ("; ".join(notes) if notes else None), snapped

    def refusal(self) -> Optional[str]:
        """Why the declaration the knobs make does not load, None where it
        does."""
        from nxs.suite.schema import ManifestError, parse_suite_config
        from nxs.tune_write import draft

        try:
            parse_suite_config(draft(self.doc, self.previous, self.channels)[0], self.path)
        except (ManifestError, SystemExit) as exc:
            return f"the declaration does not load: {getattr(exc, 'fact', exc)}"
        return None


def _roles_of(channel) -> List[Tuple[tuple, str, Optional[str]]]:
    """The blocks a channel's rows sections speak for: `(role, label, unit)`,
    the role as the writer names its node (`tune_write.draft`)."""
    if channel.kind == "port":
        return [(("hub", channel.name), "hub", None), (("host", channel.name), "host", None)]
    if channel.kind == "link":
        return [(("camera", channel.port_name, channel.letter), "camera", None)]
    out = []
    for i, f in enumerate(channel.declare().fields):
        if f.value != NONE:
            out.append((("click", channel.name, i), f"{channel.name}/{f.value}", channel.name))
    out.append((("unit", channel.name), channel.name, channel.name))
    return out


def _rebuild_section(channel, role, label, unit, doc, roles, graph, seen) -> Tuple[Optional[str], bool]:
    """One block's rows section, rebuilt on the draft: placed where its kind
    sits, removed when the role names no node."""
    from nxs import description
    from nxs.suite.options import BlockView, options_for

    old = next((s for s in channel.sections if s.kind == ROWS and getattr(s, "role", None) == role),
               None)
    name = roles.get(role)
    node_type = (doc.get("nodes") or {}).get(name, {}).get("type") if name else None
    block = None
    if node_type is not None:
        try:
            block = description.of_type(node_type)
        except (description.UnknownType, description.DescriptionError):
            block = None
    if block is None:
        if old is None:
            return None, False
        channel.sections.remove(old)
        # Rows that leave are no snap: nothing the operator set moved.
        return (f"{channel.name}: camera rows wait for a hub and a sensor"
                if role[0] == "camera" else None), False
    node = graph.nodes.get(name) if graph is not None else None
    view = seen.get(name) if name in seen else BlockView(block, node=name or "")
    if node is not None:
        block = node.description
    current = dict(node.values) if node is not None else {}
    # Declared is what the file states for the node; a knob the operator set stays spelled.
    filed = ((channel.model.doc.get("nodes") or {}).get(name) or {})
    declared = set(filed.get("parameters") or {}) if filed.get("type") == node_type else set()
    rows = [r for r in block.rows if role[0] != "host" or r.name.startswith(f"{role[1]}_")]
    options = options_for(view)
    old_fields = {f.name: f for f in (old.fields if old is not None else [])}
    # A knob that leaves keeps its value for when its row comes back.
    remembered = dict(getattr(old, "remembered", None) or {})
    fields, notes, moved = [], [], False
    if old is not None and getattr(old, "node_type", node_type) != node_type:
        # Another block's knobs carry nothing over: a value the operator set goes.
        lost = [f.name for f in old.fields if f.default is None and f.name != "firmware"]
        if lost:
            notes.append(f"{channel.name}: {node_type} opens its own rows: "
                         f"{', '.join(lost)} at {node_type}'s")
            moved = True
        old_fields, remembered = {}, {}
    for row in rows:
        option = options.get(row.name)
        if option is None or not option.applicable:
            if row.name in old_fields:
                left = old_fields[row.name]
                remembered[row.name] = (field_raw(left), left.default is None)
                notes.append(f"{channel.name}: {row.name} leaves: {option.finding if option else ''}"
                             .rstrip(": "))
            continue
        before = old_fields.get(row.name)
        back = remembered.pop(row.name, None) if before is None else None
        raw = (field_raw(before) if before is not None else back[0] if back is not None
               else int(current.get(row.name, row.default)))
        field = row_field(row, option, raw, declared=row.name in declared)
        if (before is not None and before.default is None) or (back is not None and back[1]):
            field.default = None        # the operator's word stays spelled
        # A stated value that moves is the note; a default the block narrows opens quietly.
        if field_raw(field) != raw and field.default is None:
            notes.append(f"{channel.name}: {row.name} {row_token(row, raw)} is not offered: "
                         f"set to {field.value}" + (f" ({option.finding})" if option.finding else ""))
            moved |= before is not None and before.default is None
        fields.append(field)
    if role[0] == "unit":
        fields.append(_firmware_field(filed if filed.get("type") == node_type else {}, old_fields))
    if not fields:
        if old is not None:
            channel.sections.remove(old)
        return ("; ".join(notes) or None), moved
    section = Section(label, fields, ROWS, unit_name=unit,
                      sensor_index=role[2] if role[0] == "click" else None)
    section.role, section.node, section.node_type = role, name, node_type
    section.remembered = remembered
    section.rebuild = channel.model.rebuild
    _place(channel, old, section, role)
    return ("; ".join(notes) or None), moved


def _place(channel, old, section, role) -> None:
    """Put a rows section where the old one sat, else after the DECLARE
    section and the sections of the roles before it."""
    if old is not None:
        old.fields, old.label = section.fields, section.label
        old.node, old.node_type, old.remembered = section.node, section.node_type, section.remembered
        return
    order = [r for r, _l, _u in _roles_of(channel)]
    at = 1 if channel.sections and channel.sections[0].kind == "declare" else 0
    for i, sec in enumerate(channel.sections):
        if sec.kind == ROWS and getattr(sec, "role", None) in order and \
                order.index(sec.role) < order.index(role):
            at = i + 1
    channel.sections.insert(at, section)


def _firmware_field(entry: dict, old_fields) -> Field:
    """A unit's firmware pin: the versions the installed assets hold, the
    one the file's `entry` declares first."""
    params = entry.get("parameters") or {}
    declared = str(params["firmware"]) if params.get("firmware") is not None else None
    before = old_fields.get("firmware")
    pick = token(before.value) if before is not None else (declared or NONE)
    pins = _firmware_pins()
    if declared and declared not in pins:
        pins = [declared] + pins
    options = [NONE] + pins
    field = Field("firmware", options, options.index(pick) if pick in options else 0)
    field.default = None if (declared or (before is not None and before.default is None)) else NONE
    return field


def _firmware_pins() -> List[str]:
    """The versions of the signed images the installed assets hold."""
    import glob

    from nxs.suite import FIRMWARE_DIR
    from nxs.suite.firmware import read_image_version

    pins = []
    for path in sorted(glob.glob(os.path.join(FIRMWARE_DIR, "*.bin"))):
        try:
            pins.append(".".join(map(str, read_image_version(path))))
        except (OSError, ValueError):
            continue
    return list(dict.fromkeys(pins))


# ---------------------------
# Ports and links
# ---------------------------

class PortFamily:
    """A port's channel and its links': the hub, each link's camera and pod,
    and the shape they make."""

    def __init__(self, name, spec, template, finding, bus=None):
        self.name = name
        self.spec = spec            # the declared PortSpec, None for a platform port
        self.template = template    # the platform's Topology for the port, None off-target
        self.finding = finding      # the sweep's finding for the port's bus, None unread
        self.bus = bus              # the host's bus for the port
        self.port: Channel = None
        self.links: List[Channel] = []
        #: The hub's control bus a new entry names: set where the host does not
        #: resolve the port's name to it (`generate` on a host without port rules).
        self.pinned_bus = None

    def link(self, letter):
        return next((l for l in self.links if l.letter == letter), None)

    def hub_value(self):
        return self.port.knobs().get("HUB", NONE)

    def letters(self) -> List[str]:
        """The link letters the port carries: the declared ones, the hub's, and
        the ones the sweep found something on; `A` alone for a port without
        a shape (a camera on the connector)."""
        declared = [l.name for l in self.spec.links] if self.spec is not None else []
        if self.hub_value() == CONNECTOR:
            return declared or ["A"]
        shaped = [l.name for l in self.template.links] if self.template is not None else []
        found = sorted((self.finding or {}).get("links", {}))
        return list(dict.fromkeys(declared + shaped + found)) or ["A"]

    def base_topology(self):
        """The shape the knobs dress: the platform's port, else the declared one."""
        if self.template is not None:
            return self.template
        if self.spec is None:
            return None
        from nxs.cam.topology import port_topology
        try:
            return port_topology(self.spec)
        except Exception:        # noqa: BLE001 (a port the host cannot shape is status's finding)
            return None

    def topology(self):
        """The port as the knobs shape it: the chosen hub (or none, on the
        connector), each link with the sensor its knob names. None with HUB
        at (none) or without a shape to build on."""
        from nxs.cam.contracts import LinkSpec

        hub = self.hub_value()
        if hub == NONE:
            return None
        base = self.base_topology() or self._shape_from_rules(hub)
        if base is None:
            return None
        # The platform's shape of each letter, else the declaration's own,
        # else the hub's rules for the letter (a link the platform's default
        # port leaves out).
        shapes = {l.name: l for l in base.links}
        if self.spec is not None and self.template is not None:
            from nxs.cam.topology import port_topology
            try:
                for link in port_topology(self.spec).links:
                    shapes.setdefault(link.name, link)
            except Exception:        # noqa: BLE001 (the platform's shape stands alone)
                pass
        if hub != CONNECTOR:
            ruled = self._shape_from_rules(hub)
            for link in (ruled.links if ruled is not None else ()):
                shapes.setdefault(link.name, link)
        links = []
        for ch in self.links:
            sensor = ch.knobs().get("CAM", NONE)
            shape = shapes.get(ch.letter)
            if hub == CONNECTOR:
                shape = LinkSpec(name=ch.letter, des_window=None, csi_vc=0,
                                 sensor_compatible=None, ser_compatible=None)
            if shape is None:
                continue
            links.append(dataclasses.replace(
                shape, sensor_compatible=None if sensor == NONE else sensor,
                sensor_declared=sensor != NONE))
        owner = {}
        if self.spec is not None and hub != CONNECTOR:
            # The hub's owner and address are the declaration's word.
            owner = {"hub_driver": self.spec.hub_driver, "des_addr": self.spec.hub_addr}
        if self.spec is not None and self.spec.csi_lanes_declared:
            owner["csi_lanes"] = self.spec.csi_lanes
        return dataclasses.replace(base, des_compatible=None if hub == CONNECTOR else hub,
                                   links=tuple(links), **owner)

    def _shape_from_rules(self, hub):
        """A port no platform shapes, dressed from the hub's own link rules
        (its windows and channels), or bare for the connector."""
        from nxs.cam.contracts import LinkSpec, Topology
        from nxs.cam.topology import _hub_rules

        if not self.bus:
            return None
        if hub == CONNECTOR:
            return Topology(carrier=f"platform/{self.name}", i2c_bus=self.bus, des_compatible=None, links=())
        ser, windows, vcs = _hub_rules(hub)
        if not windows:
            return None
        links = tuple(LinkSpec(name=letter, des_window=windows[letter], csi_vc=vcs.get(letter, 0),
                               sensor_compatible=None, ser_compatible=ser) for letter in sorted(windows))
        return Topology(carrier=f"platform/{self.name}", i2c_bus=self.bus, des_compatible=hub, links=links)


def _hub_option(compatible):
    """A hub's option: the compatible is its token, the product the hub names its label."""
    product = hub_product(compatible)
    return (compatible, product) if product else compatible


def _label(option):
    return option[1] if isinstance(option, tuple) else str(option)


def _port_family(model, name, spec, template, finding, bus=None) -> PortFamily:
    """The channels of one port: the port's with HUB, each link's with CAM
    and POD; the rows sections come with the model's rebuild."""
    family = PortFamily(name, spec, template, finding, bus=bus)
    declared = spec is not None
    channel = Channel(name, [], kind="port", declared=declared, presence=presence(finding),
                      template=template)
    channel.port, channel.family, channel.model = spec, family, model
    hubs = [_hub_option(h) for h in installed_hubs()]
    channel.note = ""
    if spec is not None and spec.hub_compatible:
        current = spec.hub_compatible
    elif spec is not None and spec.links:
        current = CONNECTOR
    else:
        current = NONE
    options = [next((h for h in hubs if token(h) == current), current)]
    options += [h for h in [NONE, CONNECTOR] + hubs if token(h) != current]
    declare = Section("DECLARE", [Field("HUB", options, 0, render=_label,
                                        pinned=current if current != NONE else None)], "declare")
    declare.rebuild = model.rebuild
    channel.sections.append(declare)
    family.port = channel
    sensors = installed_sensors()
    found_links = (finding or {}).get("links", {})
    declared_pods = {l.name: l.unit for l in (spec.links if spec else []) if l.unit is not None}
    for letter in family.letters():
        link_spec = next((l for l in spec.links if l.name == letter), None) if spec else None
        link = Channel(f"{name}/{letter}", [], kind="link", declared=link_spec is not None,
                       presence=channel.presence)
        link.letter, link.family, link.port_name, link.model = letter, family, name, model
        # A link the walk did not read says why, so no pod offered on it is not a silence.
        link.note = (found_links.get(letter) or {}).get("unwalked") or ""
        sensor = link_spec.camera if link_spec else None
        # (none) after a declared sensor: stepping onto it removes the camera at save.
        sensor_options = ([sensor, NONE] + [s for s in sensors if s != sensor]) if sensor else [NONE] + sensors
        pod_options: List[Any] = [NONE]
        pod = declared_pods.get(letter)
        found = (found_links.get(letter) or {}).get("pod")
        if pod is not None:
            pod_options = [(pod.name, f"{pod.name} @{pod.alias:#04x}"), NONE]
        elif found is not None:
            pod_name = _pod_name(name, letter)
            pod_options = [NONE, (pod_name, f"{pod_name} @{found['addr']:#04x}")]
        section = Section("DECLARE", [
            Field("CAM", sensor_options, 0, pinned=sensor),
            Field("POD", pod_options, 0, render=_label, pinned=pod.name if pod else None)], "declare")
        section.rebuild = model.rebuild
        link.sections.append(section)
        family.links.append(link)
    if template is None and spec is None:
        channel.note = "no hub installed: a camera on the connector declares with HUB = connector"
    elif spec is not None and spec.hub_compatible is None and not spec.links:
        channel.note = "no hub declared — set HUB and a link's CAM, then save"
    elif spec is not None and spec.hub_driver == "kernel":
        channel.note = "no camera knobs: a kernel driver owns the hub"
    elif spec is not None and spec.hub_driver == "module":
        channel.note = f"no camera knobs: {spec.owner} masters the hub's bus"
    elif spec is not None and not any(l.camera for l in spec.links):
        channel.note = "no link sensor declared — set a link's CAM, then save"
    elif spec is not None and spec.hub_compatible and hub_for(family.topology()) is None:
        channel.note = f"no camera knobs: hub {spec.hub_compatible} is not installed"
    return family


def _pod_name(port, letter):
    from nxs.generate_seed import _unit_name as pod_name
    return pod_name(port, letter)


# ---------------------------
# Units and pods
# ---------------------------

def _personality_knob(index):
    return "CLICK" if index == 0 else f"CLICK-{index + 1}"


def _unit_channel(model, name, unit, declared=True, presence_word="unknown", template=None,
                  link_ref=None, running=None) -> Channel:
    """A unit's or a pod's channel: its CLICK knobs, one per declared click
    and one to add; each click's rows and the unit's own come with the
    model's rebuild."""
    from nxs.suite.reconcile import known_click_personalities

    channel = Channel(name, [], kind="unit", declared=declared, presence=presence_word,
                      template=template)
    channel.unit_spec, channel.link_ref, channel.running = unit, link_ref, running
    channel.model = model
    known = known_click_personalities()
    if running in known:
        known = [running] + [p for p in known if p != running]
    sensors = list(unit.sensors) if unit is not None else []
    fields = []
    for i, spec in enumerate(sensors):
        fields.append(Field(_personality_knob(i),
                            [spec.personality, NONE] + [p for p in known if p != spec.personality], 0,
                            pinned=spec.personality))
    fields.append(Field(_personality_knob(len(sensors)), [NONE] + known, 0))
    declare = Section("DECLARE", fields, "declare")
    declare.rebuild = lambda: _rebuild_unit(channel)
    channel.sections.append(declare)
    return channel


def _rebuild_unit(channel):
    """After a CLICK step: a fresh (none) slot after the last named click,
    then the model's rebuild (a renamed click's rows at their defaults)."""
    declare = channel.declare()
    names = [f.value for f in declare.fields]
    if names and names[-1] != NONE:
        known = [o for o in declare.fields[-1].options if o != NONE]
        declare.fields.append(Field(_personality_knob(len(names)), [NONE] + known, 0))
    for section in [s for s in channel.sections if s.kind == ROWS and s.role[0] == "click"]:
        index = section.role[2]
        if index >= len(names) or not section.label.endswith(f"/{names[index]}"):
            channel.sections.remove(section)
    return channel.model.rebuild()


# ---------------------------
# Load
# ---------------------------

def load_model(sweep=None, path=None, buses=None, templates=None):
    """One channel per hardware node the platform can carry: every camera
    port, declared or not, with a channel per link; every declared unit;
    every pod that answered on a hub's link undeclared; every unit that
    answered on a camera bus undeclared. A missing manifest is an empty
    declaration. A caller that swept the buses already passes its `sweep`,
    and `generate` passes the walk's `buses` ({port: bus}) and `templates`
    ({port: Topology}) in place of the host's."""
    from nxs.suite import default_config_path
    from nxs.suite.schema import LinkSpec, load_suite_config
    from nxs.tune_write import read_document

    path = path or default_config_path()
    # An absent or empty file is an empty declaration, as `generate` takes it.
    cfg = load_suite_config(path) if os.path.exists(path) and os.path.getsize(path) > 0 else _EmptyConfig()
    doc = read_document(path)
    model = Model(path, doc, getattr(cfg, "graph", None), [])
    if buses is None:
        buses = _platform_port_buses()
    if sweep is None:
        sweep = sweep_buses(buses, cfg) if buses else {}
    if templates is None:
        templates = _platform_ports() if buses else {}
    channels = model.channels
    names = sorted(cfg.ports) + sorted(n for n in buses if n not in cfg.ports)
    for name in names:
        spec = cfg.ports.get(name)
        bus = (spec.bus if spec is not None and spec.bus else None) or buses.get(name)
        family = _port_family(model, name, spec, templates.get(name), sweep.get(bus), bus=bus)
        channels.append(family.port)
        channels.extend(family.links)
    # Units: the declared ones (a pod on a link and a unit on a bus alike),
    # the pods that answered where no declaration gives a unit, the units
    # that answered on a bus.
    declared_edges = {l.identity() for u in cfg.units for l in u.links}
    declared_pods = {(name, l.name) for name, p in cfg.ports.items() for l in p.links if l.unit}
    riding_addresses = {(p.bus, l.unit.alias) for p in cfg.ports.values() for l in p.links if l.unit}
    rides = {l.unit.name: f"{name}/{l.name}" for name, p in cfg.ports.items() for l in p.links if l.unit}
    for unit in cfg.units:
        word = "unknown"
        link_ref = rides.get(unit.name)
        if link_ref is not None:
            port_name, letter = link_ref.split("/", 1)
            spec = cfg.ports[port_name]
            finding = sweep.get(spec.bus or buses.get(port_name))
            if finding and finding["hub"]:
                link = finding["links"].get(letter)
                word = "present" if link and link["pod"] else ("absent" if link and not link["unwalked"] else "unknown")
        else:
            for link in unit.links:
                if link.transport == "i2c" and link.bus in sweep and sweep[link.bus]:
                    word = ("present" if any(a == link.address for a, *_ in sweep[link.bus]["units"])
                            else "absent")
        channels.append(_unit_channel(model, unit.name, unit, presence_word=word, link_ref=link_ref))
    for bus in sorted(sweep):
        found = sweep[bus]
        if not found:
            continue
        for route, serial, click in found.get("routes", []):
            if bus == ROUTES and route.identity() not in declared_edges:
                channels.append(_unit_channel(model, _unit_name(route), None, declared=False,
                                              presence_word="present", template=(route, serial),
                                              running=_running(click)))
        for letter, pod in swept_pods(found):
            if (found["port"], letter) in declared_pods:
                continue
            channels.append(_found_pod(model, found["port"], letter, bus, pod))
        for address, serial, click in found["units"]:
            route = LinkSpec(transport="i2c", bus=bus, address=address)
            if route.identity() in declared_edges or (bus, address) in riding_addresses:
                continue
            channels.append(_unit_channel(model, _unit_name(route), None, declared=False,
                                          presence_word="present", template=(route, serial),
                                          running=_running(click)))
    model.rebuild()
    for channel in channels:
        if channel.kind == "unit" and channel.declared and channel.unit_spec is not None \
                and channel.unit_spec.sensors and not any(
                    s.kind == ROWS and s.role[0] == "click" for s in channel.sections):
            channel.note = "no parameter knobs: the personality exposes none, or it did not load"
    return path, cfg, channels


def _found_pod(model, port, letter, bus, pod) -> Channel:
    """The channel of a pod that answered on `port`'s link `letter`
    undeclared: named as `generate` names it, routed through the link, the
    click personality it runs offered first."""
    from nxs.suite.schema import LinkSpec

    route = LinkSpec(transport="i2c", bus=bus, address=pod["addr"], link_ref=f"{port}/{letter}")
    return _unit_channel(model, _pod_name(port, letter), None, declared=False, presence_word="present",
                         template=(route, pod["serial"]), link_ref=f"{port}/{letter}",
                         running=_running(pod["click"]))


def _running(click) -> Optional[str]:
    """The click personality whose class compiles to the name a unit reports running."""
    from nxs.suite.scan import _module_for_driver

    return _module_for_driver(click) if click else None
