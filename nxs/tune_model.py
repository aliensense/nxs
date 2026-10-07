# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The panel's model: one channel per node of the rig (a port, each of its
links, a pod or a unit), its knobs from the declaration, the laws and the
installed personalities, and the one writer of the declaration."""

from __future__ import annotations

import dataclasses
import os
from typing import Any, Dict, List, Optional

from nxs import tune_camera as laws
from nxs.tune_fields import (NONE, Channel, Field, Section, _EmptyConfig, _platform_port_buses,
                             _platform_ports, _unit_name)
from nxs.tune_sweep import ROUTES, pods as swept_pods, presence, sweep as sweep_buses

CONNECTOR = laws.CONNECTOR
#: The decimation ladder a knob steps through; --set takes any integer inside the bounds.
DECIMATIONS = (1, 2, 4, 5, 8, 10, 16, 20, 25, 32, 50, 64, 100, 200, 250, 500, 1000)
DECIMATION_BOUNDS = (0, 0xFFFF)
#: The free-run rates a step pulls onto, where the menu offers them.
FPS_DETENTS = (30, 60)


# ---------------------------
# Fields
# ---------------------------

def _label(option):
    return option[1] if isinstance(option, tuple) else str(option)


def _token(option):
    return option[0] if isinstance(option, tuple) else option


def _choice(name, options, current, **kw):
    """A knob opened on `current` where offered, else on its first option."""
    tokens = [_token(o) for o in options]
    index = tokens.index(current) if current in tokens else 0
    return Field(name, options, index, **kw)


def _nearest(options, wanted):
    return min(options, key=lambda o: (abs(float(o) - float(wanted)), o))


def _rate_knob(name, rates, wanted, declared=None):
    """An fps knob over `rates`, on `wanted` where offered, else the nearest;
    a declared rate (29.97 stays 29.97) joins the menu."""
    options = list(rates)
    if declared is not None and options:
        value = int(declared) if float(declared).is_integer() else float(declared)
        if value not in options:
            options = sorted(set(options) | {value})
    pick = wanted if wanted in options else _nearest(options, wanted)
    detents = tuple(sorted({r for r in FPS_DETENTS if r in options} | {max(options)}))
    return Field(name, options, options.index(pick), unit="fps", detents=detents), pick


def _spelled(field) -> bool:
    """Whether a save spells the knob out: a declared value, one the
    operator set, or one stepped off the default the laws seeded."""
    return field.default is None or _token(field.value) != _token(field.default)


def _keep_spelling(old, fields):
    """A rebuilt knob the operator had set stays spelled out."""
    spelled = {f.name for f in (old.fields if old is not None else []) if f.default is None}
    for field in fields:
        if field.name in spelled:
            field.default = None


def _replace_section(channel, kind, section, after=None):
    """Put `section` (None removes) where the channel's section of `kind`
    sits, else after the section of kind `after`, else at the end. A section
    that stays keeps its identity (its fields move in), so a cursor or a
    batch holding it still holds it."""
    old = next((s for s in channel.sections if s.kind == kind), None)
    if old is not None:
        if section is None:
            channel.sections.remove(old)
        else:
            old.fields, old.label = section.fields, section.label
        return
    if section is None:
        return
    at = next((i + 1 for i, s in enumerate(channel.sections) if s.kind == after), len(channel.sections))
    channel.sections.insert(at, section)


# ---------------------------
# Ports and links
# ---------------------------

class PortFamily:
    """A port's channel and its links': the knobs that depend on one another
    (the hub, the sensors, the modes, the sync) rebuild together."""

    def __init__(self, name, spec, template, finding, bus=None):
        self.name = name
        self.spec = spec            # the declared PortSpec, None for a platform port
        self.template = template    # the platform's Topology for the port, None off-target
        self.finding = finding      # the sweep's finding for the port's bus, None unread
        self.bus = bus              # the host's bus for the port
        self.port: Channel = None
        self.links: List[Channel] = []
        #: The bus a new entry names: set where the host does not resolve
        #: the port's name to it (`generate` on a host without port rules).
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
            sensor = ch.knobs().get("SENSOR", NONE)
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

    def modes(self, written=False) -> Dict[str, str]:
        """Each link's mode by name; with `written`, the ones the declaration
        spells out (declared, or stepped off the laws' default)."""
        out = {}
        for ch in self.links:
            camera = ch.camera()
            mode = next((f for f in camera.fields if f.name == "mode"), None) if camera else None
            if mode is not None and (not written or _spelled(mode)):
                out[ch.letter] = _token(mode.value)
        return out

    def rates(self, written=False) -> Dict[str, float]:
        out = {}
        for ch in self.links:
            camera = ch.camera()
            fps = next((f for f in camera.fields if f.name == "fps"), None) if camera else None
            if fps is not None and (not written or _spelled(fps)):
                out[ch.letter] = float(fps.value)
        return out

    def sync(self, written=False) -> Optional[dict]:
        section = next((s for s in self.port.sections if s.kind == "sync"), None)
        if section is None:
            return None
        fields = {f.name: f for f in section.fields}
        if fields["source"].value != "fsync":
            return None
        out = {"source": "fsync"}
        fps = fields.get("fps")
        if fps is not None and (not written or _spelled(fps)):
            out["fps"] = float(fps.value)
        return out

    def gain(self):
        section = next((s for s in self.port.sections if s.kind == "gain"), None)
        if section is None:
            return None
        value = section.fields[0].value
        return None if value == NONE else float(value)

    def rebuild(self):
        """Re-derive the dependent knobs: each link's CAMERA (the sensor's
        modes, the rates under the port's sync), the port's SYNC and GAIN.
        Returns (note, snapped) like a section's rebuild."""
        notes, snapped = [], False
        topology = self.topology()
        hub = laws.hub_for(topology)
        synced = self._source() == "fsync" and hub is not None and not (topology is None or topology.is_direct)
        for ch in self.links:
            note, moved = self._rebuild_camera(ch, topology, hub, synced)
            notes += [note] if note else []
            snapped |= moved
        note, moved = self._rebuild_sync(topology, hub)
        notes += [note] if note else []
        snapped |= moved
        self._rebuild_gain(topology, hub)
        return ("; ".join(notes) if notes else None), snapped

    def _rebuild_camera(self, ch, topology, hub, synced):
        old = ch.camera()
        sensor = ch.knobs().get("SENSOR", NONE)
        link = next((l for l in topology.links if l.name == ch.letter), None) if topology else None
        offered = laws.mode_options(hub, sensor) if (hub is not None and sensor != NONE) else []
        if not offered:
            # Knobs that leave are no snap: nothing the operator set moved.
            if old is None:
                return None, False
            ch.sections.remove(old)
            return (f"{ch.name}: camera knobs wait for a hub and a sensor" if sensor == NONE or hub is None
                    else f"{ch.name}: {sensor} offers no mode here"), False
        # The knobs of another sensor carry nothing over: a sensor change opens
        # the new sensor's modes and rates at their defaults.
        current = {f.name: f.value for f in old.fields} if old and getattr(old, "sensor", None) == sensor else {}
        # The mode the laws run when nothing names one: what `on` would pick.
        tokens = [t for t, _l in offered]
        ruled = laws.mode_name(hub, sensor, laws.default_mode(hub, topology, link)) if link is not None else None
        default = next((o for o in offered if o[0] == ruled), offered[0])
        wanted_mode = _token(current["mode"]) if "mode" in current else ch.declared_mode
        if wanted_mode is not None and wanted_mode not in tokens:
            wanted_mode = laws.mode_name(hub, sensor, wanted_mode) or wanted_mode
        mode = _choice("mode", offered, wanted_mode if wanted_mode is not None else default[0], render=_label)
        # The declared rate joins a menu only at the declared mode; a mode
        # the declaration leaves to the laws stays implicit until stepped.
        declared_mode = laws.mode_name(hub, sensor, ch.declared_mode) or ch.declared_mode
        mode.default = None if ch.declared_mode is not None else default
        ch.at_declared_mode = declared_mode is None or _token(mode.value) == declared_mode
        declared_fps = ch.declared_fps if ch.at_declared_mode else None
        fields = [mode]
        note, moved = None, False
        if _token(mode.value) != wanted_mode and wanted_mode is not None:
            note, moved = f"{ch.name}: {sensor} does not offer {wanted_mode}: mode set to {_token(mode.value)}", True
        if not synced:
            rates = laws.free_run_rates(hub, topology, link, _token(mode.value)) if link is not None else []
            if rates:
                # The rate carries over: the knob's own, else the port's synced
                # rate when the sync just left, else the declared one.
                wanted = current.get("fps", self._synced_fps())
                if wanted is None:
                    wanted = declared_fps if declared_fps is not None else min(30, max(rates))
                field, pick = _rate_knob("fps", rates, wanted, declared=declared_fps)
                field.default = None if declared_fps is not None else _nearest(field.options, min(30, max(rates)))
                fields.append(field)
                if "fps" in current and pick != current["fps"]:
                    note = (f"{ch.name}: {_token(mode.value)} offers {min(rates):g}–{max(rates):g} fps, "
                            f"not {current['fps']:g}: fps set to {pick:g}")
                    moved = True
                elif "fps" not in current and old is not None:
                    note = f"{ch.name}: fps knob back: {pick:g} fps"
            elif "fps" in current:
                note, moved = f"{ch.name}: no fps knob for {_token(mode.value)}: the mode runs at its native timing", True
        elif "fps" in current:
            note = f"{ch.name}: fps follows the frame sync"
        if old is None or getattr(old, "sensor", None) == sensor:
            _keep_spelling(old, fields)
        section = Section("CAMERA", fields, "camera")
        section.rebuild = self.rebuild
        _replace_section(ch, "camera", section, after="declare")
        ch.camera().sensor = sensor
        return note, moved

    def _source(self):
        """The sync source the knob holds, else the declared one."""
        section = next((s for s in self.port.sections if s.kind == "sync"), None)
        if section is not None:
            return section.fields[0].value
        return self.spec.sync_source if self.spec is not None else "free_run"

    def _synced_fps(self):
        section = next((s for s in self.port.sections if s.kind == "sync"), None)
        fps = next((f for f in section.fields if f.name == "fps"), None) if section else None
        return float(fps.value) if fps is not None else None

    def _rebuild_sync(self, topology, hub):
        old = next((s for s in self.port.sections if s.kind == "sync"), None)
        cameras = topology.camera_links if topology is not None else ()
        if hub is None or not cameras:
            _replace_section(self.port, "sync", None)
            return None, False
        sources = ["free_run"] if topology.is_direct else ["free_run", "fsync"]
        current = {f.name: f.value for f in old.fields} if old else {}
        wanted = current.get("source", self.spec.sync_source if self.spec is not None else "free_run")
        source = _choice("source", sources, wanted)
        fields = [source]
        note, moved = (None, False) if source.value == wanted else (f"{self.name}: no frame sync on the connector", True)
        if source.value == "fsync":
            rates = laws.synced_rates(hub, topology, self.modes())
            if rates:
                # The declared synced rate (`sync.fps`, else the port shorthand's)
                # joins the menu at the declared modes only.
                declared = None
                if self.spec is not None and all(getattr(l, "at_declared_mode", True) for l in self.links):
                    declared = self.spec.sync_fps if self.spec.sync_fps is not None else self.spec.camera_fps
                wanted_fps = current.get("fps", declared if declared is not None else min(30, max(rates)))
                field, pick = _rate_knob("fps", rates, wanted_fps, declared=declared)
                field.default = None if declared is not None else _nearest(field.options, min(30, max(rates)))
                fields.append(field)
                if "fps" in current and pick != current["fps"]:
                    note, moved = f"{self.name}: frame sync offers {min(rates):g}–{max(rates):g} fps, not {current['fps']:g}: fps set to {pick:g}", True
            elif "fps" in current:
                note, moved = f"{self.name}: the trigger laws leave no rate for these modes", True
        _keep_spelling(old, fields)
        section = Section("SYNC", fields, "sync")
        section.rebuild = self.rebuild
        _replace_section(self.port, "sync", section, after="declare")
        return note, moved

    def _rebuild_gain(self, topology, hub):
        old = next((s for s in self.port.sections if s.kind == "gain"), None)
        span = laws.gain_range(hub, topology) if (hub is not None and topology is not None) else None
        if span is None:
            _replace_section(self.port, "gain", None)
            return
        declared = self.spec.camera_gain_db if self.spec is not None else None
        options: List[Any] = [NONE] + list(range(span[0], span[1] + 1))
        if declared is not None and declared not in options:
            options = [NONE] + sorted(set(options[1:]) | {declared})
        current = old.fields[0].value if old else (declared if declared is not None else NONE)
        section = Section("GAIN", [_choice("gain_db", options, current, unit="dB", bounds=span)], "gain")
        section.rebuild = self.rebuild
        _replace_section(self.port, "gain", section, after="sync")


def _port_family(name, spec, template, finding, cfg, bus=None) -> PortFamily:
    """The channels of one port: the port's with HUB, each link's with SENSOR
    and POD, then the dependent knobs the family rebuilds."""
    family = PortFamily(name, spec, template, finding, bus=bus)
    declared = spec is not None
    channel = Channel(name, [], kind="port", declared=declared, presence=presence(finding),
                      template=template)
    channel.port = spec
    channel.family = family
    hubs = laws.installed_hubs()
    channel.note = ""
    if spec is not None and spec.hub_compatible:
        current = spec.hub_compatible
    elif spec is not None and spec.links:
        current = CONNECTOR
    else:
        current = NONE
    options = [current] + [h for h in [NONE, CONNECTOR] + hubs if h != current]
    declare = Section("DECLARE", [Field("HUB", options, 0, pinned=current if current != NONE else None)],
                      "declare")
    declare.rebuild = family.rebuild
    channel.sections.append(declare)
    family.port = channel
    sensors = laws.installed_sensors()
    found_links = (finding or {}).get("links", {})
    declared_pods = {l.name: l.unit for l in (spec.links if spec else []) if l.unit is not None}
    for letter in family.letters():
        link_spec = next((l for l in spec.links if l.name == letter), None) if spec else None
        link = Channel(f"{name}/{letter}", [], kind="link", declared=link_spec is not None,
                       presence=channel.presence)
        link.letter, link.family, link.port_name = letter, family, name
        # A link's declared mode and rate: its own, else the port shorthand's.
        link.declared_mode = (link_spec.camera_mode or spec.camera_mode) if link_spec else None
        link.declared_fps = (link_spec.camera_fps if link_spec.camera_fps is not None
                             else spec.camera_fps) if link_spec else None
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
            Field("SENSOR", sensor_options, 0, pinned=sensor),
            Field("POD", pod_options, 0, render=_label, pinned=pod.name if pod else None)], "declare")
        section.rebuild = family.rebuild
        link.sections.append(section)
        family.links.append(link)
    family.rebuild()
    if template is None and spec is None:
        channel.note = "no hub installed: a camera on the connector declares with HUB = connector"
    elif spec is not None and spec.hub_compatible is None and not spec.links:
        channel.note = "no hub declared — set HUB and a link's SENSOR, then save"
    elif spec is not None and spec.hub_driver == "kernel":
        channel.note = "no camera knobs: a kernel driver owns the hub"
    elif spec is not None and not any(l.camera for l in spec.links):
        channel.note = "no link sensor declared — set a link's SENSOR, then save"
    elif spec is not None and spec.hub_compatible and laws.hub_for(family.topology()) is None:
        channel.note = f"no camera knobs: hub {spec.hub_compatible} is not installed"
    return family


def _pod_name(port, letter):
    from nxs.generate_seed import _unit_name as pod_name
    return pod_name(port, letter)


# ---------------------------
# Units and pods
# ---------------------------

def _personality_knob(index):
    return "PERSONALITY" if index == 0 else f"PERSONALITY-{index + 1}"


def _param_table(personality, config) -> List[dict]:
    """The compiled parameter table of a click personality at `config`:
    name, type, values, default, current, unit."""
    from nxs.suite.reconcile import load_click_personality

    compiled = load_click_personality(personality)().compile(dict(config or {}))
    return [{"name": p.name, "type": p.param_type, "values": list(p.values), "default": p.default,
             "current": p.current, "unit": p.unit} for p in compiled.params]


def _params_section(unit_name, index, personality, config) -> Optional[Section]:
    """One click personality's parameters as knobs, from its compiled table;
    None for a personality that does not load or declares none."""
    try:
        table = _param_table(personality, config)
    except Exception:        # noqa: BLE001 (a personality that does not load is status's finding)
        return None
    fields = []
    for entry in table:
        values = list(entry["values"])
        current = (config or {}).get(entry["name"], entry["current"])
        bounds = None
        if entry["type"] == "range" and len(values) >= 2:
            lo, hi = int(values[0]), int(values[1])
            bounds = (lo, hi)
            step = max(1, (hi - lo) // 50)
            values = sorted({lo, hi, *range(lo, hi + 1, step)}
                            | ({int(current)} if lo <= int(current) <= hi else set()))
        unit_label = f" {entry['unit']}".rstrip() if entry["unit"] else ""
        field = _choice(entry["name"], values, current,
                        render=lambda v, u=unit_label: f"{v}{u}", unit=entry["unit"] or None,
                        bounds=bounds)
        field.default = entry["default"]
        fields.append(field)
    if not fields:
        return None
    return Section(f"{unit_name}/{personality}", fields, "sensor", unit_name=unit_name,
                   sensor_index=index)


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


def _unit_channel(name, unit, declared=True, presence_word="unknown", template=None,
                  link_ref=None, running=None) -> Channel:
    """A unit's or a pod's channel: its PERSONALITY knobs (one per declared
    entry and one to add), each personality's parameters, then MOUNT,
    EGRESS and FIRMWARE."""
    from nxs.capabilities import ROTATION_NAMES
    from nxs.suite.reconcile import known_click_personalities

    channel = Channel(name, [], kind="unit", declared=declared, presence=presence_word,
                      template=template)
    channel.unit_spec, channel.link_ref, channel.running = unit, link_ref, running
    known = known_click_personalities()
    if running in known:
        known = [running] + [p for p in known if p != running]
    sensors = list(unit.sensors or []) if unit is not None else []
    fields = []
    for i, spec in enumerate(sensors):
        fields.append(Field(_personality_knob(i),
                            [spec.personality, NONE] + [p for p in known if p != spec.personality], 0,
                            pinned=spec.personality))
    fields.append(Field(_personality_knob(len(sensors)), [NONE] + known, 0))
    declare = Section("DECLARE", fields, "declare")
    declare.rebuild = lambda: _rebuild_unit(channel)
    channel.sections.append(declare)
    for i, spec in enumerate(sensors):
        section = _params_section(name, i, spec.personality, spec.config)
        if section is not None:
            channel.sections.append(section)
    orientation = unit.orientation if unit is not None else None
    channel.sections.append(Section("MOUNT", [_choice("orientation", [NONE] + list(ROTATION_NAMES),
                                                      orientation if orientation else NONE)], "mount"))
    decimation = unit.egress.decimation if unit is not None and unit.egress is not None else None
    ladder: List[Any] = [NONE] + sorted(set(DECIMATIONS) | ({decimation} if decimation is not None else set()))
    channel.sections.append(Section("EGRESS", [_choice("decimation", ladder,
                                                       decimation if decimation is not None else NONE,
                                                       bounds=DECIMATION_BOUNDS)], "egress"))
    firmware = unit.firmware if unit is not None else None
    pins = _firmware_pins()
    if firmware and firmware not in pins:
        pins = [firmware] + pins
    channel.sections.append(Section("FIRMWARE", [_choice("firmware", [NONE] + pins,
                                                         firmware if firmware else NONE)], "firmware"))
    if unit is not None and sensors and not any(s.kind == "sensor" for s in channel.sections):
        channel.note = "no parameter knobs: the personality exposes none, or it did not load"
    return channel


def _rebuild_unit(channel):
    """After a PERSONALITY step: a parameters section per named personality
    (at its defaults when the name changed), and a fresh (none) slot after
    the last named one."""
    declare = channel.declare()
    names = [f.value for f in declare.fields]
    if names and names[-1] != NONE:
        known = [o for o in declare.fields[-1].options if o != NONE]
        declare.fields.append(Field(_personality_knob(len(names)), [NONE] + known, 0))
    old = {s.sensor_index: s for s in channel.sections if s.kind == "sensor"}
    for section in list(old.values()):
        channel.sections.remove(section)
    note = None
    at = channel.sections.index(declare) + 1
    for i, personality in enumerate(names):
        if personality == NONE:
            continue
        previous = old.get(i)
        config = ({f.name: f.value for f in previous.fields}
                  if previous is not None and previous.label.endswith(f"/{personality}") else {})
        section = _params_section(channel.name, i, personality, config)
        if section is None:
            note = f"{personality} exposes no parameters"
            continue
        channel.sections.insert(at, section)
        at += 1
        if previous is None or not previous.label.endswith(f"/{personality}"):
            note = f"parameter knobs of {personality}"
    return note, False


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

    path = path or default_config_path()
    # An absent or empty file is an empty declaration, as `generate` takes it.
    cfg = load_suite_config(path) if os.path.exists(path) and os.path.getsize(path) > 0 else _EmptyConfig()
    if buses is None:
        buses = _platform_port_buses()
    if sweep is None:
        sweep = sweep_buses(buses, cfg) if buses else {}
    if templates is None:
        templates = _platform_ports() if buses else {}
    channels: List[Channel] = []
    names = sorted(cfg.ports) + sorted(n for n in buses if n not in cfg.ports)
    for name in names:
        spec = cfg.ports.get(name)
        bus = (spec.bus if spec is not None and spec.bus else None) or buses.get(name)
        family = _port_family(name, spec, templates.get(name), sweep.get(bus), cfg, bus=bus)
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
                word = "present" if link and link["pod"] else ("absent" if link and link["walked"] else "unknown")
        else:
            for link in unit.links:
                if link.transport == "i2c" and link.bus in sweep and sweep[link.bus]:
                    word = ("present" if any(a == link.address for a, *_ in sweep[link.bus]["units"])
                            else "absent")
        channels.append(_unit_channel(unit.name, unit, presence_word=word, link_ref=link_ref))
    for bus in sorted(sweep):
        found = sweep[bus]
        if not found:
            continue
        for route, serial, click in found.get("routes", []):
            if bus == ROUTES and route.identity() not in declared_edges:
                channels.append(_unit_channel(_unit_name(route), None, declared=False, presence_word="present",
                                              template=(route, serial), running=_running(click)))
        for letter, pod in swept_pods(found):
            if (found["port"], letter) in declared_pods:
                continue
            channels.append(_found_pod(found["port"], letter, bus, pod))
        for address, serial, click in found["units"]:
            route = LinkSpec(transport="i2c", bus=bus, address=address)
            if route.identity() in declared_edges or (bus, address) in riding_addresses:
                continue
            channels.append(_unit_channel(_unit_name(route), None, declared=False, presence_word="present",
                                          template=(route, serial), running=_running(click)))
    return path, cfg, channels


def _found_pod(port, letter, bus, pod) -> Channel:
    """The channel of a pod that answered on `port`'s link `letter`
    undeclared: named as `generate` names it, routed through the link, the
    click personality it runs offered first."""
    from nxs.suite.schema import LinkSpec

    route = LinkSpec(transport="i2c", bus=bus, address=pod["addr"], link_ref=f"{port}/{letter}")
    return _unit_channel(_pod_name(port, letter), None, declared=False, presence_word="present",
                         template=(route, pod["serial"]), link_ref=f"{port}/{letter}",
                         running=_running(pod["click"]))


def _running(click) -> Optional[str]:
    """The click personality whose class compiles to the name a unit reports running."""
    from nxs.suite.scan import _module_for_driver

    return _module_for_driver(click) if click else None
