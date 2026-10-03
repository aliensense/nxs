# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The tuning model: a field carries the options a descriptor allows, a section groups them, and a channel is one machine the panel walks."""

import math


SYNC_OPTIONS = ["free_run", "fsync"]

#: The rates the panel marks `· shipped` (requirements nxs-tune §4); with
#: the mode's ceiling, the fps knob's detents wherever its menu offers them.
FPS_DETENTS = (30, 60)
#: A step toward a detent that lands this close to it pulls onto it.
DETENT_PULL_FPS = 2

class Field:
    """One panel knob: a name and its stepped options."""

    def __init__(self, name, options, index, render=str, unit=None, bounds=None,
                 pinned=None, detents=()):
        self.name = name
        self.options = options
        #: (lo, hi) for a range parameter: any integer inside is a lawful
        #: value, offered or not.
        self.bounds = bounds
        #: An option that stays offered whatever the scope: the value the
        #: manifest declares, shown even when no pack offers it.
        self.pinned = pinned
        #: The detents among the options: the panel marks them, and a step
        #: pulls onto one it lands close to.
        self.detents = tuple(detents)
        self.index = max(0, index)
        self.render = render
        self.unit = unit

    @property
    def value(self):
        return self.options[self.index]

    def step(self, delta):
        self.index = (self.index + delta) % len(self.options)
        # Only a detent ahead pulls: one behind the step would hold the
        # cursor on the detent it just left (30 -> 31 -> 30).
        ahead = [d for d in self.detents
                 if (d - self.value) * delta > 0 and abs(d - self.value) <= DETENT_PULL_FPS]
        if ahead:
            self.index = self.options.index(min(ahead, key=lambda d: abs(d - self.value)))

class Section:
    """One part on a strip: the camera, or one sensor of a unit."""

    def __init__(self, label, fields, kind, unit_name=None,
                 sensor_index=None, rebuild=None):
        self.label = label
        self.fields = fields
        self.kind = kind  # "camera" | "sensor"
        self.unit_name = unit_name
        self.sensor_index = sensor_index
        #: Re-derives the fields that depend on others after a step; returns
        #: (note, snapped): an operator line and whether a value changed.
        self.rebuild = rebuild

def refresh(section):
    """Run a section's rebuild after one of its knobs stepped."""
    if section is None or section.rebuild is None:
        return None, False
    return section.rebuild()

#: The option that declares nothing: a hub not chosen, a link without a
#: sensor, a unit without a personality.
NONE = "(none)"

class Channel:
    """One strip = one hardware node: a carrier PORT (a hub with its camera
    links and the units riding them) or a standalone UNIT. An undeclared node
    opens with a DECLARE section whose knobs write its manifest entry at save."""

    def __init__(self, name, sections, kind="port", declared=True,
                 presence="unknown", template=None, note=""):
        self.name = name
        self.sections = sections
        self.kind = kind  # "port" | "unit"
        self.declared = declared
        self.presence = presence  # "present" | "absent" | "unknown"
        #: What a save writes for an undeclared node: a port's platform
        #: port (a Topology), a unit's (LinkSpec, serial).
        self.template = template
        self.note = note
        #: A declared port's spec, for the camera knobs' declared values.
        self.port = None

    @property
    def label(self):
        tag = "" if self.declared else " · new"
        return f"{self.kind.upper()} {self.name}{tag}"

    def declare(self):
        """The DECLARE section, or None for a node with nothing to declare."""
        return next((s for s in self.sections if s.kind == "declare"), None)

    def camera(self):
        return next((s for s in self.sections if s.kind == "camera"), None)

    def knobs(self):
        """The DECLARE knobs by name."""
        declare = self.declare()
        return {f.name: f.value for f in declare.fields} if declare else {}

    def topology(self):
        """The port as the DECLARE knobs shape it: the chosen hub,
        the links that carry a sensor. None when nothing is declared."""
        if self.kind != "port" or self.template is None:
            return None
        return _shaped_topology(self.template, self.knobs())

    def rebuild(self):
        """Re-derive the knobs that depend on the DECLARE section: a
        port's camera knobs from its hub and sensors, a unit's parameter
        knobs from its personality. Returns (note, snapped) like a section's."""
        if self.kind == "port":
            self._scope_sensors()
            return self._rebuild_camera()
        return self._rebuild_sensor()

    def _scope_sensors(self):
        """The link knobs offer the sensors of the chosen hub's pack, extensions
        included; a hub change re-scopes them, dropping unknown values to (none)."""
        declare = self.declare()
        if declare is None:
            return
        hub = self.knobs().get("HUB", NONE)
        sensors = _pack_sensors(hub)
        # The declared sensor stays offered under the declared hub; under
        # another hub it drops with the rest.
        declared_hub = self.port.hub_compatible if self.port is not None else None
        for field in declare.fields:
            if not field.name.startswith("sensor-"):
                continue
            offered = list(sensors)
            if field.pinned and hub == declared_hub and field.pinned not in offered:
                offered.insert(0, field.pinned)
            current = field.value if field.value in offered else NONE
            field.options = ([current] + [s for s in offered if s != current]
                             if current != NONE else [NONE] + offered)
            field.index = 0

    def _rebuild_camera(self):
        old = self.camera()
        topology = self.topology()
        if topology is None or not topology.links:
            if old is not None:
                self.sections.remove(old)
                return "camera knobs wait for a hub and a link sensor", True
            return None, False
        fields = _port_fields_for(topology, self.port)
        if not fields:
            if old is not None:
                self.sections.remove(old)
            return ("no camera knobs: the descriptor pack offers no "
                    "geometry every declared link shares"), old is not None
        section = Section("CAMERA", fields, "camera",
                          rebuild=getattr(fields, "rebuild", None))
        if old is None:
            self.sections.insert(1, section)
            return "camera knobs follow the declared hub and sensors", False
        self.sections[self.sections.index(old)] = section
        return None, True

    def _rebuild_sensor(self):
        old = next((s for s in self.sections if s.kind == "sensor"), None)
        personality = self.knobs().get("PERSONALITY", NONE)
        if personality == NONE:
            if old is not None:
                self.sections.remove(old)
                return "parameter knobs wait for a personality", True
            return None, False
        sections = _personality_sections(self.name, personality)
        if not sections:
            if old is not None:
                self.sections.remove(old)
            return f"personality {personality} exposes no parameters", old is not None
        if old is None:
            self.sections.append(sections[0])
            return f"parameter knobs of {personality}", False
        self.sections[self.sections.index(old)] = sections[0]
        return None, True

    @property
    def fields(self):
        return [f for sec in self.sections for f in sec.fields]

    def section_of(self, flat_index):
        for sec in self.sections:
            if flat_index < len(sec.fields):
                return sec
            flat_index -= len(sec.fields)
        return self.sections[-1] if self.sections else None

def _camera_refusal(pack, topology, token, fps, sync):
    """Why a port-wide camera declaration would be refused by `on`, or None:
    the geometry must be a mode of every link's sensor and the rate must pass
    every link's timing law (or the trigger laws when synced)."""
    from nxs.cam import timing as cam_timing
    from nxs.cam.contracts import InfeasibleConfig
    from nxs.cam.descriptors import resolve_mode

    try:
        modes = {link.name: resolve_mode(pack.descriptor(link.sensor_compatible), token)
                 for link in topology.links}
    except InfeasibleConfig as exc:
        return str(exc)
    if fps is None:
        return None
    if sync == "fsync":
        try:
            pack.flows().build_fsync(pack, topology, float(fps), modes=modes)
        except InfeasibleConfig as exc:
            return str(exc)
        except Exception as exc:  # a pack without the trigger overlay
            return str(exc)
        return None
    for link in topology.links:
        refusal = cam_timing.free_run_refusal(pack, link, modes[link.name], float(fps),
                                              topology=topology)
        if refusal:
            return f"link {link.name}: {refusal}"
    return None

def _port_camera_refusal(port_name, token, fps, sync, topology=None):
    """`_camera_refusal` for a port of the manifest on disk (what a save
    is about to rewrite), or for the port given (a port being declared);
    None when the port or its pack is unknown."""
    from nxs.cam import packs as cam_packs
    from nxs.cam import topology as cam_topo

    if topology is None:
        ports = cam_topo._ports_from_suite()
        if ports:
            topology = next((t for t in ports[0].values()
                             if t.carrier.endswith(f"/{port_name}")), None)
    if topology is None:
        return None
    try:
        pack = cam_packs.pack_for(topology)
    except Exception:
        return None
    return _camera_refusal(pack, topology, token, fps, sync)

def _camera_context(port):
    """(pack, topology) for a hub port the pack knows, else None."""
    from nxs.cam import packs as cam_packs
    from nxs.cam import topology as cam_topo

    ports = cam_topo._ports_from_suite()
    topology = None
    if ports:
        topology = next((t for t in ports[0].values()
                         if t.carrier.endswith(f"/{port.name}")), None)
    if topology is None:
        return None
    try:
        return cam_packs.pack_for(topology), topology
    except Exception:
        return None

def _modes_for(pack, topology, token):
    """{link: mode name} for a port-wide geometry token."""
    from nxs.cam.descriptors import resolve_mode

    return {link.name: resolve_mode(pack.descriptor(link.sensor_compatible), token)
            for link in topology.links}

def _shared_presets(pack, topology):
    """The geometries every link's sensor offers on the port (its
    unit-program modes), in the first link's order: token -> (mode of the
    first link, lowest free-run ceiling, highest floor) over the links'
    lawful ranges."""
    from nxs.cam import timing as cam_timing

    offered = []
    for link in topology.links:
        sen = pack.descriptor(link.sensor_compatible)
        mod = pack.chip_module(link.sensor_compatible.split(",")[-1])
        table = {}
        for name in sen.program_modes():
            mode = sen.modes[name]
            geo = mode.get("geometry") or {}
            token = f"{geo.get('width')}x{geo.get('height')}"
            try:
                rates = cam_timing.lawful_range(pack, topology, link, name)
                ceiling, floor = ((rates.ceiling, rates.floor) if rates is not None
                                  else (mod.fps_ceiling(name), None))
            except Exception:
                ceiling, floor = None, None
            table.setdefault(token, (mode, ceiling, floor))
        offered.append(table)
    presets = {}
    for token, (mode, _, _) in offered[0].items():
        if not all(token in table for table in offered[1:]):
            continue
        limits = [table[token][1] for table in offered if table[token][1]]
        floors = [table[token][2] for table in offered if table[token][2]]
        presets[token] = (mode, min(limits) if limits else None,
                          max(floors) if floors else None)
    return presets

def _fps_menu(pack, topology, token, sync, floor, free_ceiling):
    """The rates the law of `sync` composes for `token`: every integer from the
    floor to the ceiling that the timing law admits (free run), or the whole
    rates the trigger laws leave in the mode's range (fsync). Empty when the
    sync has no lawful rate; the mode then gets no knob."""
    from nxs.cam import timing as cam_timing

    if sync == "fsync":
        try:
            return cam_timing.synced_rates(pack, topology, _modes_for(pack, topology, token))
        except Exception:
            return []
    if not free_ceiling:
        return []
    return [f for f in range(floor, int(free_ceiling) + 1)
            if _camera_refusal(pack, topology, token, float(f), sync) is None]

def _nearest(options, wanted):
    return min(options, key=lambda o: (abs(float(o) - float(wanted)), o))

class _CameraFields(list):
    """The camera section's knobs, carrying the rebuild that keeps the
    fps menu in step with PRESET and SYNC (the list is the section's;
    the rebuild edits it in place)."""

    rebuild = None

def _port_fields(port):
    """PRESET / fps / SYNC knobs for a declared hub port, from the pack."""
    ctx = _camera_context(port)
    if ctx is None:
        return []
    _pack, topology = ctx
    return _port_fields_for(topology, port)

def _port_fields_for(topology, port=None):
    """PRESET / fps / SYNC knobs for a port, from the pack that covers it; the
    declared mode, rate, and sync come from `port`. The fps menu follows the
    selected PRESET and SYNC. No knobs, never a traceback, when the pack refuses."""
    try:
        return _camera_fields(topology, port)
    except Exception:
        return []

def _camera_fields(topology, port):
    from nxs.cam import packs as cam_packs

    if not topology.camera_links:
        return []                          # no camera link, or a kernel-owned hub
    pack = cam_packs.pack_for(topology)
    presets = _shared_presets(pack, topology)
    if not presets:
        return []
    first = topology.camera_links[0]
    module = pack.chip_module(first.sensor_compatible.split(",")[-1])
    default_floor = int(getattr(module, "fps_floor", lambda: 1.0)())
    menus = {}

    def menu(token, sync):
        key = (token, sync)
        if key not in menus:
            floor = presets[token][2]
            floor = int(math.ceil(floor)) if floor else default_floor
            menus[key] = _fps_menu(pack, topology, token, sync, max(floor, 1),
                                   presets[token][1])
        return menus[key]

    def detents(token, sync):
        """The detent rates the menu offers, and its ceiling."""
        rates = menu(token, sync)
        return tuple(sorted({r for r in FPS_DETENTS if r in rates} | {max(rates)}))

    def label(token):
        mode = presets[token][0]
        text = f"{token} {mode['mipi']['data_type']}"
        free = menu(token, "free_run")
        text += f" · free-run ≤{max(free):g} fps" if free else " · free-run native"
        synced = menu(token, "fsync")
        if synced:
            text += f" · fsync ≤{max(synced):g} fps"
        return text

    tokens = list(presets)
    camera_mode = getattr(port, "camera_mode", None)
    sync_source = getattr(port, "sync_source", None)
    cur = camera_mode if camera_mode in tokens else tokens[0]
    # The sync sources are what the port's nodes offer: the frame-sync
    # generator is a hub's.
    sync_options = ["free_run"] if topology.is_direct else SYNC_OPTIONS
    declared_sync = sync_source if sync_source in sync_options else "free_run"
    declared = getattr(port, "camera_fps", None)
    fields = _CameraFields([
        Field("PRESET", [(t, label(t)) for t in tokens], tokens.index(cur),
              render=lambda v: v[1]),
        Field("SYNC", sync_options, sync_options.index(declared_sync))])
    memory = {"fps": None}

    def options_for(token, sync):
        options = list(menu(token, sync))
        # The declared rate is kept as it is (29.97 stays 29.97) and joins
        # the offered options where the manifest carries it.
        if (declared is not None and (token, sync) == (cur, declared_sync)
                and options):
            fps = int(declared) if float(declared).is_integer() else float(declared)
            if fps not in options:
                options = sorted(set(options) | {fps})
        return options

    def rebuild():
        token = fields[0].value[0]
        sync = fields[-1].value
        options = options_for(token, sync)
        old = next((f for f in fields if f.name == "fps"), None)
        if old is not None:
            memory["fps"] = old.value
        where = f"{token} under {sync}"
        if not options:
            if old is None:
                return None, False
            fields.remove(old)
            return (f"no fps knob for {where}: the mode runs at its native "
                    f"timing (was {old.value:g} fps)"), True
        wanted = memory["fps"] if memory["fps"] is not None else (
            declared if declared is not None else min(30, max(options)))
        value = wanted if wanted in options else _nearest(options, wanted)
        field = Field("fps", options, options.index(value), unit="fps",
                      detents=detents(token, sync))
        if old is None:
            fields.insert(1, field)
            return f"fps knob back for {where}: {value:g} fps", False
        fields[fields.index(old)] = field
        if value != old.value:
            return (f"{where} offers {min(options):g}–{max(options):g} fps, "
                    f"not {old.value:g}: fps set to {value:g}"), True
        return None, False

    initial = options_for(cur, declared_sync)
    if initial:
        wanted = declared if declared is not None else min(30, max(initial))
        value = wanted if wanted in initial else _nearest(initial, wanted)
        fields.insert(1, Field("fps", initial, initial.index(value), unit="fps",
                               detents=detents(cur, declared_sync)))
    fields.rebuild = rebuild
    return fields

def _personality_sections(unit_name, personality):
    """The parameter knobs of a personality at its defaults, for a unit being
    declared with it."""
    from types import SimpleNamespace

    unit = SimpleNamespace(name=unit_name,
                           sensors=[SimpleNamespace(driver=personality, config={})])
    return _unit_sections(unit)

def _unit_sections(unit, prefix=""):
    from nxs.descriptor import driver_descriptor

    sections = []
    for i, spec in enumerate(unit.sensors or []):
        try:
            params = driver_descriptor(spec.driver).get("params") or []
        except Exception:
            continue
        fields = []
        for entry in params:
            values = list(entry["values"])
            current = spec.config.get(entry["name"], entry["default"])
            bounds = None
            if entry.get("type") == "range" and len(values) == 2:
                # A range is a bounded integer: offer a ladder across it that carries
                # the current value; --set takes any integer inside the bounds.
                lo, hi = int(values[0]), int(values[1])
                bounds = (lo, hi)
                step = max(1, (hi - lo) // 50)
                values = sorted({lo, hi, *range(lo, hi + 1, step)}
                                | ({int(current)} if lo <= int(current) <= hi else set()))
            idx = values.index(current) if current in values else 0
            unit_label = f" {entry.get('unit', '')}".rstrip()
            fields.append(Field(
                entry["name"], values, idx,
                render=lambda v, u=unit_label: f"{v}{u}",
                unit=entry.get("unit"), bounds=bounds))
        if fields:
            sections.append(Section(
                f"{prefix}{unit.name}/{spec.driver}", fields, "sensor",
                unit_name=unit.name, sensor_index=i))
    return sections

class _EmptyConfig:
    """What a host without a manifest declares: nothing."""
    ports: dict = {}
    units: list = []

def _sweep(buses, aliases=None):
    """A read-only pass over the platform's camera buses: per bus, whether a
    hub answers, which units do (address, serial) at the standard addresses
    and at the aliases `aliases` declares for that bus, and which bare heads;
    `probed` lists the addresses asked. None where the bus could not be read."""
    from nxs import tree
    from nxs.suite.scan import I2C_ADDRESSES

    out = {}
    for _name, bus in sorted(buses.items()):
        extra = sorted((aliases or {}).get(bus, ()))
        try:
            found = tree._scan_bus(bus, extra)
        except Exception:
            out[bus] = None
            continue
        if not found.get("readable", True):
            out[bus] = None
            continue
        out[bus] = {"hub": bool(found["hubs"]), "units": list(found["units"]),
                    "sensors": list(found["sensors"]),
                    "probed": sorted(set(I2C_ADDRESSES) | set(extra))}
    return out


def _declared_aliases(cfg):
    """{bus: {alias}}: the pods the declaration puts on each camera bus."""
    out = {}
    for port in (getattr(cfg, "ports", None) or {}).values():
        for link in port.links:
            if link.unit is not None and port.bus:
                out.setdefault(port.bus, set()).add(link.unit.alias)
    return out

def _platform_port_buses():
    """{port name: bus} from the host layer; {} off-target."""
    from nxs import host as host_layer

    try:
        return dict(host_layer.current().camera_buses())
    except Exception:
        return {}

def _platform_ports():
    """{port name: Topology}: the pack's port shape per platform port."""
    from nxs.cam import topology as cam_topo

    try:
        ports = cam_topo._ports_from_platform()
    except Exception:
        return {}
    if not ports:
        return {}
    return {t.carrier.rsplit("/", 1)[-1]: t for t in ports[0].values()}

def _pack_compatibles(role):
    """The compatibles of every chip of `role` in the installed packs."""
    from nxs.cam import packs as cam_packs

    out = []
    try:
        found = cam_packs.discover()
    except Exception:
        return out
    for pack in found:
        for chip in pack.chips:
            try:
                d = pack.descriptor(chip)
            except Exception:
                continue
            if d.role == role and d.compatible not in out:
                out.append(d.compatible)
    return out

def _pack_sensors(hub):
    """The sensor compatibles of the pack whose flows cover `hub`, its
    extensions included; [] with no hub chosen or no pack covering it."""
    from nxs.cam import packs as cam_packs

    if hub == NONE:
        return []
    try:
        found = cam_packs.discover()
    except Exception:
        return []
    for pack in found:
        if hub not in pack.flows_for:
            continue
        out = []
        for chip in pack.chips:
            try:
                d = pack.descriptor(chip)
            except Exception:
                continue
            if d.role == "SEN" and d.compatible not in out:
                out.append(d.compatible)
        return out
    return []

def _shaped_topology(template, knobs):
    """`template` (a port) with the DECLARE knobs applied: the chosen hub,
    the links that carry a sensor; None when no hub or no link is chosen."""
    import dataclasses

    hub = knobs.get("HUB", NONE)
    if hub == NONE:
        return None
    links = []
    for link in template.links:
        sensor = knobs.get(f"sensor-{link.name}", NONE)
        if sensor == NONE:
            continue
        links.append(dataclasses.replace(link, sensor_compatible=sensor,
                                         sensor_declared=True))
    if not links:
        return None
    return dataclasses.replace(template, des_compatible=hub, links=tuple(links))

def _declare_section(channel, port, template):
    """The DECLARE section of a port: the hub, then one sensor knob per
    link letter. A declared port opens on its declaration; an undeclared
    one on `(none)`. Stepping any knob rebuilds the channel's camera knobs."""
    hubs = _pack_compatibles("DES")
    letters = ([l.name for l in port.links] if port is not None and port.links
               else [l.name for l in template.links] if template is not None else [])
    declared_hub = port.hub_compatible if port is not None else None
    hub_options = ([declared_hub] + [h for h in hubs if h != declared_hub]
                   if declared_hub else [NONE] + hubs)
    fields = [Field("HUB", hub_options, 0)]
    # The sensor knobs offer the chosen hub's pack, so a hub of one pack is
    # never declared with a sensor of another; (none) alone until a hub is chosen.
    sensors = _pack_sensors(hub_options[0])
    for letter in letters:
        declared_sensor = None
        if port is not None:
            link = next((l for l in port.links if l.name == letter), None)
            declared_sensor = link.camera if link is not None else None
        options = ([declared_sensor] + [s for s in sensors if s != declared_sensor]
                   if declared_sensor else [NONE] + sensors)
        fields.append(Field(f"sensor-{letter}", options, 0, pinned=declared_sensor))
    section = Section("DECLARE", fields, "declare")
    section.rebuild = channel.rebuild
    return section

def _unit_declare_section(channel):
    """The DECLARE section of an undeclared unit: its personality."""
    from nxs.suite.reconcile import known_driver_modules

    section = Section("DECLARE", [Field("PERSONALITY", [NONE] + known_driver_modules(), 0)],
                      "declare")
    section.rebuild = channel.rebuild
    return section

def _presence(sweep, bus):
    found = sweep.get(bus)
    if found is None:
        return "unknown"
    return "present" if found["hub"] or found["units"] or found.get("sensors") else "absent"

def _unit_name(link):
    from nxs.suite.scan import _suggest_name
    return _suggest_name(link)
