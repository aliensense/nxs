# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""`nxs tune`: the personality panel for the machine's declared config. Every value
comes from a descriptor, so an impossible setting cannot be typed; curses only
renders the model, and `--list`, `--set`, `--play` drive it for scripts."""
import json
import os
import subprocess
import sys
import time

from nxs.finding import as_data
from nxs.tune_fields import (
    NONE, SYNC_OPTIONS, Channel, Field, Section, _EmptyConfig, _declare_section,
    _declared_aliases,
    _pack_sensors, _platform_port_buses, _platform_ports, _port_camera_refusal,
    _port_fields, _presence, _sweep, _unit_declare_section, _unit_name,
    _unit_sections, refresh)

#: The names `nxs.tune` has always answered to, wherever they now live.
__all__ = [
    "NONE", "SYNC_OPTIONS", "Channel", "Field", "Section", "apply_sets",
    "check_saved", "cmd_tune", "describe", "freeze", "identify", "identify_route",
    "identify_unit", "load_model",
    "nothing_tunable", "parse_set", "play", "refresh", "resolve_field",
    "run_batch", "save_model", "_port_camera_refusal",
    "_port_fields", "_unit_sections",
]


def load_model(sweep=None):
    """One strip per hardware node the platform can carry: every camera port,
    declared or not, every declared unit, and every unit that answered on a
    camera bus undeclared. A missing manifest is an empty declaration; a
    caller that swept the buses already passes its `sweep`."""
    import os

    from nxs.suite import default_config_path
    from nxs.suite.schema import LinkSpec, load_suite_config

    path = default_config_path()
    cfg = load_suite_config(path) if os.path.exists(path) else _EmptyConfig()
    by_name = {u.name: u for u in cfg.units}
    buses = _platform_port_buses()
    if sweep is None:
        sweep = _sweep(buses, _declared_aliases(cfg)) if buses else {}
    ports = _platform_ports() if buses else {}
    riding = set()
    channels = []

    # Ports: declared ones first, then the platform's undeclared ones.
    for name in sorted(cfg.ports):
        port = cfg.ports[name]
        template = ports.get(name)
        channel = Channel(name, [], kind="port", declared=True,
                          presence=_presence(sweep, port.bus), template=template)
        channel.port = port
        channel.sections.append(_declare_section(channel, port, template))
        if port.hub_compatible is None and not port.links:
            # A port declared by its bus alone: the DECLARE knobs open at
            # (none), and a save writes the hub and links into its entry.
            channel.note = ("no hub declared — set HUB and a link sensor "
                            "under DECLARE, then save")
            channels.append(channel)
            continue
        if not port.links and port.hub_driver != "kernel":
            # A hub declared with no link yet (a save in progress, one
            # knob at a time): the sensor knobs are the next step.
            channel.note = ("no link sensor declared — set a sensor knob "
                            "under DECLARE, then save")
            channels.append(channel)
            continue
        fields = _port_fields(port)
        if fields:
            channel.sections.append(Section("CAMERA", fields, "camera",
                                            rebuild=getattr(fields, "rebuild", None)))
        else:
            channel.note = ("no camera knobs: the pack covers no geometry "
                            "every declared link shares, or the pack is missing")
        for link in port.links:
            if link.unit and link.unit.name in by_name:
                riding.add(link.unit.name)
                channel.sections.extend(_unit_sections(
                    by_name[link.unit.name], prefix=f"LINK {link.name} · "))
        channels.append(channel)
    for name in sorted(buses):
        if name in cfg.ports:
            continue
        template = ports.get(name)
        channel = Channel(name, [], kind="port", declared=False,
                          presence=_presence(sweep, buses[name]), template=template)
        section = _declare_section(channel, None, template)
        # A knob offering nothing but (none) is no choice: with no pack
        # installed the note says it all, and the strip carries no DECLARE.
        if any(len(f.options) > 1 for f in section.fields):
            channel.sections.append(section)
        if template is None:
            channel.note = "no descriptor pack installed: nothing to declare it with"
        channels.append(channel)

    # Units: declared ones, then the ones that answered undeclared.
    declared_edges = {l.identity() for u in cfg.units for l in u.links}
    riding_addresses = {(p.bus, l.unit.alias) for p in cfg.ports.values()
                        for l in p.links if l.unit}
    for unit in cfg.units:
        if unit.name in riding:
            continue
        sections = _unit_sections(unit)
        presence = "unknown"
        for link in unit.links:
            if link.transport == "i2c" and link.bus in sweep and sweep[link.bus]:
                presence = ("present" if any(a == link.address for a, _ in sweep[link.bus]["units"])
                            else "absent")
        channel = Channel(unit.name, sections, kind="unit", presence=presence)
        if not sections:
            channel.note = "no parameter knobs: the personality exposes none, or it did not load"
        channels.append(channel)
    for bus in sorted(sweep):
        found = sweep[bus]
        if not found:
            continue
        for address, serial in found["units"]:
            link = LinkSpec(transport="i2c", bus=bus, address=address)
            if link.identity() in declared_edges or (bus, address) in riding_addresses:
                continue
            channel = Channel(_unit_name(link), [], kind="unit", declared=False,
                              presence="present", template=(link, serial))
            channel.sections.append(_unit_declare_section(channel))
            channels.append(channel)
    return path, cfg, channels




def nothing_tunable(cfg) -> str:
    """Why the model is empty, with the step that fills it: a unit
    without a `sensors:` entry offers no knobs, and a manifest with no
    ports or units offers nothing at all."""
    bare = [u.name for u in cfg.units if not u.sensors]
    if bare:
        return (f"nothing tunable declared — {', '.join(bare)} "
                f"{'carries' if len(bare) == 1 else 'carry'} no sensors: "
                f"entry; nxs generate writes one from the loaded "
                f"personality, or add sensors: [{{personality: <name>}}] by hand")
    if cfg.units:
        # Every unit declares sensors, yet none became a strip: their
        # descriptors did not load or expose no parameters.
        names = ", ".join(u.name for u in cfg.units)
        return (f"nothing tunable declared — {names} declare sensors whose "
                f"descriptors did not load or expose no parameters; nxs status "
                f"names the descriptor errors")
    if cfg.ports:
        return (f"nothing tunable declared — {', '.join(sorted(cfg.ports))} "
                f"offers no camera strip: the descriptor pack is missing "
                f"or names no sensor for its links")
    return ("nothing tunable — this host names no camera port (no platform "
            "rules for it) and suite.yaml declares nothing; nxs generate "
            "writes the units that answer on any route")


def save_model(path, channels, only=None):
    """Patch the manifest in place (timestamped .bak beside it); ``only`` narrows
    the write to a set of sections."""
    from nxs.suite.freeze import _round_trip_yaml

    # A round-trip load and dump keeps the operator's comments, quoting, and
    # layout; a host with no manifest yet starts from an empty one.
    yaml_rt = _round_trip_yaml()
    existed = os.path.exists(path)
    if existed:
        with open(path, encoding="utf-8") as fh:
            raw = yaml_rt.load(fh) or {}
    else:
        raw = {}
    for ch in channels:
        declare = ch.declare()
        if declare is not None and (only is None or declare in only):
            _write_declaration(raw, ch)
        for sec in ch.sections:
            if only is not None and sec not in only:
                continue
            if sec.kind == "declare":
                continue
            if sec.kind == "camera":
                if ch.name not in (raw.get("ports") or {}):
                    continue               # a port declared with no hub yet
                port = raw["ports"][ch.name]
                knobs = {f.name: f.value for f in sec.fields}
                # A preset changed since the fps menu was built must still fit the
                # rate (and every link) before the declaration is persisted.
                refusal = _port_camera_refusal(ch.name, knobs["PRESET"][0],
                                               knobs.get("fps"), knobs["SYNC"],
                                               topology=ch.topology())
                if refusal:
                    raise SystemExit(f"nxs tune: {ch.name}: {refusal}")
                camera = port.setdefault("camera", {})
                # A mode with no lawful free-running rate carries no fps
                # knob: the declaration is the mode alone.
                camera["mode"] = (f"{knobs['PRESET'][0]}@{knobs['fps']}"
                                  if "fps" in knobs else str(knobs["PRESET"][0]))
                camera["sync"] = knobs["SYNC"]
                port.pop("sync", None)
                continue
            for unit in raw.get("units") or []:
                if unit["name"] != sec.unit_name:
                    continue
                sensor = unit["sensors"][sec.sensor_index]
                cfg = sensor.setdefault("config", {})
                for f in sec.fields:
                    cfg[f.name] = f.value
    if not raw.get("ports") and not raw.get("units"):
        raise SystemExit("nxs tune: nothing declared to save — set a HUB and "
                         "a link sensor, or a unit's PERSONALITY, first")
    backup = None
    if existed:
        backup = f"{path}.bak-{time.strftime('%Y%m%d-%H%M%S')}"
        with open(path, encoding="utf-8") as fh:
            original = fh.read()
        with open(backup, "w", encoding="utf-8") as fh:
            fh.write(original)
    else:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        yaml_rt.dump(raw, fh)
    return backup


def _write_declaration(raw, ch):
    """Write (or update) the manifest entry a strip's DECLARE section
    describes: a port's hub and link sensors, a unit's personality."""
    from nxs.suite.freeze import port_block

    knobs = ch.knobs()
    if ch.kind == "port":
        hub = knobs.get("HUB", NONE)
        if hub == NONE:
            return
        ports = raw.setdefault("ports", {})
        entry = ports.get(ch.name)
        topology = ch.topology()
        if topology is None:
            # A hub with no link sensor yet is a lawful declaration that must
            # persist: one `suite_set` writes one knob at a time.
            if entry is None:
                bus = (ch.template.i2c_bus if ch.template is not None
                       else ch.port.bus if ch.port is not None else None)
                entry = ports[ch.name] = {"bus": bus}
            _refuse_stale_links(ch, hub, entry)
            _write_hub(entry, hub)
            return
        block = port_block(topology)
        block["hub"]["driver"] = "nxs"
        if entry is None:
            ports[ch.name] = block
            return
        _refuse_stale_links(ch, hub, entry, updated=block["links"])
        # An existing entry keeps what the operator wrote: the hub follows the
        # knob, a link takes the sensor and keeps its other fields.
        _write_hub(entry, hub)
        entry.setdefault("bus", block["bus"])
        entry.setdefault("csi_lanes", block["csi_lanes"])
        links = entry.setdefault("links", {})
        for name, link_block in block["links"].items():
            target = links.get(name)
            if target is None:
                links[name] = link_block
            elif isinstance(target.get("camera"), dict):
                target["camera"]["sensor"] = link_block["camera"]
            else:
                target["camera"] = link_block["camera"]
        return
    personality = knobs.get("PERSONALITY", NONE)
    if personality == NONE or ch.declared:
        return
    link, serial = ch.template
    units = raw.setdefault("units", [])
    config = {}
    sensor_section = next((s for s in ch.sections if s.kind == "sensor"), None)
    if sensor_section is not None:
        config = {f.name: f.value for f in sensor_section.fields}
    entry = {"name": ch.name, "module": "nxs",
             "links": [{"transport": "i2c", "bus": link.bus, "address": link.address}]}
    if serial and serial != "?":
        entry["serial"] = serial
    entry["sensors"] = [{"personality": personality, "config": config} if config
                        else {"personality": personality}]
    # A save is idempotent: the strip stays undeclared until the model
    # reloads, so a second save replaces the entry instead of doubling it.
    for i, existing in enumerate(units):
        if isinstance(existing, dict) and existing.get("name") == ch.name:
            units[i] = entry
            return
    units.append(entry)


def _write_hub(entry, hub):
    """The port entry's hub follows the knob; a hub written as its bare compatible
    stays so while the knob agrees, and becomes a mapping when it moves."""
    current = entry.get("hub")
    if isinstance(current, str):
        if current == hub:
            return
        current = {"compatible": current}
    if not isinstance(current, dict):
        current = {}
    entry["hub"] = current
    current["compatible"] = hub
    current.setdefault("driver", "nxs")


def _refuse_stale_links(ch, hub, entry, updated=()):
    """A hub change never leaves a link declared with a sensor the new hub's
    pack cannot serve: links the save does not rewrite must already fit, else
    the refusal names the one command that carries both knobs."""
    if ch.port is None or hub == ch.port.hub_compatible:
        return
    offered = _pack_sensors(hub)
    for name, link in (entry.get("links") or {}).items():
        if name in updated or not isinstance(link, dict):
            continue
        camera = link.get("camera")
        sensor = camera.get("sensor") if isinstance(camera, dict) else camera
        if sensor and sensor not in offered:
            example = next((s for s in offered if s != NONE), "<sensor>")
            raise SystemExit(
                f"nxs tune: {ch.name}: hub {hub} does not carry {sensor} on "
                f"link {name} — a hub change across packs carries its link "
                f"sensors in the same command: nxs tune --set {ch.name}:HUB="
                f"{hub} --set {ch.name}:sensor-{name}={example}")


def check_saved():
    from nxs.check import check_manifest
    from nxs.suite import default_config_path

    return check_manifest(default_config_path())


def play():
    """Apply: signal nxsd to reconverge the diff."""
    for cmd in (["systemctl", "kill", "-s", "HUP", "nxsd.service"],
                ["pkill", "-HUP", "-x", "nxsd"]):
        if subprocess.run(cmd, capture_output=True).returncode == 0:
            return True
    return False


def identify(section):
    """Strobe the focused section's unit LED so hands match strips."""
    if section is None or section.kind != "sensor":
        return False
    identify_unit(section.unit_name)
    return True


def identify_unit(name):
    """Strobe one unit's LED in the background."""
    subprocess.Popen([sys.executable, "-m", "nxs.cli", "--unit", name, "identify"],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def identify_route(link):
    """Strobe the LED of the unit on an I2C route the manifest does not name."""
    subprocess.Popen([sys.executable, "-m", "nxs.cli", "-t", "i2c", "-b", link.bus,
                      "-a", f"{link.address:#x}", "identify"],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def freeze():
    """Adopt live state into the manifest (the freeze machinery)."""
    from nxs.suite import default_config_path
    from nxs.suite.freeze import run_freeze
    return run_freeze(default_config_path()) == 0


# ---------------------------
# Non-interactive model access (scripts, `nxs mcp`)
# ---------------------------

def _token(option):
    """The addressable value of an option (a PRESET carries a label)."""
    return option[0] if isinstance(option, tuple) else option


def describe(path, channels):
    """The `tune` surface: every channel, section, field, and option."""
    from nxs.schemas import CONTRACT

    out = []
    for ch in channels:
        sections = []
        for sec in ch.sections:
            fields = []
            for f in sec.fields:
                entry = {"name": f.name, "value": _token(f.value),
                         "options": [_token(o) for o in f.options]}
                if any(isinstance(o, tuple) for o in f.options):
                    entry["labels"] = [f.render(o) for o in f.options]
                if f.unit:
                    entry["unit"] = f.unit
                fields.append(entry)
            sections.append({"label": sec.label, "kind": sec.kind,
                             "unit": sec.unit_name, "fields": fields})
        entry = {"name": ch.name, "kind": ch.kind, "declared": ch.declared,
                 "presence": ch.presence, "sections": sections}
        if ch.note:
            entry["note"] = ch.note
        out.append(entry)
    return {"contract": CONTRACT, "manifest": path, "channels": out}


def parse_set(spec):
    """``CHANNEL[:SECTION]:FIELD=VALUE`` to (channel, section, field, value);
    SECTION is ``CAMERA`` or a unit name, optional when the field is unique."""
    if "=" not in spec:
        raise SystemExit(f"nxs tune --set {spec!r}: expected "
                         f"CHANNEL[:SECTION]:FIELD=VALUE")
    address, value = spec.split("=", 1)
    parts = address.split(":")
    if len(parts) == 2:
        channel, section, field = parts[0], None, parts[1]
    elif len(parts) == 3:
        channel, section, field = parts
    else:
        raise SystemExit(f"nxs tune --set {spec!r}: expected "
                         f"CHANNEL[:SECTION]:FIELD=VALUE")
    return channel.strip(), (section.strip() if section else None), \
        field.strip(), value.strip()


def _section_matches(sec, selector):
    if selector is None:
        return True
    s = selector.lower()
    return (sec.label.lower() == s or (sec.kind == "camera" and s == "camera")
            or (sec.unit_name or "").lower() == s
            or sec.label.split(" · ", 1)[-1].lower() == s
            or sec.label.split(" · ", 1)[-1].split("/", 1)[0].lower() == s)


def resolve_field(channels, channel, section, field):
    """The (channel, section, field) a --set address names, or a refusal
    that lists what exists."""
    ch = next((c for c in channels if c.name.lower() == channel.lower()),
              None)
    if ch is None:
        have = ", ".join(c.name for c in channels) or "(none declared)"
        raise SystemExit(f"nxs tune: no channel {channel!r}; have {have}")
    hits = [(sec, f) for sec in ch.sections if _section_matches(sec, section)
            for f in sec.fields if f.name.lower() == field.lower()]
    if len(hits) == 1:
        sec, f = hits[0]
        return ch, sec, f
    if not hits:
        have = ", ".join(f"{sec.label}:{f.name}" for sec in ch.sections
                         for f in sec.fields)
        raise SystemExit(f"nxs tune: {channel} has no field {field!r}"
                         + (f" in section {section!r}" if section else "")
                         + f"; have {have}")
    names = ", ".join(sec.label for sec, _ in hits)
    raise SystemExit(f"nxs tune: {channel}:{field} is ambiguous — name the "
                     f"section: {channel}:<SECTION>:{field} (sections: "
                     f"{names})")


def apply_sets(channels, specs):
    """Apply ``--set`` specs to the model (values must be offered options);
    returns (applied records, touched sections)."""
    applied, touched = [], []
    moved = {}
    for spec in specs:
        channel, section, field, value = parse_set(spec)
        ch, sec, f = resolve_field(channels, channel, section, field)
        wanted = value.lower()
        index = next((i for i, o in enumerate(f.options)
                      if str(_token(o)).lower() == wanted), None)
        if index is None and f.bounds is not None:
            # A range parameter takes any integer inside its bounds.
            try:
                number = int(value)
            except ValueError:
                number = None
            if number is not None and f.bounds[0] <= number <= f.bounds[1]:
                f.options = sorted(set(f.options) | {number})
                index = f.options.index(number)
        if index is None:
            offered = (f"{f.bounds[0]}..{f.bounds[1]}" if f.bounds is not None
                       else ", ".join(str(_token(o)) for o in f.options))
            hint = (" (set HUB first: the sensor knobs offer the chosen hub's pack)"
                    if f.name.startswith("sensor-") and f.options == [NONE] else "")
            raise SystemExit(f"nxs tune: {ch.name}:{f.name} offers "
                             f"{offered} — not {value!r}{hint}")
        f.index = index
        applied.append({"channel": ch.name, "section": sec.label,
                        "field": f.name, "value": _token(f.value)})
        if sec not in touched:
            touched.append(sec)
        if sec.kind == "camera" and f.name in ("PRESET", "SYNC"):
            note, snapped = refresh(sec)
            if snapped:
                moved[sec] = note
        elif sec.kind == "camera" and f.name == "fps":
            moved.pop(sec, None)
        elif sec.kind == "declare":
            # The dependent knobs appear or change under the declaration;
            # a later --set in the same batch addresses them.
            refresh(sec)
    if moved:
        raise SystemExit("nxs tune: " + "; ".join(moved.values())
                         + " — name the fps in the same --set batch")
    return applied, touched


def _render_list(payload):
    for ch in payload["channels"]:
        tags = [t for t in (ch.get("presence") if ch.get("presence") != "unknown" else None,
                            None if ch.get("declared", True) else "undeclared") if t]
        print(f"{ch['name']}" + (f"    ({', '.join(tags)})" if tags else ""))
        if ch.get("note"):
            print(f"  {ch['note']}")
        for sec in ch["sections"]:
            print(f"  {sec['label']}")
            for f in sec["fields"]:
                opts = " | ".join(str(o) for o in f["options"])
                unit = f" {f['unit']}" if f.get("unit") else ""
                print(f"    {f['name']:<12} = {f['value']}{unit}"
                      f"    [{opts}]")


def run_batch(args):
    """`--list` / `--set` / `--play` / `--freeze` without curses; JSON on
    request."""
    from nxs.suite.schema import ManifestError

    if getattr(args, "freeze", False):
        from nxs.suite import default_config_path
        from nxs.suite.freeze import run_freeze
        return run_freeze(default_config_path(),
                          only_unit=getattr(args, "only_unit", None),
                          dry_run=getattr(args, "dry_run", False),
                          ports=getattr(args, "ports", False))
    if getattr(args, "schema", False):
        from nxs.suite import default_config_path
        from nxs.suite.rig_schema import rig_schema
        from nxs.suite.schema import load_suite_config
        try:
            cfg = load_suite_config(default_config_path())
        except ManifestError as e:
            raise SystemExit(f"nxs tune: {e}")
        print(json.dumps(rig_schema(cfg), indent=2))
        return 0
    try:
        path, _cfg, channels = load_model()
    except ManifestError as e:
        raise SystemExit(f"nxs tune: {e}")
    if not channels:
        raise SystemExit(f"nxs tune: {nothing_tunable(_cfg)}")
    payload = describe(path, channels)
    rc = 0
    if args.set:
        applied, touched = apply_sets(channels, args.set)
        backup = save_model(path, channels, only=touched)
        payload = describe(path, channels)
        payload["applied"] = applied
        if backup:
            payload["backup"] = backup
        findings = check_saved()
        payload["findings"] = as_data(findings)
        rc = 1 if findings else 0
    if args.play:
        findings = check_saved()
        payload["findings"] = as_data(findings)
        played = False if findings else play()
        payload["played"] = played
        rc = 1 if (findings or not played) else rc
    if args.json:
        print(json.dumps(payload, indent=2))
        return rc
    if args.list or not (args.set or args.play):
        _render_list(payload)
    for rec in payload.get("applied", []):
        print(f"set {rec['channel']}:{rec['section']}:{rec['field']} = "
              f"{rec['value']}")
    if "backup" in payload:
        print(f"saved (backup {os.path.basename(payload['backup'])})")
    findings = payload.get("findings")
    if findings:
        print(f"OUT OF TUNE — {len(findings)} finding(s):")
        for f in findings:
            print(f"  {f['text']}")
    elif "findings" in payload:
        print("IN TUNE ✓")
    if "played" in payload:
        print("PLAYING ▶ nxsd reconverging" if payload["played"]
              else "not applied — nxsd not running (or the manifest is "
                   "out of tune)")
    return rc




def cmd_tune(args=None) -> int:
    batch = args is not None and (
        getattr(args, "list", False) or getattr(args, "set", None)
        or getattr(args, "play", False) or getattr(args, "freeze", False)
        or getattr(args, "schema", False))
    try:
        if batch:
            return run_batch(args)
        from nxs.tune_tui import run_tui

        return run_tui() or 0
    except SystemExit:
        raise
    except Exception as e:
        raise SystemExit(f"nxs tune: {e}")
