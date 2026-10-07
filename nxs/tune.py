# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""`nxs tune`: the personality panel for the machine's declared config. Every value
comes from a descriptor, so an impossible setting cannot be typed; curses only
renders the model, and `--list`, `--set`, `--play` drive it for scripts."""
import json
import os
import subprocess
import sys

from nxs.finding import as_data
from nxs.tune_fields import NONE, Channel, Field, Section, refresh
from nxs.tune_model import load_model
from nxs.tune_write import save_model, write_model

#: The names `nxs.tune` has always answered to, wherever they now live.
__all__ = [
    "NONE", "Channel", "Field", "Section", "apply_sets",
    "check_saved", "cmd_tune", "describe", "freeze", "identify", "identify_route",
    "identify_unit", "load_model",
    "nothing_tunable", "parse_set", "play", "refresh", "resolve_field",
    "run_batch", "save_model", "write_model",
]


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
        # personalities did not load or expose no parameters.
        names = ", ".join(u.name for u in cfg.units)
        return (f"nothing tunable declared — {names} declare sensors whose "
                f"personalities did not load or expose no parameters; nxs status "
                f"names the personality errors")
    if cfg.ports:
        return (f"nothing tunable declared — {', '.join(sorted(cfg.ports))} "
                f"offers no camera strip: the hub is missing "
                f"or names no sensor for its links")
    return ("nothing tunable — this host names no camera port (no platform "
            "rules for it) and suite.yaml declares nothing; nxs generate "
            "writes the units that answer on any route")


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


def _family_knobs(family):
    """(channel, knob) -> value for every knob of a port's family of channels."""
    out = {}
    for ch in [family.port, *family.links]:
        for sec in ch.sections:
            for fld in sec.fields:
                out[(ch.name, fld.name)] = fld.value
    return out


def apply_sets(channels, specs):
    """Apply ``--set`` specs to the model (values must be offered options);
    returns (applied records, touched sections). A knob that a rebuild moved
    away from its value is refused unless the batch names it afterwards."""
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
                numbers = sorted({o for o in f.options if o != NONE} | {number})
                f.options = ([NONE] if NONE in f.options else []) + numbers
                index = f.options.index(number)
        if index is None:
            offered = (f"{f.bounds[0]}..{f.bounds[1]}" if f.bounds is not None
                       else ", ".join(str(_token(o)) for o in f.options))
            raise SystemExit(f"nxs tune: {ch.name}:{f.name} offers "
                             f"{offered} — not {value!r}")
        f.index = index
        f.default = None        # a set knob is spelled out at save
        applied.append({"channel": ch.name, "section": sec.label,
                        "field": f.name, "value": _token(f.value)})
        if sec not in touched:
            touched.append(sec)
        # The batch names the knob a rebuild moved: that note is answered.
        moved.pop((ch.name, f.name), None)
        family = getattr(ch, "family", None)
        before = _family_knobs(family) if family is not None else {}
        note, snapped = refresh(sec)
        if snapped and note:
            after = _family_knobs(family) if family is not None else {}
            for key, value in after.items():
                if key in before and before[key] != value:
                    moved[key] = note
    if moved:
        raise SystemExit("nxs tune: " + "; ".join(dict.fromkeys(moved.values()))
                         + " — name the knob in the same --set batch")
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
                          ports=getattr(args, "ports", False),
                          only_port=getattr(args, "only_port", None))
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
