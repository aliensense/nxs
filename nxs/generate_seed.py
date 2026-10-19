# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The declaration `nxs generate` writes: the seed from its walk, a first
suite.yaml naming every pod and unit that answered, by where it answered,
and the intent the live ports already carry; and a declaration written in
an earlier language, api 2 or the 1.0 form, written again in api 3."""

from __future__ import annotations

import os
import sys
from typing import Callable, List, Optional, Tuple


def _unit_name(port: str, link: str) -> str:
    return f"unit-{port}-{link.lower()}"


def pod_alias(name: str, taken=()) -> int:
    """The alias a pod behind a hub is declared at: two addresses up from
    the pair every NXS straps, its register map and its camera map, per
    link letter (A 0x32, B 0x34, the camera map one above each), so the host
    never speaks a strapped address on the port and the pods do not
    collide when the hub merges its links. A link named otherwise takes the
    first pair above the strapped one that `taken` leaves free."""
    from nxs._generated_constants import NxsDevices

    base = int(NxsDevices.RBDevice.NXS)
    if len(name) == 1 and name.isascii() and name.isalpha():
        return base + 2 * (1 + ord(name.upper()) - ord("A"))
    alias = base + 2
    while alias in taken:
        alias += 2
    return alias


def pod_aliases(names) -> dict:
    """Link name -> the alias of its pod, a pair each: a link named by a
    letter takes its letter's (`pod_alias`), any other the first pair the
    others leave free, in order."""
    aliases = {name: pod_alias(name) for name in names if len(name) == 1 and name.isascii()
               and name.isalpha()}
    for name in names:
        if name not in aliases:
            aliases[name] = pod_alias(name, set(aliases.values()))
    return {name: aliases[name] for name in names}


def name_by_trial(units: list, opener=None) -> None:
    """A unit running nothing is asked by trial: every personality the tool
    knows uploaded in turn, and the one whose sensor answers stays running
    and names the unit's Click (`nxs.detect`)."""
    from nxs.detect import detect
    from nxs.transports import open_client

    opener = opener or open_client
    for unit in units:
        if unit.personality or unit.link is None:
            continue
        client = None
        try:
            client = opener(unit.link.transport, **unit.link.client_kwargs())
            print(f"{unit.route}: runs nothing; trying every personality",
                  file=sys.stderr)
            name = detect(client, report=lambda line: print(f"  {line}",
                                                             file=sys.stderr))
            if name:
                unit.personality = client.read_personality_name() or name
        except Exception as exc:        # noqa: BLE001 (the unit's own refusals)
            print(f"{unit.route}: the trial failed: {exc}", file=sys.stderr)
        finally:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass


def seed_from_walk(ports: list, units: list, path: str) -> dict:
    """A first suite.yaml as the panel would save this walk: the model over
    the walk's findings, every new node's DECLARE knobs set to what answered
    (the hub, each link's identified head and its pod, a unit's running
    personality), the live port record's frame sync, modes and rates on the
    hub's and the cameras' rows, then the one writer. A head that only
    answers is left to declare."""
    from nxs import host as host_layer
    from nxs.cam import port_state
    from nxs.tune_fields import NONE, refresh
    from nxs.tune_model import CONNECTOR, load_model
    from nxs.tune_sweep import assemble
    from nxs.tune_write import declaration

    try:
        known = dict(host_layer.current().camera_buses())
    except Exception:        # noqa: BLE001 (a host without port rules names no bus)
        known = {}
    buses = {port.name: port.bus for port in ports if port.bus}
    sweep = assemble(ports, units, {bus: None for bus in buses.values()})
    templates = {port.name: port.topology for port in ports if port.topology is not None}
    _path, _cfg, channels = load_model(sweep=sweep, path=path, buses=buses, templates=templates)
    families = {ch.name: ch.family for ch in channels if ch.kind == "port"}
    records = {}
    for port in ports:
        family = families.get(port.name)
        if family is None:
            continue
        heads = [l for l in port.links if l.sensor and not l.unwalked]
        # A hub's entry names its control bus where the host does not resolve
        # the port's name to it; a connector's bus is the host's alone.
        family.pinned_bus = (port.bus if port.hub_present and port.hub
                             and known.get(port.name) != port.bus else None)
        if port.hub_present and port.hub:
            _turn(family.port, "HUB", port.hub)
        elif heads and port.topology is not None and port.topology.is_direct:
            _turn(family.port, "HUB", CONNECTOR)
        else:
            continue
        collided = set(port.collided[1]) if port.collided else set()
        for link in port.links:
            channel = family.link(link.name)
            if channel is None:
                continue
            if link.sensor and not link.unwalked:
                _turn(channel, "CAM", link.sensor)
            if link.unit_addr is not None or link.name in collided:
                _turn(channel, "POD", _unit_name(port.name, link.name))
        records[port.name] = port_state.port_record(port.topology) if port.topology is not None else {}
    for channel in channels:
        if channel.kind != "unit" or channel.declared:
            continue
        # Every unit found is declared, running a known personality or none.
        channel.adopt = True
        knob = channel.declare().fields[0]
        running = next((o for o in knob.options if o != NONE), None) if channel.running else None
        if running is not None:
            _turn(channel, "CLICK", running)
            refresh(channel.declare())
    # The live record on the rows the structure opened: the port's sync, each link's mode and
    # rate. A record outlives its cameras: its sync counts where a link carries a camera today.
    for name, own in records.items():
        family = families[name]
        sync = own.get("sync") or {}
        cameras = [ch for ch in family.links if ch.knobs().get("CAM", NONE) != NONE]
        synced = sync.get("source") == "fsync" and bool(cameras)
        if synced:
            _turn(family.port, "sync_source", "fsync", kind="rows")
            if sync.get("fps"):
                _turn(family.port, "sync_fps", _rate_value(sync["fps"]), kind="rows")
        modes, rates = own.get("modes") or {}, own.get("rates") or {}
        viewers = own.get("viewers") or ({l.letter: own["viewer"] for l in family.links}
                                         if own.get("viewer") else {})
        for channel in family.links:
            mode = modes.get(channel.letter)
            if mode is None and viewers.get(channel.letter):
                mode = _mode_by_geometry(channel, viewers[channel.letter])
            if mode:
                _turn(channel, "mode", str(mode), kind="rows")
            if rates.get(channel.letter) and not synced:
                # The rate row takes a rate to the thousandth of a frame a second.
                _turn(channel, "fps", _rate_value(round(float(rates[channel.letter]), 3)),
                      kind="rows")
    return declaration(channels)


def _mode_by_geometry(channel, geometry: dict):
    """The mode a link's camera offers for a viewer's geometry, None for a
    geometry its modes name none or several of."""
    from nxs.cam import cam_personalities
    from nxs.cam.descriptors import resolve_mode

    sensor = channel.knobs().get("CAM")
    try:
        return resolve_mode(cam_personalities.find(sensor),
                            f"{geometry.get('width')}x{geometry.get('height')}")
    except Exception:        # noqa: BLE001 (several modes share the geometry, or none)
        return None


def _turn(channel, name: str, value, kind: Optional[str] = None) -> bool:
    """Set the channel's knob `name` to `value` where offered, spelled out at
    save whatever the default; the dependent knobs rebuild. False when the
    knob or the value is not there."""
    from nxs.tune_fields import refresh
    from nxs.tune_model import admit

    for section in channel.sections:
        if kind is not None and section.kind != kind:
            continue
        for field in section.fields:
            if field.name != name:
                continue
            tokens = [o[0] if isinstance(o, tuple) else o for o in field.options]
            wanted = value
            if wanted not in tokens and isinstance(value, (int, float)):
                wanted = next((t for t in tokens if isinstance(t, (int, float)) and float(t) == float(value)), None)
            index = tokens.index(wanted) if wanted in tokens else admit(field, str(value))
            if index is None:
                return False
            field.index = index
            field.default = None
            refresh(section)
            return True
    return False


def _rate_value(value):
    rate = float(value)
    return int(rate) if rate.is_integer() else rate


_SEED_HEADER = ("# Written by `nxs generate` from what it found. This file is yours to\n"
                "# edit, by hand or with `nxs tune`: names, personalities, settings. The wiring\n"
                "# lives in hardware.yaml beside it and is rewritten on every run.\n")


def _undeclared(ports: list, config_path: str) -> List[str]:
    """The ports where something answered, a hub or a link's node, and the
    kept declaration does not name. A port's bus and lanes alone are no
    answer."""
    from nxs.suite.schema import ManifestError, load_suite_config
    try:
        declared = load_suite_config(config_path).ports
    except ManifestError:
        declared = {}
    return [port.name for port in ports
            if (port.hub_present or port.links) and port.name not in declared]


#: Where an earlier declaration is kept once `generate` writes it in api 3,
#: by its language: the 1.0 form, api 2.
EARLIER_SUFFIXES = {"1.0": ".1.0", "api 2": ".api2"}


def _earlier(config_path: str) -> Tuple[Optional[str], Optional[dict]]:
    """`(language, document)` of the declaration at `config_path` when it is
    written in an earlier language, the 1.0 form (`ports` and `units`) or api
    2; `(None, None)` otherwise."""
    import yaml

    from nxs.suite.convert import is_api2
    from nxs.suite.nodes import is_1_0_form
    try:
        with open(config_path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
    except (OSError, yaml.YAMLError):
        return None, None
    if is_1_0_form(raw):
        return "1.0", raw
    if is_api2(raw):
        return "api 2", raw
    return None, None


def node_form(raw: dict, language: str = "1.0") -> dict:
    """The api-3 declaration an earlier one states, a 1.0 file through its
    port view and an api-2 file as it stands, held to the loader before
    anything is written."""
    from nxs.suite.convert import api3
    from nxs.suite.port_view import nodes_from_view
    from nxs.suite.schema import parse_suite_config
    from nxs.tune_write import _camera_buses

    doc = nodes_from_view(raw, buses=_camera_buses()) if language == "1.0" else api3(raw)
    parse_suite_config(doc)
    return doc


def convert_earlier(config_path: str, dry_run: bool, log: Callable[[str], None]) -> Optional[str]:
    """Write a declaration of an earlier language (the 1.0 form, api 2) again
    in api 3, the earlier file kept beside it (`EARLIER_SUFFIXES`), and `log`
    the line that says so at once: a walk that fails after it leaves the
    file converted. Returns the kept path; None for a file of api 3. A dry
    run, and an earlier declaration api 3 cannot load, are refused with
    nothing written."""
    from nxs.suite.freeze import _write_atomic, render_seed
    from nxs.suite.nodes import WRITES_NODE_FORM
    from nxs.suite.schema import ManifestError

    language, raw = _earlier(config_path)
    if raw is None:
        return None
    if dry_run:
        raise ManifestError(f"{config_path}: declares {_named(language)}, and a dry run walks the "
                            f"rig as api 3 declares it", [WRITES_NODE_FORM])
    try:
        doc = node_form(raw, language)
    except ManifestError as exc:
        raise ManifestError(f"{config_path}: the {language} declaration does not convert: "
                            f"{exc.fact}", exc.alternatives) from None
    with open(config_path, encoding="utf-8") as f:
        original = f.read()
    kept = config_path + EARLIER_SUFFIXES[language]
    _write_atomic(kept, lambda f: f.write(original))
    header = (f"# Written by `nxs generate` from the {language} declaration, kept beside it as "
              f"{os.path.basename(kept)}.\n")
    _write_atomic(config_path, lambda f: f.write(header + render_seed(doc)))
    log(f"wrote {config_path} in api 3, the {language} declaration kept as {kept}")
    for name in _unmanaged(raw, language):
        log(f"{name}: the {language} declaration left its socket unmanaged, and api 3 declares "
            f"it empty: nxs switch empties its store unless a click node feeds {name}.click")
    for name, was, now in _realiased(raw, language):
        log(f"{name}: the {language} declaration put its pod at {was:#04x}, and api 3 puts it at "
            f"its link's letter's alias {now:#04x}")
    return kept


def _realiased(raw: dict, language: str) -> List[Tuple[str, int, int]]:
    """The pods an earlier declaration put at an alias of its own: `(unit,
    that alias, the alias its link's letter gives)`, which api 3 derives."""
    found = []
    if language == "1.0":
        for port in (raw.get("ports") or {}).values():
            for letter, link in ((port or {}).get("links") or {}).items():
                unit = (link or {}).get("unit")
                if isinstance(unit, dict) and unit.get("alias") is not None:
                    found.append((str(unit.get("name")), int(str(unit["alias"]), 0), str(letter)))
    else:
        nodes = raw.get("nodes") or {}
        for source, targets in (raw.get("connections") or {}).items():
            node, _, port = str(source).partition(".")
            alias = (((nodes.get(node) or {}).get("ports") or {}).get("out") or {}).get("alias")
            if port != "out" or alias is None:
                continue
            for target in targets or []:
                if ".links." in str(target):
                    found.append((node, int(str(alias), 0), str(target).rsplit(".", 1)[1]))
    return [(name, was, pod_alias(letter)) for name, was, letter in found
            if was != pod_alias(letter)]


def _unmanaged(raw: dict, language: str) -> list:
    """The units an earlier declaration leaves their socket unmanaged on: a
    1.0 unit without `sensors`, an api-2 unit no click feeds that does not
    declare its socket empty."""
    if language == "1.0":
        return [str(u.get("name")) for u in raw.get("units") or [] if "sensors" not in u]
    nodes = raw.get("nodes") or {}
    fed = {str(t).split(".", 1)[0] for targets in (raw.get("connections") or {}).values()
           for t in targets or [] if str(t).endswith(".click")}
    return [str(n) for n, spec in nodes.items()
            if (spec or {}).get("kind") == "unit" and str(n) not in fed
            and (spec.get("parameters") or {}).get("click") != "none"]


def _named(language: str) -> str:
    return "the 1.0 form (ports and units)" if language == "1.0" else language
