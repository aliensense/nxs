# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The seed `nxs generate` writes from its walk: a first suite.yaml naming
every pod and unit that answered, by where it answered, and the intent the
live ports already carry."""

from __future__ import annotations

import sys
from typing import Optional


def _unit_name(port: str, link: str) -> str:
    return f"unit-{port}-{link.lower()}"


def pod_alias(letter: str) -> int:
    """The alias a pod behind a hub is declared at: the addresses up from
    the one every NXS straps, one per link letter (A 0x31, B 0x32), so the
    host never speaks the strapped address on the port and the pods do
    not collide when the hub merges its links."""
    from nxs.suite.scan import I2C_ADDRESSES

    return int(I2C_ADDRESSES[0]) + 1 + ord(letter.upper()) - ord("A")


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
    personality), the live port record's mode, rate and sync on the camera
    knobs, then the one writer. A head that only answers is left to declare."""
    from nxs import tune_camera as laws
    from nxs.cam import port_state
    from nxs.tune_fields import NONE, refresh
    from nxs.tune_model import CONNECTOR, load_model
    from nxs.tune_sweep import assemble
    from nxs.tune_write import _plain, write_model

    from nxs import host as host_layer

    buses = {port.name: port.bus for port in ports if port.bus}
    sweep = assemble(ports, units, {bus: None for bus in buses.values()})
    templates = {port.name: port.topology for port in ports if port.topology is not None}
    _path, _cfg, channels = load_model(sweep=sweep, path=path, buses=buses, templates=templates)
    try:
        known = dict(host_layer.current().camera_buses())
    except Exception:        # noqa: BLE001 (a host without port rules names no bus)
        known = {}
    families = {ch.name: ch.family for ch in channels if ch.kind == "port"}
    for port in ports:
        family = families.get(port.name)
        if family is None:
            continue
        # The entry names its bus where the host does not resolve the port's name to it.
        family.pinned_bus = port.bus if known.get(port.name) != port.bus else None
        heads = [l for l in port.links if l.sensor and not l.unwalked]
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
                _turn(channel, "SENSOR", link.sensor)
            if link.unit_addr is not None or link.name in collided:
                _turn(channel, "POD", _unit_name(port.name, link.name))
        # The live record: the mode and rate each link runs, the port's sync.
        own = port_state.port_record(port.topology) if port.topology is not None else {}
        sync = own.get("sync") or {}
        if sync.get("source") == "fsync":
            _turn(family.port, "source", "fsync", kind="sync")
            if sync.get("fps"):
                _turn(family.port, "fps", _rate_value(sync["fps"]), kind="sync")
        modes, rates = own.get("modes") or {}, own.get("rates") or {}
        viewers = own.get("viewers") or ({l.name: own["viewer"] for l in port.links} if own.get("viewer") else {})
        hub = laws.hub_for(family.topology())
        for link in port.links:
            channel = family.link(link.name)
            if channel is None or channel.camera() is None:
                continue
            mode = modes.get(link.name)
            if mode is None and viewers.get(link.name) and hub is not None and link.sensor:
                geometry = viewers[link.name]
                mode = laws.mode_name(hub, link.sensor, f"{geometry.get('width')}x{geometry.get('height')}")
            if mode:
                _turn(channel, "mode", str(mode), kind="camera")
            if rates.get(link.name) and not sync.get("source") == "fsync":
                _turn(channel, "fps", _rate_value(rates[link.name]), kind="camera")
    for channel in channels:
        if channel.kind != "unit" or channel.declared:
            continue
        # Every unit found is declared, running a known personality or none.
        channel.adopt = True
        knob = channel.declare().fields[0]
        running = next((o for o in knob.options if o != NONE), None) if channel.running else None
        if running is not None:
            _turn(channel, "PERSONALITY", running)
            refresh(channel.declare())
    raw: dict = {}
    write_model(raw, channels)
    return _plain(raw)


def _turn(channel, name: str, value, kind: Optional[str] = None) -> bool:
    """Set the channel's knob `name` to `value` where offered, spelled out at
    save whatever the default; the dependent knobs rebuild. False when the
    knob or the value is not there."""
    from nxs.tune_fields import refresh

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
            if wanted not in tokens:
                return False
            field.index = tokens.index(wanted)
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
