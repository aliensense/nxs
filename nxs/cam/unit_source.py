# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""A link's unit as the source of its sensor descriptor. The camera
personality a unit holds carries the descriptor trailer; this module reads
it over the bus, decodes it, and keeps it in the operator's state directory
so that `pack_for` can adopt it without touching the bus and `on` runs on a
robot that holds no hub."""

from __future__ import annotations

import dataclasses
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from nxs.client import (ERRNO_EBADF, ERRNO_ENOTSUP, DeviceRefused, SupportsCameraRun,
                        SupportsSlotPeek)
from nxs.personality.records import ParamMap

from .contracts import ContractError

from .contracts import LinkSpec, Topology
from .descriptors import Descriptor
from . import port_state

#: Where the cached descriptors live under `port.state_dir()`.
CACHE_DIR = "unit-descriptors"
#: Store slots a unit can hold (ids 0..7).
MAX_SLOTS = 8
#: The peek refusals that name the image a slot holds, by the unit's errno,
#: as what a pod holds there: the slot is taken, and a reinstall replaces it.
STALE_PEEKS = {ERRNO_ENOTSUP: "of another format version", ERRNO_EBADF: "that does not parse"}


@dataclasses.dataclass(frozen=True)
class UnitPersonality:
    """The cam personality a unit holds: its slot, the image name the
    unit reports, the trailer's CRC-32, the descriptor it decodes to (None
    when a caller's cache was already current), and how its `mode` and
    `trigger` params are staged."""

    slot: int
    name: str
    crc: int
    descriptor: Optional[Descriptor]
    params: "ParamMap"

    @property
    def modes(self) -> Dict[str, int]:
        """Mode name -> the `mode` param value that selects it."""
        return dict(self.params.modes)

    @property
    def compatible(self) -> str:
        return self.descriptor.compatible if self.descriptor is not None else ""

    def summary(self) -> str:
        """`cam personality <name> (<compatible>), N modes`."""
        n = len(self.modes)
        return (f"cam personality {self.name} ({self.compatible}), "
                f"{n} mode{'s' if n != 1 else ''}")


def cache_path(topology: Topology, link: LinkSpec) -> Path:
    """`<state_dir>/unit-descriptors/<port>-<link>.yaml`."""
    directory = port_state.ensure_dir(port_state.state_dir() / CACHE_DIR)
    return directory / f"{port_state.port_name(topology)}-{link.name}.yaml"


def cache_descriptor(topology: Topology, link: LinkSpec, descriptor: Descriptor,
                     slot: int, crc: int, params: Optional[ParamMap] = None,
                     name: Optional[str] = None,
                     image_crc: Optional[int] = None, hub=None) -> Path:
    """Write the descriptor a unit served, with the slot it lives in, the
    trailer CRC-32 a refresh compares, the staging facts of its params, and
    the image's CRC-32 when the upload knew it. A tree descriptor is encoded
    with `hub`, the hub the staged trailer was built with, so the cached
    rows are the unit's. Returns the file written."""
    from nxs.personality.records import (decode_trailer, encode_trailer,
                                         mode_values, param_map)

    if descriptor.from_unit:
        doc = _descriptor_doc(descriptor)
    else:
        records = encode_trailer(descriptor, hub=hub)
        doc = decode_trailer(records)
        params = params or param_map(records)
    if params is None:
        params = ParamMap(None, mode_values(descriptor), None, {})
    record: Dict[str, Any] = {
        "name": descriptor.name,
        "personality": {
            "name": str(name or descriptor.name), "slot": int(slot),
            "crc": int(crc), "compatible": descriptor.compatible,
            "mode_index": params.mode_index,
            "modes": {str(k): int(v) for k, v in params.modes.items()},
            "trigger_index": params.trigger_index,
            "triggers": {str(k): int(v) for k, v in params.triggers.items()},
        },
        "descriptor": doc,
    }
    if image_crc is not None:
        record["personality"]["image_crc"] = int(image_crc)
    path = cache_path(topology, link)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(yaml.safe_dump(record, sort_keys=False), encoding="utf-8")
    os.replace(tmp, path)
    return path


def _descriptor_doc(descriptor: Descriptor) -> Dict[str, Any]:
    """The mapping behind a decoded descriptor, as `Descriptor.from_data`
    took it."""
    return {key: descriptor.raw(key) for key in _SECTIONS
            if descriptor.raw(key) is not None}


_SECTIONS = ("meta", "registers", "default_mode", "modes", "limits", "trigger",
             "sync", "test_pattern", "program", "controls", "capture", "status")


def cached_record(topology: Topology, link: LinkSpec) -> Optional[Dict[str, Any]]:
    """The cache file's record for a link, or None when nothing is cached
    or the file does not read."""
    path = cache_path(topology, link)
    try:
        record = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError):
        return None
    if not isinstance(record, dict) or "descriptor" not in record:
        return None
    return record


def cached_descriptor(topology: Topology, link: LinkSpec) -> Optional[Descriptor]:
    """The descriptor cached for a link's unit, or None."""
    record = cached_record(topology, link)
    if record is None:
        return None
    from nxs.cam.contracts import ContractError
    try:
        return Descriptor.from_data(record["descriptor"],
                                    str(record.get("name") or link.name))
    except (ContractError, KeyError, TypeError):
        return None


def cached_params(topology: Topology, link: LinkSpec) -> Optional[ParamMap]:
    """The staging facts cached for a link's unit, or None."""
    record = cached_record(topology, link)
    if record is None:
        return None
    p = record.get("personality") or {}
    return ParamMap(mode_index=p.get("mode_index"),
                    modes={str(k): int(v) for k, v in (p.get("modes") or {}).items()},
                    trigger_index=p.get("trigger_index"),
                    triggers={str(k): int(v) for k, v in (p.get("triggers") or {}).items()})


def forget(topology: Topology, link: LinkSpec) -> None:
    """Drop a link's cached descriptor (after `store rm`, or when the
    unit holds none)."""
    cache_path(topology, link).unlink(missing_ok=True)


def slot_kind(client, slot: int) -> Optional[str]:
    """`'click'`, `'cam'`, or None for an empty slot, from the slot's store
    entry; None too on a transport that reads none."""
    if not isinstance(client, SupportsSlotPeek):
        return None
    info = client.read_slot_info(slot)
    if info is None:
        return None
    kind = getattr(info, "kind", None)
    return str(kind) if kind else None


class UnreadableSlot(RuntimeError):
    """A store slot holds a cam personality whose trailer this nxs
    cannot decode; `slot` names it for `store rm <slot>`."""

    def __init__(self, slot: int, why: str) -> None:
        super().__init__(f"slot {slot} holds a cam personality this nxs cannot "
                         f"read ({why})")
        self.slot = slot


class StaleSlot(UnreadableSlot):
    """A store slot whose image the unit refuses to describe, one of
    STALE_PEEKS: `what` says which (`of another format version`), and a
    reinstall replaces the image in place."""

    def __init__(self, slot: int, what: str, reason: str) -> None:
        RuntimeError.__init__(self, f"slot {slot}: {reason}")
        self.slot = slot
        self.what = what


def read_unit_personality(client, known_crc: Optional[int] = None
                          ) -> Optional[UnitPersonality]:
    """The cam personality in the unit's store, read over an open
    client: the first camera slot whose descriptor trailer carries an
    IDENTITY record (the directory names click personality and empty slots,
    which are skipped). With `known_crc`, a trailer of that CRC-32 is not decoded and
    the returned personality carries no descriptor: the caller's cache is
    current. None when the unit holds no cam personality or the
    transport cannot read trailers; UnreadableSlot when a slot's trailer
    does not decode; StaleSlot, naming the first such slot, when the unit
    holds none it can read and a slot whose image it refuses to describe."""
    from nxs.image import parse_trailer
    from nxs.personality import records

    if not isinstance(client, SupportsCameraRun):
        return None
    stale: Optional[StaleSlot] = None
    for slot in range(MAX_SLOTS):
        try:
            kind = slot_kind(client, slot)
        except DeviceRefused as exc:
            if exc.code not in STALE_PEEKS:
                raise
            stale = stale or StaleSlot(slot, STALE_PEEKS[exc.code], str(exc))
            continue
        if kind != "cam":
            continue
        try:
            data = client.read_personality_info(slot)
        except DeviceRefused:
            continue
        if len(data) <= 1:
            continue
        parsed = parse_trailer(data)
        if not any(int(t) == records.IDENTITY for t, _ in parsed):
            continue
        crc = records.trailer_crc(data)
        name = ""
        if isinstance(client, SupportsSlotPeek):
            info = client.read_slot_info(slot)
            name = str(getattr(info, "name", "") or "")
        try:
            params = records.param_map(parsed)
            if known_crc is not None and crc == known_crc:
                return UnitPersonality(slot=slot, name=name, crc=crc,
                                       descriptor=None, params=params)
            descriptor = records.descriptor_from_trailer(data)
        except ContractError as exc:
            raise UnreadableSlot(slot, str(exc)) from exc
        return UnitPersonality(slot=slot, name=name or descriptor.name, crc=crc,
                               descriptor=descriptor, params=params)
    if stale is not None:
        raise stale
    return None


def _unit_client(topology: Topology, link: LinkSpec, opener=None):
    """An open client for the unit riding a link, or None when the link
    carries none."""
    from nxs.transports import open_client

    if not link.nxs_units:
        return None
    unit = link.nxs_units[0]
    return (opener or open_client)("i2c", bus=topology.i2c_bus,
                                   address=unit.alias_addr)



def refresh_from_unit(topology: Topology, link: LinkSpec, opener=None
                      ) -> Optional[UnitPersonality]:
    """Read the unit behind a link under the bus lock and bring the cache
    up to date: a trailer whose CRC-32 matches the cache is not decoded or
    rewritten; a unit holding no cam personality drops the cache.
    Returns the unit's personality (the cached descriptor when unchanged),
    or None."""
    client = _unit_client(topology, link, opener)
    if client is None:
        return None
    cached = cached_record(topology, link) or {}
    known = (cached.get("personality") or {}).get("crc")
    try:
        with port_state.BusLock():
            found = read_unit_personality(client, known_crc=known)
    finally:
        close = getattr(client, "close", None)
        if close is not None:
            close()
    if found is None:
        forget(topology, link)
        return None
    if found.descriptor is None:
        personality = cached["personality"]
        return dataclasses.replace(
            found, descriptor=cached_descriptor(topology, link),
            name=found.name or str(personality.get("name", "")))
    cache_descriptor(topology, link, found.descriptor, slot=found.slot,
                     crc=found.crc, params=found.params, name=found.name)
    return found


#: The `CURRENT_MODE` a door reports before a run set a mode.
NO_MODE = 0xFF


def door_state(client) -> Tuple[int, int, int]:
    """The unit door's `STATE`, `ERROR` and `CURRENT_MODE` as its header
    serves them: BUSY while a commit runs, READY after one the unit applied
    or before any, FAULT with the errno of one it refused or that failed
    (Interface Description §6.4); `NO_MODE` before a run set a mode."""
    with client.door(client.address) as door:
        head = door.header()
    return int(head["state"]), int(head["error"]), int(head["current_mode"])


def door_text(state: int, error: int, mode: int = NO_MODE) -> str:
    """The door's state in its own words: `door READY, mode 1`, `door READY,
    no mode` before a run set one, `door BUSY`, and `door FAULT, ERROR 19:
    <reason>` with the errno the door holds and the reason the host reads
    it as."""
    from nxs._libnxs_map import MapState
    from nxs.device_errors import COMMIT_ERR_REASON, err_reason

    name = MapState._NAMES.get(int(state), str(int(state)))
    if int(state) == MapState.FAULT:
        return f"door {name}, ERROR {int(error)}: {err_reason(int(error), COMMIT_ERR_REASON)}"
    if int(state) == MapState.BUSY:
        return f"door {name}"
    return f"door {name}, " + ("no mode" if int(mode) == NO_MODE else f"mode {int(mode)}")


def camera_values(client) -> Tuple[Dict[str, int], Dict[str, str]]:
    """The camera personality's rows in effect at the client's unit door, by
    name, and their units: the rows of the owner of the mode row."""
    from nxs._libnxs_map import ParamId

    with client.door(client.address) as door:
        rows = door.rows()
        owners = door.owners()
        mode = next((i for i, row in enumerate(rows) if row.id == ParamId.MODE), None)
        if mode is None:
            return {}, {}
        cells = door.cells(rows)
    mine = [i for i in range(len(rows)) if owners[i] == owners[mode]]
    return ({rows[i].name: int(cells[i]["live"]) for i in mine},
            {rows[i].name: rows[i].unit for i in mine if rows[i].unit})


def unit_status(topology: Topology, link: LinkSpec, opener=None
                ) -> Optional[Dict[str, Any]]:
    """What the link's unit holds and what its unit door says: the cam
    personality's name, slot, sensor and mode values, the door's state and
    errno, and the rows' values in effect by name; `summary` is the one line
    `status` prints. None when the link carries no unit or it holds no
    camera personality."""
    from nxs._libnxs_map import MapState

    found = refresh_from_unit(topology, link, opener)
    if found is None:
        return None
    client = _unit_client(topology, link, opener)
    values: Dict[str, int] = {}
    units: Dict[str, str] = {}
    try:
        with port_state.BusLock():
            state, error, mode = door_state(client)
            if mode != NO_MODE:
                values, units = camera_values(client)
    finally:
        close = getattr(client, "close", None)
        if close is not None:
            close()
    run = {"state": MapState._NAMES.get(state, str(state)), "error": int(error),
           "values": values}
    parts = [f"{found.summary()}, slot {found.slot}, {door_text(state, error, mode)}"]
    parts.extend(_shown(found.compatible, values, units))
    return {"name": found.name, "slot": int(found.slot), "compatible": found.compatible,
            "modes": {str(k): int(v) for k, v in found.modes.items()},
            "run": run, "summary": ", ".join(parts)}


def _shown(compatible: str, values: Dict[str, int], units: Dict[str, str]) -> List[str]:
    """Each row in effect as the camera's description shows it, `fps 30`,
    `gain 10.6 dB`; a row the description does not carry keeps the door's
    number and unit."""
    from nxs import description

    try:
        rows = {row.name: row for row in description.of_type(compatible).rows}
    except Exception:        # noqa: BLE001 (a type this host does not resolve shows the door's numbers)
        rows = {}
    out = []
    for name, value in values.items():
        row = rows.get(name)
        if row is None:
            out.append(f"{name} {value}" + (f" {units[name]}" if units.get(name) else ""))
            continue
        unit = description.unit_of(row)
        out.append(f"{name} {description.display(row, value)}" + (f" {unit}" if unit else ""))

    return out


def unit_personality(topology: Topology, link: LinkSpec, opener=None
                     ) -> Optional[Tuple[int, Descriptor]]:
    """`(slot, descriptor)` of the cam personality the link's unit holds,
    read fresh (the cache follows), or None."""
    found = refresh_from_unit(topology, link, opener)
    if found is None or found.descriptor is None:
        return None
    return found.slot, found.descriptor


def _sensor_personality(topology: Topology, link: LinkSpec, opener=None) -> Optional[str]:
    """The name of the personality the unit measures with, or None when it
    runs none."""
    client = _unit_client(topology, link, opener)
    if client is None:
        return None
    try:
        with port_state.BusLock():
            name = client.read_personality_name()
    finally:
        close = getattr(client, "close", None)
        if close is not None:
            close()
    return str(name) if name and name != "-" else None


def unit_personalities(topology: Topology, link: LinkSpec, opener=None
                       ) -> List[Dict[str, Any]]:
    """The personalities the link's unit holds, every one the same shape: its
    `kind` (`sensor`, the one it measures with; `camera`, the one a camera
    run executes), its `name` and one line of `text`. The camera one also
    carries its slot, compatible, mode values and last run. A kind the unit
    does not hold is not listed."""
    held: List[Dict[str, Any]] = []
    name = _sensor_personality(topology, link, opener)
    if name is not None:
        held.append({"kind": "click", "name": name, "text": f"click personality {name}"})
    camera = unit_status(topology, link, opener)
    if camera is not None:
        held.append({"kind": "cam",
                     **{k: v for k, v in camera.items() if k != "summary"},
                     "text": camera["summary"]})
    return held


#: The address a unit answers at before any alias is programmed: link A's.
NATIVE_UNIT_ADDR = 0x30


def port_link_for_bus(bus: str, address: int) -> Optional[Tuple[Topology, LinkSpec]]:
    """The port and link an I²C-addressed unit rides: the port whose bus
    this is (a manifest port, else a platform camera bus), and the link
    whose declared unit alias is `address`, link A for the native address.
    None off a camera bus."""
    from . import topology as topo_mod

    try:
        ports, _default = topo_mod.discover_ports()
    except Exception:
        return None
    wanted = os.path.realpath(bus)
    for topology in ports.values():
        if os.path.realpath(topology.i2c_bus) != wanted:
            continue
        for link in topology.links:
            if any(unit.alias_addr == address for unit in link.nxs_units):
                return topology, link
        if address == NATIVE_UNIT_ADDR:
            for link in topology.links:
                if link.name == "A":
                    return topology, link
    return None
