# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""A block's description, read one way for every kind: the identity head,
the ports and the rows its image declares or a unit serves live, each row
with how it reads. The tool prints and parses a row's values from the
description alone, and judges a connection by the ports' types, their
directions and the attributes the two ports share."""

from __future__ import annotations

import dataclasses
from decimal import Decimal, InvalidOperation
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from nxs._generated_constants import NxsDriverImage, NxsMap
from nxs.dsl.fields import Port
from nxs.personality.records_identity import IdentityHead, decode_identity_head
from nxs.personality.records_params import ParamRow
from nxs.personality.records_present import RowPresentation

ImageKind = NxsDriverImage.ImageKind
ModuleKind = NxsMap.ModuleKind
PortDirection = NxsMap.PortDirection
PortType = NxsMap.PortType

#: The declaration's kind word of a block, by the image kind it compiles to.
KIND_WORDS = {ImageKind.CLICK: "click", ImageKind.CAMERA: "camera", ImageKind.HUB: "hub",
              ImageKind.MODULE: "module", ImageKind.HOST: "host"}
#: The declaration's kind word of a live device, by the module kind it serves.
MODULE_KIND_WORDS = {ModuleKind.UNIT: "unit", ModuleKind.POD: "unit",
                     ModuleKind.SMART_CAMERA: "camera", ModuleKind.SMART_MODULE: "module"}
#: The compatible of every NXS unit; its identity and its rows are read live.
UNIT_COMPATIBLE = "aliensense,nxs"
#: The identity fields a declaration pins on a block.
ADDRESS, SERIAL, FIRMWARE = "address", "serial", "firmware"
#: The pins a live device takes: its address, its serial and the firmware it runs.
UNIT_PINS = (ADDRESS, SERIAL, FIRMWARE)
#: Bytes of the `SERIAL` region, the chip's UID96.
_UID_LEN = 12


class DescriptionError(ValueError):
    """A description that does not read, or a text no row takes."""


class UnknownType(LookupError):
    """No host plugin, hub, unit module or personality is of that type."""


@dataclasses.dataclass(frozen=True)
class Description:
    """What a block is and offers: its kind word, its identity head, its
    ports and its rows, each row with how it reads. `pins` are the identity
    fields a declaration asserts on it. `live` is False for a unit described
    from the contract alone, its rows unread."""

    kind: str
    identity: IdentityHead
    ports: Tuple[Port, ...] = ()
    rows: Tuple[ParamRow, ...] = ()
    pins: Tuple[str, ...] = ()
    live: bool = True
    #: The ports a block offers by name beside its own and their attribute rows: a host's buses by
    #: device name (`nxs.host.buses.bus_port`). None for a block of its own ports alone.
    buses: Optional[Callable[[str], Optional[Tuple[Port, Tuple[ParamRow, ...]]]]] = \
        dataclasses.field(default=None, compare=False)

    @property
    def type(self) -> str:
        """The type a declaration names: the block's compatible."""
        return self.identity.compatible

    @property
    def name(self) -> str:
        return self.identity.name

    def row(self, name: str) -> ParamRow:
        """The row `name`; KeyError for a name no row carries."""
        for row in self.rows:
            if row.name == name:
                return row
        raise KeyError(name)

    def port(self, name: str) -> Optional[Port]:
        return next((p for p in self.ports if p.name == name), None)

    def attributes(self, port: str) -> Dict[str, ParamRow]:
        """The rows that state a port's attributes, by attribute: a row named
        `<port>_<attribute>`, the port's dots as underscores and in lower
        case (`csi.A` states its lanes in `csi_a_lanes`)."""
        prefix = port.lower().replace(".", "_") + "_"
        return {row.name[len(prefix):]: row for row in self.rows if row.name.startswith(prefix)}

    def with_buses(self, names: Sequence[str]) -> "Description":
        """The description with each port of `names` that `buses` offers and
        the block lacks, and the rows that state its attributes; itself for
        a block that offers no bus by name."""
        if self.buses is None:
            return self
        ports, rows = list(self.ports), list(self.rows)
        for name in dict.fromkeys(names):
            found = None if self.port(name) is not None else self.buses(name)
            if found is not None:
                ports.append(found[0])
                rows.extend(r for r in found[1] if all(r.name != have.name for have in rows))
        if len(ports) == len(self.ports):
            return self
        return dataclasses.replace(self, ports=tuple(ports), rows=tuple(rows))


# ── reading ──────────────────────────────────────────────────────────────

def from_records(kind: str, records: Sequence[Tuple[int, bytes]],
                 pins: Optional[Tuple[str, ...]] = None) -> Description:
    """The description an image's trailer records declare: the IDENTITY
    head, the PORTS rows, and the PARAMS rows read as the PRESENTATION
    record says. A block with an address takes the `address` pin."""
    from nxs.personality import records as records_mod
    from nxs.personality.records_fields import IDENTITY

    by_type = {int(t): bytes(p) for t, p in records}
    if IDENTITY not in by_type:
        raise DescriptionError("the trailer carries no IDENTITY record")
    identity = decode_identity_head(by_type[IDENTITY])
    rows = records_mod.param_rows(records)
    present = records_mod.presentation(records)
    if present and len(present) != len(rows):
        raise DescriptionError(f"{len(present)} presentation entries for {len(rows)} rows")
    shown = tuple(dataclasses.replace(row, present=present[i] if present else _own(row))
                  for i, row in enumerate(rows))
    if pins is None:
        pins = (ADDRESS,) if identity.address else ()
    return Description(kind=kind, identity=identity, ports=tuple(records_mod.port_rows(records)),
                       rows=shown, pins=pins)


def from_compiled(compiled) -> Description:
    """The description of a compiled image (`CompiledDriver`)."""
    kind = KIND_WORDS.get(int(compiled.kind))
    if kind is None:
        raise DescriptionError(f"image kind {compiled.kind} describes no block")
    return from_records(kind, compiled.trailer)


def from_image(data: bytes) -> Description:
    """The description of a serialized image."""
    from nxs.image import deserialize

    return from_compiled(deserialize(bytes(data)))


def from_door(door, info: Optional[Callable[[int], bytes]] = None) -> Description:
    """A live device's description through its map door: the kind from
    `MODULE_KIND`, the ports the contract gives that kind, the identity from
    the header, the `SERIAL` region and the door's address, and the rows the
    door serves. `info(slot)` returns a stored slot's trailer, which says how
    that slot's rows read; without it, and for the device's own rows, a row
    reads in its own unit."""
    from nxs.image import parse_trailer
    from nxs.personality import records as records_mod

    header = door.header()
    kind = MODULE_KIND_WORDS.get(int(header["module_kind"]))
    if kind is None:
        raise DescriptionError(f"module kind {header['module_kind']} is none this tool knows")
    major, minor = header["fw"]
    identity = IdentityHead(address=int(door.address()),
                            fw=(int(major), int(minor), int(door.u8(NxsMap.MapReg.FW_VERSION_PATCH))),
                            serial=bytes(door.read(NxsMap.MapReg.SERIAL, _UID_LEN)),
                            compatible=UNIT_COMPATIBLE)
    rows = door.rows()
    owners = door.owners()
    shown: List[ParamRow] = []
    read: Dict[int, Dict[str, RowPresentation]] = {}
    for row, owner in zip(rows, owners):
        present = None
        if owner != NxsMap.OWNER_DEVICE and info is not None:
            if owner not in read:
                trailer = parse_trailer(bytes(info(owner)))
                read[owner] = {r.name: p for r, p in zip(records_mod.param_rows(trailer),
                                                         records_mod.presentation(trailer))}
            present = read[owner].get(row.name)
        shown.append(dataclasses.replace(row, present=present or _own(row)))
    return Description(kind=kind, identity=identity, ports=contract_ports(int(header["module_kind"])),
                       rows=tuple(shown), pins=UNIT_PINS)


def contract_ports(module_kind: int) -> Tuple[Port, ...]:
    """The ports the contract gives a device of a module kind."""
    return tuple(Port(name, getattr(PortType, type_name), getattr(PortDirection, direction))
                 for name, type_name, direction in ModuleKind.PORTS[module_kind])


def unit_from_contract(module_kind: int = ModuleKind.POD) -> Description:
    """A unit as the contract alone describes it: the ports of its module
    kind, a pod's by default, the pins a live device takes, no rows until a
    unit answers."""
    return Description(kind=MODULE_KIND_WORDS[module_kind],
                       identity=IdentityHead(compatible=UNIT_COMPATIBLE),
                       ports=contract_ports(module_kind), pins=UNIT_PINS, live=False)


def _own(row: ParamRow) -> RowPresentation:
    return RowPresentation(unit=row.unit, exponent=0)


def present(row: ParamRow) -> RowPresentation:
    """How a row reads: its presentation, else its own unit at exponent 0."""
    return row.present or _own(row)


# ── values ───────────────────────────────────────────────────────────────

def display(row: ParamRow, raw: int) -> str:
    """The text a served value shows as: its name where the row names its
    values, else the value times ten to the row's exponent."""
    shown = present(row)
    if shown.labels is not None:
        index = _index(row, int(raw))
        if index is not None and index < len(shown.labels):
            return shown.labels[index]
    return _number(int(raw), shown.exponent)


def parse(row: ParamRow, text: Any) -> int:
    """The served value a text names: one of the row's names, else a number
    in the row's display form. DescriptionError for a text that names no
    value, a number that is not finite, or one finer than the row's
    resolution."""
    shown = present(row)
    if isinstance(text, bool) and shown.labels is not None:
        # YAML 1.1 reads an unquoted on, off, yes or no as a boolean.
        picks = [pair[0] if text else pair[1] for pair in _BOOLEAN_WORDS]
        text = next((word for word in picks if word in shown.labels), text)
    word = str(text).strip()
    if shown.labels is not None and word in shown.labels:
        return _value_at(row, shown.labels.index(word))
    try:
        number = Decimal(word)
    except InvalidOperation:
        raise DescriptionError(f"{row.name}: {word!r} names no value; it takes "
                               f"{offered(row)}") from None
    if not number.is_finite():
        raise DescriptionError(f"{row.name}: {word} is not a finite number; it takes "
                               f"{offered(row)}")
    scaled = number.scaleb(-shown.exponent)
    if scaled != scaled.to_integral_value():
        raise DescriptionError(f"{row.name}: {word} is finer than its resolution of "
                               f"{_number(1, shown.exponent)} {shown.unit}".rstrip())
    return int(scaled)


#: The value names YAML 1.1 reads as a boolean, true then false.
_BOOLEAN_WORDS = (("on", "off"), ("yes", "no"), ("true", "false"))


def unit_of(row: ParamRow) -> str:
    """The unit a row's values show in."""
    return present(row).unit


def number(row: ParamRow, raw: int) -> str:
    """A served value in the row's display form, its names set aside."""
    return _number(int(raw), present(row).exponent)


def named_values(row: ParamRow, span: Optional[Tuple[int, int, int]] = None) -> Optional[Tuple[int, ...]]:
    """Every value of a row that names its values, a range's from its min
    by its step (`span` narrows it); None for a row without names."""
    if present(row).labels is None:
        return None
    if not row.is_range:
        return tuple(row.values)
    lo, hi, step = span or (row.values[0], row.values[1], row.values[2])
    return tuple(range(lo, hi + 1, max(1, step)))


def offered(row: ParamRow, values: Optional[Sequence[int]] = None,
            span: Optional[Tuple[int, int, int]] = None) -> str:
    """The values a row takes, as a reader sees them: `values` or `span`
    where given, else the row's own; a row that names its values lists
    the names."""
    unit = unit_of(row)
    tail = f" {unit}" if unit else ""
    if values is None:
        values = named_values(row, span)
    if values is None and span is None:
        if row.is_range:
            span = (row.values[0], row.values[1], row.values[2])
        else:
            values = row.values
    if values is not None:
        steps = {b - a for a, b in zip(values, values[1:])}
        if present(row).labels or len(values) < 5 or len(steps) != 1:
            return ", ".join(display(row, v) for v in values) + ("" if present(row).labels else tail)
        # An even run of numbers reads as its ends.
        span = (values[0], values[-1], steps.pop())
    lo, hi, step = span
    shown = _number(step, present(row).exponent)
    by = f" by {shown}" if step > 1 and shown != "1" else ""
    return f"{display(row, lo)}..{display(row, hi)}{by}{tail}"


def _number(raw: int, exponent: int) -> str:
    if exponent == 0:
        return str(raw)
    return format(Decimal(raw).scaleb(exponent).normalize(), "f")


def _index(row: ParamRow, raw: int) -> Optional[int]:
    if row.is_range:
        lo, hi, step = row.values
        if lo <= raw <= hi and (raw - lo) % max(1, step) == 0:
            return (raw - lo) // max(1, step)
        return None
    return row.values.index(raw) if raw in row.values else None


def _value_at(row: ParamRow, index: int) -> int:
    if row.is_range:
        lo, _hi, step = row.values
        return lo + index * max(1, step)
    return row.values[index]


# ── connections ──────────────────────────────────────────────────────────

def connect(a: Description, a_port: str, b: Description, b_port: str,
            a_values: Optional[Mapping[str, int]] = None,
            b_values: Optional[Mapping[str, int]] = None,
            a_name: str = "", b_name: str = "") -> List[str]:
    """Why `a_port` of `a` cannot feed `b_port` of `b`, [] when it can: the
    ports exist, the source is no input and the target no output, the
    target's type takes the source's (`PortType.FEEDS`), and every attribute
    both ports state agrees, each block's value from `a_values` and
    `b_values` by row name, else the row's default. `a_name` and `b_name`
    are the nodes' names, which a disagreeing attribute's rows carry."""
    source, target = a.port(a_port), b.port(b_port)
    findings = []
    if source is None:
        findings.append(f"{a.type} has no port {a_port}: its ports are "
                        f"{', '.join(p.name for p in a.ports) or 'none'}")
    if target is None:
        findings.append(f"{b.type} has no port {b_port}: its ports are "
                        f"{', '.join(p.name for p in b.ports) or 'none'}")
    if findings:
        return findings
    if source.direction == PortDirection.INPUT:
        findings.append(f"{a_port} is an input: it takes a source and feeds nothing")
    if target.direction == PortDirection.OUTPUT:
        findings.append(f"{b_port} is an output: it feeds and takes no source")
    if target.type != source.type and target.type not in PortType.FEEDS.get(source.type, ()):
        findings.append(f"{a_port} is {port_type_word(source.type)} and {b_port} takes "
                        f"{port_type_word(target.type)}")
    b_attributes = b.attributes(b_port)
    for attribute, row_a in a.attributes(a_port).items():
        row_b = b_attributes.get(attribute)
        if row_b is None:
            continue
        value_a = int((a_values or {}).get(row_a.name, row_a.default))
        value_b = int((b_values or {}).get(row_b.name, row_b.default))
        if value_a != value_b:
            where_a = f"{a_name}.{row_a.name}" if a_name else f"{a_port} {row_a.name}"
            where_b = f"{b_name}.{row_b.name}" if b_name else f"{b_port} {row_b.name}"
            findings.append(f"{attribute}: {where_a} {display(row_a, value_a)} into "
                            f"{where_b} {display(row_b, value_b)}")
    return findings


def port_type_word(port_type: int) -> str:
    """A port type as a declaration spells it (`cyphal-can`)."""
    return PortType._NAMES.get(int(port_type), str(port_type)).lower().replace("_", "-")


def direction_word(direction: int) -> str:
    return PortDirection._NAMES.get(int(direction), str(direction)).lower()


# ── the declaration's schema ─────────────────────────────────────────────

def json_schema(description: Description, options: Mapping[str, Any]) -> Dict[str, Any]:
    """The JSON Schema of a node's `parameters` for this block: one property
    per row `options` admits (an `Option` with `values`, `span` and
    `applicable`), in display form, and one per identity pin. A range
    states `multipleOf` only for a whole step over a grid from 0: a
    validator judges a fractional one in binary floating point and refuses
    lawful values (10.6 on a 0.1 grid), and the judge holds every value to
    the row's grid."""
    properties: Dict[str, Any] = {}
    for row in description.rows:
        option = options.get(row.name)
        if option is None or not option.applicable:
            continue
        unit = unit_of(row)
        entry: Dict[str, Any] = {"description": f"{row.name} in {unit}" if unit else row.name}
        values = option.values if option.values is not None else named_values(row, option.span)
        if values is None and not row.is_range and option.span is None:
            values = row.values
        span = option.span if option.span is not None else (
            tuple(row.values) if row.is_range and values is None else None)
        if values is not None:
            entry["enum"] = [_json_value(row, v) for v in values]
        elif span is not None:
            lo, hi, step = span
            exponent = present(row).exponent
            entry["type"] = "integer" if exponent >= 0 else "number"
            entry["minimum"], entry["maximum"] = _json_value(row, lo), _json_value(row, hi)
            whole = _whole(row, step) if step > 1 and lo % step == 0 else None
            if whole is not None:
                entry["multipleOf"] = whole
        properties[row.name] = entry
    for pin in description.pins:
        properties[pin] = {"type": ["string", "integer"]} if pin == ADDRESS else {"type": "string"}
    return {"type": "object", "additionalProperties": False, "properties": properties}


def _whole(row: ParamRow, raw: int) -> Optional[int]:
    """A served value in the row's display form where it is a whole number."""
    shown = Decimal(int(raw)).scaleb(present(row).exponent)
    return int(shown) if shown == shown.to_integral_value() else None


def _json_value(row: ParamRow, raw: int) -> Any:
    shown = present(row)
    if shown.labels is not None:
        return display(row, raw)
    if shown.exponent == 0:
        return int(raw)
    return float(Decimal(int(raw)).scaleb(shown.exponent))


# ── by type ──────────────────────────────────────────────────────────────

#: The unit modules a declaration names by type, by the module kind each is:
#: `nxs` a unit or a pod, `nxs-module` a compute module running the NXS map.
UNIT_TYPES = {"nxs": ModuleKind.POD, "nxs-module": ModuleKind.SMART_MODULE}
#: The descriptions read so far, by type; a host's is its plugin's to keep.
_DESCRIBED: Dict[str, Description] = {}


def of_type(node_type: str) -> Description:
    """The description of a node type: the host plugin of that name, a
    unit module (`UNIT_TYPES`), a hub by its deserializer's compatible, a
    cam personality by its compatible or name, a click personality by name,
    in that order. A type is read once until `reset_cache`. UnknownType
    when nothing installed is of that type."""
    found = _host(node_type)
    if found is not None:
        return found
    if node_type in _DESCRIBED:
        return _DESCRIBED[node_type]
    unread = None
    for finder in (_unit, _hub, _camera, _click):
        try:
            found = finder(node_type)
        except _HubsUnread as exc:
            # The hubs the host holds do not read: the type is refused with
            # their fact unless another finder describes it.
            unread = exc
            continue
        if found is not None:
            _DESCRIBED[node_type] = found
            return found
    if unread is not None:
        raise DescriptionError(str(unread))
    raise UnknownType(node_type)


def reset_cache() -> None:
    """Forget the descriptions read: the store or the search paths moved."""
    _DESCRIBED.clear()


def running_row(client, name: str) -> Optional[ParamRow]:
    """The row `name` of the click personality a unit runs, from that
    personality's description; None where no installed personality is the
    one the unit runs, or it has no such row."""
    from nxs.suite.reconcile import click_personality_for

    try:
        module = click_personality_for(client.read_personality_name())
        return of_type(module).row(name) if module else None
    except (KeyError, UnknownType, DescriptionError):
        return None


def _host(node_type: str) -> Optional[Description]:
    """A host plugin by its name: the HOST image the plugin compiles."""
    from nxs import host as host_layer

    return host_layer.plugin_description(node_type)


def _unit(node_type: str) -> Optional[Description]:
    kind = UNIT_TYPES.get(node_type)
    return unit_from_contract(kind) if kind is not None else None


class _HubsUnread(Exception):
    """The hubs the host holds do not read, a set search path among them."""


def _hub(node_type: str) -> Optional[Description]:
    """A hub by its deserializer's compatible: its program's image, with
    its flows loaded, which register the hub's options with the hook
    whether the image is the store's or compiled here."""
    from nxs.cam import compose, hubs

    try:
        found = hubs.discover()
    except hubs.HubError as exc:
        raise _HubsUnread(str(exc)) from exc
    for hub in found:
        des = next((c for c in hub.chips if hub.descriptor(c).role == "DES"), None)
        if des is None or hub.descriptor(des).compatible != node_type:
            continue
        try:
            hub.flows()
        except hubs.HubError as exc:
            raise DescriptionError(f"{node_type}: its flows do not load: {exc}") from exc
        try:
            return from_image(compose.hub_program(hub))
        except Exception as exc:        # noqa: BLE001 (a program this host cannot build)
            raise DescriptionError(f"{node_type}: its program does not build: {exc}") from exc
    return None


def _camera(node_type: str) -> Optional[Description]:
    """A cam personality by its compatible or name: the image the host holds
    or compiles. Its laws bind here, so the options hook narrows its rows."""
    from nxs import personality_cli
    from nxs.cam import cam_personalities, laws

    try:
        descriptor = cam_personalities.find(node_type)
    except Exception:        # noqa: BLE001 (no cam personality of that name)
        return None
    try:
        laws.laws(descriptor)
        source = _camera_source(descriptor)
        if source is None:
            return from_facts(descriptor)
        if source.kind == personality_cli.KIND_IMAGE:
            return _with_mode_row(from_image(source.image), descriptor)
        compiled, _image = personality_cli.compile_source(source, {})
    except Exception as exc:        # noqa: BLE001 (a personality this host cannot build)
        raise DescriptionError(f"{node_type}: its personality does not build: {exc}") from exc
    return _with_mode_row(from_compiled(compiled), descriptor)


def _with_mode_row(described: Description, descriptor) -> Description:
    """A camera whose image declares no mode row, a connector head whose
    program takes its mode privately, with the mode row its facts give, so
    a declaration names the mode its program runs."""
    row = _mode_row(descriptor)
    if row is None or any(r.name == row.name for r in described.rows):
        return described
    return dataclasses.replace(described, rows=(row, *described.rows))


def _mode_row(descriptor) -> Optional[ParamRow]:
    """The mode row a cam personality's facts give: the modes its program
    offers, by name, the facts' default first chosen; None for facts that
    list none."""
    modes = tuple(descriptor.program_modes())
    if not modes:
        return None
    default = str(descriptor.raw("default_mode") or modes[0])
    return ParamRow(name="mode", unit="", id=NxsMap.ParamId.MODE, kind=NxsDriverImage.ParamKind.RUN,
                    values=tuple(range(len(modes))),
                    default=modes.index(default) if default in modes else 0, is_range=False,
                    present=RowPresentation(unit="", exponent=0, labels=modes))


def _camera_source(descriptor):
    """What a cam personality is described from: the image the host's store
    holds where it is a camera's of the personality's compatible, else the
    behaviour class beside its facts; None for facts that ship laws alone.
    A stored image of another kind or sensor is `on`'s to refuse."""
    from nxs import personality_cli

    try:
        source = personality_cli.resolve(descriptor.name)
    except personality_cli.ResolveError:
        source = None
    if source is not None and source.kind == personality_cli.KIND_IMAGE:
        try:
            stored = from_image(source.image)
        except Exception:        # noqa: BLE001 (an image cut short or malformed)
            stored = None
        if stored is None or stored.kind != "camera" or stored.type != descriptor.compatible:
            source = None
    if source is None:
        if _program_class(descriptor) is None:
            return None
        source = personality_cli._registry_source(descriptor.name)
    return source


def _program_class(descriptor) -> Optional[type]:
    """The behaviour class beside a cam personality's facts, None for facts
    that ship laws alone."""
    from nxs import personality_cli
    from nxs.cam import cam_personalities
    from nxs.compiler import CamPersonality

    try:
        source = cam_personalities.registry().source(descriptor.name)
    except Exception:        # noqa: BLE001 (a registry that does not read)
        return None
    if source is None:
        return None
    cls = personality_cli._class_in(str(source), descriptor.name)
    return cls if cls is not None and issubclass(cls, CamPersonality) else None


def from_facts(descriptor) -> Description:
    """A cam personality that ships facts and laws and no program, as its
    facts describe it: a camera with its output `out`, its address, and its
    mode row naming the modes its facts list."""
    meta = descriptor.raw("meta") or {}
    try:
        address = int(str(meta.get("i2c_addr", 0)), 0)
    except ValueError:
        address = 0
    row = _mode_row(descriptor)
    rows: Tuple[ParamRow, ...] = (row,) if row is not None else ()
    identity = IdentityHead(address=address, compatible=descriptor.compatible, product=descriptor.name)
    return Description(kind="camera", identity=identity,
                       ports=(Port("out", PortType.HEAD, PortDirection.OUTPUT),), rows=rows,
                       pins=(ADDRESS,) if address else ())


def _click(node_type: str) -> Optional[Description]:
    """A click personality by name: its compiled image. A file of that name
    that does not load is a DescriptionError naming why."""
    from nxs.suite import personality_file
    from nxs.suite.reconcile import load_click_personality

    name = node_type.replace("-", "_")
    try:
        found = personality_file(name, "py")
    except Exception:        # noqa: BLE001 (a name no personality file carries)
        found = None
    if found is None:
        return None
    try:
        cls = load_click_personality(name)
    except Exception as exc:        # noqa: BLE001 (a file that does not import or defines two)
        raise DescriptionError(f"{node_type}: {exc}") from exc
    try:
        return from_compiled(cls().compile({}))
    except Exception as exc:        # noqa: BLE001 (a personality that does not compile)
        raise DescriptionError(f"{node_type}: its personality does not compile: {exc}") from exc


__all__ = ["ADDRESS", "Description", "DescriptionError", "FIRMWARE", "KIND_WORDS",
           "MODULE_KIND_WORDS", "SERIAL", "UNIT_COMPATIBLE", "UNIT_PINS", "UNIT_TYPES", "UnknownType",
           "connect", "contract_ports", "direction_word", "display", "from_compiled", "from_door",
           "from_facts", "from_image", "from_records", "json_schema", "named_values", "number",
           "of_type", "offered", "parse", "port_type_word", "present", "reset_cache", "running_row",
           "unit_from_contract", "unit_of"]
