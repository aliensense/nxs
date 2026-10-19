"""The PORTS record: the typed ports a personality's device offers, one
`PORT_ROW_SIZE` row each, in declaration order. A graph connects a port only
to a port of its type. The row layout is `PortField` in
constants/nxs_map.yaml."""

from __future__ import annotations

import struct
from typing import List, Sequence

from nxs._generated_constants import NxsMap
from nxs.dsl.fields import Port, PortDirection, PortType
from nxs.personality.records_fields import RecordError

#: One row: name, type, direction, a reserved byte, flags, then the reserved tail.
ROW = struct.Struct("<16sBBxB12x")
assert ROW.size == NxsMap.PORT_ROW_SIZE
#: Bytes of the name field; a name keeps its last byte NUL.
_NAME_LEN = NxsMap.PortField.TYPE - NxsMap.PortField.NAME


def encode_ports(ports: Sequence[Port], what: str) -> bytes:
    """The PORTS payload: the ports in declaration order, `MAX_PORT_ROWS` at
    most, each name once."""
    if len(ports) > NxsMap.MAX_PORT_ROWS:
        raise RecordError(f"{what}: {len(ports)} ports, a personality declares "
                          f"{NxsMap.MAX_PORT_ROWS} at most")
    names = [port.name for port in ports]
    twice = sorted({name for name in names if names.count(name) > 1})
    if twice:
        raise RecordError(f"{what}: {', '.join(twice)} declared twice, and a port's "
                          f"name is unique in its personality")
    return b"".join(_encode_row(port, what) for port in ports)


def decode_ports(payload: bytes) -> List[Port]:
    """The PORTS payload back to its ports. A type or direction this tool
    does not know comes back as its number."""
    if len(payload) % ROW.size:
        raise RecordError(f"PORTS record of {len(payload)} B is not whole "
                          f"{ROW.size}-byte rows")
    ports = []
    for pos in range(0, len(payload), ROW.size):
        name, port_type, direction, _flags = ROW.unpack_from(payload, pos)
        label = name.rstrip(b"\0")
        try:
            text = label.decode("ascii")
        except UnicodeDecodeError:
            raise RecordError(f"PORTS row {label!r} carries a byte past ASCII in its "
                              f"name") from None
        ports.append(Port(text, port_type, direction))
    return ports


def _encode_row(port: Port, what: str) -> bytes:
    where = f"{what} {port.name}"
    try:
        name = str(port.name).encode("ascii")
    except UnicodeEncodeError:
        raise RecordError(f"{where}: a port's name is ASCII") from None
    if not name or len(name) >= _NAME_LEN:
        raise RecordError(f"{where}: a port's name takes 1 to {_NAME_LEN - 1} characters")
    if port.type not in PortType._NAMES:
        raise RecordError(f"{where}: type {port.type!r} is not a PortType")
    if port.direction not in PortDirection._NAMES:
        raise RecordError(f"{where}: direction {port.direction!r} is not a PortDirection")
    return ROW.pack(name, port.type, port.direction, 0)
