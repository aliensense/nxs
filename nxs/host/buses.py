# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The buses a host reaches a unit on, as ports named for their device: an
I²C adapter `i2c-<N>` at `/dev/i2c-<N>`, a CAN interface `can<N>`, a serial
device `tty<name>` at `/dev/tty<name>` or a `/dev/serial/by-id/` name. A
host offers every such port by its name, whether the device is plugged in
or not: whether it answers is `nxs status`'s to say. A serial port states
its rate in a row named for it, `<port>_baud`."""

from __future__ import annotations

import re
from typing import Dict, Optional, Tuple

from nxs._generated_constants import NxsDriverImage, NxsMap
from nxs.dsl.fields import Port
from nxs.personality.records_params import ParamRow
from nxs.personality.records_present import RowPresentation

PortDirection = NxsMap.PortDirection
PortType = NxsMap.PortType

#: The rates a Cyphal/serial link runs at, in baud; the transport's default first.
BAUDS = (460800, 9600, 19200, 38400, 57600, 115200, 230400, 921600)
#: The serial device names udev gives under `/dev/serial/by-id/`.
BY_ID = "/dev/serial/by-id"
_I2C = re.compile(r"^i2c-\d+$")
_CAN = re.compile(r"^v?can\d+$")
_TTY = re.compile(r"^tty[A-Za-z0-9_]+$")
_BY_ID = re.compile(r"^(usb|pci|platform)-[A-Za-z0-9_.:+-]+$")


def bus_port(name: str) -> Optional[Tuple[Port, Tuple[ParamRow, ...]]]:
    """The port a host offers under `name` and the rows that state its
    attributes; None for a name that names no bus device."""
    if _I2C.match(name):
        return Port(name, PortType.I2C, PortDirection.OUTPUT), ()
    if _CAN.match(name):
        return Port(name, PortType.CYPHAL_CAN, PortDirection.BUS), ()
    if _TTY.match(name) or _BY_ID.match(name):
        return Port(name, PortType.CYPHAL_SERIAL, PortDirection.OUTPUT), (baud_row(name),)
    return None


#: The names `bus_port` offers, as a reader writes them: the form, the port's
#: type and direction, and the attributes its rows state.
bus_port.forms = (("i2c-<N>", PortType.I2C, PortDirection.OUTPUT, ()),
                  ("can<N>", PortType.CYPHAL_CAN, PortDirection.BUS, ()),
                  ("tty<name>", PortType.CYPHAL_SERIAL, PortDirection.OUTPUT, ("baud",)))


def baud_row(port: str) -> ParamRow:
    """The rate row of a serial port, named for it as an attribute row is."""
    return ParamRow(name=f"{port.lower().replace('.', '_')}_baud", unit="Bd",
                    id=NxsMap.ParamId.CUSTOM, kind=NxsDriverImage.ParamKind.APPLY,
                    values=tuple(sorted(BAUDS)), default=BAUDS[0], is_range=False,
                    present=RowPresentation(unit="Bd", exponent=0))


def route(name: str, rows: Optional[Dict[str, int]] = None) -> Dict[str, object]:
    """The route a bus port names, as `schema.parse_route` takes it, less
    the unit's own address or node id; `rows` holds the host's declared
    values by row name. KeyError for a name that names no bus device."""
    if _I2C.match(name):
        return {"transport": "i2c", "bus": f"/dev/{name}"}
    if _CAN.match(name):
        return {"transport": "cyphal-can", "iface": name}
    if _TTY.match(name) or _BY_ID.match(name):
        out: Dict[str, object] = {"transport": "cyphal-serial",
                                  "port": f"/dev/{name}" if _TTY.match(name) else f"{BY_ID}/{name}"}
        baud = (rows or {}).get(baud_row(name).name)
        if baud is not None and int(baud) != BAUDS[0]:
            out["baud"] = int(baud)
        return out
    raise KeyError(name)


def port_name(transport: str, device: str) -> Optional[str]:
    """The port a host offers for a route's device: the name that `route`
    takes back to the device; None for a device no such name reaches."""
    if transport == "cyphal-can":
        return device if _CAN.match(device) else None
    if transport == "i2c":
        found = re.match(r"^/dev/(i2c-\d+)$", device)
        return found.group(1) if found else None
    if device.startswith(BY_ID + "/"):
        name = device[len(BY_ID) + 1:]
        return name if _BY_ID.match(name) else None
    found = re.match(r"^/dev/(tty[A-Za-z0-9_]+)$", device)
    return found.group(1) if found else None


__all__ = ["BAUDS", "BY_ID", "baud_row", "bus_port", "port_name", "route"]
