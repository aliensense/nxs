# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""One row committed on a link's camera door at a value, in the row's unit,
as a host's camera driver commits it (Interface Description §6.4). The value
is judged against the row the door serves before the commit, and the line
names the value in effect after it. A pod's door takes the commit through
the library's set on the port's graph image (`door_set`); a smart camera's
door, which no pod stands behind, through the host's door binding
(`commit`)."""

from __future__ import annotations

import errno
from typing import Dict, Optional, Tuple

from nxs import term
from nxs.cam.contracts import InfeasibleConfig, Topology


def within(row, cell: Dict[str, int], value: int) -> Tuple[int, Optional[str]]:
    """The value a commit of `row` stages for `value`, and the bound it was
    clamped to (`minimum` or `maximum`), None when it stages as asked. A
    range row takes its range in effect, `cell`'s MIN and MAX: a value past
    it stages at the bound, as the device clamps it, so a negative never
    reaches an unsigned row's cell, whose two's complement the device would
    read near its maximum. An enum row takes a value on its list alone.

    Raises:
        InfeasibleConfig: An enum value off the row's list, naming the list.
    """
    value = int(value)
    if not row.is_range:
        if value not in row.values:
            raise InfeasibleConfig(f"{row.name} takes {', '.join(str(v) for v in row.values)}, not {value}",
                                   alternatives=[f"{row.name} {v}" for v in row.values])
        return value, None
    if value < int(cell["min"]):
        return int(cell["min"]), "minimum"
    if value > int(cell["max"]):
        return int(cell["max"]), "maximum"
    return value, None


def _judged(door, row, value: int):
    """The rows the door serves, the index of `row` among them, the value a
    commit stages for `value` and the bound it was clamped to."""
    from nxs._libnxs_map import find_row

    rows = door.rows()
    index = find_row(rows, row)
    if index is None:
        raise LookupError(f"no row {row} at the camera door @{door.address:#04x}")
    staged, bound = within(rows[index], door.cells(rows[:index + 1])[index], value)
    return rows, index, staged, bound


def _failed(topology: Topology, link: str, row, value: int, exc: Exception) -> str:
    """The line of a door that does not answer, that this tool does not
    speak, or that serves no such row; an INERT door's names the port's
    bring-up."""
    from nxs._libnxs_map import MapInert, ParamId
    from nxs.cam.select import _port_name

    label = row if isinstance(row, str) else ParamId._NAMES.get(int(row), str(row)).lower()
    bring_up = (f"; bring the port up first: nxs {_port_name(topology)} on"
                if isinstance(exc, MapInert) else "")
    return f"link {link}: {label} {value}: {exc}{bring_up}"


def _said(link: str, served, live: int, value: int, bound: Optional[str], addr: int) -> None:
    """The value in effect, saying where it differs from the one asked."""
    unit = f" {served.unit}" if served.unit else ""
    asked = ""
    if bound is not None:
        asked = f" ({value}{unit} asked, clamped to the row's {bound})"
    elif live != int(value):
        asked = f" ({value}{unit} asked)"
    term.info(f"link {link}: {served.name} {live}{unit} on its camera door at {addr:#04x}{asked}")


def door_set(topology: Topology, composition, link: str, row, value: int) -> Optional[str]:
    """Commit `row` (a standard `ParamId`, or a custom row's name) at `value`
    on the camera door of `link`'s pod by the library's set on the port's
    graph image (`nxs_graph_set`, an APPLY commit), and print the value in
    effect. Returns the failure line: a door that does not answer, that
    this tool does not speak or that serves no such row, a value it does
    not take, or a set the library refused (an INERT door's naming the
    port's bring-up); None after the commit."""
    from nxs._libnxs_map import open_door
    from nxs.cam import graph
    from nxs.cam.select import _port_name

    addr = next(r.pod_map for r in composition.links if r.name == link)
    try:
        with open_door(topology.i2c_bus, addr) as door:
            rows, index, staged, bound = _judged(door, row, value)
        served = rows[index]
        report = graph.set_param(topology, composition, link, index, staged)
        if not report.ok:
            unit = f" {served.unit}" if served.unit else ""
            bring_up = (f"; bring the port up first: nxs {_port_name(topology)} on"
                        if report.rc == -errno.EAGAIN else "")
            return f"link {link}: {served.name} {value}{unit}: {report.reason()}{bring_up}"
        with open_door(topology.i2c_bus, addr) as door:
            live = door.cells(rows[:index + 1])[index]["live"]
    except InfeasibleConfig as exc:
        return f"link {link}: {exc}"
    except (LookupError, OSError, RuntimeError) as exc:
        return _failed(topology, link, row, value, exc)
    _said(link, served, live, value, bound, addr)
    return None

