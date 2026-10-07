# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0
"""The walk's two renderings: the report a person reads, one line per node,
and the same walk as data for `generate --json`."""
from __future__ import annotations

from typing import Any, Dict, List

from nxs.generate import PortFinding, UnitFinding

def render_report(ports: List[PortFinding], units: List[UnitFinding]) -> str:
    """The tree, one line per node."""
    lines: List[str] = []
    for port in ports:
        lanes = f"{port.lanes}-lane" if port.lanes else "lanes unknown"
        if port.hub_present:
            head = f"HUB {port.hub} @0x{port.hub_addr:02X} ok"
        elif port.links:
            head = "the sensor answers on the port's bus"
        else:
            head = "nothing answers"
        lines.append(f"{port.name} ({port.bus}, {lanes}): {head}")
        if port.silent_hub:
            # The one absence worth a line: a node the wiring names is gone.
            lines.append(f"  hub {port.silent_hub} @0x{port.hub_addr:02X} does not answer "
                         "and is not written (check its power and cabling, then "
                         "generate again)")
        for link in port.links:
            if link.unwalked:
                tail = "; written as declared" if link.sensor is not None else ""
                lines.append(f"  {link.name} (window 0x{link.window:02X}): not walked "
                             f"({link.unwalked}){tail}")
                continue
            if link.locked is False:
                lines.append(f"  {link.name} (window 0x{link.window:02X}): link not locked, "
                             "nothing behind it")
                continue
            if link.sensor is not None:
                parts = [f"SEN {link.sensor} ({link.sensor_note})"]
            elif link.head_answers:
                parts = [f"head answers at {link.head_addr:#04x} ({link.sensor_note}); "
                         f"declare links.{link.name}.camera"]
            else:
                parts = [f"no head ({link.sensor_note})"]
            if link.ser is not None:
                parts.insert(0, f"SER {link.ser} {'ok' if link.ser_present else 'no ACK'}")
            if link.unit_addr is not None:
                unit = f"NXS unit @0x{link.unit_addr:02X}"
                if link.unit_serial:
                    unit += f" serial {link.unit_serial}"
                if link.unit_fw:
                    unit += f" fw {link.unit_fw}"
                parts.append(unit)
            elif port.collided:
                parts.append("NXS not separable")
            else:
                parts.append("no NXS unit")
            where = f" (window 0x{link.window:02X})" if link.window is not None else ""
            lines.append(f"  {link.name}{where}: " + " · ".join(parts))
        if port.shared_unit:
            addr, serial, fw = port.shared_unit
            tail = (f" serial {serial}" if serial else "") + (f" fw {fw}" if fw else "")
            lines.append(f"  NXS unit @0x{addr:02X}{tail} — answers through every "
                         "window: on the port's bus, not behind a link")
        if port.unprogrammed:
            silent, addr = port.unprogrammed
            joined = " and ".join(silent)
            lines.append(
                f"  {joined}: the declared alias answers nowhere and 0x{addr:02X} "
                "does — `on` has not programmed the translation, so the pods "
                "are still merged onto it")
            lines.append(
                f"  run `nxs {port.name} on`, then generate again: until then "
                f"what answers at 0x{addr:02X} may be several pods at once, and "
                "its serial is not one unit's")
        if port.collided:
            addr, names = port.collided
            joined = " and ".join(names)
            aliases = " and ".join(
                f"{name} a unit with alias 0x{addr + i + 1:02X}"
                for i, name in enumerate(names))
            lines.append(
                f"  NXS answers @0x{addr:02X} for {joined} — every pod straps "
                "this address and the links are merged, so the walk cannot say "
                "which answered")
            lines.append(
                f"  give {aliases} in suite.yaml "
                "— the seed writes them where there is no manifest yet, else "
                "add them by hand — then run `on` and generate again")
    for unit in units:
        tail = f" serial {unit.serial}" if unit.serial else ""
        lines.append(f"unit on {unit.route}{tail}")
    if not lines:
        lines.append("no camera ports and no units answered")
    return "\n".join(lines) + "\n"


def walk_data(ports: List[PortFinding], units: List[UnitFinding]) -> Dict[str, Any]:
    """The walk as data (the `generate` surface's tree): every port with the
    nodes that answered on it, then the units on the bare buses."""
    out_ports = []
    for port in ports:
        entry: Dict[str, Any] = {"name": port.name, "bus": port.bus, "links": []}
        if port.lanes:
            entry["csi_lanes"] = int(port.lanes)
        if port.hub_present:
            entry["hub"] = {"compatible": port.hub, "addr": int(port.hub_addr)}
        if port.silent_hub:
            entry["unanswered"] = {"kind": "hub", "compatible": port.silent_hub,
                                   "addr": int(port.hub_addr)}
        for link in port.links:
            node: Dict[str, Any] = {"name": link.name,
                                    "sensor": {"compatible": link.sensor,
                                               "identity": link.sensor_note}}
            if link.unwalked:
                node["unwalked"] = link.unwalked
            if link.locked is not None:
                node["locked"] = bool(link.locked)
            if link.ser is not None:
                node["window"] = int(link.window)
                node["ser"] = {"compatible": link.ser, "present": bool(link.ser_present)}
            if link.unit_addr is not None:
                node["unit"] = {"addr": int(link.unit_addr)}
                if link.unit_serial:
                    node["unit"]["serial"] = link.unit_serial
                if link.unit_fw:
                    node["unit"]["fw"] = link.unit_fw
                if link.unit_click:
                    node["unit"]["personality"] = link.unit_click
            entry["links"].append(node)
        out_ports.append(entry)
    out_units = [{k: v for k, v in (("route", u.route), ("serial", u.serial), ("fw", u.fw),
                                    ("personality", u.personality)) if v} for u in units]
    return {"ports": out_ports, "units": out_units}
