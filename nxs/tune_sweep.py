# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The panel's sweep of the platform's camera buses: the hub walk of `nxs
generate`, link by link through the hub's windows, so a pod is its link's
and two pods strapped alike on two links are two; the port's own bus where
no hub answers; then every bus for the units it reaches behind no link."""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple


def sweep(buses: Dict[str, str], cfg=None) -> Dict[str, Optional[dict]]:
    """{bus: finding} over `buses` ({port: bus}). A finding names the `port`,
    the `hub` that answered (its compatible, None without one), its `links`
    ({letter: {sensor, head, head_addr, pod, walked}}, a pod being {addr,
    serial, fw, click, cam}), the `units` the bus reaches behind no link
    ([(addr, serial, click)]) and the letters whose pods `collided`. None
    for a bus that could not be read."""
    out: Dict[str, Optional[dict]] = {bus: None for bus in buses.values()}
    try:
        ports, units = _walk(sorted(set(buses.values())), _declared_addresses(cfg))
    except Exception:        # noqa: BLE001 (no host layer, a tree that does not parse: nothing answered)
        return out
    return assemble(ports, units, out)


def assemble(ports, units, out: Optional[Dict[str, Optional[dict]]] = None) -> Dict[str, Optional[dict]]:
    """The sweep's shape from a walk's ports and bare units (`nxs generate`
    and the panel share it); `out` seeds the buses asked, None each."""
    out = dict(out or {})
    for port in ports:
        if not port.unreadable:
            out[port.bus] = _finding(port)
    for unit in units:
        bus = getattr(unit.link, "bus", None)
        if bus is None:
            # A unit reached over CAN or serial: no bus of the host's, its route alone.
            out.setdefault(ROUTES, _empty(None))["routes"].append((unit.link, unit.serial, unit.personality))
            continue
        if out.get(bus) is None:
            out[bus] = _empty(None)
        out[bus]["units"].append((int(unit.link.address), unit.serial, unit.personality))
    return out


def presence(finding: Optional[dict]) -> str:
    """present | absent | unknown: whether anything answered on the bus."""
    if finding is None:
        return "unknown"
    links = finding["links"].values()
    live = (finding["hub"] or finding["units"]
            or any(l["sensor"] or l["head"] or l["pod"] for l in links))
    return "present" if live else "absent"


def pods(finding: Optional[dict]) -> List[Tuple[str, dict]]:
    """(letter, pod) for every link of the finding that carries a pod."""
    if not finding:
        return []
    return [(letter, link["pod"]) for letter, link in sorted(finding["links"].items())
            if link["pod"]]


def _walk(buses: List[str], extra: Dict[str, set]) -> tuple:
    """The walk and the bare scan as `nxs generate` runs them, the scan over
    the camera buses alone."""
    from nxs import generate

    ports = generate.walk_ports()
    return ports, generate.scan_units(ports, buses=buses, extra=extra)


def _declared_addresses(cfg) -> Dict[str, set]:
    """{bus: addresses}: where the declaration puts units on each bus (a
    standalone unit's address, a pod's alias), asked beside the standard ones."""
    out: Dict[str, set] = {}
    for unit in getattr(cfg, "units", None) or []:
        for link in unit.links:
            if link.transport == "i2c" and link.bus and link.address is not None:
                out.setdefault(link.bus, set()).add(int(link.address))
    for port in (getattr(cfg, "ports", None) or {}).values():
        for link in port.links:
            if link.unit is not None and port.bus:
                out.setdefault(port.bus, set()).add(int(link.unit.alias))
    return out


#: The key of the finding that holds the units reached over no camera bus.
ROUTES = "-"


def _empty(port_name) -> dict:
    return {"port": port_name, "hub": None, "links": {}, "units": [], "collided": [], "routes": []}


def _finding(port) -> dict:
    finding = _empty(port.name)
    finding["hub"] = port.hub if port.hub_present else None
    for link in port.links:
        pod = None
        if link.unit_addr is not None:
            pod = {"addr": int(link.unit_addr), "serial": link.unit_serial, "fw": link.unit_fw,
                   "click": link.unit_click, "cam": link.unit_cam}
        finding["links"][link.name] = {
            "sensor": link.sensor, "head": bool(link.head_answers), "head_addr": link.head_addr,
            "pod": pod, "walked": not link.unwalked}
    if port.collided:
        # Pods the walk could not tell apart: each link's is declared at its
        # alias with no serial, so the next walk can (the report says so).
        address, letters = port.collided
        finding["collided"] = list(letters)
        for letter in letters:
            if letter in finding["links"] and finding["links"][letter]["pod"] is None:
                finding["links"][letter]["pod"] = {"addr": int(address), "serial": "", "fw": "",
                                                   "click": "", "cam": None}
    if port.shared_unit:
        addr, serial, _fw = port.shared_unit
        finding["units"].append((int(addr), serial, port.shared_click))
    return finding
