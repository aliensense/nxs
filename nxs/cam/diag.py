# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Descriptor-driven health diagnostics (the `nxs cam status` engine). Each
descriptor declares its own ``status:`` probes; this module walks the topology
and evaluates them. A sensor module may add ``derive_status(readings)`` lines."""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from .descriptors import to_int

from . import packs
from .contracts import Topology
from .descriptors import Descriptor


@dataclass(frozen=True)
class ProbeResult:
    """One evaluated status probe."""

    name: str
    raw: Optional[int]  # None when the read failed
    text: str
    ok: Optional[bool]  # None = informational
    desc: str

    @property
    def answered(self) -> bool:
        """Whether the register read at all. `ok` is the verdict a probe
        declaring `expect` or `warn_nonzero` earns; a decode-only probe
        never earns one, and its None must not read as silence."""
        return self.raw is not None


def _read_register(i2c: Any, device_addr: int, reg_def: Dict[str, Any]) -> int:
    """Read a register per its descriptor definition (multi-byte aware)."""
    width = to_int(reg_def.get("width", 1))
    data = i2c.read_reg(
        to_int(reg_def["addr"]), length=width, reg_width=16, data_width=8,
        addr=hex(device_addr),
    )
    order = str(reg_def.get("order", "be" if width == 1 else "le"))
    values = list(data)
    if order == "le":
        values = list(reversed(values))
    out = 0
    for byte in values:
        out = (out << 8) | byte
    return out


def run_probes(
    i2c: Any,
    device_addr: int,
    descriptor: Descriptor,
) -> List[ProbeResult]:
    """Evaluate a descriptor's declared status probes on a live device."""
    results: List[ProbeResult] = []
    for probe in descriptor.raw("status") or []:
        name = str(probe["name"])
        desc = str(probe.get("desc", ""))
        reg_def = descriptor.registers[str(probe["reg"])]
        try:
            raw = _read_register(i2c, device_addr, reg_def)
        except Exception:
            results.append(ProbeResult(name, None, "no ACK", False, desc))
            continue

        value = raw & to_int(probe["mask"]) if "mask" in probe else raw
        ok: Optional[bool] = None
        if "expect" in probe:
            ok = value == to_int(probe["expect"])
        if probe.get("warn_nonzero"):
            ok = value == 0

        decode = probe.get("decode") or {}
        decoded = {int(k): str(v) for k, v in decode.items()}
        if value in decoded:
            text = decoded[value]
        elif str(probe.get("format", "")) == "int":
            text = str(value)
        else:
            text = f"0x{value:02X}"
        results.append(ProbeResult(name, raw, text, ok, desc))
    return results


def derive_lines(pack, compatible: str,
                 results: List[ProbeResult]) -> List[str]:
    """Ask the chip module for derived status lines, if it offers any."""
    try:
        module = pack.chip_module(compatible)
    except Exception:
        module = None
    hook = getattr(module, "derive_status", None) if module else None
    if hook is None:
        return []
    readings = {r.name: r.raw for r in results if r.raw is not None}
    try:
        return list(hook(readings))
    except Exception:
        return []


def probe_topology(
    i2c: Any, topology: Topology, pack, links=None,
) -> List[Tuple[str, List, List[str]]]:
    """Walk the carrier, des first, then each link through its window; ``links``
    narrows the walk. A direct port has no des and no serializer: its sensor
    is read on the bus itself. Returns sections as (title, probe results,
    derived lines)."""
    sections: List[Tuple[str, List, List[str]]] = []
    flows = pack.flows()

    if topology.is_direct:
        for link in links or topology.links:
            send = pack.descriptor(link.sensor_compatible)
            sen_results = run_probes(i2c, packs.sensor_address(pack, link), send)
            sections.append((
                f"link {link.name} SEN {link.sensor_compatible} (direct)",
                sen_results,
                derive_lines(pack, link.sensor_compatible, sen_results),
            ))
        return sections

    desd = pack.descriptor(topology.des_compatible)
    des_results = run_probes(i2c, topology.des_addr, desd)
    sections.append((
        f"DES {topology.des_compatible} @ {hex(topology.des_addr)}",
        des_results,
        derive_lines(pack, topology.des_compatible, des_results),
    ))

    des_alive = any(r.name == "device" and r.ok for r in des_results)
    if des_alive and getattr(topology, "hub_driver", "nxs") != "nxs":
        # Window selection writes CTRL0; a kernel-owned hub is reported from
        # its directly readable registers only.
        sections.append(("links (kernel-owned hub — not walked)", [], []))
        return sections
    if des_alive:
        for link in links or topology.links:
            flows.open_window(pack, i2c, topology, link)
            serd = pack.descriptor(link.ser_compatible)
            ser_results = run_probes(i2c, link.ser_addr, serd)
            sections.append((
                f"link {link.name} SER {link.ser_compatible} "
                f"(window {hex(link.des_window)})",
                ser_results,
                derive_lines(pack, link.ser_compatible, ser_results),
            ))
            if not link.has_camera:
                continue
            from nxs.cam.identity import answering_address

            send = pack.descriptor(link.sensor_compatible)
            sen_addr, mapped = answering_address(pack, i2c, link)
            sen_results = run_probes(i2c, sen_addr, send)
            # The alias the host reaches the head at while the port is up;
            # before the first `on` nothing maps it and the head answers
            # at its own address, on every link the hub merges.
            if mapped:
                at = f" @{sen_addr:#04x}"
            elif int(packs.sensor_address(pack, link)) != sen_addr:
                at = f" (at its own address {sen_addr:#04x}; port not up)"
            else:
                at = ""
            sections.append((
                f"link {link.name} SEN {link.sensor_compatible}{at}",
                sen_results,
                derive_lines(pack, link.sensor_compatible, sen_results),
            ))
        flows.close_windows(pack, i2c, topology)

    return sections


def _use_color() -> bool:
    return sys.stdout.isatty() and not os.environ.get("NO_COLOR")


class _C:
    """Zero-dependency ANSI palette (empty strings when color is off)."""

    def __init__(self) -> None:
        on = _use_color()
        self.bold = "\033[1m" if on else ""
        self.dim = "\033[2m" if on else ""
        self.green = "\033[32m" if on else ""
        self.red = "\033[31m" if on else ""
        self.cyan = "\033[36m" if on else ""
        self.yellow = "\033[33m" if on else ""
        self.off = "\033[0m" if on else ""


def render(sections: List[Tuple[str, List, List[str]]]) -> None:
    """Print a diagnosis report: headed columns, colored verdicts."""
    c = _C()
    for title, results, derived in sections:
        print(f"{c.bold}{c.cyan}{title}{c.off}")
        if results:
            print(f"  {c.dim}{'PROBE':<18} {'VALUE':<12} {'':2} DETAIL{c.off}")
        for r in results:
            if r.ok is None:
                mark, val_color = "  ", ""
            elif r.ok:
                mark, val_color = f"{c.green}ok{c.off}", c.green
            else:
                mark, val_color = f"{c.red}!!{c.off}", c.red
            desc = f"{c.dim}{r.desc}{c.off}" if r.desc else ""
            print(f"  {r.name:<18} {val_color}{r.text:<12}{c.off} {mark} {desc}")
        for line in derived:
            print(f"  {c.yellow}{line}{c.off}")
        print()
