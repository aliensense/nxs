"""`nxs suite scan`: transcribe reality into manifest form.

Probes every plausible link — I2C buses at the NXS addresses, USB
serial ports, SocketCAN interfaces — and reports what answered.
`--init` emits a suite.yaml skeleton: hits that reported the same
serial are one board reached over several routes, so they group into
one unit with several links; the operator renames units and adds
sensors. `--diff` compares reality against an existing manifest, per
edge. Discovery widens with the manifest: I2C probes the well-known
addresses plus any declared bus/address, serial probes USB bridges
plus any declared port, CAN probes the declared node-ids plus the
factory default. Full bus enumeration arrives with the suite runtime.
"""
import glob
import os
import re
from dataclasses import dataclass
from typing import List, Optional

from nxs._generated_constants import CyphalDefaults, NxsDevices
from nxs.suite import FIRMWARE_DIR
from nxs.suite.schema import SuiteConfig, LinkSpec, stable_path
from nxs.transports import open_client

# The register-map address, plus NXS+1 — the alias convention for a
# second unit presented on the same host bus by a SerDes/mux.
I2C_ADDRESSES = (NxsDevices.RBDevice.NXS, NxsDevices.RBDevice.NXS + 1)


@dataclass
class Found:
    link: LinkSpec
    serial: str = ""
    fw_version: str = ""
    driver: str = ""


def _suggest_name(link: LinkSpec) -> str:
    if link.transport == "i2c":
        return f"unit-{os.path.basename(link.bus)}-{link.address:02x}"
    if link.transport == "cyphal-can":
        return f"unit-{link.iface}-{link.node_id}"
    return f"unit-{os.path.basename(link.port)}"


def _inspect(transport, link: LinkSpec) -> Optional[Found]:
    try:
        try:
            alive = transport.probe()
        except Exception:
            return None
        if not alive:
            return None
        # Presence is decided by probe; metadata is best-effort, so a
        # read that raises (I²C can throw mid-transaction) still reports
        # the unit — blank fields, not a dropped device.
        found = Found(link=link)
        try:
            raw = transport.read_serial()
            found.serial = raw.hex() if raw else ""
            found.fw_version = transport.read_fw_version() or ""
            found.driver = transport.read_driver_name() or ""
        except Exception:
            pass
        return found
    finally:
        try:
            transport.close()
        except Exception:
            pass


# Kernel adapter name of an i2c-mux child, e.g. "i2c-2-mux (chan_id 1)";
# group 1 is the parent adapter number.
_MUX_CHILD_NAME = re.compile(r"^i2c-(\d+)-mux\b")


def _adapter_name(bus: str) -> str:
    """The kernel adapter name behind a `/dev/i2c-N` node ('' when
    unreadable, e.g. off-Linux)."""
    node = os.path.basename(os.path.realpath(bus))
    try:
        with open(f"/sys/class/i2c-dev/{node}/device/name",
                  encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return ""


def _i2c_buses() -> List[str]:
    """Every probeable I2C device node, one entry per underlying bus.

    A stable udev alias (`/dev/i2c-cam1` -> `i2c-9`) matches the same
    glob, so paths dedupe by their resolved node — preferring the
    alias, so a transcribed manifest carries connector names, not
    enumeration order. Mux *parents* are excluded: a probe on the
    parent lands on whichever child channel the mux currently selects,
    so it can only ever duplicate — nondeterministically — a unit the
    child buses already report."""
    by_node = {}
    for path in sorted(glob.glob("/dev/i2c-*"),
                       key=lambda p: (not os.path.islink(p), p)):
        by_node.setdefault(os.path.realpath(path), path)
    buses = sorted(by_node.values())

    mux_parents = set()
    for bus in buses:
        if m := _MUX_CHILD_NAME.match(_adapter_name(bus)):
            mux_parents.add(f"i2c-{m.group(1)}")

    return [b for b in buses
            if os.path.basename(stable_path(b)) not in mux_parents]


def _scan_i2c(opener, declared: List[LinkSpec]) -> List[Found]:
    found = []
    for bus in _i2c_buses():
        node = stable_path(bus)
        addresses = sorted({link.address for link in declared
                            if link.transport == "i2c"
                            and stable_path(link.bus) == node}
                           | set(I2C_ADDRESSES))
        for address in addresses:
            link = LinkSpec(transport="i2c", bus=bus, address=address)
            try:
                transport = opener("i2c", bus=bus, address=address)
            except Exception:
                break  # bus unusable; the next address won't fare better
            hit = _inspect(transport, link)
            if hit:
                found.append(hit)
    return found


def _serial_alias(device: str) -> str:
    """The stable /dev/serial/by-id name for a port when one resolves
    to it, so a transcribed manifest survives replug reordering."""
    for alias in sorted(glob.glob("/dev/serial/by-id/*")):
        if os.path.realpath(alias) == os.path.realpath(device):
            return alias
    return device


def _scan_serial(opener, declared: List[LinkSpec]) -> List[Found]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    # USB bridges are probed blind at the default baud; other ports
    # (SoC UARTs) only when declared — a GetInfo probe writes to the
    # port, which is rude on a console UART nobody declared. Keyed by
    # resolved node so a declared by-id alias and its enumerated device
    # probe once; a declared entry wins the key so its baud rides along.
    ports = {stable_path(p.device): (_serial_alias(p.device), None)
             for p in list_ports.comports() if p.vid is not None}
    for link in declared:
        if link.transport == "cyphal-serial":
            ports[stable_path(link.port)] = (link.port, link.baud)
    found = []
    for port, baud in sorted(ports.values(), key=lambda entry: entry[0]):
        link = LinkSpec(transport="cyphal-serial", port=port, baud=baud)
        kwargs = {"port": port}
        if baud is not None:
            kwargs["baud"] = baud
        try:
            transport = opener("cyphal-serial", **kwargs)
        except Exception:
            continue
        hit = _inspect(transport, link)
        if hit:
            found.append(hit)
    return found


def _can_interfaces() -> List[str]:
    ifaces = []
    for path in sorted(glob.glob("/sys/class/net/*/type")):
        try:
            with open(path) as f:
                if f.read().strip() == "280":  # ARPHRD_CAN
                    ifaces.append(path.split("/")[-2])
        except OSError:
            pass
    return ifaces


def _scan_can(opener, declared: List[LinkSpec]) -> List[Found]:
    found = []
    for iface in _can_interfaces():
        node_ids = sorted({link.node_id for link in declared
                           if link.transport == "cyphal-can" and link.iface == iface}
                          | {CyphalDefaults.DEFAULT_NODE_ID})
        for node_id in node_ids:
            link = LinkSpec(transport="cyphal-can", iface=iface, node_id=node_id)
            try:
                transport = opener("cyphal-can", can_iface=iface,
                                   remote_node_id=node_id)
            except Exception:
                break
            hit = _inspect(transport, link)
            if hit:
                found.append(hit)
    return found


def scan_suite(cfg: Optional[SuiteConfig] = None,
               opener=open_client) -> List[Found]:
    """Probe every plausible link; `cfg` widens the I2C address, serial
    port, and CAN node-id sweeps to the declared links."""
    declared = [link for u in cfg.units for link in u.links] if cfg else []
    return (_scan_i2c(opener, declared) + _scan_serial(opener, declared)
            + _scan_can(opener, declared))


def _link_yaml(link: LinkSpec) -> str:
    if link.transport == "i2c":
        return (f"{{transport: i2c, bus: {link.bus}, "
                f"address: 0x{link.address:02X}}}")
    if link.transport == "cyphal-can":
        return (f"{{transport: cyphal-can, iface: {link.iface}, "
                f"node_id: {link.node_id}}}")
    baud = f", baud: {link.baud}" if link.baud is not None else ""
    return f"{{transport: cyphal-serial, port: {link.port}{baud}}}"


def _edge_priority(link: LinkSpec) -> tuple:
    """Sort key ordering a grouped board's links — the first one is the
    management route `apply` tries first. Wired-local I2C leads, CAN
    next (a separate power/failure domain that survives SerDes resets),
    USB-serial last (a debug cable, not product wiring)."""
    return (("i2c", "cyphal-can", "cyphal-serial").index(link.transport),)


def group_found(found: List[Found]) -> List[tuple]:
    """Group scan hits into boards: hits sharing a serial are one board
    reached over several routes. Returns `(primary hit, links)` pairs,
    links in management-priority order."""
    nodes: List[tuple] = []
    node_by_serial: dict = {}
    for hit in found:
        if hit.serial and hit.serial in node_by_serial:
            nodes[node_by_serial[hit.serial]][1].append(hit.link)
            continue
        if hit.serial:
            node_by_serial[hit.serial] = len(nodes)
        nodes.append((hit, [hit.link]))
    for _, links in nodes:
        links.sort(key=_edge_priority)
    return nodes


def unit_lines(hit: Found, links: List[LinkSpec]) -> List[str]:
    """The manifest lines for one board, in the skeleton shape."""
    lines = []
    name_line = f"  - name: {_suggest_name(links[0])}"
    if not hit.serial:
        name_line += "   # no serial read — verify this is a distinct board"
    lines.append(name_line)
    lines.append("    module: nxs")
    if len(links) == 1:
        lines.append(f"    links: [{_link_yaml(links[0])}]")
    else:
        lines.append("    links:")
        lines.extend(f"      - {_link_yaml(link)}" for link in links)
    if hit.serial:
        # Quoted: a bare <digits>e<digits> UID is YAML scientific
        # notation and would corrupt on any float-resolving parse.
        lines.append(f'    serial: "{hit.serial}"')
    if hit.fw_version:
        # The running version is a fact, not intent — an active pin
        # would demand an image the operator never staged.
        lines.append(f'    # firmware: "{hit.fw_version}"  '
                     f'# uncomment to pin; image goes in {FIRMWARE_DIR}')
    # Commented: an absent key leaves the panel unmanaged; an explicit
    # `sensors: []` would enforce an empty store on the next apply.
    lines.append("    # sensors: [{driver: iam20680, config: {sample_rate: 250}}]")
    return lines


def render_init(found: List[Found]) -> str:
    """A suite.yaml skeleton from scan results — one unit per board;
    names and sensors are left to the operator."""
    lines = ["# Generated by `nxs suite scan --init` — rename the units to",
             "# their roles and declare each unit's sensors."]
    if not found:
        # Nothing answered — emit a commented template so the operator has
        # a unit to fill in, not a bare (and unappliable) empty list.
        lines += [
            "# Nothing answered on any link — check wiring and permissions",
            "# and re-run, or declare a unit by hand, e.g.:",
            "#   - name: imu-mast",
            "#     module: nxs",
            "#     links: [{transport: cyphal-serial, port: /dev/ttyUSB0}]",
            "#     sensors: [{driver: iam20680, config: {sample_rate: 250}}]",
            "units: []",
        ]
        return "\n".join(lines) + "\n"
    lines.append("units:")
    for hit, links in group_found(found):
        lines.extend(unit_lines(hit, links))
    return "\n".join(lines) + "\n"


def merge_init(cfg: SuiteConfig, found: List[Found],
               config_path: str) -> List[str]:
    """Grow an existing manifest in place from scan results, changing
    nothing hand-written. Declared units are recognized by serial or by
    any declared link identity and left untouched; a recognized board
    answering on a new route gets that link appended at the end of its
    `links` (hand-ordered management priority survives); new boards
    append as skeleton entries. Returns the change-report lines."""
    from nxs.suite.freeze import _round_trip_yaml

    declared_serials = {u.serial: u for u in cfg.units if u.serial}
    declared_edges = {link.identity(): u for u in cfg.units
                      for link in u.links}
    report = []
    link_appends: dict = {}   # unit name -> [LinkSpec]
    new_units: List[tuple] = []

    for hit, links in group_found(found):
        unit = declared_serials.get(hit.serial) if hit.serial else None
        if unit is None:
            unit = next((declared_edges[link.identity()] for link in links
                         if link.identity() in declared_edges), None)
        if unit is None:
            new_units.append((hit, links))
            continue
        known = {link.identity() for link in unit.links}
        fresh = [link for link in links if link.identity() not in known]
        if fresh:
            link_appends.setdefault(unit.name, []).extend(fresh)
            routes = ", ".join(link.describe() for link in fresh)
            report.append(f"kept {unit.name} (new link: {routes})")
        else:
            report.append(f"kept {unit.name}")

    # Append new links through the comment-preserving round-trip writer.
    if link_appends:
        yaml_rt = _round_trip_yaml()
        with open(config_path, encoding="utf-8") as f:
            doc = yaml_rt.load(f)
        for entry in doc.get("units", []):
            for link in link_appends.get(entry.get("name"), []):
                entry["links"].append(_link_map(yaml_rt, link))
        tmp = config_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            yaml_rt.dump(doc, f)
        os.replace(tmp, config_path)

    # Append new boards textually — the file above the append survives
    # byte-for-byte, comments included.
    if new_units:
        lines = []
        for hit, links in new_units:
            lines.extend(unit_lines(hit, links))
            report.append(f"added {_suggest_name(links[0])}")
        with open(config_path, "a", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

    unchanged = sum(1 for line in report if line.startswith("kept ")
                    and "new link" not in line)
    silent = len(cfg.units) - sum(1 for line in report
                                  if line.startswith("kept "))
    tail = [f"{unchanged} unchanged"]
    if silent:
        tail.append(f"{silent} declared unit(s) silent (left untouched)")
    report.append("; ".join(tail))
    return report


def _link_map(yaml_rt, link: LinkSpec):
    """A flow-style mapping for one link, matching the skeleton shape."""
    from ruamel.yaml.comments import CommentedMap

    m = CommentedMap()
    m.fa.set_flow_style()
    m["transport"] = link.transport
    if link.transport == "i2c":
        m["bus"] = link.bus
        m["address"] = link.address
    elif link.transport == "cyphal-can":
        m["iface"] = link.iface
        m["node_id"] = link.node_id
    else:
        m["port"] = link.port
        if link.baud is not None:
            m["baud"] = link.baud
    return m


def render_diff(cfg: SuiteConfig, found: List[Found]) -> str:
    """Reality vs manifest, per edge: declared-but-silent routes,
    answering-but-undeclared ones. Keyed by `LinkSpec.identity()`, so
    an alias-declared link matches the enumeration-named bus the scan
    reports it on."""
    declared = {link.identity(): unit
                for unit in cfg.units for link in unit.links}
    seen = {hit.link.identity(): hit for hit in found}

    lines = []
    for unit in cfg.units:
        hits = [(link, seen.get(link.identity())) for link in unit.links]
        if all(hit is None for _, hit in hits):
            routes = " or ".join(link.describe() for link in unit.links)
            lines.append(f"missing   {unit.name}: no response on {routes}")
            continue
        for link, hit in hits:
            if hit is None:
                lines.append(f"edge down {unit.name}: {link.describe()} silent")
            elif unit.serial and hit.serial and unit.serial != hit.serial:
                lines.append(f"serial    {unit.name}: pinned {unit.serial}, "
                             f"{link.describe()} reports {hit.serial}")
    # An answering route to silicon the manifest already knows is a
    # one-line links: addition; anything else is a stranger.
    unit_by_serial = {u.serial: u.name for u in cfg.units if u.serial}
    for k, hit in seen.items():
        if k not in declared:
            if hit.serial and hit.serial in unit_by_serial:
                lines.append(f"undeclared edge of {unit_by_serial[hit.serial]}: "
                             f"{hit.link.describe()} — add it to the unit's "
                             f"links to manage it")
            else:
                lines.append(f"undeclared {hit.link.describe()}"
                             + (f" (serial {hit.serial})" if hit.serial else ""))
    if not lines:
        lines.append("manifest matches reality")
    return "\n".join(lines) + "\n"
