"""The link sweep behind `nxs generate` and the bare `nxs probe`: every
plausible link (I2C at the NXS addresses, USB serial ports, SocketCAN) and
what answered there."""
import glob
import sys
import os
import re
from dataclasses import dataclass
from typing import List, Optional, Tuple

from nxs._generated_constants import CyphalDefaults, NxsDevices
from nxs.suite.schema import SuiteConfig, LinkSpec, stable_path
from nxs import transports

# The register-map address, plus NXS+1: the alias convention for a second
# unit presented on the same host bus by a SerDes/mux.
I2C_ADDRESSES = (NxsDevices.RBDevice.NXS, NxsDevices.RBDevice.NXS + 1)


@dataclass
class Found:
    link: LinkSpec
    serial: str = ""
    fw_version: str = ""
    personality: str = ""


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
        # Presence is decided by probe; metadata is best-effort, so a read that
        # raises still reports the unit with blank fields.
        found = Found(link=link)
        try:
            raw = transport.read_serial()
            found.serial = raw.hex() if raw else ""
            found.fw_version = transport.read_fw_version() or ""
            found.personality = transport.read_personality_name() or ""
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
    """Every probeable I2C device node, one per underlying bus, preferring a
    stable udev alias over the enumerated name. Mux parents are excluded: a
    probe on one lands on whichever child channel the mux selects."""
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


def scan_bus_units(bus: str, addresses=I2C_ADDRESSES,
                   opener=None) -> List[Found]:
    """The units answering on one I2C bus at the given addresses. The
    opener defaults to the one `nxs.transports` holds at the call."""
    opener = opener or transports.open_client
    found = []
    for address in addresses:
        link = LinkSpec(transport="i2c", bus=bus, address=address)
        try:
            transport = opener("i2c", bus=bus, address=address)
        except Exception as e:
            # The bus is unusable and the next address will not fare better; name
            # the reason, since it reads like "no board here" otherwise.
            print(f"scan: {bus}: {e}", file=sys.stderr)
            break
        hit = _inspect(transport, link)
        if hit:
            found.append(hit)
    return found


def _scan_i2c(opener, declared: List[LinkSpec]) -> List[Found]:
    found = []
    for bus in _i2c_buses():
        node = stable_path(bus)
        addresses = sorted({link.address for link in declared
                            if link.transport == "i2c"
                            and stable_path(link.bus) == node}
                           | set(I2C_ADDRESSES))
        found.extend(scan_bus_units(bus, addresses, opener))
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
        from nxs.extras import install_line
        print("scan: pyserial not installed — serial sweep skipped "
              f"({install_line('cyphal')})", file=sys.stderr)
        return []
    # USB bridges are probed blind at the default baud; other ports only
    # when declared. Keyed by resolved node; a declared entry wins the key.
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
        except Exception as e:
            # A held port, a missing dialout membership, or absent DSDL all read
            # as "nothing answered" unless the reason is printed.
            print(f"scan: {port}: {e}", file=sys.stderr)
            continue
        hit = _inspect(transport, link)
        if hit:
            found.append(hit)
    return found


def _can_interfaces() -> List[Tuple[str, bool]]:
    """Every SocketCAN netdev as `(name, is_up)`; an adapter with no kernel
    driver bound produces no netdev, and a netdev never brought up carries nothing."""
    ifaces = []
    for path in sorted(glob.glob("/sys/class/net/*/type")):
        try:
            with open(path) as f:
                if f.read().strip() != "280":  # ARPHRD_CAN
                    continue
        except OSError:
            continue
        name = path.split("/")[-2]
        up = False
        try:
            with open(path.replace("/type", "/operstate")) as f:
                up = f.read().strip() not in ("down", "unknown")
        except OSError:
            pass
        ifaces.append((name, up))
    return ifaces


def _scan_can(opener, declared: List[LinkSpec]) -> List[Found]:
    found = []
    ifaces = _can_interfaces()
    if not ifaces:
        if any(link.transport == "cyphal-can" for link in declared):
            print("scan: no SocketCAN interface on this host — a CAN adapter "
                  "needs its kernel driver (gs_usb for Geschwister/candleLight "
                  "devices) and `ip link set up`", file=sys.stderr)
        return found
    for iface, up in ifaces:
        if not up:
            print(f"scan: {iface} is down — bring it up at the device's bit "
                  f"timing before it can answer", file=sys.stderr)
            continue
        node_ids = sorted({link.node_id for link in declared
                           if link.transport == "cyphal-can" and link.iface == iface}
                          | {CyphalDefaults.DEFAULT_NODE_ID})
        for node_id in node_ids:
            link = LinkSpec(transport="cyphal-can", iface=iface, node_id=node_id)
            try:
                transport = opener("cyphal-can", can_iface=iface,
                                   remote_node_id=node_id)
            except Exception as e:
                # The reason is diagnosable ("no pycyphal", "no CAN FD"), so name it,
                # then stop probing this iface.
                print(f"scan: {iface}: {e}", file=sys.stderr)
                break
            hit = _inspect(transport, link)
            if hit:
                found.append(hit)
    return found


def scan_suite(cfg: Optional[SuiteConfig] = None,
               opener=None) -> List[Found]:
    """Probe every plausible link; `cfg` widens the I2C address, serial
    port, and CAN node-id sweeps to the declared links. The opener defaults
    to the one `nxs.transports` holds at the call."""
    opener = opener or transports.open_client
    declared = [link for u in cfg.units for link in u.links] if cfg else []
    return (_scan_i2c(opener, declared) + _scan_serial(opener, declared)
            + _scan_can(opener, declared))


def _module_for_driver(active: str) -> Optional[str]:
    """The click personality whose class compiles to the name the device reports, or
    None when no known personality carries that class."""
    from nxs.suite.reconcile import click_personality_for
    try:
        return click_personality_for(active)
    except Exception:
        return None
