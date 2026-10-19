"""suite.yaml: load the declaration, a graph of blocks and the connections
between their ports (`nxs.suite.nodes`), and derive what every verb reads
from it: the host's camera ports and the units with the routes that reach
them (`nxs.suite.schema_nodes`). A unit is one board and its `links` the
routes that reach it; management flows over the first link that answers.
Validation is strict and names the YAML path."""
import os
import re
from dataclasses import dataclass, field
from typing import Any, List, Optional

import yaml

from nxs.suite.schema_base import ManifestError, _parse_int, _require_keys
from nxs.suite.schema_nodes import (
    HUB_DRIVERS, SYNC_SOURCES,
    PortLinkSpec, PortSpec, PortUnitRef,
)

__all__ = [
    "HUB_DRIVERS", "LinkSpec", "ManifestError", "PortLinkSpec", "PortSpec", "PortUnitRef",
    "SYNC_SOURCES", "SensorSpec", "SuiteConfig", "TRANSPORTS", "UnitSpec",
    "device_proves_patch", "device_runs", "hardware_path",
    "load_suite_config", "normalize_serial", "parse_device_version", "parse_route",
    "parse_suite_config", "parse_version", "stable_path",
    "_parse_int", "_require_keys",
]

TRANSPORTS = ("i2c", "cyphal-can", "cyphal-serial")

_LINK_KEYS = {
    "i2c": {"required": {"bus", "address"}, "optional": set()},
    "cyphal-can": {"required": {"iface", "node_id"}, "optional": set()},
    "cyphal-serial": {"required": {"port"}, "optional": {"baud"}},
}


def stable_path(path: Optional[str]) -> Optional[str]:
    """A device path resolved to its underlying node, so a udev alias and the
    kernel's enumerated node name the same link."""
    return os.path.realpath(path) if path else path


@dataclass
class LinkSpec:
    transport: str
    bus: Optional[str] = None
    address: Optional[int] = None
    iface: Optional[str] = None
    node_id: Optional[int] = None
    port: Optional[str] = None
    baud: Optional[int] = None
    ## i2c route form ("cam0/A"): resolved to bus+address against ports.
    link_ref: Optional[str] = None
    ## The camera connector whose bus the route rides ("cam0"): its bus is
    ## the host's fact, resolved at load.
    connector: Optional[str] = None

    def client_kwargs(self) -> dict:
        """Constructor kwargs for `open_client(self.transport, **kwargs)`."""
        if self.transport == "i2c":
            return {"bus": self.bus, "address": self.address}
        if self.transport == "cyphal-can":
            return {"can_iface": self.iface, "remote_node_id": self.node_id}
        if self.transport == "cyphal-serial":
            kwargs = {"port": self.port}
            if self.baud is not None:
                kwargs["baud"] = self.baud
            return kwargs
        return {}

    def describe(self) -> str:
        if self.transport == "i2c":
            return f"i2c {self.bus or self.link_ref or self.connector}@0x{self.address:02X}"
        if self.transport == "cyphal-can":
            return f"can {self.iface} node {self.node_id}"
        if self.transport == "cyphal-serial":
            return f"serial {self.port}"
        return self.transport

    def identity(self) -> tuple:
        """The fields that address a device, with device paths resolved; baud
        and other tuning fields are excluded. The bus a pod's link or a
        connector rides is the host's fact, filled at load: until then the
        port's name stands for it, so two pods strapped alike behind two
        hubs are two routes."""
        if self.transport == "i2c":
            place = stable_path(self.bus) if self.bus else (
                self.link_ref.split("/", 1)[0] if self.link_ref else self.connector)
            return ("i2c", place, self.address)
        if self.transport == "cyphal-can":
            return ("cyphal-can", self.iface, self.node_id)
        if self.transport == "cyphal-serial":
            return ("cyphal-serial", stable_path(self.port))
        return (self.transport,)


@dataclass
class SensorSpec:
    """A click on a unit's socket: its personality and its declared rows,
    served values by row name, the configuration it compiles with."""
    personality: str
    config: dict = field(default_factory=dict)


@dataclass
class UnitSpec:
    """A unit node as the unit step drives it: its routes, the clicks on its
    socket (an empty list keeps the store empty), its identity pins, and the
    unit's own rows it declares, served values by row name."""
    name: str
    module: str
    links: List[LinkSpec]
    sensors: List[SensorSpec] = field(default_factory=list)
    serial: Optional[str] = None
    firmware: Optional[str] = None
    rows: dict = field(default_factory=dict)


@dataclass
class SuiteConfig:
    name: str = ""
    units: List[UnitSpec] = field(default_factory=list)
    ports: dict = field(default_factory=dict)  # name -> PortSpec
    #: The declared graph the ports and the units derive from.
    graph: Any = None


def parse_version(text: str) -> tuple:
    """'1.2' / '1.2.3' → a 3-tuple with missing parts zeroed."""
    parts = str(text).split(".")
    if not 2 <= len(parts) <= 3 or not all(p.isdigit() for p in parts):
        raise ValueError(f"bad version {text!r} (expected MAJOR.MINOR[.PATCH])")
    nums = [int(p) for p in parts] + [0]
    return tuple(nums[:3])


_DEVICE_VERSION_RE = re.compile(
    r"v?(\d+)\.(\d+)(?:\.(\d+))?"
    r"(?:$|-dirty$|-\d+-g[0-9a-f]+(?:-dirty)?$)")


def parse_device_version(text):
    """Version triple proven by a device identity string, or None. Accepts the
    legacy "MAJOR.MINOR" pair and a `git describe` build identity; a prerelease
    tag or a bare SHA proves None. Never raises."""
    m = _DEVICE_VERSION_RE.match(str(text))
    if m is None:
        return None

    return (int(m.group(1)), int(m.group(2)), int(m.group(3) or 0))


def device_proves_patch(text):
    """Whether a device identity string pins its patch component: a legacy
    "MAJOR.MINOR" pair does not, a full build identity does."""
    m = _DEVICE_VERSION_RE.match(str(text))

    return m is not None and m.group(3) is not None


def device_runs(identity, want: tuple):
    """Whether a device identity runs version `want`, at the precision the
    identity proves: True/False for a legacy pair or a full build identity,
    None when it proves nothing."""
    ver = parse_device_version(identity)
    if ver is None:
        return None
    n = 3 if device_proves_patch(identity) else 2
    return ver[:n] == tuple(want)[:n]


def normalize_serial(text, where: str) -> str:
    """Lowercased 24-hex-digit UID96, separators stripped. Only YAML strings
    are accepted: a bare `<digits>e<digits>` UID resolves as a float."""
    if not isinstance(text, str):
        raise ManifestError(
            f"{where}: serial parsed as a YAML number, not a string — an "
            f"unquoted <digits>e<digits> UID reads as scientific notation; "
            f"quote it: serial: \"<24 hex digits>\"")
    raw = text.replace(":", "").replace("-", "").strip().lower()
    if len(raw) != 24 or any(c not in "0123456789abcdef" for c in raw):
        raise ManifestError(
            f"{where}: serial must be the 12-byte UID96 as 24 hex digits, "
            f"got {text!r}")
    return raw


def parse_node_id(raw, where: str) -> int:
    """A unit's Cyphal node id, 0 to 125, refused at `where` otherwise."""
    node_id = _parse_int(raw, where)
    if not 0 <= node_id <= 125:
        raise ManifestError(f"{where}: {node_id} out of range 0-125")

    return node_id


def parse_route(raw, where: str) -> LinkSpec:
    """One route to a unit, `{transport, ...}` as the derivation states it."""
    if not isinstance(raw, dict) or "transport" not in raw:
        raise ManifestError(f"{where}: link needs a 'transport' key "
                            f"(one of {list(TRANSPORTS)})")
    transport = raw["transport"]
    if transport not in TRANSPORTS:
        raise ManifestError(f"{where}.transport: unknown transport {transport!r} "
                            f"(one of {list(TRANSPORTS)})")
    keys = _LINK_KEYS[transport]
    _require_keys(raw, {"transport"} | keys["required"] | keys["optional"], where)
    missing = keys["required"] - set(raw)
    if missing:
        raise ManifestError(f"{where}: {transport} link needs {sorted(missing)}")

    link = LinkSpec(transport=transport)
    if transport == "i2c":
        link.bus = str(raw["bus"])
        link.address = _parse_int(raw["address"], f"{where}.address")
        if not 0x08 <= link.address <= 0x77:
            raise ManifestError(f"{where}.address: 0x{link.address:X} is not a usable 7-bit I2C "
                                f"address (0x08-0x77)")
    elif transport == "cyphal-can":
        link.iface = str(raw["iface"])
        link.node_id = parse_node_id(raw["node_id"], f"{where}.node_id")
    elif transport == "cyphal-serial":
        link.port = str(raw["port"])
        if "baud" in raw:
            link.baud = _parse_int(raw["baud"], f"{where}.baud")
    return link


def _resolve_link_refs(cfg: SuiteConfig) -> None:
    """Fill a pod's route along its camera chain (`<port>/<link>`) with the
    port's bus and the address the link presents the pod at."""
    for unit in cfg.units:
        for link in unit.links:
            if link.transport != "i2c" or link.link_ref is None:
                continue
            pname, lname = link.link_ref.split("/", 1)
            port = cfg.ports[pname]
            plink = next(l for l in port.links if l.name == lname)
            link.bus = port.bus
            link.address = (plink.unit.alias if port.hub_compatible
                            else plink.unit.target)


def parse_suite_config(raw: dict, where: str = "suite.yaml") -> SuiteConfig:
    """Validate a loaded declaration into a `SuiteConfig`: the graph's laws,
    then what it derives, then the units' own: one route reaches one board,
    and one board is one unit."""
    from nxs.suite.nodes import parse_graph
    from nxs.suite.schema_nodes import derive

    if not isinstance(raw, dict):
        raise ManifestError(f"{where}: not a mapping")
    graph = parse_graph(raw, where)
    ports, units = derive(graph, where, os.path.basename(where.split(":", 1)[0]) or "suite.yaml")
    cfg = SuiteConfig(name=graph.name, units=units, ports=ports, graph=graph)
    # Route-form links resolve to bus+address before the edge checks,
    # so identity comparison sees real addresses.
    _resolve_link_refs(cfg)

    # One route reaches one board, so an edge belongs to exactly one unit.
    edge_owner: dict = {}
    for u in units:
        identities = [l.identity() for l in u.links]
        if len(set(identities)) != len(identities):
            raise ManifestError(f"{where}.nodes.{u.name}: the same route is declared twice")
        for link in u.links:
            ident = link.identity()
            if ident in edge_owner:
                raise ManifestError(
                    f"{where}.nodes.{u.name}: {edge_owner[ident]} and {u.name} both "
                    f"declare {link.describe()}; one route reaches one board",
                    [f"merge {edge_owner[ident]} and {u.name} into one unit node with "
                     f"several routes"])
            edge_owner[ident] = u.name

    # One board is one unit: two units pinning the same silicon would fight
    # on every apply. A multi-homed board is one unit with several routes.
    pinned: dict = {}
    for u in units:
        if u.serial:
            if u.serial in pinned:
                raise ManifestError(
                    f"{where}.nodes.{u.name}.parameters.serial: {pinned[u.serial]} and "
                    f"{u.name} pin the same serial {u.serial}; one board is one unit",
                    [f"merge {pinned[u.serial]} and {u.name} into one unit node"])
            pinned[u.serial] = u.name

    return cfg


def _read_yaml(path: str) -> Any:
    try:
        with open(path, encoding="utf-8") as f:
            return yaml.safe_load(f)
    except OSError as e:
        raise ManifestError(f"{path}: {e.strerror or e}") from None
    except (yaml.YAMLError, UnicodeDecodeError) as e:
        raise ManifestError(f"{path}: not valid YAML ({e})") from None


def load_suite_config(path: str) -> SuiteConfig:
    """Load and validate the declaration at `path`. The hardware file
    beside it is `nxs generate`'s report and is never read here. Every
    read or parse failure raises `ManifestError`."""
    raw = _read_yaml(path)
    if raw is None:
        raise ManifestError(f"{path}: empty manifest")
    if not isinstance(raw, dict):
        raise ManifestError(f"{path}: not a mapping")
    cfg = parse_suite_config(raw, where=path)
    # A port that names no bus is one of the host's, by name; a unit that rides one of its
    # links, or the bus of a connector, takes the same bus.
    connectors = {link.connector for unit in cfg.units for link in unit.links if link.connector}
    if connectors or any(port.bus is None for port in cfg.ports.values()):
        from nxs import host as host_layer
        buses = host_layer.current().camera_buses()
        for port in cfg.ports.values():
            if port.bus is None:
                port.bus = buses.get(port.name)
        for unit in cfg.units:
            for link in unit.links:
                if link.bus is None and link.link_ref:
                    port = cfg.ports.get(link.link_ref.split("/", 1)[0])
                    if port is not None:
                        link.bus = port.bus
                if link.bus is None and link.connector:
                    port = cfg.ports.get(link.connector)
                    link.bus = (port.bus if port is not None and port.bus
                                else buses.get(link.connector))
    return cfg


def hardware_path(manifest: str) -> str:
    """The generated wiring file that belongs to a manifest."""
    return os.path.join(os.path.dirname(manifest) or ".", "hardware.yaml")
