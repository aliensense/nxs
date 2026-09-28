"""suite.yaml manifest: parse and validate the declared suite. A unit is one
board and its `links` the routes that reach it; management flows over the
first link that answers. Validation is strict and names the YAML path."""
import os
import re
from dataclasses import dataclass, field
from typing import Any, List, Optional

import yaml

from nxs.suite.schema_base import ManifestError, _parse_int, _require_keys
from nxs.suite.schema_ports import (
    HARDWARE_LINK_KEYS, HARDWARE_PORT_KEYS, HUB_DRIVERS, SYNC_SOURCES,
    PortLinkSpec, PortSpec, PortUnitRef, _parse_link_camera, _parse_port,
)

__all__ = [
    "EGRESS_SUBJECTS", "EgressSpec", "HARDWARE_LINK_KEYS",
    "HARDWARE_PORT_KEYS", "HUB_DRIVERS", "LinkSpec", "MODULES",
    "ManifestError", "PortLinkSpec", "PortSpec", "PortUnitRef",
    "SYNC_SOURCES", "SensorSpec", "SuiteConfig", "TRANSPORTS", "UnitSpec",
    "device_proves_patch", "device_runs", "hardware_path",
    "load_suite_config", "normalize_serial",
    "parse_device_version", "parse_suite_config", "parse_version",
    "stable_path", "_parse_int", "_parse_link_camera", "_parse_port",
    "_require_keys",
]

TRANSPORTS = ("i2c", "cyphal-can", "cyphal-serial")
MODULES = ("nxs",)

_TOP_KEYS = {"suite", "defaults", "units", "ports"}
_SUITE_KEYS = {"name"}
_DEFAULTS_KEYS = {"firmware"}
_UNIT_KEYS = {"name", "module", "links", "serial", "firmware", "sensors",
              "egress", "orientation"}
_EGRESS_KEYS = {"decimation", "subjects"}
# The SI-subject tokens the per-subject factors address (SubjectBucket
# names; the same tokens name the Cyphal decimation.<subject> registers).
EGRESS_SUBJECTS = ("acceleration", "angular_velocity", "magnetic_field",
                   "temperature", "pressure", "scalar")
_SENSOR_KEYS = {"personality", "config"}
_LINK_KEYS = {
    "i2c": {"required": set(), "optional": {"bus", "address", "link"}},
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
            return f"i2c {self.bus}@0x{self.address:02X}"
        if self.transport == "cyphal-can":
            return f"can {self.iface} node {self.node_id}"
        if self.transport == "cyphal-serial":
            return f"serial {self.port}"
        return self.transport

    def identity(self) -> tuple:
        """The fields that address a device, with device paths resolved; baud
        and other tuning fields are excluded."""
        if self.transport == "i2c":
            return ("i2c", stable_path(self.bus), self.address)
        if self.transport == "cyphal-can":
            return ("cyphal-can", self.iface, self.node_id)
        if self.transport == "cyphal-serial":
            return ("cyphal-serial", stable_path(self.port))
        return (self.transport,)


@dataclass
class EgressSpec:
    ## None = the device-wide gate is unmanaged; declared factors converge.
    decimation: Optional[int] = None
    subjects: dict = field(default_factory=dict)


@dataclass
class SensorSpec:
    driver: str
    config: dict = field(default_factory=dict)


@dataclass
class UnitSpec:
    name: str
    module: str
    links: List[LinkSpec]
    ## None = panel unmanaged (key absent); [] = enforce an empty store.
    sensors: Optional[List[SensorSpec]] = None
    ## None = egress unmanaged; a declared section is enforced.
    egress: Optional[EgressSpec] = None
    serial: Optional[str] = None
    firmware: Optional[str] = None
    # Declared mounting orientation (a ROTATION_* name): installer intent tied
    # to the position, so it transfers to a swapped board.
    orientation: Optional[str] = None


@dataclass
class SuiteConfig:
    name: str = ""
    units: List[UnitSpec] = field(default_factory=list)
    ports: dict = field(default_factory=dict)  # name -> PortSpec


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


def _parse_link(raw, where: str) -> LinkSpec:
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
        if "link" in raw:
            if "bus" in raw or "address" in raw:
                raise ManifestError(
                    f"{where}: give either link: port/LINK or bus+address, "
                    f"not both")
            link.link_ref = str(raw["link"])
        elif "bus" in raw and "address" in raw:
            link.bus = str(raw["bus"])
            link.address = _parse_int(raw["address"], f"{where}.address")
            if not 0x08 <= link.address <= 0x77:
                raise ManifestError(
                    f"{where}.address: 0x{link.address:X} is not a usable "
                    f"7-bit I2C address (0x08-0x77)")
        else:
            raise ManifestError(
                f"{where}: i2c link needs bus+address, or link: port/LINK")
    elif transport == "cyphal-can":
        link.iface = str(raw["iface"])
        link.node_id = _parse_int(raw["node_id"], f"{where}.node_id")
        if not 0 <= link.node_id <= 125:
            raise ManifestError(f"{where}.node_id: {link.node_id} out of range 0-125")
    elif transport == "cyphal-serial":
        link.port = str(raw["port"])
        if "baud" in raw:
            link.baud = _parse_int(raw["baud"], f"{where}.baud")
    return link


def _parse_sensor(raw, where: str) -> SensorSpec:
    _require_keys(raw, _SENSOR_KEYS, where)
    if not isinstance(raw.get("personality"), str):
        raise ManifestError(f"{where}: sensor needs a personality (personality: <name>)")
    # Marketing names hyphenate (neo-m9n); module files underscore.
    driver = raw["personality"].replace("-", "_")
    # Personality names become module names and filesystem paths; the token
    # shape rules out traversal (`../evil`) by construction.
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", driver):
        raise ManifestError(
            f"{where}.personality: {raw['personality']!r} is not a module name "
            f"(letters, digits, underscores)")
    config = raw.get("config", {}) or {}
    if not isinstance(config, dict):
        raise ManifestError(f"{where}.config: expected a mapping")
    return SensorSpec(driver=driver, config=config)


def _parse_unit(raw, where: str, defaults: dict) -> UnitSpec:
    _require_keys(raw, _UNIT_KEYS, where)
    name = raw.get("name")
    if not name or not isinstance(name, str):
        raise ManifestError(f"{where}: unit needs a 'name'")
    module = raw.get("module", "nxs")
    if module not in MODULES:
        raise ManifestError(f"{where}.module: unknown module {module!r} "
                            f"(one of {list(MODULES)})")
    if "links" not in raw:
        raise ManifestError(f"{where}: unit needs a 'links' list")
    links_raw = raw["links"]
    if not isinstance(links_raw, list) or not links_raw:
        raise ManifestError(f"{where}.links: expected a non-empty list of "
                            f"links (management uses the first that answers, "
                            f"in declared order)")
    links = [_parse_link(entry, f"{where}.links[{i}]")
             for i, entry in enumerate(links_raw)]
    identities = [l.identity() for l in links]
    if len(set(identities)) != len(identities):
        raise ManifestError(f"{where}.links: the same link is declared twice")

    # Tri-state panel intent: an absent `sensors` key leaves the panel
    # unmanaged; an explicit `sensors: []` converges to an empty store.
    if "sensors" not in raw or raw["sensors"] is None:
        sensors = None
    else:
        sensors_raw = raw["sensors"]
        if not isinstance(sensors_raw, list):
            raise ManifestError(f"{where}.sensors: expected a list")
        sensors = [_parse_sensor(s, f"{where}.sensors[{i}]")
                   for i, s in enumerate(sensors_raw)]
    # A driver is a stable identity within its panel (drift and freeze match
    # the active driver by name), so a duplicate is rejected.
    driver_names = [s.driver for s in (sensors or [])]
    dupes = sorted({d for d in driver_names if driver_names.count(d) > 1})
    if dupes:
        raise ManifestError(f"{where}: duplicate driver(s) {dupes} in one unit")

    egress = None
    if "egress" in raw and raw["egress"] is not None:
        egress_raw = raw["egress"]
        _require_keys(egress_raw, _EGRESS_KEYS, f"{where}.egress")
        decimation = None
        if "decimation" in egress_raw:
            decimation = _parse_int(egress_raw["decimation"],
                                    f"{where}.egress.decimation")
            if not 0 <= decimation <= 0xFFFF:
                raise ManifestError(
                    f"{where}.egress.decimation: {decimation} out of the "
                    f"u16 range")
        subjects = {}
        subjects_raw = egress_raw.get("subjects", {}) or {}
        if not isinstance(subjects_raw, dict):
            raise ManifestError(f"{where}.egress.subjects: expected a mapping")
        for subject, factor in subjects_raw.items():
            if subject not in EGRESS_SUBJECTS:
                raise ManifestError(
                    f"{where}.egress.subjects: unknown subject {subject!r} "
                    f"(one of {list(EGRESS_SUBJECTS)})")
            value = _parse_int(factor, f"{where}.egress.subjects.{subject}")
            if not 0 <= value <= 0xFFFF:
                raise ManifestError(
                    f"{where}.egress.subjects.{subject}: {value} out of the "
                    f"u16 range")
            subjects[subject] = value
        egress = EgressSpec(decimation=decimation, subjects=subjects)

    serial = raw.get("serial")
    if serial is not None:
        serial = normalize_serial(serial, f"{where}.serial")

    firmware = raw.get("firmware", defaults.get("firmware"))
    if firmware is not None:
        firmware = str(firmware)
        try:
            parse_version(firmware)
        except ValueError as e:
            raise ManifestError(f"{where}.firmware: {e}") from None

    orientation = raw.get("orientation")
    if orientation is not None:
        from nxs.client import rotation_code
        try:
            rotation_code(str(orientation))
        except ValueError as e:
            raise ManifestError(f"{where}.orientation: {e}") from None
        orientation = str(orientation).upper()

    return UnitSpec(name=name, module=module, links=links, sensors=sensors,
                    egress=egress, serial=serial, firmware=firmware,
                    orientation=orientation)


def _resolve_link_refs(cfg: SuiteConfig, where: str) -> None:
    """Fill route-form unit links (link: port/LINK) from the ports."""
    by_name = {u.name: u for u in cfg.units}
    for pname, port in cfg.ports.items():
        for plink in port.links:
            if plink.unit and plink.unit.name not in by_name:
                raise ManifestError(
                    f"{where}.ports.{pname}.links.{plink.name}.unit: "
                    f"no unit named {plink.unit.name!r}")
    for unit in cfg.units:
        for i, link in enumerate(unit.links):
            if link.transport != "i2c" or link.link_ref is None:
                continue
            lw = f"{where}.units[{unit.name}].links[{i}]"
            try:
                pname, lname = link.link_ref.split("/", 1)
            except ValueError:
                raise ManifestError(
                    f"{lw}.link: expected port/LINK, got "
                    f"{link.link_ref!r}") from None
            port = cfg.ports.get(pname)
            if port is None:
                raise ManifestError(f"{lw}.link: no port named {pname!r}")
            plink = next((l for l in port.links if l.name == lname), None)
            if plink is None:
                raise ManifestError(
                    f"{lw}.link: port {pname!r} has no link {lname!r}")
            if plink.unit is None or plink.unit.name != unit.name:
                raise ManifestError(
                    f"{lw}.link: {link.link_ref} does not declare unit "
                    f"{unit.name!r} (add unit: to the port link)")
            link.bus = port.bus
            link.address = (plink.unit.alias if port.hub_compatible
                            else plink.unit.target)


def parse_suite_config(raw: dict, where: str = "suite.yaml") -> SuiteConfig:
    """Validate a loaded YAML mapping into a `SuiteConfig`."""
    _require_keys(raw, _TOP_KEYS, where)

    suite_raw = raw.get("suite", {}) or {}
    _require_keys(suite_raw, _SUITE_KEYS, f"{where}.suite")
    defaults = raw.get("defaults", {}) or {}
    _require_keys(defaults, _DEFAULTS_KEYS, f"{where}.defaults")
    if "firmware" in defaults:
        try:
            parse_version(defaults["firmware"])
        except ValueError as e:
            raise ManifestError(f"{where}.defaults.firmware: {e}") from None

    ports_raw = raw.get("ports") or {}
    if not isinstance(ports_raw, dict):
        raise ManifestError(f"{where}.ports: expected a mapping")
    ports = {str(name): _parse_port(name, spec, f"{where}.ports.{name}")
             for name, spec in ports_raw.items()}

    units_raw = raw.get("units")
    if units_raw is None and ports:
        units_raw = []
    if not isinstance(units_raw, list) or (not units_raw and not ports):
        raise ManifestError(f"{where}: needs 'units' (or 'ports')")
    units = [_parse_unit(u, f"{where}.units[{i}]", defaults)
             for i, u in enumerate(units_raw)]

    names = [u.name for u in units]
    dupes = sorted({n for n in names if names.count(n) > 1})
    if dupes:
        raise ManifestError(f"{where}: duplicate unit name(s) {dupes}")

    # Route-form links resolve to bus+address before the edge checks,
    # so identity comparison sees real addresses.
    cfg = SuiteConfig(name=str(suite_raw.get("name", "")), units=units,
                      ports=ports)
    _resolve_link_refs(cfg, where)

    # One route reaches one board, so an edge belongs to exactly one unit.
    edge_owner: dict = {}
    for u in units:
        for link in u.links:
            ident = link.identity()
            if ident in edge_owner:
                raise ManifestError(
                    f"{where}: units {edge_owner[ident]!r} and {u.name!r} "
                    f"both declare {link.describe()} — one route reaches "
                    f"one board; merge the entries into one unit with "
                    f"several links")
            edge_owner[ident] = u.name

    # One board is one unit: two units pinning the same silicon would fight
    # on every apply. A multi-homed board is one unit with several links.
    pinned: dict = {}
    for u in units:
        if u.serial:
            if u.serial in pinned:
                raise ManifestError(
                    f"{where}: units {pinned[u.serial]!r} and {u.name!r} "
                    f"pin the same serial {u.serial} — one board is one "
                    f"unit; merge their links into one entry")
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
    # A port that names no bus is one of the host's, by name; a unit
    # that rides one of its links takes the same bus.
    if any(port.bus is None for port in cfg.ports.values()):
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
    return cfg


def hardware_path(manifest: str) -> str:
    """The generated wiring file that belongs to a manifest."""
    return os.path.join(os.path.dirname(manifest) or ".", "hardware.yaml")
