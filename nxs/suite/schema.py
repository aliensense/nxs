"""suite.yaml manifest: parse and validate the declared suite.

The manifest is units-centric — a unit is one board (a graph node),
its `links` are the routes that reach it (edges). Each unit bundles
its links, an optional firmware pin, an optional serial pin, and the
sensor panel to deploy. Config, firmware, and identity live on the
node; only reachability lives on an edge — management flows over the
first link that answers, in declared order. Validation is strict:
unknown keys and malformed values fail with the YAML path spelled
out, so a typo never half-applies.
"""
import os
import re
from dataclasses import dataclass, field
from typing import List, Optional

import yaml

TRANSPORTS = ("i2c", "cyphal-can", "cyphal-serial", "mock")
MODULES = ("nxs",)

_TOP_KEYS = {"suite", "defaults", "units"}
_SUITE_KEYS = {"name"}
_DEFAULTS_KEYS = {"firmware"}
_UNIT_KEYS = {"name", "module", "links", "serial", "firmware", "sensors",
              "egress"}
_EGRESS_KEYS = {"decimation", "subjects"}
# The SI-subject tokens the per-subject factors address (SubjectBucket
# names; the same tokens name the Cyphal decimation.<subject> registers).
EGRESS_SUBJECTS = ("acceleration", "angular_velocity", "magnetic_field",
                   "temperature", "pressure", "scalar")
_SENSOR_KEYS = {"driver", "config"}
_LINK_KEYS = {
    "i2c": {"required": {"bus", "address"}, "optional": set()},
    "cyphal-can": {"required": {"iface", "node_id"}, "optional": set()},
    "cyphal-serial": {"required": {"port"}, "optional": {"baud"}},
    "mock": {"required": set(), "optional": set()},
}


class ManifestError(ValueError):
    """A manifest that failed validation; the message names the YAML path."""


def stable_path(path: Optional[str]) -> Optional[str]:
    """A device path resolved to its underlying node, so a stable udev
    alias (`/dev/i2c-cam1`, `/dev/serial/by-id/...`) and the kernel's
    enumerated node name the same link."""
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
        """The fields that address a device, for matching a manually
        flag-addressed target against declared units (baud and other
        tuning fields deliberately excluded). Device paths are resolved,
        so an alias-declared link and an enumeration-named one compare
        equal."""
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


@dataclass
class SuiteConfig:
    name: str = ""
    units: List[UnitSpec] = field(default_factory=list)


def parse_version(text: str) -> tuple:
    """'1.2' / '1.2.3' → a 3-tuple with missing parts zeroed."""
    parts = str(text).split(".")
    if not 2 <= len(parts) <= 3 or not all(p.isdigit() for p in parts):
        raise ValueError(f"bad version {text!r} (expected MAJOR.MINOR[.PATCH])")
    nums = [int(p) for p in parts] + [0]
    return tuple(nums[:3])


def normalize_serial(text, where: str) -> str:
    """Lowercased 24-hex-digit UID96, separators stripped. Only YAML
    strings are accepted: a bare `<digits>e<digits>` UID resolves as
    scientific notation in float-resolving parsers and silently loses
    digits, so a numeric value here means the manifest needs quotes."""
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


def _require_keys(mapping: dict, allowed: set, where: str):
    if not isinstance(mapping, dict):
        raise ManifestError(f"{where}: expected a mapping, got {type(mapping).__name__}")
    unknown = set(mapping) - allowed
    if unknown:
        raise ManifestError(
            f"{where}: unknown key(s) {sorted(unknown)} (allowed: {sorted(allowed)})")


def _parse_int(value, where: str) -> int:
    if isinstance(value, bool):
        raise ManifestError(f"{where}: expected an integer, got a boolean")
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        try:
            return int(value, 0)
        except ValueError:
            pass
    raise ManifestError(f"{where}: expected an integer (any base), got {value!r}")


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
        link.bus = str(raw["bus"])
        link.address = _parse_int(raw["address"], f"{where}.address")
        if not 0x08 <= link.address <= 0x77:
            raise ManifestError(
                f"{where}.address: 0x{link.address:X} is not a usable "
                f"7-bit I2C address (0x08-0x77)")
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
    if "driver" not in raw or not isinstance(raw["driver"], str):
        raise ManifestError(f"{where}: sensor needs a 'driver' name")
    # Marketing names hyphenate (neo-m9n); module files underscore.
    driver = raw["driver"].replace("-", "_")
    # Driver names become module names and filesystem paths; the token
    # shape rules out traversal (`../evil`) by construction.
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", driver):
        raise ManifestError(
            f"{where}.driver: {raw['driver']!r} is not a module name "
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
    # unmanaged; an explicit `sensors: []` is enforced — converge to an
    # empty store.
    if "sensors" not in raw or raw["sensors"] is None:
        sensors = None
    else:
        sensors_raw = raw["sensors"]
        if not isinstance(sensors_raw, list):
            raise ManifestError(f"{where}.sensors: expected a list")
        sensors = [_parse_sensor(s, f"{where}.sensors[{i}]")
                   for i, s in enumerate(sensors_raw)]
    # A unit is one physical link, so a driver is a stable identity within
    # its panel — drift/freeze match the active driver by name. A duplicate
    # would shadow the second entry, so reject it here.
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

    return UnitSpec(name=name, module=module, links=links, sensors=sensors,
                    egress=egress, serial=serial, firmware=firmware)


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

    units_raw = raw.get("units")
    if not isinstance(units_raw, list) or not units_raw:
        raise ManifestError(f"{where}: needs a non-empty 'units' list")
    units = [_parse_unit(u, f"{where}.units[{i}]", defaults)
             for i, u in enumerate(units_raw)]

    names = [u.name for u in units]
    dupes = sorted({n for n in names if names.count(n) > 1})
    if dupes:
        raise ManifestError(f"{where}: duplicate unit name(s) {dupes}")

    # One route reaches one board, so an edge belongs to exactly one
    # unit. Mock links carry no address — their identity is degenerate,
    # so collisions between them are meaningless.
    edge_owner: dict = {}
    for u in units:
        for link in u.links:
            ident = link.identity()
            if ident == ("mock",):
                continue
            if ident in edge_owner:
                raise ManifestError(
                    f"{where}: units {edge_owner[ident]!r} and {u.name!r} "
                    f"both declare {link.describe()} — one route reaches "
                    f"one board; merge the entries into one unit with "
                    f"several links")
            edge_owner[ident] = u.name

    # One board is one unit: two units pinning the same silicon would
    # converge one store with two intents, fighting on every apply. A
    # multi-homed board is one unit with several links.
    pinned: dict = {}
    for u in units:
        if u.serial:
            if u.serial in pinned:
                raise ManifestError(
                    f"{where}: units {pinned[u.serial]!r} and {u.name!r} "
                    f"pin the same serial {u.serial} — one board is one "
                    f"unit; merge their links into one entry")
            pinned[u.serial] = u.name

    return SuiteConfig(name=str(suite_raw.get("name", "")), units=units)


def load_suite_config(path: str) -> SuiteConfig:
    """Load and validate the manifest at `path`. Every read/parse
    failure — unreadable file, bad encoding, malformed YAML — surfaces
    as `ManifestError`, so callers handle one exception type."""
    try:
        with open(path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
    except OSError as e:
        raise ManifestError(f"{path}: {e.strerror or e}") from None
    except (yaml.YAMLError, UnicodeDecodeError) as e:
        raise ManifestError(f"{path}: not valid YAML ({e})") from None
    if raw is None:
        raise ManifestError(f"{path}: empty manifest")
    return parse_suite_config(raw, where=path)
