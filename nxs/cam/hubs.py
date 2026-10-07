# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The hubs a host knows. A hub is a directory: its manifest (``hub.yaml``),
the facts of its deserializer and serializer, their programs, a ``flows``
module or package that composes a port from them, and the port's default
wiring. The product's hub ships inside the wheel (``nxs/hubs``). Under
``nxs --experimental`` the roots on ``$NXS_CAM_HUBS`` come first; without
the flag they are not read. A port with no hub takes the connector
(`Connector`): the tool's own flows for one sensor wired straight to the
host. A hub hands out every installed cam personality beside its own
chips (`cam_personalities`): no hub owns a head."""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Dict, List, Optional

import yaml

from . import cam_personalities
from .contracts import Topology
from .descriptors import Descriptor, to_int

HUBS_ENV = "NXS_CAM_HUBS"
#: The hubs of a source tree, beside `nxs/`.
TREE_DIR_NAME = "hubs"
#: The shipped hub's home inside the wheel.
PACKAGE_HUBS_DIR = Path(__file__).resolve().parents[1] / "hubs"
MANIFEST = "hub.yaml"

_HUB_KEYS = {"hub", "api", "chips", "flows", "flows_for", "topology", "golden"}
#: The hub manifest API this loader speaks; a hub declaring another API
#: is refused by name.
HUB_API = 1
_GOLDEN_KEYS = {"flow", "files"}

#: What a host with no hub is told: the wheel carries it.
_NO_HUB_HINT = ("This nxs ships its hub inside the wheel; a source checkout "
                f"finds its tree, a bench sets ${HUBS_ENV} under --experimental.")


class HubError(RuntimeError):
    """A hub is missing, malformed, or incomplete; or a cam personality is."""


def search_paths() -> List[Path]:
    """The hub search path, most specific first. ``$NXS_CAM_HUBS`` is read
    only under ``--experimental``, and then every entry must exist: a
    developer never runs the shipped facts believing they run the
    experimental ones."""
    from nxs import experimental

    paths: List[Path] = []
    if experimental.enabled():
        for entry in os.environ.get(HUBS_ENV, "").split(":"):
            entry = entry.strip()
            if not entry:
                continue
            root = Path(entry)
            if not root.is_dir():
                raise HubError(
                    f"${HUBS_ENV} names {root}, which does not exist; "
                    f"unset it to run the shipped hub")
            paths.append(root)
    if PACKAGE_HUBS_DIR.is_dir():
        paths.append(PACKAGE_HUBS_DIR)
    dev = Path(__file__).resolve().parents[2] / TREE_DIR_NAME
    if dev.is_dir():
        paths.append(dev)
    return paths


class Hub:
    """One discovered hub: its own chips, its flows, and every installed
    cam personality."""

    def __init__(self, root: Path) -> None:
        self._root = root
        raw = yaml.safe_load((root / MANIFEST).read_text())
        if not isinstance(raw, dict):
            raise HubError(f"{root / MANIFEST}: not a mapping")
        unknown = set(raw) - _HUB_KEYS
        if unknown:
            raise HubError(f"{root / MANIFEST}: unknown keys {sorted(unknown)}")
        for key in ("hub", "api", "chips", "flows", "flows_for"):
            if key not in raw:
                raise HubError(f"{root / MANIFEST}: missing {key!r}")
        _check_api(root / MANIFEST, raw)
        golden = raw.get("golden")
        if golden is not None:
            unknown = set(golden) - _GOLDEN_KEYS
            if unknown or "files" not in golden:
                raise HubError(f"{root / MANIFEST}: bad golden section")
        # After the checks with the sharper messages: the schema judges
        # the rest (types, patterns, non-empty lists).
        _check_shape(root / MANIFEST, raw)
        self._raw = raw
        self._modules: Dict[str, ModuleType] = {}
        # Family surfaces bound to the hub's own chips.
        self._bound: Dict[str, Any] = {}
        self._descriptors: Dict[str, Descriptor] = {}
        self._chips: List[str] = [str(c) for c in raw["chips"]]
        for chip in self._chips:
            if not (root / chip / f"{chip}.yaml").exists():
                raise HubError(f"{root / MANIFEST}: chip {chip!r} has no {chip}/{chip}.yaml")
        # Facts a unit served (no directory behind them), by name.
        self._adopted: Dict[str, Descriptor] = {}

    @property
    def name(self) -> str:
        return str(self._raw["hub"])

    @property
    def root(self) -> Path:
        return self._root

    @property
    def api(self) -> int:
        return int(self._raw["api"])

    @property
    def overlays(self) -> Dict[str, str]:
        """Cam personality -> the experimental overlay layered over it
        (empty without --experimental: the shipped facts layer nothing)."""
        return cam_personalities.overlays()

    @property
    def chips(self) -> List[str]:
        """The hub's own chips, then the facts units served it."""
        return list(self._chips) + [c for c in self._adopted if c not in self._chips]

    @property
    def flows_for(self) -> List[str]:
        return [str(c) for c in self._raw["flows_for"]]

    def adopt_descriptor(self, descriptor: Descriptor) -> None:
        """Facts a unit served join this hub's view under their name and
        compatible, ahead of the registry's: `descriptor()`, `sensors()`
        and `chip_module()` answer for them. Nothing is written to disk."""
        self._adopted[descriptor.name] = descriptor
        for key in [k for k in self._bound
                    if k == descriptor.name or k.startswith(f"{descriptor.name}@")]:
            self._bound.pop(key)

    def _own(self, name_or_compatible: str) -> Optional[str]:
        """The hub's own chip directory name for `name_or_compatible`, or None."""
        if name_or_compatible in self._chips:
            return name_or_compatible
        for chip in self._chips:
            if self._own_descriptor(chip).compatible == name_or_compatible:
                return chip
        return None

    def _own_descriptor(self, chip: str) -> Descriptor:
        descriptor = self._descriptors.get(chip)
        if descriptor is None:
            descriptor = Descriptor(self._root / chip, None)
            self._descriptors[chip] = descriptor
        return descriptor

    def _adopted_for(self, name_or_compatible: str) -> Optional[Descriptor]:
        if name_or_compatible in self._adopted:
            return self._adopted[name_or_compatible]
        return next((d for d in self._adopted.values()
                     if d.compatible == name_or_compatible), None)

    def is_extension(self, name: str) -> bool:
        """Whether a cam personality extends the product's set (the
        registry's judgement); the hub's own chips and the facts units
        served are the product's."""
        if name in self._chips or name in self._adopted:
            return False
        return cam_personalities.registry().is_extension(name)

    def chip_source(self, name_or_compatible: str) -> Optional[Path]:
        """The `<chip>.py` beside a chip's facts (its program class or
        physics module), or None when it ships none or a unit served it."""
        own = self._own(name_or_compatible)
        if own is not None:
            path = self._root / own / f"{own}.py"
            return path if path.exists() else None
        if self._adopted_for(name_or_compatible) is not None:
            return None
        return cam_personalities.source(name_or_compatible)

    def sensors(self) -> List[str]:
        """Every cam personality a link of this hub may carry: the installed
        ones in registry order, then the facts units served."""
        found = list(cam_personalities.names())
        for name, descriptor in self._adopted.items():
            if name not in found and descriptor.role == cam_personalities.SENSOR_ROLE:
                found.append(name)
        return found

    def descriptor(self, name_or_compatible: str,
                   lines: Optional[Dict[str, int]] = None) -> Descriptor:
        """The facts of one of the hub's own chips or of a cam personality,
        by directory name or compatible (cached; a cam personality's
        experimental overlay merged under the flag); with `lines` (mode ->
        HMAX), the view of a port that runs them (`Descriptor.at_lines`)."""
        own = self._own(name_or_compatible)
        if own is not None:
            return self._own_descriptor(own).at_lines(lines)
        adopted = self._adopted_for(name_or_compatible)
        if adopted is not None:
            return adopted.at_lines(lines)
        try:
            return cam_personalities.find(name_or_compatible, lines)
        except cam_personalities.RegistryError as exc:
            raise HubError(str(exc)) from None

    def shipped_descriptor(self, name_or_compatible: str) -> Descriptor:
        """The facts as the product ships them: the directory's yaml alone,
        never an experimental overlay, what a personality compiles from."""
        own = self._own(name_or_compatible)
        if own is not None:
            return Descriptor(self._root / own, None)
        adopted = self._adopted_for(name_or_compatible)
        if adopted is not None:
            return adopted
        try:
            return cam_personalities.shipped(name_or_compatible)
        except cam_personalities.RegistryError as exc:
            raise HubError(str(exc)) from None

    def _load_module(self, label: str, path: Path) -> ModuleType:
        if label in self._modules:
            return self._modules[label]
        # Parent stubs so the dotted reserved namespace resolves anywhere
        # the interpreter looks parents up (dataclass repr, pickling, ...).
        for parent in ("nxs_cam_hub", f"nxs_cam_hub.{self.name}"):
            if parent not in sys.modules:
                stub = ModuleType(parent)
                stub.__path__ = []  # mark as package
                sys.modules[parent] = stub
        module_name = f"nxs_cam_hub.{self.name}.{label}"
        if path.is_dir():
            # A package: its submodules import against this hub's handle,
            # so an earlier hub's stay out of the way.
            spec = importlib.util.spec_from_file_location(
                module_name, path / "__init__.py",
                submodule_search_locations=[str(path)])
        else:
            spec = importlib.util.spec_from_file_location(module_name, path)
        if spec is None or spec.loader is None:
            raise HubError(f"cannot load {path}")
        for stale in [n for n in sys.modules if n.startswith(module_name + ".")]:
            del sys.modules[stale]
        module = importlib.util.module_from_spec(spec)
        module.HUB = self  # sibling access for the hub's code
        sys.modules[module_name] = module
        try:
            spec.loader.exec_module(module)
        except Exception as exc:
            sys.modules.pop(module_name, None)
            raise HubError(f"error loading {path}: {exc}") from exc
        self._modules[label] = module
        return module

    def chip_module(self, name_or_compatible: str,
                    lines: Optional[Dict[str, int]] = None) -> Optional[Any]:
        """A chip's physics surface (cached): one of the hub's own chips
        binds the family its facts name or loads its physics module; a cam
        personality's comes from the registry; facts a unit served bind
        their family, the behaviour running on the unit. With `lines`, the
        family binds to the view of a port that runs them."""
        own = self._own(name_or_compatible)
        if own is not None:
            descriptor = self._own_descriptor(own)
            if (descriptor.raw("meta") or {}).get("chip"):
                return self._bind(descriptor, lines)
            path = self._root / own / f"{own}.py"
            return self._load_module(own, path) if path.exists() else None
        adopted = self._adopted_for(name_or_compatible)
        if adopted is not None:
            return cam_personalities.registry().bind(adopted, lines)
        try:
            return cam_personalities.module(name_or_compatible, lines)
        except cam_personalities.RegistryError as exc:
            raise HubError(str(exc)) from None

    def _bind(self, descriptor: Descriptor,
              lines: Optional[Dict[str, int]] = None) -> Any:
        key = descriptor.name
        view = descriptor.at_lines(lines)
        if view is not descriptor:
            descriptor = view
            key = f"{key}@{sorted(lines.items())}"
        if key not in self._bound:
            from .chips import bind
            self._bound[key] = bind(descriptor)
        return self._bound[key]

    def flows(self) -> ModuleType:
        """Load (cached) the hub's flow-assembly module or package."""
        return self._load_module("flows", self._root / str(self._raw["flows"]))

    def topology_path(self) -> Optional[Path]:
        """The hub's default wiring file, if declared."""
        rel = self._raw.get("topology")
        return self._root / str(rel) if rel else None

    def golden(self) -> Optional[tuple[str, List[Path]]]:
        """Golden reference configs for plan byte-diffs, if declared."""
        golden = self._raw.get("golden")
        if not golden:
            return None
        files = [self._root / str(f) for f in golden["files"]]
        return str(golden.get("flow", "dual")), files


#: The name of a port without a hub: the HUB knob's value and the
#: connector's own flows.
CONNECTOR = "connector"


class Connector(Hub):
    """A port without a hub: one sensor wired straight to the host, its
    NXS unit on the same bus. It carries no chip of its own and no carrier
    knowledge: its flows are the tool's (`nxs.cam.direct`), its sensors
    the installed cam personalities and the facts the units serve."""

    def __init__(self) -> None:
        self._root = None
        self._raw = {"hub": CONNECTOR, "api": HUB_API, "chips": [],
                     "flows": None, "flows_for": []}
        self._modules = {}
        self._bound = {}
        self._descriptors = {}
        self._chips = []
        self._adopted = {}

    def flows(self) -> ModuleType:
        from . import direct
        return direct

    def topology_path(self) -> Optional[Path]:
        return None


_cache: Optional[List[Hub]] = None
_connector: Optional[Connector] = None


def reset_cache() -> None:
    """Forget discovered hubs and the cam personalities (test seam; also
    after env changes)."""
    global _cache, _connector
    _cache = None
    _connector = None
    cam_personalities.reset_cache()


def prime(roots: List[Path]) -> List[Hub]:
    """Discover `roots` and make them the search result: the hub a build
    compiles is the hub its program classes compose from, whatever the
    interpreter's own search path holds."""
    global _cache
    _cache = discover(roots)
    return list(_cache)


def connector() -> Connector:
    """The connector: the hub-less port's flows, with the installed cam
    personalities."""
    global _connector
    if _connector is None:
        _connector = Connector()
    return _connector


def all_hubs() -> List[Hub]:
    """Every hub a link may sit behind: the discovered ones, then the
    connector."""
    return discover() + [connector()]


def _check_shape(path: Path, raw: Dict[str, Any]) -> None:
    """The hub schema is the loader's first gate: types, patterns, and
    non-empty lists are refused here naming the file."""
    from nxs import schemas

    problems = schemas.findings(raw, schemas.HUB, where=str(path))
    if problems:
        raise HubError("; ".join(problems))


def _check_api(path: Path, raw: Dict[str, Any]) -> None:
    """Refuse a manifest whose declared hub API is not the one this loader
    speaks (every manifest declares one; the callers require the key)."""
    declared = raw.get("api", HUB_API)
    try:
        declared = int(declared)
    except (TypeError, ValueError):
        raise HubError(f"{path}: api must be an integer, got {declared!r}")
    if declared != HUB_API:
        raise HubError(f"{path}: declares hub api {declared}; this nxs speaks api {HUB_API}")


def hub_classes(paths: Optional[List[Path]] = None) -> Dict[int, str]:
    """Deserializer device id -> compatible, from every discovered hub's
    deserializer facts (the chips its flows serve). Chip identity is the
    hub's data: the tool names no silicon of its own."""
    classes: Dict[int, str] = {}
    for hub in discover(paths):
        for compatible in hub.flows_for:
            try:
                d = hub.descriptor(compatible)
                dev_id = d.raw("meta").get("device_id")
            except Exception:
                continue
            if dev_id is not None:
                classes[to_int(dev_id)] = d.compatible
    return classes


def discover(paths: Optional[List[Path]] = None) -> List[Hub]:
    """Find hubs on the search path (cached for the default path): a root
    that holds a manifest, or whose children do."""
    global _cache
    if paths is None and _cache is not None:
        return list(_cache)
    roots = paths if paths is not None else search_paths()
    found: List[Hub] = []
    seen: set[Path] = set()
    for root in roots:
        candidates: List[Path] = []
        if (root / MANIFEST).exists():
            candidates.append(root)
        elif root.is_dir():
            candidates.extend(child for child in sorted(root.iterdir())
                              if (child / MANIFEST).exists())
        for candidate in candidates:
            real = candidate.resolve()
            if real in seen:
                continue
            seen.add(real)
            found.append(Hub(candidate))
    if paths is None:
        _cache = list(found)
    return found


def find(name: str) -> Optional[Hub]:
    """A hub by name: a discovered one, or the connector."""
    for hub in all_hubs():
        if hub.name == name:
            return hub
    return None


def of_chip(chip: str) -> Hub:
    """The discoverable hub whose own chips include `chip`."""
    for hub in discover():
        if chip in hub.chips:
            return hub
    raise HubError(f"no discoverable hub ships {chip}")


def for_topology(topology: Topology) -> Hub:
    """The hub whose flows cover this topology's deserializer (the
    connector for a port with none), with the facts its links' units served
    adopted from the state directory's cache (never the bus) wherever no
    installed cam personality describes the sensor; HubError naming the
    search path when no hub covers the deserializer."""
    candidates = [connector()] if topology.is_direct else discover()
    for hub in candidates:
        if topology.is_direct or topology.des_compatible in hub.flows_for:
            # The store is read here: one of another release refuses the
            # verb by name before any bus is touched.
            try:
                cam_personalities.names()
            except cam_personalities.RegistryError as exc:
                raise HubError(str(exc)) from None
            _adopt_cached_units(hub, topology)
            return hub
    searched = ", ".join(str(p) for p in search_paths())
    raise HubError(f"no hub serves {topology.des_compatible!r}; "
                   f"searched: {searched}. {_NO_HUB_HINT}")


def _adopt_cached_units(hub: Hub, topology: Topology) -> None:
    """Adopt the cached facts of every link that carries a unit, unless a cam
    personality directory (the tree's or an installed one) describes that
    sensor: a unit's facts outrank a sealed image's, never a source's."""
    from .unit_source import cached_descriptor

    for link in topology.links:
        if not link.nxs_units:
            continue
        descriptor = cached_descriptor(topology, link)
        if descriptor is None:
            continue
        if hub._own(descriptor.compatible) is not None:
            continue
        if cam_personalities.registry().directory(descriptor.compatible) is None:
            hub.adopt_descriptor(descriptor)


def sensor_address(hub: Hub, link) -> int:
    """Where the host reaches a link's sensor: the address the port record
    says the link's serializer translates for it while the port is up,
    else the link's declared address, else its facts' own (meta.i2c_addr;
    0x1A when the facts are silent)."""
    host = getattr(link, "host_addr", None)
    if host is not None:
        return int(host)
    return native_sensor_address(hub, link)


def native_sensor_address(hub: Hub, link) -> int:
    """Where a link's sensor answers on its own bus, whatever the host
    reaches it at: the link's declared address, else its facts' own
    (meta.i2c_addr; 0x1A when the facts are silent)."""
    declared = getattr(link, "sensor_addr", None)
    if declared is not None:
        return int(declared)
    if getattr(link, "sensor_compatible", None) is None:
        return 0x1A
    try:
        meta = hub.descriptor(link.sensor_compatible).raw("meta") or {}
    except HubError:
        return 0x1A
    return to_int(meta.get("i2c_addr", 0x1A))


def load_descriptor(name_or_compatible: str) -> Descriptor:
    """The facts of a cam personality or of any hub's chip; HubError when
    nothing installed carries them."""
    try:
        return cam_personalities.find(name_or_compatible)
    except cam_personalities.RegistryError:
        pass
    for hub in all_hubs():
        try:
            return hub.descriptor(name_or_compatible)
        except HubError:
            continue
    searched = ", ".join(str(p) for p in cam_personalities.search_paths() + search_paths())
    raise HubError(f"nothing installed describes {name_or_compatible!r}; "
                   f"searched: {searched}. {_NO_HUB_HINT}")
