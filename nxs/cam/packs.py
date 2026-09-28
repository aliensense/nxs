# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Descriptor-pack discovery. A pack is a directory of per-chip descriptors,
init blobs, and a ``flows`` module or package. The product's pack ships inside the wheel
(``nxs/packs``); ``extends:`` (a personality store) adds chips. Under
``nxs --experimental`` a pack root on ``$NXS_CAM_DESCRIPTORS`` comes first
and the pack's experimental overlays (``descriptors-experimental/<pack>/experimental.yaml``
beside the tree, or ``$NXS_CAM_EXPERIMENTAL``) are layered over its descriptors;
without the flag neither is read. A port with no deserializer takes the tool's
own ``direct`` pack (`DirectPack`), which an extension may name like any other."""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Dict, List, Optional

import yaml

from .contracts import Topology
from .descriptors import Descriptor, to_int

PACK_ENV = "NXS_CAM_DESCRIPTORS"
EXPERIMENTAL_ENV = "NXS_CAM_EXPERIMENTAL"
#: The experimental overlays' directory beside a source tree's ``descriptors``.
EXPERIMENTAL_DIR_NAME = "descriptors-experimental"
#: The shipped pack's home inside the wheel.
PACKAGE_PACK_DIR = Path(__file__).resolve().parents[1] / "packs"

_PACK_KEYS = {"pack", "api", "chips", "flows", "flows_for", "topology",
              "golden"}
_EXPERIMENTAL_KEYS = {"experimental", "api", "overlays", "chips"}
_EXTENSION_KEYS = {"extends", "api", "chips"}
#: The descriptor pack API this loader speaks; a pack declaring another API
#: is refused by name.
PACK_API = 1
_GOLDEN_KEYS = {"flow", "files"}


#: What a host with no pack is told: the wheel carries it.
_NO_PACK_HINT = ("This nxs ships its pack inside the wheel; a source checkout "
                 f"finds its tree, a bench sets ${PACK_ENV} under --experimental.")


class PackError(RuntimeError):
    """A descriptor pack is missing, malformed, or incomplete."""


def search_paths() -> List[Path]:
    """The pack search path, most specific first. ``$NXS_CAM_DESCRIPTORS``
    is read only under ``--experimental``, and then every entry must exist:
    a developer never runs the shipped facts believing they run the
    experimental ones."""
    from nxs import experimental

    paths: List[Path] = []
    env = os.environ.get(PACK_ENV, "")
    if experimental.enabled():
        for entry in env.split(":"):
            entry = entry.strip()
            if not entry:
                continue
            root = Path(entry)
            if not root.is_dir():
                raise PackError(
                    f"${PACK_ENV} names {root}, which does not exist; "
                    f"unset it to run the shipped pack")
            paths.append(root)
    # Camera personalities installed by `nxs personality install` are extensions of
    # the installed pack, one directory each; the store's earlier location
    # is still read.
    from nxs.suite import EARLIER_PERSONALITY_DIR, PERSONALITY_DIR
    paths.append(Path(PERSONALITY_DIR))
    paths.append(Path(EARLIER_PERSONALITY_DIR))
    if PACKAGE_PACK_DIR.is_dir():
        paths.append(PACKAGE_PACK_DIR)
    dev = Path(__file__).resolve().parents[2] / "descriptors"
    if dev.is_dir():
        paths.append(dev)
    return paths


def experimental_root(pack_root: Path, pack_name: str) -> Optional[Path]:
    """Where a pack's experimental overlays would live: ``$NXS_CAM_EXPERIMENTAL/<pack>``,
    else ``<tree>/descriptors-experimental/<pack>`` beside the pack's tree; None
    when neither holds a ``experimental.yaml``. The wheel's pack has no tree and
    therefore no experimental directory to find."""
    candidates: List[Path] = []
    env = os.environ.get(EXPERIMENTAL_ENV, "").strip()
    if env:
        candidates.append(Path(env) / pack_name)
    candidates.append(pack_root.parent.parent / EXPERIMENTAL_DIR_NAME / pack_name)
    for root in candidates:
        if (root / "experimental.yaml").exists():
            return root
    return None


class Pack:
    """One discovered descriptor pack."""

    def __init__(self, root: Path) -> None:
        self._root = root
        raw = yaml.safe_load((root / "pack.yaml").read_text())
        if not isinstance(raw, dict):
            raise PackError(f"{root / 'pack.yaml'}: not a mapping")
        unknown = set(raw) - _PACK_KEYS
        if unknown:
            raise PackError(
                f"{root / 'pack.yaml'}: unknown keys {sorted(unknown)}"
            )
        for key in ("pack", "api", "chips", "flows", "flows_for"):
            if key not in raw:
                raise PackError(f"{root / 'pack.yaml'}: missing {key!r}")
        _check_api(root / "pack.yaml", raw)
        golden = raw.get("golden")
        if golden is not None:
            unknown = set(golden) - _GOLDEN_KEYS
            if unknown or "files" not in golden:
                raise PackError(
                    f"{root / 'pack.yaml'}: bad golden section"
                )
        # After the checks with the sharper messages: the schema judges
        # the rest (types, patterns, non-empty lists).
        _check_shape(root / "pack.yaml", raw)
        self._raw = raw
        self._modules: Dict[str, ModuleType] = {}
        # Family surfaces bound to a descriptor; dropped with the descriptors
        # when an extension changes what a chip name resolves to.
        self._bound: Dict[str, Any] = {}
        self._descriptors: Dict[str, Descriptor] = {}
        # chip dir name -> the directory holding it (the pack root, or
        # an extension's root for the sensors it contributes).
        self._chip_roots: Dict[str, Path] = {
            str(c): root for c in raw["chips"]}
        self._extensions: List[Path] = []
        # Descriptors a unit served (no directory behind them), by name.
        self._adopted: List[str] = []
        # chip dir name -> the experimental overlay directory layered over it
        # (attached only under --experimental).
        self._experimental: Dict[str, Path] = {}

    def attach_experimental(self, root: Path, overlays: Dict[str, str]) -> None:
        """Layer an experimental directory's overlays over this pack's chips: a
        measured value, a jump threshold or an operating point the product
        never ships. Only `discover()` calls this, only under the flag."""
        for chip, overlay in overlays.items():
            chip, overlay = str(chip), str(overlay)
            over_dir = root / overlay
            if not (over_dir / f"{overlay}.yaml").exists():
                raise PackError(
                    f"{root / 'experimental.yaml'}: overlay {overlay!r} for {chip!r} "
                    f"has no {overlay}/{overlay}.yaml")
            if chip not in self._chip_roots:
                raise PackError(
                    f"{root / 'experimental.yaml'}: overlay for {chip!r}, which pack "
                    f"{self.name!r} does not carry (chips: {self.chips})")
            self._experimental[chip] = over_dir
        self._descriptors.clear()
        self._bound.clear()

    def extend(self, root: Path, chips: List[str]) -> None:
        """Attach an extension: its chip directories join this pack (an extension
        seen first, on a more specific path, wins)."""
        for chip in chips:
            chip = str(chip)
            if not (root / chip / f"{chip}.yaml").exists():
                raise PackError(
                    f"{root / 'pack.yaml'}: chip {chip!r} has no "
                    f"{chip}/{chip}.yaml")
            if chip in self._chip_roots and self._chip_roots[chip] != root:
                if self._chip_roots[chip] in self._extensions:
                    continue        # an earlier (more specific) extension
            self._chip_roots[chip] = root
        self._extensions.append(root)
        self._descriptors.clear()
        self._bound.clear()

    def is_extension(self, chip: str) -> bool:
        """Whether a chip joined from an extension directory rather than
        the pack's own tree."""
        return self._chip_roots.get(str(chip)) in self._extensions

    @property
    def name(self) -> str:
        return str(self._raw["pack"])

    @property
    def root(self) -> Path:
        return self._root

    @property
    def api(self) -> int:
        return int(self._raw["api"])

    @property
    def overlays(self) -> Dict[str, str]:
        """Chip directory -> the experimental overlay directory attached to it
        (empty without --experimental: the shipped pack layers nothing)."""
        return {chip: path.name for chip, path in self._experimental.items()}

    @property
    def chips(self) -> List[str]:
        return list(self._chip_roots) + [
            c for c in self._adopted if c not in self._chip_roots]

    def adopt_descriptor(self, descriptor: Descriptor) -> None:
        """A descriptor a unit served joins the pack under its name and
        compatible: `descriptor()`, `sensors()`, and `chip_module()` (the
        family bound to it) answer for it. Nothing is written to disk."""
        name = descriptor.name
        self._descriptors[name] = descriptor
        self._descriptors[descriptor.compatible] = descriptor
        for key in [k for k in self._bound if k == name or k.startswith(f"{name}@")]:
            self._bound.pop(key)
        if name not in self._adopted:
            self._adopted.append(name)

    def chip_source(self, name_or_compatible: str) -> Optional[Path]:
        """The `<chip>.py` beside a chip's descriptor (its behaviour class or
        physics module), or None when the chip ships none or a unit served it."""
        known = self._descriptors.get(name_or_compatible)
        if known is not None and known.from_unit:
            return None
        chip_dir = self._chip_dir(name_or_compatible)
        path = chip_dir / f"{chip_dir.name}.py"
        return path if path.exists() else None

    def sensors(self) -> List[str]:
        """The pack's sensor chips (role SEN), pack order first, then
        extensions in discovery order."""
        found = []
        for chip in self.chips:
            try:
                if self.descriptor(chip).role == "SEN":
                    found.append(chip)
            except Exception:
                continue
        return found

    @property
    def flows_for(self) -> List[str]:
        return [str(c) for c in self._raw["flows_for"]]

    def _chip_dir(self, name_or_compatible: str) -> Path:
        if name_or_compatible in self._chip_roots:
            return self._chip_roots[name_or_compatible] / name_or_compatible
        for chip in self._chip_roots:
            try:
                if self.descriptor(chip).compatible == name_or_compatible:
                    return self._chip_roots[chip] / chip
            except Exception:
                continue
        known = self._descriptors.get(name_or_compatible)
        if known is not None and known.from_unit:
            raise PackError(
                f"pack {self.name!r}: {known.name!r} was served by a unit and "
                f"has no directory")
        raise self._no_chip(name_or_compatible)

    def _no_chip(self, name: str) -> PackError:
        return PackError(f"pack {self.name!r} has no chip {name!r} (chips: {self.chips})")

    def descriptor(self, name_or_compatible: str, cameras: Optional[int] = None,
                   csi_lanes: Optional[int] = None,
                   pair_lines: Optional[Dict[str, int]] = None) -> Descriptor:
        """A chip descriptor by dir name or compatible (cached; the experimental
        overlay merged under the flag); with a camera count, that count's
        view, `pair_lines` the lines a pair leaves its one-camera modes."""
        descriptor = self._descriptors.get(name_or_compatible)
        if descriptor is None:
            root = self._chip_dir(name_or_compatible)
            descriptor = Descriptor(root, self._experimental.get(root.name))
            self._descriptors[name_or_compatible] = descriptor
            self._descriptors[descriptor.compatible] = descriptor
        if cameras is None or csi_lanes is None:
            return descriptor
        return descriptor.for_cameras(cameras, int(csi_lanes), pair_lines)

    def shipped_descriptor(self, name_or_compatible: str) -> Descriptor:
        """The chip's descriptor as the product ships it: the directory's
        yaml alone, never an experimental overlay: what a personality
        compiles from."""
        known = self._descriptors.get(name_or_compatible)
        if known is not None and known.from_unit:
            return known
        return Descriptor(self._chip_dir(name_or_compatible), None)

    def _load_module(self, label: str, path: Path) -> ModuleType:
        if label in self._modules:
            return self._modules[label]
        # Parent stubs so the dotted reserved namespace resolves anywhere
        # the interpreter looks parents up (dataclass repr, pickling, ...).
        for parent in ("nxs_cam_pack", f"nxs_cam_pack.{self.name}"):
            if parent not in sys.modules:
                stub = ModuleType(parent)
                stub.__path__ = []  # mark as package
                sys.modules[parent] = stub
        module_name = f"nxs_cam_pack.{self.name}.{label}"
        if path.is_dir():
            # A package: its submodules import against this pack's handle,
            # so an earlier pack's stay out of the way.
            spec = importlib.util.spec_from_file_location(
                module_name, path / "__init__.py",
                submodule_search_locations=[str(path)])
        else:
            spec = importlib.util.spec_from_file_location(module_name, path)
        if spec is None or spec.loader is None:
            raise PackError(f"cannot load {path}")
        for stale in [n for n in sys.modules if n.startswith(module_name + ".")]:
            del sys.modules[stale]
        module = importlib.util.module_from_spec(spec)
        module.PACK = self  # sibling access for pack code
        sys.modules[module_name] = module
        try:
            spec.loader.exec_module(module)
        except Exception as exc:
            sys.modules.pop(module_name, None)
            raise PackError(f"error loading {path}: {exc}") from exc
        self._modules[label] = module
        return module

    def chip_module(self, name_or_compatible: str, cameras: Optional[int] = None,
                    csi_lanes: Optional[int] = None,
                    pair_lines: Optional[Dict[str, int]] = None) -> Optional[Any]:
        """A chip's physics surface (cached): the law family its descriptor
        names (`meta.chip`), whatever sits beside the yaml (a camera
        personality's `<chip>.py` is behaviour the compiler loads, never a
        law module); else the chip's own physics module; else None for a
        yaml-only plugin. A descriptor a unit served binds its family: its
        behaviour runs on the unit. A behaviour class beside a yaml naming
        no family is refused by file. With a camera count, the family binds
        to that count's view."""
        served = self._descriptors.get(name_or_compatible)
        if served is not None and served.from_unit:
            return self._bind(served, cameras, csi_lanes, pair_lines)
        chip_dir = self._chip_dir(name_or_compatible)
        descriptor = self.descriptor(chip_dir.name)
        if (descriptor.raw("meta") or {}).get("chip"):
            return self._bind(descriptor, cameras, csi_lanes, pair_lines)
        path = chip_dir / f"{chip_dir.name}.py"
        if not path.exists():
            return None
        module = self._load_module(chip_dir.name, path)
        from nxs.compiler import CameraSensor, HubDevice
        # A hub's program class (the executor's image) sits beside the
        # chip's physics; a sensor's behaviour class needs a law family.
        if any(isinstance(obj, type) and issubclass(obj, CameraSensor)
               and not issubclass(obj, HubDevice)
               and obj is not CameraSensor for obj in vars(module).values()):
            raise PackError(
                f"{path}: a camera personality's behaviour class needs its "
                f"laws — name the law family in {chip_dir.name}.yaml "
                f"meta.chip (sony_imx or generic)")
        return module

    def _bind(self, descriptor: Descriptor, cameras: Optional[int] = None,
              csi_lanes: Optional[int] = None,
              pair_lines: Optional[Dict[str, int]] = None) -> Any:
        """The descriptor's law family, cached by name (and camera count, and
        the pair lines the view carries)."""
        key = descriptor.name
        if cameras is not None and csi_lanes is not None:
            descriptor = descriptor.for_cameras(cameras, int(csi_lanes), pair_lines)
            key = f"{key}@{cameras}/{int(csi_lanes)}/{sorted((pair_lines or {}).items())}"
        if key not in self._bound:
            from .chips import bind
            self._bound[key] = bind(descriptor)
        return self._bound[key]

    def flows(self) -> ModuleType:
        """Load (cached) the pack's flow-assembly module or package."""
        return self._load_module(
            "flows", self._root / str(self._raw["flows"])
        )

    def topology_path(self) -> Optional[Path]:
        """The pack's default topology file, if declared."""
        rel = self._raw.get("topology")
        return self._root / str(rel) if rel else None

    def golden(self) -> Optional[tuple[str, List[Path]]]:
        """Golden reference configs for plan byte-diffs, if declared."""
        golden = self._raw.get("golden")
        if not golden:
            return None
        files = [self._root / str(f) for f in golden["files"]]
        return str(golden.get("flow", "dual")), files


#: The name of the tool's own pack for direct ports, what an extension names
#: in `extends:` to add a sensor pair to it.
DIRECT_PACK = "direct"


class DirectPack(Pack):
    """The pack of a direct port: a sensor wired straight to the host, its
    NXS unit on the same bus. It carries no chip of its own and no carrier
    knowledge: its flows are the tool's (`nxs.cam.direct`), its sensors the
    pairs of the extensions that name it and the descriptors the units and
    the installed personalities serve."""

    def __init__(self) -> None:
        self._root = None
        self._raw = {"pack": DIRECT_PACK, "api": PACK_API, "chips": [],
                     "flows": None, "flows_for": []}
        self._modules = {}
        self._bound = {}
        self._descriptors = {}
        self._chip_roots = {}
        self._extensions = []
        self._adopted = []
        self._experimental = {}

    def flows(self) -> ModuleType:
        from . import direct
        return direct

    def _no_chip(self, name: str) -> PackError:
        # The pack is the tool's own: the operator knows its sensors as the
        # personalities installed, never by the pack's name.
        served = ", ".join(self.chips) or "none"
        return PackError(f"no installed personality serves {name!r} (served: {served})")

    def topology_path(self) -> Optional[Path]:
        return None


_cache: Optional[List[Pack]] = None
_direct: Optional[DirectPack] = None


def reset_cache() -> None:
    """Forget discovered packs (test seam; also after env changes)."""
    global _cache, _direct
    _cache = None
    _direct = None


def prime(roots: List[Path]) -> List[Pack]:
    """Discover `roots` and make them the search result: the pack a build
    compiles is the pack its program classes compose from, whatever the
    interpreter's own search path holds."""
    global _cache
    _cache = discover(roots)
    return list(_cache)


def _direct_pack() -> DirectPack:
    global _direct
    if _direct is None:
        _direct = DirectPack()
    return _direct


def direct_pack() -> DirectPack:
    """The direct pack, with the extensions that name it attached (discovery
    attaches them)."""
    discover()
    return _direct_pack()


def all_packs() -> List[Pack]:
    """Every pack a sensor may live in: the discovered ones, then the direct
    pack."""
    return discover() + [direct_pack()]


def _check_shape(path: Path, raw: Dict[str, Any]) -> None:
    """The pack schema is the loader's first gate: types, patterns, and non-empty
    lists are refused here naming the file."""
    from nxs import schemas

    problems = schemas.findings(raw, schemas.PACK, where=str(path))
    if problems:
        raise PackError("; ".join(problems))


def _check_api(path: Path, raw: Dict[str, Any]) -> None:
    """Refuse a manifest whose declared pack API is not the one this loader
    speaks (every manifest declares one; the callers require the key)."""
    declared = raw.get("api", PACK_API)
    try:
        declared = int(declared)
    except (TypeError, ValueError):
        raise PackError(f"{path}: api must be an integer, got {declared!r}")
    if declared != PACK_API:
        raise PackError(f"{path}: declares descriptor pack api {declared}; "
                        f"this nxs speaks api {PACK_API}")


def hub_classes(paths: Optional[List[Path]] = None) -> Dict[int, str]:
    """Deserializer device id -> compatible, from every discovered pack's
    hub descriptors (the chips its flows serve). Chip identity is pack
    data: the tool names no silicon of its own."""
    classes: Dict[int, str] = {}
    for pack in discover(paths):
        try:
            hubs = pack.flows_for
        except (KeyError, TypeError):
            continue
        for compatible in hubs:
            try:
                d = pack.descriptor(compatible)
                dev_id = d.raw("meta").get("device_id")
            except Exception:
                continue
            if dev_id is not None:
                classes[to_int(dev_id)] = d.compatible
    return classes


def discover(paths: Optional[List[Path]] = None) -> List[Pack]:
    """Find packs on the search path (cached for the default path)."""
    global _cache
    if paths is None and _cache is not None:
        return list(_cache)
    roots = paths if paths is not None else search_paths()
    found: List[Pack] = []
    extensions: List[tuple[Path, Dict[str, Any]]] = []
    seen: set[Path] = set()
    for root in roots:
        candidates: List[Path] = []
        if (root / "pack.yaml").exists():
            candidates.append(root)
        elif root.is_dir():
            candidates.extend(
                child for child in sorted(root.iterdir())
                if (child / "pack.yaml").exists()
            )
        for candidate in candidates:
            real = candidate.resolve()
            if real in seen:
                continue
            seen.add(real)
            raw = yaml.safe_load((candidate / "pack.yaml").read_text())
            if not isinstance(raw, dict):
                raise PackError(f"{candidate / 'pack.yaml'}: not a mapping")
            if "extends" in raw:
                unknown = set(raw) - _EXTENSION_KEYS
                if unknown or "chips" not in raw or "api" not in raw:
                    raise PackError(
                        f"{candidate / 'pack.yaml'}: an extension "
                        f"declares extends, api, and chips; got "
                        f"{sorted(raw)}")
                _check_api(candidate / "pack.yaml", raw)
                _check_shape(candidate / "pack.yaml", raw)
                extensions.append((candidate, raw))
                continue
            found.append(Pack(candidate))
    # The search path is most-specific first, and `find`/`pack_for` return
    # the first pack of a name: an extension attaches to that one.
    by_name: Dict[str, Pack] = {}
    for pack in found:
        by_name.setdefault(pack.name, pack)
    by_name.setdefault(DIRECT_PACK, _direct_pack())
    for root, raw in extensions:
        base = by_name.get(str(raw["extends"]))
        if base is None:
            searched = ", ".join(str(p) for p in roots)
            raise PackError(
                f"{root / 'pack.yaml'} extends pack {raw['extends']!r}, "
                f"which is not installed; searched: {searched}")
        base.extend(root, [str(c) for c in raw["chips"]])
    # The experimental overlays: only under the flag, only from an experimental directory
    # beside the pack's tree (or the one $NXS_CAM_EXPERIMENTAL names).
    from nxs import experimental
    if experimental.enabled():
        for pack in found:
            root = experimental_root(pack.root, pack.name)
            if root is not None:
                manifest = _experimental_manifest(root)
                # Unshipped sensors join after the shipped ones, then the overlays.
                if manifest.get("chips"):
                    pack.extend(root, [str(c) for c in manifest["chips"]])
                pack.attach_experimental(root, {str(k): str(v)
                                         for k, v in manifest["overlays"].items()})
    if paths is None:
        _cache = list(found)
    return found


def _experimental_manifest(root: Path) -> Dict[str, Any]:
    """An experimental directory's manifest for its pack, judged by its
    schema: the overlays per chip, and the sensors it adds to the pack."""
    from nxs import schemas

    path = root / "experimental.yaml"
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict):
        raise PackError(f"{path}: not a mapping")
    unknown = set(raw) - _EXPERIMENTAL_KEYS
    if unknown:
        raise PackError(f"{path}: unknown keys {sorted(unknown)}")
    _check_api(path, raw)
    problems = schemas.findings(raw, schemas.EXPERIMENTAL, where=str(path))
    if problems:
        raise PackError("; ".join(problems))
    if str(raw["experimental"]) != root.name:
        raise PackError(f"{path}: declares material for {raw['experimental']!r} in a "
                        f"directory named {root.name!r}")
    return raw


def find(name: str) -> Optional[Pack]:
    """A pack by name: a discovered one, or the direct pack."""
    for pack in all_packs():
        if pack.name == name:
            return pack
    return None


def pack_for(topology: Topology) -> Pack:
    """The pack whose flows cover this topology's deserializer (the direct
    pack for a port with none), with the descriptors its links' units served
    adopted from the state directory's cache (never the bus), then those of
    the camera personalities installed on the host, wherever the pack lacks
    the sensor; PackError naming the search path when no pack covers the hub."""
    candidates = [direct_pack()] if topology.is_direct else discover()
    for pack in candidates:
        if topology.is_direct or topology.des_compatible in pack.flows_for:
            _adopt_cached_units(pack, topology)
            _adopt_installed_personalities(pack)
            return pack
    searched = ", ".join(str(p) for p in search_paths())
    raise PackError(
        f"no descriptor pack covers {topology.des_compatible!r}; "
        f"searched: {searched}. {_NO_PACK_HINT}"
    )


def _adopt_cached_units(pack: Pack, topology: Topology) -> None:
    """Adopt the cached descriptor of every link that carries a unit, unless
    the pack already knows that sensor."""
    from .unit_source import cached_descriptor

    for link in topology.links:
        if not link.nxs_units:
            continue
        descriptor = cached_descriptor(topology, link)
        if descriptor is None:
            continue
        try:
            pack.descriptor(descriptor.compatible)
        except PackError:
            pack.adopt_descriptor(descriptor)


def _adopt_installed_personalities(pack: Pack) -> None:
    """Adopt the descriptor of every camera personality installed on the
    host, unless the pack or a unit already served that sensor: a
    deployment host holds no sensor source, and the overlay's capture
    table has to carry every head it may meet before a unit is provisioned."""
    from nxs.personality_cli import StoreError

    from .installed import installed_descriptors

    try:
        descriptors = installed_descriptors()
    except StoreError as exc:
        raise PackError(str(exc)) from None
    for descriptor in descriptors:
        try:
            pack.descriptor(descriptor.compatible)
        except PackError:
            pack.adopt_descriptor(descriptor)


def sensor_address(pack: Pack, link) -> int:
    """Where the host reaches a link's sensor: the address the port record
    says the link's serializer translates for it while the port is up,
    else the link's declared address, else its descriptor's own
    (meta.i2c_addr; 0x1A when the descriptor is silent)."""
    host = getattr(link, "host_addr", None)
    if host is not None:
        return int(host)
    return native_sensor_address(pack, link)


def native_sensor_address(pack: Pack, link) -> int:
    """Where a link's sensor answers on its own bus, whatever the host
    reaches it at: the link's declared address, else its descriptor's own
    (meta.i2c_addr; 0x1A when the descriptor is silent)."""
    declared = getattr(link, "sensor_addr", None)
    if declared is not None:
        return int(declared)
    if getattr(link, "sensor_compatible", None) is None:
        return 0x1A
    try:
        meta = pack.descriptor(link.sensor_compatible).raw("meta") or {}
    except PackError:
        return 0x1A
    return to_int(meta.get("i2c_addr", 0x1A))


def load_descriptor(name_or_compatible: str) -> Descriptor:
    """A chip descriptor from any pack; PackError when none carries it."""
    for pack in all_packs():
        try:
            return pack.descriptor(name_or_compatible)
        except PackError:
            continue
    searched = ", ".join(str(p) for p in search_paths())
    raise PackError(
        f"no descriptor pack carries {name_or_compatible!r}; "
        f"searched: {searched}. {_NO_PACK_HINT}"
    )
