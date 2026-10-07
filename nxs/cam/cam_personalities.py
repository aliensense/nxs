# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The cam personalities a host knows, one registry over every place they
live: the store (`nxs personality install`), the sealed images the assets
put there, and a source tree's `cam_personalities/`. A cam personality is
its own node: it is offered on any link, behind a hub or on the connector,
and no hub owns it. Under ``nxs --experimental`` the roots on
``$NXS_CAM_PERSONALITIES`` come first and the experimental tree
(``cam_personalities-experimental/`` beside the source tree, or
``$NXS_CAM_EXPERIMENTAL``) layers its overlays over the shipped facts and
adds the heads the product does not ship; without the flag neither is
read."""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Dict, List, Optional

import yaml

from .descriptors import Descriptor

PERSONALITIES_ENV = "NXS_CAM_PERSONALITIES"
EXPERIMENTAL_ENV = "NXS_CAM_EXPERIMENTAL"
#: The cam personalities of a source tree, beside `nxs/`.
TREE_DIR_NAME = "cam_personalities"
#: The experimental tree beside it: overlays and unshipped heads.
EXPERIMENTAL_DIR_NAME = "cam_personalities-experimental"
#: The experimental manifest API this loader speaks.
EXPERIMENTAL_API = 1
_EXPERIMENTAL_KEYS = {"api", "overlays"}
SENSOR_ROLE = "SEN"


class RegistryError(RuntimeError):
    """A cam personality is missing, malformed, or its tree is."""


def _tree_root() -> Path:
    return Path(__file__).resolve().parents[2]


def search_paths() -> List[Path]:
    """Where cam personality directories are read from, most specific
    first: the roots ``$NXS_CAM_PERSONALITIES`` names (under the flag only,
    and each must exist), then the store. A source tree's own directory
    is read when the variable names no root: a bench names its copy of
    the tree, a checkout finds its own."""
    from nxs import experimental
    from nxs.suite import EARLIER_PERSONALITY_DIR, PERSONALITY_DIR

    named: List[Path] = []
    if experimental.enabled():
        for entry in os.environ.get(PERSONALITIES_ENV, "").split(":"):
            entry = entry.strip()
            if not entry:
                continue
            root = Path(entry)
            if not root.is_dir():
                raise RegistryError(
                    f"${PERSONALITIES_ENV} names {root}, which does not exist; "
                    f"unset it to run the shipped cam personalities")
            named.append(root)
    paths = named + [Path(PERSONALITY_DIR), Path(EARLIER_PERSONALITY_DIR)]
    tree = _tree_root() / TREE_DIR_NAME
    if not named and tree.is_dir():
        paths.append(tree)
    return paths


def experimental_root() -> Optional[Path]:
    """The experimental tree, under the flag: ``$NXS_CAM_EXPERIMENTAL``,
    else the one beside the source tree; None without a manifest."""
    from nxs import experimental

    if not experimental.enabled():
        return None
    env = os.environ.get(EXPERIMENTAL_ENV, "").strip()
    candidates = [Path(env)] if env else []
    candidates.append(_tree_root() / EXPERIMENTAL_DIR_NAME)
    for root in candidates:
        if (root / "experimental.yaml").exists():
            return root
    return None


def _experimental_manifest(root: Path) -> Dict[str, Any]:
    """The experimental tree's manifest, judged by its schema: the overlay
    directory per cam personality."""
    from nxs import schemas

    path = root / "experimental.yaml"
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict):
        raise RegistryError(f"{path}: not a mapping")
    unknown = set(raw) - _EXPERIMENTAL_KEYS
    if unknown:
        raise RegistryError(f"{path}: unknown keys {sorted(unknown)}")
    try:
        declared = int(raw.get("api", EXPERIMENTAL_API))
    except (TypeError, ValueError):
        raise RegistryError(f"{path}: api must be an integer, got {raw.get('api')!r}")
    if declared != EXPERIMENTAL_API:
        raise RegistryError(f"{path}: declares experimental api {declared}; "
                            f"this nxs speaks api {EXPERIMENTAL_API}")
    problems = schemas.findings(raw, schemas.EXPERIMENTAL, where=str(path))
    if problems:
        raise RegistryError("; ".join(problems))
    return raw


def _is_cam_personality(directory: Path) -> bool:
    """Whether `directory` holds a cam personality: `<name>/<name>.yaml`
    whose facts describe a sensor (a click personality's facts beside it
    in the store name no role)."""
    path = directory / f"{directory.name}.yaml"
    if not path.is_file():
        return False
    try:
        doc = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError):
        return False
    meta = (doc or {}).get("meta") if isinstance(doc, dict) else None
    return isinstance(meta, dict) and meta.get("role") == SENSOR_ROLE


class Registry:
    """The cam personalities found on the search path, by name and by
    compatible; a name resolves on the first root that carries it."""

    def __init__(self, roots: List[Path], experimental: bool = True) -> None:
        from nxs.suite import EARLIER_PERSONALITY_DIR, PERSONALITY_DIR

        self._roots = roots
        self._store_roots = {Path(PERSONALITY_DIR).resolve(), Path(EARLIER_PERSONALITY_DIR).resolve()}
        self._experimental_root = experimental_root() if experimental else None
        self._dirs: Dict[str, Path] = {}
        self._overlays: Dict[str, Path] = {}
        self._descriptors: Dict[str, Descriptor] = {}
        self._modules: Dict[str, ModuleType] = {}
        self._bound: Dict[str, Any] = {}
        self._images: Optional[List[Descriptor]] = None
        for root in roots:
            if not root.is_dir():
                continue
            for child in sorted(root.iterdir()):
                if child.name not in self._dirs and _is_cam_personality(child):
                    self._dirs[child.name] = child
        experimental = self._experimental_root
        if experimental is not None:
            manifest = _experimental_manifest(experimental)
            overlays = {str(k): str(v) for k, v in (manifest.get("overlays") or {}).items()}
            for name, overlay in overlays.items():
                over_dir = experimental / overlay
                if not (over_dir / f"{overlay}.yaml").exists():
                    raise RegistryError(
                        f"{experimental / 'experimental.yaml'}: overlay {overlay!r} for "
                        f"{name!r} has no {overlay}/{overlay}.yaml")
                # An overlay for a head that is not installed is inert.
                if name in self._dirs:
                    self._overlays[name] = over_dir
            # The unshipped heads join after the shipped ones.
            for child in sorted(experimental.iterdir()):
                if (child.name not in self._dirs and child.name not in overlays.values()
                        and _is_cam_personality(child)):
                    self._dirs[child.name] = child

    @property
    def roots(self) -> List[Path]:
        return list(self._roots)

    def is_extension(self, name: str) -> bool:
        """Whether a cam personality extends the product's set: one installed
        into the store, or one of the experimental tree's heads. The product's
        own (the source tree's, a bench's copy of it, the sealed images) set
        the capture table's bit depth; an extension's rows count only where
        no own row does."""
        directory = self._dirs.get(name)
        if directory is None:
            return False
        parent = directory.parent.resolve()
        return (parent in self._store_roots
                or (self._experimental_root is not None
                    and parent == self._experimental_root.resolve()))

    def is_shipped(self, name: str) -> bool:
        """Whether the product ships this cam personality (its image rides in
        the assets): one of its own, never an extension."""
        return (name in self._dirs or any(d.name == name for d in self._installed())) \
            and not self.is_extension(name)

    @property
    def overlays(self) -> Dict[str, str]:
        """Cam personality -> the experimental overlay directory layered over
        it (empty without --experimental)."""
        return {name: path.name for name, path in self._overlays.items()}

    def names(self) -> List[str]:
        """Every cam personality by name: the directories in search order,
        then the sealed images whose sensor no directory describes."""
        found = list(self._dirs)
        for descriptor in self._installed():
            if descriptor.name not in found:
                found.append(descriptor.name)
        return found

    def _installed(self) -> List[Descriptor]:
        """The facts of every sealed cam personality image in the store,
        read once; a store of another release stops the registry."""
        if self._images is None:
            from nxs.personality_cli import StoreError

            from .installed import installed_descriptors

            try:
                self._images = installed_descriptors()
            except StoreError as exc:
                raise RegistryError(str(exc)) from None
        return self._images

    def directory(self, name_or_compatible: str) -> Optional[Path]:
        """The directory of a cam personality, by name or compatible; None
        for one only a sealed image describes."""
        if name_or_compatible in self._dirs:
            return self._dirs[name_or_compatible]
        for name in self._dirs:
            try:
                if self.find(name).compatible == name_or_compatible:
                    return self._dirs[name]
            except Exception:        # noqa: BLE001 (a directory that does not parse)
                continue
        return None

    def find(self, name_or_compatible: str,
             lines: Optional[Dict[str, int]] = None) -> Descriptor:
        """A cam personality's facts by name or compatible (cached; the
        experimental overlay merged under the flag); with `lines` (mode ->
        HMAX), the view of a port that runs them."""
        descriptor = self._descriptors.get(name_or_compatible)
        if descriptor is None:
            directory = self.directory(name_or_compatible)
            if directory is not None:
                descriptor = Descriptor(directory, self._overlays.get(directory.name))
            else:
                descriptor = next((d for d in self._installed()
                                   if name_or_compatible in (d.name, d.compatible)), None)
            if descriptor is None:
                raise self._unknown(name_or_compatible)
            self._descriptors[name_or_compatible] = descriptor
            self._descriptors[descriptor.name] = descriptor
            self._descriptors[descriptor.compatible] = descriptor
        return descriptor.at_lines(lines)

    def shipped(self, name_or_compatible: str) -> Descriptor:
        """The facts as the product ships them: the directory's yaml alone,
        never an experimental overlay, what a personality compiles from."""
        directory = self.directory(name_or_compatible)
        if directory is None:
            return self.find(name_or_compatible)
        return Descriptor(directory, None)

    def source(self, name_or_compatible: str) -> Optional[Path]:
        """The `<name>.py` beside a cam personality's facts (its behaviour
        class or physics module), or None when it ships none or only a
        sealed image describes it."""
        directory = self.directory(name_or_compatible)
        if directory is None:
            return None
        path = directory / f"{directory.name}.py"
        return path if path.exists() else None

    def module(self, name_or_compatible: str,
               lines: Optional[Dict[str, int]] = None) -> Optional[Any]:
        """A cam personality's physics surface (cached): the law family its
        facts name (`meta.chip`), whatever sits beside the yaml (a behaviour
        class is the compiler's, never a law module); else its own physics
        module; else None for a yaml-only head. A behaviour class beside
        facts naming no family is refused by file. With `lines`, the family
        binds to the view of a port that runs them."""
        descriptor = self.find(name_or_compatible)
        if (descriptor.raw("meta") or {}).get("chip") or descriptor.from_unit:
            return self._bind(descriptor, lines)
        path = self.source(descriptor.name)
        if path is None:
            return None
        module = self._load_module(descriptor, path)
        from nxs.compiler import CamPersonality, HubDevice
        if any(isinstance(obj, type) and issubclass(obj, CamPersonality)
               and not issubclass(obj, HubDevice)
               and obj is not CamPersonality for obj in vars(module).values()):
            raise RegistryError(
                f"{path}: a cam personality's behaviour class needs its "
                f"laws — name the law family in {descriptor.name}.yaml "
                f"meta.chip (sony_imx or generic)")
        return module

    def bind(self, descriptor: Descriptor,
             lines: Optional[Dict[str, int]] = None) -> Any:
        """The law family of facts a unit served, bound to the view."""
        return self._bind(descriptor, lines)

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

    def _load_module(self, descriptor: Descriptor, path: Path) -> ModuleType:
        name = descriptor.name
        if name in self._modules:
            return self._modules[name]
        parent = "nxs_cam_personality"
        if parent not in sys.modules:
            stub = ModuleType(parent)
            stub.__path__ = []
            sys.modules[parent] = stub
        module_name = f"{parent}.{name}"
        spec = importlib.util.spec_from_file_location(module_name, path)
        if spec is None or spec.loader is None:
            raise RegistryError(f"cannot load {path}")
        module = importlib.util.module_from_spec(spec)
        module.FACTS = descriptor  # the head's own facts, for its physics
        sys.modules[module_name] = module
        try:
            spec.loader.exec_module(module)
        except Exception as exc:
            sys.modules.pop(module_name, None)
            raise RegistryError(f"error loading {path}: {exc}") from exc
        self._modules[name] = module
        return module

    def _unknown(self, name: str) -> RegistryError:
        searched = ", ".join(str(p) for p in self._roots)
        return RegistryError(
            f"no installed cam personality describes {name!r} "
            f"(installed: {', '.join(self.names()) or 'none'}; searched: {searched}); "
            f"install one: nxs personality install <directory>")


_cache: Optional[Registry] = None


def reset_cache() -> None:
    """Forget the registry (test seam; also after env changes)."""
    global _cache
    _cache = None


def registry(roots: Optional[List[Path]] = None, experimental: bool = False) -> Registry:
    """The registry over the search path (cached, the experimental tree
    attached under the flag), or one over `roots` alone."""
    global _cache
    if roots is not None:
        return Registry(roots, experimental)
    if _cache is None:
        _cache = Registry(search_paths())
    return _cache


def prime(roots: List[Path], experimental: bool = False) -> Registry:
    """Make `roots` the registry: what a build compiles is what its
    program classes compose from, whatever the interpreter's search path
    holds; the experimental tree stays out unless asked for."""
    global _cache
    _cache = Registry(roots, experimental)
    return _cache


def names() -> List[str]:
    return registry().names()


def find(name_or_compatible: str, lines: Optional[Dict[str, int]] = None) -> Descriptor:
    return registry().find(name_or_compatible, lines)


def shipped(name_or_compatible: str) -> Descriptor:
    return registry().shipped(name_or_compatible)


def source(name_or_compatible: str) -> Optional[Path]:
    return registry().source(name_or_compatible)


def module(name_or_compatible: str, lines: Optional[Dict[str, int]] = None) -> Optional[Any]:
    return registry().module(name_or_compatible, lines)


def overlays() -> Dict[str, str]:
    return registry().overlays
