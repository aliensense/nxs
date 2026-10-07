# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The `personality` verb group. On a unit, `nxs [<port> <link>] personality
upload|status|show|rm`: `upload` resolves its argument in one order (an
explicit file, the personality store, a built-in click personality, an
installed cam personality), compiles what needs compiling, and lands a
click personality in the VM or a
cam personality in a store slot; `nxs upload` is the same verb. `show`
prints a personality's metadata and never its bytecode. On the host,
`personality check|install` (`nxs.personality`) judge and
install personalities; a unit's slots are `nxs store ls`."""

from __future__ import annotations

import dataclasses
import os
import sys
import zlib
from pathlib import Path
from typing import Any, Dict, List, Optional

from nxs.client import ERRNO_EEXIST, DeviceRefused, SupportsSlotPeek, peek_slot
from nxs.compiler import CamPersonality, CompileError, ClickPersonality
from nxs.image import (IMAGE_KIND_NAMES, ImageKind, deserialize, format_refusal,
                       parse_trailer, peek_format, serialize, trailer_bytes)

#: The store of compiled personalities (`<name>.nxs`), the release asset's
#: install location; `$NXS_PERSONALITIES` overrides it.
STORE_ENV = "NXS_PERSONALITIES"
IMAGE_SUFFIX = ".nxs"
#: The release asset a store image of another format belongs to.
ASSET_NAME = "nxs-assets-<version>.tar.gz"
#: Store slots a unit holds.
MAX_SLOTS = 8

KIND_IMAGE = "image"
KIND_CLICK = "click"
KIND_CAM = "cam"


def store_dir() -> str:
    from nxs.suite import PERSONALITY_DIR
    return os.environ.get(STORE_ENV) or PERSONALITY_DIR


class StoreError(Exception):
    """The personality store cannot serve this nxs; the message names the
    asset to install."""


def store_refusal(store: Optional[str] = None) -> Optional[str]:
    """Why the personality store cannot serve this nxs, or None: its
    manifest names another release (the assets tarball is versioned
    with the wheel: install that release's), or it holds images with no
    manifest at all (a hand-filled store). An experimental runs either under
    --experimental; a customer is refused by name."""
    import yaml

    from nxs import __version__, experimental

    store = store or store_dir()
    if experimental.enabled():
        return None
    manifest = os.path.join(store, "manifest.yaml")
    asset = ASSET_NAME.replace("<version>", __version__)
    if not os.path.isfile(manifest):
        try:
            images = [n for n in os.listdir(store) if n.endswith(IMAGE_SUFFIX)]
        except OSError:
            images = []
        if not images:
            return None
        return (f"the personality store {store} carries {len(images)} image(s) and no "
                f"manifest.yaml: install {asset} (a development store runs under "
                f"nxs --experimental)")
    try:
        with open(manifest, encoding="utf-8") as fh:
            doc = yaml.safe_load(fh)
    except (OSError, yaml.YAMLError) as exc:
        return f"{manifest}: unreadable ({exc}); install {asset}"
    from nxs.assets_cli import other_build

    found = str((doc or {}).get("version", "")) if isinstance(doc, dict) else ""
    if found != __version__:
        return (f"the personality store {store} is release {found or '?'}, this nxs is "
                f"{__version__} — install {asset}")
    built = str(doc.get("build") or "") if isinstance(doc, dict) else ""
    if (fact := other_build(built)) is not None:
        return f"in the personality store {store} {fact} — install the {asset} built with it"
    return None


class ResolveError(Exception):
    """The argument names nothing the resolver can upload; the message says
    what was searched. `stderr` and `prefix` say how the verb prints it."""

    def __init__(self, message: str, stderr: bool = True, prefix: str = "error: ") -> None:
        super().__init__(message)
        self.stderr = stderr
        self.prefix = prefix

    def line(self) -> str:
        return f"{self.prefix}{self}"


class CompileFailed(Exception):
    """A compile refused; `stderr` says which stream the message goes to."""

    def __init__(self, message: str, stderr: bool = True) -> None:
        super().__init__(message)
        self.stderr = stderr


@dataclasses.dataclass
class Source:
    """What a personality argument resolved to."""

    kind: str                       # KIND_IMAGE, KIND_CLICK, KIND_CAM
    origin: str                     # file, store, built-in, hub
    name: str
    path: Optional[str] = None
    image: Optional[bytes] = None
    image_kind: Optional[int] = None
    driver_cls: Optional[type] = None
    descriptor: Any = None
    #: The hub a hub source belongs to: its serializer tail counts in the
    #: capture rows the trailer carries.
    hub: Any = None

    @property
    def where(self) -> str:
        return f"{self.origin} {self.path or self.name}"


# ── resolution ───────────────────────────────────────────────────────

def resolve(token: str) -> Source:
    """The source behind an `upload` argument: an existing file
    (a `.nxs` of either kind, a `.py`, or the `.yaml` of a source pair),
    else `<store>/<name>.nxs`, else a built-in or store click personality,
    else a cam personality with a behaviour class beside its facts."""
    if os.path.isfile(token):
        return _resolve_file(token)
    name = token.lower()
    stored = os.path.join(store_dir(), f"{name}{IMAGE_SUFFIX}")
    if os.path.isfile(stored):
        refusal = store_refusal()
        if refusal:
            raise ResolveError(refusal)
        return _image_source(stored, name, origin="store")

    from nxs.suite import personality_file
    from nxs.suite.reconcile import ClickPersonalityNotFound, load_click_personality
    try:
        cls = load_click_personality(name)
    except ClickPersonalityNotFound as e:
        not_a_driver = str(e)
    else:
        path = personality_file(name, "py")
        return _class_source(cls, path or _module_file(cls), name,
                             origin="store" if path else "built-in")
    found = _registry_source(name)
    if found is not None:
        return found
    raise ResolveError(f"No personality or file '{token}': {not_a_driver}", prefix="")


def _module_file(cls) -> Optional[str]:
    module = sys.modules.get(cls.__module__)
    return getattr(module, "__file__", None)


def _resolve_file(path: str) -> Source:
    ext = os.path.splitext(path)[1].lower()
    stem = os.path.splitext(os.path.basename(path))[0]
    if ext == ".py":
        return _class_source(_load_class(path, stem), path, stem, origin="file")
    if ext in (".yaml", ".yml"):
        py = os.path.splitext(path)[0] + ".py"
        if not os.path.isfile(py):
            raise ResolveError(f"{path}: a source pair needs {py} beside it")
        return _resolve_file(py)
    return _image_source(path, stem, origin="file")


def _class_in(path: str, label: str) -> Optional[type]:
    """The personality class a source file defines, None when it defines none;
    registered like an imported module so the class finds its sibling
    descriptor. A file that fails to import is an error, never None."""
    from nxs.personality import load_source

    try:
        mod = load_source(f"nxs_local_drivers.{label}", path)
    except Exception as e:
        from nxs.client import import_failure_detail
        raise ResolveError(f"{path} failed to import: "
                           f"{import_failure_detail(e, path)}") from None
    for attr in dir(mod):
        obj = getattr(mod, attr)
        if (isinstance(obj, type) and issubclass(obj, ClickPersonality)
                and obj is not ClickPersonality and obj.__module__ == mod.__name__):
            return obj
    return None


def _load_class(path: str, label: str) -> type:
    """The one personality class a source file defines."""
    cls = _class_in(path, label)
    if cls is None:
        raise ResolveError(f"No ClickPersonality found in {label}", stderr=False, prefix="")
    return cls


def _class_source(cls: type, path: Optional[str], name: str, origin: str) -> Source:
    """A personality class as a source; a cam personality needs its facts,
    the `<name>.yaml` beside its file."""
    if not issubclass(cls, CamPersonality):
        return Source(KIND_CLICK, origin, name, path=path, driver_cls=cls)
    yaml_path = f"{os.path.splitext(path)[0]}.yaml" if path else None
    if not yaml_path or not os.path.isfile(yaml_path):
        # A behaviour class alone compiles (a development image); without its
        # datasheet the image carries no descriptor trailer, so no host
        # learns the sensor from it.
        print(f"note: no datasheet beside {path or cls.__name__} "
              f"({os.path.basename(yaml_path) if yaml_path else 'no source file'}): "
              f"the image carries no descriptor trailer", file=sys.stderr)
        return Source(KIND_CAM, origin, name, path=path, driver_cls=cls)
    return Source(KIND_CAM, origin, name, path=path, driver_cls=cls,
                  descriptor=_descriptor_at(yaml_path))


def _descriptor_at(yaml_path: str):
    """The descriptor of a source pair: the hub layout (`<x>/<x>.yaml`)
    loads with its blobs and provenance, a flat pair by its facts alone."""
    import yaml as _yaml

    from nxs.cam.contracts import ContractError
    from nxs.cam.descriptors import Descriptor

    root = Path(yaml_path).resolve().parent
    stem = Path(yaml_path).stem
    try:
        if root.name == stem:
            return Descriptor(root)
        with open(yaml_path, encoding="utf-8") as fh:
            doc = _yaml.safe_load(fh)
        if not isinstance(doc, dict):
            raise ResolveError(f"{yaml_path}: not a descriptor mapping")
        return Descriptor.from_data(doc, stem)
    except (ContractError, _yaml.YAMLError, OSError) as e:
        raise ResolveError(f"{yaml_path}: {e}") from None


def _registry_source(name: str) -> Optional[Source]:
    """A cam personality by name or compatible from the registry (its
    shipped facts and the behaviour class beside them), else a hub chip's
    program class (the executor's image) from the hub that ships it; None
    when nothing installed carries the name."""
    from nxs.cam import cam_personalities, hubs

    try:
        registry = cam_personalities.registry()
        directory = registry.directory(name)
    except cam_personalities.RegistryError:
        directory = None
    if directory is not None:
        # A personality is compiled from the shipped facts: a bench overlay
        # never reaches the unit's records.
        descriptor = registry.shipped(name)
        source = registry.source(name)
        if source is None:
            raise ResolveError(
                f"cam personality {name!r} ({descriptor.compatible}) ships no behaviour "
                f"class ({descriptor.name}.py beside {descriptor.name}.yaml): "
                f"nothing to compile")
        cls = _load_class(str(source), descriptor.name)
        if not issubclass(cls, CamPersonality):
            raise ResolveError(
                f"{source}: {cls.__name__} is not a CamPersonality; a cam personality's "
                f"behaviour class compiles to a cam personality")
        return Source(KIND_CAM, "registry", descriptor.name, path=str(source),
                      driver_cls=cls, descriptor=descriptor)
    try:
        found = hubs.discover()
    except hubs.HubError:
        return None
    from nxs.compiler import HubDevice
    for hub in found:
        if hub._own(name) is None:
            continue
        descriptor = hub.shipped_descriptor(name)
        source = hub.chip_source(name)
        cls = _class_in(str(source), descriptor.name) if source is not None else None
        if cls is None or not issubclass(cls, HubDevice):
            continue
        return Source(KIND_CAM, "hub", descriptor.name, path=str(source),
                      driver_cls=cls, descriptor=descriptor, hub=hub)
    return None


def _image_source(path: str, name: str, origin: str) -> Source:
    with open(path, "rb") as fh:
        img = fh.read()
    try:
        major, minor, kind, _flags = peek_format(img)
    except ValueError:
        raise ResolveError(f"{path} is not an NXS image — upload the personality "
                           f"name instead: nxs upload {name}") from None
    refusal = format_refusal(major, minor)
    if refusal is not None:
        text = refusal.replace("<name>", name)
        if origin == "store":
            text += (f"; the store holds another release's personalities — "
                     f"install the {ASSET_NAME} of this nxs")
        raise ResolveError(text)
    if kind not in IMAGE_KIND_NAMES:
        raise ResolveError(f"{path} declares image kind {kind}, which this tool "
                           f"does not know — update nxs")
    return Source(KIND_IMAGE, origin, name, path=path, image=img, image_kind=kind)


# ── compile ──────────────────────────────────────────────────────────

def parse_config(pairs) -> Dict[str, Any]:
    """`KEY=VALUE` flags to a config dict (integers where they parse)."""
    config: Dict[str, Any] = {}
    for kv in pairs or []:
        if "=" not in kv:
            raise CompileFailed(f"Invalid config: {kv} (expected key=value)", stderr=False)
        k, v = kv.split("=", 1)
        try:
            config[k] = int(v)
        except ValueError:
            config[k] = v
    return config


def compile_source(source: Source, config: Dict[str, Any]):
    """`(compiled, image bytes)` for a click or cam personality source; a cam
    personality's trailer is the datasheet's records with the compiled
    param indices. CompileFailed carries the user-facing line."""
    cls = source.driver_cls
    user_supplied = set(config)
    try:
        compiled = cls().compile(dict(config))
    except (CompileError, FileNotFoundError, ValueError) as e:
        raise CompileFailed(f"Error: {e}") from None
    from nxs.check import sensor_allowed_keys
    allowed = sensor_allowed_keys(cls, compiled)
    unknown = user_supplied - allowed
    if unknown:
        raise CompileFailed(f"Unknown config keys: {', '.join(sorted(unknown))}\n"
                            f"Valid: {', '.join(sorted(allowed))}", stderr=False)
    if (source.kind == KIND_CAM and source.descriptor is not None
            and compiled.kind != ImageKind.HUB):
        from nxs.personality import records
        try:
            compiled.trailer = records.encode_trailer(source.descriptor, compiled.params,
                                                      hub=source.hub)
        except records.RecordError as e:
            raise CompileFailed(f"Error: {e}") from None
    try:
        img = serialize(compiled)
    except CompileError as e:
        raise CompileFailed(f"Error: {e}") from None
    return compiled, img


def _report(compiled, img: bytes) -> None:
    print(f"Compiled {compiled.name}: {len(compiled.bytecode)}B bytecode, "
          f"{len(compiled.params)} params → {len(img)}B "
          f"{IMAGE_KIND_NAMES[compiled.kind]} image")
    if compiled.kind == ImageKind.CAMERA:
        from nxs.compiler import compile_report
        print(compile_report(compiled))


# ── upload ───────────────────────────────────────────────────────────

_SOURCE_SUFFIXES = (".py", ".yaml", ".yml")


def cmd_upload(t, args) -> int:
    """`upload <name|file>`: compile and land a personality of either kind."""
    token = getattr(args, "target", None) or args.personality
    output = getattr(args, "output", None)
    # A compiled image has its params baked in: the flags that compile are
    # refused before the file is even read.
    if os.path.isfile(token) and not token.lower().endswith(_SOURCE_SUFFIXES):
        if getattr(args, "config", None):
            print("error: --param applies only when compiling a personality from "
                  "its .py; a compiled .nxs has its params baked in", file=sys.stderr)
            return 1
        if output:
            print(f"error: {token} is already compiled; -o compiles a "
                  f"personality from its .py", file=sys.stderr)
            return 1
    try:
        source = resolve(token)
    except ResolveError as e:
        print(e.line(), file=sys.stderr if e.stderr else sys.stdout)
        return 1
    if source.kind == KIND_IMAGE:
        if output:
            print(f"error: {source.path} is already compiled; -o compiles a "
                  f"personality from its .py", file=sys.stderr)
            return 1
        if source.image_kind == ImageKind.HUB:
            print(f"error: {source.path} is a hub image; the host runs it "
                  f"(nxs switch), a unit never does", file=sys.stderr)
            return 1
        print(f"Uploading {source.path}: {len(source.image)}B "
              f"{IMAGE_KIND_NAMES[source.image_kind]} image")
        return _land(t, args, source, source.image, source.image_kind,
                     source.name, None)
    try:
        config = parse_config(getattr(args, "config", None))
        compiled, img = compile_source(source, config)
    except CompileFailed as e:
        print(str(e), file=sys.stderr if e.stderr else sys.stdout)
        return 1
    _report(compiled, img)
    if output:
        with open(output, "wb") as fh:
            fh.write(img)
        print(f"Wrote {output}: {len(img)}B image")
        return 0
    return _land(t, args, source, img, compiled.kind, compiled.name, compiled)


def _land(t, args, source: Source, img: bytes, kind: int, name: str, compiled) -> int:
    """Upload the image: a click personality runs, a cam personality lands in a
    store slot (refused on a transport that cannot save one), and the
    link's cache learns the descriptor."""
    from nxs.cli import _upload_and_run

    if kind == ImageKind.HUB:
        print(f"error: {name} is a hub personality; the host runs it (nxs switch), "
              f"a unit never does", file=sys.stderr)
        return 1
    if kind != ImageKind.CAMERA:
        return _upload_and_run(t, img)
    if not _can_save(t):
        print("error: a cam personality runs from a flash slot, and this transport "
              "cannot save one", file=sys.stderr)
        return 1
    slot = _pick_slot(t, name, getattr(args, "slot", None))
    if slot is None:
        return 1
    t.upload_image(img)
    try:
        t.save_slot(slot)
    except DeviceRefused as e:
        if e.code == ERRNO_EEXIST:
            print("Already stored: an identical personality image occupies "
                  "another slot. Nothing to do.")
            return 0
        print(f"Save refused: {e}", file=sys.stderr)
        return 1
    print(f"Uploaded cam personality {name} to slot {slot}; the camera "
          f"verbs run it from there under the bus token.")
    _remember(args, source, img, compiled, slot, name)
    return 0


def _can_save(t) -> bool:
    return isinstance(t, SupportsSlotPeek) and hasattr(t, "save_slot")


def _pick_slot(t, name: str, wanted: Optional[int]) -> Optional[int]:
    """The slot a cam personality occupies once saved, read from the
    store: the one asked for, else the slot already holding this name, else
    the slot holding the unit's cam personality (a unit runs one; the
    upload replaces it), else a slot whose image the unit refuses to
    describe (another format version: the upload reinstalls it), else the
    first empty one. The unit keeps its store packed: a save overwrites an
    occupied slot in place and appends anywhere else, so an empty slot
    asked for is the first empty one, and nothing is deleted first, which
    would move the later images down onto the slot."""
    from nxs.cam.unit_source import STALE_PEEKS
    from nxs.image import IMAGE_KIND_NAMES

    names: Dict[int, Optional[str]] = {}
    cameras: List[int] = []
    stale: List[int] = []
    for slot in range(MAX_SLOTS):
        try:
            info, held = peek_slot(t, slot)
        except DeviceRefused as exc:
            if exc.code not in STALE_PEEKS:
                raise
            stale.append(slot)
            continue
        if held:
            print("store busy: a calibration procedure or firmware push holds "
                  "the session", file=sys.stderr)
            return None
        names[slot] = str(info.name) if info is not None else None
        kind = getattr(info, "kind", None)
        if IMAGE_KIND_NAMES.get(kind, kind) == IMAGE_KIND_NAMES[ImageKind.CAMERA]:
            cameras.append(slot)
    empty = next((slot for slot, held_name in names.items() if held_name is None), None)
    if wanted is not None:
        if wanted in stale:
            return wanted
        if names.get(wanted) is not None and names[wanted].lower() != name.lower():
            print(f"slot {wanted} holds {names[wanted]}; nxs store rm {wanted} "
                  f"frees it", file=sys.stderr)
            return None
        return wanted if names.get(wanted) is not None else empty
    for slot, held_name in names.items():
        if held_name is not None and held_name.lower() == name.lower():
            return slot
    if cameras:
        return cameras[0]
    if stale:
        return stale[0]
    if empty is not None:
        return empty
    print(f"store full ({MAX_SLOTS} slots): nxs store rm <slot> frees one",
          file=sys.stderr)
    return None


def _remember(args, source: Source, img: bytes, compiled, slot: int, name: str) -> None:
    """Record the personality behind the addressed link in the state
    directory, when the command line names one."""
    from nxs.cam import unit_source
    from nxs.personality import records

    where = port_link_for(args)
    if where is None:
        return
    topology, link = where
    trailer = trailer_bytes(compiled.trailer if compiled is not None
                            else deserialize(img).trailer)
    try:
        descriptor = (source.descriptor if source.descriptor is not None
                      else records.descriptor_from_trailer(trailer))
        params = records.param_map(parse_trailer(trailer))
    except (records.RecordError, ValueError):
        return                      # an image without the records: nothing to cache
    unit_source.cache_descriptor(topology, link, descriptor, slot=slot,
                                 crc=records.trailer_crc(trailer), params=params,
                                 name=name, image_crc=zlib.crc32(img) & 0xFFFFFFFF,
                                 hub=source.hub)


def port_link_for(args):
    """`(topology, link)` an I²C-addressed unit rides, from the bus and
    address the command resolved to (a node prefix or `--unit` lands on
    them too); None for another transport or a bus no port owns."""
    from nxs.cam import unit_source

    if getattr(args, "transport", None) == "i2c" and getattr(args, "bus", None):
        return unit_source.port_link_for_bus(args.bus, int(args.addr))
    return None


def write_manifest(directory: str, version: str, build: Optional[str] = None) -> Dict[str, Any]:
    """`manifest.yaml` beside the `.nxs` files of a personalities directory:
    per image its name, the sensor it drives, the image format, and the
    file's CRC-32. Returns the manifest written."""
    import yaml

    from nxs.personality import records

    entries = []
    for entry in sorted(os.listdir(directory)):
        if not entry.endswith(IMAGE_SUFFIX):
            continue
        path = os.path.join(directory, entry)
        with open(path, "rb") as fh:
            img = fh.read()
        major, minor, kind, flags = peek_format(img)
        compiled = deserialize(img)
        record: Dict[str, Any] = {
            "name": compiled.name, "file": entry,
            "kind": IMAGE_KIND_NAMES.get(kind, str(kind)),
            "format": f"{major}.{minor}", "sealed": bool(compiled.sealed),
            "crc32": f"0x{zlib.crc32(img) & 0xFFFFFFFF:08x}"}
        if kind == ImageKind.CAMERA:
            try:
                descriptor = records.descriptor_from_trailer(trailer_bytes(compiled.trailer))
            except (records.RecordError, ValueError):
                pass
            else:
                record["sensor"] = descriptor.compatible
                record["modes"] = len(records.mode_values(descriptor))
        entries.append(record)
    manifest: Dict[str, Any] = {"version": str(version)}
    if build:
        manifest["build"] = str(build)
    manifest["personalities"] = entries
    with open(os.path.join(directory, "manifest.yaml"), "w", encoding="utf-8") as fh:
        fh.write(yaml.safe_dump(manifest, sort_keys=False))
    return manifest


def compile_images(cam_root: str, hub_root: str, out_dir: str) -> List[str]:
    """Compile every cam personality under `cam_root` that ships a behaviour
    class, and every hub chip under `hub_root` with a program class, into
    `<out_dir>/<name>.nxs`; returns the names built."""
    from nxs.cam import cam_personalities, hubs

    # The trees under compilation are what the program classes compose
    # from: not the interpreter's installed hub, which ships no sensor.
    registry = cam_personalities.prime([Path(cam_root)])
    found = hubs.prime([Path(hub_root)])
    os.makedirs(out_dir, exist_ok=True)
    built = []
    sources = [(name, registry.source(name), registry.shipped(name), None)
               for name in registry.names()]
    for hub in found:
        sources += [(chip, hub.chip_source(chip), hub.shipped_descriptor(chip), hub)
                    for chip in hub.chips]
    for name, source_path, descriptor, hub in sources:
        if source_path is None:
            continue
        # A physics module beside the facts (a head the host programs)
        # defines no class; only a behaviour class compiles to an image.
        driver_cls = _class_in(str(source_path), name)
        if driver_cls is None:
            continue
        source = Source(KIND_CAM, "hub" if hub is not None else "registry", name,
                        path=str(source_path), driver_cls=driver_cls, descriptor=descriptor,
                        hub=hub)
        _compiled, img = compile_source(source, {})
        with open(os.path.join(out_dir, f"{name}{IMAGE_SUFFIX}"), "wb") as fh:
            fh.write(img)
        built.append(name)
    return built


# ── the parser and the dispatch ──────────────────────────────────────

def add_parser(sub) -> None:
    from nxs.personality import add_host_verbs

    p = sub.add_parser("personality",
                       help="A personality on the host: check one before "
                            "installing it, install one into the store")
    ps = p.add_subparsers(dest="personality_cmd", required=True)
    add_host_verbs(ps)


def cmd_personality(args) -> int:
    from nxs.personality import run_host_verb

    return run_host_verb(args)
