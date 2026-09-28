# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The `personality` verb group. On a unit, `nxs [<port> <link>] personality
upload|status|show|rm`: `upload` resolves its argument in one order (an
explicit file, the personality store, a built-in driver, a pack sensor
pair), compiles what needs compiling, and lands a driver in the VM or a
camera personality in a store slot; `nxs upload` is the same verb. `show`
prints a personality's metadata and never its bytecode. On the host,
`personality check|install` (`nxs.personality`) judge and
install personalities; a unit's slots are `nxs store ls`."""

from __future__ import annotations

import dataclasses
import importlib.util
import os
import sys
import zlib
from pathlib import Path
from typing import Any, Dict, List, Optional

from nxs.client import ERRNO_EEXIST, DeviceRefused, SupportsSlotPeek, peek_slot
from nxs.compiler import CameraSensor, CompileError, SensorDriver
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
KIND_DRIVER = "driver"
KIND_CAMERA = "camera"


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
    found = str((doc or {}).get("version", "")) if isinstance(doc, dict) else ""
    if found != __version__:
        return (f"the personality store {store} is release {found or '?'}, this nxs is "
                f"{__version__} — install {asset}")
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

    kind: str                       # KIND_IMAGE, KIND_DRIVER, KIND_CAMERA
    origin: str                     # file, store, built-in, pack
    name: str
    path: Optional[str] = None
    image: Optional[bytes] = None
    image_kind: Optional[int] = None
    driver_cls: Optional[type] = None
    descriptor: Any = None
    #: The pack a pack source belongs to: its serializer tail counts in the
    #: capture rows the trailer carries.
    pack: Any = None

    @property
    def where(self) -> str:
        return f"{self.origin} {self.path or self.name}"


# ── resolution ───────────────────────────────────────────────────────

def resolve(token: str) -> Source:
    """The source behind an `upload` argument: an existing file
    (a `.nxs` of either kind, a `.py`, or the `.yaml` of a source pair),
    else `<store>/<name>.nxs`, else a built-in or store driver, else a pack
    sensor with a behaviour class beside its descriptor."""
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
    from nxs.suite.reconcile import DriverNotFound, load_unit_driver
    try:
        cls = load_unit_driver(name)
    except DriverNotFound as e:
        not_a_driver = str(e)
    else:
        path = personality_file(name, "py")
        return _class_source(cls, path or _module_file(cls), name,
                             origin="store" if path else "built-in")
    found = _pack_source(name)
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
    """The driver class a source file defines, None when it defines none;
    registered like an imported module so the class finds its sibling
    descriptor. A file that fails to import is an error, never None."""
    spec = importlib.util.spec_from_file_location(f"nxs_local_drivers.{label}", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    try:
        spec.loader.exec_module(mod)
    except Exception as e:
        sys.modules.pop(spec.name, None)
        from nxs.client import import_failure_detail
        raise ResolveError(f"{path} failed to import: "
                           f"{import_failure_detail(e, path)}") from None
    for attr in dir(mod):
        obj = getattr(mod, attr)
        if (isinstance(obj, type) and issubclass(obj, SensorDriver)
                and obj is not SensorDriver and obj.__module__ == mod.__name__):
            return obj
    return None


def _load_class(path: str, label: str) -> type:
    """The one driver class a source file defines."""
    cls = _class_in(path, label)
    if cls is None:
        raise ResolveError(f"No SensorDriver found in {label}", stderr=False, prefix="")
    return cls


def _class_source(cls: type, path: Optional[str], name: str, origin: str) -> Source:
    """A driver class as a source; a camera class needs its datasheet
    descriptor, the `<name>.yaml` beside its file."""
    if not issubclass(cls, CameraSensor):
        return Source(KIND_DRIVER, origin, name, path=path, driver_cls=cls)
    yaml_path = f"{os.path.splitext(path)[0]}.yaml" if path else None
    if not yaml_path or not os.path.isfile(yaml_path):
        # A behaviour class alone compiles (a development image); without its
        # datasheet the image carries no descriptor trailer, so no host
        # learns the sensor from it.
        print(f"note: no datasheet beside {path or cls.__name__} "
              f"({os.path.basename(yaml_path) if yaml_path else 'no source file'}): "
              f"the image carries no descriptor trailer", file=sys.stderr)
        return Source(KIND_CAMERA, origin, name, path=path, driver_cls=cls)
    return Source(KIND_CAMERA, origin, name, path=path, driver_cls=cls,
                  descriptor=_descriptor_at(yaml_path))


def _descriptor_at(yaml_path: str):
    """The descriptor of a source pair: the pack layout (`<x>/<x>.yaml`)
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


def _pack_source(name: str) -> Optional[Source]:
    from nxs.cam import packs

    try:
        found = packs.all_packs()
    except packs.PackError:
        return None
    for pack in found:
        # A personality is compiled from the shipped descriptor: a bench
        # overlay attached to the pack never reaches the unit's records.
        try:
            descriptor = pack.shipped_descriptor(name)
        except packs.PackError:
            continue
        source = pack.chip_source(name)
        if descriptor.role != "SEN":
            # A hub chip's program class beside its physics: the executor's
            # image; a chip with physics alone is no personality.
            from nxs.compiler import HubDevice
            cls = _class_in(str(source), descriptor.name) if source is not None else None
            if cls is None or not issubclass(cls, HubDevice):
                continue
            return Source(KIND_CAMERA, "pack", descriptor.name, path=str(source),
                          driver_cls=cls, descriptor=descriptor, pack=pack)
        if source is None:
            raise ResolveError(
                f"pack sensor {name!r} ({descriptor.compatible}) ships no behaviour "
                f"class ({descriptor.name}.py beside {descriptor.name}.yaml): "
                f"nothing to compile")
        cls = _load_class(str(source), descriptor.name)
        if not issubclass(cls, CameraSensor):
            raise ResolveError(
                f"{source}: {cls.__name__} is not a CameraSensor; a pack sensor's "
                f"behaviour class compiles to a camera personality")
        return Source(KIND_CAMERA, "pack", descriptor.name, path=str(source),
                      driver_cls=cls, descriptor=descriptor, pack=pack)
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
    """`(compiled, image bytes)` for a driver or camera source; a camera
    personality's trailer is the datasheet's records with the compiled
    param indices. CompileFailed carries the user-facing line."""
    drv_cls = source.driver_cls
    user_supplied = set(config)
    try:
        compiled = drv_cls().compile(dict(config))
    except (CompileError, FileNotFoundError, ValueError) as e:
        raise CompileFailed(f"Error: {e}") from None
    from nxs.check import sensor_allowed_keys
    allowed = sensor_allowed_keys(drv_cls, compiled)
    unknown = user_supplied - allowed
    if unknown:
        raise CompileFailed(f"Unknown config keys: {', '.join(sorted(unknown))}\n"
                            f"Valid: {', '.join(sorted(allowed))}", stderr=False)
    if (source.kind == KIND_CAMERA and source.descriptor is not None
            and compiled.kind != ImageKind.HUB):
        from nxs.personality import records
        try:
            compiled.trailer = records.encode_trailer(source.descriptor, compiled.params,
                                                      pack=source.pack)
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
    token = getattr(args, "target", None) or args.driver
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
    """Upload the image: a driver runs, a camera personality lands in a
    store slot when the transport can pick one, and the link's cache learns
    the descriptor."""
    from nxs.cli import _upload_and_run

    if kind == ImageKind.HUB:
        print(f"error: {name} is a hub personality; the host runs it (nxs switch), "
              f"a unit never does", file=sys.stderr)
        return 1
    if kind != ImageKind.CAMERA or not _can_save(t):
        return _upload_and_run(t, img, kind)
    slot = _pick_slot(t, name, getattr(args, "slot", None))
    if slot is None:
        return 1
    t.upload_image(img)
    if _slot_name(t, slot):
        t.delete_slot(slot)
    try:
        t.save_slot(slot)
    except DeviceRefused as e:
        if e.code == ERRNO_EEXIST:
            print("Already stored: an identical personality image occupies "
                  "another slot. Nothing to do.")
            return 0
        print(f"Save refused: {e}", file=sys.stderr)
        return 1
    print(f"Uploaded camera personality {name} to slot {slot}; the camera "
          f"verbs run it from there under the bus token.")
    _remember(args, source, img, compiled, slot, name)
    return 0


def _can_save(t) -> bool:
    return isinstance(t, SupportsSlotPeek) and hasattr(t, "save_slot")


def _slot_name(t, slot: int) -> str:
    info, _held = peek_slot(t, slot)
    return str(getattr(info, "name", "") or "") if info is not None else ""


def _pick_slot(t, name: str, wanted: Optional[int]) -> Optional[int]:
    """The slot a camera personality lands in: the one asked for, else the
    slot already holding this name, else the slot holding the unit's camera
    personality (a unit runs one; the upload replaces it), else the first
    empty one."""
    from nxs.image import IMAGE_KIND_NAMES

    names: Dict[int, Optional[str]] = {}
    cameras: List[int] = []
    for slot in range(MAX_SLOTS):
        info, held = peek_slot(t, slot)
        if held:
            print("store busy: a calibration procedure or firmware push holds "
                  "the session", file=sys.stderr)
            return None
        names[slot] = str(info.name) if info is not None else None
        kind = getattr(info, "kind", None)
        if IMAGE_KIND_NAMES.get(kind, kind) == IMAGE_KIND_NAMES[ImageKind.CAMERA]:
            cameras.append(slot)
    if wanted is not None:
        if names.get(wanted) is not None and names[wanted].lower() != name.lower():
            print(f"slot {wanted} holds {names[wanted]}; nxs store rm {wanted} "
                  f"frees it", file=sys.stderr)
            return None
        return wanted
    for slot, held_name in names.items():
        if held_name is not None and held_name.lower() == name.lower():
            return slot
    if cameras:
        return cameras[0]
    for slot, held_name in names.items():
        if held_name is None:
            return slot
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
                                 pack=source.pack)


def port_link_for(args):
    """`(topology, link)` an I²C-addressed unit rides, from the bus and
    address the command resolved to (a node prefix or `--unit` lands on
    them too); None for another transport or a bus no port owns."""
    from nxs.cam import unit_source

    if getattr(args, "transport", None) == "i2c" and getattr(args, "bus", None):
        return unit_source.port_link_for_bus(args.bus, int(args.addr))
    return None


def write_manifest(directory: str, version: str) -> Dict[str, Any]:
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
    manifest: Dict[str, Any] = {"version": str(version), "personalities": entries}
    with open(os.path.join(directory, "manifest.yaml"), "w", encoding="utf-8") as fh:
        fh.write(yaml.safe_dump(manifest, sort_keys=False))
    return manifest


def compile_pack(pack_root: str, out_dir: str) -> List[str]:
    """Compile every sensor of the pack at `pack_root` that ships a
    behaviour class into `<out_dir>/<name>.nxs`; returns the names built."""
    from nxs.cam import packs

    # The pack under compilation is the one its hub classes compose from:
    # not the interpreter's installed pack, which ships no sensors.
    (pack,) = packs.prime([Path(pack_root)])
    os.makedirs(out_dir, exist_ok=True)
    built = []
    hubs = [c for c in pack.chips if c not in pack.sensors()]
    for name in pack.sensors() + hubs:
        source_path = pack.chip_source(name)
        if source_path is None:
            continue
        # A physics module beside a descriptor (a head the host programs)
        # defines no class; only a behaviour class compiles to an image.
        driver_cls = _class_in(str(source_path), name)
        if driver_cls is None:
            continue
        source = Source(KIND_CAMERA, "pack", name, path=str(source_path),
                        driver_cls=driver_cls, descriptor=pack.shipped_descriptor(name),
                        pack=pack)
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
