# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""`nxs personality check|install`: the host half of the personality
verb group, which checks and installs sensor personalities. A unit
personality is a driver pair (`<name>.py` beside `<name>.yaml`); a camera
personality is a descriptor (`<name>.yaml`, `<name>.py`, `blobs/`) extending
the installed pack."""

from __future__ import annotations

import importlib.machinery
import importlib.util
import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
from dataclasses import dataclass
from typing import Dict, List, Optional

import yaml

from nxs.suite import PERSONALITY_DIR

UNIT = "unit"
CAMERA = "camera"


@dataclass
class Personality:
    name: str
    kind: str
    yaml_path: str
    py_path: Optional[str]
    root: Optional[str]      # the directory form's directory, if any
    doc: dict
    tmp_root: Optional[str] = None   # a tarball's extraction directory, removed by the verb


class PersonalityError(Exception):
    """A personality the tool cannot read as one; the message names why."""


class _Uncached(importlib.machinery.SourceFileLoader):
    """A source file loaded with no `__pycache__` written beside it."""

    def set_data(self, path, data, *, _mode=0o666):
        """The cache is the one thing the loader would write: dropped."""


def load_source(module_name: str, path: str):
    """The module `path` defines, run as `module_name` and registered so its
    classes find their sibling descriptor. An import that fails leaves
    nothing registered and raises as it failed."""
    spec = importlib.util.spec_from_file_location(module_name, path,
                                                  loader=_Uncached(module_name, path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(module_name, None)
        raise
    return module


HOST_VERBS = ('check', 'install')


def add_host_verbs(ps) -> None:
    """The host verbs of the `personality` group, on its subparsers:
    `check` and `install` need no unit."""
    p_check = ps.add_parser('check', help='Validate a personality before '
                                          'installing it')
    p_check.add_argument('path', help='A personality directory, one file of a '
                                      'pair, or a .tar.gz')
    p_install = ps.add_parser('install',
                              help=f'Copy a personality into the store ({PERSONALITY_DIR})')
    p_install.add_argument('path', help='A personality directory, one file of '
                                        'a pair, or a .tar.gz')
    p_install.add_argument('--extends', default=None,
                           help='The descriptor pack a camera personality '
                                'extends (default: the first installed '
                                'pack)')


def run_host_verb(args) -> int:
    if args.personality_cmd == 'check':
        return cmd_check(args.path)
    return cmd_install(args.path, args.extends)


def _refusing(verb):
    """A `nxs personality` verb refuses with one line on stderr, never a
    traceback."""
    def run(*args):
        try:
            return verb(*args)
        except PersonalityError as e:
            print(f"nxs personality: {e}", file=sys.stderr)
            return 1
    run.__name__ = verb.__name__
    run.__doc__ = verb.__doc__
    return run


# ── locating and reading ─────────────────────────────────────────────

def locate(path: str) -> Personality:
    """The personality at `path`: a directory, one file of a pair, or a
    tarball holding one directory."""
    if not os.path.exists(path):
        raise PersonalityError(f"{path}: no such file or directory")
    if os.path.isfile(path) and (path.endswith(".tar.gz") or path.endswith(".tgz")):
        return _locate_tarball(path)
    if os.path.isdir(path):
        yaml_path = _yaml_in(path)
        return _read(yaml_path, root=path)
    stem, ext = os.path.splitext(path)
    if ext not in (".py", ".yaml"):
        raise PersonalityError(f"{path}: a personality is a directory, a .py or .yaml of "
                         f"a pair, or a .tar.gz")
    return _read(stem + ".yaml", root=None)


def _yaml_in(directory: str) -> str:
    base = os.path.basename(os.path.normpath(directory))
    named = os.path.join(directory, f"{base}.yaml")
    if os.path.exists(named):
        return named
    found = sorted(f for f in os.listdir(directory) if f.endswith(".yaml")
                   and f != "pack.yaml")
    if len(found) != 1:
        raise PersonalityError(f"{directory}: expected {base}.yaml (one descriptor); "
                         f"found {found or 'none'}")
    return os.path.join(directory, found[0])


def _locate_tarball(path: str) -> Personality:
    """The personality inside a tarball, extracted into a directory the verb
    that asked removes again (`Personality.tmp_root`); a refusal removes it here."""
    tmp = tempfile.mkdtemp(prefix="nxs-personality-")
    try:
        with tarfile.open(path) as tar:
            members = tar.getmembers()
            for member in members:
                # Regular files and directories inside the archive's own tree only:
                # a link or a device entry could write outside the extraction dir.
                if member.name.startswith("/") or ".." in member.name.split("/"):
                    raise PersonalityError(f"{path}: refuses to extract {member.name}")
                if not (member.isfile() or member.isdir()):
                    raise PersonalityError(f"{path}: refuses to extract {member.name} "
                                     f"(not a regular file or directory)")
            try:
                tar.extractall(tmp, members=members, filter="data")
            except TypeError:              # a Python without tar filters
                tar.extractall(tmp, members=members)
        dirs = [d for d in sorted(os.listdir(tmp)) if os.path.isdir(os.path.join(tmp, d))]
        if len(dirs) != 1:
            raise PersonalityError(f"{path}: expected one personality directory at the top, "
                             f"found {dirs or 'none'}")
        personality = locate(os.path.join(tmp, dirs[0]))
    except PersonalityError:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    except (tarfile.TarError, OSError, EOFError) as e:
        # A corrupt or truncated archive is a refusal, not a traceback.
        shutil.rmtree(tmp, ignore_errors=True)
        raise PersonalityError(f"{path}: not a readable tar.gz: {e}") from None
    personality.tmp_root = tmp
    return personality


def discard(personality: Personality) -> None:
    """Remove a tarball's extraction directory once the verb is done."""
    if personality.tmp_root:
        shutil.rmtree(personality.tmp_root, ignore_errors=True)
        personality.tmp_root = None


#: A unit personality is a Python module name; a camera personality is a pack chip
#: name, which is also the directory name the loader expects.
_UNIT_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_CAMERA_NAME = re.compile(r"^[a-z][a-z0-9_]*$")


def _read(yaml_path: str, root: Optional[str]) -> Personality:
    if not os.path.exists(yaml_path):
        raise PersonalityError(f"{yaml_path}: missing (a personality is a .py beside its .yaml)")
    try:
        with open(yaml_path, encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
    except yaml.YAMLError as e:
        raise PersonalityError(f"{yaml_path}: not valid YAML: {e}") from None
    except OSError as e:
        raise PersonalityError(f"{yaml_path}: {e.strerror}") from None
    if not isinstance(doc, dict) or not isinstance(doc.get("meta"), dict):
        raise PersonalityError(f"{yaml_path}: expected a mapping with a meta: block")
    name = os.path.splitext(os.path.basename(yaml_path))[0]
    kind = kind_of(doc, yaml_path)
    pattern = _UNIT_NAME if kind == UNIT else _CAMERA_NAME
    if not pattern.fullmatch(name):
        raise PersonalityError(f"{yaml_path}: {name!r} is not a {kind} personality name "
                         f"({pattern.pattern})")
    py_path = os.path.splitext(yaml_path)[0] + ".py"
    return Personality(name=name, kind=kind, yaml_path=yaml_path,
                 py_path=py_path if os.path.exists(py_path) else None,
                 root=root, doc=doc)


def kind_of(doc: dict, where: str) -> str:
    """`meta.kind`, or the kind the descriptor's shape implies."""
    meta = doc.get("meta") or {}
    kind = meta.get("kind")
    if kind in (UNIT, CAMERA):
        return kind
    if kind is not None:
        raise PersonalityError(f"{where}: meta.kind must be {UNIT} or {CAMERA}, got {kind!r}")
    if "params" in doc or "driver" in meta:
        return UNIT
    if "compatible" in meta or "registers" in doc:
        return CAMERA
    raise PersonalityError(f"{where}: cannot tell the personality kind — set meta.kind: "
                     f"{UNIT} or {CAMERA}")


# ── check ────────────────────────────────────────────────────────────

def findings(personality: Personality) -> List[str]:
    """What is wrong with a personality, empty when it is sound."""
    if personality.kind == UNIT:
        return _unit_findings(personality)
    return _camera_findings(personality)


def _unit_findings(personality: Personality) -> List[str]:
    from nxs.compiler import CompileError, SensorDriver
    from nxs.descriptor import _load_descriptor_file

    problems: List[str] = []
    try:
        doc = _load_descriptor_file(personality.yaml_path)
    except ValueError as e:
        return [str(e)]
    declared = (doc.get("meta") or {}).get("driver")
    if declared != personality.name:
        problems.append(f"{personality.yaml_path}: meta.driver is {declared!r}, "
                        f"the file is {personality.name}")
    if personality.py_path is None:
        problems.append(f"{personality.name}.py: missing beside {personality.yaml_path}")
        return problems
    try:
        mod = load_source(f"nxs_local_drivers.{personality.name}", personality.py_path)
    except Exception as e:
        return problems + [f"{personality.py_path}: import failed: {e}"]
    classes = [obj for obj in vars(mod).values()
               if isinstance(obj, type) and issubclass(obj, SensorDriver)
               and obj.__module__ == mod.__name__]
    if len(classes) != 1:
        return problems + [f"{personality.py_path}: expected one SensorDriver "
                           f"subclass, found {len(classes)}"]
    cls = classes[0]
    buses = tuple(getattr(cls, "BUSES", None) or ())
    for config in ([{"bus": b} for b in buses] if len(buses) > 1 else [{}]):
        try:
            cls().compile(dict(config))
        except CompileError as e:
            problems.append(f"{personality.py_path}: {config or 'default'} does not "
                            f"compile: {e}")
        except Exception as e:             # noqa: BLE001
            # A user personality can raise anything while it is traced; that is a
            # finding against the personality, never a traceback out of the verb.
            problems.append(f"{personality.py_path}: {config or 'default'} does not "
                            f"compile: {type(e).__name__}: {e}")
    return problems


def _camera_findings(personality: Personality) -> List[str]:
    from nxs import schemas

    problems = schemas.findings(personality.doc, schemas.CAM_DESCRIPTOR,
                                where=personality.yaml_path)
    meta = personality.doc.get("meta") or {}
    if meta.get("role") != "SEN":
        problems.append(f"{personality.yaml_path}: a camera personality is a sensor "
                        f"(meta.role: SEN), got {meta.get('role')!r}")
    if personality.py_path is None:
        problems.append(f"{personality.name}.py: missing beside {personality.yaml_path} "
                        f"(the behaviour the unit runs)")
    else:
        # The behaviour compiles inside the pack's context at upload; here the
        # file is checked to be a Python module before it is installed.
        try:
            compile(open(personality.py_path, encoding="utf-8").read(), personality.py_path, "exec")
        except (SyntaxError, OSError, ValueError) as e:
            problems.append(f"{personality.py_path}: does not compile: {e}")
    here = os.path.dirname(personality.yaml_path)
    for mode, table in sorted(_mode_tables(personality.doc).items()):
        if not os.path.exists(os.path.join(here, table)):
            problems.append(f"{table}: the table of mode {mode}, missing beside "
                            f"{personality.yaml_path}")
    blob_dir = os.path.join(here, "blobs")
    for name in sorted(_referenced_blobs(personality.doc)):
        blob_path = os.path.join(blob_dir, f"{name}.blob.yaml")
        if not os.path.exists(blob_path):
            problems.append(f"blobs/{name}.blob.yaml: referenced by a mode, missing")
            continue
        try:
            with open(blob_path, encoding="utf-8") as fh:
                blob = yaml.safe_load(fh)
        except (yaml.YAMLError, OSError) as e:
            problems.append(f"blobs/{name}.blob.yaml: {e}")
            continue
        problems.extend(schemas.findings(blob, schemas.BLOB,
                                         where=f"blobs/{name}.blob.yaml"))
    return problems


def _mode_tables(doc: dict) -> Dict[str, str]:
    """The table file each offered mode names."""
    return {str(name): str(mode["table"])
            for name, mode in (doc.get("modes") or {}).items()
            if isinstance(mode, dict) and mode.get("table")}


def _bench(personality: Personality) -> str:
    """The bring-up line of a camera personality."""
    return f"nxs <port> <link> on --sensor {personality.name}"


def _referenced_blobs(doc: dict) -> set:
    """The blob names a camera descriptor's modes name: `blobs`, `final_blob`,
    `final_blob_primary`, and both `<solo_blob>_linkA` / `_linkB` halves."""
    names = set()
    for mode in (doc.get("modes") or {}).values():
        if not isinstance(mode, dict):
            continue
        names.update(str(b) for b in (mode.get("blobs") or []))
        for key in ("final_blob", "final_blob_primary"):
            if mode.get(key):
                names.add(str(mode[key]))
        if mode.get("solo_blob"):
            names.update(f"{mode['solo_blob']}_link{link}" for link in "AB")
    return names


def describe(personality: Personality) -> str:
    meta = personality.doc.get("meta") or {}
    if personality.kind == UNIT:
        params = [p.get("name") for p in personality.doc.get("params") or []
                  if isinstance(p, dict)]
        return (f"personality {personality.name}: unit · params "
                f"{', '.join(str(p) for p in params) or 'none'}")
    modes = list((personality.doc.get("modes") or {}).keys())
    return (f"personality {personality.name}: camera · {meta.get('compatible')} · "
            f"{len(modes)} mode(s)")


@_refusing
def cmd_check(path: str) -> int:
    personality = locate(path)
    try:
        problems = findings(personality)
        print(describe(personality))
        for problem in problems:
            print(f"  {problem}")
        if problems:
            return 1
        print("  ok" + (" — bench proof: nxs upload, then the validation contract"
                        if personality.kind == UNIT else
                        f" — bench proof: {_bench(personality)}, then capture --frames 60"))
        return 0
    finally:
        discard(personality)


# ── install ──────────────────────────────────────────────────────────

@_refusing
def cmd_install(path: str, extends: Optional[str]) -> int:
    personality = locate(path)
    try:
        return _install(personality, extends)
    finally:
        discard(personality)


def _install(personality: Personality, extends: Optional[str]) -> int:
    problems = findings(personality)
    if problems:
        print(describe(personality))
        for problem in problems:
            print(f"  {problem}")
        return 1
    store = PERSONALITY_DIR
    dest = os.path.join(store, personality.name)
    # The store is system-wide and root-owned; a store the user cannot
    # write is filled through sudo, staged in a temporary directory.
    escalate = not _store_writable(store) and os.geteuid() != 0
    if not escalate and not _store_writable(store):
        raise PersonalityError(f"{store} is not writable")
    # Staged beside the destination and swapped in whole, so a reinstall
    # never merges into what was there.
    stage = tempfile.mkdtemp(prefix=f".{personality.name}.", dir=None if escalate else store)
    try:
        if personality.kind == UNIT:
            _install_unit(personality, stage)
            nxt = f"nxs upload {personality.name}"
        else:
            base = _base_pack(extends)
            _install_camera(personality, stage, base)
            nxt = _bench(personality)
        if escalate:
            _place_as_root(stage, store, dest)
        else:
            if os.path.isdir(dest):
                shutil.rmtree(dest)
            os.replace(stage, dest)
    finally:
        shutil.rmtree(stage, ignore_errors=True)
    print(f"installed {personality.name} ({personality.kind}) into {dest}; next: {nxt}")
    return 0


def _store_writable(store: str) -> bool:
    """Whether this user can write the store, creating it when absent."""
    if not os.path.isdir(store):
        try:
            os.makedirs(store)
        except OSError:
            return False
    return os.access(store, os.W_OK)


def _place_as_root(stage: str, store: str, dest: str) -> None:
    """Put a staged personality into a root-owned store through sudo (a terminal is
    asked for the password, elsewhere sudo refuses); the copy lands root-owned."""
    from nxs.host.root import as_root

    # The copy and the ownership change land on a sibling first, so a
    # failure in either leaves the installed personality as it was.
    staged = f"{dest}.incoming"
    steps = (["mkdir", "-p", store],
             ["rm", "-rf", staged],
             ["cp", "-r", stage, staged],
             ["chown", "-R", "root:root", staged],
             ["chmod", "-R", "u=rwX,go=rX", staged],
             ["rm", "-rf", dest],
             ["mv", staged, dest])
    for argv in steps:
        try:
            proc = subprocess.run(as_root(argv), capture_output=True, text=True)
        except OSError as e:
            subprocess.run(as_root(["rm", "-rf", staged]), capture_output=True)
            raise PersonalityError(f"cannot write {store}: {e}") from None
        if proc.returncode != 0:
            why = (proc.stderr or "").strip() or "sudo refused"
            subprocess.run(as_root(["rm", "-rf", staged]), capture_output=True)
            raise PersonalityError(f"cannot write {store}: {why}")


def _copy_files(personality: Personality, dest: str) -> None:
    os.makedirs(dest, exist_ok=True)
    if personality.root:
        for entry in os.listdir(personality.root):
            if entry in ("__pycache__", "pack.yaml") or entry.endswith(".pyc"):
                continue
            src = os.path.join(personality.root, entry)
            target = os.path.join(dest, entry)
            if os.path.isdir(src):
                shutil.copytree(src, target, dirs_exist_ok=True,
                                ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
            else:
                shutil.copy2(src, target)
    else:
        shutil.copy2(personality.yaml_path, dest)
        shutil.copy2(personality.py_path, dest)
        if personality.kind == CAMERA:
            # The pair's tables and blobs beside it: `<name>_*.yaml` by the
            # naming convention, and the blobs directory when there is one.
            here = os.path.dirname(personality.yaml_path)
            for entry in sorted(os.listdir(here)):
                src = os.path.join(here, entry)
                if (entry.startswith(f"{personality.name}_") and os.path.isfile(src)
                        and entry.endswith((".yaml", ".yml", ".csv"))):
                    shutil.copy2(src, dest)
                elif entry == "blobs" and os.path.isdir(src):
                    shutil.copytree(src, os.path.join(dest, entry), dirs_exist_ok=True,
                                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))


def _install_unit(personality: Personality, dest: str) -> None:
    _copy_files(personality, dest)


def _install_camera(personality: Personality, dest: str, base: str) -> None:
    """A camera personality is an extension of the installed pack: the store
    entry carries a pack.yaml naming the pack it extends, and the chip
    directory the loader expects (`<name>/<name>.yaml`)."""
    from nxs.cam.packs import PACK_API

    os.makedirs(dest, exist_ok=True)
    _copy_files(personality, os.path.join(dest, personality.name))
    with open(os.path.join(dest, "pack.yaml"), "w", encoding="utf-8") as fh:
        fh.write(f"extends: {base}\napi: {PACK_API}\nchips: [{personality.name}]\n")


def _base_pack(extends: Optional[str]) -> str:
    """The pack a camera personality extends: the one named, checked against
    the installed packs, else the first installed one."""
    from nxs.cam import packs

    try:
        names = [p.name for p in packs.discover()]
        known = [p.name for p in packs.all_packs()]
    except packs.PackError as e:
        raise PersonalityError(f"a camera personality extends the installed descriptor "
                         f"pack, which did not load: {e}") from None
    if extends is not None:
        if extends not in known:
            raise PersonalityError(f"--extends {extends}: no such pack is installed "
                             f"(found: {', '.join(names) or 'none'})")
        return extends
    if not names:
        # The tool's own pack is never the default: a personality bound to
        # it by omission would serve no hub. It is named on purpose.
        raise PersonalityError("a camera personality extends the installed descriptor "
                         "pack, and none is installed (searched "
                         + ", ".join(str(p) for p in packs.search_paths())
                         + "); install the pack, or name one with --extends "
                         + " | ".join(known))
    return names[0]
