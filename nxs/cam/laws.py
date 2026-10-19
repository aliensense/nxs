# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The laws a cam personality's facts bind. A law family is code beside the
cam personalities, `<family>/<family>.py` with no facts file, parameterized
by the facts alone (registers, limits, trigger, sync, test_pattern,
program), so the same laws run from facts a unit served. `meta.chip` names
a sensor's family. The loader finds the family's directory on the cam
personalities' search path, the source tree's `cam_personalities/` last,
and loads its module by path. A family module's `bind(descriptor)` returns
the family bound to the facts, which names what it offers (`surface()`) and
the kernel controls it derives (`control_rows()`). What it offers includes
`options(view)`, the options a declaration may set on its camera, which the
loader registers with the options hook (`nxs.suite.options`) when the facts'
laws bind; `camera_options` is the share every family takes.

The apply law is the contract between a family and the compiler: the
constants a camera personality's `apply` action compiles into the block the
unit runs under the sensor's group hold."""

from __future__ import annotations

import dataclasses
import importlib.util
import math
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple

from nxs.cam.contracts import ContractError

#: The family of a descriptor whose facts name none.
GENERIC = "generic"
#: The family of a head that programs its own sensor and serves the camera
#: map alone, the smart camera.
NATIVE = "native"
#: One kernel control row: (control name, register name, form, parameters).
ControlRow = Tuple[str, str, int, Tuple[int, ...]]
#: The namespace a family's modules load under, one package per family so a
#: family module imports its siblings by a relative import.
_FAMILY_PACKAGE = "nxs_cam_laws"
#: The namespace a cam personality's own physics module loads under.
_PHYSICS_PACKAGE = "nxs_cam_personality"


@dataclasses.dataclass(frozen=True)
class RegSpec:
    """A sensor register the apply writes: its address, its width in
    bytes and its byte order."""

    addr: int
    width: int
    little: bool

    @classmethod
    def of(cls, registers: Dict[str, Any], name: str) -> "RegSpec":
        """The spec of the declared register `name`."""
        spec = registers[name]
        return cls(addr=int(str(spec["addr"]), 0), width=int(spec.get("width", 1)),
                   little=str(spec.get("order", "le")) != "be")


@dataclasses.dataclass(frozen=True)
class ApplyLaw:
    """What one apply writes and the bounds it clamps to.

    The frame length is clamped to `frame_length_min..frame_length_max`,
    and under `frame_length_floors`, one floor per unit mode in `mode`
    value order, to the running mode's own floor.
    The exposure, in lines, is clamped to `exposure_min` from below and,
    on an inverted-shutter part, to the frame length less
    `exposure_margin` from above; the shutter register then takes
    `frame_length - lines`, floored at `shs_floor`. A part whose exposure
    register counts lines directly takes the lines as they are. The gain
    register takes `value * gain_num // gain_den`, capped at
    `gain_reg_max`; the value itself is in the capture stack's unit,
    `gain_min..gain_max` by `gain_step`, `gain_factor` units per decibel.
    `line_times_ns` is each unit mode's line period in `mode` value order,
    what the camera map converts its microseconds with. A part names its
    black-level register with its DN bounds, its flip register with the
    bit of each axis, and the status probe its temperature reads from;
    the camera map offers the field for each one named. A native head
    converts and writes in its own logic: its law names no register
    (`frame_length`, `shutter` and `gain` are None, and the map stages no
    apply parameter) and lists in `serves` the fields the head offers
    beyond the three, `black_level`, `flips` and `temperature`. It also
    keeps its bounds in microseconds as its facts state them, the rows the
    camera map serves: the exposure's floor and margin, the longest frame
    and each unit mode's shortest. A law of lines leaves them empty."""

    hold: Optional[RegSpec]
    frame_length: Optional[RegSpec]
    shutter: Optional[RegSpec]
    gain: Optional[RegSpec]
    frame_length_min: int
    frame_length_max: int
    exposure_min: int
    exposure_margin: int
    shs_floor: int
    shutter_inverted: bool
    gain_num: int
    gain_den: int
    gain_reg_max: int
    gain_min: int
    gain_max: int
    gain_step: int
    gain_factor: int
    stream_gate: bool
    frame_length_floors: Tuple[int, ...] = ()
    line_times_ns: Tuple[int, ...] = ()
    black_level: Optional[RegSpec] = None
    black_level_min: int = 0
    black_level_max: int = 0
    flips: Optional[RegSpec] = None
    flip_x_bit: int = 0
    flip_y_bit: int = 1
    temperature: Optional[str] = None
    serves: Tuple[str, ...] = ()
    exposure_min_us: int = 0
    exposure_margin_us: int = 0
    frame_period_max_us: int = 0
    frame_periods_min_us: Tuple[int, ...] = ()


# ── finding and loading a family ─────────────────────────────────────────

#: Loaded modules by their file: the families' and the personalities' own.
_modules: Dict[Path, ModuleType] = {}


def reset_cache() -> None:
    """Forget the loaded modules (test seam; also after the cam
    personalities' search path changes)."""
    _modules.clear()


def _roots() -> List[Path]:
    """Where a family is read from, most specific first: the roots the cam
    personality registry reads, then the source tree's directory."""
    from nxs.cam import cam_personalities

    roots = list(cam_personalities.registry().roots)
    tree = cam_personalities.tree_dir()
    if tree.is_dir() and tree not in roots:
        roots.append(tree)
    return roots


def is_family(directory: Path) -> bool:
    """Whether `directory` holds a law family: its module and no facts."""
    name = directory.name
    return (directory / f"{name}.py").is_file() and not (directory / f"{name}.yaml").exists()


def families() -> List[str]:
    """Every family the search path offers, by name."""
    return sorted({child.name for root in _roots() if root.is_dir()
                   for child in root.iterdir() if child.is_dir() and is_family(child)})


def family(name: str) -> ModuleType:
    """The module of the family `name`, loaded from the first root that
    carries it (cached), in a package of its own so it imports its sibling
    modules by a relative import. ContractError names the families offered
    when no root carries it."""
    directory = next((root / name for root in _roots() if is_family(root / name)), None)
    if directory is None:
        raise ContractError(f"no law family {name!r} beside the cam personalities; "
                            f"have {families()}")
    path = directory / f"{name}.py"
    module = _modules.get(path)
    if module is None:
        package = f"{_FAMILY_PACKAGE}.{name}"
        _stub(_FAMILY_PACKAGE, [])
        # The siblings import from this directory: an earlier root's stay out.
        for stale in [n for n in sys.modules if n.startswith(f"{package}.")]:
            del sys.modules[stale]
        _stub(package, [str(directory)], replace=True)
        module = _exec(f"{package}.{name}", path, ContractError)
        _modules[path] = module
    return module


def _stub(name: str, path: List[str], replace: bool = False) -> None:
    """A package of nothing but its search path, so a dotted module name
    resolves under it."""
    if replace or name not in sys.modules:
        stub = ModuleType(name)
        stub.__path__ = path
        sys.modules[name] = stub


def _exec(name: str, path: Path, error: type, facts=None) -> ModuleType:
    """The module at `path` run under `name`, `facts` set as its `FACTS`
    first; `error` names the file when it does not load."""
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise error(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    if facts is not None:
        module.FACTS = facts
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except Exception as exc:
        sys.modules.pop(name, None)
        raise error(f"error loading {path}: {exc}") from exc
    return module


# ── binding the facts ────────────────────────────────────────────────────

def family_for(descriptor) -> str:
    """The family name a descriptor binds: `meta.chip`, generic when absent."""
    meta = descriptor.raw("meta") or {}
    return str(meta.get("chip") or GENERIC)


def bind(descriptor) -> SimpleNamespace:
    """The family the facts name bound to them: a namespace of bound
    callables that answers `getattr`, `dir`, and `inspect.signature` like a
    physics module."""
    return SimpleNamespace(**family(family_for(descriptor)).bind(descriptor).surface())


def laws(descriptor, lines: Optional[Dict[str, int]] = None) -> Optional[Any]:
    """A descriptor's laws: the family its facts name, or the generic one
    for facts a unit served, bound to the view of a port that runs `lines`
    (mode -> HMAX, `Descriptor.at_lines`). Facts naming no family take their
    own physics module, the `<name>.py` beside them, with the facts as its
    `FACTS`; None for a head with neither. A behaviour class beside facts
    naming no family is refused by file (`hubs.HubError`). Laws bound to
    the facts alone, no port's lines laid over them, register their
    `options(view)` with the options hook for the facts' compatible
    (`nxs.suite.options.register`)."""
    if (descriptor.raw("meta") or {}).get("chip") or descriptor.from_unit:
        view = descriptor.at_lines(lines)
        surface = bind(view)
    else:
        view, surface = descriptor, _physics(descriptor)
    if view is descriptor and hasattr(surface, "options"):
        from nxs.suite import options

        options.register(descriptor.compatible, surface.options)
    return surface


def _physics(descriptor) -> Optional[ModuleType]:
    """The physics module beside a head's facts (cached), None without one."""
    from nxs.cam.hubs import HubError
    from nxs.compiler import CamPersonality, HubDevice

    path = descriptor.source.with_suffix(".py") if descriptor.source is not None else None
    if path is None or not path.is_file():
        return None
    module = _modules.get(path)
    if module is None:
        _stub(_PHYSICS_PACKAGE, [])
        module = _exec(f"{_PHYSICS_PACKAGE}.{descriptor.name}", path, HubError, facts=descriptor)
        _modules[path] = module
    if any(isinstance(obj, type) and issubclass(obj, CamPersonality)
           and not issubclass(obj, HubDevice) and obj is not CamPersonality
           for obj in vars(module).values()):
        raise HubError(f"{path}: a cam personality's behaviour class needs its laws — name "
                       f"the law family in {descriptor.name}.yaml meta.chip "
                       f"({', '.join(families())})")
    return module


def control_rows(descriptor) -> List[ControlRow]:
    """The kernel control table's rows for a descriptor, register names
    unresolved; a control the descriptor lacks the facts for is absent."""
    return family(family_for(descriptor)).bind(descriptor).control_rows()


def apply_law(facts: Dict[str, Any], name: str) -> ApplyLaw:
    """The apply law of a descriptor given as its YAML mapping, the facts a
    personality module compiles against; ContractError when the family
    lacks the facts for one."""
    from nxs.cam.descriptors import Descriptor

    surface = bind(Descriptor.from_data(facts, name))
    if not hasattr(surface, "apply_law"):
        raise ContractError(f"{name}: the {family_for(surface.descriptor())} family "
                            f"derives no apply law from these facts")
    return surface.apply_law()


# ── the options a camera's laws narrow ───────────────────────────────────

#: A camera's rate row: its frame rate, in milli-frames per second.
RATE_ROW = "fps"
#: The rate row's milli-frames per second of 1 fps.
_MFPS_PER_FPS = 1000
#: A float rate this close to a whole milli-frame per second is that one.
_ROUNDING = 1e-6
#: The row a hub declares its frame sync in, and the value that runs it.
SYNC_ROW = "sync_source"
SYNC_FSYNC = 1
#: The rows a hub sets for every camera behind it.
_HUB_ROWS = ("gain", "exposure")
#: The rows a camera serves that a declaration does not stage, with what sets each.
_SET_ELSEWHERE = {
    "trigger": "the hub's frame sync sets it",
    "line_time": "the graph sets it from the pair's line",
    "black_level": "nxs <port> set <link> black_level sets it on the running camera",
    "flips": "nxs <port> set <link> flips sets it on the running camera",
}


def hub_of(view):
    """The view of the hub a camera rides: the one its `out` feeds, or the
    one past the pod its `out` feeds; None on a connector."""
    peer = view.peer_view("out")
    if peer is not None and peer.description.kind == "unit":
        peer = peer.peer_view("out")
    return peer if peer is not None and peer.description.kind == "hub" else None


def frame_synced(view) -> bool:
    """Whether the hub a camera rides declares frame sync in its
    `sync_source` row; a hub without the row runs free."""
    hub = hub_of(view)
    if hub is None:
        return False
    try:
        return hub.value(SYNC_ROW) == SYNC_FSYNC
    except KeyError:
        return False


def declared_mode(view, descriptor) -> str:
    """The mode a camera's view declares by its mode table index (`mode`),
    the facts' default mode without one."""
    modes = descriptor.program_modes()
    try:
        index = view.value("mode")
    except KeyError:
        index = -1
    return modes[index] if 0 <= index < len(modes) else str(descriptor.raw("default_mode"))


def camera_options(view, descriptor, rates) -> Dict[str, Any]:
    """What a camera's laws narrow on its rows. The rate row spans the
    rates the declared mode runs free at on the camera's port (the hub's
    range law), else `rates(mode)`, `(floor, ceiling)` in fps, as whole
    milli-frames per second inside them, held to the row's own span; under
    the hub's frame sync it is not declared, the hub sets the rate. Behind
    a hub the gain and the exposure are the hub's. The rows a declaration
    does not stage are not declared."""
    from nxs.suite.options import Option

    rows = {row.name for row in view.description.rows}
    out: Dict[str, Any] = {name: Option(applicable=False, finding=why)
                           for name, why in _SET_ELSEWHERE.items() if name in rows}
    if hub_of(view) is not None:
        out.update({name: Option(applicable=False, finding=f"behind a hub the cameras run the "
                                                           f"hub's {name}")
                    for name in _HUB_ROWS if name in rows})
    if RATE_ROW not in rows:
        return out
    row = view.description.row(RATE_ROW)
    if frame_synced(view):
        out[RATE_ROW] = Option(applicable=False, finding="under frame sync the hub sets the rate")
        return out
    mode = declared_mode(view, descriptor)
    floor, ceiling = _port_rates(view, mode) or tuple(float(fps) for fps in rates(mode))
    lo, hi, step = row.values
    fastest = min(max(lo, math.floor(ceiling * _MFPS_PER_FPS + _ROUNDING)), hi)
    slowest = min(max(lo, math.ceil(floor * _MFPS_PER_FPS - _ROUNDING)), fastest)
    runs = f"{floor:g} to {ceiling:g} fps" if floor < ceiling else f"{ceiling:g} fps only"
    out[RATE_ROW] = Option(span=(slowest, fastest, step), finding=f"{mode} runs free at {runs}")
    return out


def _port_rates(view, mode: str) -> Optional[Tuple[float, float]]:
    """`(floor, ceiling)` the hub's range law gives the mode on the camera's
    link of its port; None off a port, or for a port or a hub without the law."""
    from nxs.cam import hubs, timing

    topology = view.port()
    link = next((l for l in topology.links if l.camera_node == view.node),
                None) if topology is not None else None
    if link is None:
        return None
    try:
        rates = timing.lawful_range(hubs.for_topology(topology), topology, link, mode)
    except Exception:        # noqa: BLE001 (a mode the port refuses is status's finding)
        return None
    return (float(rates.floor), float(rates.ceiling)) if rates is not None else None
