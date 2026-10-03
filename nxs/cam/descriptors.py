# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Descriptor loading and the raw-step builders chip modules compose with.
Descriptors emit raw step dicts in the engine's YAML shape, so a composed plan
is itself a valid config. Codenames DES, SER, SEN map onto the ``ADR_*`` aliases."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from .contracts import ContractError


def to_int(value: Any) -> int:
    """A descriptor integer as the schema admits it: an integer as is, a
    string by the shared spelling rule (`0x1010`, `4112`, `010`)."""
    from nxs.ints import parse_int

    if isinstance(value, bool):
        raise ContractError(f"expected an integer, got {value!r}")
    if isinstance(value, int):
        return value
    return parse_int(value)

# Canonical device codenames and their corpus alias names.
DES = "DES"
SER = "SER"
SEN = "SEN"

ROLE_TO_ALIAS: Dict[str, str] = {
    DES: "ADR_DESERIALIZER",
    SER: "ADR_SERIALIZER",
    SEN: "ADR_SENSOR",
}
ALIAS_TO_ROLE: Dict[str, str] = {v: k for k, v in ROLE_TO_ALIAS.items()}

#: Default alias block for composed configs. `CMD_UNIT_PROGRAM` marks the
#: point where a link's unit runs its camera personality; the engine
#: never executes it (the CLI splits the plan there).
STANDARD_ALIASES: Dict[str, Dict[str, Any]] = {
    "commands": {"CMD_WAIT_MILLIS": 2, "CMD_UNIT_PROGRAM": 3},
    "addresses": {
        "ADR_SENSOR": "0x1A",
        "ADR_SERIALIZER": "0x42",
        "ADR_DESERIALIZER": "0x6A",
    },
}

#: The trigger preset every camera personality offers (internal timing).
FREERUN = "freerun"


def unit_program(link: str, mode: str, trigger: str = FREERUN) -> Dict[str, Any]:
    """Build the marker step at which `link`'s unit runs its camera
    personality with `mode` and `trigger` staged; the facts ride the
    plan's side table (`RawConfig.unit_programs`), the step only names
    them."""
    return {"cmd": "CMD_UNIT_PROGRAM",
            "comment": f"link {link}: unit program mode {mode}, trigger {trigger}"}


def is_unit_program(step: Any) -> bool:
    """Whether a raw step is the unit-program marker."""
    return isinstance(step, dict) and step.get("cmd") == "CMD_UNIT_PROGRAM"


def _split_reg16(reg16: int) -> tuple[str, str]:
    """Split a 16-bit register address into the corpus reg/offset strings."""
    return f"0x{(reg16 >> 8) & 0xFF:02X}", f"0x{reg16 & 0xFF:02X}"


def w(
    role: str,
    reg16: int,
    value: int | List[int],
    sleep_ms: int = 0,
    comment: Optional[str] = None,
) -> Dict[str, Any]:
    """Build a raw device-write step for ``role`` (DES / SER / SEN) at the 16-bit
    ``reg16``; ``value`` is a byte or a list of bytes."""
    reg, offset = _split_reg16(reg16)
    step: Dict[str, Any] = {
        "device": ROLE_TO_ALIAS[role],
        "reg": reg,
        "offset": offset,
        "value": (
            [f"0x{v & 0xFF:02X}" for v in value]
            if isinstance(value, list)
            else f"0x{value & 0xFF:02X}"
        ),
    }
    if sleep_ms:
        step["sleep_ms"] = sleep_ms
    if comment:
        step["comment"] = comment
    return step


def wait_ms(ms: int) -> Dict[str, Any]:
    """Build a raw wait command step."""
    return {"cmd": "CMD_WAIT_MILLIS", "args": [1, ms]}


def rd(
    role: str,
    reg16: int,
    length: int = 1,
    store: Optional[str] = None,
    comment: Optional[str] = None,
) -> Dict[str, Any]:
    """Build a raw read step."""
    reg, offset = _split_reg16(reg16)
    step: Dict[str, Any] = {
        "read": ROLE_TO_ALIAS[role],
        "reg": reg,
        "offset": offset,
    }
    if length != 1:
        step["length"] = length
    if store:
        step["store"] = store
    if comment:
        step["comment"] = comment
    return step


def expect(
    role: str,
    reg16: int,
    value: int,
    mask: int = 0xFF,
    timeout_ms: int = 0,
    poll_ms: int = 50,
    comment: Optional[str] = None,
    soft: bool = False,
) -> Dict[str, Any]:
    """Build a raw expect step (polled when timeout_ms > 0). ``soft`` makes it a
    settle poll: the engine moves on at the deadline with a warning."""
    reg, offset = _split_reg16(reg16)
    step: Dict[str, Any] = {
        "expect": ROLE_TO_ALIAS[role],
        "reg": reg,
        "offset": offset,
        "value": f"0x{value & 0xFF:02X}",
    }
    if mask != 0xFF:
        step["mask"] = mask
    if timeout_ms:
        step["timeout_ms"] = timeout_ms
        step["poll_ms"] = poll_ms
    if soft:
        if not timeout_ms:
            raise ValueError("a soft expect needs timeout_ms > 0")
        step["soft"] = True
    if comment:
        step["comment"] = comment
    return step


def retry(
    times: int,
    steps: List[Dict[str, Any]],
    delay_ms: int = 0,
    on_fail: str = "abort",
    comment: Optional[str] = None,
) -> Dict[str, Any]:
    """Build a raw retry-block step."""
    step: Dict[str, Any] = {"retry": times, "steps": steps}
    if delay_ms:
        step["delay_ms"] = delay_ms
    if on_fail != "abort":
        step["on_fail"] = on_fail
    if comment:
        step["comment"] = comment
    return step


def resolve_mode(descriptor, token: str) -> str:
    """A mode token (exact name or WxH geometry) to a mode the unit program
    offers; ambiguity, unknowns, and a mode the program does not offer
    raise InfeasibleConfig listing what it does."""
    from .contracts import InfeasibleConfig

    modes = descriptor.modes
    offered = descriptor.program_modes()
    if token in modes and token not in offered:
        raise InfeasibleConfig(
            f"mode {token!r}: {descriptor.unoffered_reason(token)}",
            alternatives=list(offered))
    if token in modes:
        return token
    token_l = token.lower().replace("×", "x")
    matches = []
    unoffered = []
    for name, mode in modes.items():
        geo = mode.get("geometry") or {}
        if f"{geo.get('width')}x{geo.get('height')}" == token_l:
            (matches if name in offered else unoffered).append(name)
    if len(matches) == 1:
        return matches[0]
    have = sorted({f"{modes[n]['geometry']['width']}x{modes[n]['geometry']['height']}"
                   for n in offered if modes[n].get("geometry")})
    if matches:
        raise InfeasibleConfig(
            f"{token!r} matches several modes",
            alternatives=sorted(matches))
    if unoffered:
        why = "; ".join(f"{n}: {descriptor.unoffered_reason(n)}" for n in unoffered)
        raise InfeasibleConfig(
            f"{token!r} names no mode the unit program offers ({why})",
            alternatives=have + list(offered))
    raise InfeasibleConfig(
        f"no mode {token!r}", alternatives=have + list(offered))


def mode_label(descriptor, name: str) -> str:
    """How a mode is named to a person: `1920x1080 RAW10`."""
    mode = descriptor.modes.get(name) or {}
    geo = mode.get("geometry") or {}
    dt = (mode.get("mipi") or {}).get("data_type") or f"RAW{geo.get('bit_depth', '?')}"
    return f"{geo.get('width')}x{geo.get('height')} {dt}"


def mode_token(descriptor, name: str) -> str:
    """The token `--mode` takes for a mode: its geometry, or the geometry
    with the depth when another mode shares it (WxH-rawN)."""
    geo = descriptor.modes[name].get("geometry") or {}
    token = f"{geo.get('width')}x{geo.get('height')}"
    shared = [n for n, m in descriptor.modes.items()
              if f"{(m.get('geometry') or {}).get('width')}x"
                 f"{(m.get('geometry') or {}).get('height')}" == token]
    if len(shared) > 1:
        token += f"-raw{geo.get('bit_depth')}"
    return token


#: The per-mode timing keys a bench measures; a shipped descriptor never
#: states them, a bench overlay may, and a unit-served descriptor decoded
#: from an image drops them.
EXPERIMENTAL_TIMING_KEYS = frozenset({
    "vmax_clean", "vmax_jump_threshold", "trigger_vmax", "framerate_cap",
    "exposure_us", "gain"})


def experimental_timing_keys(data: Dict[str, Any]) -> List[str]:
    """`<mode>.<key>` for every experimental timing key a descriptor mapping
    states, and `limits.<name>` for every limit sourced `experimental`."""
    found: List[str] = []
    for name, mode in (data.get("modes") or {}).items():
        timing = (mode or {}).get("timing") or {}
        found += [f"{name}.{k}" for k in sorted(EXPERIMENTAL_TIMING_KEYS & set(timing))]
    found += [f"limits.{k}" for k, v in (data.get("limits_source") or {}).items()
              if str(v) == "experimental"]
    return found


def _refuse_experimental_keys(data: Any, path: Path) -> None:
    # A yaml that is not a mapping is the schema's to refuse.
    found = experimental_timing_keys(data) if isinstance(data, dict) else []
    if found:
        raise ContractError(
            f"{path}: a shipped descriptor states experimental values {found}; "
            f"they belong in the pack's experimental overlay (descriptors-experimental)")


def _deep_merge(base: Dict[str, Any], over: Dict[str, Any]) -> Dict[str, Any]:
    """Merge an overlay over a base: mappings merge key by key, anything else
    replaces outright."""
    merged = dict(base)
    for key, value in over.items():
        current = merged.get(key)
        if isinstance(current, dict) and isinstance(value, dict):
            merged[key] = _deep_merge(current, value)
        else:
            merged[key] = value
    return merged


class Descriptor:
    """One chip's descriptor: the YAML facts plus its blob library; the law
    family ``meta.chip`` names (or a paired ``<chip>.py`` module) holds the
    computed knobs."""

    def __init__(self, root: Path, overlay: Optional[Path] = None) -> None:
        """Load the descriptor in ``root`` (``<dirname>.yaml`` and ``blobs/``), with
        an ``overlay`` directory of the same shape merged over it: overlay values
        win and its blobs are searched first."""
        self._root = root
        self._overlay = overlay
        # The descriptor a port's view (`at_lines`) lays its lines over; a
        # personality never compiles from a view.
        self._plain: Optional["Descriptor"] = None
        path = root / f"{root.name}.yaml"
        data = yaml.safe_load(path.read_text())
        # The shipped yaml carries transcription; an experimental key in it
        # would ship a measurement as the part.
        _refuse_experimental_keys(data, path)
        if overlay is not None:
            over_path = overlay / f"{overlay.name}.yaml"
            extra = yaml.safe_load(over_path.read_text()) or {}
            # Gate the overlay on its own schema before the merge, so it cannot
            # carry a datasheet source or override a limit without its source.
            from nxs import schemas as _schemas
            problems = _schemas.findings(extra, _schemas.CAM_OVERLAY,
                                         where=str(over_path))
            if problems:
                raise ContractError("; ".join(problems))
            data = _deep_merge(data, extra)
            path = over_path
        # The descriptor schema is the loader's first gate: an unknown key or
        # a malformed value is refused here, naming the file.
        from nxs import schemas
        problems = schemas.findings(data, schemas.CAM_DESCRIPTOR, where=str(path))
        if problems:
            raise ContractError("; ".join(problems))
        self._data: Dict[str, Any] = data
        self._blob_cache: Dict[str, Dict[str, Any]] = {}
        self._check_provenance(path)

    @classmethod
    def from_data(cls, data: Dict[str, Any], name: str) -> "Descriptor":
        """A descriptor from a YAML-shaped mapping with no directory behind
        it (one a unit served): the schema gates it, it carries no blobs,
        and its limits name no sources."""
        from nxs import schemas
        problems = schemas.findings(data, schemas.CAM_DESCRIPTOR, where=name)
        if problems:
            raise ContractError("; ".join(problems))
        self = cls.__new__(cls)
        self._root = None
        self._overlay = None
        self._plain = None
        self._name = str(name)
        self._data = dict(data)
        self._blob_cache = {}
        return self

    def _check_provenance(self, path: Path) -> None:
        """Every limit names its source; a limit without one, or a source without
        a limit, is refused."""
        limits = self._data.get("limits") or {}
        sources = self._data.get("limits_source") or {}
        if not limits and not sources:
            return
        missing = sorted(set(limits) - set(sources))
        if missing:
            raise ContractError(
                f"{path}: no limits_source for {missing}")
        extra = sorted(set(sources) - set(limits))
        if extra:
            raise ContractError(
                f"{path}: limits_source names absent limits {extra}")

    @property
    def limits_source(self) -> Dict[str, str]:
        """Where each limit came from: datasheet, driver, or experimental."""
        return {k: str(v)
                for k, v in (self._data.get("limits_source") or {}).items()}

    def experimental_limits(self) -> List[str]:
        """Limits an experimental overlay measured (present only under the flag);
        what a new board revision has to re-prove."""
        return sorted(k for k, v in self.limits_source.items()
                      if v == "experimental")

    def at_lines(self, lines: Optional[Dict[str, int]]) -> "Descriptor":
        """The view of a port that runs `lines` (mode -> HMAX): each named
        mode's line laid over its timing, never shorter than the mode's
        own; self when no line is longer than its mode's. A view carries
        nothing else of the port, and a personality never compiles from
        one (`product`)."""
        over: Dict[str, Any] = {}
        for name, line in (lines or {}).items():
            timing = (self.modes.get(name) or {}).get("timing") or {}
            if timing.get("hmax") is not None and int(line) > to_int(timing["hmax"]):
                over[name] = {"timing": {"hmax": int(line)}}
        if not over:
            return self
        view = copy.copy(self)
        view._plain = self._plain if self._plain is not None else self
        view._data = _deep_merge(self._data, {"modes": over})
        return view

    @property
    def name(self) -> str:
        return self._root.name if self._root is not None else self._name

    @property
    def source(self) -> Optional[Path]:
        """The yaml the base facts came from; None for a unit-served
        descriptor."""
        return self._root / f"{self._root.name}.yaml" if self._root is not None else None

    @property
    def from_unit(self) -> bool:
        """True for a descriptor a unit served (no directory, no blobs)."""
        return self._root is None

    @property
    def compatible(self) -> str:
        return str(self._data["meta"]["compatible"])

    @property
    def role(self) -> str:
        return str(self._data["meta"]["role"])

    @property
    def registers(self) -> Dict[str, Any]:
        return dict(self._data.get("registers") or {})

    @property
    def limits(self) -> Dict[str, Any]:
        return dict(self._data.get("limits") or {})

    @property
    def modes(self) -> Dict[str, Any]:
        return dict(self._data.get("modes") or {})

    def program_modes(self) -> List[str]:
        """The modes the unit program offers, in declaration order; a mode
        flagged `host_only` is judged by the laws only. The `mode` param's
        value of each is its index here."""
        return [name for name, mode in self.modes.items()
                if not mode.get("host_only")]

    def mode_value(self, name: str) -> int:
        """The `mode` param value that selects `name`; InfeasibleConfig for a
        mode the unit program does not offer."""
        from .contracts import InfeasibleConfig

        offered = self.program_modes()
        if name not in offered:
            raise InfeasibleConfig(
                f"mode {name!r}: {self.unoffered_reason(name)}",
                alternatives=list(offered))
        return offered.index(name)

    def unoffered_reason(self, name: str) -> str:
        """Why the unit program does not offer `name` (a declared mode)."""
        mode = self.modes.get(name) or {}
        if mode.get("host_only"):
            return "no unit program for it yet; the laws still judge it"
        return "not a declared mode"

    def program_triggers(self) -> List[str]:
        """The trigger presets in `trigger` value order: `freerun` first
        (every head has it), then the other `trigger:` presets carrying a
        `trigmode` in declaration order. A behaviour class implements a
        prefix of them (its `trigger` param values); a head without a
        `trigger:` block declares no `trigger` param."""
        presets = self.raw("trigger") or {}
        declared = [str(k) for k, v in presets.items()
                    if isinstance(v, dict) and "trigmode" in v]
        return [FREERUN] + [t for t in declared if t != FREERUN]

    def trigger_value(self, name: str) -> int:
        """The `trigger` param value that selects `name`; InfeasibleConfig
        for a preset the descriptor does not declare."""
        from .contracts import InfeasibleConfig

        offered = self.program_triggers()
        if name not in offered:
            raise InfeasibleConfig(
                f"{self.compatible} has no {name!r} trigger program",
                alternatives=[f"trigger {t}" for t in offered])
        return offered.index(name)

    @property
    def runtime_forbidden(self) -> List[int]:
        """16-bit register addresses that must never be written at runtime."""
        return [to_int(x) for x in self._data.get("runtime_forbidden") or []]

    def raw(self, key: str) -> Any:
        """Return an arbitrary top-level section of the descriptor YAML."""
        return self._data.get(key)

    def product(self) -> "Descriptor":
        """The descriptor as the product ships it: this one without a
        port's view and without its experimental overlay (itself when
        neither is merged, or when a unit served it): what a personality
        compiles from."""
        plain = self._plain if self._plain is not None else self
        if plain._root is None or plain._overlay is None:
            return plain
        return Descriptor(plain._root, None)

    def digest(self) -> str:
        """A digest of every fact the descriptor holds (the merged data):
        two descriptors of one part differ when an overlay does."""
        import hashlib
        import json

        canonical = json.dumps(self._data, sort_keys=True, default=str)
        return hashlib.sha1(canonical.encode()).hexdigest()

    def reg(self, name: str) -> int:
        """The 16-bit address of a named register; KeyError when it is not declared."""
        return to_int(self._data["registers"][name]["addr"])

    def has_blob(self, name: str) -> bool:
        """Whether ``blobs/<name>.blob.yaml`` ships with the descriptor."""
        if self._root is None:
            return False
        return (self._root / "blobs" / f"{name}.blob.yaml").exists()

    def blob(self, name: str) -> Dict[str, Any]:
        """Load ``blobs/<name>.blob.yaml`` (the overlay's copy first) as its raw
        config dict: meta / aliases / event_list / sequences."""
        if self._root is None:
            raise ContractError(
                f"{self.name}: a descriptor a unit served carries no blobs "
                f"({name!r}); its programs run on the unit")
        if name not in self._blob_cache:
            path = self._root / "blobs" / f"{name}.blob.yaml"
            if self._overlay is not None:
                over = self._overlay / "blobs" / f"{name}.blob.yaml"
                if over.exists():
                    path = over
            raw = yaml.safe_load(path.read_text())
            # The blob schema is the loader's contract too: an unknown step key
            # (a misspelt sleep_ms) would otherwise silently change a sequence.
            from nxs import schemas
            problems = schemas.findings(raw, schemas.BLOB, where=str(path))
            if problems:
                raise ContractError("; ".join(problems))
            self._blob_cache[name] = raw
        return self._blob_cache[name]

    def blob_sequences(self, name: str) -> List[tuple[str, List[Dict[str, Any]]]]:
        """Return a blob's sequences as (name, raw steps) in event order."""
        blob = self.blob(name)
        return [
            (seq_name, blob["sequences"][seq_name])
            for seq_name in blob["event_list"]
        ]


