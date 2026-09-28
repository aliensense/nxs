# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Shipped JSON Schema contracts: one per YAML the tool reads and a `$defs`
bundle for every ``--json`` document. Schemas describe shape only; timing and
feasibility laws stay in code and speak through ``nxs status``."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

from nxs.finding import Finding

from . import validator

#: Schema names, by the document family they describe.
SUITE = "suite"
UNIT_DRIVER = "unit-driver"
CAM_DESCRIPTOR = "cam-descriptor"
CAM_OVERLAY = "cam-overlay"
BLOB = "blob"
PACK = "pack"
EXPERIMENTAL = "experimental"
TOPOLOGY = "topology"
SURFACE = "surface"

#: Contract version stamped into every ``--json`` document. 2: a finding is
#: an object (where, fact, alternatives, text), and a verb refused under
#: ``--json`` prints a `refusal` document.
CONTRACT = 2

_DIR = Path(__file__).parent
_CACHE: Dict[str, Dict[str, Any]] = {}


def path(name: str) -> Path:
    """Filesystem path of a shipped schema (``<name>.schema.json``)."""
    return _DIR / f"{name}.schema.json"


def load(name: str) -> Dict[str, Any]:
    """A shipped schema as a dict (cached), every keyword one the checker knows."""
    if name not in _CACHE:
        schema = json.loads(path(name).read_text(encoding="utf-8"))
        validator.check_keywords(schema)
        _CACHE[name] = schema
    return _CACHE[name]


def names() -> List[str]:
    """Every shipped schema name."""
    return sorted(p.name[: -len(".schema.json")]
                  for p in _DIR.glob("*.schema.json"))


def _errors(schema: Dict[str, Any], doc: Any):
    return sorted(validator.errors(schema, doc), key=lambda e: list(e.path))


def _render(error, where: str) -> Finding:
    """One violation as a finding: the place, what the schema says, and the
    values the violated keyword admits as the alternatives."""
    dotted = ".".join(str(p) for p in error.absolute_path)
    place = f"{where}.{dotted}" if where and dotted else (where or dotted)
    return _SchemaFinding(place, error.message, error.allowed)


class _SchemaFinding(Finding):
    """A schema violation: its text stays the one line it always was (the
    message already names the admitted values), its data carries them."""

    def __new__(cls, where: str, fact: str, allowed=()):
        self = Finding.__new__(cls, where, fact)
        self._alternatives = tuple(str(a) for a in allowed)
        return self


def findings(doc: Any, name: str, where: str = "") -> List[Finding]:
    """Shape findings for ``doc`` against schema ``name``, one line per
    violation (YAML path first, prefixed by ``where``); empty when the document
    conforms."""
    return [_render(e, where) for e in _errors(load(name), doc)]


def surface_findings(doc: Any, surface: str) -> List[str]:
    """Findings for a ``--json`` document against its surface contract
    (a ``$defs`` key of the surface bundle)."""
    bundle = load(SURFACE)
    try:
        shape = bundle["$defs"][surface]
    except KeyError:
        raise KeyError(f"no surface {surface!r} in the contract bundle; "
                       f"have {sorted(bundle['$defs'])}") from None
    schema = dict(shape)
    schema["$defs"] = bundle["$defs"]
    return [_render(e, surface) for e in _errors(schema, doc)]


def validate(doc: Any, name: str, where: str = "") -> None:
    """Raise ``ValueError`` naming every finding when ``doc`` does not
    conform to schema ``name``."""
    found = findings(doc, name, where)
    if found:
        raise ValueError("\n".join(found))
