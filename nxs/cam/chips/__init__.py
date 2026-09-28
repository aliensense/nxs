# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Law families: the knobs, programs, and laws a sensor descriptor binds
so its .py stays the behaviour the unit runs. A family is parameterized by
the descriptor alone (registers, limits, trigger, sync, test_pattern,
program), so the same laws run from a descriptor a unit served."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, Dict, List, Tuple, Type

from nxs.cam.contracts import ContractError

from . import generic, sony_imx

#: `meta.chip` -> the family class; a descriptor naming no family is generic.
FAMILIES: Dict[str, Type[Any]] = {
    "sony_imx": sony_imx.SonyImx,
    "generic": generic.Generic,
}
GENERIC = "generic"

#: One kernel control row: (control name, register name, form, parameters).
ControlRow = Tuple[str, str, int, Tuple[int, ...]]


def family_for(descriptor) -> str:
    """The family name a descriptor binds: `meta.chip`, generic when absent."""
    meta = descriptor.raw("meta") or {}
    return str(meta.get("chip") or GENERIC)


def _family(descriptor):
    name = family_for(descriptor)
    cls = FAMILIES.get(name)
    if cls is None:
        raise ContractError(
            f"{descriptor.name}: unknown chip family {name!r}; "
            f"have {sorted(FAMILIES)}")
    return cls(descriptor)


def bind(descriptor) -> SimpleNamespace:
    """The descriptor's chip surface: a namespace of bound callables that
    answers `getattr`, `dir`, and `inspect.signature` like a serdes knob module."""
    return SimpleNamespace(**_family(descriptor).surface())


def control_rows(descriptor) -> List[ControlRow]:
    """The kernel control table's rows for a descriptor, register names
    unresolved; a control the descriptor lacks the facts for is absent."""
    return _family(descriptor).control_rows()
