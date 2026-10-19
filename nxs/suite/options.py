# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""What a block's rows offer a declaration at this point of the graph: the
values each row takes from its description, narrowed by the block's own
code through one hook, and the refusal for a value outside them. The
panel, `check`, `status` and the save ask the same function."""

from __future__ import annotations

import dataclasses
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple

from nxs._generated_constants import NxsDriverImage
from nxs.description import Description, display, named_values, offered, present, unit_of

ParamKind = NxsDriverImage.ParamKind


@dataclasses.dataclass(frozen=True)
class Option:
    """What one row offers: the served values it admits here, or a span
    `(lo, hi, step)`, whether a declaration may set it at all, and the
    finding that says why not or why the values narrowed."""

    values: Optional[Tuple[int, ...]] = None
    span: Optional[Tuple[int, int, int]] = None
    applicable: bool = True
    finding: Optional[str] = None

    def admits(self, raw: int) -> bool:
        if self.values is not None:
            return int(raw) in self.values
        if self.span is not None:
            lo, hi, step = self.span
            return lo <= int(raw) <= hi and (int(raw) - lo) % max(1, step) == 0
        return True


@dataclasses.dataclass(frozen=True)
class Peer:
    """A block wired to one of this block's ports: its node name, the port
    of its own, its description and its declared values by row name."""

    node: str
    port: str
    description: Description
    declared: Mapping[str, int] = dataclasses.field(default_factory=dict)


@dataclasses.dataclass(frozen=True)
class BlockView:
    """One block as the hook sees it: its description, the served values its
    declaration states by row name, its peers by its own port, `rig`, the
    view of any other node of the graph by name (None for a block seen
    alone), and `topology`, the camera port the block's chain shapes as the
    camera laws take it (None off a camera chain, and where the host
    cannot shape the port)."""

    description: Description
    declared: Mapping[str, int] = dataclasses.field(default_factory=dict)
    peers: Mapping[str, Tuple[Peer, ...]] = dataclasses.field(default_factory=dict)
    node: str = ""
    rig: Optional[Callable[[str], "BlockView"]] = None
    topology: Optional[Callable[[], Any]] = None

    def port(self):
        """The camera port the block's chain shapes, None without one."""
        return self.topology() if self.topology is not None else None

    def peer_view(self, port: str) -> Optional["BlockView"]:
        """The view of the one block wired to `port`, None without one or
        without a rig."""
        peers = self.peers.get(port) or ()
        if not peers or self.rig is None:
            return None
        return self.rig(peers[0].node)

    def value(self, row: str) -> int:
        """The served value declared for a row, else the row's default."""
        return int(self.declared.get(row, self.description.row(row).default))


Provider = Callable[[BlockView], Mapping[str, Option]]
#: The hook: a block's own options by its compatible, else by its kind word.
_PROVIDERS: Dict[str, Provider] = {}


def register(key: str, provider: Provider) -> None:
    """Register the options a block's code narrows: `key` is the block's
    compatible (`acme,des1`) or its kind word (`host`)."""
    _PROVIDERS[key] = provider


def provider_for(description: Description) -> Optional[Provider]:
    return _PROVIDERS.get(description.type) or _PROVIDERS.get(description.kind)


def static_options(view: BlockView) -> Dict[str, Option]:
    """Every row's option from the description alone: an enum's values or a
    range's span, with a read-only row and the stream row inapplicable."""
    out: Dict[str, Option] = {}
    for row in view.description.rows:
        if row.kind == ParamKind.GATE:
            out[row.name] = Option(applicable=False, finding="the run state, which no declaration sets")
        elif row.is_read_only:
            out[row.name] = Option(applicable=False, finding="read-only, the device serves it")
        elif row.is_range:
            out[row.name] = Option(span=(row.values[0], row.values[1], row.values[2]))
        else:
            out[row.name] = Option(values=tuple(row.values))
    return out


def options_for(view: BlockView) -> Dict[str, Option]:
    """Every row's option here: the static ones, each overridden by the
    block's own provider where it has one."""
    out = static_options(view)
    provider = provider_for(view.description)
    if provider is not None:
        out.update(provider(view))
    return out


def judge(view: BlockView, options: Mapping[str, Option], name: str, raw: int) -> Optional[str]:
    """The refusal for `raw` declared on row `name`, None when the options
    admit it: a fact, then the values offered."""
    try:
        row = view.description.row(name)
    except KeyError:
        names = ", ".join(r.name for r in view.description.rows) or "none"
        return f"{name} is no row of {view.description.type}: its rows are {names}"
    option = options.get(name) or Option()
    if not option.applicable:
        return f"{name} is not declared: {option.finding}"
    if option.admits(raw):
        return None
    unit = unit_of(row)
    shown = f"{display(row, raw)}{' ' + unit if unit and not row.present.labels else ''}"
    why = f"; {option.finding}" if option.finding else ""
    return f"{name} {shown} is not offered: {offered(row, option.values, option.span)}{why}"


def nearest(row, option: Optional[Option], raw: int, count: int = 2) -> List[str]:
    """What to declare in place of `raw` on `row`, `<row>: <value>` each:
    `drop the key` where the row is not declared here, a named row's names,
    else the `count` admitted values nearest `raw`."""
    if option is not None and not option.applicable:
        return ["drop the key"]
    values = option.values if option is not None else None
    span = option.span if option is not None else None
    if values is None:
        values = named_values(row, span)
    if values is None and span is None:
        if row.is_range:
            span = (row.values[0], row.values[1], row.values[2])
        else:
            values = tuple(row.values)
    if values is not None:
        picks = (list(values)[:8] if present(row).labels
                 else sorted(sorted(values, key=lambda v: (abs(v - raw), v))[:count]))
    else:
        lo, hi, step = span
        picks = [lo + (min(max(int(raw), lo), hi) - lo) // max(1, step) * max(1, step)]
    return [f"{row.name}: {display(row, v)}" for v in picks]


__all__ = ["BlockView", "Option", "Peer", "Provider", "judge", "nearest", "options_for",
           "provider_for", "register", "static_options"]
