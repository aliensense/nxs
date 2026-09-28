# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The panel's frame without a terminal: rows of (text, role) spans over the tree, the cursor, the status row, the block and the footer."""

from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple

from nxs.tune_graph import Knob, Node, find

Span = Tuple[str, str]
Row = List[Span]

ROLES = ("text", "muted", "dim", "rule", "accent", "ok", "warn", "err")
NAME_WIDTH = 12
BLOCK_ROWS = 6
HINTS = (("↑↓", "move"), ("←→", "tune"), ("s", "save"), ("f", "freeze"),
         ("i", "identify"), ("r", "refresh"), ("q", "quit"))


@dataclass
class RowSpec:
    node: Node
    spans: Row
    knob: Optional[int] = None      # a knob row: its index in node.knobs


@dataclass
class ViewState:
    tree: List[Node]
    manifest: str
    version: str
    findings: List[Any] = field(default_factory=list)
    no_manifest: bool = False
    status: Span = ("", "text")
    hint: str = ""                  # the focused node's note, shown in place of status
    block: List[Row] = field(default_factory=list)
    cursor: tuple = (None, None)    # (node key, knob index or None)
    scroll: int = 0

    def verdict(self) -> Span:
        if self.no_manifest:
            return ("no manifest", "dim")
        if self.findings:
            return (f"OUT OF TUNE · {plural(len(self.findings), 'finding')}", "err")
        return ("IN TUNE", "ok")


def plural(n: int, noun: str) -> str:
    return f"{n} {noun}" if n == 1 else f"{n} {noun}s"


def short_path(path: str, home: str) -> str:
    return "~" + path[len(home):] if home and path.startswith(home + "/") else path


def rows(state: ViewState) -> List[RowSpec]:
    """The tree in reading order, the cursor's node followed by its knobs."""
    out: List[RowSpec] = []
    for root in state.tree:
        _walk(root, "", "", state, out)
    return out


def _walk(node: Node, prefix: str, trunk: str, state: ViewState, out: List[RowSpec]) -> None:
    key, knob = state.cursor
    focused = key == node.key
    out.append(RowSpec(node, node_row(node, prefix, focused and knob is None)))
    below = trunk + ("│  " if node.children else "   ")
    if focused:
        for i, k in enumerate(node.knobs):
            out.append(RowSpec(node, knob_row(k, below + " ", knob == i), knob=i))
    for i, child in enumerate(node.children):
        last = i == len(node.children) - 1
        _walk(child, trunk + ("└─ " if last else "├─ "),
              trunk + ("   " if last else "│  "), state, out)


def node_row(node: Node, prefix: str, focused: bool) -> Row:
    row: Row = [("→ " if focused else "  ", "accent")]
    if prefix:
        row.append((prefix, "rule"))
    row.append((node.parts[0], "accent" if focused else "text"))
    for part in node.parts[1:]:
        row += [(" · ", "muted"), (part, "muted")]
    if node.state:
        row += [(" · ", "muted"), (node.state, "ok" if node.state == "up" else "muted")]
    if node.suffix:
        row += [(" · ", "muted"), (node.suffix, "warn" if node.suffix == "absent" else "accent")]
    return row


def knob_row(knob: Knob, prefix: str, focused: bool) -> Row:
    return [("→ " if focused else "  ", "accent"), (prefix, "rule"),
            (f"{knob.name.lower():<{NAME_WIDTH}} ", "dim"),
            (knob.value, "accent" if focused else "text")]


def findings_rows(findings) -> List[Row]:
    """A finding's fact in text, its `  - ` alternatives muted."""
    out: List[Row] = []
    for finding in findings:
        for i, line in enumerate(str(finding).split("\n")):
            out.append([(line, "muted" if line.startswith("  - ") or i else "text")])
    return out


def lines_rows(lines, role: str = "muted") -> List[Row]:
    return [[(line, role)] for line in lines]


def cut(row: Row, cols: int) -> Row:
    """The row at most `cols` wide, ending in `…` where it was wider."""
    if sum(len(t) for t, _ in row) <= cols:
        return row
    out: Row = []
    left = max(0, cols - 1)
    for text, role in row:
        if left <= 0:
            break
        out.append((text[:left], role))
        left -= len(text[:left])
    out.append(("…", "dim"))
    return out


def rule(cols: int) -> Row:
    return [("─" * cols, "rule")]


def footer(state: ViewState, cols: int) -> List[Row]:
    """Two dim lines: the manifest with the saved file's verdict, then the keys, dropped from the right until they fit."""
    verdict = state.verdict()
    first = [(state.manifest, "dim"), (" · ", "dim"), verdict]
    hints = list(HINTS)
    while len(hints) > 1 and sum(len(k) + len(w) + 4 for k, w in hints) - 3 > cols:
        hints.pop()
    second: Row = []
    for i, (key, word) in enumerate(hints):
        if i:
            second.append((" · ", "dim"))
        second += [(key, "dim"), (f" {word}", "dim")]
    return [cut(first, cols), cut(second, cols)]


def frame(state: ViewState, cols: int, height: int) -> List[Row]:
    """Exactly `height` rows: the header, the tree around the cursor, the status row, the block and the footer."""
    head = [cut([("nxs", "accent"), (" tune", "text"), (f"  v{state.version}", "dim")], cols),
            rule(cols)]
    block = list(state.block)
    if len(block) > BLOCK_ROWS:
        block = block[:BLOCK_ROWS - 1] + [[(f"… {len(block) - BLOCK_ROWS + 1} more", "dim")]]
    block = block[:max(0, height - len(head) - 5)]
    status = [(state.hint, "muted")] if state.hint else [state.status]
    tail = [rule(cols), cut(status, cols)] + [cut(row, cols) for row in block]
    tail += footer(state, cols)
    middle = max(0, height - len(head) - len(tail))
    flat = rows(state)
    at = _cursor_index(state, flat)
    if middle:
        state.scroll = max(0, min(state.scroll, at, len(flat) - middle))
        state.scroll = max(state.scroll, at - middle + 1)
    body = [cut(spec.spans, cols) for spec in flat[state.scroll:state.scroll + middle]]
    body += [[] for _ in range(middle - len(body))]
    return (head + body + tail)[:height]


def _cursor_index(state: ViewState, flat: List[RowSpec]) -> int:
    key, knob = state.cursor
    for i, spec in enumerate(flat):
        if spec.node.key == key and spec.knob == knob:
            return i
    return 0


def place_cursor(state: ViewState) -> None:
    """The cursor on the node it names after the tree was rebuilt, else on the first node."""
    key, knob = state.cursor
    node = find(state.tree, key) if key is not None else None
    if node is None:
        state.cursor = (state.tree[0].key if state.tree else None, None)
        return
    if knob is not None:
        state.cursor = (key, min(knob, len(node.knobs) - 1) if node.knobs else None)


def move(state: ViewState, delta: int) -> None:
    flat = rows(state)
    if not flat:
        return
    at = max(0, min(len(flat) - 1, _cursor_index(state, flat) + delta))
    state.cursor = (flat[at].node.key, flat[at].knob)
    note_hint(state)


def note_hint(state: ViewState) -> None:
    """The note of the node whose own line the cursor sits on, else nothing."""
    node = focused_node(state)
    state.hint = node.note if node is not None and state.cursor[1] is None else ""


def focused_node(state: ViewState) -> Optional[Node]:
    return find(state.tree, state.cursor[0]) if state.cursor[0] is not None else None


def focused_knob(state: ViewState) -> Optional[Knob]:
    node = focused_node(state)
    knob = state.cursor[1]
    if node is None or knob is None or knob >= len(node.knobs):
        return None
    return node.knobs[knob]
