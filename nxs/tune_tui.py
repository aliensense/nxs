# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The curses panel: it paints the frame the view lays out and hands every key to the model."""

import contextlib
import curses
import io
import locale
import os
import sys

from nxs import tune_graph as graph
from nxs import tune_view as view
from nxs.finding import parse_refusal
from nxs.suite.schema_base import ManifestError
from nxs.tune import check_saved, identify_route, identify_unit, nothing_tunable, save_model
from nxs.tune_fields import refresh

#: The 256-colour palette: gray steps for the hierarchy, one accent.
COLOURS = {"muted": 244, "dim": 241, "rule": 239, "accent": 109,
           "ok": 143, "warn": 179, "err": 167}
ANSI = {"accent": curses.COLOR_CYAN, "ok": curses.COLOR_GREEN,
        "warn": curses.COLOR_YELLOW, "err": curses.COLOR_RED}


def run_tui():
    if not (sys.stdin.isatty() and sys.stdout.isatty()):
        raise SystemExit("nxs tune: not a terminal\n  - nxs tune --list")
    try:
        locale.setlocale(locale.LC_ALL, "")
    except locale.Error:
        pass
    panel = Panel(*graph.load())
    if not panel.channels:
        raise SystemExit(nothing_tunable(panel.cfg))
    return curses.wrapper(panel.run)


def palette():
    """Role to curses attribute: colours where the terminal has them, else attributes."""
    dim, bold = curses.A_DIM, curses.A_BOLD
    attrs = {"text": 0, "muted": dim, "dim": dim, "rule": dim, "accent": bold,
             "ok": 0, "warn": bold, "err": bold}
    if os.environ.get("NO_COLOR") or not curses.has_colors():
        return attrs
    try:
        curses.use_default_colors()
    except curses.error:
        return attrs
    colours = COLOURS if curses.COLORS >= 256 else ANSI
    for pair, (role, colour) in enumerate(colours.items(), start=1):
        try:
            curses.init_pair(pair, colour, -1)
        except curses.error:
            continue
        attrs[role] = curses.color_pair(pair)
    return attrs


class Panel:
    def __init__(self, path, cfg, channels, facts):
        from nxs import __version__

        self.path, self.cfg, self.channels, self.facts = path, cfg, channels, facts
        self.edits = {}  # (channel, section label, field) -> token, until saved
        self.state = view.ViewState(tree=[], manifest=view.short_path(path, os.path.expanduser("~")),
                                    version=__version__)
        self.rebuild()
        self.verdict()
        view.note_hint(self.state)

    def run(self, scr):
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        attrs = palette()
        while True:
            self.paint(scr, attrs)
            key = scr.getch()
            if key in (ord("q"), ord("Q")):
                return 0
            if key == curses.KEY_UP:
                view.move(self.state, -1)
            elif key == curses.KEY_DOWN:
                view.move(self.state, 1)
            elif key in (curses.KEY_LEFT, curses.KEY_RIGHT):
                self.step(-1 if key == curses.KEY_LEFT else 1)
            elif key in (ord("s"), ord("S")):
                self.act(scr, attrs, "saving…", self.save)
            elif key in (ord("f"), ord("F")):
                self.act(scr, attrs, "freezing…", self.freeze)
            elif key in (ord("i"), ord("I")):
                self.identify()
            elif key in (ord("r"), ord("R")):
                self.act(scr, attrs, "refreshing…", self.refresh)

    def paint(self, scr, attrs):
        scr.erase()
        rows, cols = scr.getmaxyx()
        for y, row in enumerate(view.frame(self.state, cols, rows)):
            x = 0
            for text, role in row:
                try:
                    scr.addstr(y, x, text, attrs[role])
                except curses.error:
                    pass  # the last cell of the last row
                x += len(text)
        scr.refresh()

    def act(self, scr, attrs, busy, action):
        """Paint the busy word, run the action on this thread, and turn a refusal into the status row."""
        self.say(busy, "muted")
        self.paint(scr, attrs)
        try:
            action()
        except SystemExit as e:
            fact, alternatives = parse_refusal(str(e))
            self.say(f"refused · {fact.splitlines()[0]}", "err",
                     view.lines_rows(f"  - {a}" for a in alternatives))
        except OSError as e:
            self.say(f"refused · {e.strerror or e}", "err")
        scr.clearok(True)

    def say(self, text, role, block=()):
        """The status row and the block; the focused node's hint yields to the result."""
        self.state.status, self.state.block, self.state.hint = (text, role), list(block), ""

    def rebuild(self):
        self.state.tree = graph.build_tree(self.cfg, self.channels, self.facts)
        view.place_cursor(self.state)

    def verdict(self):
        self.state.no_manifest = not os.path.exists(self.path)
        self.state.findings = [] if self.state.no_manifest else check_saved()

    def reload(self, keep=None):
        """Load the model and the facts again and replay the unsaved edits `keep`
        allows; a file that no longer parses leaves the last model standing and
        returns the refusal, else None."""
        try:
            loaded = graph.load()
        except ManifestError as e:
            return str(e)
        self.path, self.cfg, self.channels, self.facts = loaded
        self.edits = graph.apply_edits(self.channels, self.edits, keep)
        self.rebuild()
        self.verdict()

    def step(self, delta):
        knob = view.focused_knob(self.state)
        if knob is None:
            return
        knob.field.step(delta)
        key = (knob.channel.name, knob.section.label, knob.name)
        self.edits.pop(key, None)  # the latest edit replays last
        self.edits[key] = graph.token(knob.field.value)
        note, _snapped = refresh(knob.section)
        self.rebuild()
        self.say(note, "text") if note else self.say("DETUNED · unsaved edits", "warn")

    def save(self):
        try:
            backup = save_model(self.path, self.channels)
        except OSError as e:
            raise SystemExit(f"cannot write {self.path}: {e.strerror or e}") from e
        self.edits = {}
        findings = check_saved()
        self.reload()
        if findings:
            self.say(f"OUT OF TUNE · {view.plural(len(findings), 'finding')}", "err",
                     view.findings_rows(findings))
            return
        saved = f"saved · backup {os.path.basename(backup)}" if backup else "saved"
        self.say(f"{saved} · next: nxs switch", "ok")

    def freeze(self):
        from nxs.suite.freeze import run_freeze

        node = view.focused_node(self.state)
        if node is None:
            return
        ports = node.freeze == "ports"
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
            rc = run_freeze(self.path, only_unit=None if ports else node.unit, ports=ports,
                            only_port=node.port if ports else None)
        # What the freeze adopted, its node's knobs now show; every other edit stays,
        # and all of them when the freeze failed.
        if ports:
            keep = lambda ch, sec, _f: not (ch.name == node.port and sec.kind in ("declare", "camera"))
        else:
            keep = lambda _ch, sec, _f: sec.unit_name != node.unit
        refused = self.reload(keep if rc == 0 else None)
        what = node.port if ports else node.unit
        lines = out.getvalue().splitlines() + ([refused] if refused else [])
        self.say(f"froze {what} · next: nxs switch" if rc == 0 else "freeze failed",
                 "ok" if rc == 0 else "err", view.lines_rows(lines))

    def identify(self):
        node = view.focused_node(self.state)
        if node is None or not node.unit:
            self.say("identify is for units", "warn")
            return
        if node.route is not None:
            identify_route(node.route)
        else:
            identify_unit(node.unit)
        self.say(f"identify · {node.unit} LED strobing for 10 s", "ok")

    def refresh(self):
        refused = self.reload()
        if refused:
            self.say(f"refused · {refused}", "err")
            return
        ports, units = graph.counts(self.state.tree)
        self.say(f"refreshed · {view.plural(ports, 'port')} · {view.plural(units, 'unit')}", "ok")
