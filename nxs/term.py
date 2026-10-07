"""ANSI colour helpers and status-print primitives shared by the `nxs`
verbs."""

import os
import re
import sys

GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
CYAN = "\033[36m"
GREY = "\033[90m"
NC = "\033[0m"


def use_color() -> bool:
    """Whether a verb colours its lines: stdout is a terminal and `NO_COLOR`
    is not in the environment, whatever its value. A pipe or a file gets
    plain text."""
    return sys.stdout.isatty() and "NO_COLOR" not in os.environ


def paint(color: str, text: str) -> str:
    """`text` in `color` where the verbs colour their lines, else as it is."""
    return f"{color}{text}{NC}" if use_color() else text


def status_line(text: str, done: bool = False) -> None:
    """Redraw one in-place progress line (erase to end of line first, so
    a shorter redraw leaves no tail); `done` keeps it and moves on."""
    print(f"\r\x1b[K{text}", end="\n" if done else "", flush=True)


def banner(text: str) -> None:
    """Section header: cyan, double-rule decoration. One per major step."""
    print("\n" + paint(CYAN, f"═══ {text} ═══"))


def info(text: str) -> None:
    """Tertiary detail: grey, no decoration, for low-signal context."""
    print(paint(GREY, text))


def warn(text: str) -> None:
    """Recoverable issue: yellow, no decoration."""
    print(paint(YELLOW, text))


def err(text: str) -> None:
    """Hard failure: red, no decoration. Pair with a non-zero exit."""
    print(paint(RED, text))


#: Set by the entry point when the verb was asked for `--json`: a refusal is
#: then a document, not a sentence.
_json = False


def json_mode(on: bool) -> None:
    global _json
    _json = bool(on)


def refusal(fact: str, *alternatives: str) -> None:
    """A refusal: the fact on one line, then one `  - ` line per
    alternative (a runnable command or a lawful value). Under `--json` it
    is the `refusal` surface."""
    if _json:
        import json
        print(json.dumps({"refused": {"fact": fact, "alternatives": list(alternatives)}},
                         indent=2))
        return
    err("\n".join([fact, *(f"  - {alt}" for alt in alternatives)]))


def refusal_text(text: str) -> None:
    """A refusal given as its text (the fact, then its `  - ` lines)."""
    from nxs.finding import parse_refusal
    fact, alternatives = parse_refusal(text)
    refusal(fact, *alternatives)


_ANSI = re.compile(r"\x1b\[[0-9;]*m")


def strip_ansi(text: str) -> str:
    """Drop SGR colour sequences, leaving plain text for logs and rows."""
    return _ANSI.sub("", text)
