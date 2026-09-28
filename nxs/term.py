"""ANSI colour helpers and status-print primitives shared by the `nxs`
verbs."""

import re

GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
CYAN = "\033[36m"
GREY = "\033[90m"
NC = "\033[0m"


def status_line(text: str, done: bool = False) -> None:
    """Redraw one in-place progress line (erase to end of line first, so
    a shorter redraw leaves no tail); `done` keeps it and moves on."""
    print(f"\r\x1b[K{text}", end="\n" if done else "", flush=True)


def banner(text: str) -> None:
    """Section header: cyan, double-rule decoration. One per major step."""
    print(f"\n{CYAN}═══ {text} ═══{NC}")


def info(text: str) -> None:
    """Tertiary detail: grey, no decoration, for low-signal context."""
    print(f"{GREY}{text}{NC}")


def warn(text: str) -> None:
    """Recoverable issue: yellow, no decoration."""
    print(f"{YELLOW}{text}{NC}")


def err(text: str) -> None:
    """Hard failure: red, no decoration. Pair with a non-zero exit."""
    print(f"{RED}{text}{NC}")


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
