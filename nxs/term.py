"""ANSI colour helpers and small status-print primitives.

Shared between the top-level `nxs` verbs and the `bench`
subcommand so RTT-style operator output stays consistent across the
CLI. Modules that don't need colour can ignore this entirely.
"""

GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
CYAN = "\033[36m"
GREY = "\033[90m"
NC = "\033[0m"


def banner(text: str) -> None:
    """Section header — cyan, double-rule decoration. One per major step."""
    print(f"\n{CYAN}═══ {text} ═══{NC}")


def info(text: str) -> None:
    """Tertiary detail — grey, no decoration. Use for command-being-run lines
    and other low-signal context."""
    print(f"{GREY}{text}{NC}")


def warn(text: str) -> None:
    """Recoverable issue — yellow, no decoration."""
    print(f"{YELLOW}{text}{NC}")


def err(text: str) -> None:
    """Hard failure — red, no decoration. Pair with non-zero exit when
    the caller is bailing out."""
    print(f"{RED}{text}{NC}")
