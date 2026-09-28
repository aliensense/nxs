"""The host's tab completion and the pack hint: what `nxs switch` installs beside the bus rules."""

from typing import Optional

COMPLETION_TARGET = "/etc/bash_completion.d/nxs"


def print_pack_hint():
    """A camera port needs the descriptor pack; say how to install it when
    none is found, so the first camera verb is not the first hint."""
    from nxs.cam import packs
    try:
        searched = ", ".join(str(p) for p in packs.search_paths())
        found = packs.discover()
    except packs.PackError as exc:
        print(f"nxs: {exc}")
        return
    if found:
        return
    # The wheel carries the product's pack; a wheel without one is broken,
    # a source checkout without its tree is running from the wrong place.
    print(f"no descriptor pack found (searched: {searched}) — this nxs "
          f"ships its pack inside the wheel; reinstall the wheel, or run a "
          f"source checkout from its tree")


def completion_text() -> Optional[str]:
    """argcomplete's registration for bash (zsh sources the same file after
    `bashcompinit`); None when argcomplete is not installed."""
    try:
        from argcomplete.shell_integration import shellcode
    except ImportError:
        return None
    return shellcode(["nxs"], shell="bash")
