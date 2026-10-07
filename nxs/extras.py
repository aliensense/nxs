"""Optional stacks: a verb whose dependency is absent answers one line."""
import importlib
import os
import sys


def install_line(extra: str) -> str:
    """The command that adds the optional stack `extra` to this install: the
    `pipx inject` line where pipx made the environment, else the `pip` line."""
    spec = f"'aliensense-nxs[{extra}]'"
    if os.path.isfile(os.path.join(sys.prefix, "pipx_metadata.json")):
        return f"pipx inject aliensense-nxs {spec}"
    return f"pip install {spec}"


def require(extra: str, module: str, what: str):
    """Import `module` for the optional stack `extra`, or raise ImportError
    naming the install (`<what> needs: <install_line(extra)>`)."""
    try:
        return importlib.import_module(module)
    except ImportError as exc:
        raise ImportError(f"{what} needs: {install_line(extra)}") from exc
