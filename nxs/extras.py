"""Optional stacks: a verb whose dependency is absent answers one line."""
import importlib


def require(extra: str, module: str, what: str):
    """Import `module` for the optional stack `extra`, or raise ImportError
    naming the install (`<what> needs: pip install aliensense-nxs[<extra>]`)."""
    try:
        return importlib.import_module(module)
    except ImportError as exc:
        raise ImportError(f"{what} needs: pip install aliensense-nxs[{extra}]") from exc
