"""Driver-discovery and interactive picker used by the bench harness.

`discover_drivers()` walks `nxs.drivers` and returns every
concrete `SensorDriver` subclass, keyed by class name. `pick_driver()`
renders a numbered menu and returns the chosen `(name, cls)` pair —
Enter or `s` skips, `q` exits the process.
"""

import importlib
import inspect
import pkgutil
import sys

from nxs import SensorDriver
from nxs import drivers as drivers_pkg
from nxs.term import RED, YELLOW, NC


def discover_drivers() -> dict:
    """Return `{ClassName: cls}` for every concrete `SensorDriver`
    subclass under `nxs.drivers` — captures `RegisterDriver`
    (IMUs), `I2cCommandDriver` (Sensirion-family), and `StreamDriver`
    (GPS, UART-only parts). Skips private modules and the `_reference`
    variants kept for hand-comparison."""
    drivers: dict = {}
    for m in pkgutil.iter_modules(drivers_pkg.__path__):
        if m.name.startswith("_") or m.name.endswith("_reference"):
            continue
        mod = importlib.import_module(f"nxs.drivers.{m.name}")
        for name, obj in inspect.getmembers(mod, inspect.isclass):
            try:
                if (issubclass(obj, SensorDriver) and obj is not SensorDriver
                        and obj.__module__ == mod.__name__):
                    drivers[name] = obj
            except TypeError:
                continue
    return drivers


def pick_driver(drivers: dict):
    """Numbered menu over discovered drivers; returns `(ClassName, cls)`
    on selection, `None` on skip (Enter or `s`). `q` exits the script
    via `sys.exit(0)`."""
    items = sorted(drivers.items())
    print()
    for i, (name, _) in enumerate(items, 1):
        print(f"  {i}) {name}")
    while True:
        a = input(
            f"{YELLOW}[pick driver]{NC} number (Enter=skip, q=quit): "
        ).strip().lower()
        if not a or a == "s":
            return None
        if a == "q":
            sys.exit(0)
        try:
            return items[int(a) - 1]
        except (ValueError, IndexError):
            print(f"{RED}Invalid choice — try again{NC}")
