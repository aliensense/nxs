"""Serial-port discovery shared by the CLI and the transports.
Autodetect lists USB-attached ports only (`port.vid is not None`): one match
is picked silently, several prompt, none returns the empty string."""

from nxs.extras import require
from nxs import term

list_ports = require("cyphal", "serial.tools.list_ports", "the cyphal transport")


def autodetect_serial_port() -> str:
    """Device path of the single USB serial port, or `""` if zero or many
    match; several matches drop into a picker showing each description."""
    candidates = sorted(
        (p for p in list_ports.comports() if p.vid is not None),
        key=lambda p: p.device,
    )
    if len(candidates) == 1:
        return candidates[0].device
    if len(candidates) > 1:
        print()
        term.warn("Multiple USB serial ports found:")
        for i, p in enumerate(candidates, 1):
            desc = p.description or "(no description)"
            print(f"  {i}) {p.device}  — {desc}")
        while True:
            a = input("Pick port number (Enter to abort): ").strip()
            if not a:
                return ""
            try:
                return candidates[int(a) - 1].device
            except (ValueError, IndexError):
                term.err("Invalid choice — try again")
    return ""
