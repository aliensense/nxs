"""Serial-port discovery and open helpers shared by the CLI and the transports.
Autodetect lists USB-attached ports only (`port.vid is not None`): one match
is picked silently, several prompt, none returns the empty string."""

from nxs.extras import require
from nxs.term import RED, YELLOW, NC

serial = require("cyphal", "serial", "the cyphal transport")
list_ports = require("cyphal", "serial.tools.list_ports", "the cyphal transport")


class _ConfigureOnceSerial(serial.Serial):
    """A Serial that programs the device once, at open, and ignores later
    `_reconfigure_port` calls: some CDC firmwares drop off the USB bus on a
    post-open line-coding request. Timeouts still apply (kept host-side)."""

    _configured = False

    def open(self):
        self._configured = False   # re-arm for a reopen after close()
        super().open()
        self._configured = True

    def _reconfigure_port(self, force_update=False):
        if self._configured:
            return
        super()._reconfigure_port(force_update=force_update)


def open_serial_once(port: str, baud: int) -> serial.SerialBase:
    """Open `port` at `baud`, programming the device exactly once. Raises
    `serial.SerialException` for any open failure (pyserial's POSIX backend
    leaks raw `termios.error`, which is not an `OSError`)."""
    try:
        return _ConfigureOnceSerial(port=port, baudrate=baud)
    except serial.SerialException:
        raise
    except Exception as e:
        raise serial.SerialException(f"{port}: {e}") from e


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
        print(f"\n{YELLOW}Multiple USB serial ports found:{NC}")
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
                print(f"{RED}Invalid choice — try again{NC}")
    return ""
