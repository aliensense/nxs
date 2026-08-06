"""Serial-port discovery and open helpers shared by the CLI and the transports.

Backed by `pyserial`'s `serial.tools.list_ports.comports()`, which is
platform-agnostic:
  - macOS:   `/dev/cu.usbmodem*`, `/dev/cu.usbserial*` (CDC-ACM, FTDI, CP210x)
  - Linux:   `/dev/ttyUSB*`, `/dev/ttyACM*`
  - Windows: `COM1`, `COM2`, … with USB hardware IDs

The autodetect filters to USB-attached ports (`port.vid is not None`)
so onboard / Bluetooth / virtual COM ports don't pollute the list,
picks the single match silently, prompts on multiple matches, and
returns the empty string on zero matches.
"""

import serial
from serial.tools import list_ports

from nxs.term import RED, YELLOW, NC


class _ConfigureOnceSerial(serial.Serial):
    """A Serial that programs the device once, at open, and ignores later
    `_reconfigure_port` calls. pyserial re-programs the whole termios every
    time a config attribute (`timeout`, `write_timeout`, `baudrate`, …) is
    assigned on an open port — and on macOS a baud with no named termios
    constant (460800) additionally re-issues the line coding via IOSSIOSPEED
    on every such call, changed or not. Some CDC firmwares crash on a
    post-open line-coding request — a J-Link V9 VCOM falls off the USB bus
    until replugged — and pycyphal assigns `timeout` right after constructing
    the transport and `write_timeout` before every send. Timeout semantics
    survive the freeze: pyserial's POSIX reads and writes take them from
    host-side state (`select()`), not from termios."""

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
    """Open `port` at `baud`, programming the device exactly once, and return
    the open port (see `_ConfigureOnceSerial`). Raises `serial.SerialException`
    for any open failure — pyserial's POSIX backend leaks raw `termios.error`,
    which is not an `OSError` and would bypass the CLI's clean-error paths."""
    try:
        return _ConfigureOnceSerial(port=port, baudrate=baud)
    except serial.SerialException:
        raise
    except Exception as e:
        raise serial.SerialException(f"{port}: {e}") from e


def autodetect_serial_port() -> str:
    """Return the device path/name of the single matching USB serial port,
    or `""` if zero or many match. Multiple matches drop into an
    interactive picker that shows each port's description so the
    operator can tell, say, "Silicon Labs CP210x" from "FT232R USB UART"."""
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
