# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The device parameters a unit carries beside its personality's: the
sample FIFO depth, the output decimation, the CAN bit timing and
termination, the mounting orientation. `nxs get <name>` and `nxs set
<name> <value>` reach them after the personality's parameters."""

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

from nxs.client import (SUBJECT_BUCKETS, SupportsBitTiming, SupportsCalibration,
                        SupportsCanTermination, SupportsSampleFifo, rotation_code,
                        rotation_name, ROTATION_NAMES)

SUBJECTS = sorted(SUBJECT_BUCKETS, key=SUBJECT_BUCKETS.get)


class ParamError(ValueError):
    """A device parameter refused: an unknown name, a value outside its
    law, or a transport without the capability."""


@dataclass(frozen=True)
class DeviceParam:
    name: str
    capability: Optional[type]
    read: Callable
    write: Callable
    values: str
    note: str


def parse_can_term(spec: str) -> int:
    """`on`/`off`/`default` or the raw register values 1/0/0xFFFF."""
    words = {'on': 1, 'off': 0, 'default': 0xFFFF}
    if spec.lower() in words:
        return words[spec.lower()]
    try:
        return int(spec, 0)
    except ValueError:
        raise ParamError(f"can-term takes on|off|default, not {spec!r}")


def parse_can_bitrate(spec: str) -> Tuple[int, int]:
    """`NOM[/DATA]` in any int base; a bare nominal selects Classic CAN."""
    nom, sep, data = spec.partition('/')
    try:
        nominal = int(nom, 0)
        d = int(data, 0) if sep else nominal
    except ValueError:
        raise ParamError(f"can-bitrate takes NOMINAL[/DATA] integers, not {spec!r}")
    return nominal, d


def bitrate_line(t) -> str:
    nominal, data = t.read_can_bitrate()
    return f"{nominal}/{data} ({'classic' if data == nominal else 'fd'})"


def _int(value: str, name: str) -> int:
    try:
        return int(value, 0)
    except ValueError:
        raise ParamError(f"{name} takes an integer, not {value!r}")


def _fifo_read(t, _subject):
    depth = t.read_sample_fifo_depth()
    return str(depth) if depth else "max"


def _fifo_write(t, value, _subject):
    t.write_sample_fifo_depth(_int(value, "fifo-depth"))


def _decimation_read(t, subject):
    return str(t.read_decimation(subject))


def _decimation_write(t, value, subject):
    t.write_decimation(_int(value, "decimation"), subject)


def _bitrate_read(t, _subject):
    return bitrate_line(t)


def _bitrate_write(t, value, _subject):
    t.write_can_bitrate(*parse_can_bitrate(value))


def _term_read(t, _subject):
    return "on" if t.read_can_term() else "off"


def _term_write(t, value, _subject):
    t.write_can_term(parse_can_term(value))


def _orientation_read(t, _subject):
    return rotation_name(t.read_calibration().orientation)


def _orientation_write(t, value, _subject):
    try:
        code = rotation_code(value.upper())
    except ValueError as e:
        raise ParamError(f"orientation: {e}")
    t.set_orientation(code, persist=True)


PARAMS: List[DeviceParam] = [
    DeviceParam("fifo-depth", SupportsSampleFifo, _fifo_read, _fifo_write,
                "records (0 = all the storage holds)",
                "applies live; persists on nxs commission --save"),
    DeviceParam("decimation", None, _decimation_read, _decimation_write,
                "1 = every sample, N = every Nth; decimation.<subject> per subject "
                f"({', '.join(SUBJECTS)})",
                "applies live; persists on nxs commission --save"),
    DeviceParam("can-bitrate", SupportsBitTiming, _bitrate_read, _bitrate_write,
                "NOMINAL[/DATA] bit/s (0 reverts to the default)",
                "persists on nxs commission --save; applies at the next reboot"),
    DeviceParam("can-term", SupportsCanTermination, _term_read, _term_write,
                "on | off | default",
                "applies live; persists on nxs commission --save"),
    DeviceParam("orientation", SupportsCalibration, _orientation_read,
                _orientation_write, " ".join(ROTATION_NAMES),
                "applied and persisted"),
]


def names() -> List[str]:
    return [p.name for p in PARAMS]


def lookup(t, name: str) -> Tuple[DeviceParam, Optional[str]]:
    """The device parameter `name` names, with its subject for
    `decimation.<subject>`; ParamError when there is none or the transport
    lacks the capability."""
    base, _, subject = name.partition(".")
    param = next((p for p in PARAMS if p.name == base), None)
    if param is None:
        raise ParamError(f"no parameter {name}")
    if subject and base != "decimation":
        raise ParamError(f"{base} takes no subject; decimation.<subject> does")
    if subject and subject not in SUBJECTS:
        raise ParamError(f"no subject {subject}; one of {', '.join(SUBJECTS)}")
    if param.capability is not None and not isinstance(t, param.capability):
        raise ParamError(f"{name}: this transport serves no such surface")
    return param, subject or None


def read(t, name: str) -> str:
    """`<name> = <value>` for a device parameter."""
    param, subject = lookup(t, name)
    try:
        return f"{name} = {param.read(t, subject)}"
    except ParamError:
        raise
    except (ValueError, RuntimeError) as e:
        raise ParamError(f"{name}: {e}")


def write(t, name: str, value: str) -> str:
    """Write a device parameter; `<name> = <value>` then the note on when
    it applies."""
    param, subject = lookup(t, name)
    try:
        param.write(t, value, subject)
        current = param.read(t, subject)
    except ParamError:
        raise
    except (ValueError, RuntimeError) as e:
        raise ParamError(f"{name}: {e}")
    return f"{name} = {current} ({param.note})"
