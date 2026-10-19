# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The V4L2 controls of a capture node's subdevice, by the names its kernel
driver gives them (`Gain`, `Exposure`, `Frame Rate`, `shutter_mode`,
`test_pattern`, as `v4l2-ctl -l` prints them). The subdevice is found by
the name the kernel registers it under, `<driver> <bus>-<addr>`, so the
tool names a capture node by its bus and its address alone and never by a
device number or a driver. The ioctls are the kernel's extended-control
calls over `fcntl`, with no library between."""

from __future__ import annotations

import ctypes
import errno
import fcntl
import glob
import os
import struct
from typing import Callable, Dict, Optional

#: Where the kernel lists its V4L2 devices.
VIDEO4LINUX = "/sys/class/video4linux"

# struct v4l2_query_ext_ctrl: id, type, name[32], minimum, maximum, step,
# default_value, flags, elem_size, elems, nr_of_dims, dims[4], reserved[32].
_QUERY = struct.Struct("<II32sqqQqIIII4I32I")
# struct v4l2_ext_control (packed): id, size, reserved2[1], value64.
_EXT_CONTROL = struct.Struct("<IIIq")
# struct v4l2_ext_controls: which, count, error_idx, request_fd, reserved[1],
# then the pointer to the controls, 8-aligned.
_EXT_CONTROLS = struct.Struct("<IIIiI4xQ")
_NEXT_CTRL = 0x80000000
_NEXT_COMPOUND = 0x40000000
_WHICH_CUR_VAL = 0


def _iowr(nr: int, size: int) -> int:
    """`_IOWR('V', nr, struct)` as the kernel encodes it."""
    return (3 << 30) | (size << 16) | (ord("V") << 8) | nr


VIDIOC_G_EXT_CTRLS = _iowr(71, _EXT_CONTROLS.size)
VIDIOC_S_EXT_CTRLS = _iowr(72, _EXT_CONTROLS.size)
VIDIOC_QUERY_EXT_CTRL = _iowr(103, _QUERY.size)


def control_key(name: str) -> str:
    """A control's name as the tool spells it: `Frame Rate` -> `frame_rate`."""
    return name.strip().lower().replace(" ", "_")


def _bus_index(bus: str) -> Optional[int]:
    """`/dev/i2c-9` (or an alias symlink to it) -> 9."""
    name = os.path.basename(os.path.realpath(bus))
    if not name.startswith("i2c-"):
        return None
    try:
        return int(name[4:])
    except ValueError:
        return None


def find_subdev(bus: str, addr: int, video4linux: str = VIDEO4LINUX) -> Optional[str]:
    """The device node of the subdevice registered for the I2C device at
    `addr` on `bus` (`/dev/v4l-subdevN`), by the name the kernel gives it,
    `<driver> <bus>-<addr>`; None when no driver is bound there."""
    index = _bus_index(bus)
    if index is None:
        return None
    tail = f" {index}-{int(addr):04x}"
    for entry in sorted(glob.glob(os.path.join(video4linux, "v4l-subdev*"))):
        try:
            with open(os.path.join(entry, "name")) as fh:
                name = fh.read().strip()
        except OSError:
            continue
        if name.endswith(tail):
            return f"/dev/{os.path.basename(entry)}"
    return None


class Subdev:
    """One subdevice's controls. `get`/`set` take the control's name; the
    ids are read from the device once, at the first call. `ioctl` and
    `opener` are the test seams."""

    def __init__(self, path: str, ioctl: Callable = fcntl.ioctl,
                 opener: Callable[[str, int], int] = os.open) -> None:
        self._path = path
        self._ioctl = ioctl
        self._fd = opener(path, os.O_RDWR)
        self._ids: Optional[Dict[str, int]] = None

    def __enter__(self) -> "Subdev":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def close(self) -> None:
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    @property
    def path(self) -> str:
        return self._path

    def controls(self) -> Dict[str, int]:
        """The device's controls, `control_key` -> id, as the driver lists
        them."""
        if self._ids is None:
            ids: Dict[str, int] = {}
            next_id = _NEXT_CTRL | _NEXT_COMPOUND
            while True:
                buf = bytearray(_QUERY.size)
                _QUERY.pack_into(buf, 0, next_id, 0, b"", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 *([0] * 32))
                try:
                    self._ioctl(self._fd, VIDIOC_QUERY_EXT_CTRL, buf)
                except OSError as exc:
                    if exc.errno == errno.EINVAL:
                        break
                    raise OSError(exc.errno, exc.strerror, f"query controls on {self._path}") from exc
                fields = _QUERY.unpack_from(buf, 0)
                ids[control_key(fields[2].split(b"\0", 1)[0].decode(errors="replace"))] = fields[0]
                next_id = fields[0] | _NEXT_CTRL | _NEXT_COMPOUND
            self._ids = ids
        return dict(self._ids)

    def _id(self, name: str) -> int:
        ids = self.controls()
        key = control_key(name)
        if key not in ids:
            raise LookupError(f"{self._path} has no control {key!r}; it has "
                              + ", ".join(sorted(ids)))
        return ids[key]

    def _call(self, request: int, cid: int, value: int, what: str) -> int:
        control = ctypes.create_string_buffer(_EXT_CONTROL.size)
        _EXT_CONTROL.pack_into(control, 0, cid, 0, 0, int(value))
        outer = bytearray(_EXT_CONTROLS.size)
        _EXT_CONTROLS.pack_into(outer, 0, _WHICH_CUR_VAL, 1, 0, 0, 0, ctypes.addressof(control))
        try:
            self._ioctl(self._fd, request, outer)
        except OSError as exc:
            raise OSError(exc.errno, exc.strerror, f"{what} on {self._path}") from exc
        return _EXT_CONTROL.unpack_from(control.raw, 0)[3]

    def get(self, name: str) -> int:
        """The control's current value, as the driver holds it."""
        return self._call(VIDIOC_G_EXT_CTRLS, self._id(name), 0, f"get {control_key(name)}")

    def set(self, name: str, value: int) -> None:
        """Write the control; the driver writes the head. OSError: the
        driver's refusal, its errno the bus's or the range's."""
        self._call(VIDIOC_S_EXT_CTRLS, self._id(name), int(value), f"set {control_key(name)}")
