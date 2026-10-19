# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The frames slice of the C ABI (`nxs.h`): a link's capture facts, its
part in a synced pair, its frame timing, and the session that hands out
each frame of its capture node on the capture host."""

from __future__ import annotations

import ctypes
import errno
import os
from typing import Optional, Tuple

from nxs import _libnxs as _base

_NAME_LEN = 32
#: `nxs_capture_timing.stated` bits: which fields the record states.
TIMING_READOUT_DELAY = 1
TIMING_EXPOSURE_OFFSET = 2
TIMING_EXPOSURE = 4


class Capture(ctypes.Structure):
    """`nxs_capture`: the node, the mode, the geometry, the rate and the
    exposure window a link's record carries."""

    _fields_ = [("camera_id", ctypes.c_uint32), ("sensor_mode", ctypes.c_uint32),
                ("width", ctypes.c_uint32), ("height", ctypes.c_uint32),
                ("fps_num", ctypes.c_uint32), ("fps_den", ctypes.c_uint32),
                ("exposure_min_ns", ctypes.c_uint64), ("exposure_max_ns", ctypes.c_uint64)]


class CapturePair(ctypes.Structure):
    """`nxs_capture_pair`: the link's part in a synced pair's exposure and gain."""

    _fields_ = [("ae_lock", ctypes.c_uint32), ("gain_millidb", ctypes.c_uint32),
                ("isp_gain_unity", ctypes.c_uint32), ("isp_filters_off", ctypes.c_uint32),
                ("follow_port", ctypes.c_char * _NAME_LEN)]


class CaptureTiming(ctypes.Structure):
    """`nxs_capture_timing`: a triggered link's readout delay, exposure offset
    and stated exposure, with the bits naming which the record states."""

    _fields_ = [("readout_delay_ns", ctypes.c_uint64), ("exposure_offset_ns", ctypes.c_uint64),
                ("exposure_ns", ctypes.c_uint64), ("stated", ctypes.c_uint32),
                ("triggered", ctypes.c_uint32)]


class Frame(ctypes.Structure):
    """`nxs_frame`: one frame's device buffer and what the sensor took it with;
    `timestamp_ns` is the start of frame on `CLOCK_MONOTONIC`."""

    _fields_ = [("fd", ctypes.c_int), ("width", ctypes.c_uint32), ("height", ctypes.c_uint32),
                ("pitch", ctypes.c_uint32), ("index", ctypes.c_uint64),
                ("timestamp_ns", ctypes.c_uint64), ("exposure_ns", ctypes.c_uint64),
                ("gain", ctypes.c_float)]


def _declare_frames(lib: ctypes.CDLL) -> None:
    lib.nxs_frames_supported.restype = ctypes.c_int
    lib.nxs_frames_supported.argtypes = []
    lib.nxs_capture_pair_of.restype = ctypes.c_int
    lib.nxs_capture_pair_of.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.POINTER(Capture),
                                        ctypes.POINTER(CapturePair)]
    lib.nxs_capture_timing_of.restype = ctypes.c_int
    lib.nxs_capture_timing_of.argtypes = [ctypes.c_char_p, ctypes.c_char_p,
                                          ctypes.POINTER(CaptureTiming)]
    lib.nxs_frames_open_pair.restype = ctypes.c_void_p
    lib.nxs_frames_open_pair.argtypes = [ctypes.POINTER(Capture), ctypes.POINTER(CapturePair),
                                         ctypes.POINTER(ctypes.c_int)]
    lib.nxs_frames_next.restype = ctypes.c_int
    lib.nxs_frames_next.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.POINTER(Frame)]
    lib.nxs_frame_copy.restype = ctypes.c_int
    lib.nxs_frame_copy.argtypes = [ctypes.c_void_p, ctypes.POINTER(Frame), ctypes.c_void_p,
                                   ctypes.c_size_t]
    lib.nxs_frames_close.restype = None
    lib.nxs_frames_close.argtypes = [ctypes.c_void_p]


_base._DECLARERS.append(_declare_frames)
if _base._lib is not None:
    _declare_frames(_base._lib)


def supported() -> bool:
    """Whether this host's library has a capture-stack backend."""
    try:
        return _base._load().nxs_frames_supported() != 0
    except _base.LibraryMissing:
        return False


def _record_error(rc: int, port: str, link: str) -> OSError:
    if rc == -errno.ENOENT:
        return FileNotFoundError(f"{port}/{link}: no capture record (nxs {port} {link} on)")
    return OSError(-rc, f"{port}/{link}: {os.strerror(-rc)} reading its capture record")


def capture_pair_of(port: str, link: str) -> Tuple[Capture, CapturePair]:
    """The link's recorded capture facts and its part in a synced pair.

    Raises:
        FileNotFoundError: The link was never brought up on this host.
        OSError: A record the tool did not write.
    """
    capture, pair = Capture(), CapturePair()
    rc = _base._load().nxs_capture_pair_of(port.encode(), link.encode(), ctypes.byref(capture),
                                           ctypes.byref(pair))
    if rc != 0:
        raise _record_error(rc, port, link)
    return capture, pair


def timing_of(port: str, link: str) -> CaptureTiming:
    """The link's recorded frame timing, every field zero on a free-running link."""
    timing = CaptureTiming()
    rc = _base._load().nxs_capture_timing_of(port.encode(), link.encode(), ctypes.byref(timing))
    if rc != 0:
        raise _record_error(rc, port, link)
    return timing


class Frames:
    """One frame session on a link's capture node, in the link's part of a
    synced pair: `next()` waits for a frame, `copy()` reads its luma out."""

    def __init__(self, capture: Capture, pair: CapturePair, name: str = "") -> None:
        self._lib = _base._load()
        err = ctypes.c_int(0)
        self._handle: Optional[int] = self._lib.nxs_frames_open_pair(
            ctypes.byref(capture), ctypes.byref(pair), ctypes.byref(err))
        if not self._handle:
            self._handle = None
            raise OSError(err.value, f"{name or 'frames'}: {os.strerror(err.value)}")

    def next(self, timeout_ns: int = 2_000_000_000) -> Frame:
        """The next frame; its buffer stays valid until the next call.

        Raises:
            TimeoutError: No frame within the timeout.
            BrokenPipeError: The capture stack ended the stream.
            OSError: A frame the stack could not hand over.
        """
        frame = Frame()
        rc = self._lib.nxs_frames_next(self._handle, timeout_ns, ctypes.byref(frame))
        if rc == -errno.ETIMEDOUT:
            raise TimeoutError("no frame within the timeout")
        if rc == -errno.EPIPE:
            raise BrokenPipeError("the capture stack ended the stream")
        if rc != 0:
            raise OSError(-rc, os.strerror(-rc))
        return frame

    def copy(self, frame: Frame, dst) -> None:
        """Copy the frame's luma plane, `pitch × height` bytes, into `dst`
        (a writable buffer at least that many bytes, of any shape)."""
        length = memoryview(dst).nbytes
        rc = self._lib.nxs_frame_copy(self._handle, ctypes.byref(frame),
                                      (ctypes.c_ubyte * length).from_buffer(dst), length)
        if rc != 0:
            raise OSError(-rc, os.strerror(-rc))

    def close(self) -> None:
        if self._handle is not None:
            self._lib.nxs_frames_close(self._handle)
            self._handle = None

    def __enter__(self) -> "Frames":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()
