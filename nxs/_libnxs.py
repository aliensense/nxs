# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The binding of libnxs over the library's C ABI (`nxs.h`): the bus
handle, the run of a hub or sensor image on it, and the walk of a port.
The unit client over the three wires is `nxs._libnxs_unit`.

The library is found at `$NXS_LIBNXS`, then beside this module under
`_lib/`, then in the source tree's `build-posix/`."""

from __future__ import annotations

import ctypes
import errno
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping, Optional, Tuple, Union


MAX_PARAMS = 8
_ERROR_LEN = 32
_NAME_LEN = 32
_UNIT_LEN = 16
_SERIAL_LEN = 32
_VERSION_LEN = 64
_REASON_LEN = 128
_PORT_LINKS = 2
_ADDRESS_MAP_MAX = 2
_TRAILER_MAX = 2048
_MAX_PARAM_VALUES = 16
_SAMPLE_DATA_MAX = 128
_CALIBRATION_RECORD_MAX = 256
EREFUSED = 1024
#: The default unit address on a port's bus and behind a hub before the aliases.
DEFAULT_I2C_ADDRESS = 0x30
DEFAULT_NODE_ID = 125
HOST_NODE_ID = 127
DEFAULT_BAUD = 460800
DEFAULT_CAN_MTU = 64
#: The per-subject decimation gates, by the unit's subject bucket (1 onward).
DECIMATION_SUBJECTS = ("acceleration", "angular_velocity", "magnetic_field", "temperature",
                       "pressure", "scalar")
_STORE_SLOTS = 8
_RESET_SETTLE_MS = 2000
_CAM_RUN_TIMEOUT_MS = 30000
_CAM_ABORT_TIMEOUT_MS = 5000
#: A personality parameter the records do not name.
NO_PARAM = 0xFF
ENV = "NXS_LIBNXS"
_NAME = "libnxs.dylib" if sys.platform == "darwin" else "libnxs.so"


class LibraryMissing(RuntimeError):
    """libnxs is not installed for this platform."""


class _Msg(ctypes.Structure):
    _fields_ = [("addr", ctypes.c_uint16), ("read", ctypes.c_uint8),
                ("data", ctypes.POINTER(ctypes.c_uint8)), ("len", ctypes.c_size_t)]


class _Param(ctypes.Structure):
    _fields_ = [("index", ctypes.c_uint8), ("value", ctypes.c_uint32)]


class _Report(ctypes.Structure):
    _fields_ = [("rc", ctypes.c_int), ("pc", ctypes.c_uint16), ("opcode", ctypes.c_uint8),
                ("names_reg", ctypes.c_uint8), ("reg", ctypes.c_uint16),
                ("addr", ctypes.c_uint8), ("probing", ctypes.c_uint8),
                ("error", ctypes.c_char * _ERROR_LEN), ("num_params", ctypes.c_uint8),
                ("params", ctypes.c_uint32 * MAX_PARAMS), ("reg_reads", ctypes.c_uint32),
                ("reg_writes", ctypes.c_uint32)]


class _PersonalityParams(ctypes.Structure):
    _fields_ = [("mode", ctypes.c_uint8), ("trigger", ctypes.c_uint8), ("action", ctypes.c_uint8),
                ("line_time", ctypes.c_uint8), ("frame_period", ctypes.c_uint8)]


class _PodSpec(ctypes.Structure):
    _fields_ = [("present", ctypes.c_uint8), ("alias", ctypes.c_uint8), ("target", ctypes.c_uint8),
                ("personality", ctypes.c_char * _NAME_LEN)]


class _AddressEntry(ctypes.Structure):
    _fields_ = [("alias", ctypes.c_uint8), ("target", ctypes.c_uint8)]


class _LinkSpec(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char), ("present", ctypes.c_uint8),
                ("has_camera", ctypes.c_uint8), ("ser_addr", ctypes.c_uint8),
                ("head_addr", ctypes.c_uint8), ("lanes", ctypes.c_uint8),
                ("data_type", ctypes.c_uint8), ("host_csi", ctypes.c_uint8),
                ("params", _PersonalityParams), ("mode", ctypes.c_uint32),
                ("trigger", ctypes.c_uint32), ("line_time_ns", ctypes.c_uint32),
                ("frame_period_ns", ctypes.c_uint32), ("frame_fixed", ctypes.c_uint8),
                ("pod", _PodSpec), ("num_entries", ctypes.c_uint8),
                ("entries", _AddressEntry * _ADDRESS_MAP_MAX),
                ("sensor_image", ctypes.POINTER(ctypes.c_uint8)),
                ("sensor_image_len", ctypes.c_size_t)]


class _PortSpec(ctypes.Structure):
    _fields_ = [("des_addr", ctypes.c_uint8), ("csi_lanes", ctypes.c_uint8),
                ("homogeneous", ctypes.c_uint8), ("links", _LinkSpec * _PORT_LINKS),
                ("hub_image", ctypes.POINTER(ctypes.c_uint8)), ("hub_image_len", ctypes.c_size_t),
                ("ser_image", ctypes.POINTER(ctypes.c_uint8)), ("ser_image_len", ctypes.c_size_t)]


class _Refusal(ctypes.Structure):
    _fields_ = [("device_errno", ctypes.c_int), ("reason", ctypes.c_char * _REASON_LEN)]


class _PortReport(ctypes.Structure):
    _fields_ = [("rc", ctypes.c_int), ("step", ctypes.c_char * _ERROR_LEN), ("run", _Report),
                ("refusal", _Refusal)]


class _SlotInfo(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char * _NAME_LEN), ("kind", ctypes.c_uint8),
                ("num_params", ctypes.c_uint8), ("num_outputs", ctypes.c_uint8),
                ("i2c_addr", ctypes.c_uint8)]


class _CamState(ctypes.Structure):
    _fields_ = [("state", ctypes.c_uint8), ("error", ctypes.c_uint8)]


_TRACE_FN = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(_Msg), ctypes.c_int)
_LOG_FN = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_char_p)
_POD_FN = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_char, ctypes.c_void_p)
_READ_FN = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint16,
                            ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t,
                            ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t)
_WRITE_FN = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint16,
                             ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t)

_lib: Optional[ctypes.CDLL] = None


def library_path() -> Path:
    """Where the library is, or LibraryMissing."""
    env = os.environ.get(ENV)
    if env:
        # A named library is the only candidate: a name that points at
        # nothing is a broken build, never a fallback to another copy.
        if Path(env).is_file():
            return Path(env)
        raise LibraryMissing(f"{ENV} names {env}, which is not there")
    candidates = [
        Path(__file__).parent / "_lib" / _NAME,
        Path(__file__).resolve().parents[2] / "build-posix" / _NAME,
    ]
    for path in candidates:
        if path.is_file():
            return path
    raise LibraryMissing(f"{_NAME} not found (looked at {', '.join(map(str, candidates))})")


def _load() -> ctypes.CDLL:
    global _lib
    if _lib is None:
        lib = ctypes.CDLL(str(library_path()))
        _declare(lib)
        lib.nxs_version.restype = ctypes.c_char_p
        lib.nxs_version.argtypes = []
        lib.nxs_bus_open.restype = ctypes.c_void_p
        lib.nxs_bus_open.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
        lib.nxs_bus_open_scripted.restype = ctypes.c_void_p
        lib.nxs_bus_open_scripted.argtypes = [_READ_FN, _WRITE_FN, ctypes.c_void_p]
        lib.nxs_bus_set_trace.restype = None
        lib.nxs_bus_set_trace.argtypes = [ctypes.c_void_p, _TRACE_FN, ctypes.c_void_p]
        lib.nxs_bus_close.restype = None
        lib.nxs_bus_close.argtypes = [ctypes.c_void_p]
        lib.nxs_run.restype = ctypes.c_int
        lib.nxs_run.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_char_p,
                                ctypes.c_size_t, ctypes.POINTER(_Param), ctypes.c_size_t,
                                ctypes.POINTER(_Report)]
        lib.nxs_port_up.restype = ctypes.c_int
        lib.nxs_port_up.argtypes = [ctypes.c_void_p, ctypes.POINTER(_PortSpec), _LOG_FN, _POD_FN,
                                    ctypes.c_void_p, ctypes.POINTER(_PortReport)]
        lib.nxs_unit_open_i2c.restype = ctypes.c_void_p
        lib.nxs_unit_open_i2c.argtypes = [ctypes.c_void_p, ctypes.c_uint8,
                                          ctypes.POINTER(ctypes.c_int)]
        lib.nxs_unit_close.restype = None
        lib.nxs_unit_close.argtypes = [ctypes.c_void_p]
        lib.nxs_unit_last_refusal.restype = None
        lib.nxs_unit_last_refusal.argtypes = [ctypes.c_void_p, ctypes.POINTER(_Refusal)]
        _declare_bus(lib)
        _lib = lib
    return _lib


def _declare_bus(lib: ctypes.CDLL) -> None:
    """The signatures of the calls `Bus` makes, declared with the library:
    ctypes passes an undeclared handle as a C int, which truncates it."""
    u8p = ctypes.POINTER(ctypes.c_uint8)
    for name, args in (("nxs_bus_lock", [ctypes.c_void_p, ctypes.c_uint32]),
                       ("nxs_bus_unlock", [ctypes.c_void_p]),
                       ("nxs_bus_probe", [ctypes.c_void_p, ctypes.c_uint8]),
                       ("nxs_bus_read", [ctypes.c_void_p, ctypes.c_uint8, u8p, ctypes.c_size_t,
                                         u8p, ctypes.c_size_t]),
                       ("nxs_bus_write", [ctypes.c_void_p, ctypes.c_uint8, u8p, ctypes.c_size_t])):
        fn = getattr(lib, name)
        fn.restype = ctypes.c_int
        fn.argtypes = args


def _declare(lib: ctypes.CDLL) -> None:
    """The unit client's signatures; the module that uses them registers itself."""
    for declare in _DECLARERS:
        declare(lib)


_DECLARERS: list = []


def version() -> str:
    """The library's version string."""
    return _load().nxs_version().decode()


@dataclass(frozen=True)
class Message:
    """One message as it crossed the wire."""
    addr: int
    read: bool
    data: bytes


@dataclass(frozen=True)
class Report:
    """What a run ended with: `rc` is 0 for a halt, else a negative errno;
    `error` names the cause; a fault carries the instruction (`pc`,
    `opcode`, `reg` when the op named one, `addr`, `probing`)."""
    rc: int
    error: str
    pc: int
    opcode: int
    reg: Optional[int]
    addr: int
    probing: bool
    params: Tuple[int, ...]
    reg_reads: int
    reg_writes: int

    @property
    def ok(self) -> bool:
        return self.rc == 0

    @property
    def errno_name(self) -> str:
        return errno.errorcode.get(-self.rc, str(-self.rc)) if self.rc else ""


def _report(report: _Report) -> Report:
    return Report(rc=report.rc, error=report.error.decode(), pc=report.pc,
                  opcode=report.opcode, reg=report.reg if report.names_reg else None,
                  addr=report.addr, probing=bool(report.probing),
                  params=tuple(report.params[:report.num_params]),
                  reg_reads=report.reg_reads, reg_writes=report.reg_writes)


@dataclass(frozen=True)
class Refusal:
    """What a unit refused with: its own errno numbering and the reason."""
    device_errno: int
    reason: str


@dataclass(frozen=True)
class PersonalityParams:
    """Where a sensor personality's parameter table takes its values; None
    for a parameter the personality does not declare."""
    mode: int
    trigger: Optional[int] = None
    action: Optional[int] = None
    line_time: Optional[int] = None
    frame_period: Optional[int] = None


@dataclass(frozen=True)
class Pod:
    """A link's unit: where the host reaches it, its own address, and the
    personality a store slot must hold (the walk finds the slot)."""
    alias: int
    target: int
    personality: str


@dataclass(frozen=True)
class PortLink:
    """One link of the port as the walk takes it. `sensor_image` is the
    host's image of the personality: a link with a pod runs the same build
    on it, one without runs the image on the host at the head's address.
    `line_time_ns` and `frame_period_ns` are what the personality stages
    (0 when it takes none); `frame_fixed` says a rate law fixed the frame
    the timing action writes, else the timing action starts the sensor
    and its tables set the frame."""
    name: str
    has_camera: bool = True
    ser_addr: int = 0
    head_addr: int = 0
    lanes: int = 4
    data_type: int = 0
    host_csi: bool = False
    params: Optional[PersonalityParams] = None
    mode: int = 0
    trigger: int = 0
    line_time_ns: int = 0
    frame_period_ns: int = 0
    frame_fixed: bool = False
    pod: Optional[Pod] = None
    entries: Tuple[Tuple[int, int], ...] = ()
    sensor_image: Optional[bytes] = None


@dataclass(frozen=True)
class PortSpec:
    """The port as the walk takes it: the hub, its CSI output, the links
    and the two hub images."""
    des_addr: int
    csi_lanes: int
    hub_image: bytes
    ser_image: bytes
    links: Tuple[PortLink, ...]
    homogeneous: bool = False


@dataclass(frozen=True)
class PortReport:
    """Where a walk stopped: `rc` 0 when every step ran, else the step,
    the executor's report of it and the pod's refusal."""
    rc: int
    step: str
    run: Report
    refusal: Refusal

    @property
    def ok(self) -> bool:
        return self.rc == 0

    def __str__(self) -> str:
        if self.rc == 0:
            return "ok"
        if self.refusal.reason:
            suffix = f" (errno {self.refusal.device_errno})" if self.refusal.device_errno else ""
            return f"{self.step}: {self.refusal.reason}{suffix}"
        if self.run.error:
            where = f"{self.step}: {self.run.error}"
            if self.run.reg is not None:
                where += f" at {self.run.reg:#06x}"
            return f"{where} (device {self.run.addr:#04x}, pc {self.run.pc})"
        if self.rc == -errno.ETIMEDOUT and self.step.startswith("pod "):
            # The unit took the run and reported no end: the host's wait ran out.
            return f"{self.step}: the unit's run did not end within the walk's wait"
        return f"{self.step}: {errno.errorcode.get(-self.rc, str(-self.rc))}"


ReadFn = Callable[[int, bytes, int], Union[bytes, int]]
WriteFn = Callable[[int, bytes], int]
TraceFn = Callable[[Message, int], None]
LogFn = Callable[[str], None]
PodFn = Callable[[str, object], None]   # (link, the open `nxs._libnxs_unit.I2cUnit`)


def _buffer(data: bytes):
    """A byte buffer the library reads through a `uint8_t *`."""
    buf = ctypes.create_string_buffer(bytes(data), max(1, len(data)))
    return buf, ctypes.cast(buf, ctypes.POINTER(ctypes.c_uint8))


def _index(value: Optional[int]) -> int:
    return NO_PARAM if value is None else int(value)


class Bus:
    """One libnxs bus handle: real (`open`) or scripted (`scripted`)."""

    def __init__(self, handle: int):
        self._handle = handle
        self._keep: list = []
        self._failure: Optional[BaseException] = None

    @classmethod
    def open(cls, path: str) -> "Bus":
        """A bus on the I2C character device at `path`; OSError with the
        errno the open failed with."""
        err = ctypes.c_int(0)
        handle = _load().nxs_bus_open(path.encode(), ctypes.byref(err))
        if not handle:
            raise OSError(err.value, os.strerror(err.value), path)
        return cls(handle)

    @classmethod
    def scripted(cls, read: Optional[ReadFn], write: Optional[WriteFn]) -> "Bus":
        """A bus answered by Python: `read(addr, header, length)` returns the
        bytes to answer or a negative errno, `write(addr, data)` returns 0
        or a negative errno. An exception in either fails the transaction
        with EIO and is raised again when the run returns."""
        bus = cls(0)

        def c_read(_ctx, addr, header, header_len, out, out_len):
            if read is None:
                return -errno.ENOSYS
            try:
                answer = read(addr, bytes(header[:header_len]) if header_len else b"", out_len)
            except BaseException as exc:
                bus._failure = exc
                return -errno.EIO
            if isinstance(answer, int):
                return answer
            if len(answer) != out_len:
                # A short answer would leave the tail of the VM's buffer as
                # whatever an earlier read left there.
                bus._failure = ValueError(
                    f"scripted read of {out_len} bytes at 0x{addr:02x} answered {len(answer)}")
                return -errno.EIO
            for i, b in enumerate(answer):
                out[i] = b
            return 0

        def c_write(_ctx, addr, data, length):
            if write is None:
                return -errno.ENOSYS
            try:
                return int(write(addr, bytes(data[:length])))
            except BaseException as exc:
                bus._failure = exc
                return -errno.EIO

        read_cb = _READ_FN(c_read)
        write_cb = _WRITE_FN(c_write)
        bus._keep += [read_cb, write_cb]
        bus._handle = _load().nxs_bus_open_scripted(read_cb, write_cb, None)
        if not bus._handle:
            raise MemoryError("libnxs: no bus handle")
        return bus

    def _live(self) -> int:
        """The native handle; RuntimeError once the bus is closed, since the
        library dereferences the handle it is given."""
        if not self._handle:
            raise RuntimeError("libnxs: the bus is closed")
        return self._handle

    def lock(self, timeout_s: float = 10.0) -> None:
        """Take the bus lock every tool run shares; OSError(EBUSY) past `timeout_s`."""
        rc = _load().nxs_bus_lock(self._live(), int(timeout_s * 1000))
        if rc != 0:
            raise OSError(-rc, os.strerror(-rc), "the bus lock")

    def unlock(self) -> None:
        _load().nxs_bus_unlock(self._live())

    def probe(self, addr: int) -> bool:
        """Whether a device acknowledges `addr`."""
        rc = _load().nxs_bus_probe(self._live(), addr)
        if rc < 0:
            raise OSError(-rc, os.strerror(-rc), f"probe {addr:#04x}")
        return rc == 1

    def read(self, addr: int, header: bytes, length: int) -> bytes:
        """One combined transaction: `header` written, `length` bytes read."""
        out = (ctypes.c_uint8 * max(1, length))()
        hdr = (ctypes.c_uint8 * max(1, len(header)))(*header)
        rc = _load().nxs_bus_read(self._live(), addr, hdr, len(header), out, length)
        if rc != 0:
            raise OSError(-rc, os.strerror(-rc), f"read {addr:#04x}")
        return bytes(out[:length])

    def write(self, addr: int, data: bytes) -> None:
        buf = (ctypes.c_uint8 * max(1, len(data)))(*data)
        rc = _load().nxs_bus_write(self._live(), addr, buf, len(data))
        if rc != 0:
            raise OSError(-rc, os.strerror(-rc), f"write {addr:#04x}")

    def set_trace(self, trace: Optional[TraceFn]) -> None:
        """Install an observer of every message; None removes it."""
        handle = self._live()
        if trace is None:
            _load().nxs_bus_set_trace(handle, _TRACE_FN(), None)
            return

        def c_trace(_ctx, msg, rc):
            m = msg.contents
            data = bytes(m.data[:m.len]) if m.len else b""
            try:
                trace(Message(addr=m.addr, read=bool(m.read), data=data), rc)
            except BaseException as exc:
                self._failure = exc

        cb = _TRACE_FN(c_trace)
        self._keep.append(cb)
        _load().nxs_bus_set_trace(handle, cb, None)

    def run(self, image: bytes, primary: int, params: Mapping[int, int] = {}) -> Report:
        """Run `image` with `params` (index -> value) staged, the device
        bound to `primary`."""
        handle = self._live()
        if len(params) > MAX_PARAMS:
            raise ValueError(f"at most {MAX_PARAMS} parameters, got {len(params)}")
        staged = (_Param * max(1, len(params)))()
        for i, (index, value) in enumerate(params.items()):
            staged[i].index = index
            staged[i].value = value
        report = _Report()
        self._failure = None
        _load().nxs_run(handle, primary, image, len(image), staged, len(params),
                        ctypes.byref(report))
        if self._failure is not None:
            raise self._failure
        return _report(report)

    def port_up(self, spec: PortSpec, log: Optional[LogFn] = None,
                pod: Optional[PodFn] = None) -> PortReport:
        """Bring the port's links up: the hub image's phases, the serializers'
        address maps and the pods' actions in the walk's order (`nxs_port_up`).
        `log` receives one line per step, the step's name first; `pod(link,
        unit)` runs once per link at its first turn with the unit open
        through the hub's window, before the walk looks for the slot. An
        exception there ends the walk and is raised again here."""
        handle = self._live()
        if len(spec.links) > _PORT_LINKS:
            raise ValueError(f"a port has at most {_PORT_LINKS} links, got {len(spec.links)}")
        keep = []
        c_spec = _PortSpec()
        c_spec.des_addr = spec.des_addr
        c_spec.csi_lanes = spec.csi_lanes
        c_spec.homogeneous = 1 if spec.homogeneous else 0
        for buf, field in ((spec.hub_image, "hub_image"), (spec.ser_image, "ser_image")):
            holder, pointer = _buffer(buf)
            keep.append(holder)
            setattr(c_spec, field, pointer)
            setattr(c_spec, field + "_len", len(buf))
        for i, link in enumerate(spec.links):
            c = c_spec.links[i]
            c.name = link.name.encode()
            c.present = 1
            c.has_camera = 1 if link.has_camera else 0
            c.ser_addr = link.ser_addr
            c.head_addr = link.head_addr
            c.lanes = link.lanes
            c.data_type = link.data_type
            c.host_csi = 1 if link.host_csi else 0
            params = link.params or PersonalityParams(mode=NO_PARAM)
            c.params.mode = _index(params.mode)
            c.params.trigger = _index(params.trigger)
            c.params.action = _index(params.action)
            c.params.line_time = _index(params.line_time)
            c.params.frame_period = _index(params.frame_period)
            c.mode = link.mode
            c.trigger = link.trigger
            c.line_time_ns = link.line_time_ns
            c.frame_period_ns = link.frame_period_ns
            c.frame_fixed = 1 if link.frame_fixed else 0
            if link.pod is not None:
                c.pod.present = 1
                c.pod.alias = link.pod.alias
                c.pod.target = link.pod.target
                c.pod.personality = link.pod.personality.encode()
            if len(link.entries) > _ADDRESS_MAP_MAX:
                raise ValueError(f"link {link.name}: at most {_ADDRESS_MAP_MAX} address entries")
            c.num_entries = len(link.entries)
            for e, (alias, target) in enumerate(link.entries):
                c.entries[e].alias = alias
                c.entries[e].target = target
            if link.sensor_image is not None:
                holder, pointer = _buffer(link.sensor_image)
                keep.append(holder)
                c.sensor_image = pointer
                c.sensor_image_len = len(link.sensor_image)

        def c_log(_ctx, line):
            if log is not None:
                try:
                    log(line.decode())
                except BaseException as exc:
                    self._failure = exc

        def c_pod(_ctx, link, unit_handle):
            if pod is None:
                return 0
            try:
                from nxs._libnxs_unit import I2cUnit
                pod(link.decode(), I2cUnit(unit_handle, bus=self, owned=False,
                                           describe=f"i2c pod {link.decode()}"))
            except BaseException as exc:
                self._failure = exc
                return -errno.EIO
            return 0

        log_cb = _LOG_FN(c_log)
        pod_cb = _POD_FN(c_pod)
        keep += [log_cb, pod_cb]
        report = _PortReport()
        self._failure = None
        _load().nxs_port_up(handle, ctypes.byref(c_spec), log_cb, pod_cb, None,
                            ctypes.byref(report))
        if self._failure is not None:
            raise self._failure
        return PortReport(rc=report.rc, step=report.step.decode(), run=_report(report.run),
                          refusal=Refusal(report.refusal.device_errno,
                                          report.refusal.reason.decode()))

    def close(self) -> None:
        if self._handle:
            _load().nxs_bus_close(self._handle)
            self._handle = 0

    def __enter__(self) -> "Bus":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()


