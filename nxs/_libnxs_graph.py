# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The graph calls of libnxs (`nxs.h`, The graph): what a graph image
states (`info`), the bring-up of the port it states by the library's
policy (`up`), one of its walks (`walk`), and one parameter set on a link
outside any walk (`set_param`). The owner's side of a walk is
a `GraphEnv`: its image store, its port record, its host steps, its abort
flag and its log, each called back from inside the library while the walk
runs. The caller holds the bus lock."""

from __future__ import annotations

import ctypes
import errno
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from nxs import _libnxs as _base
from nxs._generated_constants import NxsMap
from nxs._libnxs import Bus, Refusal, Report, _LOG_FN, _Refusal, _Report, _report

Walk = NxsMap.GraphWalk
StepKind = NxsMap.GraphStepKind
HostOp = NxsMap.GraphHostOp

_NAME_LEN = 32
_ERROR_LEN = 32
_LINKS = NxsMap.GRAPH_LINKS_MAX
_PARAMS = len(NxsMap.GraphParam._NAMES)
_DIGEST_LEN = NxsMap.GRAPH_DIGEST_SIZE
#: The walks a graph image names, in `GraphWalk` order, as the walk's lines name them.
WALKS: Tuple[str, ...] = ("up", "construct", "train", "recover", "status", "sync", "free", "gain",
                          "park", "gate-on", "gate-off", "timing")
assert len(WALKS) == len(Walk._NAMES)
#: The report's step and walk when none failed, `GRAPH_NONE`.
NONE = NxsMap.GRAPH_NONE
#: What `info` refuses an image with, by errno.
_INFO_REFUSALS = {errno.EBADF: "the image does not parse", errno.ENOEXEC: "it is no graph image",
                  errno.ENOENT: "a GRAPH record is missing", errno.EBADMSG: "a record is malformed",
                  errno.ESTALE: "its records compose against another program than the one it "
                                "carries"}


class _Summary(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char * _NAME_LEN), ("program", ctypes.c_char * _NAME_LEN),
                ("program_crc", ctypes.c_uint32), ("digest", ctypes.c_uint8 * _DIGEST_LEN),
                ("des_addr", ctypes.c_uint8), ("owner", ctypes.c_uint8), ("flags", ctypes.c_uint8),
                ("num_links", ctypes.c_uint8), ("links", ctypes.c_char * _LINKS),
                ("link_flags", ctypes.c_uint8 * _LINKS), ("num_outputs", ctypes.c_uint8),
                ("steps", ctypes.c_uint8 * len(WALKS))]


class _Head(ctypes.Structure):
    _fields_ = [("kept", ctypes.c_uint8), ("values", ctypes.c_uint32 * _PARAMS)]


class _Record(ctypes.Structure):
    _fields_ = [("warm", ctypes.c_uint8), ("verdict", ctypes.c_int32), ("heads", _Head * _LINKS)]


class _Image(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_uint8)), ("len", ctypes.c_size_t)]


_FIND_FN = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32,
                            ctypes.POINTER(_Image))
_RECORD_GET_FN = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.POINTER(_Record))
_RECORD_SET_FN = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.POINTER(_Record))
_HOST_FN = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint8, ctypes.c_char,
                            ctypes.c_uint32)


class _Env(ctypes.Structure):
    _fields_ = [("find_image", _FIND_FN), ("record_get", _RECORD_GET_FN),
                ("record_set", _RECORD_SET_FN), ("host", _HOST_FN),
                ("abort", ctypes.POINTER(ctypes.c_int)), ("log", _LOG_FN),
                ("ctx", ctypes.c_void_p)]


class _GraphReport(ctypes.Structure):
    _fields_ = [("rc", ctypes.c_int), ("walk", ctypes.c_uint8), ("step", ctypes.c_uint8),
                ("kind", ctypes.c_uint8), ("link", ctypes.c_char), ("locked", ctypes.c_uint8),
                ("recovered", ctypes.c_uint8), ("what", ctypes.c_char * _ERROR_LEN),
                ("run", _Report), ("refusal", _Refusal)]


def _declare(lib: ctypes.CDLL) -> None:
    lib.nxs_graph_info.restype = ctypes.c_int
    lib.nxs_graph_info.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(_Summary)]
    for name, extra in (("nxs_graph_up", []), ("nxs_graph_walk", [ctypes.c_uint8])):
        fn = getattr(lib, name)
        fn.restype = ctypes.c_int
        fn.argtypes = ([ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t] + extra
                       + [ctypes.POINTER(_Env), ctypes.POINTER(_GraphReport)])
    lib.nxs_graph_set.restype = ctypes.c_int
    lib.nxs_graph_set.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t,
                                  ctypes.POINTER(_Env), ctypes.c_char, ctypes.c_uint8,
                                  ctypes.c_uint32, ctypes.POINTER(_GraphReport)]


_base._DECLARERS.append(_declare)
_declared: List[ctypes.CDLL] = []


def _lib() -> ctypes.CDLL:
    """The library with the graph calls declared, whenever it was loaded."""
    lib = _base._load()
    if lib not in _declared:
        _declare(lib)
        _declared.append(lib)
    return lib


@dataclass(frozen=True)
class GraphSummary:
    """What a graph image states: its name (the port's), the hub program
    its records compose against, the declaration's digest, the hub's
    address, the owner, the flags, each link's name and `GRAPH_LINK_*`
    bits, the outputs and the steps of each walk by `GraphWalk`."""

    name: str
    program: str
    program_crc: int
    digest: bytes
    des_addr: int
    owner: int
    flags: int
    links: Tuple[str, ...]
    link_flags: Tuple[int, ...]
    outputs: int
    steps: Dict[int, int]

    @property
    def sync(self) -> bool:
        return bool(self.flags & NxsMap.GRAPH_FLAG_SYNC)

    @property
    def gain(self) -> bool:
        return bool(self.flags & NxsMap.GRAPH_FLAG_GAIN)


@dataclass(frozen=True)
class GraphRecord:
    """The port record a walk reads and keeps through its owner: warm while
    the last bring-up ended up, that bring-up's verdict, and what each
    link's bare head's last run on the owner left, by the image's link row
    and `GraphParam` (None for a head no run left anything for)."""

    warm: bool
    verdict: int = 0
    heads: Tuple[Optional[Tuple[int, ...]], ...] = (None,) * _LINKS


@dataclass(frozen=True)
class GraphReport:
    """Where a walk ended: the verdict, the walk, the failing step's index,
    kind and link (None when no step failed), the lock bits a lock walk
    read last, whether the bring-up ran again after a recovery, the failing
    step's line, the executor's report of it and the unit's refusal or the
    walk's reason."""

    rc: int
    walk: Optional[int]
    step: Optional[int]
    kind: int
    link: Optional[str]
    locked: int
    recovered: bool
    what: str
    run: Report
    refusal: Refusal

    @property
    def ok(self) -> bool:
        return self.rc == 0

    @property
    def walk_name(self) -> str:
        return WALKS[self.walk] if self.walk is not None and self.walk < len(WALKS) else ""

    def reason(self) -> str:
        """Why the step failed: the unit's refusal or the walk's reason, else
        the executor's fault, either with the register the fault named,
        else the errno."""
        at = (f" at {self.run.reg:#06x} (device {self.run.addr:#04x}, pc {self.run.pc})"
              if self.run.error and self.run.reg is not None else "")
        if self.refusal.reason:
            suffix = f" (errno {self.refusal.device_errno})" if self.refusal.device_errno else ""
            return f"{self.refusal.reason}{at}{suffix}"
        if self.run.error:
            where = at or f" (device {self.run.addr:#04x}, pc {self.run.pc})"
            return f"{self.run.error}{where}"
        if self.rc == -errno.ETIMEDOUT and self.kind == StepKind.POD:
            # The unit took the run and reported no end: the walk's wait ran out.
            return "the unit's run did not end within the walk's wait"
        return errno.errorcode.get(-self.rc, str(-self.rc))

    def __str__(self) -> str:
        if self.rc == 0:
            return "ok"
        return f"{self.what}: {self.reason()}" if self.what else self.reason()


class GraphEnv:
    """The owner's side of a walk, called back while the library walks:
    `find_image` answers an image of the owner's store by its name and the
    CRC-32 of its trailer, `record_get` and `record_set` read and keep the
    port record, `host` runs a HOST step (`GraphHostOp`), `log` takes one
    line per step and per decision of the policy. `stop` asks the walk to
    end before its next step. Each callback answers what the base class
    does when the owner does not override it."""

    def __init__(self) -> None:
        self._abort = ctypes.c_int(0)

    def find_image(self, name: str, crc: int) -> Optional[bytes]:
        del name, crc
        return None

    def record_get(self) -> Optional[GraphRecord]:
        return None

    def record_set(self, record: GraphRecord) -> None:
        del record

    def host(self, op: int, link: Optional[str], value: int) -> int:
        del op, link, value
        return -errno.ENOTSUP

    def log(self, line: str) -> None:
        del line

    def stop(self) -> None:
        """End the walk before its next step: it reports `-ECANCELED`."""
        self._abort.value = 1


class _Call:
    """One call's C side of an env: the callbacks and the buffers the walk
    reads, kept alive for the call, and the first exception a callback
    raised, which the call raises again once the walk returns."""

    def __init__(self, env: Optional[GraphEnv]):
        self._failure: Optional[BaseException] = None
        self._keep: list = []
        self._c_env = _Env()
        if env is None:
            return

        def guard(fn, fallback):
            def wrapped(*args):
                try:
                    return fn(*args)
                except BaseException as exc:
                    if self._failure is None:
                        self._failure = exc
                    return fallback
            return wrapped

        def find(_ctx, name, crc, out):
            image = env.find_image(name.decode(), int(crc))
            if not image:
                return -errno.ENOENT
            holder = ctypes.create_string_buffer(bytes(image), len(image))
            self._keep.append(holder)
            out.contents.data = ctypes.cast(holder, ctypes.POINTER(ctypes.c_uint8))
            out.contents.len = len(image)
            return 0

        def record_get(_ctx, out):
            record = env.record_get()
            if record is None:
                return -errno.ENOENT
            out.contents.warm = 1 if record.warm else 0
            out.contents.verdict = int(record.verdict)
            for row, values in enumerate(record.heads[:_LINKS]):
                head = out.contents.heads[row]
                head.kept = 1 if values is not None else 0
                for i, value in enumerate((values or ())[:_PARAMS]):
                    head.values[i] = int(value) & 0xFFFFFFFF
            return 0

        def record_set(_ctx, record):
            heads = tuple(tuple(int(v) for v in head.values) if head.kept else None
                          for head in record.contents.heads)
            env.record_set(GraphRecord(warm=bool(record.contents.warm),
                                       verdict=int(record.contents.verdict), heads=heads))
            return 0

        def host(_ctx, op, link, value):
            name = link.decode() if link and link != b"\0" else None
            return int(env.host(int(op), name, int(value)))

        def log(_ctx, line):
            env.log(line.decode(errors="replace"))

        callbacks = (_FIND_FN(guard(find, -errno.EIO)), _RECORD_GET_FN(guard(record_get, -errno.EIO)),
                     _RECORD_SET_FN(guard(record_set, -errno.EIO)), _HOST_FN(guard(host, -errno.EIO)),
                     _LOG_FN(guard(log, None)))
        self._keep += list(callbacks)
        c_env = self._c_env
        c_env.find_image, c_env.record_get, c_env.record_set, c_env.host = callbacks[:4]
        c_env.log = callbacks[4]
        c_env.abort = ctypes.pointer(env._abort)

    @property
    def failure(self) -> Optional[BaseException]:
        return self._failure

    @property
    def c_env(self) -> _Env:
        return self._c_env


def info(image: bytes) -> GraphSummary:
    """What a graph image states; OSError with the library's errno and why
    for an image it refuses."""
    out = _Summary()
    rc = _lib().nxs_graph_info(bytes(image), len(image), ctypes.byref(out))
    if rc != 0:
        why = _INFO_REFUSALS.get(-rc, os.strerror(-rc))
        raise OSError(-rc, f"graph image: {why}")
    count = int(out.num_links)
    # A char array field reads back as bytes cut at its first NUL: the raw cells hold every name.
    names = ctypes.string_at(ctypes.addressof(out) + _Summary.links.offset, _LINKS)
    return GraphSummary(
        name=out.name.decode(), program=out.program.decode(), program_crc=int(out.program_crc),
        digest=bytes(out.digest), des_addr=int(out.des_addr), owner=int(out.owner),
        flags=int(out.flags), links=tuple(chr(names[i]) for i in range(count)),
        link_flags=tuple(int(out.link_flags[i]) for i in range(count)),
        outputs=int(out.num_outputs), steps={w: int(out.steps[w]) for w in range(len(WALKS))})


def _graph_report(report: _GraphReport) -> GraphReport:
    link = report.link.decode() if report.link and report.link != b"\0" else None
    return GraphReport(
        rc=int(report.rc), walk=None if report.walk == NONE else int(report.walk),
        step=None if report.step == NONE else int(report.step), kind=int(report.kind),
        link=link, locked=int(report.locked), recovered=bool(report.recovered),
        what=report.what.decode(errors="replace"), run=_report(report.run),
        refusal=Refusal(int(report.refusal.device_errno),
                        report.refusal.reason.decode(errors="replace")))


def _call(bus: Bus, image: bytes, env: Optional[GraphEnv], run) -> GraphReport:
    call = _Call(env)
    report = _GraphReport()
    data = bytes(image)
    bus._failure = None
    run(bus._live(), data, len(data), ctypes.byref(call.c_env), ctypes.byref(report))
    for failure in (call.failure, bus._failure):
        if failure is not None:
            raise failure
    return _graph_report(report)


def up(bus: Bus, image: bytes, env: Optional[GraphEnv] = None) -> GraphReport:
    """Bring the port `image` states up by the library's policy
    (`nxs_graph_up`): construction or status and training, the up walk with
    one recovery, then the sync and gain walks the image's flags ask for. An
    exception a callback raised ends the call and is raised here."""
    return _call(bus, image, env, lambda handle, data, n, c_env, out:
                 _lib().nxs_graph_up(handle, data, n, c_env, out))


def walk(bus: Bus, image: bytes, which: int, env: Optional[GraphEnv] = None) -> GraphReport:
    """Run the walk `which` (`GraphWalk`) of `image` without the policy
    (`nxs_graph_walk`): a soft step's failure is logged and the walk goes
    on, any other ends it. A walk the image does not carry reports
    `-ENOENT`."""
    return _call(bus, image, env, lambda handle, data, n, c_env, out:
                 _lib().nxs_graph_walk(handle, data, n, int(which), c_env, out))


def set_param(bus: Bus, image: bytes, link: str, param: int, value: int,
              env: Optional[GraphEnv] = None) -> GraphReport:
    """Set one parameter on `link` of `image` outside any walk
    (`nxs_graph_set`). Behind a pod `param` is the camera door's row and
    `value` is in the row's unit, committed as an APPLY. On a bare head
    `param` is the personality's run parameter and `value` is in its unit,
    staged under the personality's apply action on the owner. A link the
    image does not carry reports `-ENOENT`."""
    return _call(bus, image, env, lambda handle, data, n, c_env, out:
                 _lib().nxs_graph_set(handle, data, n, c_env, link.encode(), int(param),
                                      int(value), out))
