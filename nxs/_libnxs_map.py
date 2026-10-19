# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The NXS map at one door, over the library (`nxs_door_*`): the header a
driver reads at probe, the rows and their cells, the mode table, and a
commit by rows with the transaction and the windows of Interface
Description §6.4. One door serves at each address: a unit's unit door, a
camera door, a smart camera."""

from __future__ import annotations

import contextlib
import ctypes
import errno
import os
import struct
from typing import Any, Dict, FrozenSet, Iterator, List, Mapping, Optional, Sequence, Set, Tuple, Union

from nxs import _libnxs as _base
from nxs._generated_constants import NxsDriverImage, NxsMap
from nxs._libnxs import Bus, _Param, _load
from nxs.personality.records_params import ParamRow, decode_rows

MapReg = NxsMap.MapReg
MapState = NxsMap.MapState
ParamId = NxsMap.ParamId
ParamKind = NxsDriverImage.ParamKind

#: errno EINVAL in the device's libc, what a refused commit reads in `ERROR`.
ERRNO_EINVAL = 22
#: The header from `MAP_ID` through `PROTO_VERSION`: identity, state, counts, mode, stream, commit
#: counts, sample cells, the four run times, module kind and interface version.
HEADER = struct.Struct("<14BH4H2B")
MODE_ROW = struct.Struct("<HHBBHBBIBB")
assert HEADER.size == MapReg.PROTO_VERSION + 1
assert MODE_ROW.size == NxsMap.MODE_ENTRY_SIZE
_CELL = struct.Struct("<I")
#: The cell that says how long the last commit of each kind took.
RUN_MS_CELL = {ParamKind.APPLY: MapReg.APPLY_RUN_MS, ParamKind.RUN: MapReg.MODE_RUN_MS,
               ParamKind.GATE: MapReg.GATE_RUN_MS, ParamKind.TRIGGER: MapReg.TRIGGER_RUN_MS}
#: The kinds a commit takes: every kind of the enum.
COMMIT_KINDS = (ParamKind.APPLY, ParamKind.RUN, ParamKind.GATE, ParamKind.TRIGGER)
#: The four values `STATE` takes; a read outside them is torn by the device's own transaction.
STATES = frozenset(int(s) for s in (MapState.INERT, MapState.READY, MapState.BUSY, MapState.FAULT))
#: The tries of a probe of the unit door: a pod's probe can land inside its
#: own transfer, and three tell a NAK from no unit.
UNIT_PROBES = 3
#: The owning slot's place in a row's flags: the lowest bit of the owner field.
OWNER_SHIFT = (NxsMap.ROW_FLAG_OWNER & -NxsMap.ROW_FLAG_OWNER).bit_length() - 1


class MapRefused(OSError):
    """What a door answers instead of what a client asks of it: no map this
    tool speaks, or a commit it ended without taking. The message names
    the fact."""


class MapInert(MapRefused):
    """The door reads INERT: no run has configured its sensor, and a
    `COMMIT` written now is ignored (Interface Description §6.11)."""


class _CommitEnd(ctypes.Structure):
    _fields_ = [("state", ctypes.c_uint8), ("error", ctypes.c_uint8), ("held_ms", ctypes.c_uint32)]


def _declare_door(lib: ctypes.CDLL) -> None:
    u8p = ctypes.POINTER(ctypes.c_uint8)
    lib.nxs_door_open.restype = ctypes.c_void_p
    lib.nxs_door_open.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.POINTER(ctypes.c_int)]
    lib.nxs_door_close.restype = None
    lib.nxs_door_close.argtypes = [ctypes.c_void_p]
    lib.nxs_door_naks.restype = ctypes.c_uint32
    lib.nxs_door_naks.argtypes = [ctypes.c_void_p]
    for name, args in (("nxs_door_read", [ctypes.c_void_p, ctypes.c_uint16, u8p, ctypes.c_size_t]),
                       ("nxs_door_write", [ctypes.c_void_p, ctypes.c_uint16, u8p, ctypes.c_size_t]),
                       ("nxs_door_commit", [ctypes.c_void_p, ctypes.POINTER(_Param), ctypes.c_size_t,
                                            ctypes.c_uint8, ctypes.POINTER(_CommitEnd)])):
        fn = getattr(lib, name)
        fn.restype = ctypes.c_int
        fn.argtypes = args


_base._DECLARERS.append(_declare_door)
if _base._lib is not None:
    _declare_door(_base._lib)


def identity_fact(map_id: int, version: int) -> Optional[str]:
    """Why a map of this identity and layout version is none this tool
    speaks, None when it is one: its identity cell reads a door of the NXS
    map and its version the layout this tool knows, and a driver refuses a
    version it does not know (§2)."""
    if map_id in (NxsMap.MAP_ID_UNIT, NxsMap.MAP_ID_CAMERA) and version == NxsMap.MAP_VERSION_VALUE:
        return None
    if map_id in (NxsMap.MAP_ID_UNIT, NxsMap.MAP_ID_CAMERA):
        return (f"id {map_id:#04x} v{version}, a layout this nxs does not speak (it speaks "
                f"v{NxsMap.MAP_VERSION_VALUE})")
    return (f"id {map_id:#04x}, and a door of the NXS map reads id {NxsMap.MAP_ID_UNIT:#04x} or "
            f"{NxsMap.MAP_ID_CAMERA:#04x}")


def camera_door_serves(addr: int) -> bool:
    """Whether the camera door serves `addr`: a byte of a cell the contract
    flags for it. Every other address reads 0 there and ignores a write."""
    return any(start <= addr < start + width for start, width in MapReg.CAMERA_DOOR_SPANS)


def unit_answers(unit) -> bool:
    """Whether a unit answers at its unit door, by `UNIT_PROBES` probes."""
    return any(unit.probe() for _ in range(UNIT_PROBES))


def signed(row: ParamRow, cell: int) -> int:
    """A u32 cell as the row's value: two's complement for a signed row."""
    return cell - (1 << 32) if row.is_signed and cell & 0x80000000 else cell


def staged_at(row: int) -> int:
    """Where row `row`'s staged cell sits: the staged array runs downward
    and ends at `MASK`, so a commit of the first rows is a few bytes."""
    return MapReg.MASK - 4 * (int(row) + 1)


def commit_payload(staged: Mapping[int, int]) -> Tuple[int, bytes]:
    """The write one commit is, as a driver makes it: the register it starts
    at, the cell of the highest row staged, and the bytes from there
    through the cells below it, `MASK` and `COMMIT` 1. `staged` maps a row
    index to its value in the row's unit; an empty commit is `MASK` 0 and
    `COMMIT` alone."""
    mask = 0
    for index in staged:
        mask |= 1 << int(index)
    last = max((int(i) for i in staged), default=-1)
    cells = bytearray(4 * (last + 1))
    for index, value in staged.items():
        _CELL.pack_into(cells, 4 * (last - int(index)), int(value) & 0xFFFFFFFF)
    return staged_at(last), bytes(cells) + mask.to_bytes(8, "little") + b"\x01"


def without_owner(rows: bytes) -> bytes:
    """Served rows as their record holds them: a door writes the owning slot
    into each row's flags as it serves the row."""
    out = bytearray(rows)
    for at in range(NxsMap.RowField.FLAGS, len(out), NxsMap.ROW_SIZE):
        out[at] &= ~NxsMap.ROW_FLAG_OWNER & 0xFF
    return bytes(out)


def find_row(rows: Sequence[ParamRow], row: Union[int, str]) -> Optional[int]:
    """The index of a row: by its standard id for an int, by its name for a
    str. A head names its rows as it likes, so a standard knob is found by
    its id; a custom row has its name alone."""
    if isinstance(row, str):
        return next((i for i, r in enumerate(rows) if r.name == row), None)
    return next((i for i, r in enumerate(rows) if r.id == int(row)), None)


class Door:
    """The NXS map at one I²C address on an open bus, through the library's
    door: a 2-byte pointer, a transaction the device does not acknowledge
    repeated after `BUSY_RETRY_MS`, at most `BUSY_RETRIES` times. The rows,
    their cells, the mode table and a commit are asked only of a map this
    tool speaks: the client reads the identity and the layout version once
    (`verify`)."""

    def __init__(self, bus: Bus, address: int) -> None:
        self._bus = bus
        self._address = int(address)
        err = ctypes.c_int(0)
        self._handle = _load().nxs_door_open(bus._live(), self._address, ctypes.byref(err))
        if not self._handle:
            raise OSError(err.value, os.strerror(err.value), f"door at {self._address:#04x}")
        self._last_commit_ms = 0
        self._known = False
        self._name = "map"
        self._kinds: Set[int] = set()

    @property
    def address(self) -> int:
        return self._address

    @property
    def naks(self) -> int:
        """The NAKs the device answered with so far, each repeated after the wait."""
        return int(_load().nxs_door_naks(self._live())) if self._handle else 0

    @property
    def last_commit_ms(self) -> int:
        """How long the last commit held the bus, in milliseconds."""
        return self._last_commit_ms

    @property
    def kinds_applied(self) -> FrozenSet[int]:
        """The kinds of the commits that ended READY with a row applied."""
        return frozenset(self._kinds)

    def close(self) -> None:
        if self._handle:
            _load().nxs_door_close(self._handle)
            self._handle = 0

    def __enter__(self) -> "Door":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def _live(self) -> int:
        if not self._handle:
            raise RuntimeError("libnxs: the door is closed")
        return self._handle

    def _check(self, rc: int, what: str) -> None:
        if rc == 0:
            return
        if self._bus._failure is not None:
            raise self._bus._failure
        raise OSError(-rc, os.strerror(-rc), f"{what} @{self._address:#04x}")

    def verify(self) -> None:
        """Refuse a map this tool does not speak, once per client: the
        identity and the layout version the header serves. MapRefused names
        both."""
        if self._known:
            return
        map_id, version = self.read(MapReg.MAP_ID, 2)
        fact = identity_fact(map_id, version)
        if fact is not None:
            raise MapRefused(f"the map @{self._address:#04x} reads {fact}")
        self._known = True
        self._name = "camera door" if map_id == NxsMap.MAP_ID_CAMERA else "unit door"

    def read(self, reg: int, length: int = 1) -> bytes:
        """`length` bytes from `reg`, the device's NAKs repeated."""
        out = (ctypes.c_uint8 * max(1, length))()
        self._check(_load().nxs_door_read(self._live(), int(reg), out, length), f"read {int(reg):#06x}")
        return bytes(out[:length])

    def write(self, reg: int, data: bytes) -> None:
        """`data` from `reg` on, in one transaction, the device's NAKs repeated."""
        buf = (ctypes.c_uint8 * max(1, len(data)))(*data)
        self._check(_load().nxs_door_write(self._live(), int(reg), buf, len(data)),
                    f"write {int(reg):#06x}")

    def u8(self, reg: int) -> int:
        return self.read(reg, 1)[0]

    def u16(self, reg: int) -> int:
        return int.from_bytes(self.read(reg, 2), "little")

    def state(self) -> int:
        return self.u8(MapReg.STATE)

    def header(self) -> Dict[str, Any]:
        """The header through `PROTO_VERSION`: the identity, the state, the
        counts, the mode and the stream in effect, the commit counts, the
        run times, the module's kind and the interface version."""
        (map_id, version, state, error, fw_major, fw_minor, param_count, mode_count,
         current_mode, stream, apply_count, fault_count, output_count, sample_size, _samples,
         mode_run_ms, gate_run_ms, trigger_run_ms, apply_run_ms, module_kind,
         proto_version) = HEADER.unpack(self.read(MapReg.MAP_ID, HEADER.size))
        return {"id": map_id, "version": version, "state": state, "error": error,
                "fw": (fw_major, fw_minor), "param_count": param_count,
                "mode_count": mode_count, "current_mode": current_mode, "stream": stream,
                "apply_count": apply_count, "fault_count": fault_count,
                "output_count": output_count, "sample_size": sample_size,
                "mode_run_ms": mode_run_ms, "gate_run_ms": gate_run_ms,
                "trigger_run_ms": trigger_run_ms, "apply_run_ms": apply_run_ms,
                "module_kind": module_kind, "proto_version": proto_version}

    def raw_rows(self) -> bytes:
        """The `PARAM_COUNT` rows as served, one row per read."""
        self.verify()
        count = self.u8(MapReg.PARAM_COUNT)
        return b"".join(self.read(MapReg.ROWS + i * NxsMap.ROW_SIZE, NxsMap.ROW_SIZE)
                        for i in range(count))

    def rows(self) -> List[ParamRow]:
        """The rows the door serves, decoded."""
        return decode_rows(self.raw_rows())

    def owners(self) -> List[int]:
        """The owning slot of each served row, `OWNER_DEVICE` for the device's own."""
        raw = self.raw_rows()
        flags = NxsMap.RowField.FLAGS
        return [raw[i + flags] >> OWNER_SHIFT for i in range(0, len(raw), NxsMap.ROW_SIZE)]

    def cells(self, rows: Sequence[ParamRow]) -> List[Dict[str, int]]:
        """`live`, `min`, `max` and `step` of each row in `rows`, a signed row's
        values in two's complement decoded."""
        n = len(rows)
        if n == 0:
            return []
        self.verify()
        arrays = {name: self.read(reg, 4 * n) for name, reg in (
            ("live", MapReg.LIVE), ("min", MapReg.MIN), ("max", MapReg.MAX), ("step", MapReg.STEP))}
        return [{name: signed(row, _CELL.unpack_from(raw, 4 * i)[0]) if name != "step"
                 else _CELL.unpack_from(raw, 4 * i)[0] for name, raw in arrays.items()}
                for i, row in enumerate(rows)]

    def mode_table(self) -> List[Dict[str, Any]]:
        """The mode table's rows up to `MODE_COUNT`."""
        self.verify()
        count = self.u8(MapReg.MODE_COUNT)
        raw = self.read(MapReg.MODE_TABLE, MODE_ROW.size * count) if count else b""
        table = []
        for i in range(count):
            (width, height, bit_depth, lanes, rate_mbps, data_type, flags, fps_max_milli,
             value, phase) = MODE_ROW.unpack_from(raw, i * MODE_ROW.size)
            table.append({"width": width, "height": height, "bit_depth": bit_depth, "lanes": lanes,
                          "rate_mbps": rate_mbps, "data_type": data_type, "flags": flags,
                          "fps_max_milli": fps_max_milli, "value": value, "pixel_phase": phase})
        return table

    def commit(self, staged: Mapping[int, int], kind: int) -> Tuple[int, int]:
        """One commit as a driver makes it (Interface Description §6.4), run
        by the library: `STATE` read first and a commit in flight waited
        out, one write from the highest staged row's cell through `MASK`
        and `COMMIT`, the time the door painted for the kind, then `STATE`
        polled until it leaves BUSY. `staged` maps a row index to its value
        in the row's unit, and `kind` is the kind every staged row shares.
        Returns `(state, error)` where the commit ended, BUSY when the door
        stays BUSY past the window. MapInert for an INERT door, MapRefused
        for a commit the door never took."""
        self.verify()
        params = (_Param * max(1, len(staged)))()
        for i, (index, value) in enumerate(staged.items()):
            params[i].index = int(index)
            params[i].value = int(value) & 0xFFFFFFFF
        end = _CommitEnd()
        rc = _load().nxs_door_commit(self._live(), params, len(staged), int(kind), ctypes.byref(end))
        self._last_commit_ms = int(end.held_ms)
        if rc == -errno.EAGAIN:
            raise MapInert(f"the {self._name} @{self._address:#04x} is INERT: no run has configured "
                           f"its sensor")
        if rc == -errno.EBUSY:
            return MapState.BUSY, 0
        if rc == -errno.EPROTO:
            raise MapRefused(f"the {self._name} @{self._address:#04x} ended {MapState._NAMES.get(int(end.state))} "
                             f"with its count where it stood: it never took the commit")
        self._check(rc, "commit")
        if staged and end.state == MapState.READY:
            self._kinds.add(int(kind))
        return int(end.state), int(end.error)

    def live(self, row: Union[int, str]) -> Tuple[int, ParamRow]:
        """The value a row reads in effect and the row: by its standard id
        for an int, by its name for a str. LookupError for a row the door
        does not serve."""
        rows = self.rows()
        index = find_row(rows, row)
        if index is None:
            raise LookupError(f"no row {row} at the {self._name} @{self._address:#04x}")
        return self.cells(rows[:index + 1])[index]["live"], rows[index]


@contextlib.contextmanager
def open_door(bus_path: str, address: int) -> Iterator[Door]:
    """The door at `address` on the I²C bus at `bus_path`, the bus closed on leaving."""
    bus = Bus.open(bus_path)
    try:
        with Door(bus, address) as door:
            yield door
    finally:
        bus.close()
