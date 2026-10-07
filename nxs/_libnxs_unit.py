# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The tool's client of one unit, over the library (`nxs_unit_*`): the same
operation set on I²C, Cyphal/CAN and Cyphal/serial, as `NxsClient` and the
capability surfaces the verbs check."""

from __future__ import annotations

import ctypes
import dataclasses
import errno
import os
import threading
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple

from nxs._libnxs import EREFUSED, MAX_PARAMS, Bus, _Param, _Refusal, _SlotInfo, _CamState, _load
from nxs._libnxs_abi import (
    _CalProgress, _Commission, _Diag, _Identity, _OutputInfo, _ParamInfo, _PROGRESS_FN,
    _Sample, _State, _StreamOpts, _TimeSync, _TimeSyncFit, _XferState)
from nxs.capabilities import (
    CalibrationRecord, SupportsBitTiming, SupportsCalibration, SupportsCameraRun,
    SupportsCanTermination, SupportsCommissioning, SupportsEgressDecimation,
    SupportsFaultCounters, SupportsIdentify, SupportsRecovery, SupportsSampleFifo,
    SupportsSlotPeek, SupportsTimeSync, validate_can_bitrate, validate_can_term,
    validate_commission, validate_sample_fifo_depth)
from nxs.client import COMMISSION_TOPICS, LINK_DROP_ADVICE, NxsClient
from nxs.device_errors import DeviceRefused

_TRAILER_MAX = 2048
_CALIBRATION_RECORD_MAX = 256
#: The default unit address on a port's bus and behind a hub before the aliases.
DEFAULT_I2C_ADDRESS = 0x30
DEFAULT_NODE_ID = 125
HOST_NODE_ID = 127
DEFAULT_BAUD = 460800
DEFAULT_CAN_MTU = 64
#: The per-subject decimation gates, by the unit's subject bucket (1 onward).
DECIMATION_SUBJECTS = ("acceleration", "angular_velocity", "magnetic_field", "temperature",
                       "pressure", "scalar")
_RESET_SETTLE_MS = 2000
_CAM_RUN_TIMEOUT_MS = 30000
_CAM_ABORT_TIMEOUT_MS = 5000


@dataclass(frozen=True)
class TimeSync:
    """One time-sync record: the unit's offset to host realtime, the bound,
    the rate, the window the unit holds it for, and its source."""
    offset_us: int
    bound_us: int
    rate_ppb: int
    valid_for_us: int
    source: int
    valid: bool

    def as_tuple(self) -> tuple:
        return (self.offset_us, self.bound_us, self.rate_ppb, self.valid_for_us, self.source,
                self.valid)


def _text(raw: bytes) -> str:
    return raw.decode("ascii", "replace")


def _param_dict(index: int, info: "_ParamInfo") -> dict:
    """One parameter descriptor as the tool reads it."""
    return {"idx": index, "name": _text(info.name),
            "type": "enum" if info.param_type == 0 else "range",
            "kind": "reload" if info.reload else "live",
            "default": int(info.default_value), "current": int(info.current),
            "values": [int(v) for v in info.values[:info.num_values]], "unit": _text(info.unit)}


def _output_dict(index: int, info: "_OutputInfo") -> dict:
    """One output descriptor as the tool reads it."""
    from nxs.image import FIELD_TYPE_NAMES

    return {"idx": index, "name": _text(info.name),
            "type": FIELD_TYPE_NAMES.get(info.field_type, f"type{info.field_type}"),
            "byte_order": "big" if info.byte_order == 0 else "little",
            "semantic": int(info.semantic), "byte_off": int(info.byte_offset),
            "count": int(info.count), "scale": float(info.scale), "offset": float(info.offset),
            "unit": _text(info.unit)}


def _bus_path(bus) -> str:
    """The device node of an I²C bus given as a number, a digit string or a path."""
    if bus is None:
        raise ValueError("an I2C client needs a bus: an int bus number or a device path")
    if isinstance(bus, str) and not bus.isdigit():
        if not bus.startswith("/"):
            raise ValueError(f"I2C bus '{bus}': expected an integer, 'N', or a device path "
                             f"like /dev/i2c-N")
        return bus
    return f"/dev/i2c-{int(bus)}"


class Unit(NxsClient, SupportsRecovery, SupportsTimeSync, SupportsIdentify, SupportsSlotPeek,
           SupportsFaultCounters, SupportsBitTiming, SupportsCanTermination, SupportsSampleFifo,
           SupportsCalibration, SupportsCommissioning):
    """The tool's client of one unit, over the library (`nxs_unit_*`): the
    same operation set on every transport, with `caps` naming what the
    wire cannot carry. A refusal the unit answered raises DeviceRefused
    with the unit's own errno; a dead link raises OSError, a silent unit
    TimeoutError."""

    def __init__(self, handle: int, bus: Optional[Bus] = None, owned: bool = True,
                 describe: str = ""):
        super().__init__()
        self._handle = handle
        self._bus = bus
        self._owned = owned
        self._describe = describe
        self._last_failure: Optional[str] = None
        self._poll_hz = 0
        self._stream = 0
        # A reader polls the stream from its own thread while the owner may
        # stop or close it: the stream leaves under the lock the poll holds.
        self._stream_lock = threading.Lock()

    # ── opening ───────────────────────────────────────────
    @classmethod
    def open_i2c(cls, bus, address: int = DEFAULT_I2C_ADDRESS, **_ignored) -> "Unit":
        """The unit at `address` on `bus` (a number or a device path); the
        bus handle is the client's own and closes with it."""
        path = _bus_path(bus)
        handle_bus = Bus.open(path)
        err = ctypes.c_int(0)
        handle = _load().nxs_unit_open_i2c(handle_bus._live(), address, ctypes.byref(err))
        if not handle:
            handle_bus.close()
            raise OSError(err.value, os.strerror(err.value), f"unit at {address:#04x} on {path}")
        return cls(handle, bus=handle_bus, describe=f"i2c {path}@{address:#04x}")

    @classmethod
    def on_bus(cls, bus: Bus, address: int) -> "Unit":
        """The unit at `address` on an open bus the caller keeps."""
        err = ctypes.c_int(0)
        handle = _load().nxs_unit_open_i2c(bus._live(), address, ctypes.byref(err))
        if not handle:
            raise OSError(err.value, os.strerror(err.value), f"unit at {address:#04x}")
        return cls(handle, describe=f"i2c @{address:#04x}")

    @classmethod
    def open_can(cls, can_iface: str = "can0", can_mtu: int = DEFAULT_CAN_MTU,
                 local_node_id: int = HOST_NODE_ID, remote_node_id: int = DEFAULT_NODE_ID,
                 **_ignored) -> "Unit":
        err = ctypes.c_int(0)
        handle = _load().nxs_unit_open_can(can_iface.encode(), can_mtu, local_node_id,
                                           remote_node_id, ctypes.byref(err))
        if not handle:
            raise OSError(err.value, os.strerror(err.value), can_iface)
        return cls(handle, describe=f"cyphal-can {can_iface} node {remote_node_id}")

    @classmethod
    def open_serial(cls, port: str, baud: int = DEFAULT_BAUD, local_node_id: int = HOST_NODE_ID,
                    remote_node_id: int = DEFAULT_NODE_ID, **_ignored) -> "Unit":
        err = ctypes.c_int(0)
        handle = _load().nxs_unit_open_serial(port.encode(), baud, local_node_id, remote_node_id,
                                              ctypes.byref(err))
        if not handle:
            raise OSError(err.value, os.strerror(err.value), port)
        return cls(handle, describe=f"cyphal-serial {port} node {remote_node_id}")

    def _live(self) -> int:
        if not self._handle:
            raise RuntimeError("libnxs: the unit is closed")
        return self._handle

    def _check(self, rc: int, what: str = "") -> None:
        if rc == 0:
            return
        if rc == -EREFUSED:
            refusal = _Refusal()
            _load().nxs_unit_last_refusal(self._live(), ctypes.byref(refusal))
            raise DeviceRefused(refusal.device_errno, refusal.reason.decode())
        if self._bus is not None and self._bus._failure is not None:
            raise self._bus._failure
        text = os.strerror(-rc) + (f" ({what})" if what else "")
        if rc == -errno.ETIMEDOUT:
            raise TimeoutError(f"the unit did not answer: {text}")
        raise OSError(-rc, text)

    @property
    def caps(self) -> int:
        return _load().nxs_unit_caps(self._live())

    def describe(self) -> str:
        return self._describe

    # ── liveness ──────────────────────────────────────────
    def probe(self) -> bool:
        rc = _load().nxs_unit_probe(self._live())
        if rc < 0:
            self._last_failure = os.strerror(-rc)
            return False
        self._last_failure = None
        return rc == 1

    def probe_failure_detail(self) -> Optional[str]:
        return self._last_failure

    def link_dropped(self) -> bool:
        return _load().nxs_unit_link_dropped(self._live()) == 1

    def disconnect_message(self) -> str:
        if self._describe.startswith("cyphal-serial"):
            return ("serial link dropped mid-session — the USB device disconnected. "
                    + LINK_DROP_ADVICE)
        return super().disconnect_message()

    def _identity(self) -> _Identity:
        out = _Identity()
        self._check(_load().nxs_unit_read_identity(self._live(), ctypes.byref(out)), "identity")
        return out

    def interface_version(self) -> Optional[int]:
        version = self._identity().proto_version
        return version or None

    def read_serial(self) -> Optional[bytes]:
        try:
            return bytes.fromhex(self._identity().serial.decode())
        except (OSError, TimeoutError, ValueError):
            return None

    def read_fw_version(self) -> Optional[str]:
        try:
            identity = self._identity()
        except (OSError, TimeoutError):
            return None
        describe = identity.fw_describe.decode()
        return describe or f"{identity.fw_major}.{identity.fw_minor}"

    def read_fw_version_pair(self) -> Optional[tuple]:
        try:
            identity = self._identity()
        except (OSError, TimeoutError):
            return None
        return int(identity.fw_major), int(identity.fw_minor)

    def read_fw_confirmed(self) -> Optional[bool]:
        confirmed = self._identity().fw_confirmed
        return None if confirmed < 0 else confirmed != 0

    def identify(self) -> None:
        self._check(_load().nxs_unit_identify(self._live()), "identify")

    # ── the personality VM ─────────────────────────────────────
    def _state(self) -> _State:
        out = _State()
        self._check(_load().nxs_unit_read_state(self._live(), ctypes.byref(out)), "state")
        return out

    def upload_image(self, image: bytes) -> None:
        self._check(_load().nxs_unit_upload(self._live(), bytes(image), len(image)), "upload")

    def vm_run(self) -> None:
        self._check(_load().nxs_unit_vm_run(self._live()), "run")

    def vm_stop(self) -> None:
        self._check(_load().nxs_unit_vm_stop(self._live()), "stop")

    def vm_reset(self) -> None:
        self._check(_load().nxs_unit_vm_reset(self._live(), _RESET_SETTLE_MS), "reset")

    def read_personality_name(self) -> str:
        return self._state().personality_name.decode()

    def read_sample_size(self) -> int:
        return int(self._state().sample_size)

    def read_sample_count(self) -> int:
        return int(self._state().sample_count)

    def read_status(self) -> int:
        return int(self._state().status)

    def read_vm_state(self) -> int:
        return int(self._state().vm_state)

    def read_error_code(self) -> int:
        return int(self._state().error_code)

    def read_store_count(self) -> int:
        return int(self._state().store_count)

    def read_active_slot(self) -> int:
        return int(self._state().active_slot)

    def read_runner_state(self) -> int:
        return int(self._state().runner_state)

    def read_probe_retries(self) -> int:
        return int(self._state().probe_retries)

    def read_descriptor_epoch(self) -> int:
        return int(self._state().descriptor_epoch)

    # ── the store ─────────────────────────────────────────
    def save_slot(self, slot: int) -> None:
        self._check(_load().nxs_unit_save_slot(self._live(), slot), f"save slot {slot}")

    def delete_slot(self, slot: int) -> None:
        self._check(_load().nxs_unit_delete_slot(self._live(), slot), f"delete slot {slot}")

    def clear_store(self) -> None:
        self._check(_load().nxs_unit_clear_store(self._live()), "clear store")

    def cycle(self) -> None:
        self._check(_load().nxs_unit_cycle(self._live()), "cycle")

    def read_slot_info(self, slot: int):
        """The slot's metadata, None when empty; DeviceRefused(EBUSY) while
        a session holds the store."""
        from nxs.image import IMAGE_KIND_NAMES

        info = _SlotInfo()
        rc = _load().nxs_unit_slot_info(self._live(), slot, ctypes.byref(info))
        if rc == -errno.ENOENT:
            return None
        if rc == -errno.EBUSY:
            raise DeviceRefused(errno.EBUSY, "a calibration procedure or firmware push holds "
                                             "the store")
        self._check(rc, f"peek slot {slot}")
        return SimpleNamespace(name=info.name.decode(), num_params=info.num_params,
                               num_outputs=info.num_outputs, i2c_addr=info.i2c_addr,
                               kind=IMAGE_KIND_NAMES.get(info.kind, info.kind))

    def read_personality_info(self, slot: int) -> bytes:
        out = (ctypes.c_uint8 * _TRAILER_MAX)()
        length = ctypes.c_size_t(0)
        self._check(_load().nxs_unit_personality_info(self._live(), slot, out, _TRAILER_MAX,
                                                      ctypes.byref(length)), "personality info")
        return bytes(out[:length.value])

    # ── parameters and outputs ────────────────────────────
    def read_param(self, index: int) -> dict:
        info = _ParamInfo()
        self._check(_load().nxs_unit_param(self._live(), index, ctypes.byref(info)),
                    f"param {index}")
        return _param_dict(index, info)

    def read_capabilities(self) -> List[dict]:
        out: List[dict] = []
        for index in range(255):
            info = _ParamInfo()
            rc = _load().nxs_unit_param(self._live(), index, ctypes.byref(info))
            if rc == -errno.ENOENT:
                break
            self._check(rc, f"param {index}")
            out.append(_param_dict(index, info))
        return out

    def write_param(self, index: int, value: int) -> None:
        self._check(_load().nxs_unit_set_param(self._live(), index, value), f"set param {index}")

    def set_param(self, name: str, value: int) -> None:
        for param in self.read_capabilities():
            if param["name"] == name:
                self.write_param(param["idx"], value)
                return
        raise KeyError(name)

    def read_output(self, index: int) -> dict:
        info = _OutputInfo()
        self._check(_load().nxs_unit_output(self._live(), index, ctypes.byref(info)),
                    f"output {index}")
        return _output_dict(index, info)

    def read_outputs(self) -> Optional[List[dict]]:
        """The output descriptors, bracketed by the descriptor epoch: []
        when the unit serves none, None when the set kept changing."""
        for _attempt in range(3):
            epoch = self.read_descriptor_epoch()
            if epoch == 0:
                return []
            outs: List[dict] = []
            for index in range(255):
                info = _OutputInfo()
                rc = _load().nxs_unit_output(self._live(), index, ctypes.byref(info))
                if rc == -errno.ENOENT:
                    break
                self._check(rc, f"output {index}")
                outs.append(_output_dict(index, info))
            if self.read_descriptor_epoch() == epoch:
                return outs
        return None

    def _descriptor_token(self) -> int:
        return self.read_descriptor_epoch()

    # ── cam personalities ──────────────────────────────
    def cam_stage_params(self, slot: int, values) -> None:
        staged = dict(values)
        if len(staged) > MAX_PARAMS:
            raise ValueError(f"at most {MAX_PARAMS} parameters, got {len(staged)}")
        params = (_Param * max(1, len(staged)))()
        for i, (index, value) in enumerate(staged.items()):
            params[i].index = int(index)
            params[i].value = int(value)
        self._check(_load().nxs_unit_cam_stage(self._live(), slot, params, len(staged)),
                    "camera stage")

    def cam_read_params(self, slot: int, indices) -> Dict[int, int]:
        out: Dict[int, int] = {}
        for index in indices:
            value = ctypes.c_uint32(0)
            self._check(_load().nxs_unit_cam_param(self._live(), slot, int(index),
                                                   ctypes.byref(value)), "camera param")
            out[int(index)] = value.value
        return out

    def cam_run(self, slot: int, timeout_s: Optional[float] = None) -> None:
        """Run the personality in `slot` and wait for its terminal state; a
        verdict other than DONE is the unit's refusal."""
        state = _CamState()
        timeout_ms = int((timeout_s or _CAM_RUN_TIMEOUT_MS / 1000) * 1000)
        self._check(_load().nxs_unit_cam_run(self._live(), slot, timeout_ms, ctypes.byref(state)),
                    "camera run")

    def cam_abort(self, timeout_s: Optional[float] = None) -> None:
        timeout_ms = int((timeout_s or _CAM_ABORT_TIMEOUT_MS / 1000) * 1000)
        self._check(_load().nxs_unit_cam_abort(self._live(), timeout_ms), "camera abort")

    def read_cam_state(self) -> Tuple[int, int]:
        state = _CamState()
        self._check(_load().nxs_unit_cam_state(self._live(), ctypes.byref(state)), "camera state")
        return int(state.state), int(state.error)

    # ── the stream ────────────────────────────────────────
    @staticmethod
    def _subject_index(subject) -> int:
        if subject is None:
            return 0
        if subject not in DECIMATION_SUBJECTS:
            raise ValueError(f"unknown decimation subject {subject!r}")
        return DECIMATION_SUBJECTS.index(subject) + 1

    def read_decimation(self, subject=None) -> int:
        value = ctypes.c_uint16(0)
        self._check(_load().nxs_unit_decimation(self._live(), self._subject_index(subject),
                                                ctypes.byref(value)), "decimation")
        return value.value

    def write_decimation(self, value: int, subject=None) -> None:
        self._check(_load().nxs_unit_set_decimation(self._live(), self._subject_index(subject),
                                                    int(value)), "decimation")

    def read_sample_fifo_depth(self) -> int:
        depth = ctypes.c_uint32(0)
        in_effect = ctypes.c_uint32(0)
        self._check(_load().nxs_unit_sample_fifo_depth(self._live(), ctypes.byref(depth),
                                                       ctypes.byref(in_effect)), "fifo depth")
        return depth.value

    def read_sample_fifo_depth_in_effect(self) -> int:
        depth = ctypes.c_uint32(0)
        in_effect = ctypes.c_uint32(0)
        self._check(_load().nxs_unit_sample_fifo_depth(self._live(), ctypes.byref(depth),
                                                       ctypes.byref(in_effect)), "fifo depth")
        return in_effect.value

    def write_sample_fifo_depth(self, records: int) -> None:
        validate_sample_fifo_depth(records)
        self._check(_load().nxs_unit_set_sample_fifo_depth(self._live(), int(records)),
                    "fifo depth")

    def set_output_rate(self, hz) -> None:
        super().set_output_rate(hz)
        self._poll_hz = int(round(hz))

    def _arm_stream(self, every_nth: int) -> None:
        if self._stream:
            return
        opts = _StreamOpts(poll_hz=self._poll_hz, every_nth=max(1, int(every_nth)))
        err = ctypes.c_int(0)
        stream = _load().nxs_stream_open(self._live(), ctypes.byref(opts), ctypes.byref(err))
        if not stream:
            self._check(-err.value, "stream")
        self._stream = stream

    def _disarm_stream(self) -> None:
        with self._stream_lock:
            if self._stream:
                _load().nxs_stream_close(self._stream)
                self._stream = 0

    def _next_raw(self, timeout: float) -> Optional[Tuple[int, bytes, Optional[int]]]:
        with self._stream_lock:
            if not self._stream:
                self._arm_stream(1)
            sample = _Sample()
            rc = _load().nxs_stream_next(self._stream, int(timeout * 1e9), ctypes.byref(sample))
        if rc == -errno.ETIMEDOUT:
            return None
        self._check(rc, "sample")
        return int(sample.seq), bytes(sample.data[:sample.len]), int(sample.timestamp_us)

    @property
    def lost_samples(self) -> Optional[int]:
        if not self._stream:
            return None
        lost = _load().nxs_stream_lost(self._stream)
        return None if lost < 0 else int(lost)

    # ── time sync ─────────────────────────────────────────
    def read_device_time_us(self) -> Optional[int]:
        value = ctypes.c_uint64(0)
        rc = _load().nxs_unit_time_us(self._live(), ctypes.byref(value))
        if rc in (-errno.ENODATA, -errno.ETIMEDOUT):
            return None
        self._check(rc, "time")
        return value.value

    def time_sync_round(self, pings: int = 8, interval_s: float = 1.0) -> TimeSync:
        """One round on the library's estimator: the pings, the estimate,
        the push, the read-back check."""
        out = _TimeSync()
        self._check(_load().nxs_unit_time_sync(self._live(), int(pings), int(interval_s * 1000),
                                               ctypes.byref(out)), "time sync")
        return TimeSync(out.offset_us, out.bound_us, out.rate_ppb, out.valid_for_us, out.source,
                        bool(out.valid))

    def time_sync_fit(self) -> bytes:
        """The estimator's fit, for a handle opened after a link flap to adopt."""
        out = _TimeSyncFit()
        self._check(_load().nxs_unit_time_sync_fit(self._live(), ctypes.byref(out)), "time sync fit")
        return bytes(out.bytes[:out.len])

    def adopt_time_sync_fit(self, fit: bytes) -> None:
        record = _TimeSyncFit(len=len(fit))
        ctypes.memmove(record.bytes, fit, len(fit))
        self._check(_load().nxs_unit_time_sync_adopt(self._live(), ctypes.byref(record)),
                    "time sync fit")

    def push_time_sync(self, offset_us: int, bound_us: int, rate_ppb: int = 0,
                       valid_for_us: int = 0) -> None:
        record = _TimeSync(offset_us=offset_us, bound_us=bound_us, rate_ppb=rate_ppb,
                           valid_for_us=valid_for_us)
        self._check(_load().nxs_unit_time_sync_push(self._live(), ctypes.byref(record)),
                    "time sync push")

    def read_time_sync(self) -> tuple:
        out = _TimeSync()
        self._check(_load().nxs_unit_time_sync_read(self._live(), ctypes.byref(out)), "time sync")
        return TimeSync(out.offset_us, out.bound_us, out.rate_ppb, out.valid_for_us, out.source,
                        bool(out.valid)).as_tuple()

    # ── calibration ───────────────────────────────────────
    def _calibration(self) -> Tuple[bytes, int]:
        out = (ctypes.c_uint8 * _CALIBRATION_RECORD_MAX)()
        length = ctypes.c_size_t(0)
        epoch = ctypes.c_uint8(0)
        self._check(_load().nxs_unit_calibration(self._live(), out, _CALIBRATION_RECORD_MAX,
                                                 ctypes.byref(length), ctypes.byref(epoch)),
                    "calibration")
        return bytes(out[:length.value]), epoch.value

    def read_calibration(self) -> CalibrationRecord:
        record, _epoch = self._calibration()
        return CalibrationRecord.unpack(record)

    def read_cal_epoch(self) -> int:
        return self._calibration()[1]

    def write_calibration(self, record: CalibrationRecord, persist: bool = True) -> None:
        packed = record.pack()
        buf = (ctypes.c_uint8 * len(packed))(*packed)
        self._check(_load().nxs_unit_set_calibration(self._live(), buf, len(packed),
                                                     1 if persist else 0), "calibration")

    def set_orientation(self, rotation: int, persist: bool = True) -> None:
        record = dataclasses.replace(self.read_calibration(), orientation=rotation)
        self.write_calibration(record, persist)

    def cal_gyro(self) -> None:
        self._check(_load().nxs_unit_cal_gyro(self._live()), "gyro calibration")

    def cal_mag_start(self) -> None:
        self._check(_load().nxs_unit_cal_mag_start(self._live()), "magnetometer calibration")

    def cal_mag_stop(self) -> None:
        self._check(_load().nxs_unit_cal_mag_stop(self._live()), "magnetometer calibration")

    def cal_abort(self) -> None:
        self._check(_load().nxs_unit_cal_abort(self._live()), "calibration abort")

    def read_cal_progress(self) -> Tuple[int, int, int]:
        out = _CalProgress()
        self._check(_load().nxs_unit_cal_progress(self._live(), ctypes.byref(out)),
                    "calibration progress")
        return int(out.state), int(out.coverage), int(out.result)

    def save_calibration(self) -> None:
        self._check(_load().nxs_unit_cal_save(self._live()), "calibration save")

    # ── commissioning ─────────────────────────────────────
    def _commission_record(self) -> _Commission:
        out = _Commission()
        self._check(_load().nxs_unit_commission_read(self._live(), ctypes.byref(out)),
                    "identity record")
        return out

    def read_identity(self) -> dict:
        record = self._commission_record()
        return {"node_addr": int(record.node_id),
                "topics": {name: int(record.topic_ids[i]) for i, name in enumerate(COMMISSION_TOPICS)}}

    def commission(self, node_addr=None, topics=None, can_bitrate=None, can_term=None) -> None:
        validate_commission(node_addr, topics)
        if can_bitrate is not None:
            validate_can_bitrate(*can_bitrate)
        if can_term is not None:
            validate_can_term(can_term)
        # What is not given keeps the unit's value: the record persists whole.
        record = self._commission_record()
        if node_addr is not None:
            record.node_id = int(node_addr)
        for i, name in enumerate(COMMISSION_TOPICS):
            if topics and name in topics:
                record.topic_ids[i] = int(topics[name])
        if can_bitrate is not None:
            record.can_bitrate, record.can_data_bitrate = int(can_bitrate[0]), int(can_bitrate[1])
        record.can_term = 0xFF if can_term is None else (0 if can_term == 0xFFFF else int(can_term))
        self._check(_load().nxs_unit_commission(self._live(), ctypes.byref(record)), "commission")

    def read_can_term(self) -> int:
        on = ctypes.c_uint8(0)
        self._check(_load().nxs_unit_can_term(self._live(), ctypes.byref(on)), "can term")
        return int(on.value)

    def write_can_term(self, value: int) -> None:
        validate_can_term(value)
        self._check(_load().nxs_unit_set_can_term(self._live(), 0 if value == 0xFFFF else int(value)),
                    "can term")

    def read_can_bitrate(self) -> tuple:
        record = self._commission_record()
        if record.can_bitrate == 0:
            from nxs._generated_constants import CyphalDefaults
            return CyphalDefaults.CAN_BITRATE_DEFAULT, CyphalDefaults.CAN_BITRATE_DATA_DEFAULT
        return int(record.can_bitrate), int(record.can_data_bitrate)

    def write_can_bitrate(self, nominal: int, data: int) -> None:
        self.commission(can_bitrate=(nominal, data))

    # ── firmware ──────────────────────────────────────────
    def push_image(self, bin_path: str, chunk_size: int = 32, progress_cb=None) -> int:
        with open(bin_path, "rb") as fh:
            data = fh.read()

        def c_progress(_ctx, done, total):
            if progress_cb is not None:
                progress_cb(int(done), int(total))

        cb = _PROGRESS_FN(c_progress)
        self._check(_load().nxs_unit_push_firmware(self._live(), data, len(data), cb, None),
                    "firmware push")
        return len(data)

    def confirm_fw(self) -> None:
        self._check(_load().nxs_unit_confirm_firmware(self._live()), "confirm firmware")

    def read_xfer_state(self) -> Tuple[int, int, int]:
        out = _XferState()
        self._check(_load().nxs_unit_xfer_state(self._live(), ctypes.byref(out)), "transfer state")
        return int(out.phase), int(out.type), int(out.error)

    def xfer_abort(self) -> None:
        self._check(_load().nxs_unit_xfer_abort(self._live()), "transfer abort")

    def recover(self) -> int:
        self._check(_load().nxs_unit_recover(self._live()), "recover")
        return 0

    def reboot(self) -> None:
        self._check(_load().nxs_unit_reboot(self._live()), "reboot")

    # ── diagnostics ───────────────────────────────────────
    def _diag(self) -> _Diag:
        out = _Diag()
        self._check(_load().nxs_unit_read_diag(self._live(), ctypes.byref(out)), "diagnostics")
        return out

    def read_io_err_count(self) -> int:
        return int(self._diag().vm_io_errors)

    def read_probe_failed_count(self) -> int:
        return int(self._diag().probe_failures)

    def read_drdy_coalesced_count(self) -> int:
        return int(self._diag().drdy_coalesced)

    def read_ingress_reject_count(self) -> int:
        return int(self._diag().ingress_rejects)

    def read_cmd_queue_overflow_count(self) -> int:
        return int(self._diag().cmd_queue_overflows)

    # ── lifetime ──────────────────────────────────────────
    def close(self) -> None:
        """Close the unit; one the walk handed over is the walk's to close."""
        self._disarm_stream()
        if self._handle and self._owned:
            _load().nxs_unit_close(self._handle)
        self._handle = 0
        if self._bus is not None:
            self._bus.close()
            self._bus = None

    def __enter__(self) -> "Unit":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()


class I2cUnit(Unit, SupportsCameraRun):
    """The unit over I²C: the register map, which also runs cam personalities."""


class CyphalUnit(Unit, SupportsEgressDecimation):
    """The unit over Cyphal: its samples are pushed under the decimation gate."""
