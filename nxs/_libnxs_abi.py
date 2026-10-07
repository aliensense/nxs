# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The unit client's slice of the C ABI (`nxs.h`): the structs the unit
functions fill and their signatures, registered with the library loader."""

from __future__ import annotations

import ctypes

from nxs._libnxs import _CamState, _Param, _SlotInfo

_NAME_LEN = 32
_UNIT_LEN = 16
_SERIAL_LEN = 32
_VERSION_LEN = 64
_MAX_PARAM_VALUES = 16
_SAMPLE_DATA_MAX = 128


class _Identity(ctypes.Structure):
    _fields_ = [("serial", ctypes.c_char * _SERIAL_LEN), ("fw_major", ctypes.c_uint8),
                ("fw_minor", ctypes.c_uint8), ("fw_describe", ctypes.c_char * _VERSION_LEN),
                ("fw_confirmed", ctypes.c_int8), ("proto_version", ctypes.c_uint8)]


class _State(ctypes.Structure):
    _fields_ = [("status", ctypes.c_uint8), ("vm_state", ctypes.c_uint8),
                ("error_code", ctypes.c_uint8), ("runner_state", ctypes.c_uint8),
                ("probe_retries", ctypes.c_uint8), ("store_count", ctypes.c_uint8),
                ("active_slot", ctypes.c_uint8), ("sample_size", ctypes.c_uint8),
                ("sample_count", ctypes.c_uint16), ("descriptor_epoch", ctypes.c_uint16),
                ("personality_name", ctypes.c_char * _NAME_LEN)]


class _ParamInfo(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char * _NAME_LEN), ("param_type", ctypes.c_uint8),
                ("reload", ctypes.c_uint8), ("default_value", ctypes.c_uint32),
                ("current", ctypes.c_uint32), ("num_values", ctypes.c_uint8),
                ("values", ctypes.c_uint32 * _MAX_PARAM_VALUES), ("unit", ctypes.c_char * _UNIT_LEN)]


class _OutputInfo(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char * _NAME_LEN), ("field_type", ctypes.c_uint8),
                ("byte_order", ctypes.c_uint8), ("semantic", ctypes.c_uint8),
                ("count", ctypes.c_uint16), ("byte_offset", ctypes.c_uint16),
                ("scale", ctypes.c_double), ("offset", ctypes.c_double),
                ("unit", ctypes.c_char * _UNIT_LEN)]


class _StreamOpts(ctypes.Structure):
    _fields_ = [("poll_hz", ctypes.c_uint32), ("every_nth", ctypes.c_uint16)]


class _Sample(ctypes.Structure):
    _fields_ = [("timestamp_us", ctypes.c_uint64), ("host_ns", ctypes.c_uint64),
                ("seq", ctypes.c_uint16), ("len", ctypes.c_uint8),
                ("data", ctypes.c_uint8 * _SAMPLE_DATA_MAX)]


class _TimeSyncFit(ctypes.Structure):
    _fields_ = [("len", ctypes.c_uint32), ("bytes", ctypes.c_uint8 * 65536)]


class _TimeSync(ctypes.Structure):
    _fields_ = [("offset_us", ctypes.c_int64), ("bound_us", ctypes.c_uint32),
                ("rate_ppb", ctypes.c_int32), ("valid_for_us", ctypes.c_uint32),
                ("source", ctypes.c_uint8), ("valid", ctypes.c_uint8)]


class _CalProgress(ctypes.Structure):
    _fields_ = [("state", ctypes.c_uint8), ("coverage", ctypes.c_uint8), ("result", ctypes.c_uint8)]


class _Commission(ctypes.Structure):
    _fields_ = [("node_id", ctypes.c_uint16), ("topic_ids", ctypes.c_uint16 * 9),
                ("can_bitrate", ctypes.c_uint32), ("can_data_bitrate", ctypes.c_uint32),
                ("can_term", ctypes.c_uint8)]


class _XferState(ctypes.Structure):
    _fields_ = [("phase", ctypes.c_uint8), ("type", ctypes.c_uint8), ("error", ctypes.c_uint8)]


class _Diag(ctypes.Structure):
    _fields_ = [("drdy_coalesced", ctypes.c_uint32), ("ingress_rejects", ctypes.c_uint32),
                ("cmd_queue_overflows", ctypes.c_uint32), ("vm_io_errors", ctypes.c_uint32),
                ("probe_failures", ctypes.c_uint32)]



_PROGRESS_FN = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t)


def _declare_unit(lib: ctypes.CDLL) -> None:
    u8p = ctypes.POINTER(ctypes.c_uint8)
    unit = ctypes.c_void_p
    lib.nxs_unit_open_can.restype = ctypes.c_void_p
    lib.nxs_unit_open_can.argtypes = [ctypes.c_char_p, ctypes.c_uint8, ctypes.c_uint8,
                                      ctypes.c_uint8, ctypes.POINTER(ctypes.c_int)]
    lib.nxs_unit_open_serial.restype = ctypes.c_void_p
    lib.nxs_unit_open_serial.argtypes = [ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint8,
                                         ctypes.c_uint8, ctypes.POINTER(ctypes.c_int)]
    lib.nxs_unit_caps.restype = ctypes.c_uint32
    lib.nxs_unit_caps.argtypes = [unit]
    lib.nxs_stream_open.restype = ctypes.c_void_p
    lib.nxs_stream_open.argtypes = [unit, ctypes.POINTER(_StreamOpts),
                                    ctypes.POINTER(ctypes.c_int)]
    lib.nxs_stream_next.restype = ctypes.c_int
    lib.nxs_stream_next.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.POINTER(_Sample)]
    lib.nxs_stream_lost.restype = ctypes.c_int64
    lib.nxs_stream_lost.argtypes = [ctypes.c_void_p]
    lib.nxs_stream_close.restype = None
    lib.nxs_stream_close.argtypes = [ctypes.c_void_p]
    for name, args in (("nxs_unit_link_dropped", [unit]),
                       ("nxs_unit_probe", [unit]),
                       ("nxs_unit_read_identity", [unit, ctypes.POINTER(_Identity)]),
                       ("nxs_unit_identify", [unit]),
                       ("nxs_unit_read_state", [unit, ctypes.POINTER(_State)]),
                       ("nxs_unit_vm_run", [unit]),
                       ("nxs_unit_vm_stop", [unit]),
                       ("nxs_unit_vm_reset", [unit, ctypes.c_uint32]),
                       ("nxs_unit_clear_store", [unit]),
                       ("nxs_unit_cycle", [unit]),
                       ("nxs_unit_param", [unit, ctypes.c_uint8, ctypes.POINTER(_ParamInfo)]),
                       ("nxs_unit_output", [unit, ctypes.c_uint8, ctypes.POINTER(_OutputInfo)]),
                       ("nxs_unit_set_param", [unit, ctypes.c_uint8, ctypes.c_uint32]),
                       ("nxs_unit_decimation", [unit, ctypes.c_uint8,
                                                ctypes.POINTER(ctypes.c_uint16)]),
                       ("nxs_unit_set_decimation", [unit, ctypes.c_uint8, ctypes.c_uint16]),
                       ("nxs_unit_sample_fifo_depth", [unit, ctypes.POINTER(ctypes.c_uint32),
                                                       ctypes.POINTER(ctypes.c_uint32)]),
                       ("nxs_unit_set_sample_fifo_depth", [unit, ctypes.c_uint32]),
                       ("nxs_unit_time_sync", [unit, ctypes.c_uint8, ctypes.c_uint32,
                                               ctypes.POINTER(_TimeSync)]),
                       ("nxs_unit_time_sync_push", [unit, ctypes.POINTER(_TimeSync)]),
                       ("nxs_unit_time_sync_read", [unit, ctypes.POINTER(_TimeSync)]),
                       ("nxs_unit_time_us", [unit, ctypes.POINTER(ctypes.c_uint64)]),
                       ("nxs_unit_time_sync_fit", [unit, ctypes.POINTER(_TimeSyncFit)]),
                       ("nxs_unit_time_sync_adopt", [unit, ctypes.POINTER(_TimeSyncFit)]),
                       ("nxs_unit_calibration", [unit, u8p, ctypes.c_size_t,
                                                 ctypes.POINTER(ctypes.c_size_t),
                                                 ctypes.POINTER(ctypes.c_uint8)]),
                       ("nxs_unit_set_calibration", [unit, u8p, ctypes.c_size_t, ctypes.c_int]),
                       ("nxs_unit_cal_gyro", [unit]),
                       ("nxs_unit_cal_mag_start", [unit]),
                       ("nxs_unit_cal_mag_stop", [unit]),
                       ("nxs_unit_cal_abort", [unit]),
                       ("nxs_unit_cal_progress", [unit, ctypes.POINTER(_CalProgress)]),
                       ("nxs_unit_cal_save", [unit]),
                       ("nxs_unit_commission_read", [unit, ctypes.POINTER(_Commission)]),
                       ("nxs_unit_commission", [unit, ctypes.POINTER(_Commission)]),
                       ("nxs_unit_can_term", [unit, ctypes.POINTER(ctypes.c_uint8)]),
                       ("nxs_unit_set_can_term", [unit, ctypes.c_uint8]),
                       ("nxs_unit_push_firmware", [unit, ctypes.c_char_p, ctypes.c_size_t,
                                                   _PROGRESS_FN, ctypes.c_void_p]),
                       ("nxs_unit_confirm_firmware", [unit]),
                       ("nxs_unit_xfer_state", [unit, ctypes.POINTER(_XferState)]),
                       ("nxs_unit_xfer_abort", [unit]),
                       ("nxs_unit_recover", [unit]),
                       ("nxs_unit_reboot", [unit]),
                       ("nxs_unit_read_diag", [unit, ctypes.POINTER(_Diag)]),
                       ("nxs_unit_read_cam_runs", [unit, ctypes.POINTER(ctypes.c_uint32)]),
                       ("nxs_unit_upload", [unit, ctypes.c_char_p, ctypes.c_size_t]),
                       ("nxs_unit_save_slot", [unit, ctypes.c_uint8]),
                       ("nxs_unit_delete_slot", [unit, ctypes.c_uint8]),
                       ("nxs_unit_slot_info", [unit, ctypes.c_uint8, ctypes.POINTER(_SlotInfo)]),
                       ("nxs_unit_personality_info", [unit, ctypes.c_uint8, u8p, ctypes.c_size_t,
                                                      ctypes.POINTER(ctypes.c_size_t)]),
                       ("nxs_unit_cam_stage", [unit, ctypes.c_uint8, ctypes.POINTER(_Param),
                                               ctypes.c_size_t]),
                       ("nxs_unit_cam_param", [unit, ctypes.c_uint8, ctypes.c_uint8,
                                               ctypes.POINTER(ctypes.c_uint32)]),
                       ("nxs_unit_cam_run", [unit, ctypes.c_uint8, ctypes.c_uint32,
                                             ctypes.POINTER(_CamState)]),
                       ("nxs_unit_cam_abort", [unit, ctypes.c_uint32]),
                       ("nxs_unit_cam_state", [unit, ctypes.POINTER(_CamState)])):
        fn = getattr(lib, name)
        fn.restype = ctypes.c_int
        fn.argtypes = args


from nxs import _libnxs as _base  # noqa: E402

_base._DECLARERS.append(_declare_unit)
if _base._lib is not None:
    _declare_unit(_base._lib)


