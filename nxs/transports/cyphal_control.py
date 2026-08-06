"""Full NXS control + streaming over Cyphal, as an `NxsClient`.

Implements the same `NxsClient` contract as the I2C and serial transports, so
`nxs --transport cyphal-serial <verb>` is the I2C/serial verb on the
Cyphal wire. The Cyphal side wraps stock pycyphal — `make_node` + the stock
`FileServer` for provisioning, `ExecuteCommand` for control/store, and
`register.Access` for params — never a re-implemented Cyphal service; the
firmware's vendor services (GetParamInfo / GetDriverInfo / GetOutputInfo) and
the Status subject carry the rest.

Transport-agnostic by construction: only the `make_node(transport=…)` argument
differs between Cyphal/serial and Cyphal/CAN; everything downstream is identical.

pycyphal is asyncio-based, so the node runs on a private event loop in a daemon
thread (the cyphal_source pattern): subscriber callbacks feed thread-safe state,
and the RPCs cross over via `run_coroutine_threadsafe`.
"""

import errno
import os
import queue
import tempfile
import threading
import time
from types import SimpleNamespace
from typing import List, Optional, Tuple

from nxs._generated_constants import CyphalDefaults
from nxs.client import (
    COMMISSION_ERR_REASON,
    COMMISSION_TOPICS,
    DeviceRefused,
    NxsClient,
    PULL_ERR_REASON,
    Sample,
    SupportsBitTiming,
    SupportsCanTermination,
    SupportsCommissioning,
    SupportsEgressDecimation,
    SupportsIdentify,
    SupportsRecovery,
    SupportsFaultCounters,
    SupportsSlotPeek,
    SupportsTimeSync,
    err_reason,
    validate_can_bitrate,
    validate_can_term,
    validate_commission,
)
from nxs.transports.cyphal_source import (
    DECIMATION_REGISTER,
    TIME_US_REGISTER,
    _ensure_dsdl,
)
from nxs.descriptor import parse_sample
from nxs.image import FIELD_TYPE_NAMES
from nxs.serial_util import open_serial_once

# Vendor ExecuteCommand codes — mirror the firmware command enum
# (the firmware command enum) and the Interface Description §8.3. Keep in sync.
COMMAND_BEGIN_SOFTWARE_UPDATE = 65533  # stock
COMMAND_STORE_PERSISTENT_STATES = 65530  # stock — commit staged config

# Stock register carrying the persisted CAN bit-timing profile as a
# natural32[2] {arbitration, data} pair in bit/s.
BITRATE_REGISTER = "uavcan.can.bitrate"

# Vendor register carrying the CAN split-termination selection (natural16:
# 0 = off, 1 = on, 0xFFFF = revert to the default, off). Live-applied.
CAN_TERM_REGISTER = "aliensense.nxs.can_term"

# Vendor register carrying the time discipline as one atomic integer64
# push: write {offset µs, bound µs, rate ppb}, read {offset, bound,
# rate, source}.
TIME_SYNC_REGISTER = "aliensense.nxs.time_sync"
LOAD_FROM_FILE = 0xA000
RUN = 0xA001
STOP = 0xA002
SAVE = 0xA003
DELETE_SLOT = 0xA004
CLEAR_STORE = 0xA005
CYCLE = 0xA006
RESET = 0xA007
ENTER_RECOVERY = 0xA008
IDENTIFY = 0xA009

PARAM_REGISTER_PREFIX = "aliensense.nxs.param."
ACTIVE_SLOT = 0xFF
MODE_SOFTWARE_UPDATE = 3  # uavcan.node.Mode.SOFTWARE_UPDATE

# Shared tail of every dropped-serial-link diagnosis (mid-session in
# `disconnect_message`, during open in the CLI construction guard).
LINK_DROP_ADVICE = ("A J-Link VCOM can wedge on open or drop under load; prefer a "
                    "dedicated USB-UART adapter (CP210x / FT232) for the host link.")


# ── Background-log quieting ───────────────────────────────
# When the serial link drops mid-session pycyphal emits a cascade — the reader
# thread's SerialException, then a ResourceClosedError per publisher tick — but
# it *catches and logs* them via its own `logging` loggers (pycyphal.transport.
# serial._serial, .application.heartbeat_publisher, .application._port_list_
# publisher), so no threading/asyncio hook ever sees them. The only lever is the
# logger level: a CLI wants its own one-line diagnosis, not the library's
# stack traces, so we cap the whole `pycyphal` tree at CRITICAL once a Cyphal
# client starts. The drop itself is detected structurally (the port vanishes
# from /dev — see `link_dropped`), not from the suppressed logs.
_BG_QUIETED = False


def _quiet_background_logging() -> None:
    global _BG_QUIETED
    if _BG_QUIETED:
        return
    import logging
    logging.getLogger("pycyphal").setLevel(logging.CRITICAL)
    _BG_QUIETED = True


class CyphalControlClient(NxsClient, SupportsFaultCounters, SupportsSlotPeek, SupportsCommissioning,
                          SupportsIdentify, SupportsRecovery,
                          SupportsEgressDecimation, SupportsBitTiming,
                          SupportsCanTermination, SupportsTimeSync):
    """The full NXS control contract over Cyphal, wrapping stock pycyphal."""

    def __init__(self, port: str = None, baud: int = 460800,
                 can_iface: str = None, can_mtu: int = 64,
                 local_node_id: int = CyphalDefaults.HOST_NODE_ID,
                 remote_node_id: int = CyphalDefaults.DEFAULT_NODE_ID,
                 sample_subject_id: int = CyphalDefaults.SAMPLE_SUBJECT_ID,
                 timeout: float = 2.0, autostart: bool = True):
        super().__init__()
        self._port = port
        self._baud = baud
        self._can_iface = can_iface
        self._can_mtu = can_mtu
        self._local_id = local_node_id
        self._remote_id = remote_node_id
        self._sample_subject_id = sample_subject_id
        self._timeout = timeout

        self._serve_dir = tempfile.mkdtemp(prefix="nxs-cyphal-")
        self._queue: "queue.Queue[Tuple[int, bytes, Optional[int]]]" = queue.Queue(maxsize=2000)
        self._hb_mode: Optional[int] = None
        self._hb_uptime: int = 0

        self._loop = None
        self._thread = None
        self._node = None
        self._fileserver = None
        self._cmd = self._reg = None
        self._param_info = self._driver_info = self._output_info = None
        self._node_info_client = None
        self._sample_sub = self._hb_sub = None

        if autostart:
            self._start()

    # ── pycyphal lifecycle ────────────────────────────────
    def _start(self) -> None:
        import asyncio

        _ensure_dsdl()
        _quiet_background_logging()  # cap pycyphal's disconnect-cascade logging
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True,
                                        name="cyphal-control-loop")
        self._thread.start()
        asyncio.run_coroutine_threadsafe(self._async_setup(), self._loop).result(timeout=10)

    async def _async_setup(self) -> None:
        import pycyphal.application
        from pycyphal.application.file import FileServer
        import uavcan.node.GetInfo_1_0 as GetInfo
        import uavcan.node.ExecuteCommand_1_1 as ExecuteCommand
        import uavcan.node.Heartbeat_1_0 as Heartbeat
        import uavcan.register.Access_1_0 as Access
        import aliensense.nxs.GetParamInfo_1_0 as GetParamInfo
        import aliensense.nxs.GetDriverInfo_1_0 as GetDriverInfo
        import aliensense.nxs.GetOutputInfo_1_0 as GetOutputInfo
        import aliensense.nxs.RawSample_0_1 as RawSample

        info = GetInfo.Response(name="org.aliensense.nxs")
        self._node = pycyphal.application.make_node(info, transport=self._make_transport())

        class _ProgressFileServer(FileServer):
            """Records the furthest byte the client has read so push-fw can drive
            a progress bar off the device's actual pull."""
            served = 0

            async def _serve_rd(self, request, meta):
                resp = await super()._serve_rd(request, meta)
                # Track the furthest read offset (the device reads sequentially).
                # Keyed off the request, not the response payload, whose access
                # varies by pycyphal version.
                try:
                    self.served = max(self.served,
                                      int(request.offset) + self._data_transfer_capacity)
                except Exception:
                    pass
                return resp

        self._fileserver = _ProgressFileServer(self._node, [self._serve_dir])
        self._node.start()

        self._cmd = self._node.make_client(ExecuteCommand, self._remote_id)
        self._reg = self._node.make_client(Access, self._remote_id)
        self._param_info = self._node.make_client(GetParamInfo, self._remote_id)
        self._driver_info = self._node.make_client(GetDriverInfo, self._remote_id)
        self._output_info = self._node.make_client(GetOutputInfo, self._remote_id)
        self._node_info_client = self._node.make_client(GetInfo, self._remote_id)

        # Resolve the sample subject from the device: commissioning can move it
        # off the compiled default, and a subscriber pinned to the default
        # would stream nothing from a re-addressed node. The register reports
        # the staged value, which equals the publishing subject once the
        # commission's reboot has run; an unreachable device (or a value of 0,
        # the disabled sentinel) keeps the default.
        resolved = await self._resolve_sample_subject(Access)
        if resolved is not None and 0 < resolved <= 8191:
            self._sample_subject_id = resolved

        self._sample_sub = self._node.make_subscriber(RawSample, self._sample_subject_id)
        self._sample_sub.receive_in_background(self._on_sample)
        self._hb_sub = self._node.make_subscriber(Heartbeat)
        self._hb_sub.receive_in_background(self._on_heartbeat)

    async def _resolve_sample_subject(self, access_type):
        """Read ``uavcan.pub.sample.id`` from the device, or None when the
        read times out or the register is not a natural16 (older firmware)."""
        import uavcan.register.Name_1_0 as Name
        try:
            result = await self._reg.call(access_type.Request(
                name=Name("uavcan.pub.sample.id")))
        except Exception:
            return None
        if result is None:
            return None
        resp, _transfer = result
        nat = resp.value.natural16
        if nat is None or len(nat.value) == 0:
            return None
        return int(nat.value[0])

    def _make_transport(self):
        """Serial when `port` is set, CAN-FD (SocketCAN) when `can_iface` is —
        the one thing that differs between the Cyphal/serial and Cyphal/CAN
        clients; everything downstream is identical."""
        if self._can_iface:
            from pycyphal.transport.can import CANTransport
            from pycyphal.transport.can.media.socketcan import SocketCANMedia
            return CANTransport(SocketCANMedia(self._can_iface, self._can_mtu), self._local_id)
        from pycyphal.transport.serial import SerialTransport
        # Pre-opened at the target baud with the config frozen: any post-open
        # reprogramming (pycyphal's baudrate/timeout assignments) wedges a
        # J-Link V9 VCOM (see open_serial_once).
        return SerialTransport(open_serial_once(self._port, self._baud), self._local_id)

    async def _on_sample(self, msg, _meta) -> None:
        try:
            self._queue.put_nowait((int(msg.seq), bytes(msg.data), int(msg.timestamp_us)))
        except queue.Full:
            pass

    async def _on_heartbeat(self, msg, _meta) -> None:
        self._hb_mode = int(msg.mode.value)
        self._hb_uptime = int(msg.uptime)

    def _call(self, client, request):
        import asyncio

        if self._loop is None or client is None:
            return None
        fut = asyncio.run_coroutine_threadsafe(client.call(request), self._loop)
        try:
            result = fut.result(timeout=self._timeout)
        except Exception:
            return None
        if result is None:
            return None
        response, _meta = result
        return response

    def link_dropped(self) -> bool:
        # A wedged/unplugged serial device vanishes from /dev; a device that is
        # merely silent keeps its port. That distinguishes "link dropped" from
        # "no answer", without depending on pycyphal's (now suppressed) logs.
        if self._can_iface or not self._port:
            return False
        return not os.path.exists(self._port)

    def disconnect_message(self) -> str:
        return ("serial link dropped mid-session — the USB device disconnected. "
                + LINK_DROP_ADVICE)

    def close(self):
        if self._loop is None:
            return
        import asyncio

        async def _shutdown():
            if self._node is not None:
                self._node.close()
            # pycyphal's close() only *initiates* cancellation of its
            # background receive tasks; the cancellations unwind on the
            # next loop iterations. Stop the loop before that and the
            # interpreter destroys them mid-flight ("Task was destroyed
            # but it is pending" spew after every CAN scan). Yield once,
            # then cancel-and-await whatever remains.
            await asyncio.sleep(0)
            pending = [t for t in asyncio.all_tasks()
                       if t is not asyncio.current_task()]
            for t in pending:
                t.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
        try:
            asyncio.run_coroutine_threadsafe(_shutdown(), self._loop).result(timeout=2)
        except Exception:
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._thread is None or not self._thread.is_alive():
            # Every task is done or cancelled by now, so the loop can close
            # for real — a merely-stopped loop is what produced the "Event
            # loop is closed" tracebacks from late-collected coroutines.
            try:
                self._loop.close()
            except Exception:
                pass
        # A thread that survived the join is still driving the loop —
        # closing it underneath would race; drop the reference instead.
        self._loop = None
        try:
            import shutil
            shutil.rmtree(self._serve_dir, ignore_errors=True)
        except Exception:
            pass

    # ── ExecuteCommand + completion ───────────────────────
    def _execute(self, command: int, parameter: bytes = b"") -> bool:
        """True on STATUS_SUCCESS, False on a failure status, and raises
        RuntimeError when the device never answered — a refusal and a
        dead link are different diagnoses."""
        import uavcan.node.ExecuteCommand_1_1 as ExecuteCommand

        resp = self._call(self._cmd, ExecuteCommand.Request(command=command,
                                                            parameter=list(parameter)))
        if resp is None:
            raise RuntimeError("device did not respond")
        return int(resp.status) == ExecuteCommand.Response.STATUS_SUCCESS

    def _raise_cmd_error(self, reasons=None):
        """A vendor command returned STATUS_FAILURE — fetch the reason.

        The device mirrors the last command's positive errno in the
        read-only `aliensense.nxs.cmd_error` register (the Cyphal twin
        of the I2C CMD_ERROR register). Raises DeviceRefused carrying
        it; falls back to a generic message if the register read fails.
        """
        try:
            code = self._read_natural16("aliensense.nxs.cmd_error")
        except RuntimeError:
            raise RuntimeError("device refused the command "
                               "(reason unavailable)") from None
        raise DeviceRefused(code, err_reason(code, reasons))

    def _wait_idle(self, timeout: float = 15.0) -> None:
        """Wait for a provisioning session to finish: the node's Heartbeat mode
        is SOFTWARE_UPDATE during the pull and returns to OPERATIONAL after. A
        pull shorter than the 1 Hz Heartbeat is never observed active, so a 2 s
        settle without seeing it is treated as already done."""
        end = time.monotonic() + timeout
        settle = time.monotonic() + 2.0
        saw_active = False
        while time.monotonic() < end:
            mode = self._hb_mode
            if mode == MODE_SOFTWARE_UPDATE:
                saw_active = True
            elif saw_active:
                return
            elif time.monotonic() > settle:
                return
            time.sleep(0.1)

    def _wait_for_update(self, timeout: float, total: int = 0, progress_cb=None) -> bool:
        """Hold this node + FileServer up while the device pulls the firmware
        and swaps. Returns True once the device reboots into the new image — the
        Heartbeat uptime resets after we have seen SOFTWARE_UPDATE — and False on
        timeout or a pull that drops back to OPERATIONAL without rebooting (an
        abort). Reports pull progress from the FileServer's served-byte high
        water mark."""
        end = time.monotonic() + timeout
        saw_active = False
        last_uptime = self._hb_uptime
        while time.monotonic() < end:
            if progress_cb is not None and total:
                progress_cb(min(self._fileserver.served, total), total)
            mode = self._hb_mode
            uptime = self._hb_uptime
            if mode == MODE_SOFTWARE_UPDATE:
                saw_active = True
            elif saw_active:
                # Left SOFTWARE_UPDATE: a reset uptime means the swap took; a
                # flip back without a reboot means the pull aborted.
                return uptime < last_uptime
            last_uptime = uptime
            time.sleep(0.1)
        return False

    def _serve(self, name: str, data: bytes) -> None:
        with open(os.path.join(self._serve_dir, name), "wb") as fh:
            fh.write(data)

    # ── Liveness ──────────────────────────────────────────
    def probe(self) -> bool:
        return self._info() is not None

    def read_serial(self) -> Optional[bytes]:
        """UID96 via `uavcan.node.GetInfo` — unique_id[0:12], the tail is
        zero-padding (CommThread fills 12 bytes from hwinfo)."""
        info = self._node_info()
        return bytes(info.unique_id[:12]) if info is not None else None

    def read_fw_version(self) -> Optional[str]:
        """`software_version` via `uavcan.node.GetInfo`, as "MAJOR.MINOR"
        (the wire carries no patch level)."""
        info = self._node_info()
        if info is None:
            return None
        version = info.software_version
        return f"{int(version.major)}.{int(version.minor)}"

    def _node_info(self):
        import uavcan.node.GetInfo_1_0 as GetInfo

        return self._call(self._node_info_client, GetInfo.Request())

    # ── Driver lifecycle ──────────────────────────────────
    def upload_image(self, image: bytes):
        self._serve("driver.nxs", image)
        if not self._execute(LOAD_FROM_FILE, b"driver.nxs"):
            self._raise_cmd_error(PULL_ERR_REASON)
        self._wait_idle()

    def vm_run(self):
        self._execute(RUN)

    def vm_stop(self):
        self._execute(STOP)

    def vm_reset(self):
        self._execute(RESET)

    def recover(self):
        """Reboot the device into MCUboot serial recovery (mcumgr/SMP over
        UART). The node acks, then resets — it does not return over Cyphal."""
        self._execute(ENTER_RECOVERY)

    def push_image(self, bin_path: str, chunk_size: int = 32, progress_cb=None) -> int:
        with open(bin_path, "rb") as fh:
            data = fh.read()
        name = os.path.basename(bin_path)
        self._serve(name, data)
        self._fileserver.served = 0
        if not self._execute(COMMAND_BEGIN_SOFTWARE_UPDATE, name.encode()):
            self._raise_cmd_error(PULL_ERR_REASON)
        # The device pulls the image from our FileServer asynchronously — tens
        # of seconds for a full firmware over the serial link — then swaps and
        # reboots. Hold this node + FileServer up for the whole pull (the 15 s
        # _wait_idle default cuts a 164 KB pull off mid-transfer) and report only
        # what actually happened: success is the device rebooting into the new
        # image, not the BEGIN_SOFTWARE_UPDATE ack.
        if not self._wait_for_update(max(60.0, len(data) / 2000.0),
                                     total=len(data), progress_cb=progress_cb):
            raise RuntimeError(
                "device did not reboot into the new image — the pull timed out "
                "or aborted (check the device log)")
        if progress_cb is not None:
            progress_cb(len(data), len(data))
        return len(data)

    # ── Parameters ────────────────────────────────────────
    def read_capabilities(self) -> List[dict]:
        import aliensense.nxs.GetParamInfo_1_0 as GetParamInfo

        out: List[dict] = []
        index = 0
        while True:
            resp = self._call(self._param_info, GetParamInfo.Request(index=index))
            if resp is None or index >= int(resp.num_params):
                break
            out.append({
                "idx": index,
                "name": bytes(resp.name).decode("ascii", "replace"),
                "type": "enum" if int(resp.param_type) == 0 else "range",
                "default": int(resp.default_value),
                "current": int(resp.current_value),
                "values": [int(v) for v in resp.values],
                "unit": bytes(resp.unit).decode("ascii", "replace"),
            })
            index += 1
        return out

    def set_param(self, name: str, value: int):
        import uavcan.primitive.array.Natural32_1_0 as Natural32
        import uavcan.register.Access_1_0 as Access
        import uavcan.register.Name_1_0 as Name
        import uavcan.register.Value_1_0 as Value

        request = Access.Request(name=Name(PARAM_REGISTER_PREFIX + name),
                                 value=Value(natural32=Natural32([value])))
        self._call(self._reg, request)

    # ── Driver store ──────────────────────────────────────
    def save_slot(self, slot: int):
        if not self._execute(SAVE, bytes([slot & 0xFF])):
            self._raise_cmd_error()

    def delete_slot(self, slot: int):
        if not self._execute(DELETE_SLOT, bytes([slot & 0xFF])):
            self._raise_cmd_error()

    def clear_store(self):
        if not self._execute(CLEAR_STORE):
            self._raise_cmd_error()

    def cycle(self):
        self._execute(CYCLE)

    def identify(self):
        if not self._execute(IDENTIFY):
            raise RuntimeError("device returned failure or did not respond")

    # ── Driver info (the read_* source — GetDriverInfo, like the serial _info) ──
    def _info(self, slot: int = ACTIVE_SLOT):
        import aliensense.nxs.GetDriverInfo_1_0 as GetDriverInfo

        return self._call(self._driver_info, GetDriverInfo.Request(slot=slot))

    def read_driver_name(self) -> str:
        info = self._info()
        return bytes(info.name).decode("ascii", "replace") if info is not None else ""

    def read_slot_info(self, slot: int):
        """SupportsSlotPeek: GetDriverInfo(slot) peeks a stored slot's header
        (name + counts) without loading it; None if empty. `slot == 0xFF`
        returns the active driver."""
        info = self._info(slot)
        if info is None:
            return None
        name = bytes(info.name).decode("ascii", "replace")
        if not name:
            return None
        return SimpleNamespace(name=name, num_outputs=int(info.num_outputs),
                               num_params=int(info.num_params),
                               i2c_addr=int(info.i2c_addr))

    def read_sample_size(self) -> int:
        info = self._info()
        return int(info.sample_size) if info is not None else 0

    def read_sample_count(self) -> int:
        info = self._info()
        return int(info.sample_count) if info is not None else 0

    def read_status(self) -> int:
        info = self._info()
        return int(info.status_byte) if info is not None else 0

    def read_vm_state(self) -> int:
        info = self._info()
        return int(info.vm_state) if info is not None else 0

    def read_error_code(self) -> int:
        info = self._info()
        return int(info.error_code) if info is not None else 0

    def read_store_count(self) -> int:
        info = self._info()
        return int(info.store_count) if info is not None else 0

    def read_active_slot(self) -> int:
        info = self._info()
        return int(info.active_slot) if info is not None else ACTIVE_SLOT

    def read_runner_state(self) -> int:
        info = self._info()
        return int(info.runner_state) if info is not None else 0

    def read_probe_retries(self) -> int:
        info = self._info()
        return int(info.probe_retries) if info is not None else 0

    def read_io_err_count(self) -> int:
        """Absorbed I/O-error count since boot (`aliensense.nxs.io_err_count`)."""
        return self._read_natural16("aliensense.nxs.io_err_count")

    def read_probe_failed_count(self) -> int:
        """Probe give-up count since boot (`aliensense.nxs.probe_failed_count`)."""
        return self._read_natural16("aliensense.nxs.probe_failed_count")

    # ── Output descriptors ────────────────────────────────
    def read_outputs(self) -> Optional[List[dict]]:
        import aliensense.nxs.GetOutputInfo_1_0 as GetOutputInfo

        outs: List[dict] = []
        index = 0
        while True:
            resp = self._call(self._output_info, GetOutputInfo.Request(index=index))
            if resp is None:
                # RPC timeout mid-enumeration: transiently unreadable —
                # never hand back a truncated set as if it were complete.
                return None
            if index >= int(resp.num_outputs):
                break
            ftype = int(resp.field_type)
            outs.append({
                "idx": index,
                "name": bytes(resp.name).decode("ascii", "replace"),
                "type": FIELD_TYPE_NAMES.get(ftype, f"type{ftype}"),
                "byte_order": "big" if int(resp.byte_order) == 0 else "little",
                "semantic": int(resp.semantic),
                "byte_off": int(resp.byte_off),
                "count": int(resp.count),
                "scale": float(resp.scale),
                "offset": float(resp.offset),
                "unit": bytes(resp.unit).decode("ascii", "replace"),
            })
            index += 1
        return outs

    def _descriptor_token(self) -> int:
        # The active slot changes when the loaded driver does, so it serves as a
        # cheap descriptor-set generation for mid-stream re-fetch.
        return self.read_active_slot()

    # ── Streaming primitives ──────────────────────────────
    def _read_natural16(self, name: str) -> int:
        import uavcan.register.Access_1_0 as Access
        import uavcan.register.Name_1_0 as Name
        resp = self._call(self._reg, Access.Request(name=Name(name)))
        if resp is None:
            raise RuntimeError(f"register '{name}' read timed out (no response)")
        nat = resp.value.natural16
        if nat is None or len(nat.value) == 0:
            raise RuntimeError(f"register '{name}' is not a natural16 value")
        return int(nat.value[0])

    def _write_natural16(self, name: str, value: int) -> None:
        import uavcan.primitive.array.Natural16_1_0 as Natural16
        import uavcan.register.Access_1_0 as Access
        import uavcan.register.Name_1_0 as Name
        import uavcan.register.Value_1_0 as Value
        resp = self._call(self._reg, Access.Request(
            name=Name(name), value=Value(natural16=Natural16([value]))))
        if resp is None:
            raise RuntimeError(f"register '{name}' write timed out (no response)")

    def _read_natural32_pair(self, name: str) -> tuple:
        import uavcan.register.Access_1_0 as Access
        import uavcan.register.Name_1_0 as Name
        resp = self._call(self._reg, Access.Request(name=Name(name)))
        if resp is None:
            raise RuntimeError(f"register '{name}' read timed out (no response)")
        nat = resp.value.natural32
        if nat is None or len(nat.value) < 2:
            raise RuntimeError(f"register '{name}' is not a natural32[2] value")
        return int(nat.value[0]), int(nat.value[1])

    def _write_natural32_pair(self, name: str, values) -> tuple:
        """Write a natural32[2] register; returns the echoed pair (the device
        echoes the unchanged value when it rejects a write)."""
        import uavcan.primitive.array.Natural32_1_0 as Natural32
        import uavcan.register.Access_1_0 as Access
        import uavcan.register.Name_1_0 as Name
        import uavcan.register.Value_1_0 as Value
        resp = self._call(self._reg, Access.Request(
            name=Name(name), value=Value(natural32=Natural32(list(values)))))
        if resp is None:
            raise RuntimeError(f"register '{name}' write timed out (no response)")
        nat = resp.value.natural32
        if nat is None or len(nat.value) < 2:
            raise RuntimeError(f"register '{name}' is not a natural32[2] value")
        return int(nat.value[0]), int(nat.value[1])

    def push_time_sync(self, offset_us: int, bound_us: int,
                       rate_ppb: int = 0,
                       valid_for_us: int = 0) -> None:
        """One atomic register write disciplines the unit as a host
        source; the device echoes the applied state."""
        import uavcan.primitive.array.Integer64_1_0 as Integer64
        import uavcan.register.Access_1_0 as Access
        import uavcan.register.Name_1_0 as Name
        import uavcan.register.Value_1_0 as Value
        resp = self._call(self._reg, Access.Request(
            name=Name(TIME_SYNC_REGISTER),
            value=Value(integer64=Integer64([offset_us, bound_us,
                                             rate_ppb, valid_for_us]))))
        if resp is None:
            raise RuntimeError(
                f"register '{TIME_SYNC_REGISTER}' write timed out (no response)")

    def read_time_sync(self) -> tuple:
        """The unit's live discipline:
        `(offset_us, bound_us, rate_ppb, valid_for_us, source, valid)`."""
        import uavcan.register.Access_1_0 as Access
        import uavcan.register.Name_1_0 as Name
        resp = self._call(self._reg, Access.Request(name=Name(TIME_SYNC_REGISTER)))
        if resp is None:
            raise RuntimeError(
                f"register '{TIME_SYNC_REGISTER}' read timed out (no response)")
        val = resp.value.integer64
        if val is None or len(val.value) < 5:
            raise RuntimeError(
                f"register '{TIME_SYNC_REGISTER}' is not an integer64[5] value")
        offset_us, bound_us, rate_ppb, valid_for_us, source = (
                int(val.value[0]), int(val.value[1]), int(val.value[2]),
                int(val.value[3]), int(val.value[4]))
        return offset_us, bound_us, rate_ppb, valid_for_us, source, source != 0

    def read_device_time_us(self) -> Optional[int]:
        """The device µs clock via its natural64 register; None when the
        read times out or the firmware serves no time register (empty
        value), so the sync layer degrades instead of raising."""
        import uavcan.register.Access_1_0 as Access
        import uavcan.register.Name_1_0 as Name
        resp = self._call(self._reg, Access.Request(name=Name(TIME_US_REGISTER)))
        if resp is None:
            return None
        nat = resp.value.natural64
        if nat is None or len(nat.value) == 0:
            return None
        return int(nat.value[0])

    def read_decimation(self, subject=None) -> int:
        name = DECIMATION_REGISTER + (f".{subject}" if subject else "")
        return self._read_natural16(name)

    def write_decimation(self, value: int, subject=None) -> None:
        name = DECIMATION_REGISTER + (f".{subject}" if subject else "")
        self._write_natural16(name, value)

    # ── Commissioning (identity) ──────────────────────────
    def read_identity(self) -> dict:
        node = self._read_natural16("uavcan.node.id")
        topics = {n: self._read_natural16(f"uavcan.pub.{n}.id") for n in COMMISSION_TOPICS}
        return {"node_addr": node, "topics": topics}

    # ── CAN termination ───────────────────────────────────
    def read_can_term(self) -> int:
        return self._read_natural16(CAN_TERM_REGISTER)

    def write_can_term(self, value: int) -> None:
        from nxs.client import CAN_TERM_UNSET
        validate_can_term(value)
        self._write_natural16(CAN_TERM_REGISTER, value)
        echo = self.read_can_term()
        expect = 0 if value == CAN_TERM_UNSET else value
        if echo != expect:
            raise DeviceRefused(errno.EINVAL,
                                f"device rejected can-term {value} (echo {echo})")

    # ── CAN bit timing ────────────────────────────────────
    def read_can_bitrate(self) -> tuple:
        return self._read_natural32_pair(BITRATE_REGISTER)

    def write_can_bitrate(self, nominal: int, data: int) -> None:
        validate_can_bitrate(nominal, data)
        echo = self._write_natural32_pair(BITRATE_REGISTER, (nominal, data))
        # (0, 0) reverts: the echo then reports the compiled default profile,
        # so only a non-revert write can be checked against its own value.
        if (nominal, data) != (0, 0) and echo != (nominal, data):
            raise DeviceRefused(
                errno.EINVAL,
                f"device rejected bitrate {nominal}/{data} "
                f"(echo {echo[0]}/{echo[1]} — firmware whitelist mismatch?)")

    def commission(self, node_addr=None, topics=None, can_bitrate=None,
                   can_term=None) -> None:
        validate_commission(node_addr, topics)
        # The bitrate and termination writes self-check their echoes, so a
        # refused value aborts the commission before any identity register
        # is touched.
        if can_bitrate is not None:
            self.write_can_bitrate(*can_bitrate)
        if can_term is not None:
            self.write_can_term(can_term)
        if node_addr is not None:
            self._write_natural16("uavcan.node.id", node_addr)
        for name, addr in (topics or {}).items():
            self._write_natural16(f"uavcan.pub.{name}.id", addr)
        if not self._execute(COMMAND_STORE_PERSISTENT_STATES):
            self._raise_cmd_error(COMMISSION_ERR_REASON)

    def _arm_stream(self, every_nth: int):
        self.write_decimation(max(1, every_nth))

    def _disarm_stream(self):
        self.write_decimation(0)

    def _next_raw(self, timeout: float) -> Optional[Tuple[int, bytes, Optional[int]]]:
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None
