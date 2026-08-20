"""Thin pycyphal source for NXS samples over Cyphal/serial.

Subscribes to the device's RawSample subject and walks its GetOutputInfo
descriptors, decoding each sample with the shared `parse_sample` into the same
field dict the I2C reg-map bridge produces. That dict is the one convergence
point for the ROS 2 projection, so a Cyphal sample and an I2C sample reach
the `nxs ros2` bridge identically.

This is the pycyphal half of the host container; the I2C reg-map bridge is the
other. It is deliberately *not* an `NxsClient` transport: Cyphal is a
self-describing standard wire, decodable by yakut, so the host stays a thin
pycyphal client rather than wrapping the wire in the proprietary control
contract.

pycyphal is asyncio-based, so the node runs on a private event loop in a daemon
thread: the subscriber callback feeds a thread-safe queue that `next_sample`
drains, and the descriptor / register RPCs cross over via
`run_coroutine_threadsafe`. The pycyphal layer is created in `_start()` and is
skippable (`autostart=False`) so the offline test injects descriptors and a
fake queue without a bus.
"""

import asyncio
import logging
import os
import queue
import sys
import threading
from typing import List, Optional, Tuple

import nxs
from nxs._generated_constants import CyphalDefaults
from nxs.client import Sample
from nxs.descriptor import parse_sample
from nxs.image import FIELD_TYPE_NAMES
from nxs.serial_util import open_serial_once

# Node-ID and subject-ID defaults live in the constants SSOT (CyphalDefaults).
# The GetOutputInfo service-ID is a DSDL fixed_port_id, read from the generated
# type in _async_setup. These register names are projection strings, not
# numeric addresses, so they stay here (the firmware owns them in
# REGISTER_NAMES).
DECIMATION_REGISTER = "aliensense.nxs.decimation"
TIME_US_REGISTER = "aliensense.nxs.time_us"
CALIBRATION_REGISTER = "aliensense.nxs.calibration"
FW_DESCRIBE_REGISTER = "aliensense.nxs.fw.describe"

_log = logging.getLogger(__name__)


def _dsdl_roots() -> List[str]:
    """The namespace roots to compile, highest-precedence first. `CYPHAL_PATH`
    (colon-separated, yakut's convention) takes precedence for development
    against the live repo / PRDT; the DSDL vendored in the wheel is always
    appended as a fallback, so a `CYPHAL_PATH` set for yakut that omits the
    vendor `aliensense` namespace (e.g. off the repo host) can't leave
    nxs unable to compile its own types."""
    vendored = os.path.join(os.path.dirname(os.path.abspath(nxs.__file__)), "dsdl")
    env = os.environ.get("CYPHAL_PATH")
    if env:
        return [p for p in env.split(os.pathsep) if p] + [vendored]
    return [vendored]


# Compiled once per process: pycyphal.dsdl.compile_all is slow, so cache the
# generated packages on sys.path and reuse them across instances.
_dsdl_ready = False
_dsdl_lock = threading.Lock()


def _dsdl_fingerprint(namespaces) -> str:
    """Staleness key for the compiled-DSDL cache: a hash of the source roots and
    every `.dsdl` path / mtime / size. Stat-only — no file reads."""
    import hashlib
    entries = []
    for ns in namespaces:
        # Seed the root so a changed or empty root set still moves the fingerprint.
        entries.append(("__root__", ns))
        for root, _dirs, files in os.walk(ns):
            for name in files:
                if name.endswith(".dsdl"):
                    p = os.path.join(root, name)
                    try:
                        st = os.stat(p)
                    except OSError:
                        continue  # vanished mid-walk; treat as a changed tree
                    entries.append((ns, os.path.relpath(p, ns),
                                    st.st_mtime_ns, st.st_size))
    entries.sort()
    h = hashlib.sha1()
    for e in entries:
        h.update(repr(e).encode())
    return h.hexdigest()


def _toolchain_fingerprint() -> str:
    """Toolchain identity for the cache key: a compiled tree is only valid for
    the transpiler that produced it, so a pycyphal/nunavut/Python upgrade must
    recompile even when the .dsdl sources are unchanged."""
    parts = [f"py{sys.version_info.major}.{sys.version_info.minor}"]
    for mod in ("pycyphal", "nunavut"):
        try:
            parts.append(f"{mod}{__import__(mod).__version__}")
        except Exception:
            parts.append(f"{mod}?")
    return "-".join(parts)


def _ensure_dsdl() -> None:
    global _dsdl_ready
    if _dsdl_ready:
        return
    with _dsdl_lock:  # one compile + one sys.path mutation across concurrent sources
        if _dsdl_ready:
            return
        import pycyphal.dsdl

        out = os.path.join(os.path.expanduser("~"), ".cache", "nxs", "dsdl")
        os.makedirs(out, exist_ok=True)
        # First root carrying each namespace wins: CYPHAL_PATH overrides the
        # bundled copy where it has the namespace, the bundled copy fills any
        # gap. Compiling one namespace from two roots would conflict.
        namespaces = []
        for ns in ("uavcan", "aliensense"):
            for root in _dsdl_roots():
                cand = os.path.join(root, ns)
                if os.path.isdir(cand):
                    namespaces.append(cand)
                    break
        if not namespaces:
            raise RuntimeError(
                "no DSDL sources found (uavcan/aliensense absent from the "
                "bundled copy and $CYPHAL_PATH) — reinstall nxs[cyphal]")
        # compile_all() re-validates the whole namespace (~2 s) on every call,
        # so skip it when the compiled output is present and the fingerprint matches.
        marker = os.path.join(out, ".sources.sha1")
        fingerprint = f"{_dsdl_fingerprint(namespaces)}|{_toolchain_fingerprint()}"
        compiled = all(os.path.isdir(os.path.join(out, os.path.basename(ns)))
                       for ns in namespaces)
        try:
            with open(marker, encoding="utf-8") as f:
                cached = f.read().strip()
        except OSError:
            cached = None
        if not (compiled and cached == fingerprint):
            # The vendor GetOutputInfo service carries a fixed port-ID in the
            # unregulated aliensense namespace, which nunavut only allows with
            # this flag.
            pycyphal.dsdl.compile_all(namespaces, output_directory=out,
                                      allow_unregulated_fixed_port_id=True)
            with open(marker, "w", encoding="utf-8") as f:
                f.write(fingerprint)
        if out not in sys.path:
            sys.path.insert(0, out)
        _dsdl_ready = True


class CyphalSampleSource:
    """Stream + decode NXS samples over Cyphal/serial, self-describing by RPC."""

    def __init__(self, port: str, baud: int = 460800,
                 local_node_id: int = CyphalDefaults.HOST_NODE_ID,
                 remote_node_id: int = CyphalDefaults.DEFAULT_NODE_ID,
                 sample_subject_id: int = CyphalDefaults.SAMPLE_SUBJECT_ID,
                 descriptor_service_id: Optional[int] = None,
                 timeout: float = 1.0, autostart: bool = True):
        self._port = port
        self._baud = baud
        self._local_id = local_node_id
        self._remote_id = remote_node_id
        self._subject_id = sample_subject_id
        self._service_id = descriptor_service_id
        self._timeout = timeout

        self._queue: "queue.Queue[Tuple[int, bytes, Optional[int]]]" = queue.Queue(maxsize=2000)
        self._descriptors: Optional[List[dict]] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._pres = None
        self._sub = None
        self._desc_client = None
        self._reg_client = None

        if autostart:
            self._start()

    # ── pycyphal lifecycle ────────────────────────────────
    def _start(self) -> None:
        _ensure_dsdl()
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True,
                                        name="cyphal-loop")
        self._thread.start()
        asyncio.run_coroutine_threadsafe(self._async_setup(), self._loop).result(timeout=10)

    async def _async_setup(self) -> None:
        from pycyphal.presentation import Presentation
        from pycyphal.transport.serial import SerialTransport
        import aliensense.nxs.GetOutputInfo_1_0 as GetOutputInfo
        import aliensense.nxs.RawSample_0_1 as RawSample
        import uavcan.register.Access_1_0 as Access

        # Pre-opened at the target baud with the config frozen: any post-open
        # reprogramming (pycyphal's baudrate/timeout assignments) wedges a
        # J-Link V9 VCOM (see open_serial_once).
        transport = SerialTransport(open_serial_once(self._port, self._baud), self._local_id)
        self._pres = Presentation(transport)
        self._sub = self._pres.make_subscriber(RawSample, self._subject_id)
        self._sub.receive_in_background(self._on_sample)
        service_id = (self._service_id if self._service_id is not None
                      else GetOutputInfo._FIXED_PORT_ID_)
        self._desc_client = self._pres.make_client(GetOutputInfo, service_id, self._remote_id)
        self._reg_client = self._pres.make_client(Access, Access._FIXED_PORT_ID_, self._remote_id)

    async def _on_sample(self, msg, _meta) -> None:
        try:
            self._queue.put_nowait((int(msg.seq), bytes(msg.data), int(msg.timestamp_us)))
        except queue.Full:
            pass  # drop on overflow; the host fell behind

    def _call_sync(self, client, request):
        if self._loop is None or client is None:
            return None
        fut = asyncio.run_coroutine_threadsafe(client.call(request), self._loop)
        try:
            result = fut.result(timeout=self._timeout)
        except Exception:
            _log.debug("Cyphal RPC call failed", exc_info=True)
            return None
        if result is None:
            return None
        response, _meta = result
        return response

    def close(self):
        if self._loop is None:
            return
        # Close the presentation — and with it the transport and the background
        # receive task — on the loop before stopping it, so the serial port is
        # released rather than left open until GC.
        if self._pres is not None:
            async def _shutdown():
                self._pres.close()
            try:
                asyncio.run_coroutine_threadsafe(_shutdown(), self._loop).result(timeout=2)
            except Exception:
                pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._loop = None
        self._pres = None

    # ── Liveness ──────────────────────────────────────────
    def probe(self) -> bool:
        """True if the device answers a GetOutputInfo request."""
        return self._fetch_descriptor(0) is not None

    # ── Output descriptors ────────────────────────────────
    def _fetch_descriptor(self, index: int):
        """One GetOutputInfo response, or None. The seam the offline test
        overrides to stand in for the device."""
        import aliensense.nxs.GetOutputInfo_1_0 as GetOutputInfo
        return self._call_sync(self._desc_client, GetOutputInfo.Request(index=index))

    def descriptors(self) -> List[dict]:
        """The output-field descriptor set via GetOutputInfo, cached. The
        dict shape matches the I2C reg-map bridge, so `parse_sample` decodes
        either wire's samples the same way."""
        if self._descriptors is not None:
            return self._descriptors
        outs: List[dict] = []
        index = 0
        while True:
            resp = self._fetch_descriptor(index)
            if resp is None:
                # Mid-walk RPC failure with no cached set: return empty, never a
                # partial set (parse_sample would zero-pad the missing fields).
                return []
            if index >= int(resp.num_outputs):
                break
            outs.append(self._descriptor_dict(index, resp))
            index += 1
        self._descriptors = outs  # cache only a fully walked descriptor set
        return outs

    @staticmethod
    def _descriptor_dict(index: int, resp) -> dict:
        ftype = int(resp.field_type)
        return {
            "idx": index,
            "name": bytes(resp.name).decode("ascii", "replace"),
            "type": FIELD_TYPE_NAMES.get(ftype, f"type{ftype}"),
            "byte_order": "big" if int(resp.byte_order) == 0 else "little",
            "semantic": int(resp.semantic),
            "count": int(resp.count),
            "scale": float(resp.scale),
            "offset": float(resp.offset),
            "unit": bytes(resp.unit).decode("ascii", "replace"),
        }

    # ── Stream gate ───────────────────────────────────────
    def arm(self, decimation: int = 1) -> None:
        """Open the device's sample stream, caching the output descriptors on a
        quiet link first: under load the GetOutputInfo walk contends with the
        stream and can time out mid-walk, truncating the descriptor set."""
        # DECIMATION is persisted and device-wide, so restore the prior value on
        # failure rather than leaving the transient quiet (0) to reach NVS.
        prior = self._read_register(DECIMATION_REGISTER)
        self._write_register(DECIMATION_REGISTER, 0)
        if not self.descriptors():
            self._write_register(DECIMATION_REGISTER, prior)
            raise RuntimeError("could not read output descriptors on a quiet "
                               "link; refusing to arm the stream uncached")
        self._write_register(DECIMATION_REGISTER, max(1, decimation))

    def disarm(self) -> None:
        self._write_register(DECIMATION_REGISTER, 0)

    def _write_register(self, name: str, value: int) -> None:
        if self._reg_client is None:
            return  # best-effort: a host that can't write leaves the gate as-is
        import uavcan.primitive.array.Natural16_1_0 as Natural16
        import uavcan.register.Access_1_0 as Access
        import uavcan.register.Name_1_0 as Name
        import uavcan.register.Value_1_0 as Value
        request = Access.Request(name=Name(name), value=Value(natural16=Natural16([value])))
        self._call_sync(self._reg_client, request)

    def _read_register(self, name: str) -> int:
        if self._reg_client is None:
            return 0
        import uavcan.register.Access_1_0 as Access
        import uavcan.register.Name_1_0 as Name
        resp = self._call_sync(self._reg_client, Access.Request(name=Name(name)))
        nat = resp.value.natural16 if resp is not None else None
        return int(nat.value[0]) if (nat is not None and len(nat.value)) else 0

    # ── Samples ───────────────────────────────────────────
    def next_sample(self, timeout: float = 1.0) -> Optional[Sample]:
        """Next decoded `Sample`, or None on timeout."""
        try:
            seq, raw, timestamp_us = self._queue.get(timeout=timeout)
        except queue.Empty:
            return None
        return Sample(count=seq, raw=raw,
                      values=parse_sample(raw, self.descriptors()),
                      timestamp_us=timestamp_us)

    def iter_samples(self, timeout: float = 1.0):
        """Yield decoded `Sample`s; arm the device stream on entry, disarm on
        exit (break, exception, or GC)."""
        self.arm()
        try:
            while True:
                sample = self.next_sample(timeout)
                if sample is not None:
                    yield sample
        finally:
            try:
                self.disarm()
            except Exception:
                pass
