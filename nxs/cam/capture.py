# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Headless choreographed capture: the delivery verdict without viewers.
The consumer is the viewers' source with an identity+fakesink tail, launched
consumer first and CSI gate second; the verdict counts delivered buffers."""

from __future__ import annotations

import re
import shlex
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Union

from nxs import host as host_layer

from . import viewers

CAPTURE_SETTLE_S = 2.5
KILL_GRACE_S = 2.0
ATTEMPTS = 2


def _stop(proc, timeout_s: float) -> str:
    """Drain a consumer that overran its window: SIGTERM with the viewer grace
    so the capture session closes, SIGKILL only when it ignores the term."""
    proc.terminate()
    try:
        output, _ = proc.communicate(timeout=viewers.VIEWER_STOP_GRACE_S)
    except subprocess.TimeoutExpired:
        proc.kill()
        output, _ = proc.communicate(timeout=KILL_GRACE_S)
    del timeout_s
    return output or ""


@dataclass
class CaptureResult:
    frames: int
    delivered: int
    errors: int
    returncode: int
    attempts: int
    #: Delivered rate from the buffers' timestamps (None: too few frames).
    fps: Optional[float] = None
    #: The consumer never built its pipeline (gst-launch's own complaint,
    #: e.g. a missing source element): no capture was attempted.
    consumer_error: Optional[str] = None

    @property
    def ok(self) -> bool:
        return (self.returncode == 0 and self.errors == 0
                and self.consumer_error is None
                and self.delivered >= self.frames)


def consumer_error(output: str) -> Optional[str]:
    """gst-launch's complaint when the pipeline could not be built at all —
    a missing element (the source's plugin package absent) or a bad link — or
    None when the pipeline ran. Such a run says nothing about the camera."""
    for line in output.splitlines():
        if "erroneous pipeline" in line or "could not link" in line:
            return line.strip()
    return None


def count_stack_errors(output: str) -> int:
    """Count the capture stack's error lines in the consumer's output.
    Supplementary signal only; the pass verdict rides the delivered count."""
    return host_layer.current().consumer_errors(output)


def count_delivered(output: str) -> int:
    """Count buffers the identity element saw (`-v` notify lines)."""
    return output.count("last-message = chain")


_PTS = re.compile(r"last-message = chain.*?pts: (\d+):(\d\d):(\d\d)\.(\d{9})")


def delivered_rate(output: str) -> Optional[float]:
    """The delivered frame rate from the buffers' PTS in identity's notify
    lines, display-independent. None below two timestamped buffers."""
    stamps = [
        int(h) * 3600 + int(m) * 60 + int(s) + int(ns) / 1e9
        for h, m, s, ns in _PTS.findall(output)
    ]
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return None
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


_SIZE = re.compile(r"last-message = chain.*?\((\d+) bytes")


def buffer_size_histogram(output: str) -> Dict[int, int]:
    """Buffer sizes identity reported, by count (a geometry change shows
    as a second bin)."""
    hist: Dict[int, int] = {}
    for size in _SIZE.findall(output):
        hist[int(size)] = hist.get(int(size), 0) + 1
    return hist


def timestamps(output: str) -> list:
    """Every buffer's PTS in seconds, in order."""
    return [int(h) * 3600 + int(m) * 60 + int(s) + int(ns) / 1e9
            for h, m, s, ns in _PTS.findall(output)]


#: The encoder `--encoder` takes by default: the host's own.
DEFAULT_ENCODER = "host"


def consumer_pipeline(capture_id: int, sensor_mode: int, width: int,
                      height: int, frames: int,
                      framerate: Optional[list] = None,
                      snapshot_dir: Optional[str] = None,
                      encoder: str = DEFAULT_ENCODER,
                      props: str = "") -> list:
    """The viewers' source and caps with a counting tail: a fakesink, or with
    ``snapshot_dir`` a JPEG writer producing ``frame-NNNN.jpg`` per frame
    (``encoder`` names the element, the host's own by default); ``props``
    are the source's properties for the link's caps
    (`viewers.source_props`)."""
    host = host_layer.current()
    if snapshot_dir:
        location = shlex.quote(f"{snapshot_dir}/frame-%04d.jpg")
        element = host.jpeg_encoder() if encoder == DEFAULT_ENCODER else encoder
        tail = (f"identity silent=false ! queue max-size-buffers=8 ! "
                f"{host.convert()} ! {element} ! "
                f"multifilesink location={location}")
    else:
        tail = "identity silent=false ! fakesink"
    props = f" {props}" if props else ""
    pipeline = (
        f"{host.source(capture_id, sensor_mode)}{props} "
        f"num-buffers={frames} ! "
        f"{host.caps(width, height, framerate)} ! "
        f"{tail}"
    )
    return ["gst-launch-1.0", "-v", "-e", *shlex.split(pipeline)]


def _open_gate(gate, procs, timeout_s: float) -> None:
    """Open the gate behind consumers already started; a gate that does not
    open (the bus held past the lock's wait, a control-bus fault) stops them
    before its refusal goes on, so none waits on a closed output."""
    try:
        gate(True)
    except BaseException:
        for proc in procs:
            _stop(proc, timeout_s)
        raise


def headless_capture(
    hints: Dict[str, Any],
    capture_id: int,
    gate,
    frames: int,
    timeout_s: Optional[float] = None,
    run=subprocess,
    snapshot_dir: Optional[str] = None,
    encoder: str = DEFAULT_ENCODER,
    port: Optional[str] = None,
) -> CaptureResult:
    """Capture `frames` frames headlessly with the CSI-gate choreography; a
    failed verdict restarts the capture daemon and rolls once more. `hints` are the
    resolved caps of a link of `port`, `gate(enable)` toggles the CSI gate; `.ok`
    is the verdict."""
    if timeout_s is None:
        timeout_s = 20.0 + frames / 5.0

    cmd = consumer_pipeline(capture_id, hints["sensor_mode"], hints["width"],
                            hints["height"], frames, hints.get("framerate"),
                            snapshot_dir=snapshot_dir, encoder=encoder,
                            props=viewers.source_props(hints, port=port))

    result = None
    for attempt in range(1, ATTEMPTS + 1):
        if attempt > 1:
            host_layer.current().restart_capture_daemon(run)
        # Consumer first, gate second: VI only syncs to a stream start it
        # witnessed.
        gate(False)
        proc = run.Popen(cmd, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True)
        time.sleep(CAPTURE_SETTLE_S)
        _open_gate(gate, [proc], timeout_s)
        try:
            output, _ = proc.communicate(timeout=timeout_s)
            rc = proc.returncode
        except subprocess.TimeoutExpired:
            output = _stop(proc, timeout_s)
            rc = -1

        result = CaptureResult(
            frames=frames, delivered=count_delivered(output or ""),
            fps=delivered_rate(output or ""),
            errors=count_stack_errors(output or ""), returncode=rc,
            attempts=attempt, consumer_error=consumer_error(output or ""),
        )
        # A consumer that never built its pipeline is not a roll to repeat.
        if result.ok or result.consumer_error:
            break
    if result is not None and result.returncode == -1:
        # The last attempt overran its window: whatever session the
        # consumer left behind is not the next command's problem.
        host_layer.current().restart_capture_daemon(run)

    return result



def headless_pair(hints_by_id: Dict[int, Dict[str, Any]], gate,
                  frames: Union[int, Dict[int, int]], timeout_s: Optional[float] = None,
                  run=subprocess, port: Optional[str] = None) -> Dict[int, str]:
    """Capture several links of `port` at once with one gate choreography:
    every consumer starts, the gate opens once, each runs to its frame count
    (`frames`: one count for every capture id, or a count per id).
    A roll on which any consumer delivered nothing restarts the capture
    daemon and rolls once more (`ATTEMPTS`). Returns each capture id's raw
    output (for timestamps and counts)."""
    counts = (dict(frames) if isinstance(frames, dict)
              else {capture_id: int(frames) for capture_id in hints_by_id})
    outputs: Dict[int, str] = {}
    for attempt in range(1, ATTEMPTS + 1):
        if attempt > 1:
            host_layer.current().restart_capture_daemon(run)
        outputs = _pair_roll(hints_by_id, gate, counts, timeout_s, run, port)
        if all(count_delivered(out) for out in outputs.values()):
            break
    return outputs


def _pair_roll(hints_by_id: Dict[int, Dict[str, Any]], gate, counts: Dict[int, int],
               timeout_s: Optional[float], run, port: Optional[str]) -> Dict[int, str]:
    """One simultaneous roll of every consumer, each at its link's caps and
    source properties, to its own frame count."""
    if timeout_s is None:
        timeout_s = 20.0 + max(counts.values(), default=0) / 5.0
    gate(False)
    procs = {}
    for capture_id, hints in hints_by_id.items():
        cmd = consumer_pipeline(capture_id, hints["sensor_mode"], hints["width"],
                                hints["height"], counts[capture_id], hints.get("framerate"),
                                props=viewers.source_props(hints, port=port))
        procs[capture_id] = run.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True)
    time.sleep(CAPTURE_SETTLE_S)
    _open_gate(gate, list(procs.values()), timeout_s)
    outputs: Dict[int, str] = {}
    timed_out: Dict[int, bool] = {}

    def drain(capture_id, proc):
        # Each consumer prints a line per frame; drained in series, the second
        # would fill its pipe and stall, and the captures would not be simultaneous.
        try:
            output, _ = proc.communicate(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            output = _stop(proc, timeout_s)
            timed_out[capture_id] = True
        outputs[capture_id] = output or ""

    threads = [threading.Thread(target=drain, args=(capture_id, proc), daemon=True)
               for capture_id, proc in procs.items()]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    if timed_out:
        host_layer.current().restart_capture_daemon(run)
    return outputs
