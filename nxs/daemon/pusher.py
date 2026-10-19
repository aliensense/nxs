# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The time discipline nxsd keeps while it holds the units without a data
plane: one round of the suite pusher every push interval, on a thread of
its own, each unit's transitions in the journal as the bare verb prints
them. An I²C unit's probe and push take the host's I²C bus lock without a
wait, each alone, and a unit on another link takes none: a round never
holds the bus across a CAN or serial unit, and a bus another run holds
passes over its I²C units alone. While the data plane runs, the bridge
pushes on the same cadence and this pusher stands down. A stop hands every
unit's fit to the next start, so a reload or a bring-up never restarts a
fit empty."""

from __future__ import annotations

import contextlib
import logging
import threading
from typing import Dict, Optional

from nxs.client import PUSH_INTERVAL_S

log = logging.getLogger("nxsd")

class Pusher:
    """The suite pusher as a resident the daemon starts and stops."""

    def __init__(self, interval: float = PUSH_INTERVAL_S, factory=None) -> None:
        self._interval = interval
        self._factory = factory
        self._pusher = None
        self._fits: Dict[str, object] = {}
        self._last: Dict[str, bool] = {}
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self, cfg) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        from nxs.suite.timesync import SuitePusher

        factory = self._factory or SuitePusher
        self._pusher = factory(cfg, fits=self._fits, link_free=_bus_free)
        self._stop.clear()
        self._thread = threading.Thread(target=self._serve, name="time-pusher", daemon=True)
        self._thread.start()
        log.info("time discipline: pushing to %d unit(s) every %g s while the daemon holds",
                 len(cfg.units), self._interval)

    def stop(self, reason: str = "nxsd stops") -> None:
        if self._thread is None:
            return
        self._stop.set()
        # The round ends first, on the deaf links' timeouts at most: the
        # suspend closes the links it is on.
        self._thread.join()
        self._thread = None
        if self._pusher is not None:
            self._fits = self._pusher.suspend()
            self._pusher = None
        log.info("time discipline: pusher stopped (%s)", reason)

    def _serve(self) -> None:
        while not self._stop.is_set():
            self._round()
            self._stop.wait(self._interval)

    def _round(self) -> None:
        for report in self._pusher.round():
            ok = report.bound_us is not None
            if self._last.get(report.name) != ok:
                detail = f"±{report.bound_us} µs" if ok else report.note
                log.info("time sync %s: %s %s", report.name, "✓" if ok else "✗", detail)
            self._last[report.name] = ok


@contextlib.contextmanager
def _bus_free(link):
    """The host's I²C bus lock around an I²C link's probe or push, taken
    without a wait: a push inside a unit's own bus window would collide
    with its transfers. Any other link needs none."""
    if link.transport != "i2c":
        yield True
        return
    from nxs.cam.port_state import BusLock

    lock = BusLock()
    if not lock.acquire_now():
        yield False
        return
    try:
        yield True
    finally:
        lock.release()
