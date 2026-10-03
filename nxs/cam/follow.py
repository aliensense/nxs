# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The follower: a synced pair's one gain, the leading head's gain register
copied to the following head each frame (ADR 0013, R-SYNC-4, R-SYNC-8).

The contract:

- A step runs once a frame period on the monotonic clock; a slot a step
  overran is skipped, never made up.
- A step takes the host-wide bus lock in one attempt (`Bus.lock(0)`). While
  another run holds it the step touches nothing and counts as skipped, and
  the follower head keeps its gain.
- The leader's register is read in one combined transaction. A value other
  than the last one taken is read again and taken only when both reads
  agree: the leader's driver writes the bytes under its own hold, and a
  read between them is never copied.
- The follower's register is read, and written only when it differs: the
  register hold on, the gain bytes, the hold off. The hold is released
  whatever the gain write did.
- Wire time for a two-byte gain register at 400 kHz: about 0.3 ms a frame
  while the gain holds (two reads) and about 0.7 ms in a frame that moves
  it (three reads, three writes).
- Five seconds of failed steps in a row stop the run, the last error its
  reason.
- Once a second the heartbeat (`port_state.write_follow`) names the state,
  the leader's gain and the copies, skipped frames and errors so far.

The bus is the libnxs handle (ADR 0003). No hub window is selected: each
head answers at its own host address through the window the port runs.
"""

from __future__ import annotations

import errno
import math
import time
from typing import Callable, Dict, Optional

from nxs.cam import port_state
from nxs.cam.contracts import FollowPlan

#: Failed steps in a row for this long stop the run.
FAULT_WINDOW_S = 5.0
#: How often the heartbeat is rewritten.
HEARTBEAT_S = 1.0
#: How old a `following` heartbeat may be while the follower still copies:
#: three beats.
HEARTBEAT_STALE_S = 3.0


def _reason(exc: OSError) -> str:
    """An error as the heartbeat and the journal name it: the operation and
    the errno's text (`read 0x1e: Remote I/O error`)."""
    if exc.filename and exc.strerror:
        return f"{exc.filename}: {exc.strerror}"
    return str(exc)


class Follower:
    """Copies the leader head's gain register to the follower head once a
    frame period, as `plan` names them, over `bus` (a `_libnxs.Bus`). A
    `port` gets the heartbeat; `clock` is the monotonic clock the steps are
    paced on."""

    def __init__(self, plan: FollowPlan, bus, period_s: float, port: Optional[str] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._plan = plan
        self._bus = bus
        self._period_s = float(period_s)
        self._port = port
        self._clock = clock
        self._taken: Optional[bytes] = None
        self._copies = 0
        self._skipped = 0
        self._errors = 0
        self._reason: Optional[str] = None
        self._beat_at: Optional[float] = None

    def step(self) -> str:
        """One frame's copy and what it did: `copied`, `same`, `torn` (the
        leader's two reads disagreed, nothing copied) or `skipped` (another
        run holds the bus). OSError: the bus's."""
        try:
            self._bus.lock(0)
        except OSError as exc:
            if exc.errno != errno.EBUSY:
                raise
            self._skipped += 1
            return "skipped"
        try:
            gain = self._read(self._plan.leader_addr)
            if gain != self._taken:
                if self._read(self._plan.leader_addr) != gain:
                    return "torn"
                self._taken = gain
            if self._read(self._plan.follower_addr) == gain:
                return "same"
            self._write_held(gain)
            self._copies += 1
            return "copied"
        finally:
            self._bus.unlock()

    def run(self, stop) -> str:
        """Step at the frame period until `stop` (an Event: `is_set()`,
        `wait(seconds)`) is set, or until the steps have failed for
        FAULT_WINDOW_S; returns why the run ended, the last error's reason
        for the latter."""
        due = self._clock()
        failing_since: Optional[float] = None
        while not stop.is_set():
            now = self._clock()
            try:
                self.step()
                failing_since = None
            except OSError as exc:
                self._errors += 1
                self._reason = _reason(exc)
                failing_since = now if failing_since is None else failing_since
                if now - failing_since >= FAULT_WINDOW_S:
                    self._heartbeat(now, "stopped")
                    return self._reason
            self._heartbeat(now, "following")
            due += self._period_s
            now = self._clock()
            if due < now:
                due += math.ceil((now - due) / self._period_s) * self._period_s
            stop.wait(due - now)
        return "stopped"

    def _fields(self, state: str) -> Dict[str, str]:
        """The heartbeat's fields in the state named: the leader's gain once
        one is taken, the counts, the wall time, and why a stopped run ended."""
        fields = {"state": state}
        if self._taken is not None:
            order = "little" if self._plan.register[2] == "le" else "big"
            db = int.from_bytes(self._taken, order) * self._plan.db_per_step
            fields["gain_db"] = f"{round(db, 3):g}"
        fields.update(copies=str(self._copies), skipped=str(self._skipped),
                      errors=str(self._errors), at=f"{time.time():.3f}")
        if state == "stopped" and self._reason:
            fields["reason"] = self._reason
        return fields

    def _heartbeat(self, now: float, state: str) -> None:
        """Rewrite the heartbeat once a HEARTBEAT_S, and at once for a stop."""
        if self._port is None:
            return
        if state == "following" and self._beat_at is not None and now - self._beat_at < HEARTBEAT_S:
            return
        self._beat_at = now
        port_state.write_follow(self._port, self._fields(state))

    def _read(self, addr: int) -> bytes:
        reg, width, _order = self._plan.register
        return self._bus.read(addr, reg.to_bytes(2, "big"), width)

    def _write_held(self, gain: bytes) -> None:
        """The gain bytes at the follower head under its register hold, the
        hold released whatever the writes before it did."""
        reg = self._plan.register[0]
        hold, on, off = self._plan.hold
        addr = self._plan.follower_addr
        try:
            self._bus.write(addr, hold.to_bytes(2, "big") + bytes([on]))
            self._bus.write(addr, reg.to_bytes(2, "big") + gain)
        finally:
            self._bus.write(addr, hold.to_bytes(2, "big") + bytes([off]))
