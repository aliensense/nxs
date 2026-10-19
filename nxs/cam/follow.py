# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The follower: a synced pair's one gain, the leading head's gain copied to
the following head each frame (ADR 0013, R-7.4, R-7.8).

The contract:

- A step runs once a frame period on the monotonic clock; a slot a step
  overran is skipped, never made up.
- A step takes the host-wide bus lock in one attempt. While another run
  holds it the step touches nothing and counts as skipped, and the
  follower head keeps its gain.
- Behind pods the gain is each head's GAIN row on its camera map: the
  leader's value in effect is read, the follower's is read and committed
  only when it differs, in millidecibels. The leader's map is read for its
  `STATE` first: a value counts only while two agreeing reads say READY or
  FAULT. An INERT map, after a reset, reads 0 in every live cell, and a
  BUSY one is under a commit, so either skips the frame, which counts as
  skipped. On bare heads it is the capture nodes' gain control: the
  leader's is read as its capture session set it, through its kernel
  driver, which holds the value and reads no bus; the follower's is
  written only when it differs, its driver writing the register.
- Five seconds of failed steps in a row stop the run, the last error its
  reason.
- Once a second the heartbeat (`daemon_records.write_follow`) names the state,
  the leader's gain and the copies, skipped frames and errors so far.

The maps are opened on the port's bus, the nodes are the host's V4L2
subdevices (`Host.controls`); each head's is opened at its first step and
kept until the run closes them.
"""

from __future__ import annotations

import math
import time
from typing import Callable, Dict, Optional

from nxs._generated_constants import NxsMap
from nxs.cam import daemon_records, port_state
from nxs.cam.contracts import FollowPlan

#: A bare head's capture node control the follower copies.
NODE_GAIN = "gain"

#: Failed steps in a row for this long stop the run.
FAULT_WINDOW_S = 5.0
#: How often the heartbeat is rewritten.
HEARTBEAT_S = 1.0
#: How old a `following` heartbeat may be while the follower still copies:
#: three beats.
HEARTBEAT_STALE_S = 3.0


def _reason(exc: OSError) -> str:
    """An error as the heartbeat and the journal name it: the operation and
    the errno's text (`get gain on /dev/v4l-subdev2: No such device`)."""
    if exc.filename and exc.strerror:
        return f"{exc.filename}: {exc.strerror}"
    return str(exc)


class HeadControls:
    """The gain of a port's heads by address: behind pods (`mapped`) the GAIN
    row of the camera map at the address, the bus opened at the first use
    and the row found once, each step one cell read and a commit on change;
    on bare heads the gain control of the capture node at the address,
    opened through `host` (`Host.controls`). Each is kept until `close`."""

    def __init__(self, bus: str, host=None, mapped: bool = False) -> None:
        self._bus = bus
        self._host = host
        self._mapped = mapped
        self._map_bus = None
        self._nodes: Dict[int, object] = {}
        self._gain_rows: Dict[int, int] = {}
        self._last: Dict[int, int] = {}

    def _node(self, addr: int):
        node = self._nodes.get(addr)
        if node is not None:
            return node
        if self._mapped:
            from nxs import _libnxs
            from nxs._libnxs_map import Door, find_row

            if self._map_bus is None:
                self._map_bus = _libnxs.Bus.open(self._bus)
            node = Door(self._map_bus, addr)
            index = find_row(node.rows(), NxsMap.ParamId.GAIN_MDB)
            if index is None:
                node.close()
                raise OSError(f"the camera door @{addr:#04x} serves no gain row")
            self._gain_rows[addr] = index
        else:
            if self._host is None:
                from nxs import host as host_layer
                self._host = host_layer.current()
            node = self._host.controls(self._bus, addr)
        self._nodes[addr] = node
        return node

    def get(self, addr: int) -> Optional[int]:
        """The gain the head at `addr` holds. Behind a pod a changed value
        counts from two agreeing reads of its live cell, since a read inside
        the pod's own transaction comes back torn: on two that differ the
        last value holds, and there is none before the first pair agrees."""
        node = self._node(addr)
        if not self._mapped:
            return int(node.get(NODE_GAIN))
        cell = NxsMap.MapReg.LIVE + 4 * self._gain_rows[addr]
        value = int.from_bytes(node.read(cell, 4), "little")
        last = self._last.get(addr)
        if value != last:
            if int.from_bytes(node.read(cell, 4), "little") != value:
                return last
            self._last[addr] = value
        return value

    def held(self, addr: int) -> Optional[int]:
        """The gain the head at `addr` holds in effect (`get`). Behind a pod
        there is none while two agreeing reads of its map's `STATE` name
        neither READY nor FAULT: INERT after a reset, every live cell 0, or
        BUSY under a commit."""
        if self._mapped:
            node = self._node(addr)
            state = node.state()
            if node.state() != state or state not in (NxsMap.MapState.READY, NxsMap.MapState.FAULT):
                return None
        return self.get(addr)

    def set(self, addr: int, value: int) -> None:
        """Write the gain to the head at `addr`: a GAIN commit on its camera
        map, or its node's control, its driver writing the register."""
        from nxs._libnxs_map import ParamKind

        node = self._node(addr)
        if not self._mapped:
            node.set(NODE_GAIN, int(value))
            return
        state, error = node.commit({self._gain_rows[addr]: int(value)}, ParamKind.APPLY)
        if state != NxsMap.MapState.READY:
            raise OSError(f"set gain on the camera door @{addr:#04x}: ended in state {state}, error {error}")

    def close(self) -> None:
        for node in self._nodes.values():
            node.close()
        self._nodes.clear()
        self._gain_rows.clear()
        if self._map_bus is not None:
            self._map_bus.close()
            self._map_bus = None


class Follower:
    """Copies the leader head's gain to the follower head once a frame
    period, as `plan` names them, through `controls` (a `HeadControls`:
    `held(addr)`, `get(addr)`, `set(addr, value)`). A `port` gets the
    heartbeat; `clock` is the monotonic clock the steps are paced on."""

    def __init__(self, plan: FollowPlan, controls, period_s: float, port: Optional[str] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._plan = plan
        self._controls = controls
        self._period_s = float(period_s)
        self._port = port
        self._clock = clock
        self._lock = port_state.BusLock()
        self._taken: Optional[int] = None
        self._copies = 0
        self._skipped = 0
        self._errors = 0
        self._reason: Optional[str] = None
        self._beat_at: Optional[float] = None

    def step(self) -> str:
        """One frame's copy and what it did: `copied`, `same` or `skipped`
        (another run holds the bus, or the leader holds no gain in effect).
        OSError: a capture node's."""
        if not self._lock.acquire_now():
            self._skipped += 1
            return "skipped"
        try:
            gain = self._controls.held(self._plan.leader_addr)
            if gain is None:
                self._skipped += 1
                return "skipped"
            self._taken = gain
            if self._controls.get(self._plan.follower_addr) == gain:
                return "same"
            self._controls.set(self._plan.follower_addr, gain)
            self._copies += 1
            return "copied"
        finally:
            self._lock.release()

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
            fields["gain_db"] = f"{round(self._taken * self._plan.db_per_step, 3):g}"
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
        daemon_records.write_follow(self._port, self._fields(state))
