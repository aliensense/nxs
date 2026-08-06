"""Per-NXS liveness state machine driven by the system/device_info
heartbeat (1 Hz emission, ~2 s disconnection threshold).

Single source of truth for "is this NXS up?" — other timeout-prone
sites (sample-count polls, ack waits) consult the tracker before
logging, so a disconnected NXS produces ONE WRN line instead of one
per failed poll cycle.
"""
import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, Optional


log = logging.getLogger(__name__)


class LinkState(Enum):
    """Per-NXS link state. WAITING is the initial no-heartbeat-yet
    grace state — silent on tick."""
    WAITING = "waiting"
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"


@dataclass
class HeartbeatSnapshot:
    """The most recent device_info content for an NXS. Surfaced by
    `nxs status` and used to detect mode changes (app vs
    bootloader) on reconnect."""
    uptime_us: int
    hw_id: int
    sender: int          # 0=application, 1=bootloader
    fw_version: str      # decoded ASCII, trailing NUL stripped
    serial_hex: str      # 12-byte UID rendered as 24 hex chars


@dataclass
class _NxsRecord:
    state: LinkState
    last_seen: float
    snapshot: Optional[HeartbeatSnapshot] = None


class LivenessTracker:
    """State machine for one or more NXS modules keyed by src_addr.

    Edge-triggered: every state transition produces exactly one log
    line; in-state updates are silent. The `now_fn` parameter is
    injectable so tests can drive a fake monotonic clock."""

    def __init__(self,
                 timeout_s: float = 2.0,
                 now_fn: Callable[[], float] = time.monotonic):
        self._timeout_s = timeout_s
        self._now_fn = now_fn
        self._units: Dict[int, _NxsRecord] = {}

    def on_heartbeat(self, src_addr: int, snapshot: HeartbeatSnapshot) -> None:
        """Feed a received device_info. Updates last_seen and emits a
        transition log if state changes."""
        now = self._now_fn()
        rec = self._units.get(src_addr)

        if rec is None:
            self._units[src_addr] = _NxsRecord(
                state=LinkState.CONNECTED, last_seen=now, snapshot=snapshot)
            log.info("NXS 0x%02X up: hw_id=%d fw=%s sender=%s",
                     src_addr, snapshot.hw_id, snapshot.fw_version,
                     _sender_str(snapshot.sender))
            return

        prev_state = rec.state
        prev_snapshot = rec.snapshot
        rec.last_seen = now
        rec.snapshot = snapshot

        if prev_state == LinkState.DISCONNECTED:
            rec.state = LinkState.CONNECTED
            changes = _describe_changes(prev_snapshot, snapshot)
            log.info("NXS 0x%02X back%s", src_addr,
                     f": {changes}" if changes else "")
        elif prev_state == LinkState.WAITING:
            # WAITING shouldn't happen here (first heartbeat creates
            # the record in CONNECTED), but cover it for completeness.
            rec.state = LinkState.CONNECTED
            log.info("NXS 0x%02X up: hw_id=%d fw=%s sender=%s",
                     src_addr, snapshot.hw_id, snapshot.fw_version,
                     _sender_str(snapshot.sender))
        # CONNECTED → CONNECTED is silent.

    def tick(self, now: Optional[float] = None) -> None:
        """Called from a host poll loop. Demotes any NXS whose
        last heartbeat aged past the timeout."""
        if now is None:
            now = self._now_fn()
        for src_addr, rec in self._units.items():
            if rec.state != LinkState.CONNECTED:
                continue
            age = now - rec.last_seen
            if age > self._timeout_s:
                rec.state = LinkState.DISCONNECTED
                log.warning("NXS 0x%02X down (no heartbeat for %.1fs)",
                            src_addr, age)

    def state_of(self, src_addr: int) -> LinkState:
        rec = self._units.get(src_addr)
        return rec.state if rec is not None else LinkState.WAITING

    def snapshot_of(self, src_addr: int) -> Optional[HeartbeatSnapshot]:
        rec = self._units.get(src_addr)
        return rec.snapshot if rec is not None else None

    def is_connected(self, src_addr: int) -> bool:
        """Convenience for spam-suppression guards: timeout-prone log
        sites check this before complaining about absent traffic."""
        return self.state_of(src_addr) == LinkState.CONNECTED

    def all_states(self) -> Dict[int, LinkState]:
        """For CLI `status` output."""
        return {src: rec.state for src, rec in self._units.items()}


def _sender_str(sender: int) -> str:
    return "application" if sender == 0 else \
           "bootloader" if sender == 1 else f"unknown({sender})"


def _describe_changes(prev: Optional[HeartbeatSnapshot],
                      curr: HeartbeatSnapshot) -> str:
    """Build a short note about what changed across a disconnect →
    reconnect — surfaces firmware swaps and mode flips that a bare
    'back online' would hide."""
    if prev is None:
        return f"fw={curr.fw_version} sender={_sender_str(curr.sender)}"
    diffs = []
    if prev.sender != curr.sender:
        diffs.append(
            f"sender {_sender_str(prev.sender)}→{_sender_str(curr.sender)}")
    if prev.fw_version != curr.fw_version:
        diffs.append(f"fw {prev.fw_version}→{curr.fw_version}")
    return ", ".join(diffs)
