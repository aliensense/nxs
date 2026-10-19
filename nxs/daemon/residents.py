# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""What the daemon keeps running beside the ports, as one."""

import contextlib
import logging

log = logging.getLogger("nxsd")


class Residents:
    """What the daemon keeps running beside the ports, as one: the followers
    of the synced pairs, the modes resident, the camera nodes, and the time
    pusher while the daemon holds. They start in order once the ports are
    up, and stop in reverse before a bring-up or a reload reprograms what
    they touch. A member whose start or stop raises is logged, and the
    others start or stop all the same."""

    def __init__(self, *members) -> None:
        self._members = list(members)

    def add(self, member) -> None:
        self._members.append(member)

    def remove(self, member) -> None:
        self._members.remove(member)

    def start(self, cfg) -> None:
        for member in list(self._members):
            _guarded(member, "start", cfg)

    def stop(self, reason: str = "nxsd stops") -> None:
        for member in reversed(self._members):
            _guarded(member, "stop", reason)

    @contextlib.contextmanager
    def beside(self, member, cfg, reason: str):
        """Every member but `member` stopped in reverse for the block,
        `reason` in the journal, and started again in order on `cfg` after
        it: the modes resident's walk is a bring-up, which reprograms what
        the others touch."""
        others = [other for other in self._members if other is not member]
        for other in reversed(others):
            _guarded(other, "stop", reason)
        try:
            yield
        finally:
            for other in others:
                _guarded(other, "start", cfg)


def _guarded(member, verb: str, arg) -> None:
    """One member's start or stop: what it raises is the journal's, never
    the daemon's end, and never the end of the members after it."""
    try:
        getattr(member, verb)(arg)
    except Exception:       # noqa: BLE001 (one member's fault stays its own)
        log.exception("%s: the %s failed", type(member).__name__, verb)
