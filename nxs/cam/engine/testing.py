# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Testing helpers used by sequence execution tests."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Optional


@dataclass(frozen=True)
class Call:
    """A recorded call captured by the spy runner."""

    name: str
    args: tuple[Any, ...]
    kwargs: dict[str, Any]


class SpyRunner:
    """Records every operation so tests can assert exact behavior."""
    def __init__(self) -> None:
        """Start with an empty call log."""
        self.calls: List[Call] = []

    def record(self, name: str, *args: Any, **kwargs: Any) -> None:
        """Record a call entry."""
        self.calls.append(Call(name=name, args=args, kwargs=kwargs))

    # Optional convenience methods
    def cmd(self, cmd: int, args: list[int], comment: Optional[str] = None) -> None:
        """Record a command step."""
        self.record("cmd", cmd, args, comment=comment)

    def device_write(self, device: int, width: int, reg: int, offset: int, value: int,
                     comment: Optional[str] = None) -> None:
        """Record a device write step."""
        self.record("device_write", device, width, reg, offset, value, comment=comment)

    def read(self, device: int, reg: int, offset: int, length: int,
             store: Optional[str] = None, comment: Optional[str] = None) -> None:
        """Record a device read step."""
        self.record("read", device, reg, offset, length, store=store, comment=comment)

    def expect(self, device: int, reg: int, offset: int, value: int, mask: int,
               op: str, timeout_ms: int, poll_ms: int,
               comment: Optional[str] = None) -> None:
        """Record a read-and-verify step (always passes for the spy)."""
        self.record(
            "expect", device, reg, offset, value, mask, op,
            timeout_ms=timeout_ms, poll_ms=poll_ms, comment=comment,
        )
