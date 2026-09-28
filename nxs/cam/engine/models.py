# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Typed models shared across parsing and sequence execution."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Literal, Union


@dataclass(frozen=True)
class Meta:
    """Top-level metadata block from YAML."""

    name: str
    version: str


@dataclass(frozen=True)
class CmdStep:
    """A normalized command step."""

    kind: Literal["cmd"]
    cmd: int
    args: List[int]
    sleep_ms: int = 0
    comment: Optional[str] = None


@dataclass(frozen=True)
class DeviceStep:
    """A normalized device register write step."""

    kind: Literal["device"]
    device: int
    width: int
    reg: int
    offset: int
    value: Union[int, List[int]]
    sleep_ms: int = 0
    comment: Optional[str] = None


@dataclass(frozen=True)
class ReadStep:
    """A normalized device register read step; a ``store`` name collects the
    value into the run's result map. Reads do not feed later steps."""

    kind: Literal["read"]
    device: int
    reg: int
    offset: int
    length: int = 1
    store: Optional[str] = None
    sleep_ms: int = 0
    comment: Optional[str] = None


@dataclass(frozen=True)
class ExpectStep:
    """A normalized read-and-verify step: one byte at ``(reg << 8) | offset``,
    masked and compared with ``op``, polled every ``poll_ms`` for ``timeout_ms``.
    A ``soft`` step proceeds at the deadline with a warning instead of raising."""

    kind: Literal["expect"]
    device: int
    reg: int
    offset: int
    value: int = 0
    mask: int = 0xFF
    op: Literal["eq", "ne"] = "eq"
    timeout_ms: int = 0
    poll_ms: int = 50
    sleep_ms: int = 0
    comment: Optional[str] = None
    soft: bool = False


@dataclass(frozen=True)
class RetryBlockStep:
    """A block of steps retried as a unit: any failing step re-runs the block
    after ``delay_ms``, up to ``times``; then ``on_fail`` aborts or continues."""

    kind: Literal["retry"]
    times: int
    delay_ms: int = 0
    on_fail: Literal["abort", "continue"] = "abort"
    steps: List["Step"] = field(default_factory=list)
    comment: Optional[str] = None


Step = Union[CmdStep, DeviceStep, ReadStep, ExpectStep, RetryBlockStep]


class ExpectFailedError(RuntimeError):
    """Raised when an ``ExpectStep`` predicate does not hold in time."""


@dataclass(frozen=True)
class ParsedConfig:
    """Fully parsed and normalized configuration."""

    meta: Meta
    commands: Dict[str, int]
    addresses: Dict[str, int]
    event_list: List[str]
    sequences: Dict[str, List[Step]]
