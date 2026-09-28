# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Execution utilities for normalized command/device sequences."""

from __future__ import annotations

import time
from typing import List, Optional, Protocol, Tuple

from .models import (
    CmdStep,
    DeviceStep,
    ExpectFailedError,
    ExpectStep,
    ReadStep,
    RetryBlockStep,
    Step,
)
from .i2c import I2CHandlerError


class IStepSink(Protocol):
    """Protocol implemented by sinks that consume parsed steps."""

    def cmd(self, cmd: int, args: list[int], comment: Optional[str] = None) -> None:
        """Emit a command step."""
        ...

    def device_write(
        self,
        device: int,
        width: int,
        reg: int,
        offset: int,
        value: int | list[int],
        comment: Optional[str] = None,
    ) -> None:
        """Emit a device register write step."""
        ...

    def read(
        self,
        device: int,
        reg: int,
        offset: int,
        length: int,
        store: Optional[str] = None,
        comment: Optional[str] = None,
    ) -> None:
        """Emit a device register read step; ``store`` names the result."""
        ...

    def expect(
        self,
        device: int,
        reg: int,
        offset: int,
        value: int,
        mask: int,
        op: str,
        timeout_ms: int,
        poll_ms: int,
        comment: Optional[str] = None,
    ) -> None:
        """Emit a read-and-verify step; ``timeout_ms`` 0 means a single check."""
        ...


class SequenceExecutor:
    """Dispatches parsed steps to a concrete sink implementation."""

    def __init__(self, sink: IStepSink) -> None:
        """Store the sink that receives the steps."""
        self._sink = sink

    def execute_plan(self, plan: List[tuple[str, List[Step]]]) -> None:
        """Execute each named sequence in plan order."""
        for seq_name, steps in plan:
            self._execute_steps(seq_name, steps)

    def _execute_steps(self, seq_name: str, steps: List[Step]) -> None:
        """Dispatch a step list to the sink; retry blocks flatten to their inner
        steps, so sinks see the success-path stream. RuntimeError on a failed
        device_write."""
        for step in steps:
            if isinstance(step, CmdStep):
                self._sink.cmd(step.cmd, step.args, comment=step.comment)
            elif isinstance(step, DeviceStep):
                try:
                    self._sink.device_write(
                        step.device,
                        step.width,
                        step.reg,
                        step.offset,
                        step.value,
                        comment=step.comment,
                    )
                except (I2CHandlerError, OSError, TypeError, ValueError) as exc:
                    raise RuntimeError(
                        "I2C write failed in sequence "
                        f"'{seq_name}': device=0x{step.device:02X} "
                        f"reg=0x{step.reg:04X} offset=0x{step.offset:02X} "
                        f"value={step.value}"
                    ) from exc
            elif isinstance(step, ReadStep):
                self._sink.read(
                    step.device,
                    step.reg,
                    step.offset,
                    step.length,
                    store=step.store,
                    comment=step.comment,
                )
            elif isinstance(step, ExpectStep):
                self._sink.expect(
                    step.device,
                    step.reg,
                    step.offset,
                    step.value,
                    step.mask,
                    step.op,
                    step.timeout_ms,
                    step.poll_ms,
                    comment=step.comment,
                )
            elif isinstance(step, RetryBlockStep):
                self._execute_steps(seq_name, step.steps)
            else:
                raise TypeError(f"Unknown step type: {type(step)}")
