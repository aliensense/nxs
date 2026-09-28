# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Top-level orchestration entry point for the device stack."""

from __future__ import annotations

import os
import time
from collections.abc import Callable
from typing import Any
from typing import Optional

from .models import (
    CmdStep,
    DeviceStep,
    ExpectFailedError,
    ExpectStep,
    ReadStep,
    RetryBlockStep,
    Step,
)
from .config import ConfigHandler
from .i2c import CamI2c, I2CHandlerError

VERBOSE_ENV = "NXS_CAM_VERBOSE"


class Manager:
    """Executes a parsed configuration's event plan over I2C."""

    def __init__(
        self,
        config_handler: ConfigHandler,
        path_label: str = "<config>",
    ) -> None:
        """``path_label`` names the config in messages (a path or a composed-plan name)."""
        self.config_handler = config_handler
        self.path_to_config: str = path_label
        #: The register-level trace of a run (every write, read, expect and
        #: sleep) is printed under NXS_CAM_VERBOSE=1; the verbs report the
        #: outcome otherwise.
        self.verbose: bool = os.environ.get(VERBOSE_ENV) == "1"
        self.last_read_results: dict[str, list[int]] = {}
        #: Every polled expect the run waited on: what it waited for, how
        #: long it actually took, the budget it had, and whether it held.
        self.settles: list[dict[str, Any]] = []

    def _say(self, text: str) -> None:
        """The run's trace line, printed only under NXS_CAM_VERBOSE=1."""
        if self.verbose:
            print(text)

    @staticmethod
    def _normalize_values(value: int | list[int]) -> list[int]:
        """Normalize a scalar or list value into a list of bytes."""
        return value if isinstance(value, list) else [value]

    @staticmethod
    def _fmt_values(values: list[int]) -> str:
        """Render values as compact hex byte list."""
        return "[" + ", ".join(f"0x{v & 0xFF:02X}" for v in values) + "]"

    def _retry_i2c(
        self,
        op_desc: str,
        op: Callable[[], Any],
        retries: int,
        delay_s: float,
        continue_on_error: bool,
    ) -> Any:
        """Run an I2C operation with retries for transient I/O failures."""
        last_exc: Exception | None = None
        attempts = max(1, retries + 1)
        for attempt in range(1, attempts + 1):
            try:
                op()
                return True
            except (I2CHandlerError, OSError) as exc:
                last_exc = exc
                if attempt >= attempts:
                    break
                self._say(f"Retry {attempt}/{retries} after I2C error on {op_desc}: {exc}")
                time.sleep(delay_s)

        assert last_exc is not None
        if continue_on_error:
            self._say(
                "I2C operation failed after "
                f"{attempts} attempts: {op_desc}. Continuing."
            )
            return False
        raise RuntimeError(
            f"I2C operation failed after {attempts} attempts: {op_desc}"
        ) from last_exc

    def run(
        self,
        bus: str,
        retries: int = 5,
        retry_delay_s: float = 0.05,
        continue_on_error: bool = False,
    ) -> bool:
        """Run the event plan on ``bus`` with per-operation retries; True when every
        step succeeded. ``continue_on_error`` continues past exhausted retries."""
        plan = self.config_handler.build_event_plan()
        cfg = self.config_handler.get_parsed_config()
        wait_cmd_id = cfg.commands.get("CMD_WAIT_MILLIS")

        if "ADR_SENSOR" in cfg.addresses:
            default_addr = hex(cfg.addresses["ADR_SENSOR"])
        elif cfg.addresses:
            default_addr = hex(next(iter(cfg.addresses.values())))
        else:
            default_addr = "0x00"

        i2c = CamI2c(addr=default_addr, bus=bus)
        i2c.open()
        self.last_read_results = {}
        had_errors = False
        best_effort = getattr(self.config_handler, "best_effort", frozenset())
        try:
            for seq_name, steps in plan:
                self._say(f"\nSequence: {seq_name}")
                tolerated = seq_name in best_effort
                if self._execute_steps(
                    i2c,
                    steps,
                    wait_cmd_id=wait_cmd_id,
                    retries=retries,
                    retry_delay_s=retry_delay_s,
                    continue_on_error=continue_on_error or tolerated,
                    raise_on_error=False,
                ):
                    if tolerated:
                        self._say(f"{seq_name}: the device did not answer; already off")
                    else:
                        had_errors = True
        finally:
            i2c.close()

        return not had_errors

    def _execute_steps(
        self,
        i2c: CamI2c,
        steps: list[Step],
        wait_cmd_id: Optional[int],
        retries: int,
        retry_delay_s: float,
        continue_on_error: bool,
        raise_on_error: bool,
    ) -> bool:
        """Execute steps in order; True when at least one failed. ``raise_on_error``
        raises on any failure, for retry blocks."""
        effective_continue = continue_on_error and not raise_on_error
        had_errors = False
        for step in steps:
            if isinstance(step, CmdStep):
                if wait_cmd_id is not None and step.cmd == wait_cmd_id and step.args:
                    cmd_sleep_ms = max(0, int(step.args[-1]))
                    self._say(f"Sleep {cmd_sleep_ms} ms (from CMD_WAIT_MILLIS)")
                    time.sleep(cmd_sleep_ms / 1000.0)
                if step.sleep_ms > 0:
                    self._step_sleep(step.sleep_ms)
                continue

            if isinstance(step, DeviceStep):
                if self._execute_device_write(
                    i2c, step, retries, retry_delay_s, effective_continue
                ):
                    had_errors = True
                self._step_sleep(step.sleep_ms)
                continue

            if isinstance(step, ReadStep):
                if self._execute_read(
                    i2c, step, retries, retry_delay_s, effective_continue
                ):
                    had_errors = True
                self._step_sleep(step.sleep_ms)
                continue

            if isinstance(step, ExpectStep):
                try:
                    self._execute_expect(i2c, step)
                except ExpectFailedError as exc:
                    if step.soft:
                        # A settle poll stands in for a fixed sleep: the budget is
                        # spent and the program goes on, with the fact on record.
                        self._say(f"Settle not seen, continuing (soft): {exc}")
                    elif not effective_continue:
                        raise
                    else:
                        self._say(f"Expectation failed. Continuing. {exc}")
                        had_errors = True
                self._step_sleep(step.sleep_ms)
                continue

            if isinstance(step, RetryBlockStep):
                if self._execute_retry_block(
                    i2c, step, wait_cmd_id, retries, retry_delay_s
                ):
                    had_errors = True
                continue

            raise TypeError(f"Unknown step type: {type(step)}")

        return had_errors

    def _step_sleep(self, sleep_ms: int) -> None:
        """Sleep after a step when it requests a delay."""
        if sleep_ms > 0:
            self._say(f"Sleep {sleep_ms} ms (step sleep_ms)")
            time.sleep(sleep_ms / 1000.0)

    def _execute_device_write(
        self,
        i2c: CamI2c,
        step: DeviceStep,
        retries: int,
        retry_delay_s: float,
        continue_on_error: bool,
    ) -> bool:
        """Execute one device write; return True when it failed."""
        reg_addr = (step.reg << 8) | (step.offset & 0xFF)
        values = self._normalize_values(step.value)
        device_addr = hex(step.device)

        self._say(
            f"Write {device_addr} reg 0x{reg_addr:04X} = "
            f"{self._fmt_values(values)}"
        )
        ok = self._retry_i2c(
            op_desc=f"write {device_addr} reg 0x{reg_addr:04X}",
            op=lambda: i2c.write_reg(
                reg_addr,
                values,
                reg_width=16,
                data_width=8,
                addr=device_addr,
            ),
            retries=max(0, retries),
            delay_s=max(0.0, retry_delay_s),
            continue_on_error=continue_on_error,
        )
        return ok is False

    def _execute_read(
        self,
        i2c: CamI2c,
        step: ReadStep,
        retries: int,
        retry_delay_s: float,
        continue_on_error: bool,
    ) -> bool:
        """Execute one register read; True when it failed. A ``store`` name collects
        the value into ``last_read_results``."""
        reg_addr = (step.reg << 8) | (step.offset & 0xFF)
        device_addr = hex(step.device)
        captured: list[bytes] = []

        ok = self._retry_i2c(
            op_desc=f"read {device_addr} reg 0x{reg_addr:04X}",
            op=lambda: captured.append(
                i2c.read_reg(
                    reg_addr,
                    length=step.length,
                    reg_width=16,
                    data_width=8,
                    addr=device_addr,
                )
            ),
            retries=max(0, retries),
            delay_s=max(0.0, retry_delay_s),
            continue_on_error=continue_on_error,
        )
        if ok is False:
            return True

        values = list(captured[-1])
        self._say(
            f"Read {device_addr} reg 0x{reg_addr:04X} = "
            f"{self._fmt_values(values)}"
        )
        if step.store is not None:
            self.last_read_results[step.store] = values
        return False

    def _execute_expect(self, i2c: CamI2c, step: ExpectStep) -> None:
        """Check an expectation, polling until it holds or times out;
        ExpectFailedError when it does not hold in time."""
        reg_addr = (step.reg << 8) | (step.offset & 0xFF)
        device_addr = hex(step.device)
        want = step.value & step.mask
        started = time.monotonic()
        deadline = started + step.timeout_ms / 1000.0
        last_got: Optional[int] = None
        last_exc: Optional[Exception] = None
        what = (f"{device_addr} reg 0x{reg_addr:04X} & 0x{step.mask:02X} "
                f"{step.op} 0x{want:02X}")

        while True:
            try:
                data = i2c.read_reg(
                    reg_addr, length=1, reg_width=16, data_width=8,
                    addr=device_addr,
                )
                last_got = data[0] & step.mask
                last_exc = None
                passed = (
                    last_got == want if step.op == "eq" else last_got != want
                )
                if passed:
                    elapsed_ms = int((time.monotonic() - started) * 1000)
                    self._record_settle(step, what, elapsed_ms, True)
                    self._say(f"Expect {what}: OK"
                          + (f" after {elapsed_ms} ms" if step.timeout_ms else ""))
                    return
            except (I2CHandlerError, OSError) as exc:
                # A non-ACKing device counts as a miss; keep polling until the
                # deadline so expects survive link retrains.
                last_exc = exc

            if time.monotonic() >= deadline:
                break
            time.sleep(step.poll_ms / 1000.0)

        detail = (
            f"last read error: {last_exc}"
            if last_exc is not None
            else f"got 0x{last_got:02X}" if last_got is not None else "no read"
        )
        self._record_settle(step, what, step.timeout_ms, False)
        raise ExpectFailedError(
            f"expect {what} within {step.timeout_ms} ms: {detail}"
        )

    def _record_settle(self, step: ExpectStep, what: str, elapsed_ms: int,
                       held: bool) -> None:
        """Keep the wait on record (polled expects only)."""
        if step.timeout_ms:
            self.settles.append({
                "what": step.comment or what, "elapsed_ms": elapsed_ms,
                "budget_ms": step.timeout_ms, "held": held,
                "soft": step.soft,
            })

    def settle_summary(self) -> Optional[str]:
        """One line on the run's polled waits: count, time spent, budget."""
        if not self.settles:
            return None
        spent = sum(s["elapsed_ms"] for s in self.settles) / 1000.0
        budget = sum(s["budget_ms"] for s in self.settles) / 1000.0
        missed = [s for s in self.settles if not s["held"]]
        line = (f"{len(self.settles)} polled waits took {spent:.1f} s "
                f"of a {budget:.1f} s budget")
        if missed:
            line += (f"; {len(missed)} ran to the deadline: "
                     + ", ".join(s["what"] for s in missed))
        return line

    def _execute_retry_block(
        self,
        i2c: CamI2c,
        step: RetryBlockStep,
        wait_cmd_id: Optional[int],
        retries: int,
        retry_delay_s: float,
    ) -> bool:
        """Run a retry block; True when it ultimately failed. Any failing nested
        step re-runs the whole block; ``on_fail`` decides abort or continue."""
        attempts = max(1, step.times)
        last_exc: Optional[Exception] = None
        for attempt in range(1, attempts + 1):
            try:
                self._execute_steps(
                    i2c,
                    step.steps,
                    wait_cmd_id=wait_cmd_id,
                    retries=retries,
                    retry_delay_s=retry_delay_s,
                    continue_on_error=False,
                    raise_on_error=True,
                )
                # The nested had_errors flag is absorbed: an on_fail=continue child
                # is a best-effort block whose expected failures must not fail the run.
                return False
            except (RuntimeError, I2CHandlerError, OSError) as exc:
                last_exc = exc
                if attempt < attempts:
                    self._say(
                        f"Retry block attempt {attempt}/{attempts} failed: "
                        f"{exc}. Retrying in {step.delay_ms} ms."
                    )
                    time.sleep(step.delay_ms / 1000.0)

        assert last_exc is not None
        if step.on_fail == "continue":
            self._say(
                f"Retry block failed after {attempts} attempts: {last_exc}. "
                "Continuing."
            )
            return True
        raise RuntimeError(
            f"Retry block failed after {attempts} attempts"
        ) from last_exc
