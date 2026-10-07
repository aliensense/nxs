"""Hub personalities: the program of a device on the host's own bus, run by the host executor one phase at a time."""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional

from nxs.dsl.camera import CamPersonality
from nxs.dsl.compiled import ImageKind
from nxs.dsl.errors import CompileError

WAIT_CMD = "CMD_WAIT_MILLIS"


def _int(value: Any) -> int:
    return int(str(value), 0)


def _reg16(step: Dict[str, Any]) -> int:
    reg = _int(step["reg"])
    offset = step.get("offset")
    return (reg << 8) | _int(offset) if offset is not None else reg


def _wait_ms(step: Dict[str, Any]) -> int:
    if "ms" in step:
        return _int(step["ms"])
    args = step.get("args") or [1, 0]
    return _int(args[-1])


def _is_expect(step: Dict[str, Any]) -> bool:
    return "expect" in step or "poll" in step


class HubDevice(CamPersonality):
    """Base class for a hub personality: a deserializer, or a serializer
    reached through its window. The program has the camera shape (a probe,
    a configure of `select()` blocks, no measure loop) and compiles to a
    HUB-kind image the host executor runs against the device's own bus
    address, with the run's parameters staged; a unit refuses the kind.

    A hub program is run once per phase: `configure()` dispatches on a
    `phase` enum param, and each phase's block dispatches on the run's
    other params (the link, the lane count, the data type). The host
    walks the port's graph and stages one phase after another, so the
    program never branches on what it read: a device fact a phase needs
    arrives as a parameter.

    `emit_steps` transcribes a hub's engine steps (the `w`, `expect`,
    `wait_ms` and `retry` dicts its knob modules and blobs build) into
    the program, so a phase is authored from the same rows the host
    engine replayed: `DEVICE_ALIAS` names the alias that is this device,
    `COMPANION_ALIASES` maps every other alias a step may name to a
    declared companion."""

    IMAGE_KIND = ImageKind.HUB
    #: A link's worst-case settle is seconds long and the host image has
    #: the whole program budget: the sleep unrolls past the pod's cap.
    SLEEP_MAX_UNROLLED_MS = 10_000
    #: The engine alias of this device (`ADR_DESERIALIZER`, `ADR_SERIALIZER`).
    DEVICE_ALIAS = ""
    #: Engine alias -> companion name in `I2C_COMPANIONS`.
    COMPANION_ALIASES: Dict[str, str] = {}

    def emit_steps(self, steps: List[Dict[str, Any]]) -> None:
        """Emit engine steps in order: a write as `write` (a byte list as a
        burst) followed by its settle, an expect as `poll` (or `check`
        when it has no window), a wait as `sleep_ms`, a read as a read
        into scratch, a retry block as `retry` around its writes with its
        last hard expect as the condition. A step naming a device this
        program does not declare is a CompileError."""
        for step in steps:
            if "retry" in step:
                self._emit_retry(step)
            elif "device" in step:
                self._emit_write(step)
            elif _is_expect(step):
                self._emit_expect(step)
            elif "read" in step:
                self.read(_reg16(step), dev=self._dev(step["read"]))
            elif step.get("cmd") == WAIT_CMD:
                self.sleep_ms(_wait_ms(step))
            else:
                raise CompileError(f"emit_steps: cannot transcribe {step!r}")

    def emit_sequences(self, cfg, names: List[str]) -> None:
        """Emit the named sequences of a composed plan, in the given order."""
        for name in names:
            self.emit_steps(cfg.steps_of(name))

    def select_bit(self, param_name: str, bit: int, on: Callable[[], None],
                   off: Optional[Callable[[], None]] = None) -> None:
        """Dispatch on one bit of a declared enum param: `on` emits once for
        the declared values with the bit set, `off` (nothing by default)
        once for the rest. The shape of a per-link fact packed into one
        param."""
        param = self._params.get(param_name)
        if param is None:
            raise CompileError(f"select_bit({param_name!r}): parameter not declared")
        values = [int(v) for v in param.values]
        groups = {frozenset(v for v in values if v & bit): on,
                  frozenset(v for v in values if not v & bit): off or (lambda: None)}
        self.select_grouped(param_name, {g: b for g, b in groups.items() if g})

    def _dev(self, alias: str) -> Optional[str]:
        if alias == self.DEVICE_ALIAS:
            return None
        name = self.COMPANION_ALIASES.get(alias)
        if name is None:
            raise CompileError(
                f"{type(self).__name__}: a step names {alias}, which is neither "
                f"DEVICE_ALIAS {self.DEVICE_ALIAS!r} nor in COMPANION_ALIASES")
        return name

    def _emit_write(self, step: Dict[str, Any]) -> None:
        dev = self._dev(step["device"])
        reg = _reg16(step)
        value = step["value"]
        if isinstance(value, list):
            self._write_burst(reg, bytes(_int(v) & 0xFF for v in value), dev=dev)
        else:
            self.write(reg, _int(value) & 0xFF, dev=dev)
        settle = int(step.get("sleep_ms") or 0)
        if settle:
            self.sleep_ms(settle)

    def _emit_expect(self, step: Dict[str, Any], soft: Optional[bool] = None) -> None:
        dev = self._dev(step.get("expect") or step.get("poll"))
        reg = _reg16(step)
        value = _int(step["value"]) & 0xFF
        mask = _int(step.get("mask", 0xFF)) & 0xFF
        timeout = int(step.get("timeout_ms") or 0)
        ne = str(step.get("op", "eq")) == "ne"
        if timeout:
            self.poll(reg, mask, value, timeout, int(step.get("poll_ms") or 50),
                      ne=ne, soft=bool(step.get("soft")) if soft is None else soft, dev=dev)
        elif ne:
            raise CompileError("emit_steps: an unpolled expect with op ne has no verb")
        else:
            self.check(reg, value, mask, dev=dev)

    def _emit_retry(self, step: Dict[str, Any]) -> None:
        """A retry block gates on its last hard polled expect; the block's
        other hard polled expects wait softly inside it and are asserted
        once after the block, so a condition that never holds still faults
        naming its register. An expect without a window stays a check."""
        inner = list(step["steps"])
        gates = [s for s in inner if _is_expect(s) and not s.get("soft")
                 and int(s.get("timeout_ms") or 0)]
        if not gates:
            raise CompileError("emit_steps: a retry block needs a hard expect to gate on")
        gate = gates[-1]
        alias = gate.get("expect") or gate.get("poll")
        if self._dev(alias) is not None:
            raise CompileError(
                f"emit_steps: a retry must gate on the primary's register, not {alias}")
        others = [s for s in gates if s is not gate]

        def body():
            for s in inner:
                if s is gate:
                    continue
                if s in others:
                    self._emit_expect(s, soft=True)
                elif "retry" in s:
                    self._emit_retry(s)
                elif "device" in s:
                    self._emit_write(s)
                elif _is_expect(s):
                    self._emit_expect(s)
                elif "read" in s:
                    self.read(_reg16(s), dev=self._dev(s["read"]))
                elif s.get("cmd") == WAIT_CMD:
                    self.sleep_ms(_wait_ms(s))
                else:
                    raise CompileError(f"emit_steps: cannot transcribe {s!r}")

        self.retry(int(step["retry"]), int(step.get("delay_ms") or 0), body,
                   until=(_reg16(gate), _int(gate.get("mask", 0xFF)) & 0xFF,
                          _int(gate["value"]) & 0xFF),
                   timeout_ms=int(gate.get("timeout_ms") or 1),
                   poll_ms=int(gate.get("poll_ms") or 50))
        for s in others:
            self._emit_expect(s, soft=False)
