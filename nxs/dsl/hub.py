"""Hub personalities: the program of a device on the host's own bus, run by the host executor one phase at a time."""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from nxs._generated_constants import NxsMap
from nxs.opcodes import Op
from nxs.dsl.camera import CamPersonality
from nxs.dsl.compiled import ImageKind
from nxs.dsl.errors import CompileError
from nxs.dsl.fields import Port, Show
from nxs.dsl.loop import _ASTCompiler

WAIT_CMD = "CMD_WAIT_MILLIS"

#: The most reads a gate's wait takes after one alternative of a ladder.
LADDER_TRIES_MAX = 255

#: Engine steps, the dicts `emit_steps` transcribes.
Steps = Sequence[Dict[str, Any]]


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


def _steps(verb: str, name: str, steps: Any) -> List[Dict[str, Any]]:
    """`steps` as a list of engine steps; anything else is a CompileError
    naming the verb's argument."""
    if not isinstance(steps, (list, tuple)) or not all(isinstance(s, dict) for s in steps):
        raise CompileError(
            f"{verb}: {name} takes a list of engine steps, the dicts emit_steps transcribes")
    return list(steps)


class HubDevice(CamPersonality):
    """Base class for a hub personality: a deserializer, or a serializer
    reached through its window. The program has the camera shape (a probe,
    a configure of `select()` blocks, no measure loop) and compiles to a
    HUB-kind image the host executor runs against the device's own bus
    address, with the run's parameters staged; a unit refuses the kind.

    A hub program is run once per phase: `configure()` dispatches on a
    `phase` param, and each phase's block dispatches on the run's other
    params (the link, the lane count, the data type). The host walks the
    port's graph and stages one phase after another. A device fact a
    phase needs arrives as a parameter. A phase branches on a read of the
    hub's own registers alone (`when`, `ladder`): a read through the
    serializer that the far end refuses faults the run, and the VM has no
    branch on a refusal. `store_reg` hands a field the phase read to the
    walk in the run's parameters, `store_fields` several fields packed in
    one.

    `emit_steps` transcribes a hub's engine steps (the `w`, `raw`,
    `expect`, `wait_ms` and `retry` dicts its knob modules and blobs build)
    into the program, so a phase is authored from the same rows the host
    engine replays: `DEVICE_ALIAS` names the alias that is this device,
    `COMPANION_ALIASES` maps every other alias a step may name to a
    declared companion.

    `PHASES` maps each phase's name to the value of the `phase` param that
    selects it. A program that offers ports, the node a graph image
    composes against, carries them as its HUB_PHASES record, so a graph's
    walks name phases the program runs and its report names a failed
    phase. A device the hub reaches through its window offers no port, and
    its phases stay its own."""

    IMAGE_KIND = ImageKind.HUB
    #: A hub program declares its device's ports; a device the hub reaches
    #: through its window, a serializer, is no node of the graph and offers none.
    PORTS: Tuple[Port, ...] = ()
    #: Each phase's name, 15 characters at most, to the value of `phase` that selects it.
    PHASES: Dict[str, int] = {}
    #: A link's worst-case settle is seconds long and the host image has
    #: the whole program budget: the sleep unrolls past the pod's cap.
    SLEEP_MAX_UNROLLED_MS = 10_000
    #: The engine alias of this device (`ADR_DESERIALIZER`, `ADR_SERIALIZER`).
    DEVICE_ALIAS = ""
    #: Engine alias -> companion name in `I2C_COMPANIONS`.
    COMPANION_ALIASES: Dict[str, str] = {}

    def declare_param(self, name: str, values: list, default: Any,
                      param_type: str = "enum", unit: str = "", kind: str = "run",
                      id: int = NxsMap.ParamId.CUSTOM, show: Optional[Show] = None,
                      labels: Optional[Sequence[str]] = None, public: bool = False):
        """A hub program's parameter: a run parameter its walks stage, which
        stays out of the image's PARAMS record unless `public` makes it a row
        the graph sets by name (`ClickPersonality.declare_param`)."""
        super().declare_param(name, values, default, param_type=param_type, unit=unit,
                              kind=kind, id=id, show=show, labels=labels, public=public)

    def _identity_names(self) -> Tuple[str, str]:
        """`(compatible, product)` of a hub program: its device's compatible,
        the facts beside its module, and the product its hub's manifest names
        for the deserializers its flows drive; a program with no facts beside
        it is named by its class."""
        from nxs.cam import hubs

        facts = self._pair_facts()
        compatible = facts.compatible if facts is not None else type(self).__name__
        try:
            product = hubs.product_name(compatible) or ""
        except hubs.HubError as exc:
            raise CompileError(f"{type(self).__name__}: its product is its hub's, and the "
                               f"hubs do not load: {exc}") from None

        return compatible, product

    def emit_steps(self, steps: List[Dict[str, Any]]) -> None:
        """Emit engine steps in order: a write as `write` (a byte list as a
        burst) followed by its settle, a raw write as `write_raw` to its
        companion, an expect as `poll` (or `check` when it has no window),
        a wait as `sleep_ms`, a read as a read into scratch, a retry block
        as `retry` around its writes with its last hard expect as the
        condition. A step naming a device this program does not declare is
        a CompileError."""
        for step in steps:
            self._emit_step(step)

    def steps_of(self, cfg, name: str) -> List[Dict[str, Any]]:
        """The steps of sequence `name` of a composed plan. A best-effort
        sequence is a CompileError: the host engine ends one whose device
        does not answer with a note, and the image has no note, since a
        refused transaction faults the run."""
        if cfg.is_best_effort(name):
            raise CompileError(
                f"{type(self).__name__}: sequence {name!r} is best effort, and a hub image "
                f"cannot note a device that does not answer; a refused transaction faults "
                f"the run")
        return cfg.steps_of(name)

    def emit_sequences(self, cfg, names: List[str]) -> None:
        """Emit the named sequences of a composed plan, in the given order."""
        for name in names:
            self.emit_steps(self.steps_of(cfg, name))

    def select_bit(self, param_name: str, bit: int, on: Callable[[], None],
                   off: Optional[Callable[[], None]] = None) -> None:
        """Dispatch on one bit of a declared param: `on` emits once for
        the declared values with the bit set, `off` (nothing by default)
        once for the rest. The shape of a per-link fact packed into one
        param."""
        values = [int(v) for v in self._dispatch_values("select_bit", param_name)]
        groups = {frozenset(v for v in values if v & bit): on,
                  frozenset(v for v in values if not v & bit): off or (lambda: None)}
        self.select_grouped(param_name, {g: b for g, b in groups.items() if g})

    def write_raw(self, dev: str, data: bytes) -> None:
        """Write `data` to the declared companion `dev` as one transaction
        with no register framing: `MEMCPY_IMM` stages the bytes in the
        sample buffer and `BUS_WRITE_RAW` sends them, inside the
        companion's `I2C_TARGET` bracket. The shape of a part whose
        register address is one byte under the hub's two-byte profile, the
        carrier's port expander: the first byte is its register. The raw
        write takes no retry ladder, so a refusal faults the run at once."""
        if self._trace_phase != "configure":
            raise CompileError("write_raw() belongs in configure()")
        if not isinstance(data, (bytes, bytearray)):
            raise CompileError(f"write_raw: data is bytes, got {type(data).__name__}")
        if not 1 <= len(data) <= _ASTCompiler.VM_SAMPLE_BUF_SIZE:
            raise CompileError(
                f"write_raw: {len(data)} B; a raw write stages 1 to "
                f"{_ASTCompiler.VM_SAMPLE_BUF_SIZE} B in the sample buffer")
        addr = self._companion_addr(dev)
        em = self._emitter
        em.emit(Op.I2C_TARGET, addr)
        em.emit(Op.MEMCPY_IMM, 0, len(data), *data)
        em.emit(Op.BUS_WRITE_RAW, 0, len(data))
        em.emit(Op.I2C_TARGET, 0)

    def when(self, reg: int, mask: int, value: int, then: Steps = (),
             else_: Steps = ()) -> None:
        """Read the hub's own register `reg` once and run the `then` steps
        when `(read & mask) == value`, the `else_` steps otherwise (`else`
        is a Python keyword). The steps are engine steps, as `emit_steps`
        takes them. Compiles to `REG_READ`, `AND` and `CMP_EQ`, then a jump
        past the steps that do not run."""
        if self._trace_phase != "configure":
            raise CompileError("when() belongs in configure()")
        self._condition("when", reg, mask, value)
        then = _steps("when", "then", then)
        else_ = _steps("when", "else_", else_)
        if not then and not else_:
            raise CompileError(f"when(0x{reg:X}): neither then nor else_ holds a step")
        em = self._emitter
        uid = self._fresh_label("when")
        other, end = f"{uid}_else", f"{uid}_end"
        with self._regs.scope():
            got = self._regs.get(f"__when_{uid}")
            self._emit_test(reg, mask, value, got)
            if not then:
                em.emit_jmp(Op.JNZ, end, got)
            else:
                em.emit_jmp(Op.JZ, other if else_ else end, got)
        if then:
            self.emit_steps(then)
            if else_:
                em.emit_jmp(Op.JMP, end)
                em.label(other)
        self.emit_steps(else_)
        em.label(end)

    def ladder(self, alternatives: Sequence[Steps], gate: Tuple[int, int, int],
               tries: int, delay_ms: int) -> None:
        """Run the alternatives' steps one after another until the gate
        holds. After each alternative the program reads the hub's own
        register of `gate`, a `(reg, mask, value)` condition, up to
        `tries` times with `delay_ms` before each read, and leaves the
        ladder at the first read whose masked value equals `value`. When
        no alternative passes, the ladder ends without a fault: the phase
        hands the gate to the walk with `store_reg`, or faults the run with
        `check`. Every read follows a `SLEEP_MS`, which resets the VM's
        runaway count, so a gate of any length runs."""
        if self._trace_phase != "configure":
            raise CompileError("ladder() belongs in configure()")
        if not isinstance(alternatives, (list, tuple)) or not alternatives:
            raise CompileError("ladder: alternatives is a non-empty list of step lists")
        rungs = [_steps("ladder", f"alternative {i}", alt) for i, alt in enumerate(alternatives)]
        if not isinstance(gate, (list, tuple)) or len(gate) != 3:
            raise CompileError(f"ladder: gate is a (reg, mask, value) condition, got {gate!r}")
        reg, mask, value = gate
        self._condition("ladder", reg, mask, value)
        if (isinstance(tries, bool) or not isinstance(tries, int)
                or not 1 <= tries <= LADDER_TRIES_MAX):
            raise CompileError(f"ladder: tries must be 1..{LADDER_TRIES_MAX}, got {tries!r}")
        if isinstance(delay_ms, bool) or not isinstance(delay_ms, int) or delay_ms < 1:
            raise CompileError(
                f"ladder: delay_ms must be 1 or more, got {delay_ms!r}; every read of the "
                f"gate follows a sleep")
        em = self._emitter
        uid = self._fresh_label("ladder")
        done = f"{uid}_done"
        for i, steps in enumerate(rungs):
            self.emit_steps(steps)
            with self._regs.scope():
                left = self._regs.get(f"__ladder_left_{uid}") if tries > 1 else None
                got = self._regs.get(f"__ladder_got_{uid}")
                read = f"{uid}_read{i}"
                if left is not None:
                    em.emit_u32(Op.LOAD_IMM, left, tries)
                    em.label(read)
                self.sleep_ms(delay_ms)
                self._emit_test(reg, mask, value, got)
                em.emit_jmp(Op.JNZ, done, got)
                if left is not None:
                    em.emit_cmp(Op.SUB, left, 1, left)
                    em.emit_jmp(Op.JNZ, read, left)
        em.label(done)

    def store_fields(self, param: str, fields: Sequence[Tuple[int, int, int]]) -> None:
        """Read each `(reg, mask, at)` field of the hub's own registers and
        record the fields packed as run parameter `param`: a field's masked
        bits move down to start at bit `at`, no two fields share a bit, and
        the packed value is their sum. How a phase hands several status bits
        to the walk in one parameter. Compiles to `REG_READ`, `AND` and a
        shift per field, `ADD_REG` from the second field on, then one
        `PARAM_STORE`. The parameter's declared values cover every packed
        value."""
        if self._trace_phase != "configure":
            raise CompileError("store_fields() belongs in configure()")
        if not isinstance(fields, (list, tuple)) or not fields:
            raise CompileError("store_fields: fields is a non-empty list of (reg, mask, at) fields")
        taken, placed = 0, []
        for field in fields:
            if not isinstance(field, (list, tuple)) or len(field) != 3:
                raise CompileError(f"store_fields: a field is (reg, mask, at), got {field!r}")
            reg, mask, at = field
            self._emitter.check_reg(reg)
            self._register_value("store_fields", "mask", mask)
            if mask == 0:
                raise CompileError(f"store_fields(0x{reg:X}): mask 0 keeps no bit")
            if isinstance(at, bool) or not isinstance(at, int) or not 0 <= at < 32:
                raise CompileError(f"store_fields(0x{reg:X}): at must be 0..31, got {at!r}")
            low = (mask & -mask).bit_length() - 1
            bits = (mask >> low) << at
            if bits >> 32:
                raise CompileError(f"store_fields(0x{reg:X}): the field at bit {at} runs past bit 31")
            if bits & taken:
                raise CompileError(
                    f"store_fields(0x{reg:X}): the field at bit {at} lands on a bit another "
                    f"field holds")
            taken |= bits
            placed.append((reg, mask, low, at))
        p, index = self._run_param(param, "store_fields")
        if p.param_type == "range":
            lo, hi = self._param_bounds(p)
            covered, declared = lo == 0 and hi >= taken, f"{lo}..{hi}"
        else:
            values = {int(v) for v in p.values}
            covered, declared = set(range(taken + 1)) <= values, sorted(values)
        if not covered:
            raise CompileError(
                f"store_fields({param!r}): the fields pack to 0..{taken}, and the parameter "
                f"declares {declared}")
        em = self._emitter
        with self._regs.scope():
            acc = self._regs.get("__fields_acc")
            got = self._regs.get("__fields_got")
            for i, (reg, mask, low, at) in enumerate(placed):
                dst = got if i else acc
                em.emit_reg(Op.REG_READ, reg, dst)
                em.emit_cmp(Op.AND, dst, mask, dst)
                if low > at:
                    em.emit(Op.SHR, dst, low - at, dst)
                elif at > low:
                    em.emit(Op.SHL, dst, at - low, dst)
                if i:
                    em.emit(Op.ADD_REG, acc, got, acc)
            em.emit(Op.PARAM_STORE, index, acc)

    def store_reg(self, reg: int, mask: int, shift: int, param: str) -> None:
        """Read the hub's own register `reg` and record `(read & mask) >>
        shift` as run parameter `param`, which the run's report returns:
        how a phase hands a lock bit or a status to the walk. Compiles to
        `REG_READ`, `AND`, `SHR` and `PARAM_STORE`. The parameter's
        declared values cover every value the field takes, 0 to
        `mask >> shift`."""
        if self._trace_phase != "configure":
            raise CompileError("store_reg() belongs in configure()")
        self._emitter.check_reg(reg)
        self._register_value("store_reg", "mask", mask)
        bits = 8 * self._i2c_profile().data_width
        if isinstance(shift, bool) or not isinstance(shift, int) or not 0 <= shift < bits:
            raise CompileError(f"store_reg: shift must be 0..{bits - 1}, got {shift!r}")
        top = mask >> shift
        if top == 0:
            raise CompileError(
                f"store_reg(0x{reg:X}): mask 0x{mask:X} shifted by {shift} keeps no bit, so "
                f"{param!r} would always read 0")
        p, index = self._run_param(param, "store_reg")
        if p.param_type == "range":
            lo, hi = self._param_bounds(p)
            covered, declared = lo == 0 and hi >= top, f"{lo}..{hi}"
        else:
            values = {int(v) for v in p.values}
            covered, declared = top < len(values) and set(range(top + 1)) <= values, sorted(values)
        if not covered:
            raise CompileError(
                f"store_reg({param!r}): the field reads 0..{top}, and the parameter declares "
                f"{declared}")
        em = self._emitter
        with self._regs.scope():
            got = self._regs.get("__store_reg")
            em.emit_reg(Op.REG_READ, reg, got)
            em.emit_cmp(Op.AND, got, mask, got)
            if shift:
                em.emit(Op.SHR, got, shift, got)
            em.emit(Op.PARAM_STORE, index, got)

    def _register_value(self, verb: str, name: str, operand: Any) -> None:
        """Refuse an operand past the register value the profile declares."""
        bits = 8 * self._i2c_profile().data_width
        if (isinstance(operand, bool) or not isinstance(operand, int)
                or not 0 <= operand < 1 << bits):
            raise CompileError(
                f"{verb}: {name} {operand!r} does not fit the {bits}-bit register value "
                f"the profile declares")

    def _condition(self, verb: str, reg: int, mask: int, value: int) -> None:
        """Refuse a condition a read cannot meet as written: a register past
        the profile's address width, a mask or value past its value width,
        or a value with a bit outside the mask."""
        self._emitter.check_reg(reg)
        self._register_value(verb, "mask", mask)
        self._register_value(verb, "value", value)
        if value & ~mask:
            raise CompileError(
                f"{verb}(0x{reg:X}): value 0x{value:X} has bits outside mask 0x{mask:X}, so "
                f"the masked read never equals it")

    def _emit_test(self, reg: int, mask: int, value: int, got: int) -> None:
        """r[got] = 1 when the hub's register `reg` reads `value` under
        `mask`, else 0."""
        em = self._emitter
        em.emit_reg(Op.REG_READ, reg, got)
        em.emit_cmp(Op.AND, got, mask, got)
        em.emit_cmp(Op.CMP_EQ, got, value, got)

    def _dev(self, alias: str) -> Optional[str]:
        if alias == self.DEVICE_ALIAS:
            return None
        name = self.COMPANION_ALIASES.get(alias)
        if name is None:
            raise CompileError(
                f"{type(self).__name__}: a step names {alias}, which is neither "
                f"DEVICE_ALIAS {self.DEVICE_ALIAS!r} nor in COMPANION_ALIASES")
        return name

    def _emit_step(self, step: Dict[str, Any]) -> None:
        if "retry" in step:
            self._emit_retry(step)
        elif "device" in step:
            self._emit_write(step)
        elif "raw" in step:
            self._emit_raw(step)
        elif _is_expect(step):
            self._emit_expect(step)
        elif "read" in step:
            self.read(_reg16(step), dev=self._dev(step["read"]))
        elif step.get("cmd") == WAIT_CMD:
            self.sleep_ms(_wait_ms(step))
        else:
            raise CompileError(f"emit_steps: cannot transcribe {step!r}")

    def _emit_raw(self, step: Dict[str, Any]) -> None:
        """A raw write goes to a companion: the hub's own register framing
        is the two-byte profile."""
        dev = self._dev(step["raw"])
        if dev is None:
            raise CompileError(
                f"emit_steps: a raw write goes to a declared companion, and {step['raw']} is "
                f"this device")
        self.write_raw(dev, bytes(_int(v) & 0xFF for v in step["value"]))
        settle = int(step.get("sleep_ms") or 0)
        if settle:
            self.sleep_ms(settle)

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
        naming its register. An expect without a window stays a check. A
        block that goes on past its last miss (`on_fail: continue`) is a
        CompileError: the image's retry faults there."""
        on_fail = str(step.get("on_fail", "abort"))
        if on_fail != "abort":
            raise CompileError(
                f"emit_steps: a retry block with on_fail {on_fail!r} goes on past its last "
                f"miss, and the image's retry faults there; a phase that goes on is a "
                f"ladder() whose gate store_reg() hands to the walk")
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
                else:
                    self._emit_step(s)

        self.retry(int(step["retry"]), int(step.get("delay_ms") or 0), body,
                   until=(_reg16(gate), _int(gate.get("mask", 0xFF)) & 0xFF,
                          _int(gate["value"]) & 0xFF),
                   timeout_ms=int(gate.get("timeout_ms") or 1),
                   poll_ms=int(gate.get("poll_ms") or 50))
        for s in others:
            self._emit_expect(s, soft=False)
