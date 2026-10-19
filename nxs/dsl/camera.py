"""Cam personalities: a probe, a configure with one block per mode, no measure loop."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Tuple

import yaml

from nxs._generated_constants import NxsDriverImage
from nxs.opcodes import Op
from nxs.cam.laws import ApplyLaw, RegSpec
from nxs.dsl.compiled import ImageKind
from nxs.dsl.emit import OpErrorCode, _emit_store_bytes
from nxs.dsl.errors import CompileError
from nxs.dsl.fields import ParamDescriptor, Port, PortDirection, PortType
from nxs.dsl.register import RegisterClickPersonality
from nxs.dsl.values import RunValue

#: The highest value a `select` table reaches: the op's `len` byte holds
#: twice the slot count.
SELECT_MAX_VALUE = 126

#: The run parameters the apply block reads, by the names the trailer's
#: `RUN_PARAMS` record carries them under (`nxs.personality.records`).
FRAME_LENGTH_PARAM = "frame_length"
EXPOSURE_PARAM = "exposure"
GAIN_PARAM = "gain"
MODE_PARAM = "mode"
#: The longest frame period a run stages, in ns (the parameter's u32).
PERIOD_MAX_NS = 4_000_000_000


class CamPersonality(RegisterClickPersonality):
    """Base class for a cam personality: `probe()` is the alive or
    identity check and `configure()` the init program with `select()`
    blocks per mode; there is no measure loop. Compiles to a CAMERA-kind
    image the unit runs once per RUN commit on the pod bus, then halts.

    A camera class carries behaviour only (writes, bursts, sleeps, polls).
    The laws (exposure, frame rate, bandwidth) live in the law families
    beside the cam personalities (`nxs.cam.laws`), parameterized by the
    datasheet YAML, because the robot holds only the compiled image; a
    host-side hook on the class is refused.

    `NATIVE = True` marks the behaviour class of a native head, a device that
    programs its own sensor and serves the camera map alone. The class carries
    no `probe()` or `configure()`: `compile()` declares `mode`, `trigger`,
    `action`, `line_time` and `frame_period` from the facts beside the class
    and emits the halt alone, so the image is its records.

    A table-only head whose class declares no `action` and stages no apply
    gets one from its facts' stream gate (`controls.standby`): `compile()`
    runs the class's configure as the configure, the gate's standby value as
    the park and its streaming value as the start, so a walk starts, gates
    and parks the head by its own program (`_configure_gated`)."""

    IMAGE_KIND = ImageKind.CAMERA
    BUSES = ('i2c',)
    #: A camera head's one output, which a pod's head port takes.
    PORTS = (Port("out", PortType.HEAD, PortDirection.OUTPUT),)
    #: A native head: the facts declare the run parameters, the program is the halt.
    NATIVE = False

    # Attribute names of the host-side law surface the hub modules carry;
    # any of them on a camera class is a CompileError.
    HOST_HOOK_PREFIXES = ('knob_', 'expect_', 'derive_', 'export_')
    HOST_HOOK_NAMES = frozenset({
        'measure', 'start', 'stop', 'restart_stream', 'default_mode',
        'default_vmax', 'fps_ceiling', 'fps_floor', 'vmax_for_fps',
        'validate_vmax', 'line_time_us', 'sync_capability', 'descriptor',
        'shs_floor', 'shs_for_exposure_us', 'frame_length_delta_formula',
    })

    def _validate_kind(self) -> None:
        cls = type(self)
        if cls.NATIVE:
            for name in ("probe", "configure"):
                if hasattr(cls, name):
                    raise CompileError(
                        f"{cls.__name__}: a native head runs its own logic, so {name}() has "
                        f"nothing to run on it; its run parameters come from the facts")
        else:
            self._refuse_native_facts()
        for name in dir(cls):
            fn = getattr(cls, name, None)
            if callable(fn) and getattr(fn, "_measure_loop", False):
                raise CompileError(
                    f"{cls.__name__}: a cam personality has no measure "
                    f"loop; remove @measure_loop from {name}()")
            if name.startswith('_'):
                continue
            if name in self.HOST_HOOK_NAMES or name.startswith(self.HOST_HOOK_PREFIXES):
                if name == 'measure':
                    raise CompileError(
                        f"{cls.__name__}: a cam personality has no "
                        f"measure(); probe() and configure() are its whole "
                        f"program")
                raise CompileError(
                    f"{cls.__name__}: {name} is a host-side hook; a camera "
                    f"personality carries behaviour only (probe, configure, "
                    f"the blocks they call). Laws live in the law families "
                    f"beside the cam personalities, parameterized by the "
                    f"datasheet YAML.")

    def _facts_path(self) -> Optional[Path]:
        """`<name>.yaml` beside the class's module, the pair's facts; None
        for a class with no module file."""
        path = getattr(sys.modules.get(type(self).__module__), "__file__", None)

        return Path(path).with_suffix(".yaml") if path else None

    def _refuse_native_facts(self) -> None:
        """Refuse a program over facts of the `native` family: such a head
        programs its own sensor, and an image of its class would serve a
        camera map with no apply value to stage. A class compiled without
        facts beside it has nothing to judge."""
        from nxs.cam import laws

        yaml_path = self._facts_path()
        if yaml_path is None or not yaml_path.is_file():
            return
        try:
            facts = yaml.safe_load(yaml_path.read_text())
        except (yaml.YAMLError, OSError) as exc:
            raise CompileError(f"{yaml_path}: {exc}") from None
        meta = facts.get("meta") if isinstance(facts, dict) else None
        if isinstance(meta, dict) and str(meta.get("chip") or "") == laws.NATIVE:
            raise CompileError(
                f"{type(self).__name__}: {yaml_path.name} names chip: native, a head that "
                f"programs its own sensor; its class marks NATIVE = True and carries no "
                f"probe() or configure()")

    def _native_facts(self):
        """The facts of a native head: `<name>.yaml` beside the class's
        module, the pair's descriptor, of the `native` family."""
        from nxs.cam import laws

        cls = type(self)
        facts = self._pair_facts()
        if facts is None:
            raise CompileError(
                f"{cls.__name__}: a native head declares its run parameters from the "
                f"facts beside its module, and {self._facts_path() or cls.__module__} is "
                f"not there")
        yaml_path = self._facts_path()
        family = laws.family_for(facts)
        if family != laws.NATIVE:
            raise CompileError(
                f"{cls.__name__}: NATIVE = True marks a native head, and {yaml_path.name} names "
                f"the {family} family; a native head's facts declare chip: native")

        return facts

    def _pair_facts(self):
        """The facts beside the class's module as the pair's descriptor; None
        for a class with no facts file beside it."""
        from nxs.cam.contracts import ContractError
        from nxs.cam.descriptors import Descriptor

        yaml_path = self._facts_path()
        if yaml_path is None or not yaml_path.is_file():
            return None
        try:
            if yaml_path.parent.name == yaml_path.stem:
                return Descriptor(yaml_path.parent)
            return Descriptor.from_data(yaml.safe_load(yaml_path.read_text()), yaml_path.stem)
        except (ContractError, yaml.YAMLError, OSError) as exc:
            raise CompileError(f"{yaml_path}: {exc}") from None

    def _facts_stream_gate(self) -> Optional[Tuple[list, list]]:
        """The writes of a table-only part's stream gate, its standby value's
        and its streaming value's, as its family composes them from the facts
        beside the class; None for a part of another family or without a
        gate."""
        from nxs.cam import laws
        from nxs.cam.contracts import ContractError, InfeasibleConfig

        facts = self._pair_facts()
        if facts is None or laws.family_for(facts) != laws.GENERIC:
            return None
        try:
            surface = laws.bind(facts)
            if not hasattr(surface, "start"):
                return None
            return list(surface.stop()), list(surface.start())
        except (ContractError, InfeasibleConfig) as exc:
            raise CompileError(f"{facts.name}: {exc}") from None

    def _configure_gated(self, config: dict) -> None:
        """The program of a table-only part whose class declares no action
        and stages no apply: the `action` parameter over every action, the
        class's configure as the configure, the stream gate's standby value
        as the park and its streaming value as the start. The part takes no
        timing, apply or trigger, so those actions write nothing."""
        from nxs.personality.records_fields import ACTIONS, ACTION_PARAM

        configure, park, start, timing, apply, trigger = (
            ACTIONS[a] for a in ("configure", "park", "start", "timing", "apply", "trigger"))
        self.declare_param(ACTION_PARAM, values=sorted(ACTIONS.values()),
                           default=configure, kind="run")
        self.select_grouped(ACTION_PARAM, {
            frozenset({configure}): lambda: self.configure(config),
            frozenset({park}): lambda: self.stream_gate(False),
            frozenset({start}): lambda: self.stream_gate(True),
            frozenset({timing, apply, trigger}): lambda: None})

    def stream_gate(self, on: bool) -> None:
        """Write a table-only part's stream gate, the `controls.standby` row
        of the facts beside the class: its streaming value when `on`, else
        its standby value, one register per write as the family composes
        the gate. Call in configure(), in the start and park arms of the
        `action` dispatch."""
        gate = self._facts_stream_gate()
        if gate is None:
            raise CompileError(
                f"{type(self).__name__}: stream_gate() writes the table-only part's gate, and "
                f"the facts beside the class declare none (controls.standby with its "
                f"standby and streaming values)")
        for step in gate[1] if on else gate[0]:
            reg = int(str(step["reg"]), 0) << 8 | int(str(step["offset"]), 0)
            for value in step["value"] if isinstance(step["value"], list) else [step["value"]]:
                self.write(reg, int(str(value), 0))

    def _declare_native_params(self) -> None:
        """A native head's run parameters, from its facts: `mode` over the
        unit modes, `trigger` over the declared presets (none without a
        `trigger:` block), `action` over every action, and the `line_time`
        and `frame_period` ranges the mode rows span, in nanoseconds. Each
        is marked used: the CAMERA_MAP record indexes them, the mode and
        trigger rows follow from them, and no bytecode reads them."""
        from nxs.cam import laws
        from nxs.cam.contracts import ContractError, InfeasibleConfig
        from nxs.cam.descriptors import FREERUN
        from nxs.personality.records_fields import (
            ACTIONS, ACTION_PARAM, FRAME_PERIOD_PARAM, LINE_TIME_PARAM, TRIGGER_PARAM)
        from nxs.personality.records_sensor import trigger_values

        facts = self._native_facts()
        try:
            surface = laws.bind(facts)
            if not hasattr(surface, "apply_law"):
                native = laws.family(laws.NATIVE)
                raise CompileError(
                    f"{facts.name}: a native head's facts state the camera map's bounds "
                    f"(limits.{', '.join(native.LAW_LIMITS)} and each mode's "
                    f"timing.{' and '.join(native.MODE_TIMING)})")
            law = surface.apply_law()
        except (ContractError, InfeasibleConfig) as exc:
            raise CompileError(f"{facts.name}: {exc}") from None
        modes = facts.program_modes()
        default = modes.index(str(facts.raw("default_mode")))
        self.declare_param(MODE_PARAM, values=list(range(len(modes))), default=default,
                           kind="run")
        triggers = trigger_values(facts)
        if triggers:
            self.declare_param(TRIGGER_PARAM, values=sorted(triggers.values()),
                               default=triggers.get(FREERUN, min(triggers.values())),
                               kind="run")
        self.declare_param(ACTION_PARAM, values=sorted(ACTIONS.values()),
                           default=ACTIONS["configure"], kind="run")
        lines = law.line_times_ns
        periods = [us * 1000 for us in law.frame_periods_min_us]
        longest = max(max(periods), min(law.frame_period_max_us * 1000, PERIOD_MAX_NS))
        self.declare_param(LINE_TIME_PARAM, values=[min(lines), max(lines)],
                           default=lines[default], param_type="range", unit="ns", kind="apply")
        self.declare_param(FRAME_PERIOD_PARAM, values=[min(periods), longest],
                           default=periods[default], param_type="range", unit="ns",
                           kind="apply")
        self._params_used.update(self._params)

    def param(self, name: str) -> RunValue:
        """The value staged for the declared parameter `name` in this run,
        as a run value: `PARAM_LOAD` when a verb emits it. Its interval is
        the declared range (an enum's lowest to highest value)."""
        p, index = self._run_param(name, "param")

        return RunValue.of_param(index, name, *self._param_bounds(p))

    def write_wide(self, reg: int, value: Any, width: int,
                   byte_order: str = "big", dev=None) -> None:
        """Write the low `width` bytes of a run value to the consecutive
        registers from `reg`, staged through sample_buf then
        `REG_WRITE_BURST`; `dev=` targets a declared I2C companion. A value
        whose interval can exceed the width is refused."""
        if self.FRAME is not None:
            raise CompileError(
                "write_wide with a FRAME schema is not supported")
        if width not in (1, 2, 3, 4):
            raise CompileError(
                f"write_wide(0x{reg:X}): width must be 1..4 bytes, got {width}")
        if byte_order not in ("big", "little"):
            raise CompileError(
                f"write_wide(0x{reg:X}): byte_order must be 'big' or "
                f"'little', got {byte_order!r}")
        if not isinstance(value, RunValue):
            value = RunValue.of_const(value, f"write_wide(0x{reg:X})")
        if value.hi >= 1 << (8 * width):
            raise CompileError(
                f"write_wide(0x{reg:X}): {value} can reach {value.hi}, past "
                f"the {width}-byte register")
        addr = self._companion_addr(dev) if dev is not None else None
        em = self._emitter
        with self._regs.scope():
            src = value.emit(em, self._regs, self._fresh_label)
            _emit_store_bytes(em, self._regs, src, 0, width, byte_order)
            if addr is not None:
                em.emit(Op.I2C_TARGET, addr)
            em.emit_reg(Op.REG_WRITE_BURST, reg, 0, width)
            if addr is not None:
                em.emit(Op.I2C_TARGET, 0)

    def store_param(self, name: str, value: Any) -> None:
        """Record a run value as parameter `name`: the value the host reads
        back for it once the run ends. What a run achieved may sit outside
        the staged range by its rounding, so the interval is not checked
        against the declaration."""
        _p, index = self._run_param(name, "store_param")
        if not isinstance(value, RunValue):
            value = RunValue.of_const(value, f"store_param({name!r})")
        em = self._emitter
        with self._regs.scope():
            src = value.emit(em, self._regs, self._fresh_label)
            em.emit(Op.PARAM_STORE, index, src)

    def sleep_ms(self, ms: int) -> None:
        """One `SLEEP_MS`: the thread that runs a camera program slices a
        wait at its abort latency, so the unroll a click runner needs to
        observe a stop buys a camera program nothing but bytes."""
        if ms > self.SLEEP_MAX_UNROLLED_MS:
            raise CompileError(
                f"sleep_ms({ms}) exceeds the {self.SLEEP_MAX_UNROLLED_MS} ms cap: a settle "
                f"longer than a second is a table's job, split across its rows")
        self._emitter.emit_u16(Op.SLEEP_MS, ms)

    def apply_block(self, law: ApplyLaw) -> None:
        """The `apply` action: the staged `frame_length`, `exposure` and `gain`
        run parameters written to the sensor under its group hold, each clamped
        to the law. The frame length and the exposure written are stored back
        for the host, and the gain as staged, in capture units, which the
        camera map serves in millidecibels. The exposure is held the law's
        margin under the frame written, floored for the running mode. On an
        inverted-shutter part the shutter register takes the frame length less
        the exposure lines; a part whose exposure register counts lines takes
        them as they are. The gain register takes the staged value scaled by
        the law's ratio, capped at the register's maximum."""
        if self._trace_phase != "configure":
            raise CompileError("apply_block() belongs in configure()")
        em = self._emitter
        uid = self._fresh_label("apply")
        with self._regs.scope():
            frame = self.param(FRAME_LENGTH_PARAM)
            vmax = frame.emit(em, self._regs, self._fresh_label)
            floors = law.frame_length_floors
            if len(set(floors)) > 1:
                # Each mode keeps the frame its datasheet row guarantees:
                # the floor is the running mode's, dispatched on `mode`.
                self._emit_clamp(vmax, 0, law.frame_length_max, f"{uid}_vmax", frame)
                self.select_each(MODE_PARAM, {
                    value: (lambda floor=floor, value=value: self._emit_clamp(
                        vmax, floor, None, f"{uid}_vmax_m{value}", frame))
                    for value, floor in enumerate(floors)})
            else:
                self._emit_clamp(vmax, law.frame_length_min, law.frame_length_max,
                                 f"{uid}_vmax", frame)
            exposure = self.param(EXPOSURE_PARAM)
            lines = exposure.emit(em, self._regs, self._fresh_label)
            self._emit_clamp(lines, law.exposure_min, None, f"{uid}_lines", exposure)
            # The frame keeps the margin above the exposure whichever way the
            # shutter counts: the ceiling is the floored frame less the margin.
            if law.exposure_margin:
                ceiling = self._regs.get(f"__apply_ceiling_{uid}")
                em.emit_u32(Op.LOAD_IMM, ceiling, law.exposure_margin)
                em.emit(Op.SUB_REG, vmax, ceiling, ceiling)
            else:
                ceiling = vmax
            self._emit_min(lines, ceiling, f"{uid}_ceiling")
            if law.shutter_inverted:
                # The shutter counts from the frame's end, and the margin is at least the shutter
                # floor, so the shutter the subtraction leaves never needs flooring.
                shutter = self._regs.get(f"__apply_shutter_{uid}")
                em.emit(Op.SUB_REG, vmax, lines, shutter)
            else:
                shutter = lines
            scaled = self.param(GAIN_PARAM)
            if law.gain_num != law.gain_den:
                scaled = scaled * law.gain_num
                if law.gain_den != 1:
                    scaled = scaled // law.gain_den
            gain = scaled.emit(em, self._regs, self._fresh_label)
            self._emit_clamp(gain, 0, law.gain_reg_max, f"{uid}_gain", scaled)
            if law.hold is not None:
                self.write(law.hold.addr, 1)
            self._emit_write_reg(law.frame_length, vmax)
            self._emit_write_reg(law.shutter, shutter)
            self._emit_write_reg(law.gain, gain)
            if law.hold is not None:
                self.write(law.hold.addr, 0)
            self._emit_store(FRAME_LENGTH_PARAM, vmax)
            self._emit_store(EXPOSURE_PARAM, lines)

    def _emit_clamp(self, reg: int, lo: int, hi: Optional[int], uid: str,
                    value: Optional[RunValue] = None) -> None:
        """Clamp r[reg] into `lo..hi` in place, `hi` None for no ceiling.
        A bound the value's own interval already respects emits nothing.
        The compares are signed, which the declared ranges keep below
        2^31."""
        if value is not None:
            if value.lo >= lo:
                lo = 0
            if hi is not None and value.hi <= hi:
                hi = None
        em = self._emitter
        with self._regs.scope():
            flag = self._regs.get(f"__clamp_{uid}")
            if lo > 0:
                em.emit_cmp(Op.CMP_LT, reg, lo, flag)
                em.emit_jmp(Op.JZ, f"{uid}_lo", flag)
                em.emit_u32(Op.LOAD_IMM, reg, lo)
                em.label(f"{uid}_lo")
            if hi is not None:
                em.emit_cmp(Op.CMP_LT, reg, hi + 1, flag)
                em.emit_jmp(Op.JNZ, f"{uid}_hi", flag)
                em.emit_u32(Op.LOAD_IMM, reg, hi)
                em.label(f"{uid}_hi")

    def _emit_min(self, reg: int, bound: int, uid: str) -> None:
        """r[reg] = min(r[reg], r[bound]), both below 2^31: the difference
        read as a signed value says which is smaller."""
        em = self._emitter
        with self._regs.scope():
            diff = self._regs.get(f"__min_{uid}")
            em.emit(Op.SUB_REG, reg, bound, diff)
            em.emit_cmp(Op.CMP_LT, diff, 0, diff)
            em.emit_jmp(Op.JNZ, f"{uid}_min", diff)
            em.emit(Op.MOV, reg, bound)
            em.label(f"{uid}_min")

    def _emit_write_reg(self, spec: RegSpec, reg: int) -> None:
        """Write the low `spec.width` bytes of r[reg] to the register in
        its byte order, staged through sample_buf then a burst."""
        em = self._emitter
        _emit_store_bytes(em, self._regs, reg, 0, spec.width, "little" if spec.little else "big")
        em.emit_reg(Op.REG_WRITE_BURST, spec.addr, 0, spec.width)

    def _emit_store(self, name: str, reg: int) -> None:
        """Record r[reg] as run parameter `name`."""
        _p, index = self._run_param(name, "apply_block")
        self._emitter.emit(Op.PARAM_STORE, index, reg)

    def _run_param(self, name: str, verb: str) -> Tuple[ParamDescriptor, int]:
        """The declared parameter and its file index, marked as used by the
        program."""
        p = self._params.get(name)
        if p is None:
            raise CompileError(
                f"{verb}({name!r}): parameter not declared — call "
                f"declare_param({name!r}, ...) first")
        index = list(self._params).index(name)
        if index >= NxsDriverImage.MAX_PARAMS:
            raise CompileError(
                f"{verb}({name!r}): parameter {index} is past the "
                f"{NxsDriverImage.MAX_PARAMS}-entry run parameter file")
        self._params_used.add(name)

        return p, index

    @staticmethod
    def _param_bounds(p: ParamDescriptor) -> Tuple[int, int]:
        if p.param_type == "range":
            return int(p.values[0]), int(p.values[1])

        return int(min(p.values)), int(max(p.values))

    def check(self, reg: int, value: int, mask: int = 0xFF, dev=None) -> None:
        """Read the byte at `reg` once and fault the run with `MISMATCH`
        unless `(read & mask) == value`: a write's read-back, or a register
        that must already hold its value. `dev` names a declared companion."""
        if self._trace_phase not in ("probe", "configure"):
            raise CompileError("check() belongs in probe() or configure()")
        for name, operand in (("mask", mask), ("value", value)):
            if not isinstance(operand, int) or not 0 <= operand <= 0xFF:
                raise CompileError(f"check: {name} 0x{operand:X} does not fit a byte")
        em = self._emitter
        addr = self._companion_addr(dev) if dev is not None else None
        ok = self._fresh_label("check_ok")
        with self._regs.scope():
            got = self._regs.get(f"__check_{ok}")
            if addr is not None:
                em.emit(Op.I2C_TARGET, addr)
            em.emit_reg(Op.REG_READ, reg, got)
            if addr is not None:
                em.emit(Op.I2C_TARGET, 0)
            em.emit_cmp(Op.AND, got, mask, got)
            em.emit_cmp(Op.CMP_EQ, got, value, got)
            em.emit_jmp(Op.JNZ, ok, got)
            em.emit(Op.ERROR, OpErrorCode.MISMATCH)
            em.label(ok)

    def retry(self, times: int, delay_ms: int, block: Callable[[], None],
              until: Tuple[int, int, int], timeout_ms: int, poll_ms: int = 20) -> None:
        """Run `block`, then wait up to `timeout_ms` for `until`, a
        `(reg, mask, value)` condition on a byte register; when the wait
        runs out, wait `delay_ms` and run the block again, `times` runs
        in all. The last run's wait is hard: the run faults naming the
        register when the condition never holds. The shape of a one-shot
        that has to re-lock before the next write."""
        if self._trace_phase != "configure":
            raise CompileError("retry() belongs in configure()")
        if not isinstance(times, int) or not 1 <= times <= 255:
            raise CompileError(f"retry: times must be 1..255, got {times!r}")
        if not callable(block):
            raise CompileError("retry: block must be callable")
        reg, mask, value = until
        if self._i2c_profile().data_width != 1:
            raise CompileError(
                "retry: the condition reads one byte; the profile declares "
                f"{self._i2c_profile().data_width}-byte values")
        for name, operand in (("mask", mask), ("value", value)):
            if not isinstance(operand, int) or not 0 <= operand <= 0xFF:
                raise CompileError(f"retry: {name} 0x{operand:X} does not fit a byte")
        em = self._emitter
        uid = self._fresh_label("retry")
        loop, last, done = f"{uid}_loop", f"{uid}_last", f"{uid}_done"
        with self._regs.scope():
            left = self._regs.get(f"__retry_left_{uid}")
            got = self._regs.get(f"__retry_got_{uid}")
            em.emit_u32(Op.LOAD_IMM, left, times - 1)
            em.label(loop)
            block()
            em.emit_cmp(Op.CMP_EQ, left, 0, got)
            em.emit_jmp(Op.JNZ, last, got)
            self.poll(reg, mask, value, timeout_ms, poll_ms, soft=True)
            em.emit_reg(Op.REG_READ, reg, got)
            em.emit_cmp(Op.AND, got, mask, got)
            em.emit_cmp(Op.CMP_EQ, got, value, got)
            em.emit_jmp(Op.JNZ, done, got)
            em.emit_cmp(Op.SUB, left, 1, left)
            if delay_ms:
                self.sleep_ms(delay_ms)
            em.emit_jmp(Op.JMP, loop)
            em.label(last)
            self.poll(reg, mask, value, timeout_ms, poll_ms)
            em.label(done)

    def select(self, param_name: str, blocks: Dict[Any, Callable[[], None]]) -> None:
        """Dispatch on a declared param: `PARAM_LOAD` of the value staged
        for the run, one `SELECT` into the blocks, each block ending in a
        jump to the join point. The param is an enum of run kind, or a
        range whose every value takes a block, a dispatch past the 16
        values an enum lists. `blocks` maps every declared value to a
        callable that emits that value's program."""
        if self._trace_phase != "configure":
            raise CompileError(
                f"select({param_name!r}) belongs in configure()")
        declared = self._dispatch_values("select", param_name)
        given = list(blocks)
        if sorted(given) != sorted(declared) or len(given) != len(declared):
            raise CompileError(
                f"select({param_name!r}): blocks {sorted(given)} must cover "
                f"the declared values {sorted(declared)} exactly")
        for value, block in blocks.items():
            if not callable(block):
                raise CompileError(
                    f"select({param_name!r}): the block for {value!r} is not "
                    f"callable")

        em = self._emitter
        _p, index = self._run_param(param_name, "select")
        uid = self._fresh_label(f"sel_{param_name}")
        labels = {value: f"{uid}_v{i}" for i, value in enumerate(declared)}
        join = f"{uid}_join"

        dispatch = f"{param_name} dispatch"
        chain_start = em._current_offset()
        with self._regs.scope():
            sel = self._regs.get(f"__sel_{uid}")
            em.emit(Op.PARAM_LOAD, sel, index)
            self._emit_table(param_name, uid, {int(v): labels[v] for v in declared}, sel)
            self._budget_spans.append((dispatch, chain_start, em._current_offset()))
            for value in declared:
                em.label(labels[value])
                start = em._current_offset()
                blocks[value]()
                self._budget_spans.append(
                    (f"{param_name}={value} block", start, em._current_offset()))
                jmp_start = em._current_offset()
                em.emit_jmp(Op.JMP, join)
                self._budget_spans.append((dispatch, jmp_start, em._current_offset()))
            em.label(join)

    def _dispatch_values(self, verb: str, param_name: str) -> list:
        """The values a dispatch on `param_name` covers: an enum run param's
        declared values, or every value of a range param from its low end
        to its high end, which the jump table reaches up to
        `SELECT_MAX_VALUE`."""
        param = self._params.get(param_name)
        if param is None:
            raise CompileError(
                f"{verb}({param_name!r}): parameter not declared — call "
                f"declare_param({param_name!r}, ...) first")
        if param.param_type == "range":
            lo, hi = self._param_bounds(param)
            if hi > SELECT_MAX_VALUE:
                raise CompileError(
                    f"{verb}({param_name!r}): the values must lie in 0..{SELECT_MAX_VALUE}, "
                    f"and the range spans {lo}..{hi}")
            return list(range(lo, hi + 1))
        if param.kind != "run":
            raise CompileError(
                f"{verb}({param_name!r}): the param must be an enum run param or a "
                f"range param; a mode switch re-runs configure()")

        return list(param.values)

    def _emit_table(self, param_name: str, uid: str, arms: Dict[int, str], sel: int) -> None:
        """The dispatch as one `SELECT` over the values 0 to the highest
        declared: each declared value's slot names its arm, a gap's slot
        the MISMATCH arm, and a value past the table faults in the op."""
        top = max(arms)
        if min(arms) < 0 or top > SELECT_MAX_VALUE:
            raise CompileError(
                f"select({param_name!r}): the values must lie in 0..{SELECT_MAX_VALUE}, "
                f"got {sorted(arms)}")
        em = self._emitter
        miss = f"{uid}_miss"
        slots = [arms.get(value, miss) for value in range(top + 1)]
        em.emit_select(sel, slots)
        if miss in slots:
            # A declared gap: a foreign patch value faults rather than
            # skipping the delta silently.
            em.label(miss)
            em.emit(Op.ERROR, OpErrorCode.MISMATCH)

    def select_grouped(self, param_name: str, groups: Dict[frozenset, Callable[[], None]]) -> None:
        """Dispatch on a declared param, as `select` takes one, with one
        block per group of values: the table sends every value of a group
        to the same block, so a block shared by several values is emitted
        once. The groups must cover the declared values exactly."""
        if self._trace_phase != "configure":
            raise CompileError(f"select_grouped({param_name!r}) belongs in configure()")
        declared = sorted(int(v) for v in self._dispatch_values("select_grouped", param_name))
        covered = sorted(int(v) for group in groups for v in group)
        if covered != declared:
            raise CompileError(f"select_grouped({param_name!r}): groups {covered} must cover "
                               f"the declared values {declared} exactly")
        em = self._emitter
        _p, index = self._run_param(param_name, "select_grouped")
        uid = self._fresh_label(f"selg_{param_name}")
        labels = {group: f"{uid}_g{i}" for i, group in enumerate(groups)}
        join = f"{uid}_join"
        # The table's register is released before the blocks emit, so a
        # dispatch nested in a block costs the block nothing.
        with self._regs.scope():
            sel = self._regs.get(f"__selg_{uid}")
            em.emit(Op.PARAM_LOAD, sel, index)
            self._emit_table(param_name, uid,
                             {int(value): label for group, label in labels.items()
                              for value in group}, sel)
        for group, block in groups.items():
            em.label(labels[group])
            start = em._current_offset()
            block()
            self._budget_spans.append(
                (f"{param_name} in {sorted(group)} block", start, em._current_offset()))
            em.emit_jmp(Op.JMP, join)
        em.label(join)

    def select_each(self, param_name: str, blocks: Dict[Any, Callable[[], None]]) -> None:
        """`select` with the table's register released before the blocks:
        one block per declared value."""
        self.select_grouped(param_name, {frozenset({value}): block
                                         for value, block in blocks.items()})
