"""Camera personalities: a probe, a configure with one block per mode, no measure loop."""

from __future__ import annotations

from typing import Any, Callable, Dict, Tuple

from nxs._generated_constants import NxsDriverImage
from nxs.opcodes import Op
from nxs.dsl.compiled import ImageKind
from nxs.dsl.emit import OpErrorCode, _emit_store_bytes
from nxs.dsl.errors import CompileError
from nxs.dsl.fields import ParamDescriptor
from nxs.dsl.register import RegisterDriver
from nxs.dsl.values import RunValue


class CameraSensor(RegisterDriver):
    """Base class for a camera personality: `probe()` is the alive or
    identity check and `configure()` the init program with `select()`
    blocks per mode; there is no measure loop. Compiles to a CAMERA-kind
    image the unit runs once per `CAM_RUN` on the pod bus, then halts.

    A camera class carries behaviour only (writes, bursts, sleeps, polls).
    The laws (exposure, frame rate, bandwidth) live in the tool's law
    families, parameterized by the datasheet YAML, because the robot holds
    only the compiled image; a host-side hook on the class is refused."""

    IMAGE_KIND = ImageKind.CAMERA
    BUSES = ('i2c',)

    # Attribute names of the host-side law surface the pack modules carry;
    # any of them on a camera class is a CompileError.
    HOST_HOOK_PREFIXES = ('knob_', 'expect_', 'derive_', 'export_')
    HOST_HOOK_NAMES = frozenset({
        'measure', 'start', 'stop', 'restart_stream', 'default_mode',
        'default_vmax', 'fps_ceiling', 'fps_floor', 'vmax_for_fps',
        'validate_vmax', 'line_time_us', 'sync_capability', 'descriptor',
        'shs_floor', 'shs_for_exposure_us', 'frame_length_delta_formula',
        'rows_delivered',
    })

    def _validate_kind(self) -> None:
        cls = type(self)
        for name in dir(cls):
            fn = getattr(cls, name, None)
            if callable(fn) and getattr(fn, "_measure_loop", False):
                raise CompileError(
                    f"{cls.__name__}: a camera personality has no measure "
                    f"loop; remove @measure_loop from {name}()")
            if name.startswith('_'):
                continue
            if name in self.HOST_HOOK_NAMES or name.startswith(self.HOST_HOOK_PREFIXES):
                if name == 'measure':
                    raise CompileError(
                        f"{cls.__name__}: a camera personality has no "
                        f"measure(); probe() and configure() are its whole "
                        f"program")
                raise CompileError(
                    f"{cls.__name__}: {name} is a host-side hook; a camera "
                    f"personality carries behaviour only (probe, configure, "
                    f"the blocks they call). Laws live in the tool's law "
                    f"families, parameterized by the datasheet YAML.")

    def param(self, name: str) -> RunValue:
        """The value staged for the declared parameter `name` in this run,
        as a run value: `PARAM_LOAD` when a verb emits it. Its interval is
        the declared range (an enum's lowest to highest value)."""
        p, index = self._run_param(name, "param")

        return RunValue.of_param(index, name, *self._param_bounds(p))

    def write_wide(self, reg: int, value: Any, width: int,
                   byte_order: str = "big") -> None:
        """Write the low `width` bytes of a run value to the consecutive
        registers from `reg`, staged through sample_buf then
        `REG_WRITE_BURST`. A value whose interval can exceed the width is
        refused."""
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
        em = self._emitter
        with self._regs.scope():
            src = value.emit(em, self._regs, self._fresh_label)
            _emit_store_bytes(em, self._regs, src, 0, width, byte_order)
            em.emit_reg(Op.REG_WRITE_BURST, reg, 0, width)

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
        """Dispatch on a declared enum reload param: `PARAM_LOAD` of the
        value staged for the run, a `CMP_EQ`/`JNZ` chain into the blocks,
        each block ending in a jump to the join point. `blocks` maps every
        declared value to a callable that emits that value's program."""
        if self._trace_phase != "configure":
            raise CompileError(
                f"select({param_name!r}) belongs in configure()")
        param = self._params.get(param_name)
        if param is None:
            raise CompileError(
                f"select({param_name!r}): parameter not declared — call "
                f"declare_param({param_name!r}, ...) first")
        if param.param_type != "enum" or param.kind != "reload":
            raise CompileError(
                f"select({param_name!r}): the param must be an enum reload "
                f"param; a mode switch re-runs configure()")
        declared = list(param.values)
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
            eq = self._regs.get(f"__sel_eq_{uid}")
            em.emit(Op.PARAM_LOAD, sel, index)
            for value in declared:
                em.emit_cmp(Op.CMP_EQ, sel, int(value) & 0xFFFFFFFF, eq)
                em.emit_jmp(Op.JNZ, labels[value], eq)
            # Unreachable once every declared value has a block; a foreign
            # patch value faults rather than skipping the delta silently.
            em.emit(Op.ERROR, OpErrorCode.MISMATCH)
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

    def select_grouped(self, param_name: str, groups: Dict[frozenset, Callable[[], None]]) -> None:
        """Dispatch on a declared enum reload param with one block per group
        of values: the dispatch chain jumps every value of a group to the
        same block, so a block shared by several values is emitted once.
        The groups must cover the declared values exactly."""
        if self._trace_phase != "configure":
            raise CompileError(f"select_grouped({param_name!r}) belongs in configure()")
        param = self._params.get(param_name)
        if param is None:
            raise CompileError(f"select_grouped({param_name!r}): parameter not declared")
        if param.param_type != "enum" or param.kind != "reload":
            raise CompileError(f"select_grouped({param_name!r}): the param must be an enum "
                               f"reload param")
        declared = sorted(int(v) for v in param.values)
        covered = sorted(int(v) for group in groups for v in group)
        if covered != declared:
            raise CompileError(f"select_grouped({param_name!r}): groups {covered} must cover "
                               f"the declared values {declared} exactly")
        em = self._emitter
        _p, index = self._run_param(param_name, "select_grouped")
        uid = self._fresh_label(f"selg_{param_name}")
        labels = {group: f"{uid}_g{i}" for i, group in enumerate(groups)}
        join = f"{uid}_join"
        # The chain's two registers are released before the blocks emit, so
        # a dispatch nested in a block costs the block nothing.
        with self._regs.scope():
            sel = self._regs.get(f"__selg_{uid}")
            eq = self._regs.get(f"__selg_eq_{uid}")
            em.emit(Op.PARAM_LOAD, sel, index)
            for group, label in labels.items():
                for value in sorted(group):
                    em.emit_cmp(Op.CMP_EQ, sel, int(value) & 0xFFFFFFFF, eq)
                    em.emit_jmp(Op.JNZ, label, eq)
            em.emit(Op.ERROR, OpErrorCode.MISMATCH)
        for group, block in groups.items():
            em.label(labels[group])
            start = em._current_offset()
            block()
            self._budget_spans.append(
                (f"{param_name} in {sorted(group)} block", start, em._current_offset()))
            em.emit_jmp(Op.JMP, join)
        em.label(join)

    def select_each(self, param_name: str, blocks: Dict[Any, Callable[[], None]]) -> None:
        """`select` with the chain's registers released before the blocks:
        one block per declared value."""
        self.select_grouped(param_name, {frozenset({value}): block
                                         for value, block in blocks.items()})
