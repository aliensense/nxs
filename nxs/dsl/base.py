"""The personality base class: the declarations a personality makes in probe() and configure()."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

from nxs._generated_constants import Calibration, FieldSemantics, NxsMap
from nxs.opcodes import Op
from nxs.profiles import I2cProfile
from nxs.dsl.compile import _CompilePhase
from nxs.dsl.compiled import HOST_KINDS, ImageKind
from nxs.dsl.emit import WideRef, _Emitter, _RegAlloc, _TraceReadValue, _WorkAlloc, _load_spec_byte
from nxs.dsl.errors import CompileError
from nxs.dsl.fields import PWM_FREQ_MAX_HZ, PWM_FREQ_MIN_HZ, ParamDescriptor, PatchEntry, Port, PortDirection, PortType, REG_ADDR_MAX_16, REG_ADDR_MAX_8, Show, infer_semantic, resolve_field_offsets, validate_field_layout
from nxs.dsl.loop import _ASTCompiler


ParamId = NxsMap.ParamId
#: A standard id by its name, the spelling a facts file takes.
_PARAM_IDS = {name: number for number, name in ParamId._NAMES.items()}
#: The values a row of a standard id takes, for the ids whose values the
#: generated registry names: the mount rotation codes, the trigger
#: conversions and the flip bits.
_ID_VALUES = {ParamId.MOUNT_ORIENTATION: frozenset(Calibration.Rotation._NAMES),
              ParamId.TRIGGER: frozenset(NxsMap.TriggerCode._NAMES),
              ParamId.FLIPS: frozenset(range((NxsMap.FLIP_X | NxsMap.FLIP_Y) + 1))}


def _param_id_named(param: str, name: Optional[str]) -> int:
    """The standard id a facts file names, `CUSTOM` where it names none."""
    if name is None:
        return ParamId.CUSTOM
    if name not in _PARAM_IDS:
        raise CompileError(f"params[{param}]: id {name!r} is no ParamId; the ids are "
                           f"{', '.join(_PARAM_IDS)}")
    return _PARAM_IDS[name]


def _standard_id(verb: str, name: str, pid: Any, values: list, param_type: str) -> int:
    """A declared row's standard id: one the registry names, never the
    stream row's, which the compiler adds, over values the registry gives
    the id where it names them (`_ID_VALUES`)."""
    if isinstance(pid, bool) or not isinstance(pid, int) or pid not in ParamId._NAMES:
        raise CompileError(f"{verb}({name!r}): id {pid!r} is no ParamId")
    if pid == ParamId.STREAM:
        raise CompileError(f"{verb}({name!r}): STREAM is the id of the row the compiler adds")
    allowed = _ID_VALUES.get(int(pid))
    declared = range(values[0], values[1] + 1) if param_type == "range" else values
    if allowed is not None and (len(declared) > len(allowed)
                                or any(value not in allowed for value in declared)):
        raise CompileError(f"{verb}({name!r}): a {ParamId._NAMES[pid]} row takes "
                           f"{min(allowed)}..{max(allowed)}, not {values!r}")
    return int(pid)


def _show(name: str, show: Optional[Show]) -> Optional[Show]:
    """A declared display: a unit of `ROW_UNIT_LEN` printable ASCII
    characters at most and an exponent of a signed byte."""
    if show is None:
        return None
    if not isinstance(show, Show):
        raise CompileError(f"declare_param({name!r}): show takes a Show(unit, exponent), "
                           f"not {show!r}")
    if not _printable(show.unit, NxsMap.ROW_UNIT_LEN, allow_empty=True):
        raise CompileError(f"declare_param({name!r}): show unit {show.unit!r} is not 0 to "
                           f"{NxsMap.ROW_UNIT_LEN} printable ASCII characters")
    if isinstance(show.exponent, bool) or not isinstance(show.exponent, int) \
            or not -128 <= show.exponent <= 127:
        raise CompileError(f"declare_param({name!r}): show exponent {show.exponent!r} is no "
                           f"whole number from -128 to 127")
    return show


def _value_names(name: str, values: list, param_type: str,
                 labels: Optional[Sequence[str]]) -> Optional[Tuple[str, ...]]:
    """A declared name per value: as many as an enum's values, or a range's
    `max - min + 1`, each 1 to `LABEL_CHARS_MAX` printable ASCII characters
    and no two alike."""
    if labels is None:
        return None
    if isinstance(labels, str) or not all(isinstance(label, str) for label in labels):
        raise CompileError(f"declare_param({name!r}): labels take a list of names")
    names = tuple(labels)
    count = len(values) if param_type == "enum" else values[1] - values[0] + 1
    if len(names) != count:
        raise CompileError(
            f"declare_param({name!r}): {len(names)} labels for {count} values; a label names "
            f"each value, an enum's in its order and a range's from its min")
    for label in names:
        if not _printable(label, NxsMap.LABEL_CHARS_MAX):
            raise CompileError(f"declare_param({name!r}): label {label!r} is not 1 to "
                               f"{NxsMap.LABEL_CHARS_MAX} printable ASCII characters")
    twice = sorted({label for label in names if names.count(label) > 1})
    if twice:
        raise CompileError(f"declare_param({name!r}): {', '.join(twice)} labels two values")
    return names


def _printable(text: str, limit: int, allow_empty: bool = False) -> bool:
    return (isinstance(text, str) and (allow_empty or bool(text)) and len(text) <= limit
            and all(0x20 <= ord(c) <= 0x7E for c in text))


class ClickPersonality(_CompilePhase):
    """Base class for NXS click personalities: a datasheet in code. The YAML config
    selects the mode; compile(config) produces bytecode plus output field
    descriptors."""

    # Bus family declared by the subclass; plain ClickPersonality is abstract and
    # compile() rejects it.
    BUS_KIND: Optional[str] = None

    # mikroBUS reset active level: 'low' (default) or 'high'. Firmware pulses
    # the shared reset at bind for every personality kind.
    RESET_ACTIVE = 'low'

    # The ports the personality's device offers, the image's PORTS record: a
    # click board's one output, the mikroBUS edge a unit's socket takes.
    PORTS: Tuple[Port, ...] = (Port("out", PortType.CLICK, PortDirection.OUTPUT),)

    def __init__(self):
        self._emitter = self._new_emitter()
        self._regs = _RegAlloc()
        # 64-bit work-buffer slots; persists across configure() and measure()
        # so coefficients read in configure() stay addressable.
        self._work = _WorkAlloc(_ASTCompiler.VM_WORK_BUF_SIZE)
        self._sample_size = 14
        self._read_responses: Dict[int, list] = {}
        self._output_fields: List[dict] = []
        self._config: dict = {}
        self._params: Dict[str, ParamDescriptor] = {}
        # The rows a hub or host program declares apart from its run parameters.
        self._rows: Dict[str, ParamDescriptor] = {}
        self._patch_entries: List[PatchEntry] = []
        self._patch_accum: Dict[str, Dict] = {}  # param_name → {config_val → byte_val}
        # Params the program reads or stores at run time (`param()`,
        # `select()`, `store_param()`); they need no patch site.
        self._params_used: set = set()
        # Counter for unique labels inside multi-instruction helpers.
        self._label_counter = 0
        # Swapped by _finalize_patches to capture per-parameter bytes;
        # normal use points at _record_patch.
        self._patch_recorder_fn = self._record_patch
        # Named bytecode spans `(label, start, end)` for the budget report;
        # select() adds its blocks and dispatch.
        self._budget_spans: List[Tuple[str, int, int]] = []

    def _i2c_profile(self) -> I2cProfile:
        """The class's ``I2C_PROFILE``, or the conventional defaults."""
        prof = getattr(type(self), 'I2C_PROFILE', None)
        return prof if isinstance(prof, I2cProfile) else I2cProfile()

    def _new_emitter(self) -> _Emitter:
        """An emitter whose register cap follows the I²C profile's address
        width: 16-bit registers only for ``addr_bytes=2``."""
        wide = self._i2c_profile().addr_bytes == 2
        return _Emitter(REG_ADDR_MAX_16 if wide else REG_ADDR_MAX_8)

    def _validate_kind(self) -> None:
        """Kind-specific class checks before tracing; a personality has none."""

    def _fresh_label(self, base: str) -> str:
        """Generate a unique label name; safe to call multiple times
        per helper invocation."""
        self._label_counter += 1
        return f"__{base}_{self._label_counter}"

    def _budget_lines(self, probe_end: int, configure_end: int,
                      total: int) -> List[Tuple[str, int]]:
        """The budget report's `(label, bytes)` rows from the recorded
        spans: probe, configure minus the union of its select() spans (a
        nested select sits inside its block), one row per span label, and
        whatever follows configure (a measure loop or the halt)."""
        lines = [("probe", probe_end)]
        covered = 0
        reach = probe_end
        for _, start, end in sorted(self._budget_spans, key=lambda s: s[1]):
            start = max(start, reach)
            if end > start:
                covered += end - start
                reach = end
        lines.append(("configure (shared)", configure_end - probe_end - covered))
        # A byte belongs to the innermost span holding it: a select inside
        # a block's program is that select's, not the block's.
        sizes: Dict[str, int] = {}
        ordered = sorted(self._budget_spans, key=lambda s: (s[1], -s[2]))
        for i, (label, start, end) in enumerate(ordered):
            inner = 0
            reach = start
            for _, s, e in ordered[i + 1:]:
                if s >= end:
                    break
                if e <= end:
                    s = max(s, reach)
                    if e > s:
                        inner += e - s
                        reach = e
            sizes[label] = sizes.get(label, 0) + (end - start - inner)
        lines.extend(sizes.items())
        tail = total - configure_end
        if tail:
            lines.append(("halt" if getattr(type(self), 'IMAGE_KIND', None)
                          in (ImageKind.CAMERA, *HOST_KINDS) else "measure loop", tail))
        return lines

    def _check_param_value(self, name, value, values, param_type, unit):
        """Raise CompileError if `value` is illegal for this param: outside
        [min, max] for a range, or not in the allowed set for an enum."""
        unit_str = f" {unit}" if unit else ""
        # Params serialize as u32, so a non-integer override is always invalid;
        # bool is an int subclass and is excluded explicitly.
        if not isinstance(value, int) or isinstance(value, bool):
            raise CompileError(
                f"Parameter {name!r} = {value!r} must be an integer for "
                f"{type(self).__name__}.")
        if param_type == "range":
            if not (values[0] <= value <= values[1]):
                raise CompileError(
                    f"Parameter {name!r} = {value!r} out of range for "
                    f"{type(self).__name__}. "
                    f"Valid range: {values[0]}-{values[1]}{unit_str}")
        elif value not in values:
            raise CompileError(
                f"Parameter {name!r} = {value!r} not supported by "
                f"{type(self).__name__}. "
                f"Valid values: {values}{unit_str}")

    def declare_params_from_descriptor(self) -> None:
        """Declare this personality's parameter table from its sibling YAML
        descriptor. Declaration order is the YAML order, which fixes the wire's
        param indices."""
        from nxs.click_facts import click_params

        for entry in click_params(type(self)):
            param_type = entry.get("type", "enum")
            show = entry.get("show")
            # A range has no bytecode patch site, so it always applies in
            # place: an entry that names no kind gets the only kind that loads.
            self.declare_param(
                entry["name"],
                values=list(entry["values"]),
                default=entry["default"],
                param_type=param_type,
                unit=entry.get("unit", ""),
                kind=entry.get("kind", "apply" if param_type == "range" else "run"),
                id=_param_id_named(entry["name"], entry.get("id")),
                show=None if show is None else Show(str(show.get("unit", "")),
                                                    show.get("exponent", 0)),
                labels=entry.get("labels"),
            )

    def declare_param(self, name: str, values: list, default: Any,
                      param_type: str = "enum", unit: str = "",
                      kind: str = "run", id: int = ParamId.CUSTOM,
                      show: Optional[Show] = None,
                      labels: Optional[Sequence[str]] = None, public: bool = True):
        """Declare a configurable parameter with valid values. Call in
        configure() before using the parameter. The declared default and any
        config override are validated here, at declaration time. `id` is the
        row's standard id, `show` how a host shows its values and `labels` a
        name per value, an enum's in its order and a range's from its min.
        `public` puts a hub or host program's parameter in its image's PARAMS
        record."""
        self._check_values("declare_param", name, values, param_type)
        # Both the default and a config override reach the image: default as the
        # fallback, current as the value applied at load.
        self._check_param_value(name, default, values, param_type, unit)
        if name in self._config:
            current = self._config[name]
            self._check_param_value(name, current, values, param_type, unit)
        else:
            current = default
        if kind not in ("run", "apply"):
            raise CompileError(
                f"declare_param({name!r}): kind must be 'run' or 'apply', "
                f"got {kind!r}")
        # A range param has no bytecode patch site, so the firmware accepts it
        # only as kind APPLY.
        if param_type == "range" and kind != "apply":
            raise CompileError(
                f"declare_param({name!r}): a range param must be kind='apply' "
                f"(the firmware rejects a range of another kind).")
        self._params[name] = ParamDescriptor(
            name=name, param_type=param_type, values=values,
            default=default, current=current, unit=unit, kind=kind,
            id=_standard_id("declare_param", name, id, values, param_type),
            show=_show(name, show),
            labels=_value_names(name, values, param_type, labels), public=bool(public),
        )
        self._patch_accum[name] = {}

    def declare_row(self, name: str, values: list, default: Any,
                    param_type: str = "enum", unit: str = "", kind: str = "run",
                    id: int = ParamId.CUSTOM, show: Optional[Show] = None,
                    labels: Optional[Sequence[str]] = None):
        """A row of a hub or host program's PARAMS record that no phase
        reads: what a declaration sets on the block, which the graph's walk
        stages into the run parameters. Its values, kind, id, display and
        names are a parameter's; it takes no config override."""
        self._check_values("declare_row", name, values, param_type)
        self._check_param_value(name, default, values, param_type, unit)
        if kind not in ("run", "apply"):
            raise CompileError(f"declare_row({name!r}): kind must be 'run' or 'apply', got {kind!r}")
        if name in self._params or name in self._rows:
            raise CompileError(f"declare_row({name!r}): the name is declared already")
        self._rows[name] = ParamDescriptor(
            name=name, param_type=param_type, values=values, default=default, current=default,
            unit=unit, kind=kind, id=_standard_id("declare_row", name, id, values, param_type),
            show=_show(name, show),
            labels=_value_names(name, values, param_type, labels), public=True)

    @staticmethod
    def _check_values(verb: str, name: str, values: list, param_type: str) -> None:
        """A declaration's values: an enum's or a range's `[min, max]`, each
        a whole number the descriptor wire carries."""
        if param_type not in ("enum", "range"):
            raise CompileError(
                f"{verb}({name!r}): param_type must be 'enum' or "
                f"'range', got {param_type!r}")
        if param_type == "range" and (len(values) != 2 or values[0] > values[1]):
            raise CompileError(
                f"{verb}({name!r}): a range param must declare exactly "
                f"[min, max] with min <= max, got {values!r}")
        # The descriptor wire carries values as unsigned 32-bit cells; a
        # fractional or negative value would ship a corrupt allowed set.
        for v in values:
            if not isinstance(v, int) or isinstance(v, bool) or not 0 <= v <= 0xFFFFFFFF:
                raise CompileError(
                    f"{verb}({name!r}): value {v!r} is not a whole "
                    f"number from 0 to 4294967295 — the descriptor wire carries "
                    f"unsigned 32-bit values. Encode fractional physical values in a "
                    f"smaller integer unit (mHz, mV), offset a signed range "
                    f"to start at 0, drop the rows with a stated exclusion, or "
                    f"make the setting a compile-time config key.")

    def drive_pwm(self, freq: int = 1000, duty: int = 50):
        """Drive the mikroBUS PWM pin via the apply params ``pwm_freq`` (Hz) and
        ``pwm_duty`` (%), retunable at runtime with no restart. ``freq`` and
        ``duty`` are the initial drive applied at load. Call in configure()."""
        if not (PWM_FREQ_MIN_HZ <= freq <= PWM_FREQ_MAX_HZ):
            raise CompileError(
                f"drive_pwm(freq={freq}): must be "
                f"{PWM_FREQ_MIN_HZ}-{PWM_FREQ_MAX_HZ} Hz")
        if not (0 <= duty <= 100):
            raise CompileError(f"drive_pwm(duty={duty}): must be 0-100 %")
        self.declare_param("pwm_freq", values=[PWM_FREQ_MIN_HZ, PWM_FREQ_MAX_HZ],
                           default=freq, param_type="range", unit="Hz",
                           kind="apply")
        self.declare_param("pwm_duty", values=[0, 100], default=duty,
                           param_type="range", unit="%", kind="apply")

    def persistent(self, name: str, init: int = 0):
        """Declare a loop-carried integer: initialized once in configure(),
        preserved across measure() iterations. Call in configure()."""
        reg = self._regs.pin(name)
        self._emitter.emit_u32(Op.LOAD_IMM, reg, init & 0xFFFFFFFF)

    def _record_patch(self, param_name: str, config_value: Any,
                      byte_value: int, offset: int, size: int = 1,
                      reg: Optional[int] = None):
        """Record a patchable byte in the bytecode stream."""
        # declare_param seeds _patch_accum; a tag without it has no value set.
        param = self._params.get(param_name)
        if param_name not in self._patch_accum or param is None:
            raise CompileError(
                f"param=({param_name!r}, ...) tagged on a write before "
                f"declare_param({param_name!r}, ...); declare the parameter "
                f"first so its value set is known and the write patches.")
        # The value_map is built from the declared set; the tagged value must
        # be in it.
        self._check_param_value(param_name, config_value, param.values,
                                param.param_type, param.unit)
        self._patch_accum[param_name][config_value] = byte_value
        self._patch_entries.append(PatchEntry(
            offset=offset, param_name=param_name,
            value_map={},  # filled in _finalize_patches
            reg=reg, size=size,
        ))

    def _patch_poll_rate(self, sleep_operand_offset: int):
        """Patch a poll loop's SLEEP_MS interval from a declared sample_rate.
        The interval is a pure function of the rate, so no re-trace is needed."""
        p = self._params.get("sample_rate")
        if p is None or p.param_type != "enum":
            return
        # A tagged register write already owns the rate; don't add a second
        # site on the loop tick.
        if any(pe.param_name == "sample_rate" for pe in self._patch_entries):
            return
        value_map = {rate: max(1, 1000 // int(rate)) for rate in p.values}
        self._patch_entries.append(PatchEntry(
            offset=sleep_operand_offset, param_name="sample_rate",
            value_map=value_map, size=2,
        ))

    def _patch_drdy_div(self, div_operand_offset: int, base_hz: int):
        """Patch a drdy loop's OP_EVENT_DIV divider from a declared sample_rate
        on a fixed-sync part. Every declared rate must divide the sync exactly."""
        p = self._params.get("sample_rate")
        if p is None or p.param_type != "enum":
            return
        # drdy_base_hz asserts "no rate register"; a part that also tags a rate
        # register would double-pace, so the contradiction is rejected.
        if any(pe.param_name == "sample_rate" for pe in self._patch_entries):
            raise CompileError(
                "drdy_base_hz declared, but sample_rate already patches a "
                "register write — a part with a rate register paces through "
                "it; drop drdy_base_hz (or the param= tag, if the register "
                "is not the rate).")
        for rate in p.values:
            if rate <= 0 or base_hz % int(rate) != 0:
                raise CompileError(
                    f"sample_rate value {rate} does not divide the {base_hz} "
                    f"Hz hardware sync; declare exact divisors (base/N) so "
                    f"every settable rate is delivered exactly.")
        value_map = {rate: base_hz // int(rate) for rate in p.values}
        self._patch_entries.append(PatchEntry(
            offset=div_operand_offset, param_name="sample_rate",
            value_map=value_map, size=2,
        ))

    # Above this, sleep_ms() unrolls into OP_SLEEP_MS chunks, each returning
    # VM_YIELD so the runner can observe a STOP request within one chunk.
    SLEEP_CHUNK_MS = 50

    # Hard cap on unrolled sleeps: the VM has no runtime decrement opcode, and
    # the 2 KB program size leaves no room for sleeps measured in seconds.
    SLEEP_MAX_UNROLLED_MS = 1000

    def sleep_ms(self, ms: int):
        if ms <= self.SLEEP_CHUNK_MS:
            self._emitter.emit_u16(Op.SLEEP_MS, ms)
            return
        if ms > self.SLEEP_MAX_UNROLLED_MS:
            raise CompileError(
                f"sleep_ms({ms}) exceeds the {self.SLEEP_MAX_UNROLLED_MS} "
                f"ms unrolled cap. The VM lacks a runtime decrement "
                f"opcode, so sleeps must unroll into "
                f"OP_SLEEP_MS({self.SLEEP_CHUNK_MS}) chunks at compile "
                f"time. If this is a microsecond delay, use sleep_us(). "
                f"For genuine long waits, factor the personality to use "
                f"trigger=drdy or split the wait across measure-loop "
                f"iterations.")
        full = ms // self.SLEEP_CHUNK_MS
        remainder = ms % self.SLEEP_CHUNK_MS
        for _ in range(full):
            self._emitter.emit_u16(Op.SLEEP_MS, self.SLEEP_CHUNK_MS)
        if remainder > 0:
            self._emitter.emit_u16(Op.SLEEP_MS, remainder)

    def sleep_us(self, us: int):
        self._emitter.emit_u16(Op.SLEEP_US, us)

    def read(self, reg, width=1, signed=False, endian="big", dev=None):
        """Read `width` bytes at `reg`, unsigned big-endian unless `signed` /
        `endian` say otherwise; `dev=` targets a declared I2C companion. In
        configure() a multi-byte read returns a `WideRef`, a byte read a mock."""
        if not 1 <= width <= 4:
            raise CompileError(
                f"read(reg, width) supports width 1..4 (OP_LOAD), got {width}.")
        if width == 1 and (signed or endian != "big"):
            raise CompileError(
                "read(reg, signed=/endian=) needs width > 1 (a single byte "
                "has no byte order; use width 2+ for a signed field).")
        addr = self._companion_addr(dev) if dev is not None else None
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, addr)
        if width > 1:
            # One reused scratch register stages the burst; the value is dead
            # once CVT64 copies it into the work buffer.
            self._emitter.emit_reg(Op.REG_READ_BURST, reg, width, 0)
            if addr is not None:
                self._emitter.emit(Op.I2C_TARGET, 0)
            r = self._regs.get("__coef_scratch")
            self._emitter.emit(Op.LOAD, r, 0, _load_spec_byte(width, signed, endian))
            off = self._work.alloc()
            self._emitter.emit(Op.CVT64, r, off)
            return WideRef(off)
        dst = self._regs.get(f"__reg_read_{reg}")
        self._emitter.emit_reg(Op.REG_READ, reg, dst)
        if addr is not None:
            self._emitter.emit(Op.I2C_TARGET, 0)
        responses = self._read_responses.get((dev, reg) if dev else reg)
        val = responses.pop(0) if responses else 0
        # In configure() the value is a compile-time mock; probe() keeps the
        # real mock for its WHO_AM_I assert.
        if self._trace_phase == "configure":
            return _TraceReadValue(reg)
        return val

    def set_sample_size(self, size: int):
        if not (0 < size <= _ASTCompiler.VM_SAMPLE_BUF_SIZE):
            raise CompileError(
                f"set_sample_size({size}): the sample buffer holds "
                f"{_ASTCompiler.VM_SAMPLE_BUF_SIZE} bytes.")
        self._sample_size = size
        self._emitter.emit(Op.SET_SAMPLE_SIZE, size)

    def set_output(self, fields: List[dict]):
        """Declare the output fields, each {name, scale, unit?, offset?, type?,
        count?, scale_param?, at?}. A known-semantic name inherits its canonical
        SI unit; any other field must declare `unit` ('' for unitless)."""
        self._output_fields = []
        for f in fields:
            semantic = infer_semantic(f['name'])
            canonical = FieldSemantics.FIELD_SEMANTIC_UNIT.get(semantic)
            unit = f.get('unit')
            if canonical is not None:
                if unit is not None and unit != canonical:
                    raise CompileError(
                        f"set_output field {f['name']!r}: unit {unit!r} "
                        f"contradicts the canonical SI unit {canonical!r} "
                        f"for its semantic — emit SI at the source (omit "
                        f"`unit` to inherit it).")
                unit = canonical
            elif unit is None:
                raise CompileError(
                    f"set_output field {f['name']!r}: no canonical SI unit "
                    f"to inherit for its semantic — declare 'unit' "
                    f"explicitly ('' for unitless).")
            entry = {
                'name': f['name'],
                'type': f.get('type', 'int16'),
                'byte_order': f.get('byte_order', 'big'),
                'scale': float(f['scale']),
                'offset': float(f.get('offset', 0.0)),
                'unit': unit,
                'semantic': semantic,
            }
            # A scale_param must already be declared: the firmware multiplies
            # the base scale by that param's live value.
            scale_param = f.get('scale_param')
            if scale_param is not None and scale_param not in self._params:
                raise CompileError(
                    f"set_output field {f['name']!r}: scale_param "
                    f"{scale_param!r} is not a declared parameter — call "
                    f"declare_param({scale_param!r}, ...) before set_output().")
            # The value multiplies the scale, so a 0 in the set zeroes the
            # field; register-code value sets are the common mistake.
            if scale_param is not None and 0 in self._params[scale_param].values:
                raise CompileError(
                    f"set_output field {f['name']!r}: scale_param "
                    f"{scale_param!r} has 0 in its values; the value multiplies "
                    f"the scale, so 0 zeroes the field. Its values must be "
                    f"physical magnitudes, not register codes.")
            entry['scale_param'] = scale_param
            # Strings carry an explicit width; numeric types' width is
            # implied by `type` so `count` isn't meaningful for them.
            if entry['type'] == 'string':
                entry['count'] = int(f.get('count', 0))
            # `at=` places the field at an explicit sample-buffer byte (the
            # binary-record idiom).
            if 'at' in f:
                entry['byte_off'] = int(f['at'])
            self._output_fields.append(entry)

        # Explicit placement is all-or-none: mixing `at=` with hub-sequential
        # fields makes the implicit offsets depend on declaration order.
        n_at = sum(1 for e in self._output_fields if 'byte_off' in e)
        if 0 < n_at < len(self._output_fields):
            raise CompileError(
                f"set_output mixes fields with and without 'at': declare "
                f"an explicit byte offset on every field or on none.")
        resolve_field_offsets(self._output_fields)
        validate_field_layout(self._output_fields,
                              _ASTCompiler.VM_SAMPLE_BUF_SIZE)

    @staticmethod
    def measure_loop(trigger: str = "drdy", sample_rate: int = 250,
                     drdy_base_hz: int = 0,
                     when: Optional[tuple] = None, default: bool = False):
        """Decorator marking the measure loop for AST compilation. `trigger` may
        be "from_config". `drdy_base_hz` paces a fixed-sync part (OP_EVENT_DIV).
        `when=(key, value)` picks a config variant; `default` marks the fallback."""
        def decorator(func):
            func._measure_loop = True
            func._trigger = trigger
            func._sample_rate = sample_rate
            func._drdy_base_hz = drdy_base_hz
            func._when = when
            func._when_default = default
            return func
        return decorator

    def _select_measure_fn(self, config: dict):
        """Resolve the measure method this compile targets: the single bare
        loop, or the `when=` variant the config selects (the `default=True` one
        when the key is absent). Every ambiguity is a CompileError."""
        variants = []
        for name in dir(type(self)):
            fn = getattr(type(self), name)
            if callable(fn) and getattr(fn, "_measure_loop", False):
                variants.append(fn)
        if not variants:
            return None

        tagged = [fn for fn in variants if getattr(fn, "_when", None)]
        if not tagged:
            if len(variants) > 1:
                names = sorted(fn.__name__ for fn in variants)
                raise CompileError(
                    f"{len(variants)} measure loops ({', '.join(names)}) "
                    f"but no `when=` selectors — tag each variant with "
                    f"when=(key, value), or keep a single measure loop.")
            return variants[0]
        if len(tagged) != len(variants):
            bare = sorted(fn.__name__ for fn in variants
                          if not getattr(fn, "_when", None))
            raise CompileError(
                f"measure loops {', '.join(bare)} carry no `when=` while "
                f"others do — tag every variant or none.")

        keys = {fn._when[0] for fn in tagged}
        if len(keys) != 1:
            raise CompileError(
                f"measure-loop `when=` selectors use different config "
                f"keys {sorted(keys)} — all variants key on one.")
        key = keys.pop()
        values = [fn._when[1] for fn in tagged]
        if len(set(values)) != len(values):
            raise CompileError(
                f"two measure loops claim the same {key!r} value — "
                f"each variant's `when` value must be distinct.")

        chosen = config.get(key)
        if chosen is None:
            defaults = [fn for fn in tagged
                        if getattr(fn, "_when_default", False)]
            if len(defaults) != 1:
                raise CompileError(
                    f"config omits {key!r} and {len(defaults)} variants "
                    f"are marked default=True — exactly one must be.")
            fn = defaults[0]
            config[key] = fn._when[1]
            return fn
        for fn in tagged:
            if fn._when[1] == chosen:
                return fn
        raise CompileError(
            f"config {key}={chosen!r} matches no measure variant; "
            f"declared values: {sorted(map(str, values))}.")

