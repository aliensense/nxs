"""The compile phase of a personality: probe and configure traced, the measure loop compiled, the image assembled."""

from __future__ import annotations

from typing import Dict, List, Optional

from nxs.opcodes import Op
from nxs.profiles import I2cProfile, SpiProfile
from nxs.dsl.compiled import CompiledDriver, ImageKind
from nxs.dsl.emit import BUS_REGISTER, OpErrorCode, _BUS_CODE_TO_NAME, _BUS_NAME_TO_CODE, _RegAlloc, _WorkAlloc
from nxs.dsl.errors import CompileError
from nxs.dsl.fields import (MAX_PATCH_SITES, ParamDescriptor, PatchEntry, VM_HOST_PROGRAM_SIZE,
                            VM_MAX_PROGRAM_SIZE, resolve_field_offsets)
from nxs.dsl.loop import _ASTCompiler


class _CompilePhase:
    """The `compile()` half of `ClickPersonality`: the image from the traced probe and
    configure and the compiled measure loop."""

    def compile(self, config: Optional[dict] = None) -> CompiledDriver:
        """Compile the personality into VM bytecode. `config` is the sensor
        configuration dict (from YAML) passed to configure(); None means {}."""
        if self.BUS_KIND is None:
            raise CompileError(
                f"{type(self).__name__} inherits ClickPersonality directly; "
                "use RegisterClickPersonality or StreamClickPersonality as the base class.")

        if config is None:
            config = {}
        # Canonicalise enum-string params to their integer codes first, so
        # `bus=spi` from the CLI compares against the declared int set.
        if isinstance(config.get('bus'), str):
            config['bus'] = _BUS_NAME_TO_CODE.get(
                config['bus'].lower().strip(), config['bus'])
        self._config = config

        cls = type(self)
        kind = int(getattr(cls, 'IMAGE_KIND', ImageKind.DRIVER))
        # A camera or hub image runs once: probe, configure, halt.
        is_camera = kind in (ImageKind.CAMERA, ImageKind.HUB)

        self._emitter = self._new_emitter()
        self._regs = _RegAlloc()
        self._work = _WorkAlloc(_ASTCompiler.VM_WORK_BUF_SIZE)
        self._output_fields = []
        self._params = {}
        self._patch_entries = []
        self._trace_phase = None
        self._patch_accum = {}
        self._budget_spans = []

        # Snapshot the declared `_read_responses`: tracing pops from the lists,
        # and each re-trace in _finalize_patches refills from this template.
        self._read_responses_template = {
            reg: list(vals) for reg, vals in self._read_responses.items()
        }

        # Phase 0: emit the WHO_AM_I check from class attributes before any
        # probe() body; a Python `assert` in probe() compiles to nothing.
        self._validate_kind()
        self._validate_companions()
        self._emit_who_am_i_prologue()

        # Phase 1: Trace probe()
        if hasattr(self, "probe"):
            self._trace_phase = "probe"
            self.probe()
        probe_end = self._emitter._current_offset()

        # Phase 2: trace configure(). The measure variant is selected first:
        # selection stamps the chosen value into `config`, which configure() reads.
        measure_fn = self._select_measure_fn(config)

        if hasattr(self, "configure"):
            self._trace_phase = "configure"
            self.configure(config)
        self._trace_phase = None
        configure_end = self._emitter._current_offset()

        # Phase 3: AST-compile measure(); a cam personality halts instead
        # (its one pass is probe, configure, release the bus).
        if is_camera:
            self._emitter.emit(Op.HALT)
        elif measure_fn is not None:
            self._regs.reset()
            trigger = getattr(measure_fn, "_trigger", "drdy")
            sample_rate = getattr(measure_fn, "_sample_rate", 250)

            # "from_config" reads trigger and sample_rate from config dict
            if trigger == "from_config":
                trigger = config.get("trigger", "drdy")
            # Always allow config to override sample_rate
            sample_rate = config.get("sample_rate", sample_rate)

            ast_compiler = _ASTCompiler(
                self._emitter, self._regs, trigger, sample_rate, self.BUS_KIND,
                frame=getattr(self, 'FRAME', None),
                personality=self,
                drdy_base_hz=getattr(measure_fn, "_drdy_base_hz", 0))
            ast_compiler.compile_function(measure_fn)

        bytecode = self._emitter.build()
        budget = self._budget_lines(probe_end, configure_end, len(bytecode))

        # Finalize patch map: build full value_map for each patch entry
        # by compiling the personality with each valid parameter value
        finalized_patches = self._finalize_patches(config)

        # Over the program cap the compile fails with the budget, never a
        # truncated program. A hub image runs on the host and has its own.
        cap = VM_HOST_PROGRAM_SIZE if kind == ImageKind.HUB else VM_MAX_PROGRAM_SIZE
        if len(bytecode) > cap:
            report = "\n".join(f"  {label:<40} {size:>5} B"
                               for label, size in budget)
            raise CompileError(
                f"{cls.__name__}: {len(bytecode)} B of bytecode exceeds the "
                f"{cap} B VM program limit; the budget:\n"
                f"{report}")

        # Auto-declare the runtime `bus` param on every register personality from its
        # `BUSES` tuple; the first entry is the default. No bytecode is patched.
        # A cam personality binds the pod bus and carries no bus switch.
        if (self.BUS_KIND == BUS_REGISTER and 'bus' not in self._params
                and not is_camera):
            buses = getattr(self, 'BUSES', None)
            if not buses:
                raise CompileError(
                    f"{type(self).__name__}: BUSES class attribute is "
                    f"empty or missing. Declare which physical buses "
                    f"the sensor supports, e.g. BUSES = ('spi',) for a "
                    f"SPI-only part.")
            bus_values = []
            for name in buses:
                code = _BUS_NAME_TO_CODE.get(name.lower())
                if code is None:
                    raise CompileError(
                        f"{type(self).__name__}: BUSES contains unknown "
                        f"entry {name!r}. Valid entries: "
                        f"{sorted(_BUS_NAME_TO_CODE.keys())}")
                if code not in bus_values:
                    bus_values.append(code)
            default_bus = bus_values[0]
            # A `BUS = '<name>'` class attribute (the one-line Communication
            # Profile selector) overrides the BUSES-order default.
            declared = getattr(self, 'BUS', None)
            if declared is not None:
                code = _BUS_NAME_TO_CODE.get(str(declared).lower().strip())
                if code is None or code not in bus_values:
                    raise CompileError(
                        f"{type(self).__name__}: BUS = {declared!r} must name "
                        f"one of BUSES {buses}")
                default_bus = code
            current_bus = default_bus
            if 'bus' in config:
                raw = config['bus']
                if isinstance(raw, str):
                    raw = _BUS_NAME_TO_CODE.get(raw.lower().strip(), raw)
                if raw not in bus_values:
                    bus_labels = [_BUS_CODE_TO_NAME[c] for c in bus_values]
                    raise CompileError(
                        f"Parameter 'bus' = {config['bus']!r} not "
                        f"supported by {type(self).__name__}. "
                        f"Valid values: {bus_labels}")
                current_bus = raw
            self._params['bus'] = ParamDescriptor(
                name='bus', param_type='enum',
                values=bus_values,
                default=default_bus,
                current=current_bus,
                unit='',
            )

        # Auto-inject a runtime `reset_active` param when the personality overrides
        # the mikroBUS reset polarity; the pod bus has no reset line.
        reset_active = str(getattr(self, 'RESET_ACTIVE', 'low')).lower().strip()
        if reset_active not in ('low', 'high'):
            raise CompileError(
                f"{type(self).__name__}: RESET_ACTIVE must be 'low' or "
                f"'high', got {getattr(self, 'RESET_ACTIVE')!r}")
        if (reset_active == 'high' and 'reset_active' not in self._params
                and not is_camera):
            self._params['reset_active'] = ParamDescriptor(
                name='reset_active', param_type='enum',
                values=[0, 1], default=1, current=1, unit='')

        who_am_i_values = [
            int(v) & 0xFF
            for v in getattr(cls, 'WHO_AM_I_VALUES', []) or []
        ][:16]
        skip_reason = getattr(cls, 'WHO_AM_I_SKIP_REASON', None)
        if skip_reason is not None:
            skip_reason = str(skip_reason).strip() or None
        # The WHO_AM_I rule applies only to register personalities that declare
        # WHO_AM_I_VALUES somewhere in their MRO; stream personalities have no probe.
        who_am_i_declared = any(
            'WHO_AM_I_VALUES' in c.__dict__ for c in cls.__mro__
        )
        if self.BUS_KIND == BUS_REGISTER and who_am_i_declared:
            if not who_am_i_values and not skip_reason:
                raise CompileError(
                    f"{cls.__name__}: WHO_AM_I_VALUES = [] requires a "
                    f"non-empty WHO_AM_I_SKIP_REASON attribute explaining "
                    f"why this personality opts out of the WHO_AM_I probe. "
                    f"Either declare a real WHO_AM_I check (`WHO_AM_I_REG`, "
                    f"`WHO_AM_I_VALUES`), define a synthetic `probe()` "
                    f"that reads any register and asserts a known "
                    f"reset-state value, or document the opt-out:\n"
                    f"  WHO_AM_I_SKIP_REASON = \"...\"")
            if who_am_i_values and skip_reason:
                raise CompileError(
                    f"{cls.__name__}: WHO_AM_I_SKIP_REASON is set but "
                    f"WHO_AM_I_VALUES is non-empty. The skip reason "
                    f"only applies when opting out "
                    f"(WHO_AM_I_VALUES = []). Remove "
                    f"WHO_AM_I_SKIP_REASON or empty WHO_AM_I_VALUES.")
        # A reload/enum param that patches no bytecode is a no-op `set`.
        # Injected runtime params (bus, reset_active) are exempt.
        patched = {pe.param_name for pe in finalized_patches}
        for name, p in self._params.items():
            if (name in self._patch_accum and p.kind == "reload"
                    and p.param_type == "enum" and name not in patched
                    and name not in self._params_used):
                raise CompileError(
                    f"param {name!r} is declared reload/enum but patches no "
                    f"bytecode; a `set {name}` would reload identical bytecode. "
                    f"Tag a write with param=({name!r}, ...), dispatch on it "
                    f"with select(), or remove the declaration.")

        # A live param needs a runtime consumer: the PWM pair, an output
        # field's scale_param, or the program itself. Any other live param
        # is a no-op set.
        live_consumers = {"pwm_freq", "pwm_duty"}
        scale_refs = {f.get("scale_param") for f in self._output_fields
                      if f.get("scale_param")}
        for name, p in self._params.items():
            if (p.kind == "live" and name not in live_consumers
                    and name not in scale_refs and name not in self._params_used):
                raise CompileError(
                    f"param {name!r} is kind='live' but nothing reads it at "
                    f"runtime (not PWM, not an output scale_param, not the "
                    f"program through param()); a `set {name}` would be a "
                    f"no-op. Use kind='reload', or reference it.")

        return CompiledDriver(
            bytecode=bytecode,
            sample_size=self._sample_size,
            name=cls.__name__,
            config=config,
            output_fields=resolve_field_offsets(list(self._output_fields)),
            params=list(self._params.values()),
            patch_map=finalized_patches,
            who_am_i_reg=int(getattr(cls, 'WHO_AM_I_REG', 0) or 0),
            who_am_i_values=who_am_i_values,
            i2c_addrs=[
                int(a) & 0x7F
                for a in getattr(cls, 'I2C_ADDRS', []) or []
            ][:8],
            who_am_i_skip_reason=skip_reason,
            bus_config=self._build_bus_config(cls, config),
            kind=kind,
            budget=budget,
            probe_len=probe_end,
        )

    def _build_bus_config(self, cls, config: dict) -> Optional[list]:
        """Assemble the bus_config trailer: one register-access profile per
        entry in ``BUSES`` (from ``SPI_PROFILE`` / ``I2C_PROFILE`` or the
        conventional defaults). None for a stream personality."""
        if self.BUS_KIND == BUS_REGISTER:
            profiles = []
            seen = set()
            for name in getattr(cls, 'BUSES', ('i2c', 'spi')):
                key = str(name).lower()
                if key in seen:
                    continue
                seen.add(key)
                if key == 'i2c':
                    profiles.append(self._i2c_profile_dict(cls))
                elif key == 'spi':
                    profiles.append(self._spi_profile_dict(cls))
            return profiles or None
        return None

    @staticmethod
    def _spi_profile_dict(cls) -> dict:
        """One SPI register-access profile dict for the NXS trailer, from the
        personality's ``SPI_PROFILE`` descriptor (conventional wire-shape defaults
        when the personality declares none)."""
        prof = getattr(cls, 'SPI_PROFILE', None)
        if not isinstance(prof, SpiProfile):
            prof = SpiProfile()
        if prof.auto_inc == 'none':
            raise CompileError(
                "SpiProfile(auto_inc='none'): parts without auto-increment "
                "are not supported; use 'implicit' or 'msb'")
        max_hz = int(prof.max_hz) if prof.max_hz else 0
        spi_mode = (prof.mode & 0x03) | (0x04 if prof.bit_order == 'lsb' else 0)
        return {
            'kind': 'spi', 'max_hz': max_hz, 'spi_mode': spi_mode,
            'addr_bytes': prof.addr_bytes, 'rw_read_level': prof.rw_read_level,
            'dummy_bytes': prof.dummy_bytes, 'auto_inc': prof.auto_inc,
        }

    @staticmethod
    def _i2c_profile_dict(cls) -> dict:
        """One I²C register-access profile dict for the NXS trailer, from the
        personality's ``I2C_PROFILE`` descriptor (conventional defaults when the
        personality declares none)."""
        prof = getattr(cls, 'I2C_PROFILE', None)
        if not isinstance(prof, I2cProfile):
            prof = I2cProfile()
        if prof.pec == 'crc8':
            raise CompileError(
                "I2cProfile(pec='crc8'): SMBus PEC is not applied by "
                "firmware; a part needing it would run with no error "
                "checking, so it is rejected rather than silently dropped")
        if prof.auto_inc == 'none':
            raise CompileError(
                "I2cProfile(auto_inc='none'): parts without auto-increment "
                "are not supported; use 'implicit' or 'msb'")
        max_hz = int(prof.max_hz) if prof.max_hz else 0
        return {'kind': 'i2c', 'max_hz': max_hz,
                'auto_inc': prof.auto_inc, 'pec': prof.pec,
                'addr_bytes': prof.addr_bytes, 'data_width': prof.data_width,
                'byte_order': prof.byte_order}

    def _companion_addr(self, dev, lineno=None):
        """Resolve a `dev=` name to its declared companion address; an unknown
        name is a CompileError."""
        companions = getattr(type(self), 'I2C_COMPANIONS', None) or {}
        spec = companions.get(dev)
        if spec is None:
            where = f" (line {lineno})" if lineno else ""
            raise CompileError(
                f"dev={dev!r} names no declared companion{where}; "
                f"I2C_COMPANIONS declares {sorted(companions) or 'none'}.")
        return int(spec['addr'])

    def _validate_companions(self):
        """Structural checks on ``I2C_COMPANIONS``: the personality must be I2C-only,
        and each companion needs an identity anchor or a documented skip."""
        companions = getattr(type(self), 'I2C_COMPANIONS', None) or {}
        if not companions:
            return
        name = type(self).__name__
        if tuple(getattr(self, 'BUSES', ())) != ('i2c',):
            raise CompileError(
                f"{name}: I2C_COMPANIONS requires BUSES = ('i2c',) — a "
                f"non-I2C bind cannot retarget a slave address.")
        primaries = set(getattr(type(self), 'I2C_ADDRS', None) or [])
        for dev, spec in companions.items():
            if not isinstance(dev, str) or not dev.isidentifier():
                raise CompileError(
                    f"{name}: companion name {dev!r} must be an identifier.")
            if not isinstance(spec, dict):
                raise CompileError(
                    f"{name}: I2C_COMPANIONS[{dev!r}] must be a dict with "
                    f"'addr' and identity fields.")
            addr = spec.get('addr')
            if not isinstance(addr, int) or not 1 <= addr <= 0x7F:
                raise CompileError(
                    f"{name}: companion {dev!r} addr must be a 7-bit I2C "
                    f"address (1..0x7F), got {addr!r}.")
            if addr in primaries:
                raise CompileError(
                    f"{name}: companion {dev!r} addr 0x{addr:02X} is also a "
                    f"primary strap candidate in I2C_ADDRS — a companion is "
                    f"a different die, not a strap alternative.")
            values = list(spec.get('who_am_i_values') or [])
            if values:
                wai_reg = spec.get('who_am_i_reg')
                if not isinstance(wai_reg, int) or not 0 <= wai_reg <= 0xFF:
                    raise CompileError(
                        f"{name}: companion {dev!r} who_am_i_reg must be "
                        f"0..0xFF, got {wai_reg!r}.")
                for v in values:
                    if not isinstance(v, int) or not 0 <= v <= 0xFF:
                        raise CompileError(
                            f"{name}: companion {dev!r} who_am_i value {v!r} "
                            f"doesn't fit in a byte (OP_REG_READ is 8-bit).")
            elif not spec.get('who_am_i_skip_reason'):
                raise CompileError(
                    f"{name}: companion {dev!r} declares no who_am_i_values "
                    f"and no who_am_i_skip_reason — same audit contract as "
                    f"the primary's WHO_AM_I_SKIP_REASON.")

    def _emit_who_am_i_prologue(self):
        """Emit the runtime identity checks: read ``WHO_AM_I_REG`` and OP_ERROR
        with ``WHO_AM_I_MISMATCH`` unless the value is in ``WHO_AM_I_VALUES``,
        then a bracketed check per ``I2C_COMPANIONS`` entry with an anchor."""
        if self.BUS_KIND != BUS_REGISTER:
            return
        values = list(getattr(type(self), 'WHO_AM_I_VALUES', None) or [])
        if values:
            self._emit_primary_who_am_i(values)
        self._emit_companion_checks()

    def _emit_primary_who_am_i(self, values):
        reg = int(getattr(type(self), 'WHO_AM_I_REG', 0) or 0)
        is_frame = getattr(self, 'FRAME', None) is not None

        SCRATCH_READ = 6
        SCRATCH_MATCH = 7
        em = self._emitter

        if is_frame:
            # The value is compared against the frame's data field; it must fit
            # that field's width.
            width = self.FRAME.data_byte_width()
            limit = (1 << (8 * width)) - 1
            for v in values:
                if not isinstance(v, int) or v < 0 or v > limit:
                    raise CompileError(
                        f"{type(self).__name__}: WHO_AM_I_VALUES entry {v!r} "
                        f"doesn't fit the {width}-byte frame data field.")
            self._emit_frame_read(reg, SCRATCH_READ)
        else:
            # OP_REG_READ is an 8-bit read; values must fit a byte.
            for v in values:
                if not isinstance(v, int) or v < 0 or v > 0xFF:
                    raise CompileError(
                        f"{type(self).__name__}: WHO_AM_I_VALUES entry {v!r} "
                        f"doesn't fit in a byte (OP_REG_READ is 8-bit).")
            em.emit_reg(Op.REG_READ, reg, SCRATCH_READ)

        pass_label = "__whoami_ok"
        for v in values:
            em.emit_cmp(Op.CMP_EQ, SCRATCH_READ, v, SCRATCH_MATCH)
            em.emit_jmp(Op.JNZ, pass_label, SCRATCH_MATCH)
        em.emit(Op.ERROR, OpErrorCode.WHO_AM_I_MISMATCH)
        em.label(pass_label)

    def _emit_companion_checks(self):
        """Bracketed identity check per companion die: retarget, read, compare,
        restore the primary. The ERROR path needs no restore; the VM halts."""
        companions = getattr(type(self), 'I2C_COMPANIONS', None) or {}
        if not companions:
            return
        SCRATCH_READ = 6
        SCRATCH_MATCH = 7
        em = self._emitter
        for dev in sorted(companions):
            spec = companions[dev]
            values = list(spec.get('who_am_i_values') or [])
            if not values:
                continue  # documented opt-out (who_am_i_skip_reason)
            em.emit(Op.I2C_TARGET, int(spec['addr']))
            em.emit_reg(Op.REG_READ, int(spec['who_am_i_reg']), SCRATCH_READ)
            pass_label = f"__whoami_ok_{dev}"
            for v in values:
                em.emit_cmp(Op.CMP_EQ, SCRATCH_READ, v, SCRATCH_MATCH)
                em.emit_jmp(Op.JNZ, pass_label, SCRATCH_MATCH)
            em.emit(Op.ERROR, OpErrorCode.COMPANION_MISMATCH)
            em.label(pass_label)
            em.emit(Op.I2C_TARGET, 0)

    def _finalize_patches(self, config: dict) -> List[PatchEntry]:
        """Build complete value maps by compiling with each param value."""
        if not self._patch_entries:
            return []

        # Group sites per param. A direct REG_WRITE clobbers, so two params
        # can't share a register; RMW sites pass reg=None and skip that check.
        sites_by_param: Dict[str, List[PatchEntry]] = {}
        reg_owner: Dict[int, str] = {}
        for entry in self._patch_entries:
            lst = sites_by_param.setdefault(entry.param_name, [])
            if any(s.offset == entry.offset for s in lst):
                continue  # same site re-recorded
            if entry.reg is not None:
                owner = reg_owner.get(entry.reg)
                if owner is not None and owner != entry.param_name:
                    raise CompileError(
                        f"params {owner!r} and {entry.param_name!r} both patch "
                        f"register 0x{entry.reg:02X}; a shared register can't be "
                        f"driven by two params — compose it from one param.")
                reg_owner[entry.reg] = entry.param_name
            lst.append(PatchEntry(
                offset=entry.offset,
                param_name=entry.param_name,
                value_map=dict(entry.value_map),  # keep a pre-filled map (poll rate)
                reg=entry.reg,
                size=entry.size,
            ))

        for param_name, lst in sites_by_param.items():
            if len(lst) > MAX_PATCH_SITES:
                raise CompileError(
                    f"param {param_name!r} patches {len(lst)} bytecode sites; "
                    f"the image carries at most {MAX_PATCH_SITES} per param.")

        # For each declared param, compile with each valid value to get the
        # patch byte at every one of the param's sites.
        for param_name, param in self._params.items():
            if param_name not in sites_by_param:
                continue
            if param.param_type != "enum":
                continue

            sites = sites_by_param[param_name]
            if all(s.value_map for s in sites):
                continue  # already built directly (poll-rate SLEEP_MS patch)
            # The re-trace runs from a fresh emitter, so its offsets differ in
            # absolute value but keep emission order; map records by rank.
            ordered_sites = sorted(sites, key=lambda s: s.offset)
            ref_offsets = None
            for val in param.values:
                alt_config = dict(config)
                alt_config[param_name] = val
                # Mini-compile: re-trace configure() with this value, keeping
                # patches for this parameter only.
                alt_emitter = self._new_emitter()
                alt_regs = _RegAlloc()
                alt_patches: List[tuple] = []
                alt_records_target = param_name

                def _alt_recorder(pn, cv, bv, off, size=1, reg=None,
                                  _target=alt_records_target):
                    if pn == _target:
                        alt_patches.append((cv, bv, off))

                # Swap emitter/regs, the config, and the patch recorder pointer.
                orig_em = self._emitter
                orig_regs_obj = self._regs
                orig_params = self._params
                orig_accum = self._patch_accum
                orig_fields = self._output_fields
                orig_recorder = self._patch_recorder_fn
                orig_read_responses = self._read_responses
                orig_config = self._config
                orig_spans = self._budget_spans
                orig_work = self._work

                self._emitter = alt_emitter
                self._regs = alt_regs
                self._params = {}
                self._patch_accum = {}
                self._output_fields = []
                self._patch_recorder_fn = _alt_recorder
                self._config = alt_config
                self._budget_spans = []
                self._work = _WorkAlloc(_ASTCompiler.VM_WORK_BUF_SIZE)
                # Refill read-response queue from the personality's template
                # so probe() reads the declared WHO_AM_I etc. again.
                self._read_responses = {
                    reg: list(vals)
                    for reg, vals in self._read_responses_template.items()
                }

                try:
                    if hasattr(self, "probe"):
                        self._trace_phase = "probe"
                        self.probe()
                    if hasattr(self, "configure"):
                        self._trace_phase = "configure"
                        self.configure(alt_config)
                    self._trace_phase = None
                finally:
                    self._emitter = orig_em
                    self._regs = orig_regs_obj
                    self._params = orig_params
                    self._patch_accum = orig_accum
                    self._output_fields = orig_fields
                    self._patch_recorder_fn = orig_recorder
                    self._read_responses = orig_read_responses
                    self._config = orig_config
                    self._budget_spans = orig_spans
                    self._work = orig_work

                # Records for this value, in emission order (re-trace offset).
                recs = sorted(((bv, off) for cv, bv, off in alt_patches
                               if cv == val), key=lambda r: r[1])
                # One record per site; a mismatch means the tagged write is
                # conditional on the value.
                if len(recs) != len(ordered_sites):
                    raise CompileError(
                        f"param {param_name!r} value {val!r} produces "
                        f"{len(recs)} patch record(s) but the param has "
                        f"{len(ordered_sites)} site(s); tag a write that runs "
                        f"for every value.")
                # Re-trace offsets must be identical across values; a moving
                # shape means value-dependent bytecode the sites can't track.
                these_offsets = tuple(off for _bv, off in recs)
                if ref_offsets is None:
                    ref_offsets = these_offsets
                elif these_offsets != ref_offsets:
                    raise CompileError(
                        f"param {param_name!r} patch sites move between values "
                        f"({list(ref_offsets)} vs {list(these_offsets)}); "
                        f"configure() must emit the same bytecode shape for "
                        f"every value of a patched param.")
                for site, (bv, _off) in zip(ordered_sites, recs):
                    site.value_map[val] = bv

        # Flatten: sites ordered by offset within each param (matches the
        # deterministic wire order in image._write_param).
        result: List[PatchEntry] = []
        for lst in sites_by_param.values():
            result.extend(sorted(lst, key=lambda s: s.offset))
        return result


