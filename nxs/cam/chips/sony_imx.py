# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The Sony image-sensor family: standby-wrapped programs over STANDBY and
XMSTA, REGHOLD-wrapped live knobs, and the frame, exposure, and gain laws,
every constant read from the descriptor or stated by the family's
datasheet with its section.

The frame-length law a mode is judged by is the datasheet's recommended
frame V_TR: the register list's row `sony.vmax`, else
`timing.min_frame_length`, else the rows plus the blanking delta from
`timing.delta_kind` through the wait-register formula, or from
`limits.min_frame_length_delta`. A rate is the law at the mode's own line
length: VMAX = INCK / (HMAX x fps), the same arithmetic the capture stack's
frame-rate control runs; the fastest is V_TR's, the slowest the longest
frame the register holds or the capture stack's `capture.framerate.min_fps`.
Exposure needs the shutter register, `integration_offset_us`,
`min_integration_lines`, and a floor (`shs_floor`, or the `shs_floor_regs`
waits); a fast-trigger frame's output delay needs a `fast` preset and the
GMRWT and GMTWT waits; gain needs `gain_max`, gain in
dB `gain_reg_per_db`; the black level per bit depth is `program.blklevel`.
Programs need their sleep set under `program:`; a knob whose facts are
absent is not offered."""

from __future__ import annotations

import math
from typing import Any, Callable, Dict, List, Optional, Tuple

from nxs.cam.contracts import InfeasibleConfig, MipiContract
from nxs.cam.descriptors import SEN, Descriptor, to_int, w
from nxs.cam.chips.sony_imx_knobs import (
    FREERUN, _KnobMixin, _require_positive)

#: The shutter register by shutter kind: SHS on the global-shutter parts,
#: SHR0 on the rolling-shutter ones.
SHUTTER_REGS = ("SHS", "SHR0")
#: The trigger registers a preset writes, in program order; a preset names
#: their values by the lower-case register name.
TRIGGER_REGS = ("TRIGMODE", "VINT_EN")
#: VINT_EN (the family datasheet's V-interrupt enable register): bit0
#: VINT_EN, bit1 VINT_EN_NOR, bits [7:2] the readout mode's field.
VINT_BITS = 0x03
VINT_MODE_SHIFT = 2
#: The kernel's trigger control slots, in order.
TRIGGER_SLOTS = (FREERUN, "fast", "sequential")
#: The fast trigger's data output delay (datasheet 20.7.3):
#: t_TGDLY = 0.42 us + 4H + GMRWT + GMTWT, the waits in lines.
OUTPUT_DELAY_US = 0.42
OUTPUT_DELAY_LINES = 4
OUTPUT_DELAY_WAITS = ("GMRWT", "GMTWT")


class SonyImx(_KnobMixin):
    """One descriptor's laws and knobs; `surface()` names what it offers."""

    def __init__(self, descriptor: Descriptor) -> None:
        self._d = descriptor

    # --- facts ------------------------------------------------------------
    def descriptor(self) -> Descriptor:
        return self._d

    def default_mode(self) -> str:
        """The descriptor-declared default operating mode."""
        return str(self._d.raw("default_mode"))

    def _timing(self, mode: str) -> Dict[str, Any]:
        return self._d.modes[mode]["timing"]

    def _inck(self) -> int:
        return int(self._d.limits["inck_hz"])

    def _program(self, key: str) -> Optional[Dict[str, Any]]:
        return (self._d.raw("program") or {}).get(key)

    def _has(self, *regs: str) -> bool:
        return all(r in self._d.registers for r in regs)

    def _shutter_reg(self) -> Optional[str]:
        return next((r for r in SHUTTER_REGS if self._has(r)), None)

    def _trigger_presets(self) -> Dict[str, Dict[str, Any]]:
        trig = self._d.raw("trigger") or {}
        return {str(k): v for k, v in trig.items()
                if isinstance(v, dict) and "trigmode" in v}

    def _sync_roles(self) -> Dict[str, int]:
        roles = (self._d.raw("sync") or {}).get("syncsel") or {}
        return {str(k): to_int(v) for k, v in roles.items()}

    def _exposure_facts(self) -> bool:
        limits = self._d.limits
        floor = ("shs_floor" in limits
                 or ("shs_floor_regs" in limits and "captured_waits" in limits))
        return (self._shutter_reg() is not None
                and "integration_offset_us" in limits
                and "min_integration_lines" in limits and floor)

    def _delta_formula_facts(self) -> bool:
        limits = self._d.limits
        return "captured_waits" in limits and "frame_length_delta_const" in limits

    def _output_delay_facts(self) -> bool:
        waits = self._d.limits.get("captured_waits") or {}
        return ("fast" in self._trigger_presets()
                and all(reg in waits for reg in OUTPUT_DELAY_WAITS))

    def _gain_db_law(self) -> Optional[Tuple[int, int]]:
        """Register steps per dB as (numerator, denominator)."""
        law = self._d.limits.get("gain_reg_per_db")
        if law is None:
            return None
        if isinstance(law, (list, tuple)):
            return int(law[0]), int(law[1])
        return int(law), 1

    # --- register writes --------------------------------------------------
    def _write(self, name: str, value: int, comment: Optional[str] = None,
               sleep_ms: int = 0) -> List[Dict[str, Any]]:
        """One write per byte of a register, in address order by its declared
        width and byte order; the settle rides the last byte."""
        spec = self._d.registers[name]
        addr = to_int(spec["addr"])
        width = int(spec.get("width", 1))
        value = int(value)
        if not 0 <= value < (1 << (8 * width)):
            raise InfeasibleConfig(
                f"{name} {value} does not fit its {width}-byte register")
        chunks = [(value >> (8 * i)) & 0xFF for i in range(width)]
        if str(spec.get("order", "le")) == "be":
            chunks.reverse()
        steps = []
        for i, byte in enumerate(chunks):
            steps.append(w(SEN, addr + i, byte,
                           sleep_ms=sleep_ms if i == width - 1 else 0,
                           comment=comment if i == 0 else None))
        return steps

    def _hold(self, steps: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """A live change under REGHOLD: the bytes land in one frame."""
        return (self._write("REGHOLD", 1, comment="REGHOLD") + steps
                + self._write("REGHOLD", 0))

    def _trigger_writes(self, trigger: str,
                        mode: Optional[str] = None) -> List[Dict[str, Any]]:
        """TRIGMODE and VINT_EN for a preset. A mode that declares
        `timing.vint_mode` (the readout mode's VINT_EN[7:2] field from its
        register list) gets the preset's two interrupt bits under it; a
        mode without one takes the preset value whole."""
        presets = self._trigger_presets()
        if trigger not in presets:
            raise InfeasibleConfig(
                f"{self._d.compatible} has no {trigger!r} trigger mode",
                alternatives=[f"trigger {p}" for p in presets])
        preset = presets[trigger]
        steps: List[Dict[str, Any]] = []
        for reg in TRIGGER_REGS:
            steps += self._write(reg, self._trigger_value(reg, preset, mode),
                                 comment=f"{reg} {trigger}")
        return steps

    def _trigger_value(self, reg: str, preset: Dict[str, Any],
                       mode: Optional[str]) -> int:
        value = to_int(preset[reg.lower()])
        if reg != "VINT_EN":
            return value
        field = self._timing(mode or self.default_mode()).get("vint_mode")
        if field is None:
            return value
        return (int(field) << VINT_MODE_SHIFT) | (value & VINT_BITS)

    # --- laws -------------------------------------------------------------
    def export_mipi_contract(self, mode: str,
                             fps: Optional[float] = None) -> MipiContract:
        """The MIPI contract of a mode; `fps` defaults to the mode's own
        ceiling (the datasheet frame at its line length)."""
        m = self._d.modes[mode]
        geo = m["geometry"]
        if fps is None:
            fps = self.fps_ceiling(mode)
        return MipiContract(
            lanes=int(geo["lanes"]),
            rate_mbps=int(geo["rate_mbps"]),
            data_type=str(m["mipi"]["data_type"]),
            bit_depth=int(geo["bit_depth"]),
            width=int(geo["width"]),
            height=int(geo["height"]),
            fps=float(fps),
            trigger_input=m["mipi"].get("trigger_input"),
            embedded_lines=int(m["mipi"].get("embedded_lines", 0)),
        )

    def vmax_capacity(self) -> int:
        """The longest frame the register holds: `limits.vmax_max` where
        the datasheet bounds it, else the register's width."""
        declared = self._d.limits.get("vmax_max")
        if declared is not None:
            return to_int(declared)
        width = int(self._d.registers["VMAX"].get("width", 1))
        return (1 << (8 * width)) - 1

    def fps_ceiling(self, mode: str) -> float:
        """The highest free-run rate: the timing law at the mode's
        recommended frame V_TR and its line, INCK / (HMAX x V_TR)."""
        hmax = int(self._timing(mode)["hmax"])
        return self._inck() / (hmax * self.recommended_frame_length(mode))

    def fps_floor(self, mode: Optional[str] = None) -> float:
        """The lowest free-run rate: the timing law at the longest frame
        the register holds, raised to the capture stack's minimum rate
        (`capture.framerate.min_fps`, the vendor driver's)."""
        mode = mode or self.default_mode()
        hmax = int(self._timing(mode)["hmax"])
        floor = self._inck() / (hmax * self.vmax_capacity())
        framerate = (self._d.raw("capture") or {}).get("framerate") or {}
        if framerate.get("min_fps") is not None:
            floor = max(floor, float(framerate["min_fps"]))
        return floor

    def vmax_for_fps(self, fps: float, hmax: int) -> int:
        """VMAX for a frame rate at a line length."""
        _require_positive("fps", fps)
        return round(self._inck() / (hmax * fps))

    def line_time_us(self, hmax: int) -> float:
        """One line period in microseconds at the given HMAX."""
        return hmax * 1e6 / self._inck()

    def frame_length_delta_formula(self, kind: str) -> int:
        """Minimum blanking from the captured wait registers:
        GMRWT + 2*GMRWT2 + GMTWT + GSDLY + C(kind)."""
        waits = self._d.limits["captured_waits"]
        const = int(self._d.limits["frame_length_delta_const"][kind])
        return (int(waits["GMRWT"]) + 2 * int(waits["GMRWT2"])
                + int(waits["GMTWT"]) + int(waits["GSDLY"]) + const)

    def _color(self) -> bool:
        """Whether the part is the colour variant (`meta.chromacity`,
        colour when undeclared)."""
        return str((self._d.raw("meta") or {}).get("chromacity", "color")) == "color"

    def _frame_length_delta(self, mode: str,
                            color: Optional[bool] = None) -> Tuple[int, str]:
        """The native blanking delta of a mode and the fact it came from;
        `color` overrides the part's declared chromacity."""
        kind = self._timing(mode).get("delta_kind")
        if kind:
            return self.frame_length_delta_formula(str(kind)), f"formula[{kind}]"
        table = self._d.limits.get("min_frame_length_delta")
        if isinstance(table, dict):
            bits = int(self._d.modes[mode]["geometry"]["bit_depth"])
            if color is None:
                color = self._color()
            key = f"{'color' if color else 'mono'}{bits}"
            return int(table[key]), f"min_frame_length_delta[{key}]"
        if table is not None:
            return int(table), "min_frame_length_delta"
        raise InfeasibleConfig(
            f"{self._d.compatible} declares no frame-length law for {mode} "
            "(timing.min_frame_length, timing.delta_kind, or "
            "limits.min_frame_length_delta)")

    def _native_minimum(self, mode: str, color: Optional[bool] = None) -> int:
        timing = self._timing(mode)
        if timing.get("min_frame_length") is not None:
            return int(timing["min_frame_length"])
        height = int(self._d.modes[mode]["geometry"]["height"])
        return height + self._frame_length_delta(mode, color)[0]

    def _recommended(self, mode: str, color: Optional[bool] = None) -> Tuple[int, str]:
        """V_TR of a mode and the fact that states it: the register list's
        row with its section, else the declared frame, else the rows plus
        the blanking law (`color` picks the variant of a blanking table)."""
        row = self._d.modes[mode].get("sony") or {}
        if row.get("vmax") is not None:
            return int(row["vmax"]), f"datasheet {row['section']}"
        timing = self._timing(mode)
        if timing.get("min_frame_length") is not None:
            return int(timing["min_frame_length"]), "the declared min_frame_length"
        height = int(self._d.modes[mode]["geometry"]["height"])
        delta, law = self._frame_length_delta(mode, color)
        return height + delta, f"active {height} + {law} {delta}"

    def recommended_frame_length(self, mode: str) -> int:
        """The frame length V_TR a mode's register list notes: the
        datasheet row's (`sony.vmax`), else a declared
        `timing.min_frame_length`, else rows plus the blanking law. The
        datasheet guarantees the imaging characteristics at it."""
        return self._recommended(mode)[0]

    def trigger_frame_length(self, mode: str) -> int:
        """The frame length the fast-trigger program fixes for a mode: a
        declared `timing.trigger_vmax` (an experimental overlay's), else
        its recommended frame V_TR, which reads the whole picture."""
        declared = self._timing(mode).get("trigger_vmax")
        if declared is not None:
            return int(declared)
        return self.recommended_frame_length(mode)

    def data_output_delay_lines(self, mode: str, hmax: int) -> int:
        """How many lines late a fast-trigger frame's valid pixels arrive,
        rounded up to whole lines: t_TGDLY = 0.42 us + 4H + GMRWT + GMTWT
        (datasheet 20.7.3) at the line `hmax`, the waits the captured ones.
        A capture window of the mode's rows ends in that many lines of
        filler.

        Raises:
            InfeasibleConfig: For a mode whose frame does not follow from
                the captured waits (one declared whole runs its own).
        """
        if self._timing(mode).get("delta_kind") is None:
            raise InfeasibleConfig(
                f"{self._d.compatible} {mode}: its frame does not follow from the "
                f"captured waits, so the laws carry no output delay for it")
        waits = self._d.limits["captured_waits"]
        lines = OUTPUT_DELAY_LINES + sum(int(waits[reg]) for reg in OUTPUT_DELAY_WAITS)
        return lines + math.ceil(OUTPUT_DELAY_US / self.line_time_us(hmax))

    def validate_vmax(self, vmax: int, mode: str, color: Optional[bool] = None) -> None:
        """The frame-length law: no shorter than the mode's recommended
        frame V_TR (`_recommended`, whose fact the refusal names), no
        longer than the register holds.

        Raises:
            InfeasibleConfig: If VMAX is outside the law.
        """
        minimum, source = self._recommended(mode, color)
        if vmax < minimum:
            raise InfeasibleConfig(
                f"VMAX {vmax} < {minimum} (the recommended frame of {mode}, {source}): "
                "the frame comes up short below the minimum vertical blanking",
                alternatives=[f"vmax {minimum} (the recommended frame, {source})"])
        capacity = self.vmax_capacity()
        if vmax > capacity:
            raise InfeasibleConfig(
                f"VMAX {vmax} exceeds the register's {capacity}",
                alternatives=[f"vmax {capacity} (the longest frame)"])

    def shs_floor(self) -> int:
        """Minimum shutter value: the `shs_floor` constant, else the sum of
        the `shs_floor_regs` waits."""
        limits = self._d.limits
        if "shs_floor" in limits:
            return int(limits["shs_floor"])
        waits = limits["captured_waits"]
        return sum(int(waits[r]) for r in limits["shs_floor_regs"])

    def shs_for_exposure_us(self, exposure_us: float, hmax: int, vmax: int) -> int:
        """Shutter value for an integration time (free-run; the shutter is
        inert under fast trigger): lines = (us - offset) / 1H, SHS = VMAX -
        lines, floored at the wait sum and capped one line short of the frame."""
        offset = float(self._d.limits["integration_offset_us"])
        min_lines = int(self._d.limits["min_integration_lines"])
        lines = round((exposure_us - offset) / self.line_time_us(hmax))
        lines = max(min_lines, min(lines, vmax - min_lines))
        return max(self.shs_floor(), vmax - lines)


    # --- readings ---------------------------------------------------------
    def derive_status(self, readings: Dict[str, int]) -> List[str]:
        """Derived status lines from raw probe readings."""
        lines: List[str] = []
        hmax, vmax = readings.get("hmax"), readings.get("vmax")
        if hmax and vmax:
            inck = self._inck()
            lines.append(f"-> sensor rate ~= {inck / (hmax * vmax):.1f} fps "
                         "(INCK/(HMAX*VMAX))")
        shs = readings.get("shs")
        if shs is not None and vmax and self._exposure_facts():
            lines.append(f"-> SHS {shs} vs VMAX {vmax}"
                         + (" (SHS >= VMAX: near-zero exposure!)" if shs >= vmax else ""))
        return lines

    def knob_readback(self, readings: Dict[str, int]) -> Dict[str, str]:
        """Derived knob values from raw probe readings: fps from the timing
        registers, exposure from the shutter where the law is declared."""
        derived: Dict[str, str] = {}
        hmax, vmax = readings.get("hmax"), readings.get("vmax")
        if not (hmax and vmax):
            return derived
        inck = self._inck()
        derived["fps"] = f"{inck / (hmax * vmax):.2f}"
        shs = readings.get("shs")
        if shs is not None and "integration_offset_us" in self._d.limits:
            offset = float(self._d.limits["integration_offset_us"])
            exposure_us = max(vmax - shs, 0) * hmax / inck * 1e6 + offset
            derived["exposure"] = f"{exposure_us:.0f} us"
        return derived

    def sync_capability(self) -> Tuple[bool, str]:
        """Whether the sensor takes the hub's frame-sync pulse, and how."""
        sync = self._d.raw("sync") or {}
        return bool(sync.get("takes_trigger")), str(sync.get("text", ""))

    # --- the offered surface ---------------------------------------------
    def surface(self) -> Dict[str, Callable[..., Any]]:
        """Name -> bound callable for every law, program, and knob whose
        facts the descriptor declares."""
        prog = self._d.raw("program") or {}
        presets = self._trigger_presets()
        names = ["descriptor", "default_mode", "export_mipi_contract",
                 "fps_ceiling", "fps_floor", "vmax_for_fps", "vmax_capacity",
                 "validate_vmax", "recommended_frame_length",
                 "trigger_frame_length", "line_time_us",
                 "derive_status", "knob_readback"]
        if self._adbit_facts():
            names.append("adbit_monosel")
        if self._program("init_inck_hz") is not None:
            names.append("check_inck")
        if self._has("STANDBY"):
            names.append("expect_alive")
            if "stop_ms" in prog:
                names.append("stop")
            if self._has("XMSTA"):
                if "start" in prog:
                    names.append("start")
                if "restart" in prog:
                    names.append("restart_stream")
                if "timing_start" in prog and self._has("HMAX", "VMAX"):
                    names.append("knob_timing_start")
                if presets and self._has(*TRIGGER_REGS):
                    if "trigger_switch" in prog:
                        names.append("knob_trigger")
                    if "fast" in presets and "fast_trigger" in prog and self._has("VMAX"):
                        names.append("knob_fast_trigger")
                if self._sync_roles() and "sync_switch" in prog and self._has("SYNCSEL"):
                    names.append("knob_sync")
        if self._delta_formula_facts():
            names.append("frame_length_delta_formula")
        if self._output_delay_facts():
            names.append("data_output_delay_lines")
        if self._has("REGHOLD"):
            if self._has("VMAX"):
                names.append("knob_fps")
            if self._exposure_facts():
                names += ["shs_floor", "shs_for_exposure_us", "knob_exposure"]
            if self._has("GAIN") and "gain_max" in self._d.limits:
                names.append("knob_gain")
                if self._gain_db_law() is not None:
                    names += ["knob_gain_db", "gain_db_max"]
        if self._d.raw("test_pattern"):
            names.append("knob_test_pattern")
        if "takes_trigger" in (self._d.raw("sync") or {}):
            names.append("sync_capability")
        return {name: getattr(self, name) for name in names}

    # --- the kernel control table ----------------------------------------
    def control_rows(self) -> List[Tuple[str, str, int, Tuple[int, ...]]]:
        """(control, register, form, parameters) for every control the
        descriptor has the facts for; the host's contract generator reads
        them."""
        rows: List[Tuple[str, str, int, Tuple[int, ...]]] = []
        limits = self._d.limits
        if self._has("REGHOLD"):
            rows.append(("group-hold", "REGHOLD", 1, (1, 0)))
        law = self._gain_db_law()
        if law is not None and "gain_max" in limits and self._has("GAIN"):
            # The capture stack hands the kernel dB x the capture table's gain
            # factor (`use_decibel_gain`), so the register law divides it out:
            # reg = value x (steps per dB) / factor. Without the factor a 13.9
            # dB request landed as 139 dB, clamped to the register's 48 dB,
            # and auto-exposure hunted between the clamp and the floor.
            factor = int(((self._d.raw("capture") or {}).get("gain") or {})
                         .get("factor", 1))
            rows.append(("gain", "GAIN", 1,
                         (law[0], law[1] * factor, self._gain_max())))
        if self._exposure_facts():
            offset_ns = round(float(limits["integration_offset_us"]) * 1000)
            rows.append(("exposure", self._shutter_reg(), 1,
                         (offset_ns, self.shs_floor(),
                          int(limits["min_integration_lines"]))))
        if self._has("VMAX"):
            mode = self.default_mode()
            height = int(self._d.modes[mode]["geometry"]["height"])
            rows.append(("frame-length", "VMAX", 1,
                         (self._native_minimum(mode) - height,)))
        if self._has("HMAX"):
            rows.append(("line-length", "HMAX", 0, ()))
        if self._has("STANDBY"):
            rows.append(("standby", "STANDBY", 0, (1, 0)))
        if self._has("XMSTA"):
            rows.append(("start", "XMSTA", 0, (0,)))
        presets = self._trigger_presets()
        if all(slot in presets for slot in TRIGGER_SLOTS) and self._has(*TRIGGER_REGS):
            # The kernel writes whole registers: VINT_EN carries the
            # default mode's field over each preset's interrupt bits.
            for control, reg in (("trigger", "TRIGMODE"), ("vint-en", "VINT_EN")):
                rows.append((control, reg, 0, tuple(
                    self._trigger_value(reg, presets[slot], None)
                    for slot in TRIGGER_SLOTS)))
        roles = self._sync_roles()
        if {"master", "slave"} <= set(roles) and self._has("SYNCSEL"):
            rows.append(("sync-sel", "SYNCSEL", 0, (roles["master"], roles["slave"])))
        tp = self._d.raw("test_pattern")
        if tp and len(tp.get("enable") or {}) == 1:
            (reg, on), = tp["enable"].items()
            off = tp["disable"][reg]
            rows.append(("test-pattern", str(reg), 0,
                         (to_int(off), to_int(on), self._d.reg(str(tp["select"])))))
        if self._has("BLKLEVEL"):
            rows.append(("black-level", "BLKLEVEL", 0, ()))
        return rows
