# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The register writes a Sony knob produces: the standby-wrapped start, the trigger and sync selections, and the live rate, exposure, gain and test-pattern edits."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from nxs.cam.contracts import InfeasibleConfig
from nxs.cam.descriptors import SEN, expect, to_int

#: ADBIT_MONOSEL (18.3): bit0 keep 1, bit2 MONOSEL (1 monochrome), bits
#: [5:4] ADBIT (the code per bit depth is the descriptor's program.adbit).
ADBIT_KEEP = 0x01
MONOSEL_BIT = 1 << 2
ADBIT_SHIFT = 4

#: The trigger slot a free-running mode takes.
FREERUN = "freerun"


def _require_positive(name: str, value: float) -> None:
    """Knob inputs divide sensor timing; zero, negative, and non-finite
    values are refused, never a traceback."""
    if not math.isfinite(value) or value <= 0:
        raise InfeasibleConfig(f"{name} must be finite and positive, "
                               f"got {value}")


class _KnobMixin:
    """The knob surface of `SonyImx`."""

    # --- programs ---------------------------------------------------------
    def knob_timing_start(
        self,
        hmax: int,
        vmax: int,
        mode: Optional[str] = None,
        trigger: str = FREERUN,
        exposure_us: Optional[float] = None,
        gain: Optional[int] = None,
        inck_hz: Optional[int] = None,
    ) -> List[Dict[str, Any]]:
        """Standby-wrapped timing program + sensor start: STANDBY -> repairs
        -> AD depth and chromacity -> trigger regs -> SYNCSEL -> HMAX/VMAX
        -> black level -> shutter/gain -> release STANDBY -> release XMSTA.
        A mode without a declared exposure or gain keeps the program's own;
        VMAX answers to the frame law (`validate_vmax`); `inck_hz` is the
        clock the pod feeds the sensor, checked against the one the init
        table is set for."""
        prog = self._program("timing_start") or {}
        mode = mode or self.default_mode()
        timing = self._timing(mode)
        if exposure_us is None:
            exposure_us = timing.get("exposure_us")
        if gain is None:
            gain = timing.get("gain")
        self.validate_vmax(vmax, mode)
        self.check_inck(inck_hz)
        presets = self._trigger_presets()
        if trigger != FREERUN and trigger not in presets:
            raise InfeasibleConfig(
                f"{self._d.compatible} has no {trigger!r} trigger mode",
                alternatives=[f"trigger {p}" for p in presets])
        steps = self._write("STANDBY", 1, sleep_ms=int(prog.get("standby_ms", 0)),
                            comment="standby for timing program")
        for reg, value in (prog.get("repairs") or {}).items():
            steps += self._write(str(reg), to_int(value),
                                 comment=f"{reg} repair (idempotent)")
        steps += self._adbit_monosel(mode)
        if presets:
            steps += self._trigger_writes(trigger, mode)
        role = prog.get("syncsel")
        if role:
            steps += self._write("SYNCSEL", self._sync_roles()[str(role)],
                                 comment=f"SYNCSEL {role}")
        steps += self._write("HMAX", hmax, comment=f"HMAX {hmax}")
        steps += self._write("VMAX", vmax, comment=f"VMAX {vmax}")
        steps += self._blklevel(mode)
        if exposure_us is not None and self._exposure_facts():
            shutter = self._shutter_reg()
            shs = self.shs_for_exposure_us(float(exposure_us), hmax, vmax)
            exposure_ms = (vmax - shs) * self.line_time_us(hmax) / 1000
            steps += self._write(
                shutter, shs,
                comment=f"{shutter} {shs} (~{exposure_ms:.1f} ms exposure)")
        if gain is not None and self._has("GAIN"):
            steps += self._write("GAIN", int(gain), comment=f"analog gain {gain}")
        steps += self._write("STANDBY", 0, sleep_ms=int(prog.get("release_ms", 0)),
                             comment="release standby")
        steps += self._write("XMSTA", 0, sleep_ms=int(prog.get("start_ms", 0)),
                             comment="master start")
        return steps

    def adbit_monosel(self, mode: Optional[str] = None) -> List[Dict[str, Any]]:
        """The ADBIT_MONOSEL write for a mode, for a program composed
        outside the timing program (a captured program replayed whole):
        the caller places it where the sensor sits in standby."""
        return self._adbit_monosel(mode or self.default_mode())

    def _adbit_facts(self) -> bool:
        return bool(self._program("adbit")) and self._has("ADBIT_MONOSEL")

    def _adbit_monosel(self, mode: str) -> List[Dict[str, Any]]:
        """The AD conversion depth and colour/monochrome register (the
        family datasheet's ADBIT/MONOSEL register): the ADBIT code of the
        mode's bit depth from `program.adbit` (AD depth equals output
        depth), MONOSEL
        from `meta.chromacity`, bit0 kept set. Nothing for a sensor that
        declares no code table; a bit depth without a code is refused
        with the depths the table knows."""
        if not self._adbit_facts():
            return []
        codes = {int(k): to_int(v) for k, v in self._program("adbit").items()}
        bits = int(self._d.modes[mode]["geometry"]["bit_depth"])
        if bits not in codes:
            raise InfeasibleConfig(
                f"{self._d.compatible} has no ADBIT code for {bits}-bit output",
                alternatives=[f"a {b}-bit mode" for b in sorted(codes)])
        chromacity = str((self._d.raw("meta") or {}).get("chromacity", "color"))
        value = ADBIT_KEEP | (codes[bits] << ADBIT_SHIFT)
        if chromacity == "mono":
            value |= MONOSEL_BIT
        return self._write("ADBIT_MONOSEL", value,
                           comment=f"AD {bits}-bit, {chromacity} (ADBIT_MONOSEL)")

    def _blklevel(self, mode: str) -> List[Dict[str, Any]]:
        """The black-level register for a mode's bit depth (`program.blklevel`,
        the recommended value per depth): the mode tables end on one
        depth's value, so the timing program writes the mode's own.
        Nothing for a sensor that declares no table."""
        table = self._program("blklevel")
        if not table or not self._has("BLKLEVEL"):
            return []
        values = {int(k): to_int(v) for k, v in table.items()}
        bits = int(self._d.modes[mode]["geometry"]["bit_depth"])
        if bits not in values:
            raise InfeasibleConfig(
                f"{self._d.compatible} has no black level for {bits}-bit output",
                alternatives=[f"a {b}-bit mode" for b in sorted(values)])
        return self._write("BLKLEVEL", values[bits],
                           comment=f"black level {values[bits]} ({bits}-bit)")

    def check_inck(self, inck_hz: Optional[int]) -> None:
        """A link's declared pod clock against the one the init table is
        set for (`program.init_inck_hz`, the datasheet's INCK-dependent
        rows): another clock needs its own table, so it is refused."""
        declared = self._program("init_inck_hz")
        if inck_hz is None or declared is None:
            return
        if int(inck_hz) != int(declared):
            raise InfeasibleConfig(
                f"{self._d.compatible}: the init table is set for a "
                f"{int(declared) / 1e6:g} MHz INCK, the link declares "
                f"{int(inck_hz) / 1e6:g} MHz (the datasheet's INCK-dependent "
                "rows differ per clock)",
                alternatives=[f"inck_hz {int(declared)}"])

    def knob_fast_trigger(self, trigger_vmax: int,
                          mode: Optional[str] = None) -> List[Dict[str, Any]]:
        """Enter fast trigger from normal mode at the given VMAX, through
        standby: the datasheet routes every shutter-mode transition through
        it, and re-entering trigger mode from a triggered pair stalls.
        `mode` owns the VINT_EN field (the default mode when None)."""
        prog = self._program("fast_trigger") or {}
        steps = self._write("STANDBY", 1, sleep_ms=int(prog.get("standby_ms", 0)),
                            comment="standby for trigger switch")
        steps += self._trigger_writes("fast", mode)
        steps += self._write("VMAX", int(trigger_vmax), comment=f"VMAX {trigger_vmax}")
        steps += self._write("STANDBY", 0, sleep_ms=int(prog.get("release_ms", 0)))
        steps += self._write("XMSTA", 0, sleep_ms=int(prog.get("start_ms", 0)))
        return steps

    def knob_trigger(self, trigger: str,
                     mode: Optional[str] = None) -> List[Dict[str, Any]]:
        """Switch the trigger preset (standby-wrapped); `mode` owns the
        VINT_EN field (the default mode when None)."""
        prog = self._program("trigger_switch") or {}
        steps = self._write("STANDBY", 1, sleep_ms=int(prog.get("standby_ms", 0)),
                            comment="standby for trigger switch")
        steps += self._trigger_writes(trigger, mode)
        steps += self._write("STANDBY", 0, sleep_ms=int(prog.get("release_ms", 0)))
        steps += self._write("XMSTA", 0, sleep_ms=int(prog.get("start_ms", 0)))
        return steps

    def knob_sync(self, role: str) -> List[Dict[str, Any]]:
        """XVS/XHS role: master drives the pads, slave receives external
        sync. The XMASTER pin is a carrier line, set by the flow."""
        roles = self._sync_roles()
        if role not in roles:
            raise InfeasibleConfig(
                f"unknown sync role {role!r}",
                alternatives=[f"sync {r}" for r in sorted(roles)])
        prog = self._program("sync_switch") or {}
        steps = self._write("STANDBY", 1, sleep_ms=int(prog.get("standby_ms", 0)),
                            comment="standby for sync role switch")
        steps += self._write("SYNCSEL", roles[role], comment=f"SYNCSEL {role}")
        steps += self._write("STANDBY", 0, sleep_ms=int(prog.get("release_ms", 0)))
        steps += self._write("XMSTA", 0, sleep_ms=int(prog.get("start_ms", 0)),
                             comment="restart")
        return steps

    def restart_stream(self) -> List[Dict[str, Any]]:
        """Stop/start dance: STANDBY and XMSTA cycled."""
        prog = self._program("restart") or {}
        return (self._write("STANDBY", 1, sleep_ms=int(prog.get("standby_ms", 0)))
                + self._write("XMSTA", 1, sleep_ms=int(prog.get("stop_ms", 0)))
                + self._write("STANDBY", 0, sleep_ms=int(prog.get("release_ms", 0)))
                + self._write("XMSTA", 0, sleep_ms=int(prog.get("start_ms", 0))))

    def expect_alive(self, timeout_ms: int = 1500,
                     soft: bool = False) -> List[Dict[str, Any]]:
        """Poll the sensor answering (any STANDBY readback): the settle
        after its reset line is released."""
        return [expect(SEN, self._d.reg("STANDBY"), 0x00, mask=0x00,
                       timeout_ms=timeout_ms, poll_ms=20,
                       comment="sensor answers", soft=soft)]

    def stop(self) -> List[Dict[str, Any]]:
        """Pause the sensor (standby)."""
        prog = self._d.raw("program") or {}
        return self._write("STANDBY", 1, sleep_ms=int(prog.get("stop_ms", 0)),
                           comment="sensor standby")

    def start(self) -> List[Dict[str, Any]]:
        """Release standby and master-start."""
        prog = self._program("start") or {}
        return (self._write("STANDBY", 0, sleep_ms=int(prog.get("release_ms", 0)),
                            comment="release standby")
                + self._write("XMSTA", 0, sleep_ms=int(prog.get("start_ms", 0)),
                              comment="master start"))

    # --- live knobs -------------------------------------------------------
    def _vmax_writes(self, vmax: int, mode: Optional[str] = None) -> List[Dict[str, Any]]:
        """Live VMAX change, REGHOLD-wrapped; the same law as the start."""
        self.validate_vmax(vmax, mode or self.default_mode())
        return self._hold(self._write("VMAX", vmax, comment=f"VMAX {vmax}"))

    def _hmax_writes(self, hmax: int) -> List[Dict[str, Any]]:
        """Live HMAX change, REGHOLD-wrapped."""
        return self._hold(self._write("HMAX", hmax, comment=f"HMAX {hmax}"))

    def knob_fps(self, fps: float, hmax: Optional[int] = None,
                 mode: Optional[str] = None) -> List[Dict[str, Any]]:
        """Live frame-rate change via VMAX (validated) at the running line
        length, the mode's when none is given."""
        mode = mode or self.default_mode()
        if hmax is None:
            hmax = int(self._timing(mode)["hmax"])
        return self._vmax_writes(self.vmax_for_fps(fps, hmax), mode)

    def knob_exposure(self, exposure_us: float, hmax: Optional[int] = None,
                      vmax: Optional[int] = None) -> List[Dict[str, Any]]:
        """Live exposure change via the shutter register, REGHOLD-wrapped
        (free-run only; under fast trigger the pulse width integrates).
        Defaults: the default mode's line length and its recommended frame."""
        _require_positive("exposure", exposure_us)
        mode = self.default_mode()
        if hmax is None:
            hmax = int(self._timing(mode)["hmax"])
        if vmax is None:
            vmax = self.recommended_frame_length(mode)
        shutter = self._shutter_reg()
        shs = self.shs_for_exposure_us(exposure_us, hmax, vmax)
        actual = (vmax - shs) * self.line_time_us(hmax)
        return self._hold(self._write(
            shutter, shs,
            comment=f"{shutter} {shs} (~{actual / 1000:.1f} ms exposure)"))

    def _gain_max(self) -> int:
        return to_int(self._d.limits["gain_max"])

    def gain_db_max(self) -> Optional[float]:
        """The highest analog gain in dB the register holds, None without the dB law."""
        law = self._gain_db_law()
        if law is None:
            return None
        return self._gain_max() * law[1] / law[0]

    def knob_gain(self, value: int) -> List[Dict[str, Any]]:
        """Live analog gain register value, refused above `gain_max`."""
        maximum = self._gain_max()
        if not 0 <= int(value) <= maximum:
            db_max = self.gain_db_max()
            in_db = f" (0..{db_max:g} dB)" if db_max is not None else ""
            raise InfeasibleConfig(
                f"gain {value} outside 0..{maximum}{in_db}",
                alternatives=[f"gain {min(max(int(value), 0), maximum)}"])
        return self._hold(self._write("GAIN", int(value),
                                      comment=f"analog gain {value}"))

    def knob_gain_db(self, db: float) -> List[Dict[str, Any]]:
        """Analog gain in dB: reg = dB * gain_reg_per_db, up to gain_max."""
        num, den = self._gain_db_law()
        max_db = self.gain_db_max()
        if not 0 <= db <= max_db:
            raise InfeasibleConfig(
                f"gain {db:g} dB outside 0..{max_db:g} dB",
                alternatives=[f"gain_db {min(max(db, 0), max_db):g}"])
        reg = round(db * num / den)
        return self._hold(self._write("GAIN", reg,
                                      comment=f"gain {db:g} dB ({reg})"))

    def knob_test_pattern(self, value: str) -> List[Dict[str, Any]]:
        """Sensor test pattern by code, or "off": the transport-vs-sensor
        bisector."""
        tp = self._d.raw("test_pattern")
        if value == "off":
            return [step for reg, val in tp["disable"].items()
                    for step in self._write(str(reg), to_int(val),
                                            comment="test pattern off")]
        codes = {str(k): to_int(v) for k, v in tp["codes"].items()}
        if value not in codes:
            raise InfeasibleConfig(
                f"unknown test pattern {value!r}",
                alternatives=[f"test_pattern {c}" for c in sorted(codes)]
                + ["test_pattern off"])
        steps = [step for reg, val in tp["enable"].items()
                 for step in self._write(str(reg), to_int(val),
                                         comment="test pattern on")]
        return steps + self._write(str(tp["select"]), codes[value],
                                   comment=f"pattern {value}")
