# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The table-only family: what a part gets when no law family fits it.
Modes are fixed at their declared rate, every control is a direct register
write clamped by the `controls:` table, and nothing is derived: no frame
law, no exposure arithmetic, no trigger. `sync_capability` is always
free-run. The `standby` control is the stream gate: it states its two
values, and the family starts and stops the stream with it."""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional, Tuple

from nxs.cam.contracts import InfeasibleConfig, MipiContract
from nxs.cam.descriptors import SEN, Descriptor, expect, to_int, w

#: Controls the kernel scales rather than writes verbatim: gain is register
#: steps per dB (form 1), given by a control's `num`/`den`.
SCALED_CONTROLS = ("gain",)
#: The control that gates the stream (the kernel vocabulary's `standby`
#: row): its `standby` and `streaming` values are the register's two states.
STREAM_GATE = "standby"


class Generic:
    """One descriptor's table-only surface; `surface()` names what it offers."""

    def __init__(self, descriptor: Descriptor) -> None:
        self._d = descriptor

    def descriptor(self) -> Descriptor:
        return self._d

    def default_mode(self) -> str:
        """The descriptor-declared default operating mode."""
        return str(self._d.raw("default_mode"))

    def _controls(self) -> Dict[str, Dict[str, Any]]:
        return {str(k): v for k, v in (self._d.raw("controls") or {}).items()}

    def _declared_fps(self, mode: str) -> float:
        fps = (self._d.modes[mode].get("timing") or {}).get("fps")
        if fps is None:
            raise InfeasibleConfig(
                f"{self._d.compatible} declares no fps for {mode} "
                "(timing.fps): a table-only part runs its modes at a fixed rate")
        return float(fps)

    def export_mipi_contract(self, mode: str,
                             fps: Optional[float] = None) -> MipiContract:
        """The MIPI contract of a mode at its declared rate."""
        m = self._d.modes[mode]
        geo = m["geometry"]
        return MipiContract(
            lanes=int(geo["lanes"]),
            rate_mbps=int(geo["rate_mbps"]),
            data_type=str(m["mipi"]["data_type"]),
            bit_depth=int(geo["bit_depth"]),
            width=int(geo["width"]),
            height=int(geo["height"]),
            fps=float(fps) if fps is not None else self._declared_fps(mode),
            trigger_input=m["mipi"].get("trigger_input"),
            embedded_lines=int(m["mipi"].get("embedded_lines", 0)),
        )

    def fps_ceiling(self, mode: str) -> float:
        """A fixed mode's only rate."""
        return self._declared_fps(mode)

    def fps_floor(self) -> float:
        return float(self._d.limits.get("min_fps", 1))

    def sync_capability(self) -> Tuple[bool, str]:
        """A table-only part free-runs under a port trigger."""
        sync = self._d.raw("sync") or {}
        return False, str(sync.get("text", "no trigger input declared"))

    def _alive_reg(self) -> Optional[int]:
        prog = self._d.raw("program") or {}
        if prog.get("alive_reg"):
            return self._d.reg(str(prog["alive_reg"]))
        meta = self._d.raw("meta") or {}
        if meta.get("device_id_reg") is not None:
            return to_int(meta["device_id_reg"])
        return None

    def expect_alive(self, timeout_ms: int = 1500,
                     soft: bool = False) -> List[Dict[str, Any]]:
        """Poll the sensor answering on its alive register."""
        return [expect(SEN, self._alive_reg(), 0x00, mask=0x00,
                       timeout_ms=timeout_ms, poll_ms=20,
                       comment="sensor answers", soft=soft)]

    def derive_status(self, readings: Dict[str, int]) -> List[str]:
        return []

    def knob_readback(self, readings: Dict[str, int]) -> Dict[str, str]:
        return {}

    def _write(self, name: str, value: int, comment: str) -> List[Dict[str, Any]]:
        spec = self._d.registers[name]
        addr = to_int(spec["addr"])
        width = int(spec.get("width", 1))
        chunks = [(value >> (8 * i)) & 0xFF for i in range(width)]
        if str(spec.get("order", "le")) == "be":
            chunks.reverse()
        return [w(SEN, addr + i, byte, comment=comment if i == 0 else None)
                for i, byte in enumerate(chunks)]

    def _stream_gate(self) -> Optional[Dict[str, Any]]:
        """The `standby` control, the stream gate (the schema requires its
        two values). A value outside the control's range, or wider than its
        register, is refused: a write masks to the register's width, and a
        gate that lands on the wrong value leaves the part where it was."""
        control = self._controls().get(STREAM_GATE)
        if control is None or "standby" not in control or "streaming" not in control:
            return None
        spec = self._d.registers[str(control["reg"])]
        top = (1 << (8 * int(spec.get("width", 1)))) - 1
        low, high = max(to_int(control["min"]), 0), min(to_int(control["max"]), top)
        for state in ("standby", "streaming"):
            value = to_int(control[state])
            if not low <= value <= high:
                raise InfeasibleConfig(
                    f"{self._d.compatible}: the stream gate's {state} value {value} is "
                    f"outside {control['reg']}'s {low}..{high}")
        if to_int(control["standby"]) == to_int(control["streaming"]):
            raise InfeasibleConfig(
                f"{self._d.compatible}: the stream gate's standby and streaming values "
                f"are both {to_int(control['standby'])}, so it gates nothing")
        return control

    def start(self) -> List[Dict[str, Any]]:
        """The stream gate's streaming value."""
        gate = self._stream_gate()
        return self._write(str(gate["reg"]), to_int(gate["streaming"]), comment="stream on")

    def stop(self) -> List[Dict[str, Any]]:
        """The stream gate's standby value."""
        gate = self._stream_gate()
        return self._write(str(gate["reg"]), to_int(gate["standby"]), comment="stream off")

    def _knob(self, name: str, control: Dict[str, Any]) -> Callable[[int], List[Dict[str, Any]]]:
        low, high = to_int(control["min"]), to_int(control["max"])
        reg = str(control["reg"])

        def knob(value: int) -> List[Dict[str, Any]]:
            if not low <= int(value) <= high:
                raise InfeasibleConfig(
                    f"{name} {value} outside {low}..{high}",
                    alternatives=[f"{name} {min(max(int(value), low), high)}"])
            return self._write(reg, int(value), comment=f"{name} {value}")

        knob.__name__ = f"knob_{name}"
        knob.__doc__ = f"Direct {reg} write, {low}..{high}."
        return knob

    def surface(self) -> Dict[str, Callable[..., Any]]:
        names = ["descriptor", "default_mode", "export_mipi_contract",
                 "fps_ceiling", "fps_floor", "sync_capability", "derive_status",
                 "knob_readback"]
        if self._alive_reg() is not None:
            names.append("expect_alive")
        gate = self._stream_gate()
        if gate is not None:
            names += ["start", "stop"]
        offered: Dict[str, Callable[..., Any]] = {n: getattr(self, n) for n in names}
        for name, control in self._controls().items():
            if gate is not None and name == STREAM_GATE:
                continue        # the gate is the flows', never a live knob
            offered[f"knob_{name}"] = self._knob(name, control)
        return offered

    def control_rows(self) -> List[Tuple[str, str, int, Tuple[int, ...]]]:
        """One kernel row per declared control: gain scaled by its
        `num`/`den` (register steps per dB) up to `max`, the stream gate
        with its standby and streaming values, the rest verbatim."""
        rows: List[Tuple[str, str, int, Tuple[int, ...]]] = []
        gate = self._stream_gate()
        for name, control in self._controls().items():
            kernel = name.replace("_", "-")
            if gate is not None and name == STREAM_GATE:
                rows.append((kernel, str(gate["reg"]), 0,
                             (to_int(gate["standby"]), to_int(gate["streaming"]))))
            elif name in SCALED_CONTROLS:
                rows.append((kernel, str(control["reg"]), 1,
                             (to_int(control.get("num", 1)),
                              to_int(control.get("den", 1)),
                              to_int(control["max"]))))
            else:
                rows.append((kernel, str(control["reg"]), 0, ()))
        return rows
