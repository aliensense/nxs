# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The native family: a head that programs its own sensor and serves the
camera map alone, the smart camera. The descriptor's `limits:`
are the map's physical bounds (the FPGA's specification is the head's
datasheet) and each mode states its line time and its frame floor; the
family derives the apply law from them and offers the host nothing to
write: no alive poll, no stream gate writes, no timing program, no control
rows."""

from __future__ import annotations

import math
from typing import Any, Callable, Dict, List, Optional, Tuple

from nxs.cam.contracts import InfeasibleConfig, MipiContract
from nxs.cam.descriptors import Descriptor, to_int
from nxs.cam.laws import ApplyLaw, camera_options

#: Capture units per decibel: the camera map converts millidecibels with it.
GAIN_FACTOR = 10
#: Millidecibels per capture unit.
MDB_PER_UNIT = 1000 // GAIN_FACTOR
#: The limits the apply law is built from, in microseconds and millidecibels.
LAW_LIMITS = ("exposure_min_us", "exposure_margin_us", "frame_period_max_us",
              "gain_min_mdb", "gain_max_mdb", "gain_step_mdb")
#: The fields a head serves in its own logic when its limits name them.
SERVED = ("black_level", "flips", "temperature")
#: A mode's timing facts: its line period and its frame floor.
MODE_TIMING = ("line_time_ns", "frame_period_min_us")


def _ceil_lines(us: int, line_ns: int) -> int:
    """Microseconds as whole lines, a minimum rounded up."""
    return math.ceil(us * 1000 / line_ns)


class Native:
    """One descriptor's native surface; `surface()` names what it offers."""

    def __init__(self, descriptor: Descriptor) -> None:
        self._d = descriptor

    def descriptor(self) -> Descriptor:
        return self._d

    def default_mode(self) -> str:
        """The descriptor-declared default operating mode."""
        return str(self._d.raw("default_mode"))

    def _timing(self, mode: str) -> Dict[str, Any]:
        return self._d.modes[mode].get("timing") or {}

    def _declared_fps(self, mode: str) -> float:
        fps = self._timing(mode).get("fps")
        if fps is None:
            raise InfeasibleConfig(
                f"{self._d.compatible} declares no fps for {mode} "
                "(timing.fps): a native head runs its modes at a declared rate")
        return float(fps)

    def _mode_timing(self, mode: str) -> Tuple[int, int]:
        """`(line_time_ns, frame_period_min_us)` of a mode; a mode without
        either, or with a zero, is refused by name."""
        timing = self._timing(mode)
        values = []
        for key in MODE_TIMING:
            value = to_int(timing.get(key, 0))
            if value < 1:
                raise InfeasibleConfig(
                    f"{self._d.compatible} {mode} states no timing.{key}: "
                    f"the camera map converts its periods with it")
            values.append(value)
        return values[0], values[1]

    def _gain_units(self, key: str) -> int:
        """A gain limit in capture units; a value off the unit's grid is refused."""
        mdb = to_int(self._d.limits[key])
        if mdb < 0 or mdb % MDB_PER_UNIT:
            raise InfeasibleConfig(
                f"{self._d.compatible} limits.{key} {mdb} mdB is not a multiple of "
                f"{MDB_PER_UNIT} mdB, the camera map's gain unit")
        return mdb // MDB_PER_UNIT

    def _law_facts(self) -> bool:
        limits = self._d.limits
        return (all(key in limits for key in LAW_LIMITS)
                and all(all(key in self._timing(mode) for key in MODE_TIMING)
                        for mode in self._d.program_modes()))

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
        """A mode's declared rate, the one the head runs it at."""
        return self._declared_fps(mode)

    def fps_floor(self) -> float:
        """The slowest rate: the head's longest frame period
        (`limits.frame_period_max_us`), 1 fps without one."""
        longest = self._d.limits.get("frame_period_max_us")
        return 1e6 / to_int(longest) if longest else 1.0

    def sync_capability(self) -> Tuple[bool, str]:
        """Whether the head takes the port's frame-sync pulse, and how."""
        sync = self._d.raw("sync") or {}
        return bool(sync.get("takes_trigger")), str(sync.get("text", "no trigger input declared"))

    def apply_law(self) -> ApplyLaw:
        """The constants the camera map converts a commit with, from the
        descriptor's physical limits: the frame and exposure bounds in
        lines, the gain in capture units at `GAIN_FACTOR` per decibel. The
        law head's lines (the frame length ceiling, the exposure minimum
        and its margin) are counted at the default mode's line time; each
        mode's law row keeps its own line time and frame floor. The bounds
        in microseconds ride beside them as the facts state them, since
        the rows the camera map serves are those. The law names no
        register: the head converts and writes in its own logic, so the
        map stages no apply parameter."""
        limits = self._d.limits
        modes = self._d.program_modes()
        rows = [self._mode_timing(mode) for mode in modes]
        line_times = tuple(line for line, _floor in rows)
        floors = tuple(_ceil_lines(floor_us, line) for line, floor_us in rows)
        longest_us = to_int(limits["frame_period_max_us"])
        for mode, (line, floor_us) in zip(modes, rows):
            if floor_us > longest_us:
                raise InfeasibleConfig(
                    f"{self._d.compatible} {mode}: frame_period_min_us {floor_us} is past "
                    f"limits.frame_period_max_us {longest_us}")
        default_line = line_times[modes.index(self.default_mode())]
        gain_min, gain_max, gain_step = (self._gain_units(key) for key in
                                         ("gain_min_mdb", "gain_max_mdb", "gain_step_mdb"))
        if gain_min > gain_max:
            raise InfeasibleConfig(
                f"{self._d.compatible} limits.gain_min_mdb {gain_min * MDB_PER_UNIT} is past "
                f"limits.gain_max_mdb {gain_max * MDB_PER_UNIT}")
        black = limits.get("black_level") or {}
        return ApplyLaw(
            hold=None, frame_length=None, shutter=None, gain=None,
            frame_length_min=min(floors),
            frame_length_max=longest_us * 1000 // default_line,
            frame_length_floors=floors, line_times_ns=line_times,
            exposure_min=_ceil_lines(to_int(limits["exposure_min_us"]), default_line),
            exposure_margin=_ceil_lines(to_int(limits["exposure_margin_us"]), default_line),
            shs_floor=0, shutter_inverted=False,
            gain_num=1, gain_den=1, gain_reg_max=gain_max,
            gain_min=gain_min, gain_max=gain_max, gain_step=max(1, gain_step),
            gain_factor=GAIN_FACTOR,
            stream_gate=bool(limits.get("stream_gate")),
            black_level_min=to_int(black.get("min", 0)),
            black_level_max=to_int(black.get("max", 0)),
            serves=tuple(name for name in SERVED if limits.get(name)),
            exposure_min_us=to_int(limits["exposure_min_us"]),
            exposure_margin_us=to_int(limits["exposure_margin_us"]),
            frame_period_max_us=longest_us,
            frame_periods_min_us=tuple(floor_us for _line, floor_us in rows))

    def options(self, view) -> Dict[str, Any]:
        """What the family narrows on its camera's rows: the rate row spans
        the declared mode's rates, from the longest frame's to the rate the
        mode declares (`camera_options`)."""
        return camera_options(view, self._d, lambda mode: (self.fps_floor(), self.fps_ceiling(mode)))

    def surface(self) -> Dict[str, Callable[..., Any]]:
        """Name -> bound callable: the facts and rates every family answers,
        and the apply law when the limits and every mode's timing state it."""
        names = ["descriptor", "default_mode", "export_mipi_contract", "fps_ceiling",
                 "fps_floor", "sync_capability", "options"]
        if self._law_facts():
            names.append("apply_law")
        return {name: getattr(self, name) for name in names}

    def control_rows(self) -> List[Tuple[str, str, int, Tuple[int, ...]]]:
        """No kernel control row: the host's camera driver speaks to the
        head through its camera map."""
        return []


def bind(descriptor: Descriptor) -> Native:
    """The family bound to a descriptor's facts."""
    return Native(descriptor)
