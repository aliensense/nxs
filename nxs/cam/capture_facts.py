# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The capture-side facts of a sensor for the host's capture table. A
table row that states no line length or top rate takes them from the
unit-program mode of its geometry and depth: the line length from the
mode's HMAX in the table's pixel clock (the Sony parts count their line
in INCK cycles), the top rate from the shipped ceiling, else the laws'
(the serializer's tail counted when the pack knows it). A stated value
stands: a part whose line is not HMAX in the pixel clock, or a mode
without a program, says its own."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from . import shipped
from .contracts import ContractError, InfeasibleConfig


def _program_mode_for(descriptor, entry: Dict[str, Any]) -> Optional[str]:
    """The unit-program mode a table row describes, by geometry and depth."""
    key = (int(entry["width"]), int(entry["height"]), int(entry["bit_depth"]))
    for name in descriptor.program_modes():
        geo = descriptor.modes[name].get("geometry") or {}
        if (int(geo.get("width", 0)), int(geo.get("height", 0)),
                int(geo.get("bit_depth", 0))) == key:
            return name
    return None


def _call(fn, *args, **kwargs):
    """Call a family law with the keyword arguments it takes."""
    import inspect
    try:
        params = inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return fn(*args, **kwargs)
    return fn(*args, **{k: v for k, v in kwargs.items() if k in params})


def _law_module(pack, descriptor):
    chip_module = getattr(pack, "chip_module", None)
    if chip_module is not None:
        module = chip_module(descriptor.compatible)
        if module is not None:
            return module
    from . import chips
    return chips.bind(descriptor)


def _tail_us(pack, descriptor) -> Optional[float]:
    """The serializer's measured pixel-mode tail, from the pack's flows
    (None without a pack, or for a pack whose flows carry no tail)."""
    flows = getattr(pack, "flows", None)
    if flows is None:
        return None
    readings = getattr(flows(), "status_readings", None)
    if readings is None:
        return None
    try:
        tail = readings(pack, descriptor.compatible).get("pixel_tail_us")
    except Exception:
        return None
    return float(tail) if tail is not None else None


def derived_rows(pack, descriptor) -> List[Dict[str, Any]]:
    """The descriptor's `capture.table` with every row's `line_length`,
    `max_fps` and `default_fps` in place: what a row states stands, the
    rest is derived from the unit-program mode of its geometry (a row with
    neither and no such mode is refused by name). A row a unit served
    states the line and the top rate its trailer carries and takes its
    default rate here, the same way a derived row does, so every host
    books the same frame."""
    cap = descriptor.raw("capture") or {}
    rows: List[Dict[str, Any]] = []
    laws: Dict[str, Any] = {}

    def free_run_for(mode: str, timing: Dict[str, Any], inck: int) -> float:
        """The mode's free-run ceiling: the shipped points' top, else the
        law's clean frame at the mode's line, the serializer's tail
        counted, at the resolution the trailer carries a rate."""
        ceilings = [float(e["fps"]["ceiling"]) for e in shipped.entries(descriptor, mode)]
        if ceilings:
            return round(max(ceilings), 3)
        if not laws:
            laws["module"] = _law_module(pack, descriptor)
            laws["tail_us"] = _tail_us(pack, descriptor)
        tail_rows = None
        if laws["tail_us"] is not None:
            tail_rows = math.ceil(laws["tail_us"] * inck / int(timing["hmax"]) / 1e6)
        try:
            return round(float(_call(laws["module"].fps_ceiling, mode, tail_rows=tail_rows)), 3)
        except InfeasibleConfig as exc:
            raise ContractError(f"{descriptor.name}: no top rate for {mode}: {exc}") from exc

    for entry in cap.get("table") or []:
        row = dict(entry)
        mode = _program_mode_for(descriptor, entry)
        if mode is None:
            # A row no pod program can run stays out of the boot table: the
            # capture stack's tuning file holds seven knob sets past its
            # template's, and a row that can never stream would take one.
            continue
        timing = descriptor.modes[mode].get("timing") or {}
        inck = descriptor.limits.get("inck_hz")
        missing = [k for k in ("line_length", "max_fps") if k not in row]
        derivable = timing.get("hmax") is not None and inck is not None
        if missing and not derivable:
            raise ContractError(
                f"{descriptor.name}: capture row {entry['width']}x{entry['height']} "
                f"RAW{entry['bit_depth']} states no {', '.join(missing)} and no "
                f"unit-program mode derives them")
        if "line_length" not in row:
            row["line_length"] = round(int(timing["hmax"]) * int(cap["pix_clk_hz"]) / int(inck))
        if "max_fps" in row:
            # A stated top rate's default: the free-run ceiling where a law
            # gives one, never above the stated top; a table-only part's
            # stated rate is its only rate. A row a unit served and the
            # tree's own row read the same, whatever else the row states.
            if "default_fps" not in row:
                default = float(row["max_fps"])
                if derivable:
                    try:
                        default = min(default, free_run_for(mode, timing, int(inck)))
                    except (ContractError, InfeasibleConfig):
                        pass
                row["default_fps"] = default
            rows.append(row)
            continue
        hmax = int(timing["hmax"])
        free_run = free_run_for(mode, timing, int(inck))
        # A consumer that sets no rate gets the free-run ceiling. The row's
        # top rate also admits the trigger frame, a shorter frame than a
        # measured clean free-run one: caps above the top are clamped by
        # the capture stack and the kernel's frame-rate control would then
        # move the frame the trigger program fixed.
        row["default_fps"] = free_run
        row["max_fps"] = free_run
        trigger_law = getattr(laws.get("module") or _law_module(pack, descriptor),
                              "trigger_frame_length", None)
        if trigger_law is not None:
            tail_rows = None
            if laws.get("tail_us") is not None:
                tail_rows = math.ceil(laws["tail_us"] * int(inck) / hmax / 1e6)
            try:
                frame = int(_call(trigger_law, mode, tail_rows=tail_rows))
            except (InfeasibleConfig, KeyError, TypeError, ValueError):
                frame = 0
            if frame > 0:
                row["max_fps"] = max(free_run, round(int(inck) / (hmax * frame), 3))
        rows.append(row)
    return rows
