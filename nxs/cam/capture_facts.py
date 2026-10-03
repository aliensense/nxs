# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The capture-side facts of a sensor for the host's capture table. A
table row that states no line length or top rate takes them from the
unit-program mode of its geometry and depth: the line length from the
mode's HMAX in the table's pixel clock (the Sony parts count their line
in INCK cycles), the top rate from the mode's own ceiling or one camera's
lane law on the port's lanes, whichever is lower, and the default rate
from the port's declaration, never past the mode's own ceiling. A stated
value stands: a part whose line is not HMAX in the pixel clock, or a mode
without a program, says its own."""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from .contracts import FPS_DEFAULT, ContractError, InfeasibleConfig


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


def _lane_law(pack):
    """The pack's lane law for one camera, `lane_ceiling(lanes, contract)`;
    None for a pack whose flows state none (a port without a hub)."""
    flows = getattr(pack, "flows", None)
    return getattr(flows(), "lane_ceiling", None) if flows is not None else None


def derived_rows(pack, descriptor, lanes: Optional[int] = None,
                 default_fps: Optional[float] = None) -> List[Dict[str, Any]]:
    """The descriptor's `capture.table` with every row's `line_length`,
    `max_fps` and `default_fps` in place, for a port of `lanes` CSI lanes
    that runs at `default_fps`: what a row states stands, the rest is
    derived from the unit-program mode of its geometry (a row with neither
    and no such mode is refused by name). The top rate is the mode's own
    ceiling (the family's datasheet frame at the mode's line, INCK / (HMAX
    x V_TR)) or one camera's lane law on the lanes, whichever is lower, and
    the ceiling alone without lanes or without the pack's lane law (a port
    without a hub); the default is the declared rate, FPS_DEFAULT without
    one, never above the top nor, on the frame law, the mode's own
    ceiling, and a table-only part's stated rate is its only rate. A
    unit's trailer carries the family's ceiling where the yaml states no
    top rate: a host derives that row as the tree's is derived, so every
    host books the same frame."""
    cap = descriptor.raw("capture") or {}
    rows: List[Dict[str, Any]] = []
    laws: Dict[str, Any] = {}
    lane_law = _lane_law(pack) if lanes is not None else None

    def module():
        if not laws:
            laws["module"] = _law_module(pack, descriptor)
        return laws["module"]

    def ceiling_for(mode: str) -> float:
        """The mode's free-run ceiling: the law's datasheet frame at the
        mode's line, at the resolution the trailer carries a rate."""
        try:
            return round(float(_call(module().fps_ceiling, mode)), 3)
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
        derivable = timing.get("hmax") is not None and inck is not None
        if (descriptor.from_unit and derivable and "max_fps" in row
                and float(row["max_fps"]) == ceiling_for(mode)):
            # The trailer's rate for a row its yaml leaves to the laws.
            del row["max_fps"]
        missing = [k for k in ("line_length", "max_fps") if k not in row]
        if missing and not derivable:
            raise ContractError(
                f"{descriptor.name}: capture row {entry['width']}x{entry['height']} "
                f"RAW{entry['bit_depth']} states no {', '.join(missing)} and no "
                f"unit-program mode derives them")
        if "line_length" not in row:
            row["line_length"] = round(int(timing["hmax"]) * int(cap["pix_clk_hz"]) / int(inck))
        if "max_fps" not in row:
            # The capture stack's tuning build takes a row past the sensor's
            # own ceiling for a mode it cannot open, and writes no knob set.
            row["max_fps"] = (round(min(float(lane_law(int(lanes), module().export_mipi_contract(mode))),
                                        ceiling_for(mode)), 3)
                              if lane_law is not None else ceiling_for(mode))
        if "default_fps" not in row:
            rate = float(FPS_DEFAULT if default_fps is None else default_fps)
            if derivable and hasattr(module(), "vmax_for_fps"):
                # A mode on the frame law never defaults past its own ceiling.
                rate = min(rate, ceiling_for(mode))
            row["default_fps"] = min(float(row["max_fps"]), rate) if derivable else float(row["max_fps"])
        rows.append(row)
    return rows
