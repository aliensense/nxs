# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0
"""The host capture budget: what every camera adds up to against what the
host was measured to sustain. Advisory only, a warning that names the rate
that fits, never a refusal."""
from __future__ import annotations

from typing import Iterable, Optional, Tuple

Stream = Tuple[str, int, int, int, float]   # (port, cameras, width, height, fps)


def total_mpix_s(streams: Iterable[Stream]) -> float:
    return sum(c * w * h * fps for _, c, w, h, fps in streams) / 1e6


def budget_note(streams: Iterable[Stream], budget: Optional[float],
                port: str) -> Optional[str]:
    """One sentence when the streams exceed the host budget, else None. Names
    the per-camera rate that fits and the rate for `port` with the others as
    they are."""
    streams = list(streams)
    if budget is None or not streams:
        return None
    total = total_mpix_s(streams)
    if total <= budget:
        return None
    cams = sum(c for _, c, _, _, _ in streams)
    pixels_per_frame = sum(c * w * h for _, c, w, h, _ in streams) / cams
    each = budget * 1e6 / (cams * pixels_per_frame)
    others = total_mpix_s(s for s in streams if s[0] != port)
    mine = [s for s in streams if s[0] == port]
    parts = " + ".join(f"{c} x {w}x{h} at {fps:g} fps" for _, c, w, h, fps in streams)
    text = (f"host budget: {parts} = {total:.0f} MP/s; this host delivered "
            f"~{budget:.0f} MP/s with viewers running — expect drops. Fits: "
            f"{each:.0f} fps on every camera")
    if mine and others < budget:
        # Every stream of the port at once (a mixed hub lists one per
        # link): the rate that fits is over their summed pixels per frame.
        per_frame = sum(c * w * h for _, c, w, h, _ in mine)
        fits = (budget - others) * 1e6 / per_frame
        text += f", or {fits:.0f} fps on {port} with the others as they are"
    return text
