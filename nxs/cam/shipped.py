# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The shipped points of a mode: per camera count and lane count the fps
range, the line the port runs and the trigger frame it syncs at. Every
"is this mode shipped" decision goes through here; how a point was proven
lives in the repository's ledger, never on the unit."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple


def cameras(links) -> int:
    """The camera count a set of links makes: 1 for one camera link, 2 for
    more; a link with a pod and no camera counts for none."""
    heads = [l for l in links if getattr(l, "sensor_compatible", l) is not None]
    return 1 if len(heads) <= 1 else 2


def entries(descriptor, mode: str) -> List[Dict[str, Any]]:
    """The shipped points of a mode, in declaration order."""
    block = descriptor.shipped_points() if hasattr(descriptor, "shipped_points") else {}
    return [dict(e) for e in block.get(mode) or []]


def entry(descriptor, mode: str, cameras: int, csi_lanes: int) -> Optional[Dict[str, Any]]:
    """The point a mode carries for a camera count on a lane count, if any."""
    for point in entries(descriptor, mode):
        if (int(point.get("cameras", 0)) == int(cameras)
                and int(point.get("csi_lanes", 0)) == int(csi_lanes)):
            return point
    return None


def fps_range(descriptor, mode: str, cameras: int,
              csi_lanes: int) -> Optional[Tuple[float, float]]:
    """(floor, ceiling) a mode ships for a camera count, None when it ships
    none."""
    point = entry(descriptor, mode, cameras, csi_lanes)
    if point is None:
        return None
    return float(point["fps"]["floor"]), float(point["fps"]["ceiling"])


def derived_point(descriptor, mode: str, csi_lanes: int, hmax: int) -> Optional[Dict[str, Any]]:
    """The pair point a mode's one-camera point implies at the line `hmax`
    the hub's output leaves it: the same floor, the ceiling scaled by the
    lines (a frame keeps its lines), no trigger frame, `derived` set. None
    without a one-camera point on these lanes, at a line shorter than that
    point's, or when the ceiling falls under the floor."""
    solo = entry(descriptor, mode, 1, csi_lanes)
    if solo is None or int(hmax) < int(solo["hmax"]):
        return None
    floor = float(solo["fps"]["floor"])
    ceiling = round(float(solo["fps"]["ceiling"]) * int(solo["hmax"]) / int(hmax), 2)
    if ceiling < floor:
        return None
    return {"cameras": 2, "csi_lanes": int(csi_lanes),
            "fps": {"floor": floor, "ceiling": ceiling}, "hmax": int(hmax),
            "derived": True}


def operating_point(point: Dict[str, Any]) -> Dict[str, int]:
    """The line length a point runs at, and its trigger frame if synced."""
    out = {"hmax": int(point["hmax"])}
    if point.get("trigger_vmax") is not None:
        out["trigger_vmax"] = int(point["trigger_vmax"])
    return out


def synced(point: Dict[str, Any]) -> bool:
    """Whether the point runs under frame sync (it carries the trigger frame)."""
    return point.get("trigger_vmax") is not None


def shipped_modes(descriptor, cameras: int, csi_lanes: int) -> List[str]:
    """The modes a customer may run with a camera count on a lane count:
    the unit program's modes that carry a point for it, in declaration
    order."""
    return [name for name in descriptor.program_modes()
            if entry(descriptor, name, cameras, csi_lanes) is not None]


def label(cameras: int) -> str:
    """`1 camera` or `2 cameras`."""
    return f"{int(cameras)} camera{'s' if int(cameras) > 1 else ''}"
