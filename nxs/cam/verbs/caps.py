"""`caps`: what the port's sensor offers, every mode with its range, and the knobs."""

from __future__ import annotations

import argparse
import dataclasses
import json
from typing import Any, Dict, List, Optional


from nxs.cam import port_state
from nxs.cam import timing as cam_timing
from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs.cam.descriptors import mode_token
from nxs.cam.select import _link_descriptor, _hub_for, _port_name, _refuse_pod_only, select_port_links
from nxs.host import capture_table as tables


def _camera_knobs(hub, link: LinkSpec, topology=None) -> List[str]:
    """The knobs the host names for a link: the hub's, and the port's
    `sync` where a node on the port generates frame sync (a hub's)."""
    knobs = set(_knob_names(hub, link))
    if topology is None or not topology.is_direct:
        knobs.add("sync")
    return sorted(knobs)


def _hub_fsync_rate(hub, i2c, topology) -> Optional[float]:
    """The hub generator's live frame rate through the deserializer
    module's `fsync_rate` hook; None when the hub has no such hook or
    the generator is off."""
    from nxs.cam.diag import run_probes

    try:
        module = hub.chip_module(topology.des_compatible)
    except Exception:
        return None
    hook = getattr(module, "fsync_rate", None) if module else None
    if hook is None:
        return None
    desd = hub.descriptor(topology.des_compatible)
    readings = {r.name: r.raw for r in run_probes(i2c, topology.des_addr, desd)
                if r.raw is not None}
    try:
        return hook(readings)
    except Exception:
        return None


def _knob_names(hub, link: LinkSpec) -> List[str]:
    """The link's knobs from hubs that know per-sensor surfaces, the
    hub's list otherwise."""
    fn = hub.flows().knob_names
    try:
        return list(fn(hub, link))
    except TypeError:
        return list(fn(hub))


def _cameras_label(cameras: int) -> str:
    """`1 camera` or `2 cameras`."""
    return f"{int(cameras)} camera{'s' if int(cameras) > 1 else ''}"


def _runs_alone(hub, topology: Topology, alone: Topology, link: LinkSpec, name: str) -> bool:
    """On a pair, whether the range law refuses `name` for the pair (the
    pair's program does not carry it) while one camera runs it on the port
    `alone`."""
    try:
        cam_timing.lawful_range(hub, topology, link, name)
    except InfeasibleConfig:
        pass
    else:
        return False
    try:
        return cam_timing.lawful_range(hub, alone, link, name) is not None
    except InfeasibleConfig:
        return False


def _own_ceiling(module, name: str) -> Optional[float]:
    """The sensor's own ceiling for a mode the port does not run."""
    try:
        return round(float(module.fps_ceiling(name)), 2)
    except Exception:
        return None


def caps_payload(topology: Topology, hub, link: LinkSpec,
                 port: Optional[Topology] = None) -> Dict[str, Any]:
    """The `caps` surface: what the link's sensor offers on this port, from
    the hub and the unit's records. A mode runs on the port when the unit's
    program carries it, the table the port boots carries its row (one pixel
    format per table, `capture_row`) and, on a pair, the pair's program
    carries it: such a mode lists its range, on a pair its line, and the
    whole rates frame sync runs it at. A mode the unit's program does not
    carry is marked `offered: false` with the sensor's own ceiling, one the
    pair's program does not carry `runs_alone` and one the table boots no
    row for `capture_row: false`, each without a range or a line. ``port``
    is the whole port whose table the host boots, where ``topology`` is
    narrowed to the links judged together."""
    from nxs.schemas import CONTRACT

    from nxs.cam.contracts import shown_rate

    send = _link_descriptor(hub, topology, link)
    # The line the laws give, where the port runs one the delivery check found.
    plain = (_link_descriptor(hub, dataclasses.replace(topology, lines=()), link)
             if topology.lines else send)
    guarantee = getattr(hub.flows(), "frame_guarantee", None)
    module = hub.chip_module(link.sensor_compatible.split(",")[-1])
    cameras = topology.cameras
    if port is None:
        port = topology
    narrow = getattr(hub.flows(), "port_links", None)
    alone_port = (narrow(topology, [link]) if narrow is not None
                  else dataclasses.replace(topology, links=(link,)))
    modes = []
    together = []
    for name, mode in send.modes.items():
        geo = mode["geometry"]
        offered = name in send.program_modes()
        index = tables.port_index(hub, port, send, name)
        # A descriptor without capture rows has no table to judge.
        capture_row = (index is not None) if send.raw("capture") else None
        alone = offered and cameras > 1 and _runs_alone(hub, topology, alone_port, link, name)
        runs = offered and capture_row is not False and not alone
        if runs:
            together.append(name)
        hmax = (mode.get("timing") or {}).get("hmax")
        line_ns = (round(float(module.line_time_us(int(hmax))) * 1000)
                   if runs and hmax is not None and hasattr(module, "line_time_us") else None)
        # The mode's lawful range on this port; the sensor's own ceiling for
        # a mode without a program.
        floor = ceiling = rates = None
        if runs:
            try:
                rates = cam_timing.lawful_range(hub, topology, link, name)
            except Exception:
                rates = None
            if rates is not None:
                floor, ceiling = rates.shown()
            else:
                ceiling = _own_ceiling(module, name)
        elif not offered:
            ceiling = _own_ceiling(module, name)
        # A slower rate stretches the frame past the datasheet's guaranteed one.
        guaranteed = (guarantee(hub, topology, link, name)
                      if guarantee is not None and rates is not None else None)
        modes.append({
            "name": name, "token": mode_token(send, name),
            "width": int(geo["width"]), "height": int(geo["height"]),
            "data_type": str(mode["mipi"]["data_type"]),
            "lanes": int(geo["lanes"]), "rate_mbps": int(geo["rate_mbps"]),
            "fps_ceiling": ceiling, "fps_floor": floor,
            "fps_guaranteed": shown_rate(guaranteed) if guaranteed is not None else None,
            "line_time_ns": line_ns,
            # A program mode runs the datasheet's line (the sensor's, or the
            # pair line law's), or the delivery check's longer one on the rig.
            "line_source": (("found" if hmax != (plain.modes[name].get("timing") or {}).get("hmax")
                             else "datasheet") if line_ns is not None else None),
            "offered": offered,
            "runs_alone": alone,
            "capture_row": capture_row,
            "sensor_mode": index,
        })
    trig = send.raw("trigger") or {}
    knobs = _camera_knobs(hub, link, topology)
    pairs = []
    if cameras > 1:
        for name in together:
            rates = cam_timing.synced_rates(hub, topology, name)
            if rates:
                pairs.append({"mode": name, "token": mode_token(send, name),
                              "fps_floor": float(min(rates)),
                              "fps_ceiling": float(max(rates))})
    payload = {
        "contract": CONTRACT,
        "port": _port_name(topology), "link": link.name,
        "sensor": send.compatible,
        "csi_lanes": int(topology.csi_lanes), "hub": hub.name,
        "cameras": cameras,
        "modes": modes,
        "sync_pairs": pairs,
        "knobs": knobs,
        "trigger_modes": sorted(k for k in trig if isinstance(trig[k], dict)
                                and "trigmode" in trig[k]),
    }
    if link.ser_compatible is not None:
        # The nodes on the link: a serializer where the link has one.
        payload["serializer"] = link.ser_compatible
    return payload


def _rate_text(fps) -> str:
    return f"{fps:.10g}" if float(fps).is_integer() else f"{fps:.2f}"


def _range_text(floor, ceiling) -> str:
    if floor is None and ceiling is None:
        return "no lawful rate"
    if floor is None:
        return f"<= {_rate_text(ceiling)} fps"
    return f"{_rate_text(floor)}–{_rate_text(ceiling)} fps"


def cmd_caps(args: argparse.Namespace) -> int:
    """What a link may run on the port the selection makes: one link named
    is one camera, the port form the pair."""
    topology, selected = select_port_links(args)
    hub = _hub_for(topology)
    # A pair's modes run at the line the delivery check found for the port.
    topology = port_state.with_found_lines(topology)
    # The table is the whole port's, whichever links the selection judges.
    port = topology
    narrow = getattr(hub.flows(), "port_links", None)
    if selected and narrow is not None:
        topology = narrow(topology, selected)
    link = selected[0] if selected else topology.links[0]
    if not link.has_camera:
        raise _refuse_pod_only(topology, link, "caps")
    payload = caps_payload(topology, hub, link, port=port)
    if getattr(args, "json", False):
        print(json.dumps(payload, indent=2))
        return 0
    section = getattr(hub.flows(), "GUARANTEED_FRAME_SECTION", None)
    print(f"{payload['port']} ({payload['csi_lanes']} CSI lanes, "
          f"{_cameras_label(payload['cameras'])})")
    print(f"link {link.name}  {payload['sensor']}")
    for mode in payload["modes"]:
        geometry = f"{mode['width']}x{mode['height']}"
        line = f"  {geometry:<10} {mode['data_type']:<6} "
        if not mode["offered"]:
            line += f"{_range_text(mode['fps_floor'], mode['fps_ceiling'])}  [no unit program]"
        elif mode["runs_alone"]:
            line += "[runs alone on the port]"
        elif mode["capture_row"] is False:
            line += "[no capture row]"
        else:
            line += _range_text(mode["fps_floor"], mode["fps_ceiling"])
            if payload["cameras"] > 1 and mode.get("line_source"):
                found = ", found" if mode["line_source"] == "found" else ""
                line += f"  [pair line {mode['line_time_ns']} ns{found}]"
        print(line)
        if mode.get("fps_guaranteed") is not None:
            print(f"    below {_rate_text(mode['fps_guaranteed'])} fps: beyond the datasheet's "
                  f"guaranteed frame" + (f" ({section})" if section else ""))
    if payload["sync_pairs"]:
        print("sync pairs (fsync, one rate):")
        for pair in payload["sync_pairs"]:
            print(f"  {pair['token']} + {pair['token']}  "
                  f"{_range_text(pair['fps_floor'], pair['fps_ceiling'])}")
    print(f"knobs: {' '.join(payload['knobs']) or 'none'}")
    if payload["trigger_modes"]:
        print(f"trigger modes: {' '.join(payload['trigger_modes'])}")
    return 0


