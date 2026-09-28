"""`caps`: what the port's sensor offers, the shipped points and the knobs."""

from __future__ import annotations

import argparse
import json
from typing import Any, Dict, List, Optional


from nxs import experimental

from nxs.cam.contracts import LinkSpec, Topology
from nxs.cam.select import _link_descriptor, _pack_for, _port_name, _refuse_pod_only, select_port_links
from nxs.host import capture_table as tables


def _camera_knobs(pack, link: LinkSpec, topology=None) -> List[str]:
    """The knobs the host names for a link: the pack's, and the port's
    `sync` where a node on the port generates frame sync (a hub's)."""
    knobs = set(_knob_names(pack, link))
    if topology is None or not topology.is_direct:
        knobs.add("sync")
    return sorted(knobs)


def _hub_fsync_rate(pack, i2c, topology) -> Optional[float]:
    """The hub generator's live frame rate through the deserializer
    module's `fsync_rate` hook; None when the hub has no such hook or
    the generator is off."""
    from nxs.cam.diag import run_probes

    try:
        module = pack.chip_module(topology.des_compatible)
    except Exception:
        return None
    hook = getattr(module, "fsync_rate", None) if module else None
    if hook is None:
        return None
    desd = pack.descriptor(topology.des_compatible)
    readings = {r.name: r.raw for r in run_probes(i2c, topology.des_addr, desd)
                if r.raw is not None}
    try:
        return hook(readings)
    except Exception:
        return None


def _knob_names(pack, link: LinkSpec) -> List[str]:
    """The link's knobs from packs that know per-sensor surfaces, the
    pack's list otherwise."""
    fn = pack.flows().knob_names
    try:
        return list(fn(pack, link))
    except TypeError:
        return list(fn(pack))


def _mode_token(send, name: str) -> str:
    """The token `--mode` takes for a mode: its geometry, or the geometry
    with the depth when another mode shares it (WxH-rawN)."""
    geo = send.modes[name].get("geometry") or {}
    token = f"{geo.get('width')}x{geo.get('height')}"
    shared = [n for n, m in send.modes.items()
              if f"{(m.get('geometry') or {}).get('width')}x"
                 f"{(m.get('geometry') or {}).get('height')}" == token]
    if len(shared) > 1:
        token += f"-raw{geo.get('bit_depth')}"
    return token


def caps_payload(topology: Topology, pack, link: LinkSpec) -> Dict[str, Any]:
    """The `caps` surface: what the link's sensor offers on this port, from
    the pack and the unit's records. Without --experimental the modes are
    the shipped ones alone, each with its range; the flag adds the other
    modes, marked experimental."""
    from nxs.schemas import CONTRACT

    from nxs.cam import timing as cam_timing
    from nxs.cam import shipped

    send = _link_descriptor(pack, topology, link)
    module = pack.chip_module(link.sensor_compatible.split(",")[-1])
    cameras = shipped.cameras(topology.links)
    unlocked = experimental.enabled()
    modes = []
    for name, mode in send.modes.items():
        geo = mode["geometry"]
        offered = name in send.program_modes()
        point = shipped.entry(send, name, cameras, int(topology.csi_lanes)) if offered else None
        if point is None and not unlocked:
            continue
        hmax = (mode.get("timing") or {}).get("hmax")
        line_ns = (round(float(module.line_time_us(int(hmax))) * 1000)
                   if hmax is not None and hasattr(module, "line_time_us") else None)
        # The mode's lawful range on this port (the shipped point where the
        # mode ships one); the sensor's own ceiling for a mode without a program.
        floor = None
        try:
            rates = cam_timing.lawful_range(pack, topology, link, name) if offered else None
        except Exception:
            rates = None
        if rates is not None:
            ceiling, floor = round(rates.ceiling, 2), round(rates.floor, 2)
        else:
            try:
                ceiling = round(float(module.fps_ceiling(name)), 2)
            except Exception:
                ceiling = None
        modes.append({
            "name": name, "token": _mode_token(send, name),
            "width": int(geo["width"]), "height": int(geo["height"]),
            "data_type": str(mode["mipi"]["data_type"]),
            "lanes": int(geo["lanes"]), "rate_mbps": int(geo["rate_mbps"]),
            "fps_ceiling": ceiling, "fps_floor": floor,
            "line_time_ns": line_ns,
            "line_source": (None if point is None
                            else "pair" if point.get("derived") else "shipped"),
            "experimental": point is None,
            "offered": offered,
            "sensor_mode": tables.mode_index(pack, send, name,
                                             [send.name] if topology.is_direct else None),
        })
    trig = send.raw("trigger") or {}
    knobs = _camera_knobs(pack, link, topology)
    pairs = []
    if len(topology.links) > 1:
        for name in send.program_modes():
            record = shipped.entry(send, name, 2,
                                             int(topology.csi_lanes))
            if record is not None and shipped.synced(record):
                pairs.append({"mode": name, "token": _mode_token(send, name),
                              "fps_floor": float(record["fps"]["floor"]),
                              "fps_ceiling": float(record["fps"]["ceiling"])})
    payload = {
        "contract": CONTRACT,
        "port": _port_name(topology), "link": link.name,
        "sensor": send.compatible,
        "csi_lanes": int(topology.csi_lanes), "pack": pack.name,
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


def _range_text(floor, ceiling) -> str:
    def one(v):
        return f"{v:.10g}" if float(v).is_integer() else f"{v:.2f}"
    if floor is None and ceiling is None:
        return "no lawful rate"
    if floor is None:
        return f"<= {one(ceiling)} fps"
    return f"{one(floor)}–{one(ceiling)} fps"


def cmd_caps(args: argparse.Namespace) -> int:
    """What a link may run on the port the selection makes: one link named
    is one camera, the port form the pair."""
    from nxs.cam import shipped

    topology, selected = select_port_links(args)
    pack = _pack_for(topology)
    narrow = getattr(pack.flows(), "port_links", None)
    if selected and narrow is not None:
        topology = narrow(topology, selected)
    link = selected[0] if selected else topology.links[0]
    if not link.has_camera:
        raise _refuse_pod_only(topology, link, "caps")
    payload = caps_payload(topology, pack, link)
    if getattr(args, "json", False):
        print(json.dumps(payload, indent=2))
        return 0
    print(f"{payload['port']} ({payload['csi_lanes']} CSI lanes, "
          f"{shipped.label(payload['cameras'])})")
    print(f"link {link.name}  {payload['sensor']}")
    if not payload["modes"]:
        print("  no shipped mode for this port: the unit's shipped points carry none "
              f"({experimental.FLAG} lists the unshipped ones)")
    for mode in payload["modes"]:
        geometry = f"{mode['width']}x{mode['height']}"
        line = f"  {geometry:<10} {mode['data_type']:<6} {_range_text(mode['fps_floor'], mode['fps_ceiling'])}"
        if mode["experimental"]:
            why = ("no unit program" if not mode["offered"] else "unshipped")
            line += f"  [experimental: {why}]"
        elif mode.get("line_source") == "pair":
            line += f"  [pair line {mode['line_time_ns']} ns]"
        print(line)
    if payload["sync_pairs"]:
        print("sync pairs (fsync, one rate):")
        for pair in payload["sync_pairs"]:
            print(f"  {pair['token']} + {pair['token']}  "
                  f"{_range_text(pair['fps_floor'], pair['fps_ceiling'])}")
    print(f"knobs: {' '.join(payload['knobs']) or 'none'}")
    if payload["trigger_modes"]:
        print(f"trigger modes: {' '.join(payload['trigger_modes'])}")
    return 0


