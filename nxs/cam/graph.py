# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The port's walk as libnxs takes it: the facts of every link, the two
hub images and the sensors' staging gathered into the spec `Bus.port_up`
runs. The library orders the hub image's phases, the serializers' address
maps and the pods' actions; the tool states the port."""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Tuple

from nxs import _libnxs
from nxs.cam import hubimage as hi
from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology


def address_entries(pack, spec: LinkSpec) -> List[Tuple[int, int]]:
    """The `(alias, target)` pairs a link's serializer translates: every
    pod at an alias, and the head when the host reaches it at another
    address than it straps (the pack's `_address_map` rule)."""
    from nxs.cam.packs import native_sensor_address

    entries = [(int(u.alias_addr), int(u.target_addr)) for u in spec.nxs_units
               if int(u.alias_addr) != int(u.target_addr)]
    native = native_sensor_address(pack, spec)
    if spec.host_addr is not None and int(spec.host_addr) != native:
        entries.append((int(spec.host_addr), native))
    if len(entries) > 2:
        raise InfeasibleConfig(f"link {spec.name}: the serializer has two translation slots "
                               f"and {len(entries)} devices ask for one")
    return entries


@dataclasses.dataclass(frozen=True)
class Sensor:
    """What a camera link's personality is run with: the indices its
    parameter table gives the mode, the trigger, the action and the periods
    (None for one it does not declare), the mode and trigger values, the
    frame length a rate law fixed (None when the tables set the frame), the
    pod running it (None when the host does), and the host's image of it:
    the build the pod must hold, or what the host runs."""
    params: _libnxs.PersonalityParams
    mode: int
    trigger: int
    frame_length: Optional[int]
    pod: Optional[_libnxs.Pod]
    image: Optional[bytes]


def run_timing(sen, mode: str, frame_length: Optional[int] = None) -> Dict[str, int]:
    """The line and frame periods a run of `mode` stages, in nanoseconds by
    run-param name: the descriptor's line (a port view's is the line the
    pair leaves the mode) and its recommended frame, or `frame_length`
    lines when the caller fixes the frame."""
    from nxs.cam import chips
    from nxs.personality.records import FRAME_PERIOD_PARAM, LINE_TIME_PARAM

    module = chips.bind(sen)
    hmax = int(sen.modes[mode]["timing"]["hmax"])
    line_ns = round(module.line_time_us(hmax) * 1000)
    lines = module.recommended_frame_length(mode) if frame_length is None else frame_length
    return {LINE_TIME_PARAM: line_ns, FRAME_PERIOD_PARAM: int(lines) * line_ns}


def homogeneous(pack, links: List[LinkSpec], modes: Dict[str, str]) -> bool:
    """Whether a pair runs one sensor kind in one mode at one address: the
    walk then starts both sensors before the timing fix, as the captured
    program does."""
    from nxs.cam.packs import sensor_address

    a, b = links
    return (a.sensor_compatible == b.sensor_compatible and modes[a.name] == modes[b.name]
            and sensor_address(pack, a) == sensor_address(pack, b))


def port_spec(pack, topology: Topology, links: List[LinkSpec], modes: Dict[str, str],
              hub: bytes, ser: bytes, sensors: Dict[str, Sensor]) -> _libnxs.PortSpec:
    """The spec of the walk of `links` on the port in `modes`: the pair, or
    one link, or a pod alone; `sensors` stages every camera link."""
    from nxs.cam.packs import sensor_address
    from nxs.cam.select import _link_descriptor
    from nxs.personality.records import FRAME_PERIOD_PARAM, LINE_TIME_PARAM

    # An alias is one host address for one device: the pack's rule refuses
    # a collision before anything is mapped.
    check = getattr(pack.flows(), "require_distinct_host_addresses", None)
    if check is not None:
        check(pack, topology)
    by_name = {l.name: l for l in links}
    pods = [name for name, spec in by_name.items() if not spec.has_camera]
    if pods and len(links) > 1:
        raise InfeasibleConfig(
            f"link {pods[0]} carries a pod and no camera, and one hub brings a pod "
            f"alone up on its own",
            alternatives=[f"link {name}" for name in sorted(by_name)])
    if len(links) == 2 and set(by_name) != {"A", "B"}:
        raise InfeasibleConfig(f"a pair is links A and B; got {sorted(by_name)}")
    if not 1 <= len(links) <= 2:
        raise InfeasibleConfig("a walk brings up one link or the pair")
    if int(topology.csi_lanes) not in hi.CSI_LANES:
        raise InfeasibleConfig(f"the hub's CSI output takes 2 or 4 lanes, not "
                               f"{topology.csi_lanes}")
    # A pair's head runs the pair line beside the mode the partner runs.
    port = (topology if len(links) == len(topology.links)
            else dataclasses.replace(topology, links=tuple(links))).with_modes(modes)
    out = []
    for name, spec in by_name.items():
        facts = (hi.link_facts(pack.descriptor(spec.sensor_compatible), modes[name])
                 if spec.has_camera else dict(hi.POD_ONLY_FACTS))
        link = _libnxs.PortLink(
            name=name, has_camera=bool(spec.has_camera), ser_addr=int(spec.ser_addr),
            head_addr=sensor_address(pack, spec), lanes=int(facts["lanes"]),
            data_type=hi.DATA_TYPES[str(facts["data_type"])], host_csi=bool(facts["host_csi"]),
            entries=tuple(address_entries(pack, spec)))
        if spec.has_camera:
            sensor = sensors.get(name)
            if sensor is None:
                raise InfeasibleConfig(f"link {name}: nothing stages its sensor")
            timed = (sensor.params.line_time is not None
                     and sensor.params.frame_period is not None)
            periods = (run_timing(_link_descriptor(pack, port, spec), modes[name],
                                  sensor.frame_length) if timed else {})
            link = dataclasses.replace(
                link, params=sensor.params, mode=int(sensor.mode), trigger=int(sensor.trigger),
                line_time_ns=int(periods.get(LINE_TIME_PARAM, 0)),
                frame_period_ns=int(periods.get(FRAME_PERIOD_PARAM, 0)),
                frame_fixed=sensor.frame_length is not None, pod=sensor.pod,
                sensor_image=sensor.image)
        out.append(link)
    return _libnxs.PortSpec(
        des_addr=int(topology.des_addr), csi_lanes=int(topology.csi_lanes), hub_image=hub,
        ser_image=ser, links=tuple(out),
        homogeneous=len(links) == 2 and homogeneous(pack, links, modes))
