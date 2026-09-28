# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The flows of a port that serves the sensor on its own bus: the sensor's
lanes and its I2C reach the host as they are, and an NXS unit, when the link
carries one, answers on the same bus at its own address. The bring-up is
the unit's program, the ready gate, and the family's start. The sensor's
own stream gate stands where a hub's CSI output gate does: a capture
consumer sees the stream start because `stream` and `capture` stop the
sensor, open the consumer, and start it again.

The module answers the flows surface the verbs dispatch to and carries no
chip knowledge: every write comes from the sensor's law family."""

from __future__ import annotations

import inspect
from fractions import Fraction
from typing import Any, Dict, List, Optional, Tuple

from nxs import experimental

from . import shipped
from .contracts import (CsiContract, InfeasibleConfig, LinkSpec, RateRange, Topology,
                        VcGeometry)
from .descriptors import FREERUN, resolve_mode
from .packs import sensor_address
from .plan import RawConfig, scan_forbidden

#: The transport the port's CSI contract records: the sensor's own lanes.
TRANSPORT = "direct"
#: The hard ready gate after a unit run: the head answers the host within
#: this budget, so the bus token is back before the host writes.
READY_GATE_MS = 1500
#: Rates this close (relative) are the same rate: a declared rate is a
#: decimal, a law's rate a float.
_RATE_TOLERANCE = 1e-3
#: Family attributes that start with `knob_` and are programs or hooks, not
#: live knobs.
_NOT_KNOBS = ("timing_start", "fast_trigger", "readback")


def _call(fn, *args, **kwargs):
    """Call a family law with the keyword arguments it takes: the families
    share a surface, not one signature."""
    params = inspect.signature(fn).parameters
    if any(p.kind == p.VAR_KEYWORD for p in params.values()):
        return fn(*args, **kwargs)
    return fn(*args, **{k: v for k, v in kwargs.items() if k in params})


def sensor_module(pack, link, topology: Optional[Topology] = None):
    """The law family bound to a link's sensor, at the port's view when the
    port is known (one camera on its lanes)."""
    compatible = str(getattr(link, "sensor_compatible", link))
    if topology is not None:
        module = pack.chip_module(compatible, shipped.cameras(topology.links),
                                  int(topology.csi_lanes))
    else:
        module = pack.chip_module(compatible)
    if module is None:
        raise InfeasibleConfig(f"sensor {compatible} binds no law family in pack {pack.name}")
    return module


def default_mode(pack, link, topology: Optional[Topology] = None) -> str:
    """The mode a link runs when nobody names one: the sensor's default."""
    return str(sensor_module(pack, link, topology).default_mode())


def resolve_modes(pack, links, mode=None,
                  topology: Optional[Topology] = None) -> Dict[str, str]:
    """Per-link mode names. ``mode`` is None (the link's declared mode, then
    the port's, then the sensor's default), one token, or {link: token};
    a token is a mode name or a WxH geometry."""
    out: Dict[str, str] = {}
    for link in links:
        token = mode.get(link.name) if isinstance(mode, dict) else mode
        if token is None:
            token = link.mode or (topology.camera_mode if topology is not None else None)
        module = sensor_module(pack, link, topology)
        sen = module.descriptor()
        try:
            out[link.name] = resolve_mode(sen, str(token)) if token else str(module.default_mode())
        except InfeasibleConfig as exc:
            raise InfeasibleConfig(f"link {link.name} ({sen.compatible}): {exc.reason}",
                                   alternatives=exc.alternatives)
    return out


def fps_range(pack, topology: Topology, link, mode: str,
              partner_mode_name: Optional[str] = None) -> RateRange:
    """The free-run rates a link's mode may run at: the family's range,
    narrowed to the shipped point when the mode ships one. A table-only
    part runs its mode at the one rate its table sets."""
    del partner_mode_name
    module = sensor_module(pack, link, topology)
    sen = module.descriptor()
    ceiling = float(_call(module.fps_ceiling, mode))
    floor = float(_call(module.fps_floor, mode=mode))
    if not hasattr(module, "vmax_for_fps"):
        floor = ceiling
    cameras, lanes = shipped.cameras(topology.links), int(topology.csi_lanes)
    point = shipped.fps_range(sen, mode, cameras, lanes)
    if point is not None:
        floor, ceiling = max(floor, point[0]), min(ceiling, point[1])
    return RateRange(floor=floor, ceiling=ceiling, shipped=point is not None,
                     binds="the sensor's mode")


def _rate(pack, module, mode: str, asked: Optional[float], spec: LinkSpec,
          topology: Topology) -> float:
    """The rate the link runs: the asked one, else the link's declared, the
    port's, the mode's ceiling; judged by the mode's lawful range."""
    sen = module.descriptor()
    rates = fps_range(pack, topology, spec, mode)
    declared = asked if asked is not None else (spec.fps or topology.camera_fps)
    if declared is None:
        return rates.ceiling
    declared = float(declared)
    slack = _RATE_TOLERANCE * rates.ceiling
    if rates.floor - slack <= declared <= rates.ceiling + slack:
        return rates.ceiling if rates.floor == rates.ceiling else declared
    if rates.floor == rates.ceiling:
        raise InfeasibleConfig(
            f"{sen.compatible} runs {mode} at {rates.ceiling:g} fps only",
            alternatives=[f"--fps {rates.ceiling:g}"])
    raise InfeasibleConfig(
        f"{declared:g} fps is outside {sen.compatible} {mode}'s "
        f"{rates.floor:g} to {rates.ceiling:g} fps", alternatives=[f"--fps {rates.ceiling:g}"])


def _start_steps(module, spec: LinkSpec, mode: str, rate: float,
                 vmax: Optional[int]) -> Tuple[List[Dict[str, Any]], float]:
    """The host's writes after the unit's run, and the rate they set: the
    family's timing program at the rate's frame length, else its stream
    gate."""
    sen = module.descriptor()
    timing = getattr(module, "knob_timing_start", None)
    if timing is None:
        if vmax is not None:
            raise InfeasibleConfig(f"{sen.compatible} has no frame-length law, and a "
                                   f"frame length (vmax) was asked",
                                   alternatives=["no vmax: the mode's table sets the frame"])
        start, _stop = _stream_gate(module)
        return list(start()), rate
    hmax = int(sen.modes[mode]["timing"]["hmax"])
    if vmax is None:
        vmax = int(module.vmax_for_fps(float(rate), hmax))
    else:
        rate = int(sen.limits["inck_hz"]) / (hmax * int(vmax))
    extra = {"inck_hz": int(spec.inck_hz)} if spec.inck_hz is not None else {}
    return list(_call(timing, hmax, int(vmax), mode=mode, trigger=FREERUN, **extra)), rate


def build_solo(pack, topology: Topology, link: str, mode=None,
               vmax: Optional[int] = None, fps=None) -> Tuple[RawConfig, CsiContract]:
    """The bring-up of the port's link: the unit runs its personality at the
    marker, the host gates on the head answering, then starts it."""
    spec = topology.link(link)
    if isinstance(fps, dict):
        fps = fps.get(link)
    module = sensor_module(pack, spec, topology)
    sen = module.descriptor()
    _stream_gate(module)            # refused here, before anything is composed
    mode = resolve_modes(pack, [spec], mode, topology=topology)[link]
    sen.mode_value(mode)
    cameras, lanes = shipped.cameras(topology.links), int(topology.csi_lanes)
    if shipped.entry(sen, mode, cameras, lanes) is None and not experimental.enabled():
        raise InfeasibleConfig(
            f"link {link}: {sen.compatible} {mode} is not shipped with "
            f"{shipped.label(cameras)} on {lanes} CSI lanes",
            alternatives=[f"--mode {m} (shipped)"
                          for m in shipped.shipped_modes(sen, cameras, lanes)]
            + [experimental.refusal(f"an unshipped mode ({mode})")])
    rate = _rate(pack, module, mode, fps, spec, topology)
    mipi = module.export_mipi_contract(mode, fps=rate)
    if int(mipi.lanes) != lanes:
        raise InfeasibleConfig(
            f"{sen.compatible} {mode} drives {mipi.lanes} lanes and the port "
            f"receives {lanes}", alternatives=[f"csi_lanes: {int(mipi.lanes)}"])
    steps, rate = _start_steps(module, spec, mode, rate, vmax)

    cfg = RawConfig(f"direct-{link}-{mode}")
    addr = sensor_address(pack, spec)
    # Nothing translates on the port's own bus: a unit at the sensor's
    # address would take every sensor write, and the sensor every unit write.
    if any(int(u.alias_addr) == int(addr) for u in spec.nxs_units):
        raise InfeasibleConfig(
            f"link {link}: the unit and the sensor both answer at {int(addr):#04x} on "
            f"the port's own bus",
            alternatives=["unit.target: the address the unit straps",
                          "sensor_addr: the address the sensor straps"])
    cfg.set_address("ADR_SENSOR", addr)
    cfg.add_unit_program(f"unit_program_{link}", link, mode, FREERUN)
    alive = getattr(module, "expect_alive", None)
    if alive is not None:
        cfg.add(f"ready_{link}", alive(READY_GATE_MS))
    if steps:
        cfg.add(f"start_{link}", steps)
    scan_forbidden(cfg, {addr: sen.runtime_forbidden})
    csi = CsiContract(
        port=0, transport=TRANSPORT,
        virtual_channels=(VcGeometry(vc=int(spec.csi_vc), dt=mipi.data_type,
                                     width=mipi.width, height=mipi.height,
                                     bit_depth=mipi.bit_depth),),
        rates={link: float(rate)})
    return cfg, csi


def build_dual(pack, topology: Topology, mode=None, vmax=None, **_) -> None:
    del pack, mode, vmax
    raise InfeasibleConfig("the port's receiver takes one sensor's lanes",
                           alternatives=[f"nxs {topology.carrier.split('/')[-1]} "
                                         f"{l.name} on" for l in topology.links[:1]])


def _no_sync_generator(topology: Topology) -> InfeasibleConfig:
    return InfeasibleConfig("frame sync needs a generator, and the sensor on the "
                            "port's own bus has none",
                            alternatives=[f"nxs {topology.carrier.split('/')[-1]} "
                                          f"set sync free_run"])


def build_fsync(pack, topology: Topology, fps: float, **_) -> None:
    del pack, fps
    raise _no_sync_generator(topology)


def build_trigger_off(pack, topology: Topology) -> RawConfig:
    """Free-run is the only sync the port has: nothing to switch off."""
    del pack, topology
    return RawConfig("fsync-off")


def _stream_gate(module):
    """The family's `start` and `stop`: the port has no hub to gate the
    lanes, so a sensor whose family has no stream gate cannot be brought
    up, captured or parked here."""
    hooks = tuple(getattr(module, name, None) for name in ("start", "stop"))
    if any(hook is None for hook in hooks):
        sen = module.descriptor()
        raise InfeasibleConfig(
            f"{sen.compatible}'s family has no stream gate, and the port has no hub "
            f"to gate the lanes", alternatives=["controls.standby (the stream gate) "
                                                 "in the personality"])
    return hooks


def _gate(module, enable: bool) -> List[Dict[str, Any]]:
    start, stop = _stream_gate(module)
    return list((start if enable else stop)())


def csi_gate_steps(pack, topology: Topology, enable: bool) -> List[Dict[str, Any]]:
    """The capture-consumer start choreography's gate: the sensor's own
    stream gate. The caller points `ADR_SENSOR` at the link's sensor."""
    return _gate(sensor_module(pack, topology.links[0], topology), enable)


def build_park(pack, topology: Topology) -> RawConfig:
    """Park: the sensor in standby, by best effort (a head that never came
    up is already off)."""
    cfg = RawConfig("down")
    for spec in topology.links:
        cfg.set_address("ADR_SENSOR", sensor_address(pack, spec))
        steps = _gate(sensor_module(pack, spec, topology), False)
        if steps:
            cfg.add(f"sensor_standby_{spec.name}", steps, best_effort=True)
    return cfg


def open_window(pack, i2c, topology: Topology, link=None) -> None:
    """Nothing stands between the host and the sensor."""
    del pack, i2c, topology, link


def close_windows(pack, i2c, topology: Topology) -> None:
    del pack, i2c, topology


def links_reachable(pack, i2c, topology: Topology, links) -> bool:
    """The link has no lock to lose: the unit's run is the first contact."""
    del pack, i2c, topology, links
    return True


def train(pack, i2c, topology: Topology, links=None, rounds: int = 0) -> Dict[str, Any]:
    """Nothing to train: no link stands between the port and the sensor."""
    del pack, i2c, topology, links, rounds
    return {}


def _fraction(fps: float) -> List[int]:
    value = Fraction(str(round(float(fps), 3))).limit_denominator(1000)
    return [value.numerator, value.denominator]


def viewer_hints(pack, topology: Topology, mode=None, triggered: bool = False,
                 link: Optional[str] = None, fps=None, **_) -> Dict[str, Any]:
    """Capture caps for the link's viewer, from its sensor's mode entry: the
    geometry, the rate the sensor runs, the mode's capture index (the
    booted tree's wins where it has one)."""
    del triggered
    spec = topology.link(link) if link else topology.links[0]
    module = sensor_module(pack, spec, topology)
    sen = module.descriptor()
    resolved = resolve_modes(pack, [spec], mode, topology=topology)[spec.name]
    if isinstance(fps, dict):
        fps = fps.get(spec.name)
    from nxs.host import capture_table as tables

    m = sen.modes[resolved]
    geo = m["geometry"]
    # The port boots this sensor's rows alone: the session's index is its own.
    index = tables.mode_index(pack, sen, resolved, [sen.name])
    return {
        "width": int(geo["width"]),
        "height": int(geo["height"]),
        "framerate": _fraction(_rate(pack, module, resolved, fps, spec, topology)),
        "sensor_mode": 0 if index is None else int(index),
        "locked_props": True,
        "crop_bottom": 0,
        "mode": resolved,
        "sensor": sen.compatible,
        "data_type": str(m["mipi"]["data_type"]),
    }


def viewer_hints_by_link(pack, topology: Topology, links=None, modes=None,
                         triggered: bool = False, fps=None,
                         **_) -> Dict[str, Dict[str, Any]]:
    return {spec.name: viewer_hints(pack, topology, modes, triggered=triggered,
                                    link=spec.name, fps=fps)
            for spec in (links or topology.links)}


def _sensor_knobs(module) -> List[str]:
    return sorted(name[len("knob_"):] for name in dir(module)
                  if name.startswith("knob_") and name[len("knob_"):] not in _NOT_KNOBS
                  and callable(getattr(module, name)))


def knob_names(pack, link=None) -> List[str]:
    """The live knobs: a link's sensor's, or the union over the pack's sensors."""
    if link is not None:
        return _sensor_knobs(sensor_module(pack, link))
    names: set = set()
    for chip in pack.sensors():
        module = pack.chip_module(chip)
        if module is not None:
            names.update(_sensor_knobs(module))
    return sorted(names)


def build_knob(pack, topology: Topology, knob: str, value: str, *,
               link: Optional[str] = None,
               readings: Optional[Dict[str, int]] = None,
               mode: Optional[str] = None) -> RawConfig:
    """A live-knob plan: the knob's steps on the link's sensor. A knob whose
    arithmetic rides the running timing takes it from ``readings``."""
    spec = topology.link(link) if link else topology.links[0]
    module = sensor_module(pack, spec, topology)
    sen = module.descriptor()
    have = _sensor_knobs(module)
    if knob not in have:
        raise KeyError(f"unknown knob {knob!r} for {sen.compatible}; have {have}")
    live = {k: int(v) for k, v in (readings or {}).items() if k in ("hmax", "vmax") and v}
    try:
        number: Any = int(str(value), 0)
    except ValueError:
        number = value
    steps = _call(getattr(module, f"knob_{knob}"), number, mode=mode, **live)
    cfg = RawConfig(f"set-{knob}")
    addr = sensor_address(pack, spec)
    cfg.set_address("ADR_SENSOR", addr)
    cfg.add(f"set_{knob}", list(steps))
    scan_forbidden(cfg, {addr: sen.runtime_forbidden})
    return cfg
