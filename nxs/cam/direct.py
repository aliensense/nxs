# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The flows of a port that serves the sensor on its own bus: the sensor's
lanes and its I2C reach the host as they are, and an NXS unit, when the link
carries one, answers on the same bus at its own address. The port's graph
image composes against the connector's program (`program`) and walks its
head alone: the pod's or the host's run of the personality, a smart
camera's rows on its camera door. The head's own stream gate stands where a
hub's CSI output gate does: a capture consumer sees the stream start
because `stream` and `capture` stop the head, open the consumer, and start
it again.

The module answers the flows surface the verbs dispatch to and carries no
chip knowledge: its plans are the sensor's law family's, which judge a
bring-up and a knob and state the port's CSI contract."""

from __future__ import annotations

import functools
import inspect
from fractions import Fraction
from typing import Any, Dict, List, Optional, Tuple

from .contracts import (FPS_DEFAULT, CsiContract, InfeasibleConfig, LinkSpec, RateRange,
                        Topology, VcGeometry)
from . import laws
from .descriptors import resolve_mode

from .hubs import is_smart_camera, sensor_address
from .map_knob import map_knob, rate as map_rate
from .plan import RawConfig

#: The transport the port's CSI contract records: the sensor's own lanes.
TRANSPORT = "direct"
#: Rates this close (relative) are the same rate: a declared rate is a
#: decimal, a law's rate a float.
_RATE_TOLERANCE = 1e-3
@functools.lru_cache(maxsize=1)
def program() -> bytes:
    """The program a graph image of a port with no hub carries: one phase,
    `idle`, that writes nothing. Such a port's walks run its head alone,
    and an image composes against a program whose `phase` parameter its
    walks' record names. The program is compiled here once, the same for
    every port, and never sealed: no rig's fact rides it."""
    from nxs import I2cProfile, Port, PortDirection, PortType
    from nxs.compiler import HubDevice
    from nxs.image import serialize

    class Connector(HubDevice):
        """The connector's program: the one phase, idle."""

        I2C_PROFILE = I2cProfile(addr_bytes=2)
        PHASES = {"idle": 0}
        PORTS = (Port("csi.A", PortType.CSI, PortDirection.OUTPUT),)

        def probe(self):
            pass

        def configure(self, config):
            self.declare_param("phase", values=[0], default=0)
            self.select("phase", {0: lambda: None})

    return serialize(Connector().compile({}))


def _call(fn, *args, **kwargs):
    """Call a family law with the keyword arguments it takes: the families
    share a surface, not one signature."""
    params = inspect.signature(fn).parameters
    if any(p.kind == p.VAR_KEYWORD for p in params.values()):
        return fn(*args, **kwargs)
    return fn(*args, **{k: v for k, v in kwargs.items() if k in params})


def sensor_module(hub, link, topology: Optional[Topology] = None):
    """The law family bound to a link's sensor: the port carries one camera
    at the mode's own line, so the port adds nothing to it."""
    del topology
    compatible = str(getattr(link, "sensor_compatible", link))
    module = laws.laws(hub.descriptor(compatible))
    if module is None:
        raise InfeasibleConfig(f"sensor {compatible} binds no law family in hub {hub.name}")
    return module


def default_mode(hub, link, topology: Optional[Topology] = None) -> str:
    """The mode a link runs when nobody names one: the sensor's default."""
    return str(sensor_module(hub, link, topology).default_mode())


def resolve_modes(hub, links, mode=None,
                  topology: Optional[Topology] = None) -> Dict[str, str]:
    """Per-link mode names. ``mode`` is None (the link's declared mode, then
    the port's, then the sensor's default), one token, or {link: token};
    a token is a mode name or a WxH geometry."""
    out: Dict[str, str] = {}
    for link in links:
        token = mode.get(link.name) if isinstance(mode, dict) else mode
        if token is None:
            token = link.mode
        module = sensor_module(hub, link, topology)
        sen = module.descriptor()
        try:
            out[link.name] = resolve_mode(sen, str(token)) if token else str(module.default_mode())
        except InfeasibleConfig as exc:
            raise InfeasibleConfig(f"link {link.name} ({sen.compatible}): {exc.reason}",
                                   alternatives=exc.alternatives)
    return out


def fps_range(hub, topology: Topology, link, mode: str,
              partner_mode_name: Optional[str] = None) -> RateRange:
    """The free-run rates a link's mode may run at: the family's range, from
    the driver's minimum rate to the datasheet frame at the mode's line. A
    table-only part runs its mode at the one rate its table sets."""
    del partner_mode_name
    module = sensor_module(hub, link, topology)
    ceiling = float(_call(module.fps_ceiling, mode))
    floor = float(_call(module.fps_floor, mode=mode))
    if hasattr(module, "vmax_for_fps"):
        return RateRange(floor=floor, ceiling=ceiling, binds="datasheet frame")
    if laws.family_for(module.descriptor()) == laws.NATIVE:
        # A smart camera runs any rate its row spans, down to its longest frame's.
        return RateRange(floor=floor, ceiling=ceiling, binds="rate row")
    return RateRange(floor=ceiling, ceiling=ceiling, binds="mode table")


def _rate(hub, module, mode: str, asked: Optional[float], spec: LinkSpec,
          topology: Topology) -> float:
    """The rate the link runs: the asked one, else the link's declared, the
    port's, FPS_DEFAULT inside the mode's range; judged by the mode's
    lawful range."""
    sen = module.descriptor()
    rates = fps_range(hub, topology, spec, mode)
    declared = asked if asked is not None else spec.fps
    if declared is None:
        return min(max(FPS_DEFAULT, rates.floor), rates.ceiling)
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
        f"{rates.floor:g} to {rates.ceiling:g} fps", alternatives=[f"--fps {rates.whole()[1]}"])


def _vmax_rate(module, mode: str, vmax: int) -> float:
    """The rate a fixed frame length runs at by the family's timing law.

    Raises:
        InfeasibleConfig: A family without a frame-length law: its mode's
            table sets the frame.
    """
    sen = module.descriptor()
    if getattr(module, "knob_timing_start", None) is None:
        raise InfeasibleConfig(f"{sen.compatible} has no frame-length law, and a "
                               f"frame length (vmax) was asked",
                               alternatives=["no vmax: the mode's table sets the frame"])
    hmax = int(sen.modes[mode]["timing"]["hmax"])
    return int(sen.limits["inck_hz"]) / (hmax * int(vmax))


def build_solo(hub, topology: Topology, link: str, mode=None,
               vmax: Optional[int] = None, fps=None) -> Tuple[RawConfig, CsiContract]:
    """The bring-up of the port's link judged by the sensor's laws, and the
    CSI contract the capture takes: the mode, the rate the law runs it at,
    the lanes the port receives, a table-only part's stream gate, a pod and
    the sensor at addresses of their own on the port's bus. The port's
    graph image runs the bring-up (`compose_walks`), so the plan is empty."""
    spec = topology.link(link)
    if isinstance(fps, dict):
        fps = fps.get(link)
    module = sensor_module(hub, spec, topology)
    sen = module.descriptor()
    _stream_gate(module)            # refused here, before anything is composed
    mode = resolve_modes(hub, [spec], mode, topology=topology)[link]
    sen.mode_value(mode)
    lanes = int(topology.csi_lanes)
    rate = _rate(hub, module, mode, fps, spec, topology)
    mipi = module.export_mipi_contract(mode, fps=rate)
    if int(mipi.lanes) != lanes:
        raise InfeasibleConfig(
            f"{sen.compatible} {mode} drives {mipi.lanes} lanes and the port "
            f"receives {lanes}",
            alternatives=[f"the host's {topology.carrier.split('/')[-1]}_lanes: {int(mipi.lanes)}"])
    if not is_smart_camera(hub, spec):
        if vmax is not None:
            rate = _vmax_rate(module, mode, vmax)
        addr = sensor_address(hub, spec)
        # Nothing translates on the port's own bus: a unit at the sensor's
        # address would take every sensor write, and the sensor every unit write.
        if any(int(u.alias_addr) == int(addr) for u in spec.nxs_units):
            raise InfeasibleConfig(
                f"link {link}: the unit and the sensor both answer at {int(addr):#04x} on "
                f"the port's own bus",
                alternatives=["unit.target: the address the unit straps",
                              "sensor_addr: the address the sensor straps"])
    csi = CsiContract(
        port=0, transport=TRANSPORT,
        virtual_channels=(VcGeometry(vc=int(spec.csi_vc), dt=mipi.data_type,
                                     width=mipi.width, height=mipi.height,
                                     bit_depth=mipi.bit_depth),),
        rates={link: float(rate)})
    return RawConfig(f"direct-{link}-{mode}"), csi


def build_dual(hub, topology: Topology, mode=None, vmax=None, **_) -> None:
    del hub, mode, vmax
    raise InfeasibleConfig("the port's receiver takes one sensor's lanes",
                           alternatives=[f"nxs {topology.carrier.split('/')[-1]} "
                                         f"{l.name} on" for l in topology.links[:1]])


def _no_sync_generator(topology: Topology) -> InfeasibleConfig:
    return InfeasibleConfig("frame sync needs a generator, and the sensor on the "
                            "port's own bus has none",
                            alternatives=[f"nxs {topology.carrier.split('/')[-1]} "
                                          f"set sync free_run"])


def build_fsync(hub, topology: Topology, fps: float, **_) -> None:
    del hub, fps
    raise _no_sync_generator(topology)


def build_trigger_off(hub, topology: Topology) -> RawConfig:
    """Free-run is the only sync the port has: nothing to switch off."""
    del hub, topology
    return RawConfig("fsync-off")


def _stream_gate(module):
    """The family's `start` and `stop`: the port has no hub to gate the
    lanes, so a sensor whose family has no stream gate cannot be brought
    up, captured or parked here. A head that gates its own stream through
    its camera map (its law offers the gate and the host writes no sensor
    register) has nothing for the host to write."""
    hooks = tuple(getattr(module, name, None) for name in ("start", "stop"))
    if any(hook is None for hook in hooks):
        law = getattr(module, "apply_law", None)
        if law is not None and getattr(law(), "stream_gate", False):
            return (lambda: [], lambda: [])
        sen = module.descriptor()
        raise InfeasibleConfig(
            f"{sen.compatible}'s family has no stream gate, and the port has no hub "
            f"to gate the lanes", alternatives=["controls.standby (the stream gate) "
                                                 "in the personality"])
    return hooks


def open_window(hub, i2c, topology: Topology, link=None) -> None:
    """Nothing stands between the host and the sensor."""
    del hub, i2c, topology, link


def close_windows(hub, i2c, topology: Topology) -> None:
    del hub, i2c, topology


def links_reachable(hub, i2c, topology: Topology, links) -> bool:
    """The link has no lock to lose: the unit's run is the first contact."""
    del hub, i2c, topology, links
    return True


def train(hub, i2c, topology: Topology, links=None, rounds: int = 0) -> Dict[str, Any]:
    """Nothing to train: no link stands between the port and the sensor."""
    del hub, i2c, topology, links, rounds
    return {}


def _fraction(fps: float) -> List[int]:
    value = Fraction(str(round(float(fps), 3))).limit_denominator(1000)
    return [value.numerator, value.denominator]


def viewer_hints(hub, topology: Topology, mode=None, triggered: bool = False,
                 link: Optional[str] = None, fps=None, **_) -> Dict[str, Any]:
    """Capture caps for the link's viewer, from its sensor's mode entry: the
    geometry, the rate the sensor runs, the mode's capture index (the
    booted tree's wins where it has one)."""
    del triggered
    spec = topology.link(link) if link else topology.links[0]
    module = sensor_module(hub, spec, topology)
    sen = module.descriptor()
    resolved = resolve_modes(hub, [spec], mode, topology=topology)[spec.name]
    if isinstance(fps, dict):
        fps = fps.get(spec.name)
    from nxs.host import capture_table as tables

    m = sen.modes[resolved]
    geo = m["geometry"]
    # The port boots this sensor's rows alone: the session's index is its own.
    index = tables.port_index(hub, topology, sen, resolved)
    return {
        "width": int(geo["width"]),
        "height": int(geo["height"]),
        "framerate": _fraction(_rate(hub, module, resolved, fps, spec, topology)),
        "sensor_mode": 0 if index is None else int(index),
        "crop_bottom": 0,
        "mode": resolved,
        "sensor": sen.compatible,
        "data_type": str(m["mipi"]["data_type"]),
    }


def viewer_hints_by_link(hub, topology: Topology, links=None, modes=None,
                         triggered: bool = False, fps=None,
                         **_) -> Dict[str, Dict[str, Any]]:
    return {spec.name: viewer_hints(hub, topology, modes, triggered=triggered,
                                    link=spec.name, fps=fps)
            for spec in (links or topology.links)}


def build_knob(hub, topology: Topology, knob: str, value: str, *,
               link: Optional[str] = None,
               readings: Optional[Dict[str, int]] = None,
               mode: Optional[str] = None) -> RawConfig:
    """A live-knob plan: the knob's steps on the link's sensor, or the knob's
    row committed on the camera map of a head that serves one, a pod's or a
    smart camera's, as `caps` and `get` name it. A knob whose arithmetic
    rides the running timing takes it from ``readings``."""
    spec = topology.link(link) if link else topology.links[0]
    module = sensor_module(hub, spec, topology)
    sen = module.descriptor()
    if is_smart_camera(hub, spec) or spec.nxs_units:
        from .pods import camera_map_knobs

        # The rows an APPLY commit takes and their forms: a gate, a trigger or
        # a mode row has its own verb, and a read-only row takes no value.
        have = camera_map_knobs(hub, spec) or []
        if knob not in have:
            raise KeyError(f"unknown knob {knob!r} for {sen.compatible}; have {have}")
        if knob == "fps":
            _rate(hub, module, mode or default_mode(hub, spec, topology), map_rate(value), spec, topology)
        return map_knob(spec.name, knob, value)
    # A bare head's knob is its personality's apply on the host, which judges
    # the value under the head's law (`compose_heads.bare_knob`): no plan here.
    del readings
    return RawConfig(f"set-{knob}")
