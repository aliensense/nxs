"""The frame-sync plan and the viewer hints: which links take the trigger, the generator rate, the recorded sync."""

from __future__ import annotations

from typing import Any, Dict, List, Optional


from nxs import term

from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs import host as host_layer
from nxs.cam import port_state
from nxs.cam import packs
from nxs.cam.run import _accepted


def _booted_index(pack, topology: Topology, link: LinkSpec,
                  hints: Dict[str, Any]) -> Dict[str, Any]:
    """The capture mode index the booted tree gives this link's
    geometry, over the pack table's (the tree is what the capture stack
    reads; the pack's table is what the next overlay will carry)."""
    try:
        sen = pack.descriptor(hints.get("sensor") or link.sensor_compatible)
        index = host_layer.current().mode_index(
            topology.i2c_bus, sen.compatible, int(hints["width"]),
            int(hints["height"]),
            int(sen.modes[hints["mode"]]["geometry"]["bit_depth"]),
            direct=bool(topology.is_direct))
    except Exception:
        index = None
    if index is not None and index != hints.get("sensor_mode"):
        hints = dict(hints, sensor_mode=index)
    return hints


def _rate_kwarg(fn, rates: Optional[Dict[str, float]]) -> Dict[str, Any]:
    """`fps=` for a caps hook that takes the port's free-run rates: the
    caps must carry the rate the sensor was programmed for, or the
    capture stack's frame-rate control moves the frame to its own."""
    if rates and "fps" in _accepted(fn):
        return {"fps": dict(rates)}
    return {}


def _frames_kwarg(fn, frames: Optional[Dict[str, int]]) -> Dict[str, Any]:
    """`trigger_frames=` for a caps hook that takes the plan's frames."""
    if frames and "trigger_frames" in _accepted(fn):
        return {"trigger_frames": dict(frames)}
    return {}


def _exposure_kwarg(fn, exposure_us: Optional[float]) -> Dict[str, Any]:
    """`exposure_us=` for a caps hook that pins the plan's exposure."""
    if exposure_us is not None and "exposure_us" in _accepted(fn):
        return {"exposure_us": float(exposure_us)}
    return {}


def _port_viewer_hint(pack, flows, topology: Topology, links: List[LinkSpec],
                      mode, triggered: bool = False,
                      rates: Optional[Dict[str, float]] = None,
                      trigger_frames: Optional[Dict[str, int]] = None,
                      exposure_us: Optional[float] = None
                      ) -> Optional[Dict[str, Any]]:
    """The port-level viewer caps, for a whole port only. A partial
    bring-up is described by its per-link caps (the port record writer promotes
    the first of them), and the port's declared pair may not fit the
    lanes the running links use: the hint is advisory and never fails a
    bring-up that locked."""
    if len(links) < len(topology.links):
        return None
    try:
        return flows.viewer_hints(pack, topology, mode, triggered=triggered,
                                  **_rate_kwarg(flows.viewer_hints, rates),
                                  **_frames_kwarg(flows.viewer_hints, trigger_frames),
                                  **_exposure_kwarg(flows.viewer_hints, exposure_us))
    except InfeasibleConfig:
        return None


def _hints_by_link(pack, flows, topology: Topology, links: List[LinkSpec],
                   modes, triggered: bool = False,
                   rates: Optional[Dict[str, float]] = None,
                   trigger_frames: Optional[Dict[str, int]] = None,
                   exposure_us: Optional[float] = None
                   ) -> Optional[Dict[str, Dict[str, Any]]]:
    """Per-link viewer caps from packs that compose them per link, their
    capture mode index taken from the booted tree when it has one."""
    hook = getattr(flows, "viewer_hints_by_link", None)
    if hook is None:
        return None
    try:
        hints = hook(pack, topology, links=links, modes=modes,
                     triggered=triggered,
                     **_rate_kwarg(hook, rates),
                     **_frames_kwarg(hook, trigger_frames),
                     **_exposure_kwarg(hook, exposure_us))
    except InfeasibleConfig:
        # Caps are advisory; a pair the port cannot carry is the mode
        # planner's verdict, given before the program ran, not here.
        return None
    by_name = {l.name: l for l in links}
    return {name: _booted_index(pack, topology, by_name[name], h)
            if name in by_name else h for name, h in hints.items()}


def _sync_links(pack, flows, topology: Topology) -> Optional[Dict[str, str]]:
    """What each link does under the port trigger, from the pack's sync
    plan: `fsync`, or `free_run` with the sensor's reason."""
    plan_fn = getattr(flows, "sync_plan", None)
    if plan_fn is None:
        return None
    plan = plan_fn(pack, topology)
    return {name: ("fsync" if ok else f"free_run ({text})")
            for name, (ok, text) in plan.items()}


def _nth(n: int) -> str:
    return {1: "1st", 2: "2nd", 3: "3rd"}.get(int(n), f"{int(n)}th")


def _fsync_plan(flows, pack, topology: Topology, fps: float,
                exposure_us: Optional[float], modes):
    """The pack's frame-sync plan (generator multiple, exposure, trigger
    frames), None for a pack without the law."""
    hook = getattr(flows, "fsync_plan", None)
    if hook is None:
        return None
    kwargs: Dict[str, Any] = {}
    if exposure_us is not None:
        kwargs["exposure_us"] = exposure_us
    if modes and "modes" in _accepted(hook):
        kwargs["modes"] = modes
    return hook(pack, topology, fps, **kwargs)


def plan_text(plan) -> str:
    """One line on what the synced pair runs: the frame rate, the
    pulse multiple, the exposure the pulse sets, the trigger frame."""
    n = int(plan.pulses_per_frame)
    parts = [f"fsync {plan.fps:g} fps"]
    parts.append(f"every {_nth(n)} pulse of {plan.pulse_hz:.2f} Hz" if n > 1
                 else "one pulse per frame")
    if plan.exposure_us is not None:
        parts.append(f"exposure {plan.exposure_us / 1000:.2f} ms")
    frames = sorted(set(plan.trigger_vmax.values()))
    if len(frames) == 1:
        parts.append(f"trigger frame {frames[0]} lines")
    else:
        parts.append("trigger frame " + ", ".join(
            f"{k} {v}" for k, v in sorted(plan.trigger_vmax.items())))
    return ": ".join([parts[0], ", ".join(parts[1:])])


def sync_text(live: Dict[str, Any]) -> str:
    """The live sync record as `status` prints it."""
    fps = live.get("fps")
    text = str(live["source"]) + (f" {fps:g} fps" if fps else "")
    n = int(live.get("pulses_per_frame") or 1)
    exposure = live.get("exposure_us")
    details = []
    if fps and n > 1:
        details.append(f"every {_nth(n)} pulse of {n * float(fps):.2f} Hz")
    if exposure is not None:
        details.append(f"exposure {float(exposure) / 1000:.2f} ms")
    return text + (f" ({', '.join(details)})" if details else "")


def _record_sync(pack, flows, topology: Topology, source: str,
                 fps: Optional[float], plan=None) -> None:
    triggered = source == "fsync"
    links = list(topology.links)
    modes = {l.name: m for l in links
             if (m := port_state.port_mode(topology, l))} or None
    # Free-running again, the caps carry the rate each sensor was
    # programmed for (the port record); under the trigger the pack's
    # hook derives the trigger frame's, the plan's frame when it has one.
    rates = ({l.name: r for l in links
              if (r := port_state.port_rate(topology, l)) is not None} or None
             if not triggered else None)
    frames = dict(plan.trigger_vmax) if (triggered and plan is not None) else None
    exposure = plan.exposure_us if (triggered and plan is not None) else None
    port_state.set_viewer_hints(
        _port_viewer_hint(pack, flows, topology, links, None,
                          triggered=triggered, rates=rates, trigger_frames=frames,
                          exposure_us=exposure),
        topology=topology,
        viewers=_hints_by_link(pack, flows, topology, links, modes,
                               triggered=triggered, rates=rates,
                               trigger_frames=frames, exposure_us=exposure))
    per_link = _sync_links(pack, flows, topology) if triggered else None
    port_state.set_sync(
        source, fps if triggered else None, topology=topology, links=per_link,
        pulses_per_frame=(plan.pulses_per_frame if triggered and plan else None),
        exposure_us=(plan.exposure_us if triggered and plan else None),
        trigger_vmax=frames)
    for name, what in (per_link or {}).items():
        if not what.startswith("fsync"):
            term.warn(f"link {name} is not synced: "
                      f"{what[len('free_run ('):-1]}")


def _budget_warning(pack, topology: Topology, fps: float) -> None:
    """Warn when this port's rate plus the other ports' up ports exceed
    the host's measured capture budget (advisory; see nxs.cam.budget)."""
    from nxs.cam import budget as host_budget
    from nxs.cam.topology import load_ports

    limit = host_layer.current().capture_budget_mpix_s()
    if limit is None:
        return
    streams = []
    try:
        ports, _ = load_ports(None)
    except Exception:
        ports = {0: topology}
    me = port_state.port_name(topology)
    for port in ports.values():
        name = port_state.port_name(port)
        own = port_state.port_record(port)
        hints = own.get("viewer") or {}
        viewers = own.get("viewers") or {}
        if name == me:
            # One stream per link at its own geometry (`viewers`) or the shared one;
            # a link that free-runs under the trigger keeps its own rate.
            mine = [l for l in topology.links
                    if port_state.link_state(topology, l) == port_state.STATE_UP
                    ] or list(topology.links)
            plan = _sync_plan(pack, topology)
            for link in mine:
                link_hints = (viewers.get(link.name) or hints
                              or port_state.viewer_hints(topology, link=link.name) or {})
                if not link_hints.get("width"):
                    continue
                rate = float(fps)
                if plan and not plan.get(link.name, (True,))[0]:
                    rate = _native_rate(topology, link, link_hints) or rate
                streams.append((name, 1, int(link_hints["width"]),
                                int(link_hints["height"]), rate))
            continue
        sync = own.get("sync") or {}
        up = [l for l in port.links
              if port_state.link_state(port, l) == port_state.STATE_UP]
        # One stream per up link: a mixed hub records each link's own
        # geometry under `viewers`; a uniform port shares `viewer`.
        for other in up:
            link_hints = viewers.get(other.name) or hints
            if not link_hints.get("width"):
                continue
            rate = float(sync.get("fps") or 0.0) or _native_rate(port, other, link_hints)
            if not rate:
                continue
            streams.append((name, 1, int(link_hints["width"]), int(link_hints["height"]),
                            float(rate)))
    note = host_budget.budget_note(streams, limit, me)
    if note:
        term.warn(note)


def _sync_plan(pack, topology: Topology) -> Dict[str, Any]:
    """Which links take the hub's trigger, per the pack ({} without one)."""
    try:
        return dict(pack.flows().sync_plan(pack, topology)) if pack is not None else {}
    except Exception:
        return {}


def _native_rate(port: Topology, link: LinkSpec, hints: Dict[str, Any]) -> Optional[float]:
    """A free-running link's rate: the one its last `on` recorded, else
    its mode's lawful ceiling from the port's own pack (None when the
    pack or the mode is unknown)."""
    recorded = port_state.port_rate(port, link)
    if recorded is not None:
        return recorded
    try:
        own_pack = packs.pack_for(port)
        mode = hints.get("mode") or own_pack.flows().default_mode(own_pack, link)
        from nxs.cam import timing as cam_timing
        rates = cam_timing.lawful_range(own_pack, port, link, mode)
        if rates is not None:
            return rates.ceiling
        # A pack without the range law: the mode's own frame at its line.
        sen = own_pack.descriptor(link.sensor_compatible)
        timing = sen.modes[mode]["timing"]
        frame = timing.get("vmax") or timing.get("min_frame_length")
        return int(sen.limits["inck_hz"]) / (int(timing["hmax"]) * int(frame))
    except Exception:
        return None


