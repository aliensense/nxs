"""The frame-sync plan and the viewer hints: which links take the trigger, the generator rate, whether the pulse sets the exposure, who sets a pair's exposure and gain, the recorded sync."""

from __future__ import annotations

from typing import Any, Dict, List, Optional


from nxs import term

from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs import host as host_layer
from nxs.cam import port_state
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


def _ae_roles(ae: Optional[Dict[str, Any]]) -> Dict[str, str]:
    """Each camera link's part in the port's exposure and gain as the record
    carries it (the pack's `pair_ae`): `leader` and `follower` of a pair
    that follows its leader's loop, `locked` on a declared gain; {} where
    each link runs its own loop."""
    ae = ae or {}
    if ae.get("mode") == "follow":
        return {str(ae["leader"]): "leader", str(ae["follower"]): "follower"}
    if ae.get("mode") == "locked":
        return {str(name): "locked" for name in ae.get("links") or []}
    return {}


def _ae_kwargs(fn, ae: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """`ae_roles=` and `gain_db=` for a caps hook that takes them."""
    roles = _ae_roles(ae)
    if roles and "ae_roles" in _accepted(fn):
        return {"ae_roles": roles, "gain_db": (ae or {}).get("gain_db")}
    return {}


def _pair_ae(pack, flows, topology: Topology, source: str) -> Optional[Dict[str, Any]]:
    """Who sets the exposure and the gain of the port's camera links under
    `source`, from the pack's `pair_ae`; None for a pack without the law."""
    hook = getattr(flows, "pair_ae", None)
    return dict(hook(pack, topology, source)) if hook is not None else None


def _port_viewer_hint(pack, flows, topology: Topology, links: List[LinkSpec],
                      mode, triggered: bool = False,
                      rates: Optional[Dict[str, float]] = None,
                      trigger_frames: Optional[Dict[str, int]] = None
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
                                  **_frames_kwarg(flows.viewer_hints, trigger_frames))
    except InfeasibleConfig:
        return None


def _hints_by_link(pack, flows, topology: Topology, links: List[LinkSpec],
                   modes, triggered: bool = False,
                   rates: Optional[Dict[str, float]] = None,
                   trigger_frames: Optional[Dict[str, int]] = None,
                   ae: Optional[Dict[str, Any]] = None
                   ) -> Optional[Dict[str, Dict[str, Any]]]:
    """Per-link viewer caps from packs that compose them per link, their
    capture mode index taken from the booted tree when it has one, each
    carrying the link's part in the exposure and gain `ae` records."""
    hook = getattr(flows, "viewer_hints_by_link", None)
    if hook is None:
        return None
    try:
        hints = hook(pack, topology, links=links, modes=modes,
                     triggered=triggered,
                     **_rate_kwarg(hook, rates),
                     **_frames_kwarg(hook, trigger_frames),
                     **_ae_kwargs(hook, ae))
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


#: Why an exposure asked in free run is refused, or under a frame sync whose
#: pulse does not set it: the capture stack's loop owns the exposure.
LOOP_EXPOSURE = ("the capture stack's exposure loop sets the exposure; "
                 "`set exposure` sets it on a running link")


def _fsync_plan(flows, pack, topology: Topology, fps: float, modes):
    """The pack's frame-sync plan (whether the pulse sets the exposure, the
    trigger frames), None for a pack without the law."""
    hook = getattr(flows, "fsync_plan", None)
    if hook is None:
        return None
    kwargs: Dict[str, Any] = {}
    if modes and "modes" in _accepted(hook):
        kwargs["modes"] = modes
    return hook(pack, topology, fps, **kwargs)


#: What sets a synced link's exposure when its pulse does.
PULSE_EXPOSURE = "the trigger pulse's low time"


def plan_text(plan) -> str:
    """One line on what the synced pair runs: the frame rate, the exposure
    when the pulse sets it, the trigger frame."""
    parts = []
    if plan.pulse_exposure:
        parts.append(f"exposure {PULSE_EXPOSURE}")
    frames = sorted(set(plan.trigger_vmax.values()))
    if len(frames) == 1:
        parts.append(f"trigger frame {frames[0]} lines")
    else:
        parts.append("trigger frame " + ", ".join(
            f"{k} {v}" for k, v in sorted(plan.trigger_vmax.items())))
    return f"fsync {plan.fps:g} fps: " + ", ".join(parts)


def sync_text(live: Dict[str, Any]) -> str:
    """The live sync record as `status` prints it."""
    fps = live.get("fps")
    text = str(live["source"]) + (f" {fps:g} fps" if fps else "")
    if live.get("pulse_exposure"):
        text += f" (exposure {PULSE_EXPOSURE})"
    return text


def pulse_exposure_fact(fps: Optional[float]) -> str:
    """Why an exposure asked under frame sync is refused: the trigger pulse
    sets it, and the rate sets the pulse. The fact names the rate, never a
    number the pulse's width would need."""
    at = f" at {float(fps):g} fps" if fps is not None else ""
    return (f"under frame sync the exposure is {PULSE_EXPOSURE}{at}; "
            f"a shorter exposure needs a higher rate or less light")


def pulse_sets_exposure(topology: Topology, link: str) -> bool:
    """True when the recorded frame sync's pulse sets the link's exposure:
    the port runs fsync, the link takes the pulse, and its sensor integrates
    for the pulse's low time (False where the sensor's shutter works under
    its trigger)."""
    sync = port_state.port_sync(topology) or {}
    if sync.get("source") != "fsync" or not sync.get("pulse_exposure"):
        return False
    return str((sync.get("links") or {}).get(link, "fsync")).startswith("fsync")


def pair_gain_fact(topology: Topology) -> Optional[str]:
    """Why a gain asked of the port is refused while the recorded sync gives
    its camera links one gain (the pack's `pair_ae`): the leader's capture
    session decides it and nxsd copies it to the follower's head, or the
    declared `camera.gain_db` locks it; None where each link runs its own
    loop."""
    ae = (port_state.port_sync(topology) or {}).get("ae") or {}
    if ae.get("mode") == "follow":
        return (f"link {ae['leader']}'s capture session decides the pair's gain and nxsd "
                f"copies it to link {ae['follower']}")
    if ae.get("mode") == "locked":
        return (f"camera.gain_db locks links {' and '.join(ae.get('links') or [])} "
                f"at {float(ae['gain_db']):.1f} dB")
    return None


def declared_exposure_fact(topology: Topology, synced: bool) -> str:
    """Why a declared `camera.exposure_us` is refused: under the port's frame
    sync (``synced``) the pulse sets the exposure at the rate the port runs
    (`synced_fps`); otherwise the capture stack's loop does."""
    if not synced:
        return LOOP_EXPOSURE
    return pulse_exposure_fact(topology.synced_fps)


def _record_hints(pack, flows, topology: Topology, triggered: bool,
                  frames: Optional[Dict[str, int]] = None,
                  ae: Optional[Dict[str, Any]] = None) -> None:
    """Re-derive every link's capture caps from the port record. Free
    running, the caps carry the rate each sensor was programmed for (the
    record's rates); under the trigger the pack's hook derives the trigger
    frame's, the plan's `frames` when given. Each link's caps carry its
    part in the exposure and gain `ae` records."""
    links = list(topology.links)
    modes = {l.name: m for l in links
             if (m := port_state.port_mode(topology, l))} or None
    rates = ({l.name: r for l in links
              if (r := port_state.port_rate(topology, l)) is not None} or None
             if not triggered else None)
    port_state.set_viewer_hints(
        _port_viewer_hint(pack, flows, topology, links, None,
                          triggered=triggered, rates=rates, trigger_frames=frames),
        topology=topology,
        viewers=_hints_by_link(pack, flows, topology, links, modes,
                               triggered=triggered, rates=rates,
                               trigger_frames=frames, ae=ae))


def _record_sync(pack, flows, topology: Topology, source: str,
                 fps: Optional[float], plan=None) -> None:
    """Record the port's live sync: the caps each link's consumers open
    with, and the sync itself with who sets the exposure and the gain of
    its camera links under it."""
    triggered = source == "fsync"
    frames = dict(plan.trigger_vmax) if (triggered and plan is not None) else None
    ae = _pair_ae(pack, flows, topology, source)
    _record_hints(pack, flows, topology, triggered, frames, ae=ae)
    per_link = _sync_links(pack, flows, topology) if triggered else None
    port_state.set_sync(
        source, fps if triggered else None, topology=topology, links=per_link,
        pulse_exposure=bool(triggered and plan is not None and plan.pulse_exposure),
        trigger_vmax=frames, ae=ae)
    for name, what in (per_link or {}).items():
        if not what.startswith("fsync"):
            term.warn(f"link {name} is not synced: "
                      f"{what[len('free_run ('):-1]}")
