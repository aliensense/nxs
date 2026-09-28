"""`stream` and `capture`: viewers on the display and the headless delivery check."""

from __future__ import annotations

import argparse
from pathlib import Path


from nxs import term

from nxs.cam.contracts import LinkSpec, Topology
from nxs.cam.plan import RawConfig
from nxs import host as host_layer
from nxs.cam import port_state
from nxs.cam.port_state import port_capture_id
from nxs.cam import capture, packs, viewers
from nxs.cam import run as cam_run
from nxs.cam.identity import _require_nxs_hub
from nxs.cam.select import _pack_for, _port_name, _refuse, _refuse_pod_only, _require_up, select_port_links


def _selected_up_links(args, verb):
    """Selection preamble for running-stream verbs: explicit link tokens
    must be up; a bare invocation narrows to the links that are."""
    topology, selected = select_port_links(args, require_port=True)
    _require_nxs_hub(topology, verb)
    if getattr(args, "_link_tokens", None):
        _require_up(topology, selected, verb)
    else:
        selected = [l for l in selected
                    if port_state.link_state(topology, l)
                    == port_state.STATE_UP]
    pods = [l for l in selected if not l.has_camera]
    selected = [l for l in selected if l.has_camera]
    if pods and not selected:
        raise _refuse_pod_only(topology, pods[0], verb)
    return topology, selected


def _no_capture_id(topology: Topology, link: LinkSpec) -> str:
    """Why a link has no capture id: no node for its channel, none the host reads, or no port state."""
    try:
        ids = host_layer.current().capture_ids(topology.i2c_bus)
    except Exception:
        ids = {}
    if ids and link.csi_vc not in ids:
        port = _port_name(topology)
        return (f"link {link.name} has no capture id on this boot: its virtual "
                f"channel {link.csi_vc} has no capture node in the booted overlay "
                f"(the label carries {', '.join(f'VC{vc}' for vc in sorted(ids))} only)\n"
                f"  - nxs switch (installs {port}'s overlay), then reboot")
    if not ids:
        try:
            booted = host_layer.current().booted_modes(topology.i2c_bus)
        except Exception:
            booted = []
        if booted:
            return (f"link {link.name} has no capture id: the booted tree carries "
                    f"{len(booted)} camera modes but no capture node this host reads\n"
                    f"  - nxs host info")
    return (f"link {link.name} has no capture id in the current port\n"
            f"  - nxs {_port_name(topology)} {link.name} on")


def _csi_gate(pack, flows, topology):
    """The CSI output-gate toggle both choreographed verbs share; a gate program
    that fails is a control-bus fault, raised as such."""
    def gate(enable: bool) -> None:
        cfg = RawConfig("csi-gate")
        if topology.is_direct:
            # The lanes are the sensor's own: its stream gate is the gate.
            cfg.set_address("ADR_SENSOR", packs.sensor_address(pack, topology.links[0]))
        cfg.add("gate", flows.csi_gate_steps(pack, topology, enable))
        if not cam_run._execute(cfg.to_dict(), topology.i2c_bus, "csi-gate",
                        guard=(pack, topology)):
            raise _refuse("csi-gate program failed (a control-bus fault)",
                          f"nxs {_port_name(topology)} status")
    return gate


def cmd_stream(args: argparse.Namespace) -> int:
    topology, selected = _selected_up_links(args, "stream")
    if not selected:
        raise _refuse(f"nxs: no link is up on {_port_name(topology)}",
                      f"nxs {_port_name(topology)} <link> on")
    pack = _pack_for(topology)
    flows = pack.flows()
    # A link without a capture id on this boot gets no viewer: say why
    # (the missing VC overlay, or a port not up) instead of silence.
    for link in selected:
        if port_capture_id(topology, link) is None:
            term.warn(_no_capture_id(topology, link))
    # Replace, never stack: a second `stream` on a live link would leave
    # two clients fighting for one capture id.
    for stopped in viewers.stop_viewers(topology, selected):
        term.info(f"replacing viewer for link {stopped}")

    gate = _csi_gate(pack, flows, topology)

    exposure, gain = getattr(args, "exposure", None), getattr(args, "gain", None)
    if (exposure is None) != (gain is None):
        raise _refuse("nxs: --exposure and --gain lock the ISP together",
                      "both, or neither (the ISP's own 3A)")
    with port_state.BusLock():
        count = viewers.launch_viewers(
            topology, selected, gate,
            exposure=exposure, gain=gain,
            hud=getattr(args, "hud", True),
        )
    print(f"viewers: {count}/{len(selected)}")
    return 0 if count == len(selected) else 1


def cmd_capture(args: argparse.Namespace) -> int:
    topology, selected = _selected_up_links(args, "capture")
    if len(selected) != 1:
        return _capture_port(args, topology, selected)
    link = selected[0]

    hints = viewers.resolve_capture_hints(
        port_state.viewer_hints(topology, link.name))
    capture_id = port_capture_id(topology, link)
    if capture_id is None:
        raise SystemExit(_no_capture_id(topology, link))

    pack = _pack_for(topology)
    flows = pack.flows()
    gate = _csi_gate(pack, flows, topology)

    snapshot_dir = getattr(args, "snapshot", None)
    if snapshot_dir:
        Path(snapshot_dir).mkdir(parents=True, exist_ok=True)
    with port_state.BusLock():
        result = capture.headless_capture(
            hints, capture_id, gate, frames=args.frames,
            timeout_s=args.timeout, snapshot_dir=snapshot_dir,
            encoder=getattr(args, "encoder", capture.DEFAULT_ENCODER),
        )
    detail = (f"{result.delivered}/{result.frames} frames via capture id "
              f"{capture_id} mode {hints['sensor_mode']} "
              f"({hints['width']}x{hints['height']})"
              + (f" at {result.fps:.2f} fps by the buffers' timestamps"
                 if result.fps else ""))
    if snapshot_dir:
        written = sorted(Path(snapshot_dir).glob("frame-*.jpg"))
        sizes = [p.stat().st_size for p in written]
        print(f"snapshots: {len(written)} in {snapshot_dir}"
              + (f" ({min(sizes)}..{max(sizes)} bytes)" if sizes else ""))
        if written and max(sizes) == 0:
            term.warn("snapshot files are empty: the encoder produced nothing\n"
                      "  - --encoder jpegenc")
    if result.ok:
        term.info(f"capture: {detail}")
        rate = f" at {result.fps:.1f} fps" if result.fps else ""
        print(f"{_port_name(topology)}/{link.name}: {result.delivered}/{result.frames}{rate}")
        return 0
    if result.consumer_error:
        print(f"capture FAILED before it started: {result.consumer_error}\n"
              f"  - {host_layer.current().consumer_hint()}")
        return 1
    if result.errors:
        why = f"{result.errors} capture-stack errors"
    elif result.delivered:
        why = "fewer frames than asked"
    else:
        why = "no frames delivered"
    print(f"capture FAILED after {result.attempts} attempts ({why}): {detail}")
    return 1


def _capture_port(args: argparse.Namespace, topology, selected) -> int:
    """Every up link of the port at once, one verdict line per link: the
    delivered count against the frames asked, at the rate the buffers'
    timestamps give. 0 when every link delivered every frame."""
    if getattr(args, "snapshot", None):
        raise _refuse("snapshots come from one link at a time",
                      f"nxs {_port_name(topology)} <link> capture --snapshot ...")
    hints_by_id = {}
    names = {}
    for link in selected:
        capture_id = port_capture_id(topology, link)
        if capture_id is None:
            raise SystemExit(_no_capture_id(topology, link))
        hints_by_id[capture_id] = viewers.resolve_capture_hints(
            port_state.viewer_hints(topology, link.name))
        names[capture_id] = link.name
    pack = _pack_for(topology)
    gate = _csi_gate(pack, pack.flows(), topology)
    with port_state.BusLock():
        outputs = capture.headless_pair(hints_by_id, gate, frames=args.frames,
                                        timeout_s=args.timeout)
    port = _port_name(topology)
    rc = 0
    for capture_id in sorted(hints_by_id, key=lambda c: names[c]):
        output = outputs.get(capture_id, "")
        delivered = capture.count_delivered(output)
        errors = capture.count_stack_errors(output)
        fps = capture.delivered_rate(output)
        rate = f" at {fps:.1f} fps" if fps else ""
        tail = f", {errors} capture-stack errors" if errors else ""
        print(f"{port}/{names[capture_id]}: {delivered}/{args.frames}{rate}{tail}")
        if delivered < args.frames or errors:
            rc = 1
    return rc


