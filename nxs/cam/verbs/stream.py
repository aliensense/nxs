"""`stream` and `capture`: viewers on the display and the headless delivery check."""

from __future__ import annotations

import argparse
from pathlib import Path


from nxs import term

from nxs import host as host_layer
from nxs.cam import port_state
from nxs.cam.port_state import port_capture_id
from nxs.cam import capture, viewers
from nxs.cam.identity import _require_nxs_hub
from nxs.cam.select import _hub_for, _port_name, _refuse, _refuse_pod_only, _require_up, select_port_links
from nxs.cam.verbs import verify


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


def cmd_stream(args: argparse.Namespace) -> int:
    topology, selected = _selected_up_links(args, "stream")
    if not selected:
        raise _refuse(f"nxs: no link is up on {_port_name(topology)}",
                      f"nxs {_port_name(topology)} <link> on")
    hub = _hub_for(topology)
    flows = hub.flows()
    # A link without a capture id on this boot gets no viewer: say why
    # (the missing VC overlay, or a port not up) instead of silence.
    for link in selected:
        if port_capture_id(topology, link) is None:
            term.warn(verify._no_capture_id(topology, link))
    # Replace, never stack: a second `stream` on a live link would leave
    # two clients fighting for one capture id.
    for stopped in viewers.stop_viewers(topology, selected):
        term.info(f"replacing viewer for link {stopped}")

    gate = verify.csi_gate(hub, flows, topology)

    exposure, gain = getattr(args, "exposure", None), getattr(args, "gain", None)
    if (exposure is None) != (gain is None):
        raise _refuse("nxs: --exposure and --gain lock the ISP together",
                      "both, or neither (the ISP's own 3A)")
    # The gate takes the bus lock for its writes alone: the viewers start
    # with the bus free.
    alive = viewers.launch_viewers(
        topology, selected, gate,
        exposure=exposure, gain=gain,
        hud=getattr(args, "hud", True),
    )
    print(f"viewers: {alive}/{len(selected)}")
    return 0 if alive == len(selected) else 1


def cmd_capture(args: argparse.Namespace) -> int:
    topology, selected = _selected_up_links(args, "capture")
    if len(selected) != 1:
        return _capture_port(args, topology, selected)
    link = selected[0]

    hints = viewers.resolve_capture_hints(
        port_state.viewer_hints(topology, link.name))
    capture_id = port_capture_id(topology, link)
    if capture_id is None:
        raise SystemExit(verify._no_capture_id(topology, link))

    hub = _hub_for(topology)
    flows = hub.flows()
    gate = verify.csi_gate(hub, flows, topology)

    snapshot_dir = getattr(args, "snapshot", None)
    if snapshot_dir:
        Path(snapshot_dir).mkdir(parents=True, exist_ok=True)
    # The tool's own viewer on the link holds its capture session: stopped
    # here, past the refusals, just before the consumer takes the session.
    verify.stop_viewers_for_count(topology, [link], "the capture")
    result = capture.headless_capture(
        hints, capture_id, gate, frames=args.frames,
        timeout_s=args.timeout, snapshot_dir=snapshot_dir,
        encoder=getattr(args, "encoder", capture.DEFAULT_ENCODER),
        port=_port_name(topology),
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
    verify.stop_viewers_for_count(topology, selected, "the capture")
    outputs = verify.count(topology, selected, _hub_for(topology),
                    {link.name: args.frames for link in selected}, timeout_s=args.timeout)
    port = _port_name(topology)
    rc = 0
    for name, output in sorted(outputs.items()):
        delivered = capture.count_delivered(output)
        errors = capture.count_stack_errors(output)
        fps = capture.delivered_rate(output)
        rate = f" at {fps:.1f} fps" if fps else ""
        tail = f", {errors} capture-stack errors" if errors else ""
        print(f"{port}/{name}: {delivered}/{args.frames}{rate}{tail}")
        if delivered < args.frames or errors:
            rc = 1
    return rc


