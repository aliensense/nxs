# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Viewers: capture-stack clients on the local display. The first client
starts against a gated-off CSI output, then the gate opens; later clients start
against live video. Viewer caps come from the port record written at `on`;
the pipeline's platform elements come from the host layer."""

from __future__ import annotations

import os
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from nxs import host as host_layer

from .contracts import LinkSpec, Topology
from . import port_state

def resolve_capture_hints(hints: Optional[Dict[str, Any]],
                          link: Optional[str] = None) -> Dict[str, Any]:
    """The port record's capture caps for a link: geometry, the capture
    node's mode index, the rate. A record without them is a named refusal
    (the port was never brought up by this tool, or by another version
    of it): a capture on a guessed node is the grey screen."""
    hints = hints or {}
    missing = [k for k in ("width", "height") if not hints.get(k)]
    if hints.get("sensor_mode") is None:
        missing.append("sensor_mode")
    if missing:
        who = f"link {link}" if link else "the link"
        raise SystemExit(
            f"no capture caps recorded for {who} ({', '.join(missing)} "
            f"missing): the port record predates this tool or the link "
            f"was never brought up — rerun on")
    resolved = {
        "width": int(hints["width"]),
        "height": int(hints["height"]),
        "sensor_mode": int(hints["sensor_mode"]),
        "framerate": hints.get("framerate"),
    }
    for key in ("exposure_us", "exposure_min_us", "exposure_max_us", "gain_db"):
        if hints.get(key) is not None:
            resolved[key] = float(hints[key])
    for key in ("ae_role", "ae_peer"):
        if hints.get(key):
            resolved[key] = str(hints[key])
    return resolved


def source_props(hints: Dict[str, Any], host=None, exposure_ns: Optional[int] = None,
                 gain: Optional[int] = None, port: Optional[str] = None) -> str:
    """The capture source's properties for a link's caps, the same for
    every consumer (a viewer, `capture`, `nxs.cam.frames`, the ROS 2
    camera node). A developer's `exposure_ns` and `gain` lock the loop at
    them; else a synced pair's link runs its part (`ae_role`: the pair's
    one exposure and gain, no ISP digital gain, noise reduction or edge
    enhancement), a follower's session starting at the leader's gain the
    follower heartbeat of `port` names, so the driver's write at the
    session's start leaves the leader's gain on the follower head; else a
    pinned `exposure_us` fixes the exposure and the loop drives gain; else
    the row's range bounds the loop to the frame; else the source runs as
    it opens."""
    host = host or host_layer.current()
    if exposure_ns is not None and gain is not None:
        return host.locked_props(exposure_ns, gain)
    role = hints.get("ae_role")
    if role:
        gain_db = hints.get("gain_db")
        if role == "follower":
            gain_db = port_state.follow_gain_db(port) if port else None
        return host.pair_props(role, hints.get("exposure_us"), gain_db)
    if hints.get("exposure_us") is not None:
        return host.exposure_props(hints["exposure_us"])
    if hints.get("exposure_max_us") is not None:
        return host.ae_props(hints.get("exposure_min_us"), hints["exposure_max_us"])
    return ""


def _display_candidates():
    """(display, xauthority) pairs to try, the caller's own env first."""
    env_display = os.environ.get("DISPLAY")
    env_xauth = os.environ.get("XAUTHORITY")
    if env_display:
        yield env_display, env_xauth
    x11 = Path("/tmp/.X11-unix")
    if not x11.is_dir():
        return
    xauths = [env_xauth] if env_xauth else []
    xauths += sorted(str(p) for p in Path("/run/user").glob("*/gdm/Xauthority"))
    xauths.append(None)
    for sock in sorted(x11.iterdir()):
        display = ":" + sock.name.lstrip("X")
        for xauth in xauths:
            yield display, xauth


def find_display() -> Optional[Dict[str, str]]:
    """An answering display's environment: the caller's own DISPLAY/XAUTHORITY
    first, then every X socket against every known Xauthority."""
    seen = set()
    for display, xauth in _display_candidates():
        if (display, xauth) in seen:
            continue
        seen.add((display, xauth))
        env = dict(os.environ, DISPLAY=display)
        if xauth:
            env["XAUTHORITY"] = xauth
        else:
            env.pop("XAUTHORITY", None)
        probe = subprocess.run(["xset", "q"], env=env, capture_output=True)
        if probe.returncode == 0:
            return env
    return None


#: How long a viewer gets to close its capture session after SIGTERM before
#: it is killed outright.
VIEWER_STOP_GRACE_S = 4.0


def _gone(pattern: str, timeout_s: float = VIEWER_STOP_GRACE_S, run=subprocess) -> bool:
    """True once no process matches `pattern`, polling up to the grace."""
    deadline = time.time() + timeout_s
    while True:
        if run.run(["pgrep", "-f", pattern], capture_output=True).returncode != 0:
            return True
        if time.time() >= deadline:
            return False
        time.sleep(0.1)


def stop_viewers(topology: Topology, links: List[LinkSpec], run=subprocess) -> List[str]:
    """Stop the viewers for the selected links: SIGTERM, a grace for the
    capture session to close, then SIGKILL. Names each link whose viewer
    ran; `pkill` matching nothing is a link without one."""
    host = host_layer.current()
    stopped = []
    for link in links:
        capture_id = port_state.port_capture_id(topology, link)
        if capture_id is None:
            continue
        match = host.viewer_match(capture_id)
        if run.run(["pkill", "-f", match], capture_output=True).returncode != 0:
            continue
        forced = ""
        if not _gone(match, run=run):
            run.run(["pkill", "-9", "-f", match], capture_output=True)
            forced = ", killed after the grace"
        stopped.append(f"{link.name} (capture id {capture_id}{forced})")
    return stopped


def viewer_geometry(n: int, row: int, width: int, shown_height: int,
                    rows: int = 1) -> Dict[str, int]:
    """Slot ``n`` of a port's viewers on screen row ``row``: links side by side,
    ports one row below the other, at 3/4 scale for one port and 1/2 for two."""
    scale = 0.75 if rows <= 1 else 0.5
    w, h = int(width * scale), int(shown_height * scale)
    return {"x": 60 + n * (w + 40), "y": 60 + row * (h + 90), "w": w, "h": h}


def port_row(port: str) -> Tuple[int, int]:
    """(row, rows): a port's screen row among the known ports by name
    (cam0 above cam1), and how many ports there are; a port the ports do
    not know sits alone on row 0."""
    try:
        from .topology import load_ports
        ports, _ = load_ports(None)
        names = sorted({str(t.carrier).rsplit("/", 1)[-1] for t in ports.values()})
        return (names.index(port) if port in names else 0), max(1, len(names))
    except Exception:
        return 0, 1


def other_viewers_alive(own: Dict[int, subprocess.Popen]) -> bool:
    """Viewers of another port on this host (HUD or plain), by process list;
    the capture daemon must not be bounced under them."""
    mine = {p.pid for p in own.values()}
    try:
        out = subprocess.run(
            ["pgrep", "-f", host_layer.current().viewers_pattern()],
            capture_output=True, text=True).stdout
    except OSError:
        return False
    return any(int(pid) not in mine for pid in out.split() if pid.isdigit())


def launch_viewers(
    topology: Topology,
    links: List[LinkSpec],
    gate,
    hints: Optional[Dict[str, Any]] = None,
    exposure: Optional[int] = None,
    gain: Optional[int] = None,
    hud: bool = True,
) -> int:
    """Start viewers with the CSI-gate choreography; ``gate(enable)`` toggles the
    CSI output gate, ``hints`` override the port record's caps, ``hud`` picks
    the overlay viewer over the plain client. With both ``exposure`` (ns) and
    ``gain`` given the ISP's adaptation is locked at them; otherwise each
    link's caps set its source (`source_props`). Returns the viewers alive."""
    host = host_layer.current()
    env = find_display()
    if not env:
        raise SystemExit("no answering display found — log in (or export "
                         "DISPLAY/XAUTHORITY) and rerun")

    # Each link has its own caps on a mixed hub (its sensor's geometry
    # and DT mode); one set of hints applies to every link otherwise.
    def hints_for(link: LinkSpec) -> Dict[str, Any]:
        return (hints or port_state.viewer_hints(topology, link.name)
                or port_state.viewer_hints(topology) or {})

    by_id = {}
    for link in links:
        capture_id = port_state.port_capture_id(topology, link)
        if capture_id is not None:
            by_id[capture_id] = link
    if not by_id:
        raise SystemExit("selected links have no capture id in this port")
    sync = port_state.port_sync(topology) or {}
    port = port_state.port_name(topology)
    first_link = by_id[sorted(by_id, key=lambda a: by_id[a].name)[0]]
    first = hints_for(first_link)
    first_resolved = resolve_capture_hints(first, first_link.name)
    width = first_resolved["width"]
    shown_height = first_resolved["height"] - int(first.get("crop_bottom", 0) or 0)

    def viewer_log(capture_id: int) -> str:
        return str(port_state.state_dir() / f"viewer-{capture_id}.log")

    row, rows = port_row(port)

    def geometry(n: int) -> Dict[str, int]:
        return viewer_geometry(n, row, width, shown_height, rows)

    def client(capture_id: int, n: int) -> subprocess.Popen:
        link = by_id[capture_id]
        link_hints = hints_for(link)
        resolved = resolve_capture_hints(link_hints, link.name)
        w_, h_ = resolved["width"], resolved["height"]
        fr = resolved["framerate"]
        sensor_mode = resolved["sensor_mode"]
        crop_bottom = int(link_hints.get("crop_bottom", 0) or 0)
        # A named exposure and gain lock the ISP-side adaptation at them (the
        # developer's fixed picture); else the caps decide (`source_props`).
        locked = exposure is not None and gain is not None
        props = source_props(resolved, host, exposure, gain, port)
        caps = host.caps(w_, h_, fr)
        g = geometry(n)
        if hud:
            from . import hud as hud_app
            text = hud_app.hud_text(
                port, link.name, capture_id, w_, h_,
                str(link_hints.get("data_type", "")), sensor_mode, sync,
                ae=hud_app.ae_token(resolved, locked),
                sensor=str(link_hints.get("sensor") or link.sensor_compatible or ""),
                link_sync=str((sync.get("links") or {}).get(link.name, "")))
            cmd = [sys.executable, "-m", "nxs.cam.hud",
                   "--capture-id", str(capture_id),
                   "--sensor-mode", str(sensor_mode),
                   "--caps", caps, "--props", props,
                   "--crop-bottom", str(crop_bottom), "--text", text,
                   "--declared", hud_app.sync_text(sync),
                   "--x", str(g["x"]), "--y", str(g["y"]),
                   "--w", str(g["w"]), "--h", str(g["h"])]
        else:
            pipeline = host.viewer_pipeline(capture_id, sensor_mode, props, caps,
                                            crop_bottom, g)
            cmd = ["gst-launch-1.0", *shlex.split(pipeline)]
        log = open(viewer_log(capture_id), "w")
        return subprocess.Popen(cmd, env=env, stdout=log,
                                stderr=subprocess.STDOUT)

    def reason_line(capture_id: int) -> str:
        """The line a viewer's log names its failure on: the last that
        carries `ERROR`, else the last. The capture stack's shutdown lines
        follow the error, and a byte that is not UTF-8 reads as a mark."""
        try:
            text = Path(viewer_log(capture_id)).read_text(errors="replace")
        except OSError:
            return ""
        lines = [l.strip() for l in text.splitlines() if l.strip()]
        errors = [l for l in lines if "ERROR" in l]
        return (errors or lines or [""])[-1]

    # Capture first, gate second; all-or-nothing with retries behind a fresh
    # daemon. Slots go by link name (A left of B), not by capture id.
    ordered = sorted(by_id, key=lambda aid: by_id[aid].name)
    procs: Dict[int, subprocess.Popen] = {}
    # A daemon bounce kills every capture session on the host: only bounce
    # when no other port's viewers are streaming.
    shared = other_viewers_alive(procs)
    if shared:
        print("another port's viewers are streaming — the capture daemon "
              "stays up")
    else:
        host.restart_capture_daemon()
    for attempt in range(3):
        started: Dict[int, subprocess.Popen] = {}
        gate(False)
        procs[ordered[0]] = started[ordered[0]] = client(ordered[0], 0)
        time.sleep(2.5)
        try:
            gate(True)
        except BaseException:
            # A gate that does not open leaves no viewer on a closed output.
            procs[ordered[0]].terminate()
            raise
        time.sleep(5)
        if procs[ordered[0]].poll() is None:
            for n, capture_id in enumerate(ordered[1:], start=1):
                procs[capture_id] = started[capture_id] = client(capture_id, n)
            time.sleep(8)
            if all(p.poll() is None for p in procs.values()):
                break
        if attempt == 2:
            break
        # The next attempt rewrites each log: the reason a viewer exited is
        # read now and said here, not left in a file the restart truncates.
        again = "behind a fresh capture daemon" if not shared else "on the running capture daemon"
        for capture_id, proc in sorted(started.items()):
            if proc.poll() is not None:
                tail = reason_line(capture_id)
                print(f"viewer for capture id {capture_id} exited"
                      + (f" ({tail})" if tail else "")
                      + f"; starting the viewers again {again}")
        for p in procs.values():
            if p.poll() is None:
                p.terminate()
        if not shared:
            host.restart_capture_daemon()
    # Count only these viewers, and say why a dead one died: the log tail
    # names the failing element.
    alive = 0
    for capture_id, proc in sorted(procs.items()):
        if proc.poll() is None:
            alive += 1
        else:
            tail = reason_line(capture_id)
            print(f"viewer for capture id {capture_id} died — "
                  f"log: {viewer_log(capture_id)}"
                  + (f" ({tail})" if tail else ""))
    return alive
