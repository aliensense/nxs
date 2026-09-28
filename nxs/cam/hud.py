# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The viewer with a heads-up display: one capture client whose picture carries
its identity (port, link, capture id, geometry, transport, sync) and the frame
rate the source delivers. A debugging surface only, launched by `stream`."""

from __future__ import annotations

import argparse
import shlex
import signal
import sys
import time
from typing import Any, Dict, Optional

from nxs import host as host_layer

FPS_WINDOW_S = 1.0
#: Every this many ticks the measured line also goes to the viewer log
#: (`~/.local/state/nxs/viewer-N.log`).
LOG_EVERY_TICKS = 5
#: Overlay font, small and in the corner.
FONT = "DejaVu Sans Mono 9"
#: Frame timestamps kept for the period/jitter measurement.
PTS_KEEP = 64
#: The metering queue's name and the probe pad: its sink pad sees every
#: buffer the source pushes, whatever the display side drops.
METER = "meter"
PROBE_PADS = ("sink",)


def sync_text(sync: Optional[Dict[str, Any]]) -> str:
    """`fsync 30 fps` / `free-run` / `sync ?` from the port record."""
    if not sync:
        return "sync ?"
    if sync.get("source") == "fsync":
        fps = sync.get("fps")
        return f"fsync {fps:g} fps" if fps else "fsync"
    return "free-run"


def hud_text(port: str, link: str, capture_id: int, width: int, height: int,
             data_type: str, sensor_mode: int,
             sync: Optional[Dict[str, Any]], locked: bool = True,
             sensor: str = "", link_sync: str = "") -> str:
    """The static identity line: port and link, the sensor, capture id,
    geometry; a link the port trigger leaves free-running says so here."""

    del sync
    short = sensor.split(",")[-1] if sensor else ""
    parts = [f"{port} {link}", short, f"capture {capture_id}",
             f"{width}x{height} {data_type} dt-mode {sensor_mode}".rstrip(),
             "isp-locked" if locked else "isp-auto"]
    if link_sync.startswith("free_run ("):
        parts.append(f"not synced: {link_sync[len('free_run ('):-1]}")
    return " · ".join(p for p in parts if p)


def fps_line(delivered: int, elapsed_s: float, window_fps: float,
             period_ms: Optional[float] = None,
             jitter_ms: Optional[float] = None,
             shown_fps: Optional[float] = None,
             declared: Optional[str] = None) -> str:
    """The live line: the declared sync, the rate measured from frame timestamps
    (period and jitter, or the per-second count without timestamps), the rate
    painted on the monitor, then total frames and elapsed time."""
    if period_ms:
        rate = (f"measured {1000.0 / period_ms:.2f} fps ±{jitter_ms:.2f} ms"
                if jitter_ms is not None else
                f"measured {1000.0 / period_ms:.2f} fps")
    else:
        rate = f"measured {window_fps:.1f} fps"
    head = f"declared {declared} · " if declared else ""
    shown = f" · shown {shown_fps:.1f} fps" if shown_fps is not None else ""
    return f"{head}{rate}{shown} · {delivered} frames · {elapsed_s:.0f} s"


#: The display sink is X11 XVideo; an EGL sink caps the source at the monitor refresh.
SINK = "sink"


def build_pipeline(capture_id: int, sensor_mode: int, caps: str, props: str,
                   crop_bottom: int, x: int = 0, y: int = 0, w: int = 0,
                   h: int = 0) -> str:
    """The viewer pipeline: source, meter, overlay, XVideo sink. The
    window geometry is applied by the program's own window, not here."""
    del x, y, w, h
    host = host_layer.current()
    crop = f"videocrop bottom={crop_bottom} ! " if crop_bottom else ""
    return (
        f"{host.source(capture_id, sensor_mode)} {props} ! {caps} ! "
        f"queue name={METER} leaky=downstream max-size-buffers=1 "
        f"max-size-bytes=0 max-size-time=0 ! "
        f"{host.convert()} ! {crop}"
        f'textoverlay name=hud text="" valignment=top halignment=left '
        f'font-desc="{FONT}" shaded-background=false line-alignment=left '
        f"wrap-mode=none xpad=6 ypad=4 ! "
        f"xvimagesink name={SINK} sync=false"
    )


def measure_period(pts_ns) -> tuple:
    """(mean period ms, jitter ms) over the kept frame timestamps, jitter being
    the half-range of the intervals; (None, None) below two frames."""
    if len(pts_ns) < 2:
        return None, None
    gaps = [b - a for a, b in zip(pts_ns, pts_ns[1:]) if b > a]
    if not gaps:
        return None, None
    mean_ms = sum(gaps) / len(gaps) / 1e6
    jitter_ms = (max(gaps) - min(gaps)) / 2 / 1e6
    return mean_ms, jitter_ms


def _parse(argv):
    p = argparse.ArgumentParser(prog="nxs.cam.hud")
    p.add_argument("--capture-id", type=int, required=True)
    p.add_argument("--sensor-mode", type=int, required=True)
    p.add_argument("--caps", required=True)
    p.add_argument("--props", default="")
    p.add_argument("--crop-bottom", type=int, default=0)
    p.add_argument("--text", default="")
    p.add_argument("--declared", default="",
                   help="the declared sync (`fsync 100 fps`, `free-run`)")
    p.add_argument("--x", type=int, default=60)
    p.add_argument("--y", type=int, default=60)
    p.add_argument("--w", type=int, default=774)
    p.add_argument("--h", type=int, default=582)
    return p.parse_args(argv)


def install_term_handler(glib, signal_number: int, quit_loop) -> None:
    """Route a termination signal into the GLib main loop: the callback
    quits the loop (once) instead of the process dying mid-pipeline."""
    def on_signal(*_args):
        quit_loop()
        return False                       # one shot: GLib.SOURCE_REMOVE

    glib.unix_signal_add(glib.PRIORITY_HIGH, signal_number, on_signal)


def main(argv=None) -> int:
    args = _parse(argv)
    import gi
    gi.require_version("Gst", "1.0")
    gi.require_version("GstVideo", "1.0")
    gi.require_version("Gtk", "3.0")
    gi.require_version("GdkX11", "3.0")
    from gi.repository import GdkX11, GLib, Gst, GstVideo, Gtk  # noqa: F401

    Gst.init(None)
    # A normal managed window placed like the plain viewer's; the drawing
    # area's size request keeps GNOME from cascading and inflating it.
    window = Gtk.Window(type=Gtk.WindowType.TOPLEVEL)
    window.set_title(f"nxs {args.text.split(' · ')[0]}")
    area = Gtk.DrawingArea()
    area.set_size_request(args.w, args.h)
    window.add(area)
    window.move(args.x, args.y)
    window.set_default_size(args.w, args.h)
    window.show_all()
    xid = area.get_window().get_xid()
    desc = build_pipeline(args.capture_id, args.sensor_mode, args.caps,
                          args.props, args.crop_bottom,
                          args.x, args.y, args.w, args.h)
    print(f"hud pipeline: {desc}", flush=True)
    pipeline = Gst.parse_launch(desc)
    overlay = pipeline.get_by_name("hud")
    state = {"delivered": 0, "window": 0, "shown": 0, "t0": time.monotonic(),
             "t_win": time.monotonic(), "pts": []}

    def on_buffer(pad, info):
        state["delivered"] += 1
        state["window"] += 1
        buf = info.get_buffer()
        pts = buf.pts if buf is not None else Gst.CLOCK_TIME_NONE
        if pts != Gst.CLOCK_TIME_NONE:
            state["pts"].append(int(pts))
            del state["pts"][:-PTS_KEEP]
        return Gst.PadProbeReturn.OK

    # Count on the metering queue's sink pad: every buffer the source pushes
    # passes it, even the ones the leaky queue drops for a slow display.
    meter = pipeline.get_by_name(METER)
    pad = next((meter.get_static_pad(n) for n in PROBE_PADS
                if meter.get_static_pad(n) is not None), None)
    if pad is None:
        print(f"ERROR: queue {METER} has none of the pads "
              f"{PROBE_PADS} — cannot count frames", flush=True)
        return 1
    pad.add_probe(Gst.PadProbeType.BUFFER, on_buffer)

    # And what the monitor actually gets: the buffers reaching the sink.
    def on_shown(pad, info):
        state["shown"] += 1
        return Gst.PadProbeReturn.OK

    sink_pad = pipeline.get_by_name(SINK).get_static_pad("sink")
    if sink_pad is not None:
        sink_pad.add_probe(Gst.PadProbeType.BUFFER, on_shown)

    def tick():
        now = time.monotonic()
        span = now - state["t_win"]
        fps = state["window"] / span if span > 0 else 0.0
        shown_fps = state["shown"] / span if span > 0 else 0.0
        state["window"], state["shown"], state["t_win"] = 0, 0, now
        period_ms, jitter_ms = measure_period(state["pts"])
        line = fps_line(state["delivered"], now - state["t0"], fps,
                        period_ms, jitter_ms, shown_fps,
                        declared=args.declared or None)
        overlay.set_property("text", f"{args.text}\n{line}")
        state["ticks"] = state.get("ticks", 0) + 1
        if state["ticks"] % LOG_EVERY_TICKS == 0:
            print(f"hud: {line}", flush=True)
        return True

    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.enable_sync_message_emission()
    rc = {"code": 0}

    def on_sync(_bus, msg):
        # The sink asks for a window: hand it this one (GstVideoOverlay).
        s = msg.get_structure()
        if s is not None and s.get_name() == "prepare-window-handle":
            msg.src.set_window_handle(xid)

    def on_message(_bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"ERROR from {msg.src.get_name()}: {err.message}"
                  + (f" ({dbg})" if dbg else ""), flush=True)
            rc["code"] = 1
            Gtk.main_quit()
        elif msg.type == Gst.MessageType.EOS:
            print("EOS", flush=True)
            Gtk.main_quit()

    bus.connect("sync-message::element", on_sync)
    bus.connect("message", on_message)
    window.connect("destroy", lambda *_: Gtk.main_quit())
    # `nxs cam0 off` stops a viewer with SIGTERM: leave through the main loop
    # so the pipeline reaches NULL and the capture session closes.
    install_term_handler(GLib, signal.SIGTERM, Gtk.main_quit)
    GLib.timeout_add(int(FPS_WINDOW_S * 1000), tick)
    pipeline.set_state(Gst.State.PLAYING)
    try:
        Gtk.main()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
    return rc["code"]


if __name__ == "__main__":
    sys.exit(main())
