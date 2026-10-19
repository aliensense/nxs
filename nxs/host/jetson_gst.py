# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The GStreamer dialect of a Jetson host: the Argus source and its
properties, the colour conversions, the viewer pipeline and the JPEG
encoder, as the strings the verbs launch."""

from __future__ import annotations

from typing import Dict, Optional

#: The Argus source's white-balance modes a fixed picture runs: off for a
#: developer's locked one, auto on each link of a synced pair.
WB_OFF, WB_AUTO = 0, 1


def _fixed_props(exposure_ns: Optional[int], wbmode: int, gain: Optional[float] = None) -> str:
    """The Argus source's properties for a picture the ISP leaves alone: the
    exposure pinned at `exposure_ns` (none when None), no digital gain,
    noise reduction or edge enhancement, white balance `wbmode`; with `gain`
    the exposure loop locked at that linear gain."""
    props = [f'exposuretimerange="{exposure_ns} {exposure_ns}"'] if exposure_ns is not None else []
    props += ['ispdigitalgainrange="1 1"', f"wbmode={int(wbmode)}", "tnr-mode=0", "ee-mode=0"]
    if gain is not None:
        props += ["aelock=true", f'gainrange="{gain:g} {gain:g}"']
    return " ".join(props)


class JetsonGst:
    """The pipeline strings; the host mixes it into `JetsonHost`."""

    # --- the capture consumer (Argus) --------------------------------------
    def source(self, capture_id: int, mode_index: int) -> str:
        return f"{self.source_match(capture_id)}sensor-mode={int(mode_index)}"

    def source_match(self, capture_id: int) -> str:
        return f"nvarguscamerasrc sensor-id={int(capture_id)} "

    def viewers_pattern(self) -> str:
        return "nxs[.]cam[.]hud|gst-launch-1[.]0 .*nvarguscamerasrc"

    def caps(self, width: int, height: int, framerate=None) -> str:
        out = f"video/x-raw(memory:NVMM),width={int(width)},height={int(height)}"
        if framerate:
            out += f",framerate={framerate[0]}/{framerate[1]}"
        return out

    def convert(self) -> str:
        return "nvvidconv ! video/x-raw,format=I420"

    def rgba_convert(self) -> str:
        # The converter reaches system memory as RGBA only from an RGBA device buffer: from the
        # source's buffer it hands out nothing on this release.
        return ("nvvidconv ! video/x-raw(memory:NVMM),format=RGBA ! "
                "nvvidconv ! video/x-raw,format=RGBA")

    def topic_convert(self, encoding: str) -> str:
        # The hardware converter writes UYVY and GRAY8 into system memory at the link's rate; RGB
        # needs the software converter after the RGBA path, a core at 1080p.
        tail = "queue max-size-buffers=2 leaky=downstream"
        if encoding == "jpeg":
            return f"nvjpegenc ! {tail}"
        if encoding == "rgb8":
            return f"{self.rgba_convert()} ! videoconvert ! video/x-raw,format=RGB ! {tail}"
        fmt = {"yuv422": "UYVY", "mono8": "GRAY8"}[encoding]
        return (f"nvvidconv ! video/x-raw(memory:NVMM),format={fmt} ! "
                f"nvvidconv ! video/x-raw,format={fmt} ! {tail}")

    def exposure_props(self, exposure_us: float) -> str:
        """The Argus property pinning the exposure at a port's value (the
        source takes nanoseconds); gain stays the loop's."""
        ns = int(round(float(exposure_us) * 1000))
        return f'exposuretimerange="{ns} {ns}"'

    def ae_props(self, exposure_min_us: Optional[float], exposure_max_us: float) -> str:
        """The Argus property bounding its exposure loop to the frame the
        sensor runs: a longer request would stretch the frame."""
        low = int(round(float(exposure_min_us or 1.0) * 1000))
        high = int(round(float(exposure_max_us) * 1000))
        return f'exposuretimerange="{low} {high}"'

    def locked_props(self, exposure_ns: int, gain: int) -> str:
        return _fixed_props(int(exposure_ns), WB_OFF, float(gain))

    def pair_props(self, role: str, exposure_us: Optional[float],
                   gain_db: Optional[float] = None) -> str:
        """Argus takes its exposure in nanoseconds and its gain as a linear
        factor: the `gain_db` of a locked link or a follower is 10^(dB/20),
        what the driver's decibel gain control converts back; a follower
        without one runs at unity."""
        ns = None if exposure_us is None else int(round(float(exposure_us) * 1000))
        if role == "leader":
            return _fixed_props(ns, WB_AUTO)
        if role == "follower" and gain_db is None:
            return _fixed_props(ns, WB_AUTO, 1.0)
        if role in ("follower", "locked") and gain_db is not None:
            return _fixed_props(ns, WB_AUTO, 10 ** (float(gain_db) / 20))
        raise ValueError(f"no source properties for a pair's {role!r} link"
                         + (" without its gain" if role == "locked" else ""))

    def viewer_pipeline(self, capture_id: int, mode_index: int, props: str, caps: str,
                        crop_bottom: int, geometry: Dict[str, int]) -> str:
        # videocrop removes a triggered pair's filler rows; it needs system
        # memory, so the buffers are handed back to NVMM for nv3dsink.
        crop = (f"{self.convert()} ! videocrop bottom={int(crop_bottom)} ! nvvidconv ! "
                f"video/x-raw(memory:NVMM),format=NV12 ! " if crop_bottom else "")
        return (f"{self.source(capture_id, mode_index)} {props} ! {caps} ! {crop}queue ! "
                f"nv3dsink window-x={geometry['x']} window-y={geometry['y']} "
                f"window-width={geometry['w']} window-height={geometry['h']}")

    def jpeg_encoder(self) -> str:
        return "nvjpegenc"

    def consumer_errors(self, output: str) -> int:
        return sum(1 for line in output.splitlines()
                   if "Error generated" in line or "(Argus) Error" in line)

    def consumer_hint(self) -> str:
        return ("a missing nvarguscamerasrc means the nvidia-l4t-gstreamer package "
                "is not installed on this host")
