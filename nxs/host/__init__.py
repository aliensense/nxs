# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The host capture layer: the booted capture contract (lane count, capture
ids, mode table), its generation and installation, the capture consumer and
the capture daemon. Everything the camera verbs need from the platform goes
through one `Host`; a port implements a subclass and a detector. The SerDes
pack and the sensor plugins never name a host."""

from __future__ import annotations

import glob
import os
import platform
import subprocess
from pathlib import Path
from typing import Any, Callable, Collection, Dict, List, Optional


#: The camera port aliases the bus rules create, one per connector.
CAMERA_BUS_GLOB = "/dev/i2c-cam*"
#: Where a host's camera bus rules install.
BUS_RULES_TARGET = "/etc/udev/rules.d/99-aliensense-i2c.rules"


def known_bus_rules() -> Dict[str, Path]:
    """The bus rules files the host package ships, by file name."""
    return {path.name: path for path in sorted(Path(__file__).parent.glob("*.rules"))}

class Host:
    """A Linux host with a V4L2 capture node per camera and no capture
    contract of its own. Methods raise NotImplementedError where the host
    lacks the capability; `describe()` says which host."""

    name = "generic"
    #: Whether this host boots a capture table to compare a mode against;
    #: without one there is nothing for the reboot rule to judge.
    capture_contract = False

    # --- the booted contract -------------------------------------------
    def installed_overlay(self, name: str) -> Optional[bytes]:
        """The compiled overlay of that name the boot entry carries; None
        when the host installs none or carries no such file."""
        del name
        return None

    def booted_lanes(self, bus: str) -> Optional[int]:
        """The CSI lane count the booted contract fixed for a port's bus,
        None when the host does not say."""
        del bus
        return None

    def capture_ids(self, bus: str) -> Dict[int, int]:
        """Virtual channel -> capture id for a port's bus ({} when the
        host does not say)."""
        del bus
        return {}

    def node_addrs(self, bus: str) -> Dict[int, int]:
        """Virtual channel -> the address of its capture node on a port's
        bus, where the host's per-frame controls for that channel are
        written ({} when the host does not say)."""
        del bus
        return {}

    def node_aliases(self, port: str) -> Dict[int, int]:
        """Virtual channel -> the host alias the port's overlay gives that
        channel's node behind a hub ({} when the host generates none)."""
        del port
        return {}

    def booted_modes(self, bus: str) -> List[Dict[str, Any]]:
        """The capture modes the booted contract offers on a port's bus:
        dicts with index, pool (sensor compatibles), width, height,
        bit_depth, lanes, vc, max_fps, and direct (the mode is a direct
        port's)."""
        del bus
        return []

    def mode_index(self, bus: str, compatible: str, width: int, height: int,
                   bit_depth: int, direct: Optional[bool] = None) -> Optional[int]:
        """The booted mode a sensor geometry lands on, None when the
        booted contract lacks it (the reboot rule's question). ``direct``
        asks for a direct port's mode (True) or a hub's (False)."""
        for mode in self.booted_modes(bus):
            if (compatible in mode["pool"] and mode["width"] == int(width)
                    and mode["height"] == int(height)
                    and mode["bit_depth"] == int(bit_depth)
                    and (direct is None or bool(mode.get("direct")) == bool(direct))):
                return int(mode["index"])
        return None

    def lane_mismatch(self, bus: str, declared: int) -> Optional[str]:
        """A sentence when the declared lane count disagrees with the
        booted one, else None."""
        del bus, declared
        return None

    # --- the camera ports -------------------------------------------------
    def camera_buses(self) -> Dict[str, str]:
        """Camera ports by name: the bus aliases, then what the host finds without one."""
        ports = {dev.rsplit("/i2c-", 1)[1]: dev for dev in glob.glob(CAMERA_BUS_GLOB)}
        for name, dev in self.unaliased_camera_buses().items():
            ports.setdefault(name, dev)
        return ports

    def unaliased_camera_buses(self) -> Dict[str, str]:
        """Camera ports found without an alias; {} here."""
        return {}

    def bus_rules(self) -> Optional[Path]:
        """The udev rules naming this host's camera buses; None without."""
        return None

    def connector_lanes(self, port: str) -> Optional[int]:
        """The CSI lane count the port's connector wires, the count a port
        takes where neither the declaration nor the booted tree names one;
        None when the host does not say."""
        del port
        return None

    def camera_bus_missing(self) -> bool:
        """Whether the booted tree carries no camera bus where this host boots
        them from its boot configuration (`install_camera_buses`); False here."""
        return False

    def camera_bus_package_missing(self) -> Optional[str]:
        """The fact line when the package the camera buses boot from is absent;
        None when it is there, or on a host that boots its buses by itself."""
        return None

    def install_camera_buses(self, fdt: Optional[str] = None, dry_run: bool = False) -> List[str]:
        """Make the boot configuration boot the camera buses, `fdt` naming the
        base tree; the lines to report, [] when it already does. The buses
        appear after the reboot; a RuntimeError names what the host can tell
        when that configuration booted and brought up none."""
        del fdt, dry_run
        raise NotImplementedError(f"{self.name}: no camera bus installer")

    # --- generating and installing the contract --------------------------
    def overlay(self, pack, port: str, lanes: int,
                sensors: Optional[List[str]] = None, direct: bool = False,
                node_addr: Optional[int] = None, fps: Optional[float] = None,
                bit_depth: Optional[int] = None) -> str:
        """The contract source for a port; ``direct`` for a port whose
        sensor is wired straight to the host, ``node_addr`` the address
        that sensor answers at when it is not the descriptor's own, ``fps``
        the rate the port runs at (the rows' default rate), ``bit_depth``
        the bit depth its declaration runs (None: the depth of the row
        with the longest exposure)."""
        raise NotImplementedError(f"{self.name}: no capture contract generator")

    def compile(self, dts: str, out: Path) -> Path:
        raise NotImplementedError(f"{self.name}: no contract compiler")

    def install_records(self, records: List[Dict[str, Any]], label: Optional[str],
                        select: bool, keep_other_ports: bool = True,
                        fdt: Optional[str] = None,
                        declared: Optional[Collection[str]] = None) -> List[str]:
        """Install compiled contract files under the boot configuration, `fdt` naming
        the base tree; the entry keeps the contract files of the ports in
        `declared` (None: of every port it names) and drops the others. One
        report line per file, per dropped file and one for the entry."""
        raise NotImplementedError(f"{self.name}: no contract installer")

    def boot_state(self) -> Dict[str, Any]:
        """What the boot configuration selects: `entry` (the boot entry's
        name) and `overlays` (the contract files it applies), {} when the
        host has none."""
        return {}

    def boot_entry_ports(self) -> List[str]:
        """The camera ports whose contract files the boot entry the installer
        writes names; [] when the host installs none."""
        return []

    def kernel_package_missing(self) -> Optional[str]:
        """The fact line when this host's camera kernel package is absent for
        the booted kernel; None on a host that needs none."""
        return None

    def kernel_package_next(self) -> str:
        """The command that installs the camera kernel package for this host."""
        return ""

    def platform_warnings(self) -> List[str]:
        """Sentences about this host the generated contract cannot report
        itself (a release the package was never built for, a carrier the
        contract does not name)."""
        return []

    # --- the capture stack's tuning -----------------------------------------
    def tuning_required(self) -> bool:
        """Whether the capture stack needs a tuning file built for the booted
        table before a head streams; False here."""
        return False

    def tuning_prerequisite_missing(self) -> Optional[str]:
        """The fact line when this host cannot build a tuning file until
        something is installed; None when it can, or takes none."""
        return None

    def tuning_prerequisite_next(self) -> str:
        """The step that installs what the tuning build needs."""
        return ""

    def tuning_install(self, path: Path, port: str) -> Path:
        """Install a head's tuning file for a port where this host's capture
        stack looks for it; returns the path written."""
        raise NotImplementedError(f"{self.name}: its capture stack takes no tuning file")

    def tuning_badge(self, port: str) -> str:
        """The module name a port's tuning file is made under: a file serves
        the badge it was made under and no other. The port's own name on a
        host whose capture stack names none."""
        return port

    def tuning_state(self, port: str) -> Optional[str]:
        """The tuning file the capture stack holds for a port, None when none."""
        del port
        return None

    def tuning_make(self, port: str, bus: str, sensor_id: int, run=None,
                    sleep=None, overrides: Optional[Dict[str, Path]] = None) -> Path:
        """Make a port's tuning file on this host for the booted capture
        table and install it, folding in `overrides` (a vendor override
        file per sensor compatible); returns the path written."""
        del bus, sensor_id, run, sleep, overrides
        raise NotImplementedError(f"{self.name}: its capture stack takes no tuning file")

    # --- the capture consumer (GStreamer) ----------------------------------
    def source(self, capture_id: int, mode_index: int) -> str:
        """The source element for one capture id in one booted mode."""
        del mode_index
        return f"v4l2src device=/dev/video{int(capture_id)}"

    def source_match(self, capture_id: int) -> str:
        """The prefix a process list shows for that source; the trailing
        space keeps id 1 from matching id 12."""
        return f"v4l2src device=/dev/video{int(capture_id)} "

    def viewer_match(self, capture_id: int) -> str:
        """The pkill/pgrep pattern for one capture id's viewer, plain or HUD."""
        return f"{self.source_match(capture_id)}|nxs[.]cam[.]hud --capture-id {int(capture_id)} "

    def viewers_pattern(self) -> str:
        """The pgrep pattern for any viewer of this host's sources."""
        return "nxs[.]cam[.]hud|gst-launch-1[.]0 .*v4l2src"

    def caps(self, width: int, height: int, framerate=None) -> str:
        """The caps a source is asked for."""
        out = f"video/x-raw,width={int(width)},height={int(height)}"
        if framerate:
            out += f",framerate={framerate[0]}/{framerate[1]}"
        return out

    def convert(self) -> str:
        """The elements that bring a source buffer into system memory as I420."""
        return "videoconvert ! video/x-raw,format=I420"

    def rgba_convert(self) -> str:
        """The elements that bring a source buffer into system memory as RGBA."""
        return "videoconvert ! video/x-raw,format=RGBA"

    def topic_convert(self, encoding: str) -> str:
        """The elements that bring a source buffer into system memory in a
        ROS image encoding (`yuv422`, `mono8`, `rgb8`) or through the JPEG
        encoder (`jpeg`), ending on a short leaky queue: the publisher pulls
        the newest frame and never holds the source."""
        tail = "queue max-size-buffers=2 leaky=downstream"
        if encoding == "jpeg":
            return f"videoconvert ! video/x-raw,format=I420 ! jpegenc ! {tail}"
        formats = {"yuv422": "UYVY", "mono8": "GRAY8", "rgb8": "RGB"}
        return f"videoconvert ! video/x-raw,format={formats[encoding]} ! {tail}"

    def exposure_props(self, exposure_us: float) -> str:
        """Source properties pinning the exposure at a port's value; empty
        where the source takes none."""
        del exposure_us
        return ""

    def ae_props(self, exposure_min_us, exposure_max_us: float) -> str:
        """Source properties bounding the capture stack's own exposure loop
        to the frame the sensor runs; empty where the source takes none."""
        del exposure_min_us, exposure_max_us
        return ""

    def locked_props(self, exposure_ns: int, gain: int) -> str:
        """Source properties locking the capture stack's adaptation at an
        exposure and a gain; empty where the source takes none."""
        del exposure_ns, gain
        return ""

    def pair_props(self, role: str, exposure_us: Optional[float],
                   gain_db: Optional[float] = None) -> str:
        """Source properties for a link of a synced pair, one exposure and one
        gain on both heads: `leader`, whose loop decides the pair's gain;
        `follower`, its loop locked at `gain_db` (the leader's gain as the
        session starts, unity without one) while its head takes the leader's
        gain; `locked`, the loop locked at the declared `gain_db`. The
        exposure is pinned at `exposure_us` where the port fixes one, and the
        ISP adds no digital gain, noise reduction or edge enhancement, each
        link keeping its own white balance. Empty where the source takes
        none."""
        del role, exposure_us, gain_db
        return ""

    def viewer_pipeline(self, capture_id: int, mode_index: int, props: str, caps: str,
                        crop_bottom: int, geometry: Dict[str, int]) -> str:
        """The plain viewer: the source into a window at `geometry`."""
        crop = f"{self.convert()} ! videocrop bottom={int(crop_bottom)} ! " if crop_bottom else ""
        return (f"{self.source(capture_id, mode_index)} {props} ! {caps} ! {crop}"
                f"queue ! videoconvert ! xvimagesink sync=false")

    def jpeg_encoder(self) -> str:
        return "jpegenc"

    def consumer_errors(self, output: str) -> int:
        """Capture-stack error lines in a consumer's output."""
        del output
        return 0

    def consumer_hint(self) -> str:
        """What a consumer that never built its pipeline usually means here."""
        return "a missing element means the GStreamer plugins are not installed on this host"

    def capture_daemon_unit(self) -> Optional[str]:
        """The capture daemon's systemd unit, which nxsd orders itself after; None without."""
        return None

    def restart_capture_daemon(self, run=subprocess) -> None:
        """Bounce the capture daemon between attempts; nothing to bounce here."""
        del run

    def setup(self, write: Callable[[str, str], bool]) -> List[str]:
        """Host-side setup beyond the udev rules (`write(path, body)` lands a
        file as root); the lines to report."""
        del write
        return []

    # --- identity ---------------------------------------------------------
    def keeps_system_declaration(self) -> bool:
        """Whether the host's one declaration is the system file nxsd reads,
        whoever runs the verb (a camera host); False here, where a user
        keeps a per-user one until the system file exists."""
        return False

    def stack(self) -> str:
        """The capture stack this host runs, as the ledger names it: the
        platform's release where it has one, else the kernel."""
        return f"linux-{platform.release()}"

    def describe(self) -> str:
        return self.name


#: The hosts this tool knows, each a callable returning its Host when the
#: process runs on one, else None; the first match wins.
DETECTORS: List[Callable[[], Optional[Host]]] = []


def _register_detectors() -> None:
    if DETECTORS:
        return
    from . import jetson
    DETECTORS.append(jetson.detect)


def detect() -> Host:
    """The host this process runs on: the first detector that claims it,
    else the generic host (V4L2 sources, nothing that writes /boot)."""
    _register_detectors()
    for detector in DETECTORS:
        host = detector()
        if host is not None:
            return host
    return Host()


_current: Optional[Host] = None


def current() -> Host:
    """The detected host (cached; tests replace it)."""
    global _current
    if _current is None:
        _current = detect()
    return _current


def set_current(host: Optional[Host]) -> None:
    """Replace the detected host (test seam)."""
    global _current
    _current = host


def env_flag(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in ("1", "true", "yes")
