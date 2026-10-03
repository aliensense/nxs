# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The Jetson host on the p3768 carrier. The booted capture contract is the
device tree; this module reads it (through ``nxs.host.jetson_dt``), generates it from
the pack (``jetson_overlay``), compiles it with ``dtc``, installs it under an
extlinux boot label, and speaks to the Argus capture stack."""

from __future__ import annotations

import glob
import os
import platform
import re
import shutil
import stat
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any, Callable, Collection, Dict, List, Optional, Tuple

from . import Host
from . import capture_table as tables
from . import jetson_dt as dt
from . import jetson_overlay as gen
from .jetson_boot import (FDT_ALTERNATIVE, GMSL_OVERLAY_SUFFIX, LABEL_MARKER, blessed_dtb, carried_overlays,
                          companion_overlays, default_label, fdtoverlay_result, label_fdt, label_overlay_order,
                          label_overlays, mux_overlay, on_disk, overlay_port, template_label,
                          undeclared_overlays, with_label)
from .root import as_root

DT_BASE = "/proc/device-tree"
RELEASE_FILE = "/etc/nv_tegra_release"
#: The running kernel's command line, under the host's root.
CMDLINE = "proc/cmdline"
BOOT_DTBO_DIR = Path("/boot/camera-dtbos")
#: Where a JetPack 6 flash puts the one `kernel_*.dtb` blessed for this module SKU.
BOOT_DTB_DIR = Path("/boot/dtb")
#: Where a JetPack 7 image leaves the vendor base DTBs of every module SKU.
BASE_DTB_DIR = Path("/boot")
EXTLINUX = Path("/boot/extlinux/extlinux.conf")
#: `put` as root: the same sibling-temp-then-rename, in a shell run as root.
_PUT_AS_ROOT = (
    'd=$(dirname "$1") && mkdir -p "$d" && t=$(mktemp "$d/.nxs-put.XXXXXX") '
    '&& cat > "$t" && if [ -e "$1" ]; then chmod --reference="$1" "$t" '
    '&& chown --reference="$1" "$t"; else chmod 0644 "$t"; fi '
    '&& sync "$t" && mv -f "$t" "$1"'
)
#: The label the automatic install writes; the operator's own labels are
#: never touched.
GENERATED_LABEL = "aliensense_gen"

#: The L4T releases the camera kernel package is built and gate-tested for:
#: one `nxs-jetson_<l4t>_arm64.deb` per release, tagged `<l4t>` on the
#: release page of aliensense/nxs-jetson.
TESTED_L4T = ("36.4.4", "39.2.1")
KERNEL_PACKAGE_REPO = "aliensense/nxs-jetson"
#: What the package installs under /lib/modules/<kernel>/.
KERNEL_MODULES = ("universal_aliensense.ko", "aliensense_generic_des.ko")
KERNEL_MODULES_SUBDIR = "updates/drivers/media/i2c"
LIB_MODULES = "/lib/modules"


def kernel_package_name(l4t: Optional[str]) -> str:
    """The camera kernel package file for an L4T release (`<l4t>` when the
    release is unknown)."""
    return f"nxs-jetson_{l4t or '<l4t>'}_arm64.deb"


def kernel_modules_missing(kernel: str, lib_modules: str = LIB_MODULES) -> bool:
    """Whether the booted kernel lacks the camera package's modules."""
    d = Path(lib_modules, kernel, KERNEL_MODULES_SUBDIR)
    return not all((d / m).is_file() for m in KERNEL_MODULES)

#: The capture daemon has a configuration mode (the tuning build) from L4T 39
#: on; L4T 36's Argus has none and streams on its built-in tuning.
TUNING_L4T_MAJOR = 39
#: From L4T 39 on, the capture daemon's configuration mode (the tuning
#: build) needs NVIDIA's camera hotfix, which delivers these files; a
#: stock image refuses the mode.
HOTFIX_L4T_MAJOR = "39"
HOTFIX_FILES = ("usr/sbin/nvcfg2nito",
                "usr/lib/aarch64-linux-gnu/nvidia/libnvm_cam_tuning_l4t_cfg2nito.so",
                "var/nvidia/nvcam/settings/template.nito")
HOTFIX_STEP = "install NVIDIA's camera hotfix (NVIDIA Jetson deployment guide, step 3)"


def l4t_release(release_file: str = RELEASE_FILE) -> Optional[str]:
    """The booted L4T version from the release file, `36.4.4` shaped; None when
    the file is absent or shaped otherwise."""
    try:
        head = Path(release_file).read_text(errors="replace").splitlines()[0]
    except (OSError, IndexError):
        return None
    major = re.search(r"\bR(\d+)\b", head)
    revision = re.search(r"REVISION:\s*([0-9][0-9.]*)", head)
    if major is None or revision is None:
        return None
    return f"{major.group(1)}.{revision.group(1).rstrip('.')}"


def board_compatible(dt_base: str = DT_BASE) -> List[str]:
    """The booted tree's compatible strings, the NUL-separated property split."""
    try:
        raw = Path(dt_base, "compatible").read_bytes()
    except OSError:
        return []
    return [s.decode(errors="replace") for s in raw.split(b"\x00") if s]


#: Device-tree model prefixes of the carriers this host layer generates
#: overlays for; another Jetson reads its booted tree through the generic host.
SUPPORTED_MODELS = ("NVIDIA Jetson Orin Nano",)


def is_supported_jetson(dt_base: str = DT_BASE) -> bool:
    """True when the booted device-tree model names a supported carrier."""
    try:
        model = Path(dt_base, "model").read_bytes().rstrip(b"\x00").decode(errors="replace")
    except OSError:
        return False
    return any(model.startswith(prefix) for prefix in SUPPORTED_MODELS)


def _read(path: Path) -> Optional[str]:
    try:
        return path.read_bytes().rstrip(b"\x00").decode(errors="replace")
    except OSError:
        return None


def detect() -> Optional["JetsonHost"]:
    """This host when the booted tree names a supported carrier."""
    return JetsonHost() if is_supported_jetson(DT_BASE) else None


#: The udev rules naming this carrier's camera buses, beside this module.
BUS_RULES = "jetson-orin-nano.rules"
#: The tool restarts nvargus-daemon itself, so apport's crash report for it is noise.
APPORT_BLACKLIST = "/etc/apport/blacklist.d/nxs-nvargus-daemon"
APPORT_BLACKLIST_BODY = "/usr/sbin/nvargus-daemon\n"
#: The camera connectors are branches of one I2C mux, named by the adapter.
MUX_ADAPTER_NAMES = "/sys/class/i2c-adapter/i2c-*/name"
_MUX_BRANCH = re.compile(r"mux \(chan_id (\d+)\)")

#: The Argus daemon every capture session goes through.
CAPTURE_DAEMON_UNIT = "nvargus-daemon.service"
#: After a restart the daemon is polled for `active` this long, in these
#: steps, then settles this long before a session opens.
DAEMON_READY_S = 15.0
DAEMON_POLL_S = 0.5
DAEMON_SETTLE_S = 3.0
#: Where the capture stack looks for a module's tuning, by the badge the
#: generated overlay gives the module.
TUNING_DIR = Path("/var/nvidia/nvcam/settings")
#: The capture daemon's binary, run under a transient unit in configuration
#: mode while the tuning file is made; the environment variable that mode
#: reads; the characters of the badge the stack keeps as the module name.
CAPTURE_DAEMON_BIN = "/usr/sbin/nvargus-daemon"
TUNING_UNIT = "nxs-tuning"
#: The override the capture daemon folds into a configuration-mode session.
TUNING_OVERRIDE_NAME = "camera_overrides.isp"
TUNING_CONFIG_ENV = "NVCAMERA_NITO_PATH"
TUNING_BADGE_CHARS = 31
#: One session per mode writes its knob set at the session's start; the
#: session itself runs to this deadline when the head streams no such mode.
TUNING_DAEMON_SETTLE_S = 3.0
TUNING_SESSION_S = 15

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


class JetsonHost(Host):
    name = "jetson"
    capture_contract = True

    def __init__(self, sysfs_i2c: str = dt.SYSFS_I2C, modules: str = dt.DT_MODULES,
                 dt_base: str = DT_BASE, lib_modules: str = LIB_MODULES,
                 release_file: str = RELEASE_FILE, root: str = "/") -> None:
        self._sysfs_i2c = sysfs_i2c
        self._modules = modules
        self._dt_base = dt_base
        self._lib_modules = lib_modules
        self._release_file = release_file
        self._root = root

    # --- the capture stack's prerequisite ----------------------------------
    def tuning_required(self) -> bool:
        """From L4T 39 on; a release file the host cannot read takes the build."""
        release = l4t_release(self._release_file)
        return release is None or int(release.split(".")[0]) >= TUNING_L4T_MAJOR

    def tuning_prerequisite_missing(self) -> Optional[str]:
        release = l4t_release(self._release_file)
        if release is None or release.split(".")[0] != HOTFIX_L4T_MAJOR:
            return None
        for rel in HOTFIX_FILES:
            if not Path(self._root, rel).is_file():
                return (f"the capture daemon's configuration mode on L4T {release} needs "
                        f"NVIDIA's camera hotfix (/{rel} is absent)")
        return None

    def tuning_prerequisite_next(self) -> str:
        return HOTFIX_STEP

    # --- the camera kernel package ----------------------------------------
    def kernel_package_missing(self) -> Optional[str]:
        kernel = platform.release()
        if kernel_modules_missing(kernel, self._lib_modules):
            return f"no camera kernel package for {kernel}"
        return None

    def kernel_package_next(self) -> str:
        return f"sudo apt install ./{kernel_package_name(l4t_release(RELEASE_FILE))}"

    # --- the camera ports -------------------------------------------------
    def unaliased_camera_buses(self) -> Dict[str, str]:
        ports: Dict[str, str] = {}
        for name_file in glob.glob(MUX_ADAPTER_NAMES):
            try:
                name = Path(name_file).read_text()
            except OSError:
                continue
            match = _MUX_BRANCH.search(name)
            if match:
                ports[f"cam{match.group(1)}"] = f"/dev/{name_file.rsplit('/', 2)[-2]}"
        return ports

    def bus_rules(self) -> Optional[Path]:
        return Path(__file__).with_name(BUS_RULES)

    def connector_lanes(self, port: str) -> Optional[int]:
        return gen.PORTS[port]["lanes"] if port in gen.PORTS else None

    def camera_bus_missing(self) -> bool:
        return not self.camera_buses()

    def camera_bus_package_missing(self) -> Optional[str]:
        if mux_overlay(BOOT_DTBO_DIR) is None:
            return f"no camera kernel package for {platform.release()}"
        return None

    def install_camera_buses(self, fdt: Optional[str] = None, dry_run: bool = False) -> List[str]:
        """The generated label with the camera package's mux overlay first and
        the label's other overlays kept, made the DEFAULT; [] when the DEFAULT
        label already boots it, its base DTB (`fdt`'s, when given) and every
        overlay on disk. A stock label names no overlay, and the connectors'
        buses exist only under the mux. A RuntimeError when that label booted
        (the command line carries its marker) and brought up no bus: another
        reboot would boot the same."""
        mux = mux_overlay(BOOT_DTBO_DIR)
        if mux is None:
            raise RuntimeError(f"{self.camera_bus_package_missing()}\n  - {self.kernel_package_next()}")
        text = EXTLINUX.read_text() if EXTLINUX.exists() else ""
        default = default_label(text)
        current = carried_overlays(text, GENERATED_LABEL)
        kept = [entry for entry in current if Path(entry).name != Path(mux).name]
        base = label_fdt(text, GENERATED_LABEL)
        # The launcher applies overlays only under an FDT line it can read: a
        # label without one boots no bus, whatever it lists.
        if (default == GENERATED_LABEL and len(kept) < len(current)
                and on_disk(base) is not None and fdt in (None, base)
                and all(Path(entry).is_file() for entry in current)):
            if self._booted_label() == GENERATED_LABEL and self.camera_bus_missing():
                raise RuntimeError(self._overlays_dropped(base, mux))
            return []
        named, carried = self._boot_fdt(text, GENERATED_LABEL, fdt)
        if dry_run:
            return [f"would install the camera bus mux under {GENERATED_LABEL} (FDT {named or carried})"]
        reports = self.install_many([], GENERATED_LABEL, select=True, overlays=[mux] + kept, fdt=fdt)
        return ([f"camera bus mux installed under {GENERATED_LABEL} (FDT {named or carried})"]
                + [line for line in reports if line.startswith("dropped ")])

    def _booted_label(self) -> Optional[str]:
        """The label the running kernel booted, by the marker its APPEND
        carries; None when the command line names none."""
        try:
            args = Path(self._root, CMDLINE).read_text(errors="replace").split()
        except OSError:
            return None
        named = [arg.split("=", 1)[1] for arg in args if arg.startswith(f"{LABEL_MARKER}=")]
        return named[-1] if named else None

    def _overlays_dropped(self, base: str, mux: str) -> str:
        """The generated label booted with no bus: the fact, what fdtoverlay
        says of the mux on the label's base DTB, and the flag that names
        another base DTB (this module's pick, where it is another file)."""
        booted = board_compatible(self._dt_base)
        try:
            pick: Optional[str] = str(blessed_dtb(BOOT_DTB_DIR, BASE_DTB_DIR, booted[0] if booted else None))
        except RuntimeError:
            pick = None
        alternative = FDT_ALTERNATIVE if pick in (None, base) else FDT_ALTERNATIVE.replace("<dtb>", pick)
        said = fdtoverlay_result(base, mux)
        return "\n".join([f"the boot entry {GENERATED_LABEL} booted without its overlays"]
                         + ([said] if said else []) + [f"  - {alternative}"])

    # --- the booted tree ----------------------------------------------------
    def installed_overlay(self, name: str) -> Optional[bytes]:
        path = BOOT_DTBO_DIR / name
        return path.read_bytes() if path.exists() else None

    def booted_lanes(self, bus: str) -> Optional[int]:
        return dt.booted_lanes(bus, sysfs_i2c=self._sysfs_i2c)

    def capture_ids(self, bus: str) -> Dict[int, int]:
        return dt.capture_ids(bus, sysfs_i2c=self._sysfs_i2c, modules=self._modules)

    def node_addrs(self, bus: str) -> Dict[int, int]:
        return dt.node_addrs(bus, sysfs_i2c=self._sysfs_i2c)

    def node_aliases(self, port: str) -> Dict[int, int]:
        from . import jetson_overlay as gen
        return gen.node_aliases(port) if port in gen.PORTS else {}

    def booted_modes(self, bus: str) -> List[Dict[str, Any]]:
        return dt.booted_modes(bus, sysfs_i2c=self._sysfs_i2c)

    def lane_mismatch(self, bus: str, declared: int) -> Optional[str]:
        return dt.lane_mismatch(bus, declared, sysfs_i2c=self._sysfs_i2c)

    # --- the generated contract ------------------------------------------
    def overlay(self, pack, port: str, lanes: int,
                sensors: Optional[List[str]] = None, direct: bool = False,
                node_addr: Optional[int] = None, fps: Optional[float] = None,
                bit_depth: Optional[int] = None) -> str:
        # A port without a hub tops its rows out at the sensor's own ceiling.
        layout = {"lanes": None if direct else int(lanes), "default_fps": fps,
                  "bit_depth": bit_depth}
        table = tables.capture_table(pack, sensors, **layout)
        return gen.overlay_dts(port, int(lanes), table, tables.pool_sensors(pack, sensors, **layout),
                               direct=direct, node_addr=node_addr)

    def compile(self, dts: str, out: Path) -> Path:
        dtc = shutil.which("dtc")
        if dtc is None:
            raise RuntimeError(
                "dtc is not installed — sudo apt install device-tree-compiler")
        out = Path(out)
        out.parent.mkdir(parents=True, exist_ok=True)
        result = subprocess.run([dtc, "-@", "-q", "-I", "dts", "-O", "dtb", "-o", str(out)],
                                input=dts, capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"dtc failed: {result.stderr.strip()}")
        return out

    def install_many(self, dtbos: List[Path], label: str = GENERATED_LABEL,
                     select: bool = False, boot_dir: Optional[Path] = None,
                     extlinux: Optional[Path] = None,
                     overlays: Optional[List[str]] = None,
                     dtb_dir: Optional[Path] = None,
                     fdt: Optional[str] = None,
                     base_dtb_dir: Optional[Path] = None) -> List[str]:
        """Settle the label's base DTB, copy every overlay into the boot directory,
        then write the boot label once; the first install keeps the original
        extlinux.conf beside it, and a later one leaves that copy alone. Returns
        one line per file, one per carried overlay the label drops and one for
        the label. `fdt` names the base DTB instead of the search. ``boot_dir``
        / ``extlinux`` / ``dtb_dir`` / ``base_dtb_dir`` are test seams."""
        boot_dir = Path(boot_dir or BOOT_DTBO_DIR)
        extlinux = Path(extlinux or EXTLINUX)
        text = extlinux.read_text() if extlinux.exists() else ""
        backup = extlinux.with_suffix(extlinux.suffix + ".bak-nxs")
        fdt, carried = self._boot_fdt(text, label, fdt, dtb_dir, base_dtb_dir)
        targets = [boot_dir / Path(dtbo).name for dtbo in dtbos]
        # The launcher applies OVERLAYS as one: a file it cannot read and it
        # boots none of them, so a carried entry that is not on disk goes.
        mine = {str(target) for target in targets}
        names: List[str] = []
        dropped: List[str] = []
        for entry in overlays or []:
            if entry in mine or Path(entry).is_file():
                names.append(entry)
            else:
                dropped.append(entry)
        # The caller's order is meaning (a port's GMSL overlay follows the
        # universal ones): keep a target where the list has it, else append.
        for target in targets:
            if str(target) not in names:
                names.append(str(target))
        for dtbo, target in zip(dtbos, targets):
            self.put(target, Path(dtbo).read_bytes())
        kept = backup.exists()
        if text and not kept:
            self.put(backup, text.encode())
        self.put(extlinux, with_label(text, label, names, select, fdt).encode())
        lines = [f"installed {target}" for target in targets]
        lines += [f"dropped {entry} from {label}: no such file (a label naming a missing "
                  f"overlay boots none)" for entry in dropped]
        if fdt is not None:
            lines.append(f"FDT {fdt} named in {label}: the boot entry it copies names "
                         + (carried if carried is not None else
                            "no device tree, and overlays apply only under one"))
        lines.append(f"boot label {label}" + (" (DEFAULT)" if select else "")
                     + " written; reboot to apply: sudo reboot"
                     + (f"; original extlinux.conf at {backup}" if text or kept else ""))
        return lines

    def _boot_fdt(self, text: str, label: str, fdt: Optional[str] = None,
                  dtb_dir: Optional[Path] = None,
                  base_dtb_dir: Optional[Path] = None) -> Tuple[Optional[str], Optional[str]]:
        """(the FDT the label is written with, None to copy its template's;
        the FDT that template names). Raises RuntimeError when `fdt` names no
        file or no base DTB can be picked."""
        dtbs, base = Path(dtb_dir or BOOT_DTB_DIR), Path(base_dtb_dir or BASE_DTB_DIR)
        if fdt is not None and not Path(fdt).is_file():
            raise RuntimeError(f"--fdt {fdt}: no such file, and the boot entry names only a base DTB "
                               f"on disk\n  - ls {dtbs} {base}/*.dtb")
        # OVERLAYS apply only under an FDT the launcher can read: the label keeps its own or
        # its template's while that file is on disk, else it names this module's DTB.
        template = template_label(text, label)
        own = on_disk(label_fdt(text, label))
        carried = own if template == label else on_disk(label_fdt(text, template))
        if fdt is None:
            fdt = own
        if fdt is None and carried is None:
            booted = board_compatible(self._dt_base)
            fdt = str(blessed_dtb(dtbs, base, booted[0] if booted else None))
        return fdt, carried

    def install(self, dtbo: Path, label: str = GENERATED_LABEL,
                select: bool = False, boot_dir: Optional[Path] = None,
                extlinux: Optional[Path] = None,
                overlays: Optional[List[str]] = None) -> str:
        """One overlay: `install_many` for a single file, reported as one line."""
        return "; ".join(self.install_many([dtbo], label, select, boot_dir,
                                           extlinux, overlays))

    def put(self, path: Path, data: bytes) -> None:
        """Write a file atomically (a sibling temp file, fsynced, renamed over the
        target with its mode kept); a path root owns is written the same way
        through sudo, which asks a terminal for the password, and a sudo that
        refuses is a PermissionError naming its answer."""
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._replace(path, data)
            return
        except PermissionError:
            pass
        proc = subprocess.run(as_root(["sh", "-c", _PUT_AS_ROOT, "nxs-put", str(path)]),
                              input=data, capture_output=True, check=False)
        if proc.returncode != 0:
            reason = proc.stderr.decode(errors="replace").strip() or "sudo refused"
            raise PermissionError(f"{path}: {reason}")

    @staticmethod
    def _replace(path: Path, data: bytes) -> None:
        fd, tmp = tempfile.mkstemp(prefix=".nxs-put.", dir=str(path.parent))
        try:
            with os.fdopen(fd, "wb") as fh:
                fh.write(data)
                fh.flush()
                os.fsync(fh.fileno())
            if path.exists():
                os.chmod(tmp, stat.S_IMODE(path.stat().st_mode))
            os.replace(tmp, path)
        except BaseException:
            Path(tmp).unlink(missing_ok=True)
            raise

    # --- the boot entry ---------------------------------------------------
    def boot_state(self) -> Dict[str, Any]:
        try:
            text = EXTLINUX.read_text() if EXTLINUX.exists() else ""
        except OSError:
            return {}
        entry = default_label(text)
        return {"entry": entry, "overlays": label_overlays(text, entry) if entry else []}

    def boot_entry_ports(self) -> List[str]:
        try:
            text = EXTLINUX.read_text() if EXTLINUX.exists() else ""
        except OSError:
            return []
        return sorted({port for port in map(overlay_port, carried_overlays(text, GENERATED_LABEL))
                       if port is not None})

    def install_records(self, records: List[Dict[str, Any]], label: Optional[str],
                        select: bool, keep_other_ports: bool = True,
                        fdt: Optional[str] = None,
                        declared: Optional[Collection[str]] = None) -> List[str]:
        """Install compiled overlays under a boot label: the other declared
        ports' overlays of the current DEFAULT label ride along, an undeclared
        port's are dropped with a line each, this port's are replaced, and the
        package's mux and GMSL overlays take their places. ``declared`` None
        declares every port the label names."""
        for record in records:
            if not record["dtbo"]:
                raise RuntimeError("nothing compiled to install — "
                                   "sudo apt install device-tree-compiler")
        text = EXTLINUX.read_text() if EXTLINUX.exists() else ""
        target_label = label or GENERATED_LABEL
        current = carried_overlays(text, target_label) if keep_other_ports else []
        port = records[0]["port"]
        companions = companion_overlays(port, BOOT_DTBO_DIR)
        if records[0].get("direct"):
            # The receiver takes the sensor's own lanes: the port's GMSL
            # overlay describes a deserializer node and goes.
            companions.pop("gmsl", None)
            current = [o for o in current
                       if not Path(o).name.endswith(f"{port}{GMSL_OVERLAY_SUFFIX}")]
        dropped = undeclared_overlays(current, port, declared)
        carried = label_overlay_order(current, port,
                                      [Path(r["dtbo"]).name for r in records],
                                      BOOT_DTBO_DIR, companions, declared)
        # Every file first, the label once: one backup of the original
        # extlinux.conf, and the label never names a missing file.
        return ([f"dropped {entry} from {target_label}: port {owner} is not declared"
                 for entry, owner in dropped.items()]
                + self.install_many([Path(r["dtbo"]) for r in records], target_label,
                                    select=select, overlays=carried, fdt=fdt))

    def platform_warnings(self) -> List[str]:
        """An L4T release the modules were never built for, and a carrier the
        overlay's compatible list does not name."""
        warnings = []
        release = l4t_release(RELEASE_FILE)
        if release is not None and release not in TESTED_L4T:
            warnings.append(f"L4T {release}; the overlays and modules are gate-tested "
                            f"against {', '.join(TESTED_L4T)}")
        booted = board_compatible(self._dt_base)
        if booted and not set(booted) & set(gen.COMPATIBLE_P3768):
            warnings.append(f"this board is {booted[0]}, which the overlay's compatible list "
                            f"does not name: the launcher applies it to none of "
                            f"{', '.join(gen.COMPATIBLE_P3768[:2])}, … and boots without cameras")
        return warnings

    # --- the capture stack's tuning -----------------------------------------
    def tuning_install(self, path: Path, port: str) -> Path:
        badge = self._badge(port)
        target = TUNING_DIR / f"{badge}{Path(path).suffix.lower()}"
        self.put(target, Path(path).read_bytes())
        return target

    def tuning_badge(self, port: str) -> str:
        return self._badge(port)

    def tuning_state(self, port: str) -> Optional[str]:
        try:
            badge = self._badge(port)
        except RuntimeError:
            return None
        found = sorted(p for p in TUNING_DIR.glob(f"{badge}.*") if p.is_file()) \
            if TUNING_DIR.is_dir() else []
        return str(found[0]) if found else None

    def tuning_make(self, port: str, bus: str, sensor_id: int, run=subprocess,
                    sleep: Callable[[float], None] = time.sleep,
                    overrides: Optional[Dict[str, Path]] = None) -> Path:
        """Make the port's tuning file on the device and install it: the capture
        daemon runs in configuration mode under a transient unit, one session
        per booted mode writes that mode's knob set into the file the stack
        derives from the module's badge, and the file lands under the badge.
        Before each session the vendor override of the mode's sensor (from
        `overrides`, by compatible) is placed where the daemon folds it in,
        or removed when the sensor names none. The knob set is written when
        the session starts, so no mode has to stream. Raises RuntimeError
        when a mode got no knob set, and PermissionError when sudo refuses."""
        badge = self._badge(port)
        # The tree lists a port's table once per capture node: one session
        # per mode index serves the module, whichever node names it.
        modes = {int(m["index"]): m for m in self.booted_modes(bus)}
        if not modes:
            raise RuntimeError(f"{port}: the booted tree carries no capture mode, "
                               f"nothing to tune")
        produced = Path("/root") / f"{badge[:TUNING_BADGE_CHARS]}.nito"
        self._sudo(run, ["systemctl", "stop", CAPTURE_DAEMON_UNIT])
        self._sudo(run, ["rm", "-f", str(produced)])
        override = TUNING_DIR / TUNING_OVERRIDE_NAME
        for n in sorted(modes):
            mode = modes[n]
            unit = f"{TUNING_UNIT}-{n}"
            vendor = (overrides or {}).get((mode.get("pool") or [None])[0])
            if vendor is not None:
                self.put(override, Path(vendor).read_bytes())
            else:
                self._sudo(run, ["rm", "-f", str(override)], check=False)
            self._sudo(run, ["systemd-run", "--quiet", f"--unit={unit}",
                             "--working-directory=/root", "--setenv=HOME=/root",
                             f"--setenv={TUNING_CONFIG_ENV}=CONFIG", CAPTURE_DAEMON_BIN])
            sleep(TUNING_DAEMON_SETTLE_S)
            # The consumer opens a session on any mode the table carries; the
            # knob set is written as the session starts, frames or not. The
            # session asks for the row's default rate, the one the port runs:
            # a rate the mode cannot do is refused at the caps and no session
            # starts, and a row's top rate is the lanes', not the sensor's.
            rate = max(1, int(float(mode.get("default_fps") or mode.get("max_fps") or 30)))
            self._sudo(run, ["timeout", str(TUNING_SESSION_S), "gst-launch-1.0",
                             "nvarguscamerasrc", f"sensor-id={int(sensor_id)}",
                             f"sensor-mode={n}", "num-buffers=2", "!",
                             f"video/x-raw(memory:NVMM),width={int(mode['width'])},"
                             f"height={int(mode['height'])},framerate={rate}/1",
                             "!", "fakesink"], check=False)
            self._sudo(run, ["systemctl", "stop", unit], check=False)
            self._sudo(run, ["systemctl", "reset-failed", unit], check=False)
        # The running daemon reads no override: the built object carries the fold.
        self._sudo(run, ["rm", "-f", str(override)], check=False)
        journal = self._sudo(run, ["journalctl", "--no-pager", "-u", f"{TUNING_UNIT}-*"],
                             check=False)
        written = set(int(m) for m in re.findall(rb"written to\s+Knobset (\d+)",
                                                  journal.stdout or b""))
        missing = sorted(n for n in modes if n not in written)
        # A daemon a transient unit left behind would hold the socket, and
        # the service's own start would find the server "already operational":
        # a survivor of the stops is ended through its unit before the service.
        survivor = self._sudo(run, ["pgrep", "-x", "nvargus-daemon"], check=False)
        if survivor.returncode == 0:
            for unit in [f"{TUNING_UNIT}-{n}" for n in sorted(modes)] + [CAPTURE_DAEMON_UNIT]:
                self._sudo(run, ["systemctl", "kill", "--signal=SIGKILL", unit], check=False)
            sleep(TUNING_DAEMON_SETTLE_S)
        self._sudo(run, ["systemctl", "reset-failed", CAPTURE_DAEMON_UNIT], check=False)
        self._sudo(run, ["systemctl", "start", CAPTURE_DAEMON_UNIT], check=False)
        if missing:
            raise RuntimeError(f"{port}: no knob set was written for mode "
                               f"{', '.join(str(n) for n in missing)} of {len(modes)}; the "
                               f"capture daemon's journal for {TUNING_UNIT}-* says why")
        data = self._sudo(run, ["cat", str(produced)]).stdout
        target = TUNING_DIR / f"{badge}.nito"
        self.put(target, data)
        self._sudo(run, ["systemctl", "restart", CAPTURE_DAEMON_UNIT], check=False)
        return target

    @staticmethod
    def _sudo(run, argv: List[str], check: bool = True):
        """Run a command as root (`root.as_root`: sudo asks a terminal for the
        password); a sudo that refuses is a PermissionError, any other failure
        a RuntimeError when checked. The output stays bytes."""
        proc = run.run(as_root(argv), capture_output=True)
        if proc.returncode != 0:
            err = (proc.stderr or b"").decode(errors="replace").strip()
            if "password" in err:
                raise PermissionError(f"{' '.join(argv[:2])}: {err}")
            if check:
                raise RuntimeError(f"{' '.join(argv[:2])}: {err or f'exit {proc.returncode}'}")
        return proc

    @staticmethod
    def _badge(port: str) -> str:
        try:
            return str(gen.PORTS[port]["badge"])
        except KeyError:
            raise RuntimeError(f"no camera port {port!r} on this carrier "
                               f"(have {', '.join(sorted(gen.PORTS))})") from None

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
        # The converter reaches system memory as RGBA only from an RGBA
        # device buffer; straight from the source's buffer it hands out
        # nothing on this release.
        return ("nvvidconv ! video/x-raw(memory:NVMM),format=RGBA ! "
                "nvvidconv ! video/x-raw,format=RGBA")

    def topic_convert(self, encoding: str) -> str:
        # The hardware converter writes UYVY and GRAY8 into system memory
        # by itself, at the link's rate; RGB needs the software converter
        # after the RGBA path, which costs a core at 1080p.
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

    def capture_daemon_unit(self) -> Optional[str]:
        return CAPTURE_DAEMON_UNIT

    def restart_capture_daemon(self, run=subprocess, sleep=time.sleep) -> None:
        """Bounce nvargus-daemon and wait for it to be back: a wedged daemon
        takes its stop timeout to die, so the wait is for `active`, then a
        settle for its camera providers. When sudo refuses, say what it
        answered and skip the wait."""
        result = run.run(as_root(["systemctl", "restart", "nvargus-daemon"]), capture_output=True)
        if result.returncode != 0:
            why = (result.stderr or b"").decode(errors="replace").strip() or f"exit {result.returncode}"
            print(f"cannot restart nvargus-daemon: {why}\n  - sudo systemctl restart nvargus-daemon")
            return
        waited = 0.0
        while waited < DAEMON_READY_S:
            if run.run(["systemctl", "is-active", "--quiet", "nvargus-daemon"],
                       capture_output=True).returncode == 0:
                break
            sleep(DAEMON_POLL_S)
            waited += DAEMON_POLL_S
        sleep(DAEMON_SETTLE_S)

    def setup(self, write: Callable[[str, str], bool]) -> List[str]:
        blacklist = APPORT_BLACKLIST
        if not os.path.isdir(os.path.dirname(blacklist)):
            return []
        if not write(blacklist, APPORT_BLACKLIST_BODY):
            raise RuntimeError(f"could not write {blacklist}")
        return [f"installed {blacklist} (apport stays quiet about nvargus-daemon)"]

    # --- identity ---------------------------------------------------------
    def keeps_system_declaration(self) -> bool:
        return True

    def stack(self) -> str:
        release = l4t_release(RELEASE_FILE)
        return f"l4t-{release}" if release else super().stack()

    def describe(self) -> str:
        model = (_read(Path(self._dt_base, "model")) or "Jetson").strip()
        return f"jetson ({model})"
