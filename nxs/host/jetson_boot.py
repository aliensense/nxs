# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The extlinux boot configuration of a Jetson host as text: the labels, the
overlays and the FDT each carries, the vendor base tree blessed for the module,
and the camera package's companion overlays a port's label needs."""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path
from typing import Dict, List, Optional

#: Vendor trees end -nv or -nv-super; the suffix-less and kernel_ files beside them share the compatible.
_VENDOR_DTB = re.compile(r"^(?!kernel_).*-nv(-super)?\.dtb$")
#: How the camera package names a port's GMSL overlay, after the port name.
GMSL_OVERLAY_SUFFIX = "-gmsl-overlay.dtbo"


def label_overlays(text: str, label: str) -> List[str]:
    """The OVERLAYS a boot label carries, [] when the label is absent."""
    current = None
    for line in text.splitlines():
        if line.startswith("LABEL "):
            current = line.split(None, 1)[1].strip()
        elif current == label and line.strip().startswith("OVERLAYS"):
            rest = line.strip()[len("OVERLAYS"):].strip()
            return [o.strip() for o in rest.split(",") if o.strip()]
    return []


def label_fdt(text: str, label: Optional[str]) -> Optional[str]:
    """The FDT a boot label names, None when it names none or is absent."""
    current = None
    for line in text.splitlines():
        if line.startswith("LABEL "):
            current = line.split(None, 1)[1].strip()
        elif current == label and line.strip().startswith("FDT"):
            return line.strip()[len("FDT"):].strip() or None
    return None


def default_label(text: str) -> Optional[str]:
    for line in text.splitlines():
        if line.startswith("DEFAULT "):
            return line.split(None, 1)[1].strip()
    return None


def template_label(text: str, label: str) -> Optional[str]:
    """The label `with_label` copies the kernel lines from: the DEFAULT one,
    else the first in the file."""
    default = default_label(text)
    labels = [l.split(None, 1)[1].strip() for l in text.splitlines()
              if l.startswith("LABEL ")]
    if default in labels:
        return default
    return labels[0] if labels else None


def dtb_compatible(path: Path) -> Optional[str]:
    """The first root compatible a DTB file declares; None when it reads none."""
    fdtget = shutil.which("fdtget")
    if fdtget is None:
        raise RuntimeError(
            "the boot entry names no FDT and picking this module's base DTB needs "
            "fdtget, which is not installed — sudo apt install device-tree-compiler")
    result = subprocess.run([fdtget, "-t", "s", str(path), "/", "compatible"],
                            capture_output=True, text=True)
    words = result.stdout.split() if result.returncode == 0 else []
    return words[0] if words else None


def blessed_dtb(dtb_dir: Path, base_dtb_dir: Path, compatible: Optional[str]) -> Path:
    """This module's base DTB: the one kernel_*.dtb, else the vendor tree declaring
    the booted compatible; anything else refuses, never guesses."""
    found = sorted(Path(dtb_dir).glob("kernel_*.dtb"))
    if len(found) == 1:
        return found[0]
    if found:
        names = ", ".join(p.name for p in found)
        raise RuntimeError(
            f"the boot entry names no FDT and {dtb_dir} holds several DTBs ({names}): "
            f"name this module's with --fdt")
    vendor = sorted(p for p in Path(base_dtb_dir).glob("*.dtb") if _VENDOR_DTB.match(p.name))
    if not vendor:
        raise RuntimeError(
            f"the boot entry names no FDT, {dtb_dir} holds no kernel_*.dtb and "
            f"{base_dtb_dir} no vendor base DTB (*-nv.dtb, *-nv-super.dtb): overlays "
            f"apply only under an FDT line naming this module's DTB — name it with --fdt")
    names = ", ".join(p.name for p in vendor)
    if compatible is None:
        raise RuntimeError(
            f"the boot entry names no FDT and the booted tree's compatible is "
            f"unreadable, so none of {names} in {base_dtb_dir} can be picked: "
            f"name this module's with --fdt")
    declaring = [p for p in vendor if dtb_compatible(p) == compatible]
    if len(declaring) == 1:
        return declaring[0]
    if not declaring:
        raise RuntimeError(
            f"the boot entry names no FDT and no base DTB in {base_dtb_dir} declares "
            f"{compatible} ({names}): name this module's with --fdt")
    names = ", ".join(p.name for p in declaring)
    raise RuntimeError(
        f"the boot entry names no FDT and several base DTBs in {base_dtb_dir} declare "
        f"{compatible} ({names}): name one with --fdt")


def with_label(text: str, label: str, overlays: List[str], select: bool,
               fdt: Optional[str] = None) -> str:
    """extlinux.conf with a LABEL carrying these OVERLAYS: replaced when present,
    appended otherwise (LINUX/INITRD/FDT/APPEND copied from the DEFAULT label,
    `fdt` named instead of the copied line when given); DEFAULT moves only on
    request."""
    lines = text.splitlines()
    blocks: List[List[str]] = []
    head: List[str] = []
    current: Optional[List[str]] = None
    for line in lines:
        if line.startswith("LABEL "):
            current = [line]
            blocks.append(current)
        elif current is not None:
            current.append(line)
        else:
            head.append(line)
    default = default_label(text)
    if select:
        head = [f"DEFAULT {label}" if l.startswith("DEFAULT ") else l for l in head]
        if default is None:
            head.insert(0, f"DEFAULT {label}")
    template = next((b for b in blocks if b[0].split(None, 1)[1].strip() == default), None)
    if template is None and blocks:
        template = blocks[0]
    body: List[str] = [f"LABEL {label}",
                       f"      MENU LABEL {label} (nxs-generated camera overlays)"]
    copied = {}
    for line in (template or [])[1:]:
        key = line.strip().split(None, 1)[0] if line.strip() else ""
        if key in ("LINUX", "INITRD", "FDT", "APPEND"):
            copied[key] = line
    for key in ("LINUX", "INITRD", "FDT", "APPEND"):
        if key == "FDT" and fdt:
            body.append(f"      FDT {fdt}")
        elif key in copied:
            body.append(copied[key])
    body.append("      OVERLAYS " + ",".join(overlays))
    blocks = [b for b in blocks if b[0].split(None, 1)[1].strip() != label]
    blocks.append(body)
    out = list(head)
    if out and out[-1].strip():
        out.append("")
    for block in blocks:
        out.extend(l for l in block if l.strip() or l == "")
        if out and out[-1].strip():
            out.append("")
    return "\n".join(out).rstrip("\n") + "\n"


def companion_overlays(port: str, boot_dir: Path) -> Dict[str, str]:
    """The camera package's overlays a port's boot label needs beside the
    generated one: the carrier's mux overlay (first) and the port's GMSL overlay
    (after the port's own). An absent file is an absent key."""
    found: Dict[str, str] = {}
    if not boot_dir.is_dir():
        return found
    for path in sorted(boot_dir.glob("*.dtbo")):
        if path.name.endswith("general-mux-ch-overlay.dtbo"):
            found.setdefault("mux", str(path))
        elif path.name.endswith(f"{port}{GMSL_OVERLAY_SUFFIX}"):
            found.setdefault("gmsl", str(path))
    return found


def with_companions(order: List[str], port: str,
                    companions: Dict[str, str]) -> List[str]:
    """The label's overlays with the package's companions in place: the mux
    overlay first, the port's GMSL overlay after the port's last overlay. One
    the list already carries stays where it is."""
    have = {Path(o).name for o in order}
    mux = companions.get("mux")
    if mux and Path(mux).name not in have:
        order = [mux] + order
    gmsl = companions.get("gmsl")
    if gmsl and Path(gmsl).name not in have:
        mine = [i for i, o in enumerate(order) if f"-{port}-" in Path(o).name]
        at = (mine[-1] + 1) if mine else len(order)
        order = order[:at] + [gmsl] + order[at:]
    return order


def label_overlay_order(current: List[str], port: str, names: List[str], boot_dir: Path,
                        companions: Optional[Dict[str, str]] = None) -> List[str]:
    """The boot label's OVERLAYS with this port's overlays replaced in place by
    the newly compiled ones under ``boot_dir`` and everything else kept in order
    (a port's GMSL overlay must follow its universal ones). ``companions`` fills
    in the package's."""
    fresh = {n: str(boot_dir / n) for n in names}
    out: List[str] = []
    slot = None
    for entry in current:
        name = Path(entry).name
        if f"-{port}-" not in name:
            out.append(entry)
            continue
        if name in fresh:
            out.append(fresh.pop(name))
        slot = len(out)
    leftovers = list(fresh.values())
    if slot is None:
        out.extend(leftovers)
    else:
        out[slot:slot] = leftovers
    return with_companions(out, port, companions or {})
