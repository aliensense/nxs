# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0
"""What the booted device tree says about a camera port: the CSI lane count
(`num_lanes` in the sensor mode nodes) and the capture ids, read through
/sys/bus/i2c/devices/i2c-N/of_node and compared before a program runs."""
from __future__ import annotations

import os
import re
from typing import Optional

SYSFS_I2C = "/sys/bus/i2c/devices"


def bus_index(bus: str) -> Optional[int]:
    """`/dev/i2c-9` (or an alias symlink to it) -> 9."""
    try:
        real = os.path.realpath(bus)
    except OSError:
        return None
    name = os.path.basename(real)
    if not name.startswith("i2c-"):
        return None
    try:
        return int(name[4:])
    except ValueError:
        return None


def booted_lanes(bus: str, sysfs_i2c: str = SYSFS_I2C) -> Optional[int]:
    """The lane count the booted overlay declares for the sensor behind
    ``bus``, or None when the device tree does not say (no such bus, no
    sensor node, not a Jetson)."""
    index = bus_index(bus)
    if index is None:
        return None
    node = os.path.join(sysfs_i2c, f"i2c-{index}", "of_node")
    try:
        children = sorted(os.listdir(node))
    except OSError:
        return None
    for child in children:
        path = os.path.join(node, child, "mode0", "num_lanes")
        try:
            with open(path, "rb") as fh:
                return int(fh.read().rstrip(b"\0").decode() or 0)
        except (OSError, ValueError):
            continue
    return None


def lane_mismatch(bus: str, declared: int,
                  sysfs_i2c: str = SYSFS_I2C) -> Optional[str]:
    """One sentence when the booted overlay's lanes differ from the
    declared count, else None (also None when the tree is silent)."""
    booted = booted_lanes(bus, sysfs_i2c)
    if booted is None or booted == int(declared):
        return None
    return (f"the booted overlay declares {booted} CSI lanes for this port "
            f"but the port says csi_lanes: {declared} — boot the "
            f"{declared}-lane overlay for this camera, "
            f"or declare csi_lanes: {booted}")


DT_MODULES = "/proc/device-tree/tegra-camera-platform/modules"
DT_BASE = "/sys/firmware/devicetree/base"


def _read(path: str) -> Optional[str]:
    try:
        with open(path, "rb") as fh:
            return fh.read().rstrip(b"\0").decode()
    except (OSError, UnicodeDecodeError):
        return None


def _u32(path: str) -> Optional[int]:
    try:
        with open(path, "rb") as fh:
            raw = fh.read()
    except OSError:
        return None
    return int.from_bytes(raw[:4], "big") if len(raw) >= 4 else None


def capture_ids(bus: str, sysfs_i2c: str = SYSFS_I2C,
                modules: str = DT_MODULES,
                dt_base: str = DT_BASE) -> dict:
    """The capture ids the booted tree gives a port's links, {vc: id}: a channel's
    id is its sensor node's rank among the enabled `tegra-camera-platform`
    modules. Empty when the tree is silent."""
    index = bus_index(bus)
    if index is None:
        return {}
    node = os.path.join(sysfs_i2c, f"i2c-{index}", "of_node")
    try:
        real = os.path.realpath(node)
        children = sorted(os.listdir(node))
    except OSError:
        return {}
    tail = real.split("devicetree/base/", 1)[-1]
    nodes_by_vc = {}
    for child in children:
        vc = _u32(os.path.join(node, child, "ports", "port@0", "endpoint", "vc-id"))
        if vc is not None:
            nodes_by_vc[vc] = f"{tail}/{child}"
    if not nodes_by_vc:
        return {}
    try:
        names = sorted((n for n in os.listdir(modules)
                        if re.fullmatch(r"module\d+", n)),
                       key=lambda n: int(n[6:]))
    except OSError:
        return {}
    order = []
    for name in names:
        mdir = os.path.join(modules, name)
        if (_read(os.path.join(mdir, "status")) or "okay") != "okay":
            continue
        sysfs = _read(os.path.join(mdir, "drivernode0", "sysfs-device-tree")) or ""
        order.append(sysfs.split("devicetree/base/", 1)[-1])
    ids = {}
    for vc, path in nodes_by_vc.items():
        if path in order:
            ids[vc] = order.index(path)
    return ids


def node_addrs(bus: str, sysfs_i2c: str = SYSFS_I2C) -> dict:
    """The address of the capture node the booted tree gives each of a
    port's virtual channels, {vc: addr}: the node's `reg`, where the
    kernel's per-frame controls for that channel are written. Empty when
    the tree is silent."""
    index = bus_index(bus)
    if index is None:
        return {}
    node = os.path.join(sysfs_i2c, f"i2c-{index}", "of_node")
    try:
        children = sorted(os.listdir(node))
    except OSError:
        return {}
    addrs = {}
    for child in children:
        vc = _u32(os.path.join(node, child, "ports", "port@0", "endpoint", "vc-id"))
        addr = _u32(os.path.join(node, child, "reg"))
        if vc is not None and addr is not None:
            addrs[vc] = addr
    return addrs


def booted_modes(bus: str, sysfs_i2c: str = SYSFS_I2C) -> list:
    """The capture modes the booted tree offers on a port's bus, one dict per
    ``modeN`` (index, pool, width, height, bit_depth, lanes, vc, max_fps,
    default_fps, max_exp_us, and direct: the mode names no SerDes pixel
    clock, the receiver takes a sensor's own lanes). Empty when the tree is
    silent."""
    index = bus_index(bus)
    if index is None:
        return []
    node = os.path.join(sysfs_i2c, f"i2c-{index}", "of_node")
    try:
        children = sorted(os.listdir(node))
    except OSError:
        return []
    modes = []
    for child in children:
        cdir = os.path.join(node, child)
        try:
            names = [n for n in os.listdir(cdir) if re.fullmatch(r"mode\d+", n)]
        except OSError:
            continue
        for name in sorted(names, key=lambda n: int(n[4:])):
            mdir = os.path.join(cdir, name)

            def prop(key: str, default: str = "") -> str:
                value = _read(os.path.join(mdir, key))
                return value.strip() if value else default

            factor = int(prop("framerate_factor", "1") or 1) or 1
            modes.append({
                "index": int(name[4:]),
                "pool": [c for c in prop("aliensense_sensors_pool").split(";") if c],
                "width": int(prop("active_w", "0") or 0),
                "height": int(prop("active_h", "0") or 0),
                "bit_depth": int(prop("csi_pixel_bit_depth", "0") or 0),
                "lanes": int(prop("num_lanes", "0") or 0),
                "vc": int(prop("vc_id", "0") or 0),
                "max_fps": int(prop("max_framerate", "0") or 0) / factor,
                "default_fps": int(prop("default_framerate", "0") or 0) / factor,
                "max_exp_us": int(prop("max_exp_time", "0") or 0),
                "direct": not prop("serdes_pix_clk_hz"),
            })
    return modes
