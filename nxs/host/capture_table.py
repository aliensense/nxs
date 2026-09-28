# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The host capture table, generated from the sensor descriptors and free of
any platform: one row per entry of every sensor's ``capture.table`` in pack
order, each row's line length and top rate from the mode the unit program
runs, and per sensor the bus identity and the control rows a generic kernel
driver interprets. A platform's contract generator lays this out in its own
device-tree shape."""

from __future__ import annotations

import dataclasses
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from nxs.cam.descriptors import to_int

#: The register map a node serves when its descriptors are silent.
REG_BITS_DEFAULT = 16
VAL_BITS_DEFAULT = 8
#: A control row carries four parameters after address, width, order, form.
CONTROL_PARAMS = 4
#: The sensor address a descriptor without one answers at.
SENSOR_ADDR_DEFAULT = 0x1A


#: Lines kept between the longest exposure a consumer may ask for and the
#: frame: the host driver's shutter minimum, with room to spare.
EXPOSURE_MARGIN_LINES = 32


@dataclasses.dataclass(frozen=True)
class CaptureMode:
    """One capture-side mode: a sensor's table entry with its host facts."""

    pool: str            # the sensor compatible ("vendor,sensor")
    width: int
    height: int
    bit_depth: int
    pixel_phase: str
    line_length: int
    max_fps: float
    min_exp_us: int
    mclk_khz: int
    pix_clk_hz: int
    gain: Dict[str, int]
    hdr_ratio: Dict[str, int]
    exposure: Dict[str, int]
    framerate: Dict[str, Any]
    default_fps: Optional[float] = None   # the rate a consumer without caps gets
    lane_polarity: Optional[int] = None   # the sensor's own declaration
    #: Embedded-data lines the VI sees in this row's stream: the pixel path
    #: parks the sensor's own line on the hub. The VI faults every frame
    #: that carries one it did not expect.
    embedded_lines: int = 0
    #: The sensor gates its clock lane between bursts, and the receiver of
    #: its lanes is configured for it (the kernel's `clock-noncontinuous`).
    clock_noncontinuous: bool = False

    @property
    def key(self) -> tuple:
        return (self.pool, self.width, self.height, self.bit_depth)

    def exposure_ceiling_us(self) -> int:
        """The row's longest exposure: the frame at the row's rate less the
        shutter margin, or the sensor's own limit when that is lower. An
        exposure past the frame makes the driver stretch the frame length
        the unit programmed, and the port's framing breaks."""
        rate = self.default_fps if self.default_fps is not None else self.max_fps
        line_us = self.line_length * 1e6 / self.pix_clk_hz
        frame_lines = int(self.pix_clk_hz / (self.line_length * float(rate)))
        return min(int(self.exposure["max_us"]),
                   int((frame_lines - EXPOSURE_MARGIN_LINES) * line_us))


def _rows(pack, desc) -> List[CaptureMode]:
    """A descriptor's capture rows as the host's table carries them."""
    from nxs.cam.capture_facts import derived_rows

    cap = desc.raw("capture")
    if not cap:
        return []
    return [CaptureMode(
        pool=desc.compatible,
        width=int(entry["width"]), height=int(entry["height"]),
        bit_depth=int(entry["bit_depth"]),
        pixel_phase=str(entry["pixel_phase"]),
        line_length=int(entry["line_length"]),
        max_fps=float(entry["max_fps"]),
        default_fps=(float(entry["default_fps"])
                     if entry.get("default_fps") is not None else None),
        min_exp_us=int(entry["min_exp_us"]),
        mclk_khz=int(cap["mclk_khz"]), pix_clk_hz=int(cap["pix_clk_hz"]),
        gain=dict(cap["gain"]), hdr_ratio=dict(cap["hdr_ratio"]),
        exposure=dict(cap["exposure"]), framerate=dict(cap["framerate"]),
        lane_polarity=(int(cap["lane_polarity"])
                       if cap.get("lane_polarity") is not None else None),
        embedded_lines=int(entry.get("embedded_lines", 0) or 0),
        clock_noncontinuous=bool(cap.get("clock_noncontinuous")),
    ) for entry in derived_rows(pack, desc)]


def capture_table(pack, sensors: Optional[Iterable[str]] = None) -> List[CaptureMode]:
    """The host capture table: every entry of every sensor's
    ``capture.table`` that shares the port's Bayer phase, sensors by their
    compatible (or in the order given), each row's line length and top
    rate derived from the mode the unit program runs
    (`nxs.cam.capture_facts`), the pack's own sensors' rows in the order
    of their exposure ceilings, an extension's after them. The capture
    stack demosaics every mode of a node by the phase of the node's first
    row, so the table carries one phase: the phase of the row with the
    longest exposure, the row the exposure rule ends the table on
    (`excluded_rows` names the rest). The order is a function of the
    sensors alone, so every tool and host that holds the same sensors
    derives the same table, whatever order a pack or a store lists them."""
    own, extensions = _ordered_rows(pack, sensors)
    phase = table_phase(own, extensions)
    return [m for m in own + extensions if m.pixel_phase == phase]


def excluded_rows(pack, sensors: Optional[Iterable[str]] = None) -> List[CaptureMode]:
    """The sensors' rows the port's table leaves out: those of another
    Bayer phase than the table's."""
    own, extensions = _ordered_rows(pack, sensors)
    phase = table_phase(own, extensions)
    return [m for m in own + extensions if m.pixel_phase != phase]


def table_phase(own: Sequence[CaptureMode],
                extensions: Sequence[CaptureMode] = ()) -> Optional[str]:
    """The Bayer phase a table carries: its last own row's, the row with
    the longest exposure; an extension's only when no own row exists."""
    rows = own or extensions
    return rows[-1].pixel_phase if rows else None


def _ordered_rows(pack, sensors: Optional[Iterable[str]] = None
                  ) -> Tuple[List[CaptureMode], List[CaptureMode]]:
    """Every sensor's rows in the table's order, the pack's own and the
    extensions', before the phase rule."""
    chips = (list(sensors) if sensors is not None
             else sorted(pack.sensors(), key=lambda c: pack.descriptor(c).compatible))
    own: List[Tuple[int, CaptureMode]] = []
    extensions: List[CaptureMode] = []
    for chip in chips:
        desc = pack.descriptor(chip)
        rows = _rows(pack, desc)
        if getattr(pack, "is_extension", lambda _name: False)(desc.name):
            extensions += rows
            continue
        # An experimental overlay moves a row's ceiling; the row keeps the
        # place the shipped table gives it, so the indexes agree with a
        # host booted without the flag.
        as_shipped = ({m.key: m.exposure_ceiling_us()
                       for m in _rows(pack, pack.shipped_descriptor(desc.name))}
                      if desc.name in getattr(pack, "overlays", {}) else {})
        own += [(as_shipped.get(m.key, m.exposure_ceiling_us()), m) for m in rows]
    # The capture stack's source validates a consumer's exposure range
    # against the node's last row, whatever mode the consumer names, and
    # hands a mode given no range that row's: the table ends on the row
    # with the longest exposure, so every row's own range passes. The
    # pack's own sensors' rows come first, so the table a host boots is a
    # prefix of the one the experimental flag pools (a session names a
    # mode by its index).
    return ([m for _, m in sorted(own, key=lambda t: (t[0], t[1].key))],
            sorted(extensions, key=lambda m: (m.exposure_ceiling_us(), m.key)))


@dataclasses.dataclass(frozen=True)
class PoolSensor:
    """One sensor a node serves: its bus identity and its kernel controls."""

    compatible: str
    i2c_addr: int
    reg_bits: int
    val_bits: int
    inck_hz: Optional[int]
    #: (control, [addr, width, order, form, p0, p1, p2, p3]) per control.
    controls: Tuple[Tuple[str, Tuple[int, ...]], ...]


def control_table(descriptor) -> List[Tuple[str, List[int]]]:
    """The kernel control rows of a sensor: for each control its register's
    address, width in bytes, byte order (0 little-endian, 1 big-endian), the
    formula form, and four parameters, from the descriptor's registers and
    its family's law parameters. A control the sensor lacks is absent."""
    from nxs.cam import chips

    rows: List[Tuple[str, List[int]]] = []
    for name, reg, form, params in chips.control_rows(descriptor):
        spec = descriptor.registers[reg]
        order = 1 if str(spec.get("order", "le")) == "be" else 0
        padded = list(params) + [0] * (CONTROL_PARAMS - len(params))
        rows.append((name, [to_int(spec["addr"]), int(spec.get("width", 1)),
                            order, int(form), *padded]))
    return rows


def pool_sensors(pack, sensors: Optional[Iterable[str]] = None) -> List[PoolSensor]:
    """The sensors of a node's pool, in the table's order: every sensor
    that contributes a row, with its identity and control rows. The order
    follows the table, never the order the sensors are named in, so the
    overlay two callers generate is one and the same."""
    chips = list(sensors) if sensors is not None else list(pack.sensors())
    by_compatible = {pack.descriptor(chip).compatible: chip for chip in chips}
    ordered: List[str] = []
    for mode in capture_table(pack, sensors):
        if mode.pool in by_compatible and mode.pool not in ordered:
            ordered.append(mode.pool)
    pool: List[PoolSensor] = []
    for compatible in ordered:
        desc = pack.descriptor(by_compatible[compatible])
        meta = desc.raw("meta") or {}
        inck = desc.limits.get("inck_hz")
        pool.append(PoolSensor(
            compatible=desc.compatible,
            i2c_addr=to_int(meta.get("i2c_addr", SENSOR_ADDR_DEFAULT)),
            reg_bits=int(meta.get("reg_bits", REG_BITS_DEFAULT)),
            val_bits=int(meta.get("val_bits", VAL_BITS_DEFAULT)),
            inck_hz=int(inck) if inck is not None else None,
            controls=tuple((name, tuple(row)) for name, row in control_table(desc)),
        ))
    return pool


def table_index(table: Sequence[CaptureMode], pool: str, width: int,
                height: int, bit_depth: int) -> Optional[int]:
    """The ``modeN`` index a geometry lands on in a table, if present."""
    for i, mode in enumerate(table):
        if mode.key == (pool, int(width), int(height), int(bit_depth)):
            return i
    return None


def mode_index(pack, descriptor, mode: str,
               sensors: Optional[Iterable[str]] = None) -> Optional[int]:
    """The ``modeN`` index a descriptor's program mode lands on in the pack's
    capture table, the index a capture session names it by; None for a
    mode the table carries no row for. ``sensors`` is the table's sensor
    set when it is not the whole pack's (a direct port boots its one
    sensor's rows alone)."""
    geo = descriptor.modes[mode]["geometry"]
    return table_index(capture_table(pack, sensors), descriptor.compatible, int(geo["width"]),
                       int(geo["height"]), int(geo["bit_depth"]))
