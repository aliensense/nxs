# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The host capture table, generated from the sensor descriptors and free of
any platform: one row per entry of every sensor's ``capture.table`` of the
port's bit depth and Bayer phase, each row's line length from the mode the
unit program runs and its rates the port's (the lane law's ceiling on its
lanes, the rate it runs at by default), and per sensor the bus identity and
the control rows a generic kernel driver interprets. A platform's contract
generator lays this out in its own device-tree shape."""

from __future__ import annotations

import dataclasses
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from nxs.cam.contracts import FPS_DEFAULT, InfeasibleConfig
from nxs.cam.descriptors import mode_label, mode_token, resolve_mode, to_int

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
    #: Embedded-data lines the sensor emits with each frame. A connector
    #: port's node declares them, a hub's declares none since the hub parks
    #: them; the VI faults every frame that carries one it did not expect.
    embedded_lines: int = 0
    #: The sensor gates its clock lane between bursts, and the receiver of
    #: its lanes is configured for it (the kernel's `clock-noncontinuous`).
    clock_noncontinuous: bool = False
    #: The mode's link rate per lane in Mbps, from its geometry; None when
    #: the descriptor states none.
    rate_mbps: Optional[int] = None

    def csi_pix_clk_hz(self, lanes: int) -> Optional[int]:
        """The pixel clock of the CSI stream itself: the lanes' bits per
        second over the bits per pixel. A receiver that sees the sensor's
        lanes times them with it; None without the link rate."""
        if self.rate_mbps is None:
            return None
        return int(round(self.rate_mbps * 1_000_000 * int(lanes) / self.bit_depth))

    @property
    def key(self) -> tuple:
        return (self.pool, self.width, self.height, self.bit_depth)

    def exposure_ceiling_us(self, rate: Optional[float] = None) -> int:
        """The row's longest exposure: the frame at `rate` (the row's
        default rate when None) less the shutter margin, or the sensor's
        own limit when that is lower. An exposure past the frame makes the
        driver stretch the frame length the unit programmed, and the port's
        framing breaks."""
        if rate is None:
            rate = self.default_fps if self.default_fps is not None else self.max_fps
        line_us = self.line_length * 1e6 / self.pix_clk_hz
        frame_lines = int(self.pix_clk_hz / (self.line_length * float(rate)))
        return min(int(self.exposure["max_us"]),
                   int((frame_lines - EXPOSURE_MARGIN_LINES) * line_us))


def _rows(hub, desc, lanes: Optional[int] = None,
          default_fps: Optional[float] = None) -> List[CaptureMode]:
    """A descriptor's capture rows as the host's table carries them, on a
    port of `lanes` CSI lanes that runs at `default_fps` (`derived_rows`)."""
    from nxs.cam.capture_facts import derived_rows

    cap = desc.raw("capture")
    if not cap:
        return []
    rates = {}
    for mode in (desc.raw("modes") or {}).values():
        geo = mode.get("geometry") or {}
        if geo.get("rate_mbps") is not None:
            rates[(int(geo["width"]), int(geo["height"]), int(geo["bit_depth"]))] = int(geo["rate_mbps"])
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
        rate_mbps=rates.get((int(entry["width"]), int(entry["height"]), int(entry["bit_depth"]))),
    ) for entry in derived_rows(hub, desc, lanes, default_fps)]


def capture_table(hub, sensors: Optional[Iterable[str]] = None,
                  lanes: Optional[int] = None,
                  default_fps: Optional[float] = None,
                  bit_depth: Optional[int] = None) -> List[CaptureMode]:
    """The host capture table: every entry of every sensor's
    ``capture.table`` of the port's bit depth and Bayer phase, sensors by
    their compatible (or in the order given), each row's line length derived
    from the mode the unit program runs and its rates from the port's
    `lanes` and `default_fps` (`nxs.cam.capture_facts`), the hub's own
    sensors' rows in the order of their exposure ceilings, an extension's
    after them. The capture stack takes every mode of a node in the bit
    depth and the Bayer phase of the node's first row, so the table
    carries one of each (`table_format`): the depth `bit_depth` names (the
    port's declaration, `table_layout`), else the depth of the row with the
    longest exposure, the row the exposure rule ends the table on, and
    within that depth the phase of its row with the longest exposure
    (`excluded_rows` names the rest). The order is a function of the
    sensors and the port's layout alone, so every tool and host that holds
    the same sensors derives the same table for a port, whatever order a
    hub or a store lists them."""
    own, extensions = _ordered_rows(hub, sensors, lanes, default_fps)
    carried = table_format(own, extensions, bit_depth)
    return [m for m in own + extensions if (m.bit_depth, m.pixel_phase) == carried]


def table_layout(hub, topology, lanes: Optional[int] = None) -> Dict[str, Any]:
    """The `lanes`, `default_fps` and `bit_depth` a port's table is laid
    out at: the port's CSI lanes (`lanes` where the caller fixes others),
    none on a port without a hub; the rate the port runs at
    (`Topology.synced_fps`; FPS_DEFAULT without a port); and the bit depth
    its declaration runs (None without a port)."""
    direct = topology is not None and topology.is_direct
    if lanes is None and topology is not None:
        lanes = int(topology.csi_lanes)
    rates = {"lanes": None if direct else lanes,
             "default_fps": topology.synced_fps if topology is not None else FPS_DEFAULT}
    return {**rates, "bit_depth": (_declared_depth(hub, topology, **rates)
                                   if topology is not None else None)}


def port_sensors(hub, topology) -> Optional[List[str]]:
    """The sensors a port's table carries: a port without a hub boots its
    own sensor's rows alone, a hub's port every sensor the hub serves."""
    if not topology.is_direct:
        return None
    return [hub.descriptor(link.sensor_compatible).name for link in topology.camera_links]


def port_table(hub, topology) -> List[CaptureMode]:
    """The capture table a port boots: its sensors' rows at its lanes, the
    rate it runs at and the bit depth its declaration runs
    (`table_layout`). Every caller that names a mode by its index takes
    the table from here, so the indexes agree."""
    return capture_table(hub, port_sensors(hub, topology), **table_layout(hub, topology))


def port_index(hub, topology, descriptor, mode: str) -> Optional[int]:
    """The ``modeN`` index a descriptor's program mode lands on in the
    table the port boots (`port_table`); None for a mode it carries no row
    for."""
    geo = descriptor.modes[mode]["geometry"]
    return table_index(port_table(hub, topology), descriptor.compatible, int(geo["width"]),
                       int(geo["height"]), int(geo["bit_depth"]))


def refuse_without_row(hub, topology, link, descriptor, mode: str,
                       port: str, command: str) -> None:
    """Refuse `mode` where the table the port boots (`port_table`) carries
    no row for it: the table carries one pixel format (`table_format`),
    the depth the declaration runs and the phase of that depth's row with
    the longest exposure, so a row of another depth or phase boots on no
    node. A descriptor without capture rows has nothing to judge.

    Raises:
        InfeasibleConfig: Naming the port's link and the mode, the table's
            format, the declaration that would carry the mode where its
            depth differs, and `command` with each mode the table carries.
    """
    if not descriptor.raw("capture") or port_index(hub, topology, descriptor, mode) is not None:
        return
    depth, phase = table_format(port_table(hub, topology))
    geo = descriptor.modes[mode]["geometry"]
    alternatives = []
    if int(geo["bit_depth"]) != depth:
        carried = f"RAW{depth}"
        alternatives.append(f"ports.{port}.links.{link.name}.camera.mode: {mode_token(descriptor, mode)} "
                            f"in suite.yaml (every link of the port), then nxs switch and the reboot")
    else:
        carried = f"RAW{depth} {phase}"
    alternatives += [f"{command} --mode {mode_token(descriptor, name)}"
                     for name in descriptor.program_modes()
                     if port_index(hub, topology, descriptor, name) is not None]
    raise InfeasibleConfig(
        f"{port}/{link.name}: {mode_label(descriptor, mode)} boots on no row of this port's "
        f"table (the table carries one pixel format, {carried}, chosen by the declaration)",
        alternatives=alternatives)


def excluded_rows(hub, sensors: Optional[Iterable[str]] = None,
                  lanes: Optional[int] = None,
                  default_fps: Optional[float] = None,
                  bit_depth: Optional[int] = None) -> List[CaptureMode]:
    """The sensors' rows the port's table leaves out: those of another bit
    depth or another Bayer phase than the table's."""
    own, extensions = _ordered_rows(hub, sensors, lanes, default_fps)
    carried = table_format(own, extensions, bit_depth)
    return [m for m in own + extensions if (m.bit_depth, m.pixel_phase) != carried]


def table_format(own: Sequence[CaptureMode],
                 extensions: Sequence[CaptureMode] = (),
                 bit_depth: Optional[int] = None) -> Tuple[Optional[int], Optional[str]]:
    """The bit depth and the Bayer phase a table carries: the depth
    `bit_depth` names, else its last own row's, the row with the longest
    exposure; and the phase of the last own row of that depth. An
    extension's rows count only where no own row does."""
    rows = own or extensions
    depth = bit_depth if bit_depth is not None else (rows[-1].bit_depth if rows else None)
    of_depth = ([m for m in own if m.bit_depth == depth]
                or [m for m in extensions if m.bit_depth == depth])
    return depth, (of_depth[-1].pixel_phase if of_depth else None)


def _declared_depth(hub, topology, lanes: Optional[int] = None,
                    default_fps: Optional[float] = None) -> Optional[int]:
    """The bit depth a port's declaration runs: the depth of the modes its
    camera links run, each its declared mode (its own, else the port's) or
    the highest mode the port's laws admit, as `on` resolves them, in the
    table at `lanes` and `default_fps`. Where the links run two depths the
    row with the longest exposure sets it, and `check` names the other
    link's mode. None where no link runs a mode with a row."""
    from nxs.cam.run import _resolve_modes

    cams = list(topology.camera_links)
    if not cams:
        return None
    declared = {link.name: resolve_mode(hub.descriptor(link.sensor_compatible),
                                        str(link.mode or topology.camera_mode))
                for link in cams if link.mode or topology.camera_mode}
    modes = _resolve_modes(hub.flows(), hub, cams, declared, topology)
    runs = set()
    for link in cams:
        if link.name in modes:
            sen = hub.descriptor(link.sensor_compatible)
            geo = sen.modes[modes[link.name]]["geometry"]
            runs.add((sen.compatible, int(geo["width"]), int(geo["height"]), int(geo["bit_depth"])))
    own, extensions = _ordered_rows(hub, port_sensors(hub, topology), lanes, default_fps)
    rows = [m for m in own if m.key in runs] or [m for m in extensions if m.key in runs]
    return rows[-1].bit_depth if rows else None


def _ordered_rows(hub, sensors: Optional[Iterable[str]] = None,
                  lanes: Optional[int] = None, default_fps: Optional[float] = None
                  ) -> Tuple[List[CaptureMode], List[CaptureMode]]:
    """Every sensor's rows in the table's order, the hub's own and the
    extensions', before the table keeps one bit depth and one phase."""
    chips = (list(sensors) if sensors is not None
             else sorted(hub.sensors(), key=lambda c: hub.descriptor(c).compatible))
    own: List[Tuple[int, CaptureMode]] = []
    extensions: List[CaptureMode] = []
    for chip in chips:
        desc = hub.descriptor(chip)
        rows = _rows(hub, desc, lanes, default_fps)
        if getattr(hub, "is_extension", lambda _name: False)(desc.name):
            extensions += rows
            continue
        # An experimental overlay moves a row's ceiling; the row keeps the
        # place the product's table gives it, so the indexes agree with a
        # host booted without the flag.
        as_product = ({m.key: m.exposure_ceiling_us()
                       for m in _rows(hub, hub.shipped_descriptor(desc.name), lanes, default_fps)}
                      if desc.name in getattr(hub, "overlays", {}) else {})
        own += [(as_product.get(m.key, m.exposure_ceiling_us()), m) for m in rows]
    # The capture stack's source validates a consumer's exposure range
    # against the node's last row, whatever mode the consumer names, and
    # hands a mode given no range that row's: the table ends on the row
    # with the longest exposure, so every row's own range passes. The
    # hub's own sensors' rows come first, so the table a host boots is a
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


def pool_sensors(hub, sensors: Optional[Iterable[str]] = None,
                 lanes: Optional[int] = None,
                 default_fps: Optional[float] = None,
                 bit_depth: Optional[int] = None) -> List[PoolSensor]:
    """The sensors of a node's pool, in the order of the table at `lanes`,
    `default_fps` and `bit_depth`: every sensor that contributes a row,
    with its identity and control rows. The order follows the table, never
    the order the sensors are named in, so the overlay two callers
    generate is one and the same."""
    chips = list(sensors) if sensors is not None else list(hub.sensors())
    by_compatible = {hub.descriptor(chip).compatible: chip for chip in chips}
    ordered: List[str] = []
    for mode in capture_table(hub, sensors, lanes, default_fps, bit_depth):
        if mode.pool in by_compatible and mode.pool not in ordered:
            ordered.append(mode.pool)
    pool: List[PoolSensor] = []
    for compatible in ordered:
        desc = hub.descriptor(by_compatible[compatible])
        meta = desc.raw("meta") or {}
        # The clock the driver's frame arithmetic counts a line in: the
        # family's INCK where a frame law states one, else the pixel clock
        # of a table-only part, whose line length counts pixel clocks.
        inck = desc.limits.get("inck_hz")
        if inck is None:
            inck = (desc.raw("capture") or {}).get("pix_clk_hz")
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


def mode_index(hub, descriptor, mode: str,
               sensors: Optional[Iterable[str]] = None,
               lanes: Optional[int] = None,
               default_fps: Optional[float] = None) -> Optional[int]:
    """The ``modeN`` index a descriptor's program mode lands on in the hub's
    capture table at `lanes` and `default_fps`, the index a capture session
    names it by; None for a mode the table carries no row for. ``sensors``
    is the table's sensor set when it is not the whole hub's (a direct
    port boots its one sensor's rows alone)."""
    geo = descriptor.modes[mode]["geometry"]
    return table_index(capture_table(hub, sensors, lanes, default_fps), descriptor.compatible,
                       int(geo["width"]), int(geo["height"]), int(geo["bit_depth"]))
