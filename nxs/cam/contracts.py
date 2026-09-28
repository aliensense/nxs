# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Socket contracts between the sensor, serdes, and capture layers: the sensor
exports a MipiContract, the serdes layer turns MipiContracts plus a Topology into
a CsiContract, and the capture layer validates the CsiContract against the DT."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple


class ContractError(ValueError):
    """Base error for contract violations between layers."""


class InfeasibleConfig(ContractError):
    """A requested configuration cannot work on this hardware: the fact on
    one line, then one line per lawful alternative."""

    def __init__(self, reason: str, alternatives: Optional[list[str]] = None):
        self._reason = reason
        self._alternatives = list(alternatives or [])
        super().__init__("\n".join([reason, *(f"  - {alt}" for alt in self._alternatives)]))

    @property
    def reason(self) -> str:
        return self._reason

    @property
    def alternatives(self) -> list[str]:
        return list(self._alternatives)


@dataclass(frozen=True)
class MipiContract:
    """What a sensor emits on its MIPI output for a chosen mode; exported by the
    sensor descriptor, consumed by the serdes layer."""

    lanes: int
    rate_mbps: int
    data_type: str  # "RAW10" | "RAW12" | ...
    bit_depth: int
    width: int
    height: int
    fps: float
    trigger_input: Optional[str] = None  # e.g. "XTRIG1"
    embedded_lines: int = 0


@dataclass(frozen=True)
class VcGeometry:
    """One virtual channel on the shared CSI-2 output."""

    vc: int
    dt: str  # "RAW10" | "RAW12"
    width: int
    height: int
    bit_depth: int
    soft_bpp: Optional[int] = None  # e.g. 16 on the doubled path


@dataclass(frozen=True)
class CsiContract:
    """What the deserializer puts on the CSI-2 port; exported by the serdes
    layer, consumed by the capture layer. ``transport`` names the
    serializer's path: the pixel block (RAW10 doubled, RAW12 as is)."""

    port: int
    transport: str  # "pixel"
    virtual_channels: Tuple[VcGeometry, ...]
    #: The deserializer pipe each link's video rides (link name -> pipe); a solo
    #: rides the first pipe, a dual keeps one pipe per link. Empty when unset.
    pipes: Dict[str, str] = field(default_factory=dict)
    #: The free-run rate each link's sensor was programmed for (link name ->
    #: fps): the declared rate, or the rate of an overriding frame length.
    rates: Dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class RateRange:
    """The free-run rates a mode may run at in a port: the laws' floor and
    ceiling narrowed to the shipped point the mode ships (``shipped``), or the
    bare laws when it ships none; ``binds`` names the law that sets the ceiling."""

    floor: float
    ceiling: float
    shipped: bool
    binds: str

    #: Rates this close to an end (relative) are inside: a declared rate is
    #: a decimal, the ends are the law's floats.
    TOLERANCE = 1e-6

    def contains(self, fps: float) -> bool:
        slack = self.TOLERANCE * max(self.ceiling, 1.0)
        return self.floor - slack <= float(fps) <= self.ceiling + slack

    def text(self) -> str:
        """`20–60 fps` (whole rates without decimals)."""
        return f"{_rate_text(self.floor)}–{_rate_text(self.ceiling)} fps"


def _rate_text(fps: float) -> str:
    return f"{fps:.10g}" if float(fps).is_integer() else f"{fps:.2f}"


@dataclass(frozen=True)
class NxsUnitSpec:
    """A sensor pod wired in-line on a link. The pod answers a fixed address; the
    deserializer's I2C translation presents it to the host at a distinct alias.
    On a direct port nothing translates: the alias is the pod's own address."""

    alias_addr: int  # host-side address (after translation behind a hub)
    target_addr: int = 0x30  # the pod's own fixed address


@dataclass(frozen=True)
class LinkSpec:
    """One link of a port: a camera head behind a GMSL link or on a direct
    port wired straight to the host (no serializer, no deserializer
    window), or a pod alone on a hub link (`sensor_compatible` None: the
    link carries an NXS unit and no camera)."""

    name: str
    des_window: Optional[int]  # deserializer window value selecting this link
    csi_vc: int
    sensor_compatible: Optional[str]
    ser_compatible: Optional[str]
    ser_addr: int = 0x42
    # None: the sensor descriptor's own address (meta.i2c_addr); the IMX219
    # sits at 0x10, the Sony industrial parts at 0x1A.
    sensor_addr: Optional[int] = None
    tca_addr: int = 0x20
    capture_id: Optional[int] = None  # capture-stack sensor id for viewers
    nxs_units: Tuple[NxsUnitSpec, ...] = ()
    # Declared mode token for this link (a descriptor mode name or WxH);
    # None follows the port's declaration, then the sensor's default.
    mode: Optional[str] = None
    # The sensor came from the manifest (a link's or the port's `camera`):
    # the port's memory of a probe or an `on --sensor` never overrides it.
    sensor_declared: bool = False
    # The clock the pod feeds the sensor (Hz); None: the descriptor's
    # limits.inck_hz. The timing program derives the sensor's PLL selector
    # from it; a pod on the other clock reads out flat at half the rate.
    inck_hz: Optional[int] = None
    # The declared free-run rate for this link (a link's `camera.fps`);
    # None follows the port's declaration, then the mode's ceiling.
    fps: Optional[float] = None
    # The address the host reaches the sensor at: its capture node's, from
    # the booted tree, which the link's serializer maps to the sensor's
    # own. None: the sensor answers at its own address (no node, or a
    # direct port).
    host_addr: Optional[int] = None

    @property
    def has_camera(self) -> bool:
        """True for a link with a camera head; False for a pod alone."""
        return self.sensor_compatible is not None


@dataclass(frozen=True)
class SyncSpec:
    """Carrier-level sync: one generator, all links inherit."""

    source: str = "free_run"  # "free_run" | "des_fsync"
    fps: Optional[float] = None  # generator rate when source == des_fsync


@dataclass(frozen=True)
class Topology:
    """A port: one deserializer with one or two links and a shared sync, or a
    direct port, where the sensor's lanes and I2C reach the host with no
    SerDes between them (`des_compatible` None)."""

    carrier: str
    i2c_bus: str  # "/dev/i2c-9"
    links: Tuple[LinkSpec, ...]
    des_compatible: Optional[str]
    des_addr: int = 0x6A
    csi_lanes: int = 4  # deserializer CSI-TX output lane count; the
    # sensor->serializer MIPI side is a descriptor property.
    hub_driver: str = "nxs"  # "kernel" = a kernel driver owns the SerDes; nxs
    # then writes nothing, so link walks are skipped and get/set refuse.
    camera_mode: Optional[str] = None  # declared mode token (suite.yaml)
    camera_fps: Optional[float] = None
    camera_exposure_us: Optional[float] = None  # under frame sync, microseconds
    sync: SyncSpec = field(default_factory=SyncSpec)
    # The booted capture tree's node address per virtual channel, (vc, addr)
    # pairs: the address the kernel's per-frame controls for that channel
    # are written to. Empty when the tree is silent.
    node_addrs: Tuple[Tuple[int, int], ...] = ()

    @property
    def camera_links(self) -> Tuple[LinkSpec, ...]:
        """The links with a camera head, in the port's order."""
        return tuple(l for l in self.links if l.has_camera)

    @property
    def is_dual(self) -> bool:
        return len(self.camera_links) > 1

    def node_addr(self, vc: int) -> Optional[int]:
        """The booted capture node's address for a virtual channel; None
        when the tree does not say."""
        for known, addr in self.node_addrs:
            if int(known) == int(vc):
                return int(addr)
        return None

    @property
    def is_direct(self) -> bool:
        """True for a port with no deserializer: the sensor is wired to the host."""
        return self.des_compatible is None

    def link(self, name: str) -> LinkSpec:
        """Return the link named ``name``; raises ContractError when there is none."""
        for spec in self.links:
            if spec.name == name:
                return spec
        raise ContractError(f"No link named {name!r} in topology")


@dataclass(frozen=True)
class LinkTrainingResult:
    """Outcome of GMSL link training for one link."""

    link: str
    locked: bool
    rounds_used: int
    dips: int
