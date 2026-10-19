# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Socket contracts between the sensor, serdes, and capture layers: the sensor
exports a MipiContract, the serdes layer turns MipiContracts plus a Topology into
a CsiContract, and the capture layer validates the CsiContract against the DT."""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Dict, Optional, Tuple


class ContractError(ValueError):
    """Base error for contract violations between layers."""


class NoLock(str):
    """What a port's construction found on a link where no serializer locked
    at any rate the hub listens at, in the hub's words: nothing answers on
    the link."""


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
    serializer's path: the pixel block (RAW10 and RAW12, both doubled)."""

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
    ceiling; ``binds`` names the law that sets the ceiling."""

    floor: float
    ceiling: float
    binds: str

    #: Rates this close to an end (relative) are inside: a declared rate is
    #: a decimal, the ends are the law's floats.
    TOLERANCE = 1e-6

    def contains(self, fps: float) -> bool:
        slack = self.TOLERANCE * max(self.ceiling, 1.0)
        return self.floor - slack <= float(fps) <= self.ceiling + slack

    def shown(self) -> Tuple[float, float]:
        """The ends to the hundredth and inside the range, the floor rounded
        up and the ceiling down, so a rate copied from them is one the range
        takes."""
        return shown_rate(self.floor, up=True), shown_rate(self.ceiling)

    def text(self) -> str:
        """`20–60 fps` (whole rates without decimals), the ends as `shown`."""
        return "–".join(_rate_text(end) for end in self.shown()) + " fps"

    def whole(self) -> Tuple[int, int]:
        """The lowest and the highest whole rate inside the range, the
        rates a refusal names."""
        slack = self.TOLERANCE * max(self.ceiling, 1.0)
        return math.ceil(self.floor - slack), math.floor(self.ceiling + slack)


#: The decimals a rate in hundredths is rounded to before `shown_rate`
#: rounds it to a whole hundredth: a rate that lands on one stays on it.
SHOWN_DIGITS = 6


def shown_rate(fps: float, up: bool = False) -> float:
    """A rate to the hundredth as a person reads it, rounded down (up with
    `up`)."""
    scaled = round(float(fps) * 100, SHOWN_DIGITS)
    return (math.ceil(scaled) if up else math.floor(scaled)) / 100


def _rate_text(fps: float) -> str:
    return f"{fps:.10g}" if float(fps).is_integer() else f"{fps:.2f}"


@dataclass(frozen=True)
class NxsUnitSpec:
    """A sensor pod wired in-line on a link. The pod answers two fixed
    addresses, its register map and its camera map; the serializer's I2C
    translation presents each to the host at a distinct alias, the camera map's
    one above the map's. On a direct port nothing translates: the aliases
    are the pod's own addresses."""

    alias_addr: int  # host-side address of the register map (after translation behind a hub)
    target_addr: int = 0x30  # the pod's own fixed address
    # Host-side address of the camera map (`map_target` behind it): one above the map's alias
    # behind a hub, the camera map's own where nothing translates. Left out, it follows the map's alias.
    camera_map_addr: Optional[int] = None
    map_target: int = 0x31  # the pod's own camera-map address

    def __post_init__(self) -> None:
        if self.camera_map_addr is None:
            direct = int(self.alias_addr) == int(self.target_addr)
            object.__setattr__(self, "camera_map_addr",
                               int(self.map_target) if direct else int(self.alias_addr) + 1)


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
    camera_node: Optional[str] = None  # the camera node that declares the head
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
    # The link's capture node address in the booted tree, where its per-frame controls go: a pod's
    # camera map, else the sensor. None: no node, or a direct port, where nothing translates.
    host_addr: Optional[int] = None

    @property
    def has_camera(self) -> bool:
        """True for a link with a camera head; False for a pod alone."""
        return self.sensor_compatible is not None


#: The rate a port runs at when its declaration names none: a frame sync's
#: generator, a free-running link (inside its lawful range), and the
#: default of the port's capture table rows.
FPS_DEFAULT = 30.0


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
    hub_node: Optional[str] = None  # the hub node's name
    camera_exposure_us: Optional[float] = None  # declared (us): refused where the pulse or the loop sets it
    camera_gain_db: Optional[float] = None  # declared (dB): the camera links locked at it
    sync: SyncSpec = field(default_factory=SyncSpec)
    # The booted capture tree's node address per virtual channel, (vc, addr)
    # pairs: the address the kernel's per-frame controls for that channel
    # are written to. Empty when the tree is silent.
    node_addrs: Tuple[Tuple[int, int], ...] = ()
    # The hub's outputs the declaration states, (port, lanes, streams,
    # copy) by port name: a port B feeding a module. Empty for port A alone.
    outputs: Tuple[Tuple[str, int, Tuple[str, ...], bool], ...] = ()
    # The line the delivery check found a pair's head needs on this rig,
    # (link, mode, HMAX in the head's clocks): the pair line law gives way
    # to it for that mode. Empty on a port at the datasheet's line.
    lines: Tuple[Tuple[str, str, int], ...] = ()
    # The hub node's (row, served value) pairs, its declared values over the rows' defaults;
    # empty for a port no declaration builds.
    hub_values: Tuple[Tuple[str, int], ...] = ()
    # The declaration the port is built from, one digest of every field that
    # shapes it (`schema_nodes.port_signature`); None for a port no manifest
    # declares, and for one `on` brings up by hand away from its declaration.
    declared: Optional[str] = None

    @property
    def name(self) -> str:
        """The port's short name, the carrier's suffix: `pixelmate/cam1` is `cam1`."""
        return self.carrier.split("/")[-1].lower()

    def hub_param_path(self, key: str) -> str:
        """Where the port's hub-wide parameter `key` is declared: the hub
        node's parameters, on a connector its camera's, else the port's
        view for a port built from no graph."""
        source = self.hub_node or next((l.camera_node for l in self.links if l.camera_node), None)
        return f"nodes.{source}.parameters.{key}" if source else f"ports.{self.name}.{key}"

    def link_param_path(self, letter: str, key: str) -> str:
        """Where a link's camera parameter `key` is declared: its camera
        node's parameters, else the port's view."""
        link = next((l for l in self.links if l.name == letter), None)
        if link is not None and link.camera_node:
            return f"nodes.{link.camera_node}.parameters.{key}"
        return f"ports.{self.name}.links.{letter}.camera.{key}"

    @property
    def camera_links(self) -> Tuple[LinkSpec, ...]:
        """The links with a camera head, in the port's order."""
        return tuple(l for l in self.links if l.has_camera)

    @property
    def is_dual(self) -> bool:
        return len(self.camera_links) > 1

    @property
    def cameras(self) -> int:
        """The camera count the port's links make: 1 for one camera link
        (or none), 2 for the pair; a link with a pod and no camera counts
        for none."""
        return 2 if self.is_dual else 1

    def with_modes(self, modes: Dict[str, str]) -> "Topology":
        """The port with each link `modes` names declared in its mode (link
        name -> mode name or token), the others as they are: the laws a
        pair's head is judged by read its partner's mode from the port."""
        if not modes:
            return self
        return replace(self, links=tuple(
            replace(l, mode=modes[l.name]) if l.name in modes else l for l in self.links))

    def with_lines(self, lines: Dict[str, Tuple[str, int]]) -> "Topology":
        """The port with each named link running its mode at the line found
        for it on the rig (link name -> (mode, HMAX)); the others at the
        line the laws give."""
        return replace(self, lines=tuple((name, str(mode), int(hmax))
                                         for name, (mode, hmax) in sorted(lines.items())))

    @property
    def synced_fps(self) -> float:
        """The rate the port runs under a declared frame sync: `sync.fps`,
        else the one rate its links declare, else FPS_DEFAULT."""
        if self.sync.fps is not None:
            return float(self.sync.fps)
        rates = {float(l.fps) for l in self.camera_links if l.fps is not None}
        return rates.pop() if len(rates) == 1 else FPS_DEFAULT

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
class FollowPlan:
    """What the follower copies each frame on a synced pair that follows its
    leader: the two links, each head's address on the port's bus, and the
    gain one step is, in dB. `mapped` heads are pods: the addresses are
    their camera maps and the gain is the GAIN row in millidecibels.
    Otherwise they are bare heads' capture nodes, where the kernel driver's
    gain control is written."""

    leader: str
    follower: str
    leader_addr: int
    follower_addr: int
    db_per_step: float
    mapped: bool = False


@dataclass(frozen=True)
class LinkTrainingResult:
    """Outcome of GMSL link training for one link."""

    link: str
    locked: bool
    rounds_used: int
    dips: int
