# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""What a hub image is compiled from and staged with: the hub a hub chip
belongs to, the reference ports its phases are composed on, and the
parameter vocabulary the image and a graph image's walks share. A walk
stages the pair's and a solo's phases with port B off; the outputs phase
runs where a caller stages it with port B's parameters, and every step
around a port is a phase of its own past them."""

from __future__ import annotations

from typing import Dict, Optional, Sequence, Tuple

from nxs.cam.contracts import InfeasibleConfig, LinkSpec, NxsUnitSpec, Topology
from nxs.cam.descriptors import Descriptor

#: The `shape` param: which port program the phases belong to.
SHAPE_PAIR = 0
SHAPE_SOLO_A = 1
SHAPE_SOLO_B = 2
SHAPES = (SHAPE_PAIR, SHAPE_SOLO_A, SHAPE_SOLO_B)
#: Phases per shape: the pair's host segments between pod runs, the solo's.
PAIR_PHASES = 8
SOLO_PHASES = 4
#: The per-link bits of the `lanes` and `ser_csi` params.
LINK_BIT = {"A": 1, "B": 2}
#: The `dt` param: the solo link's CSI-2 data type.
DATA_TYPES = {"RAW10": 0, "RAW12": 1}
CSI_LANES = (2, 4)
#: The outputs phase, port B's routing: it runs after the pair's phase 4 and
#: a solo's phase 2, and with port B off it writes nothing.
OUTPUTS_PHASE = 8
#: The `csi_b` param: port B's lane count, 0 with port B off.
CSI_B = (0, 2, 4)
#: The `streams` param: a bit per pipe and output a stream rides.
STREAM_BIT = {("Y", "A"): 1, ("Z", "A"): 2, ("Y", "B"): 4, ("Z", "B"): 8}
#: The `streams` value with port B off: both pipes on port A.
STREAMS_PORT_A = STREAM_BIT[("Y", "A")] | STREAM_BIT[("Z", "A")]
#: The `copy` param: 1 when port B carries port A's PHY copy.
COPY = (0, 1)
#: The links whose head supply is on, a mask: bit 0 link A, bit 1 link B.
HEADS = (0, 1, 2, 3)
HEADS_BOTH = 3
#: The `sync_source` row's values: each camera at its own rate, or the hub's frame sync.
SYNC_FREE, SYNC_FSYNC = 0, 1
#: The widest analog gain a cam personality's law states, in mdB, and the longest exposure in us.
GAIN_MDB_MAX = 72_000
EXPOSURE_US_MAX = 1_000_000

# The phases past the port program, each one step around a port that runs alone. A phase
# that reads a lock hands the lock bits to the walk in `result`.

#: A link's host address map, from `entries` and `amap`.
ADDRESS_MAP = 9
#: The lock of `link` at the hub's defaults, then at 12 Gbps after the GMSL3 setup.
PROBE_6G = 10
PROBE_12G = 11
#: A pristine serializer's program through `link`'s window, when the link
#: locked at the hub's defaults.
CONSTRUCT = 12
#: One training pass of `link` up its fallbacks.
TRAIN = 13
#: The heads' supply off and on through the port expander, the training's
#: dip and the recovery's longer one.
DIP_SHORT = 14
DIP_LONG = 15
#: The hub's RESET_ALL and its settle.
HUB_RESET = 16
#: Both links' window with the one-shot retrain, then the locks the shape's links waited for,
#: handed to the walk; and both links' window without the one-shot.
ONE_SHOT = 17
RUNTIME_WINDOW = 18
#: The link locks and the video locks, read.
STATUS = 19
#: The generator over the free-running pair at `fsync_period`, and the
#: pipes re-armed after a head converted.
FSYNC_ON = 20
FSYNC_REARM = 21
#: The video locks of the pipes `shape` runs.
VIDEO_LOCK = 22
#: The generator off.
TRIGGER_OFF = 23
#: The CSI output gated off and both links' window.
PARK = 24
#: The gate of `output` opened and closed, by `csi_b` and `copy`.
GATE_ON = 25
GATE_OFF = 26
#: The heads' supplies as `heads` masks them, through the port expander's outputs.
POWER = 27
PHASE_MAX = POWER
#: Each phase's name, the program's `PHASES` and the HUB_PHASES record a walk's log names it by;
#: phases 2 and 3 run the pair's primary rows and addon link, a solo's rows and its arms.
PHASE_NAMES = {0: "prologue", 1: "release", 2: "csi_rows", 3: "addon_or_arm", 4: "addon_rows",
               5: "fix_primary", 6: "fix_addon", 7: "verify", OUTPUTS_PHASE: "outputs",
               ADDRESS_MAP: "address_map", PROBE_6G: "probe_6g", PROBE_12G: "probe_12g",
               CONSTRUCT: "construct", TRAIN: "train", DIP_SHORT: "dip", DIP_LONG: "dip_long",
               HUB_RESET: "hub_reset", ONE_SHOT: "one_shot", RUNTIME_WINDOW: "runtime_window",
               STATUS: "status", FSYNC_ON: "fsync_on", FSYNC_REARM: "fsync_rearm",
               VIDEO_LOCK: "video_lock", TRIGGER_OFF: "trigger_off", PARK: "park",
               GATE_ON: "gate_on", GATE_OFF: "gate_off", POWER: "power"}
#: The `link` and `output` params: the link a phase trains or probes, the
#: CSI output a gate phase gates.
LINK_VALUE = {"A": 0, "B": 1}
OUTPUT_VALUE = {"A": 0, "B": 1}
#: The address map phase's `amap`: 7-bit fields from the low bits, the first entry's alias and
#: target, then the second's; `entries` counts the entries.
AMAP_FIELD_BITS = 7
ADDRESS_MAP_MAX = 2
AMAP_MAX = (1 << (2 * ADDRESS_MAP_MAX * AMAP_FIELD_BITS)) - 1
#: The status phase's `result`, a bit per status probe as the facts file names it; the first two are
#: the links' lock bits by `LINK_BIT`. Line-memory overflow is no field: its read clears it.
STATUS_FIELDS = (("link_lock_A", 0), ("link_lock_B", 1), ("vlock_Y", 2), ("vlock_Z", 3))
LOCK_FIELDS = STATUS_FIELDS[:2]
RESULT_MAX = 0x0F
#: The video-lock phase's `result`: a bit per pipe.
VIDEO_BIT = {"Y": 0, "Z": 1}
#: The generator's period in its clock's ticks, a 24-bit field.
FSYNC_PERIOD_MAX = 0xFFFFFF


def pack_address_map(entries: Sequence[Tuple[int, int]]) -> int:
    """The `amap` value of a link's `(alias, target)` entries, each a 7-bit
    address in its field; a third entry, or an address past 7 bits, is a
    ValueError."""
    entries = list(entries)
    if len(entries) > ADDRESS_MAP_MAX:
        raise ValueError(f"the address map holds {ADDRESS_MAP_MAX} entries, not {len(entries)}")
    packed = 0
    for i, address in enumerate(a for entry in entries for a in entry):
        if not 0 <= int(address) < 1 << AMAP_FIELD_BITS:
            raise ValueError(f"{address:#x} is no 7-bit address")
        packed |= int(address) << (AMAP_FIELD_BITS * i)
    return packed


def of_chip(chip: str):
    """The discoverable hub whose own chips include `chip`."""
    from nxs.cam import hubs

    try:
        return hubs.of_chip(chip)
    except hubs.HubError as exc:
        raise InfeasibleConfig(str(exc)) from None


def reference_topology(hub, sensors: Dict[str, str], csi_lanes: int = 2,
                       modes: Optional[Dict[str, str]] = None) -> Topology:
    """A two-link port of the hub's hub with `sensors` (link name ->
    compatible), each link in its mode from `modes` (link name -> mode
    name; the laws' choice for a link it leaves out), a pod on each link
    at the seeded aliases: what a hub image's phases are composed on."""
    ser = next(c for c in hub.chips if hub.descriptor(c).role == "SER")
    links = tuple(
        LinkSpec(name=name, des_window=0x21 + i, csi_vc=1 - i,
                 sensor_compatible=sensors[name],
                 ser_compatible=hub.descriptor(ser).compatible,
                 mode=(modes or {}).get(name),
                 nxs_units=(NxsUnitSpec(0x32 + 2 * i, 0x30, camera_map_addr=0x33 + 2 * i),))
        for i, name in enumerate(("A", "B")))
    return Topology(carrier=f"{hub.name}/cam0", i2c_bus="/dev/i2c-0", csi_lanes=csi_lanes,
                    des_compatible=hub.flows_for[0], links=links)


#: The facts a link with a pod and no camera stages the hub image with:
#: phase 0 (the prologue and the routing) dispatches on the shape alone,
#: and no CSI block follows.
POD_ONLY_FACTS: Dict[str, object] = {"lanes": 4, "data_type": "RAW10", "host_csi": False}


def link_facts(sen: Descriptor, mode: str) -> Dict[str, object]:
    """What the hub image needs to know about a link's sensor in `mode`:
    its MIPI lane count, its data type, and whether the host writes the
    serializer's CSI block (a personality without `serializer_csi`)."""
    m = sen.modes[mode]
    lanes = int(m["geometry"]["lanes"])
    if lanes not in (2, 4):
        raise InfeasibleConfig(f"{sen.compatible} {mode} drives {lanes} lanes; the "
                               f"serializer takes 2 or 4")
    data_type = str(m["mipi"]["data_type"])
    if data_type not in DATA_TYPES:
        raise InfeasibleConfig(f"{sen.compatible} {mode} sends {data_type}; the hub "
                               f"image carries {', '.join(DATA_TYPES)}")
    return {"lanes": lanes, "data_type": data_type,
            "host_csi": not bool(m.get("serializer_csi"))}
