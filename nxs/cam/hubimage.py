# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""What a hub image is compiled from and staged with: the pack a hub chip
belongs to, the reference ports its phases are composed on, and the
parameter vocabulary the images and the walk (`nxs_port_up`) share."""

from __future__ import annotations

from typing import Dict

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


def pack_of(chip: str):
    """The discoverable pack that ships `chip` with flows for its hub."""
    from nxs.cam import packs

    for pack in packs.discover():
        if chip in pack.chips and pack.flows_for:
            return pack
    raise InfeasibleConfig(f"no discoverable pack ships {chip} with its flows")


def reference_topology(pack, sensors: Dict[str, str], csi_lanes: int = 2) -> Topology:
    """A two-link port of the pack's hub with `sensors` (link name ->
    compatible), a pod on each link at the seeded aliases: what a hub
    image's phases are composed on."""
    ser = next(c for c in pack.chips if pack.descriptor(c).role == "SER")
    links = tuple(
        LinkSpec(name=name, des_window=0x21 + i, csi_vc=1 - i,
                 sensor_compatible=sensors[name],
                 ser_compatible=pack.descriptor(ser).compatible,
                 nxs_units=(NxsUnitSpec(0x31 + i, 0x30),))
        for i, name in enumerate(("A", "B")))
    return Topology(carrier=f"{pack.name}/cam0", i2c_bus="/dev/i2c-0", csi_lanes=csi_lanes,
                    des_compatible=pack.flows_for[0], links=links)


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
