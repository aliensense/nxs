# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The pod behind a link holds the declared sensor's personality: what it
holds is read, and the declared one is uploaded when it holds another or
none. The declaration is the truth; the pod is brought to it."""

from __future__ import annotations

import argparse
import sys
from typing import Optional

from nxs.cam import unit_source
from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs.cam.port_state import port_name


def declared_name(pack, link: LinkSpec) -> str:
    """The personality name of the sensor a link declares."""
    return str(pack.descriptor(link.sensor_compatible).name)


def holds_declared(found, pack, link: LinkSpec) -> bool:
    """Whether the personality read from the pod is the declared sensor's."""
    if found is None:
        return False
    sen = pack.descriptor(link.sensor_compatible)
    compatible = str(found.descriptor.compatible) if found.descriptor is not None else ""
    return compatible == sen.compatible or str(found.name).lower() == str(sen.name).lower()


def host_trailer_crc(source) -> Optional[int]:
    """The trailer CRC-32 of the personality image this host holds for a
    declared name; None for a source that is not an image (a class the
    host compiles carries no fixed trailer)."""
    from nxs import personality_cli
    from nxs.image import deserialize, trailer_bytes
    from nxs.personality import records

    if source.kind != personality_cli.KIND_IMAGE or not source.image:
        return None
    try:
        trailer = deserialize(source.image).trailer
        if not isinstance(trailer, (bytes, bytearray)):
            trailer = trailer_bytes(trailer)       # the records, as the unit serves them
        return int(records.trailer_crc(trailer))
    except (ValueError, records.RecordError):
        return None


def _current_build(found, name: str) -> bool:
    """Whether the personality a pod holds is the build this host holds
    under the same name: the trailers' CRC-32 agree. A name this host
    cannot resolve, or holds as a class, compares as current."""
    from nxs import personality_cli

    try:
        source = personality_cli.resolve(name)
    except personality_cli.ResolveError:
        return True
    crc = host_trailer_crc(source)
    return crc is None or int(found.crc) == crc


def param_names(pack, link: LinkSpec) -> list:
    """The parameter table of the link's declared personality as this host
    holds it (the store's image, else the pack's class compiled): what a
    pod that `converge` brought to the declaration stages by index."""
    from nxs import personality_cli
    from nxs.image import deserialize

    name = declared_name(pack, link)
    try:
        source = personality_cli.resolve(name)
    except personality_cli.ResolveError as exc:
        raise InfeasibleConfig(f"no personality {name} on this host ({exc})",
                               alternatives=["nxs assets install"]) from exc
    if source.kind == personality_cli.KIND_IMAGE:
        compiled = deserialize(source.image)
    else:
        compiled, _img = personality_cli.compile_source(source, {})
    return [p.name for p in compiled.params]


def converge(client, pack, topology: Topology, link: LinkSpec, *,
             unit_name: Optional[str] = None, dry_run: bool = False,
             log=print) -> Optional[str]:
    """Bring the pod reached over `client` to the link's declaration.
    Returns the action line when the declared personality was uploaded
    (or would be, under `dry_run`), None when the pod already held it.
    InfeasibleConfig when the declared personality is not installed on
    this host, RuntimeError when the upload was refused."""
    from nxs import personality_cli

    name = declared_name(pack, link)
    who = unit_name or f"@{link.nxs_units[0].alias_addr:#04x}"
    where = f"{port_name(topology)}/{link.name}"
    try:
        found = unit_source.read_unit_personality(client)
    except unit_source.UnreadableSlot:
        found = None
    if holds_declared(found, pack, link):
        if _current_build(found, name):
            unit_source.cache_descriptor(topology, link, found.descriptor, slot=found.slot,
                                         crc=found.crc, params=found.params, name=found.name) \
                if found.descriptor is not None else None
            return None
        # The declared personality, an earlier build of it: the assets moved.
        held = f"an earlier {found.name}"
    else:
        held = str(found.name) if found is not None and found.name else "nothing"
    line = f"{where}: pod {who} holds {held}, uploading {name}"
    if dry_run:
        return f"{line} (dry run)"
    try:
        source = personality_cli.resolve(name)
    except personality_cli.ResolveError as exc:
        raise InfeasibleConfig(
            f"no personality {name} on this host ({exc})",
            alternatives=["nxs assets install"]) from exc
    if source.kind == personality_cli.KIND_IMAGE:
        compiled, img, kind = None, source.image, source.image_kind
    else:
        compiled, img = personality_cli.compile_source(source, {})
        kind = compiled.kind
    log(f"{line} ...")
    args = argparse.Namespace(slot=None, port=port_name(topology), unit=unit_name,
                              link=link.name, json=False)
    unit_source.forget(topology, link)
    stdout = sys.stdout
    try:
        sys.stdout = sys.stderr
        rc = personality_cli._land(client, args, source, img, kind, name, compiled)
    finally:
        sys.stdout = stdout
    if rc != 0:
        raise RuntimeError(f"{where}: the upload of {name} was refused")
    return f"{line} ... ok"
