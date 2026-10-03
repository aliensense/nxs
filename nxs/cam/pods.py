# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The pod behind a link holds the declared sensor's personality: what it
holds is read, and the declared one is uploaded when it holds another or
none. The declaration is the truth; the pod is brought to it."""

from __future__ import annotations

import argparse
import contextlib
import io
import os
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


def held_stale(stale, topology: Topology, link: LinkSpec) -> str:
    """What a pod holds in a slot whose image it refuses to describe: the
    name the link's cache recorded for that slot, else a personality image,
    then why (`<name> of another format version`)."""
    personality = (unit_source.cached_record(topology, link) or {}).get("personality") or {}
    name = str(personality.get("name") or "") if personality.get("slot") == stale.slot else ""
    return f"{name or 'a personality image'} {stale.what}"


def held_instead(found, pack, link: LinkSpec) -> Optional[str]:
    """What the pod holds in place of the declared personality in the
    build this host holds: `nothing`, `an earlier <Name>`, or another
    sensor's `<Name>`. None when it holds that build, the one a run staged
    from this host's parameter table lands on."""
    if holds_declared(found, pack, link):
        if _current_build(found, declared_name(pack, link)):
            return None
        return f"an earlier {found.name}"
    return str(found.name) if found is not None and found.name else "nothing"


class PodSilent(RuntimeError):
    """A pod that answers at none of its link's addresses; the line names
    the port and the link, so a caller prints it bare."""


class PodRefused(RuntimeError):
    """A pod that refused the upload of the declared personality; the line
    names the port and the link, so a caller prints it bare."""


def _answers(topology: Topology, address: int) -> bool:
    """Whether a unit answers at `address` on the port's bus."""
    from nxs.transports import open_client

    try:
        client = open_client("i2c", bus=topology.i2c_bus, address=address)
    except (OSError, RuntimeError, ValueError):
        return False
    try:
        return bool(client.probe())
    except (OSError, RuntimeError):
        return False
    finally:
        client.close()


def _failure(exc: OSError) -> str:
    """`<operation>: <OS error text>` from a client error, which carries the
    operation after the text (`Remote I/O error (peek slot 0)`)."""
    text = exc.strerror or str(exc)
    base = os.strerror(exc.errno) if exc.errno else ""
    if base and text.startswith(f"{base} (") and text.endswith(")"):
        return f"{text[len(base) + 2:-1]}: {base}"
    return text


def _unanswered(topology: Topology, link: LinkSpec, exc: OSError) -> Optional[str]:
    """The line for a pod whose store did not read: the link, the addresses
    tried (the alias, then the address the pod straps) and the failed
    operation. None when a unit answers at the alias: the failure was not
    silence."""
    unit = link.nxs_units[0]
    alias, own = int(unit.alias_addr), int(unit.target_addr)
    if _answers(topology, alias):
        return None
    where = f"{port_name(topology)}/{link.name}"
    if own == alias:
        return f"{where}: no pod answers at {alias:#04x} ({_failure(exc)})"
    if _answers(topology, own):
        return (f"{where}: no pod answers at {alias:#04x}, a unit answers at {own:#04x} "
                f"({_failure(exc)})")
    return f"{where}: no pod answers at {alias:#04x} or at {own:#04x} ({_failure(exc)})"


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
             unit_name: Optional[str] = None, dry_run: bool = False) -> Optional[str]:
    """Bring the pod reached over `client` to the link's declaration.
    Returns the action line when the declared personality was uploaded
    (or would be, under `dry_run`), None when the pod already held it.
    InfeasibleConfig when the declared personality is not installed on
    this host, PodRefused when the upload was refused, PodSilent when no
    pod answers at the link's addresses, DeviceRefused when the pod refuses
    a read of its store."""
    from nxs import personality_cli

    name = declared_name(pack, link)
    who = unit_name or f"@{link.nxs_units[0].alias_addr:#04x}"
    where = f"{port_name(topology)}/{link.name}"
    stale = None
    try:
        found = unit_source.read_unit_personality(client)
    except unit_source.StaleSlot as exc:
        found, stale = None, exc
    except unit_source.UnreadableSlot:
        found = None
    except OSError as exc:
        # A timeout is a unit that took the command and never finished it,
        # not a silent pod: that error stands.
        line = None if isinstance(exc, TimeoutError) else _unanswered(topology, link, exc)
        if line is None:
            raise
        raise PodSilent(line) from exc
    # An earlier build of the declared personality is replaced too: the
    # assets moved. So is an image of another format version, which the
    # upload saves over in its slot.
    held = held_stale(stale, topology, link) if stale is not None else held_instead(found, pack, link)
    if held is None:
        if found.descriptor is not None:
            unit_source.cache_descriptor(topology, link, found.descriptor, slot=found.slot,
                                         crc=found.crc, params=found.params, name=found.name)
        return None
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
    args = argparse.Namespace(slot=None, port=port_name(topology), unit=unit_name,
                              link=link.name, json=False)
    unit_source.forget(topology, link)
    # The upload's own lines are this line's: its refusals still reach stderr.
    with contextlib.redirect_stdout(io.StringIO()):
        rc = personality_cli._land(client, args, source, img, kind, name, compiled)
    if rc != 0:
        raise PodRefused(f"{where}: the upload of {name} was refused")
    return f"{line} ... ok"
