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

from nxs._generated_constants import NxsMap
from nxs.cam import unit_source
from nxs.device_errors import ERRNO_ENOTSUP, DeviceRefused
from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs.cam.port_state import port_name


def declared_name(hub, link: LinkSpec) -> str:
    """The personality name of the sensor a link declares."""
    return str(hub.descriptor(link.sensor_compatible).name)


def holds_declared(found, hub, link: LinkSpec) -> bool:
    """Whether the personality read from the pod is the declared sensor's."""
    if found is None:
        return False
    sen = hub.descriptor(link.sensor_compatible)
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


def held_instead(found, hub, link: LinkSpec) -> Optional[str]:
    """What the pod holds in place of the declared personality in the
    build this host holds: `nothing`, `an earlier <Name>`, or another
    sensor's `<Name>`. None when it holds that build, the one a run staged
    from this host's parameter table lands on."""
    if holds_declared(found, hub, link):
        if _current_build(found, declared_name(hub, link)):
            return None
        return f"an earlier {found.name}"
    return str(found.name) if found is not None and found.name else "nothing"


class PodSilent(RuntimeError):
    """A pod that answers at none of its link's addresses; the line names
    the port and the link, so a caller prints it bare."""


class PodRefused(RuntimeError):
    """A pod that refused the upload of the declared personality; the line
    names the port and the link, so a caller prints it bare."""


class PodFirmwareOld(PodRefused):
    """A pod whose firmware predates the image format of the declared
    personality, which it refused with ENOTSUP: the line names the port,
    the link, what the pod holds and the push of the firmware that takes
    the image. `fact` is the line without the push, for a run whose own
    firmware step converges the pod first."""

    def __init__(self, fact: str, push: str) -> None:
        super().__init__(f"{fact}\n  - {push}")
        self.fact = fact


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


#: The host's forms of a standard row: the rate in frames a second the rate
#: row's, the gain in decibels the gain's, the mode the RUN row's.
ROW_FORMS = ((NxsMap.ParamId.FRAME_RATE_MFPS, "fps"), (NxsMap.ParamId.GAIN_MDB, "gain_db"),
             (NxsMap.ParamId.MODE, "mode"))


def declared_records(hub, link: LinkSpec) -> list:
    """The trailer records of the declared personality as this host builds
    it: what the pod serves once it converges, and the records a smart
    camera's map is built from."""
    return records_of(declared_name(hub, link))


def records_of(name: str) -> list:
    """The trailer records of the personality `name` as this host builds it;
    InfeasibleConfig when it is not installed here."""
    from nxs import personality_cli
    from nxs.image import deserialize, parse_trailer, trailer_bytes

    try:
        source = personality_cli.resolve(name)
    except personality_cli.ResolveError as exc:
        raise InfeasibleConfig(f"no personality {name} on this host ({exc})",
                               alternatives=["nxs assets install"]) from exc
    if source.kind == personality_cli.KIND_IMAGE:
        trailer = deserialize(source.image).trailer
    else:
        trailer = personality_cli.compile_source(source, {})[0].trailer
    if not isinstance(trailer, (bytes, bytearray)):
        trailer = trailer_bytes(trailer)
    return list(parse_trailer(trailer))


def camera_map_knobs(hub, link: LinkSpec) -> Optional[list]:
    """The knobs a link's head offers through its camera map, from the rows
    of the declared personality: each row an APPLY commit takes, by its
    name, and `fps`, `gain_db` and `mode` where the rate, the gain and the
    mode rows stand. None for a personality without a camera map
    (the host writes its head)."""
    from nxs._generated_constants import NxsDriverImage
    from nxs.personality.records import camera_map_info

    info = camera_map_info(declared_records(hub, link))
    if info is None:
        return None
    knobs = {row.name for row in info["rows"]
             if row.kind == NxsDriverImage.ParamKind.APPLY and not row.is_read_only}
    ids = {row.id for row in info["rows"]}
    knobs.update(form for pid, form in ROW_FORMS if pid in ids)
    return sorted(knobs)


def converge(client, hub, topology: Topology, link: LinkSpec, *,
             unit_name: Optional[str] = None, dry_run: bool = False) -> Optional[str]:
    """Bring the pod reached over `client` to the link's declaration.
    Returns the action line when the declared personality was uploaded
    (or would be, under `dry_run`), None when the pod already held it.
    InfeasibleConfig when the declared personality is not installed on
    this host, PodRefused when the upload was refused, PodSilent when no
    pod answers at the link's addresses, DeviceRefused when the pod refuses
    a read of its store or the upload. A pod whose firmware predates the
    image's format refuses the upload with ENOTSUP and keeps what it holds:
    PodFirmwareOld names the push, and only a run with a firmware step of
    its own (`nxs switch`) goes on past it."""
    from nxs import personality_cli

    name = declared_name(hub, link)
    who = unit_name or f"@{link.nxs_units[0].alias_addr:#04x}"
    where = f"{port_name(topology)}/{link.name}"
    # A pod on the first interface serves another map: nothing of its store
    # reads, and its firmware is converged before anything else is asked.
    if first_interface(client):
        from nxs.firmware import push_fw_asset

        raise PodFirmwareOld(f"{where}: pod {who} speaks interface version 1, the first interface",
                             f"nxs {_push_target(topology, link, unit_name)} push-fw "
                             f"{push_fw_asset()}")
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
    held = held_stale(stale, topology, link) if stale is not None else held_instead(found, hub, link)
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
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            rc = personality_cli._land(client, args, source, img, kind, name, compiled)
    except DeviceRefused as exc:
        if exc.code != ERRNO_ENOTSUP:
            raise
        from nxs.firmware import push_fw_asset

        raise PodFirmwareOld(f"{where}: pod {who} holds {held}; its firmware predates {name}'s "
                             f"image format",
                             f"nxs {_push_target(topology, link, unit_name)} push-fw "
                             f"{push_fw_asset()}") from exc
    if rc != 0:
        raise PodRefused(f"{where}: the upload of {name} was refused")
    return f"{line} ... ok"


def first_interface(client) -> bool:
    """Whether the unit behind `client` speaks the first host interface; False
    where the transport serves no version or the read fails."""
    from nxs.firmware import FIRST_PROTO_VERSION

    reader = getattr(client, "interface_version", None)
    if reader is None:
        return False
    try:
        return reader() == FIRST_PROTO_VERSION
    except (OSError, RuntimeError, NotImplementedError):
        return False


def _push_target(topology: Topology, link: LinkSpec, unit_name: Optional[str]) -> str:
    """The flags that address a link's pod on a command line: its declared
    name, else its alias on the port's bus."""
    from nxs.cam.verbs.status import _pod_names

    name = unit_name or _pod_names(topology).get(link.name)
    if name:
        return f"--unit {name}"
    return f"-t i2c -b {topology.i2c_bus} -a 0x{int(link.nxs_units[0].alias_addr):02X}"
