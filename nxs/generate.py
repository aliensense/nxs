# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0
"""`nxs generate`: the rig as the bench finds it, written down.

The rig's nixos-generate-config. Walk every camera port — the hub,
each link through its window, the serializer and sensor behind it, the
NXS unit riding the link, or the sensor and the unit on the port's own
bus — and the bare buses for units on their own, then write
`hardware.yaml` (what answered, regenerated every run) and seed
`suite.yaml` (what you want of it) when there is none. A port's wiring is
what answered: a hub the last walk wrote down and nothing answers for now
is reported and not written.
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional


@dataclass
class LinkFinding:
    """One link as the walk found it; the window and the serializer are a
    hub link's."""
    name: str
    window: Optional[int]
    ser: Optional[str]
    ser_present: bool
    #: The sensor that self-described (an identity register, the pod's
    #: personality); None for a head that only answers.
    sensor: Optional[str]
    sensor_note: str = ""
    head_answers: bool = False
    head_addr: Optional[int] = None
    unit_addr: Optional[int] = None
    unit_serial: str = ""
    unit_fw: str = ""
    #: The driver the pod runs, by the name it reports; "" when it runs none.
    unit_driver: str = ""
    #: The camera personality the pod holds, as a sensor compatible.
    unit_personality: Optional[str] = None
    #: True when this link's pod is reached through a translation, so the
    #: address probed belongs to this link alone.
    unit_aliased: bool = False
    #: Why the walk did not reach this link (a kernel-owned hub selects its
    #: own windows): the link is the declaration's word, written as declared.
    unwalked: str = ""
    #: The hub's own word on the link: locked, not locked, or None for a
    #: hub that reports no lock.
    locked: Optional[bool] = None


@dataclass
class PortFinding:
    """One camera port as the walk found it; `hub` is the hub that answered,
    `silent_hub` one the wiring or the platform names and nothing answers
    for."""
    name: str
    bus: str
    lanes: Optional[int]
    hub: Optional[str]
    hub_addr: int
    hub_present: bool
    silent_hub: Optional[str] = None
    links: List[LinkFinding] = field(default_factory=list)
    topology: Any = None
    #: (addr, serial, fw) of a unit that answers through every window —
    #: on the port's bus, owned by no link.
    shared_unit: Optional[tuple] = None
    #: The driver that unit runs, by the name it reports.
    shared_driver: str = ""
    #: Links whose pods share one un-aliased address, so the walk cannot
    #: say which of them answered. (addr, [link names]).
    collided: Optional[tuple] = None
    #: Links whose declared alias answers nowhere while another link
    #: answers where every pod straps: the translation is declared but
    #: not programmed. ([silent link names], strapped address).
    unprogrammed: Optional[tuple] = None


@dataclass
class UnitFinding:
    """A unit on a bare bus, outside any hub."""
    route: str
    serial: str = ""
    fw: str = ""
    driver: str = ""
    link: Any = None


def _port_name(topology) -> str:
    return str(topology.carrier).rsplit("/", 1)[-1]


def _unit_addresses(link) -> tuple:
    """The host-side addresses this link's pod answers at.

    The aliases the topology declares for the link, which `on` programs
    into the serializer's address translation — else the address every
    NXS straps in firmware, which every pod behind the port shares.
    """
    from nxs.suite.scan import I2C_ADDRESSES

    return (tuple(int(u.alias_addr) for u in link.nxs_units)
            or I2C_ADDRESSES[:1])


def _pod_personality(hit) -> Optional[str]:
    """The sensor compatible of the camera personality a scanned pod holds,
    read over its route; None when it holds none or the read fails."""
    from nxs.cam import unit_source
    from nxs.transports import open_client

    client = None
    try:
        client = open_client(hit.link.transport, **hit.link.client_kwargs())
        found = unit_source.read_unit_personality(client)
    except Exception:        # noqa: BLE001 (a pod that answers the scan and not the store)
        return None
    finally:
        if client is not None:
            try:
                client.close()
            except Exception:        # noqa: BLE001
                pass
    if found is None:
        return None
    if found.descriptor is not None:
        return str(found.descriptor.compatible)
    return None


def _translated(link) -> bool:
    """Whether the link's pod is reached through a translation: an alias
    that differs from the address the pod straps. A pod declared at its
    strapped address answers on every link the hub merges, alias key or
    not."""
    return any(int(u.alias_addr) != int(u.target_addr) for u in link.nxs_units)


def walked_ports():
    """The ports to walk, as `on` will program them: the manifest's when
    one exists (its declared aliases are the ones `on` writes into the
    serializer), else the platform's and the pack's. A walk against a
    different topology than `on` uses would judge the rig by aliases
    nobody programs."""
    from nxs.cam.topology import _ports_from_suite, discover_ports

    loaded = _ports_from_suite()
    if loaded:
        return loaded
    return discover_ports()


def _platform_candidates() -> Dict[str, tuple]:
    """Port name -> (the port as `on` will program it, the platform's shape
    of it): the manifest's ports, then the platform's that it leaves out.
    The platform's shape is the hub a port may have been re-cabled to."""
    from nxs.cam import packs
    from nxs.cam.topology import discover_ports

    declared, _ = walked_ports()
    try:
        platform = {_port_name(t): t for t in discover_ports()[0].values()}
    except packs.PackError:
        platform = {}
    out = {_port_name(t): (t, platform.get(_port_name(t))) for t in declared.values()}
    for name, topology in platform.items():
        out.setdefault(name, (topology, topology))
    return out


def walk_ports() -> List[PortFinding]:
    """Every camera port the manifest and the platform know, walked live. A
    hub that answers is walked link by link through its windows; where none
    answers the port's own bus is walked for a sensor and a unit; a port
    where nothing answers is its bus and its lanes."""
    found: List[PortFinding] = []
    for _name, (topology, platform) in sorted(_platform_candidates().items()):
        shape = topology if not topology.is_direct else platform
        port = None
        if shape is not None and not shape.is_direct:
            if platform is not None and not platform.is_direct:
                # The declaration names the links it uses; the hub has
                # every link the platform knows, and a head re-cabled to
                # one the declaration leaves out is what the probe is for.
                shape = _with_platform_links(shape, platform)
            port = _walk_hub(shape)
            if not port.hub_present:
                silent, port = port.hub, None
            else:
                silent = None
        else:
            silent = None
        if port is None:
            port = _walk_bare(topology)
            port.silent_hub = silent
        found.append(port)
    return found


#: Where a pod answers on an isolated link: the address every pod straps
#: and the link aliases the declarations assign (`unit.alias` 0x31, 0x32,
#: one per link, and the next).
POD_ADDRESSES = tuple(range(0x30, 0x34))


def _link_locks(descriptor, results) -> Dict[str, Optional[bool]]:
    """Link name -> whether the hub reports the link locked, None for a lock
    it did not answer (`nxs.cam.diag.link_locks`)."""
    from nxs.cam.diag import link_locks

    return link_locks(descriptor, results)


def _with_platform_links(declared, platform):
    """The declared port with the platform's links it leaves out appended,
    each as the platform shapes it (its window, serializer and alias)."""
    import dataclasses

    names = {link.name for link in declared.links}
    extra = tuple(link for link in platform.links if link.name not in names)
    if not extra:
        return declared
    return dataclasses.replace(declared, links=tuple(declared.links) + extra)


def _walk_hub(topology) -> PortFinding:
    """A hub port's walk: the hub's identity, then each link through its
    window. A hub that does not answer leaves its links unwalked. The port
    written is what answered: each link carries the sensor that gave its
    identity (the declared one, else another the pack serves), or the
    declared one when it has no identity register and answers; a link with
    no serializer and no sensor answering is not written."""
    import dataclasses

    from nxs.cam.cli import _pack_for
    from nxs.cam.diag import run_probes
    from nxs.cam.engine import CamI2c
    from nxs.cam import packs
    from nxs.cam.identity import acks, answering_address, detect_sensor
    from nxs.cam.port_state import BusLock
    from nxs.suite.scan import scan_bus_units

    port = PortFinding(name=_port_name(topology), bus=topology.i2c_bus,
                       lanes=getattr(topology, "csi_lanes", None),
                       hub=topology.des_compatible, hub_addr=int(topology.des_addr),
                       hub_present=False, topology=topology)
    pack = _pack_for(topology)
    flows = pack.flows()
    desd = pack.descriptor(topology.des_compatible)
    kept = []
    with BusLock():
        i2c = CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        try:
            i2c.open()
            des = run_probes(i2c, topology.des_addr, desd)
            port.hub_present = any(r.name == "device" and r.ok for r in des)
            if not port.hub_present:
                return port
            if getattr(topology, "hub_driver", "nxs") != "nxs":
                # A kernel-owned hub selects its own windows: the links are
                # the declaration's word, and the report says so.
                port.links.extend(LinkFinding(
                    name=link.name, window=int(link.des_window), ser=link.ser_compatible,
                    ser_present=False, sensor=link.sensor_compatible,
                    unwalked="kernel-owned hub") for link in topology.links)
                return port
            # A head behind a serializer no program has touched sits in
            # reset and answers nothing: release the heads before asking.
            release = getattr(flows, "release_heads", None)
            if release is not None:
                release(pack, i2c, topology)
            # The windows do not isolate: a part on a locked link answers
            # through every window, so the hub's lock bit says which link
            # a part belongs to. An unlocked link has nothing behind it.
            locks = _link_locks(desd, des)
            alone = [name for name, up in locks.items() if up]
            # Two locked links answer at the same addresses through every
            # window: each is probed alone through its bring-up window
            # (the other link down meanwhile), as `on` addresses them.
            isolate = len(alone) > 1 and hasattr(flows, "isolate_link")
            for link in topology.links:
                if link.name in locks and locks[link.name] is not True:
                    port.links.append(LinkFinding(
                        name=link.name, window=int(link.des_window),
                        ser=link.ser_compatible, ser_present=False, sensor=None,
                        sensor_note=("link not locked" if locks[link.name] is False
                                     else "link lock not read"),
                        unwalked="" if locks[link.name] is False else "link lock not read",
                        locked=locks[link.name]))
                    continue
                if isolate:
                    if not flows.isolate_link(pack, i2c, topology, link):
                        port.links.append(LinkFinding(
                            name=link.name, window=int(link.des_window),
                            ser=link.ser_compatible, ser_present=False, sensor=None,
                            sensor_note="its window did not re-lock", locked=True,
                            unwalked="its window did not re-lock; the parts answer on "
                                     "every link until `switch` programs their aliases"))
                        continue
                else:
                    flows.open_window(pack, i2c, topology, link)
                serd = pack.descriptor(link.ser_compatible)
                ser = run_probes(i2c, link.ser_addr, serd)
                present = any(r.name == "device" and r.ok for r in ser)
                send = pack.descriptor(link.sensor_compatible) if link.has_camera else None
                # The head answers at its alias while the port is up, at
                # its own address before the first `on` maps it. Alone on
                # the bus it may answer at an alias a previous declaration
                # mapped for any link: its serializer keeps the map.
                isolated = isolate or alone == [link.name]
                sen_addr, _mapped = answering_address(pack, i2c, link)
                if isolated and not acks(i2c, sen_addr):
                    for other in topology.links:
                        alias = int(packs.sensor_address(pack, other))
                        if alias != sen_addr and acks(i2c, alias):
                            sen_addr = alias
                            break
                sen = run_probes(i2c, sen_addr, send) if send is not None else []
                identity = next((r for r in sen if r.name == "device"), None)
                # What self-describes names the sensor: an identity
                # register, else the pod's personality. A head that only
                # answers is written without a sensor, and the report says
                # what to declare.
                sensor = None
                answers = False
                if identity is None:
                    try:
                        i2c.read_reg(0x0000, reg_width=16, data_width=8,
                                     addr=hex(sen_addr))
                        answers = True
                    except (OSError, RuntimeError):
                        answers = False
                    note = (f"no identity register; the head answers at {sen_addr:#04x}"
                            if answers else "no identity register; no ACK")
                elif identity.ok:
                    note, sensor, answers = "identity ok", link.sensor_compatible, True
                else:
                    detected, _detail = detect_sensor(pack, i2c, link)
                    if detected is not None:
                        note, sensor, answers = f"identity {detected}", detected, True
                    else:
                        note = f"identity {identity.text}"
                finding = LinkFinding(
                    name=link.name, window=int(link.des_window),
                    ser=link.ser_compatible, ser_present=present,
                    sensor=sensor, sensor_note=note, head_answers=answers,
                    head_addr=int(sen_addr))
                # The window does not isolate — `open_window` keeps
                # both links enabled, because a runtime LINK_CFG
                # change without a one-shot wedges them — so only a
                # translated address belongs to this link alone.
                finding.unit_aliased = _translated(link)
                finding.locked = locks.get(link.name)
                addresses = _unit_addresses(link)
                if isolated:
                    # The only link that answers (the one locked, or the one
                    # enabled): a pod at the address every pod straps, no
                    # alias programmed yet, or at an alias a previous
                    # declaration mapped, can only be its.
                    addresses = tuple(dict.fromkeys(list(addresses) + list(POD_ADDRESSES)))
                for hit in scan_bus_units(topology.i2c_bus, addresses):
                    finding.unit_addr = int(hit.link.address)
                    finding.unit_serial = hit.serial
                    finding.unit_fw = hit.fw_version
                    finding.unit_driver = hit.driver
                    finding.unit_personality = _pod_personality(hit)
                    break
                if sensor is None and finding.unit_personality:
                    sensor = finding.unit_personality
                    finding.sensor = sensor
                    finding.sensor_note = f"the pod's personality; {note}"
                if present or sensor is not None:
                    kept.append(dataclasses.replace(link, sensor_compatible=sensor or ""))
                port.links.append(finding)
            if isolate:
                flows.restore_links(pack, i2c, topology, alone)
            else:
                flows.close_windows(pack, i2c, topology)
        except (OSError, RuntimeError):
            return port
        finally:
            i2c.close()
    port.topology = dataclasses.replace(topology, links=tuple(kept))
    _disown_shared_unit(port)
    _flag_unprogrammed_alias(port)
    return port


def _walk_bare(topology) -> PortFinding:
    """The walk of a port's own bus, where no hub answers: the sensor that
    answers with its identity (the declared one, else any the tool knows
    there), and the unit beside it. The port's topology becomes what
    answered: that one link, or its bus and lanes alone."""
    import dataclasses

    from nxs.cam import packs
    from nxs.cam.contracts import LinkSpec, NxsUnitSpec
    from nxs.cam.engine import CamI2c
    from nxs.cam.identity import detect_sensor, identity_facts
    from nxs.cam.port_state import BusLock
    from nxs.suite.scan import scan_bus_units

    declared = topology.links[0] if topology.is_direct and topology.links else None
    # The port's pack as the camera verbs take it: the tool's own, with the
    # installed personalities and the cached unit descriptors adopted.
    bare = dataclasses.replace(topology, des_compatible=None,
                               links=(declared,) if declared is not None else ())
    pack = packs.pack_for(bare)
    port = PortFinding(name=_port_name(topology), bus=topology.i2c_bus,
                       lanes=getattr(topology, "csi_lanes", None), hub=None,
                       hub_addr=int(topology.des_addr), hub_present=False)
    detected = None
    answers = False
    with BusLock():
        i2c = CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        try:
            i2c.open()
            detected, _detail = detect_sensor(pack, i2c, declared)
            if detected is None and declared is not None:
                # A declared sensor with no identity register is written
                # when it answers at its address, and not otherwise.
                desc = pack.descriptor(declared.sensor_compatible)
                if identity_facts(desc) is None:
                    i2c.read_reg(0x0000, reg_width=16, data_width=8,
                                 addr=hex(packs.sensor_address(pack, declared)))
                    answers = True
        except (OSError, RuntimeError):
            pass
        finally:
            i2c.close()
    # What self-describes names the sensor: an identity register, else the
    # pod's personality below. A head that only answers is reported with
    # what to declare; a head that does not answer is not written.
    sensor = detected
    sen_addr = int(packs.sensor_address(pack, declared)) if declared is not None else None
    links = ()
    finding = None
    if sensor is not None or answers:
        if detected is None:
            note = f"no identity register; the head answers at {sen_addr:#04x}"
        elif declared is None or detected == declared.sensor_compatible:
            note = "identity ok"
        else:
            note = f"identity {detected}, declared {declared.sensor_compatible}"
        finding = LinkFinding(name=declared.name if declared is not None else "A",
                              window=None, ser=None, ser_present=False,
                              sensor=sensor, sensor_note=note, head_answers=True,
                              head_addr=sen_addr)
        # The declared unit's address first, then every address a unit
        # straps: nothing on the port's own bus separates two, so the first
        # answer is the link's, and a stale declaration hides no unit.
        declared_addrs = (tuple(int(u.alias_addr) for u in declared.nxs_units)
                          if declared is not None and declared.nxs_units else ())
        from nxs.suite.scan import I2C_ADDRESSES

        addresses = declared_addrs + tuple(a for a in I2C_ADDRESSES if a not in declared_addrs)
        for hit in scan_bus_units(topology.i2c_bus, addresses):
            finding.unit_addr = int(hit.link.address)
            finding.unit_serial = hit.serial
            finding.unit_fw = hit.fw_version
            finding.unit_driver = hit.driver
            finding.unit_personality = _pod_personality(hit)
            break
        if sensor is None and finding.unit_personality:
            sensor = finding.unit_personality
            finding.sensor = sensor
            finding.sensor_note = f"the pod's personality; {finding.sensor_note}"
        port.links.append(finding)
        units = ((NxsUnitSpec(alias_addr=finding.unit_addr, target_addr=finding.unit_addr),)
                 if finding.unit_addr is not None else ())
        # The link's unit is the one that answered, at the address it
        # answered; a head no one names is reported and not written.
        if sensor is not None:
            links = (dataclasses.replace(declared, sensor_compatible=sensor, nxs_units=units)
                     if declared is not None else
                     LinkSpec(name="A", des_window=None, csi_vc=0, sensor_compatible=sensor,
                              ser_compatible=None, nxs_units=units),)
    port.topology = dataclasses.replace(bare, links=links)
    return port


def _flag_unprogrammed_alias(port: PortFinding) -> None:
    """A declared alias answering nowhere, on a port where another link
    answers where every pod straps, means `on` has not programmed the
    translation yet. The pods are still merged onto that one address, so
    what answered there may be several of them at once — the bytes read
    back are whatever the bus settles on, not one unit's. The address
    stands; the serial and the driver do not."""
    silent = [l.name for l in port.links if l.unit_aliased and not l.unit_serial]
    merged = [l for l in port.links if not l.unit_aliased and l.unit_serial]
    if not silent or not merged:
        return
    port.unprogrammed = (silent, merged[0].unit_addr)
    for link in merged:
        link.unit_serial, link.unit_fw, link.unit_driver = "", "", ""


def _disown_shared_unit(port: PortFinding) -> None:
    """Resolve a serial seen by more than one link.

    Two cases, and they are not the same. When the links probed
    *distinct* addresses and one serial answered at all of them, the
    unit is on the port's own bus, behind no serializer, and no link may
    claim it — the seed would otherwise declare it once per link. When
    the links probed the *same* address, they were never separable:
    every pod straps the native address until `on` programs a declared
    alias, so the answer says nothing about which link it came from.
    That is a gap in the topology, recorded as one, not a unit on the
    trunk.
    """
    answered = [l for l in port.links if l.unit_serial]
    if len(port.links) < 2 or len(answered) != len(port.links):
        return
    if len({l.unit_serial for l in port.links}) != 1:
        return
    addrs = {l.unit_addr for l in port.links}
    if len(addrs) == 1 and not all(l.unit_aliased for l in port.links):
        # One address, no alias: a collision, not a shared unit.
        port.collided = (port.links[0].unit_addr,
                         [l.name for l in port.links])
        for link in port.links:
            link.unit_addr, link.unit_serial, link.unit_fw, link.unit_driver = None, "", "", ""
        return
    shared = port.links[0]
    port.shared_unit = (shared.unit_addr, shared.unit_serial, shared.unit_fw)
    port.shared_driver = shared.unit_driver
    for link in port.links:
        link.unit_addr, link.unit_serial, link.unit_fw, link.unit_driver = None, "", "", ""


def scan_units(ports: List[PortFinding]) -> List[UnitFinding]:
    """Units on the bare buses — everything the link sweep finds — less
    the ones already attributed to a link, which the bare bus reaches
    only through whichever window was left open."""
    from nxs.suite.scan import scan_suite

    attributed = {l.unit_serial for p in ports for l in p.links if l.unit_serial}
    attributed |= {p.shared_unit[1] for p in ports if p.shared_unit and p.shared_unit[1]}
    # A port whose translation is not programmed answers for its merged
    # pods at one address. That answer belongs to no bare bus, and its
    # serial is not a unit's — the sweep must not hand it out as one.
    merged = {(p.bus, p.unprogrammed[1]) for p in ports if p.unprogrammed}
    merged |= {(p.bus, p.collided[0]) for p in ports if p.collided}
    units: List[UnitFinding] = []
    for hit in scan_suite(None):
        if hit.serial and hit.serial in attributed:
            continue
        if (getattr(hit.link, "bus", None),
                getattr(hit.link, "address", None)) in merged:
            continue
        units.append(UnitFinding(route=hit.link.describe(), serial=hit.serial,
                                 fw=hit.fw_version, driver=hit.driver,
                                 link=hit.link))
    return units


def _unit_name(port: str, link: str) -> str:
    return f"unit-{port}-{link.lower()}"


def _serial_or_none(serial: str) -> Optional[str]:
    """A serial the manifest parser will take back — the 12-byte UID96
    as 24 hex digits — else nothing: a seed that does not parse would
    wedge the bootstrap it exists to start."""
    from nxs.suite.schema import ManifestError, normalize_serial
    if not serial:
        return None
    try:
        return normalize_serial(serial, "seed")
    except ManifestError:
        return None


def _unit_entry(name: str, route: Dict[str, Any], serial: str, driver: str) -> Dict[str, Any]:
    """A seeded unit: its name and route, its serial when the parser takes
    it back, and the personality that compiles to the driver it runs. A unit
    running no personality the tool knows gets no `sensors` key, which leaves
    its store undeclared: `sensors: []` would clear it on the first switch."""
    from nxs.suite.scan import _module_for_driver

    entry: Dict[str, Any] = {"name": name, "module": "nxs", "links": [route]}
    if _serial_or_none(serial):
        entry["serial"] = _serial_or_none(serial)
    personality = _module_for_driver(driver) if driver else None
    if personality:
        entry["sensors"] = [{"personality": personality}]
    return entry


def name_by_trial(units: List[UnitFinding], opener=None) -> None:
    """A unit running nothing is asked by trial: every personality the tool
    knows uploaded in turn, and the one whose sensor answers stays running
    and names the unit's Click (`nxs.detect`)."""
    from nxs.detect import detect
    from nxs.transports import open_client

    opener = opener or open_client
    for unit in units:
        if unit.driver or unit.link is None:
            continue
        client = None
        try:
            client = opener(unit.link.transport, **unit.link.client_kwargs())
            print(f"{unit.route}: runs nothing; trying every personality",
                  file=sys.stderr)
            name = detect(client, report=lambda line: print(f"  {line}",
                                                             file=sys.stderr))
            if name:
                unit.driver = client.read_driver_name() or name
        except Exception as exc:        # noqa: BLE001 (the unit's own refusals)
            print(f"{unit.route}: the trial failed: {exc}", file=sys.stderr)
        finally:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass


def seed_intent(ports: List[PortFinding], units: List[UnitFinding],
                intent_by_port: Dict[str, dict]) -> dict:
    """A first suite.yaml: the intent the live port already carries
    (mode, sync), a name and alias for every unit found riding a link,
    and every unit found on a bare bus — named by where it answers,
    for the operator to rename."""
    from nxs.suite.freeze import hex_address
    from nxs.suite.scan import I2C_ADDRESSES

    doc: Dict[str, Any] = {}
    port_docs: Dict[str, dict] = {}
    unit_docs: List[dict] = []
    alias_base = int(I2C_ADDRESSES[0])
    for port in ports:
        entry = dict(intent_by_port.get(port.name) or {})
        # The declaration names the hub and each link's sensor: what
        # self-described is seeded, a head that only answered is left for
        # the person to declare.
        if port.bus:
            entry.setdefault("bus", port.bus)
        if port.hub_present and port.hub:
            entry.setdefault("hub", port.hub)
        for link in port.links:
            if link.sensor and not link.unwalked:
                entry.setdefault("links", {}).setdefault(link.name, {}).setdefault(
                    "camera", link.sensor)
        # A collision is the answer of at least one pod behind links the
        # walk cannot separate, and nothing separates them until each is
        # declared: the alias `on` programs is what makes the next walk
        # able to tell them apart. Declaring only what already answered
        # would leave the rig unable to ever discover the rest.
        collided = set(port.collided[1]) if port.collided else set()
        for index, link in enumerate(port.links):
            if link.unit_addr is None and link.name not in collided:
                continue
            name = _unit_name(port.name, link.name)
            # Behind a hub every pod is presented at its own alias, the
            # next addresses up from the one every NXS straps, so the
            # host never speaks the strapped address on the port and the
            # pods do not collide when the hub merges its links.
            ref: Dict[str, Any] = {"name": name}
            if port.hub is None:
                # Nothing translates on the port's own bus: the unit is
                # reached where it answered, its own address.
                if link.unit_addr is not None and link.unit_addr != alias_base:
                    ref["target"] = hex_address(link.unit_addr)
            else:
                ref["alias"] = hex_address(alias_base + index + 1)
            entry.setdefault("links", {}).setdefault(link.name, {})["unit"] = ref
            route = {"transport": "i2c", "link": f"{port.name}/{link.name}"}
            unit_docs.append(_unit_entry(name, route, link.unit_serial, link.unit_driver))
        if entry:
            port_docs[port.name] = entry
        if port.shared_unit:
            addr, serial, _fw = port.shared_unit
            route = {"transport": "i2c", "bus": port.bus, "address": hex_address(addr)}
            unit_docs.append(_unit_entry(f"unit-{port.name}", route, serial, port.shared_driver))
    from nxs.suite.scan import _suggest_name

    for unit in units:
        link = unit.link
        route = {"transport": link.transport}
        for key in ("bus", "address", "port", "iface", "node_id"):
            value = getattr(link, key, None)
            if value is not None:
                route[key] = hex_address(value) if key == "address" else value
        unit_docs.append(_unit_entry(_suggest_name(link), route, unit.serial, unit.driver))
    if port_docs:
        doc["ports"] = port_docs
    if unit_docs:
        doc["units"] = unit_docs
    return doc


_SEED_HEADER = ("# Written by `nxs generate` from what it found. This file is yours to\n"
                "# edit, by hand or with `nxs tune`: names, personalities, settings. The wiring\n"
                "# lives in hardware.yaml beside it and is rewritten on every run.\n")


def generate(config_path: str, dry_run: bool = False,
             walker: Optional[Callable[[], List[PortFinding]]] = None,
             unit_scanner: Optional[Callable[[List[PortFinding]], List[UnitFinding]]] = None,
             ) -> Dict[str, Any]:
    """Walk, then write: hardware.yaml regenerated, suite.yaml seeded
    only when absent. Returns the report and what was written."""
    walker = walker or walk_ports
    unit_scanner = unit_scanner or scan_units

    from nxs.cam import port_state
    from nxs.suite.freeze import (_render_hardware, _split_entry, _write_atomic,
                                  port_block, render_seed)
    from nxs.suite.schema import hardware_path

    from nxs.generate_report import render_report, walk_data

    ports = walker()
    units = unit_scanner(ports)
    if not dry_run:
        name_by_trial(units)
    wiring: Dict[str, dict] = {}
    intent: Dict[str, dict] = {}
    for port in ports:
        if port.topology is None:
            continue
        own = port_state.port_record(port.topology)
        # The sensors are the walk's, never the state cache's: the wiring
        # is what answered today. The mode and the viewer hints are the
        # port's own record.
        block = port_block(port.topology, sync=own.get("sync"),
                           viewer=own.get("viewer"), viewers=own.get("viewers"),
                           modes=own.get("modes"))
        wired, want = _split_entry(block)
        if wired:
            wiring[port.name] = wired
        if want:
            intent[port.name] = want
    seed = seed_intent(ports, units, intent)
    exists = os.path.exists(config_path) and os.path.getsize(config_path) > 0
    result = {"report": render_report(ports, units), "hardware": None,
              "seeded": None, "kept": config_path if exists else None,
              "ports": len(wiring), "units": len(seed.get("units") or []),
              "walk": walk_data(ports, units)}
    if dry_run:
        return result
    hw = hardware_path(config_path)
    _write_atomic(hw, lambda f: f.write(_render_hardware(wiring)))
    result["hardware"] = hw
    if not exists and seed:
        _write_atomic(config_path, lambda f: f.write(_SEED_HEADER + render_seed(seed)))
        result["seeded"] = config_path
    return result


def cmd_generate(args) -> int:
    from nxs.suite import default_config_path

    from nxs.suite import stray_declaration

    config_path = getattr(args, "config", None) or default_config_path()
    stray = stray_declaration(config_path)
    if stray:
        raise SystemExit(f"nxs generate: {stray}")
    try:
        result = generate(config_path, dry_run=bool(getattr(args, "dry_run", False)))
    except Exception as exc:        # no pack, a silent tree, an unwritable path
        raise SystemExit(f"nxs generate: {exc}")
    if getattr(args, "json", False):
        import json

        from nxs.schemas import CONTRACT
        print(json.dumps({"contract": CONTRACT, "manifest": config_path,
                          "dry_run": bool(getattr(args, "dry_run", False)),
                          "hardware": result["hardware"], "seeded": result["seeded"],
                          "kept": result["kept"], **result["walk"]}, indent=2))
        return 0
    print(result["report"], end="")
    if getattr(args, "dry_run", False):
        print(f"dry run: would write {os.path.basename(result.get('hardware') or 'hardware.yaml')}"
              f" ({result['ports']} port(s))")
        return 0
    print(f"wrote {result['hardware']} ({result['ports']} port(s))")
    if result["seeded"]:
        print(f"seeded {result['seeded']} ({result['units']} unit(s)); next: nxs tune")
    elif result["kept"]:
        print(f"kept {result['kept']}; next: nxs tune")
    else:
        print("nothing to seed; connect a unit and run nxs generate again")
    return 0
