"""The silicon identities a bring-up verifies: the hub before the first write, the sensors after the program."""

from __future__ import annotations

import errno
from typing import Any, Dict, List, Optional, Tuple



from nxs.cam.descriptors import to_int
from nxs.cam.contracts import LinkSpec, Topology
from nxs.cam.engine import CamI2c
from nxs.cam.select import _port_name, _refuse


def _identity_read(i2c: CamI2c, addr: int, reg: int, width: int) -> int:
    """An identity register, MSB first, through a live bus."""
    value = 0
    for i in range(int(width)):
        byte = i2c.read_reg(int(reg) + i, length=1, reg_width=16,
                            data_width=8, addr=hex(addr))[0]
        value = (value << 8) | int(byte)
    return value


def identity_facts(descriptor) -> Optional[Tuple[int, int, int]]:
    """(register, expected value, byte width) when the descriptor
    declares an identity register, else None (declared-only parts)."""
    meta = descriptor.raw("meta") or {}
    reg, want = meta.get("device_id_reg"), meta.get("device_id")
    if reg is None or want is None:
        return None
    return to_int(reg), to_int(want), to_int(meta.get("device_id_width", 1))


def _verify_hub_identity(hub, topology: Topology, i2c: CamI2c) -> None:
    """Declare + verify: refuse a hub whose silicon identity is not the
    declared part (an ACK proves only an address). A port that names no
    hub has none to verify."""
    if topology.is_direct:
        return
    facts = identity_facts(hub.descriptor(topology.des_compatible))
    if facts is None:
        # A hub the hub cannot identify is a hub the tool never programs:
        # the identity read is the gate every write stands behind.
        raise _refuse(
            f"hub descriptor {topology.des_compatible} declares no identity "
            f"register (meta.device_id_reg / device_id)",
            "declare the register in the hub")
    reg, want, width = facts
    try:
        got = _identity_read(i2c, topology.des_addr, reg, width)
    except (OSError, RuntimeError):
        raise _refuse(
            f"hub at {topology.i2c_bus}:{hex(topology.des_addr)} does not answer",
            "check DES power and cabling", f"nxs {_port_name(topology)} status",
            "nxs generate (writes what answers on the port)")
    if got != want:
        raise _refuse(
            f"hub at {topology.i2c_bus}:{hex(topology.des_addr)} reads "
            f"device id 0x{got:02X}, not {topology.des_compatible} (0x{want:02X})",
            f"nxs {_port_name(topology)} status")


def verify_hub(hub, topology: Topology) -> None:
    """The hub answers as the declared silicon (`_verify_hub_identity`),
    read through a handle of its own before a walk's first write; a bus
    that does not open is a hub that does not answer."""
    if topology.is_direct:
        return
    i2c = CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
    try:
        i2c.open()
    except Exception as exc:
        if isinstance(getattr(exc, "__cause__", None), PermissionError):
            raise
        raise _refuse(
            f"hub at {topology.i2c_bus}:{hex(topology.des_addr)} does not answer "
            f"({exc.__class__.__name__})",
            "check DES power and cabling", f"nxs {_port_name(topology)} status",
            "nxs generate (writes what answers on the port)") from exc
    try:
        _verify_hub_identity(hub, topology, i2c)
    finally:
        i2c.close()


def acks(i2c: CamI2c, addr: int) -> bool:
    """Whether a device answers at ``addr`` (one register read)."""
    try:
        i2c.read_reg(0x0000, reg_width=16, data_width=8, addr=hex(int(addr)))
        return True
    except Exception:
        return False


def unit_acks(i2c: CamI2c, addr: int, tries: int = 1) -> bool:
    """Whether a unit answers at ``addr``: a one-byte read at register 0,
    which every interface answers, up to `tries` times. A walk that judges a pod by one look tries as the
    unit door's probe does (`nxs._libnxs_map.UNIT_PROBES`), since a pod's
    answer can land inside its own transfer and NAK once."""
    for _ in range(tries):
        try:
            i2c.read_reg(0x00, reg_width=8, data_width=8, addr=hex(int(addr)))
            return True
        except Exception:
            continue
    return False


def answering_address(hub, i2c: CamI2c, link: LinkSpec) -> Tuple[int, bool]:
    """Where the link's sensor answers now, through an open window: its host
    alias while the link's serializer maps it (the port up), else its own
    address (the port down, nothing mapped). Returns (address, at_alias)."""
    from nxs.cam import hubs

    alias = int(hubs.sensor_address(hub, link))
    native = int(hubs.native_sensor_address(hub, link))
    if alias != native and acks(i2c, alias):
        return alias, True
    return native, False


def detect_sensor(hub, i2c: CamI2c, link: LinkSpec) -> Tuple[Optional[str], Optional[str]]:
    """Through an open link window: which of the hub's identity-bearing
    sensors answers with its id. Returns (compatible, detail); (None, None)
    when none does. The probe goes where the link's sensor answers: the
    alias its serializer maps while the port is up, else the link's
    declared address, else each candidate's own. Behind a pod the link's
    host address is the unit's camera map, never the sensor."""
    host = None if getattr(link, "nxs_units", ()) else getattr(link, "host_addr", None)
    declared = getattr(link, "sensor_addr", None)
    mapped = host is not None and acks(i2c, int(host))
    # The declared sensor is the hypothesis: it is tried first, so two heads
    # that share an identity register and id never shadow it.
    wanted = getattr(link, "sensor_compatible", None)
    candidates = sorted(hub.sensors(), key=lambda c: hub.descriptor(c).compatible != wanted)
    for chip in candidates:
        desc = hub.descriptor(chip)
        facts = identity_facts(desc)
        if facts is None:
            continue
        reg, want, width = facts
        if mapped:
            addr = int(host)
        elif declared is not None:
            addr = int(declared)
        else:
            addr = to_int((desc.raw("meta") or {}).get("i2c_addr", 0x1A))
        try:
            got = _identity_read(i2c, addr, reg, width)
        except Exception:
            continue
        if got == want:
            return desc.compatible, f"id 0x{got:0{2 * width}X} at {hex(addr)}"
    return None, None


def map_head(hub, topology: Topology, link: LinkSpec) -> Dict[str, Any]:
    """A pod's head as its camera map says it runs, never a read of the
    sensor (R-10.2): the pod's serializer forwards the address a head
    straps, so a read there could answer from the other link's head. The
    presence walk's entry for the link: `sen` is the map's own answer at
    the link's map address, and `sensor` is ok while the map serves the
    declared personality's rows and runs a mode, READY or BUSY. An INERT
    map has not configured its head, and a FAULT one faulted on its last
    commit; a fault of a head that did not answer the pod (ENODEV at the
    personality's address, EIO past the register retries) is the head's
    absence, `head: silent` with the head's address on the pod's bus."""
    from nxs._libnxs_map import MapState, open_door
    from nxs.cam import pods, smart_camera
    from nxs.cam.contracts import InfeasibleConfig
    from nxs.cam.hubs import map_address, native_sensor_address

    addr = int(map_address(hub, link))
    declared = str(link.sensor_compatible)
    entry: Dict[str, Any] = {"sen_addr": addr, "mapped": addr != int(link.nxs_units[0].map_target)}
    line, running, serves, silent = "", False, None, False
    try:
        with open_door(topology.i2c_bus, addr) as cmap:
            if smart_camera.answers(cmap):
                boot = cmap.header()
                line = smart_camera.map_line(cmap)
                running = (boot["current_mode"] != 0xFF
                           and boot["state"] in (MapState.READY, MapState.BUSY))
                silent = (boot["state"] == MapState.FAULT
                          and int(boot["error"]) in (errno.ENODEV, errno.EIO))
                try:
                    serves = smart_camera.serves(cmap, pods.declared_records(hub, link))
                except InfeasibleConfig:
                    serves = None       # this host holds no build of the declared personality
    except OSError:
        line = ""
    if not line:
        return {**entry, "sen": "no ACK",
                "sensor": {"declared": declared, "detected": None, "detail": f"no ACK at {addr:#04x}",
                           "ok": False, "text": f"{declared}: no camera door answers at {addr:#04x}"}}
    if silent:
        head = int(native_sensor_address(hub, link))
        return {**entry, "sen": "present",
                "sensor": {"declared": declared, "detected": None, "detail": line, "ok": False,
                           "head": "silent", "head_addr": head,
                           "text": f"{declared}: no head answers at {head:#04x} on the pod"}}
    text = f"{line}; its rows are not {declared}'s" if serves is False else f"{declared} ({line})"
    return {**entry, "sen": "present",
            "sensor": {"declared": declared, "detected": declared if serves else None, "detail": line,
                       "ok": running and serves is not False, "text": text}}


def sensor_identity_line(link: LinkSpec, declared_desc, detected: Optional[str],
                         detail: Optional[str]) -> Tuple[bool, str]:
    """The probe's verdict on a link's sensor: (ok, text)."""
    declared = declared_desc.compatible
    if detected is None:
        if identity_facts(declared_desc) is None:
            return True, f"{declared} (declared; no identity register)"
        return False, (f"{declared} declared but its identity register "
                       f"did not answer")
    if detected == declared:
        return True, f"{declared} ({detail})"
    return False, (f"MISMATCH: declared {declared}, answers as {detected} "
                   f"({detail})")


def _require_nxs_hub(topology: Topology, verb: str) -> None:
    """Write verbs refuse a hub this host does not master: a kernel driver's,
    which a second register writer races, or a module's, whose bus this
    host does not touch."""
    if topology.hub_driver == "module":
        raise _refuse(f"port {_port_name(topology)}'s hub is a module's, which masters its bus: "
                      f"{verb} is not this host's to run",
                      f"nxs {_port_name(topology)} status")
    if topology.hub_driver != "nxs":
        raise _refuse(f"port {_port_name(topology)}'s hub is kernel-managed: "
                      f"{verb} is not its to run",
                      f"nxs {_port_name(topology)} status")


def _declared_camera(hub, topology: Topology, links: List[LinkSpec],
                     modes: Dict[str, str], args) -> Dict[str, str]:
    """Fill the mode from the declaration's camera nodes where no `--mode`
    names one. A camera node's free-run fps is the flows' to resolve per
    link against the lawful range; under a declared frame sync the rate is
    the generator's, applied after `on`."""
    from nxs.cam.descriptors import resolve_mode

    modes = dict(modes)
    for link in links:
        if link.name in modes or not link.has_camera:
            continue
        token = link.mode
        if token is None:
            continue
        send = hub.descriptor(link.sensor_compatible)
        modes[link.name] = resolve_mode(send, token)
    return modes

