"""The silicon identities a bring-up verifies: the hub before the first write, the sensors after the program."""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple



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


def acks(i2c: CamI2c, addr: int) -> bool:
    """Whether a device answers at ``addr`` (one register read)."""
    try:
        i2c.read_reg(0x0000, reg_width=16, data_width=8, addr=hex(int(addr)))
        return True
    except Exception:
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
    declared address, else each candidate's own."""
    host = getattr(link, "host_addr", None)
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
    """Write verbs refuse a kernel-owned hub; a second register writer races
    the kernel driver."""
    if topology.hub_driver != "nxs":
        raise _refuse(f"port {_port_name(topology)}'s hub is kernel-managed: "
                      f"{verb} is not its to run",
                      f"nxs {_port_name(topology)} status")


def _declared_camera(hub, topology: Topology, links: List[LinkSpec],
                     modes: Dict[str, str], args) -> Dict[str, str]:
    """Fill the mode from the manifest's camera declaration where no `--mode`
    names one. A declared free-run fps (a link's, else the port's) is the
    flows' to resolve per link against the lawful range; under a declared
    frame sync the rate is the generator's, applied after `on`."""
    from nxs.cam.descriptors import resolve_mode

    modes = dict(modes)
    for link in links:
        if link.name in modes or not link.has_camera:
            continue
        token = link.mode or topology.camera_mode
        if token is None:
            continue
        send = hub.descriptor(link.sensor_compatible)
        modes[link.name] = resolve_mode(send, token)
    return modes

