# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""What the daemon reads of a declared port: whether it programs the port
(`owned`), the capture stack's state, the verified rates, a hub gone
silent, and a port left down or running another declaration."""

import logging
from typing import Optional, Tuple

log = logging.getLogger("nxsd")


def owned(port) -> bool:
    """A port this daemon programs: a hub declared with the nxs driver, or a
    sensor on the port's own bus. A link there runs its personality on its
    unit, or on the host when it declares none, so every such port with a
    link is the daemon's to bring up at boot."""
    if port.hub_compatible is not None:
        return port.hub_driver == "nxs"
    return bool(port.links)


def capture_stack(name: str, port) -> Tuple[Optional[str], int]:
    """The port's capture-stack state for the booted table (`ready`,
    `preparing`, `missing`) with the table's mode count; (None, 0) for a
    port that streams no video, or one the host cannot shape (its `on`
    says why). A missing configuration is the port's `on` to build, once
    its walk has the heads streaming."""
    from nxs import host as host_layer
    from nxs.cam import topology as cam_topo
    from nxs.host.cli import _table_rows, capture_stack_state

    if not owned(port) or not any(link.camera for link in port.links):
        return None, 0
    try:
        topology = cam_topo.port_topology(port)
    except Exception:        # noqa: BLE001 (the port's `on` refuses it with the reason)
        return None, 0
    host = host_layer.current()
    return capture_stack_state(host, name, topology), len(_table_rows(host, topology.i2c_bus))


def log_capture_stack(name: str, port) -> None:
    """After the `on` of a port whose capture stack had no configuration
    for the booted table: the one it built, in the journal, or that none
    was (the verb's refusal above it says why)."""
    state, modes = capture_stack(name, port)
    if state == "ready":
        log.info("port %s: capture stack built (%d modes)", name, modes)
    else:
        log.error("port %s: the capture stack's configuration was not built", name)


def log_verified(name: str, port) -> None:
    """The delivery check the port's `on` passed, in the journal."""
    from nxs.cam import port_state
    from nxs.cam import topology as cam_topo
    from nxs.cam.verbs import verify

    topology = cam_topo.port_topology(port)
    seen = port_state.verified(topology)
    if seen:
        log.info("port %s: verified %s", name, verify.summary(topology, seen["rates"]))


def hub_silent(port) -> bool:
    """Whether the port declares a hub and it does not answer its first
    register; a bus that does not open is a silent hub too."""
    from nxs.cam import hubs
    from nxs.cam import run as cam_run
    from nxs.cam import topology as cam_topo
    from nxs.cam.descriptors import to_int
    from nxs.cam.verbs.status import _hub_answers

    if port.hub_compatible is None:
        return False
    try:
        topology = cam_topo.port_topology(port)
        registers = hubs.for_topology(topology).descriptor(topology.des_compatible).registers
    except Exception:        # noqa: BLE001 (the port's `on` refuses it with the reason)
        return False
    i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
    try:
        i2c.open()
        return not _hub_answers(i2c, to_int(registers.get("REG0", {}).get("addr", 0)),
                                topology.des_addr)
    except (OSError, RuntimeError):
        return True
    finally:
        i2c.close()


def left_down(port) -> bool:
    """Whether an owned port has a link a step recorded this boot neither up
    nor parked: a bring-up that did not verify, or a port `nxs switch`
    recorded down to come up again (a pod took a new build, its hub lost
    the aliases, or the port was parked). A port `off` parked is the
    operator's, and one no step recorded is the construction's."""
    from nxs.cam import port_state
    from nxs.cam import topology as cam_topo

    if not owned(port):
        return False
    try:
        topology = cam_topo.port_topology(port)
    except Exception:        # noqa: BLE001 (the port's `on` refuses it with the reason)
        return False
    states = set(port_state.recorded_states(topology).values())
    return port_state.STATE_UNKNOWN in states and port_state.STATE_PARKED not in states


def runs_another_declaration(port) -> bool:
    """Whether an owned port is recorded up under a declaration other than
    `port`: its record carries another digest, or none (a record another
    build wrote, or an `on` that departed from the declaration). A port
    the host cannot shape is left to its `on`, which says why."""
    from nxs.cam import port_state
    from nxs.cam import topology as cam_topo

    if not owned(port):
        return False
    try:
        topology = cam_topo.port_topology(port)
    except Exception:        # noqa: BLE001 (the port's `on` refuses it with the reason)
        return False
    if not any(port_state.link_state(topology, link) == port_state.STATE_UP
               for link in topology.links):
        return False
    return port_state.port_record(topology).get("declared") != topology.declared
