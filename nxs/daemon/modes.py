# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The row each pod runs, read from its camera door once a second and
reconciled against the port record: a lawful row a pod moved to walks the
port at that row (the hub's pipes for its data type, the declared sync,
the delivery check). The walk is the port's `on` over every link that is
up, each at its recorded row but the moved ones, so a synced pair keeps
its trigger and both its links. A row the booted table lacks is refused
there, and the walk back to the recorded rows leaves that refusal the
port's verdict; a value outside the personality's mode table is walked
back to the recorded row. The daemon's other residents stop for a walk
and start again after it, as for any bring-up, and a walk never waits on
the port work another thread holds: that thread may be the one stopping
this resident."""

from __future__ import annotations

import contextlib
import logging
import threading
from typing import Callable, Dict, Optional, Tuple

from nxs._generated_constants import NxsMap
from nxs.cam import port_state
from nxs.cam.contracts import LinkSpec, Topology

log = logging.getLogger("nxsd")

#: How often the pods' rows are read.
RECONCILE_S = 1.0
#: `CURRENT_MODE` before the first run.
NO_MODE = 0xFF


def read_current_mode(bus: str, address: int) -> Optional[int]:
    """`CURRENT_MODE` of the camera door at `address` on `bus` while the door
    reads READY; None while it is INERT, BUSY or FAULT, when the row it
    names is not the one in effect. The block is read twice: a read that
    lands on the pod's own bus transaction comes back torn, and only two
    agreeing reads count."""
    from nxs._libnxs_map import open_door

    first, last = int(NxsMap.MapReg.STATE), int(NxsMap.MapReg.CURRENT_MODE)
    with open_door(bus, address) as door:
        block = door.read(first, last - first + 1)
        again = door.read(first, last - first + 1)
    if block != again or block[0] != int(NxsMap.MapState.READY):
        return None
    return int(block[last - first])


class Modes:
    """The pods' rows, reconciled once a RECONCILE_S on a thread of their
    own. `reader(bus, address)` reads a camera door's `CURRENT_MODE`
    (`read_current_mode`); `walk(port, rows)` brings the links `rows`
    names up, each at its row (the daemon's `on`); `residents` are the
    daemon's residents this one runs among, every other one stopped for the
    length of a walk."""

    def __init__(self, reader: Callable[[str, int], Optional[int]] = read_current_mode,
                 walk: Optional[Callable[[str, Dict[str, str]], int]] = None,
                 residents=None) -> None:
        self._reader = reader
        self._walk = walk
        self._residents = residents
        self._cfg = None
        self._ports: Dict[str, Tuple[Topology, object]] = {}
        self._halt = threading.Event()
        self._ticker: Optional[threading.Thread] = None
        # The row each link was last walked at, by port and link: a walk the bring-up refused, or
        # one whose record did not follow, is not repeated while the pod still reads that row.
        self._walked: Dict[Tuple[str, str], int] = {}
        # The fault each link's last look raised, by port and link: logged once until a look passes.
        self._faults: Dict[Tuple[str, str], str] = {}

    def start(self, cfg) -> None:
        """Reconcile once a RECONCILE_S until `stop`, the first look a
        RECONCILE_S on: a walk stops the other residents, which the start
        that runs this one has not all started yet."""
        self._halt.clear()
        self._ticker = threading.Thread(target=self._tick, args=(cfg,), name="nxsd-modes",
                                        daemon=True)
        self._ticker.start()

    def reconcile(self, cfg) -> None:
        """Read every up pod link's row and walk the port where a pod moved,
        once for every link that moved. A look that raises, on a record
        naming a mode the program no longer offers or a personality the
        host lacks, skips its link alone and is logged once until the link's
        look passes again."""
        if cfg is not self._cfg:
            self._cfg, self._ports = cfg, _pod_ports(cfg)
        for name, (topology, hub) in self._ports.items():
            moved: Dict[str, Tuple[int, Optional[str]]] = {}
            for link in topology.links:
                key = (name, link.name)
                try:
                    found = self._look(name, topology, hub, link)
                except Exception as exc:        # noqa: BLE001 (the link's look, never the daemon)
                    fault = str(exc).splitlines()[0] if str(exc) else type(exc).__name__
                    if self._faults.get(key) != fault:
                        log.warning("%s/%s: the pod's row is not followed (%s)", name, link.name,
                                    fault)
                    self._faults[key] = fault
                    continue
                self._faults.pop(key, None)
                if found is not None:
                    moved[link.name] = found
            if moved:
                self._walk_port(name, topology, moved)

    def stop(self, reason: str = "nxsd stops") -> None:
        """End the reconciling after its current tick. The tick never waits
        on the port work, so a stop from under it, the watch's or a
        reload's, returns once the tick's reads end."""
        del reason
        self._halt.set()
        if self._ticker is not None:
            self._ticker.join()
            self._ticker = None

    def _tick(self, cfg) -> None:
        while not self._halt.wait(RECONCILE_S):
            try:
                self.reconcile(cfg)
            except Exception:           # noqa: BLE001 (the next tick tries again)
                log.exception("the pods' rows were not reconciled")

    def _look(self, name: str, topology: Topology, hub,
              link: LinkSpec) -> Optional[Tuple[int, Optional[str]]]:
        """One link's look: the row its pod runs against the recorded one.
        `(value, row)` where the pod moved to a value no walk took yet, the
        row None for a value outside the personality's table; None where the
        pod runs the recorded row or names none."""
        if not link.nxs_units or not link.has_camera:
            return None
        if port_state.link_state(topology, link) != port_state.STATE_UP:
            return None
        recorded = port_state.port_mode(topology, link)
        if recorded is None:
            return None
        value = self._read(topology, link)
        if value is None or value == NO_MODE:
            return None
        sen = hub.descriptor(link.sensor_compatible)
        if value == int(sen.mode_value(recorded)):
            self._walked.pop((name, link.name), None)
            return None
        if self._walked.get((name, link.name)) == value:
            return None
        return value, _row_named(sen, value)

    def _walk_port(self, name: str, topology: Topology,
                   moved: Dict[str, Tuple[int, Optional[str]]]) -> None:
        """The port walked over every link that is up, each moved link at
        the row its pod runs and every other one at its recorded row, or
        all back at their recorded rows where no moved row is the table's.
        The port work is taken without a wait (the next tick looks again),
        and the other residents stop for the walk."""
        from nxs import daemon

        if not daemon._port_work.acquire(blocking=False):
            return
        try:
            if self._halt.is_set():
                return
            recorded = {link.name: port_state.port_mode(topology, link) for link in topology.links
                        if port_state.link_state(topology, link) == port_state.STATE_UP}
            recorded = {link: row for link, row in recorded.items() if row is not None}
            # A bring-up since the look may have taken a link down: its row is not followed.
            moved = {link: found for link, found in moved.items() if link in recorded}
            if not moved:
                return
            asked = dict(recorded)
            for link, (value, row) in moved.items():
                self._walked[(name, link)] = value
                if row is None:
                    log.warning("%s/%s: the pod runs row %d, which the table does not boot; "
                                "walking it back to %s", name, link, value, recorded[link])
                else:
                    log.info("%s/%s: the pod runs %s (recorded %s); walking the port at %s",
                             name, link, row, recorded[link], row)
                    asked[link] = row
            with self._others_stopped(f"{name} walks to the rows its pods run"):
                if asked == recorded:
                    self._bring_up(name, recorded, "walked back to the recorded rows")
                    return
                rows = ", ".join(f"{link} {row}" for link, row in asked.items())
                if self._bring_up(name, asked, f"walked at {rows}") == 0:
                    return
                # The bring-up refused the rows (a booted table that lacks one, or another fault):
                # the pods go back to the recorded rows and the refusal stays the port's verdict.
                refused = daemon.port_verdict(name)
                log.warning("%s: %s does not come up; walking it back to the recorded rows",
                            name, rows)
                self._bring_up(name, recorded, "walked back to the recorded rows", refused)
        finally:
            daemon._port_work.release()

    def _others_stopped(self, reason: str):
        """The other residents stopped for a walk: its delivery check ends
        every capture session on the port, the camera nodes' among them,
        and it reprograms the heads the followers write."""
        if self._residents is None:
            return contextlib.nullcontext()
        return self._residents.beside(self, self._cfg, reason)

    def _read(self, topology: Topology, link: LinkSpec) -> Optional[int]:
        """The link's camera door row, None when the bus is held or the read fails."""
        lock = port_state.BusLock()
        if not lock.acquire_now():
            return None
        try:
            return self._reader(topology.i2c_bus, int(link.nxs_units[0].camera_map_addr))
        except (OSError, RuntimeError):
            return None
        finally:
            lock.release()

    def _bring_up(self, port: str, rows: Dict[str, str], done: str,
                  kept: Optional[dict] = None) -> int:
        """The links `rows` names brought up together through `on`, each at
        its row: every link of a port that is up whole, so its declared sync
        runs again; `kept` is the verdict the run leaves standing when it
        comes up (`daemon._bring_up`)."""
        from nxs import daemon

        if self._walk is not None:
            return self._walk(port, dict(rows))
        return daemon._bring_up(port, done, "the walk failed",
                                args=daemon._port_args(port, links=list(rows),
                                                       modes=list(rows.values())),
                                kept=kept)


def _pod_ports(cfg) -> Dict[str, Tuple[Topology, object]]:
    """The owned ports with a camera link that carries a unit, each with
    its topology and hub."""
    from nxs.cam import hubs
    from nxs.cam import topology as cam_topo
    from nxs.daemon.ports import owned

    ports: Dict[str, Tuple[Topology, object]] = {}
    for name, port in sorted(cfg.ports.items()):
        if not owned(port) or not any(link.camera and link.unit for link in port.links):
            continue
        try:
            topology = cam_topo.port_topology(port)
            ports[name] = (topology, hubs.for_topology(topology))
        except Exception as exc:        # noqa: BLE001 (the port's `on` names it)
            log.warning("%s: the pods' rows are not followed (%s)", name, exc)
    return ports


def _row_named(sen, value: int) -> Optional[str]:
    """The mode name of `value` in the sensor's table, None outside it."""
    from nxs.personality.records import mode_values

    for name, mode_value in mode_values(sen).items():
        if int(mode_value) == int(value):
            return str(name)
    return None
