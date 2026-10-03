# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The port watch nxsd runs beside the followers: every PORT_WATCH_S it reads
each pod of a port recorded up at its alias, and a pod that stopped
answering there on SILENT_READS reads in a row (a hub
power cycle leaves the pods where they strap, the port still recorded up)
marks the port's links unknown and brings the port up again through `on`,
which maps the aliases; a bring-up that fails runs again on each tick until
one succeeds, and one that stops at `REBOOT NEEDED` is left to a reload,
since another `on` writes the same boot entry. The followers stop before
that bring-up and start again after it, as on a reload, and the watch's
tick is one piece of the daemon's port work, so it never interleaves with a
construction or a reload's reconverge."""

from __future__ import annotations

import logging
import threading
from typing import List

log = logging.getLogger("nxsd")

#: How often the watch looks for a pod that stopped answering at its alias
#: on a port the daemon brought up, one read per pod when it looks.
PORT_WATCH_S = 5.0
#: The reads in a row a pod misses before the watch brings its port up: one
#: missed read is a transient a bring-up would turn into an outage.
SILENT_READS = 3

#: The declaration the watch reads: the one the daemon serves now.
_cfg: list = [None]
#: Whether another run held the bus at the watch's last read, so a stretch
#: of held ticks logs once.
_held: list = [False]
#: The ports whose bring-up by the watch failed: their links are no longer
#: recorded up, so each tick brings them up again until one succeeds.
_pending: set = set()
#: The reads in a row a pod of each port has missed.
_misses: dict = {}
_stop = threading.Event()


def serve(cfg) -> None:
    """The declaration the watch reads from here on."""
    _cfg[0] = cfg


def _silent_pods(topology, links) -> List[str]:
    """The links whose pod does not answer at its alias, one read each under
    the bus lock and nothing else: a port recorded up keeps the control
    channel of every link it brought up open, so no window write and no
    settle hold the lock the followers and the verbs share. Every link with
    a pod reads silent when the hub itself does not answer."""
    from nxs.cam import port_state
    from nxs.cam import run as cam_run

    pods = [link for link in links if link.nxs_units]
    if not pods:
        return []
    silent: List[str] = []
    with port_state.held_wait(), port_state.BusLock():
        i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        try:
            i2c.open()
            for link in pods:
                try:
                    i2c.read_reg(0x0000, reg_width=16, data_width=8,
                                 addr=hex(link.nxs_units[0].alias_addr))
                except Exception:        # noqa: BLE001 (a silent pod, or a silent hub)
                    silent.append(link.name)
        finally:
            i2c.close()
    return silent


def tick(cfg, followers=None) -> List[str]:
    """One look at the owned ports: a port recorded up whose pod missed
    SILENT_READS reads in a row at its alias has its links marked unknown
    and comes up again through `on`, which maps the aliases, and a port
    whose last such bring-up failed comes up again the same way, unless a
    link was parked since or the `on` stopped at `REBOOT NEEDED`. The
    followers stop before the first bring-up and start again after the last
    (a bring-up reprograms the heads they write, as on a reload). Returns
    the ports brought up."""
    from nxs import daemon
    from nxs.cam import port_state
    from nxs.cam.select import select_port_links

    reconverged: List[str] = []
    stopped = False
    _pending.intersection_update(cfg.ports)
    for gone in set(_misses) - set(cfg.ports):
        del _misses[gone]
    try:
        for name in sorted(cfg.ports):
            if not daemon._owned(cfg.ports[name]):
                _pending.discard(name)
                continue
            try:
                topology, links = select_port_links(daemon._port_args(name), require_port=True)
            except SystemExit:
                continue
            if not any(link.nxs_units for link in links):
                continue
            states = {port_state.link_state(topology, link) for link in links}
            if states == {port_state.STATE_UP}:
                _pending.discard(name)
                try:
                    silent = _silent_pods(topology, links)
                except port_state.BusHeld:
                    # A SystemExit, which would end the thread. The lock is every
                    # port's, so the tick ends here and the next one reads again.
                    if not _held[0]:
                        log.info("port watch: another run holds the bus; the next tick reads again")
                    _held[0] = True
                    break
                except Exception as exc:        # noqa: BLE001 (a bus the watch cannot open now)
                    log.debug("port %s: the watch skipped it: %s", name, exc)
                    continue
                _held[0] = False
                if not silent:
                    _misses.pop(name, None)
                    continue
                _misses[name] = _misses.get(name, 0) + 1
                if _misses[name] < SILENT_READS:
                    log.debug("port %s: a pod missed its read at its alias (%d of %d)",
                              name, _misses[name], SILENT_READS)
                    continue
                log.warning("port %s: a pod stopped answering at its alias; reconverging", name)
            elif name in _pending and port_state.STATE_PARKED not in states:
                log.info("port %s: the last reconverge failed; reconverging again", name)
            else:
                # Down by a step of its own, `off` or an `on` by hand: not the watch's.
                _pending.discard(name)
                continue
            _misses.pop(name, None)
            if followers is not None and not stopped:
                followers.stop("a pod stopped answering at its alias")
                stopped = True
            port_state.mark_unknown(topology, links)
            rc = daemon._bring_up(name, "reconverged", "reconvergence failed")
            if rc == 0:
                _pending.discard(name)
                reconverged.append(name)
            elif rc == 3:
                # The verdict stands for `switch` and a reload; another `on`
                # would write the same boot entry.
                _pending.discard(name)
                log.warning("port %s: REBOOT NEEDED; the watch leaves the port to a reload", name)
            else:
                _pending.add(name)
    finally:
        if stopped:
            followers.start(cfg)
    return reconverged


def _watch(stop: threading.Event, period: float = PORT_WATCH_S, followers=None) -> None:
    """A tick every `period` seconds on the declaration the daemon serves,
    one piece of port work at a time, until `stop` is set."""
    from nxs import daemon

    while not stop.wait(period):
        with daemon._port_work:
            try:
                tick(_cfg[0], followers)
            except Exception:        # noqa: BLE001 (the watch outlives one bad tick)
                log.exception("port watch failed")


def start(cfg, followers=None) -> threading.Thread:
    """Run the watch on `cfg`'s ports, on a thread of its own, until `stop`."""
    serve(cfg)
    _stop.clear()
    thread = threading.Thread(target=_watch, args=(_stop, PORT_WATCH_S, followers),
                              name="port-watch", daemon=True)
    thread.start()
    return thread


def stop() -> None:
    """End the watch after its current tick."""
    _stop.set()
