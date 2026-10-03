# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The followers nxsd runs (ADR 0013, R-SYNC-4, R-SYNC-8): a synced pair's one
gain on every owned hub port whose record says the pair follows its leader.

Once a second the port records are read. A port under frame sync whose pair
follows (`sync.ae.mode` `follow`) with both links up gets one thread and one
bus handle running `nxs.cam.follow.Follower` at the pair's rate. A follower
stops when its port runs free, parks or changes its rate or its leader, its
heartbeat removed; a reload stops every follower before the changed ports
reconverge. A follower that stops on its own faults keeps its heartbeat,
which names the reason, and runs again once its port's record changes or
the daemon reloads."""

from __future__ import annotations

import dataclasses
import logging
import threading
from typing import Any, Callable, Dict, Optional, Tuple

from nxs import _libnxs
from nxs.cam import port_state
from nxs.cam.contracts import ContractError, InfeasibleConfig, Topology
from nxs.cam.follow import Follower

log = logging.getLogger("nxsd")

#: How often the port records are read.
RECONCILE_S = 1.0
#: How long a stopped follower's thread is waited for: a step and the rest
#: of a frame period at the slowest rate take less.
JOIN_S = 2.0


@dataclasses.dataclass(frozen=True)
class _Want:
    """What a port's record asks the follower to do: the pair's leader and
    follower links, its rate, and the port's bus."""

    leader: str
    follower: str
    fps: float
    bus: str


def _wants(topology: Topology) -> Tuple[Optional[_Want], str]:
    """What the port's record asks of a follower, or why it asks nothing."""
    sync = port_state.port_sync(topology) or {}
    ae = sync.get("ae") or {}
    if sync.get("source") != "fsync":
        return None, "the port runs free"
    if ae.get("mode") != "follow":
        return None, ("camera.gain_db locks the pair" if ae.get("mode") == "locked"
                      else str(ae.get("reason") or "the record names no leader"))
    for name in (ae["leader"], ae["follower"]):
        try:
            state = port_state.link_state(topology, topology.link(str(name)))
        except ContractError:
            return None, f"link {name} is not declared"
        if state != port_state.STATE_UP:
            return None, f"link {name} is {state}"
    return _Want(str(ae["leader"]), str(ae["follower"]), float(sync["fps"]), topology.i2c_bus), ""


def _changed(before: _Want, after: _Want) -> str:
    """Why a running follower gives way to another on its port."""
    if after.leader != before.leader:
        return f"link {after.leader} leads"
    return f"the pair runs {after.fps:g} fps"


def _pair_ports(cfg) -> Dict[str, Tuple[Topology, Any]]:
    """The owned hub ports that declare a pair, each with its topology (each
    head at its capture node's host address, from the booted tree) and its
    pack; a port that does not resolve runs no follower, and its `on` says
    why."""
    from nxs.cam import packs
    from nxs.cam import topology as cam_topo
    from nxs.daemon import _owned

    ports: Dict[str, Tuple[Topology, Any]] = {}
    for name, port in sorted(cfg.ports.items()):
        if port.hub_compatible is None or not _owned(port):
            continue
        if sum(1 for link in port.links if link.camera) < 2:
            continue
        try:
            topology = cam_topo.port_topology(port)
            ports[name] = (topology, packs.pack_for(topology))
        except Exception as exc:        # noqa: BLE001 (the port's `on` names it)
            log.warning("%s: no follower (%s)", name, exc)
    return ports


class _Run:
    """One port's follower on its own thread: what it follows, its stop, and
    what its run returned once it ended."""

    def __init__(self, port: str, want: _Want, follower, bus) -> None:
        self._port = port
        self._want = want
        self._stop = threading.Event()
        self._result: Optional[str] = None
        self._thread = threading.Thread(target=self._serve, args=(follower, bus),
                                        name=f"nxsd-follow-{port}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(JOIN_S)
        if self._thread.is_alive():
            log.warning("%s: the follower did not stop within %g s", self._port, JOIN_S)

    @property
    def want(self) -> _Want:
        return self._want

    @property
    def result(self) -> Optional[str]:
        return self._result

    def ended(self) -> bool:
        """Whether the run ended on its own: its faults stopped it."""
        return not self._thread.is_alive() and not self._stop.is_set()

    def _serve(self, follower, bus) -> None:
        try:
            self._result = follower.run(self._stop)
        except Exception as exc:        # noqa: BLE001 (the journal names it)
            self._result = f"{type(exc).__name__}: {exc}"
        finally:
            bus.close()


class Followers:
    """The followers of the owned hub ports, reconciled against the port
    records once a second on a thread of their own. `open_bus` opens a
    port's bus (`_libnxs.Bus.open`) and `follower` builds a follower
    (`nxs.cam.follow.Follower`)."""

    def __init__(self, open_bus: Callable[[str], Any] = _libnxs.Bus.open,
                 follower: Callable[..., Any] = Follower) -> None:
        self._open_bus = open_bus
        self._follower = follower
        self._runs: Dict[str, _Run] = {}
        self._faulted: Dict[str, _Want] = {}
        self._cfg = None
        self._ports: Dict[str, Tuple[Topology, Any]] = {}
        self._guard = threading.Lock()
        self._halt = threading.Event()
        self._ticker: Optional[threading.Thread] = None

    def start(self, cfg) -> None:
        """Run the followers `cfg`'s ports need, then reconcile them once a
        RECONCILE_S until `stop`."""
        self.reconcile(cfg)
        self._halt.clear()
        self._ticker = threading.Thread(target=self._tick, args=(cfg,), name="nxsd-followers",
                                        daemon=True)
        self._ticker.start()

    def reconcile(self, cfg) -> None:
        """Start a follower on every port whose record asks for one and stop
        every one whose port no longer does."""
        with self._guard:
            if cfg is not self._cfg:
                self._cfg, self._ports = cfg, _pair_ports(cfg)
            wants = {name: _wants(topology) for name, (topology, _pack) in self._ports.items()}
            for name, run in list(self._runs.items()):
                want, reason = wants.get(name, (None, "the port is not declared"))
                if run.ended():
                    del self._runs[name]
                    self._faulted[name] = run.want
                    log.warning("%s: %s stops following %s (%s)", name, run.want.follower,
                                run.want.leader, run.result)
                elif want != run.want:
                    self._stop_run(name, reason or _changed(run.want, want))
            for name, (want, _reason) in wants.items():
                if name in self._faulted and self._faulted[name] != want:
                    del self._faulted[name]
                    port_state.remove_follow(name)
                if want is not None and name not in self._runs and name not in self._faulted:
                    self._start_run(name, want)

    def stop(self, reason: str = "nxsd stops") -> None:
        """Stop the reconciling and every follower, `reason` in the journal;
        every heartbeat is removed."""
        self._halt.set()
        if self._ticker is not None:
            self._ticker.join()
            self._ticker = None
        with self._guard:
            for name in list(self._runs):
                self._stop_run(name, reason)
            for name in list(self._faulted):
                port_state.remove_follow(name)
            self._faulted.clear()

    def _tick(self, cfg) -> None:
        while not self._halt.wait(RECONCILE_S):
            try:
                self.reconcile(cfg)
            except Exception:           # noqa: BLE001 (the next tick tries again)
                log.exception("the followers were not reconciled")

    def _start_run(self, name: str, want: _Want) -> None:
        topology, pack = self._ports[name]
        build = getattr(pack.flows(), "build_follow", None)
        try:
            if build is None:
                raise InfeasibleConfig(f"pack {pack.name} copies no gain")
            plan = build(pack, topology, want.leader, want.follower)
            bus = self._open_bus(want.bus)
        except (InfeasibleConfig, OSError) as exc:
            # Not tried again until the port's record changes: the same
            # record gives the same refusal.
            self._faulted[name] = want
            log.warning("%s: %s does not follow %s (%s)", name, want.follower, want.leader,
                        getattr(exc, "reason", None) or exc)
            return
        self._runs[name] = _Run(name, want, self._follower(plan, bus, 1.0 / want.fps, port=name), bus)
        log.info("%s: %s follows %s at %g fps", name, want.follower, want.leader, want.fps)

    def _stop_run(self, name: str, reason: str) -> None:
        run = self._runs.pop(name)
        run.stop()
        port_state.remove_follow(name)
        log.info("%s: %s stops following %s (%s)", name, run.want.follower, run.want.leader, reason)
