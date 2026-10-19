# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The camera nodes nxsd runs: one process per camera link the ports'
records carry, publishing the link's frames as ROS 2 topics the way the
launch's `cameras:=true` does (`nxs.ros2_cameras`), reconciled against the
port records once a second as the followers are. The nodes start in plan
order, one per `CAMERA_STAGGER_S`. A port's first node, once it says its
stream started, has the port's output restarted under it, since the capture
stack locks onto a stream it saw start; a node that starts beside a session
delivering longer than `CAMERA_OPEN_S` needs nothing. A node that exits comes
back after `RESTART_S`, and `FAILS_MAX` exits within `FAILS_WINDOW_S` park the
link's node as failed, its last line kept, until the link's record changes
or a reload. A stop sends SIGINT, SIGTERM and SIGKILL with a grace each, in
reverse start order. The nodes stop before every bring-up and start after it,
as the followers do: a bring-up's delivery check ends every capture session
on the port. The resident records that it serves the camera links, with the
links the plan carries now (`plane.camera_plane`), from its first start to
the daemon's end, the stops for a bring-up included: an operator's launch
reads it and starts no node beside the daemon's."""

from __future__ import annotations

import collections
import logging
import signal
import subprocess
import threading
import time
from typing import Any, Callable, Deque, Dict, List, Optional

from nxs.cam import daemon_records, port_state
from nxs.daemon.plane import record_camera_plane
from nxs.ros2_cameras import (CAMERA_OPEN_S, CAMERA_STAGGER_S, CAMERA_STARTED, CameraTopic,
                              camera_plan, node_argv_for, port_openers, stream_start)
from nxs.suite.schema_base import ManifestError

log = logging.getLogger("nxsd")

#: How often the port records are read.
RECONCILE_S = 1.0
#: How long a node that exited waits before it starts again.
RESTART_S = 5.0
#: Exits within the window that park a link's node.
FAILS_MAX = 3
FAILS_WINDOW_S = 60.0
#: How long each stop signal gets before the next.
GRACE_S = 3.0


def _key(spec: CameraTopic) -> str:
    return f"{spec.port}/{spec.link}"


class _Node:
    """One link's camera node: its process, the spec it runs, whether it is
    the port's opener, its last line, and the thread that reads its output
    into the journal."""

    def __init__(self, key: str, spec: CameraTopic, process, opener: bool, started: float,
                 on_line: Callable[["_Node", str], None]) -> None:
        self.key = key
        self.spec = spec
        self.process = process
        self.opener = opener
        self.started = started
        self.line = ""
        self.opened = False
        self._on_line = on_line
        self.reader = threading.Thread(target=self._read, name=f"nxsd-cam-{key}", daemon=True)
        self.reader.start()

    def _read(self) -> None:
        try:
            for line in self.process.stdout:
                text = line.rstrip()
                if text:
                    self.line = text
                    self._on_line(self, text)
        except (OSError, ValueError):     # the pipe closed under the reader
            pass


class Cameras:
    """The camera nodes of the camera links that are up, reconciled against
    the port records once a second on a thread of their own. `popen` runs a
    node's command line (`subprocess.Popen`), `plan` lists the links to serve
    (`camera_plan`), `argv` gives a link's command line (`node_argv_for`),
    `opener` restarts a port's output under its first session (`stream_start`),
    and `clock` is the monotonic clock."""

    def __init__(self, popen: Callable[..., Any] = subprocess.Popen,
                 plan: Callable[[], List[CameraTopic]] = camera_plan,
                 argv: Callable[[CameraTopic], List[str]] = node_argv_for,
                 opener: Callable[[str, Optional[float]], int] = stream_start,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._popen = popen
        self._plan = plan
        self._argv = argv
        self._opener = opener
        self._clock = clock
        self._nodes: Dict[str, _Node] = {}
        self._failed: Dict[str, CameraTopic] = {}
        self._exits: Dict[str, Deque[float]] = {}
        self._restart_due: Dict[str, float] = {}
        self._last_start = float("-inf")
        # The links the record names, None before the first reconcile writes it.
        self._recorded: Optional[List[str]] = None
        # Whether the latest plan was refused, so the refusal is logged once.
        self._refused = False
        self._guard = threading.RLock()
        self._halt = threading.Event()
        self._ticker: Optional[threading.Thread] = None

    def start(self, cfg) -> None:
        """Run the nodes the links that are up need, then reconcile them
        once a RECONCILE_S until `stop`."""
        self.reconcile(cfg)
        self._halt.clear()
        self._ticker = threading.Thread(target=self._tick, args=(cfg,), name="nxsd-cameras",
                                        daemon=True)
        self._ticker.start()

    def reconcile(self, cfg) -> None:
        """Start a node on every link the plan carries, in order and one per
        stagger, and stop every one whose link left the plan or changed. The
        record names the plan's links, rewritten when they change."""
        del cfg
        from nxs import daemon

        with self._guard:
            try:
                plan = self._plan()
            except ManifestError as exc:
                # A declaration that stops loading changes nothing: the nodes keep
                # the links they serve, and the journal names the refusal once.
                if not self._refused:
                    log.error("the camera nodes keep their links, the manifest is rejected: %s",
                              str(exc).splitlines()[0])
                self._refused = True
                return
            self._refused = False
            wanted = {_key(spec): spec for spec in plan}
            if list(wanted) != self._recorded:
                record_camera_plane(list(wanted))
                self._recorded = list(wanted)
            openers = {_key(plan[index]) for index in port_openers(plan).values()}
            for key, node in list(self._nodes.items()):
                spec = wanted.get(key)
                if spec is None:
                    self._stop_node(key, "the link is not up")
                elif spec != node.spec:
                    self._stop_node(key, "the link's record changed")
                elif node.process.poll() is not None:
                    self._exited(key, node)
            for key in list(self._failed):
                if wanted.get(key) != self._failed[key]:
                    del self._failed[key]
                    daemon_records.remove_camera_node(*key.split("/"))
            now = self._clock()
            for spec in plan:
                key = _key(spec)
                if key in self._nodes or key in self._failed:
                    continue
                if now < self._restart_due.get(key, 0.0) or now - self._last_start < CAMERA_STAGGER_S:
                    continue
                # A start never lands inside a bring-up: the next tick tries again.
                if not daemon._port_work.acquire(blocking=False):
                    return
                try:
                    self._start_node(key, spec, key in openers)
                finally:
                    daemon._port_work.release()
                self._last_start = now

    def stop(self, reason: str = "nxsd stops") -> None:
        """Stop the reconciling and every node, last started first, `reason`
        in the journal; every record is removed."""
        self._halt.set()
        if self._ticker is not None:
            self._ticker.join()
            self._ticker = None
        with self._guard:
            for key in reversed(list(self._nodes)):
                self._stop_node(key, reason)
            for key in list(self._failed):
                daemon_records.remove_camera_node(*key.split("/"))
            self._failed.clear()
            self._restart_due.clear()

    def _tick(self, cfg) -> None:
        while not self._halt.wait(RECONCILE_S):
            try:
                self.reconcile(cfg)
            except Exception:           # noqa: BLE001 (the next tick tries again)
                log.exception("the camera nodes were not reconciled")

    def _start_node(self, key: str, spec: CameraTopic, opener: bool) -> None:
        port, link = key.split("/")
        try:
            argv = self._argv(spec)
        except LookupError as exc:
            # Not tried again until the link's record changes: the host lacks the node.
            self._failed[key] = spec
            daemon_records.write_camera_node(port, link, {"state": "failed", "line": str(exc).splitlines()[0],
                                                      "at": time.time()})
            log.warning("%s: no camera node (%s)", key, str(exc).splitlines()[0])
            return
        process = self._popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                              start_new_session=True)
        self._nodes[key] = _Node(key, spec, process, opener, self._clock(), self._on_line)
        daemon_records.write_camera_node(port, link, {"state": "running", "pid": process.pid,
                                                  "exits": len(self._exits.get(key, ())),
                                                  "at": time.time()})
        log.info("%s: camera node started (pid %s)", key, process.pid)

    def _exited(self, key: str, node: _Node) -> None:
        port, link = key.split("/")
        del self._nodes[key]
        node.reader.join(1.0)
        fails = self._exits.setdefault(key, collections.deque())
        now = self._clock()
        fails.append(now)
        while fails and now - fails[0] > FAILS_WINDOW_S:
            fails.popleft()
        rc = node.process.returncode
        if len(fails) >= FAILS_MAX:
            self._failed[key] = node.spec
            daemon_records.write_camera_node(port, link, {"state": "failed", "exits": len(fails),
                                                      "line": node.line, "at": time.time()})
            log.warning("%s: the camera node exited %d times in %g s; parked until the link's "
                        "record changes (rc %s: %s)", key, len(fails), FAILS_WINDOW_S, rc, node.line)
            return
        self._restart_due[key] = now + RESTART_S
        daemon_records.write_camera_node(port, link, {"state": "stopped", "exits": len(fails),
                                                  "line": node.line, "at": time.time()})
        log.warning("%s: the camera node exited (rc %s: %s); starting again in %g s", key, rc,
                    node.line, RESTART_S)

    def _stop_node(self, key: str, reason: str) -> None:
        node = self._nodes.pop(key)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            if node.process.poll() is not None:
                break
            try:
                node.process.send_signal(sig)
                node.process.wait(GRACE_S)
            except subprocess.TimeoutExpired:
                continue
            except OSError:
                break
        node.reader.join(1.0)
        daemon_records.remove_camera_node(*key.split("/"))
        log.info("%s: camera node stopped (%s)", key, reason)

    def _on_line(self, node: _Node, text: str) -> None:
        log.info("%s: %s", node.key, text)
        if node.opener and not node.opened and CAMERA_STARTED in text:
            node.opened = True
            self._open_port(node)

    def _open_port(self, node: _Node) -> None:
        """Restart the port's output under its first session, once: a node
        that starts beside a session delivering longer than CAMERA_OPEN_S
        meets that session's stream and needs nothing."""
        from nxs import daemon

        since = time.time()
        with self._guard:
            delivering = [other for other in self._nodes.values()
                          if other is not node and other.spec.port == node.spec.port
                          and self._clock() - other.started > CAMERA_OPEN_S]
        if delivering:
            return
        with daemon._port_work, port_state.held_wait():
            try:
                self._opener(node.spec.port, since)
            except Exception as exc:    # noqa: BLE001 (the journal names it; the node runs on)
                log.warning("%s: the output was not restarted under the camera node (%s)",
                            node.key, exc)
