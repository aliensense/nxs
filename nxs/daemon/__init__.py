# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""nxsd: the one resident NXS service. On start it runs the same idempotent
port construction the CLI uses (`nxs <port> on`, which builds the capture
stack's configuration the booted table lacks once its walk has the heads
streaming) for every nxs-owned hub port, then owns the data plane: unit
capture bridged to ROS 2 per the manifest, one camera node per camera link
that is up (`nxs.daemon.cameras`), the followers that keep each synced pair
at one gain (`nxs.daemon.follow`), the time discipline while it holds
(`nxs.daemon.pusher`) and the watch that brings a port up again when its
pods stop answering at their aliases (`nxs.daemon.watch`).
A reload (SIGHUP) brings up again every port whose declaration changed and
every port recorded up under another one, since the declaration is the
truth: a by-hand `on --mode` override is undone by a reload. Each port's
`on` leaves its verdict beside the build record, where `nxs switch` reads
it (`port_verdict`). A manifest out of tune at start, or none yet, holds
the daemon for a reload that brings one in tune."""

import argparse
import contextlib
import io
import logging
import os
import shutil
import signal
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

from nxs.daemon.build import BUILD_FILE, record_build, running_build, running_since
from nxs.daemon.environment import (ROS_ENVIRONMENT_KEYS, ROS_ENVIRONMENT_REQUIRED, SOURCE_ROS,  # noqa: F401 (the daemon's surface)
                                    record_environment, render_systemd_unit, ros_environment,
                                    running_environment, unit_environment, with_transport_default)
from nxs.daemon.plane import NO_ROS2, camera_plane, data_plane, hold_reason  # noqa: F401 (the daemon's surface)
from nxs.daemon.ports import (capture_stack as _capture_stack, hub_silent as _hub_silent,  # noqa: F401 (the daemon's surface)
                              left_down as _left_down, log_capture_stack as _log_capture_stack,
                              log_verified as _log_verified, owned as _owned,
                              runs_another_declaration as _runs_another_declaration)
from nxs.daemon.residents import Residents
from nxs.daemon.plane import (clear_camera_plane as _clear_camera_plane,
                              clear_data_plane as _clear_data_plane, clear_hold as _clear_hold,
                              record_data_plane as _record_data_plane, record_hold as _record_hold)
from nxs.daemon.verdict import BRINGING_UP, port_verdict, record_verdict, settle_stale, verdict_of

log = logging.getLogger("nxsd")

#: Set by the SIGHUP handler, read and cleared by the holds: state that
#: outlives a signal landing between a hold's check and its sleep. A plain
#: flag the holds poll, since the handler runs between two bytecodes of the
#: main thread and a lock it took (an Event's) may be one that thread holds.
_reload_flag: list = [False]
#: How often a hold looks at the flag.
RELOAD_POLL_S = 0.2

#: One piece of port work at a time across the daemon's threads: construction, a
#: reload's reconverge with the followers' stop and start, and the watch's tick.
_port_work = threading.RLock()


def _port_args(name: str, links=(), modes=()) -> argparse.Namespace:
    """`nxs <port> [<links>] on [--mode <mode> ...]` as the daemon runs it,
    one mode for every link or one per link in order, its lines in the
    journal."""
    return argparse.Namespace(topology=None, port=name, links=list(links),
                              mode=list(modes) or None, fps=None, dry_run=False,
                              cam_cmd="on", inner=True)


class _Reload(BaseException):
    """Raised out of the running data plane by the SIGHUP handler so the reload
    runs at once (a BaseException, so the bridge's `except Exception` guards
    let it through)."""


class _Terminate(BaseException):
    """Raised by the SIGTERM handler so the daemon ends through `main`'s
    cleanup: the followers stop and their heartbeats go. `unbuilt` names
    the ports a construction the stop ended left down, the ones that failed
    and the ones it skipped; None for a stop outside a construction."""

    def __init__(self, unbuilt: Optional[List[str]] = None) -> None:
        super().__init__()
        self.unbuilt = unbuilt


class _Tee(io.TextIOBase):
    """The standard output of a port's `on`: every write goes on to the
    journal and is kept for the port's verdict."""

    def __init__(self, stream) -> None:
        super().__init__()
        self._stream = stream
        self._kept = io.StringIO()

    def write(self, text: str) -> int:
        self._stream.write(text)
        self._kept.write(text)
        return len(text)

    def flush(self) -> None:
        self._stream.flush()

    def lines(self) -> List[str]:
        """What was written, a line each, without colour or blank lines."""
        from nxs.term import strip_ansi

        return [line for line in strip_ansi(self._kept.getvalue()).splitlines() if line.strip()]


#: True while the unit data plane runs in the main thread: a SIGHUP
#: then interrupts it. Elsewhere the handler only sets the flag.
_data_plane_running = False


def _on_hup(signum, frame):
    """SIGHUP: request the reload. The handler sets the flag and takes no
    lock (the journal line is the hold's, when it takes the request)."""
    _reload_flag[0] = True
    if _data_plane_running:
        raise _Reload()


def _reload_wait(timeout: Optional[float] = None) -> bool:
    """Sleep until a SIGHUP has set the flag, `timeout` seconds at most;
    whether it is set."""
    deadline = None if timeout is None else time.monotonic() + timeout
    while not _reload_flag[0]:
        if deadline is not None and time.monotonic() >= deadline:
            return False
        time.sleep(RELOAD_POLL_S)
    return True


def _take_reload() -> bool:
    """Whether a SIGHUP requested a reload since the last look, the request
    cleared: a SIGHUP after the look is the next one's."""
    if not _reload_flag[0]:
        return False
    _reload_flag[0] = False
    log.info("SIGHUP: manifest reload requested")
    return True


#: The port work the main thread is inside (`_deferring_term`), and a SIGTERM held until it ends.
#: The watch thread's bring-up ends under `_port_work` before the daemon's exit takes the lock.
_deferrals = 0
_term_pending = False


def _on_term(signum, frame):
    """SIGTERM: end the daemon through `main`'s cleanup, after the port work
    it is on."""
    global _term_pending
    if _deferrals:
        _term_pending = True
        return
    raise _Terminate()


@contextlib.contextmanager
def _deferring_term():
    """Port work on the main thread a SIGTERM must not cut short: the stop
    lands when the work ends."""
    global _deferrals, _term_pending
    if threading.current_thread() is not threading.main_thread():
        yield
        return
    _deferrals += 1
    try:
        yield
    finally:
        _deferrals -= 1
    if _deferrals == 0 and _term_pending:
        _term_pending = False
        log.info("SIGTERM: stopping after the port work")
        raise _Terminate()


def _load_checked():
    """The manifest, loaded and validated; None (with logs) when unusable."""
    cfg = _loaded()
    if cfg is None or _out_of_tune(cfg):
        return None
    return cfg


def _loaded():
    """The manifest, loaded against the shipped schema; None (with logs)
    when there is none at the path or it is rejected."""
    from nxs.cam import hubs
    from nxs.check import shape_findings
    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config
    from nxs.suite.switch import manifest_present

    # Hub discovery is cached per process while the personality store is written
    # at run time; reset so a personality installed after start is visible.
    hubs.reset_cache()
    path = default_config_path()
    if not manifest_present(path):
        log.error("no manifest at %s", path)
        return None
    # The same contract as `nxs status`: the shipped schema first, so a
    # manifest the checker rejects is never one the daemon operates.
    shape = shape_findings(path)
    if shape:
        for f in shape:
            log.error("manifest rejected: %s", f)
        return None
    try:
        return load_suite_config(path)
    except (ManifestError, OSError) as e:
        log.error("manifest rejected: %s", e)
        return None


def _out_of_tune(cfg) -> list:
    """The manifest's findings against this host, as `nxs status` judges
    it, each in the journal; empty when it is in tune."""
    from nxs.check import check_config

    findings = check_config(cfg)
    for f in findings:
        log.error("out of tune: %s", f)
    return findings


def resolve_nxsd_path(argv0: str, which=shutil.which) -> Optional[str]:
    """Absolute path of the nxsd entry point: beside the invoked nxs (the
    same install), else on PATH; None when neither has it."""
    beside = os.path.join(os.path.dirname(os.path.abspath(argv0)), "nxsd")
    if os.path.exists(beside):
        return beside
    return which("nxsd")


def _relinquishes(before, after) -> bool:
    """The new declaration does not own what the old one built: the hub is
    kernel-driven, the port's nodes changed (a hub came or went), or the port
    sits on another bus or behind another hub."""
    return (not _owned(after) or before.bus != after.bus
            or before.hub_compatible != after.hub_compatible
            or before.hub_addr != after.hub_addr)


def _stopped_short(name: str) -> bool:
    """Whether the last `on` the daemon ran on the port stopped short of up
    (`port_verdict`): a reload runs it again, so the `nxs switch` that sent
    the reload reads a verdict of its own."""
    verdict = port_verdict(name)
    return verdict is not None and verdict["verdict"] not in ("up", BRINGING_UP)


def _converge(cfg, previous) -> List[str]:
    """Reconstruct the ports whose declaration changed, the ports recorded
    up under another declaration, the ports whose capture stack has no
    configuration for the booted table (their `on` builds it), the ports
    whose last `on` stopped short and the ports left down, and park the
    ports the new manifest does not declare; returns the names of the
    ports that did not converge, retained and retried on the next reload."""
    from nxs.suite.schema_nodes import changed_nodes, port_signature

    changed = [n for n in sorted(cfg.ports)
               if previous is None
               or n not in previous.ports
               or port_signature(cfg.ports[n])
               != port_signature(previous.ports[n])
               or _runs_another_declaration(cfg.ports[n])
               or _capture_stack(n, cfg.ports[n])[0] == "missing"
               or _stopped_short(n)
               or _left_down(cfg.ports[n])]
    removed = [n for n in sorted(previous.ports)
               if n not in cfg.ports] if previous is not None else []
    if not changed and not removed:
        log.info("ports unchanged")
        return []
    from nxs.cam import cli as cam_cli
    from nxs.cam import port_state
    from nxs.cam import topology as cam_topo

    failed: List[str] = []
    # A bus another run holds is waited for, as `status` waits, on every
    # lock a step takes; a wait outlasted fails that port's step.
    with _port_work, port_state.held_wait():
        for name in removed:
            port = previous.ports[name]
            if not _owned(port):
                continue
            # The new manifest does not name this port, so the park takes the port
            # built from its previous declaration.
            try:
                topology = cam_topo.port_topology(port)
                rc = cam_cli.park_port(topology, list(topology.links))
                log.info("port %s: undeclared, parked %s", name,
                         "ok" if rc == 0 else f"rc={rc}")
            except SystemExit as e:
                rc = 1
                log.warning("port %s: %s", name, e)
            except Exception:
                rc = 1
                log.exception("port %s: parking failed", name)
            if rc != 0:
                failed.append(name)
        for name in changed:
            port = cfg.ports[name]
            before = previous.ports.get(name) if previous is not None else None
            # The journal names the nodes whose declaration moved.
            if moved := (changed_nodes(before, port) if before is not None else []):
                log.info("port %s reconverges: %s changed", name, ", ".join(moved))
            if before is not None and _owned(before) and _relinquishes(before, port):
                # The port built under the old declaration is not the new one's:
                # park it so nothing keeps streaming behind the change.
                try:
                    topology = cam_topo.port_topology(before)
                    rc = cam_cli.park_port(topology, list(topology.links))
                    log.info("port %s: previous port parked %s", name,
                             "ok" if rc == 0 else f"rc={rc}")
                except SystemExit as e:
                    rc = 1
                    log.warning("port %s: %s", name, e)
                except Exception:
                    rc = 1
                    log.exception("port %s: parking the previous port failed", name)
                if rc != 0:
                    failed.append(name)
                    continue
            if not _owned(port):
                continue
            unbuilt = _capture_stack(name, port)[0] == "missing"
            rc = _bring_up(name, "reconverged", "reconvergence failed")
            if unbuilt:
                _log_capture_stack(name, port)
            if rc != 0:
                failed.append(name)
            else:
                _log_verified(name, port)
    return failed


def _retained(fresh, previous, failed: List[str]):
    """The configuration the daemon holds after a reload: the fresh one,
    with every port that did not converge kept as it was (or absent when
    it was new), so the next reload sees it changed and retries."""
    if previous is not None:
        for name in failed or []:
            if name in previous.ports:
                fresh.ports[name] = previous.ports[name]
            else:
                fresh.ports.pop(name, None)
    return fresh


def _bring_up(name: str, done: str, failed: str, args: Optional[argparse.Namespace] = None,
              kept: Optional[dict] = None) -> int:
    """The port's `on`, as `nxs <port> on` runs it (`args` names another
    form, a link at a row): its lines go to the journal, the outcome is
    logged as `port <name>: <done> ok` (or `rc=N`), and the verdict is
    recorded for `nxs switch` (`port_verdict`). `kept`, a recorded verdict,
    stands in place of `up` when the run comes up: a walk back to a link's
    recorded row leaves the refusal of the walk it undoes."""
    from nxs.cam import cli as cam_cli
    from nxs.cam.contracts import InfeasibleConfig

    printed = _Tee(sys.stdout)
    lines: List[str] = []
    # The run is recorded before it starts: a `switch` that finds the port
    # being brought up waits for its verdict instead of racing it for the bus.
    record_verdict(name, BRINGING_UP, [])
    try:
        with contextlib.redirect_stdout(printed):
            rc = cam_cli.cmd_up(args if args is not None else _port_args(name))
        # The verb's lines reach the journal before the line that sums them up.
        printed.flush()
        if rc == 0:
            log.info("port %s: %s ok", name, done)
        else:
            log.warning("port %s: %s rc=%d, %s", name, done, rc, verdict_of(rc, printed.lines())[0])
    except InfeasibleConfig as exc:
        # The laws' refusal, as the verb prints it at the command line.
        rc = 2
        lines = [exc.reason, *(f"  - {alt}" for alt in exc.alternatives)]
        log.warning("port %s: %s", name, exc)
    except SystemExit as e:
        rc = 1
        lines = e.code.splitlines() if isinstance(e.code, str) else []
        log.warning("port %s: %s", name, e)
    except Exception as exc:
        rc = 1
        lines = [f"{name}: {failed} ({exc})", "  - journalctl -u nxsd"]
        log.exception("port %s: %s", name, failed)
    if rc == 0 and kept is not None:
        record_verdict(name, kept["verdict"], kept["lines"])
    else:
        record_verdict(name, *verdict_of(rc, lines or printed.lines()))
    return rc


#: How long a port's first construction waits for a hub that does not
#: answer yet (one powered after the host is silent at the daemon's start),
#: and how often it is tried again meanwhile.
CONSTRUCT_WAIT_S = 60.0
CONSTRUCT_RETRY_S = 5.0


def _construct(name: str, port) -> int:
    """The port's first bring-up, tried again every CONSTRUCT_RETRY_S while
    it fails on a hub that does not answer, CONSTRUCT_WAIT_S at most; a
    stop ends the tries."""
    deadline = time.monotonic() + CONSTRUCT_WAIT_S
    while True:
        rc = _bring_up(name, "construction", "construction failed")
        if rc == 0 or _term_pending or time.monotonic() >= deadline or not _hub_silent(port):
            return rc
        log.info("port %s: the hub does not answer; constructing again in %g s",
                 name, CONSTRUCT_RETRY_S)
        if _stop_waited(CONSTRUCT_RETRY_S):
            return rc


def _stop_waited(seconds: float) -> bool:
    """Sleep `seconds`, ended early by a SIGTERM held for the port work;
    whether one is held."""
    deadline = time.monotonic() + seconds
    while not _term_pending and time.monotonic() < deadline:
        time.sleep(max(0.0, min(RELOAD_POLL_S, deadline - time.monotonic())))
    return _term_pending


def _construct_ports(cfg) -> List[str]:
    """Construct every nxs-driven port the manifest declares, each through
    its `on`, which builds the capture stack's configuration the booted
    table lacks once the port's heads stream; returns the names of the
    ports that did not come up (the oneshot mode's exit status, and the
    deploy's gate before it flashes units behind them)."""
    from nxs.cam import port_state

    failed: List[str] = []
    try:
        # A bus another run holds is waited for, as `status` waits.
        with _deferring_term(), _port_work, port_state.held_wait():
            for name in sorted(cfg.ports):
                port = cfg.ports[name]
                if not _owned(port):
                    continue
                if _term_pending:
                    log.info("SIGTERM: port %s waits for the next start", name)
                    failed.append(name)
                    continue
                unbuilt = _capture_stack(name, port)[0] == "missing"
                rc = _construct(name, port)
                if unbuilt:
                    _log_capture_stack(name, port)
                if rc != 0:
                    failed.append(name)
                else:
                    _log_verified(name, port)
    except _Terminate as stop:
        stop.unbuilt = failed
        raise
    return failed


def _ros2_available(importer=None) -> bool:
    """Whether the bridge's ROS 2 client library imports in this process,
    with the message types it publishes. The import itself, not a lookup:
    an rclpy the path finds but whose libraries it does not would exit the
    bridge into the unit's restart loop, a port construction each time."""
    import importlib

    importer = importer or importlib.import_module
    try:
        importer("rclpy")
        importer("sensor_msgs.msg")
    except Exception:  # noqa: BLE001 (a broken install holds, never exits)
        return False
    return True


def _reload(fresh, cfg, followers=None):
    """The configuration the daemon holds once the changed ports reconverge,
    the watch's from then on. The followers stop first, since a bring-up
    reprograms the heads they write, and start again on what the daemon
    holds; the port work is held throughout, so the watch's own bring-up
    never interleaves."""
    with _deferring_term():
        from nxs.daemon import watch

        with _port_work:
            if followers is not None:
                followers.stop("nxsd reloads")
            try:
                cfg = _retained(fresh, cfg, _converge(fresh, cfg))
            finally:
                if followers is not None:
                    followers.start(cfg)
        watch.serve(cfg)
        return cfg


#: How long a daemon whose data plane stopped holds before it tries the plane again: a unit that
#: restarts its personality after a camera run serves its descriptors seconds later.
DATA_PLANE_RETRY_S = 30.0


def _hold_then_retry(cfg, residents, wait=_reload_wait, retry_s: float = DATA_PLANE_RETRY_S):
    """Hold the ports, the units disciplined by the daemon's own pusher,
    until a reload lands or `retry_s` passes, and return the configuration
    the data plane tries again with."""
    from nxs.daemon.pusher import Pusher

    pusher = Pusher()
    _join(residents, pusher, cfg)
    try:
        deadline = time.monotonic() + retry_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            wait(remaining)
            if not _take_reload():
                continue
            fresh = _load_checked()
            if fresh is None:
                log.error("keeping the previous configuration")
                continue
            cfg = _reload(fresh, cfg, residents)
            break
    finally:
        _leave(residents, pusher, "the data plane tries again")
    return cfg


def _join(residents, member, cfg) -> None:
    """`member` added to the residents and started, under the port work: a
    walk of the modes resident stops and starts the other members, and
    meets this one either before it joins or after."""
    with _port_work:
        residents.add(member)
        member.start(cfg)


def _leave(residents, member, reason: str) -> None:
    """`member` stopped and taken out of the residents, under the port work,
    so a walk never starts it again once it left."""
    with _port_work:
        member.stop(reason)
        residents.remove(member)


def _hold_for_reload(cfg, wait=_reload_wait, followers=None) -> int:
    """No unit data plane to run: hold the process for SIGHUP and reconverge
    the changed ports on each one."""
    while True:
        wait()
        if not _take_reload():
            continue
        fresh = _load_checked()
        if fresh is None:
            log.error("keeping the previous configuration")
            continue
        cfg = _reload(fresh, cfg, followers)


def _await_in_tune(cfg=None, findings=(), wait=_reload_wait):
    """Hold for SIGHUP until a manifest loads and is in tune, and return it;
    `cfg` and `findings` are the manifest and the findings a check at start
    ended on, None with no manifest yet. The build is recorded first, so
    `nxs switch` reloads this daemon instead of restarting it, and each check
    that finds the manifest out of tune records the findings as every owned
    port's verdict, which that switch ends on."""
    from nxs.suite import default_config_path
    from nxs.suite.switch import manifest_present

    record_build()
    why = f"no manifest at {default_config_path()}" if cfg is None else "the manifest is out of tune"
    while True:
        if cfg is not None:
            _refuse_owned(cfg, findings)
        log.info("%s; holding for SIGHUP", why)
        while not _take_reload():
            wait()
        cfg, findings = None, ()
        path = default_config_path()
        if not manifest_present(path):
            why = f"no manifest at {path}"
            continue
        cfg = _loaded()
        if cfg is None:
            why = "the manifest is rejected"
            continue
        findings = _out_of_tune(cfg)
        if not findings:
            return cfg
        why = "the manifest is out of tune"


def _refuse_owned(cfg, findings) -> None:
    """Record the manifest's findings as every owned port's verdict: the
    daemon brings none of them up while it holds."""
    lines = [line for finding in findings for line in f"out of tune: {finding}".splitlines()]
    for name in sorted(cfg.ports):
        if _owned(cfg.ports[name]):
            record_verdict(name, f"refused: {lines[0]}", lines)


def main(argv=None) -> int:
    from nxs.suite import default_config_path
    from nxs.suite.switch import manifest_present

    parser = argparse.ArgumentParser(
        prog="nxsd", description="Aliensense NXS daemon")
    parser.add_argument("--no-construct", action="store_true",
                        help="skip port construction at start")
    parser.add_argument("--construct-only", action="store_true",
                        help="construct ports, then exit (oneshot mode)")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO,
                        format="%(name)s: %(message)s")

    global _term_pending
    _term_pending = False
    signal.signal(signal.SIGHUP, _on_hup)
    signal.signal(signal.SIGTERM, _on_term)
    _clear_hold()
    try:
        for name in settle_stale():
            log.info("port %s: the daemon restarted during its bring-up; recorded stopped", name)
        cfg = _loaded()
        if cfg is None:
            # A rejected manifest is the service's failure and the oneshot's; with
            # none yet (a fresh host before `nxs generate`) the service holds for one.
            if args.construct_only or manifest_present(default_config_path()):
                return 1
            cfg = _await_in_tune()
        else:
            findings = _out_of_tune(cfg)
            if findings:
                # The oneshot's status is the manifest's; the service holds for one in tune.
                if args.construct_only:
                    return 1
                cfg = _await_in_tune(cfg, findings)
        failed = _construct_ports(cfg) if not args.no_construct else []
    except _Terminate as stop:
        log.info("SIGTERM: stopping")
        # The oneshot's status is the construction's: a stop before it, or
        # one that left a port down, is no success.
        if args.construct_only and stop.unbuilt is None:
            log.error("SIGTERM before the construction: no port was built")
            return 1
        if args.construct_only and stop.unbuilt:
            log.error("construction stopped short for: %s", ", ".join(stop.unbuilt))
            return 1
        return 0
    # The record lands after the construction: `nxs switch` reads it as the
    # restarted daemon's readiness before its pod steps reach through the ports.
    record_build()
    if args.construct_only:
        # The oneshot's status is the construction's: a deploy flashing
        # units behind these ports must not proceed on a false success.
        if failed:
            log.error("construction failed for: %s", ", ".join(failed))
            return 1
        return 0

    from nxs.daemon import watch
    from nxs.daemon.follow import Followers
    from nxs.daemon.modes import Modes

    residents = Residents(Followers())
    # A walk of the modes resident is a bring-up: the other residents stop for its length.
    residents.add(Modes(residents=residents))
    with _port_work:
        residents.start(cfg)
    # The ports are up: the watch brings one whose pods lost their aliases up again.
    watch.start(cfg, residents)
    try:
        return _serve(cfg, residents)
    except _Terminate:
        log.info("SIGTERM: stopping")
        return 0
    finally:
        watch.stop()
        # A bring-up the watch or a walk runs ends before the residents stop,
        # and none starts while they do.
        with _port_work:
            residents.stop()
        _clear_hold()
        _clear_data_plane()
        _clear_camera_plane()


def _serve(cfg, residents) -> int:
    """The daemon past construction: the unit data plane, or a hold for
    the reloads that reconverge the ports. The units are disciplined
    either way: by the bridge while it serves them, by the daemon's own
    pusher while it holds."""
    from nxs.daemon.pusher import Pusher

    ros2 = _ros2_available()
    if ros2:
        # One camera node runs per up camera link, beside the bridge or a unitless hold, reconciled
        # against the port records; the resident records the links it serves, so a launch adds none.
        from nxs.daemon.cameras import Cameras

        _join(residents, Cameras(), cfg)
    if not cfg.units:
        log.info("no units declared; holding for SIGHUP (port reconvergence)")
        return _hold_for_reload(cfg, followers=residents)
    if not ros2:
        # The ports are up and the bridge has no ROS 2 to publish on: hold them rather than exit
        # into a restart loop that rebuilds them, and say so where `nxs status` reads it.
        log.error("no ROS 2 environment for the unit bridge (the unit carries none: "
                  "source the distribution, then nxs switch); holding the ports for SIGHUP")
        _record_hold(NO_ROS2)
        _join(residents, Pusher(), cfg)
        return _hold_for_reload(cfg, followers=residents)
    _clear_hold()

    # The data plane is the existing suite->ROS2 bridge, run in-process;
    # a SIGHUP between bridge exits reconverges changed ports only.
    from nxs.cli import build_parser, cmd_ros2
    global _data_plane_running
    while True:
        ros2_args = build_parser().parse_args(["ros2"])
        stopped = None
        try:
            # The record names the units this daemon bridges: an operator's bridge or launch
            # defers to it instead of doubling every node.
            _record_data_plane([unit.name for unit in cfg.units])
            # A SIGHUP interrupts the bridge from here on; before, the handler only sets the flag
            # read after the bridge, since a reload raised out of the record above ends the daemon.
            _data_plane_running = True
            # The bridge serves the configuration this process validated: after
            # a rejected reload it restarts on the retained one.
            rc = cmd_ros2(ros2_args, cfg=cfg)
            if rc:
                stopped = f"exit {rc}"
        except _Reload:
            rc = 0
            log.info("data plane interrupted for the reload")
        except SystemExit as e:
            rc = e.code if isinstance(e.code, int) else 1
            stopped = str(e) or f"exit {rc}"
            log.error("data plane exited: %s", e)
        except Exception as e:  # noqa: BLE001 (any bridge failure holds the ports)
            rc = 1
            stopped = f"{type(e).__name__}: {e}"
            log.exception("data plane failed")
        finally:
            _data_plane_running = False
            _clear_data_plane()
        if not _take_reload():
            if stopped is None:
                return rc
            # An exit would restart the unit into a construction of every
            # port: hold them instead, disciplined, say why, and try again.
            _record_hold(f"the data plane stopped: {stopped.splitlines()[0]}")
            cfg = _hold_then_retry(cfg, residents)
            _clear_hold()
            continue
        fresh = _load_checked()
        if fresh is None:
            log.error("keeping the previous configuration")
            continue
        cfg = _reload(fresh, cfg, residents)


if __name__ == "__main__":
    sys.exit(main())
