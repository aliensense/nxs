# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""nxsd: the one resident NXS service. On start it runs the same idempotent
port construction the CLI uses (`nxs <port> on`, which builds the capture
stack's configuration the booted table lacks once its walk has the heads
streaming) for every nxs-owned hub port, then owns the data plane: unit
capture bridged to ROS 2 per the manifest, the followers that keep each
synced pair at one gain (`nxs.daemon.follow`) and the watch that brings a
port up again when its pods stop answering at their aliases
(`nxs.daemon.watch`).
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
from typing import List, Optional, Tuple

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


def _port_args(name: str) -> argparse.Namespace:
    """`nxs <port> on` as the daemon runs it, its lines in the journal."""
    return argparse.Namespace(topology=None, port=name, links=[], mode=None, fps=None,
                              dry_run=False, cam_cmd="on", inner=True)


class _Reload(BaseException):
    """Raised out of the running data plane by the SIGHUP handler so the reload
    runs at once (a BaseException, so the bridge's `except Exception` guards
    let it through)."""


class _Terminate(BaseException):
    """Raised by the SIGTERM handler so the daemon ends through `main`'s
    cleanup: the followers stop and their heartbeats go."""


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


def _on_term(signum, frame):
    """SIGTERM: end the daemon through `main`'s cleanup."""
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


def render_systemd_unit(exec_path: str, user: Optional[str],
                        capture_daemon: Optional[str] = None) -> str:
    """The nxsd unit with an absolute entry point (systemd's PATH lacks ~/.local/bin),
    ordered after the host's capture daemon. It runs as root: the boot table,
    the capture stack's configuration and the ports are root's to touch, and
    the state store under /var/lib is shared with the operator's shell."""
    user_line = f"User={user}\n" if user else ""
    after = "network.target" + (f" {capture_daemon}" if capture_daemon else "")
    return ("[Unit]\n"
            "Description=Aliensense NXS daemon (port construction + unit data plane)\n"
            f"After={after}\n"
            "\n"
            "[Service]\n"
            f"{user_line}"
            f"ExecStart={exec_path}\n"
            "Restart=on-failure\n"
            "RestartSec=5\n"
            "\n"
            "[Install]\n"
            "WantedBy=multi-user.target\n")


def _owned(port) -> bool:
    """A port this daemon programs: a hub declared with the nxs driver, or a
    sensor on the port's own bus. A link there runs its personality on its
    unit, or on the host when it declares none, so every such port with a
    link is the daemon's to bring up at boot."""
    if port.hub_compatible is not None:
        return port.hub_driver == "nxs"
    return bool(port.links)


def _relinquishes(before, after) -> bool:
    """The new declaration does not own what the old one built: the hub is
    kernel-driven, the port's nodes changed (a hub came or went), or the port
    sits on another bus or behind another hub."""
    return (not _owned(after) or before.bus != after.bus
            or before.hub_compatible != after.hub_compatible
            or before.hub_addr != after.hub_addr)


def _runs_another_declaration(port) -> bool:
    """Whether an owned port is recorded up under a declaration other than
    `port`: its record carries another digest, or none (a record another
    build wrote, or an `on` that departed from the declaration). A port
    the host cannot shape is left to its `on`, which says why."""
    from nxs.cam import port_state
    from nxs.cam import topology as cam_topo

    if not _owned(port):
        return False
    try:
        topology = cam_topo.port_topology(port)
    except Exception:        # noqa: BLE001 (the port's `on` refuses it with the reason)
        return False
    if not any(port_state.link_state(topology, link) == port_state.STATE_UP
               for link in topology.links):
        return False
    return port_state.port_record(topology).get("declared") != topology.declared


def _left_down(port) -> bool:
    """Whether an owned port has a link a step recorded this boot neither up
    nor parked: a bring-up that did not verify, or a port `nxs switch`
    recorded down to come up again (a pod took a new build, its hub lost
    the aliases, or the port was parked). A port `off` parked is the
    operator's, and one no step recorded is the construction's."""
    from nxs.cam import port_state
    from nxs.cam import topology as cam_topo

    if not _owned(port):
        return False
    try:
        topology = cam_topo.port_topology(port)
    except Exception:        # noqa: BLE001 (the port's `on` refuses it with the reason)
        return False
    states = set(port_state.recorded_states(topology).values())
    return port_state.STATE_UNKNOWN in states and port_state.STATE_PARKED not in states


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
    from nxs.suite.schema_ports import port_signature

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


def _capture_stack(name: str, port) -> Tuple[Optional[str], int]:
    """The port's capture-stack state for the booted table (`ready`,
    `preparing`, `missing`) with the table's mode count; (None, 0) for a
    port that streams no video, or one the host cannot shape (its `on`
    says why). A missing configuration is the port's `on` to build, once
    its walk has the heads streaming."""
    from nxs import host as host_layer
    from nxs.cam import topology as cam_topo
    from nxs.host.cli import _table_rows, capture_stack_state

    if not _owned(port) or not any(link.camera for link in port.links):
        return None, 0
    try:
        topology = cam_topo.port_topology(port)
    except Exception:        # noqa: BLE001 (the port's `on` refuses it with the reason)
        return None, 0
    host = host_layer.current()
    return capture_stack_state(host, name, topology), len(_table_rows(host, topology.i2c_bus))


def _log_capture_stack(name: str, port) -> None:
    """After the `on` of a port whose capture stack had no configuration
    for the booted table: the one it built, in the journal, or that none
    was (the verb's refusal above it says why)."""
    state, modes = _capture_stack(name, port)
    if state == "ready":
        log.info("port %s: capture stack built (%d modes)", name, modes)
    else:
        log.error("port %s: the capture stack's configuration was not built", name)


def _log_verified(name: str, port) -> None:
    """The delivery check the port's `on` passed, in the journal."""
    from nxs.cam import port_state
    from nxs.cam import topology as cam_topo
    from nxs.cam.verbs import verify

    topology = cam_topo.port_topology(port)
    seen = port_state.verified(topology)
    if seen:
        log.info("port %s: verified %s", name, verify.summary(topology, seen["rates"]))


def _bring_up(name: str, done: str, failed: str) -> int:
    """The port's `on`, as `nxs <port> on` runs it: its lines go to the
    journal, the outcome is logged as `port <name>: <done> ok` (or `rc=N`),
    and the verdict is recorded for `nxs switch` (`port_verdict`)."""
    from nxs.cam import cli as cam_cli
    from nxs.cam.contracts import InfeasibleConfig

    printed = _Tee(sys.stdout)
    lines: List[str] = []
    # The run is recorded before it starts: a `switch` that finds the port
    # being brought up waits for its verdict instead of racing it for the bus.
    record_verdict(name, BRINGING_UP, [])
    try:
        with contextlib.redirect_stdout(printed):
            rc = cam_cli.cmd_up(_port_args(name))
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
    record_verdict(name, *verdict_of(rc, lines or printed.lines()))
    return rc


#: How long a port's first construction waits for a hub that does not
#: answer yet (one powered after the host is silent at the daemon's start),
#: and how often it is tried again meanwhile.
CONSTRUCT_WAIT_S = 60.0
CONSTRUCT_RETRY_S = 5.0


def _hub_silent(port) -> bool:
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


def _construct(name: str, port) -> int:
    """The port's first bring-up, tried again every CONSTRUCT_RETRY_S while
    it fails on a hub that does not answer, CONSTRUCT_WAIT_S at most."""
    deadline = time.monotonic() + CONSTRUCT_WAIT_S
    while True:
        rc = _bring_up(name, "construction", "construction failed")
        if rc == 0 or time.monotonic() >= deadline or not _hub_silent(port):
            return rc
        log.info("port %s: the hub does not answer; constructing again in %g s",
                 name, CONSTRUCT_RETRY_S)
        time.sleep(CONSTRUCT_RETRY_S)


def _construct_ports(cfg) -> List[str]:
    """Construct every nxs-driven port the manifest declares, each through
    its `on`, which builds the capture stack's configuration the booted
    table lacks once the port's heads stream; returns the names of the
    ports that did not come up (the oneshot mode's exit status, and the
    deploy's gate before it flashes units behind them)."""
    from nxs.cam import port_state

    failed: List[str] = []
    # A bus another run holds is waited for, as `status` waits.
    with _port_work, port_state.held_wait():
        for name in sorted(cfg.ports):
            port = cfg.ports[name]
            if not _owned(port):
                continue
            unbuilt = _capture_stack(name, port)[0] == "missing"
            rc = _construct(name, port)
            if unbuilt:
                _log_capture_stack(name, port)
            if rc != 0:
                failed.append(name)
            else:
                _log_verified(name, port)
    return failed


def _ros2_available() -> bool:
    """Whether the bridge's ROS 2 client library imports in this process."""
    import importlib.util

    return importlib.util.find_spec("rclpy") is not None


def _reload(fresh, cfg, followers=None):
    """The configuration the daemon holds once the changed ports reconverge,
    the watch's from then on. The followers stop first, since a bring-up
    reprograms the heads they write, and start again on what the daemon
    holds; the port work is held throughout, so the watch's own bring-up
    never interleaves."""
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

    _record_build()
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


#: Where a running daemon records the build it runs, beside the port record.
BUILD_FILE = "nxsd.build"
def running_build() -> Optional[str]:
    """The build the running daemon recorded at start, None when none did."""
    from nxs.cam import port_state

    try:
        return (port_state.state_dir() / BUILD_FILE).read_text().strip() or None
    except OSError:
        return None


def _record_build() -> None:
    """Record this daemon's build once its ports are constructed, or once it
    holds for a manifest in tune: `nxs switch` restarts a daemon that a
    newer wheel left running (a reload changes the manifest only) and
    waits for the record to say so."""
    from nxs.cam import port_state
    from nxs.cli import _git_version

    try:
        (port_state.state_dir() / BUILD_FILE).write_text(_git_version() + "\n")
    except OSError as exc:
        log.warning("the build record was not written: %s", exc)


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

    signal.signal(signal.SIGHUP, _on_hup)
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
    # The record lands after the construction: `nxs switch` reads it as the
    # restarted daemon's readiness before its pod steps reach through the ports.
    _record_build()
    if args.construct_only:
        # The oneshot's status is the construction's: a deploy flashing
        # units behind these ports must not proceed on a false success.
        if failed:
            log.error("construction failed for: %s", ", ".join(failed))
            return 1
        return 0

    from nxs.daemon import watch
    from nxs.daemon.follow import Followers

    followers = Followers()
    followers.start(cfg)
    signal.signal(signal.SIGTERM, _on_term)
    # The ports are up: the watch brings one whose pods lost their aliases up again.
    watch.start(cfg, followers)
    try:
        return _serve(cfg, followers)
    except _Terminate:
        log.info("SIGTERM: stopping")
        return 0
    finally:
        watch.stop()
        followers.stop()


def _serve(cfg, followers) -> int:
    """The daemon past construction: the unit data plane, or a hold for
    the reloads that reconverge the ports."""
    if not cfg.units:
        log.info("no units declared; holding for SIGHUP (port reconvergence)")
        return _hold_for_reload(cfg, followers=followers)
    if not _ros2_available():
        # The ports are up; the bridge has no ROS 2 to publish on. Hold
        # them rather than exit into a restart loop that rebuilds them.
        log.error("no ROS 2 environment for the unit bridge (nxs ros2 needs: pip install "
                  "'aliensense-nxs[ros2]', inside a sourced ROS 2 environment); "
                  "holding the ports for SIGHUP")
        return _hold_for_reload(cfg, followers=followers)
    # The data plane is the existing suite->ROS2 bridge, run in-process;
    # a SIGHUP between bridge exits reconverges changed ports only.
    from nxs.cli import build_parser, cmd_ros2
    global _data_plane_running
    while True:
        ros2_args = build_parser().parse_args(["ros2"])
        _data_plane_running = True
        try:
            # The bridge serves the configuration this process validated: after
            # a rejected reload it restarts on the retained one.
            rc = cmd_ros2(ros2_args, cfg=cfg)
        except _Reload:
            rc = 0
            log.info("data plane interrupted for the reload")
        except SystemExit as e:
            rc = e.code if isinstance(e.code, int) else 1
            log.error("data plane exited: %s", e)
        finally:
            _data_plane_running = False
        if not _take_reload():
            return rc
        fresh = _load_checked()
        if fresh is None:
            log.error("keeping the previous configuration")
            continue
        cfg = _reload(fresh, cfg, followers)


if __name__ == "__main__":
    sys.exit(main())
