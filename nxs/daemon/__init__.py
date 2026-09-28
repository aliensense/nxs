# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""nxsd: the one resident NXS service. On start it runs the same idempotent
port construction the CLI uses (`nxs <port> on`) for every nxs-owned hub port,
then owns the data plane: unit capture bridged to ROS 2 per the manifest."""

import argparse
import logging
import os
import shutil
import signal
import sys
from typing import List, Optional

log = logging.getLogger("nxsd")

_reload_requested = False


class _Reload(BaseException):
    """Raised out of the running data plane by the SIGHUP handler so the reload
    runs at once (a BaseException, so the bridge's `except Exception` guards
    let it through)."""


#: True while the unit data plane runs in the main thread: a SIGHUP
#: then interrupts it. Elsewhere the handler only sets the flag.
_data_plane_running = False


def _on_hup(signum, frame):
    global _reload_requested
    _reload_requested = True
    log.info("SIGHUP: manifest reload requested")
    if _data_plane_running:
        raise _Reload()


def _load_checked():
    """The manifest, loaded and validated; None (with logs) when unusable."""
    from nxs.cam import packs
    from nxs.check import check_config, shape_findings
    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config

    # Pack discovery is cached per process while the personality store is written
    # at run time; reset so a personality installed after start is visible.
    packs.reset_cache()
    path = default_config_path()
    # The same contract as `nxs status`: the shipped schema first, so a
    # manifest the checker rejects is never one the daemon operates.
    shape = shape_findings(path)
    if shape:
        for f in shape:
            log.error("manifest rejected: %s", f)
        return None
    try:
        cfg = load_suite_config(path)
    except (ManifestError, OSError) as e:
        log.error("manifest rejected: %s", e)
        return None
    findings = check_config(cfg)
    if findings:
        for f in findings:
            log.error("out of tune: %s", f)
        return None
    return cfg


def _port_signature(port):
    """Every declared field that shapes the port's construction; a change
    to any of them makes the port reconverge on SIGHUP."""
    return (port.bus, port.hub_compatible, port.hub_addr, port.hub_driver,
            port.csi_lanes, port.csi_lanes_declared,
            port.camera_mode, port.camera_fps, port.camera_sensor,
            port.sync_source, port.sync_fps,
            tuple((l.name, l.camera, l.camera_mode, l.ser, l.ser_addr,
                   l.sensor_addr, l.tca_addr, l.des_window, l.csi_vc,
                   l.capture_id,
                   (l.unit.name, l.unit.alias, l.unit.target)
                   if l.unit else None)
                  for l in port.links))


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
    sensor on the port's own bus whose links all carry a unit to run it. A
    link with no unit is nobody's to bring up at boot (the source-tree
    emulator serves it by hand), whatever the environment says."""
    if port.hub_compatible is not None:
        return port.hub_driver == "nxs"
    return bool(port.links) and all(link.unit is not None for link in port.links)


def _relinquishes(before, after) -> bool:
    """The new declaration does not own what the old one built: the hub is
    kernel-driven, the port's nodes changed (a hub came or went), or the port
    sits on another bus or behind another hub."""
    return (not _owned(after) or before.bus != after.bus
            or before.hub_compatible != after.hub_compatible
            or before.hub_addr != after.hub_addr)


def _converge(cfg, previous) -> List[str]:
    """Reconstruct only the ports whose declaration changed, and park the
    ports the new manifest does not declare; returns the names of the
    ports that did not converge, retained and retried on the next
    reload."""
    changed = [n for n in sorted(cfg.ports)
               if previous is None
               or n not in previous.ports
               or _port_signature(cfg.ports[n])
               != _port_signature(previous.ports[n])]
    removed = [n for n in sorted(previous.ports)
               if n not in cfg.ports] if previous is not None else []
    if not changed and not removed:
        log.info("ports unchanged")
        return []
    from nxs.cam import cli as cam_cli
    from nxs.cam import topology as cam_topo

    failed: List[str] = []
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
        args = argparse.Namespace(
            topology=None, port=name, links=[], mode=None, fps=None,
            dry_run=False, cam_cmd="on")
        try:
            rc = cam_cli.cmd_up(args)
            log.info("port %s: reconverged %s", name,
                     "ok" if rc == 0 else f"rc={rc}")
        except SystemExit as e:
            rc = 1
            log.warning("port %s: %s", name, e)
        except Exception:
            rc = 1
            log.exception("port %s: reconvergence failed", name)
        if rc != 0:
            failed.append(name)
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


def _prepare_capture_stack(cfg) -> None:
    """Every owned hub port's capture-stack configuration is built for the
    booted table before any consumer: the marker `status` reads stands
    while the build runs, and a failed build is logged, not fatal (the
    port's `on` refuses with the reason)."""
    from nxs import host as host_layer
    from nxs.cam.select import select_port_links
    from nxs.host.cli import ensure_tuning, preparing_marker

    host = host_layer.current()
    if not getattr(host, "capture_contract", False):
        return
    for name in sorted(cfg.ports):
        port = cfg.ports[name]
        if not _owned(port) or port.hub_compatible is None:
            continue
        if not any(link.camera for link in port.links):
            continue        # a pod alone streams no video: no tuning to build
        marker = preparing_marker(name)
        try:
            args = argparse.Namespace(topology=None, port=name, links=[], mode=None,
                                      fps=None, sensor=None, dry_run=False, cam_cmd="on")
            topology, _links = select_port_links(args, require_port=True)
            marker.write_text("")
            done = ensure_tuning(host, topology, log=lambda line: log.info("%s", line))
            if done:
                log.info("port %s: %s", name, done)
        except Exception as exc:        # noqa: BLE001 (the port's `on` says why)
            log.error("port %s: the capture stack's configuration was not built: %s",
                      name, exc)
        finally:
            marker.unlink(missing_ok=True)


def _construct_ports(cfg) -> List[str]:
    """Construct every nxs-driven port the manifest declares; returns the
    names of the ports that did not come up (the oneshot mode's exit
    status, and the deploy's gate before it flashes units behind them)."""
    from nxs.cam import cli as cam_cli

    failed: List[str] = []
    for name in sorted(cfg.ports):
        port = cfg.ports[name]
        if not _owned(port):
            continue
        args = argparse.Namespace(
            topology=None, port=name, links=[], mode=None, fps=None,
            dry_run=False, cam_cmd="on")
        try:
            rc = cam_cli.cmd_up(args)
            log.info("port %s: construction %s", name,
                     "ok" if rc == 0 else f"rc={rc}")
        except SystemExit as e:
            rc = 1
            log.warning("port %s: %s", name, e)
        except Exception:
            rc = 1
            log.exception("port %s: construction failed", name)
        if rc != 0:
            failed.append(name)
    return failed


def _ros2_available() -> bool:
    """Whether the bridge's ROS 2 client library imports in this process."""
    import importlib.util

    return importlib.util.find_spec("rclpy") is not None


def _hold_for_reload(cfg, wait=signal.pause) -> int:
    """No unit data plane to run: hold the process for SIGHUP and reconverge
    the changed ports on each one."""
    global _reload_requested
    while True:
        wait()
        if not _reload_requested:
            continue
        _reload_requested = False
        fresh = _load_checked()
        if fresh is None:
            log.error("keeping the previous configuration")
            continue
        cfg = _retained(fresh, cfg, _converge(fresh, cfg))


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
    """Record this daemon's build once its ports are constructed: `nxs
    switch` restarts a daemon that a newer wheel left running (a reload
    changes the manifest only) and waits for the record to say so."""
    from nxs.cam import port_state
    from nxs.cli import _git_version

    try:
        (port_state.state_dir() / BUILD_FILE).write_text(_git_version() + "\n")
    except OSError as exc:
        log.warning("the build record was not written: %s", exc)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="nxsd", description="Aliensense NXS daemon")
    parser.add_argument("--no-construct", action="store_true",
                        help="skip port construction at start")
    parser.add_argument("--construct-only", action="store_true",
                        help="construct ports, then exit (oneshot mode)")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO,
                        format="%(name)s: %(message)s")

    cfg = _load_checked()
    if cfg is None:
        return 1
    signal.signal(signal.SIGHUP, _on_hup)

    if not args.no_construct:
        _prepare_capture_stack(cfg)
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

    if not cfg.units:
        log.info("no units declared; holding for SIGHUP (port reconvergence)")
        return _hold_for_reload(cfg)
    if not _ros2_available():
        # The ports are up; the bridge has no ROS 2 to publish on. Hold
        # them rather than exit into a restart loop that rebuilds them.
        log.error("no ROS 2 environment for the unit bridge (nxs ros2 needs: pip install "
                  "aliensense-nxs[ros2], inside a sourced ROS 2 environment); "
                  "holding the ports for SIGHUP")
        return _hold_for_reload(cfg)
    # The data plane is the existing suite->ROS2 bridge, run in-process;
    # a SIGHUP between bridge exits reconverges changed ports only.
    from nxs.cli import build_parser, cmd_ros2
    global _reload_requested, _data_plane_running
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
        if not _reload_requested:
            return rc
        _reload_requested = False
        fresh = _load_checked()
        if fresh is None:
            log.error("keeping the previous configuration")
            continue
        cfg = _retained(fresh, cfg, _converge(fresh, cfg))


if __name__ == "__main__":
    sys.exit(main())
