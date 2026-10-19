# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The ROS 2 environment the daemon's unit carries: what a sourced
distribution sets in the shell `nxs switch` runs in, and what an installed
unit's `Environment=` lines hold."""

import logging
import os
from typing import Dict, Optional

#: The environment a sourced ROS 2 distribution sets, in the unit's order: the two every sourced
#: shell has, then the ones the shell sets (a workspace overlay, the middleware, the discovery).
ROS_ENVIRONMENT_REQUIRED = ("ROS_DISTRO", "AMENT_PREFIX_PATH")
ROS_ENVIRONMENT_KEYS = ROS_ENVIRONMENT_REQUIRED + (
    "ROS_VERSION", "ROS_PYTHON_VERSION", "PYTHONPATH", "LD_LIBRARY_PATH", "COLCON_PREFIX_PATH",
    "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "ROS_AUTOMATIC_DISCOVERY_RANGE",
    "ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER", "ROS_SUPER_CLIENT", "FASTRTPS_DEFAULT_PROFILES_FILE",
    "FASTDDS_DEFAULT_PROFILES_FILE", "FASTDDS_BUILTIN_TRANSPORTS", "CYCLONEDDS_URI")


def ros_environment(environ=None) -> Dict[str, str]:
    """The ROS 2 environment a shell carries, by key in the unit's order;
    empty when the shell sourced no distribution."""
    environ = os.environ if environ is None else environ
    if any(not environ.get(key) for key in ROS_ENVIRONMENT_REQUIRED):
        return {}
    # A unit file holds one line per assignment: a value with a line break
    # has no form there.
    return {key: environ[key] for key in ROS_ENVIRONMENT_KEYS
            if environ.get(key) and "\n" not in environ[key]}


def unit_environment(text: Optional[str]) -> Dict[str, str]:
    """The ROS 2 environment an installed unit's `Environment=` lines carry,
    by key; the unit's own lines (root's `HOME`) are not part of it."""
    out: Dict[str, str] = {}
    for line in (text or "").splitlines():
        if not line.startswith("Environment="):
            continue
        value = line[len("Environment="):].strip()
        if len(value) >= 2 and value[0] == value[-1] == '"':
            value = value[1:-1].replace('\\"', '"').replace("\\\\", "\\")
        value = value.replace("%%", "%")
        key, sep, val = value.partition("=")
        if sep and key in ROS_ENVIRONMENT_KEYS:
            out[key] = val
    return out


#: The middleware transport the unit carries unless the shell set one. The daemon publishes as root,
#: and shared memory does not cross users: over it the operator's subscribers see topics, no data.
TRANSPORT_DEFAULT = ("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")


def with_transport_default(environment: Dict[str, str]) -> Dict[str, str]:
    """The ROS 2 environment with the transport default added, where the
    environment is one and names no transport."""
    out = dict(environment)
    if out and TRANSPORT_DEFAULT[0] not in out:
        out[TRANSPORT_DEFAULT[0]] = TRANSPORT_DEFAULT[1]
    return out


#: How long systemd lets a stopping daemon end the port bring-up it is on
#: before it kills it, which would leave the port's hub half programmed.
STOP_TIMEOUT_S = 180

#: The ROS 2 environment the running daemon started under, recorded beside its build: `nxs switch`
#: restarts a daemon whose unit carries another one.
ENVIRONMENT_FILE = "nxsd.environment"


def record_environment(environ=None) -> None:
    """Record the ROS 2 environment this daemon runs under, a `key=value`
    line each, empty without one."""
    from nxs.cam import port_state

    lines = "".join(f"{key}={value}\n" for key, value in ros_environment(environ).items())
    try:
        port_state.write_store_file(port_state.state_dir() / ENVIRONMENT_FILE, lines)
    except OSError as exc:
        logging.getLogger("nxsd").warning("the environment record was not written: %s", exc)


def running_environment() -> Optional[Dict[str, str]]:
    """The ROS 2 environment the running daemon recorded at start, None
    when none did."""
    from nxs.cam import port_state

    try:
        text = (port_state.state_dir() / ENVIRONMENT_FILE).read_text()
    except OSError:
        return None
    return dict(line.split("=", 1) for line in text.splitlines() if "=" in line)


#: The next command that brings a ROS 2 environment into the daemon's unit.
SOURCE_ROS = "source /opt/ros/<distro>/setup.bash, then nxs switch"


def render_systemd_unit(exec_path: str, user: Optional[str],
                        capture_daemon: Optional[str] = None,
                        environment: Optional[Dict[str, str]] = None) -> str:
    """The nxsd unit with an absolute entry point (systemd's PATH lacks ~/.local/bin),
    ordered after the host's capture daemon, carrying the ROS 2 environment
    `switch` captured from the shell that ran it (`ros_environment`), one
    `Environment=` line per key in that order, so the data plane imports the
    distribution's client library. It runs as root: the boot table, the
    capture stack's configuration and the ports are root's to touch, and the
    state store under /var/lib is shared with the operator's shell. As root it
    carries root's `HOME`, which systemd sets for no root unit: the ROS 2
    client library logs under it and does not start without it."""
    user_line = f"User={user}\n" if user else ""
    home_line = "" if user else 'Environment="HOME=/root"\n'
    after = "network.target" + (f" {capture_daemon}" if capture_daemon else "")
    # systemd reads `%` as a specifier: a literal one is doubled.
    env_lines = "".join(
        'Environment="{}={}"\n'.format(
            key, value.replace("\\", "\\\\").replace('"', '\\"').replace("%", "%%"))
        for key, value in (environment or {}).items())
    return ("[Unit]\n"
            "Description=Aliensense NXS daemon (port construction + unit data plane)\n"
            f"After={after}\n"
            "\n"
            "[Service]\n"
            f"{user_line}"
            f"{home_line}"
            f"{env_lines}"
            f"ExecStart={exec_path}\n"
            "Restart=on-failure\n"
            "RestartSec=5\n"
            f"TimeoutStopSec={STOP_TIMEOUT_S}\n"
            "\n"
            "[Install]\n"
            "WantedBy=multi-user.target\n")
