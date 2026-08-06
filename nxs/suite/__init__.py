"""Declarative suite management: one manifest describes every NXS unit
attached to this host (link, firmware pin, sensor drivers); `nxs suite
switch` converges reality to it, `nxs suite scan` transcribes reality
into a manifest skeleton, `nxs suite status` reports per-unit health.

The manifest is intent; everything switch discovers (TOFU-recorded
serials, applied versions) lives in a separate state file so the
manifest stays hand-owned.
"""
import os

FIRMWARE_DIR = "/opt/aliensense/firmware"
DRIVERS_DIR = "/opt/aliensense/drivers"

_SYSTEM_CONFIG = "/etc/aliensense/suite.yaml"
_SYSTEM_STATE_DIR = "/var/lib/aliensense"


def default_config_path() -> str:
    """The system manifest when present, else the XDG per-user one."""
    if os.path.exists(_SYSTEM_CONFIG):
        return _SYSTEM_CONFIG
    base = os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config"))
    return os.path.join(base, "aliensense", "suite.yaml")


def default_state_path() -> str:
    """/var/lib when writable (a provisioned host), else the XDG data dir."""
    if os.path.isdir(_SYSTEM_STATE_DIR) and os.access(_SYSTEM_STATE_DIR, os.W_OK):
        return os.path.join(_SYSTEM_STATE_DIR, "state.yaml")
    base = os.environ.get("XDG_DATA_HOME", os.path.expanduser("~/.local/share"))
    return os.path.join(base, "aliensense", "state.yaml")
