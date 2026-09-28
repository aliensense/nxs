"""Declarative suite management: one manifest describes every NXS unit attached
to this host; `nxs generate` transcribes reality into it, `nxs switch`
converges reality to it, `nxs status` reports every node against it. Learned
state lives apart."""
import os

FIRMWARE_DIR = "/opt/aliensense/firmware"
#: The personality store: one directory per personality, `<name>/<name>.{py,yaml}`,
#: written by `nxs personality install`; a flat `<name>.py` pair is accepted too.
PERSONALITY_DIR = "/opt/aliensense/personalities"
#: The store's earlier location, read after PERSONALITY_DIR and never
#: written, so an install that filled it keeps resolving.
EARLIER_PERSONALITY_DIR = "/opt/aliensense/patches"
#: A third store, read last.
DRIVERS_DIR = "/opt/aliensense/drivers"


def personality_dirs(drivers_dir=None):
    """The directories a personality name resolves in: the one given, else
    every store, the current location first."""
    if drivers_dir is not None:
        return [drivers_dir]
    return [PERSONALITY_DIR, EARLIER_PERSONALITY_DIR, DRIVERS_DIR]


def personality_file(name: str, ext: str, drivers_dir=None):
    """`<dir>/<name>/<name>.<ext>` or `<dir>/<name>.<ext>` in the first
    store directory that carries the personality, or None."""
    for directory in personality_dirs(drivers_dir):
        for candidate in (os.path.join(directory, name, f"{name}.{ext}"),
                          os.path.join(directory, f"{name}.{ext}")):
            if os.path.exists(candidate):
                return candidate
    return None

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
