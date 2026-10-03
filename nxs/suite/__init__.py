"""Declarative suite management: one manifest describes every NXS unit attached
to this host; `nxs generate` transcribes reality into it, `nxs switch`
converges reality to it, `nxs status` reports every node against it. Learned
state lives apart."""
import os
from typing import Optional

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


def _user_config_path() -> str:
    base = os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config"))
    return os.path.join(base, "aliensense", "suite.yaml")


def _camera_host() -> bool:
    from nxs import host as host_layer

    return host_layer.current().keeps_system_declaration()


_config_override: Optional[str] = None


def use_config_path(path: Optional[str]) -> None:
    """Resolve the declaration at `path` for the rest of the run, so every
    reader of it (the camera topology, the laws, the ports' bring-up)
    follows `nxs switch -c`; None returns to the host's own."""
    global _config_override
    _config_override = os.path.abspath(path) if path else None


def default_config_path() -> str:
    """The declaration named for this run (`use_config_path`), else the
    system manifest when present, under root (the daemon runs the host's
    declaration, present or not yet) and on a camera host (its one
    declaration is the daemon's), else the XDG per-user one."""
    from nxs.host import root

    if _config_override is not None:
        return _config_override
    if os.path.exists(_SYSTEM_CONFIG) or root.is_root() or _camera_host():
        return _SYSTEM_CONFIG
    return _user_config_path()


def stray_declaration(config_path: str) -> Optional[str]:
    """The refusal when `config_path` is a camera host's absent system
    manifest and a per-user one stands, which no verb reads there: the
    fact, then the move of it and its report, which first makes the
    directory as `nxs switch` does where it does not exist yet. None
    otherwise."""
    from nxs.suite.schema import hardware_path

    if (os.path.abspath(config_path) != os.path.abspath(_SYSTEM_CONFIG)
            or os.path.exists(_SYSTEM_CONFIG) or not _camera_host()):
        return None
    stray = _user_config_path()
    if not os.path.exists(stray):
        return None
    from nxs.suite.switch_cam import STATE_GROUP

    moved = [path for path in (stray, hardware_path(stray)) if os.path.exists(path)]
    target = os.path.dirname(_SYSTEM_CONFIG)
    move = f"mv {' '.join(moved)} {target}/"
    if not os.path.isdir(target):
        move = f"sudo install -d -m 2775 -g {STATE_GROUP} {target} && {move}"
    return (f"the declaration on this host is {_SYSTEM_CONFIG}, and {stray} is not read here\n"
            f"  - {move}")


def default_state_path() -> str:
    """/var/lib when writable (a provisioned host), else the XDG data dir."""
    if os.path.isdir(_SYSTEM_STATE_DIR) and os.access(_SYSTEM_STATE_DIR, os.W_OK):
        return os.path.join(_SYSTEM_STATE_DIR, "state.yaml")
    base = os.environ.get("XDG_DATA_HOME", os.path.expanduser("~/.local/share"))
    return os.path.join(base, "aliensense", "state.yaml")
