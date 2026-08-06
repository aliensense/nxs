"""Apply-discovered per-unit state, kept out of the manifest.

Serials recorded on first contact (TOFU), applied firmware versions,
and panel hashes live here — facts the tool learned, not intent the
operator declared. The file mirrors SSH's known_hosts split: the
manifest stays hand-owned, this file is the tool's.
"""
import logging
import os
import time

import yaml

log = logging.getLogger("nxs.suite")


class SuiteState:
    """The state file: `{units: {name: {serial, fw_version, panel_hash, applied_at}}}`."""

    def __init__(self, path: str, units: dict = None):
        self._path = path
        self._units = units or {}
        self._dirty = False

    @classmethod
    def load(cls, path: str) -> "SuiteState":
        try:
            with open(path, encoding="utf-8") as f:
                raw = yaml.safe_load(f) or {}
        except FileNotFoundError:
            raw = {}  # normal first run
        except (OSError, yaml.YAMLError, UnicodeDecodeError) as e:
            # Any other unreadability — permissions, bad encoding, malformed
            # YAML — degrades to no recorded state rather than a traceback;
            # TOFU records re-learn on the next apply.
            log.warning("state file %s unreadable (%s); starting fresh", path, e)
            raw = {}
        if not isinstance(raw, dict):
            log.warning("state file %s is not a mapping; starting fresh", path)
            raw = {}
        units = raw.get("units", {})
        if not isinstance(units, dict):
            units = {}
        # Drop malformed entries (a hand-edited or corrupt file) rather
        # than crashing every later unit() caller.
        units = {name: entry for name, entry in units.items()
                 if isinstance(entry, dict)}
        return cls(path, units)

    def save(self):
        """Atomic write (tmp + rename) so a crash never truncates state.
        A no-op unless something was recorded — a converged switch leaves
        the file untouched."""
        if not self._dirty:
            return
        directory = os.path.dirname(self._path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp = self._path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            yaml.safe_dump({"units": self._units}, f, sort_keys=True)
        os.replace(tmp, self._path)
        self._dirty = False

    def unit(self, name: str) -> dict:
        """A copy of the unit's record — writes go through `record()`,
        which is what gates the save; a mutated copy changes nothing."""
        return dict(self._units.get(name, {}))

    def record(self, name: str, **fields):
        entry = self._units.setdefault(name, {})
        entry.update(fields)
        entry["applied_at"] = int(time.time())
        self._dirty = True

    def unit_names(self):
        """Names with a recorded entry (for orphan reporting and GC)."""
        return list(self._units)

    def forget(self, name: str):
        if self._units.pop(name, None) is not None:
            self._dirty = True

    @property
    def path(self) -> str:
        return self._path
