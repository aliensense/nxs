"""`nxs suite timesync`: one resident pusher for the whole bench.

Pushes every declared unit's time discipline on one cadence.
Transports stay open across rounds. A unit that fails a round is
reopened on the next one with its estimator intact.
"""
import logging
import os
import shutil
import sys
from dataclasses import dataclass
from typing import List, Optional

from nxs.client import (DeviceRefused, PUSH_INTERVAL_S, SupportsTimeSync,
                        XFER_EBUSY, estimate_and_push, exc_detail)
from nxs.suite.schema import SuiteConfig
from nxs.transports import open_client

log = logging.getLogger(__name__)


@dataclass
class PushReport:
    name: str
    bound_us: Optional[int] = None
    note: str = ""


def resolve_nxs_path(argv0: str, which=shutil.which) -> str:
    """Absolute entry-point path for the emitted ExecStart: the PATH
    resolution of the invoked name, else the script's absolute path."""
    found = which(os.path.basename(argv0))
    return found or os.path.abspath(argv0)


def render_systemd_unit(nxs_path: str, user: str, only_units=None,
                        interval: float = PUSH_INTERVAL_S) -> str:
    """Render a systemd unit that runs this pusher resident."""
    args = "".join(f" --unit {name}" for name in only_units or [])
    if interval != PUSH_INTERVAL_S:
        args += f" --interval {interval:g}"
    return (
        "[Unit]\n"
        "Description=NXS suite time discipline pusher\n"
        "\n"
        "[Service]\n"
        f"User={user}\n"
        f"ExecStart={nxs_path} suite timesync{args}\n"
        "Restart=on-failure\n"
        "RestartSec=2\n"
        "\n"
        "[Install]\n"
        "WantedBy=multi-user.target\n"
    )


class SuitePusher:
    def __init__(self, cfg: SuiteConfig, only_units=None, opener=open_client,
                 pusher=estimate_and_push):
        self._mux_held = set()
        self._open_warned = set()
        declared = {unit.name for unit in cfg.units}
        for name in only_units or []:
            if name not in declared:
                raise ValueError(f"no unit named {name!r} in the manifest")
        self._units = [unit for unit in cfg.units
                       if only_units is None or unit.name in only_units]
        self._opener = opener
        self._pusher = pusher
        self._open_units = {}
        self._estimators = {}

    def round(self) -> List[PushReport]:
        """Push every selected unit once. Per-unit isolation: one
        failing unit never blocks the rest."""
        return [self._push_unit(unit) for unit in self._units]

    def close(self):
        for transport in self._open_units.values():
            try:
                transport.close()
            except Exception:
                pass
        self._open_units.clear()

    def warn_once(self, name: str, message: str):
        """One warning per unit per episode. This pusher is resident and
        runs every interval, so an unrecoverable link would otherwise
        repeat its reason forever."""
        if name not in self._open_warned:
            self._open_warned.add(name)
            log.warning("%s: %s", name, message)

    def _push_unit(self, unit) -> PushReport:
        if unit.name not in self._open_units:
            transport = self._open_unit(unit)
            if transport is None:
                return PushReport(unit.name, note="no link answered")
            self._open_warned.discard(unit.name)
            if hasattr(transport, "get_time_sync"):
                held = self._estimators.get(unit.name)
                if held is not None:
                    transport.adopt_time_sync(held)
                else:
                    self._estimators[unit.name] = transport.get_time_sync()
            self._open_units[unit.name] = transport
        transport = self._open_units[unit.name]
        if not isinstance(transport, SupportsTimeSync):
            return PushReport(unit.name, note="no time surface")
        note = "no observation (link silent?)"
        try:
            bound = self._pusher(transport)
        except DeviceRefused as e:
            if e.code == XFER_EBUSY:    # a transfer session is live
                # An upload or firmware push holds the mux. Skip the interval
                # and keep the unit open — the estimator's state is fine, the
                # device just can't take a record right now. One warning per
                # episode; the next successful push re-arms it.
                if unit.name not in self._mux_held:
                    self._mux_held.add(unit.name)
                    log.warning("%s: transfer session live — skipping sync "
                                "until it ends", unit.name)
                return PushReport(unit.name, note="mux held — sync skipped")
            # Any other refusal is this unit's own failure, not the round's:
            # report it and let the other units continue (round() isolates
            # per unit — a re-raise here would abort the whole fleet's sync).
            bound = None
            note = f"push refused ({exc_detail(e)})"
        except Exception as e:
            bound = None
            note = f"push failed ({exc_detail(e)})"
        if bound is None:
            self._drop(unit.name)
            return PushReport(unit.name, note=note)
        self._mux_held.discard(unit.name)
        return PushReport(unit.name, bound_us=bound)

    def _open_unit(self, unit):
        """First answering declared link serves, as in `suite status`."""
        for link in unit.links:
            candidate = None
            try:
                candidate = self._opener(link.transport, **link.client_kwargs())
                if candidate.probe():
                    return candidate
                # probe() answers False for a refused bus as well as for
                # absent hardware; the transport kept which it was.
                if reason := candidate.probe_failure_detail():
                    self.warn_once(unit.name, f"{link.describe()}: {reason}")
            except Exception as e:
                self.warn_once(unit.name, f"{link.describe()}: {exc_detail(e)}")
            if candidate is not None:
                try:
                    candidate.close()
                except Exception:
                    pass
        return None

    def _drop(self, name):
        transport = self._open_units.pop(name)
        try:
            transport.close()
        except Exception:
            pass
