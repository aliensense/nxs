"""The bare `nxs timesync`: one resident pusher for every declared unit's
time discipline on one cadence. Transports stay open across rounds; a unit
that fails a round is reopened on the next one with its estimator intact."""
import logging
import os
import shutil
import sys
from dataclasses import dataclass
from typing import List, Optional

from nxs.client import (DeviceRefused, PUSH_INTERVAL_S, SupportsTimeSync,
                        ERRNO_EBUSY, estimate_and_push, exc_detail)
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
        f"ExecStart={nxs_path} timesync{args}\n"
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
        self._fits = {}

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
            # The fit is the unit's, not the link's: a reopened link carries on
            # from the fit the dropped one held.
            if hasattr(transport, "adopt_time_sync_fit") and unit.name in self._fits:
                transport.adopt_time_sync_fit(self._fits[unit.name])
            elif hasattr(transport, "get_time_sync"):
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
            if e.code == ERRNO_EBUSY:    # a transfer session is live
                # An upload or firmware push holds the mux: skip the interval and
                # keep the unit open, warning once per episode.
                if unit.name not in self._mux_held:
                    self._mux_held.add(unit.name)
                    log.warning("%s: transfer session live — skipping sync "
                                "until it ends", unit.name)
                return PushReport(unit.name, note="mux held — sync skipped")
            # Any other refusal is this unit's own failure, not the round's:
            # report it and let the other units continue.
            bound = None
            note = f"push refused ({exc_detail(e)})"
        except (OSError, TimeoutError):
            bound = None   # a deaf link: the note the unit's row carries
        except Exception as e:
            bound = None
            note = f"push failed ({exc_detail(e)})"
        if bound is None:
            self._drop(unit.name)
            return PushReport(unit.name, note=note)
        self._mux_held.discard(unit.name)
        return PushReport(unit.name, bound_us=bound)

    def _open_unit(self, unit):
        """First answering declared link serves, as in the bare `nxs status`."""
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
        if hasattr(transport, "time_sync_fit"):
            try:
                self._fits[name] = transport.time_sync_fit()
            except Exception:
                pass
        try:
            transport.close()
        except Exception:
            pass


def cmd_timesync_suite(args) -> int:
    """`nxs timesync [--unit NAME…] [--interval S] [--once]` with no unit
    addressed, or `--systemd` for the resident unit file."""
    import getpass
    import time

    from nxs.suite import default_config_path
    from nxs.suite.switch import load_manifest

    interval = args.interval if args.interval is not None else PUSH_INTERVAL_S
    if args.systemd:
        print(render_systemd_unit(resolve_nxs_path(sys.argv[0]),
                                  getpass.getuser(), args.only_units,
                                  interval), end="")
        return 0
    cfg = load_manifest(default_config_path())
    if cfg is None:
        return 1
    try:
        pusher = SuitePusher(
                cfg, only_units=args.only_units,
                pusher=lambda t: estimate_and_push(t, interval_s=interval))
    except ValueError as e:
        print(f"nxs timesync: {e}", file=sys.stderr)
        return 1
    count = len(args.only_units) if args.only_units else len(cfg.units)
    if not args.once:
        plural = "s" if count != 1 else ""
        print(f"disciplining {count} unit{plural} every {interval:g} s",
              flush=True)
    last = {}
    try:
        while True:
            failed = 0
            for report in pusher.round():
                ok = report.bound_us is not None
                if not ok:
                    failed += 1
                if args.once or last.get(report.name) != ok:
                    mark = "✓" if ok else "✗"
                    detail = f"±{report.bound_us} µs" if ok else report.note
                    print(f"{mark} {report.name}  {detail}", flush=True)
                last[report.name] = ok
            if args.once:
                return 1 if failed else 0
            time.sleep(interval)
    except KeyboardInterrupt:
        return 0
    finally:
        pusher.close()
