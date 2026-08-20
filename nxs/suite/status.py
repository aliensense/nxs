"""`nxs suite status`: one row per declared unit, read from the device.

On-demand probing — no resident process required. Every declared link
is probed: STATE is `up` (managed over the first link that answers),
`degraded` (managed, but another declared link is silent), or `down`
(no link answers); LINK names the link the row was read over. Reads
are best-effort: a partial metadata failure leaves the unit up with
unknown fields. The DRIFT column runs the same detection `apply`
repairs with and `freeze` adopts with: `-` means the device matches
the manifest, otherwise the drift kinds are named (`config`, `driver`,
`shape`, `fw`), and `?` means the comparison itself failed (e.g. the
manifest names a driver that won't compile). VM is the VM's own state,
except that a running VM reports its runner when the runner is not
measuring (`no-probe` parked, `probing` still trying) — the VM executes
happily while the board delivers nothing, so `running` alone hid a dead
sensor.
"""
import sys
from dataclasses import dataclass
from typing import List, Optional

from nxs._generated_constants import RunnerStates
from nxs.client import exc_detail
from nxs.suite import DRIVERS_DIR
from nxs.suite.drift import detect_unit_drift
from nxs.suite.schema import SuiteConfig
from nxs.suite.state import SuiteState
from nxs.transports import open_client

_VM_STATES = {0: "idle", 1: "running", 2: "error"}


@dataclass
class UnitStatus:
    name: str
    link: str
    up: bool = False
    degraded: bool = False
    driver: str = ""
    vm_state: str = ""
    fw_version: str = ""
    serial_ok: str = ""
    drift: str = ""
    sync: str = ""
    cal: str = ""
    samples: Optional[int] = None


def collect_status(cfg: SuiteConfig, state: SuiteState,
                   opener=open_client, drivers_dir: str = DRIVERS_DIR) -> List[UnitStatus]:
    rows = []
    for unit in cfg.units:
        row = UnitStatus(name=unit.name, link=unit.links[0].describe())
        # Probe every declared link: the first that answers serves the
        # metadata reads; any other link staying silent marks the row
        # degraded — the board is manageable but a route is lost.
        transport = None
        for link in unit.links:
            candidate = None
            try:
                candidate = opener(link.transport, **link.client_kwargs())
                answered = candidate.probe()
                # probe() answers False for a refused bus as well as for
                # absent hardware; the transport kept which it was.
                if not answered and (reason := candidate.probe_failure_detail()):
                    print(f"status: {link.describe()}: {reason}", file=sys.stderr)
            except Exception as e:
                # A permission error, a busy port, or a missing dependency
                # all render as STATE=down otherwise — identical to an
                # unplugged board, which is the one thing they are not.
                answered = False
                print(f"status: {link.describe()}: {exc_detail(e)}", file=sys.stderr)
            if answered and transport is None:
                transport = candidate
                row.link = link.describe()
                continue
            if not answered:
                row.degraded = True
            if candidate is not None:
                try:
                    candidate.close()
                except Exception:
                    pass
        row.up = transport is not None
        row.degraded = row.degraded and row.up
        if row.up:
            # Each field is read independently: one failing read (a
            # degraded link can throw mid-transaction) leaves that column
            # blank without blanking the others.
            row.driver = _try(transport.read_driver_name, "") or "-"
            row.vm_state = _vm_verdict(transport)
            row.fw_version = _try(transport.read_fw_version, "") or "-"
            row.sync = _sync_verdict(transport)
            row.serial_ok = _try(lambda: _serial_verdict(unit, state, transport), "-")
            row.drift = _try(lambda: _drift_verdict(unit, state, transport,
                                                    drivers_dir), "?")
            row.cal = _try(lambda: _calib_verdict(transport), "-")
            row.samples = _try(transport.read_sample_count, None)
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass
        rows.append(row)
    return rows


def _try(fn, default):
    """Run a best-effort metadata read; a raised read yields `default`
    rather than aborting the rest of the row."""
    try:
        return fn()
    except Exception:
        return default


def _serial_verdict(unit, state: SuiteState, transport) -> str:
    raw = transport.read_serial()
    if raw is None:
        return "-"
    seen = raw.hex()
    expected = unit.serial or state.unit(unit.name).get("serial")
    if expected is None:
        return "new"
    return "ok" if seen == expected else "MISMATCH"


def _vm_verdict(transport) -> str:
    """The VM column: the VM's own state, except that a running VM reports
    its runner when the runner is not measuring — `no-probe` for a parked
    PROBE_FAILED, `probing` while it is still trying.

    The two are different axes — the VM executes the driver, the runner owns
    the probe — and a board parked in PROBE_FAILED runs its VM happily while
    delivering nothing. Reporting `running` there was the reason a dead
    sensor looked healthy in this table."""
    label = _VM_STATES.get(_try(transport.read_vm_state, -1), "?")
    if label != "running":
        return label
    # Wrapped so the attribute lookup happens inside the guard too: a
    # transport with no runner surface keeps the plain VM verdict.
    runner = _try(lambda: transport.read_runner_state(), None)
    if runner == RunnerStates.RunnerState.PROBE_FAILED:
        return "no-probe"
    if runner == RunnerStates.RunnerState.PROBING:
        return "probing"

    return label


def _calib_verdict(transport) -> str:
    """Solved-calibration verdict from the device's own record.

    `ok` — every solved bucket is bound to the running sensor;
    `unguarded` — something is solved and applied, but carries no identity, so
    nothing verifies it belongs to this sensor;
    `STALE` — a solved bucket is guarded off (solved for another sensor);
    `-` — nothing solved (identity affines, no tags).

    The three states come from `CalibrationRecord.bucket_guard`, the same
    method the decode path asks, so this column and the applier cannot
    disagree about one record."""
    from nxs._generated_constants import Calibration as CalConstants
    from nxs.client import SupportsCalibration, active_driver_tag
    from nxs.descriptor import IDENTITY_M
    if not isinstance(transport, SupportsCalibration):
        return "-"
    record = transport.read_calibration()
    # Costs nothing on an untagged record: `active_driver_tag` short-circuits
    # rather than peeking the device for a comparison that cannot matter.
    active_tag = active_driver_tag(transport, record)
    guard_of = CalConstants.BucketGuard
    solved = False
    unguarded = False
    buckets = [(record.bucket_guard(v, active_tag),
                tuple(record.m[v]) != IDENTITY_M or any(record.b[v]))
               for v in range(3)]
    buckets.append((record.bucket_guard(len(record.driver_tags), active_tag),
                    bool(record.encoder_zero)))
    for guard, touched in buckets:
        if guard == guard_of.UNGUARDED and not touched:
            continue
        if guard == guard_of.STALE:
            return "STALE"
        solved = True
        unguarded = unguarded or guard == guard_of.UNGUARDED
    if not solved:
        return "-"

    return "unguarded" if unguarded else "ok"


def _drift_verdict(unit, state: SuiteState, transport, drivers_dir: str) -> str:
    from nxs.suite.reconcile import load_unit_driver, panel_hash

    try:
        panel = [(spec, load_unit_driver(spec.driver, drivers_dir)().compile(spec.config))
                 for spec in (unit.sensors or [])]
        drift = detect_unit_drift(unit, panel, transport, state,
                                  panel_hash(unit))
    except Exception:
        return "?"
    return "+".join(drift.kinds()) if drift.any() else "-"



def _sync_verdict(transport) -> str:
    """`±<bound>µs` while the unit holds a fresh discipline, `-` otherwise."""
    from nxs.client import SupportsTimeSync

    if not isinstance(transport, SupportsTimeSync):
        return "-"
    try:
        (_offset, bound_us, _rate, _window, _source,
         valid) = transport.read_time_sync()
    except Exception:
        return "-"
    return f"±{bound_us}µs" if valid else "-"

def render_status(rows: List[UnitStatus]) -> str:
    header = ("UNIT", "LINK", "STATE", "DRIVER", "VM", "FW", "SERIAL",
              "DRIFT", "SYNC", "CAL", "SAMPLES")
    table = [header]
    for r in rows:
        state_word = "degraded" if r.degraded else ("up" if r.up else "down")
        table.append((r.name, r.link, state_word,
                      r.driver or "-", r.vm_state or "-", r.fw_version or "-",
                      r.serial_ok or "-", r.drift or "-", r.sync or "-",
                      r.cal or "-",
                      str(r.samples) if r.samples is not None else "-"))
    widths = [max(len(row[i]) for row in table) for i in range(len(header))]
    return "\n".join("  ".join(cell.ljust(widths[i])
                               for i, cell in enumerate(row)).rstrip()
                     for row in table) + "\n"
