"""The unit rows of a bare `nxs status`, read on demand: `up`, `degraded`
(another declared link is silent), or `down`; the drift kinds (`config`,
`driver`, `shape`, `fw`), `-` for none, `?` when unknown; the sync,
serial and calibration verdicts; the personality and its outputs."""
import sys
from dataclasses import dataclass, field
from typing import List, Optional

from nxs._generated_constants import RunnerStates
from nxs.client import exc_detail
from nxs.suite.drift import detect_unit_drift
from nxs.suite.schema import SuiteConfig
from nxs.suite.state import SuiteState
from nxs._generated_constants import VmStates
from nxs.transports import open_client

_VM_STATES = {v: n.lower() for v, n in VmStates.VmState._NAMES.items()}


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
    serial: str = ""
    outputs: List[str] = field(default_factory=list)


def collect_status(cfg: SuiteConfig, state: SuiteState,
                   opener=open_client, drivers_dir: "str | None" = None) -> List[UnitStatus]:
    rows = []
    for unit in cfg.units:
        row = UnitStatus(name=unit.name, link=unit.links[0].describe())
        # Probe every declared link: the first that answers serves the reads;
        # any other link staying silent marks the row degraded.
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
                # A permission error, a busy port, or a missing dependency would
                # render as STATE=down otherwise, identical to an unplugged board.
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
            # Each field is read independently: one failing read leaves that
            # column blank without blanking the others.
            row.driver = _try(transport.read_driver_name, "") or "-"
            row.vm_state = _vm_verdict(transport)
            row.fw_version = _try(transport.read_fw_version, "") or "-"
            row.sync = _sync_verdict(transport)
            row.serial = _try(lambda: (transport.read_serial() or b"").hex(), "")
            row.serial_ok = _try(lambda: _serial_verdict(unit, state, transport), "-")
            row.drift = _try(lambda: _drift_verdict(unit, state, transport,
                                                    drivers_dir), "?")
            row.cal = _try(lambda: _calib_verdict(transport), "-")
            row.samples = _try(transport.read_sample_count, None)
            row.outputs = _try(lambda: [o["name"] for o in transport.read_outputs() or []], [])
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
    """The VM column: the VM's own state, except that a running VM reports its
    runner when the runner is not measuring: `no-probe` for a parked
    PROBE_FAILED, `probing` while it is still trying."""
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
    """Solved-calibration verdict from the device's record: `ok` (every solved
    bucket bound to the running sensor), `unguarded` (solved, no identity),
    `STALE` (a bucket solved for another sensor), `-` (nothing solved)."""
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


def _drift_verdict(unit, state: SuiteState, transport, drivers_dir) -> str:
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
