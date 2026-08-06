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
manifest names a driver that won't compile).
"""
from dataclasses import dataclass
from typing import List, Optional

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
            except Exception:
                answered = False
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
            row.vm_state = _VM_STATES.get(_try(transport.read_vm_state, -1), "?")
            row.fw_version = _try(transport.read_fw_version, "") or "-"
            row.sync = _sync_verdict(transport)
            row.serial_ok = _try(lambda: _serial_verdict(unit, state, transport), "-")
            row.drift = _try(lambda: _drift_verdict(unit, state, transport,
                                                    drivers_dir), "?")
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
              "DRIFT", "SYNC", "SAMPLES")
    table = [header]
    for r in rows:
        state_word = "degraded" if r.degraded else ("up" if r.up else "down")
        table.append((r.name, r.link, state_word,
                      r.driver or "-", r.vm_state or "-", r.fw_version or "-",
                      r.serial_ok or "-", r.drift or "-", r.sync or "-",
                      str(r.samples) if r.samples is not None else "-"))
    widths = [max(len(row[i]) for row in table) for i in range(len(header))]
    return "\n".join("  ".join(cell.ljust(widths[i])
                               for i, cell in enumerate(row)).rstrip()
                     for row in table) + "\n"
