"""`nxs suite reset`: return declared units to a blank state.

The decommission half of the lifecycle: `apply` builds a unit up,
`reset` takes it down — stop the driver, clear the store, forget the
unit's learned state. The board stays addressable: node-id, subject
remaps, and decimation factors survive a blank reset, so a reset unit
is one `apply` away from service. `--factory` additionally reverts the
node-id to the compiled default and the decimation factors to their
shipping values, then persists. Declared units only — the tool never
touches devices the manifest does not claim.
"""
from dataclasses import dataclass, field
from typing import List, Optional

from nxs.client import SupportsCommissioning
from nxs.suite.schema import SuiteConfig
from nxs.suite.state import SuiteState
from nxs.transports import open_client

# The 0xFFFF node-id sentinel reverts to the compiled default.
NODE_ADDR_UNSET = 0xFFFF
# Shipping decimation profile (the out-of-box defaults the device boots
# with before any commissioning): device gate at every sample,
# temperature thinned to 1/25th, every other subject at every sample.
FACTORY_DEVICE_DECIMATION = 1
FACTORY_SUBJECT_DECIMATION = {"temperature": 25}
FACTORY_SUBJECT_DEFAULT = 1


@dataclass
class ResetReport:
    name: str
    link: str
    ok: bool = True
    actions: List[str] = field(default_factory=list)
    error: str = ""


def reset_suite(cfg: SuiteConfig, state: SuiteState, *,
                only_unit: Optional[str] = None, factory: bool = False,
                opener=open_client) -> List[ResetReport]:
    """Reset the selected unit(s); persists the state file at the end."""
    from nxs.suite.reconcile import _open_unit

    reports = []
    for unit in cfg.units:
        if only_unit is not None and unit.name != only_unit:
            continue
        report = ResetReport(name=unit.name, link=unit.links[0].describe())
        transport = None
        try:
            transport, mgmt_link, notes, error = _open_unit(unit, opener)
            if transport is None:
                report.ok = False
                report.error = error
                reports.append(report)
                continue
            report.link = mgmt_link.describe()
            report.actions.extend(notes)
            _reset_unit(unit, state, transport, report, factory=factory)
        except Exception as e:
            report.ok = False
            report.error = f"{type(e).__name__}: {e}"
        finally:
            if transport is not None:
                try:
                    transport.close()
                except Exception:
                    pass
        reports.append(report)
    try:
        state.save()
    except OSError as e:
        import logging
        logging.getLogger("nxs.suite").warning(
            "could not persist suite state to %s (%s)", state.path, e)
    return reports


def collect_garbage(cfg: SuiteConfig, state: SuiteState) -> List[str]:
    """Drop state entries not rooted in the manifest and return their
    names. The manifest is the GC root set; a dropped entry's TOFU
    serial is gone, so a re-added unit re-records on first contact."""
    declared = {u.name for u in cfg.units}
    orphans = sorted(set(state.unit_names()) - declared)
    for name in orphans:
        state.forget(name)
    if orphans:
        state.save()
    return orphans


def _reset_unit(unit, state: SuiteState, transport, report,
                *, factory: bool):
    transport.vm_stop()
    transport.vm_reset()
    transport.clear_store()
    report.actions.append("stopped driver, cleared store")

    if factory:
        transport.write_decimation(FACTORY_DEVICE_DECIMATION)
        for subject, factor in FACTORY_SUBJECT_DECIMATION.items():
            transport.write_decimation(factor, subject=subject)
        for subject in ("acceleration", "angular_velocity", "magnetic_field",
                        "pressure", "scalar"):
            transport.write_decimation(FACTORY_SUBJECT_DEFAULT, subject=subject)
        report.actions.append("restored shipping decimation profile")
        if isinstance(transport, SupportsCommissioning):
            transport.commission(node_addr=NODE_ADDR_UNSET)
            report.actions.append("reverted node-id to the compiled default "
                                  "(adopts on next power-cycle)")

    state.forget(unit.name)
    report.actions.append("forgot recorded state (serial, firmware, panel)")
