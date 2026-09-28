"""Drift detection: compare a unit's live state against its manifest entry.
`apply` consumes the result to repair (manifest wins), `freeze` to adopt
(device wins), `status` to display."""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from nxs._generated_constants import RunnerStates
from nxs.suite.schema import (UnitSpec, device_proves_patch, device_runs,
                              parse_version)
from nxs.suite.state import SuiteState


@dataclass
class UnitDrift:
    driver: bool = False
    shape: bool = False
    params: Dict[str, Tuple[object, object]] = field(default_factory=dict)
    egress: Dict[str, Tuple[object, object]] = field(default_factory=dict)
    fw: bool = False
    orientation: bool = False

    def any(self) -> bool:
        return (self.driver or self.shape or bool(self.params)
                or bool(self.egress) or self.fw or self.orientation)

    def kinds(self) -> List[str]:
        """Render labels, most invasive first."""
        kinds = []
        if self.driver:
            kinds.append("driver")
        if self.shape:
            kinds.append("shape")
        if self.params:
            kinds.append("config")
        if self.egress:
            kinds.append("egress")
        if self.fw:
            kinds.append("fw")
        if self.orientation:
            kinds.append("orientation")
        return kinds


def orientation_drift(unit: UnitSpec, transport) -> bool:
    """True when the declared mounting orientation differs from the device."""
    from nxs.client import SupportsCalibration, rotation_code
    if unit.orientation is None or not isinstance(transport, SupportsCalibration):
        return False
    return transport.read_calibration().orientation \
        != rotation_code(unit.orientation)


def firmware_drift(unit: UnitSpec, state: SuiteState, transport,
                   pin: Optional[str] = None) -> bool:
    """True when apply would flash: provable mismatch, or unknown
    version without a matching state record. `pin` is the version to
    hold the unit at; the unit's own `firmware` when None."""
    pin = pin or unit.firmware
    if not pin:
        return False
    want = parse_version(pin)
    device = transport.read_fw_version()

    # A full build identity proves the whole triple and decides alone; the
    # state record never overrides what the wire proves.
    runs = device_runs(device, want) if device is not None else None
    if runs is not None and device_proves_patch(device):
        return not runs
    # A legacy identity proves major.minor only; a pair mismatch is a
    # contradiction and flashes.
    if runs is False:
        return True
    # major.minor agrees, or the wire serves nothing. The state record refines
    # to the exact pin: a recorded version other than the pin is drift.
    recorded = state.unit(unit.name).get("fw_version")
    if recorded is not None:
        # Compare parsed versions ("1.1.0" equals "1.1"). An unparseable
        # record reflashes.
        try:
            return parse_version(recorded) != want
        except ValueError:
            return True
    # No record: a proven major.minor match converges; flash only when the
    # wire cannot prove even major.minor.
    return runs is None


def egress_drift(unit: UnitSpec, transport) -> Dict[str, Tuple[object, object]]:
    """Declared egress factors that differ from the device, keyed by 'device'
    (the gate) or the subject token. Empty when no egress section is declared."""
    if unit.egress is None:
        return {}
    drift = {}
    if unit.egress.decimation is not None:
        live = transport.read_decimation()
        if live != unit.egress.decimation:
            drift["device"] = (unit.egress.decimation, live)
    for subject, want in unit.egress.subjects.items():
        live = transport.read_decimation(subject=subject)
        if live != want:
            drift[subject] = (want, live)
    return drift


def detect_unit_drift(unit: UnitSpec, panel: list, transport,
                      state: SuiteState, expected_hash: str) -> UnitDrift:
    """Compare the unit's live state against its compiled manifest panel.
    `panel` is the compiled `[(SensorSpec, CompiledDriver), ...]` and
    `expected_hash` the manifest-side panel digest."""
    drift = UnitDrift(fw=firmware_drift(unit, state, transport),
                      egress=egress_drift(unit, transport),
                      orientation=orientation_drift(unit, transport))
    if not panel:
        # Declared-empty (`sensors: []`): anything running or stored is
        # driver drift. An unmanaged panel (key absent) is never drift.
        if unit.sensors is not None and (
                transport.read_driver_name() or transport.read_store_count()):
            drift.driver = True
        return drift

    names = [compiled.name for _, compiled in panel]
    active = transport.read_driver_name()
    if active not in names:
        drift.driver = True
        return drift
    # A driver that loaded but never probed is not the manifest realized; the
    # runner parks in PROBE_FAILED until a host command intervenes.
    if transport.read_runner_state() == RunnerStates.RunnerState.PROBE_FAILED:
        drift.driver = True
        return drift
    if transport.read_store_count() != len(panel) \
            or state.unit(unit.name).get("panel_hash") != expected_hash:
        drift.shape = True
        return drift

    compiled = next(c for _, c in panel if c.name == active)
    desired = {p.name: p.current for p in compiled.params}
    live = {p["name"]: p["current"] for p in transport.read_capabilities()}
    drift.params = {name: (desired[name], live[name])
                    for name in desired.keys() & live.keys()
                    if desired[name] != live[name]}
    return drift
