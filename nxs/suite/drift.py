"""Drift detection: the one comparison of a unit's live state against
its manifest entry, at the granularity the wire can prove.

`apply` consumes it to repair (manifest wins), `freeze` to adopt
(device wins), `status` to display. Axes:

- driver: the active driver is not one the manifest declares;
- shape: the stored panel differs (slot count, or the manifest's
  sensor list changed since the last deploy);
- params: the active driver runs values other than the manifest's
  `config:` resolves to — visible only for the active driver, since
  only it serves capabilities;
- fw: the running firmware provably (or possibly) differs from the
  pin, mirroring the apply DFU rule: a readable MAJOR.MINOR mismatch
  is drift, an unreadable version is drift unless the state file
  records this exact pin.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from nxs.suite.schema import UnitSpec, parse_version
from nxs.suite.state import SuiteState


@dataclass
class UnitDrift:
    driver: bool = False
    shape: bool = False
    params: Dict[str, Tuple[object, object]] = field(default_factory=dict)
    egress: Dict[str, Tuple[object, object]] = field(default_factory=dict)
    fw: bool = False

    def any(self) -> bool:
        return (self.driver or self.shape or bool(self.params)
                or bool(self.egress) or self.fw)

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
        return kinds


def firmware_drift(unit: UnitSpec, state: SuiteState, transport) -> bool:
    """True when apply would flash: provable mismatch, or unknown
    version without a matching state record."""
    if not unit.firmware:
        return False
    want = parse_version(unit.firmware)
    device = transport.read_fw_version()

    # A provable major.minor mismatch always flashes — the wire is
    # authoritative for a contradiction, whatever the state record says.
    if device is not None and parse_version(device)[:2] != want[:2]:
        return True
    # major.minor agrees (or the wire serves nothing). The state record
    # refines to the exact pin — this tool knows the patch it flashed, so
    # a recorded version different from the pin is drift (a patch bump).
    recorded = state.unit(unit.name).get("fw_version")
    if recorded is not None:
        # Compare by parsed version, not raw string: a pin re-spelled
        # "1.1.0" → "1.1" is the same version, not a reflash. An
        # unparseable record (hand-edited state) reflashes to be safe.
        try:
            return parse_version(recorded) != want
        except ValueError:
            return True
    # No record: accept a proven major.minor match as converged rather
    # than a disruptive reflash (patch is below what the wire proves);
    # flash once only when the wire can't even prove major.minor
    # (firmware predating the FW_VERSION registers).
    return device is None


def egress_drift(unit: UnitSpec, transport) -> Dict[str, Tuple[object, object]]:
    """Declared egress factors that differ from the device, keyed by
    'device' (the gate) or the subject token. Empty when the unit
    declares no egress section."""
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
    `expected_hash` the manifest-side panel digest; both come from the
    caller so apply, freeze, and status share one compilation.
    """
    drift = UnitDrift(fw=firmware_drift(unit, state, transport),
                      egress=egress_drift(unit, transport))
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
