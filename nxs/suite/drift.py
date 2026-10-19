"""Drift detection: compare a unit's live state against its manifest entry.
`apply` consumes the result to repair (manifest wins), `freeze` to adopt
(device wins), `status` to display."""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from nxs._generated_constants import NxsMap, RunnerStates
from nxs.suite.firmware import runs_build
from nxs.suite.schema import (UnitSpec, device_proves_patch, device_runs,
                              parse_version)
from nxs.suite.state import SuiteState

#: The slots a unit's personality store holds.
STORE_SLOTS = 8


@dataclass
class UnitDrift:
    personality: bool = False
    shape: bool = False
    params: Dict[str, Tuple[object, object]] = field(default_factory=dict)
    #: The unit's own rows whose served value differs from the declared one,
    #: by name: (declared, live).
    rows: Dict[str, Tuple[object, object]] = field(default_factory=dict)
    fw: bool = False

    def any(self) -> bool:
        return self.personality or self.shape or bool(self.params) or bool(self.rows) or self.fw

    def kinds(self) -> List[str]:
        """Render labels, most invasive first."""
        kinds = []
        if self.personality:
            kinds.append("personality")
        if self.shape:
            kinds.append("shape")
        if self.params:
            kinds.append("config")
        if self.rows:
            kinds.append("rows")
        if self.fw:
            kinds.append("fw")
        return kinds


def declared_rows(unit: UnitSpec) -> Dict[str, int]:
    """The unit's own declared rows the row step commits: every one but the
    node id a Cyphal/CAN route commissions."""
    from nxs.suite.schema_nodes import CAN_NODE_ID

    by_can = any(link.transport == "cyphal-can" for link in unit.links)
    return {k: v for k, v in unit.rows.items() if not (by_can and k == CAN_NODE_ID)}


def unit_door(transport):
    """The unit's map door at the address `transport` reaches it at, None
    for a route that reaches no door (a Cyphal link, a test's client)."""
    door = getattr(transport, "door", None)
    address = getattr(transport, "address", None)
    if door is None or address is None:
        return None
    try:
        at = address() if callable(address) else address
        return door(int(at)) if at is not None else None
    except (RuntimeError, OSError, TypeError):
        return None


def rows_drift(unit: UnitSpec, transport) -> Optional[Dict[str, Tuple[int, Optional[int]]]]:
    """The declared rows of the unit that it serves at another value, by
    name: (declared, live), live None for a row the unit serves none of. {}
    when every declared row holds or none is declared; None when rows are
    declared and the route reaches no unit door."""
    from nxs._libnxs_map import find_row

    declared = declared_rows(unit)
    if not declared:
        return {}
    door = unit_door(transport)
    if door is None:
        return None
    with door:
        rows = door.rows()
        cells = door.cells(rows)
    out: Dict[str, Tuple[int, Optional[int]]] = {}
    for name, want in declared.items():
        index = find_row(rows, name)
        live = cells[index]["live"] if index is not None else None
        if live != want:
            out[name] = (int(want), live)
    return out


def firmware_drift(unit: UnitSpec, state: SuiteState, transport,
                   pin: Optional[str] = None, image: Optional[str] = None) -> bool:
    """True when apply would flash. `image` is the build identity the pinned
    image carries: the unit runs that image exactly when it reports the
    identity and neither is a dirty build, as `push-fw` judges a push.
    Without one, true on a provable mismatch or an unknown version without a
    matching state record. `pin` is the version to hold the unit at; the
    unit's own `firmware` when None."""
    pin = pin or unit.firmware
    if not pin:
        return False
    device = transport.read_fw_version()
    if image is not None:
        return not runs_build(device, image)
    want = parse_version(pin)

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
        # Compare parsed versions ("X.Y.0" equals "X.Y"). An unparseable
        # record reflashes.
        try:
            return parse_version(recorded) != want
        except ValueError:
            return True
    # No record: a proven major.minor match converges; flash only when the
    # wire cannot prove even major.minor.
    return runs is None


def click_slots(transport) -> List[int]:
    """The populated store slots that hold a click personality: every one
    but a cam personality's, which the camera steps own. A slot the unit
    refuses to describe counts as a sensor slot; a transport that cannot
    peek a slot counts every populated one."""
    from nxs.client import ERRNO_EBUSY, DeviceRefused, SupportsSlotPeek
    from nxs.image import IMAGE_KIND_NAMES, ImageKind

    if not isinstance(transport, SupportsSlotPeek):
        return list(range(transport.read_store_count()))
    camera = IMAGE_KIND_NAMES[ImageKind.CAMERA]
    slots = []
    for slot in range(STORE_SLOTS):
        try:
            info = transport.read_slot_info(slot)
        except DeviceRefused as exc:
            if exc.code == ERRNO_EBUSY:
                raise
            slots.append(slot)
            continue
        if info is not None and getattr(info, "kind", None) != camera:
            slots.append(slot)
    return slots


def declared_values(compiled) -> Dict[str, object]:
    """The values a compiled click personality configures, by name: every
    parameter but its GATE row, which starts and parks the sampling."""
    return {p.name: p.current for p in compiled.params if p.kind != "gate"}


def live_values(params, compiled) -> Dict[str, object]:
    """The configuration among the parameters a transport reads, by name.
    The GATE row starts and parks the sampling, and a row the device owns
    (`OWNER_DEVICE`) belongs to no personality, so neither is configuration.
    A transport that serves no kind names no row's kind, so the compiled
    personality's GATE rows are left out by name as well."""
    gates = {p.name for p in compiled.params if p.kind == "gate"}
    return {p["name"]: p["current"] for p in params
            if p.get("kind") != "gate" and p["name"] not in gates
            and p.get("owner") != NxsMap.OWNER_DEVICE}


def detect_unit_drift(unit: UnitSpec, panel: list, transport,
                      state: SuiteState, expected_hash: str) -> UnitDrift:
    """Compare the unit's live state against its compiled manifest panel.
    `panel` is the compiled `[(SensorSpec, CompiledDriver), ...]` and
    `expected_hash` the manifest-side panel digest."""
    drift = UnitDrift(fw=firmware_drift(unit, state, transport),
                      rows=rows_drift(unit, transport) or {})
    if not panel:
        # A socket no click feeds is empty: a click personality running or
        # stored is personality drift.
        if transport.read_personality_name() or click_slots(transport):
            drift.personality = True
        return drift

    names = [compiled.name for _, compiled in panel]
    active = transport.read_personality_name()
    if active not in names:
        drift.personality = True
        return drift
    # A personality that is not measuring is not the manifest realized: one that
    # never probed parks in PROBE_FAILED until a host command intervenes, a
    # stopped one waits in LOADING. Either is redeployed, and the deploy's
    # own verdict names why it does not come up.
    if transport.read_runner_state() != RunnerStates.RunnerState.MEASURING:
        drift.personality = True
        return drift
    if len(click_slots(transport)) != len(panel) \
            or state.unit(unit.name).get("panel_hash") != expected_hash:
        drift.shape = True
        return drift

    compiled = next(c for _, c in panel if c.name == active)
    desired = declared_values(compiled)
    live = live_values(transport.read_capabilities(), compiled)
    drift.params = {name: (desired[name], live[name])
                    for name in desired.keys() & live.keys()
                    if desired[name] != live[name]}
    return drift
