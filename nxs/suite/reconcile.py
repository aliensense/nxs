"""`nxs suite switch`: converge every declared unit to its manifest entry.

Per unit, in order: compile the sensor panel (fail fast, no hardware
touched), open the management link (the first of the unit's links that
answers, in declared order — a board that lost one route is still
managed over another), verify identity (serial pin or TOFU record),
prove every other declared link reaches the same silicon, DFU to the
pinned firmware, commission the Cyphal node-id on every CAN link,
deploy the panel — skipping whatever already matches. Firmware precedes
commissioning so a DFU reboot lands the node back on its current
address, not a freshly-staged node-id. Units fail independently; one
dead unit never blocks the rest of the suite.
"""
import hashlib
import importlib.util
import json
import logging
import os
import time
from dataclasses import dataclass, field
from typing import List, Optional

from nxs._generated_constants import CyphalDefaults
from nxs.client import SupportsTimeSync, estimate_and_push
from nxs.compiler import SensorDriver
from nxs.descriptor import load_driver
from nxs.suite import DRIVERS_DIR, FIRMWARE_DIR
from nxs.suite.drift import UnitDrift, detect_unit_drift, firmware_drift
from nxs.suite.firmware import find_image
from nxs.suite.schema import SuiteConfig, UnitSpec, parse_version
from nxs.suite.state import SuiteState
from nxs.image import serialize
from nxs.transports import open_client

log = logging.getLogger("nxs.suite")

PROBE_AFTER_DFU_S = 20.0


class DriverNotFound(Exception):
    """No driver source for a manifest `driver:` name; message lists the
    searched locations and the generation path."""


@dataclass
class UnitReport:
    name: str
    link: str
    ok: bool = True
    actions: List[str] = field(default_factory=list)
    error: str = ""


def load_unit_driver(name: str, drivers_dir: str = DRIVERS_DIR):
    """Resolve a driver class: the host drivers dir first, then the
    package built-ins."""
    path = os.path.join(drivers_dir, f"{name}.py")
    if os.path.exists(path):
        spec = importlib.util.spec_from_file_location(f"nxs_suite_drivers.{name}", path)
        if spec is None or spec.loader is None:
            raise DriverNotFound(f"{path}: not importable as a Python module")
        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
        except Exception as e:
            raise DriverNotFound(f"{path}: import failed: {e}") from None
        classes = [obj for obj in vars(module).values()
                   if isinstance(obj, type) and issubclass(obj, SensorDriver)
                   and obj.__module__ == module.__name__]
        if not classes:
            raise DriverNotFound(f"{path} defines no SensorDriver subclass")
        if len(classes) > 1:
            names = ", ".join(sorted(c.__name__ for c in classes))
            raise DriverNotFound(
                f"{path} defines {len(classes)} driver classes ({names}) — "
                f"one driver per file")
        return classes[0]
    try:
        return load_driver(name)
    except ImportError:
        raise DriverNotFound(
            f"driver '{name}' not found (searched {drivers_dir}/{name}.py and "
            f"the built-in nxs.drivers). Generate one from the sensor's "
            f"datasheet with the generate-sensor-driver skill — see "
            f"docs/specs/nxs-driver-development.md.") from None


def panel_hash(unit: UnitSpec) -> str:
    """Digest of the deploy-relevant intent: the sensor list and configs."""
    payload = [{"driver": s.driver, "config": s.config}
               for s in (unit.sensors or [])]
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True).encode()).hexdigest()


def switch_suite(cfg: SuiteConfig, state: SuiteState, *,
                dry_run: bool = False, only_unit: Optional[str] = None,
                accept_new_serial: bool = False, opener=open_client,
                firmware_dir: str = FIRMWARE_DIR,
                drivers_dir: str = DRIVERS_DIR) -> List[UnitReport]:
    """Reconcile the suite; returns one report per (selected) unit."""
    reports = []
    seen_serials: dict = {}
    for unit in cfg.units:
        if only_unit is not None and unit.name != only_unit:
            continue
        reports.append(_apply_unit(unit, state, dry_run=dry_run,
                                   accept_new_serial=accept_new_serial,
                                   opener=opener, firmware_dir=firmware_dir,
                                   drivers_dir=drivers_dir,
                                   seen_serials=seen_serials))
    declared = {u.name for u in cfg.units}
    orphans = sorted(set(state.unit_names()) - declared)
    if orphans and only_unit is None:
        log.info("state holds %d unit(s) absent from the manifest (%s) — "
                 "`nxs suite collect-garbage` drops them",
                 len(orphans), ", ".join(orphans))
    if not dry_run:
        try:
            state.save()  # no-op unless a unit recorded something
        except OSError as e:
            # The units are already converged on the hardware; only the
            # local bookkeeping failed, so warn rather than fail — the
            # TOFU serials re-learn on the next apply.
            log.warning("could not persist suite state to %s (%s); "
                        "TOFU serials re-learn on the next apply",
                        state.path, e)
    return reports


def _apply_unit(unit: UnitSpec, state: SuiteState, *, dry_run: bool,
                accept_new_serial: bool, opener, firmware_dir: str,
                drivers_dir: str,
                seen_serials: Optional[dict] = None) -> UnitReport:
    report = UnitReport(name=unit.name, link=unit.links[0].describe())
    would = "would " if dry_run else ""

    # Stage everything that can fail before any hardware is touched. The
    # broad catch is the per-unit isolation contract: a malformed host
    # driver file must fail this unit's report, never the whole run.
    try:
        panel = [(spec, load_unit_driver(spec.driver, drivers_dir)().compile(spec.config))
                 for spec in (unit.sensors or [])]
        image_path = (find_image(firmware_dir, unit.firmware)
                      if unit.firmware else None)
    except Exception as e:
        report.ok = False
        report.error = str(e) or type(e).__name__
        return report

    transport = None
    try:
        transport, mgmt_link, notes, error = _open_unit(unit, opener)
        if transport is None:
            report.ok = False
            report.error = error
            return report
        report.link = mgmt_link.describe()
        report.actions.extend(notes)

        ok, observed = _check_serial(unit, state, transport, report,
                                     dry_run=dry_run,
                                     accept_new_serial=accept_new_serial)
        if not ok:
            return report
        # Cross-unit guard: two declared units observing one silicon
        # would converge the same store with two intents. Fail the
        # later unit before it writes anything.
        reference = observed or unit.serial or state.unit(unit.name).get("serial")
        if seen_serials is not None and reference:
            other = seen_serials.get(reference)
            if other is not None and other != unit.name:
                report.ok = False
                report.error = (f"same silicon as unit {other!r} (serial "
                                f"{reference}) — one board is one unit; "
                                f"merge their links into one entry")
                return report
            seen_serials[reference] = unit.name
        if not _verify_edges(unit, mgmt_link, reference, report, opener):
            return report
        # Firmware before node-ID commissioning (the identity checks
        # above already ran), and before the panel: a DFU reboot must land
        # the node back on its *current* address (so `_reprobe` finds it),
        # not a freshly-commissioned one. Commissioning only stages the
        # next-boot node-ID, so deferring it past the flash is safe; the panel
        # reads come last so a DFU reboot never leaves them stale.
        if unit.firmware and not _converge_firmware(
                unit, state, transport, report, image_path, would, dry_run,
                needs_flash=firmware_drift(unit, state, transport)):
            return report
        _commission(unit, mgmt_link, transport, report, would, opener)
        if panel:
            digest = panel_hash(unit)
            drift = detect_unit_drift(unit, panel, transport, state, digest)
            _converge_panel(unit, state, transport, report, panel, digest,
                            drift, would, dry_run)
        elif unit.sensors is not None:
            # Explicit `sensors: []` — the declared panel is empty, so a
            # running driver or a populated store is drift to repair. An
            # absent key (None) leaves the panel unmanaged.
            _converge_empty_panel(unit, state, transport, report, would,
                                  dry_run)
        _converge_egress(unit, transport, report, would, dry_run)
        if not dry_run and isinstance(transport, SupportsTimeSync):
            # Seed the time discipline so the unit is synced from the
            # first converge; `nxs timesync` keeps it fresh thereafter.
            if (bound := estimate_and_push(transport)) is not None:
                report.actions.append(f"time sync seeded (±{bound} µs)")
        if not report.actions:
            report.actions.append("converged")
    except Exception as e:
        report.ok = False
        report.error = f"{type(e).__name__}: {e}"
    finally:
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass
    return report


def _open_unit(unit: UnitSpec, opener):
    """Open the unit's management transport: the first link that
    answers, in declared order. Links that were tried and stayed silent
    are noted (the report shows the board is managed over a fallback
    route); when none answers, the error names every route tried."""
    notes, down = [], []
    for link in unit.links:
        transport, note = _open_edge(link, opener)
        if transport is not None:
            if note:
                notes.append(note)
            notes.extend(f"edge {d}: down" for d in down)
            return transport, link, notes, ""
        down.append(link.describe() + (f" ({note})" if note else ""))
    return None, None, [], "no response on " + " or ".join(down)


def _open_edge(link, opener):
    """Open and probe one link. A cyphal-can link that is silent at its
    declared node-id is retried at the factory default — the
    uncommissioned-board path. Returns `(transport, note)` on success
    and `(None, reason)` when the link is down."""
    try:
        transport = opener(link.transport, **link.client_kwargs())
    except Exception as e:
        return None, str(e) or type(e).__name__
    if _probe_or_close(transport):
        return transport, ""

    factory = CyphalDefaults.DEFAULT_NODE_ID
    if link.transport == "cyphal-can" and link.node_id != factory:
        kwargs = dict(link.client_kwargs(), remote_node_id=factory)
        try:
            transport = opener(link.transport, **kwargs)
        except Exception as e:
            return None, str(e) or type(e).__name__
        if _probe_or_close(transport):
            return transport, f"found at factory node-id {factory}"
    return None, ""


def _verify_edges(unit: UnitSpec, mgmt_link, reference, report,
                  opener) -> bool:
    """Prove every non-management link reaches the same silicon before
    anything is written. A down link is reported, not fatal — the board
    is still converged over the management link. An *answering* link
    with a missing or different serial fails the unit: it may reach a
    different board (miswiring) or a merged-bus ghost."""
    for link in unit.links:
        if link is mgmt_link:
            continue
        transport, _ = _open_edge(link, opener)
        if transport is None:
            report.actions.append(f"edge {link.describe()}: down")
            continue
        try:
            try:
                raw = transport.read_serial()
            except Exception:
                raw = None
            seen = raw.hex() if raw else None
        finally:
            try:
                transport.close()
            except Exception:
                pass
        if reference is None or seen != reference:
            report.ok = False
            report.error = (f"cannot verify {link.describe()} reaches this "
                            f"board: it reports {seen or 'no serial'}, the "
                            f"management link saw {reference or 'no serial'}")
            return False
        report.actions.append(f"edge {link.describe()}: up")
    return True


def _probe_or_close(transport) -> bool:
    """Probe the transport, closing it on a negative answer *or* a raised
    probe (a serial link can throw) so a dead unit never leaks a handle."""
    try:
        if transport.probe():
            return True
    except Exception:
        pass
    try:
        transport.close()
    except Exception:
        pass
    return False


def _check_serial(unit: UnitSpec, state: SuiteState, transport, report,
                  *, dry_run: bool, accept_new_serial: bool):
    """Verify the board behind the management link is the one this unit
    expects; returns `(ok, observed_serial)`.

    An explicit manifest pin is authoritative — a mismatch always fails.
    Without a pin, the first contact records the serial (TOFU) and later
    applies verify against it; `--accept-new-serial` re-records after a
    deliberate board swap.
    """
    raw = transport.read_serial()
    if raw is None:
        if unit.serial:
            report.actions.append("serial pin not verifiable on this transport")
        return True, None
    seen = raw.hex()

    if unit.serial:
        if seen != unit.serial:
            report.ok = False
            report.error = (f"serial mismatch: manifest pins {unit.serial}, "
                            f"device reports {seen} — update the unit's "
                            f"serial pin in the manifest if this board swap "
                            f"is intended")
            return False, seen
        return True, seen

    recorded = state.unit(unit.name).get("serial")
    if recorded is None:
        if not dry_run:
            state.record(unit.name, serial=seen)
        report.actions.append(f"{'would record' if dry_run else 'recorded'} "
                              f"serial {seen}")
        return True, seen
    if seen != recorded:
        if accept_new_serial:
            if not dry_run:
                state.record(unit.name, serial=seen)
            report.actions.append(f"accepted new serial {seen} "
                                  f"(was {recorded})")
            return True, seen
        report.ok = False
        report.error = (f"serial changed: recorded {recorded}, device reports "
                        f"{seen} — re-run with --accept-new-serial if this "
                        f"board swap is intended")
        return False, seen
    return True, seen


def _commission(unit: UnitSpec, mgmt_link, transport, report, would: str,
                opener):
    """Persist the declared Cyphal node-id on every CAN link whose
    device differs. The management transport serves its own link; other
    CAN links reopen briefly. A down link was already reported by
    `_verify_edges` and converges on a later apply."""
    for link in unit.links:
        if link.transport != "cyphal-can":
            continue
        if link is mgmt_link:
            _commission_edge(link, transport, report, would)
            continue
        edge_transport, _ = _open_edge(link, opener)
        if edge_transport is None:
            continue
        try:
            _commission_edge(link, edge_transport, report, would)
        finally:
            try:
                edge_transport.close()
            except Exception:
                pass


def _commission_edge(link, transport, report, would: str):
    identity = transport.read_identity()
    current = identity.get("node_addr")
    if current == link.node_id:
        return
    if not would:
        transport.commission(node_addr=link.node_id)
    report.actions.append(f"{would}commission node-id {link.node_id} "
                          f"(now {current}; adopts on next power-cycle)")


def _converge_firmware(unit: UnitSpec, state: SuiteState, transport, report,
                       image_path: str, would: str, dry_run: bool, *,
                       needs_flash: bool) -> bool:
    """DFU to the pinned version when drift detection says the device
    provably (or possibly) runs something else.

    `needs_flash` is `UnitDrift.fw`, computed by `drift.firmware_drift`,
    which encodes the pin rule: every transport serves major.minor at
    best, so the state file refines to full-pin granularity — a matching
    record means this tool already flashed that exact pin, an unknown
    version flashes once and records, and the pin is the declared truth.
    """
    if not needs_flash:
        return True
    want = parse_version(unit.firmware)
    device = transport.read_fw_version()

    report.actions.append(
        f"{would}flash firmware {unit.firmware} "
        f"(device reports {device or 'no version'})")
    if dry_run:
        return True

    transport.push_image(image_path)
    if not _reprobe(transport, PROBE_AFTER_DFU_S):
        report.ok = False
        report.error = f"device did not return within {PROBE_AFTER_DFU_S:.0f}s after DFU"
        return False
    after = transport.read_fw_version()
    if after and parse_version(after)[:2] != want[:2]:
        report.ok = False
        report.error = f"DFU verify failed: device reports {after}, pinned {unit.firmware}"
        return False
    state.record(unit.name, fw_version=unit.firmware)
    return True


def _converge_panel(unit: UnitSpec, state: SuiteState, transport, report,
                    panel, digest: str, drift: UnitDrift, would: str,
                    dry_run: bool):
    """Repair proportionately to the drift: wrong/missing driver or a
    changed panel shape redeploys the store; param-only drift is
    retuned in place with `set_param` — no store wipe, no sample gap."""
    if not (drift.driver or drift.shape):
        for name, (desired, live) in sorted(drift.params.items()):
            report.actions.append(f"{would}retune {name} {live}→{desired}")
            if not dry_run:
                transport.set_param(name, desired)
        return

    names = [compiled.name for _, compiled in panel]
    report.actions.append(f"{would}deploy panel: {', '.join(names)}")
    if dry_run:
        return

    transport.clear_store()
    for slot, (_, compiled) in enumerate(panel):
        transport.upload_image(serialize(compiled))
        transport.save_slot(slot)
    transport.vm_run()
    active = transport.read_active_slot()
    if 0 <= active < len(panel):
        report.actions.append(f"active: {names[active]} (slot {active})")
    else:
        report.actions.append("no driver probed a sensor yet "
                              "(check wiring, then `nxs suite status`)")
    state.record(unit.name, panel_hash=digest)


def _converge_empty_panel(unit: UnitSpec, state: SuiteState, transport,
                          report, would: str, dry_run: bool):
    """Converge a declared-empty panel: stop the driver and clear the
    store when either is present; record the empty panel hash."""
    if transport.read_store_count() == 0 and not transport.read_driver_name():
        return
    report.actions.append(f"{would}clear panel (declared empty)")
    if dry_run:
        return
    transport.vm_stop()
    transport.vm_reset()
    transport.clear_store()
    state.record(unit.name, panel_hash=panel_hash(unit))


def _converge_egress(unit: UnitSpec, transport, report, would: str,
                     dry_run: bool):
    """Repair declared egress factors in place, then persist them (the
    same Save the commissioning path ends with, so the factors survive
    a power-cycle like every other converged intent)."""
    from nxs.suite.drift import egress_drift

    drift = egress_drift(unit, transport)
    if not drift:
        return
    for key, (want, live) in sorted(drift.items()):
        label = "decimation" if key == "device" else f"decimation[{key}]"
        report.actions.append(f"{would}retune {label} {live}→{want}")
        if not dry_run:
            transport.write_decimation(
                want, subject=None if key == "device" else key)
    if not dry_run:
        transport.commission()


def _reprobe(transport, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            if transport.probe():
                return True
        except Exception:
            pass
        time.sleep(0.5)
    return False
