"""`nxs switch`: converge every declared unit to its manifest entry.
Compiles the panel, opens the management link, verifies identity and every
other link, then DFU, commission, and deploy, skipping what already matches."""
import hashlib
import importlib.util
import sys
import json
import logging
import os
import time
from dataclasses import dataclass, field
from typing import List, Optional

from nxs._generated_constants import CyphalDefaults, RunnerStates
from nxs.client import (DFU_REBOOT_TIMEOUT_S, SupportsTimeSync, await_driver_up,
                        await_reachable, estimate_and_push)
from nxs.compiler import SensorDriver
from nxs.descriptor import load_driver
from nxs.suite import DRIVERS_DIR, FIRMWARE_DIR
from nxs.suite.drift import UnitDrift, detect_unit_drift, firmware_drift
from nxs.suite.firmware import find_image
from nxs.suite.schema import (SuiteConfig, UnitSpec, device_runs,
                              parse_version)
from nxs.suite.state import SuiteState
from nxs.image import serialize
from nxs.client import contract_mismatch, exc_detail, import_failure_detail
from nxs.transports import open_client

log = logging.getLogger("nxs.suite")


class DriverNotFound(Exception):
    """No driver source for a manifest `driver:` name; message lists the
    searched locations and the generation path."""


@dataclass
class UnitReport:
    """One unit's converge result: `actions` done to the device, `notes`
    observations independent of drift. `converged` derives from `actions` alone."""
    name: str
    link: str
    ok: bool = True
    actions: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)
    error: str = ""


def _is_camera_personality(directory: str, name: str) -> bool:
    """Whether the store entry is a camera personality; both kinds share the
    `<name>/<name>.py` layout and only the descriptor beside the module tells."""
    import yaml

    from nxs.personality import CAMERA, PersonalityError, kind_of

    for candidate in (os.path.join(directory, name, f"{name}.yaml"),
                      os.path.join(directory, f"{name}.yaml")):
        if not os.path.exists(candidate):
            continue
        try:
            with open(candidate) as handle:
                doc = yaml.safe_load(handle)
            return kind_of(doc if isinstance(doc, dict) else {},
                           candidate) == CAMERA
        except (OSError, yaml.YAMLError, PersonalityError):
            return False          # unreadable: let the loader say why
    return False


def known_driver_modules(drivers_dir: Optional[str] = None) -> List[str]:
    """The unit personalities the tool can load: the store first, then the package
    built-ins; a store name shadows a built-in. Camera personalities are skipped."""
    import pkgutil

    import nxs.drivers
    from nxs.suite import personality_dirs

    names: List[str] = []
    for directory in personality_dirs(drivers_dir):
        if not os.path.isdir(directory):
            continue
        for entry in sorted(os.listdir(directory)):
            if entry.startswith("_"):
                continue
            if entry.endswith(".py"):
                name = entry[:-3]
            elif os.path.exists(os.path.join(directory, entry, f"{entry}.py")):
                name = entry
            else:
                continue
            if not _is_camera_personality(directory, name):
                names.append(name)
    names += [m.name for m in pkgutil.iter_modules(nxs.drivers.__path__)
              if not m.name.startswith("_")]
    seen = set()
    return [n for n in names if not (n in seen or seen.add(n))]


def module_for_driver_name(active: str, drivers_dir: Optional[str] = None):
    """The personality module whose class compiles to the driver name the
    device reports (`Fxos8700` -> `fxos8700`), or None."""
    for name in known_driver_modules(drivers_dir):
        try:
            cls = load_unit_driver(name, drivers_dir)
        except DriverNotFound:
            continue
        if cls.__name__ == active:
            return name
    return None


def load_unit_driver(name: str, drivers_dir: Optional[str] = None):
    """Resolve a driver class: the personality store first (`<name>/<name>.py`
    or a flat `<name>.py`), then the package built-ins."""
    from nxs.suite import personality_dirs, personality_file

    path = personality_file(name, "py", drivers_dir)
    if path is not None:
        spec = importlib.util.spec_from_file_location(f"nxs_suite_drivers.{name}", path)
        if spec is None or spec.loader is None:
            raise DriverNotFound(f"{path}: not importable as a Python module")
        module = importlib.util.module_from_spec(spec)
        # Registered like any imported module: the driver's class finds its
        # own module (and its sibling descriptor) through sys.modules.
        sys.modules[spec.name] = module
        try:
            spec.loader.exec_module(module)
        except Exception as e:
            sys.modules.pop(spec.name, None)
            raise DriverNotFound(
                f"{path}: import failed: "
                f"{import_failure_detail(e, str(path))}") from e
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
        searched = ", ".join(f"{d}/{name}/{name}.py" for d in personality_dirs(drivers_dir))
        raise DriverNotFound(
            f"personality '{name}' not found (searched {searched} and the built-in "
            f"nxs.drivers). Generate one from the sensor's datasheet with "
            f"the generate-sensor-personality skill, then nxs personality install it.") from None


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
                drivers_dir: Optional[str] = None) -> List[UnitReport]:
    """Reconcile the suite; returns one report per (selected) unit. A
    None `drivers_dir` resolves personalities through the store chain."""
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
        log.info("state holds %d unit(s) absent from the manifest (%s)",
                 len(orphans), ", ".join(orphans))
    if not dry_run:
        try:
            state.save()  # no-op unless a unit recorded something
        except OSError as e:
            # The hardware is converged; only the local bookkeeping failed, so
            # warn. The TOFU serials re-learn on the next apply.
            log.warning("could not persist suite state to %s (%s); "
                        "TOFU serials re-learn on the next apply",
                        state.path, e)
    return reports


def _apply_unit(unit: UnitSpec, state: SuiteState, *, dry_run: bool,
                accept_new_serial: bool, opener, firmware_dir: str,
                drivers_dir: Optional[str],
                seen_serials: Optional[dict] = None) -> UnitReport:
    report = UnitReport(name=unit.name, link=unit.links[0].describe())
    would = "would " if dry_run else ""

    # Stage everything that can fail before any hardware is touched; a
    # malformed driver file fails this unit's report, never the whole run.
    try:
        panel = [(spec, load_unit_driver(spec.driver, drivers_dir)().compile(spec.config))
                 for spec in (unit.sensors or [])]
        pin = unit.firmware or assets_pin(firmware_dir)
        image_path = find_image(firmware_dir, pin) if pin else None
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
        report.notes.extend(notes)

        ok, observed = _check_serial(unit, state, transport, report,
                                     dry_run=dry_run,
                                     accept_new_serial=accept_new_serial)
        if not ok:
            return report
        # Two declared units observing one silicon would converge the same
        # store with two intents; fail the later unit before it writes.
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
        # Firmware first: a DFU reboot must land on the current address, and
        # commissioning only stages the next-boot node-ID.
        mismatch = contract_mismatch(transport, unreadable_is_skew=True)
        if mismatch and not pin:
            report.ok = False
            report.error = (mismatch + " — pin firmware for this unit to "
                            "upgrade it, or use a matching nxs")
            return report
        if mismatch:
            report.actions.append(mismatch + " — converging firmware first")
        # A contract mismatch is drift whatever the versions say: a v1 and a
        # v2 image can report the same `fw`.
        if pin and not _converge_firmware(
                unit, state, transport, report, image_path, would, dry_run, pin,
                needs_flash=(firmware_drift(unit, state, transport, pin)
                             or bool(mismatch))):
            return report
        if mismatch and not dry_run:
            mismatch = contract_mismatch(transport, unreadable_is_skew=True)
            if mismatch:
                report.ok = False
                report.error = (mismatch + " after flashing the pinned "
                                "firmware — that image does not speak this "
                                "contract either")
                return report
        _commission(unit, mgmt_link, transport, report, would, opener)
        _push_orientation(unit, transport, report, would)
        if panel:
            digest = panel_hash(unit)
            drift = detect_unit_drift(unit, panel, transport, state, digest)
            _converge_panel(unit, state, transport, report, panel, digest,
                            drift, would, dry_run)
        elif unit.sensors is not None:
            # Explicit `sensors: []`: a running driver or a populated store is
            # drift to repair. An absent key leaves the panel unmanaged.
            _converge_empty_panel(unit, state, transport, report, would,
                                  dry_run)
        _converge_egress(unit, transport, report, would, dry_run)
        # Counted before the seed: the time sync runs on every switch, so
        # folding it in would leave `converged` unreachable.
        if not dry_run and isinstance(transport, SupportsTimeSync):
            # Seed the time discipline on the first converge. A failed seed is
            # a note, not a unit failure; `nxs timesync` re-seeds later.
            try:
                if (bound := estimate_and_push(transport)) is not None:
                    report.notes.append(f"time sync seeded (±{bound} µs)")
            except Exception as e:
                report.notes.append(f"time sync not seeded ({exc_detail(e)}) "
                                    f"— `nxs timesync` will retry")
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
    """Open the first link that answers, in declared order. Silent links are
    noted; when none answers, the error names every route."""
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
    """Open and probe one link, returning `(transport, note)` or `(None,
    reason)`. A silent cyphal-can link is retried at the factory default node-id."""
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
    """Prove every non-management link reaches the same silicon before anything
    is written. A down link is a note; a different serial fails the unit."""
    for link in unit.links:
        if link is mgmt_link:
            continue
        transport, _ = _open_edge(link, opener)
        if transport is None:
            report.notes.append(f"edge {link.describe()}: down")
            continue
        unreadable = None
        try:
            try:
                raw = transport.read_serial()
            except Exception as e:
                # A NACKed transaction is not evidence of miswiring; keep the
                # two apart so a transient read never re-pins a good harness.
                raw, unreadable = None, e
            seen = raw.hex() if raw else None
        finally:
            try:
                transport.close()
            except Exception:
                pass
        if unreadable is not None:
            report.ok = False
            report.error = (f"edge {link.describe()}: serial unreadable "
                            f"({unreadable}) — retry; this is not a "
                            f"miswiring verdict")
            return False
        if reference is None or seen != reference:
            report.ok = False
            report.error = (f"cannot verify {link.describe()} reaches this "
                            f"board: it reports {seen or 'no serial'}, the "
                            f"management link saw {reference or 'no serial'}")
            return False
        report.notes.append(f"edge {link.describe()}: up")
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
    """Verify the board behind the management link, returning `(ok, serial)`.
    A manifest pin is authoritative; without one, first contact records the
    serial and `--accept-new-serial` re-records it after a board swap."""
    raw = transport.read_serial()
    if raw is None:
        if unit.serial:
            report.notes.append("serial pin not verifiable on this transport")
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
    """Persist the declared Cyphal node-id on every CAN link whose device
    differs. A down link converges on a later apply."""
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


def _push_orientation(unit: UnitSpec, transport, report, would: str):
    """Persist the declared mounting orientation when the device differs. Only
    the orientation byte moves; the solved per-silicon affines are untouched."""
    from nxs.client import SupportsCalibration, rotation_code, rotation_name
    if unit.orientation is None or not isinstance(transport, SupportsCalibration):
        return
    declared = rotation_code(unit.orientation)
    current = transport.read_calibration().orientation
    if current == declared:
        return
    if not would:
        transport.set_orientation(declared)
    report.actions.append(f"{would}set orientation {unit.orientation} "
                          f"(was {rotation_name(current)})")


def _converge_firmware(unit: UnitSpec, state: SuiteState, transport, report,
                       image_path: str, would: str, dry_run: bool, pin: str, *,
                       needs_flash: bool) -> bool:
    """DFU to `pin` (the unit's own, else the assets' release) when
    `needs_flash` (`UnitDrift.fw`) says the device runs something else."""
    if not needs_flash:
        return True
    want = parse_version(pin)
    device = transport.read_fw_version()

    report.actions.append(
        f"{would}flash firmware {pin} "
        f"(device reports {device or 'no version'})")
    if dry_run:
        return True

    transport.push_image(image_path)
    if not await_reachable(transport, DFU_REBOOT_TIMEOUT_S):
        report.ok = False
        report.error = f"device did not return within {DFU_REBOOT_TIMEOUT_S:.0f}s after DFU"
        return False
    after = transport.read_fw_version()
    # A full build identity proves the patch; recording the pin over a
    # wrong-patch boot would mask the failed update.
    if after and device_runs(after, want) is False:
        report.ok = False
        report.error = f"DFU verify failed: device reports {after}, pinned {pin}"
        return False
    state.record(unit.name, fw_version=pin)
    return True


def tool_base_version() -> Optional[str]:
    """The release this tool is, as a bare M.N.P; None in a source checkout."""
    from nxs import __version__
    base = str(__version__).split("+", 1)[0].lstrip("v").split("rc", 1)[0]
    return base if base and base != "0.0.0" else None


def assets_pin(firmware_dir: str) -> Optional[str]:
    """The firmware pin a unit without one takes: the tool's own release,
    when the installed assets hold that image; None otherwise, and the
    unit's firmware is left as it runs."""
    base = tool_base_version()
    if base is None:
        return None
    try:
        find_image(firmware_dir, base)
    except (FileNotFoundError, OSError, ValueError):
        return None
    return base


def _converge_panel(unit: UnitSpec, state: SuiteState, transport, report,
                    panel, digest: str, drift: UnitDrift, would: str,
                    dry_run: bool):
    """Repair proportionately: a wrong or missing driver or a changed panel
    shape redeploys the store; param-only drift is retuned in place."""
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
    runner = await_driver_up(transport)
    active = transport.read_active_slot()
    if runner == RunnerStates.RunnerState.MEASURING:
        if 0 <= active < len(panel):
            report.actions.append(f"active: {names[active]} (slot {active})")
        else:
            # A fresh deploy runs the just-uploaded RAM image, which carries no
            # slot number until a reboot loads slot 0; MEASURING is the success
            # signal, and the running driver names itself.
            report.actions.append(
                f"active: {transport.read_driver_name() or names[-1]} (running)")
    elif runner == RunnerStates.RunnerState.PROBE_FAILED:
        # The runner parks in PROBE_FAILED until a host command intervenes, so
        # this is a verdict: the deploy did not realize the manifest.
        report.ok = False
        report.error = ("no sensor answered the deployed driver "
                        "(check wiring, then `nxs status`)")
    else:
        report.ok = False
        report.error = (f"driver did not come up — runner is "
                        f"{RunnerStates.RunnerState._NAMES.get(runner, runner)}")
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
    """Repair declared egress factors in place, then persist them so they
    survive a power-cycle."""
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


