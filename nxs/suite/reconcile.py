"""`nxs switch`: converge every declared unit to its manifest entry.
Compiles the panel, opens the management link, verifies identity and every
other link, then DFU, commission, and deploy, skipping what already matches."""
import hashlib
import json
import logging
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from nxs._generated_constants import CyphalDefaults, RunnerStates
from nxs.client import SupportsTimeSync, estimate_and_push, push_and_verify
from nxs.compiler import ClickPersonality
from nxs.suite import FIRMWARE_DIR
from nxs.suite.drift import (STORE_SLOTS, UnitDrift, detect_unit_drift,
                             click_slots, firmware_drift)
from nxs.suite.firmware import find_image, image_identity
from nxs.suite.schema import (SuiteConfig, UnitSpec, device_runs,
                              parse_version)
from nxs.suite.state import SuiteState
from nxs.image import serialize
from nxs.client import contract_mismatch, exc_detail, import_failure_detail
from nxs.time_sync import await_driver_up, park
from nxs import transports

log = logging.getLogger("nxs.suite")


class ClickPersonalityNotFound(Exception):
    """No personality source for a manifest `personality:` name; message lists the
    searched locations and the generation path."""


@dataclass
class UnitReport:
    """One unit's converge result: `actions` done to the device, `notes`
    observations independent of drift. `converged` derives from `actions` alone.
    `restarted` says the run pushed a firmware image, which restarts the unit
    (on a dry run, that it would)."""
    name: str
    link: str
    ok: bool = True
    actions: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)
    error: str = ""
    restarted: bool = False


def _is_camera_personality(directory: str, name: str) -> bool:
    """Whether the store entry is a cam personality; both kinds share the
    `<name>/<name>.py` layout and only the descriptor beside the module tells."""
    import yaml

    from nxs.personality import CAM, PersonalityError, kind_of

    for candidate in (os.path.join(directory, name, f"{name}.yaml"),
                      os.path.join(directory, f"{name}.yaml")):
        if not os.path.exists(candidate):
            continue
        try:
            with open(candidate) as handle:
                doc = yaml.safe_load(handle)
            return kind_of(doc if isinstance(doc, dict) else {},
                           candidate) == CAM
        except (OSError, yaml.YAMLError, PersonalityError):
            return False          # unreadable: let the loader say why
    return False


def known_click_personalities(personalities_dir: Optional[str] = None) -> List[str]:
    """The click personalities the tool can load: the store first, then the
    wheel's own; a store name shadows a shipped one. Cam personalities are skipped."""
    from nxs.suite import personality_dirs

    names: List[str] = []
    for directory in personality_dirs(personalities_dir):
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
    seen = set()
    return [n for n in names if not (n in seen or seen.add(n))]


def click_personality_for(active: str, personalities_dir: Optional[str] = None):
    """The personality module whose class compiles to the personality name the
    device reports (`Fxos8700` -> `fxos8700`), or None."""
    for name in known_click_personalities(personalities_dir):
        try:
            cls = load_click_personality(name, personalities_dir)
        except ClickPersonalityNotFound:
            continue
        if cls.__name__ == active:
            return name
    return None


def load_click_personality(name: str, personalities_dir: Optional[str] = None):
    """Resolve a click personality's class: `<name>/<name>.py` (or a flat
    `<name>.py`) in the store first, then among the wheel's own."""
    from nxs.personality import load_source
    from nxs.suite import personality_dirs, personality_file

    path = personality_file(name, "py", personalities_dir)
    if path is None:
        searched = ", ".join(f"{d}/{name}/{name}.py" for d in personality_dirs(personalities_dir))
        raise ClickPersonalityNotFound(
            f"personality '{name}' not found (searched {searched}). Generate one from the "
            f"sensor's datasheet with the generate-click-personality skill, then "
            f"nxs personality install it.")
    try:
        module = load_source(f"nxs_click_personalities.{name}", str(path))
    except Exception as e:
        raise ClickPersonalityNotFound(
            f"{path}: import failed: "
            f"{import_failure_detail(e, str(path))}") from e
    classes = [obj for obj in vars(module).values()
               if isinstance(obj, type) and issubclass(obj, ClickPersonality)
               and obj.__module__ == module.__name__]
    if not classes:
        raise ClickPersonalityNotFound(f"{path} defines no ClickPersonality subclass")
    if len(classes) > 1:
        names = ", ".join(sorted(c.__name__ for c in classes))
        raise ClickPersonalityNotFound(
            f"{path} defines {len(classes)} personality classes ({names}) — "
            f"one personality per file")
    return classes[0]


def panel_hash(unit: UnitSpec) -> str:
    """Digest of the deploy-relevant intent: the sensor list and configs."""
    payload = [{"personality": s.personality, "config": s.config}
               for s in (unit.sensors or [])]
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True).encode()).hexdigest()


def switch_suite(cfg: SuiteConfig, state: SuiteState, *,
                dry_run: bool = False, only_unit: Optional[str] = None,
                accept_new_serial: bool = False, opener=None,
                firmware_dir: str = FIRMWARE_DIR,
                personalities_dir: Optional[str] = None,
                skip: Optional[Dict[str, List[str]]] = None) -> List[UnitReport]:
    """Reconcile the suite; returns one report per (selected) unit. A
    None `personalities_dir` resolves personalities through the store chain, and
    the opener defaults to the one `nxs.transports` holds at the call. A
    unit in `skip` is not touched: its report carries the lines it was
    skipped with (a pod where it straps, behind a port that did not come up)."""
    opener = opener or transports.open_client
    reports = []
    seen_serials: dict = {}
    for unit in cfg.units:
        if only_unit is not None and unit.name != only_unit:
            continue
        if skip and unit.name in skip:
            reports.append(UnitReport(name=unit.name, link=unit.links[0].describe(), ok=False,
                                      error="\n    ".join(skip[unit.name])))
            continue
        reports.append(_apply_unit(unit, state, dry_run=dry_run,
                                   accept_new_serial=accept_new_serial,
                                   opener=opener, firmware_dir=firmware_dir,
                                   personalities_dir=personalities_dir,
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
                personalities_dir: Optional[str],
                seen_serials: Optional[dict] = None) -> UnitReport:
    report = UnitReport(name=unit.name, link=unit.links[0].describe())
    would = "would " if dry_run else ""

    # Stage everything that can fail before any hardware is touched; a
    # malformed personality file fails this unit's report, never the whole run.
    try:
        panel = [(spec, load_click_personality(spec.personality, personalities_dir)().compile(spec.config))
                 for spec in (unit.sensors or [])]
        pin = unit.firmware or assets_pin(firmware_dir)
        image_path = (find_image(firmware_dir, pin, None if unit.firmware else tool_build_identity())
                      if pin else None)
        image = image_identity(image_path) if image_path else None
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
                unit, state, transport, report, image_path, image, would, dry_run, pin,
                needs_flash=(firmware_drift(unit, state, transport, pin, image)
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
            # Explicit `sensors: []`: a running personality or a populated store is
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
                       image_path: str, image: Optional[str], would: str,
                       dry_run: bool, pin: str, *, needs_flash: bool) -> bool:
    """DFU to `pin` (the unit's own, else the assets' release) when
    `needs_flash` (`UnitDrift.fw`) says the device runs something else, and
    judge the push as `push-fw` does: the unit fails unless it boots the
    pushed image, proven by `image`, the build identity the image carries,
    where it carries one."""
    if not needs_flash:
        if image is not None and not dry_run and state.unit(unit.name).get("fw_version") != pin:
            # The identity proves the pinned image: `status`'s rule reads the record.
            state.record(unit.name, fw_version=pin)
        return True
    device = transport.read_fw_version()

    report.actions.append(
        f"{would}flash firmware {pin} "
        f"(device reports {device or 'no version'})")
    report.restarted = True
    if dry_run:
        return True

    ok, line, after = push_and_verify(transport, image_path, pin, before=device)
    # Without an identity in the image, a boot into another version than the
    # pin is a failed update, whatever else changed.
    if ok and image is None and after and device_runs(after, parse_version(pin)) is False:
        ok, line = False, (f"✗ rejected or reverted: the device reports {after} after "
                           f"the push of {pin}")
    if not ok:
        report.ok = False
        report.error = line.removeprefix("✗ ")
        return False
    state.record(unit.name, fw_version=pin)
    return True


def tool_base_version() -> Optional[str]:
    """The release this tool is, as a bare M.N.P; None in a source checkout."""
    from nxs import __version__
    base = str(__version__).split("+", 1)[0].lstrip("v").split("rc", 1)[0]
    return base if base and base != "0.0.0" else None


def tool_build_identity() -> Optional[str]:
    """The build identity the firmware built with this wheel carries: the
    wheel's own build record (`v1.1.0-rc1-174-gd7bb22bf` on a build between
    tags, the tag on a release build), else the tag its version names
    (`v1.1.0-rc1`); None in a source checkout, which records no build."""
    from nxs import __version__
    from nxs.assets_cli import release_tag, tool_build

    if tool_base_version() is None:
        return None
    if (build := tool_build()) is not None:
        return build
    try:
        return release_tag(str(__version__).split("+", 1)[0].lstrip("v"))
    except ValueError:
        return None


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
    """Repair proportionately: a wrong or missing personality or a changed panel
    shape redeploys the store; param-only drift is retuned in place."""
    if not (drift.personality or drift.shape):
        for name, (desired, live) in sorted(drift.params.items()):
            report.actions.append(f"{would}retune {name} {live}→{desired}")
            if not dry_run:
                transport.set_param(name, desired)
        return

    names = [compiled.name for _, compiled in panel]
    report.actions.append(f"{would}deploy panel: {', '.join(names)}")
    if dry_run:
        return

    # Read before anything stops: a store a session holds, or one too full,
    # refuses the deploy with the unit as it runs.
    room = _room(transport)
    if room < len(panel):
        report.ok = False
        report.error = (f"the store has room for {room} of the panel's "
                        f"{len(panel)} personalities")
        return
    # Parked before the store changes: a runner left running reloads from it
    # on its own (a probe retry, the watchdog), and its verdict must not
    # outlive it.
    park(transport)
    slots = _clear_click_slots(transport)[:len(panel)]
    for slot, (_, compiled) in zip(slots, panel):
        transport.upload_image(serialize(compiled))
        transport.save_slot(slot)
    transport.vm_run()
    runner = await_driver_up(transport)
    active = transport.read_active_slot()
    if runner == RunnerStates.RunnerState.MEASURING:
        if active in slots:
            report.actions.append(f"active: {names[slots.index(active)]} (slot {active})")
        else:
            # A fresh deploy runs the just-uploaded RAM image, which carries no
            # slot number until a reboot loads the first sensor slot; MEASURING
            # is the success signal, and the running personality names itself.
            report.actions.append(
                f"active: {transport.read_personality_name() or names[-1]} (running)")
    elif runner == RunnerStates.RunnerState.PROBE_FAILED:
        # The runner parks in PROBE_FAILED until a host command intervenes, so
        # this is a verdict: the deploy did not realize the manifest.
        report.ok = False
        report.error = ("no sensor answered the deployed personality "
                        "(check wiring, then `nxs status`)")
    else:
        report.ok = False
        report.error = (f"the personality did not come up — runner is "
                        f"{RunnerStates.RunnerState._NAMES.get(runner, runner)}")
    state.record(unit.name, panel_hash=digest)


def _room(transport) -> int:
    """How many slots a panel can take once the click personalities are
    cleared: every slot but a cam personality's."""
    from nxs.client import SupportsSlotPeek

    if not isinstance(transport, SupportsSlotPeek):
        return STORE_SLOTS
    sensors = set(click_slots(transport))
    return sum(slot in sensors or transport.read_slot_info(slot) is None
               for slot in range(STORE_SLOTS))


def _clear_click_slots(transport) -> List[int]:
    """Delete every click personality's slot, the last first so the others
    keep their index, and leave a cam personality's in place: it is the
    camera steps'. Returns the empty slots after, lowest first, where a
    panel lands."""
    from nxs.client import SupportsSlotPeek

    for slot in reversed(click_slots(transport)):
        transport.delete_slot(slot)
    if not isinstance(transport, SupportsSlotPeek):
        return list(range(STORE_SLOTS))
    return [slot for slot in range(STORE_SLOTS) if transport.read_slot_info(slot) is None]


def _converge_empty_panel(unit: UnitSpec, state: SuiteState, transport,
                          report, would: str, dry_run: bool):
    """Converge a declared-empty panel: stop the personality and clear the
    sensor slots when either is present; record the empty panel hash."""
    if not click_slots(transport) and not transport.read_personality_name():
        return
    report.actions.append(f"{would}clear panel (declared empty)")
    if dry_run:
        return
    transport.vm_stop()
    transport.vm_reset()
    _clear_click_slots(transport)
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


