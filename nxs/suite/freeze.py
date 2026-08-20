"""`nxs suite freeze`: adopt a unit's live tuning into the manifest.

The inverse of `apply`: where apply makes the device match the
manifest, freeze makes the manifest match the device. The flow it
serves: tune imperatively (`nxs set`, driver experiments) until the
unit performs, then freeze the result declaratively.

Freeze captures the active driver's full live parameter set — every
tunable, not just non-defaults, so the frozen config reproduces the
performance even if a later driver revision changes its defaults. It
updates only units and drivers the manifest already declares; new
topology is `scan --init`'s job, and an undeclared driver is a
one-line manifest edit first. The write is surgical: only the matching
sensor's `config:` (and `firmware:` under --pin-firmware) changes;
comments and layout survive via a round-trip YAML edit.
"""
import os
import sys
from dataclasses import dataclass, field
from typing import List, Optional

from nxs.client import exc_detail
from nxs.suite import DRIVERS_DIR
from nxs.suite.reconcile import load_unit_driver
from nxs.suite.schema import (ManifestError, SuiteConfig, UnitSpec,
                              device_proves_patch, parse_device_version,
                              parse_version)
from nxs.transports import open_client


@dataclass
class FreezeReport:
    name: str
    ok: bool = True
    changed: bool = False
    sensor_index: int = -1
    config: dict = field(default_factory=dict)
    egress_decimation: Optional[int] = None
    egress_subjects: dict = field(default_factory=dict)
    firmware: Optional[str] = None
    orientation: Optional[str] = None
    actions: List[str] = field(default_factory=list)
    error: str = ""


def freeze_suite(cfg: SuiteConfig, config_path: str, *,
                 only_unit: Optional[str] = None, dry_run: bool = False,
                 pin_firmware: bool = False, opener=open_client,
                 drivers_dir: str = DRIVERS_DIR) -> List[FreezeReport]:
    """Freeze the selected unit(s); write the manifest once at the end."""
    reports = []
    for unit in cfg.units:
        if only_unit is not None and unit.name != only_unit:
            continue
        reports.append(_freeze_unit(unit, pin_firmware=pin_firmware,
                                    opener=opener, drivers_dir=drivers_dir))
    changed = [r for r in reports if r.ok and r.changed]
    if changed and not dry_run:
        _write_manifest(config_path, changed)
    return reports


def _freeze_unit(unit: UnitSpec, *, pin_firmware: bool, opener,
                 drivers_dir: str) -> FreezeReport:
    report = FreezeReport(name=unit.name)
    try:
        panel = [(spec, load_unit_driver(spec.driver, drivers_dir)().compile(spec.config))
                 for spec in (unit.sensors or [])]
    except Exception as e:
        report.ok = False
        report.error = str(e) or type(e).__name__
        return report

    transport = None
    try:
        # First link that answers, in declared order — freeze reads the
        # running tuning, which is the same over any route to the board.
        for link in unit.links:
            candidate = None
            try:
                candidate = opener(link.transport, **link.client_kwargs())
                if candidate.probe():
                    transport = candidate
                    break
                # probe() answers False for a refused bus as well as for
                # absent hardware; the transport kept which it was.
                if reason := candidate.probe_failure_detail():
                    print(f"freeze: {link.describe()}: {reason}",
                          file=sys.stderr)
            except Exception as e:
                print(f"freeze: {link.describe()}: {exc_detail(e)}",
                      file=sys.stderr)
            if candidate is not None:
                try:
                    candidate.close()
                except Exception:
                    pass
        if transport is None:
            report.ok = False
            report.error = ("no response on "
                            + " or ".join(l.describe() for l in unit.links))
            return report

        active = transport.read_driver_name()
        if not active:
            report.ok = False
            report.error = "no active driver to freeze"
            return report
        index = next((i for i, (_, compiled) in enumerate(panel)
                      if compiled.name == active), None)
        if index is None:
            report.ok = False
            report.error = (f"active driver {active} is not declared for "
                            f"this unit — declare it in the manifest (or "
                            f"redeploy with `nxs suite switch`) before "
                            f"freezing")
            return report

        live = {p["name"]: p["current"] for p in transport.read_capabilities()}
        _, compiled = panel[index]
        desired = {p.name: p.current for p in compiled.params}
        if set(live) != set(desired):
            # A same-named driver serving a different parameter set is a
            # different revision. Adopting its values would write keys a
            # later switch compiles against the host driver and silently
            # drops, so the frozen state would not reproduce.
            detail = []
            if extra := sorted(set(live) - set(desired)):
                detail.append(f"device-only: {', '.join(extra)}")
            if missing := sorted(set(desired) - set(live)):
                detail.append(f"host-only: {', '.join(missing)}")
            report.ok = False
            report.error = (f"device parameter set does not match the "
                            f"compiled {compiled.name} driver "
                            f"({'; '.join(detail)}) — the unit runs a "
                            f"different driver revision; redeploy with "
                            f"`nxs suite switch` before freezing")
            return report
        report.sensor_index = index
        report.config = live
        for name in sorted(desired.keys() & live.keys()):
            if desired[name] != live[name]:
                report.changed = True
                report.actions.append(f"{name}: {desired[name]}→{live[name]}")

        if unit.egress is not None:
            # The unit declares an egress section, so freeze adopts the
            # live factors into it — the same device-wins rule as config.
            if unit.egress.decimation is not None:
                live = transport.read_decimation()
                report.egress_decimation = live
                if live != unit.egress.decimation:
                    report.changed = True
                    report.actions.append(
                        f"egress decimation: {unit.egress.decimation}→{live}")
            for subject, want in sorted(unit.egress.subjects.items()):
                live = transport.read_decimation(subject=subject)
                report.egress_subjects[subject] = live
                if live != want:
                    report.changed = True
                    report.actions.append(f"egress[{subject}]: {want}→{live}")

        if pin_firmware:
            version = transport.read_fw_version()
            proven = parse_device_version(version) if version is not None else None
            if version is None:
                report.actions.append("firmware not readable on this "
                                      "transport; pin unchanged")
            elif proven is None:
                # A bare SHA (untagged build) proves no release version —
                # nothing the manifest's numeric pin can hold.
                report.actions.append(f"firmware identity {version!r} proves "
                                      "no version; pin unchanged")
            else:
                # Pin exactly what the wire proves: the full triple for a
                # build identity, the pair for legacy firmware — and treat
                # an equivalent existing pin as unchanged so a freeze
                # never rewrites "1.0" into "1.0.0".
                try:
                    want = (parse_version(unit.firmware)
                            if unit.firmware is not None else None)
                except ValueError:
                    want = None
                if device_proves_patch(version):
                    pin = ".".join(str(n) for n in proven)
                    same = want is not None and want == proven
                else:
                    pin = f"{proven[0]}.{proven[1]}"
                    same = want is not None and want[:2] == proven[:2]
                if not same:
                    report.firmware = pin
                    report.changed = True
                    report.actions.append(f"firmware: {unit.firmware}→{pin}")

        # Adopt a hand-set mounting orientation into the manifest — declared
        # intent, like the firmware pin. The solved affines stay on-device.
        from nxs.client import SupportsCalibration, rotation_name
        if isinstance(transport, SupportsCalibration):
            try:
                code = transport.read_calibration().orientation
            except Exception:
                # The class implements the surface; the connected firmware
                # may predate it. An optional adoption must not fail the
                # freeze of everything else.
                code = None
                report.actions.append("calibration surface unavailable; "
                                      "orientation not adopted")
            live = rotation_name(code) if code is not None else None
            if live is not None and live.isdigit():
                # rotation_name preserves an unknown code as its number.
                # The manifest vocabulary cannot hold it, and writing it
                # would corrupt the file for every later command.
                report.ok = False
                report.error = (f"device reports orientation code {live}, "
                                "which this tool's vocabulary does not name "
                                "— update the nxs tool, then freeze again")
                return report
            if live is not None and unit.orientation != live \
                    and (unit.orientation or live != "NONE"):
                report.orientation = live
                report.changed = True
                report.actions.append(f"orientation: "
                                      f"{unit.orientation or 'NONE'}→{live}")

        if not report.changed:
            report.actions.append("no tuning to adopt")
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


def _round_trip_yaml():
    """A ruamel instance matching the documented manifest style, so a
    one-value freeze doesn't reformat the whole file."""
    from ruamel.yaml import YAML

    yaml_rt = YAML()
    yaml_rt.preserve_quotes = True
    yaml_rt.indent(mapping=2, sequence=4, offset=2)
    return yaml_rt


def _reject_mangled_serials(doc):
    """Refuse a round-trip write when a serial loaded as a number: a
    bare `<digits>e<digits>` UID resolves as a YAML float, the digits
    are already gone at load time, and dumping would write the
    float64-rounded corpse back over the operator's pin."""
    for entry in doc.get("units", []) or []:
        serial = entry.get("serial")
        if serial is not None and not isinstance(serial, str):
            raise ManifestError(
                f"unit {entry.get('name', '?')!r}: serial is unquoted and "
                f"parsed as a number — quote it in the manifest "
                f"(serial: \"<24 hex digits>\") before freezing")


def _apply_frozen(entry, report: FreezeReport):
    """Write a report's frozen values into a unit's manifest entry,
    updating the `config` mapping *in place* so each surviving key keeps
    its per-key comment and position (replacing the whole map would drop
    them); new keys append, dropped keys are removed."""
    if report.sensor_index >= 0:
        sensor = entry["sensors"][report.sensor_index]
        existing = sensor.get("config")
        if not isinstance(existing, dict):
            sensor["config"] = dict(report.config)
        else:
            for key, value in report.config.items():
                existing[key] = value
            for key in [k for k in existing if k not in report.config]:
                del existing[key]
    if report.egress_decimation is not None or report.egress_subjects:
        egress = entry.setdefault("egress", {})
        if report.egress_decimation is not None:
            egress["decimation"] = report.egress_decimation
        if report.egress_subjects:
            subjects = egress.setdefault("subjects", {})
            for subject, live in report.egress_subjects.items():
                subjects[subject] = live
    if report.firmware is not None:
        entry["firmware"] = report.firmware
    if report.orientation is not None:
        entry["orientation"] = report.orientation


def _write_manifest(config_path: str, reports: List[FreezeReport]):
    """Round-trip edit: replace only the frozen values, keep the
    operator's comments and layout, land atomically."""
    yaml_rt = _round_trip_yaml()
    with open(config_path, encoding="utf-8") as f:
        doc = yaml_rt.load(f)
    _reject_mangled_serials(doc)

    by_name = {r.name: r for r in reports}
    for entry in doc.get("units", []):
        report = by_name.get(entry.get("name"))
        if report is not None:
            _apply_frozen(entry, report)

    tmp = config_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        yaml_rt.dump(doc, f)
    os.replace(tmp, config_path)


def render_frozen_block(config_path: str, report: FreezeReport) -> str:
    """The unit's would-be manifest block, for --dry-run review."""
    import io

    yaml_rt = _round_trip_yaml()
    with open(config_path, encoding="utf-8") as f:
        doc = yaml_rt.load(f)
    _reject_mangled_serials(doc)
    entry = next(u for u in doc.get("units", [])
                 if u.get("name") == report.name)
    _apply_frozen(entry, report)
    out = io.StringIO()
    yaml_rt.dump([entry], out)
    return out.getvalue()
