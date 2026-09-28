"""`nxs tune --freeze`: adopt a unit's live tuning into the manifest, the
inverse of `switch`. It captures the active driver's full live parameter set
for units and drivers the manifest already declares; the write keeps comments
and layout."""
import io
import os

import yaml
import sys
from dataclasses import dataclass, field
from typing import List, Optional

from nxs.client import exc_detail
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
                 drivers_dir: "str | None" = None) -> List[FreezeReport]:
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
                 drivers_dir) -> FreezeReport:
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
        # First link that answers, in declared order; the running tuning is
        # the same over any route to the board.
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
                            f"redeploy with `nxs switch`) before "
                            f"freezing")
            return report

        live = {p["name"]: p["current"] for p in transport.read_capabilities()}
        _, compiled = panel[index]
        desired = {p.name: p.current for p in compiled.params}
        if set(live) != set(desired):
            # A same-named driver serving a different parameter set is a different
            # revision; adopting its values would not reproduce.
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
                            f"`nxs switch` before freezing")
            return report
        report.sensor_index = index
        report.config = live
        for name in sorted(desired.keys() & live.keys()):
            if desired[name] != live[name]:
                report.changed = True
                report.actions.append(f"{name}: {desired[name]}→{live[name]}")

        if unit.egress is not None:
            # The unit declares an egress section, so freeze adopts the live
            # factors into it: the same device-wins rule as config.
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
                # A bare SHA (untagged build) proves no release version; nothing the
                # manifest's numeric pin can hold.
                report.actions.append(f"firmware identity {version!r} proves "
                                      "no version; pin unchanged")
            else:
                # Pin exactly what the wire proves (the full triple, or the pair), and
                # treat an equivalent existing pin as unchanged.
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

        # Adopt a hand-set mounting orientation into the manifest, declared
        # intent like the firmware pin. The solved affines stay on-device.
        from nxs.client import SupportsCalibration, rotation_name
        if isinstance(transport, SupportsCalibration):
            try:
                code = transport.read_calibration().orientation
            except Exception:
                # The connected firmware may predate the surface; an optional adoption
                # must not fail the freeze of everything else.
                code = None
                report.actions.append("calibration surface unavailable; "
                                      "orientation not adopted")
            live = rotation_name(code) if code is not None else None
            if live is not None and live.isdigit():
                # rotation_name preserves an unknown code as its number; the manifest
                # vocabulary cannot hold it, and writing it would corrupt the file.
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
    """Refuse a round-trip write when a serial loaded as a number: a bare
    `<digits>e<digits>` UID resolves as a YAML float and its digits are gone."""
    for entry in doc.get("units", []) or []:
        serial = entry.get("serial")
        if serial is not None and not isinstance(serial, str):
            raise ManifestError(
                f"unit {entry.get('name', '?')!r}: serial is unquoted and "
                f"parsed as a number — quote it in the manifest "
                f"(serial: \"<24 hex digits>\") before freezing")


def _apply_frozen(entry, report: FreezeReport):
    """Write a report's frozen values into a unit's manifest entry, updating the
    `config` mapping in place so surviving keys keep their comments and position."""
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


# --- ports: the camera side of the manifest, from the booted tree and the live port

def _rate_value(fps: float):
    """A rate as the manifest spells it: whole when it is one."""
    return int(fps) if float(fps).is_integer() else round(float(fps), 3)


def _geometry_token(hints) -> str:
    if hints and hints.get("width") and hints.get("height"):
        return f"{int(hints['width'])}x{int(hints['height'])}"
    return ""


def _mode_token(pack, sensor: str, geometry: str, mode: Optional[str]) -> str:
    """The token the manifest resolves back to the running mode: the
    geometry when it names one mode of the sensor, the mode's own name
    when several modes share it."""
    if not geometry or not mode or pack is None:
        return geometry
    from nxs.cam.contracts import InfeasibleConfig
    from nxs.cam.descriptors import resolve_mode
    try:
        resolve_mode(pack.descriptor(sensor), geometry)
    except InfeasibleConfig as exc:
        return mode if getattr(exc, "alternatives", None) else geometry
    except Exception:
        return geometry
    return geometry


def _link_sensor(topology, name: str) -> str:
    for link in topology.links:
        if link.name == name:
            return link.sensor_compatible
    return ""


def port_block(topology, sync=None, viewer=None, viewers=None,
               sensors=None, modes=None, rates=None) -> dict:
    """One `ports:` entry from a port: bus, lanes, hub, links with their sensors
    and capture ids, and the live port's camera mode and sync. A mixed hub
    declares the mode per link; ``modes`` names the mode each link runs,
    ``rates`` the free-run rate each link was programmed for (written per
    link when they differ, at the port when they agree)."""
    from nxs.cam import packs

    sensors = sensors or {}
    viewers = viewers or {}
    modes = modes or {}
    rates = {str(k): float(v) for k, v in (rates or {}).items()}
    try:
        pack = packs.pack_for(topology)
    except Exception:
        pack = None
    tokens = {name: _mode_token(pack, sensors.get(name) or _link_sensor(topology, name),
                                _geometry_token(h), modes.get(name))
              for name, h in viewers.items()}
    present = {name: t for name, t in tokens.items() if t}
    kinds = {sensors.get(l.name) or l.sensor_compatible for l in topology.links}
    # Per-link modes when the links run different geometries, or when a
    # mixed hub has only some links up.
    per_link = (len(set(present.values())) > 1
                or (len(kinds) > 1 and bool(present)
                    and len(present) < len(topology.links)))
    synced = bool(sync and sync.get("source") == "fsync" and sync.get("fps"))
    # Free-running links may run different rates; one rate is the port's.
    per_link_rates = (not synced and len({round(r, 6) for r in rates.values()}) > 1)
    links = {}
    for link in topology.links:
        sensor = sensors.get(link.name) or link.sensor_compatible
        # A direct port's link has no SerDes wiring to write down; a head
        # no one names has no camera row.
        entry = ({} if topology.is_direct else
                 {"ser": link.ser_compatible, "des_window": int(link.des_window),
                  "csi_vc": int(link.csi_vc)})
        if sensor:
            entry = {"camera": sensor, **entry}
        if per_link and tokens.get(link.name):
            entry["camera"] = {"sensor": sensor, "mode": tokens[link.name]}
        if per_link_rates and link.name in rates:
            if not isinstance(entry["camera"], dict):
                entry["camera"] = {"sensor": sensor}
            entry["camera"]["fps"] = _rate_value(rates[link.name])
        if link.capture_id is not None:
            entry["capture_id"] = int(link.capture_id)
        # Where the wiring says the sensor answers, when it is not the
        # descriptor's own address: the node and the tool address it there.
        if link.sensor_addr is not None:
            entry["sensor_addr"] = int(link.sensor_addr)
        links[link.name] = entry
    block = {
        "bus": topology.i2c_bus,
        "csi_lanes": int(topology.csi_lanes),
        "hub": {"compatible": topology.des_compatible,
                "driver": topology.hub_driver},
        "links": links,
    }
    if topology.is_direct:
        del block["hub"]            # naming no hub is what makes the port direct
    token = ""
    if not per_link:
        any_link = next(iter(topology.links), None)
        one_mode = next(iter(modes.values()), None) if len(set(modes.values())) == 1 else None
        token = _mode_token(pack, sensors.get(any_link.name) if any_link else "",
                            _geometry_token(viewer), one_mode) if any_link else _geometry_token(viewer)
    if token:
        if synced:
            block["camera"] = {"mode": f"{token}@{float(sync['fps']):g}",
                               "sync": "fsync"}
        elif rates and not per_link_rates:
            block["camera"] = {"mode": f"{token}@{_rate_value(next(iter(rates.values()))):g}"}
        else:
            block["camera"] = {"mode": token}
    elif synced:
        block["camera"] = {"sync": "fsync"}
        block["sync"] = {"source": "fsync", "fps": float(sync["fps"])}
    return block


#: Port keys a freeze owns entirely: present when the live port carries
#: the behavior, removed when it does not.
_FROZEN_BEHAVIOR_KEYS = ("sync", "camera")


def freeze_ports(config_path: str, dry_run: bool = False,
                 only_port: Optional[str] = None) -> str:
    """Write every camera port (or `only_port` alone) into the manifest as the
    booted tree and the live port have it; create the manifest when there is
    none. Returns the rendered `ports:` block. A scoped freeze leaves the
    wiring report to `generate`."""
    import io

    from nxs.cam.packs import PackError
    from nxs.cam.topology import TopologyError, _ports_from_suite, discover_ports
    from nxs.suite.schema import hardware_path
    from nxs.cam import port_state

    # What discovery found, never the manifest read back: the platform's
    # buses, the booted tree's lanes and ids, the pack's link shape. A port
    # that serves the sensor on its own bus is no pack's shape: the walk
    # wrote it down, and it is frozen as it stands, with or without a pack
    # that shapes the other ports.
    try:
        standing = {t.carrier.rsplit("/", 1)[-1]: t
                    for t in (_ports_from_suite() or ({}, 0))[0].values() if t.is_direct}
    except TopologyError:
        standing = {}
    try:
        discovered, _ = discover_ports()
    except PackError:
        if not standing:
            raise
        discovered = {}
    candidates = {t.carrier.rsplit("/", 1)[-1]: t for t in discovered.values()}
    candidates.update(standing)
    if only_port is not None:
        if only_port not in candidates:
            raise ValueError(f"no camera port named {only_port} answers on this host")
        candidates = {only_port: candidates[only_port]}
    ports = {}
    for name, topology in sorted(candidates.items()):
        own = port_state.port_record(topology)      # this port only
        ports[name] = port_block(topology, sync=own.get("sync"),
                                 viewer=own.get("viewer"),
                                 viewers=own.get("viewers"),
                                 sensors=own.get("sensors"),
                                 modes=own.get("modes"),
                                 rates=own.get("rates"))
    yaml_rt = _round_trip_yaml()
    if os.path.exists(config_path) and os.path.getsize(config_path) > 0:
        with open(config_path, encoding="utf-8") as f:
            doc = yaml_rt.load(f) or {}
        _reject_mangled_serials(doc)
    else:
        doc = {}
    existing = doc.setdefault("ports", {}) or {}
    for name, block in ports.items():
        entry = existing.setdefault(name, {})
        # The frozen links carry what the tree and the port know; the
        # unit riding a link is the operator's declaration and survives.
        old_links = entry.get("links") if isinstance(entry.get("links"), dict) else {}
        for link_name, link_entry in block["links"].items():
            old = old_links.get(link_name) or {}
            if "unit" in old and "unit" not in link_entry:
                link_entry["unit"] = old["unit"]
        # Who owns the hub is the operator's word too (a kernel-driven hub
        # has no port state to freeze from).
        old_hub = entry.get("hub") if isinstance(entry.get("hub"), dict) else {}
        if "driver" in old_hub and "hub" in block:
            block["hub"]["driver"] = old_hub["driver"]
        for key, value in block.items():
            entry[key] = value
        # The block is the whole frozen declaration: a behavior key an earlier
        # freeze wrote and the live port does not carry must not survive it.
        for key in _FROZEN_BEHAVIOR_KEYS:
            if key not in block:
                entry.pop(key, None)
    doc["ports"] = existing
    # Rendered before the split: the block is the whole declaration, the
    # two halves merged, which is what a reader wants to see.
    merged = {n: dict(existing[n]) for n in sorted(ports)}
    # Two files describe the rig: what is wired, and what you want of it.
    # Split the frozen entries so a re-cable regenerates one and leaves the other.
    wiring, intent = {}, {}
    for name in sorted(ports):
        wired, want = _split_entry(existing[name])
        if wired:
            wiring[name] = wired
        if want:
            intent[name] = want
        existing[name] = want
    doc["ports"] = {n: existing[n] for n in existing if existing[n]}
    if not doc["ports"]:
        doc.pop("ports", None)
    out = io.StringIO()
    # What is rendered is what is (or would be) written: the merged
    # entries, with the operator-owned fields the freeze preserved.
    yaml_rt.dump({"ports": merged}, out)
    if not dry_run:
        os.makedirs(os.path.dirname(config_path) or ".", exist_ok=True)
        _write_atomic(config_path, lambda f: yaml_rt.dump(doc, f))
        if only_port is None:
            hw = hardware_path(config_path)
            _write_atomic(hw, lambda f: f.write(_render_hardware(wiring)))
    return out.getvalue()


#: What only the report carries: the host's and the booted tree's facts.
REPORT_ONLY_PORT_KEYS = {"csi_lanes"}
REPORT_ONLY_LINK_KEYS = {"ser", "des_window", "csi_vc", "ser_addr", "sensor_addr",
                         "tca_addr", "capture_id"}


def _split_entry(entry: dict) -> tuple:
    """Partition one port entry into (the report's wiring, the declaration):
    the declaration keeps the bus, the hub, every link's sensor and unit and
    the wants; the report keeps what answered, wiring included."""
    from nxs.suite.schema import HARDWARE_LINK_KEYS, HARDWARE_PORT_KEYS

    wiring, intent = {}, {}
    for key, value in entry.items():
        if key == "links":
            continue
        if key in HARDWARE_PORT_KEYS:
            wiring[key] = value
        if key not in REPORT_ONLY_PORT_KEYS:
            intent[key] = value
    wired_links, want_links = {}, {}
    for name, link in (entry.get("links") or {}).items():
        w = {k: v for k, v in link.items() if k in HARDWARE_LINK_KEYS}
        i = {k: v for k, v in link.items() if k not in REPORT_ONLY_LINK_KEYS}
        if w:
            wired_links[name] = w
        if i:
            want_links[name] = i
    if wired_links:
        wiring["links"] = wired_links
    if want_links:
        intent["links"] = want_links
    return wiring, intent


def _render_hardware(ports: dict) -> str:
    """The generated wiring file.

    Written plainly and in a fixed order, with no timestamp: re-probing
    an unchanged rig must produce an identical file, or committing the
    pair beside the manifest would bury every real change in noise.
    """
    header = ("# Written by `nxs generate`. What is wired:\n"
              "# regenerate it after a re-cable, and do not hand-edit it.\n"
              "# What you want of the rig lives in suite.yaml beside it.\n")
    return header + yaml.safe_dump({"ports": ports}, sort_keys=True,
                                   default_flow_style=False, width=100)


def _write_atomic(path: str, render) -> None:
    """Land a rendered file: a temporary beside it and an atomic replace
    where this user may write, else through the root helper (the
    declaration and its report live under /etc on a provisioned host)."""
    directory = os.path.dirname(path) or "."
    if not os.path.isdir(directory) or os.access(directory, os.W_OK):
        os.makedirs(directory, exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            render(f)
        os.replace(tmp, path)
        return
    from nxs.host import root
    out = io.StringIO()
    render(out)
    root.write_text(path, out.getvalue())


def run_freeze(config_path: str, only_unit: Optional[str] = None,
               dry_run: bool = False, ports: bool = False,
               only_port: Optional[str] = None) -> int:
    """The verb: every declared unit's live tuning (or one unit's) into the
    manifest, or with `ports` the camera ports (or `only_port` alone) as the
    booted overlay and the live state have them (the manifest is created when
    there is none)."""
    if ports:
        try:
            block = freeze_ports(config_path, dry_run=dry_run, only_port=only_port)
        except Exception as e:      # a silent tree, no pack, an unwritable path
            print(f"nxs tune: cannot freeze the ports: {e}", file=sys.stderr)
            return 1
        print(block, end="")
        if not dry_run:
            print(f"froze {only_port or 'the camera ports'} into {config_path}")
        return 0
    from nxs.suite.switch import load_manifest
    cfg = load_manifest(config_path)
    if cfg is None:
        return 1
    try:
        reports = freeze_suite(cfg, config_path, only_unit=only_unit,
                               dry_run=dry_run, pin_firmware=False)
    except (ManifestError, OSError) as e:
        print(f"nxs tune: cannot rewrite {config_path}: {e}", file=sys.stderr)
        return 1
    if not reports:
        print(f"nxs tune: no unit named {only_unit!r} in the manifest",
              file=sys.stderr)
        return 1
    failed = 0
    for report in reports:
        mark = "✓" if report.ok else "✗"
        if not report.ok:
            verb = "failed"
        elif report.changed:
            verb = "would freeze" if dry_run else "froze"
        else:
            verb = "unchanged"
        print(f"{mark} {report.name}: {verb}")
        for action in report.actions:
            print(f"    {action}")
        if not report.ok:
            print(f"    {report.error}")
            failed += 1
        elif report.changed and dry_run:
            try:
                print(render_frozen_block(config_path, report), end="")
            except ManifestError as e:
                print(f"    {e}", file=sys.stderr)
                failed += 1
    if failed:
        print(f"{failed}/{len(reports)} unit(s) failed", file=sys.stderr)
    return 1 if failed else 0
