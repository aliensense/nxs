"""`nxs tune --freeze`: adopt a unit's live tuning into the manifest, the
inverse of `switch`. It captures the active personality's full live parameter set
for units and personalities the manifest already declares; the write keeps comments
and layout."""
import dataclasses
import io
import os

import yaml
from ruamel.yaml.scalarint import HexInt
import sys
from dataclasses import dataclass, field
from typing import List, Optional

from nxs.client import exc_detail
from nxs.suite.drift import declared_values, live_values, rows_drift
from nxs.suite.reconcile import load_click_personality
from nxs.suite.schema import (ManifestError, SuiteConfig, UnitSpec,
                              device_proves_patch, parse_device_version,
                              parse_version)
from nxs import transports


@dataclass
class FreezeReport:
    name: str
    ok: bool = True
    changed: bool = False
    sensor_index: int = -1
    #: The running click's rows, each in its display form, by row name.
    config: dict = field(default_factory=dict)
    #: The unit's own declared rows at the values it serves, by row name.
    rows: dict = field(default_factory=dict)
    firmware: Optional[str] = None
    actions: List[str] = field(default_factory=list)
    error: str = ""


def freeze_suite(cfg: SuiteConfig, config_path: str, *,
                 only_unit: Optional[str] = None, dry_run: bool = False,
                 pin_firmware: bool = False, opener=None,
                 personalities_dir: "str | None" = None) -> List[FreezeReport]:
    """Freeze the selected unit(s); write the manifest once at the end. The
    opener defaults to the one `nxs.transports` holds at the call."""
    opener = opener or transports.open_client
    reports = []
    for unit in cfg.units:
        if only_unit is not None and unit.name != only_unit:
            continue
        reports.append(_freeze_unit(unit, pin_firmware=pin_firmware,
                                    opener=opener, personalities_dir=personalities_dir))
    changed = [r for r in reports if r.ok and r.changed]
    if changed and not dry_run:
        _write_manifest(config_path, changed)
    return reports


def _freeze_unit(unit: UnitSpec, *, pin_firmware: bool, opener,
                 personalities_dir) -> FreezeReport:
    report = FreezeReport(name=unit.name)
    try:
        panel = [(spec, load_click_personality(spec.personality, personalities_dir)().compile(spec.config))
                 for spec in unit.sensors]
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

        active = transport.read_personality_name()
        if not active:
            report.ok = False
            report.error = "no active personality to freeze"
            return report
        index = next((i for i, (_, compiled) in enumerate(panel)
                      if compiled.name == active), None)
        if index is None:
            report.ok = False
            report.error = (f"active personality {active} is not declared for "
                            f"this unit — declare it in the manifest (or "
                            f"redeploy with `nxs switch`) before "
                            f"freezing")
            return report

        _, compiled = panel[index]
        live = live_values(transport.read_capabilities(), compiled)
        desired = declared_values(compiled)
        if set(live) != set(desired):
            # A same-named personality serving a different parameter set is a different
            # revision; adopting its values would not reproduce.
            detail = []
            if extra := sorted(set(live) - set(desired)):
                detail.append(f"device-only: {', '.join(extra)}")
            if missing := sorted(set(desired) - set(live)):
                detail.append(f"host-only: {', '.join(missing)}")
            report.ok = False
            report.error = (f"device parameter set does not match the "
                            f"compiled {compiled.name} personality "
                            f"({'; '.join(detail)}) — the unit runs a "
                            f"different personality revision; redeploy with "
                            f"`nxs switch` before freezing")
            return report
        report.sensor_index = index
        shown = _shown(unit.sensors[index].personality, live)
        report.config = shown
        for name in sorted(desired.keys() & live.keys()):
            if desired[name] != live[name]:
                report.changed = True
                report.actions.append(f"{name}: {_shown(unit.sensors[index].personality, desired)[name]}"
                                      f"→{shown[name]}")

        # The unit's own declared rows: the live values, the same
        # device-wins rule as the click's.
        drift = rows_drift(unit, transport)
        if drift:
            for name, (want, live) in sorted(drift.items()):
                if live is None:
                    continue
                report.rows[name] = live
                report.changed = True
                report.actions.append(f"{name}: {want}→{live}")

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


def _shown(personality: str, values: dict) -> dict:
    """A click's served values as its rows show them, by row name: a name
    where the row names its values, else the number in the row's unit."""
    from nxs import description

    try:
        block = description.of_type(personality)
    except (description.UnknownType, description.DescriptionError):
        return dict(values)
    out = {}
    for name, raw in values.items():
        try:
            out[name] = _plain_value(description.display(block.row(name), int(raw)))
        except (KeyError, TypeError, ValueError):
            out[name] = raw
    return out


def _plain_value(text: str):
    """A display text as YAML spells it: a number as a number, a name as text."""
    try:
        number = float(text)
    except ValueError:
        return text
    return int(number) if number.is_integer() and "." not in text else number


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
    for name, node in (doc.get("nodes") or {}).items():
        params = node.get("parameters") if isinstance(node, dict) else None
        serial = params.get("serial") if isinstance(params, dict) else None
        if serial is not None and not isinstance(serial, str):
            raise ManifestError(
                f"nodes.{name}.parameters.serial: unquoted and parsed as a number; quote it "
                f"(serial: \"<24 hex digits>\") before freezing")


def _clicks_of(doc, unit: str) -> list:
    """The click nodes wired into a unit's socket in a declaration's
    document, as (name, node), in the file's order: the order the loader
    derives the unit's panel in."""
    clicks = []
    for source, targets in (doc.get("connections") or {}).items():
        if f"{unit}.click" in (targets or []):
            name = str(source).split(".", 1)[0]
            clicks.append((name, (doc.get("nodes") or {}).get(name)))
    return clicks


def _apply_frozen(doc, report: FreezeReport):
    """Write a report's frozen values into the declaration: the click's rows
    into the click node on the unit's socket, the unit's own rows and the
    firmware pin into the unit node, each mapping updated in place so
    surviving keys keep their comments and position."""
    unit = doc["nodes"][report.name]
    if report.sensor_index >= 0:
        _name, click = _clicks_of(doc, report.name)[report.sensor_index]
        existing = click.get("parameters")
        if not isinstance(existing, dict):
            click["parameters"] = dict(report.config)
        else:
            for key, value in report.config.items():
                existing[key] = value
            for key in [k for k in existing if k not in report.config]:
                del existing[key]
    params = unit.get("parameters")
    if not isinstance(params, dict):
        params = unit["parameters"] = {}
    for name, live in report.rows.items():
        params[name] = live
    if report.firmware is not None:
        params["firmware"] = report.firmware
    if not params:
        del unit["parameters"]


def _write_manifest(config_path: str, reports: List[FreezeReport]):
    """Round-trip edit: replace only the frozen values, keep the
    operator's comments and layout, land atomically."""
    yaml_rt = _round_trip_yaml()
    with open(config_path, encoding="utf-8") as f:
        doc = yaml_rt.load(f)
    _reject_mangled_serials(doc)
    for report in reports:
        if report.name in (doc.get("nodes") or {}):
            _apply_frozen(doc, report)

    tmp = config_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        yaml_rt.dump(doc, f)
    os.replace(tmp, config_path)


def render_frozen_block(config_path: str, report: FreezeReport) -> str:
    """The unit's would-be nodes, the unit and the clicks on its socket, for
    --dry-run review."""
    import io

    yaml_rt = _round_trip_yaml()
    with open(config_path, encoding="utf-8") as f:
        doc = yaml_rt.load(f)
    _reject_mangled_serials(doc)
    _apply_frozen(doc, report)
    block = {report.name: doc["nodes"][report.name]}
    block.update(_clicks_of(doc, report.name))
    out = io.StringIO()
    yaml_rt.dump(block, out)
    return out.getvalue()


# --- ports: the camera side of the manifest, from the booted tree and the live port

def _rate_value(fps: float):
    """A rate as the manifest spells it: whole when it is one."""
    return int(fps) if float(fps).is_integer() else round(float(fps), 3)


def _geometry_token(hints) -> str:
    if hints and hints.get("width") and hints.get("height"):
        return f"{int(hints['width'])}x{int(hints['height'])}"
    return ""


def _mode_name(hub, sensor: str, geometry: str, mode: Optional[str]) -> Optional[str]:
    """The mode a link runs, by name: the record's own token or the viewer's
    geometry resolved against the sensor's modes; the token itself when it
    names several modes or a sensor nothing describes."""
    token = str(mode) if mode else geometry
    if not token:
        return None
    if not sensor or hub is None:
        return token
    from nxs.cam.descriptors import resolve_mode
    try:
        return resolve_mode(hub.descriptor(sensor), token)
    except Exception:        # noqa: BLE001 (several modes, or a sensor nothing describes)
        return token


def _link_sensor(topology, name: str) -> str:
    for link in topology.links:
        if link.name == name:
            return link.sensor_compatible
    return ""


#: What the declaration carries from the file it rewrites: the bus and the
#: lane count an operator pinned, a link's clock, and the port's exposure
#: and gain. The booted tree's other facts (the capture ids, the addresses
#: the wiring translates) are the report's alone.
_CARRIED_PORT_KEYS = ("bus", "csi_lanes")
_CARRIED_CAMERA_KEYS = ("exposure_us", "gain_db")
HUB_ADDR_DEFAULT = 0x6A
POD_TARGET_DEFAULT = 0x30


def port_block(topology, sync=None, modes=None, rates=None, gain_db=None, units=None,
               sensors=None, viewers=None, declared=None, viewer=None) -> dict:
    """One `ports:` entry of the port view (`port_view`), the shape the
    converter writes into api 3: per port `hub`,
    `sync: {source, fps}` and `camera: {gain_db}`; per link `camera:
    {sensor, mode, fps}` and `unit: {name, alias}`. `modes` names the mode
    each link runs and `rates` its free-run rate, written where the
    camera's block has an `fps` row; `units` the pod on each link
    ({name, alias}); `viewers` (a geometry per link, `viewer` the port's)
    stands in for a mode no record names. `declared` is the entry the file
    holds: its host facts and its pods where `units` is silent are carried,
    and nothing else of it."""
    from nxs.cam import hubs

    declared = declared if isinstance(declared, dict) else {}
    declared_links = declared.get("links") if isinstance(declared.get("links"), dict) else {}
    sensors = sensors or {}
    modes = {str(k): v for k, v in (modes or {}).items() if v}
    rates = {str(k): float(v) for k, v in (rates or {}).items() if v}
    viewers = dict(viewers or {})
    if viewer and not viewers:
        viewers = {link.name: viewer for link in topology.links}
    try:
        hub = hubs.for_topology(topology)
    except Exception:        # noqa: BLE001 (a hub nothing serves names its mode by token)
        hub = None
    # A port's record outlives its cameras: the sync and the gain it holds
    # count for the links that carry a camera today.
    cameras = [l for l in topology.links if sensors.get(l.name) or l.sensor_compatible]
    synced = bool(cameras and sync and sync.get("source") == "fsync")
    if not cameras:
        gain_db = None
    block: dict = {}
    for key in _CARRIED_PORT_KEYS:
        if key in declared:
            block[key] = declared[key]
    if not topology.is_direct:
        extras = {}
        if str(getattr(topology, "hub_driver", "nxs")) != "nxs":
            extras["driver"] = str(topology.hub_driver)
        if int(getattr(topology, "des_addr", HUB_ADDR_DEFAULT)) != HUB_ADDR_DEFAULT:
            extras["addr"] = hex_address(topology.des_addr)
        block["hub"] = ({"compatible": topology.des_compatible, **extras} if extras
                        else topology.des_compatible)
    if synced:
        block["sync"] = {"source": "fsync"}
        if sync.get("fps"):
            block["sync"]["fps"] = _rate_value(float(sync["fps"]))
    camera = {}
    old_camera = declared.get("camera") if isinstance(declared.get("camera"), dict) else {}
    for key in _CARRIED_CAMERA_KEYS:
        if key in old_camera:
            camera[key] = old_camera[key]
    if gain_db is not None:
        camera["gain_db"] = _rate_value(float(gain_db))
    if camera:
        block["camera"] = camera
    links = {}
    for link in topology.links:
        old = declared_links.get(link.name) if isinstance(declared_links.get(link.name), dict) else {}
        entry: dict = {}
        sensor = sensors.get(link.name) or link.sensor_compatible
        if sensor:
            cam = {"sensor": sensor}
            mode = _mode_name(hub, sensor, _geometry_token(viewers.get(link.name)), modes.get(link.name))
            if mode:
                cam["mode"] = mode
            if not synced and link.name in rates and _has_rate_row(sensor):
                cam["fps"] = _rate_value(rates[link.name])
            entry["camera"] = cam
        unit = (units or {}).get(link.name)
        if unit is None and isinstance(old.get("unit"), dict):
            unit = old["unit"]
        if unit:
            ref = {"name": str(unit["name"])}
            if unit.get("alias") is not None:
                ref["alias"] = hex_address(int(unit["alias"]))
            if unit.get("target") is not None and int(unit["target"]) != POD_TARGET_DEFAULT:
                ref["target"] = hex_address(int(unit["target"]))
            entry["unit"] = ref
        if entry:
            links[link.name] = entry
    if links:
        block["links"] = links
    return block


def _has_rate_row(sensor: str) -> bool:
    """Whether a camera's block declares its rate, an `fps` row; a head
    whose rate is its mode's has none."""
    from nxs import description

    try:
        description.of_type(sensor).row("fps")
    except (KeyError, description.UnknownType, description.DescriptionError):
        return False
    return True


def port_intent(port) -> dict:
    """What a parsed port declares, as `port_block` takes it: its sync, the
    mode and free-run rate of each link, the pod on each link and the
    port's gain."""
    synced = port.sync_source == "fsync"
    return {
        "sync": {"source": port.sync_source, "fps": port.sync_fps},
        "modes": {l.name: l.camera_mode for l in port.links},
        "rates": {l.name: None if synced else l.camera_fps for l in port.links},
        "units": {l.name: {"name": l.unit.name, "alias": l.unit.alias, "target": l.unit.target}
                  for l in port.links if l.unit is not None},
        "gain_db": port.camera_gain_db,
    }


def _declared_owner(topology, declared: dict):
    """`topology` with the hub's owner as the declaration names it: the booted
    tree does not say whether a kernel driver, the tool or a module runs the
    hub."""
    hub = declared.get("hub")
    driver = hub.get("driver") if isinstance(hub, dict) else None
    if not driver or topology.is_direct:
        return topology
    return dataclasses.replace(topology, hub_driver=str(driver))


def port_wiring(topology, sensors=None) -> dict:
    """What answered on a port, for `hardware.yaml`: the bus, the lane
    count, the hub, and every link's wiring (serializer, window, channel,
    capture id, addresses) with the sensor it carries."""
    sensors = sensors or {}
    links = {}
    for link in topology.links:
        entry = ({} if topology.is_direct else
                 {"ser": link.ser_compatible, "des_window": hex_address(link.des_window),
                  "csi_vc": int(link.csi_vc)})
        sensor = sensors.get(link.name) or link.sensor_compatible
        if sensor:
            entry = {"camera": sensor, **entry}
        if link.capture_id is not None:
            entry["capture_id"] = int(link.capture_id)
        if link.sensor_addr is not None:
            entry["sensor_addr"] = hex_address(link.sensor_addr)
        links[link.name] = entry
    block = {"bus": topology.i2c_bus, "csi_lanes": int(topology.csi_lanes)}
    if not topology.is_direct:
        block["hub"] = {"compatible": topology.des_compatible, "driver": topology.hub_driver}
    if links:
        block["links"] = links
    return block


def freeze_ports(config_path: str, dry_run: bool = False,
                 only_port: Optional[str] = None) -> str:
    """Write every camera port (or `only_port` alone) into the manifest as the
    booted tree and the live port have it; create the manifest when there is
    none. Returns the rendered camera chains: the host, the nodes on a chain
    into one of its connectors and the connections among them. A scoped
    freeze leaves the wiring report to `generate`."""
    import io

    from nxs.cam.hubs import HubError
    from nxs.cam.topology import TopologyError, _ports_from_suite, discover_ports
    from nxs.suite.schema import hardware_path
    from nxs.cam import port_state

    # What discovery found, never the manifest read back: the platform's
    # buses, the booted tree's lanes and ids, the hub's link shape. A port
    # that serves the sensor on its own bus is no hub's shape: the walk
    # wrote it down, and it is frozen as it stands, with or without a hub
    # that shapes the other ports.
    try:
        standing = {t.carrier.rsplit("/", 1)[-1]: t
                    for t in (_ports_from_suite() or ({}, 0))[0].values() if t.is_direct}
    except TopologyError:
        standing = {}
    try:
        discovered, _ = discover_ports()
    except HubError:
        if not standing:
            raise
        discovered = {}
    candidates = {t.carrier.rsplit("/", 1)[-1]: t for t in discovered.values()}
    candidates.update(standing)
    if only_port is not None:
        if only_port not in candidates:
            raise ValueError(f"no camera port named {only_port} answers on this host")
        candidates = {only_port: candidates[only_port]}
    from nxs.suite.nodes import parse_graph
    from nxs.suite.port_view import chain_scope, merge, nodes_from_view
    from nxs.tune_write import _camera_buses, _plain, declared_view

    yaml_rt = _round_trip_yaml()
    if os.path.exists(config_path) and os.path.getsize(config_path) > 0:
        with open(config_path, encoding="utf-8") as f:
            doc = yaml_rt.load(f) or {}
        _reject_mangled_serials(doc)
    else:
        doc = {}
    previous, view = declared_view(doc)
    existing = view.setdefault("ports", {})
    wiring = {}
    for name, topology in sorted(candidates.items()):
        own = port_state.port_record(topology)      # this port only
        # The block is the whole declaration of the port: what the operator
        # pinned (the host facts, the pods, a link's addresses) is carried
        # from the entry, the rest is the live port's.
        declared = existing.get(name)
        if not isinstance(declared, dict):
            declared = {}
        topology = _declared_owner(topology, declared)
        existing[name] = port_block(topology, sync=own.get("sync"), viewer=own.get("viewer"),
                                    viewers=own.get("viewers"), sensors=own.get("sensors"),
                                    modes=own.get("modes"), rates=own.get("rates"),
                                    declared=declared)
        wiring[name] = port_wiring(topology, sensors=own.get("sensors"))
    view["ports"] = {n: existing[n] for n in existing if existing[n]}
    emitted = nodes_from_view(view, previous=previous, buses=_camera_buses())
    _adopt(emitted, previous)
    merge(doc, emitted, previous)
    out = io.StringIO()
    # What is rendered is what is (or would be) written: the camera chains.
    chains = chain_scope(parse_graph(_plain(doc), config_path), set(candidates))
    yaml_rt.dump({"nodes": {n: e for n, e in doc["nodes"].items() if n in chains},
                  "connections": {s: [t for t in ts if str(t).split(".", 1)[0] in chains]
                                  for s, ts in doc["connections"].items()
                                  if str(s).split(".", 1)[0] in chains}}, out)
    if not dry_run:
        os.makedirs(os.path.dirname(config_path) or ".", exist_ok=True)
        _write_atomic(config_path, lambda f: yaml_rt.dump(doc, f))
        if only_port is None:
            hw = hardware_path(config_path)
            _write_atomic(hw, lambda f: f.write(_render_hardware(wiring)))
    return out.getvalue()


#: The rows a port freeze adopts from the live port: a hub's frame sync and
#: each camera's mode and rate. A node's other rows stay as the file has them.
_ADOPTED = ("sync_source", "sync_fps", "mode", "fps")


def _adopt(emitted: dict, previous) -> None:
    """Each emitted node the file already names keeps the parameters the
    file gives it, the adopted rows taken from the live port."""
    if previous is None:
        return
    for name, entry in emitted["nodes"].items():
        node = previous.nodes.get(name)
        if node is None or node.type != entry.get("type"):
            continue
        params = {k: v for k, v in node.parameters.items() if k not in _ADOPTED}
        params.update({k: v for k, v in (entry.get("parameters") or {}).items() if k in _ADOPTED})
        if params:
            entry["parameters"] = params
        else:
            entry.pop("parameters", None)


def _render_hardware(ports: dict) -> str:
    """The generated wiring file.

    Written plainly and in a fixed order, with no timestamp: re-probing
    an unchanged rig must produce an identical file, or committing the
    pair beside the manifest would bury every real change in noise.
    """
    header = ("# Written by `nxs generate`. What is wired:\n"
              "# regenerate it after a re-cable, and do not hand-edit it.\n"
              "# What you want of the rig lives in suite.yaml beside it.\n")
    return header + yaml.dump({"ports": ports}, Dumper=_SeedDumper, sort_keys=True,
                              default_flow_style=False, width=100)


def hex_address(value: int) -> HexInt:
    """An I²C address as the declaration files write it, in hex as every
    line the tool prints names it (`alias: 0x32`); it reads back as the int.
    The round-trip writer `nxs tune` saves through prints it as it is."""
    return HexInt(int(value), width=2)


class _SeedDumper(yaml.SafeDumper):
    """The dumper of the seed and the wiring file: block style, I²C
    addresses in hex."""


_SeedDumper.add_representer(HexInt, lambda dumper, value: dumper.represent_scalar(
    "tag:yaml.org,2002:int", f"{int(value):#04x}"))


def render_seed(doc: dict) -> str:
    """The seeded declaration's body, in the order it was built."""
    return yaml.dump(doc, Dumper=_SeedDumper, sort_keys=False)


def _write_atomic(path: str, render) -> None:
    """Land a rendered file: a temporary beside it and an atomic replace
    where this user may write, else through the root helper (the
    declaration and its report live under /etc on a camera host). A
    directory this user cannot create is made under root for the I²C
    group, as `nxs switch` makes it."""
    directory = os.path.dirname(path) or "."
    if not os.path.isdir(directory):
        try:
            os.makedirs(directory, exist_ok=True)
        except PermissionError:
            from nxs.suite.switch_host import make_group_dir
            make_group_dir(directory)
    if os.access(directory, os.W_OK):
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
        except Exception as e:      # a silent tree, no hub, an unwritable path
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
