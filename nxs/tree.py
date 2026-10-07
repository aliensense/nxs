# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Bare `nxs probe` and `nxs status`: `render_scan` is config-free discovery of
hubs, units, and direct sensors on the leaf I2C buses; `render_tree` is the
declaration against the rig, node by node. Neither writes a register; `--json`
gives the surfaces."""

import errno
import functools
import json
import operator
import os

from nxs import term


# Hub detection sweeps the des control address; the device-id register
# separates a real hub from an address squatter, the hubs name the silicon.
HUB_ADDRESSES = (0x6A,)
SENSOR_ADDRESSES = (0x1A, 0x1D)


def unit_addresses(hub: bool, extra=()) -> list:
    """The unit addresses a sweep of a leaf bus asks, in order: the standard
    ones, every pod address where a hub answers on the bus, and `extra`."""
    from nxs.generate import POD_ADDRESSES
    from nxs.suite.scan import I2C_ADDRESSES

    return sorted(set(I2C_ADDRESSES) | set(extra) | (set(POD_ADDRESSES) if hub else set()))


def _scan_bus(bus: str, extra=()):
    """One leaf bus: hubs, units, and bare sensors found there; `extra` names
    unit addresses to probe beside the standard ones, and `readable` is False
    when the bus could not be opened, with the `errno` of the refusal."""
    from nxs import _libnxs, _libnxs_unit

    found = {"hubs": [], "units": [], "sensors": [], "readable": True}
    path = _libnxs_unit._bus_path(bus)
    for addr in HUB_ADDRESSES:
        try:
            with _libnxs.Bus.open(path) as handle:
                dev_id = handle.read(addr, b"\x00\x0d", 1)[0]
            from nxs.cam.hubs import hub_classes
            kind = hub_classes().get(dev_id)
            found["hubs"].append(
                (addr, f"hub {kind}" if kind else f"id 0x{dev_id:02X}"))
        except Exception:
            continue
    try:
        handle = _libnxs.Bus.open(path)
    except OSError as exc:
        found["readable"], found["errno"] = False, exc.errno or errno.EIO
        return found
    with handle:
        for addr in unit_addresses(bool(found["hubs"]), extra):
            try:
                t = _libnxs_unit.I2cUnit.on_bus(handle, addr)
            except OSError:
                continue
            try:
                if not t.probe():
                    continue
                serial = t.read_serial()
                found["units"].append(
                    (addr, bytes(serial).hex() if serial else "?"))
            except Exception:
                continue
            finally:
                t.close()
        for addr in SENSOR_ADDRESSES:
            try:
                if handle.probe(addr):
                    found["sensors"].append(addr)
            except OSError:
                continue
    if found["hubs"]:
        found["units"] = _each_pod_once(found["units"])
    return found


def _each_pod_once(units):
    """The pods behind a hub, each listed once. A pod answers at its link's
    alias, and where the hub maps the aliases at the address every pod straps
    too: there the pods answer together, and the serial read is the AND of
    theirs. That answer is left out when the aliased pods account for it. It
    stays without a serial when a pod with no alias answers in it, and as it
    reads when it is no merge of the aliased pods: a pod of its own."""
    from nxs._generated_constants import NxsDevices

    strap = int(NxsDevices.RBDevice.NXS)
    aliased = [(addr, serial) for addr, serial in units if addr != strap]
    shared = next((serial for addr, serial in units if addr == strap), None)
    if shared is None or not aliased:
        return units
    unnamed = [(strap, "?")] + aliased
    try:
        together = functools.reduce(operator.and_, (int(serial, 16) for _addr, serial in aliased))
        answer = int(shared, 16)
    except ValueError:
        return unnamed
    if answer == together:
        return aliased
    return unnamed if answer & together == answer else units


def _sweep() -> list:
    """Every leaf bus with what it answered."""
    from nxs.suite.scan import _i2c_buses

    return [(str(bus), _scan_bus(bus)) for bus in _i2c_buses()]


def scan_payload(swept):
    """The `scan` surface: what every leaf bus of a sweep answered, and the
    buses that did not open, which it did not sweep, with the reason."""
    from nxs.schemas import CONTRACT

    buses, not_opened = [], []
    for bus, found in swept:
        if not found.get("readable", True):
            not_opened.append({"bus": bus, "reason": os.strerror(found["errno"])})
            continue
        unit_addrs = {a for a, _ in found["units"]}
        buses.append({
            "bus": bus,
            "hubs": [{"addr": a, "kind": d} for a, d in found["hubs"]],
            "units": [{"addr": a, "serial": s} for a, s in found["units"]],
            "sensors": [a for a in found["sensors"] if a not in unit_addrs],
        })
    return {"contract": CONTRACT, "buses": buses, "not_opened": not_opened}


def render_scan(as_json: bool = False) -> int:
    """Config-free discovery of every leaf bus (bare `nxs probe`)."""
    swept = _sweep()
    payload = scan_payload(swept)
    anything = any(b["hubs"] or b["units"] or b["sensors"]
                   for b in payload["buses"])
    closed = _closed(swept)
    if as_json:
        print(json.dumps(payload, indent=2))
        return 0 if anything else 1
    if not swept:
        print("no I2C buses found")
        return 1
    for entry in payload["buses"]:
        rows = []
        rows += [f"NXS Hub @0x{h['addr']:02X} ({h['kind']})"
                 for h in entry["hubs"]]
        rows += [f"NXS unit @0x{u['addr']:02X} serial {u['serial']}"
                 for u in entry["units"]]
        rows += [f"sensor @0x{a:02X}" for a in entry["sensors"]]
        if rows:
            print(f"{entry['bus']}:")
            for row in rows:
                print(f"  {row}")
    for line in closed:
        print(line)
    if not anything:
        opened = sum(found.get("readable", True) for _bus, found in swept)
        print(f"no hubs, units, or sensors answered "
              f"({opened} bus{'' if opened == 1 else 'es'} swept)")
    print(_next_after_probe(anything))
    return 0 if anything else 1


def _closed(swept) -> list:
    """The lines for the buses of a sweep that did not open, which it did not
    sweep: one per reason, and under a permission refusal the group that owns
    the bus nodes, where the host has it."""
    from nxs.suite.switch_cam import STATE_GROUP, _group_exists

    by_errno: dict = {}
    for bus, found in swept:
        if not found.get("readable", True):
            by_errno.setdefault(found["errno"], []).append(bus)
    lines = []
    for code, buses in by_errno.items():
        lines.append(f"not opened ({os.strerror(code)}): {' '.join(buses)}")
        if code == errno.EACCES and _group_exists(STATE_GROUP):
            lines.append(f'  - sudo usermod -aG {STATE_GROUP} "$USER", then log in again')
    return lines


def _next_after_probe(anything: bool) -> str:
    """The step after discovery: the camera buses when the host boots none
    (the camera connectors answer on them alone), then compare against the
    declaration when there is one, write one from what answered when there
    is none."""
    import os

    from nxs import host as host_layer
    from nxs.suite import default_config_path

    from nxs.suite import stray_declaration

    if host_layer.current().camera_bus_missing():
        return "no camera bus is booted\n  - nxs switch"
    stray = stray_declaration(default_config_path())
    if stray:
        return stray
    if os.path.exists(default_config_path()):
        return "declared-vs-actual: nxs status"
    if not anything:
        return ("no suite.yaml yet, and nothing to declare — connect a unit "
                "and run nxs probe again")
    return "no suite.yaml yet — write down what answered: nxs generate"


def _holds_descriptor(directory: str) -> bool:
    """Whether a store directory is an installed personality: a unit's
    carries its descriptor, as `locate` reads one, and a camera's the chip
    directory beside its hub.yaml (`<name>/<name>.yaml`); Python's
    bytecode cache carries neither."""
    name = os.path.basename(directory)
    if os.path.isfile(os.path.join(directory, name, f"{name}.yaml")):
        return True
    try:
        return any(entry.endswith(".yaml") and entry != "hub.yaml"
                   for entry in os.listdir(directory))
    except OSError:
        return False


def _personality_counts() -> tuple[int, int]:
    """(shipped, installed) in the personality store: the assets' sealed
    personalities (`<name>.nxs`; the hub's and the serializer's images are
    the host's programs, not counted), and the ones installed beside them
    (a directory holding its descriptor, or a `.py`, each)."""
    from nxs.image import HEADER_SIZE, ImageKind, peek_format
    from nxs.suite import PERSONALITY_DIR

    try:
        entries = os.listdir(PERSONALITY_DIR)
    except OSError:
        return 0, 0
    shipped = installed = 0
    for entry in entries:
        path = os.path.join(PERSONALITY_DIR, entry)
        if os.path.isdir(path):
            installed += _holds_descriptor(path)
        elif entry.endswith(".py"):
            installed += 1
        elif entry.endswith(".nxs"):
            try:
                with open(path, "rb") as fh:
                    kind = peek_format(fh.read(HEADER_SIZE))[2]
            except (OSError, ValueError):
                continue
            shipped += kind != ImageKind.HUB
    return shipped, installed


#: The VM verdicts of a unit that does not run its personality.
_VM_NOT_RUNNING = ("idle", "error", "no-probe", "probing")


def _deviations(row) -> dict:
    """What a present unit reports against its declaration: the drift
    kinds, a VM not measuring, a serial mismatch, a stale calibration, a
    silent declared link. An empty dict is a unit as declared."""
    out = {}
    if row.degraded:
        out["link"] = "degraded (a declared link is silent)"
    if row.vm_state and row.vm_state != "running":
        out["vm"] = row.vm_state
    if row.drift and row.drift != "-":
        out["drift"] = row.drift
    if row.serial_ok == "MISMATCH":
        out["serial"] = "MISMATCH"
    if row.cal in ("STALE", "unguarded"):
        out["cal"] = row.cal
    return out


def _unit_rows(cfg, held_buses, state):
    """One `collect_status` row per declared unit, None for one not read.
    A unit on a port's bus (it rides a link, or names the bus) is read
    under the bus lock, waited for as the port's walk waits, and not at
    all when the walk left its bus held or the lock outlasts the wait; a
    unit on a bus of its own is read as it is."""
    from nxs.cam import port_state
    from nxs.suite.status import collect_status

    port_buses = {port.bus for port in cfg.ports.values() if port.bus}
    on_port = {u.name for u in cfg.units if any(l.bus in port_buses for l in u.links)}
    alone = [u for u in cfg.units if u.name not in on_port]
    rows = dict(zip((u.name for u in alone), collect_status(cfg, state, units=alone)))
    locked = [u for u in cfg.units
              if u.name in on_port and not any(l.bus in held_buses for l in u.links)]
    if locked:
        try:
            with port_state.held_wait(), port_state.BusLock():
                rows.update(zip((u.name for u in locked),
                                collect_status(cfg, state, units=locked)))
        except port_state.BusHeld:
            pass
    return [rows.get(u.name) for u in cfg.units]


def tree_payload(cfg, path):
    """The `tree` surface: the declaration's verdict, the store, and every
    declared node, present or absent."""
    from nxs.check import check_manifest, node_findings
    from nxs.finding import Finding, as_data
    from nxs.schemas import CONTRACT
    from nxs.suite import PERSONALITY_DIR, default_state_path
    from nxs.suite.state import SuiteState

    findings = check_manifest(path)
    by_node, rest = node_findings(findings)
    ok = not findings
    ports = []
    if cfg.ports:
        from nxs.cam import cli as cam_cli
        from nxs.cam import topology as cam_topo
        from nxs.cam.verbs.status import daemon_refusal
        # The manifest's ports keyed by carrier, to pair a declared port
        # with the live topology behind it. Its own name: `ports` is the
        # rendered list this builds.
        topologies = {}
        loaded = cam_topo._ports_from_suite()
        if loaded:
            topologies = {t.carrier.split("/")[-1]: t
                          for t in loaded[0].values()}
        # The bus lock is one for every camera bus: a walk that met it held
        # marks the ports after it held without waiting again.
        bus_held = False
        for name in sorted(cfg.ports):
            port = cfg.ports[name]
            entry = {"name": name, "bus": port.bus,
                     "findings": as_data(by_node.get(f"ports.{name}", []))}
            if port.hub_compatible is not None:
                # The entry lists the nodes on the port: a hub, with its driver.
                entry.update({"hub": port.hub_compatible, "driver": port.hub_driver})
            topology = topologies.get(name)
            if topology is not None:
                try:
                    entry["presence"] = cam_cli.presence_payload(topology, held=bus_held)
                    bus_held = bus_held or bool(entry["presence"].get("held"))
                    ok = ok and entry["presence"]["ok"]
                    hub = entry["presence"].get("hub")
                    if (port.hub_source and hub is not None and not hub["present"]
                            and not entry["presence"].get("held")):
                        # The wiring file's hub is gone: the walk that wrote
                        # it down is older than the cabling.
                        from nxs.suite.schema_ports import WIRING_ALTERNATIVE
                        entry["findings"].append(Finding(
                            f"ports.{name}",
                            f"{port.hub_source} names hub {port.hub_compatible} at "
                            f"{hex(port.hub_addr)} and nothing answers there",
                            [WIRING_ALTERNATIVE, "check the hub's power and cabling"]
                        ).to_dict())
                except SystemExit as e:
                    entry["error"] = str(e)
                    ok = False
                except Exception as e:
                    entry["error"] = f"bus unavailable: {e}"
                    ok = False
                # A port whose bring-up nxsd stopped: the refusal it recorded
                # is the port's finding, as the port status says it.
                refused = daemon_refusal(name, topology)
                if refused is not None:
                    entry["findings"].append(
                        Finding(f"ports.{name}", refused.fact, refused.alternatives).to_dict())
                    ok = False
                # A followed pair whose gain nothing copies is the port's gap.
                gap = cam_cli.follow_gap(topology, cam_cli.pair_gain(topology))
                if gap is not None:
                    entry["findings"].append(gap.to_dict())
                    ok = False
            ports.append(entry)

    units = []
    # A unit that rides a camera link is that link's node: `rides` names the
    # link, and the tree prints the unit there, once.
    rides = {link.unit.name: f"{name}/{link.name}" for name, port in cfg.ports.items()
             for link in port.links if link.unit}
    if cfg.units:
        held = {e["bus"] for e in ports if e.get("presence", {}).get("held")}
        if held:
            held = {e["bus"] for e in ports}
        rows = _unit_rows(cfg, held, SuiteState.load(default_state_path()))
        for unit, row in zip(cfg.units, rows):
            entry = {"name": unit.name,
                     "routes": [l.describe() for l in unit.links],
                     "ok": row is not None and row.up,
                     "findings": as_data(by_node.get(f"units.{unit.name}", []))}
            if unit.name in rides:
                entry["rides"] = rides[unit.name]
            if row is None:
                entry["held"] = True
            if not entry["ok"]:
                ok = False
            else:
                if unit.sensors and row.vm_state in _VM_NOT_RUNNING:
                    # The declared personality does not run on a unit that
                    # answers: the rig does not realize the declaration. The
                    # unit's `vm` line says how it stands.
                    off = Finding(f"units.{unit.name}", "its declared personality does not run",
                                  ["nxs switch"])
                    findings.append(off)
                    entry["findings"].append(off.to_dict())
                    ok = False
                entry.update({"route": row.link, "serial": row.serial,
                              "fw": row.fw_version, "personality": row.personality,
                              "vm": row.vm_state, "sync": row.sync,
                              "cal": row.cal, "drift": row.drift,
                              "samples": row.samples, "outputs": row.outputs,
                              "deviations": _deviations(row)})
            units.append(entry)
    shipped, installed = _personality_counts()
    return {"contract": CONTRACT, "manifest": path, "ok": ok,
            "declaration": {"in_tune": not findings, "findings": as_data(findings),
                            "unplaced": as_data(rest)},
            "personalities": {"dir": PERSONALITY_DIR, "shipped": shipped,
                              "installed": installed},
            "ports": ports, "units": units}


def _render_port(entry, riders=()):
    owner = ("" if entry.get("driver", "nxs") == "nxs"
             else f"  ({entry['driver']} — read-only)")
    hub = f"  hub {entry['hub']}" if "hub" in entry else ""
    print(f"  {entry['name']}  {entry['bus']}{hub}{owner}")
    if "error" in entry:
        print(f"    ({entry['error']})")
    elif "presence" in entry:
        from nxs.cam.cli import presence_rows
        for row in presence_rows(entry["presence"]):
            print(f"    {row}")
    # The units riding this port's links: each is its link's node, so its
    # identity and its deviations print here and nowhere else. What it holds
    # is on the link's NXS rows above.
    if riders:
        _render_units(riders, indent="    ", personality=False)
    for finding in entry.get("findings", []):
        print(f"    ! {finding['text']}")


def _placement(units):
    """Port name -> the units that print under it: the units riding its
    links, and every declaration that answers with one of their serials
    (one board declared twice is one node, wherever its routes are)."""
    port_of_serial = {u["serial"]: u["rides"].split("/")[0]
                      for u in units if u.get("rides") and u.get("ok") and u.get("serial")}
    placed = {}
    for u in units:
        port = u["rides"].split("/")[0] if u.get("rides") else port_of_serial.get(u.get("serial"))
        if port is not None:
            placed.setdefault(port, []).append(u)
    return placed


def _render_units(units, indent="  ", personality=True):
    from nxs.cam.verbs.status import bus_held_text

    # One physical board may answer for several declared units; the serial
    # is the identity, so same-serial rows collapse.
    by_serial = {}
    for u in units:
        if u["ok"] and u.get("serial"):
            by_serial.setdefault(u["serial"], []).append(u["name"])
    rendered = set()
    for u in units:
        if not u["ok"]:
            # Not read (the port's bus stayed another run's), or silent.
            state = bus_held_text() if u.get("held") else "NO ANSWER"
            print(f"{indent}{u['name']}  {'; '.join(u['routes'])}  {state}")
            for finding in u.get("findings", []):
                print(f"{indent}  ! {finding['text']}")
            continue
        serial = u.get("serial")
        if serial in rendered:
            continue
        names = by_serial.get(serial) or [u["name"]]
        if serial:
            rendered.add(serial)
        labels = {"fw": "fw", "personality": "click personality"}
        keys = ("fw", "personality") if personality else ("fw",)
        extra = "  ".join(f"{labels[k]} {u[k]}" for k in keys if u.get(k) and u[k] != "-")
        ident = f"serial {serial}  " if serial else ""
        if len(names) > 1:
            routes = "; ".join(x["route"] for x in units
                               if x["name"] in names and x.get("route"))
            print(f"{indent}{' + '.join(names)}  {routes}  ok  {ident}{extra}")
            print(f"{indent}  one board answers all of {', '.join(names)} — "
                  f"declare it once with several links")
        else:
            print(f"{indent}{u['name']}  {u.get('route', '')}  ok  {ident}{extra}")
        if u.get("outputs"):
            print(f"{indent}  outputs: {' '.join(u['outputs'])}")
        for key, value in (u.get("deviations") or {}).items():
            print(f"{indent}  ! {key}: {value}")
        for name in names:
            for finding in next((x for x in units if x["name"] == name), {}).get("findings", []):
                print(f"{indent}  ! {finding['text']}")


def render_tree(as_json: bool = False) -> int:
    from nxs.suite import default_config_path, stray_declaration
    from nxs.suite.schema import ManifestError, load_suite_config

    path = default_config_path()
    if (stray := stray_declaration(path)) is not None:
        term.refusal_text(stray)
        return 1
    if not os.path.exists(path):
        term.refusal(f"no suite manifest at {path}",
                     "nxs generate (writes one from what answers)",
                     "nxs -b <bus> status (one unit, no manifest)")
        return 1
    try:
        cfg = load_suite_config(path)
    except ManifestError as e:
        # The shape findings are the report: a manifest the loader refuses
        # still has a verdict.
        if as_json:
            from nxs.check import check_manifest
            from nxs.finding import as_data
            from nxs.schemas import CONTRACT
            findings = as_data(check_manifest(path))
            print(json.dumps({"contract": CONTRACT, "manifest": path, "ok": False,
                              "declaration": {"in_tune": False,
                                              "findings": findings,
                                              "unplaced": findings},
                              "ports": [], "units": []}, indent=2))
            return 1
        raise SystemExit(f"nxs status: {e}")

    payload = tree_payload(cfg, path)
    if as_json:
        print(json.dumps(payload, indent=2))
        return 0 if payload["ok"] else 1
    findings = payload["declaration"]["findings"]
    # The verdict line counts the declared units that do not answer, which
    # print as NO ANSWER rows far below it.
    silent = sum(1 for u in payload["units"] if not u["ok"] and not u.get("held"))
    unanswered = (f", {silent} unit{'s do' if silent != 1 else ' does'} not answer"
                  if silent else "")
    if findings:
        print(f"declaration: OUT OF TUNE ({len(findings)} finding(s)){unanswered}")
        for finding in payload["declaration"]["unplaced"]:
            print(f"  ! {finding['text']}")
    else:
        print(f"declaration: IN TUNE{unanswered}")
    store = payload["personalities"]
    print(f"personalities: {store['dir']} ({store['shipped']} shipped, "
          f"{store['installed']} installed)")
    if payload["ports"]:
        print("ports:")
        placed = _placement(payload["units"])
        for entry in payload["ports"]:
            _render_port(entry, placed.get(entry["name"], []))
    alone = [u for u in payload["units"] if u["name"] not in
             {u["name"] for group in _placement(payload["units"]).values() for u in group}]
    if alone:
        print("units:")
        _render_units(alone)
    return 0 if payload["ok"] else 1
