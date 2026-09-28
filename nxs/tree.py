# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Bare `nxs probe` and `nxs status`: `render_scan` is config-free discovery of
hubs, units, and direct sensors on the leaf I2C buses; `render_tree` is the
declaration against the rig, node by node. Neither writes a register; `--json`
gives the surfaces."""

import json
import os

from nxs import term


# Hub detection sweeps the des control address; the device-id register
# separates a real hub from an address squatter, the packs name the silicon.
HUB_ADDRESSES = (0x6A,)
SENSOR_ADDRESSES = (0x1A, 0x1D)


def _scan_bus(bus: str, extra=()):
    """One leaf bus: hubs, units, and bare sensors found there; `extra` names
    unit addresses to probe beside the standard ones, and `readable` is False
    when the bus could not be opened."""
    from nxs import _libnxs, _libnxs_unit

    found = {"hubs": [], "units": [], "sensors": [], "readable": True}
    path = _libnxs_unit._bus_path(bus)
    for addr in HUB_ADDRESSES:
        try:
            with _libnxs.Bus.open(path) as handle:
                dev_id = handle.read(addr, b"\x00\x0d", 1)[0]
            from nxs.cam.packs import hub_classes
            kind = hub_classes().get(dev_id)
            found["hubs"].append(
                (addr, f"hub {kind}" if kind else f"id 0x{dev_id:02X}"))
        except Exception:
            continue
    from nxs.suite.scan import I2C_ADDRESSES
    try:
        handle = _libnxs.Bus.open(path)
    except OSError:
        found["readable"] = False
        return found
    with handle:
        for addr in sorted(set(I2C_ADDRESSES) | set(extra)):
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
    return found


def scan_payload():
    """The `scan` surface: what every leaf bus answered."""
    from nxs.schemas import CONTRACT
    from nxs.suite.scan import _i2c_buses

    buses = []
    for bus in _i2c_buses():
        found = _scan_bus(bus)
        unit_addrs = {a for a, _ in found["units"]}
        buses.append({
            "bus": str(bus),
            "hubs": [{"addr": a, "kind": d} for a, d in found["hubs"]],
            "units": [{"addr": a, "serial": s} for a, s in found["units"]],
            "sensors": [a for a in found["sensors"] if a not in unit_addrs],
        })
    return {"contract": CONTRACT, "buses": buses}


def render_scan(as_json: bool = False) -> int:
    """Config-free discovery of every leaf bus (bare `nxs probe`)."""
    payload = scan_payload()
    anything = any(b["hubs"] or b["units"] or b["sensors"]
                   for b in payload["buses"])
    if as_json:
        print(json.dumps(payload, indent=2))
        return 0 if anything else 1
    if not payload["buses"]:
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
    if not anything:
        print("no hubs, units, or sensors answered "
              f"({len(payload['buses'])} buses swept)")
    print(_next_after_probe(anything))
    return 0 if anything else 1


def _next_after_probe(anything: bool) -> str:
    """The step after discovery: compare against the declaration when there is
    one, write one from what answered when there is none."""
    import os

    from nxs.suite import default_config_path

    if os.path.exists(default_config_path()):
        return "declared-vs-actual: nxs status"
    if not anything:
        return ("no suite.yaml yet, and nothing to declare — connect a unit "
                "and run nxs probe again")
    return "no suite.yaml yet — write down what answered: nxs generate"


def _installed_personalities() -> int:
    """How many personalities the store holds (a directory or a file pair
    each)."""
    from nxs.suite import PERSONALITY_DIR

    try:
        entries = os.listdir(PERSONALITY_DIR)
    except OSError:
        return 0
    return sum(1 for e in entries
               if os.path.isdir(os.path.join(PERSONALITY_DIR, e)) or e.endswith(".py"))


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


def tree_payload(cfg, path):
    """The `tree` surface: the declaration's verdict, the store, and every
    declared node, present or absent."""
    from nxs.check import check_manifest, node_findings
    from nxs.finding import Finding, as_data
    from nxs.schemas import CONTRACT
    from nxs.suite import PERSONALITY_DIR, default_state_path
    from nxs.suite.state import SuiteState
    from nxs.suite.status import collect_status

    findings = check_manifest(path)
    by_node, rest = node_findings(findings)
    ok = not findings
    ports = []
    if cfg.ports:
        from nxs.cam import cli as cam_cli
        from nxs.cam import topology as cam_topo
        # The manifest's ports keyed by carrier, to pair a declared port
        # with the live topology behind it. Its own name: `ports` is the
        # rendered list this builds.
        topologies = {}
        loaded = cam_topo._ports_from_suite()
        if loaded:
            topologies = {t.carrier.split("/")[-1]: t
                          for t in loaded[0].values()}
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
                    entry["presence"] = cam_cli.presence_payload(topology)
                    ok = ok and entry["presence"]["ok"]
                    hub = entry["presence"].get("hub")
                    if port.hub_source and hub is not None and not hub["present"]:
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
            ports.append(entry)

    units = []
    # A unit that rides a camera link is that link's node: `rides` names the
    # link, and the tree prints the unit there, once.
    rides = {link.unit.name: f"{name}/{link.name}" for name, port in cfg.ports.items()
             for link in port.links if link.unit}
    if cfg.units:
        rows = collect_status(cfg, SuiteState.load(default_state_path()))
        for unit, row in zip(cfg.units, rows):
            entry = {"name": unit.name,
                     "routes": [l.describe() for l in unit.links],
                     "ok": row.up,
                     "findings": as_data(by_node.get(f"units.{unit.name}", []))}
            if unit.name in rides:
                entry["rides"] = rides[unit.name]
            if not row.up:
                ok = False
            else:
                entry.update({"route": row.link, "serial": row.serial,
                              "fw": row.fw_version, "personality": row.driver,
                              "vm": row.vm_state, "sync": row.sync,
                              "cal": row.cal, "drift": row.drift,
                              "samples": row.samples, "outputs": row.outputs,
                              "deviations": _deviations(row)})
            units.append(entry)
    return {"contract": CONTRACT, "manifest": path, "ok": ok,
            "declaration": {"in_tune": not findings, "findings": as_data(findings),
                            "unplaced": as_data(rest)},
            "personalities": {"dir": PERSONALITY_DIR,
                              "installed": _installed_personalities()},
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
        _render_units(riders, indent="    ", held=False)
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


def _render_units(units, indent="  ", held=True):
    # One physical board may answer for several declared units; the serial
    # is the identity, so same-serial rows collapse.
    by_serial = {}
    for u in units:
        if u["ok"] and u.get("serial"):
            by_serial.setdefault(u["serial"], []).append(u["name"])
    rendered = set()
    for u in units:
        if not u["ok"]:
            print(f"{indent}{u['name']}  {'; '.join(u['routes'])}  NO ANSWER")
            for finding in u.get("findings", []):
                print(f"{indent}  ! {finding['text']}")
            continue
        serial = u.get("serial")
        if serial in rendered:
            continue
        names = by_serial.get(serial) or [u["name"]]
        if serial:
            rendered.add(serial)
        labels = {"fw": "fw", "personality": "sensor personality"}
        extra = "  ".join(f"{labels[k]} {u[k]}" for k in (("fw", "personality") if held else ("fw",))
                          if u.get(k) and u[k] != "-")
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
    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config

    path = default_config_path()
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
    if findings:
        print(f"declaration: OUT OF TUNE ({len(findings)} finding(s))")
        for finding in payload["declaration"]["unplaced"]:
            print(f"  ! {finding['text']}")
    else:
        print("declaration: IN TUNE")
    store = payload["personalities"]
    print(f"personalities: {store['dir']} ({store['installed']} installed)")
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
