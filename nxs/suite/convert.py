# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The converter `nxs generate` writes an earlier declaration with: an
api-2 file, and a 1.0 file through its port view (`port_view`), into api 3.
Each value lands on the row of the block that states it: a hub's frame
sync, gain and exposure on its rows, a host connector's lanes and driver
on the host's rows named for the connector, a camera's mode by name and
its rate on its `fps` row (under the hub's frame sync the pair's one rate
on the hub's `sync_fps`), a unit's egress factors on its own rows, its
mounting orientation on the row of its IMU or magnetometer click, a port
fact on its identity pin, a hub's control bus off its connector on the I²C
adapter it names. A fact api 3 derives and states nowhere (a link's
window, channel or serializer address, a capture id, a sensor's clock) is
refused, naming it, so a file never converts into another rig in silence.
A pod's own alias converts to its link's letter's, which `generate` names. A mode token naming a camera's one mode, a
camera with no mode row, states nothing and is left out."""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from nxs.suite.schema_base import ManifestError

#: The language this converter reads.
SOURCE_API = 2
#: The unit rows an api-2 egress section's factors land on, by its subject token.
_DECIMATION_ROWS = {"acceleration": "decim.accel", "angular_velocity": "decim.gyro",
                    "magnetic_field": "decim.mag", "temperature": "decim.temp",
                    "pressure": "decim.press", "scalar": "decim.scalar"}
#: A hub's frame sync source by its api-2 spelling, as the hub's row names it.
_SYNC_WORDS = {"free_run": "free", "fsync": "fsync"}
#: The api-2 kind word of a smart module, and the unit module api 3 names it by.
_MODULE_TYPE = "nxs-module"


def is_api2(raw) -> bool:
    """Whether a declaration is written in api 2."""
    return isinstance(raw, dict) and raw.get("api") == SOURCE_API


def api3(doc: dict, where: str = "suite.yaml") -> dict:
    """The api-3 declaration an api-2 one states. ManifestError names a
    value no row and no pin of api 3 takes, and the type that does not
    describe."""
    nodes = doc.get("nodes") or {}
    connections = {str(k): list(v or []) for k, v in (doc.get("connections") or {}).items()}
    defaults = doc.get("defaults") or {}
    host = next((str(n) for n, spec in nodes.items() if (spec or {}).get("kind") == "host"), None)
    out: Dict[str, dict] = {}
    renames: Dict[Tuple[str, str], str] = {}
    params: Dict[str, Dict[str, Any]] = {}
    for name, spec in nodes.items():
        name, spec = str(name), dict(spec or {})
        node_type = str(spec.get("type", ""))
        if spec.get("kind") == "module":
            node_type = _MODULE_TYPE
        out[name] = {"type": node_type}
        params[name] = {}
    for name, spec in nodes.items():
        name, spec = str(name), dict(spec or {})
        nwhere = f"{where}.nodes.{name}"
        kind = spec.get("kind")
        given = dict(spec.get("parameters") or {})
        facts = {str(p): dict(f or {}) for p, f in (spec.get("ports") or {}).items()}
        if kind == "host":
            _host(name, facts, params[name], renames, connections, nwhere)
        elif kind == "hub":
            _hub(name, out[name]["type"], given, facts, params, host, connections, nwhere)
        elif kind == "unit":
            _unit(name, given, facts, params[name], defaults, connections, nodes, params, nwhere)
        elif kind == "camera":
            _camera(name, out[name]["type"], given, facts, params[name], nwhere)
        elif kind == "click":
            _click(out[name]["type"], given, params[name], nwhere)
        elif given or facts:
            raise ManifestError(f"{nwhere}: a {kind} states parameters, and api 3 gives it none",
                                ["drop the node's parameters and ports"])
    for name, spec in nodes.items():
        if (spec or {}).get("kind") == "hub" and params[str(name)].get("sync_source") == "fsync":
            _synced_rate(str(name), nodes, params, connections, f"{where}.nodes.{name}")
    for name, node in out.items():
        if params[name]:
            node["parameters"] = params[name]
    _lanes_agree(out, params, host, connections)
    doc3: dict = {"api": 3}
    if doc.get("suite"):
        doc3["suite"] = dict(doc["suite"])
    doc3["nodes"] = out
    doc3["connections"] = {_renamed(key, renames): [_renamed(t, renames) for t in targets]
                           for key, targets in connections.items()}
    return doc3


def _renamed(end: str, renames: Dict[Tuple[str, str], str]) -> str:
    node, _, port = str(end).partition(".")
    return f"{node}.{renames.get((node, port), port)}"


def _describe(node_type: str, where: str):
    from nxs.suite.nodes import describe_type

    return describe_type(node_type, where)


def _refuse(where: str, what: str) -> None:
    raise ManifestError(f"{where}: {what}, and api 3 derives it and states it nowhere",
                        ["drop the fact, then nxs generate again"])


# ── the host ─────────────────────────────────────────────────────────

def _host(name: str, facts: dict, rows: dict, renames: dict, connections: dict,
          where: str) -> None:
    """A host's port facts: a connector's lanes onto its `<connector>_lanes`
    row, each bus it declares renamed for its device, a serial port's rate
    onto its `<port>_baud` row. A hub's control bus other than the one the
    host resolves the connector's name to is the I²C adapter it names."""
    from nxs.host import buses

    for port, fact in facts.items():
        pwhere = f"{where}.ports.{port}"
        if port.startswith("bus_cam"):
            connector = port[len("bus_"):]
            bus = fact.get("bus") if fact else None
            if bus is None or bus in _connector_buses(connector):
                continue
            adapter = buses.port_name("i2c", str(bus))
            if adapter is None or f"{name}.{port}" not in connections:
                _refuse(f"{pwhere}.bus", f"{bus!r} names the bus behind {connector}, which the "
                                         f"host resolves to {_connector_buses(connector)[-1]}")
            renames[(name, port)] = adapter
            continue
        kind = fact.get("type")
        if kind is None:
            for key in fact:
                if key == "lanes":
                    rows[f"{port}_lanes"] = int(str(fact["lanes"]), 0)
                else:
                    _refuse(f"{pwhere}.{key}", f"the connector's {key} {fact[key]!r}")
            continue
        device = {"i2c": fact.get("bus"), "cyphal-can": fact.get("iface") or port,
                  "cyphal-serial": fact.get("port")}.get(kind)
        found = buses.port_name(kind, str(device)) if device else None
        if found is None:
            raise ManifestError(f"{pwhere}: the {kind} bus {device!r} has no port named for its "
                                f"device, i2c-<N>, can<N> or tty<name>",
                                [f"{port}: a bus at /dev/i2c-<N>, a CAN interface or a "
                                 f"/dev/tty<name> device"])
        renames[(name, port)] = found
        if kind == "cyphal-serial" and fact.get("baud") is not None:
            baud = int(str(fact["baud"]), 0)
            if baud != buses.BAUDS[0]:
                rows[buses.baud_row(found).name] = baud


def _connector_buses(connector: str) -> Tuple[str, ...]:
    """The buses a connector's name resolves to: its udev alias, and the
    bus this host gives the name."""
    from nxs import host as host_layer

    try:
        found = host_layer.current().camera_buses().get(connector)
    except Exception:        # noqa: BLE001 (a host without port rules names no bus)
        found = None
    return tuple(dict.fromkeys(b for b in (f"/dev/i2c-{connector}", found) if b))


# ── the hub ──────────────────────────────────────────────────────────

def _hub(name: str, node_type: str, given: dict, facts: dict, params: dict, host: Optional[str],
         connections: dict, where: str) -> None:
    """A hub's parameters onto its rows: its frame sync, its gain and its
    exposure; its `driver` onto the host's row for the connector its csi.A
    feeds; its output ports' lanes and copy onto its rows."""
    rows = params[name]
    block = _describe(node_type, f"{where}.type")
    for key, raw in given.items():
        kwhere = f"{where}.parameters.{key}"
        if key == "address":
            rows["address"] = raw
        elif key == "driver":
            if raw == "nxs":
                continue
            connector = next((str(t).split(".", 1)[1] for t in connections.get(f"{name}.csi.A", [])
                              if host is not None and str(t).split(".", 1)[0] == host), None)
            if connector is None:
                raise ManifestError(f"{kwhere}: names who drives the hub from the host, and its "
                                    f"csi.A feeds no connector of the host", ["drop the key"])
            params[host][f"{connector}_driver"] = raw
        elif key == "sync":
            sync = dict(raw or {})
            source = str(sync.get("source", "free_run"))
            if source not in _SYNC_WORDS:
                raise ManifestError(f"{kwhere}.source: {source!r} (one of {', '.join(_SYNC_WORDS)})")
            if source != "free_run":
                rows["sync_source"] = _SYNC_WORDS[source]
            if sync.get("fps") is not None:
                rows["sync_fps"] = _number(_rate(sync["fps"], f"{kwhere}.fps"))
        elif key in ("gain_db", "exposure_us"):
            rows["gain" if key == "gain_db" else "exposure"] = _number(_finite(raw, key, kwhere))
        else:
            raise ManifestError(f"{kwhere}: no hub parameter of api 2 is named {key}")
    for port, fact in facts.items():
        pwhere = f"{where}.ports.{port}"
        if port in ("csi.A", "csi.B"):
            prefix = port.lower().replace(".", "_")
            for key, raw in fact.items():
                if key == "lanes":
                    rows[f"{prefix}_lanes"] = int(str(raw), 0)
                elif key == "copy" and port == "csi.B":
                    rows["csi_b_copy"] = "on" if raw else "off"
                else:
                    _refuse(f"{pwhere}.{key}", f"the {port} output's {key} {raw!r}")
            continue
        derived = _link_rules(node_type, port.split(".", 1)[-1])
        for key, raw in fact.items():
            # A fact that states the hub's own rule is the hub's to state.
            if key in derived and _same(raw, derived[key]):
                continue
            _refuse(f"{pwhere}.{key}", f"the link's {key} {_fact_text(key, raw)}"
                    + (f" departs from the hub's {_fact_text(key, derived[key])}"
                       if key in derived else ""))
    for key in [k for k in rows if k != "address"]:
        try:
            block.row(key)
        except KeyError:
            raise ManifestError(f"{where}: {node_type} has no row {key}",
                                [r.name for r in block.rows]) from None


def _synced_rate(hub: str, nodes: dict, params: dict, connections: dict, where: str) -> None:
    """Under the hub's frame sync the pair runs the hub's rate: the rate its
    `sync` states, else the one rate the cameras behind it state, which
    moves onto its `sync_fps` row. A camera behind it states no rate of its
    own, and two rates are refused."""
    rates = {letter: params[camera].pop("fps")
             for letter, camera in sorted(_cameras_behind(hub, nodes, connections).items())
             if "fps" in params[camera]}
    if not rates or "sync_fps" in params[hub]:
        return
    if len({float(rate) for rate in rates.values()}) > 1:
        stated = ", ".join(f"{letter} {float(rate):g}" for letter, rate in rates.items())
        raise ManifestError(f"{where}.parameters.sync: under frame sync the pair runs one rate "
                            f"({stated})", [f"nodes.{hub}.parameters.sync.fps: <one rate>"])
    params[hub]["sync_fps"] = next(iter(rates.values()))


def _cameras_behind(hub: str, nodes: dict, connections: dict) -> Dict[str, str]:
    """The camera on each of a hub's links, by the link's letter: the one the
    link takes, or the one on the head of the pod the link takes."""
    def kind(name: str):
        return (nodes.get(name) or {}).get("kind")

    def source(target: str) -> Optional[str]:
        return next((key.split(".", 1)[0] for key, targets in connections.items()
                     if target in map(str, targets)), None)

    cameras: Dict[str, str] = {}
    for key, targets in connections.items():
        node = key.split(".", 1)[0]
        for target in map(str, targets):
            if not target.startswith(f"{hub}.links."):
                continue
            camera = source(f"{node}.head") if kind(node) == "unit" else node
            if camera is not None and kind(camera) == "camera":
                cameras[target.rsplit(".", 1)[1]] = camera
    return cameras


def _link_rules(hub_type: str, letter: str) -> Dict[str, Any]:
    """What a hub's rules give a link behind it: its serializer and the
    serializer's and expander's addresses, its window and its channel."""
    from nxs.cam.topology import _hub_rules
    from nxs.suite.schema_nodes import PortLinkSpec

    ser, windows, vcs = _hub_rules(hub_type)
    plain = PortLinkSpec(name=letter, camera=None)
    rules: Dict[str, Any] = {"ser_addr": plain.ser_addr, "tca_addr": plain.tca_addr}
    for key, value in (("ser", ser), ("des_window", windows.get(letter)), ("csi_vc", vcs.get(letter))):
        if value is not None:
            rules[key] = value
    return rules


#: The link facts that are I²C addresses, named in hex as the hub's rules name them.
_ADDRESS_FACTS = frozenset({"des_window", "ser_addr", "tca_addr"})


def _fact_text(key: str, value) -> str:
    """A link fact as a refusal names it: an address in hex, else as stated."""
    if key in _ADDRESS_FACTS:
        try:
            return f"{int(str(value), 0):#04x}"
        except ValueError:
            pass
    return repr(value)


def _same(raw, rule) -> bool:
    """Whether a stated fact is the rule's value: a name as text, a number in any base."""
    if isinstance(rule, str):
        return str(raw) == rule
    try:
        return int(str(raw), 0) == int(rule)
    except ValueError:
        return False


def _lanes_agree(nodes: dict, params: dict, host: Optional[str], connections: dict) -> None:
    """A hub's output into a host connector runs one lane count, on both
    ports: the count the earlier file states, else the connector's own. The
    hub's `csi_a_lanes` states it where it is not the hub's default; a
    count the file states stays stated, on the host's row where the hub's
    does not carry it."""
    if host is None:
        return
    host_block = _describe(nodes[host]["type"], f"nodes.{host}.type")
    for key, targets in connections.items():
        node, _, port = key.partition(".")
        if port != "csi.A" or node not in params:
            continue
        for target in targets:
            to, _, connector = str(target).partition(".")
            if to != host:
                continue
            row = f"{connector}_lanes"
            try:
                wired = host_block.row(row).default
                own = _describe(nodes[node]["type"], f"nodes.{node}.type").row("csi_a_lanes").default
            except KeyError:
                continue
            host_said, hub_said = params[host].get(row), params[node].get("csi_a_lanes")
            if host_said is not None and hub_said is not None and int(host_said) != int(hub_said):
                raise ManifestError(
                    f"nodes.{node}.ports.csi.A.lanes: {hub_said}, and {host}.{connector} takes "
                    f"{host_said}; the hub's output and the connector run one lane count",
                    [f"nodes.{node}.ports.csi.A.lanes: {host_said}"])
            stated = host_said if host_said is not None else hub_said
            want = int(stated) if stated is not None else int(wired)
            if want != own:
                params[node]["csi_a_lanes"] = want
            # A count the hub's row carries needs the host's only where it is not the wired one.
            if host_said is not None and want == int(wired) and "csi_a_lanes" in params[node]:
                del params[host][row]
            for name in (node, host):
                if params[name]:
                    nodes[name]["parameters"] = params[name]
                else:
                    nodes[name].pop("parameters", None)


# ── the unit and the clicks on it ────────────────────────────────────

def _unit(name: str, given: dict, facts: dict, rows: dict, defaults: dict, connections: dict,
          nodes: dict, params: dict, where: str) -> None:
    """A unit's pins, its egress factors onto its rows, its orientation
    onto the row of the click on it that takes one, its ports' facts onto
    its `address` pin and its `can_node_id` row."""
    from nxs.generate_seed import pod_alias

    for key, raw in given.items():
        kwhere = f"{where}.parameters.{key}"
        if key in ("serial", "firmware"):
            rows[key] = raw
        elif key == "click":
            if raw != "none":
                raise ManifestError(f"{kwhere}: {raw!r}; a click on the socket is a node wired "
                                    f"into {name}.click")
        elif key == "egress":
            egress = dict(raw or {})
            if egress.get("decimation") is not None:
                rows["decimation"] = int(str(egress["decimation"]), 0)
            for subject, factor in dict(egress.get("subjects") or {}).items():
                if subject not in _DECIMATION_ROWS:
                    raise ManifestError(f"{kwhere}.subjects: unknown subject {subject!r} "
                                        f"(one of {', '.join(_DECIMATION_ROWS)})")
                rows[_DECIMATION_ROWS[subject]] = int(str(factor), 0)
        elif key == "orientation":
            _orientation(name, str(raw).upper(), connections, nodes, params, kwhere)
        else:
            raise ManifestError(f"{kwhere}: no unit parameter of api 2 is named {key}")
    if "firmware" not in rows and defaults.get("firmware") is not None:
        rows["firmware"] = str(defaults["firmware"])
    letter = next((str(t).rsplit(".", 1)[1] for t in connections.get(f"{name}.out", [])
                   if ".links." in str(t)), None)
    for port, fact in facts.items():
        for key, raw in fact.items():
            kwhere = f"{where}.ports.{port}.{key}"
            if (port, key) in (("out", "target"), ("i2c", "address")):
                if "address" in rows and int(str(rows["address"]), 0) != int(str(raw), 0):
                    raise ManifestError(f"{kwhere}: {raw}, and the unit straps "
                                        f"{rows['address']} by its other port's fact",
                                        ["state one address the unit straps"])
                rows["address"] = raw
            elif (port, key) == ("out", "alias"):
                # A pod's alias follows its link's letter: an earlier
                # declaration's own alias converts to it, and generate names it.
                continue
            elif (port, key) == ("can", "node_id"):
                rows["can_node_id"] = int(str(raw), 0)
            else:
                raise ManifestError(f"{kwhere}: no fact of a unit's {port} port is named {key}")


def _orientation(unit: str, rotation: str, connections: dict, nodes: dict, params: dict,
                 where: str) -> None:
    """A unit's mounting orientation onto every click on its socket whose
    personality declares the row."""
    placed = False
    for key, targets in connections.items():
        click = key.split(".", 1)[0]
        if f"{unit}.click" not in targets or click not in nodes:
            continue
        block = _describe(str(nodes[click].get("type")), f"nodes.{click}.type")
        try:
            block.row("orientation")
        except KeyError:
            continue
        params[click].setdefault("orientation", rotation)
        placed = True
    if not placed:
        raise ManifestError(f"{where}: {rotation} is the mounting rotation of an IMU or "
                            f"magnetometer click, and no click on {unit} declares the row",
                            [f"connections.<click>.out: [{unit}.click] with an IMU or "
                             f"magnetometer personality", "drop the key"])


def _click(node_type: str, given: dict, rows: dict, where: str) -> None:
    """A click's configuration onto its rows, each value in the row's display form."""
    from nxs.description import display

    block = _describe(node_type, f"{where}.type")
    for key, raw in given.items():
        try:
            row = block.row(str(key))
        except KeyError:
            raise ManifestError(f"{where}.parameters.{key}: {node_type} has no row {key}",
                                [r.name for r in block.rows]) from None
        try:
            rows[str(key)] = _plain(display(row, int(str(raw), 0)))
        except ValueError:
            rows[str(key)] = raw


# ── the camera ───────────────────────────────────────────────────────

def _camera(name: str, node_type: str, given: dict, facts: dict, rows: dict, where: str) -> None:
    """A camera's mode by name, its rate on its `fps` row, its address pin,
    and on a connector its gain and exposure."""
    from nxs.description import display

    if facts:
        raise ManifestError(f"{where}.ports: a camera's port takes no facts",
                            ["drop the node's ports"])
    block = _describe(node_type, f"{where}.type")
    for key, raw in given.items():
        kwhere = f"{where}.parameters.{key}"
        if key == "address":
            rows["address"] = raw
        elif key == "mode":
            name = mode_name(block, node_type, str(raw).strip(), kwhere)
            if name is not None:
                rows["mode"] = name
        elif key == "fps":
            if not _has(block, "fps"):
                raise ManifestError(f"{kwhere}: {node_type} offers no fps row: its rate "
                                    f"is its mode's", ["drop the key"])
            # The row serves milli-frames per second.
            rows["fps"] = _plain(display(block.row("fps"), round(_rate(raw, kwhere) * 1000)))
        elif key in ("gain_db", "exposure_us"):
            row = "gain" if key == "gain_db" else "exposure"
            if not _has(block, row):
                raise ManifestError(f"{kwhere}: {node_type} offers no {row} row", ["drop the key"])
            rows[row] = _number(_finite(raw, key, kwhere))
        elif key == "inck_hz":
            _refuse(kwhere, f"the clock {raw} Hz the pod feeds the sensor")
        else:
            raise ManifestError(f"{kwhere}: no camera parameter of api 2 is named {key}")


def mode_name(block, node_type: str, token: str, where: str) -> Optional[str]:
    """The mode a token names, as the camera's mode row names it: the name
    itself, or the one mode a geometry names among the sensor's modes. A
    camera whose program stages no mode switch has no mode row and runs its
    one mode: a token naming it states nothing (None)."""
    from nxs.cam import cam_personalities
    from nxs.cam.descriptors import resolve_mode

    try:
        labels = block.row("mode").present.labels or ()
    except KeyError:
        try:
            descriptor = cam_personalities.find(node_type)
            named, (only, *more) = resolve_mode(descriptor, token), descriptor.program_modes()
        except Exception as exc:        # noqa: BLE001 (a token naming no mode, or none offered)
            raise ManifestError(f"{where}: {token!r} names no mode {node_type} runs: {exc}",
                                ["drop the key"]) from None
        if named == only and not more:
            return None
        raise ManifestError(f"{where}: {node_type} runs {only} alone, and {token!r} names {named}",
                            ["drop the key"])
    if token in labels:
        return token
    try:
        return str(resolve_mode(cam_personalities.find(node_type), token))
    except Exception as exc:        # noqa: BLE001 (a token naming several modes, or none)
        raise ManifestError(f"{where}: {token!r} names no one mode of {node_type}: {exc}",
                            [f"mode: {m}" for m in labels]) from None


def _has(block, row: str) -> bool:
    try:
        block.row(row)
    except KeyError:
        return False
    return True


def _finite(value, key: str, where: str) -> float:
    """A gain in dB or an exposure in microseconds, a finite number; the
    row it lands on judges its span."""
    unit = "dB" if key == "gain_db" else "microseconds"
    try:
        number = float(value) if not isinstance(value, bool) else float("nan")
    except (TypeError, ValueError):
        number = float("nan")
    if number != number or number in (float("inf"), float("-inf")):
        raise ManifestError(f"{where}: {value!r}, and it is a finite number of {unit}",
                            ["drop the key"])
    return number


def _rate(value, where: str) -> float:
    """A rate in frames per second, a positive number; ManifestError names
    any other value."""
    try:
        rate = float(value) if not isinstance(value, bool) else 0.0
    except (TypeError, ValueError):
        rate = 0.0
    if not rate > 0 or rate == float("inf"):
        raise ManifestError(f"{where}: {value!r}, and a rate is a positive number of frames per "
                            f"second", ["drop the key"])
    return rate


def _number(value):
    """A number as a declaration spells it: whole when it is one."""
    number = float(value)
    return int(number) if number.is_integer() else round(number, 3)


def _plain(text: str):
    """A display text as YAML spells it: a whole or a decimal number as a
    number, a name as text."""
    try:
        number = float(text)
    except ValueError:
        return text
    return int(number) if number.is_integer() and "." not in text else number


__all__ = ["SOURCE_API", "api3", "is_api2", "mode_name"]
