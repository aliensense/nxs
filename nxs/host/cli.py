# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""`nxs host ...`: the capture host's contract, read: `info` (which host, boot
label, booted ports) and `modes <port>` (the booted capture table). The
overlays and the capture stack's configuration are derivations `nxs switch`
and nxsd realize through the functions below."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path
from typing import Any, Callable, Collection, Dict, List, Optional, Tuple

from nxs import host as host_layer


def add_host_parser(sub) -> None:
    p = sub.add_parser("host", help="the capture host's contract: booted "
                                    "modes, the boot label")
    host_sub = p.add_subparsers(dest="host_cmd", required=True)

    q = host_sub.add_parser("info", help="which host, boot label, booted ports")
    q.add_argument("--json", action="store_true")

    q = host_sub.add_parser("modes", help="the booted capture table of a port")
    q.add_argument("port", help="port name (cam0, cam1)")
    q.add_argument("--json", action="store_true")


def cmd_host(args: argparse.Namespace) -> int:
    if args.host_cmd == "info":
        return _cmd_info(args)
    if args.host_cmd == "modes":
        return _cmd_modes(args)
    raise SystemExit(f"nxs host: unknown verb {args.host_cmd!r}")


def _ports() -> Dict[str, Any]:
    from nxs.cam.topology import load_ports
    from nxs.cam.port_state import port_name

    try:
        ports, _ = load_ports(None)
    except Exception as exc:
        raise SystemExit(f"nxs host: no camera ports known ({exc})")
    return {port_name(t): t for t in ports.values()}


def _port(args) -> Any:
    ports = _ports()
    name = str(args.port).lower()
    if name not in ports:
        raise SystemExit(f"nxs host: no port {args.port!r}; have "
                         f"{', '.join(sorted(ports)) or 'none'}")
    return ports[name]


def host_info() -> Dict[str, Any]:
    host = host_layer.current()
    info: Dict[str, Any] = {"host": host.describe(), "kind": host.name,
                            "stack": host.stack(),
                            "dtc": shutil.which("dtc") is not None,
                            "ports": {}}
    boot = host.boot_state()
    info["boot_label"] = boot.get("entry")
    info["boot_overlays"] = list(boot.get("overlays") or [])
    try:
        ports = _ports()
    except SystemExit:
        ports = {}
    for name, topology in sorted(ports.items()):
        modes = host.booted_modes(topology.i2c_bus)
        info["ports"][name] = {
            "bus": topology.i2c_bus,
            "booted_lanes": host.booted_lanes(topology.i2c_bus),
            "booted_modes": len(modes),
            "sensors_in_table": sorted({c for m in modes for c in m["pool"]}),
            "capture_ids": host.capture_ids(topology.i2c_bus),
            "tuning": host.tuning_state(name),
        }
    return info


def _cmd_info(args) -> int:
    info = host_info()
    if args.json:
        print(json.dumps(info, indent=2))
        return 0
    print(f"host: {info['host']}, {info['stack']}"
          + ("" if info["dtc"] else " (dtc not installed)"))
    if info.get("boot_label"):
        print(f"boot label: {info['boot_label']}")
        for o in info["boot_overlays"]:
            print(f"  {o}")
    for name, port in info["ports"].items():
        lanes = port["booted_lanes"]
        print(f"{name}: {port['bus']} — "
              + (f"{lanes}-lane, {port['booted_modes']} booted modes for "
                 f"{', '.join(port['sensors_in_table']) or 'no sensor'}"
                 if port["booted_modes"] else "device tree silent")
              + (f"; capture ids {port['capture_ids']}" if port["capture_ids"]
                 else "; no capture ids read" if port["booted_modes"] else "")
              + (f"; tuning {port['tuning']}" if port.get("tuning") else ""))
    return 0


def tuning_record_path(port: str) -> Path:
    """Where the tool records the capture table a port's tuning file was made
    for: the operator's state, beside the port record."""
    from nxs.cam import port_state

    return port_state.state_dir() / "tuning" / f"{port}.yaml"


def _table_rows(host, bus: str) -> List[Dict[str, Any]]:
    """The facts the capture stack matches a tuning file's knob sets by, one
    row per booted mode index (the tree lists the table once per capture
    node): geometry, bit depth and top rate."""
    rows = {int(m["index"]): {"index": int(m["index"]), "width": int(m["width"]),
                              "height": int(m["height"]), "bit_depth": int(m["bit_depth"]),
                              "max_fps": round(float(m["max_fps"]), 2)}
            for m in host.booted_modes(bus)}
    return [rows[n] for n in sorted(rows)]


def tuning_hash(stack: str, rows: List[Dict[str, Any]],
                overrides: Optional[Dict[str, Dict[str, str]]] = None) -> str:
    """The name of the tuning derivation: a digest of the capture stack,
    the booted rows it is made for (the inputs the stack matches by) and
    the vendor overrides folded in, by sensor and digest."""
    import hashlib

    facts: Dict[str, Any] = {"stack": stack, "rows": rows}
    if overrides:
        facts["overrides"] = {c: o["sha256"] for c, o in sorted(overrides.items())}
    return hashlib.sha256(json.dumps(facts, sort_keys=True).encode()).hexdigest()[:16]


def tuning_overrides(host, hub, bus: str) -> Dict[str, Dict[str, str]]:
    """Per booted sensor, the vendor override its descriptor names
    (`capture.tuning`: url and sha256); a sensor the hub has no
    descriptor for, or whose descriptor names none, is left out."""
    from nxs.cam.hubs import HubError

    out: Dict[str, Dict[str, str]] = {}
    if hub is None:
        return out
    for mode in host.booted_modes(bus):
        for compatible in mode.get("pool") or []:
            if compatible in out:
                continue
            try:
                capture = hub.descriptor(compatible).raw("capture") or {}
            except HubError:
                continue
            tuning = capture.get("tuning") or {}
            if tuning.get("override_url") and tuning.get("sha256"):
                out[compatible] = {"url": str(tuning["override_url"]),
                                   "sha256": str(tuning["sha256"]).lower()}
    return out


def override_store_path(sha256: str) -> Path:
    """Where a fetched vendor override lives, by its digest."""
    from nxs.cam import port_state

    return port_state.state_dir() / "tuning" / "overrides" / f"{sha256}.isp"


def fetch_override(url: str, sha256: str, log=print) -> Path:
    """The vendor override at `url`, in the store under its digest: reused
    while the stored file's digest matches, fetched once otherwise. An
    unreachable URL or another digest is a TuningRefused naming the
    offline copy step or the descriptor's digest."""
    import hashlib
    import os
    import urllib.request

    target = override_store_path(sha256)
    if target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest() == sha256:
        return target
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_suffix(".tmp")
    log(f"tuning: fetching the vendor override {url}")
    try:
        urllib.request.urlretrieve(url, partial)
    except OSError as exc:
        raise TuningRefused(f"cannot fetch {url} ({exc})",
                            [f"copy the file to {target} by hand (sha256 {sha256}), "
                             f"then nxs switch"]) from exc
    got = hashlib.sha256(partial.read_bytes()).hexdigest()
    if got != sha256:
        partial.unlink(missing_ok=True)
        raise TuningRefused(f"{url}: sha256 {got}, the descriptor names {sha256}",
                            ["capture.tuning.sha256 names the published file"])
    os.replace(partial, target)
    return target


def _hub_of(topology):
    """The hub whose descriptors name the port's sensors, None when none
    covers the port (a host whose sensors no hub knows builds no override)."""
    from nxs.cam import hubs

    try:
        return hubs.for_topology(topology)
    except Exception:        # noqa: BLE001 (the build reports its own facts)
        return None


def tuning_store_path(badge: str, digest: str) -> Path:
    """Where the tool keeps a built tuning object: by the badge it was made
    under, since the capture stack refuses the object under any other, and
    by its derivation name."""
    from nxs.cam import port_state

    return port_state.state_dir() / "tuning" / "store" / f"{badge}-{digest}.nito"


def _default_sensor_id(host, bus: str) -> int:
    """The capture source the tuning sessions open: the port's lowest
    capture id, 0 where the host names none."""
    ids = host.capture_ids(bus) or {}
    return min(ids.values()) if ids else 0


def _record_tuning(port: str, host, rows: List[Dict[str, Any]], digest: str,
                   installed: Path,
                   overrides: Optional[Dict[str, Dict[str, str]]] = None) -> Path:
    import yaml

    record = tuning_record_path(port)
    record.parent.mkdir(parents=True, exist_ok=True)
    made: Dict[str, Any] = {"port": port, "badge": host.tuning_badge(port), "stack": host.stack(),
                            "hash": digest, "file": str(installed), "rows": rows}
    if overrides:
        made["overrides"] = {c: o["sha256"] for c, o in sorted(overrides.items())}
    record.write_text(yaml.safe_dump(made, sort_keys=False))
    return record


def build_tuning(host, topology, sensor_id: Optional[int] = None,
                 log=print) -> Tuple[Path, Path]:
    """Make the port's tuning object for the booted table, keep it in the
    store under its derivation name, install it, and record the rows it
    serves. Returns (installed, record). Raises what `tuning_make` raises."""
    from nxs.cam import port_state

    port = port_state.port_name(topology)
    rows = _table_rows(host, topology.i2c_bus)
    overrides = tuning_overrides(host, _hub_of(topology), topology.i2c_bus)
    digest = tuning_hash(host.stack(), rows, overrides)
    if sensor_id is None:
        sensor_id = _default_sensor_id(host, topology.i2c_bus)
    files = {}
    for compatible, override in sorted(overrides.items()):
        files[compatible] = fetch_override(override["url"], override["sha256"], log)
        log(f"tuning: vendor override for {compatible} ({override['sha256'][:16]})")
    log(f"{port}: making the capture stack's configuration for {len(rows)} booted modes; "
        f"about {TUNING_BUILD_MINUTES} minutes, the capture daemon restarts")
    installed = host.tuning_make(port, topology.i2c_bus, int(sensor_id), overrides=files)
    stored = tuning_store_path(host.tuning_badge(port), digest)
    stored.parent.mkdir(parents=True, exist_ok=True)
    stored.write_bytes(installed.read_bytes())
    return installed, _record_tuning(port, host, rows, digest, installed, overrides)


class TuningRefused(RuntimeError):
    """A tuning build refused before it started: the fact and what to do."""

    def __init__(self, fact: str, alternatives) -> None:
        self.fact = fact
        self.alternatives = list(alternatives)
        super().__init__(fact)

    def __str__(self) -> str:
        return "\n".join([self.fact] + [f"  - {a}" for a in self.alternatives])


def ensure_tuning(host, topology, log=print) -> Optional[str]:
    """The tuning derivation `on` needs: nothing when the installed object was
    made for the booted table under the port's badge; the stored object
    installed when the table's derivation was built under that badge
    before; a build otherwise. Returns one sentence for
    what was done, None when nothing was. A host whose capture stack takes no
    tuning (`Host.tuning_required`) is left alone. Raises RuntimeError or
    PermissionError when a build fails, TuningRefused when the host lacks
    what a build needs."""
    import yaml

    from nxs.cam import port_state

    if not getattr(host, "capture_contract", False) or not host.tuning_required():
        return None
    port = port_state.port_name(topology)
    rows = _table_rows(host, topology.i2c_bus)
    if not rows:
        return None
    overrides = tuning_overrides(host, _hub_of(topology), topology.i2c_bus)
    digest = tuning_hash(host.stack(), rows, overrides)
    badge = host.tuning_badge(port)
    record = tuning_record_path(port)
    if record.is_file() and host.tuning_state(port):
        try:
            made = yaml.safe_load(record.read_text()) or {}
        except (OSError, yaml.YAMLError):
            made = {}
        wanted = {c: o["sha256"] for c, o in overrides.items()}
        if made.get("rows") == rows and made.get("badge") == badge and (made.get("overrides") or {}) == wanted:
            return None
    # The capture stack refuses an object made under another badge: another
    # port's object for the same rows is never installed here.
    stored = tuning_store_path(badge, digest)
    if stored.is_file():
        installed = host.tuning_install(stored, port)
        host.restart_capture_daemon()
        _record_tuning(port, host, rows, digest, installed, overrides)
        return f"tuning: installed the stored configuration for {port}'s booted table ({digest})"
    missing = host.tuning_prerequisite_missing()
    if missing:
        raise TuningRefused(missing, [host.tuning_prerequisite_next()])
    try:
        installed, _record = build_tuning(host, topology, log=log)
    except NotImplementedError:
        return None
    return f"tuning: made and installed {installed} for {port}'s booted table ({digest})"


#: What a build takes for a table of eight modes, said before it starts.
TUNING_BUILD_MINUTES = 2


def preparing_marker(port: str) -> Path:
    """The file `on` keeps while it builds the port's configuration (nxsd's
    `on` after a reboot); `status`, `switch` and another `on` read
    `preparing the capture stack` from it."""
    from nxs.cam import port_state

    path = port_state.state_dir() / "tuning" / f"{port}.preparing"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def capture_stack_state(host, port: str, topology) -> str:
    """`ready` when the port's capture stack is configured for the booted
    table under the port's badge (or the host takes no configuration),
    `preparing` while an `on` builds it, `missing` when nothing is
    installed for it."""
    import yaml

    if (not getattr(host, "capture_contract", False) or not hasattr(host, "tuning_state")
            or not host.tuning_required()):
        return "ready"
    if preparing_marker(port).is_file():
        return "preparing"
    rows = _table_rows(host, topology.i2c_bus)
    if not rows or not host.tuning_state(port):
        return "missing" if rows else "ready"
    record = tuning_record_path(port)
    try:
        made = yaml.safe_load(record.read_text()) or {} if record.is_file() else {}
    except (OSError, yaml.YAMLError):
        made = {}
    return "ready" if made.get("rows") == rows and made.get("badge") == host.tuning_badge(port) else "missing"


def _cmd_modes(args) -> int:
    topology = _port(args)
    host = host_layer.current()
    modes = host.booted_modes(topology.i2c_bus)
    if args.json:
        print(json.dumps({"port": str(args.port), "bus": topology.i2c_bus,
                          "modes": modes}, indent=2))
        return 0
    if not modes:
        print(f"{args.port}: the booted device tree offers no capture modes "
              f"on {topology.i2c_bus} (no camera overlay, or not a Jetson)")
        return 1
    for m in modes:
        print(f"mode{m['index']:<3} {m['width']}x{m['height']} RAW{m['bit_depth']} "
              f"{m['lanes']}-lane vc{m['vc']} <= {m['max_fps']:g} fps  "
              f"{', '.join(m['pool'])}")
    return 0


def generate(hub, topology, port_name: str, lanes: int,
             sensors: Optional[List[str]], out_dir: Path) -> List[Dict[str, Any]]:
    """Generate (and compile when dtc is present) the port's overlay into
    ``out_dir``: one record, both virtual channels. A port that serves the
    sensor on its own bus carries that sensor alone (the node answers at
    its address) on virtual channel 0."""
    from nxs.host import capture_table as tables
    from nxs.host import jetson, jetson_overlay as gen

    host = host_layer.current()
    if not getattr(host, "capture_contract", False):
        # Generation is host-independent; only compile/install need the Jetson.
        host = jetson.JetsonHost()
    direct = bool(topology is not None and topology.is_direct)
    node_addr = None
    if direct:
        from nxs.cam.hubs import native_sensor_address
        if sensors is None:
            sensors = [hub.descriptor(link.sensor_compatible).name for link in topology.links]
        # The node answers where the link's sensor does: its declared
        # address when the wiring names one, else the descriptor's own. A
        # wiring that found no sensor names no node, and no capture mode.
        if topology.links:
            node_addr = native_sensor_address(hub, topology.links[0])
    layout = tables.table_layout(hub, topology, int(lanes))
    table = tables.capture_table(hub, sensors, **layout)
    if not table:
        from nxs.suite import PERSONALITY_DIR
        raise SystemExit(
            f"nxs host: no capture modes for {port_name}: hub {hub.name!r} "
            f"carries no sensor, no cam personality is installed under "
            f"{PERSONALITY_DIR}, and no unit of the port has served one — "
            f"install the assets (nxs-assets-<version>.tar.gz, nxs assets install) "
            f"or upload a personality to the link's unit")
    out_dir.mkdir(parents=True, exist_ok=True)
    dts = host.overlay(hub, port_name, lanes, sensors, direct=direct, node_addr=node_addr,
                       fps=layout["default_fps"], bit_depth=layout["bit_depth"])
    name = gen.dtbo_basename(port_name, lanes, direct)
    dts_path = out_dir / name.replace(".dtbo", ".dts")
    dts_path.write_text(dts)
    record: Dict[str, Any] = {"port": port_name, "lanes": lanes,
                              "dts": str(dts_path), "dtbo": None,
                              "modes": len(table), "direct": direct}
    if shutil.which("dtc"):
        record["dtbo"] = str(host.compile(dts, out_dir / name))
    return [record]


def boot_write_refusal(exc: PermissionError) -> str:
    """A /boot write sudo refused: its answer, then the next step."""
    return (f"writing /boot needs root, and sudo refused: {exc}\n"
            f"  - run it in a terminal, where sudo asks for the password")


def install(records: List[Dict[str, Any]], label: Optional[str], select: bool,
            keep_other_ports: bool = True, fdt: Optional[str] = None,
            declared: Optional[Collection[str]] = None) -> List[str]:
    """Install compiled overlays under the host's boot entry (`Host.install_records`),
    keeping the overlays of the `declared` ports alone (None: of every port the
    entry names); returns the reports."""
    host = host_layer.current()
    try:
        return host.install_records(records, label, select, keep_other_ports, fdt, declared)
    except NotImplementedError:
        raise SystemExit("nxs host: overlays install on a supported Jetson "
                         f"carrier only (this host: {host.describe()}); the "
                         "generated .dtbo files are yours to install by hand")
    except RuntimeError as exc:
        raise SystemExit(f"nxs host: {exc}")
    except PermissionError as exc:
        raise SystemExit(f"nxs host: {boot_write_refusal(exc)}")


class BootTableRefused(RuntimeError):
    """A port's boot table the host did not install: `gap` holds what the
    booted table lacks, the message why nothing was installed (the fact,
    then its alternatives). A reboot would boot the same table."""

    def __init__(self, gap: List[str], refusal: str) -> None:
        self.gap = list(gap)
        super().__init__(refusal)


def reboot_rule(host, hub, topology, links, modes: Dict[str, str],
                install_overlays: bool = True,
                vcs: Optional[Dict[str, int]] = None,
                declared: Optional[Collection[str]] = None,
                fdt: Optional[str] = None,
                report: Optional[Callable[[str], None]] = None) -> Optional[str]:
    """The `on` gate: the booted tree must carry the port's lane count, every
    selected link's mode and, given ``vcs`` (link -> virtual channel), a
    capture node for it at the address the host reaches the sensor at; given
    ``declared`` (the ports the manifest declares), the boot entry names no
    other port's overlays. A gap regenerates the port's overlay at its lane
    count, installs it under a boot entry naming ``fdt`` when given, and
    returns a sentence. None when whole. The install's lines (which files it
    wrote, which entries it dropped) go to ``report`` when given, else into
    the sentence; an overlay that is not generated or not installed raises
    BootTableRefused."""
    from nxs.cam import port_state
    from nxs.cam.hubs import native_sensor_address
    from nxs.host import capture_table as tables

    links = [l for l in links if getattr(l, "has_camera", True)]
    if not links:
        return None
    booted = host.booted_modes(topology.i2c_bus)
    if not booted and not getattr(host, "capture_contract", False):
        return None
    missing = []
    ids = {}
    if vcs:
        try:
            ids = host.capture_ids(topology.i2c_bus) or {}
        except Exception:
            ids = {}
    wanted = {}
    if vcs:
        try:
            if topology.is_direct:
                # The node on a port without a hub is the sensor's own
                # address: the kernel's controls land nowhere else.
                wanted = {int(vcs[link.name]): native_sensor_address(hub, link)
                          for link in links if link.name in vcs}
            else:
                wanted = host.node_aliases(port_state.port_name(topology)) or {}
        except Exception:
            wanted = {}
    for link in links:
        vc = (vcs or {}).get(link.name)
        if ids and vc is not None and int(vc) not in ids:
            missing.append(f"a capture node for virtual channel {int(vc)} (link {link.name})")
        elif vc is not None and int(vc) in wanted:
            booted_addr = topology.node_addr(int(vc))
            if booted_addr is not None and booted_addr != int(wanted[int(vc)]):
                place = "the sensor's address" if topology.is_direct else "its alias"
                missing.append(f"virtual channel {int(vc)}'s node at {place} "
                               f"{int(wanted[int(vc)]):#04x} (booted {booted_addr:#04x}, "
                               f"link {link.name})")
    for link in links:
        sen = hub.descriptor(link.sensor_compatible)
        mode = modes.get(link.name)
        if not mode:
            continue
        geo = sen.modes[mode]["geometry"]
        # A hub's lanes and a sensor's own configure the receiver differently:
        # the booted mode has to be the port's kind.
        index = host.mode_index(topology.i2c_bus, sen.compatible, geo["width"],
                                geo["height"], geo["bit_depth"],
                                direct=bool(topology.is_direct))
        # The port's table at its lanes and rate: a direct port's carries
        # its one sensor's rows alone.
        wanted_index = tables.port_index(hub, topology, sen, mode)
        if index is None:
            missing.append(f"{sen.compatible} {geo['width']}x{geo['height']} "
                           f"RAW{geo['bit_depth']}"
                           + (" on the sensor's own lanes" if topology.is_direct else "")
                           + f" (link {link.name})")
        elif wanted_index is not None and int(index) != int(wanted_index):
            # The capture stack names the mode by its table index: a row
            # that moved takes another sensor's controls and tuning.
            missing.append(f"{sen.compatible} {geo['width']}x{geo['height']} "
                           f"RAW{geo['bit_depth']} at mode index {int(wanted_index)} "
                           f"(booted {int(index)}, link {link.name})")
        elif wanted_index is not None:
            # The row's exposure ceiling is the driver's law for the frame:
            # a booted row with another one stretches the frame.
            row = tables.port_table(hub, topology)[int(wanted_index)]
            wanted_ceiling = row.exposure_ceiling_us()
            booted_row = next((m for m in booted if int(m.get("index", -1)) == int(index)
                               and sen.compatible in (m.get("pool") or [])), None)
            booted_ceiling = int((booted_row or {}).get("max_exp_us") or 0)
            if booted_ceiling and booted_ceiling != wanted_ceiling:
                missing.append(f"{sen.compatible} {geo['width']}x{geo['height']} "
                               f"RAW{geo['bit_depth']} with an exposure ceiling of "
                               f"{wanted_ceiling} us (booted {booted_ceiling}, link {link.name})")
    port = port_state.port_name(topology)
    # The port's count: the declared one, else the booted, else the connector's
    # (`port_topology`). A tree booted at another count lacks the port's table.
    lanes = int(topology.csi_lanes)
    booted_lanes = host.booted_lanes(topology.i2c_bus)
    if booted_lanes is not None and int(booted_lanes) != lanes:
        missing.append(f"csi_lanes: {lanes} (booted {int(booted_lanes)})")
    records: List[Dict[str, Any]] = []
    stale = False
    from nxs import experimental
    if not missing and not experimental.enabled():
        # A row's other facts (its default exposure, the receiver's SerDes
        # clock) live in the overlay alone: an installed overlay that
        # differs from the generated one is stale. The flag's pooled
        # table is never the booted one; it has no verdict here.
        try:
            records = generate(hub, topology, port, int(lanes), None,
                               port_state.state_dir() / "host-overlays")
        except SystemExit:
            records = []
        for record in records:
            compiled = record.get("dtbo")
            installed = host.installed_overlay(Path(compiled).name) if compiled else None
            stale = installed is not None and installed != Path(compiled).read_bytes()
            if stale:
                break
    # Another port's overlays boot its capture nodes with no camera behind
    # them, and the capture stack numbers its sensors across every node.
    stray = [] if declared is None else sorted(set(host.boot_entry_ports()) - {*declared, port})
    if not missing and not stale and not stray:
        return None
    # The booted table's rows prove the modes alone. The rest of a generated
    # overlay (the receiver's clock, a default exposure) is proven booted when
    # the generated file is the one the host recorded as this boot's.
    proven = all(host.booted_overlay_digest(Path(record["dtbo"]).name)
                 == hashlib.sha256(Path(record["dtbo"]).read_bytes()).hexdigest()
                 for record in records) if stale and not missing and not stray else False
    if proven:
        # A declaration put back after a reboot it asked for: the next boot
        # needs the file that fits, and this boot needs no reboot.
        say = report or (lambda line: print(f"{port}: {line}"))
        for record in records:
            compiled = Path(record["dtbo"])
            if not install_overlays:
                say(f"would install {compiled.name} (the booted table fits the declaration)")
                continue
            try:
                where = host.refresh_overlay(compiled)
            except NotImplementedError:
                raise BootTableRefused([], "overlays install on a supported Jetson carrier only "
                                           f"(this host: {host.describe()})") from None
            except PermissionError as exc:
                raise BootTableRefused([], boot_write_refusal(exc)) from None
            say(f"installed {where} (the booted table fits the declaration)")
        return None
    # One fact per line: the gap, the table, what was written, the next step.
    lines = ([f"the booted overlay on {port} lacks " + "; ".join(missing)] if missing
             else [f"the installed overlay on {port} is stale: its rows' facts moved"] if stale
             else [])
    if booted and lines:
        lines.append(f"its table serves {', '.join(sorted({c for m in booted for c in m['pool']}))}")
    lines += [f"the boot entry names the overlays of {other}, a port the manifest does not declare"
              for other in stray]
    if not install_overlays:
        lines.append("nxs switch installs the port's overlay, then reboot")
        return "\n".join(lines)
    try:
        records = records or generate(hub, topology, port, int(lanes), None,
                                      port_state.state_dir() / "host-overlays")
        # The entry the tool writes is the one the next boot takes: a
        # fresh host has no other that carries the port.
        reports = install(records, None, select=True, fdt=fdt, declared=declared)
    except SystemExit as exc:
        raise BootTableRefused(lines, str(exc).removeprefix("nxs host: ")) from None
    if report is None:
        lines.extend(reports)
    else:
        for line in reports:
            report(line)
    lines.append("then run this command again")
    return "\n".join(lines)
