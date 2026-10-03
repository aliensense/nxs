# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""`nxs status`: validate the manifest against the hardware's descriptors
without touching a register: schema shape first, then camera declarations
against the pack's laws and unit sensor configs against the driver descriptors."""

import re
import os

from nxs.finding import Finding, as_data, parse_refusal


def sensor_allowed_keys(drv_cls, compiled) -> set:
    """Config keys a driver accepts: declared params, `sample_rate`, `bus`, and
    `trigger` for from_config loops."""
    allowed = {p.name for p in compiled.params}
    allowed.add("sample_rate")
    for attr_name in dir(drv_cls):
        fn = getattr(drv_cls, attr_name)
        if (callable(fn) and getattr(fn, "_measure_loop", False)
                and getattr(fn, "_trigger", "") == "from_config"):
            allowed.add("trigger")
            break
    allowed.add("bus")
    return allowed


def shape_findings(path: str) -> list:
    """The manifest's schema findings (shape only); read failures and malformed
    YAML are findings too."""
    import yaml

    from nxs import schemas

    try:
        with open(path, encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
    except OSError as e:
        return [Finding("", f"{path}: {e.strerror or e}")]
    except (yaml.YAMLError, UnicodeDecodeError) as e:
        return [Finding("", f"{path}: not valid YAML ({e})")]
    if raw is None:
        return [Finding("", f"{path}: empty manifest")]
    return schemas.findings(raw, schemas.SUITE, where=path)


def check_config(cfg, booted: bool = True) -> list:
    """Findings for a loaded SuiteConfig; empty means IN TUNE. With `booted`
    False the findings that hold the declaration against the booted tree
    (its lane count, its modes, its capture nodes) are left out: `nxs
    switch` judges a declaration before it writes that tree."""
    findings = []

    for name in sorted(cfg.ports):
        port = cfg.ports[name]
        where = f"ports.{name}"
        if (booted and (port.hub_compatible is not None or port.links)
                and port.csi_lanes_declared):
            # The overlay fixed the capture side's lane count at boot; a port
            # programmed for another count gives no video.
            try:
                from nxs import host as host_layer
                mismatch = host_layer.current().lane_mismatch(port.bus, port.csi_lanes)
            except Exception as exc:
                # A host without a contract answers None; a contract that
                # cannot be read is a finding, never an approval.
                mismatch = f"the booted lane contract could not be read: {exc}"
            if mismatch:
                findings.append(Finding(f"{where}.csi_lanes", mismatch))
        pods = [l for l in port.links if l.camera is None and l.unit is not None
                and port.hub_compatible is not None]
        for link in port.links:
            if link.camera is None and link not in pods:
                findings.append(Finding(
                    f"ports.{name}.links.{link.name}", "no sensor declared",
                    [f"ports.{name}.links.{link.name}.camera: <the sensor compatible>"]))
        # One hub brings a pod-only link and a camera link up only as a
        # pair, and a pair is two camera programs.
        if pods and len(pods) < len(port.links):
            for link in pods:
                findings.append(Finding(
                    f"ports.{name}.links.{link.name}",
                    f"link {link.name} carries a pod and no camera beside a camera "
                    f"link, and one hub brings them up together only as a pair",
                    [f"ports.{name}.links.{link.name}.camera: <the head on link "
                     f"{link.name}>"]))
            continue
        stale = _stale_hub_findings(name, port)
        if stale:
            findings.extend(stale)
            continue
        # Camera behavior is declared at the port or per link; a port
        # declaring none has nothing to judge.
        declared = (port.camera_mode is not None or port.camera_fps is not None
                    or port.camera_exposure_us is not None or port.camera_gain_db is not None
                    or port.sync_source != "free_run"
                    or any(l.camera_mode for l in port.links))
        if (port.hub_compatible is None and not port.links) or not declared:
            continue
        try:
            from nxs.cam import packs as cam_packs
            from nxs.cam import topology as cam_topo
            from nxs.cam.contracts import InfeasibleConfig
            from nxs.cam.descriptors import resolve_mode
            from nxs.cam.run import _resolve_modes
        except Exception as exc:
            findings.append(Finding(where, f"camera layer unavailable ({exc})"))
            continue
        ports = cam_topo._ports_from_suite()
        topology = None
        if ports:
            topology = next((t for t in ports[0].values()
                             if t.carrier.endswith(f"/{name}")), None)
        if topology is None:
            findings.append(Finding(where, "no camera port carries this declaration"))
            continue
        try:
            pack = cam_packs.pack_for(topology)
        except Exception as exc:
            findings.append(Finding.of(where, exc))
            continue
        cams = topology.camera_links
        if not cams:
            findings.append(Finding(f"{where}.links", "no camera link, and a camera "
                                                      "declaration needs one to bring up"))
            continue
        # The port's mode token must name a mode of every link's sensor
        # (a mixed hub declares per-link modes in links.<name>.camera).
        declared_modes, refused = {}, False
        for link in cams:
            token = link.mode or port.camera_mode
            if token is None:
                continue
            try:
                declared_modes[link.name] = resolve_mode(
                    pack.descriptor(link.sensor_compatible), token)
            except InfeasibleConfig as exc:
                refused = True
                findings.append(Finding(
                    f"{where}.camera.mode",
                    f"link {link.name} ({link.sensor_compatible}): {exc.reason}",
                    exc.alternatives))
        if refused:
            continue
        # A link with no declared mode runs the highest mode the port's
        # laws admit (`on` resolves it the same way); that mode is judged.
        try:
            resolved = _resolve_modes(pack.flows(), pack, list(cams), declared_modes, topology)
        except InfeasibleConfig as exc:
            findings.append(Finding.of(f"{where}.camera.mode", exc))
            continue
        # The capture stack takes a node's modes in its first mode's bit depth
        # and Bayer phase, so the port's boot table carries one of each.
        from nxs.host import capture_table as tables
        left_out = {m.key: m for m in tables.excluded_rows(
            pack, tables.port_sensors(pack, topology), **tables.table_layout(pack, topology))}
        if left_out:
            depth, phase = tables.table_format(tables.port_table(pack, topology))
            for link in cams:
                sen = pack.descriptor(link.sensor_compatible)
                geo = sen.modes[resolved[link.name]]["geometry"]
                row = left_out.get((sen.compatible, int(geo["width"]), int(geo["height"]),
                                    int(geo["bit_depth"])))
                if row is None:
                    continue
                if row.bit_depth != depth:
                    fact = (f"{resolved[link.name]} (RAW{row.bit_depth}) cannot share the port's "
                            f"boot table with its RAW{depth} rows: the capture stack captures "
                            f"every mode of a node in one pixel format")
                    alternative = f"declare a RAW{depth} mode on the link"
                else:
                    fact = (f"{resolved[link.name]} ({row.pixel_phase}) cannot share the port's "
                            f"boot table with its {phase} rows: the capture stack demosaics "
                            f"every mode of a node by one Bayer phase")
                    alternative = f"declare a {phase} mode on the link"
                findings.append(Finding(f"{where}.links.{link.name}",
                                        f"link {link.name}: {fact}", [alternative]))
        try:
            from nxs import host as host_layer
            host = host_layer.current()
            if booted and host.booted_modes(topology.i2c_bus):
                for link in cams:
                    sen = pack.descriptor(link.sensor_compatible)
                    geo = sen.modes[resolved[link.name]]["geometry"]
                    if host.mode_index(topology.i2c_bus, sen.compatible, geo["width"],
                                       geo["height"], geo["bit_depth"],
                                       direct=bool(topology.is_direct)) is None:
                        findings.append(Finding(
                            f"{where}.links.{link.name}",
                            f"the booted overlay lacks {sen.compatible} "
                            f"{geo['width']}x{geo['height']} RAW{geo['bit_depth']}",
                            ["nxs switch, then reboot"]))
                # Two links ride two virtual channels; each needs its node.
                ids = host.capture_ids(topology.i2c_bus) if len(cams) > 1 else {}
                for link in cams:
                    if ids and link.csi_vc not in ids:
                        findings.append(Finding(
                            f"{where}.links.{link.name}",
                            f"the booted overlay has no capture node for virtual "
                            f"channel {link.csi_vc}",
                            ["nxs switch, then reboot"]))
        except Exception as exc:
            findings.append(Finding(where, f"the booted capture table could not be "
                                           f"checked: {exc}"))
        if port.sync_source == "fsync":
            # A synced pair runs the generator's rate, `synced_fps` as in `on`;
            # the trigger laws judge it, not the free-run ceiling.
            declared = {l.name: l.fps for l in cams if l.fps is not None}
            if (port.camera_fps is None and port.sync_fps is None
                    and len({round(r, 6) for r in declared.values()}) > 1):
                rates = ", ".join(f"{n} {r:g}" for n, r in sorted(declared.items()))
                findings.append(Finding(
                    f"{where}.links", f"under frame sync the pair runs one rate ({rates})",
                    ["declare one fps"]))
            else:
                label = ("camera.mode" if port.camera_fps is not None
                         else "sync.fps" if port.sync_fps is not None
                         else f"links.{min(declared)}.camera.fps" if declared else "sync")
                try:
                    pack.flows().build_fsync(pack, topology, topology.synced_fps, modes=resolved)
                except InfeasibleConfig as exc:
                    findings.append(Finding.of(f"{where}.{label}", exc))
                except Exception as exc:  # a pack without the trigger overlay
                    findings.append(Finding.of(f"{where}.camera.sync", exc))
        else:
            # Every link's declared rate (its own, else the port's) is
            # judged by its resolved mode: the lawful range on the port,
            # then the pack's timing law at that rate's VMAX (the same laws
            # `on` composes with); free-running links keep their own frames.
            from nxs.cam import timing as cam_timing
            for link in cams:
                fps = link.fps if link.fps is not None else port.camera_fps
                if fps is None:
                    continue
                link_mode = resolved[link.name]
                key = ("camera.mode" if link.fps is None
                       else f"links.{link.name}.camera.fps")
                at = f" (link {link.name}, {link_mode})" if len(cams) > 1 else ""
                refusal = cam_timing.free_run_refusal(
                    pack, link, link_mode, float(fps), topology=topology)
                if refusal:
                    fact, alternatives = parse_refusal(refusal)
                    findings.append(Finding(f"{where}.{key}", f"{fact}{at}", alternatives))
        synced = port.sync_source == "fsync"
        if port.camera_exposure_us is not None and (synced or port.camera_gain_db is None):
            # The finding names what sets the exposure: the pulse on a synced
            # pair, the capture stack's loop unless a declared gain locks it.
            from nxs.cam.verbs.sync import declared_exposure_fact
            fact = declared_exposure_fact(topology, synced)
            findings.append(Finding(f"{where}.camera.exposure_us", fact, ["drop the key"]))
        if port.camera_gain_db is not None:
            # The lock `on` writes, judged by the pack's laws.
            lock = getattr(pack.flows(), "build_gain_lock", None)
            if lock is None:
                findings.append(Finding(f"{where}.camera.gain_db",
                                        f"pack {pack.name} writes no gain lock", ["drop the key"]))
            else:
                try:
                    lock(pack, topology, port.camera_gain_db, synced)
                except InfeasibleConfig as exc:
                    findings.append(Finding.of(f"{where}.camera.gain_db", exc))

    for unit in cfg.units:
        for i, spec in enumerate(unit.sensors or []):
            where = f"units.{unit.name}.sensors[{i}]"
            try:
                from nxs.suite.reconcile import load_unit_driver
                drv_cls = load_unit_driver(spec.driver)
            except Exception as exc:
                findings.append(Finding.of(where, exc))
                continue
            try:
                compiled = drv_cls().compile(dict(spec.config))
            except Exception as exc:
                findings.append(Finding.of(where, exc))
                continue
            unknown = set(spec.config) - sensor_allowed_keys(drv_cls,
                                                             compiled)
            if unknown:
                valid = sorted(sensor_allowed_keys(drv_cls, compiled))
                findings.append(_KeyFinding(
                    where, f"unknown config key(s) {', '.join(sorted(unknown))} "
                           f"(valid: {', '.join(valid)})", valid))
    return findings


def _stale_hub_findings(name: str, port) -> list:
    """The declaration names a hub and a camera its pack does not serve."""
    if port.hub_compatible is None or not port.hub_source or not port.links:
        return []
    try:
        from nxs.cam import packs as cam_packs
        from nxs.cam import topology as cam_topo
        from nxs.suite.schema_ports import WIRING_ALTERNATIVE
        pack = cam_packs.pack_for(cam_topo.port_topology(port))
        served = sorted(pack.descriptor(chip).compatible for chip in pack.sensors())
        rides = getattr(pack.flows(), "LINK_VC", None) or {}
    except Exception:
        return []        # a pack that does not load is the other checks' finding
    findings = [Finding(f"ports.{name}.links.{link.name}",
                        f"{port.hub_source} names hub {port.hub_compatible} and its pack "
                        f"serves no {link.camera}",
                        [WIRING_ALTERNATIVE]
                        + [f"ports.{name}.links.{link.name}.camera: {c}" for c in served])
                for link in port.links if link.camera and link.camera not in served]
    # The pack's programs fix the channel each link rides, and with it the
    # capture node and the host alias the head answers at; a wiring that
    # says otherwise steers the wrong head. A link that declares no channel
    # takes the pack's.
    findings += [Finding(f"ports.{name}.links.{link.name}.csi_vc",
                         f"link {link.name} rides virtual channel {rides[link.name]} "
                         f"behind {port.hub_compatible}, and the wiring declares "
                         f"csi_vc {link.csi_vc}",
                         [WIRING_ALTERNATIVE])
                 for link in port.links
                 if link.name in rides and link.csi_vc is not None
                 and int(link.csi_vc) != int(rides[link.name])]
    return findings


class _KeyFinding(Finding):
    """A finding whose sentence already lists the valid keys: the text stays
    one line, the data carries them."""

    def __new__(cls, where: str, fact: str, valid=()):
        self = Finding.__new__(cls, where, fact)
        self._alternatives = tuple(str(v) for v in valid)
        return self


def _manifest_finding(error) -> Finding:
    """A loader refusal as a finding: its fact names the YAML path first."""
    where, sep, fact = error.fact.partition(": ")
    if not sep:
        where, fact = "", error.fact
    return Finding(where, fact, error.alternatives)


def node_findings(findings: list) -> tuple:
    """Findings grouped by the node they name (`ports.<name>` or
    `units.<name>`), and the rest."""
    by_node: dict = {}
    rest = []
    for finding in findings:
        m = re.match(r"^(ports|units)\.([^.:\[\s]+)", getattr(finding, "where", "") or finding)
        if m:
            by_node.setdefault(f"{m.group(1)}.{m.group(2)}", []).append(finding)
        else:
            rest.append(finding)
    return by_node, rest


def check_manifest(path: str) -> list:
    """Every finding for the manifest at `path`: shape first (the schema
    and the strict parser), then the laws. Empty means IN TUNE."""
    from nxs.suite.schema import ManifestError, load_suite_config

    if not os.path.exists(path):
        return [Finding("", f"no manifest at {path}")]
    findings = shape_findings(path)
    if findings:
        return findings
    try:
        cfg = load_suite_config(path)
    except ManifestError as e:
        return [_manifest_finding(e)]
    return check_config(cfg)


def check_payload(path: str) -> dict:
    """The `nxs status --json` document (surface `check`)."""
    from nxs.schemas import CONTRACT

    findings = check_manifest(path)
    return {"contract": CONTRACT, "manifest": path,
            "in_tune": not findings, "findings": as_data(findings)}
