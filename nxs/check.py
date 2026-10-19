# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""`nxs status`: validate the declaration against the hardware's facts
without touching a register: schema shape first, then the graph's laws
(the loader's), then every declared value against what its block offers at
that point of the graph (the options hook, `nxs.suite.views`), then each
camera port against its hub's laws (the lanes of each output, the frame
sync, the modes and rates, the declared gain by the composer's law) and
each click against its personality. A finding names the node it is about
(`nodes.<name>...`)."""

import re
import os

from nxs._generated_constants import NxsMap
from nxs.finding import Finding, as_data, parse_refusal


class _Places:
    """Where a port's value is declared, as a node path: the hub's rows, the
    host's row for the connector's lanes, a link's camera node. A port the
    loader built from no graph (a test's) names its port-view place."""

    def __init__(self, cfg):
        self._graph = getattr(cfg, "graph", None)
        self._host = self._graph.host() if self._graph is not None else None

    def _source(self, port: str):
        if self._graph is None or self._host is None:
            return None
        found = self._graph.source(self._host.name, port)
        return found[0] if found is not None else None

    def port(self, name: str) -> str:
        source = self._source(name)
        return f"nodes.{source}" if source else f"ports.{name}"

    def hub_param(self, port, key: str) -> str:
        """A port-wide row: the hub's, or on a connector its camera's."""
        source = port.hub_node or next((l.camera_node for l in port.links if l.camera_node), None)
        return f"nodes.{source}.parameters.{key}" if source else f"ports.{port.name}.{key}"

    def lanes(self, name: str) -> str:
        if self._host is None:
            return f"ports.{name}.csi_lanes"
        return f"nodes.{self._host.name}.parameters.{name}_lanes"

    def link(self, port, letter: str) -> str:
        link = next((l for l in port.links if l.name == letter), None)
        node = (link.camera_node or (link.unit.name if link.unit else None)) if link else None
        return f"nodes.{node}" if node else f"ports.{port.name}.links.{letter}"

    def link_param(self, port, letter: str, key: str) -> str:
        link = next((l for l in port.links if l.name == letter), None)
        if link is not None and link.camera_node:
            return f"nodes.{link.camera_node}.parameters.{key}"
        return f"ports.{port.name}.links.{letter}.camera.{key}"

    def click(self, unit: str, index: int) -> str:
        clicks = self._graph.sources(unit, "click") if self._graph is not None else []
        if index < len(clicks):
            return f"nodes.{clicks[index][0]}.parameters"
        return f"units.{unit}.sensors[{index}]"

    def node_of(self, where: str):
        """The port or the unit a node path's finding belongs under, as
        `ports.<name>` or `units.<name>`; None for a node of neither."""
        if self._graph is None:
            return None
        found = re.match(r"^nodes\.([^.\s:]+)", where)
        if found is None:
            return None
        return _belongs(self._graph, found.group(1))


def _belongs(graph, name: str):
    """`ports.<connector>` for a node on a camera chain into one of the
    host's connectors, `units.<name>` for a unit or the click on it."""
    from nxs.suite.schema_nodes import _is_connector

    node = graph.nodes.get(name)
    if node is None:
        return None
    if node.kind == "click":
        for target in graph.targets(name, "out"):
            return f"units.{target[0]}"
        return None
    if node.kind == "unit":
        return f"units.{name}"
    host = graph.host()
    if node.kind == "host" or host is None:
        return None
    seen, frontier = set(), [name]
    while frontier:
        current = frontier.pop()
        if current in seen:
            continue
        seen.add(current)
        for edge in graph.edges:
            if edge.source[0] != current:
                continue
            if edge.target[0] == host.name and _is_connector(host, edge.target[1]):
                return f"ports.{edge.target[1]}"
            frontier.append(edge.target[0])
    return None


def _placed(alternatives, key: str, place: str) -> list:
    """Alternatives a law names by its own key (`fps 146`, `fps: 146`), at
    the node path that declares the value."""
    return [f"{place}: {alt[len(key) + 1:].strip()}"
            if alt.startswith((f"{key}:", f"{key} ")) else alt for alt in alternatives]


def shape_findings(path: str) -> list:
    """The declaration's schema findings (shape only); read failures,
    malformed YAML and a declaration of an earlier language are findings
    too."""
    import yaml

    from nxs import schemas
    from nxs.suite.convert import is_api2
    from nxs.suite.nodes import is_1_0_form

    try:
        with open(path, encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
    except OSError as e:
        return [Finding("", f"{path}: {e.strerror or e}")]
    except (yaml.YAMLError, UnicodeDecodeError) as e:
        return [Finding("", f"{path}: not valid YAML ({e})")]
    if raw is None:
        return [Finding("", f"{path}: empty manifest")]
    if is_1_0_form(raw) or is_api2(raw):
        from nxs.suite.nodes import parse_graph
        from nxs.suite.schema import ManifestError
        try:
            parse_graph(raw, path)
        except ManifestError as e:
            return [_manifest_finding(e)]
    return schemas.findings(raw, schemas.SUITE, where=path)


def check_config(cfg, booted: bool = True) -> list:
    """Findings for a loaded SuiteConfig; empty means IN TUNE. With `booted`
    False the findings that hold the declaration against the booted tree
    (its lane count, its modes, its capture nodes) are left out: `nxs
    switch` judges a declaration before it writes that tree."""
    from nxs.suite import views

    findings = []
    places = _Places(cfg)
    wired = _wired_to_nothing(cfg)
    # A value the block's options refuse; a law below that names the same
    # place says why, and its finding stands for both.
    offered = views.findings(cfg)

    for name in sorted(cfg.ports):
        port = cfg.ports[name]
        where = places.port(name)
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
                findings.append(Finding(places.lanes(name), mismatch))
        pods = [l for l in port.links if l.camera is None and l.unit is not None
                and port.hub_compatible is not None]
        # One hub brings a pod-only link and a camera link up only as a
        # pair, and a pair is two camera programs.
        if pods and len(pods) < len(port.links):
            for link in pods:
                findings.append(Finding(
                    places.link(port, link.name),
                    f"link {link.name} carries a pod and no camera beside a camera "
                    f"link, and one hub brings them up together only as a pair",
                    [f"connections.<the head on link {link.name}>.out: "
                     f"[{link.unit.name}.head]"]))
            continue
        # Camera behavior is declared on the hub or on each camera node; a
        # port declaring none has nothing to judge.
        declared = (port.camera_exposure_us is not None or port.camera_gain_db is not None
                    or port.sync_source != "free_run" or bool(port.outputs)
                    or any(l.camera_mode or l.camera_fps is not None for l in port.links))
        if (port.hub_compatible is None and not port.links) or not declared:
            continue
        try:
            from nxs.cam import hubs
            from nxs.cam import topology as cam_topo
            from nxs.cam.contracts import InfeasibleConfig
            from nxs.cam.descriptors import resolve_mode
            from nxs.cam.run import _resolve_modes
        except Exception as exc:
            findings.append(Finding(where, f"camera layer unavailable ({exc})"))
            continue
        try:
            ports = cam_topo._ports_from_suite()
        except cam_topo.TopologyError as exc:
            # A declared port no booted bus answers for: the host's refusal, once.
            refusal = Finding.parse("", str(exc))
            if refusal not in findings:
                findings.append(refusal)
            continue
        topology = None
        if ports:
            topology = next((t for t in ports[0].values()
                             if t.carrier.endswith(f"/{name}")), None)
        if topology is None:
            findings.append(Finding(where, "no camera port carries this declaration"))
            continue
        try:
            hub = hubs.for_topology(topology)
        except Exception as exc:
            findings.append(Finding.of(where, exc))
            continue
        cams = topology.camera_links
        if not cams:
            findings.append(Finding(where, "no camera link, and a camera declaration needs one "
                                           "to bring up",
                                    [f"connections.<camera>.out: [{port.hub_node}.links.A]"]
                                    if port.hub_node else []))
            continue
        # Each camera's mode token must name a mode of its sensor.
        declared_modes, refused = {}, False
        for link in cams:
            token = link.mode
            if token is None:
                continue
            try:
                declared_modes[link.name] = resolve_mode(
                    hub.descriptor(link.sensor_compatible), token)
            except InfeasibleConfig as exc:
                refused = True
                findings.append(Finding(
                    places.link_param(port, link.name, "mode"),
                    f"link {link.name} ({link.sensor_compatible}): {exc.reason}",
                    exc.alternatives))
        if refused:
            continue
        # A link with no declared mode runs the highest mode the port's
        # laws admit (`on` resolves it the same way); that mode is judged.
        try:
            resolved = _resolve_modes(hub.flows(), hub, list(cams), declared_modes, topology)
        except InfeasibleConfig as exc:
            findings.append(Finding.of(places.link_param(port, cams[0].name, "mode"), exc))
            continue
        lanes = _lane_findings(hub, topology, cams, resolved, port, places)
        if lanes:
            findings.extend(lanes)
            continue
        # The capture stack takes a node's modes in its first mode's bit depth
        # and Bayer phase, so the port's boot table carries one of each.
        from nxs.host import capture_table as tables
        left_out = {m.key: m for m in tables.excluded_rows(
            hub, tables.port_sensors(hub, topology), **tables.table_layout(hub, topology))}
        if left_out:
            depth, phase = tables.table_format(tables.port_table(hub, topology))
            for link in cams:
                sen = hub.descriptor(link.sensor_compatible)
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
                findings.append(Finding(places.link(port, link.name),
                                        f"link {link.name}: {fact}", [alternative]))
        try:
            from nxs import host as host_layer
            host = host_layer.current()
            if booted and host.booted_modes(topology.i2c_bus):
                for link in cams:
                    sen = hub.descriptor(link.sensor_compatible)
                    geo = sen.modes[resolved[link.name]]["geometry"]
                    if host.mode_index(topology.i2c_bus, sen.compatible, geo["width"],
                                       geo["height"], geo["bit_depth"],
                                       direct=bool(topology.is_direct)) is None:
                        findings.append(Finding(
                            places.link(port, link.name),
                            f"the booted overlay lacks {sen.compatible} "
                            f"{geo['width']}x{geo['height']} RAW{geo['bit_depth']}",
                            ["nxs switch, then reboot"]))
                # Two links ride two virtual channels; each needs its node.
                ids = host.capture_ids(topology.i2c_bus) if len(cams) > 1 else {}
                for link in cams:
                    if ids and link.csi_vc not in ids:
                        findings.append(Finding(
                            places.link(port, link.name),
                            f"the booted overlay has no capture node for virtual "
                            f"channel {link.csi_vc}",
                            ["nxs switch, then reboot"]))
        except Exception as exc:
            findings.append(Finding(where, f"the booted capture table could not be "
                                           f"checked: {exc}"))
        sync_place = places.hub_param(port, "sync_source")
        rate_place = places.hub_param(port, "sync_fps")
        if port.sync_source == "fsync":
            # A synced pair runs the generator's rate, `synced_fps` as in `on`;
            # the trigger laws judge it, not the free-run ceiling.
            declared = {l.name: l.fps for l in cams if l.fps is not None}
            if port.sync_fps is None and len({round(r, 6) for r in declared.values()}) > 1:
                rates = ", ".join(f"{n} {r:g}" for n, r in sorted(declared.items()))
                findings.append(Finding(
                    sync_place, f"under frame sync the pair runs one rate ({rates})",
                    [f"{rate_place}: <one rate>"]))
            else:
                label = (rate_place if port.sync_fps is not None
                         else places.link_param(port, min(declared), "fps") if declared
                         else sync_place)
                try:
                    hub.flows().build_fsync(hub, topology, topology.synced_fps, modes=resolved)
                except InfeasibleConfig as exc:
                    rated = any(alt.startswith("fps") for alt in exc.alternatives)
                    findings.append(Finding(label if rated else sync_place, exc.reason,
                                            _placed(exc.alternatives, "fps", rate_place)))
                except Exception as exc:  # a hub without the trigger overlay
                    findings.append(Finding.of(sync_place, exc))
        else:
            # Each declared rate is judged by its link's resolved mode: the port's lawful range, then
            # the hub's timing law at its VMAX, as `on` composes; a free-running link keeps its frames.
            from nxs.cam import timing as cam_timing
            for link in cams:
                fps = link.fps
                if fps is None:
                    continue
                link_mode = resolved[link.name]
                at = f" (link {link.name}, {link_mode})" if len(cams) > 1 else ""
                refusal = cam_timing.free_run_refusal(
                    hub, link, link_mode, float(fps), topology=topology)
                if refusal:
                    fact, alternatives = parse_refusal(refusal)
                    findings.append(Finding(places.link_param(port, link.name, "fps"),
                                            f"{fact}{at}", alternatives))
        synced = port.sync_source == "fsync"
        if port.camera_exposure_us is not None and (synced or port.camera_gain_db is None):
            # The finding names what sets the exposure: the pulse on a synced
            # pair, the capture stack's loop unless a declared gain locks it.
            from nxs.cam.verbs.sync import declared_exposure_fact
            fact = declared_exposure_fact(topology, synced)
            findings.append(Finding(places.hub_param(port, "exposure"), fact,
                                    ["drop the key"]))
        if port.camera_gain_db is not None:
            place = places.hub_param(port, "gain")
            finding = (_gain_shape_finding(hub, topology, synced, place)
                       or _gain_finding(hub, topology, cams, resolved, port.camera_gain_db, place))
            if finding is not None:
                findings.append(finding)

    for unit in cfg.units:
        for i, spec in enumerate(unit.sensors):
            where = places.click(unit.name, i)
            try:
                from nxs.suite.reconcile import load_click_personality
                cls = load_click_personality(spec.personality)
            except Exception as exc:
                findings.append(Finding.of(where, exc))
                continue
            try:
                cls().compile(dict(spec.config))
            except Exception as exc:
                findings.append(Finding.of(where, exc))
    named = {f.where for f in findings}
    return wired + [f for f in offered if f.where not in named] + findings


PortDirection = NxsMap.PortDirection


def _wired_to_nothing(cfg) -> list:
    """A camera, a click or a hub the declaration names and wires to nothing
    this host or a unit takes: a node declared and placed nowhere."""
    graph = getattr(cfg, "graph", None)
    if graph is None:
        return []
    from nxs.description import UnknownType, of_type

    findings = []
    for node in graph.nodes.values():
        if node.kind in ("camera", "click", "hub") and not any(
                e.source[0] == node.name for e in graph.edges):
            # The port named is the block's first output, from its description.
            try:
                outputs = [p.name for p in of_type(node.type).ports
                           if p.direction != PortDirection.INPUT]
            except UnknownType:
                outputs = []
            port = outputs[0] if outputs else "out"
            findings.append(Finding(f"nodes.{node.name}",
                                    f"{node.name} feeds nothing: its {port} is wired to no port",
                                    [f"connections.{node.name}.{port}: [<the port it feeds>]",
                                     f"drop nodes.{node.name}"]))
    return findings


def _lane_findings(hub, topology, cams, resolved, port, places) -> list:
    """The lanes of each output the declaration states: the hub's outputs
    as `csi.Output`, each judged on its own lanes with the streams it
    carries (`fps_range` with the outputs, `port_outputs`)."""
    from nxs.cam.contracts import InfeasibleConfig

    if not port.outputs:
        return []
    flows = hub.flows()
    output = getattr(flows, "Output", None)
    if output is None:
        return [Finding(places.port(port.name), f"hub {hub.name} drives no output but port A",
                        [f"drop connections.{port.hub_node}.csi.B"])]
    outs = {name: output(int(lanes), tuple(streams), bool(copy))
            for name, (lanes, streams, copy) in port.outputs.items()}

    def placed(name: str) -> str:
        return (f"nodes.{port.hub_node}.parameters.csi_{name.lower()}_lanes" if port.hub_node
                else places.port(port.name))

    place = placed("B" if "B" in port.outputs else "A")
    findings = []
    for link in cams:
        try:
            rates = flows.fps_range(hub, topology, link, resolved[link.name], outputs=outs)
        except InfeasibleConfig as exc:
            # The hub names the output its refusal is about first.
            return [Finding(placed("A") if exc.reason.startswith("port A") else place,
                            exc.reason, exc.alternatives)]
        fps = link.fps if link.fps is not None else (
            topology.synced_fps if port.sync_source == "fsync" else None)
        if fps is not None and float(fps) > rates.ceiling + 1e-6:
            rated = (places.link_param(port, link.name, "fps") if link.fps is not None
                     else places.hub_param(port, "sync_fps"))
            findings.append(Finding(
                place, f"link {link.name} at {float(fps):g} fps is past the {rates.ceiling:.2f} "
                       f"fps the hub's outputs carry it at (the {rates.binds} law)",
                [f"{rated}: {int(rates.ceiling)}", f"{place}: 4"]))
    return findings


def _gain_shape_finding(hub, topology, synced: bool, place: str):
    """The ports a declared gain locks, by the law `on` holds before its
    first bus write (`on._gain_shape`): a synced pair of one sensor kind,
    or in free run the camera links with a declared exposure. None when
    the port fits."""
    from nxs.cam.contracts import InfeasibleConfig
    from nxs.cam.verbs.on import _gain_shape

    try:
        _gain_shape(hub, hub.flows(), topology, synced)
    except InfeasibleConfig as exc:
        return Finding(place, exc.reason.removeprefix(f"{place}: "), list(exc.alternatives))
    return None


def _gain_finding(hub, topology, cams, resolved, gain_db: float, place: str):
    """A declared gain judged by the composer's law: each camera head's
    CAMERA_MAP law, through the apply a bare head takes it under
    (`compose._gain_units`), so one gain law judges the declaration and the
    graph image alike. None when every head takes it."""
    from nxs.cam import compose
    from nxs.cam.compose_heads import head
    from nxs.cam.contracts import InfeasibleConfig
    from nxs.cam.hubs import is_smart_camera

    try:
        heads = {l.name: head(hub, topology, l, resolved[l.name], None)[0] for l in cams}
        doors = [l.name for l in cams if l.nxs_units or is_smart_camera(hub, l)]
        compose._gain_units(hub, topology, heads, [l.name for l in cams], doors, float(gain_db))
    except InfeasibleConfig as exc:
        return Finding(place, exc.reason.removeprefix(f"{place}: "), list(exc.alternatives))
    return None


def _manifest_finding(error) -> Finding:
    """A loader refusal as a finding: its fact names the YAML path first."""
    where, sep, fact = error.fact.partition(": ")
    if not sep:
        where, fact = "", error.fact
    return Finding(where, fact, error.alternatives)


def node_findings(findings: list, cfg=None) -> tuple:
    """Findings grouped by the port or the unit they are about
    (`ports.<name>`, `units.<name>`): a finding at a node path goes where
    that node is, on a camera chain into a connector, or the unit or the
    click on it. The rest are the declaration's."""
    places = _Places(cfg) if cfg is not None else None
    by_node: dict = {}
    rest = []
    for finding in findings:
        where = getattr(finding, "where", "") or finding
        group = places.node_of(where) if places is not None else None
        if group is None:
            m = re.match(r"^(ports|units)\.([^.:\[\s]+)", where)
            group = f"{m.group(1)}.{m.group(2)}" if m else None
        if group is not None:
            by_node.setdefault(group, []).append(finding)
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
