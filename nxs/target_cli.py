"""What a command addresses: the node grammar it rewrites, the manifest link behind a unit, and the transport that opens."""

import os
import sys

from nxs._generated_constants import CyphalDefaults
from nxs.transports import open_client
from nxs import experimental


def _default_can_mtu():
    """$NXS_CAN_MTU as the --mtu default, validated against the choices here
    since argparse never checks a default."""
    raw = os.environ.get('NXS_CAN_MTU', '64')
    try:
        value = int(raw, 0)
    except ValueError:
        value = None
    if value not in (8, 64):
        print(f"nxs: ignoring NXS_CAN_MTU={raw!r} (must be 8 or 64); using 64",
              file=sys.stderr)
        return 64
    return value

def _default_local_node_id():
    """$NXS_LOCAL_NODE_ID as the --local-node-id default, or None to let the
    transport claim the stock HOST_NODE_ID. Junk is dropped loudly here: an
    uncaught ValueError out of argparse's default would abort every command,
    including the ones that never open a Cyphal node."""
    raw = os.environ.get('NXS_LOCAL_NODE_ID')
    if not raw:
        return None
    try:
        return int(raw, 0)
    except ValueError:
        print(f"nxs: ignoring NXS_LOCAL_NODE_ID={raw!r} (not an integer); "
              f"using {CyphalDefaults.HOST_NODE_ID}", file=sys.stderr)
        return None

def _default_transport():
    """Transport used when -t is omitted: $NXS_TRANSPORT if set, else the
    per-platform default."""
    env = os.environ.get('NXS_TRANSPORT')
    if env:
        return env.strip().lower()
    if sys.platform == 'darwin' or sys.platform == 'win32':
        return 'cyphal-serial'
    return 'i2c'

def _manifest_unit(unit_name: str):
    """Resolve a unit from the suite manifest, for `--unit` addressing.
    Exits with a clear message on any miss."""
    import os

    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config

    path = default_config_path()
    if not os.path.exists(path):
        raise SystemExit(f"nxs: --unit needs a suite manifest; none at {path}")
    try:
        cfg = load_suite_config(path)
    except (ManifestError, OSError) as e:
        raise SystemExit(f"nxs: --unit: {e}") from None
    for unit in cfg.units:
        if unit.name == unit_name:
            return unit
    raise SystemExit(f"nxs: no unit named {unit_name!r} in {path}")

def _pick_unit_link(unit):
    """The unit's management link, the first whose device answers a probe in
    declared order, with its open transport; the first link blind when none answers."""
    for link in unit.links:
        transport = None
        try:
            transport = open_client(link.transport, **link.client_kwargs())
            if transport.probe():
                return link, transport
        except Exception:
            pass
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass
    return unit.links[0], None

def _apply_unit_link(args, link):
    """Sync the display/decision args to a manifest-resolved `--unit` link, so
    a command that prints its target shows the real link."""
    args.transport = link.transport
    if link.transport == 'i2c':
        args.bus, args.addr = link.bus, link.address
    elif link.transport == 'cyphal-can':
        args.port, args.remote_node_id = link.iface, link.node_id
    elif link.transport == 'cyphal-serial':
        args.port = link.port
        if link.baud is not None:
            args.baud = link.baud

def _resolve_default_link(args, opener=None):
    """No -b and no $NXS_BUS: the manifest names the bus, and one unit with an
    i2c link supplies bus and address; else the platform's camera buses are swept."""
    import os

    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config

    hint = ("no I2C bus given — pass -b/--bus, set $NXS_BUS, or declare "
            "the unit in suite.yaml and use --unit")
    path = default_config_path()
    links = []
    if os.path.exists(path):
        # A manifest that does not parse never yields to the sweep: the
        # operator's declaration is not bypassed because it is broken.
        try:
            cfg = load_suite_config(path)
        except (ManifestError, OSError):
            raise SystemExit(f"nxs: {hint}") from None
        links = [(u.name, l) for u in cfg.units for l in u.links
                 if l.transport == 'i2c']
    if len(links) == 1:
        args.bus, args.addr = links[0][1].bus, links[0][1].address
        return
    if links:
        names = ", ".join(sorted({n for n, _ in links}))
        raise SystemExit(f"nxs: several i2c units in {path} ({names}) — "
                         f"say --unit NAME or pass -b/--bus")
    hits, swept = _units_on_camera_buses(opener)
    if len(hits) == 1:
        args.bus, args.addr = hits[0].bus, hits[0].address
        return
    if hits:
        found = ", ".join(h.describe() for h in hits)
        raise SystemExit(f"nxs: several NXS units on the camera buses "
                         f"({found}) — pass -b/--bus and -a/--addr, or "
                         f"declare them in suite.yaml and use --unit")
    if swept:
        hint += f"; nothing answered at 0x30/0x31 on {', '.join(swept)}"
    raise SystemExit(f"nxs: {hint}")

def _units_on_camera_buses(opener=None):
    """(links of the units answering on the platform's camera buses,
    the port names swept). Empty off-target."""
    from nxs import host as host_layer
    from nxs.suite.scan import scan_bus_units

    try:
        buses = host_layer.current().camera_buses()
    except Exception:
        return [], []
    hits = []
    for name in sorted(buses):
        hits += [found.link for found
                 in scan_bus_units(buses[name], opener=opener or open_client)]
    return hits, sorted(buses)

def _open_transport(args):
    """Build a transport from --transport and its flags, or from the named
    unit's manifest link under `--unit`. A missing `--port` is autodetected."""
    if getattr(args, 'unit', None):
        link, transport = _pick_unit_link(_manifest_unit(args.unit))
        _apply_unit_link(args, link)
        if transport is not None:
            return transport
        return open_client(link.transport, **link.client_kwargs())
    # The transport-to-class table lives in open_client(); this assembles the
    # CLI-derived kwargs and delegates.
    if args.transport == 'i2c':
        if args.bus is None:
            _resolve_default_link(args)
        return open_client('i2c', bus=args.bus, address=args.addr)
    # A node-ID of None lets each Cyphal client fall back to its own default;
    # only forward an explicit --remote-node-id (i2c ignores it).
    cyphal_kwargs = {}
    if getattr(args, 'remote_node_id', None) is not None:
        cyphal_kwargs['remote_node_id'] = args.remote_node_id
    if getattr(args, 'local_node_id', None) is not None:
        cyphal_kwargs['local_node_id'] = args.local_node_id
    if args.transport == 'cyphal-serial':
        if not args.port:
            from nxs.serial_util import autodetect_serial_port
            args.port = autodetect_serial_port()
            if not args.port:
                raise SystemExit(
                    "no serial port: autodetect found zero or many — "
                    "pass --port explicitly.")
        return open_client('cyphal-serial', port=args.port, baud=args.baud,
                           **cyphal_kwargs)
    if args.transport == 'cyphal-can':
        # --port carries the SocketCAN interface name for CAN (e.g. can0).
        return open_client('cyphal-can', can_iface=(args.port or 'can0'),
                           can_mtu=args.mtu, **cyphal_kwargs)
    raise ValueError("unknown transport")

# Camera-family verbs owned by the port/link node; everything else on a
# node routes to the link's NXS unit.
_CAM_VERBS = {"on", "off", "stream", "capture", "status", "caps",
              "get", "set"}

# Verbs no unit command claims, so they route to the camera family even
# with no port token in front.
_CAM_ONLY_VERBS = {"on", "off"}

def _manifest_nodes():
    """(port names, unit names) from suite.yaml; empty when absent."""
    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config
    try:
        cfg = load_suite_config(default_config_path())
    except (ManifestError, OSError):
        return {}, set()
    return cfg.ports, {u.name for u in cfg.units}

def _unit_behind(port, link):
    """`(link name, unit name)` of the unit riding a port link; refuses
    ambiguity by listing."""
    carried = [(l.name, l.unit.name) for l in port.links if l.unit]
    if link is not None:
        for lname, uname in carried:
            if lname == link:
                return lname, uname
        raise SystemExit(f"no unit on {port.name}/{link}")
    if len(carried) == 1:
        return carried[0]
    if not carried:
        raise SystemExit(f"no units on port {port.name}")
    choices = ", ".join(f"{l}:{u}" for l, u in carried)
    raise SystemExit(f"port {port.name} carries several units ({choices}) "
                     f"— name the link")

def _platform_ports():
    """Camera ports this machine exposes ({} off-target or on error)."""
    from nxs import host as host_layer
    try:
        return host_layer.current().camera_buses()
    except Exception:
        return {}

def _node_rewrite(argv):
    """`nxs cam0 A on …` and `nxs imu-front status …` to the classic forms: the
    node decides the wires, the verb the family. A leading `--experimental`
    unlocks the experimental surface before the parser is built, so the verbs
    take their deep options."""
    if len(argv) >= 2 and argv[1] == experimental.FLAG:
        experimental.enable()
        rest = _node_rewrite([argv[0], *argv[2:]])
        return [rest[0], experimental.FLAG, *rest[1:]]
    if len(argv) < 2 or argv[1].startswith("-"):
        return argv
    token = argv[1]
    port_token, colon_link = (
        token.split(":", 1) if ":" in token else (token, None))
    ports, units = _manifest_nodes()
    if token in units:
        return [argv[0], "--unit", token, *argv[2:]]
    declared = port_token in ports
    plat_ports = {}
    if not declared:
        plat_ports = _platform_ports()
        if port_token not in plat_ports:
            if token in _CAM_ONLY_VERBS:
                # A bare camera verb lets the port-required refusal name the
                # ports; verbs shared with units keep their top-level meaning.
                return [argv[0], "cam", "--_node", *argv[1:]]
            return argv
    rest = list(argv[2:])
    link_names = ({l.name for l in ports[port_token].links} if declared
                  else {"A", "B"})
    # A run of link names, not one: the camera verbs take a selector of
    # several links (`nxs cam0 A B stream`, the line `up` prints as its
    # own next step), and stopping after the first read the second as
    # the verb.
    links = [colon_link.upper()] if colon_link else []
    while not colon_link and rest and rest[0].upper() in link_names:
        links.append(rest.pop(0).upper())
    if declared and rest and len(rest[0]) == 1 and rest[0].isalpha():
        # A link the declaration does not carry: the fact and the key to
        # add, not the parser's usage.
        from nxs.suite import default_config_path
        wanted = rest[0].upper()
        raise SystemExit(
            f"{port_token}: no link {wanted} in the declaration "
            f"(declared: {', '.join(sorted(link_names)) or 'none'})\n"
            f"  - ports.{port_token}.links.{wanted} in {default_config_path()}, "
            f"then nxs switch")
    if not rest:
        rest = ["status"]
    verb, vargs = rest[0], rest[1:]
    if verb in _CAM_VERBS:
        out = [argv[0], "cam", "--_node", "--port", port_token, verb]
        if links and verb in {"set", "get"}:
            # set/get name one link as a flag; the rest take a selector.
            if len(links) > 1:
                raise SystemExit(
                    f"nxs {port_token} {verb}: name one link, got "
                    f"{' '.join(links)}")
            out += ["--link", links[0]]
        else:
            out += links
        return out + vargs
    link = links[0] if links else None
    if declared:
        if len(links) > 1:
            raise SystemExit(
                f"nxs {port_token} {verb}: a unit rides one link, got "
                f"{' '.join(links)}")
        _link, unit = _unit_behind(ports[port_token], link)
        return [argv[0], "--unit", unit, verb, *vargs]
    # Manifest-less unit verbs ride the bare transport: the port's bus reaches
    # the link-A unit at its native address. A second link needs a declaration.
    if link is not None and link != "A":
        raise SystemExit(
            f"nxs {port_token} {link}: the unit behind link {link} answers "
            f"at its declared alias — declare it (ports.{port_token}.links."
            f"{link}.unit: {{name, alias: 0x31}}) or pass "
            f"-b {plat_ports[port_token]} -a 0x31")
    return [argv[0], "--transport", "i2c",
            "--bus", plat_ports[port_token], verb, *vargs]
