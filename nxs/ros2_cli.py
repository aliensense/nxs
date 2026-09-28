"""`nxs ros2`: what the bridge addresses, the host node-IDs a shared CAN iface needs, and the run."""

import os
import sys
import time

from nxs._generated_constants import CyphalDefaults
from nxs.descriptor import is_decodable
from nxs.stamp_modes import STAMP_ITOW, STAMP_MODES, STAMP_SYNCED
from nxs.stream_cli import _configure_stream
from nxs.transports import open_client


_DEVICE_FLAGS = {'-t', '--transport', '-b', '--bus', '-p', '--port',
                 '-a', '--addr', '--baud', '--remote-node-id'}

def _device_flags_given(argv=None) -> bool:
    """True when the command line addresses a device explicitly; those flags
    select ad-hoc single-device mode even when a manifest exists."""
    argv = sys.argv[1:] if argv is None else argv
    return any(a.split('=', 1)[0] in _DEVICE_FLAGS for a in argv)

def _allocate_can_local_ids(targets):
    """Give each additional cyphal-can client on a shared iface its own local
    node-ID: the first keeps HOST_NODE_ID, later ones count down past manifest IDs."""
    used = {kw.get('remote_node_id') for _, tr, kw in targets
            if tr == 'cyphal-can'}
    seen = set()
    next_id = CyphalDefaults.HOST_NODE_ID - 1
    for _, tr, kw in targets:
        if tr != 'cyphal-can':
            continue
        iface = kw.get('can_iface')
        if iface in seen:
            while next_id in used:
                next_id -= 1
            if next_id <= 0:
                raise SystemExit("nxs ros2: no free host node-IDs left "
                                 "for a shared CAN iface")
            kw['local_node_id'] = next_id
            used.add(next_id)
        seen.add(iface)
    return targets

def _ros2_targets(args, argv=None, cfg=None):
    """Resolve what `ros2` bridges: `(suite_mode, [(name, transport, kwargs)])`,
    where a None transport opens via the global flags. A unit is bridged over
    its first-declared link, and ``cfg`` defaults to the manifest on disk."""
    from nxs.cli import _manifest_unit

    if getattr(args, 'unit', None):
        link = _manifest_unit(args.unit).links[0]
        return False, [(args.unit, link.transport, link.client_kwargs())]
    from nxs.suite import default_config_path
    path = default_config_path()
    if cfg is not None or (os.path.exists(path) and not _device_flags_given(argv)):
        if cfg is None:
            from nxs.suite.schema import ManifestError, load_suite_config
            try:
                cfg = load_suite_config(path)
            except (ManifestError, OSError) as e:
                raise SystemExit(f"nxs ros2: {e}") from None
        if not cfg.units:
            raise SystemExit(f"nxs ros2: no units in {path}")
        targets = [(u.name, u.links[0].transport, u.links[0].client_kwargs())
                   for u in cfg.units]
        return True, _allocate_can_local_ids(targets)
    return False, [(None, None, None)]

def _ros2_lock_tokens(args, targets):
    """One run-lock token per bridged target, keyed on the resolved link kwargs
    (suite and --unit) or the global device flags (ad-hoc)."""
    adhoc = [(f, getattr(args, f, None))
             for f in ('transport', 'port', 'bus', 'addr',
                       'remote_node_id')]
    return [f"{tr or 'flags'}:{sorted(kw.items()) if kw else adhoc}"
            for _, tr, kw in targets]

def cmd_ros2(args, opener=open_client, argv=None, cfg=None):
    """Bridge decoded samples onto ROS 2 topics: the whole suite by
    default, one unit with --unit, or an ad-hoc flag-addressed device."""
    from nxs.cli import _open_transport
    from nxs.ros2_bridge import (
        Ros2Bridge, UnitPlan, acquire_run_lock, epoch_binding, format_plan,
        join_topic, load_map, plan_publications, run_bridge)

    if getattr(args, 'launch_file', False):
        print(os.path.join(os.path.dirname(__file__), 'ros2',
                           'bridge.launch.py'))
        return 0

    suite_mode, targets = _ros2_targets(args, argv, cfg=cfg)
    if not getattr(args, 'plan', False):
        run_lock = acquire_run_lock(  # noqa: F841 (held for process lifetime)
            _ros2_lock_tokens(args, targets))
    if args.map and suite_mode:
        raise SystemExit("nxs ros2: --map applies to a single device — "
                         "use --unit or the transport flags")
    if args.frame_id and suite_mode:
        raise SystemExit("nxs ros2: suite units take their frame_id from "
                         "the manifest name; --frame-id needs --unit or "
                         "the transport flags")

    def refuse(label, reason, client=None):
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        if suite_mode:
            print(f"nxs ros2: {label}: {reason} — skipped", file=sys.stderr)
            return
        raise SystemExit(f"nxs ros2: {label}: {reason}")

    clients, plans = [], []
    for name, transport, kwargs in targets:
        label = name or 'device'
        try:
            client = _open_transport(args) if transport is None \
                else opener(transport, **kwargs)
        except (OSError, ValueError, ImportError, RuntimeError) as e:
            refuse(label, str(e))
            continue
        if not client.probe():
            refuse(label, "no probe answer", client)
            continue
        driver = ""
        for attempt in range(3):
            driver = client.read_driver_name()
            if driver:
                break
            time.sleep(0.2)
        fields = client.read_outputs()
        if not fields:
            refuse(label, "serves no output descriptors — upload and run "
                          "a personality first", client)
            continue
        if not is_decodable(fields):
            refuse(label, "declares an output type this nxs version "
                          "can't decode — update nxs", client)
            continue
        pubs = load_map(args.map) if args.map else plan_publications(fields)
        frame = args.frame_id or name or driver or 'nxs'
        plans.append(UnitPlan(name=name, frame_id=frame, publications=pubs,
                              epoch=epoch_binding(fields)))
        clients.append(client)

    if not plans:
        raise SystemExit("nxs ros2: no bridgeable units")

    if args.plan:
        print(format_plan(plans, args.topic_base))
        for client in clients:
            client.close()
        return 0

    try:
        bridge = Ros2Bridge(plans, topic_base=args.topic_base,
                            stamp_mode=args.stamp,
                            time_syncs=[c.get_time_sync() for c in clients])
    except ImportError:
        for client in clients:
            client.close()
        raise SystemExit(
            "nxs ros2 needs: pip install 'aliensense-nxs[ros2]', inside a sourced ROS 2 "
            "environment (source /opt/ros/<distro>/setup.bash)") from None

    for plan in plans:
        for pub in plan.publications:
            print(f"publishing {join_topic(args.topic_base, plan.name, pub.topic)}"
                  f"  [{pub.msg_type}]")
    if args.stamp in (STAMP_SYNCED, STAMP_ITOW):
        # Warm-start the estimators; itow's documented fallback is the synced
        # projection. The per-unit bound banner prints from run_bridge.
        for client in clients:
            client.time_sync_ping()

    samples = 0
    try:
        for client in clients:
            client.start_stream(_configure_stream(client, args))
        samples = run_bridge(clients, bridge, count=args.count)
    finally:
        for client in clients:
            for step in (client.stop_stream, client.close):
                try:
                    step()
                except Exception:
                    pass
        bridge.shutdown()
    print(f"{samples} sample(s) bridged")
    return 0


def add_ros2_parser(sub) -> None:
    """Register `nxs ros2`."""
    p_ros2 = sub.add_parser('ros2', help='Bridge decoded samples onto '
                                         'ROS 2 topics')
    p_ros2.add_argument('--hz', type=int, default=None,
                        help='Target per-unit output rate in Hz (as '
                             '`stream`)')
    p_ros2.add_argument('-n', '--count', type=int, default=None,
                        help='Stop after N samples total (default: run '
                             'until interrupted)')
    p_ros2.add_argument('--topic-base', default='nxs',
                        help='Leading topic namespace (default: nxs)')
    p_ros2.add_argument('--frame-id', default=None,
                        help='header.frame_id for a single device '
                             '(default: the unit name, else the personality '
                             'name)')
    p_ros2.add_argument('--stamp', choices=list(STAMP_MODES),
                        default=STAMP_SYNCED,
                        help='header.stamp source: the device clock '
                             'projected onto host time via two-way sync '
                             '(synced, default), the raw device '
                             'microsecond clock (device), ROS time '
                             'on arrival, or the GNSS in-message epoch '
                             'mapped to UTC (itow; epoch-capable personalities '
                             'only, gated on fix validity, falls back to '
                             'synced)')
    p_ros2.add_argument('--map', metavar='FILE', default=None,
                        help='Explicit mapping YAML replacing the '
                             'semantic auto-map (single device only)')
    p_ros2.add_argument('--plan', action='store_true',
                        help='Print the topic plan and exit without '
                             'publishing (no ROS required)')
    p_ros2.add_argument('--launch-file', action='store_true',
                        help='Print the path to the shipped ROS 2 launch '
                             'file and exit: ros2 launch "$(nxs ros2 '
                             '--launch-file)"')
