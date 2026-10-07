"""Launch the NXS ROS 2 bridge from the suite manifest: one `nxs --unit <name>
ros2` process per unit (one suite-mode process without a manifest); `viz:=true`
adds RViz2 and a static TF per unit; `cameras:=true` adds one capture node per
camera link the ports' records carry, publishing `/<topic_base>/<port>/<link>/
image_raw` and `camera_info`. Usage: ros2 launch "$(nxs ros2 --launch-file)"."""

import os
import sys
import time

try:
    import nxs  # noqa: F401
except ImportError:
    # `ros2 launch` runs this file under the distribution's interpreter, which
    # does not see an `nxs` installed with pipx. The package's own root goes
    # last on the path, behind every package that interpreter already has.
    package = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.append(os.path.dirname(package))

from nxs.stamp_modes import STAMP_MODES, STAMP_SYNCED

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, TimerAction,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

BASE_FRAME = "nxs"


def _manifest_units():
    """(unit name, sanitized frame, sensor specs) per manifest unit; an absent
    or unreadable manifest yields an empty list."""
    try:
        from nxs.ros2_bridge import sanitize_ros_name
        from nxs.suite import default_config_path
        from nxs.suite.schema import load_suite_config
        cfg = load_suite_config(default_config_path())
        return [(u.name, sanitize_ros_name(u.name), u.sensors)
                for u in cfg.units]
    except Exception:
        return []


def _viz_actions(context):
    """Deferred until launch so `viz` and `topic_base` resolve to
    strings: the RViz config is generated from the manifest, one Imu
    display per unit whose click personality plans an imu topic."""
    if LaunchConfiguration("viz").perform(context).lower() not in ("true",
                                                                   "1"):
        return []
    from nxs.ros2_bridge import build_viz_rviz_config, sensor_plans_imu
    units = _manifest_units()
    imu_units = [(name, frame) for name, frame, sensors in units
                 if any(sensor_plans_imu(s.personality, s.config)
                        for s in (sensors or []))]
    topic_base = LaunchConfiguration("topic_base").perform(context)
    rviz = Node(package="rviz2", executable="rviz2",
                arguments=["-d", build_viz_rviz_config(imu_units, topic_base)],
                output="log")

    # One static transform per unit so RViz can place each frame, spread along
    # Y (real mounting is the consumer's URDF); positional args are the stable form.
    transforms = [
        Node(package="tf2_ros", executable="static_transform_publisher",
             output="log",
             arguments=["0", str(0.5 * i), "0", "0", "0", "0",
                        BASE_FRAME, frame])
        for i, (_, frame, _) in enumerate(units)]

    return [rviz, *transforms]


def _open_port(port):
    """The handler for a port's first camera node's output: when the node
    says its stream started, restart the port's output under it, once. A
    later start of that node is a reopen, which meets the other sessions of
    the port, and a restart would end those."""
    from nxs.ros2_cameras import CAMERA_STARTED
    started = False

    def on_output(event):
        nonlocal started
        if started or CAMERA_STARTED.encode() not in event.text:
            return None
        started = True
        return [ExecuteProcess(cmd=["nxs", "ros2", "--stream-start", port,
                                    "--since", f"{time.time():.3f}"], output="screen")]

    return on_output


def _camera_actions(context):
    """Deferred until launch so `cameras`, `camera_source` and `topic_base`
    resolve: one capture node per camera link with recorded caps, a
    GStreamer camera node each, or the Argus nodes in one component
    container."""
    if LaunchConfiguration("cameras").perform(context).lower() not in ("true", "1"):
        return []
    from nxs.ros2_cameras import (CAMERA_SOURCES, CAMERA_STAGGER_S, argus_node, camera_plan,
                                  gscam_node, port_openers)
    topic_base = LaunchConfiguration("topic_base").perform(context)
    source = LaunchConfiguration("camera_source").perform(context)
    encoding = LaunchConfiguration("camera_encoding").perform(context)
    if source not in CAMERA_SOURCES:
        raise ValueError(f"camera_source must be one of {', '.join(CAMERA_SOURCES)}, not {source!r}")
    plan = camera_plan()
    if not plan:
        return []
    if source == "gstreamer":
        # One source at a time: the second and later nodes start after a
        # stagger, the way the tool starts its own viewers.
        nodes = [Node(**gscam_node(topic, topic_base, encoding=encoding), output="screen")
                 for topic in plan]
        # A port's first session starts on a running output and never sees
        # the stream start: the output is restarted under it.
        openers = []
        for port, i in port_openers(plan).items():
            on_output = _open_port(port)
            openers.append(RegisterEventHandler(OnProcessIO(
                target_action=nodes[i], on_stdout=on_output, on_stderr=on_output)))
        return openers + [nodes[0]] + [TimerAction(period=i * CAMERA_STAGGER_S, actions=[node])
                                       for i, node in enumerate(nodes) if i > 0]
    nodes = [ComposableNode(**argus_node(topic, topic_base)) for topic in plan]
    return [ComposableNodeContainer(name="nxs_cameras", namespace="",
                                    package="rclcpp_components",
                                    executable="component_container_mt",
                                    composable_node_descriptions=nodes,
                                    output="screen")]


def generate_launch_description():
    stamp = LaunchConfiguration("stamp")
    topic_base = LaunchConfiguration("topic_base")

    units = _manifest_units()
    if units:
        bridges = [ExecuteProcess(
            cmd=["nxs", "--unit", name, "ros2",
                 "--stamp", stamp, "--topic-base", topic_base],
            output="screen") for name, _, _ in units]
    else:
        bridges = [ExecuteProcess(
            cmd=["nxs", "ros2", "--stamp", stamp, "--topic-base", topic_base],
            output="screen")]

    return LaunchDescription([
        DeclareLaunchArgument("stamp", default_value=STAMP_SYNCED,
                              description="header.stamp source: "
                                          + "|".join(STAMP_MODES)),
        DeclareLaunchArgument("topic_base", default_value="nxs",
                              description="leading topic namespace"),
        DeclareLaunchArgument("viz", default_value="false",
                              description="also start RViz + per-unit static TF"),
        DeclareLaunchArgument("cameras", default_value="false",
                              description="also publish every camera link's frames"),
        DeclareLaunchArgument("camera_source", default_value="gstreamer",
                              description="the camera node kind: gstreamer|argus"),
        DeclareLaunchArgument("camera_encoding", default_value="yuv422",
                              description="the GStreamer node's image encoding: "
                                          "yuv422|mono8|rgb8|jpeg"),
        *bridges, OpaqueFunction(function=_viz_actions),
        OpaqueFunction(function=_camera_actions)])
