"""Launch the NXS ROS 2 bridge from the suite manifest.

A thin, distro-agnostic wrapper (no ament package, no per-distro
build). With a readable manifest it starts one `nxs --unit <name>
ros2` process per unit — the ROS-idiomatic process-per-node shape, and
the rate-correct one: a single Python interpreter caps the combined
sample pipeline well under two 200 Hz units, while per-unit processes
each poll, decode, and publish at the device rate. Without a manifest
it falls back to one suite-mode process. `viz:=true` adds a
`static_transform_publisher` per manifest unit and RViz2 with a config
generated from the manifest — one Imu display per IMU-publishing unit,
wired to its `/<base>/<unit>/imu` topic — so the sensors show up in
one command.

    ros2 launch "$(nxs ros2 --launch-file)"
    ros2 launch "$(nxs ros2 --launch-file)" viz:=true stamp:=arrival
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BASE_FRAME = "nxs"


def _manifest_units():
    """(unit name, sanitized frame, sensor specs) per manifest unit,
    best-effort — an absent or unreadable manifest yields an empty list
    (the launch falls back to one suite-mode bridge; RViz still runs)."""
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
    display per unit whose driver plans an imu topic."""
    if LaunchConfiguration("viz").perform(context).lower() not in ("true",
                                                                   "1"):
        return []
    from nxs.ros2_bridge import build_viz_rviz_config, sensor_plans_imu
    units = _manifest_units()
    imu_units = [(name, frame) for name, frame, sensors in units
                 if any(sensor_plans_imu(s.driver, s.config)
                        for s in (sensors or []))]
    topic_base = LaunchConfiguration("topic_base").perform(context)
    rviz = Node(package="rviz2", executable="rviz2",
                arguments=["-d", build_viz_rviz_config(imu_units, topic_base)],
                output="log")

    # One static transform per unit so RViz can place each frame; spread
    # along Y so they read apart on the bench (real mounting is the
    # consumer's URDF). Positional args (x y z yaw pitch roll parent
    # child) are the form stable across Humble→Jazzy.
    transforms = [
        Node(package="tf2_ros", executable="static_transform_publisher",
             output="log",
             arguments=["0", str(0.5 * i), "0", "0", "0", "0",
                        BASE_FRAME, frame])
        for i, (_, frame, _) in enumerate(units)]

    return [rviz, *transforms]


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
        DeclareLaunchArgument("stamp", default_value="synced",
                              description="header.stamp source: synced|device|arrival"),
        DeclareLaunchArgument("topic_base", default_value="nxs",
                              description="leading topic namespace"),
        DeclareLaunchArgument("viz", default_value="false",
                              description="also start RViz + per-unit static TF"),
        *bridges, OpaqueFunction(function=_viz_actions)])
