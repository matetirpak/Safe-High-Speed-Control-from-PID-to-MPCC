"""Spawn the rig, run the controller, and publish ground truth.

Launches three nodes (no Foxglove bridge):

  control_node        -- spawns ego + sensors, drives the controller, ticks the
                         world, publishes reference + telemetry (the single ticker)
  gt_pose_publisher   -- ground-truth driven path/odom + gt_map->hero TF anchor
  ego_model_publisher -- the ego car MarkerArray in the `hero` frame

Carla must already be running with `--ros2`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PKG = "carla_mpc_bringup"


def generate_launch_description():
    """Declare launch arguments and assemble the three-node launch description."""
    share = get_package_share_directory(PKG)
    default_vehicle = os.path.join(share, "config", "vehicle.yaml")
    default_carla = os.path.join(share, "config", "carla.yaml")
    default_controller = os.path.join(share, "config", "controller.yaml")

    vehicle_config = LaunchConfiguration("vehicle_config")
    carla_config = LaunchConfiguration("carla_config")
    controller_config = LaunchConfiguration("controller_config")
    controller = LaunchConfiguration("controller")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_gt = LaunchConfiguration("publish_gt")
    condition = LaunchConfiguration("condition")
    sensors = LaunchConfiguration("sensors")
    no_rendering = LaunchConfiguration("no_rendering")
    warmup = LaunchConfiguration("warmup")
    cmap = LaunchConfiguration("map")
    spawn_index = LaunchConfiguration("spawn_index")
    seed = LaunchConfiguration("seed")
    max_distance = LaunchConfiguration("max_distance")
    route_extend = LaunchConfiguration("route_extend")
    record_camera = LaunchConfiguration("record_camera")
    route = LaunchConfiguration("route")

    return LaunchDescription([
        DeclareLaunchArgument("vehicle_config", default_value=default_vehicle),
        DeclareLaunchArgument("carla_config", default_value=default_carla),
        DeclareLaunchArgument("controller_config", default_value=default_controller),
        DeclareLaunchArgument("controller", default_value="pid",
                              description="controller: pid | pidpp | mpc | mpcc"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("publish_gt", default_value="true"),
        DeclareLaunchArgument("condition", default_value="",
                              description="named condition / CARLA preset; '' = carla.yaml weather"),
        DeclareLaunchArgument("sensors", default_value="",
                              description="comma-separated sensor whitelist; '' = all enabled"),
        DeclareLaunchArgument("no_rendering", default_value="auto",
                              choices=["auto", "on", "off"]),
        DeclareLaunchArgument("map", default_value="",
                              description="CARLA town override (empty = carla.yaml)"),
        DeclareLaunchArgument("spawn_index", default_value="-1",
                              description="spawn-point index (-1 = carla.yaml)"),
        DeclareLaunchArgument("seed", default_value="-1",
                              description="route RNG seed (-1 = carla.yaml)"),
        DeclareLaunchArgument("max_distance", default_value="0.0",
                              description="auto-stop after this driven distance (m); 0 = unbounded"),
        DeclareLaunchArgument("route_extend", default_value="true",
                              description="extend route with random goals at the end (true|false); "
                                          "false for a clean deterministic scenario"),
        DeclareLaunchArgument("warmup", default_value="-1.0",
                              description="seconds to hold still before driving; <0 = carla.yaml"),
        DeclareLaunchArgument("record_camera", default_value="false",
                              description="save the driver-view camera as camera.mp4 (forces rendering ON)"),
        DeclareLaunchArgument("route", default_value="",
                              description="route XML, e.g. routes/sharp_turns.xml ('' = carla.yaml route)"),

        # The single world ticker.
        Node(
            package=PKG, executable="control_node", name="control_node",
            output="screen",
            # control_node owns the run: when it exits (route complete / max distance /
            # crash) tear the whole launch down so no gt nodes linger. Its own teardown
            # (actors destroyed, metrics + camera.mp4 written) finishes before exit, so
            # nothing gets truncated.
            on_exit=Shutdown(),
            arguments=[
                "--vehicle-config", vehicle_config,
                "--carla-config", carla_config,
                "--controller-config", controller_config,
                "--controller", controller,
                "--condition", condition,
                "--sensors", sensors,
                "--no-rendering-mode", no_rendering,
                "--warmup", warmup,
                "--map", cmap,
                "--spawn-index", spawn_index,
                "--seed", seed,
                "--max-distance", max_distance,
                "--route-extend", route_extend,
                "--route", route,
                PythonExpression(["'--record-camera' if '", record_camera, "' == 'true' else ''"]),
            ],
        ),

        Node(
            package=PKG, executable="ego_model_publisher", name="ego_model_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        Node(
            package=PKG, executable="gt_pose_publisher", name="gt_pose_publisher",
            output="screen",
            condition=IfCondition(publish_gt),
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
