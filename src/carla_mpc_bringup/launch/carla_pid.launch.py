"""Top-level control pipeline: Carla rig + controller + ground truth + Foxglove.

Launch the full driving stack (sensor/ego spawn, the selected controller, the
ground-truth publisher, the path overlay, and optional Foxglove bridge and
rosbag recorder) with one command:

    ros2 launch carla_mpc_bringup carla_pid.launch.py

Carla must already be running with the native ROS2 bridge enabled:

    ./CarlaUE4.sh -RenderOffScreen --ros2

The default controller is the PID path follower; swap controllers with the
`controller` argument. Common knobs:

    controller:=pid|pidpp|mpc|mpcc   which controller drives
    with_foxglove:=false             skip the Foxglove bridge
    publish_gt:=false                skip the ground-truth publisher
    condition:=rainy_night           named condition from config/conditions.yaml ('' = carla.yaml)
    sensors:=cam_front               sensor whitelist ('' = all enabled in vehicle.yaml)
    foxglove_port:=8765

Config overrides:

    vehicle_config:=/abs/vehicle.yaml  carla_config:=/abs/carla.yaml
    controller_config:=/abs/controller.yaml

The route source is set in config/carla.yaml (route.mode = random | file). For a
reproducible benchmark route, set mode: file and file: routes/<name>.xml there.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PKG = "carla_mpc_bringup"


def generate_launch_description():
    """Build the launch description for the full Carla control pipeline."""
    share = get_package_share_directory(PKG)
    launch_dir = os.path.join(share, "launch")
    default_vehicle = os.path.join(share, "config", "vehicle.yaml")
    default_carla = os.path.join(share, "config", "carla.yaml")
    default_controller = os.path.join(share, "config", "controller.yaml")
    # Invoke record_run by its install path rather than via `ros2 run`: the launch's
    # shutdown signal must reach record_run directly so it can finalize the bag --
    # `ros2 run` does not forward the signal to its child process.
    record_run_exe = os.path.join(get_package_prefix(PKG), "lib", PKG, "record_run")

    vehicle_config = LaunchConfiguration("vehicle_config")
    carla_config = LaunchConfiguration("carla_config")
    controller_config = LaunchConfiguration("controller_config")
    controller = LaunchConfiguration("controller")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_gt = LaunchConfiguration("publish_gt")
    condition = LaunchConfiguration("condition")
    sensors = LaunchConfiguration("sensors")
    no_rendering = LaunchConfiguration("no_rendering")
    with_foxglove = LaunchConfiguration("with_foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    cmap = LaunchConfiguration("map")
    spawn_index = LaunchConfiguration("spawn_index")
    seed = LaunchConfiguration("seed")
    max_distance = LaunchConfiguration("max_distance")
    route_extend = LaunchConfiguration("route_extend")
    warmup = LaunchConfiguration("warmup")
    record = LaunchConfiguration("record")
    record_inputs = LaunchConfiguration("record_inputs")
    record_compress = LaunchConfiguration("record_compress")
    record_camera = LaunchConfiguration("record_camera")
    route = LaunchConfiguration("route")

    def include(name, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=kwargs.items())

    return LaunchDescription([
        DeclareLaunchArgument("vehicle_config", default_value=default_vehicle),
        DeclareLaunchArgument("carla_config", default_value=default_carla),
        DeclareLaunchArgument("controller_config", default_value=default_controller),
        DeclareLaunchArgument("controller", default_value="pid",
                              description="controller: pid | pidpp | mpc | mpcc"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("publish_gt", default_value="true"),
        DeclareLaunchArgument("condition", default_value=""),
        DeclareLaunchArgument("sensors", default_value=""),
        DeclareLaunchArgument("no_rendering", default_value="auto",
                              choices=["auto", "on", "off"]),
        DeclareLaunchArgument("with_foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("map", default_value="",
                              description="CARLA town override (empty = carla.yaml)"),
        DeclareLaunchArgument("spawn_index", default_value="-1",
                              description="spawn-point index (-1 = carla.yaml)"),
        DeclareLaunchArgument("seed", default_value="-1",
                              description="route RNG seed (-1 = carla.yaml)"),
        DeclareLaunchArgument("max_distance", default_value="0.0",
                              description="auto-stop after driven distance (m); 0 = unbounded"),
        DeclareLaunchArgument("route_extend", default_value="true",
                              description="extend route at the end (true|false); false = clean scenario"),
        DeclareLaunchArgument("warmup", default_value="-1.0"),
        DeclareLaunchArgument("record", default_value="false",
                              description="ALSO record a rosbag to ./runs/<ts>_<map>_<controller>/ "
                                          "(for Foxglove replay). OFF by default -- every run already "
                                          "writes human-readable metrics+plots to ./logs/. record:=true to enable"),
        DeclareLaunchArgument("record_inputs", default_value="false",
                              description="also record the camera stream (larger bags)"),
        DeclareLaunchArgument("record_compress", default_value="false",
                              description="zstd-compress the bag (saves disk, costs CPU)"),
        DeclareLaunchArgument("record_camera", default_value="false",
                              description="save the driver-view camera as camera.mp4 (forces rendering ON)"),
        DeclareLaunchArgument("route", default_value="",
                              description="route XML, e.g. routes/sharp_turns.xml ('' = carla.yaml route)"),

        # Carla rig + controller + GT + ego model.
        include("spawn.launch.py",
                vehicle_config=vehicle_config,
                carla_config=carla_config,
                controller_config=controller_config,
                controller=controller,
                publish_gt=publish_gt,
                condition=condition,
                sensors=sensors,
                no_rendering=no_rendering,
                warmup=warmup,
                map=cmap,
                spawn_index=spawn_index,
                seed=seed,
                max_distance=max_distance,
                route_extend=route_extend,
                record_camera=record_camera,
                route=route,
                use_sim_time=use_sim_time),

        # Project the predicted (orange) + reference (blue) paths into the camera image.
        Node(
            package=PKG, executable="predicted_path_overlay", name="predicted_path_overlay",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "vehicle_config": vehicle_config}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "foxglove.launch.py")),
            launch_arguments={"port": foxglove_port,
                              "use_sim_time": use_sim_time}.items(),
            condition=IfCondition(with_foxglove)),

        # Auto-record the run (rosbag2 + manifest + config snapshot -> ./runs/...).
        # On Ctrl-C or launch shutdown the bag finalizes cleanly. Review: ros2 run carla_mpc_bringup review_run.
        ExecuteProcess(
            condition=IfCondition(record),
            output="screen",
            cmd=[record_run_exe,
                 "--controller", controller,
                 "--carla-config", carla_config,
                 "--controller-config", controller_config,
                 "--vehicle-config", vehicle_config,
                 "--condition", condition,
                 PythonExpression(["'--inputs' if '", record_inputs, "' == 'true' else ''"]),
                 PythonExpression(["'--compress' if '", record_compress, "' == 'true' else ''"])],
        ),
    ])
