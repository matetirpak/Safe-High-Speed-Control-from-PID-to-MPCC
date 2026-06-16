"""Expose the ROS2 graph to Foxglove Studio over a WebSocket bridge.

Connect Foxglove Studio to ``ws://localhost:8765`` and import the layout in
foxglove/carla_mpc_layout.json. The bridge package installs once via
``sudo apt install ros-humble-foxglove-bridge``.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Foxglove bridge node with configurable port, address, and topic filter."""
    port = LaunchConfiguration("port")
    address = LaunchConfiguration("address")
    use_sim_time = LaunchConfiguration("use_sim_time")
    topic_whitelist = LaunchConfiguration("topic_whitelist")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("address", default_value="0.0.0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("topic_whitelist", default_value="['.*']",
                              description="regex list of topics to expose"),
        Node(
            package="foxglove_bridge", executable="foxglove_bridge",
            name="foxglove_bridge", output="screen",
            parameters=[{
                "port": port,
                "address": address,
                "use_sim_time": use_sim_time,
                "topic_whitelist": topic_whitelist,
                "send_buffer_limit": 10000000,
                "max_qos_depth": 10,
                "use_compression": False,
            }],
        ),
    ])
