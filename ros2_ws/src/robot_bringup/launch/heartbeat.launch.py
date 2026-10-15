"""Heartbeat only. UNVERIFIED: never launched in a ROS 2 environment."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rate_hz", default_value="2.0"),
        Node(
            package="robot_base_bridge",
            executable="heartbeat_node",
            name="heartbeat",
            output="screen",
            parameters=[{"rate_hz": LaunchConfiguration("rate_hz")}],
        ),
    ])
