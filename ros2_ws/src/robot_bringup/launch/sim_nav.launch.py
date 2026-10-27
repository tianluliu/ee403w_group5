"""Dynamic simulator with SLAM Toolbox and Nav2 navigation.

TF ownership:
    SLAM Toolbox:       map -> odom
    mock_robot_node:    odom -> base_link
    mock_robot_node:    base_link -> laser

Nav2 consumes the live map, odometry, TF and LaserScan data and produces /cmd_vel.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    root = os.environ.get("LIDAR_ROBOT_ROOT")

    if not root:
        raise RuntimeError(
            "Set LIDAR_ROBOT_ROOT to the repository root before launching."
        )

    nav2_config = os.path.join(
        root,
        "config",
        "nav2_sim.yaml",
    )

    sim_slam_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare("robot_bringup"),
            "launch",
            "sim_slam.launch.py",
        ])
    )

    nav2_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare("nav2_bringup"),
            "launch",
            "navigation_launch.py",
        ])
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            sim_slam_launch,
        ),

        IncludeLaunchDescription(
            nav2_launch,
            launch_arguments={
                "params_file": nav2_config,
                "use_sim_time": "false",
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        ),
    ])
