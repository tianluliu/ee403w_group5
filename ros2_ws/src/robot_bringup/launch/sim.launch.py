"""Standalone dynamic 2D robot simulation without SLAM.

odom -> base_link is dynamic and owned by mock_robot_node. Because this launch
profile intentionally omits SLAM, it publishes a simulation-only identity
map -> odom for visualization. sim_slam.launch.py and sim_nav.launch.py instead
let SLAM Toolbox be the sole owner of map -> odom.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root = os.environ.get("LIDAR_ROBOT_ROOT")

    if not root:
        raise RuntimeError(
            "Set LIDAR_ROBOT_ROOT to the repository root before launching."
        )

    config_path = os.path.join(
        root,
        "config",
        "robot.mock.toml",
    )

    return LaunchDescription([
        Node(
            package="robot_base_bridge",
            executable="heartbeat_node",
            name="heartbeat",
            output="screen",
        ),

        Node(
            package="robot_mock",
            executable="mock_robot_node",
            name="mock_robot",
            output="screen",
            parameters=[{
                "config_path": config_path,
                "pose_x": 1.0,
                "pose_y": 2.0,
                "pose_yaw": 0.0,
            }],
        ),

        # Standalone-simulation-only map -> odom identity.
        # SLAM launch profiles do not include this publisher.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_map_to_odom",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--yaw", "0.0",
                "--pitch", "0.0",
                "--roll", "0.0",
                "--frame-id", "map",
                "--child-frame-id", "odom",
            ],
        ),
    ])
