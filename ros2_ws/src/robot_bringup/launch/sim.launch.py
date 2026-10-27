"""Dynamic 2D robot simulation.

Unlike mock.launch.py, odom -> base_link is dynamic and owned by mock_robot_node.
map -> odom is temporarily identity for visualization. When SLAM Toolbox is added,
SLAM will become the sole owner of map -> odom.
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

        # Temporary simulation-only map -> odom identity.
        # Remove this publisher once SLAM Toolbox owns map -> odom.
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
