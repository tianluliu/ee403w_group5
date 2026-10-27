"""Dynamic simulator with online SLAM Toolbox mapping.

TF ownership:
    SLAM Toolbox:       map -> odom
    mock_robot_node:    odom -> base_link
    mock_robot_node:    base_link -> laser

Unlike sim.launch.py, this launch file deliberately does NOT publish a static
map -> odom transform because SLAM Toolbox must be the sole owner of that edge.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    root = os.environ.get("LIDAR_ROBOT_ROOT")

    if not root:
        raise RuntimeError(
            "Set LIDAR_ROBOT_ROOT to the repository root before launching."
        )

    robot_config = os.path.join(
        root,
        "config",
        "robot.mock.toml",
    )

    slam_config = os.path.join(
        root,
        "config",
        "slam_toolbox_sim.yaml",
    )

    slam_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare("slam_toolbox"),
            "launch",
            "online_async_launch.py",
        ])
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
                "config_path": robot_config,
                "pose_x": 1.0,
                "pose_y": 2.0,
                "pose_yaw": 0.0,
            }],
        ),

        IncludeLaunchDescription(
            slam_launch,
            launch_arguments={
                "slam_params_file": slam_config,

                # Our simulator publishes normal wall-clock ROS timestamps.
                # It does not publish a /clock topic.
                "use_sim_time": "false",
            }.items(),
        ),
    ])
