"""Mock graph: heartbeat + simulated /scan + a static map->odom->base_link->laser chain.

UNVERIFIED: never launched in a ROS 2 environment.

Safety: this file is hard-wired to config/robot.mock.toml (there is no launch argument that
could point it at the real configuration), and the identity transforms come from
``mock_tf_edges``, which raises for any non-mock config. It starts no motor-related node.
Requires the environment variable LIDAR_ROBOT_ROOT (repository root) and <repo>/src on
PYTHONPATH; see docs/dev_workflow.md.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root = os.environ.get("LIDAR_ROBOT_ROOT")
    if not root:
        raise RuntimeError("Set LIDAR_ROBOT_ROOT to the repository root before launching.")
    config_path = os.path.join(root, "config", "robot.mock.toml")

    from lidar_robot.config import load_config
    from lidar_robot.frames import mock_tf_edges, static_publisher_args

    edges = mock_tf_edges(load_config(config_path))   # raises unless the config is a mock

    actions = [
        Node(package="robot_base_bridge", executable="heartbeat_node", name="heartbeat",
             output="screen"),
        Node(package="robot_mock", executable="mock_scan_node", name="mock_scan",
             output="screen", parameters=[{"config_path": config_path}]),
    ]
    for edge in edges:
        actions.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"mock_tf_{edge.parent}_to_{edge.child}",
            arguments=static_publisher_args(edge)))
    return LaunchDescription(actions)
