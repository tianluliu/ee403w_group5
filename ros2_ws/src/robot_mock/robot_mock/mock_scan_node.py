"""Publishes a simulated sensor_msgs/LaserScan from lidar_robot.MockLidar.
UNVERIFIED: never run in a ROS 2 environment.

Safety: this node publishes FAKE obstacles. It therefore refuses to start unless the
configuration is explicitly marked mock, so it cannot be mistaken for the real LiDAR driver.
It needs ``<repo>/src`` on PYTHONPATH (see docs/dev_workflow.md). It publishes only; it
commands nothing.
"""

try:
    from lidar_robot.config import load_config
    from lidar_robot.mock_lidar import MockLidar, Pose2D
    from lidar_robot.world import World
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    raise ImportError(
        "lidar_robot is not importable. Put <repo>/src on PYTHONPATH before running "
        "(export PYTHONPATH=$LIDAR_ROBOT_ROOT/src:$PYTHONPATH)."
    ) from exc

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Fixed test room: 4 m x 4 m with one box. The robot pose is a node parameter.
ROOM = (0.0, 0.0, 4.0, 4.0)
BOX = (2.5, 1.0, 3.0, 1.5)


def build_world() -> World:
    return World.room(*ROOM).with_box(*BOX)


class MockScanNode(Node):
    def __init__(self, **kwargs):
        super().__init__("mock_scan", **kwargs)
        self.declare_parameter("config_path", "")
        self.declare_parameter("topic", "/scan")
        self.declare_parameter("pose_x", 1.0)
        self.declare_parameter("pose_y", 2.0)
        self.declare_parameter("pose_yaw", 0.0)

        path = str(self.get_parameter("config_path").value)
        if not path:
            raise RuntimeError("parameter 'config_path' is required (path to robot.mock.toml)")
        cfg = load_config(path)
        if not cfg.is_mock:
            raise RuntimeError(
                f"refusing to publish simulated scans from a non-mock configuration: {path}")

        self._lidar = MockLidar(cfg.lidar, build_world())   # raises if mount values are missing
        self._pose = Pose2D(float(self.get_parameter("pose_x").value),
                            float(self.get_parameter("pose_y").value),
                            float(self.get_parameter("pose_yaw").value))
        self._publisher = self.create_publisher(
            LaserScan, str(self.get_parameter("topic").value), qos_profile_sensor_data)
        self._timer = self.create_timer(1.0 / cfg.lidar.scan_hz, self._tick)

    def _tick(self):
        now = self.get_clock().now()
        data = self._lidar.scan(self._pose, now.nanoseconds // 1_000_000)
        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = data.frame_id
        msg.angle_min = data.angle_min
        msg.angle_max = data.angle_max
        msg.angle_increment = data.angle_increment
        msg.time_increment = data.time_increment
        msg.scan_time = data.scan_time
        msg.range_min = data.range_min
        msg.range_max = data.range_max
        msg.ranges = list(data.ranges)
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockScanNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
