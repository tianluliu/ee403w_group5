"""Heartbeat publisher. UNVERIFIED: never run in a ROS 2 environment.

Publishes a std_msgs/Header at a fixed rate so other tools can see that the robot software
graph is alive. This is NOT the motor safety mechanism: the motor watchdog lives on the
Arduino (command TTL and link timeout, docs/interfaces.md section 3.4). This node commands
nothing and touches no serial port.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

MIN_RATE_HZ = 0.1
MAX_RATE_HZ = 50.0


def validate_rate_hz(value) -> float:
    rate = float(value)
    if not math.isfinite(rate) or not MIN_RATE_HZ <= rate <= MAX_RATE_HZ:
        raise ValueError(f"rate_hz must be within [{MIN_RATE_HZ}, {MAX_RATE_HZ}], got {value!r}")
    return rate


class HeartbeatNode(Node):
    def __init__(self, **kwargs):
        super().__init__("heartbeat", **kwargs)
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("topic", "/robot/heartbeat")
        self.declare_parameter("source_id", "base_bridge")
        rate = validate_rate_hz(self.get_parameter("rate_hz").value)
        self._source_id = str(self.get_parameter("source_id").value)
        self._publisher = self.create_publisher(Header, str(self.get_parameter("topic").value), 10)
        self._timer = self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = self._source_id
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeartbeatNode()
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
