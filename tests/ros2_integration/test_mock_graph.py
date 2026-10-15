"""ROS 2 integration tests. *** UNVERIFIED: these have NEVER been executed. ***

They need a real ROS 2 Jazzy environment: `source /opt/ros/jazzy/setup.bash`, build
ros2_ws with colcon, `source ros2_ws/install/setup.bash`, and put <repo>/src on PYTHONPATH.
On a machine without rclpy (the MacBook) every test here is SKIPPED, which is expected and
does NOT count as a pass. Treat nothing in this file as validated until it has run green on
the Pi (or another Jazzy machine).

Run from the repository root:  scripts/test.sh ros
"""

import os
import sys
import time
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header
    HAVE_RCLPY = True
except ImportError:
    HAVE_RCLPY = False

try:
    from robot_base_bridge.heartbeat_node import HeartbeatNode
    from robot_mock.mock_scan_node import MockScanNode
    HAVE_PACKAGES = HAVE_RCLPY
except ImportError:
    HAVE_PACKAGES = False

MOCK_CONFIG = str(REPO_ROOT / "config" / "robot.mock.toml")
REAL_CONFIG = str(REPO_ROOT / "config" / "robot.toml")
SKIP_REASON = ("UNVERIFIED: rclpy or the built robot_* packages are unavailable; run on a "
               "ROS 2 Jazzy machine after `colcon build` and sourcing the workspace")


def spin_until(executor, predicate, timeout_s=5.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end and not predicate():
        executor.spin_once(timeout_sec=0.1)
    return predicate()


@unittest.skipUnless(HAVE_PACKAGES, SKIP_REASON)
class MockGraphIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.executor = SingleThreadedExecutor()
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            self.executor.remove_node(node)
            node.destroy_node()

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def test_mock_scan_node_publishes_a_laserscan_with_the_expected_shape(self):
        self.add(MockScanNode(parameter_overrides=[Parameter("config_path", value=MOCK_CONFIG)]))
        listener = self.add(Node("scan_listener"))
        got = []
        listener.create_subscription(LaserScan, "/scan", got.append, qos_profile_sensor_data)
        self.assertTrue(spin_until(self.executor, lambda: bool(got)), "no /scan message within 5 s")
        scan = got[0]
        self.assertEqual(scan.header.frame_id, "laser")
        self.assertEqual(len(scan.ranges), 500)
        self.assertAlmostEqual(scan.range_min, 0.05, places=5)
        self.assertAlmostEqual(scan.range_max, 12.0, places=4)

    def test_mock_scan_node_refuses_the_real_configuration(self):
        with self.assertRaises(RuntimeError):
            MockScanNode(parameter_overrides=[Parameter("config_path", value=REAL_CONFIG)])

    def test_mock_scan_node_requires_a_config_path(self):
        with self.assertRaises(RuntimeError):
            MockScanNode()

    def test_heartbeat_publishes_headers(self):
        self.add(HeartbeatNode(parameter_overrides=[Parameter("rate_hz", value=10.0)]))
        listener = self.add(Node("heartbeat_listener"))
        got = []
        listener.create_subscription(Header, "/robot/heartbeat", got.append, 10)
        self.assertTrue(spin_until(self.executor, lambda: len(got) >= 2), "fewer than 2 heartbeats in 5 s")
        self.assertEqual(got[0].frame_id, "base_bridge")

    def test_heartbeat_rejects_invalid_rates(self):
        for bad in (0.0, -1.0, 1000.0):
            with self.subTest(rate=bad):
                with self.assertRaises(ValueError):
                    HeartbeatNode(parameter_overrides=[Parameter("rate_hz", value=bad)])


if __name__ == "__main__":
    unittest.main()
