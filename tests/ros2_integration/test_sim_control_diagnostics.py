"""Opt-in simulator topic smoke test; requires built ROS 2 Jazzy packages."""
import json
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from robot_mock.mock_robot_node import MockRobotNode
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS, 'Requires ROS 2 Jazzy and built robot_mock')
class SimulatorDiagnosticWiringTests(unittest.TestCase):
    def test_topic_reports_idle_without_arming(self):
        rclpy.init(args=['--ros-args', '-p', f'config_path:={ROOT}/config/robot.mock.toml',
                         '-p', 'control_diagnostics:=true'])
        executor = SingleThreadedExecutor()
        nodes = []
        try:
            simulator = MockRobotNode()
            nodes.append(simulator)
            listener = Node('sim_diagnostic_test_listener')
            nodes.append(listener)
            rows = []
            subscription = listener.create_subscription(
                String, '/sim/control_diagnostics', lambda msg: rows.append(json.loads(msg.data)), 10)
            for node in nodes:
                executor.add_node(node)
            deadline = time.monotonic() + 5.
            while not rows and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.1)
            self.assertTrue(rows)
            self.assertEqual(rows[0]['schema_version'], 1)
            self.assertEqual(rows[0]['mcu']['state'], 'DISARMED')
            self.assertFalse(rows[0]['motion_session_active'])
            self.assertEqual(rows[0]['mcu']['outputs'], [0, 0])
            self.assertTrue({'physics', 'control', 'scan'} <= rows[0]['timers'].keys())
            self.assertNotIn('ARM', rows[0]['mcu']['command_counts'])
        finally:
            for node in nodes:
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
