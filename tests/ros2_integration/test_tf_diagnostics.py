"""Optional diagnostic wiring smoke test; skipped without a built Jazzy workspace."""
import json
import tempfile
import time
import unittest
from pathlib import Path

try:
    import rclpy
    from robot_explorer.explorer_node import ExplorerNode
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS, 'Requires ROS 2 Jazzy and built robot_explorer')
class DiagnosticWiringTests(unittest.TestCase):
    def test_opt_in_timer_and_shutdown_produce_diagnostic_jsonl(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'diagnostic.jsonl'
            rclpy.init(args=['--ros-args', '-p', 'autonomous:=true', '-p',
                             f'benchmark_path:={path}', '-p', 'benchmark_tf_diagnostics:=true'])
            node = None
            try:
                node = ExplorerNode()
                deadline = time.monotonic() + 2.5
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.1)
                node.finish_benchmark('interrupted')
            finally:
                if node is not None:
                    node.destroy_node()
                rclpy.shutdown()
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            snapshots = [row for row in rows if row['event'] == 'tf_snapshot']
            self.assertTrue(snapshots)
            self.assertIn('map->odom', snapshots[0]['edges'])
            timings = [row for row in rows if row['event'] == 'evaluator_timing']
            self.assertTrue(timings)
            self.assertGreaterEqual(timings[0]['duration_s'], 0)
            self.assertEqual(rows[-1]['outcome'], 'interrupted')
