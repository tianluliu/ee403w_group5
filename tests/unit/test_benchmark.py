import json
import tempfile
import unittest
from pathlib import Path

from _bootstrap import REPO_ROOT  # noqa: F401
from lidar_robot.benchmark import Benchmark, BenchmarkLog


class BenchmarkTests(unittest.TestCase):
    def make(self):
        return Benchmark(10, {"suppression_radius_m": 0.35})

    def test_distance_only_during_run_and_no_duplicate_samples(self):
        b = self.make()
        b.odometry(100, 100, 0)
        b.event("ready", 12)
        b.odometry(0, 0, 1)
        b.odometry(3, 4, 2)
        b.odometry(99, 99, 2)
        b.odometry(float("nan"), 0, 3)
        b.event("complete", 20)
        b.odometry(6, 8, 4)
        r = b.summary(100, "COMPLETE")
        self.assertEqual(r["path_distance_m"], 5)
        self.assertEqual(r["elapsed_s"], 8)
        self.assertEqual(r["startup_s"], 2)
        self.assertEqual(r["odometry_samples"], 2)
        self.assertEqual(r["counts"]["invalid_odometry"], 1)
        self.assertEqual(r["outcome"], "complete")
        self.assertIsNone(b.event("interrupted", 100))

    def test_retry_requires_expired_nearby_failure(self):
        b = self.make()
        b.event("suppression", 11, x=1, y=2, expires=30)
        b.event("latched", 20, x=1, y=2)
        b.event("latched", 31, x=3, y=2)
        b.event("latched", 32, x=1.1, y=2)
        self.assertEqual(b.counts["retry"], 1)

    def test_planner_failure_is_not_navigation_failure(self):
        b = self.make()
        b.event("failure", 12, phase="VALIDATING", reason="empty path")
        b.event("navigation_result", 14, status=6)
        self.assertEqual(b.counts["failure_VALIDATING"], 1)
        self.assertEqual(b.counts["navigation_result_status_6"], 1)
        self.assertEqual(b.counts["navigation_request"], 0)

    def test_partial_run_and_exclusive_jsonl_output(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "run.jsonl"
            log = BenchmarkLog(path, 10, {"suppression_radius_m": 0.35})
            with self.assertRaises(FileExistsError):
                BenchmarkLog(path, 10, {})
            log.event("interrupted", 15)
            log.close(15, "SEARCHING")
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(rows[-1]["outcome"], "interrupted")
            self.assertIsNone(rows[-1]["elapsed_s"])
            self.assertIsNone(rows[-1]["coverage"])


class SupervisorObservationTests(unittest.TestCase):
    """Exercise actual completion/observer methods without importing ROS on macOS."""

    def node(self):
        import ast
        import time
        from types import SimpleNamespace
        from unittest.mock import Mock

        path = REPO_ROOT / 'ros2_ws/src/robot_explorer/robot_explorer/explorer_node.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        selected = {"_handle_no_usable_frontiers", "_observe", "finish_benchmark", "_set_state", "_evaluate_timed"}
        cls.body = [n for n in cls.body if isinstance(n, ast.Assign)
                    or isinstance(n, ast.FunctionDef) and n.name in selected]
        cls.bases = []
        module = ast.Module(body=[ast.ImportFrom(module='__future__',
                            names=[ast.alias(name='annotations')], level=0), cls], type_ignores=[])
        ast.fix_missing_locations(module)
        import math
        namespace = {"time": time, "math": math, "Bool": SimpleNamespace}
        exec(compile(module, str(path), 'exec'), namespace)
        node = namespace['ExplorerNode']()
        node._benchmark = None
        node._reset_candidate = Mock()
        node._cleanup_suppressed = Mock()
        node.get_logger = Mock(return_value=Mock())
        node._suppressed = []
        node._completion_count = 0
        node._completion_checks = 5
        node._completion_max_known_growth_cells = 5
        node._last_known_count = None
        node._last_summary = None
        node._state = 'SEARCHING'
        node._complete_pub = Mock()
        return node

    def test_clusters_and_suppression_block_completion(self):
        node = self.node()
        for _ in range(10):
            node._handle_no_usable_frontiers(1, 100)
        self.assertEqual(node._state, 'SEARCHING')
        node._suppressed = [(1, 2, 30)]
        for _ in range(10):
            node._handle_no_usable_frontiers(0, 100)
        self.assertEqual(node._state, 'SEARCHING')
        node._complete_pub.publish.assert_not_called()

    def test_completion_still_requires_stable_checks_and_writes_summary(self):
        from unittest.mock import Mock
        node = self.node()
        node._benchmark = Mock()
        for _ in range(4):
            node._handle_no_usable_frontiers(0, 100)
        self.assertEqual(node._state, 'SEARCHING')
        log = node._benchmark
        node._handle_no_usable_frontiers(0, 100)
        self.assertEqual(node._state, 'COMPLETE')
        log.event.assert_called_once()
        self.assertEqual(log.event.call_args.args[0], 'complete')
        log.close.assert_called_once()
        self.assertTrue(node._complete_pub.publish.call_args.args[0].data)

    def test_recording_failure_does_not_escape_control_callback(self):
        from unittest.mock import Mock
        node = self.node()
        node._benchmark = Mock()
        node._benchmark.event.side_effect = OSError('disk full')
        node._benchmark.stream.close.side_effect = OSError('disk full')
        node._observe('evaluation')
        self.assertIsNone(node._benchmark)

    def test_state_observation_preserves_transition_and_ignores_same_state(self):
        from unittest.mock import Mock
        node = self.node()
        node._tf_diagnostics = True
        node._observe = Mock()
        node._set_state('VALIDATING', 'planner_request')
        self.assertEqual(node._state, 'VALIDATING')
        node._observe.assert_called_once_with('state_transition', previous='SEARCHING',
                                             state='VALIDATING', reason='planner_request')
        node._set_state('VALIDATING', 'same')
        self.assertEqual(node._observe.call_count, 1)

    def test_evaluator_timing_observes_overrun_without_hiding_failure(self):
        from unittest.mock import Mock, patch
        node = self.node()
        node._decision_period_s = 1.
        node._evaluate_due = 9.
        node._last_evaluate_start = 7.
        node._observe = Mock()
        node._evaluate = Mock(side_effect=RuntimeError("original failure"))
        with patch('time.monotonic', side_effect=[10., 12.]):
            with self.assertRaisesRegex(RuntimeError, 'original failure'):
                node._evaluate_timed()
        node._observe.assert_called_once_with('evaluator_timing', duration_s=2.,
                                             estimated_lateness_s=1., interval_s=3., state='SEARCHING')
        self.assertEqual(node._evaluate_due, 11.)
