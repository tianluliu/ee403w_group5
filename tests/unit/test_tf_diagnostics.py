"""Diagnostic serialization/probes run without ROS; controller state is untouched."""
import importlib.util
import unittest
from types import SimpleNamespace as NS

from _bootstrap import REPO_ROOT

path = REPO_ROOT / 'ros2_ws/src/robot_explorer/robot_explorer/tf_diagnostics.py'
spec = importlib.util.spec_from_file_location('tf_diagnostics', path)
d = importlib.util.module_from_spec(spec)
spec.loader.exec_module(d)


def transform(nanoseconds):
    sec, nano = divmod(nanoseconds, 1_000_000_000)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=nano)),
              transform=NS(translation=NS(x=1., y=2., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))


class DiagnosticTests(unittest.TestCase):
    def test_latest_success_does_not_hide_timestamp_failure(self):
        calls = []
        def lookup(parent, child, when):
            calls.append((parent, child, when))
            if parent == 'map' and when == 10_000_000_000:
                raise RuntimeError('Extrapolation into future: requested 10, latest 9')
            return transform(9_000_000_000)
        row = d.tf_snapshot(lookup, 10_000_000_000, 10_000_000_000, 8_999_000_000)
        edge = row['edges']['map->odom']
        self.assertTrue(edge['latest']['available'])
        self.assertEqual(edge['latest']['age_s'], 1.)
        self.assertFalse(edge['at_now']['available'])
        self.assertIn('requested 10', edge['at_now']['error'])
        self.assertEqual(row['odom_tf_minus_odometry_s'], .001)
        self.assertIn(('map', 'odom', 9_000_000_000), calls)
        self.assertNotIn('age_s', row['edges']['base_link->laser']['latest'])

    def test_future_transform_and_missing_odometry_are_explicit(self):
        row = d.tf_snapshot(lambda *args: transform(11_000_000_000), 10_000_000_000, 0)
        self.assertEqual(row['edges']['map->odom']['latest']['age_s'], -1.)
        self.assertIsNone(row['odometry_age_s'])
        self.assertIsNone(row['odom_tf_minus_odometry_s'])

    def test_exact_action_fields_and_missing_fields(self):
        self.assertEqual(d.result_fields(NS(error_code=102, error_msg='exact TF text')),
                         {'error_code': 102, 'error_msg': 'exact TF text'})
        self.assertEqual(d.result_fields(NS()), {'error_code': None, 'error_msg': None})
        self.assertEqual(d.result_fields(NS(error_code=0, error_msg=''))['error_msg'], '')

    def test_plan_probe_uses_endpoint_stamp_and_source_frame(self):
        calls = []
        def lookup(*args):
            calls.append(args)
            return transform(5)
        plan = {'frame_id': 'map', 'endpoint_stamp_ns': 123}
        row = d.tf_snapshot(lookup, 1000, 900, plan=plan)
        self.assertIn(('odom', 'map', 123), calls)
        self.assertTrue(row['checks']['plan_to_odom_at_endpoint_stamp']['available'])
        self.assertIn(('map', 'odom', 900), calls)

    def test_requested_pose_preserves_frame_timestamp_and_quaternion(self):
        pose = NS(header=NS(frame_id='map', stamp=NS(sec=4, nanosec=5)),
                  pose=NS(position=NS(x=1., y=2., z=0.), orientation=NS(x=0., y=0., z=.5, w=.5)))
        row = d.pose_fields(pose)
        self.assertEqual(row['stamp_ns'], 4_000_000_005)
        self.assertEqual(row['orientation']['z'], .5)
        self.assertEqual(row['frame_id'], 'map')
