"""Replay filtering, geometry, and experiment gates without ROS dependencies."""
import copy
import subprocess
import sys
from types import SimpleNamespace as NS
import unittest
from _bootstrap import REPO_ROOT
from lidar_robot.slam_replay import (EVENT_NS, angle_delta, baseline_gate, check_ab,
    grid_metrics, retained_transforms, reproduced, wall_distance)


def baseline(trial='A1'):
    return dict(trial=trial, complete=True, parameters=dict(do_loop_closing=True,
        restamp_tf=True, use_sim_time=True), input_digest='same', versions='same', settings={},
        jumps=[dict(time_ns=EVENT_NS, translation_m=1.52)],
        maps=[dict(time_ns=EVENT_NS-1, fraction_over_015=0.),
              dict(time_ns=EVENT_NS+1, fraction_over_015=.45)])


class ReplayTests(unittest.TestCase):
    def test_filter_individual_transforms_preserves_objects_and_order(self):
        def t(a, b):
            return NS(header=NS(frame_id=a, stamp=123), child_frame_id=b)
        keep = t('odom', 'base_link')
        mixed = [t('map', 'odom'), keep, t('base_link', 'other')]
        self.assertEqual(retained_transforms('/tf', mixed), [keep])
        self.assertIs(retained_transforms('/tf', mixed)[0], keep)
        self.assertEqual(retained_transforms('/tf_static', mixed), [])
        static = t('base_link', 'laser')
        self.assertEqual(retained_transforms('/tf_static', [static]), [static])

    def test_geometry_threshold_cell_centers_and_fixed_origin(self):
        m = grid_metrics([64, 65, 100, -1], 2, 2, 1., [0., 0., 0.], [[0, 0, 0, 3]])
        self.assertEqual(m['occupied'], 2)
        self.assertEqual(m['median_m'], 1.)
        self.assertEqual(m['p90_m'], 1.4)
        self.assertEqual(m['fraction_over_015'], 1.)
        shifted = grid_metrics([65], 1, 1, 1., [-.5, 0., 0.], [[0, 0, 0, 3]])
        self.assertEqual(shifted['median_m'], 0.)

    def test_segment_endpoints_and_yaw_wrap(self):
        self.assertEqual(wall_distance(2, 0, [[0, 0, 1, 0]]), 1.)
        self.assertAlmostEqual(angle_delta(-3.14, 3.14), .003185307179586)

    def test_empty_grid_has_no_fake_quality(self):
        self.assertIsNone(grid_metrics([-1], 1, 1, 1, [0, 0, 0], [[0, 0, 1, 0]])['median_m'])

    def test_ab_only_one_behavior_change(self):
        a = baseline()['parameters']
        check_ab(a, dict(a, do_loop_closing=False))
        for b in (a, dict(a, do_loop_closing=False, restamp_tf=False),
                  dict(a, do_loop_closing=False, new_parameter=1)):
            with self.assertRaises(ValueError):
                check_ab(a, b)

    def test_gate_requires_two_matching_complete_reproductions(self):
        self.assertTrue(baseline_gate([baseline(), baseline('A2')])['passed'])
        for field, value in [('complete', False), ('input_digest', 'other'), ('versions', 'other')]:
            b = baseline('A2')
            b[field] = value
            with self.assertRaises(ValueError):
                baseline_gate([baseline(), b])

    def test_jump_alone_or_wrong_sequence_does_not_pass(self):
        for change in ('time', 'map', 'increase', 'jump'):
            r = copy.deepcopy(baseline())
            if change == 'time':
                r['jumps'][0]['time_ns'] += 3_000_000_000
            elif change == 'map':
                r['maps'][1]['fraction_over_015'] = .2
            elif change == 'increase':
                r['maps'][0]['fraction_over_015'] = .4
            else:
                r['jumps'][0]['translation_m'] = .5
            self.assertFalse(reproduced(r))

    def test_script_help_needs_no_ros(self):
        for name in ('prepare', 'run', 'analyze'):
            result = subprocess.run([sys.executable, str(REPO_ROOT/'scripts'/f'{name}_slam_replay.py'), '--help'], capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
