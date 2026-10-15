"""Differential-drive kinematics tests (platform independent, no hardware).

Mock drivetrain used by the config-driven tests: wheel radius 0.04 m, track 0.20 m,
80 rpm, derate 0.7  =>  full-duty wheel speed = 2*pi*0.04*80/60*0.7 = 0.234572 m/s.
Limits: 0.15 m/s, 0.6 rad/s, duty cap 500 permille.
"""

import dataclasses
import math
import random
import unittest

import _bootstrap
from lidar_robot import kinematics as k
from lidar_robot.config import IncompleteHardwareConfig, load_config
from lidar_robot.kinematics import Twist, WheelSpeeds

MOCK = load_config(_bootstrap.CONFIG_DIR / "robot.mock.toml")
REAL = load_config(_bootstrap.CONFIG_DIR / "robot.toml")
FULL = 2 * math.pi * 0.04 * 80 / 60 * 0.7  # hand-derived in the module docstring


class TwistWheelConversionTests(unittest.TestCase):
    def test_straight_line_gives_equal_wheel_speeds(self):
        ws = k.twist_to_wheel_speeds(Twist(0.1, 0.0), 0.2)
        self.assertEqual((ws.left, ws.right), (0.1, 0.1))

    def test_spin_in_place_gives_opposite_speeds_ccw_positive(self):
        # w = 1 rad/s, track 0.2 => wheels at +/- 0.1 m/s; CCW => right wheel forward.
        ws = k.twist_to_wheel_speeds(Twist(0.0, 1.0), 0.2)
        self.assertAlmostEqual(ws.left, -0.1)
        self.assertAlmostEqual(ws.right, 0.1)

    def test_arc(self):
        # v=0.2, w=0.5, track 0.4 => half = 0.1 => left 0.1, right 0.3
        ws = k.twist_to_wheel_speeds(Twist(0.2, 0.5), 0.4)
        self.assertAlmostEqual(ws.left, 0.1)
        self.assertAlmostEqual(ws.right, 0.3)

    def test_roundtrip(self):
        rng = random.Random(7)
        for _ in range(200):
            t = Twist(rng.uniform(-2, 2), rng.uniform(-5, 5))
            track = rng.uniform(0.05, 1.0)
            back = k.wheel_speeds_to_twist(k.twist_to_wheel_speeds(t, track), track)
            self.assertAlmostEqual(back.linear, t.linear, places=9)
            self.assertAlmostEqual(back.angular, t.angular, places=9)

    def test_non_finite_and_bad_types_rejected(self):
        for bad in (float("nan"), float("inf"), float("-inf"), True, "0.1", None):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    k.twist_to_wheel_speeds(Twist(bad, 0.0), 0.2)
                with self.assertRaises(ValueError):
                    k.twist_to_wheel_speeds(Twist(0.0, bad), 0.2)
                with self.assertRaises(ValueError):
                    k.wheel_speeds_to_twist(WheelSpeeds(bad, 0.0), 0.2)

    def test_track_width_must_be_positive_and_finite(self):
        for bad in (0, -0.2, float("nan"), float("inf")):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    k.twist_to_wheel_speeds(Twist(0.1, 0.0), bad)
                with self.assertRaises(ValueError):
                    k.wheel_speeds_to_twist(WheelSpeeds(0.1, 0.1), bad)


class ClampTests(unittest.TestCase):
    def test_inside_limits_unchanged(self):
        self.assertEqual(k.clamp_twist(Twist(0.1, -0.3), 0.15, 0.6), Twist(0.1, -0.3))

    def test_each_component_clamped_independently_both_signs(self):
        self.assertEqual(k.clamp_twist(Twist(5.0, 0.1), 0.15, 0.6), Twist(0.15, 0.1))
        self.assertEqual(k.clamp_twist(Twist(-5.0, 9.0), 0.15, 0.6), Twist(-0.15, 0.6))
        self.assertEqual(k.clamp_twist(Twist(0.0, -9.0), 0.15, 0.6), Twist(0.0, -0.6))

    def test_limits_must_be_positive(self):
        for bad in (0, -1, float("nan")):
            with self.assertRaises(ValueError):
                k.clamp_twist(Twist(0.1, 0.1), bad, 0.6)
            with self.assertRaises(ValueError):
                k.clamp_twist(Twist(0.1, 0.1), 0.15, bad)

    def test_non_finite_twist_is_an_error_not_silently_clamped(self):
        for bad in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                k.clamp_twist(Twist(bad, 0.0), 0.15, 0.6)


class DutyMappingTests(unittest.TestCase):
    def test_linear_mapping(self):
        self.assertEqual(k.wheel_speeds_to_duty(WheelSpeeds(0.5, -0.25), 1.0, 1000), (500, -250))

    def test_zero_is_zero(self):
        self.assertEqual(k.wheel_speeds_to_duty(WheelSpeeds(0.0, -0.0), 1.0, 500), (0, 0))

    def test_cap_scales_both_wheels_preserving_curvature(self):
        # raw duty (500, 250), cap 300 => factor 0.6 => (300, 150)
        self.assertEqual(k.wheel_speeds_to_duty(WheelSpeeds(0.5, 0.25), 1.0, 300), (300, 150))
        self.assertEqual(k.wheel_speeds_to_duty(WheelSpeeds(-0.5, 0.25), 1.0, 300), (-300, 150))

    def test_value_exactly_at_cap_is_not_scaled(self):
        self.assertEqual(k.wheel_speeds_to_duty(WheelSpeeds(0.3, 0.1), 1.0, 300), (300, 100))

    def test_invalid_arguments(self):
        ws = WheelSpeeds(0.1, 0.1)
        for full in (0, -1.0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                k.wheel_speeds_to_duty(ws, full, 500)
        for cap in (0, -5, 1001, True, 500.0, "500"):
            with self.subTest(cap=cap):
                with self.assertRaises(ValueError):
                    k.wheel_speeds_to_duty(ws, 1.0, cap)

    def test_duty_never_exceeds_cap_for_wild_inputs(self):
        rng = random.Random(99)
        for _ in range(2000):
            cap = rng.randint(1, 1000)
            ws = WheelSpeeds(rng.uniform(-50, 50), rng.uniform(-50, 50))
            full = rng.uniform(0.01, 3.0)
            left, right = k.wheel_speeds_to_duty(ws, full, cap)
            self.assertLessEqual(abs(left), cap)
            self.assertLessEqual(abs(right), cap)
            self.assertIsInstance(left, int)
            # direction is never reversed (it may round to zero)
            self.assertFalse(left * ws.left < 0)
            self.assertFalse(right * ws.right < 0)

    def test_inverse_mapping(self):
        ws = k.duty_to_wheel_speeds(-250, 500, 0.8)
        self.assertAlmostEqual(ws.left, -0.2)
        self.assertAlmostEqual(ws.right, 0.4)


class ConfigDrivenPipelineTests(unittest.TestCase):
    def test_hand_checked_straight_drive(self):
        # 0.1 m/s / 0.234572 m/s = 0.42631 => 426 permille on both wheels
        cmd = k.twist_to_drive(MOCK, Twist(0.1, 0.0))
        self.assertEqual((cmd.left_permille, cmd.right_permille), (426, 426))
        self.assertAlmostEqual(cmd.effective.linear, 0.426 * FULL, places=9)
        self.assertAlmostEqual(cmd.effective.angular, 0.0, places=9)

    def test_reverse(self):
        cmd = k.twist_to_drive(MOCK, Twist(-0.1, 0.0))
        self.assertEqual((cmd.left_permille, cmd.right_permille), (-426, -426))

    def test_stop_command_is_exact_zero(self):
        cmd = k.twist_to_drive(MOCK, Twist(0.0, 0.0))
        self.assertEqual((cmd.left_permille, cmd.right_permille), (0, 0))
        self.assertEqual(cmd.effective, Twist(0.0, 0.0))

    def test_spin_in_place_ccw(self):
        # w=0.6 (the limit) => wheels -/+0.06 m/s => 0.06/0.234572 = 0.25578 => 256 permille
        cmd = k.twist_to_drive(MOCK, Twist(0.0, 0.6))
        self.assertEqual((cmd.left_permille, cmd.right_permille), (-256, 256))
        self.assertGreater(cmd.effective.angular, 0)

    def test_requests_beyond_twist_limits_are_clamped_first(self):
        spin = k.twist_to_drive(MOCK, Twist(0.0, 100.0))
        self.assertEqual((spin.left_permille, spin.right_permille), (-256, 256))  # same as 0.6
        # With a tight linear limit the clamp (not the duty cap) is what binds:
        tight = dataclasses.replace(
            MOCK, limits=dataclasses.replace(MOCK.limits, max_linear_mps=0.05, max_duty_permille=1000))
        cmd = k.twist_to_drive(tight, Twist(10.0, 0.0))
        self.assertEqual((cmd.left_permille, cmd.right_permille), (213, 213))  # 0.05/0.234572

    def test_duty_cap_binds_and_preserves_curvature(self):
        # v=0.15 would be 639 permille > cap 500 => both scaled to 500.
        fast = k.twist_to_drive(MOCK, Twist(10.0, 0.0))
        self.assertEqual((fast.left_permille, fast.right_permille), (500, 500))
        self.assertAlmostEqual(fast.effective.linear, 0.5 * FULL, places=9)
        # v=0.15,w=0.6 => wheels 0.09 / 0.21 m/s; right hits the cap, left keeps ratio 0.09/0.21
        arc = k.twist_to_drive(MOCK, Twist(0.15, 0.6))
        self.assertEqual(arc.right_permille, 500)
        self.assertLessEqual(abs(arc.left_permille - 500 * 0.09 / 0.21), 1.0)

    def test_effective_twist_never_exceeds_configured_limits(self):
        rng = random.Random(5)
        for _ in range(500):
            cmd = k.twist_to_drive(MOCK, Twist(rng.uniform(-3, 3), rng.uniform(-9, 9)))
            self.assertLessEqual(abs(cmd.left_permille), MOCK.limits.max_duty_permille)
            self.assertLessEqual(abs(cmd.right_permille), MOCK.limits.max_duty_permille)
            self.assertLessEqual(abs(cmd.effective.linear), MOCK.limits.max_linear_mps + 1e-9)
            self.assertLessEqual(abs(cmd.effective.angular), MOCK.limits.max_angular_rps + 1e-2)

    def test_non_finite_twist_raises(self):
        for bad in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                k.twist_to_drive(MOCK, Twist(bad, 0.0))
            with self.assertRaises(ValueError):
                k.twist_to_drive(MOCK, Twist(0.0, bad))

    def test_real_config_refuses_and_lists_every_missing_value(self):
        with self.assertRaises(IncompleteHardwareConfig) as ctx:
            k.twist_to_drive(REAL, Twist(0.1, 0.0))
        self.assertEqual(
            sorted(ctx.exception.missing),
            ["drivetrain.output_free_speed_rpm", "drivetrain.track_width_m", "drivetrain.wheel_radius_m"])

    def test_partial_hardware_values_still_refuse(self):
        partial = dataclasses.replace(
            REAL, drivetrain=dataclasses.replace(REAL.drivetrain, wheel_radius_m=0.04, track_width_m=0.2))
        with self.assertRaises(IncompleteHardwareConfig) as ctx:
            k.twist_to_drive(partial, Twist(0.1, 0.0))
        self.assertEqual(ctx.exception.missing, ["drivetrain.output_free_speed_rpm"])


if __name__ == "__main__":
    unittest.main()
