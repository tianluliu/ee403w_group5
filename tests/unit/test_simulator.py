"""Unit tests for the lightweight differential-drive simulator.

These tests require no ROS and no physical hardware.
"""

import dataclasses
import math
import unittest

import _bootstrap
from lidar_robot.config import load_config
from lidar_robot.mock_lidar import Pose2D
from lidar_robot.simulator import Simulator2D
from lidar_robot.world import World


MOCK = load_config(_bootstrap.CONFIG_DIR / "robot.mock.toml")
REAL = load_config(_bootstrap.CONFIG_DIR / "robot.toml")

ROOM = World.room(0.0, 0.0, 4.0, 4.0)


class Simulator2DTests(unittest.TestCase):
    def test_straight_forward_motion(self):
        sim = Simulator2D(
            MOCK,
            ROOM,
            Pose2D(2.0, 2.0, 0.0),
        )

        actual = sim.step(500, 500, 1.0)

        expected_speed = MOCK.full_duty_speed_mps() * 0.5

        self.assertAlmostEqual(
            sim.pose.x,
            2.0 + expected_speed,
            places=9,
        )
        self.assertAlmostEqual(sim.pose.y, 2.0, places=9)
        self.assertAlmostEqual(sim.pose.yaw, 0.0, places=9)

        self.assertAlmostEqual(
            actual.linear,
            expected_speed,
            places=9,
        )
        self.assertAlmostEqual(actual.angular, 0.0, places=9)

    def test_reverse_motion(self):
        sim = Simulator2D(
            MOCK,
            ROOM,
            Pose2D(2.0, 2.0, 0.0),
        )

        sim.step(-500, -500, 1.0)

        expected_speed = MOCK.full_duty_speed_mps() * 0.5

        self.assertAlmostEqual(
            sim.pose.x,
            2.0 - expected_speed,
            places=9,
        )
        self.assertAlmostEqual(sim.pose.y, 2.0, places=9)

    def test_rotation_in_place(self):
        sim = Simulator2D(
            MOCK,
            ROOM,
            Pose2D(2.0, 2.0, 0.0),
        )

        actual = sim.step(-500, 500, 1.0)

        wheel_speed = MOCK.full_duty_speed_mps() * 0.5
        expected_angular = (
            2.0 * wheel_speed
            / MOCK.drivetrain.track_width_m
        )

        self.assertAlmostEqual(sim.pose.x, 2.0, places=9)
        self.assertAlmostEqual(sim.pose.y, 2.0, places=9)
        self.assertAlmostEqual(
            sim.pose.yaw,
            expected_angular,
            places=9,
        )

        self.assertAlmostEqual(actual.linear, 0.0, places=9)
        self.assertAlmostEqual(
            actual.angular,
            expected_angular,
            places=9,
        )

    def test_arc_motion_changes_position_and_heading(self):
        sim = Simulator2D(
            MOCK,
            ROOM,
            Pose2D(2.0, 2.0, 0.0),
        )

        actual = sim.step(250, 500, 1.0)

        self.assertGreater(sim.pose.x, 2.0)
        self.assertGreater(sim.pose.y, 2.0)
        self.assertGreater(sim.pose.yaw, 0.0)

        self.assertGreater(actual.linear, 0.0)
        self.assertGreater(actual.angular, 0.0)

    def test_wall_collision_blocks_translation(self):
        sim = Simulator2D(
            MOCK,
            ROOM,
            Pose2D(3.80, 2.0, 0.0),
        )

        actual = sim.step(500, 500, 1.0)

        self.assertAlmostEqual(sim.pose.x, 3.80, places=9)
        self.assertAlmostEqual(sim.pose.y, 2.0, places=9)

        self.assertEqual(sim.collision_count, 1)
        self.assertAlmostEqual(actual.linear, 0.0, places=9)

    def test_rotation_is_allowed_when_translation_collides(self):
        sim = Simulator2D(
            MOCK,
            ROOM,
            Pose2D(3.80, 2.0, 0.0),
        )

        actual = sim.step(250, 500, 1.0)

        # Proposed arc intersects the nearby wall, so x/y stay fixed,
        # but the circular robot may still rotate.
        if sim.collision_count:
            self.assertAlmostEqual(sim.pose.x, 3.80, places=9)
            self.assertAlmostEqual(sim.pose.y, 2.0, places=9)
            self.assertNotEqual(sim.pose.yaw, 0.0)
            self.assertAlmostEqual(actual.linear, 0.0, places=9)

    def test_initial_pose_in_collision_is_rejected(self):
        with self.assertRaises(ValueError):
            Simulator2D(
                MOCK,
                ROOM,
                Pose2D(3.95, 2.0, 0.0),
            )

    def test_real_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            Simulator2D(
                REAL,
                ROOM,
                Pose2D(2.0, 2.0, 0.0),
            )

    def test_invalid_timestep_is_rejected(self):
        sim = Simulator2D(
            MOCK,
            ROOM,
            Pose2D(2.0, 2.0, 0.0),
        )

        for dt in (
            0.0,
            -0.1,
            math.inf,
            -math.inf,
            math.nan,
        ):
            with self.subTest(dt=dt):
                with self.assertRaises(ValueError):
                    sim.step(0, 0, dt)

    def test_pose_property_tracks_latest_pose(self):
        initial = Pose2D(2.0, 2.0, 0.0)

        sim = Simulator2D(
            MOCK,
            ROOM,
            initial,
        )

        self.assertEqual(sim.pose, initial)

        sim.step(500, 500, 0.5)

        self.assertNotEqual(sim.pose, initial)


if __name__ == "__main__":
    unittest.main()
