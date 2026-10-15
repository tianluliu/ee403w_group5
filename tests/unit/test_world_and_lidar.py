"""Tests for the minimal simulated environment and mock LiDAR (no ROS, no hardware).

Geometry used throughout: a 4 m x 4 m room with corners (0,0)-(4,4); robot at its centre
(2,2); mock lidar mounted 0.05 m ahead of base_link, 500 beams (0.72 deg), beam i at angle
-pi + i*2*pi/500 in the laser frame, so i=250 is straight ahead, i=125 is -90 deg
(right), i=375 is +90 deg (left), i=0 is straight behind.
"""

import dataclasses
import math
import unittest

import _bootstrap
from lidar_robot.config import IncompleteHardwareConfig, load_config
from lidar_robot.mock_lidar import MockLidar, Pose2D, ScanData
from lidar_robot.world import World

MOCK = load_config(_bootstrap.CONFIG_DIR / "robot.mock.toml")
REAL = load_config(_bootstrap.CONFIG_DIR / "robot.toml")
ROOM = World.room(0, 0, 4, 4)
CENTRE = Pose2D(2.0, 2.0, 0.0)


class WorldRaycastTests(unittest.TestCase):
    def test_axis_aligned_rays_from_an_off_centre_point(self):
        for angle, expected in ((0.0, 3.0), (math.pi / 2, 3.0), (math.pi, 1.0), (-math.pi / 2, 1.0)):
            with self.subTest(angle=angle):
                self.assertAlmostEqual(ROOM.raycast(1.0, 1.0, angle), expected, places=9)

    def test_diagonal_ray_hits_the_corner_at_the_expected_distance(self):
        self.assertAlmostEqual(ROOM.raycast(2.0, 2.0, math.pi / 4), 2 * math.sqrt(2), places=9)

    def test_max_range_is_inclusive(self):
        self.assertEqual(ROOM.raycast(1.0, 1.0, 0.0, max_range=3.0), 3.0)
        self.assertIsNone(ROOM.raycast(1.0, 1.0, 0.0, max_range=2.999))

    def test_segments_behind_the_ray_are_ignored(self):
        self.assertAlmostEqual(ROOM.raycast(3.5, 1.0, 0.0), 0.5, places=9)

    def test_empty_world_and_open_directions_return_none(self):
        self.assertIsNone(World().raycast(0.0, 0.0, 1.0))
        self.assertIsNone(World.room(0, 0, 4, 4).raycast(10.0, 10.0, 0.0))  # outside, facing away

    def test_box_obstacle_occludes_the_wall_behind_it(self):
        world = ROOM.with_box(2.0, 1.0, 3.0, 3.0)
        self.assertAlmostEqual(world.raycast(1.0, 2.0, 0.0), 1.0, places=9)
        self.assertAlmostEqual(world.raycast(1.0, 3.5, 0.0), 3.0, places=9)  # passes above the box

    def test_ray_through_a_box_corner_does_not_leak(self):
        world = ROOM.with_box(2.0, 2.0, 3.0, 3.0)
        self.assertAlmostEqual(world.raycast(1.0, 1.0, math.pi / 4), math.sqrt(2), places=9)

    def test_with_box_returns_a_new_world(self):
        bigger = ROOM.with_box(1, 1, 2, 2)
        self.assertEqual((len(ROOM.segments), len(bigger.segments)), (4, 8))

    def test_invalid_geometry_and_arguments_rejected(self):
        for args in ((0, 0, 0, 4), (0, 0, 4, 0), (4, 0, 0, 4), (0, 0, float("nan"), 4), (0, 0, float("inf"), 4)):
            with self.subTest(args=args):
                with self.assertRaises(ValueError):
                    World.room(*args)
                with self.assertRaises(ValueError):
                    ROOM.with_box(*args)
        for x, y, a in ((float("nan"), 0, 0), (0, float("inf"), 0), (0, 0, float("nan"))):
            with self.assertRaises(ValueError):
                ROOM.raycast(x, y, a)
        for rng_max in (-1.0, float("nan")):
            with self.assertRaises(ValueError):
                ROOM.raycast(1, 1, 0.0, max_range=rng_max)


class MockLidarScanTests(unittest.TestCase):
    def setUp(self):
        self.lidar = MockLidar(MOCK.lidar, ROOM)

    def test_scan_metadata_matches_laserscan_conventions(self):
        s = self.lidar.scan(CENTRE, stamp_ms=1234)
        self.assertIsInstance(s, ScanData)
        self.assertEqual(self.lidar.beam_count, 500)
        self.assertEqual(len(s.ranges), 500)
        self.assertIsInstance(s.ranges, tuple)
        self.assertEqual((s.stamp_ms, s.frame_id), (1234, "laser"))
        self.assertAlmostEqual(s.angle_min, -math.pi)
        self.assertAlmostEqual(s.angle_increment, 2 * math.pi / 500)
        self.assertAlmostEqual(s.angle_max, s.angle_min + 499 * s.angle_increment)
        self.assertAlmostEqual(s.angle_max, math.pi - s.angle_increment)   # no duplicate +pi beam
        self.assertAlmostEqual(s.scan_time, 0.1)
        self.assertAlmostEqual(s.time_increment, 0.1 / 500)
        self.assertEqual((s.range_min, s.range_max), (0.05, 12.0))

    def test_beam_angles(self):
        self.assertAlmostEqual(self.lidar.beam_angle(0), -math.pi)
        self.assertAlmostEqual(self.lidar.beam_angle(125), -math.pi / 2)
        self.assertAlmostEqual(self.lidar.beam_angle(250), 0.0)
        self.assertAlmostEqual(self.lidar.beam_angle(375), math.pi / 2)

    def test_cardinal_ranges_include_the_mount_offset(self):
        r = self.lidar.scan(CENTRE, 0).ranges
        self.assertAlmostEqual(r[250], 1.95, places=9)  # ahead: laser at x=2.05, wall x=4
        self.assertAlmostEqual(r[0], 2.05, places=9)    # behind: wall x=0
        self.assertAlmostEqual(r[125], 2.0, places=9)   # right: wall y=0
        self.assertAlmostEqual(r[375], 2.0, places=9)   # left:  wall y=4

    def test_robot_yaw_rotates_the_whole_scan(self):
        r = self.lidar.scan(Pose2D(2.0, 2.0, math.pi / 2), 0).ranges   # facing +y
        self.assertAlmostEqual(r[250], 1.95, places=6)   # laser at y=2.05, wall y=4
        self.assertAlmostEqual(r[375], 2.0, places=6)    # robot-left is now world -x: wall x=0 from x=2
        self.assertAlmostEqual(r[0], 2.05, places=6)     # behind = world -y: wall y=0 from y=2.05

    def test_mount_yaw_and_lateral_offset_are_applied(self):
        # Off-centre on purpose: at the exact centre of a square room every direction
        # looks alike (2.0 m) and a missing rotation would go unnoticed.
        turned = dataclasses.replace(MOCK.lidar, mount_yaw_rad=math.pi / 2, mount_x_m=0.0)
        r = MockLidar(turned, ROOM).scan(Pose2D(1.0, 2.0, 0.0), 0).ranges
        self.assertAlmostEqual(r[250], 2.0, places=6)    # laser 0 deg now points world +y: wall y=4
        self.assertAlmostEqual(r[375], 1.0, places=6)    # laser +90 deg points world -x: wall x=0
        straight = dataclasses.replace(MOCK.lidar, mount_yaw_rad=0.0, mount_x_m=0.0)
        r = MockLidar(straight, ROOM).scan(Pose2D(1.0, 2.0, 0.0), 0).ranges
        self.assertAlmostEqual(r[250], 3.0, places=6)    # same pose, no mount yaw: wall x=4
        shifted = dataclasses.replace(MOCK.lidar, mount_x_m=0.0, mount_y_m=0.1)
        r = MockLidar(shifted, ROOM).scan(CENTRE, 0).ranges
        self.assertAlmostEqual(r[250], 2.0, places=9)    # ahead unaffected by a left shift
        self.assertAlmostEqual(r[375], 1.9, places=9)    # left wall is 0.1 m closer

    def test_obstacle_in_front_shortens_only_the_covered_beams(self):
        lidar = MockLidar(MOCK.lidar, ROOM.with_box(3.0, 1.5, 3.5, 2.5))
        r = lidar.scan(CENTRE, 0).ranges
        self.assertAlmostEqual(r[250], 0.95, places=9)
        self.assertAlmostEqual(r[0], 2.05, places=9)     # behind is untouched
        self.assertAlmostEqual(r[375], 2.0, places=9)

    def test_too_close_is_negative_infinity_per_rep117(self):
        lidar = MockLidar(dataclasses.replace(MOCK.lidar, mount_x_m=0.0), ROOM)
        r = lidar.scan(Pose2D(3.97, 2.0, 0.0), 0).ranges   # 0.03 m from the wall, range_min 0.05
        self.assertEqual(r[250], -math.inf)

    def test_range_min_boundary_is_valid(self):
        cfg = dataclasses.replace(MOCK.lidar, mount_x_m=0.0, range_min_m=0.5)
        r = MockLidar(cfg, ROOM).scan(Pose2D(3.5, 2.0, 0.0), 0).ranges   # exactly 0.5 m (binary exact)
        self.assertEqual(r[250], 0.5)
        r = MockLidar(cfg, ROOM).scan(Pose2D(3.75, 2.0, 0.0), 0).ranges  # 0.25 m < range_min
        self.assertEqual(r[250], -math.inf)

    def test_beyond_max_range_is_positive_infinity(self):
        hall = World.room(0, 0, 30, 4)
        r = MockLidar(MOCK.lidar, hall).scan(Pose2D(1.0, 2.0, 0.0), 0).ranges
        self.assertEqual(r[250], math.inf)                 # wall 29 m away > 12 m
        self.assertAlmostEqual(r[0], 1.05, places=9)       # wall behind is in range

    def test_empty_world_is_all_positive_infinity(self):
        r = MockLidar(MOCK.lidar, World()).scan(CENTRE, 0).ranges
        self.assertEqual(set(r), {math.inf})

    def test_no_nan_and_every_value_is_in_one_of_the_three_categories(self):
        world = ROOM.with_box(2.5, 2.5, 3.0, 3.0)
        s = MockLidar(MOCK.lidar, world).scan(Pose2D(1.0, 1.0, 0.3), 0)
        for v in s.ranges:
            self.assertFalse(math.isnan(v))
            self.assertTrue(v in (math.inf, -math.inf) or s.range_min <= v <= s.range_max)

    def test_scanning_is_deterministic_and_does_not_mutate_inputs(self):
        pose = Pose2D(1.3, 2.2, 0.7)
        self.assertEqual(self.lidar.scan(pose, 5), self.lidar.scan(pose, 5))
        self.assertEqual(pose, Pose2D(1.3, 2.2, 0.7))

    def test_beam_count_follows_configured_increment(self):
        cfg = dataclasses.replace(MOCK.lidar, angular_increment_deg=1.0)
        self.assertEqual(MockLidar(cfg, ROOM).beam_count, 360)
        with self.assertRaises(ValueError):
            MockLidar(dataclasses.replace(MOCK.lidar, angular_increment_deg=200.0), ROOM)


class MockLidarHardwareIsolationTests(unittest.TestCase):
    def test_real_config_lacks_mount_values_so_the_simulator_refuses_it(self):
        with self.assertRaises(IncompleteHardwareConfig) as ctx:
            MockLidar(REAL.lidar, ROOM)
        self.assertEqual(sorted(ctx.exception.missing),
                         ["lidar.mount_x_m", "lidar.mount_y_m", "lidar.mount_yaw_rad"])

    def test_each_missing_mount_value_is_individually_fatal(self):
        for key in ("mount_x_m", "mount_y_m", "mount_yaw_rad"):
            with self.subTest(key=key):
                with self.assertRaises(IncompleteHardwareConfig) as ctx:
                    MockLidar(dataclasses.replace(MOCK.lidar, **{key: None}), ROOM)
                self.assertEqual(ctx.exception.missing, [f"lidar.{key}"])


if __name__ == "__main__":
    unittest.main()
