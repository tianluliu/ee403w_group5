"""Lightweight deterministic 2D differential-drive robot simulator.

This deliberately models only the pieces needed to develop the robot software before
hardware assembly: differential-drive motion, a circular robot footprint, and collisions
against the existing World line segments.

Motor input is the same signed permille duty (-1000..1000) used by the Pi <-> Uno
protocol, allowing the simulator to sit behind MockMotorController just like the real
Cytron MDD10A will sit behind the Elegoo Uno R3.
"""

from __future__ import annotations

import math

from .config import RobotConfig
from .kinematics import (
    Twist,
    duty_to_wheel_speeds,
    wheel_speeds_to_twist,
)
from .mock_lidar import Pose2D
from .world import World


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _point_segment_distance(px, py, ax, ay, bx, by) -> float:
    vx = bx - ax
    vy = by - ay
    length_sq = vx * vx + vy * vy

    if length_sq == 0.0:
        return math.hypot(px - ax, py - ay)

    t = ((px - ax) * vx + (py - ay) * vy) / length_sq
    t = max(0.0, min(1.0, t))

    cx = ax + t * vx
    cy = ay + t * vy

    return math.hypot(px - cx, py - cy)


class Simulator2D:
    def __init__(
        self,
        cfg: RobotConfig,
        world: World,
        initial_pose: Pose2D,
    ) -> None:
        if not cfg.is_mock:
            raise ValueError("Simulator2D requires a mock configuration")

        if cfg.drivetrain.track_width_m is None:
            raise ValueError("mock drivetrain.track_width_m is required")

        if cfg.body.footprint_radius_m is None:
            raise ValueError("mock body.footprint_radius_m is required")

        self._cfg = cfg
        self._world = world
        self._pose = initial_pose
        self._radius = cfg.body.footprint_radius_m
        self._track = cfg.drivetrain.track_width_m
        self._full_speed = cfg.full_duty_speed_mps()

        if self._collides(initial_pose.x, initial_pose.y):
            raise ValueError("initial simulated robot pose is in collision")

        self.collision_count = 0

    @property
    def pose(self) -> Pose2D:
        return self._pose

    def _collides(self, x: float, y: float) -> bool:
        for (ax, ay), (bx, by) in self._world.segments:
            distance = _point_segment_distance(x, y, ax, ay, bx, by)
            if distance < self._radius - 1e-9:
                return True
        return False

    def step(
        self,
        left_permille: int,
        right_permille: int,
        dt: float,
    ) -> Twist:
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be a finite positive number")

        wheels = duty_to_wheel_speeds(
            left_permille,
            right_permille,
            self._full_speed,
        )

        commanded = wheel_speeds_to_twist(wheels, self._track)

        x = self._pose.x
        y = self._pose.y
        yaw = self._pose.yaw

        v = commanded.linear
        w = commanded.angular

        new_yaw = _wrap(yaw + w * dt)

        if abs(w) < 1e-9:
            new_x = x + v * math.cos(yaw) * dt
            new_y = y + v * math.sin(yaw) * dt
        else:
            radius = v / w
            new_x = x + radius * (math.sin(new_yaw) - math.sin(yaw))
            new_y = y - radius * (math.cos(new_yaw) - math.cos(yaw))

        if self._collides(new_x, new_y):
            self.collision_count += 1

            # Circular footprint: rotation in place is still allowed.
            self._pose = Pose2D(x, y, new_yaw)
            return Twist(0.0, w)

        self._pose = Pose2D(new_x, new_y, new_yaw)
        return commanded
