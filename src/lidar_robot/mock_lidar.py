"""Deterministic simulated 2D LiDAR producing data shaped like sensor_msgs/LaserScan.

Pure Python (no rclpy). ``ScanData`` carries the same fields as LaserScan (minus
intensities) so a thin ROS 2 node can copy it across without interpretation.

Range conventions follow REP-117: a return closer than ``range_min`` is -inf, no return
within ``range_max`` is +inf. NaN ("invalid") is never produced by this ideal sensor.
Angles are measured counter-clockwise from the laser frame's +x axis (REP-103). The real
RPLIDAR C1 scan direction/zero angle as published by sllidar_ros2 is UNVERIFIED
(docs/open_questions.md) and must be checked in Phase 2 before trusting this model.

The sensor pose in the world is: robot pose (base_link) composed with the configured
mount (lidar.mount_x/y/yaw). Missing mount values raise IncompleteHardwareConfig; nothing
is defaulted.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from .config import IncompleteHardwareConfig, Lidar
from .world import World


@dataclass(frozen=True)
class Pose2D:
    """base_link pose in the world frame: metres, radians (CCW positive)."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class ScanData:
    stamp_ms: int
    frame_id: str
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    ranges: Tuple[float, ...]


class MockLidar:
    def __init__(self, lidar: Lidar, world: World) -> None:
        missing = [f"lidar.{k}" for k in ("mount_x_m", "mount_y_m", "mount_yaw_rad")
                   if getattr(lidar, k) is None]
        if missing:
            raise IncompleteHardwareConfig("sim", missing)
        self._cfg = lidar
        self._world = world
        self._count = int(round(360.0 / lidar.angular_increment_deg))
        if self._count < 3:
            raise ValueError("angular_increment_deg too large for a 360-degree scan")
        self._increment = 2.0 * math.pi / self._count

    @property
    def beam_count(self) -> int:
        return self._count

    def beam_angle(self, index: int) -> float:
        """Beam angle in the laser frame (radians); index 0 is -pi."""
        return -math.pi + index * self._increment

    def scan(self, pose: Pose2D, stamp_ms: int) -> ScanData:
        cfg = self._cfg
        cos_y, sin_y = math.cos(pose.yaw), math.sin(pose.yaw)
        laser_x = pose.x + cfg.mount_x_m * cos_y - cfg.mount_y_m * sin_y
        laser_y = pose.y + cfg.mount_x_m * sin_y + cfg.mount_y_m * cos_y
        laser_yaw = pose.yaw + cfg.mount_yaw_rad

        ranges = []
        for i in range(self._count):
            hit = self._world.raycast(laser_x, laser_y, laser_yaw + self.beam_angle(i), cfg.range_max_m)
            if hit is None:
                ranges.append(math.inf)           # no return within range_max
            elif hit < cfg.range_min_m:
                ranges.append(-math.inf)          # too close to measure
            else:
                ranges.append(hit)

        scan_time = 1.0 / cfg.scan_hz
        return ScanData(
            stamp_ms=int(stamp_ms),
            frame_id=cfg.frame_id,
            angle_min=-math.pi,
            angle_max=-math.pi + (self._count - 1) * self._increment,
            angle_increment=self._increment,
            time_increment=scan_time / self._count,
            scan_time=scan_time,
            range_min=cfg.range_min_m,
            range_max=cfg.range_max_m,
            ranges=tuple(ranges),
        )
