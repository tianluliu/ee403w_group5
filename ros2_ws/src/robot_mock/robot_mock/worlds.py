"""Reusable deterministic worlds for ROS simulation."""

from lidar_robot.world import World

ROOM = (0.0, 0.0, 4.0, 4.0)
BOX = (2.5, 1.0, 3.0, 1.5)


def build_demo_world() -> World:
    return World.room(*ROOM).with_box(*BOX)
