"""Reusable deterministic worlds for ROS simulation."""

from lidar_robot.world import World


# Original simple validation world.
ROOM = (0.0, 0.0, 4.0, 4.0)
BOX = (2.5, 1.0, 3.0, 1.5)


def build_demo_world() -> World:
    return World.room(*ROOM).with_box(*BOX)


def build_complex_world() -> World:
    """Larger exploration world with obstacles, rooms, and doorways.

    Overall room:
        8 m x 6 m

    Contains:
      - central rectangular obstacle,
      - sealed upper-left room,
      - right-side room with a traversable doorway,
      - partial upper walls that create occluded regions.

    The sealed room intentionally contains space that LiDAR cannot observe,
    but the robot cannot physically reach. The explorer should therefore
    eventually ignore that unknown region.

    The right-side room has an 0.8 m doorway and must not be ruled out as
    inaccessible.
    """

    world = World.room(
        0.0,
        0.0,
        8.0,
        6.0,
    )

    # Central obstacle. Forces paths around it and creates LiDAR occlusion.
    world = world.with_box(
        2.7,
        2.35,
        3.6,
        3.25,
    )

    # Completely sealed room in the upper-left area.
    #
    # Its interior should remain unknown, but because there is no doorway,
    # frontier exploration should eventually determine that there is no
    # reachable path into it.
    world = world.with_box(
        0.9,
        4.25,
        2.3,
        5.35,
    )

    extra_walls = (
        # ------------------------------------------------------------
        # Right-side interior room.
        #
        # Rectangle from x=5.0..7.3, y=0.8..3.6.
        # Left wall has an 0.8 m doorway from y=1.8..2.6.
        # ------------------------------------------------------------

        # Bottom wall
        ((5.0, 0.8), (7.3, 0.8)),

        # Right wall
        ((7.3, 0.8), (7.3, 3.6)),

        # Top wall
        ((7.3, 3.6), (5.0, 3.6)),

        # Left wall below doorway
        ((5.0, 0.8), (5.0, 1.8)),

        # Left wall above doorway
        ((5.0, 2.6), (5.0, 3.6)),

        # ------------------------------------------------------------
        # Upper partial walls.
        # These create another occluded region, but leave routes around
        # their ends so the robot must move to inspect the area.
        # ------------------------------------------------------------

        ((3.6, 4.0), (3.6, 5.35)),
        ((3.6, 5.35), (6.2, 5.35)),

        # Small freestanding wall to make the lower-left/open area less
        # trivial without blocking the starting pose at (1, 2).
        ((1.7, 0.9), (2.8, 0.9)),
    )

    return World(
        world.segments + extra_walls
    )
