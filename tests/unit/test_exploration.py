import unittest

from lidar_robot.exploration import (
    GridMap,
    choose_frontier_goal,
    cluster_frontier_cells,
    find_frontier_cells,
    find_frontier_clusters,
    frontier_information_gain,
    rank_frontiers,
    reachable_free_cells,
)


def grid_from_text(
    text: str,
    *,
    resolution_m: float = 0.05,
) -> tuple[GridMap, tuple[int, int]]:
    """Create a GridMap from ASCII.

    # = occupied
    . = known free
    ? = unknown
    R = robot on known free space
    """

    lines = [
        line.strip()
        for line in text.strip().splitlines()
        if line.strip()
    ]

    width = len(lines[0])
    height = len(lines)

    if any(len(line) != width for line in lines):
        raise ValueError("all rows must have equal width")

    values = []
    robot = None

    for y, line in enumerate(lines):
        for x, char in enumerate(line):
            if char == "#":
                values.append(100)
            elif char == ".":
                values.append(0)
            elif char == "?":
                values.append(-1)
            elif char == "R":
                values.append(0)
                robot = (x, y)
            else:
                raise ValueError(f"unknown map character: {char!r}")

    if robot is None:
        raise ValueError("map must contain R")

    return (
        GridMap(
            width,
            height,
            tuple(values),
            resolution_m=resolution_m,
        ),
        robot,
    )


class FrontierTests(unittest.TestCase):
    def test_opening_into_unknown_space_creates_frontier(self):
        grid, robot = grid_from_text(
            """
            #######
            #R....#
            #.....#
            #...???
            #...???
            #######
            """
        )

        frontiers = find_frontier_cells(grid, robot)

        self.assertTrue(frontiers)

    def test_unknown_space_behind_complete_wall_is_not_frontier(self):
        grid, robot = grid_from_text(
            """
            #########
            #R......#
            #.......#
            #.#####.#
            #.#???.##
            #.#####.#
            #.......#
            #########
            """
        )

        frontiers = find_frontier_cells(grid, robot)

        self.assertEqual(frontiers, set())

    def test_narrow_doorway_is_rejected_with_robot_clearance(self):
        grid, robot = grid_from_text(
            """
            #################
            #...............#
            #...............#
            #...R...........#
            #...............#
            #.....#####.....#
            #.....#???#.....#
            #.....#???#.....#
            #.....##.##.....#
            #...............#
            #...............#
            #################
            """
        )

        # As a point robot, the one-cell doorway appears reachable.
        self.assertTrue(
            find_frontier_cells(grid, robot, clearance_m=0.0)
        )

        # At 5 cm/cell, a 10 cm clearance already closes this 5 cm doorway.
        self.assertEqual(
            find_frontier_cells(grid, robot, clearance_m=0.10),
            set(),
        )

    def test_wide_doorway_remains_reachable_with_clearance(self):
        grid, robot = grid_from_text(
            """
            #####################
            #...................#
            #...................#
            #...R...............#
            #...................#
            #.....#########.....#
            #.....#???????#.....#
            #.....#???????#.....#
            #.....##.....##.....#
            #...................#
            #...................#
            #...................#
            #...................#
            #...................#
            #####################
            """
        )

        frontiers = find_frontier_cells(
            grid,
            robot,
            clearance_m=0.10,
        )

        self.assertTrue(frontiers)



class FrontierClusterTests(unittest.TestCase):
    def test_connected_cells_form_one_cluster(self):
        cells = {
            (1, 1),
            (2, 1),
            (3, 1),
            (8, 8),
        }

        clusters = cluster_frontier_cells(
            cells,
            min_cluster_size=1,
        )

        self.assertEqual(
            [cluster.size for cluster in clusters],
            [3, 1],
        )

    def test_diagonal_frontier_cells_are_same_cluster(self):
        cells = {
            (1, 1),
            (2, 2),
            (3, 3),
        }

        clusters = cluster_frontier_cells(
            cells,
            min_cluster_size=1,
        )

        self.assertEqual(len(clusters), 1)
        self.assertEqual(clusters[0].size, 3)

    def test_single_cell_unknown_glitch_is_filtered(self):
        grid, robot = grid_from_text(
            """
            #########
            #R......#
            #.......#
            #...?...#
            #.......#
            #.......#
            #########
            """
        )

        unfiltered = find_frontier_clusters(
            grid,
            robot,
            min_cluster_size=1,
        )

        self.assertEqual(len(unfiltered), 1)
        self.assertEqual(unfiltered[0].size, 4)

        filtered = find_frontier_clusters(
            grid,
            robot,
            min_cluster_size=5,
        )

        self.assertEqual(filtered, ())

    def test_large_unexplored_boundary_survives_filter(self):
        grid, robot = grid_from_text(
            """
            ############
            #R.........#
            #..........#
            #.....?????#
            #.....?????#
            #.....?????#
            ############
            """
        )

        clusters = find_frontier_clusters(
            grid,
            robot,
            min_cluster_size=5,
        )

        self.assertTrue(clusters)
        self.assertGreaterEqual(clusters[0].size, 5)


class FrontierGoalTests(unittest.TestCase):
    def _example_frontier(self):
        grid, robot = grid_from_text(
            """
            ############
            #R......???#
            #.......???#
            #.......???#
            #.......???#
            ############
            """
        )

        clusters = find_frontier_clusters(
            grid,
            robot,
            min_cluster_size=3,
        )

        self.assertEqual(len(clusters), 1)
        return grid, robot, clusters[0]

    def test_goal_is_known_free_and_reachable(self):
        grid, robot, cluster = self._example_frontier()

        goal = choose_frontier_goal(
            grid,
            robot,
            cluster,
            standoff_m=0.10,
        )

        reachable = reachable_free_cells(grid, robot)

        self.assertTrue(grid.is_free(*goal.cell))
        self.assertIn(goal.cell, reachable)

    def test_goal_is_on_known_side_of_frontier(self):
        grid, robot, cluster = self._example_frontier()

        goal = choose_frontier_goal(
            grid,
            robot,
            cluster,
            standoff_m=0.10,
        )

        # Unknown space is to the right, so the approach goal should be left
        # of the frontier centroid.
        self.assertLess(
            goal.cell[0],
            cluster.centroid[0],
        )

    def test_goal_faces_toward_unknown_space(self):
        grid, robot, cluster = self._example_frontier()

        goal = choose_frontier_goal(
            grid,
            robot,
            cluster,
            standoff_m=0.10,
        )

        # Unknown space is directly to the right, which is approximately yaw=0.
        self.assertAlmostEqual(
            goal.yaw_rad,
            0.0,
            places=6,
        )


class FrontierRankingTests(unittest.TestCase):
    def test_information_gain_counts_unknown_space(self):
        grid, robot = grid_from_text(
            """
            #############
            #R.......???#
            #........???#
            #........???#
            #........???#
            #############
            """
        )

        clusters = find_frontier_clusters(
            grid,
            robot,
            min_cluster_size=3,
        )

        self.assertEqual(len(clusters), 1)

        gain = frontier_information_gain(
            grid,
            clusters[0],
            radius_m=0.30,
        )

        self.assertGreater(gain, 0)

    def test_rank_frontiers_returns_highest_score_first(self):
        grid, robot = grid_from_text(
            """
            #################
            #????...........#
            #????........???#
            #????........???#
            #....R.......???#
            #............???#
            #################
            """
        )

        clusters = find_frontier_clusters(
            grid,
            robot,
            min_cluster_size=2,
        )

        self.assertGreaterEqual(len(clusters), 2)

        ranked = rank_frontiers(
            grid,
            robot,
            clusters,
            standoff_m=0.05,
            information_radius_m=0.30,
        )

        self.assertGreaterEqual(len(ranked), 2)

        for first, second in zip(ranked, ranked[1:]):
            self.assertGreaterEqual(first.score, second.score)

    def test_path_distance_uses_free_space_not_straight_line(self):
        grid, robot = grid_from_text(
            """
            #################
            #R......#.....??#
            #.......#.....??#
            #.......#.....??#
            #.......#.....??#
            #.......#.....??#
            #.............??#
            #################
            """
        )

        clusters = find_frontier_clusters(
            grid,
            robot,
            min_cluster_size=2,
        )

        self.assertEqual(len(clusters), 1)

        ranked = rank_frontiers(
            grid,
            robot,
            clusters,
            standoff_m=0.05,
            information_radius_m=0.25,
        )

        self.assertEqual(len(ranked), 1)

        # The wall forces a detour, so the reachable path must be longer than
        # a simple straight-line estimate would suggest.
        gx, gy = ranked[0].goal.cell
        rx, ry = robot

        straight_line_m = (
            ((gx - rx) ** 2 + (gy - ry) ** 2) ** 0.5
            * grid.resolution_m
        )

        self.assertGreater(
            ranked[0].path_distance_m,
            straight_line_m,
        )


if __name__ == "__main__":
    unittest.main()
