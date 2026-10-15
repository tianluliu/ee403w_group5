"""Coverage-metric tests (pure Python). Expected counts are worked out by hand in comments."""

import dataclasses
import unittest

from _bootstrap import REPO_ROOT  # noqa: F401  (sets sys.path)
from lidar_robot import metrics as m
from lidar_robot.metrics import Grid

# 7 wide x 6 high. Interior 5x4 = 20 cells, one of them a box => 19 free cells.
ROOM = ["#######",
        "#.....#",
        "#.....#",
        "#..#..#",
        "#.....#",
        "#######"]
START = (1, 1)   # bottom-left interior cell (row 0 of the grid is the BOTTOM text row)

# 7 wide x 4 high. Interior wall at x=3 splits the room: 2x2 free on the start side.
SPLIT = ["#######",
         "#..#..#",
         "#..#..#",
         "#######"]

# 7x7 with a 5x5 interior = 25 free cells.
SQUARE = ["#######"] + ["#.....#"] * 5 + ["#######"]

# 10 free cells (5x2): lets coverage hit exactly 0.9.
STRIP = ["#######",
         "#.....#",
         "#.....#",
         "#######"]


def with_changes(rows, changes, char):
    """Copy of ascii rows with chars replaced at grid cells (x, y), y counted from the bottom."""
    out = [list(r) for r in rows]
    for x, y in changes:
        out[len(rows) - 1 - y][x] = char
    return ["".join(r) for r in out]


class GridTests(unittest.TestCase):
    def test_ascii_orientation_first_row_is_top(self):
        g = Grid.from_ascii(["#.", "?."])
        self.assertEqual((g.width, g.height), (2, 2))
        self.assertEqual((g.value(0, 1), g.value(1, 1)), (100, 0))     # top row
        self.assertEqual((g.value(0, 0), g.value(1, 0)), (-1, 0))      # bottom row

    def test_ascii_errors(self):
        for rows in ([], ["##", "#"], ["#x"]):
            with self.subTest(rows=rows):
                with self.assertRaises(ValueError):
                    Grid.from_ascii(rows)

    def test_validation(self):
        base = dict(width=2, height=1, resolution=0.05, data=(0, 0))
        Grid(**base)
        for override in ({"data": (0,)}, {"data": (0, 101)}, {"data": (0, -2)}, {"data": (0, True)},
                         {"data": (0, 0.5)}, {"resolution": 0.0}, {"resolution": float("nan")},
                         {"width": 0, "data": ()}, {"origin_x": float("inf")}):
            with self.subTest(override=override):
                with self.assertRaises(ValueError):
                    Grid(**{**base, **override})

    def test_out_of_bounds_access(self):
        g = Grid.from_ascii(["..", ".."])
        for xy in ((-1, 0), (0, -1), (2, 0), (0, 2)):
            with self.assertRaises(IndexError):
                g.value(*xy)

    def test_world_cell_conversion_with_origin(self):
        g = Grid(width=4, height=4, resolution=0.5, data=(0,) * 16, origin_x=-1.0, origin_y=-1.0)
        self.assertEqual(g.world_to_cell(0.1, 0.1), (2, 2))
        self.assertEqual(g.world_to_cell(-1.0, -1.0), (0, 0))          # cell edge belongs to the cell above
        self.assertEqual(g.world_to_cell(0.0, 0.0), (2, 2))
        self.assertIsNone(g.world_to_cell(-1.01, 0.0))
        self.assertIsNone(g.world_to_cell(1.0, 0.0))                   # x = origin + width*res is outside
        self.assertEqual(g.cell_to_world(2, 2), (0.25, 0.25))

    def test_geometry_comparison(self):
        a = Grid.from_ascii(["..", ".."], 0.05)
        self.assertTrue(a.same_geometry(Grid.from_ascii(["##", "##"], 0.05)))
        self.assertFalse(a.same_geometry(Grid.from_ascii(["..", ".."], 0.10)))
        self.assertFalse(a.same_geometry(Grid.from_ascii(["...", "..."], 0.05)))
        self.assertFalse(a.same_geometry(Grid.from_ascii(["..", ".."], 0.05, origin=(0.5, 0.0))))


class CellStateTests(unittest.TestCase):
    def test_default_thresholds(self):
        cases = {-1: m.UNKNOWN, 0: m.FREE, 25: m.FREE, 26: m.UNKNOWN, 50: m.UNKNOWN,
                 64: m.UNKNOWN, 65: m.OCCUPIED, 100: m.OCCUPIED}
        for value, expected in cases.items():
            with self.subTest(value=value):
                self.assertEqual(m.cell_state(value), expected)

    def test_custom_thresholds(self):
        self.assertEqual(m.cell_state(40, free_max=40), m.FREE)
        self.assertEqual(m.cell_state(40, occupied_min=40), m.OCCUPIED)


class AccessibleCellsTests(unittest.TestCase):
    def test_room_with_a_box_has_19_accessible_cells(self):
        self.assertEqual(len(m.accessible_cells(Grid.from_ascii(ROOM), START)), 19)

    def test_a_wall_splits_the_room_and_only_the_start_side_counts(self):
        cells = m.accessible_cells(Grid.from_ascii(SPLIT), START)
        self.assertEqual(cells, frozenset({(1, 1), (2, 1), (1, 2), (2, 2)}))

    def test_unknown_cells_block_movement(self):
        g = Grid.from_ascii(["#####", "#.?.#", "#####"])
        self.assertEqual(m.accessible_cells(g, (1, 1)), frozenset({(1, 1)}))

    def test_inflation_shrinks_the_accessible_area(self):
        g = Grid.from_ascii(SQUARE)
        self.assertEqual(len(m.accessible_cells(g, (3, 3), 0.0)), 25)
        self.assertEqual(len(m.accessible_cells(g, (3, 3), 1.0)), 9)    # 3x3 core
        self.assertEqual(len(m.accessible_cells(g, (3, 3), 1.5)), 9)    # diagonals add nothing here
        self.assertEqual(m.accessible_cells(g, (3, 3), 2.0), frozenset({(3, 3)}))

    def test_start_must_be_traversable(self):
        g = Grid.from_ascii(SQUARE)
        for start, inflate in (((0, 0), 0.0), ((10, 10), 0.0), ((-1, 2), 0.0), ((1, 1), 1.0)):
            with self.subTest(start=start, inflate=inflate):
                with self.assertRaises(ValueError):
                    m.accessible_cells(g, start, inflate)
        with self.assertRaises(ValueError):
            m.accessible_cells(Grid.from_ascii(["#####", "#?..#", "#####"]), (1, 1))   # start unknown

    def test_bad_inflation_values(self):
        g = Grid.from_ascii(SQUARE)
        for bad in (-0.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                m.accessible_cells(g, (3, 3), bad)


class CoverageTests(unittest.TestCase):
    def setUp(self):
        self.truth = Grid.from_ascii(ROOM)
        self.free_cells = sorted(m.accessible_cells(self.truth, START))

    def robot_missing(self, cells):
        return Grid.from_ascii(with_changes(ROOM, cells, "?"))

    def test_perfect_map(self):
        r = m.coverage(self.truth, self.truth, START)
        self.assertEqual((r.accessible, r.covered, r.fraction), (19, 19, 1.0))
        self.assertTrue(r.meets())

    def test_one_missing_cell_still_passes_two_do_not(self):
        r1 = m.coverage(self.robot_missing(self.free_cells[:1]), self.truth, START)
        self.assertEqual((r1.covered, r1.accessible), (18, 19))                # 0.947
        self.assertTrue(r1.meets(0.90))
        r2 = m.coverage(self.robot_missing(self.free_cells[:2]), self.truth, START)
        self.assertEqual(r2.covered, 17)                                       # 0.8947
        self.assertFalse(r2.meets(0.90))

    def test_exact_boundary_nine_of_ten_meets_ninety_percent(self):
        truth = Grid.from_ascii(STRIP)
        cells = sorted(m.accessible_cells(truth, START))
        self.assertEqual(len(cells), 10)
        nine = m.coverage(Grid.from_ascii(with_changes(STRIP, cells[:1], "?")), truth, START)
        self.assertEqual((nine.covered, nine.fraction), (9, 0.9))
        self.assertTrue(nine.meets(0.90))
        eight = m.coverage(Grid.from_ascii(with_changes(STRIP, cells[:2], "?")), truth, START)
        self.assertFalse(eight.meets(0.90))

    def test_empty_robot_map_covers_nothing(self):
        empty = Grid(self.truth.width, self.truth.height, self.truth.resolution,
                     (-1,) * len(self.truth.data))
        r = m.coverage(empty, self.truth, START)
        self.assertEqual((r.covered, r.fraction), (0, 0.0))
        self.assertFalse(r.meets())

    def test_wrongly_occupied_or_ambiguous_cells_are_not_covered(self):
        data = list(self.truth.data)
        x, y = self.free_cells[0]
        data[y * self.truth.width + x] = 100            # robot thinks a free cell is a wall
        x2, y2 = self.free_cells[1]
        data[y2 * self.truth.width + x2] = 50           # ambiguous probability
        robot = dataclasses.replace(self.truth, data=tuple(data))
        self.assertEqual(m.coverage(robot, self.truth, START).covered, 17)

    def test_unreachable_regions_do_not_count_either_way(self):
        truth = Grid.from_ascii(SPLIT)
        full = m.coverage(truth, truth, START)
        self.assertEqual((full.accessible, full.covered), (4, 4))              # right side ignored
        robot = Grid.from_ascii(with_changes(SPLIT, [(4, 1), (5, 1), (4, 2), (5, 2)], "?"))
        self.assertEqual(m.coverage(robot, truth, START).fraction, 1.0)        # unknown pocket is fine

    def test_inflation_changes_the_denominator(self):
        truth = Grid.from_ascii(SQUARE)
        robot = Grid.from_ascii(with_changes(SQUARE, [(1, 1), (1, 2), (2, 1)], "?"))   # near the wall
        self.assertAlmostEqual(m.coverage(robot, truth, (3, 3), 0.0).fraction, 22 / 25)
        self.assertEqual(m.coverage(robot, truth, (3, 3), 1.0).fraction, 1.0)          # only 3x3 core

    def test_geometry_mismatch_is_an_error(self):
        other = Grid.from_ascii(STRIP)
        with self.assertRaises(ValueError):
            m.coverage(other, self.truth, START)
        shifted = Grid(self.truth.width, self.truth.height, self.truth.resolution, self.truth.data, 1.0, 0.0)
        with self.assertRaises(ValueError):
            m.coverage(shifted, self.truth, START)

    def test_target_validation(self):
        r = m.coverage(self.truth, self.truth, START)
        for bad in (0, -0.1, 1.01):
            with self.assertRaises(ValueError):
                r.meets(bad)
        self.assertTrue(r.meets(1.0))


if __name__ == "__main__":
    unittest.main()
