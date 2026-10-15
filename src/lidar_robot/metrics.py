"""Map-coverage metric for acceptance test T-COV (pure Python).

Definitions (PROPOSED in docs/requirements.md, pending team ratification):
  accessible area = free cells of the ground-truth grid that a robot of the configured
                    footprint can reach from the start cell (4-connected; obstacles inflated);
  coverage        = accessible cells that are known-free in the robot's map / accessible cells.

Grids use nav_msgs/OccupancyGrid semantics: row-major, cell (x, y) at index y*width + x, y
increasing upward, values -1 (unknown) or 0..100 (occupancy probability). This module only
measures maps; it does not build them.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import FrozenSet, Optional, Sequence, Set, Tuple

FREE, OCCUPIED, UNKNOWN = "free", "occupied", "unknown"
Cell = Tuple[int, int]


def cell_state(value: int, free_max: int = 25, occupied_min: int = 65) -> str:
    """Classify a cell. Ambiguous probabilities count as UNKNOWN, never as free."""
    if value < 0:
        return UNKNOWN
    if value <= free_max:
        return FREE
    if value >= occupied_min:
        return OCCUPIED
    return UNKNOWN


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    data: Tuple[int, ...]
    origin_x: float = 0.0
    origin_y: float = 0.0

    def __post_init__(self) -> None:
        if not (isinstance(self.width, int) and isinstance(self.height, int)
                and self.width > 0 and self.height > 0):
            raise ValueError("width and height must be positive integers")
        for name in ("resolution", "origin_x", "origin_y"):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")
        if not self.resolution > 0:
            raise ValueError("resolution must be > 0")
        if len(self.data) != self.width * self.height:
            raise ValueError(f"data has {len(self.data)} cells, expected {self.width * self.height}")
        if any((not isinstance(v, int)) or isinstance(v, bool) or v < -1 or v > 100 for v in self.data):
            raise ValueError("cell values must be integers within -1..100")

    @classmethod
    def from_ascii(cls, rows: Sequence[str], resolution: float = 0.05,
                   origin: Tuple[float, float] = (0.0, 0.0)) -> "Grid":
        """Build from text, first row = TOP of the map. '#'=occupied, '.'=free, '?'=unknown."""
        if not rows or len({len(r) for r in rows}) != 1:
            raise ValueError("rows must be non-empty and equally long")
        values = {"#": 100, ".": 0, "?": -1}
        flat = []
        for row in reversed(rows):            # row 0 of the grid is the bottom (lowest y)
            for ch in row:
                if ch not in values:
                    raise ValueError(f"unsupported map character {ch!r}")
                flat.append(values[ch])
        return cls(len(rows[0]), len(rows), resolution, tuple(flat), origin[0], origin[1])

    def value(self, x: int, y: int) -> int:
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise IndexError(f"cell ({x}, {y}) outside {self.width}x{self.height} grid")
        return self.data[y * self.width + x]

    def world_to_cell(self, wx: float, wy: float) -> Optional[Cell]:
        cx = math.floor((wx - self.origin_x) / self.resolution)
        cy = math.floor((wy - self.origin_y) / self.resolution)
        if 0 <= cx < self.width and 0 <= cy < self.height:
            return (cx, cy)
        return None

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        """Centre of the cell."""
        return (self.origin_x + (cx + 0.5) * self.resolution,
                self.origin_y + (cy + 0.5) * self.resolution)

    def same_geometry(self, other: "Grid") -> bool:
        return (self.width == other.width and self.height == other.height
                and math.isclose(self.resolution, other.resolution, rel_tol=0, abs_tol=1e-12)
                and math.isclose(self.origin_x, other.origin_x, rel_tol=0, abs_tol=1e-9)
                and math.isclose(self.origin_y, other.origin_y, rel_tol=0, abs_tol=1e-9))


def accessible_cells(truth: Grid, start: Cell, inflate_cells: float = 0.0) -> FrozenSet[Cell]:
    """Free truth cells reachable from ``start``; unknown cells block, obstacles are inflated."""
    if inflate_cells < 0 or not math.isfinite(inflate_cells):
        raise ValueError("inflate_cells must be a finite number >= 0")
    reach = int(math.floor(inflate_cells))
    offsets = [(dx, dy) for dx in range(-reach, reach + 1) for dy in range(-reach, reach + 1)
               if dx * dx + dy * dy <= inflate_cells * inflate_cells]
    blocked: Set[Cell] = set()
    for y in range(truth.height):
        for x in range(truth.width):
            if cell_state(truth.value(x, y)) == OCCUPIED:
                for dx, dy in offsets:
                    blocked.add((x + dx, y + dy))

    def traversable(c: Cell) -> bool:
        x, y = c
        return (0 <= x < truth.width and 0 <= y < truth.height
                and c not in blocked and cell_state(truth.value(x, y)) == FREE)

    if not traversable(start):
        raise ValueError(f"start cell {start} is not a traversable free cell of the truth map")
    seen = {start}
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        for nxt in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if nxt not in seen and traversable(nxt):
                seen.add(nxt)
                queue.append(nxt)
    return frozenset(seen)


@dataclass(frozen=True)
class CoverageResult:
    accessible: int
    covered: int

    @property
    def fraction(self) -> float:
        return self.covered / self.accessible

    def meets(self, target: float = 0.90) -> bool:
        if not 0 < target <= 1:
            raise ValueError("target must be within (0, 1]")
        return self.covered >= target * self.accessible - 1e-9


def coverage(robot_map: Grid, truth: Grid, start: Cell, inflate_cells: float = 0.0) -> CoverageResult:
    """Fraction of the truth map's accessible area that ``robot_map`` shows as known-free."""
    if not robot_map.same_geometry(truth):
        raise ValueError("robot map and truth map must have identical size, resolution and origin")
    accessible = accessible_cells(truth, start, inflate_cells)
    covered = sum(1 for (x, y) in accessible if cell_state(robot_map.value(x, y)) == FREE)
    return CoverageResult(accessible=len(accessible), covered=covered)
