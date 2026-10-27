"""Pure frontier-detection logic for autonomous exploration.

This module intentionally has no ROS dependencies so the map logic can be
unit-tested independently of Nav2, SLAM Toolbox, and the simulator.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable


UNKNOWN = -1


@dataclass(frozen=True)
class GridMap:
    """Minimal row-major occupancy grid.

    ROS OccupancyGrid convention:
      -1     unknown
       0     free
       100   occupied
    """

    width: int
    height: int
    data: tuple[int, ...]
    resolution_m: float = 0.05
    free_threshold: int = 20

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("occupancy data size does not match width * height")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def value(self, x: int, y: int) -> int:
        if not self.in_bounds(x, y):
            raise IndexError((x, y))
        return self.data[y * self.width + x]

    def is_free(self, x: int, y: int) -> bool:
        value = self.value(x, y)
        return value != UNKNOWN and value <= self.free_threshold

    def is_unknown(self, x: int, y: int) -> bool:
        return self.value(x, y) == UNKNOWN

    def is_obstacle(self, x: int, y: int) -> bool:
        value = self.value(x, y)
        return value != UNKNOWN and value > self.free_threshold


def _neighbors4(x: int, y: int) -> Iterable[tuple[int, int]]:
    yield x + 1, y
    yield x - 1, y
    yield x, y + 1
    yield x, y - 1


def inflated_obstacle_cells(
    grid: GridMap,
    clearance_m: float,
) -> set[tuple[int, int]]:
    """Return cells too close to known obstacles for the robot center.

    Unknown space is intentionally not inflated here. Unknown cells are already
    non-traversable, while their boundary with reachable free space is exactly
    what frontier exploration needs to detect.
    """

    if clearance_m < 0.0:
        raise ValueError("clearance_m must be non-negative")

    if clearance_m == 0.0:
        return set()

    radius_cells = math.ceil(clearance_m / grid.resolution_m)

    obstacles = [
        (x, y)
        for y in range(grid.height)
        for x in range(grid.width)
        if grid.is_obstacle(x, y)
    ]

    blocked: set[tuple[int, int]] = set()

    for ox, oy in obstacles:
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                x = ox + dx
                y = oy + dy

                if not grid.in_bounds(x, y):
                    continue

                distance_m = math.hypot(dx, dy) * grid.resolution_m

                if distance_m <= clearance_m + 1e-12:
                    blocked.add((x, y))

    return blocked


def reachable_free_cells(
    grid: GridMap,
    start: tuple[int, int],
    *,
    clearance_m: float = 0.0,
) -> set[tuple[int, int]]:
    """Flood-fill free space the robot can reach with the given clearance."""

    sx, sy = start

    if not grid.in_bounds(sx, sy):
        raise ValueError("robot start cell is outside the map")

    blocked = inflated_obstacle_cells(grid, clearance_m)

    if not grid.is_free(sx, sy) or (sx, sy) in blocked:
        return set()

    reachable: set[tuple[int, int]] = {(sx, sy)}
    queue = deque([(sx, sy)])

    while queue:
        x, y = queue.popleft()

        for nx, ny in _neighbors4(x, y):
            if not grid.in_bounds(nx, ny):
                continue
            if (nx, ny) in reachable:
                continue
            if (nx, ny) in blocked:
                continue
            if not grid.is_free(nx, ny):
                continue

            reachable.add((nx, ny))
            queue.append((nx, ny))

    return reachable


def find_frontier_cells(
    grid: GridMap,
    start: tuple[int, int],
    *,
    clearance_m: float = 0.0,
) -> set[tuple[int, int]]:
    """Return reachable free cells directly bordering unexplored space."""

    reachable = reachable_free_cells(
        grid,
        start,
        clearance_m=clearance_m,
    )

    frontiers: set[tuple[int, int]] = set()

    for x, y in reachable:
        # Use four-neighbor adjacency here so unknown space diagonally across a
        # wall corner does not create a false frontier.
        for nx, ny in _neighbors4(x, y):
            if grid.in_bounds(nx, ny) and grid.is_unknown(nx, ny):
                frontiers.add((x, y))
                break

    return frontiers


def _neighbors8(x: int, y: int) -> Iterable[tuple[int, int]]:
    """Yield all horizontal, vertical, and diagonal neighboring cells."""

    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            yield x + dx, y + dy


@dataclass(frozen=True)
class FrontierCluster:
    """A connected group of frontier cells representing one frontier."""

    cells: frozenset[tuple[int, int]]

    def __post_init__(self) -> None:
        if not self.cells:
            raise ValueError("frontier cluster cannot be empty")

    @property
    def size(self) -> int:
        return len(self.cells)

    @property
    def centroid(self) -> tuple[float, float]:
        """Return the cluster centroid in grid-cell coordinates."""

        count = len(self.cells)
        return (
            sum(x for x, _ in self.cells) / count,
            sum(y for _, y in self.cells) / count,
        )


def cluster_frontier_cells(
    frontier_cells: Iterable[tuple[int, int]],
    *,
    min_cluster_size: int = 1,
) -> tuple[FrontierCluster, ...]:
    """Group frontier cells using 8-connected components.

    Small components can be discarded to prevent isolated SLAM/map noise from
    becoming exploration goals.
    """

    if min_cluster_size < 1:
        raise ValueError("min_cluster_size must be >= 1")

    remaining = set(frontier_cells)
    clusters: list[FrontierCluster] = []

    while remaining:
        # Deterministic seed selection keeps tests and behavior reproducible.
        seed = min(remaining, key=lambda cell: (cell[1], cell[0]))
        remaining.remove(seed)

        component = {seed}
        queue = deque([seed])

        while queue:
            x, y = queue.popleft()

            for neighbor in _neighbors8(x, y):
                if neighbor not in remaining:
                    continue

                remaining.remove(neighbor)
                component.add(neighbor)
                queue.append(neighbor)

        if len(component) >= min_cluster_size:
            clusters.append(
                FrontierCluster(frozenset(component))
            )

    # Prefer larger frontiers first. The centroid tie-breakers make ordering
    # deterministic when two clusters have the same size.
    clusters.sort(
        key=lambda cluster: (
            -cluster.size,
            cluster.centroid[1],
            cluster.centroid[0],
        )
    )

    return tuple(clusters)


def find_frontier_clusters(
    grid: GridMap,
    start: tuple[int, int],
    *,
    clearance_m: float = 0.0,
    min_cluster_size: int = 1,
) -> tuple[FrontierCluster, ...]:
    """Find meaningful reachable frontier clusters."""

    frontier_cells = find_frontier_cells(
        grid,
        start,
        clearance_m=clearance_m,
    )

    return cluster_frontier_cells(
        frontier_cells,
        min_cluster_size=min_cluster_size,
    )


@dataclass(frozen=True)
class FrontierGoal:
    """A safe known-free approach point for one frontier."""

    cell: tuple[int, int]
    yaw_rad: float
    cluster_size: int
    frontier_centroid: tuple[float, float]
    unknown_centroid: tuple[float, float]


def _adjacent_unknown_cells(
    grid: GridMap,
    cluster: FrontierCluster,
) -> set[tuple[int, int]]:
    """Return unknown cells directly across a frontier."""

    unknown: set[tuple[int, int]] = set()

    for x, y in cluster.cells:
        for nx, ny in _neighbors4(x, y):
            if grid.in_bounds(nx, ny) and grid.is_unknown(nx, ny):
                unknown.add((nx, ny))

    return unknown


def choose_frontier_goal(
    grid: GridMap,
    start: tuple[int, int],
    cluster: FrontierCluster,
    *,
    clearance_m: float = 0.0,
    standoff_m: float = 0.20,
) -> FrontierGoal:
    """Choose a reachable known-free goal on the safe side of a frontier.

    The frontier-to-unknown direction estimates which way the robot should face.
    The desired goal is shifted backward from the frontier by ``standoff_m``.
    The actual returned cell is the reachable free cell nearest that desired
    point.
    """

    if standoff_m < 0.0:
        raise ValueError("standoff_m must be non-negative")

    unknown = _adjacent_unknown_cells(grid, cluster)

    if not unknown:
        raise ValueError("frontier cluster does not border unknown space")

    frontier_x, frontier_y = cluster.centroid

    unknown_x = sum(x for x, _ in unknown) / len(unknown)
    unknown_y = sum(y for _, y in unknown) / len(unknown)

    direction_x = unknown_x - frontier_x
    direction_y = unknown_y - frontier_y
    magnitude = math.hypot(direction_x, direction_y)

    if magnitude < 1e-12:
        # A highly symmetric frontier can make the centroid direction cancel.
        # Fall back to one deterministic frontier/unknown neighbor pair.
        pair = min(
            (
                (x, y, nx, ny)
                for x, y in cluster.cells
                for nx, ny in _neighbors4(x, y)
                if grid.in_bounds(nx, ny) and grid.is_unknown(nx, ny)
            ),
            key=lambda item: (item[1], item[0], item[3], item[2]),
        )

        x, y, nx, ny = pair
        direction_x = nx - x
        direction_y = ny - y
        magnitude = math.hypot(direction_x, direction_y)

    unit_x = direction_x / magnitude
    unit_y = direction_y / magnitude

    standoff_cells = standoff_m / grid.resolution_m

    desired_x = frontier_x - unit_x * standoff_cells
    desired_y = frontier_y - unit_y * standoff_cells

    reachable = reachable_free_cells(
        grid,
        start,
        clearance_m=clearance_m,
    )

    if not reachable:
        raise ValueError("robot has no reachable free cells")

    # Limit candidates to the neighborhood of this frontier. This prevents a
    # blocked standoff point from selecting an unrelated location elsewhere.
    search_radius_m = max(0.50, standoff_m + 0.25)
    search_radius_cells = search_radius_m / grid.resolution_m

    candidates = [
        cell
        for cell in reachable
        if math.hypot(
            cell[0] - frontier_x,
            cell[1] - frontier_y,
        ) <= search_radius_cells
    ]

    if not candidates:
        raise ValueError("no reachable goal cell exists near frontier")

    goal_cell = min(
        candidates,
        key=lambda cell: (
            math.hypot(cell[0] - desired_x, cell[1] - desired_y),
            math.hypot(cell[0] - frontier_x, cell[1] - frontier_y),
            cell[1],
            cell[0],
        ),
    )

    yaw_rad = math.atan2(direction_y, direction_x)

    return FrontierGoal(
        cell=goal_cell,
        yaw_rad=yaw_rad,
        cluster_size=cluster.size,
        frontier_centroid=cluster.centroid,
        unknown_centroid=(unknown_x, unknown_y),
    )


def make_frontier_goals(
    grid: GridMap,
    start: tuple[int, int],
    clusters: Iterable[FrontierCluster],
    *,
    clearance_m: float = 0.0,
    standoff_m: float = 0.20,
) -> tuple[FrontierGoal, ...]:
    """Create one safe exploration goal for each frontier cluster."""

    goals = []

    for cluster in clusters:
        goals.append(
            choose_frontier_goal(
                grid,
                start,
                cluster,
                clearance_m=clearance_m,
                standoff_m=standoff_m,
            )
        )

    return tuple(goals)


@dataclass(frozen=True)
class RankedFrontier:
    """A frontier goal with its exploration priority."""

    goal: FrontierGoal
    score: float
    path_distance_m: float
    information_gain_cells: int


def _grid_path_distances(
    grid: GridMap,
    start: tuple[int, int],
    *,
    clearance_m: float = 0.0,
) -> dict[tuple[int, int], int]:
    """Shortest 4-connected path distance, in cells, from the robot."""

    reachable = reachable_free_cells(
        grid,
        start,
        clearance_m=clearance_m,
    )

    if start not in reachable:
        return {}

    distances = {start: 0}
    queue = deque([start])

    while queue:
        x, y = queue.popleft()
        next_distance = distances[(x, y)] + 1

        for neighbor in _neighbors4(x, y):
            if neighbor not in reachable:
                continue
            if neighbor in distances:
                continue

            distances[neighbor] = next_distance
            queue.append(neighbor)

    return distances


def frontier_information_gain(
    grid: GridMap,
    cluster: FrontierCluster,
    *,
    radius_m: float = 0.50,
) -> int:
    """Estimate useful unexplored area near a frontier.

    Returns the number of unique unknown cells within ``radius_m`` of any
    frontier cell. This is only a ranking heuristic; Nav2 will later perform
    the authoritative path validation.
    """

    if radius_m <= 0.0:
        raise ValueError("radius_m must be positive")

    radius_cells = math.ceil(radius_m / grid.resolution_m)
    unknown_cells: set[tuple[int, int]] = set()

    for fx, fy in cluster.cells:
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if math.hypot(dx, dy) * grid.resolution_m > radius_m:
                    continue

                x = fx + dx
                y = fy + dy

                if grid.in_bounds(x, y) and grid.is_unknown(x, y):
                    unknown_cells.add((x, y))

    return len(unknown_cells)


def rank_frontiers(
    grid: GridMap,
    start: tuple[int, int],
    clusters: Iterable[FrontierCluster],
    *,
    clearance_m: float = 0.0,
    standoff_m: float = 0.20,
    information_radius_m: float = 0.50,
    distance_bias_m: float = 0.25,
) -> tuple[RankedFrontier, ...]:
    """Create and rank frontier goals from most to least useful."""

    if distance_bias_m <= 0.0:
        raise ValueError("distance_bias_m must be positive")

    distances = _grid_path_distances(
        grid,
        start,
        clearance_m=clearance_m,
    )

    ranked: list[RankedFrontier] = []

    for cluster in clusters:
        goal = choose_frontier_goal(
            grid,
            start,
            cluster,
            clearance_m=clearance_m,
            standoff_m=standoff_m,
        )

        distance_cells = distances.get(goal.cell)

        if distance_cells is None:
            continue

        path_distance_m = distance_cells * grid.resolution_m

        information_gain = frontier_information_gain(
            grid,
            cluster,
            radius_m=information_radius_m,
        )

        score = information_gain / (
            path_distance_m + distance_bias_m
        )

        ranked.append(
            RankedFrontier(
                goal=goal,
                score=score,
                path_distance_m=path_distance_m,
                information_gain_cells=information_gain,
            )
        )

    ranked.sort(
        key=lambda item: (
            -item.score,
            item.path_distance_m,
            -item.goal.cluster_size,
            item.goal.cell[1],
            item.goal.cell[0],
        )
    )

    return tuple(ranked)
