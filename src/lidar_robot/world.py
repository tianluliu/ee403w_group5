"""Minimal 2D environment for unit tests: walls and boxes as line segments + ray casting.

Not a physics engine and not a mapper. It exists so the mock LiDAR can return
deterministic, hand-checkable ranges.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

Point = Tuple[float, float]
Segment = Tuple[Point, Point]

_PARALLEL_EPS = 1e-12
_EDGE_EPS = 1e-9  # keeps rays from slipping through shared corners


def _rect_segments(x_min: float, y_min: float, x_max: float, y_max: float) -> Tuple[Segment, ...]:
    for name, v in (("x_min", x_min), ("y_min", y_min), ("x_max", x_max), ("y_max", y_max)):
        if not math.isfinite(v):
            raise ValueError(f"{name} must be finite")
    if not (x_max > x_min and y_max > y_min):
        raise ValueError("rectangle must have x_max > x_min and y_max > y_min")
    a, b, c, d = (x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)
    return ((a, b), (b, c), (c, d), (d, a))


@dataclass(frozen=True)
class World:
    segments: Tuple[Segment, ...] = ()

    @classmethod
    def room(cls, x_min: float, y_min: float, x_max: float, y_max: float) -> "World":
        return cls(_rect_segments(x_min, y_min, x_max, y_max))

    def with_box(self, x_min: float, y_min: float, x_max: float, y_max: float) -> "World":
        """New World with an additional rectangular obstacle (original is unchanged)."""
        return World(self.segments + _rect_segments(x_min, y_min, x_max, y_max))

    def raycast(self, x: float, y: float, angle: float, max_range: float = math.inf) -> Optional[float]:
        """Distance to the nearest segment along the ray, or None if nothing within max_range."""
        for name, v in (("x", x), ("y", y), ("angle", angle)):
            if not math.isfinite(v):
                raise ValueError(f"{name} must be finite")
        if math.isnan(max_range) or max_range < 0:
            raise ValueError("max_range must be >= 0")
        dx, dy = math.cos(angle), math.sin(angle)
        best: Optional[float] = None
        for (ax, ay), (bx, by) in self.segments:
            ex, ey = bx - ax, by - ay
            denom = dx * ey - dy * ex
            if abs(denom) < _PARALLEL_EPS:
                continue  # parallel / collinear: no single intersection
            wx, wy = ax - x, ay - y
            t = (wx * ey - wy * ex) / denom
            s = (wx * dy - wy * dx) / denom
            if t >= 0.0 and -_EDGE_EPS <= s <= 1.0 + _EDGE_EPS and (best is None or t < best):
                best = t
        if best is None or best > max_range:
            return None
        return best
