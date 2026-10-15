"""TF-chain validation (pure Python; no ROS needed).

Models the required chain  map -> odom -> base_link -> laser  and the rules in
docs/interfaces.md: one parent per frame, no cycles, exactly one publisher per edge, and a
static laser mount. ``Transform`` follows REP-103 and means "pose of the child frame
expressed in the parent frame".
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Tuple

from .config import IncompleteHardwareConfig, Lidar, RobotConfig

MAP, ODOM, BASE_LINK, LASER = "map", "odom", "base_link", "laser"
REQUIRED_CHAIN: Tuple[str, ...] = (MAP, ODOM, BASE_LINK, LASER)

# Sole publisher of each edge on the real robot (docs/interfaces.md section 2.1).
HARDWARE_OWNERS: Mapping[Tuple[str, str], str] = {
    (MAP, ODOM): "slam_toolbox",
    (ODOM, BASE_LINK): "odometry_source",
    (BASE_LINK, LASER): "robot_description",
}

MOCK_OWNER = "mock_tf"


class TfError(ValueError):
    """The transform tree violates a rule."""


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Transform:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    def __post_init__(self) -> None:
        for name in ("x", "y", "z", "yaw"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise TfError(f"transform {name} must be a finite number, got {value!r}")

    def compose(self, other: "Transform") -> "Transform":
        """self * other: ``other`` is expressed in self's frame; result is in self's parent."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Transform(self.x + c * other.x - s * other.y,
                         self.y + s * other.x + c * other.y,
                         self.z + other.z,
                         _wrap(self.yaw + other.yaw))

    def inverse(self) -> "Transform":
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Transform(-(c * self.x + s * self.y), -(-s * self.x + c * self.y),
                         -self.z, _wrap(-self.yaw))


@dataclass(frozen=True)
class TfEdge:
    parent: str
    child: str
    transform: Transform
    owner: str
    static: bool = False


class TfTree:
    def __init__(self) -> None:
        self._by_child: Dict[str, TfEdge] = {}

    def add(self, edge: TfEdge) -> None:
        if not edge.parent or not edge.child or edge.parent == edge.child:
            raise TfError(f"invalid edge {edge.parent!r} -> {edge.child!r}")
        if not edge.owner:
            raise TfError("every edge needs an owner (its single publisher)")
        existing = self._by_child.get(edge.child)
        if existing is not None:
            if existing.parent != edge.parent:
                raise TfError(f"frame {edge.child!r} would have two parents: "
                              f"{existing.parent!r} and {edge.parent!r}")
            if existing.owner != edge.owner:
                raise TfError(f"edge {edge.parent!r} -> {edge.child!r} has two publishers: "
                              f"{existing.owner!r} and {edge.owner!r}")
        elif edge.child in self.ancestors(edge.parent):
            raise TfError(f"edge {edge.parent!r} -> {edge.child!r} would create a cycle")
        self._by_child[edge.child] = edge

    def parent_of(self, frame: str) -> Optional[str]:
        edge = self._by_child.get(frame)
        return edge.parent if edge else None

    def ancestors(self, frame: str) -> List[str]:
        """[frame, parent, grandparent, ..., root]."""
        chain = [frame]
        while chain[-1] in self._by_child:
            chain.append(self._by_child[chain[-1]].parent)
            if len(chain) > len(self._by_child) + 1:   # defensive; add() prevents cycles
                raise TfError("cycle detected")
        return chain

    def _pose_in_root(self, frame: str) -> Tuple[str, Transform]:
        path = self.ancestors(frame)
        pose = Transform()
        for child in reversed(path[:-1]):
            pose = pose.compose(self._by_child[child].transform)
        return path[-1], pose

    def lookup(self, target: str, source: str) -> Transform:
        """Pose of ``source`` expressed in ``target`` (the TF 'lookup_transform' sense)."""
        root_t, pose_t = self._pose_in_root(target)
        root_s, pose_s = self._pose_in_root(source)
        if root_t != root_s:
            raise TfError(f"no connection between {target!r} and {source!r}")
        return pose_t.inverse().compose(pose_s)

    def validate_required_chain(self, expected_owners: Optional[Mapping[Tuple[str, str], str]] = None) -> None:
        for parent, child in zip(REQUIRED_CHAIN, REQUIRED_CHAIN[1:]):
            edge = self._by_child.get(child)
            if edge is None:
                raise TfError(f"missing required edge {parent!r} -> {child!r}")
            if edge.parent != parent:
                raise TfError(f"frame {child!r} must be a child of {parent!r}, found parent {edge.parent!r}")
            if expected_owners is not None and edge.owner != expected_owners[(parent, child)]:
                raise TfError(f"edge {parent!r} -> {child!r} must be published by "
                              f"{expected_owners[(parent, child)]!r}, found {edge.owner!r}")
        if not self._by_child[LASER].static:
            raise TfError("base_link -> laser must be a static transform")


def laser_mount_transform(lidar: Lidar) -> Transform:
    """base_link -> laser from configuration. Missing mount values are an error, never defaulted."""
    missing = [f"lidar.{k}" for k in ("mount_x_m", "mount_y_m", "mount_z_m", "mount_yaw_rad")
               if getattr(lidar, k) is None]
    if missing:
        raise IncompleteHardwareConfig("navigation", missing)
    return Transform(lidar.mount_x_m, lidar.mount_y_m, lidar.mount_z_m, lidar.mount_yaw_rad)


def mock_tf_edges(cfg: RobotConfig) -> List[TfEdge]:
    """Static identity map/odom/base_link chain for the mock graph. Mock configs ONLY."""
    if not cfg.is_mock:
        raise TfError("mock transforms may only be built from a mock configuration")
    return [
        TfEdge(MAP, ODOM, Transform(), MOCK_OWNER, static=True),
        TfEdge(ODOM, BASE_LINK, Transform(), MOCK_OWNER, static=True),
        TfEdge(BASE_LINK, LASER, laser_mount_transform(cfg.lidar), MOCK_OWNER, static=True),
    ]


def static_publisher_args(edge: TfEdge) -> List[str]:
    """Arguments for tf2_ros static_transform_publisher (named-flag form)."""
    t = edge.transform
    return ["--x", repr(float(t.x)), "--y", repr(float(t.y)), "--z", repr(float(t.z)),
            "--yaw", repr(float(t.yaw)), "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", edge.parent, "--child-frame-id", edge.child]
