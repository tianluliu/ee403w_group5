"""Frontier-based autonomous exploration node.

The explorer:
  1. reads the live SLAM map,
  2. finds and ranks reachable frontiers,
  3. waits for the best candidate to persist,
  4. validates the candidate with Nav2,
  5. sends one NavigateToPose goal,
  6. waits for that goal to finish,
  7. lets the map settle, then repeats.

Autonomous motion is disabled by default and must be enabled explicitly.
"""

from __future__ import annotations

import copy
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from lidar_robot.exploration import (
    GridMap,
    find_frontier_clusters,
    rank_frontiers,
)


def _quaternion_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _yaw_to_quaternion(yaw: float):
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


class ExplorerNode(Node):
    SEARCHING = "SEARCHING"
    VALIDATING = "VALIDATING"
    NAVIGATING = "NAVIGATING"
    SETTLING = "SETTLING"
    COMPLETE = "COMPLETE"

    def __init__(self) -> None:
        super().__init__("explorer")

        self.declare_parameter("autonomous", False)

        self.declare_parameter("clearance_m", 0.18)
        self.declare_parameter("min_cluster_size", 5)
        self.declare_parameter("standoff_m", 0.20)
        self.declare_parameter("information_radius_m", 0.50)

        self.declare_parameter("decision_period_s", 1.0)
        self.declare_parameter("persistence_checks", 3)
        self.declare_parameter("candidate_tolerance_m", 0.20)

        self.declare_parameter("settle_time_s", 2.0)
        self.declare_parameter("failure_suppression_s", 20.0)
        self.declare_parameter("suppression_radius_m", 0.35)

        self.declare_parameter("completion_checks", 5)
        self.declare_parameter(
            "completion_max_known_growth_cells",
            5,
        )

        self._autonomous = bool(
            self.get_parameter("autonomous").value
        )

        self._clearance_m = float(
            self.get_parameter("clearance_m").value
        )
        self._min_cluster_size = int(
            self.get_parameter("min_cluster_size").value
        )
        self._standoff_m = float(
            self.get_parameter("standoff_m").value
        )
        self._information_radius_m = float(
            self.get_parameter("information_radius_m").value
        )

        decision_period_s = float(
            self.get_parameter("decision_period_s").value
        )
        self._persistence_checks = int(
            self.get_parameter("persistence_checks").value
        )
        self._candidate_tolerance_m = float(
            self.get_parameter("candidate_tolerance_m").value
        )

        self._settle_time_s = float(
            self.get_parameter("settle_time_s").value
        )
        self._failure_suppression_s = float(
            self.get_parameter("failure_suppression_s").value
        )
        self._suppression_radius_m = float(
            self.get_parameter("suppression_radius_m").value
        )

        self._completion_checks = int(
            self.get_parameter("completion_checks").value
        )
        self._completion_max_known_growth_cells = int(
            self.get_parameter(
                "completion_max_known_growth_cells"
            ).value
        )

        if self._persistence_checks < 1:
            raise ValueError("persistence_checks must be >= 1")

        if self._completion_checks < 1:
            raise ValueError("completion_checks must be >= 1")

        if self._completion_max_known_growth_cells < 0:
            raise ValueError(
                "completion_max_known_growth_cells must be >= 0"
            )

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._map = None

        self._complete_pub = self.create_publisher(
            Bool,
            "/exploration/complete",
            map_qos,
        )

        initial_complete = Bool()
        initial_complete.data = False
        self._complete_pub.publish(initial_complete)

        self._map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            map_qos,
        )

        self._goal_pub = self.create_publisher(
            PoseStamped,
            "/exploration/goal_preview",
            10,
        )

        self._marker_pub = self.create_publisher(
            MarkerArray,
            "/exploration/markers",
            10,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
        )

        self._path_client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose",
        )

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        self._state = self.SEARCHING

        self._candidate_pose = None
        self._candidate_count = 0

        self._latched_goal = None

        # (world_x, world_y, monotonic_expiry)
        self._suppressed = []

        self._resume_after = 0.0
        self._last_summary = None

        self._completion_count = 0
        self._last_known_count = None

        self._timer = self.create_timer(
            decision_period_s,
            self._evaluate,
        )

        mode = "AUTONOMOUS" if self._autonomous else "DRY-RUN"

        self.get_logger().info(
            f"Explorer ready in {mode} mode; "
            "waiting for /map and map->base_link TF"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    @staticmethod
    def _world_to_grid(
        msg: OccupancyGrid,
        world_x: float,
        world_y: float,
    ) -> tuple[int, int]:
        origin = msg.info.origin
        origin_yaw = _quaternion_to_yaw(origin.orientation)

        dx = world_x - origin.position.x
        dy = world_y - origin.position.y

        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)

        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy

        return (
            math.floor(local_x / msg.info.resolution),
            math.floor(local_y / msg.info.resolution),
        )

    @staticmethod
    def _grid_to_world(
        msg: OccupancyGrid,
        cell: tuple[int, int],
    ) -> tuple[float, float]:
        gx, gy = cell

        local_x = (gx + 0.5) * msg.info.resolution
        local_y = (gy + 0.5) * msg.info.resolution

        origin = msg.info.origin
        origin_yaw = _quaternion_to_yaw(origin.orientation)

        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)

        return (
            origin.position.x + c * local_x - s * local_y,
            origin.position.y + s * local_x + c * local_y,
        )

    def _ranked_to_pose(
        self,
        msg: OccupancyGrid,
        ranked,
    ) -> PoseStamped:
        x, y = self._grid_to_world(
            msg,
            ranked.goal.cell,
        )

        origin_yaw = _quaternion_to_yaw(
            msg.info.origin.orientation
        )
        yaw = origin_yaw + ranked.goal.yaw_rad

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = _yaw_to_quaternion(yaw)

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def _fresh_pose(self, pose: PoseStamped) -> PoseStamped:
        fresh = copy.deepcopy(pose)
        fresh.header.stamp = self.get_clock().now().to_msg()
        return fresh

    def _poses_close(
        self,
        first: PoseStamped,
        second: PoseStamped,
    ) -> bool:
        dx = first.pose.position.x - second.pose.position.x
        dy = first.pose.position.y - second.pose.position.y

        return math.hypot(dx, dy) <= self._candidate_tolerance_m

    def _cleanup_suppressed(self) -> None:
        now = time.monotonic()

        self._suppressed = [
            item
            for item in self._suppressed
            if item[2] > now
        ]

    def _is_suppressed(self, pose: PoseStamped) -> bool:
        self._cleanup_suppressed()

        x = pose.pose.position.x
        y = pose.pose.position.y

        return any(
            math.hypot(x - sx, y - sy)
            <= self._suppression_radius_m
            for sx, sy, _ in self._suppressed
        )

    def _reset_candidate(self) -> None:
        self._candidate_pose = None
        self._candidate_count = 0

    def _evaluate(self) -> None:
        if self._state == self.COMPLETE:
            return

        if self._state in (
            self.VALIDATING,
            self.NAVIGATING,
        ):
            return

        if self._state == self.SETTLING:
            if time.monotonic() < self._resume_after:
                return

            self._state = self.SEARCHING
            self.get_logger().info(
                "Map settling period complete; searching again"
            )

        msg = self._map

        if msg is None:
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                "map",
                "base_link",
                Time(),
            )
        except Exception:
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        robot_cell = self._world_to_grid(
            msg,
            robot_x,
            robot_y,
        )

        grid = GridMap(
            width=msg.info.width,
            height=msg.info.height,
            data=tuple(msg.data),
            resolution_m=msg.info.resolution,
        )

        if not grid.in_bounds(*robot_cell):
            self.get_logger().warning(
                f"Robot cell {robot_cell} is outside occupancy grid"
            )
            return

        clusters = find_frontier_clusters(
            grid,
            robot_cell,
            clearance_m=self._clearance_m,
            min_cluster_size=self._min_cluster_size,
        )

        ranked = rank_frontiers(
            grid,
            robot_cell,
            clusters,
            clearance_m=self._clearance_m,
            standoff_m=self._standoff_m,
            information_radius_m=self._information_radius_m,
        )

        known_count = sum(
            1
            for value in msg.data
            if value != -1
        )

        usable = []

        for item in ranked:
            pose = self._ranked_to_pose(msg, item)

            if not self._is_suppressed(pose):
                usable.append((item, pose))

        self._publish_markers(
            msg,
            clusters,
            usable,
        )

        if not usable:
            self._handle_no_usable_frontiers(
                len(clusters),
                known_count,
            )
            return

        self._completion_count = 0
        self._last_known_count = known_count

        best, pose = usable[0]

        self._goal_pub.publish(pose)

        summary = (
            f"Frontiers={len(usable)}; "
            f"best=({pose.pose.position.x:.2f}, "
            f"{pose.pose.position.y:.2f}); "
            f"score={best.score:.1f}; "
            f"path={best.path_distance_m:.2f} m; "
            f"gain={best.information_gain_cells} cells"
        )

        if summary != self._last_summary:
            self.get_logger().info(summary)
            self._last_summary = summary

        if not self._autonomous:
            return

        if (
            self._candidate_pose is not None
            and self._poses_close(self._candidate_pose, pose)
        ):
            self._candidate_pose = pose
            self._candidate_count += 1
        else:
            self._candidate_pose = pose
            self._candidate_count = 1

        self.get_logger().info(
            f"Candidate persistence "
            f"{self._candidate_count}/{self._persistence_checks}"
        )

        if self._candidate_count < self._persistence_checks:
            return

        self._latched_goal = copy.deepcopy(pose)
        self._reset_candidate()

        self.get_logger().info(
            "Frontier persisted; latching goal "
            f"({pose.pose.position.x:.2f}, "
            f"{pose.pose.position.y:.2f})"
        )

        self._validate_latched_goal()

    def _handle_no_usable_frontiers(
        self,
        cluster_count: int,
        known_count: int,
    ) -> None:
        """Confirm exploration completion conservatively."""

        self._reset_candidate()
        self._cleanup_suppressed()

        # Frontiers still exist, but none is currently usable. This is not
        # completion: failed/suppressed candidates must be reconsidered.
        if cluster_count > 0:
            self._completion_count = 0
            self._last_known_count = known_count

            summary = (
                f"No usable frontiers; "
                f"clusters={cluster_count}; "
                "not declaring completion"
            )

            if summary != self._last_summary:
                self.get_logger().info(summary)
                self._last_summary = summary

            return

        # If a previously failed frontier is still under temporary
        # suppression, wait until its retry period expires.
        if self._suppressed:
            self._completion_count = 0
            self._last_known_count = known_count

            summary = (
                "No current frontier clusters, but failed "
                "frontiers are still awaiting retry"
            )

            if summary != self._last_summary:
                self.get_logger().info(summary)
                self._last_summary = summary

            return

        if self._last_known_count is None:
            known_growth = 0
            self._completion_count = 1
        else:
            known_growth = abs(
                known_count - self._last_known_count
            )

            if (
                known_growth
                <= self._completion_max_known_growth_cells
            ):
                self._completion_count += 1
            else:
                self._completion_count = 1

        self._last_known_count = known_count

        summary = (
            "No reachable frontiers; "
            f"completion check "
            f"{self._completion_count}/"
            f"{self._completion_checks}; "
            f"known-space change={known_growth} cells"
        )

        if summary != self._last_summary:
            self.get_logger().info(summary)
            self._last_summary = summary

        if self._completion_count < self._completion_checks:
            return

        self._state = self.COMPLETE

        complete = Bool()
        complete.data = True
        self._complete_pub.publish(complete)

        self.get_logger().info(
            "EXPLORATION COMPLETE: no persistent reachable "
            "frontiers and known map area is stable"
        )

    def _validate_latched_goal(self) -> None:
        if self._latched_goal is None:
            return

        if not self._path_client.wait_for_server(
            timeout_sec=0.0
        ):
            self.get_logger().warning(
                "ComputePathToPose server is not ready"
            )
            self._state = self.SETTLING
            self._resume_after = time.monotonic() + 1.0
            return

        self._state = self.VALIDATING

        request = ComputePathToPose.Goal()
        request.goal = self._fresh_pose(
            self._latched_goal
        )
        request.use_start = False

        self.get_logger().info(
            "Validating frontier with Nav2 planner"
        )

        future = self._path_client.send_goal_async(
            request
        )
        future.add_done_callback(
            self._on_path_goal_response
        )

    def _on_path_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_latched_goal(
                f"path request failed: {exc}"
            )
            return

        if not goal_handle.accepted:
            self._fail_latched_goal(
                "planner rejected path request"
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self._on_path_result
        )

    def _on_path_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self._fail_latched_goal(
                f"path result failed: {exc}"
            )
            return

        path = wrapped.result.path

        if (
            wrapped.status
            != GoalStatus.STATUS_SUCCEEDED
            or not path.poses
        ):
            self._fail_latched_goal(
                "Nav2 could not produce a usable path"
            )
            return

        self.get_logger().info(
            f"Path validated with {len(path.poses)} poses"
        )

        self._start_navigation()

    def _start_navigation(self) -> None:
        if self._latched_goal is None:
            self._state = self.SEARCHING
            return

        if not self._nav_client.wait_for_server(
            timeout_sec=0.0
        ):
            self._fail_latched_goal(
                "NavigateToPose server is not ready"
            )
            return

        request = NavigateToPose.Goal()
        request.pose = self._fresh_pose(
            self._latched_goal
        )

        self._state = self.NAVIGATING

        self.get_logger().info(
            "Sending autonomous NavigateToPose goal "
            f"({request.pose.pose.position.x:.2f}, "
            f"{request.pose.pose.position.y:.2f})"
        )

        future = self._nav_client.send_goal_async(
            request
        )
        future.add_done_callback(
            self._on_nav_goal_response
        )

    def _on_nav_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_latched_goal(
                f"navigation request failed: {exc}"
            )
            return

        if not goal_handle.accepted:
            self._fail_latched_goal(
                "NavigateToPose rejected goal"
            )
            return

        self.get_logger().info(
            "Autonomous navigation goal accepted"
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self._on_nav_result
        )

    def _on_nav_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self._fail_latched_goal(
                f"navigation result failed: {exc}"
            )
            return

        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            if self._latched_goal is not None:
                x = self._latched_goal.pose.position.x
                y = self._latched_goal.pose.position.y

                self.get_logger().info(
                    "Exploration goal succeeded at "
                    f"({x:.2f}, {y:.2f})"
                )

            self._latched_goal = None
            self._state = self.SETTLING
            self._resume_after = (
                time.monotonic()
                + self._settle_time_s
            )
            return

        error_code = getattr(
            wrapped.result,
            "error_code",
            "unknown",
        )

        self._fail_latched_goal(
            f"navigation failed with "
            f"status={wrapped.status}, "
            f"error_code={error_code}"
        )

    def _fail_latched_goal(self, reason: str) -> None:
        self.get_logger().warning(
            f"Frontier goal failed: {reason}"
        )

        if self._latched_goal is not None:
            x = self._latched_goal.pose.position.x
            y = self._latched_goal.pose.position.y

            self._suppressed.append(
                (
                    x,
                    y,
                    time.monotonic()
                    + self._failure_suppression_s,
                )
            )

            self.get_logger().warning(
                f"Temporarily suppressing frontier near "
                f"({x:.2f}, {y:.2f}) for "
                f"{self._failure_suppression_s:.0f} s"
            )

        self._latched_goal = None
        self._reset_candidate()

        self._state = self.SETTLING
        self._resume_after = (
            time.monotonic()
            + self._settle_time_s
        )

    def _publish_markers(
        self,
        msg: OccupancyGrid,
        clusters,
        usable,
    ) -> None:
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        now = self.get_clock().now().to_msg()

        frontier_marker = Marker()
        frontier_marker.header.frame_id = "map"
        frontier_marker.header.stamp = now
        frontier_marker.ns = "frontiers"
        frontier_marker.id = 0
        frontier_marker.type = Marker.POINTS
        frontier_marker.action = Marker.ADD

        frontier_marker.scale.x = (
            msg.info.resolution * 0.8
        )
        frontier_marker.scale.y = (
            msg.info.resolution * 0.8
        )

        frontier_marker.color.r = 1.0
        frontier_marker.color.g = 0.5
        frontier_marker.color.b = 0.0
        frontier_marker.color.a = 0.9

        for cluster in clusters:
            for cell in cluster.cells:
                x, y = self._grid_to_world(
                    msg,
                    cell,
                )

                point = Point()
                point.x = x
                point.y = y
                point.z = 0.03

                frontier_marker.points.append(
                    point
                )

        markers.markers.append(frontier_marker)

        if usable:
            _, best_pose = usable[0]

            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = now
            goal_marker.ns = "goal_preview"
            goal_marker.id = 1
            goal_marker.type = Marker.ARROW
            goal_marker.action = Marker.ADD

            goal_marker.pose = best_pose.pose

            goal_marker.pose.position.z = 0.08

            goal_marker.scale.x = 0.30
            goal_marker.scale.y = 0.07
            goal_marker.scale.z = 0.07

            goal_marker.color.r = 0.0
            goal_marker.color.g = 1.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0

            markers.markers.append(goal_marker)

        self._marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ExplorerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
