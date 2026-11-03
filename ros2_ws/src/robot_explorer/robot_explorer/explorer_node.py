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
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Log
from .tf_diagnostics import pose_fields, result_fields, stamp_ns, tf_snapshot
from lidar_robot.benchmark import BenchmarkLog
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
        self.declare_parameter("benchmark_path", "")
        self.declare_parameter("benchmark_label", "")
        self._benchmark = None
        self.declare_parameter("benchmark_tf_diagnostics", False)
        self._tf_diagnostics = bool(self.get_parameter("benchmark_tf_diagnostics").value)
        self._attempt_id = 0
        self._diagnostic_odom_stamp = None
        self._diagnostic_plan = None

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

        benchmark_path = self.get_parameter("benchmark_path").value
        if self._tf_diagnostics and not benchmark_path:
            raise ValueError("benchmark_tf_diagnostics requires benchmark_path")
        if benchmark_path:
            if not self._autonomous:
                raise ValueError("benchmark_path requires autonomous:=true")
            metadata = {name: self.get_parameter(name).value for name in (
                "clearance_m", "min_cluster_size", "standoff_m", "information_radius_m",
                "decision_period_s", "persistence_checks", "candidate_tolerance_m",
                "settle_time_s", "failure_suppression_s", "suppression_radius_m",
                "completion_checks", "completion_max_known_growth_cells", "benchmark_label",
                "benchmark_tf_diagnostics", "use_sim_time")}
            self._benchmark = BenchmarkLog(benchmark_path, time.monotonic(), metadata)
            self._benchmark_odom = self.create_subscription(
                Odometry, "/odom", self._on_benchmark_odom, 100)

        if self._tf_diagnostics:
            self._diagnostic_timer = self.create_timer(1.0, lambda: self._diagnostic_snapshot())
            self._diagnostic_rosout = self.create_subscription(Log, "/rosout", self._on_diagnostic_log, 100)
            self._diagnostic_path = self.create_subscription(Path, "/plan", self._on_diagnostic_plan, 1)
            self._decision_period_s = decision_period_s
            self._evaluate_due = time.monotonic() + decision_period_s
            self._last_evaluate_start = None

        self._timer = self.create_timer(
            decision_period_s,
            self._evaluate_timed if self._tf_diagnostics else self._evaluate,
        )

        mode = "AUTONOMOUS" if self._autonomous else "DRY-RUN"

        self.get_logger().info(
            f"Explorer ready in {mode} mode; "
            "waiting for /map and map->base_link TF"
        )

    def _observe(self, name, **fields):
        if self._benchmark is None:
            return
        try:
            if getattr(self, "_tf_diagnostics", False):
                fields.setdefault("attempt_id", self._attempt_id)
                fields.setdefault("ros_now_ns", self.get_clock().now().nanoseconds)
            self._benchmark.event(name, time.monotonic(), **fields)
        except Exception as exc:
            self.get_logger().error(f"Benchmark recording failed: {exc}")
            try:
                self._benchmark.stream.close()
            except OSError:
                pass
            self._benchmark = None

    def _set_state(self, state, reason):
        previous = self._state
        self._state = state
        if getattr(self, "_tf_diagnostics", False) and previous != state:
            self._observe("state_transition", previous=previous, state=state, reason=reason)

    def _evaluate_timed(self):
        started = time.monotonic()
        due = self._evaluate_due
        interval = None if self._last_evaluate_start is None else started - self._last_evaluate_start
        self._last_evaluate_start = started
        # Estimate the nominal periodic deadline, skipping elapsed periods.
        self._evaluate_due += max(1, math.floor((started - due) / self._decision_period_s) + 1) * self._decision_period_s
        try:
            self._evaluate()
        finally:
            self._observe("evaluator_timing", duration_s=time.monotonic() - started,
                          estimated_lateness_s=max(0.0, started - due), interval_s=interval,
                          state=self._state)

    def _on_diagnostic_plan(self, msg):
        if msg.poses:
            self._diagnostic_plan = {
                "frame_id": msg.header.frame_id, "stamp_ns": stamp_ns(msg.header.stamp),
                "endpoint_stamp_ns": stamp_ns(msg.poses[-1].header.stamp),
                "received_ros_ns": self.get_clock().now().nanoseconds}

    def _on_diagnostic_log(self, msg):
        if msg.level >= 30 and any(name in msg.name for name in (
                "controller_server", "bt_navigator", "slam_toolbox", "local_costmap", "global_costmap")):
            self._observe("nav2_log", logger=msg.name, level=msg.level,
                          source_stamp_ns=stamp_ns(msg.stamp), message=msg.msg)

    def _diagnostic_snapshot(self, requested_stamp_ns=None, reason="periodic"):
        if self._benchmark is None:
            return
        try:
            now = self.get_clock().now()
            def lookup(parent, child, when):
                return self._tf_buffer.lookup_transform(
                    parent, child, Time(nanoseconds=when, clock_type=now.clock_type))
            snapshot = tf_snapshot(lookup, now.nanoseconds,
                                   now.nanoseconds if requested_stamp_ns is None else requested_stamp_ns,
                                   self._diagnostic_odom_stamp, self._diagnostic_plan)
            self._observe("tf_snapshot", reason=reason, state=self._state, **snapshot)
        except Exception as exc:
            self._observe("diagnostic_error", error=str(exc))

    def _on_benchmark_odom(self, msg):
        if self._benchmark is not None:
            if self._tf_diagnostics:
                self._diagnostic_odom_stamp = stamp_ns(msg.header.stamp)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._benchmark.metrics.odometry(
                msg.pose.pose.position.x, msg.pose.pose.position.y, stamp)

    def finish_benchmark(self, outcome):
        if self._benchmark is not None:
            self._observe(outcome)
            if self._benchmark is not None:
                try:
                    self._benchmark.close(time.monotonic(), self._state)
                except Exception as exc:
                    self.get_logger().error(f"Benchmark summary failed: {exc}")
                finally:
                    self._benchmark = None

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

            self._set_state(self.SEARCHING, "settle_elapsed")
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

        if self._benchmark is not None and self._benchmark.metrics.started is None:
            self._observe("ready")

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
            else:
                self._observe("suppressed_skip")

        self._observe("evaluation", clusters=len(clusters), usable=len(usable),
                      known_cells=known_count, state=self._state)

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
            if self._candidate_pose is not None:
                self._observe("candidate_replacement")
            self._candidate_pose = pose
            self._candidate_count = 1

        self.get_logger().info(
            f"Candidate persistence "
            f"{self._candidate_count}/{self._persistence_checks}"
        )

        if self._candidate_count < self._persistence_checks:
            return

        self._attempt_id += 1
        self._observe("latched", x=pose.pose.position.x, y=pose.pose.position.y)
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

        self._set_state(self.COMPLETE, "completion_checks_passed")
        self.finish_benchmark("complete")

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
            self._set_state(self.SETTLING, "planner_unavailable")
            self._observe("planner_unavailable")
            self._resume_after = time.monotonic() + 1.0
            return

        self._set_state(self.VALIDATING, "planner_request")

        request = ComputePathToPose.Goal()
        request.goal = self._fresh_pose(
            self._latched_goal
        )
        request.use_start = False

        self.get_logger().info(
            "Validating frontier with Nav2 planner"
        )

        self._observe("planner_request")
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

        self._observe("planner_result", status=wrapped.status, **result_fields(wrapped.result))
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
            self._set_state(self.SEARCHING, "no_latched_goal")
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

        self._set_state(self.NAVIGATING, "navigation_request")

        self.get_logger().info(
            "Sending autonomous NavigateToPose goal "
            f"({request.pose.pose.position.x:.2f}, "
            f"{request.pose.pose.position.y:.2f})"
        )

        if self._tf_diagnostics:
            self._observe("navigation_start", goal=pose_fields(request.pose))
            self._diagnostic_snapshot(stamp_ns(request.pose.header.stamp), reason="navigation_start")
        self._observe("navigation_request")
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

        self._observe("navigation_accepted")
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

        self._observe("navigation_result", status=wrapped.status, **result_fields(wrapped.result))
        if self._tf_diagnostics:
            self._diagnostic_snapshot(reason="navigation_result")
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            if self._latched_goal is not None:
                x = self._latched_goal.pose.position.x
                y = self._latched_goal.pose.position.y

                self.get_logger().info(
                    "Exploration goal succeeded at "
                    f"({x:.2f}, {y:.2f})"
                )

            self._latched_goal = None
            self._set_state(self.SETTLING, "navigation_succeeded")
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
        self._observe("failure", reason=reason, phase=self._state)
        self.get_logger().warning(
            f"Frontier goal failed: {reason}"
        )

        if self._latched_goal is not None:
            x = self._latched_goal.pose.position.x
            y = self._latched_goal.pose.position.y

            self._observe("suppression", x=x, y=y,
                          expires=time.monotonic() + self._failure_suppression_s)
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

        self._set_state(self.SETTLING, "goal_failure")
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

    outcome = "interrupted"
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        outcome = "error"
        raise
    finally:
        node.finish_benchmark(outcome)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
