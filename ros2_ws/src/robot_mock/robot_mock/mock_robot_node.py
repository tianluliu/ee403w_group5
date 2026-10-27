"""Dynamic ROS 2 simulator for the LiDAR mapping robot.

The control path intentionally includes the existing mock Uno controller:

/cmd_vel
 -> MotorBridge
 -> simulated serial LoopbackTransport
 -> MockMotorController (Elegoo Uno R3 stand-in)
 -> left/right duty
 -> Simulator2D
 -> /odom, TF and moving /scan
"""

from dataclasses import replace
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import (
    Point,
    PoseStamped,
    TransformStamped,
    Twist as RosTwist,
)
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from lidar_robot.bridge import MotorBridge
from lidar_robot.clock import SystemClock
from lidar_robot.config import load_config
from lidar_robot.kinematics import Twist
from lidar_robot.mock_lidar import MockLidar, Pose2D
from lidar_robot.mock_mcu import MockMotorController
from lidar_robot.simulator import Simulator2D
from lidar_robot.transport import LoopbackTransport

from .worlds import build_demo_world


def yaw_quaternion(yaw):
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


class MockRobotNode(Node):
    def __init__(self):
        super().__init__("mock_robot")

        self.declare_parameter("config_path", "")
        self.declare_parameter("pose_x", 1.0)
        self.declare_parameter("pose_y", 2.0)
        self.declare_parameter("pose_yaw", 0.0)

        config_path = str(self.get_parameter("config_path").value)
        if not config_path:
            raise RuntimeError("config_path is required")

        cfg = load_config(config_path)

        if not cfg.is_mock:
            raise RuntimeError("mock_robot_node refuses non-mock configuration")

        # The real config defaults to motors disabled. Only this in-memory copy is
        # enabled so MotorBridge can exercise its normal drive path in simulation.
        sim_cfg = replace(
            cfg,
            safety=replace(cfg.safety, motors_enabled=True),
        )

        self._cfg = sim_cfg
        self._world = build_demo_world()

        initial_pose = Pose2D(
            float(self.get_parameter("pose_x").value),
            float(self.get_parameter("pose_y").value),
            float(self.get_parameter("pose_yaw").value),
        )

        self._sim = Simulator2D(sim_cfg, self._world, initial_pose)
        self._lidar = MockLidar(sim_cfg.lidar, self._world)

        # ----- simulated Elegoo Uno R3 control path -----------------------------
        self._mono_clock = SystemClock()
        self._mcu = MockMotorController.from_config(sim_cfg, self._mono_clock)
        self._transport = LoopbackTransport(self._mcu)
        self._bridge = MotorBridge(self._transport, sim_cfg, self._mono_clock)

        info = self._bridge.connect()

        # Keep the simulated MCU DISARMED during ROS/Nav2 startup. Arming here
        # can trip the MCU link watchdog if the Pi is briefly busy configuring
        # the navigation stack. Each independent motion session arms explicitly.
        #
        # A watchdog failure during an active motion session is not automatically
        # recovered; a later explicit idle boundary is required before a new
        # motion session may arm again.
        self._motion_session_active = False

        self.get_logger().info(
            f"Simulated Uno connected: firmware={info.fw_id}, "
            f"state={self._bridge.mcu_state}"
        )

        # ----- ROS publishers/subscriber ----------------------------------------
        self._cmd_sub = self.create_subscription(
            RosTwist,
            "/cmd_vel",
            self._on_cmd_vel,
            10,
        )

        self._odom_pub = self.create_publisher(Odometry, "/odom", 20)

        self._scan_pub = self.create_publisher(
            LaserScan,
            "/scan",
            qos_profile_sensor_data,
        )

        self._path_pub = self.create_publisher(Path, "/sim/path", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray,
            "/sim/world",
            10,
        )

        self._tf = TransformBroadcaster(self)
        self._tf_static = StaticTransformBroadcaster(self)

        self._path = Path()
        self._path.header.frame_id = "odom"

        self._latest_cmd = Twist()
        self._latest_cmd_ms = None

        now_ms = self._mono_clock.now_ms()
        self._last_physics_ms = now_ms
        self._last_path_ms = now_ms

        # Simulation physics: 50 Hz
        self._physics_timer = self.create_timer(0.02, self._physics_tick)

        # Pi -> Uno command scheduling: matches config, currently 10 Hz.
        self._control_timer = self.create_timer(
            sim_cfg.limits.cmd_period_ms / 1000.0,
            self._control_tick,
        )

        # LiDAR rate from config, currently 10 Hz.
        self._scan_timer = self.create_timer(
            1.0 / sim_cfg.lidar.scan_hz,
            self._scan_tick,
        )

        self._marker_timer = self.create_timer(0.5, self._publish_markers)

        self._publish_laser_static_tf()
        self._publish_markers()

        self.get_logger().info(
            "Dynamic simulator ready: /cmd_vel -> simulated Uno -> drivetrain -> "
            "/odom + /scan"
        )

    def _on_cmd_vel(self, msg):
        self._latest_cmd = Twist(
            linear=float(msg.linear.x),
            angular=float(msg.angular.z),
        )
        self._latest_cmd_ms = self._mono_clock.now_ms()

    def _control_tick(self):
        now = self._mono_clock.now_ms()

        fresh = (
            self._latest_cmd_ms is not None
            and now - self._latest_cmd_ms < self._cfg.limits.cmd_ttl_ms
        )

        motion_requested = (
            abs(self._latest_cmd.linear) > 1e-9
            or abs(self._latest_cmd.angular) > 1e-9
        )

        try:
            # Each explicit motion session begins with ARM.  A fresh zero Twist
            # ends the session, allowing a later independent navigation goal to
            # arm again.
            #
            # A stale command does NOT end the motion session.  Therefore, if
            # the MCU watchdog disarms during active motion, subsequent drive
            # commands in that same session cannot silently re-arm it.
            if fresh and motion_requested:
                if not self._motion_session_active:
                    self._bridge.arm()
                    self._motion_session_active = True
                    self.get_logger().info(
                        "Simulated Uno armed for new motion session"
                    )

                self._bridge.send_twist(self._latest_cmd)

            elif fresh:
                # A fresh zero velocity command is an explicit idle boundary.
                self._bridge.stop()

                if self._motion_session_active:
                    self.get_logger().info(
                        "Simulated motion session ended"
                    )

                self._motion_session_active = False

            else:
                # Stale command: stop safely, but do not make this a new
                # session.  A watchdog failure during active motion remains
                # latched until an explicit zero command ends the session.
                self._bridge.stop()

        except Exception as exc:
            self._bridge.stop()
            self.get_logger().error(f"drive command failed safely: {exc}")

    def _physics_tick(self):
        now_ms = self._mono_clock.now_ms()
        dt = (now_ms - self._last_physics_ms) / 1000.0
        self._last_physics_ms = now_ms

        if dt <= 0.0:
            return

        # Avoid a giant physics jump after debugger pauses/system stalls.
        dt = min(dt, 0.1)

        self._mcu.tick()
        left, right = self._mcu.outputs

        actual = self._sim.step(left, right, dt)

        self._publish_odom(actual)
        self._publish_tf()
        self._update_path(now_ms)

    def _publish_odom(self, actual):
        now = self.get_clock().now()
        pose = self._sim.pose

        qx, qy, qz, qw = yaw_quaternion(pose.yaw)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = actual.linear
        msg.twist.twist.angular.z = actual.angular

        self._odom_pub.publish(msg)

    def _publish_tf(self):
        now = self.get_clock().now()
        pose = self._sim.pose
        _, _, qz, qw = yaw_quaternion(pose.yaw)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = pose.x
        tf.transform.translation.y = pose.y
        tf.transform.translation.z = 0.0

        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self._tf.sendTransform(tf)

    def _publish_laser_static_tf(self):
        lidar = self._cfg.lidar

        _, _, qz, qw = yaw_quaternion(lidar.mount_yaw_rad)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = lidar.frame_id

        tf.transform.translation.x = lidar.mount_x_m
        tf.transform.translation.y = lidar.mount_y_m
        tf.transform.translation.z = lidar.mount_z_m

        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self._tf_static.sendTransform(tf)

    def _scan_tick(self):
        now = self.get_clock().now()

        data = self._lidar.scan(
            self._sim.pose,
            now.nanoseconds // 1_000_000,
        )

        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = data.frame_id

        msg.angle_min = data.angle_min
        msg.angle_max = data.angle_max
        msg.angle_increment = data.angle_increment
        msg.time_increment = data.time_increment
        msg.scan_time = data.scan_time
        msg.range_min = data.range_min
        msg.range_max = data.range_max
        msg.ranges = list(data.ranges)

        self._scan_pub.publish(msg)

    def _update_path(self, now_ms):
        if now_ms - self._last_path_ms < 100:
            return

        self._last_path_ms = now_ms

        pose = self._sim.pose
        _, _, qz, qw = yaw_quaternion(pose.yaw)

        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = "odom"

        p.pose.position.x = pose.x
        p.pose.position.y = pose.y
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw

        self._path.poses.append(p)

        # Keep visualization bounded.
        if len(self._path.poses) > 3000:
            self._path.poses = self._path.poses[-3000:]

        self._path.header.stamp = p.header.stamp
        self._path_pub.publish(self._path)

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()

        walls = Marker()
        walls.header.stamp = now
        walls.header.frame_id = "odom"
        walls.ns = "sim_world"
        walls.id = 0
        walls.type = Marker.LINE_LIST
        walls.action = Marker.ADD
        walls.pose.orientation.w = 1.0
        walls.scale.x = 0.03
        walls.color.r = 0.8
        walls.color.g = 0.8
        walls.color.b = 0.8
        walls.color.a = 1.0

        for (ax, ay), (bx, by) in self._world.segments:
            a = Point()
            a.x = ax
            a.y = ay
            a.z = 0.02

            b = Point()
            b.x = bx
            b.y = by
            b.z = 0.02

            walls.points.extend([a, b])

        body = Marker()
        body.header.stamp = now
        body.header.frame_id = "base_link"
        body.ns = "sim_robot"
        body.id = 1
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.orientation.w = 1.0
        body.pose.position.z = 0.04

        diameter = 2.0 * self._cfg.body.footprint_radius_m
        body.scale.x = diameter
        body.scale.y = diameter
        body.scale.z = 0.08

        body.color.r = 0.2
        body.color.g = 0.7
        body.color.b = 0.3
        body.color.a = 0.8

        heading = Marker()
        heading.header.stamp = now
        heading.header.frame_id = "base_link"
        heading.ns = "sim_robot"
        heading.id = 2
        heading.type = Marker.ARROW
        heading.action = Marker.ADD

        start = Point()
        start.z = 0.09

        end = Point()
        end.x = self._cfg.body.footprint_radius_m * 1.5
        end.z = 0.09

        heading.points = [start, end]

        heading.scale.x = 0.025
        heading.scale.y = 0.06
        heading.scale.z = 0.08

        heading.color.r = 1.0
        heading.color.g = 0.5
        heading.color.b = 0.0
        heading.color.a = 1.0

        self._marker_pub.publish(MarkerArray(markers=[walls, body, heading]))

    def destroy_node(self):
        try:
            self._bridge.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MockRobotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
