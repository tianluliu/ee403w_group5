"""Real CDR mixed-TF filtering contract; not a full replay validation."""
import unittest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src'))
from lidar_robot.slam_replay import retained_transforms
try:
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped
    from rclpy.serialization import serialize_message, deserialize_message
except ImportError:
    TFMessage = None


@unittest.skipIf(TFMessage is None, 'ROS 2 message packages unavailable')
class ReplayCDRTests(unittest.TestCase):
    def test_mixed_tf_round_trip_keeps_simulator_stamp(self):
        transforms = []
        for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.header.stamp.sec, t.header.stamp.nanosec = 123, 456
            t.transform.rotation.w = 1.
            transforms.append(t)
        msg = deserialize_message(serialize_message(TFMessage(transforms=transforms)), TFMessage)
        msg.transforms = retained_transforms('/tf', msg.transforms)
        out = deserialize_message(serialize_message(msg), TFMessage)
        self.assertEqual(len(out.transforms), 1)
        self.assertEqual(out.transforms[0], transforms[1])


@unittest.skipIf(TFMessage is None, 'ROS 2 message packages unavailable')
class ReplayBagTests(unittest.TestCase):
    def test_prepare_preserves_source_and_retained_bag_sequence(self):
        import contextlib
        import io
        import tempfile
        from unittest.mock import patch
        import rosbag2_py as rb
        from geometry_msgs.msg import Point
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from visualization_msgs.msg import Marker, MarkerArray
        root = Path(__file__).resolve().parents[2]
        sys.path.insert(0, str(root/'scripts'))
        import prepare_slam_replay as prepare
        from lidar_robot.slam_replay import digest_tree
        with tempfile.TemporaryDirectory() as tmp:
            source, output = Path(tmp)/'source', Path(tmp)/'prepared'
            writer = rb.SequentialWriter()
            writer.open(rb.StorageOptions(uri=str(source), storage_id='mcap'), rb.ConverterOptions('', ''))
            types = {'/scan': 'sensor_msgs/msg/LaserScan', '/odom': 'nav_msgs/msg/Odometry',
                     '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage',
                     '/sim/world': 'visualization_msgs/msg/MarkerArray'}
            for topic, kind in types.items():
                writer.create_topic(rb.TopicMetadata(id=0, name=topic, type=kind, serialization_format='cdr'))
            dynamic, static, slam = TransformStamped(), TransformStamped(), TransformStamped()
            for t, parent, child in [(dynamic, 'odom', 'base_link'), (static, 'base_link', 'laser'), (slam, 'map', 'odom')]:
                t.header.frame_id, t.child_frame_id = parent, child
                t.header.stamp.sec, t.header.stamp.nanosec = 123, 456
                t.transform.rotation.w = 1.
            wall = Marker()
            wall.ns, wall.header.frame_id, wall.pose.orientation.w = 'sim_world', 'odom', 1.
            wall.points = [Point(x=0.), Point(x=1.)]
            inputs = [('/tf_static', TFMessage(transforms=[static])),
                      ('/tf', TFMessage(transforms=[slam, dynamic])),
                      ('/scan', LaserScan()), ('/odom', Odometry()),
                      ('/sim/world', MarkerArray(markers=[wall])),
                      ('/tf', TFMessage(transforms=[slam]))]
            for topic, msg in inputs:
                writer.write(topic, serialize_message(msg), 123_000_000_000)
            del writer
            before = digest_tree(source)
            with patch.object(prepare, 'require_pi'), patch.object(sys, 'argv',
                    ['prepare', '--source', str(source), '--output', str(output),
                     '--config', str(root/'config/slam_toolbox_sim.yaml')]), contextlib.redirect_stdout(io.StringIO()):
                prepare.main()
            self.assertEqual(digest_tree(source), before)
            rows = list(prepare.messages(output/'input'))
            self.assertEqual([r[0] for r in rows], ['/tf_static', '/tf', '/scan', '/odom'])
            self.assertTrue(all(r[2] == 123_000_000_000 for r in rows))
            self.assertEqual(rows[0][1].transforms, [static])
            self.assertEqual(rows[1][1].transforms, [dynamic])
            self.assertEqual(serialize_message(rows[2][1]), serialize_message(inputs[2][1]))
