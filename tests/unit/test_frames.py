"""TF-chain validator tests (pure Python)."""

import dataclasses
import math
import random
import unittest

import _bootstrap
from lidar_robot import frames as f
from lidar_robot.config import IncompleteHardwareConfig, Meta, load_config
from lidar_robot.frames import TfEdge, TfError, TfTree, Transform

MOCK = load_config(_bootstrap.CONFIG_DIR / "robot.mock.toml")
REAL = load_config(_bootstrap.CONFIG_DIR / "robot.toml")


def hardware_tree(map_odom=Transform(), odom_base=Transform(), laser=Transform(0.05, 0.0, 0.15, 0.0)):
    tree = TfTree()
    tree.add(TfEdge(f.MAP, f.ODOM, map_odom, "slam_toolbox"))
    tree.add(TfEdge(f.ODOM, f.BASE_LINK, odom_base, "odometry_source"))
    tree.add(TfEdge(f.BASE_LINK, f.LASER, laser, "robot_description", static=True))
    return tree


class TransformTests(unittest.TestCase):
    def assertTf(self, t, x, y, z, yaw, places=9):
        self.assertAlmostEqual(t.x, x, places=places)
        self.assertAlmostEqual(t.y, y, places=places)
        self.assertAlmostEqual(t.z, z, places=places)
        self.assertAlmostEqual(t.yaw, yaw, places=places)

    def test_identity_is_neutral(self):
        t = Transform(1.0, -2.0, 0.3, 0.7)
        self.assertTf(Transform().compose(t), 1.0, -2.0, 0.3, 0.7)
        self.assertTf(t.compose(Transform()), 1.0, -2.0, 0.3, 0.7)

    def test_rotation_then_translation_by_hand(self):
        # parent frame turned 90 deg left at (1,0): a point 1 m ahead of it is at (1, 1).
        self.assertTf(Transform(1, 0, 0, math.pi / 2).compose(Transform(1, 0, 0, 0)), 1.0, 1.0, 0.0, math.pi / 2)

    def test_yaw_is_wrapped(self):
        t = Transform(0, 0, 0, math.pi).compose(Transform(0, 0, 0, math.pi / 2))
        self.assertAlmostEqual(t.yaw, -math.pi / 2)

    def test_inverse_cancels(self):
        rng = random.Random(3)
        for _ in range(100):
            t = Transform(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-1, 1), rng.uniform(-3, 3))
            self.assertTf(t.compose(t.inverse()), 0, 0, 0, 0)
            self.assertTf(t.inverse().compose(t), 0, 0, 0, 0)

    def test_non_finite_and_wrong_types_rejected(self):
        for bad in (float("nan"), float("inf"), True, "1", None):
            with self.subTest(bad=bad):
                with self.assertRaises(TfError):
                    Transform(x=bad)
                with self.assertRaises(TfError):
                    Transform(yaw=bad)


class TfTreeTests(unittest.TestCase):
    def test_required_chain_constant_matches_the_specification(self):
        self.assertEqual(f.REQUIRED_CHAIN, ("map", "odom", "base_link", "laser"))

    def test_valid_hardware_tree(self):
        tree = hardware_tree()
        tree.validate_required_chain()
        tree.validate_required_chain(f.HARDWARE_OWNERS)
        self.assertEqual(tree.ancestors("laser"), ["laser", "base_link", "odom", "map"])
        self.assertEqual(tree.parent_of("odom"), "map")
        self.assertIsNone(tree.parent_of("map"))

    def test_lookup_composes_the_whole_chain_by_hand(self):
        tree = hardware_tree(Transform(1.0, 2.0, 0, 0), Transform(0, 0, 0, math.pi / 2))
        t = tree.lookup(f.MAP, f.LASER)       # base at (1,2) facing +y; laser 0.05 ahead of it
        self.assertAlmostEqual(t.x, 1.0)
        self.assertAlmostEqual(t.y, 2.05)
        self.assertAlmostEqual(t.z, 0.15)
        self.assertAlmostEqual(t.yaw, math.pi / 2)
        back = tree.lookup(f.LASER, f.MAP)    # the opposite direction is the inverse
        round_trip = t.compose(back)
        for v in (round_trip.x, round_trip.y, round_trip.z, round_trip.yaw):
            self.assertAlmostEqual(v, 0.0)

    def test_lookup_between_siblings_and_to_self(self):
        tree = hardware_tree()
        tree.add(TfEdge(f.BASE_LINK, "camera", Transform(0.1, 0.0, 0.3, 0.0), "robot_description", static=True))
        t = tree.lookup(f.LASER, "camera")
        self.assertAlmostEqual(t.x, 0.05)
        self.assertAlmostEqual(t.z, 0.15)
        self.assertEqual(tree.lookup(f.ODOM, f.ODOM), Transform())

    def test_lookup_without_a_connection_is_an_error(self):
        tree = hardware_tree()
        with self.assertRaises(TfError):
            tree.lookup(f.MAP, "nowhere")
        other = TfTree()
        other.add(TfEdge("a", "b", Transform(), "x"))
        other.add(TfEdge("c", "d", Transform(), "x"))
        with self.assertRaises(TfError):
            other.lookup("b", "d")

    def test_same_owner_may_update_its_edge(self):
        tree = hardware_tree()
        tree.add(TfEdge(f.ODOM, f.BASE_LINK, Transform(3.0, 0, 0, 0), "odometry_source"))
        self.assertAlmostEqual(tree.lookup(f.ODOM, f.BASE_LINK).x, 3.0)

    def test_two_parents_rejected_even_from_the_same_owner(self):
        # Same owner as the existing laser edge, so ONLY the one-parent rule can reject this.
        tree = hardware_tree()
        with self.assertRaises(TfError) as ctx:
            tree.add(TfEdge(f.ODOM, f.LASER, Transform(), "robot_description", static=True))
        self.assertIn("two parents", str(ctx.exception))
        self.assertEqual(tree.parent_of(f.LASER), f.BASE_LINK)       # tree unchanged

    def test_two_publishers_for_one_edge_rejected(self):
        # Same parent and child as an existing edge, different owner: ONLY the owner rule applies.
        tree = hardware_tree()
        with self.assertRaises(TfError) as ctx:
            tree.add(TfEdge(f.MAP, f.ODOM, Transform(), "amcl"))
        self.assertIn("two publishers", str(ctx.exception))

    def test_cycles_and_self_edges_rejected(self):
        tree = hardware_tree()
        with self.assertRaises(TfError):
            tree.add(TfEdge(f.LASER, f.MAP, Transform(), "x"))
        with self.assertRaises(TfError):
            tree.add(TfEdge("a", "a", Transform(), "x"))
        for bad in (TfEdge("", "a", Transform(), "x"), TfEdge("a", "", Transform(), "x"),
                    TfEdge("a", "b", Transform(), "")):
            with self.assertRaises(TfError):
                TfTree().add(bad)

    def test_a_rejected_edge_leaves_the_tree_unchanged(self):
        tree = hardware_tree()
        before = tree.ancestors("laser")
        with self.assertRaises(TfError):
            tree.add(TfEdge(f.LASER, f.MAP, Transform(), "x"))
        self.assertEqual(tree.ancestors("laser"), before)
        tree.validate_required_chain()


class RequiredChainValidationTests(unittest.TestCase):
    def test_missing_edges(self):
        for skip in range(3):
            tree = TfTree()
            edges = [TfEdge(f.MAP, f.ODOM, Transform(), "a"), TfEdge(f.ODOM, f.BASE_LINK, Transform(), "b"),
                     TfEdge(f.BASE_LINK, f.LASER, Transform(), "c", static=True)]
            for i, e in enumerate(edges):
                if i != skip:
                    tree.add(e)
            with self.subTest(missing=edges[skip].child):
                with self.assertRaises(TfError):
                    tree.validate_required_chain()

    def test_an_extra_frame_inserted_into_the_chain_is_rejected(self):
        tree = TfTree()
        tree.add(TfEdge(f.MAP, f.ODOM, Transform(), "a"))
        tree.add(TfEdge(f.ODOM, "base_footprint", Transform(), "b"))
        tree.add(TfEdge("base_footprint", f.BASE_LINK, Transform(), "b"))
        tree.add(TfEdge(f.BASE_LINK, f.LASER, Transform(), "c", static=True))
        with self.assertRaises(TfError) as ctx:
            tree.validate_required_chain()
        self.assertIn("base_link", str(ctx.exception))

    def test_wrong_parent(self):
        tree = TfTree()
        tree.add(TfEdge(f.MAP, f.ODOM, Transform(), "a"))
        tree.add(TfEdge(f.ODOM, f.BASE_LINK, Transform(), "b"))
        tree.add(TfEdge(f.ODOM, f.LASER, Transform(), "c", static=True))   # laser hung off odom
        with self.assertRaises(TfError):
            tree.validate_required_chain()

    def test_owner_mismatch(self):
        with self.assertRaises(TfError):
            hardware_tree().validate_required_chain({**f.HARDWARE_OWNERS, (f.MAP, f.ODOM): "amcl"})

    def test_laser_mount_must_be_static(self):
        tree = TfTree()
        tree.add(TfEdge(f.MAP, f.ODOM, Transform(), "a"))
        tree.add(TfEdge(f.ODOM, f.BASE_LINK, Transform(), "b"))
        tree.add(TfEdge(f.BASE_LINK, f.LASER, Transform(), "c", static=False))
        with self.assertRaises(TfError):
            tree.validate_required_chain()


class ConfigBackedTransformTests(unittest.TestCase):
    def test_laser_mount_from_mock_config(self):
        self.assertEqual(f.laser_mount_transform(MOCK.lidar), Transform(0.05, 0.0, 0.15, 0.0))

    def test_laser_mount_is_never_defaulted(self):
        with self.assertRaises(IncompleteHardwareConfig) as ctx:
            f.laser_mount_transform(REAL.lidar)
        self.assertEqual(sorted(ctx.exception.missing),
                         ["lidar.mount_x_m", "lidar.mount_y_m", "lidar.mount_yaw_rad", "lidar.mount_z_m"])
        partial = dataclasses.replace(MOCK.lidar, mount_z_m=None)
        with self.assertRaises(IncompleteHardwareConfig) as ctx:
            f.laser_mount_transform(partial)
        self.assertEqual(ctx.exception.missing, ["lidar.mount_z_m"])

    def test_mock_graph_forms_a_valid_chain(self):
        tree = TfTree()
        for edge in f.mock_tf_edges(MOCK):
            tree.add(edge)
        tree.validate_required_chain()
        t = tree.lookup(f.MAP, f.LASER)
        self.assertEqual((t.x, t.y, t.z, t.yaw), (0.05, 0.0, 0.15, 0.0))
        with self.assertRaises(TfError):                       # mock owners are not the real owners
            tree.validate_required_chain(f.HARDWARE_OWNERS)

    def test_mock_transforms_cannot_be_built_from_a_real_or_relabelled_config(self):
        with self.assertRaises(TfError):
            f.mock_tf_edges(REAL)
        relabelled = dataclasses.replace(MOCK, meta=Meta(mock=False))
        with self.assertRaises(TfError):
            f.mock_tf_edges(relabelled)

    def test_static_publisher_arguments_are_exact(self):
        laser = [e for e in f.mock_tf_edges(MOCK) if e.child == f.LASER][0]
        self.assertEqual(f.static_publisher_args(laser), [
            "--x", "0.05", "--y", "0.0", "--z", "0.15", "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "laser"])
        ident = f.mock_tf_edges(MOCK)[0]
        self.assertEqual(f.static_publisher_args(ident)[-4:], ["--frame-id", "map", "--child-frame-id", "odom"])


if __name__ == "__main__":
    unittest.main()
