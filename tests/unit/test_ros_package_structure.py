"""STATIC structure checks for the ROS 2 package skeletons (no ROS needed).

These prove that metadata is consistent and that the ROS tree contains no motor-command
path. They do NOT prove the packages build, import rclpy, or run: that is UNVERIFIED until
executed in a real ROS 2 Jazzy environment (see tests/ros2_integration).
"""

import ast
import re
import sys
import unittest
import xml.etree.ElementTree as ET

from _bootstrap import REPO_ROOT

WS_SRC = REPO_ROOT / "ros2_ws" / "src"
PACKAGES = ("robot_base_bridge", "robot_mock", "robot_bringup")
ALLOWED_THIRD_PARTY = {"rclpy", "std_msgs", "sensor_msgs", "launch", "launch_ros", "setuptools",
                       "lidar_robot"}
LOCAL = set(PACKAGES)


def py_files(pkg):
    return sorted(p for p in (WS_SRC / pkg).rglob("*.py") if "__pycache__" not in p.parts)


def code_tokens(path):
    """Identifiers, import module names and non-docstring string constants of a module."""
    tree = ast.parse(path.read_text(), filename=str(path), feature_version=(3, 12))
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            body = node.body
            if body and isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant) \
                    and isinstance(body[0].value.value, str):
                docstrings.add(id(body[0].value))
    tokens = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            tokens.append(node.id)
        elif isinstance(node, ast.Attribute):
            tokens.append(node.attr)
        elif isinstance(node, ast.alias):
            tokens.append(node.name)
        elif isinstance(node, ast.ImportFrom) and node.module:
            tokens.append(node.module)
            tokens.extend(f"{node.module}.{a.name}" for a in node.names)
        elif isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docstrings:
            tokens.append(node.value)
    return tokens


class PackageMetadataTests(unittest.TestCase):
    def test_expected_packages_exist_and_nothing_unexpected_does(self):
        found = sorted(p.name for p in WS_SRC.iterdir() if p.is_dir())
        self.assertEqual(found, sorted(PACKAGES))

    def test_package_xml_is_well_formed_and_consistent(self):
        for pkg in PACKAGES:
            with self.subTest(pkg=pkg):
                root = ET.parse(WS_SRC / pkg / "package.xml").getroot()
                self.assertEqual(root.tag, "package")
                self.assertEqual(root.get("format"), "3")
                self.assertEqual(root.findtext("name"), pkg)
                self.assertEqual(root.findtext("buildtool_depend"), "ament_python")
                self.assertEqual(root.find("export").findtext("build_type"), "ament_python")
                self.assertTrue(root.findtext("version"))
                self.assertIn("UNVERIFIED", " ".join(root.findtext("description").split()))

    def test_resource_marker_and_python_package_exist(self):
        for pkg in PACKAGES:
            with self.subTest(pkg=pkg):
                self.assertTrue((WS_SRC / pkg / "resource" / pkg).is_file())
                self.assertTrue((WS_SRC / pkg / pkg / "__init__.py").is_file())
                self.assertTrue((WS_SRC / pkg / "setup.py").is_file())

    def test_setup_py_name_matches_and_entry_points_resolve(self):
        expected = {"robot_base_bridge": {"heartbeat_node": ("robot_base_bridge.heartbeat_node", "main")},
                    "robot_mock": {"mock_scan_node": ("robot_mock.mock_scan_node", "main")},
                    "robot_bringup": {}}
        for pkg in PACKAGES:
            with self.subTest(pkg=pkg):
                text = (WS_SRC / pkg / "setup.py").read_text()
                self.assertIn(f'package_name = "{pkg}"', text)
                entries = {m[0]: (m[1], m[2]) for m in
                           re.findall(r'"(\w+) = ([\w.]+):(\w+)"', text)}
                self.assertEqual(entries, expected[pkg])
                for module, func in entries.values():
                    path = WS_SRC / pkg / (module.replace(".", "/") + ".py")
                    self.assertTrue(path.is_file(), path)
                    tree = ast.parse(path.read_text())
                    names = {n.name for n in tree.body if isinstance(n, ast.FunctionDef)}
                    self.assertIn(func, names)

    def test_setup_cfg_installs_scripts_into_the_package_libdir(self):
        for pkg in ("robot_base_bridge", "robot_mock"):
            text = (WS_SRC / pkg / "setup.cfg").read_text()
            self.assertIn(f"script_dir=$base/lib/{pkg}", text)
            self.assertIn(f"install_scripts=$base/lib/{pkg}", text)

    def test_launch_files_parse_and_define_generate_launch_description(self):
        launch_dir = WS_SRC / "robot_bringup" / "launch"
        files = sorted(launch_dir.glob("*.launch.py"))
        self.assertEqual([f.name for f in files], ["heartbeat.launch.py", "mock.launch.py"])
        for f in files:
            with self.subTest(file=f.name):
                tree = ast.parse(f.read_text(), feature_version=(3, 12))
                self.assertIn("generate_launch_description",
                              {n.name for n in tree.body if isinstance(n, ast.FunctionDef)})

    def test_launch_files_installed_by_setup_py_use_the_launch_glob(self):
        self.assertIn('glob("launch/*.launch.py")', (WS_SRC / "robot_bringup" / "setup.py").read_text())


class SafetyBoundaryTests(unittest.TestCase):
    def test_every_import_is_stdlib_ros_or_ours(self):
        stdlib = set(sys.stdlib_module_names)
        for pkg in PACKAGES:
            for path in py_files(pkg):
                tree = ast.parse(path.read_text(), filename=str(path), feature_version=(3, 12))
                for node in ast.walk(tree):
                    mods = []
                    if isinstance(node, ast.Import):
                        mods = [a.name.split(".")[0] for a in node.names]
                    elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                        mods = [node.module.split(".")[0]]
                    for mod in mods:
                        with self.subTest(file=path.name, module=mod):
                            self.assertTrue(mod in stdlib or mod in ALLOWED_THIRD_PARTY or mod in LOCAL)

    def test_no_motor_command_or_serial_path_exists_in_the_ros_tree(self):
        # Inspect code tokens only (identifiers, imports, non-docstring strings), so prose in
        # docstrings such as "touches no serial port" is not a false alarm.
        forbidden = ("cmd_vel", "Twist", "serial", "/dev/tty", "MotorBridge", "twist_to_drive",
                     "geometry_msgs", "lidar_robot.bridge", "lidar_robot.transport",
                     "lidar_robot.kinematics", "lidar_robot.mock_mcu")
        for pkg in PACKAGES:
            for path in py_files(pkg):
                tokens = code_tokens(path)
                if path.read_text().strip():          # an empty __init__.py has nothing to inspect
                    self.assertTrue(tokens, f"no tokens extracted from {path.name}")
                for word in forbidden:
                    with self.subTest(file=path.name, word=word):
                        self.assertFalse([t for t in tokens if word in t], tokens)

    def test_mock_launch_is_hard_wired_to_the_mock_config(self):
        text = (WS_SRC / "robot_bringup" / "launch" / "mock.launch.py").read_text()
        self.assertIn('"robot.mock.toml"', text)
        self.assertNotIn('"robot.toml"', text)
        self.assertNotIn("DeclareLaunchArgument", text)       # nothing can redirect it to another config
        self.assertIn("mock_tf_edges", text)

    def test_mock_scan_node_refuses_non_mock_configs(self):
        text = (WS_SRC / "robot_mock" / "robot_mock" / "mock_scan_node.py").read_text()
        self.assertIn("if not cfg.is_mock:", text)
        self.assertLess(text.index("if not cfg.is_mock:"), text.index("MockLidar(cfg.lidar"))

    def test_the_code_token_extractor_actually_sees_code_and_skips_docstrings(self):
        sample = WS_SRC / "robot_base_bridge" / "robot_base_bridge" / "heartbeat_node.py"
        tokens = code_tokens(sample)
        self.assertIn("rclpy", tokens)
        self.assertIn("/robot/heartbeat", tokens)
        self.assertFalse([t for t in tokens if "no serial port" in t])   # docstring prose excluded


if __name__ == "__main__":
    unittest.main()
