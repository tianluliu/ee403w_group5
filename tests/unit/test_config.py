"""Configuration loader tests, including real-vs-mock hardware isolation.

These tests use only the stdlib and never touch hardware.
"""

import dataclasses
import tempfile
import unittest
from pathlib import Path

import _bootstrap
from lidar_robot import config as cfgmod
from lidar_robot.config import (
    ConfigError, IncompleteHardwareConfig, RobotConfig, load_config, parse_config,
)

REAL = _bootstrap.CONFIG_DIR / "robot.toml"
MOCK = _bootstrap.CONFIG_DIR / "robot.mock.toml"
ALL_SCOPES = ("sim", "navigation", "odometry", "firmware", "all")


class RepoConfigFileTests(unittest.TestCase):
    def test_both_files_exist_and_load(self):
        for path in (REAL, MOCK):
            with self.subTest(path=path.name):
                self.assertIsInstance(load_config(path), RobotConfig)

    def test_real_config_is_not_flagged_mock_and_mock_is(self):
        self.assertFalse(load_config(REAL).is_mock)
        self.assertTrue(load_config(MOCK).is_mock)

    def test_motors_disabled_in_both_repo_configs(self):
        self.assertIs(load_config(REAL).safety.motors_enabled, False)
        self.assertIs(load_config(MOCK).safety.motors_enabled, False)

    def test_real_config_text_has_motors_enabled_false_literal(self):
        # Guards against someone flipping it while the loader default hides the change.
        text = REAL.read_text()
        self.assertRegex(text, r"(?m)^motors_enabled\s*=\s*false\b")
        self.assertNotRegex(text, r"(?m)^motors_enabled\s*=\s*true\b")

    def test_real_config_has_no_invented_physical_values(self):
        c = load_config(REAL)
        for name in ("wheel_radius_m", "track_width_m", "output_free_speed_rpm"):
            self.assertIsNone(getattr(c.drivetrain, name), name)
        for name in ("footprint_radius_m", "length_m", "width_m"):
            self.assertIsNone(getattr(c.body, name), name)
        for name in ("mount_x_m", "mount_y_m", "mount_z_m", "mount_yaw_rad"):
            self.assertIsNone(getattr(c.lidar, name), name)
        for f in dataclasses.fields(c.arduino_pins):
            self.assertIsNone(getattr(c.arduino_pins, f.name), f.name)
        for f in dataclasses.fields(c.power):
            self.assertIsNone(getattr(c.power, f.name), f.name)
        self.assertEqual(c.encoders.present, "unknown")  # not assumed either way

    def test_real_config_keeps_pdr_and_datasheet_values(self):
        c = load_config(REAL)
        self.assertEqual(c.lidar.model, "RPLIDAR C1")
        self.assertEqual(c.lidar.frame_id, "laser")


class IncompleteConfigReportingTests(unittest.TestCase):
    def setUp(self):
        self.real = load_config(REAL)

    def test_every_scope_reports_missing_values(self):
        for scope in ALL_SCOPES:
            with self.subTest(scope=scope):
                self.assertTrue(self.real.missing(scope))
                with self.assertRaises(IncompleteHardwareConfig) as ctx:
                    self.real.require_complete(scope)
                self.assertEqual(ctx.exception.scope, scope)
                self.assertEqual(ctx.exception.missing, self.real.missing(scope))

    def test_missing_names_are_reported_not_guessed(self):
        m = self.real.missing("firmware")
        for pin in ("left_pwm", "left_dir", "right_pwm", "right_dir"):
            self.assertIn(f"arduino_pins.{pin}", m)
        self.assertTrue(any(x.startswith("encoders.present") for x in m))
        self.assertIn("drivetrain.wheel_radius_m", self.real.missing("odometry"))
        self.assertIn("body.footprint_radius_m", self.real.missing("navigation"))
        self.assertIn("lidar.mount_yaw_rad", self.real.missing("sim"))

    def test_error_message_lists_the_missing_keys(self):
        with self.assertRaises(IncompleteHardwareConfig) as ctx:
            self.real.require_complete("odometry")
        self.assertIn("drivetrain.wheel_radius_m", str(ctx.exception))

    def test_open_loop_speed_refuses_to_guess(self):
        with self.assertRaises(IncompleteHardwareConfig):
            self.real.full_duty_speed_mps()

    def test_allow_mock_flag_does_not_bypass_missing_values(self):
        with self.assertRaises(IncompleteHardwareConfig):
            self.real.require_complete("all", allow_mock=True)

    def test_partial_fill_still_reports_the_rest(self):
        partial = dataclasses.replace(
            self.real, drivetrain=dataclasses.replace(self.real.drivetrain, wheel_radius_m=0.04))
        m = partial.missing("odometry")
        self.assertNotIn("drivetrain.wheel_radius_m", m)
        self.assertIn("drivetrain.track_width_m", m)

    def test_unknown_scope_is_an_error(self):
        with self.assertRaises(ConfigError):
            self.real.missing("bogus")

    def test_encoders_yes_requires_counts_and_pins(self):
        yes = dataclasses.replace(
            load_config(MOCK), encoders=cfgmod.Encoders(present="yes"))
        m = yes.missing("firmware")
        self.assertIn("encoders.counts_per_wheel_rev", m)
        self.assertIn("arduino_pins.enc_left_a", m)
        self.assertEqual(yes.missing("sim"), [])

    def test_encoders_unknown_blocks_firmware_and_odometry_even_if_pins_set(self):
        mock = load_config(MOCK)
        unknown = dataclasses.replace(mock, encoders=cfgmod.Encoders(present="unknown"))
        self.assertTrue(unknown.missing("firmware"))
        self.assertTrue(unknown.missing("odometry"))


class MockIsolationTests(unittest.TestCase):
    def setUp(self):
        self.mock = load_config(MOCK)

    def test_mock_is_complete_but_refused_for_every_scope(self):
        for scope in ALL_SCOPES:
            with self.subTest(scope=scope):
                self.assertEqual(self.mock.missing(scope), [])  # complete...
                with self.assertRaises(IncompleteHardwareConfig) as ctx:
                    self.mock.require_complete(scope)             # ...yet refused
                self.assertIn("mock", str(ctx.exception))

    def test_mock_allowed_only_with_explicit_opt_in(self):
        self.mock.require_complete("sim", allow_mock=True)  # must not raise

    def test_mock_flag_survives_dataclass_replace(self):
        changed = dataclasses.replace(self.mock, safety=cfgmod.Safety(motors_enabled=True))
        self.assertTrue(changed.is_mock)
        with self.assertRaises(IncompleteHardwareConfig):
            changed.require_complete("firmware")

    def test_mock_values_do_not_leak_into_real_config(self):
        real, mock = load_config(REAL), load_config(MOCK)
        for section in ("drivetrain", "body", "arduino_pins", "power"):
            for f in dataclasses.fields(getattr(real, section)):
                r, m = getattr(getattr(real, section), f.name), getattr(getattr(mock, section), f.name)
                if m is not None and r is not None and f.name != "open_loop_derate":
                    self.assertNotEqual(r, m, f"{section}.{f.name} identical to mock value")

    def test_library_never_references_the_mock_file(self):
        src = _bootstrap.SRC / "lidar_robot"
        for py in src.rglob("*.py"):
            self.assertNotIn("robot.mock", py.read_text(), f"{py.name} references the mock config")

    def test_missing_real_file_raises_instead_of_falling_back(self):
        with self.assertRaises(FileNotFoundError):
            load_config(_bootstrap.CONFIG_DIR / "does_not_exist.toml")

    def test_real_file_cannot_become_mock_by_omission(self):
        self.assertFalse(parse_config({}).is_mock)

    def test_meta_mock_must_be_a_real_boolean(self):
        with self.assertRaises(ConfigError):
            parse_config({"meta": {"mock": "true"}})


class ParseAndValidateTests(unittest.TestCase):
    def test_empty_config_is_safe_and_unknown(self):
        c = parse_config({})
        self.assertIs(c.safety.motors_enabled, False)
        self.assertIsNone(c.drivetrain.wheel_radius_m)
        self.assertEqual(c.encoders.present, "unknown")

    def test_empty_safety_section_defaults_to_disabled(self):
        self.assertIs(parse_config({"safety": {}}).safety.motors_enabled, False)

    def test_motors_enabled_requires_real_boolean(self):
        for bad in ("false", "true", 1, 0, "yes"):
            with self.subTest(value=bad):
                with self.assertRaises(ConfigError):
                    parse_config({"safety": {"motors_enabled": bad}})

    def test_motors_enabled_true_is_loaded_when_explicit(self):
        self.assertTrue(parse_config({"safety": {"motors_enabled": True}}).safety.motors_enabled)

    def test_unknown_section_and_key_rejected(self):
        with self.assertRaises(ConfigError):
            parse_config({"motor_safety": {}})
        with self.assertRaises(ConfigError):
            parse_config({"safety": {"motors_enabeld": True}})
        with self.assertRaises(ConfigError):
            parse_config({"drivetrain": {"wheel_radius": 0.04}})

    def test_section_must_be_a_table(self):
        with self.assertRaises(ConfigError):
            parse_config({"limits": 5})

    def test_type_errors(self):
        bad = [
            {"drivetrain": {"wheel_radius_m": "0.04"}},
            {"drivetrain": {"wheel_radius_m": True}},
            {"limits": {"cmd_ttl_ms": 250.5}},
            {"limits": {"max_duty_permille": True}},
            {"lidar": {"model": 5}},
            {"serial": {"lidar_baud": "460800"}},
            {"arduino_pins": {"left_pwm": 5.0}},
        ]
        for data in bad:
            with self.subTest(data=data):
                with self.assertRaises(ConfigError):
                    parse_config(data)

    def test_non_finite_numbers_rejected(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(value=value):
                with self.assertRaises(ConfigError):
                    parse_config({"drivetrain": {"wheel_radius_m": value}})

    def test_int_accepted_for_float_field(self):
        self.assertEqual(parse_config({"limits": {"max_linear_mps": 1}}).limits.max_linear_mps, 1.0)

    def test_non_positive_dimensions_rejected(self):
        for section, key in (("drivetrain", "wheel_radius_m"), ("drivetrain", "track_width_m"),
                             ("drivetrain", "output_free_speed_rpm"), ("body", "footprint_radius_m"),
                             ("limits", "max_linear_mps"), ("limits", "max_angular_rps"),
                             ("encoders", "counts_per_wheel_rev")):
            for value in (0, -1.0):
                with self.subTest(key=f"{section}.{key}", value=value):
                    with self.assertRaises(ConfigError):
                        parse_config({section: {key: value}})

    def test_duty_cap_range(self):
        for ok in (1, 500, 1000):
            parse_config({"limits": {"max_duty_permille": ok}})
        for bad in (0, -1, 1001):
            with self.assertRaises(ConfigError):
                parse_config({"limits": {"max_duty_permille": bad}})

    def test_derate_range(self):
        parse_config({"drivetrain": {"open_loop_derate": 1.0}})
        for bad in (0, -0.1, 1.01):
            with self.assertRaises(ConfigError):
                parse_config({"drivetrain": {"open_loop_derate": bad}})

    def test_ttl_and_timeout_relationships(self):
        parse_config({"limits": {"cmd_period_ms": 100, "cmd_ttl_ms": 200, "link_timeout_ms": 200}})
        bad = [
            {"cmd_period_ms": 100, "cmd_ttl_ms": 199},                      # < 2 periods
            {"cmd_period_ms": 100, "cmd_ttl_ms": 1001, "link_timeout_ms": 2000},  # > protocol max
            {"cmd_ttl_ms": 250, "link_timeout_ms": 249},                    # link < ttl
            {"cmd_period_ms": 0},
        ]
        for limits in bad:
            with self.subTest(limits=limits):
                with self.assertRaises(ConfigError):
                    parse_config({"limits": limits})

    def test_encoder_state_enum(self):
        for ok in ("unknown", "yes", "no"):
            parse_config({"encoders": {"present": ok}})
        for bad in ("maybe", "Yes", "", "true"):
            with self.assertRaises(ConfigError):
                parse_config({"encoders": {"present": bad}})

    def test_lidar_range_ordering(self):
        with self.assertRaises(ConfigError):
            parse_config({"lidar": {"range_min_m": 12.0, "range_max_m": 12.0}})
        with self.assertRaises(ConfigError):
            parse_config({"lidar": {"range_min_m": 0.0}})

    def test_arduino_pin_rules(self):
        parse_config({"arduino_pins": {"left_pwm": 5, "left_dir": 4, "right_pwm": 6, "right_dir": 7}})
        bad = [
            {"left_pwm": 4},                      # not PWM-capable on Uno
            {"left_pwm": 0}, {"left_dir": 1},     # USB serial pins
            {"left_dir": 14}, {"left_dir": -1},
            {"left_pwm": 5, "right_pwm": 5},      # duplicate
            {"left_dir": 4, "right_dir": 4},
            {"left_pwm": 5, "left_dir": 5},
        ]
        for pins in bad:
            with self.subTest(pins=pins):
                with self.assertRaises(ConfigError):
                    parse_config({"arduino_pins": pins})

    def test_toml_syntax_error_becomes_config_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "broken.toml"
            path.write_text("[safety\nmotors_enabled = false\n")
            with self.assertRaises(ConfigError):
                load_config(path)

    def test_config_is_immutable(self):
        c = parse_config({})
        with self.assertRaises(dataclasses.FrozenInstanceError):
            c.safety.motors_enabled = True


if __name__ == "__main__":
    unittest.main()
