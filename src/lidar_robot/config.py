"""Hardware configuration loader.

Design rules (see docs/design.md):
  * Unknown physical values stay ``None``. They are NEVER filled from the mock config
    or from guesses.
  * Hardware-dependent consumers call ``require_complete(scope)``; it raises
    ``IncompleteHardwareConfig`` listing every missing key, and refuses mock configs.
  * ``safety.motors_enabled`` defaults to False when absent.
  * Unknown sections/keys and wrongly typed values are errors (typo protection).
"""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from . import protocol

UNO_PWM_PINS = frozenset({3, 5, 6, 9, 10, 11})
UNO_DIGITAL_PINS = frozenset(range(2, 14))  # D0/D1 are the USB serial port
ENCODER_STATES = ("unknown", "yes", "no")


class ConfigError(ValueError):
    """Invalid configuration (bad type, unknown key, out-of-range value)."""


class IncompleteHardwareConfig(ConfigError):
    def __init__(self, scope: str, missing: List[str], reason: str = "") -> None:
        self.scope = scope
        self.missing = list(missing)
        detail = reason or "missing: " + ", ".join(self.missing)
        super().__init__(f"hardware config incomplete for scope '{scope}': {detail}")


@dataclass(frozen=True)
class Safety:
    motors_enabled: bool = False


@dataclass(frozen=True)
class Limits:
    max_linear_mps: float = 0.15
    max_angular_rps: float = 0.6
    max_duty_permille: int = 500
    cmd_period_ms: int = 100
    cmd_ttl_ms: int = 250
    link_timeout_ms: int = 500


@dataclass(frozen=True)
class Drivetrain:
    wheel_radius_m: Optional[float] = None
    track_width_m: Optional[float] = None
    output_free_speed_rpm: Optional[float] = None
    open_loop_derate: float = 0.7


@dataclass(frozen=True)
class Encoders:
    present: str = "unknown"
    counts_per_wheel_rev: Optional[float] = None


@dataclass(frozen=True)
class Body:
    footprint_radius_m: Optional[float] = None
    length_m: Optional[float] = None
    width_m: Optional[float] = None


@dataclass(frozen=True)
class Lidar:
    model: str = "RPLIDAR C1"
    frame_id: str = "laser"
    range_min_m: float = 0.05
    range_max_m: float = 12.0
    scan_hz: float = 10.0
    angular_increment_deg: float = 0.72
    mount_x_m: Optional[float] = None
    mount_y_m: Optional[float] = None
    mount_z_m: Optional[float] = None
    mount_yaw_rad: Optional[float] = None


@dataclass(frozen=True)
class Serial:
    lidar_port: str = "/dev/ttyUSB0"
    lidar_baud: int = 460800
    arduino_port: str = "/dev/ttyACM0"
    arduino_baud: int = 115200


@dataclass(frozen=True)
class ArduinoPins:
    left_pwm: Optional[int] = None
    left_dir: Optional[int] = None
    right_pwm: Optional[int] = None
    right_dir: Optional[int] = None
    enc_left_a: Optional[int] = None
    enc_left_b: Optional[int] = None
    enc_right_a: Optional[int] = None
    enc_right_b: Optional[int] = None


@dataclass(frozen=True)
class Power:
    battery_chemistry: Optional[str] = None
    battery_nominal_v: Optional[float] = None
    battery_capacity_ah: Optional[float] = None


@dataclass(frozen=True)
class Meta:
    mock: bool = False


@dataclass(frozen=True)
class RobotConfig:
    meta: Meta = field(default_factory=Meta)
    safety: Safety = field(default_factory=Safety)
    limits: Limits = field(default_factory=Limits)
    drivetrain: Drivetrain = field(default_factory=Drivetrain)
    encoders: Encoders = field(default_factory=Encoders)
    body: Body = field(default_factory=Body)
    lidar: Lidar = field(default_factory=Lidar)
    serial: Serial = field(default_factory=Serial)
    arduino_pins: ArduinoPins = field(default_factory=ArduinoPins)
    power: Power = field(default_factory=Power)

    @property
    def is_mock(self) -> bool:
        return self.meta.mock

    def full_duty_speed_mps(self) -> float:
        """Open-loop estimate of wheel surface speed at 100 % duty (needs drivetrain values)."""
        d = self.drivetrain
        missing = [
            f"drivetrain.{k}"
            for k in ("wheel_radius_m", "output_free_speed_rpm")
            if getattr(d, k) is None
        ]
        if missing:
            raise IncompleteHardwareConfig("drivetrain", missing)
        return 2.0 * math.pi * d.wheel_radius_m * d.output_free_speed_rpm / 60.0 * d.open_loop_derate

    # ---- completeness -------------------------------------------------------------------

    def missing(self, scope: str) -> List[str]:
        """Dotted names of values that are required for ``scope`` but unknown."""
        if scope == "all":
            names: List[str] = []
            for sub in ("sim", "navigation", "odometry", "firmware"):
                for n in self.missing(sub):
                    if n not in names:
                        names.append(n)
            return names
        if scope not in _SCOPES:
            raise ConfigError(f"unknown scope {scope!r}; choose from {sorted(_SCOPES) + ['all']}")
        missing = []
        for dotted in _SCOPES[scope]:
            section, key = dotted.split(".")
            if getattr(getattr(self, section), key) is None:
                missing.append(dotted)
        if scope in ("firmware", "odometry") and self.encoders.present == "unknown":
            missing.append("encoders.present (must be 'yes' or 'no', currently 'unknown')")
        if self.encoders.present == "yes" and scope in ("firmware", "odometry"):
            keys = ["encoders.counts_per_wheel_rev"]
            if scope == "firmware":
                keys += [f"arduino_pins.{k}" for k in
                         ("enc_left_a", "enc_left_b", "enc_right_a", "enc_right_b")]
            for dotted in keys:
                section, key = dotted.split(".")
                if getattr(getattr(self, section), key) is None:
                    missing.append(dotted)
        return missing

    def require_complete(self, scope: str, *, allow_mock: bool = False) -> None:
        """Raise unless every value needed for ``scope`` is known and this is not a mock config."""
        if self.is_mock and not allow_mock:
            raise IncompleteHardwareConfig(
                scope, [], "mock configuration may not be used for hardware-dependent output"
            )
        missing = self.missing(scope)
        if missing:
            raise IncompleteHardwareConfig(scope, missing)


_SCOPES: Dict[str, Tuple[str, ...]] = {
    "sim": (
        "drivetrain.wheel_radius_m", "drivetrain.track_width_m",
        "drivetrain.output_free_speed_rpm",
        "lidar.mount_x_m", "lidar.mount_y_m", "lidar.mount_yaw_rad",
    ),
    "navigation": (
        "body.footprint_radius_m", "drivetrain.track_width_m",
        "lidar.mount_x_m", "lidar.mount_y_m", "lidar.mount_z_m", "lidar.mount_yaw_rad",
    ),
    "odometry": ("drivetrain.wheel_radius_m", "drivetrain.track_width_m"),
    "firmware": (
        "arduino_pins.left_pwm", "arduino_pins.left_dir",
        "arduino_pins.right_pwm", "arduino_pins.right_dir",
    ),
}

# ---- parsing ---------------------------------------------------------------------------

_SECTION_TYPES = {
    "meta": Meta, "safety": Safety, "limits": Limits, "drivetrain": Drivetrain,
    "encoders": Encoders, "body": Body, "lidar": Lidar, "serial": Serial,
    "arduino_pins": ArduinoPins, "power": Power,
}


def _coerce(section: str, key: str, value: Any, default: Any) -> Any:
    # Infer the expected type from the default; Optional fields without a default use
    # the annotation convention encoded in _OPTIONAL_KINDS.
    kind = _OPTIONAL_KINDS.get(f"{section}.{key}")
    if kind is None:
        kind = type(default)
    where = f"[{section}] {key}"
    if kind is bool:
        if not isinstance(value, bool):
            raise ConfigError(f"{where}: expected true/false, got {value!r}")
        return value
    if kind is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{where}: expected integer, got {value!r}")
        return value
    if kind is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"{where}: expected number, got {value!r}")
        if not math.isfinite(value):
            raise ConfigError(f"{where}: must be finite")
        return float(value)
    if kind is str:
        if not isinstance(value, str):
            raise ConfigError(f"{where}: expected string, got {value!r}")
        return value
    raise ConfigError(f"{where}: internal error, unsupported type {kind}")


_OPTIONAL_KINDS: Dict[str, type] = {
    "drivetrain.wheel_radius_m": float, "drivetrain.track_width_m": float,
    "drivetrain.output_free_speed_rpm": float,
    "encoders.counts_per_wheel_rev": float,
    "body.footprint_radius_m": float, "body.length_m": float, "body.width_m": float,
    "lidar.mount_x_m": float, "lidar.mount_y_m": float, "lidar.mount_z_m": float,
    "lidar.mount_yaw_rad": float,
    "power.battery_chemistry": str, "power.battery_nominal_v": float,
    "power.battery_capacity_ah": float,
}
for _k in ("left_pwm", "left_dir", "right_pwm", "right_dir",
           "enc_left_a", "enc_left_b", "enc_right_a", "enc_right_b"):
    _OPTIONAL_KINDS[f"arduino_pins.{_k}"] = int


def _build_section(name: str, cls: type, raw: Any) -> Any:
    if not isinstance(raw, dict):
        raise ConfigError(f"[{name}] must be a table")
    known = {f.name: f for f in fields(cls)}
    unknown = sorted(set(raw) - set(known))
    if unknown:
        raise ConfigError(f"[{name}] unknown key(s): {', '.join(unknown)}")
    defaults = cls()
    kwargs = {k: _coerce(name, k, v, getattr(defaults, k)) for k, v in raw.items()}
    return cls(**kwargs)


def parse_config(data: Dict[str, Any]) -> RobotConfig:
    unknown = sorted(set(data) - set(_SECTION_TYPES))
    if unknown:
        raise ConfigError(f"unknown section(s): {', '.join(unknown)}")
    sections = {name: _build_section(name, cls, data[name])
                for name, cls in _SECTION_TYPES.items() if name in data}
    cfg = RobotConfig(**sections)
    validate(cfg)
    return cfg


def load_config(path: "str | Path") -> RobotConfig:
    """Load and validate a config file. Missing physical values remain ``None``."""
    with open(path, "rb") as handle:
        try:
            data = tomllib.load(handle)
        except tomllib.TOMLDecodeError as exc:
            raise ConfigError(f"{path}: TOML syntax error: {exc}") from exc
    return parse_config(data)


# ---- validation ------------------------------------------------------------------------

def _positive(name: str, value: Optional[float]) -> None:
    if value is not None and not value > 0:
        raise ConfigError(f"{name} must be > 0 (got {value})")


def validate(cfg: RobotConfig) -> None:
    lim = cfg.limits
    for name in ("max_linear_mps", "max_angular_rps"):
        _positive(f"limits.{name}", getattr(lim, name))
    if not 1 <= lim.max_duty_permille <= 1000:
        raise ConfigError("limits.max_duty_permille must be within 1..1000")
    if lim.cmd_period_ms <= 0:
        raise ConfigError("limits.cmd_period_ms must be > 0")
    if not lim.cmd_period_ms * 2 <= lim.cmd_ttl_ms <= protocol.MAX_TTL_MS:
        raise ConfigError(
            f"limits.cmd_ttl_ms must satisfy 2*cmd_period_ms <= ttl <= {protocol.MAX_TTL_MS}"
        )
    if lim.link_timeout_ms < lim.cmd_ttl_ms:
        raise ConfigError("limits.link_timeout_ms must be >= limits.cmd_ttl_ms")

    dt = cfg.drivetrain
    for name in ("wheel_radius_m", "track_width_m", "output_free_speed_rpm"):
        _positive(f"drivetrain.{name}", getattr(dt, name))
    if not 0 < dt.open_loop_derate <= 1:
        raise ConfigError("drivetrain.open_loop_derate must be within (0, 1]")

    if cfg.encoders.present not in ENCODER_STATES:
        raise ConfigError(f"encoders.present must be one of {ENCODER_STATES}")
    _positive("encoders.counts_per_wheel_rev", cfg.encoders.counts_per_wheel_rev)

    for name in ("footprint_radius_m", "length_m", "width_m"):
        _positive(f"body.{name}", getattr(cfg.body, name))

    ld = cfg.lidar
    if not 0 < ld.range_min_m < ld.range_max_m:
        raise ConfigError("lidar range must satisfy 0 < range_min_m < range_max_m")
    _positive("lidar.scan_hz", ld.scan_hz)
    if not 0 < ld.angular_increment_deg <= 360:
        raise ConfigError("lidar.angular_increment_deg must be within (0, 360]")

    pins = cfg.arduino_pins
    for name in ("left_pwm", "right_pwm"):
        pin = getattr(pins, name)
        if pin is not None and pin not in UNO_PWM_PINS:
            raise ConfigError(f"arduino_pins.{name}={pin} is not an Uno PWM pin {sorted(UNO_PWM_PINS)}")
    for name in ("left_dir", "right_dir", "enc_left_a", "enc_left_b", "enc_right_a", "enc_right_b"):
        pin = getattr(pins, name)
        if pin is not None and pin not in UNO_DIGITAL_PINS:
            raise ConfigError(f"arduino_pins.{name}={pin} must be digital pin 2..13 (D0/D1 are USB serial)")
    used = [getattr(pins, f.name) for f in fields(ArduinoPins) if getattr(pins, f.name) is not None]
    if len(used) != len(set(used)):
        raise ConfigError("arduino_pins: the same pin is assigned twice")
