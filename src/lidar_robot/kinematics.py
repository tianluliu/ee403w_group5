"""Differential-drive kinematics: pure math, driven by configuration, no hardware access.

Conventions (REP-103): x forward, y left, yaw/angular positive counter-clockwise.
Units: metres, seconds, radians. Wheel "duty" is open-loop PWM in permille
(-1000..1000, sign = direction) because encoders are not assumed to exist.

Failure policy: non-finite input raises ``ValueError`` and missing hardware values raise
``IncompleteHardwareConfig``. Callers on the drive path MUST treat any exception as "stop
the motors"; nothing here silently substitutes a default for an unknown physical value.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from .config import IncompleteHardwareConfig, RobotConfig


@dataclass(frozen=True)
class Twist:
    linear: float = 0.0   # m/s along +x of base_link
    angular: float = 0.0  # rad/s about +z (counter-clockwise positive)


@dataclass(frozen=True)
class WheelSpeeds:
    left: float   # m/s at the wheel contact patch, forward positive
    right: float


@dataclass(frozen=True)
class DriveCommand:
    left_permille: int
    right_permille: int
    effective: Twist  # what the open-loop duty implies after clamping/rounding


def _finite(name: str, value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number, got {value!r}")
    return float(value)


def _positive(name: str, value: float) -> float:
    value = _finite(name, value)
    if not value > 0:
        raise ValueError(f"{name} must be > 0, got {value!r}")
    return value


def twist_to_wheel_speeds(twist: Twist, track_width_m: float) -> WheelSpeeds:
    track = _positive("track_width_m", track_width_m)
    v, w = _finite("linear", twist.linear), _finite("angular", twist.angular)
    half = w * track / 2.0
    return WheelSpeeds(left=v - half, right=v + half)


def wheel_speeds_to_twist(speeds: WheelSpeeds, track_width_m: float) -> Twist:
    track = _positive("track_width_m", track_width_m)
    left, right = _finite("left", speeds.left), _finite("right", speeds.right)
    return Twist(linear=(left + right) / 2.0, angular=(right - left) / track)


def clamp_twist(twist: Twist, max_linear_mps: float, max_angular_rps: float) -> Twist:
    """Clamp each component independently to +/- its limit (curvature may change)."""
    max_v = _positive("max_linear_mps", max_linear_mps)
    max_w = _positive("max_angular_rps", max_angular_rps)
    v, w = _finite("linear", twist.linear), _finite("angular", twist.angular)
    return Twist(linear=max(-max_v, min(max_v, v)), angular=max(-max_w, min(max_w, w)))


def wheel_speeds_to_duty(
    speeds: WheelSpeeds, full_duty_speed_mps: float, max_duty_permille: int
) -> Tuple[int, int]:
    """Open-loop mapping speed -> duty. If either wheel would exceed the cap, BOTH are
    scaled by the same factor so the commanded curvature is preserved."""
    full = _positive("full_duty_speed_mps", full_duty_speed_mps)
    if isinstance(max_duty_permille, bool) or not isinstance(max_duty_permille, int) \
            or not 1 <= max_duty_permille <= 1000:
        raise ValueError(f"max_duty_permille must be an int within 1..1000, got {max_duty_permille!r}")
    left = _finite("left", speeds.left) / full * 1000.0
    right = _finite("right", speeds.right) / full * 1000.0
    peak = max(abs(left), abs(right))
    if peak > max_duty_permille:
        scale = max_duty_permille / peak
        left, right = left * scale, right * scale
    # Round, then re-clamp: rounding can never push a value past the (integer) cap.
    cap = max_duty_permille
    return (max(-cap, min(cap, int(round(left)))), max(-cap, min(cap, int(round(right)))))


def duty_to_wheel_speeds(left_permille: int, right_permille: int, full_duty_speed_mps: float) -> WheelSpeeds:
    full = _positive("full_duty_speed_mps", full_duty_speed_mps)
    return WheelSpeeds(left=left_permille / 1000.0 * full, right=right_permille / 1000.0 * full)


def _drive_params(cfg: RobotConfig) -> Tuple[float, float]:
    """(track_width_m, full_duty_speed_mps), reporting ALL missing values at once."""
    d = cfg.drivetrain
    missing = [f"drivetrain.{k}" for k in ("wheel_radius_m", "track_width_m", "output_free_speed_rpm")
               if getattr(d, k) is None]
    if missing:
        raise IncompleteHardwareConfig("drivetrain", missing)
    return d.track_width_m, cfg.full_duty_speed_mps()


def twist_to_drive(cfg: RobotConfig, twist: Twist) -> DriveCommand:
    """Config-driven pipeline: clamp to configured limits -> wheel speeds -> capped duty."""
    track, full = _drive_params(cfg)
    lim = cfg.limits
    clamped = clamp_twist(twist, lim.max_linear_mps, lim.max_angular_rps)
    left, right = wheel_speeds_to_duty(
        twist_to_wheel_speeds(clamped, track), full, lim.max_duty_permille)
    effective = wheel_speeds_to_twist(duty_to_wheel_speeds(left, right, full), track)
    return DriveCommand(left, right, effective)
