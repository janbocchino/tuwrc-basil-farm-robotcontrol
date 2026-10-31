"""Validation shared by the browser control API and its tests."""

from __future__ import annotations

import math


JOINT_LIMITS = {
    "rail_joint": (0.0, 0.715),
    "1": (-118.48, 118.48),
    "2": (-106.17, 106.17),
    "3": (-96.68, 96.77),
    "4": (-103.53, 103.53),
    "5": (-165.76, 165.76),
    "6": (-10.64, 116.45),
}
MIN_DURATION_SEC = 0.1
MAX_DURATION_SEC = 60.0


def validate_duration(value) -> float:
    duration = float(value)
    if not math.isfinite(duration):
        raise ValueError("duration must be finite")
    if not MIN_DURATION_SEC <= duration <= MAX_DURATION_SEC:
        raise ValueError(
            f"duration must be between {MIN_DURATION_SEC:g} and "
            f"{MAX_DURATION_SEC:g} seconds"
        )
    return duration


def validate_joint_targets(joints) -> dict[str, float]:
    if not isinstance(joints, dict) or not joints:
        raise ValueError("joints must be a non-empty object")

    validated = {}
    for name, raw_value in joints.items():
        if name not in JOINT_LIMITS:
            raise ValueError(f"unknown joint: {name}")
        value = float(raw_value)
        if not math.isfinite(value):
            raise ValueError(f"{name} target must be finite")
        lower, upper = JOINT_LIMITS[name]
        if not lower <= value <= upper:
            unit = "m" if name == "rail_joint" else "deg"
            raise ValueError(
                f"{name} target {value:g} {unit} is outside "
                f"[{lower:g}, {upper:g}] {unit}"
            )
        validated[name] = value
    return validated
