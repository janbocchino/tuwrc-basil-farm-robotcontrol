"""Pure helpers for FollowJointTrajectory timing."""

from __future__ import annotations

import math


def validate_time_from_start(times) -> list[float]:
    values = [float(value) for value in times]
    previous = None
    for value in values:
        if (
            not math.isfinite(value)
            or value < 0.0
            or (previous is not None and value <= previous)
        ):
            raise ValueError(
                "trajectory time_from_start values must be finite, "
                "non-negative, and strictly increasing"
            )
        previous = value
    return values


def segment_speed_steps_s(
    start_progress,
    target_progress,
    duration_sec: float,
    max_speed: int,
) -> int:
    if duration_sec <= 0.0:
        raise ValueError("segment duration must be positive")
    largest_distance = max(
        (
            abs(int(target) - int(start))
            for start, target in zip(start_progress, target_progress)
        ),
        default=0,
    )
    requested_speed = max(1, math.ceil(largest_distance / duration_sec))
    return min(int(max_speed), requested_speed)
