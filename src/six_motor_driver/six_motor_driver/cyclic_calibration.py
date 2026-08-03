"""Conversion between cyclic ST3215 steps and ordinary ROS joint angles."""

from __future__ import annotations

import math


STEPS_PER_REVOLUTION = 4096
RADIANS_PER_STEP = 2.0 * math.pi / STEPS_PER_REVOLUTION


def forward_distance(start: int, end: int) -> int:
    """Return positive cyclic distance from start to end."""
    return (int(end) - int(start)) % STEPS_PER_REVOLUTION


class CyclicJointCalibration:
    """One allowed cyclic raw interval represented as a linear ROS interval."""

    def __init__(
        self,
        lower_steps: int,
        upper_steps: int,
        zero_steps: int,
        direction: int = 1,
    ):
        for label, value in (
            ("lower_steps", lower_steps),
            ("upper_steps", upper_steps),
            ("zero_steps", zero_steps),
        ):
            if not 0 <= int(value) < STEPS_PER_REVOLUTION:
                raise ValueError(f"{label} muss zwischen 0 und 4095 liegen")
        if int(direction) not in (-1, 1):
            raise ValueError("direction muss -1 oder 1 sein")

        self.lower_steps = int(lower_steps)
        self.upper_steps = int(upper_steps)
        self.zero_steps = int(zero_steps)
        self.direction = int(direction)
        self.span_steps = forward_distance(
            self.lower_steps, self.upper_steps
        )
        if self.span_steps == 0:
            raise ValueError(
                "Untere und obere Grenze dürfen nicht identisch sein"
            )

        self.zero_progress = forward_distance(
            self.lower_steps, self.zero_steps
        )
        if self.zero_progress > self.span_steps:
            raise ValueError(
                f"Nullposition {self.zero_steps} liegt nicht im erlaubten "
                f"Bereich {self.lower_steps} -> {self.upper_steps}"
            )

    def progress_steps(self, raw_steps: int) -> int:
        return forward_distance(self.lower_steps, int(raw_steps))

    def contains(self, raw_steps: int) -> bool:
        return self.progress_steps(raw_steps) <= self.span_steps

    def raw_to_angle(self, raw_steps: int) -> float:
        progress = self.progress_steps(raw_steps)
        return (
            self.direction
            * (progress - self.zero_progress)
            * RADIANS_PER_STEP
        )

    def angle_to_raw(self, angle: float) -> int:
        progress = self.zero_progress + int(
            round(float(angle) / (self.direction * RADIANS_PER_STEP))
        )
        if not 0 <= progress <= self.span_steps:
            lower, upper = self.angle_limits()
            raise ValueError(
                f"Winkel {angle:.6f} außerhalb [{lower:.6f}, "
                f"{upper:.6f}] rad"
            )
        return (self.lower_steps + progress) % STEPS_PER_REVOLUTION

    def angle_limits(self) -> tuple[float, float]:
        lower_angle = self.raw_to_angle(self.lower_steps)
        upper_angle = self.raw_to_angle(self.upper_steps)
        return min(lower_angle, upper_angle), max(
            lower_angle, upper_angle
        )
