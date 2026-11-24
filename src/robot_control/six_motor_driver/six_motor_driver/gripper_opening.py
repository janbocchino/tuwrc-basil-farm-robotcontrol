"""Jaw opening for the v2.1 parallel gripper.

Joint 6 is prismatic. Position 0 is closed (jaws meet on the centerline) and
0.017 m is fully open, which is 17 mm of travel on each jaw and a 34 mm gap.
The actuator is still the STS3215 on servo ID 6. Closed and open raw steps
are filled in after the end positions are measured. Until then the joint is
not commanded.
"""

from __future__ import annotations

from .cyclic_calibration import STEPS_PER_REVOLUTION, forward_distance


GRIPPER_JOINT = "6"
DEFAULT_TRAVEL_M = 0.017


class GripperOpeningCalibration:
    """Linear map between jaw opening in metres and one servo revolution."""

    def __init__(
        self,
        closed_steps: int,
        open_steps: int,
        travel_m: float = DEFAULT_TRAVEL_M,
    ):
        self.closed_steps = int(closed_steps)
        self.open_steps = int(open_steps)
        self.travel_m = float(travel_m)
        self.span_steps = 0
        self.calibrated = False
        if self.closed_steps < 0 or self.open_steps < 0:
            return
        for label, value in (
            ("gripper_closed_steps", self.closed_steps),
            ("gripper_open_steps", self.open_steps),
        ):
            if not 0 <= value < STEPS_PER_REVOLUTION:
                raise ValueError(f"{label} must be in 0..4095")
        if self.travel_m <= 0.0:
            raise ValueError("gripper travel must be positive")
        self.span_steps = forward_distance(self.closed_steps, self.open_steps)
        if self.span_steps == 0:
            raise ValueError("closed and open gripper steps must differ")
        self.calibrated = True

    def limits(self) -> tuple[float, float]:
        return (0.0, self.travel_m)

    def meters_to_raw(self, opening_m: float) -> int:
        if not self.calibrated:
            raise RuntimeError("Gripper servo endpoints are not calibrated")
        opening = float(opening_m)
        if opening < -1e-9 or opening > self.travel_m + 1e-9:
            raise ValueError(
                f"Jaw opening {opening:.4f} m is outside "
                f"[0, {self.travel_m:.4f}] m"
            )
        opening = min(max(opening, 0.0), self.travel_m)
        progress = int(round(self.span_steps * opening / self.travel_m))
        return (self.closed_steps + progress) % STEPS_PER_REVOLUTION

    def raw_to_meters(self, raw_steps: int) -> float:
        if not self.calibrated:
            raise RuntimeError("Gripper servo endpoints are not calibrated")
        progress = forward_distance(self.closed_steps, int(raw_steps))
        if progress > self.span_steps:
            raise ValueError(
                f"Servo step {raw_steps} is outside the calibrated jaw travel"
            )
        return self.travel_m * progress / self.span_steps
