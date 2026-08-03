#!/usr/bin/env python3
"""Verify the GUI uses the shared action contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GUI = ROOT / "src/lerobot_gui/lerobot_gui/joint_state_gui.py"
MOCK = ROOT / "src/tuwrc_mock_hardware/tuwrc_mock_hardware/mock_hardware.py"
MOTION = ROOT / "src/tuwrc_motion_examples/tuwrc_motion_examples/small_arm_motion.py"


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> None:
    gui = GUI.read_text(encoding="utf-8")
    for token in (
        "ActionClient",
        "/six_motor_controller/follow_joint_trajectory",
        "/rail_controller/follow_joint_trajectory",
        'BASE_FRAME = "world"',
        'END_EFFECTOR_FRAME = "tcp"',
        'LINEAR_JOINTS = {"rail_joint"}',
        "--mode",
    ):
        if token not in gui:
            fail(f"GUI missing expected token: {token}")

    if "/arm_controller/joint_trajectory" in gui:
        fail("GUI still publishes basil topic trajectories; use actions instead")

    mock = MOCK.read_text(encoding="utf-8")
    for token in (
        "/six_motor_controller/follow_joint_trajectory",
        "/rail_controller/follow_joint_trajectory",
        "rail_joint",
    ):
        if token not in mock:
            fail(f"mock_hardware missing expected token: {token}")

    motion = MOTION.read_text(encoding="utf-8")
    if "--allow-hardware" not in motion:
        fail("small_arm_motion must require --allow-hardware for real hardware")

    print("OK: GUI/action contract checks passed")


if __name__ == "__main__":
    main()
