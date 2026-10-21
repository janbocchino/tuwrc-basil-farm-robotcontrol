#!/usr/bin/env python3
"""Unit checks for pure runtime helpers; no live ROS graph required."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT / "src/robot_control"
sys.path.insert(0, str(CONTROL / "lerobot_gui"))
sys.path.insert(0, str(CONTROL / "six_motor_driver"))
sys.path.insert(0, str(CONTROL / "tuwrc_mock_hardware"))

from lerobot_gui.control_validation import (  # noqa: E402
    validate_duration,
    validate_joint_targets,
)
from lerobot_gui.joint_state_store import JointStateStore  # noqa: E402
from six_motor_driver.trajectory_timing import (  # noqa: E402
    segment_speed_steps_s,
    validate_time_from_start,
)
from tuwrc_mock_hardware.rail_protocol import parse_status_line  # noqa: E402


def joint_state(names, positions):
    return SimpleNamespace(
        name=names,
        position=positions,
        velocity=[],
        effort=[],
    )


class JointStateStoreTests(unittest.TestCase):
    def test_partial_messages_are_merged(self):
        store = JointStateStore()
        store.update(joint_state(["1", "2"], [0.1, 0.2]))
        store.update(joint_state(["rail_joint"], [0.3]))

        self.assertEqual(
            {"1": 0.1, "2": 0.2, "rail_joint": 0.3},
            store.joint_positions(),
        )
        self.assertEqual(
            {"1", "2", "rail_joint"},
            {item["name"] for item in store.snapshot()["joints"]},
        )

    def test_stale_joints_are_reported_and_retained_for_ik_seed(self):
        store = JointStateStore(stale_after_sec=0.001)
        store.update(joint_state(["1"], [0.1]))
        time.sleep(0.01)

        self.assertEqual({"1": 0.1}, store.joint_positions())
        self.assertTrue(store.snapshot()["joints"][0]["stale"])


class ControlValidationTests(unittest.TestCase):
    def test_joint_targets_reject_unknown_and_out_of_bounds(self):
        with self.assertRaises(ValueError):
            validate_joint_targets({"unknown": 0})
        with self.assertRaises(ValueError):
            validate_joint_targets({"rail_joint": 0.8})
        with self.assertRaises(ValueError):
            validate_joint_targets({"1": math.inf})

    def test_duration_is_bounded(self):
        self.assertEqual(2.0, validate_duration(2))
        for value in (0, 61, math.nan):
            with self.assertRaises(ValueError):
                validate_duration(value)


class TrajectoryTimingTests(unittest.TestCase):
    def test_times_must_be_non_negative_and_increasing(self):
        self.assertEqual([0.0, 2.5], validate_time_from_start([0.0, 2.5]))
        for values in ([-1], [1, 1], [2, 1]):
            with self.assertRaises(ValueError):
                validate_time_from_start(values)

    def test_speed_uses_largest_joint_distance_and_is_bounded(self):
        self.assertEqual(
            100,
            segment_speed_steps_s([0, 100], [100, 300], 2.0, 600),
        )
        self.assertEqual(
            600,
            segment_speed_steps_s([0], [2000], 1.0, 600),
        )
        self.assertEqual(
            1,
            segment_speed_steps_s([5], [5], 2.0, 600),
        )


class RailProtocolTests(unittest.TestCase):
    def test_status_line_parsing(self):
        status = parse_status_line("ST 1 0 71500 1")
        self.assertTrue(status.lm_left)
        self.assertFalse(status.lm_right)
        self.assertEqual(71500, status.position_steps)
        self.assertTrue(status.homed)

    def test_invalid_status_is_ignored(self):
        self.assertIsNone(parse_status_line("hello"))
        self.assertIsNone(parse_status_line("ST 1 0 nope 1"))
        self.assertIsNone(parse_status_line("ST 2 0 10 1"))


if __name__ == "__main__":
    unittest.main()
