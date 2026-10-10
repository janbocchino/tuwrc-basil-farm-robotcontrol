#!/usr/bin/env python3
"""Forward reach + gripper close with the arm only; the rail is never commanded."""

from __future__ import annotations

import argparse
import sys

import rclpy

from .leaf_pick_imitation import (
    ARM_JOINTS,
    GRIPPER_OPEN,
    LeafPickImitation,
    clamp,
    scale_arm_pose,
)
from .rail_reach_grip import GRIP_SEQUENCE


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Reach forward and grip with the arm only (no rail motion)."
    )
    parser.add_argument(
        "--allow-hardware",
        action="store_true",
        help="Required confirmation when commanding the real arm.",
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Declare that this run targets real hardware (requires --allow-hardware).",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=0.5,
        help="Scale arm joints 1–5 toward zero (0.2–1.0, default 0.5).",
    )
    parser.add_argument(
        "--duration-scale",
        type=float,
        default=1.0,
        help="Multiply each waypoint duration (default 1.0).",
    )
    parser.add_argument(
        "--release",
        action="store_true",
        help="Open the gripper again after retracting.",
    )
    parser.add_argument(
        "--return-home",
        action="store_true",
        help="Return the arm to its starting joint positions at the end.",
    )
    args = parser.parse_args(argv)

    if args.hardware and not args.allow_hardware:
        print("Refusing to command hardware without --allow-hardware.", file=sys.stderr)
        return 2
    if not 0.2 <= args.scale <= 1.0:
        print("--scale must be between 0.2 and 1.0.", file=sys.stderr)
        return 2
    if args.duration_scale <= 0.0:
        print("--duration-scale must be > 0.", file=sys.stderr)
        return 2

    rclpy.init()
    node = LeafPickImitation()
    try:
        node.get_logger().info("Waiting for /joint_states …")
        node.wait_for_state(need_rail=False)
        start_arm = {name: node._positions[name] for name in ARM_JOINTS}

        sequence = list(GRIP_SEQUENCE)
        if args.release:
            retract_pose = sequence[-1][1]
            sequence.append(("release", {**retract_pose, "6": GRIPPER_OPEN}, 1.5))

        for label, pose, duration in sequence:
            targets = {
                name: clamp(name, value)
                for name, value in scale_arm_pose(pose, args.scale).items()
            }
            dur = duration * args.duration_scale
            node.get_logger().info(f"Waypoint '{label}': {targets} ({dur:.1f}s)")
            node.send_arm(targets, dur)

        if args.return_home:
            node.get_logger().info("Returning to start positions …")
            node.send_arm(start_arm, 2.5 * args.duration_scale)

        node.get_logger().info("Arm reach + grip finished.")
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
