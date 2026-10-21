#!/usr/bin/env python3
"""Drive the rail to a target, then do a forward reach + gripper close.

Commands whichever arm and rail bringup is already running.
On hardware, rail positions are measured from the left end (lm_left = 0 m)
and the rail must be calibrated first (/tuwrc_rail_driver/calibrate).
"""

from __future__ import annotations

import argparse
import sys

import rclpy

from .leaf_pick_imitation import (
    ARM_JOINTS,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    RAIL_JOINT,
    LeafPickImitation,
    _arm,
    clamp,
    scale_arm_pose,
)
from .limits import JOINT_LIMITS

# Forward reach with open jaws, close on the object, pull back holding it.
GRIP_SEQUENCE: list[tuple[str, dict[str, float], float]] = [
    ("open", _arm(0.0, 0.0, 0.0, 0.0, 0.0, GRIPPER_OPEN), 1.5),
    ("approach", _arm(0.0, 0.45, -0.70, 0.40, 0.0, GRIPPER_OPEN), 2.5),
    ("reach", _arm(0.0, 0.65, -0.95, 0.55, 0.0, GRIPPER_OPEN), 2.0),
    ("grip", _arm(0.0, 0.65, -0.95, 0.55, 0.0, GRIPPER_CLOSED), 1.5),
    ("retract", _arm(0.0, 0.35, -0.55, 0.30, 0.0, GRIPPER_CLOSED), 2.0),
]


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Move the rail to a position, then reach forward and grip."
    )
    parser.add_argument(
        "--rail-m",
        type=float,
        required=True,
        help="Rail target in metres; on hardware from the left end (0 = lm_left).",
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
        help="Multiply each arm waypoint duration (default 1.0).",
    )
    parser.add_argument(
        "--release",
        action="store_true",
        help="Open the gripper again after retracting.",
    )
    parser.add_argument(
        "--return-home",
        action="store_true",
        help="Return arm and rail to their starting positions at the end.",
    )
    args = parser.parse_args(argv)

    if not 0.2 <= args.scale <= 1.0:
        print("--scale must be between 0.2 and 1.0.", file=sys.stderr)
        return 2
    if args.duration_scale <= 0.0:
        print("--duration-scale must be > 0.", file=sys.stderr)
        return 2
    if not JOINT_LIMITS[RAIL_JOINT][0] <= args.rail_m <= JOINT_LIMITS[RAIL_JOINT][1]:
        print("--rail-m must be within 0–0.715 m.", file=sys.stderr)
        return 2

    rclpy.init()
    node = LeafPickImitation()
    try:
        node.get_logger().info("Waiting for /joint_states …")
        node.wait_for_state(need_rail=True)
        start_arm = {name: node._positions[name] for name in ARM_JOINTS}
        start_rail = node._positions[RAIL_JOINT]

        rail_target = clamp(RAIL_JOINT, args.rail_m)
        node.get_logger().info(
            f"Moving rail {start_rail:.3f} m → {rail_target:.3f} m …"
        )
        # The real rail runs at Arduino speed; time_from_start only paces the mock.
        node.send_rail(rail_target, 3.0)

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
            node.send_rail(start_rail, 3.0)

        node.get_logger().info("Rail reach + grip finished.")
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
