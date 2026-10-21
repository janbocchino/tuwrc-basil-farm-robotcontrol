#!/usr/bin/env python3
"""Move the arm (and later the rail) to the saved home position."""

from __future__ import annotations

import argparse
import sys

import rclpy

from .leaf_pick_imitation import LeafPickImitation

# Home pose (radians), read from the real arm via /joint_states on 2026-10-10
# with the current six_motor_calibration.yaml. Re-measure after recalibrating.
HOME_ARM = {
    "1": 0.0107,
    "2": -1.5616,
    "3": 1.5447,
    "4": 0.0123,
    "5": 2.3363,
    "6": -0.6995,
}

# Rail home in metres from the left end (lm_left = 0, set by ~/calibrate).
# Disabled for now; uncomment here and below.
# HOME_RAIL_M = 0.0


def main(argv=None):
    parser = argparse.ArgumentParser(description="Move to the saved home position.")
    parser.add_argument(
        "--duration",
        type=float,
        default=4.0,
        help="Arm move duration in seconds (default 4.0).",
    )
    args = parser.parse_args(argv)

    if args.duration <= 0.0:
        print("--duration must be > 0.", file=sys.stderr)
        return 2

    rclpy.init()
    node = LeafPickImitation()
    try:
        node.get_logger().info("Waiting for /joint_states …")
        node.wait_for_state(need_rail=False)

        # No clamp(): the hardcoded JOINT_LIMITS predate the current
        # calibration; the arm driver checks the calibrated limits itself.
        node.get_logger().info(f"Moving arm to home: {HOME_ARM} ({args.duration:.1f}s)")
        node.send_arm(HOME_ARM, args.duration)

        # node.get_logger().info(f"Moving rail to home: {HOME_RAIL_M:.3f} m …")
        # node.send_rail(HOME_RAIL_M, 3.0)

        node.get_logger().info("Home reached.")
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
