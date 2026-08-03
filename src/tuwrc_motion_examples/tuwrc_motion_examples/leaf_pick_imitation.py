#!/usr/bin/env python3
"""Imitate a front reach + leaf pick/cut using the parallel-jaw gripper.

No cutter tool is modeled: joint 6 open/close stands in for the snip.
Optional rail advance works in view mode only (hardware rail is held at 0).
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_ACTION = "/six_motor_controller/follow_joint_trajectory"
RAIL_ACTION = "/rail_controller/follow_joint_trajectory"
ARM_JOINTS = ["1", "2", "3", "4", "5", "6"]
RAIL_JOINT = "rail_joint"

JOINT_LIMITS = {
    "rail_joint": (-0.5, 0.5),
    "1": (-2.06780610, 2.06780610),
    "2": (-1.85304879, 1.85304879),
    "3": (-1.68737887, 1.68891285),
    "4": (-1.80702937, 1.80702937),
    "5": (-2.89308777, 2.89308777),
    "6": (-0.18561168, 2.03252454),
}

# Larger jaw values ≈ open on the measured SO-101 calibration.
GRIPPER_OPEN = 1.25
GRIPPER_CLOSED = 0.08

# Absolute arm waypoints (radians) for a front reach + snip imitation.
# Tuned well inside measured limits; assumes the arm starts near home (zeros).
def _arm(j1: float, j2: float, j3: float, j4: float, j5: float, j6: float) -> dict[str, float]:
    return {"1": j1, "2": j2, "3": j3, "4": j4, "5": j5, "6": j6}


DEFAULT_SEQUENCE: list[tuple[str, dict[str, float], float]] = [
    ("approach", _arm(0.0, 0.45, -0.70, 0.40, 0.0, GRIPPER_OPEN), 2.5),
    ("reach", _arm(0.0, 0.65, -0.95, 0.55, 0.0, GRIPPER_OPEN), 2.0),
    ("snip", _arm(0.0, 0.65, -0.95, 0.55, 0.0, GRIPPER_CLOSED), 1.2),
    ("lift", _arm(0.0, 0.35, -0.55, 0.30, 0.0, GRIPPER_CLOSED), 2.0),
    ("release", _arm(0.0, 0.35, -0.55, 0.30, 0.0, GRIPPER_OPEN), 1.2),
]

DEFAULT_RAIL_M = 0.15


class LeafPickImitation(Node):
    def __init__(self):
        super().__init__("leaf_pick_imitation")
        self._positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._arm_client = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
        self._rail_client = ActionClient(self, FollowJointTrajectory, RAIL_ACTION)

    def _on_js(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in ARM_JOINTS or name == RAIL_JOINT:
                self._positions[name] = float(position)

    def wait_for_state(self, need_rail: bool, timeout_sec: float = 20.0):
        needed = list(ARM_JOINTS)
        if need_rail:
            needed.append(RAIL_JOINT)
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(name in self._positions for name in needed):
                return
        missing = [name for name in needed if name not in self._positions]
        raise RuntimeError(f"Missing joint states for: {missing}")

    def send_arm(self, targets: dict[str, float], duration_sec: float):
        self._send(self._arm_client, ARM_ACTION, targets, duration_sec)

    def send_rail(self, position_m: float, duration_sec: float):
        self._send(
            self._rail_client,
            RAIL_ACTION,
            {RAIL_JOINT: position_m},
            duration_sec,
        )

    def _send(
        self,
        client: ActionClient,
        action_name: str,
        targets: dict[str, float],
        duration_sec: float,
    ):
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Action server {action_name} is not available")

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(targets.keys())
        point = JointTrajectoryPoint()
        point.positions = [float(targets[name]) for name in traj.joint_names]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1.0) * 1e9),
        )
        traj.points = [point]
        goal.trajectory = traj

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"Trajectory goal rejected on {action_name}")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration_sec + 15.0
        )
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(result.error_string or f"error_code={result.error_code}")


def clamp(name: str, value: float) -> float:
    lower, upper = JOINT_LIMITS[name]
    return max(lower, min(upper, value))


def scale_arm_pose(pose: dict[str, float], scale: float) -> dict[str, float]:
    """Shrink arm joints 1–5 toward zero; keep gripper open/closed extremes."""
    out: dict[str, float] = {}
    for name, value in pose.items():
        if name == "6":
            out[name] = value
        else:
            out[name] = value * scale
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Imitate moving to the front and picking/cutting a leaf "
            "with the jaw gripper (no cutter tool)."
        )
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
        "--return-home",
        action="store_true",
        help="Return to the starting joint positions after the sequence.",
    )
    parser.add_argument(
        "--move-rail",
        action="store_true",
        help=(
            f"Advance rail_joint to {DEFAULT_RAIL_M} m before the arm sequence "
            "(view mode only; hardware rail hold rejects non-zero targets)."
        ),
    )
    parser.add_argument(
        "--rail-m",
        type=float,
        default=DEFAULT_RAIL_M,
        help=f"Rail target in metres when --move-rail is set (default {DEFAULT_RAIL_M}).",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Scale arm joints 1–5 toward zero (0.2–1.0). Use <1 for a gentler first try.",
    )
    parser.add_argument(
        "--duration-scale",
        type=float,
        default=1.0,
        help="Multiply each waypoint duration (default 1.0).",
    )
    args = parser.parse_args(argv)

    if args.hardware and not args.allow_hardware:
        print(
            "Refusing to command hardware without --allow-hardware.",
            file=sys.stderr,
        )
        return 2

    if not 0.2 <= args.scale <= 1.0:
        print("--scale must be between 0.2 and 1.0.", file=sys.stderr)
        return 2
    if args.duration_scale <= 0.0:
        print("--duration-scale must be > 0.", file=sys.stderr)
        return 2
    if abs(args.rail_m) > 0.5:
        print("--rail-m must be within ±0.5 m.", file=sys.stderr)
        return 2
    if args.move_rail and args.hardware:
        print(
            "Refusing --move-rail with --hardware: physical rail motion is disabled.",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = LeafPickImitation()
    try:
        node.get_logger().info("Waiting for /joint_states …")
        node.wait_for_state(need_rail=args.move_rail or args.return_home)
        start_arm = {name: node._positions[name] for name in ARM_JOINTS}
        start_rail = node._positions.get(RAIL_JOINT)

        if args.move_rail:
            rail_target = clamp(RAIL_JOINT, args.rail_m)
            node.get_logger().info(f"Moving rail to {rail_target:.3f} m …")
            node.send_rail(rail_target, 2.0 * args.duration_scale)

        for label, pose, duration in DEFAULT_SEQUENCE:
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
            if args.move_rail and start_rail is not None:
                node.send_rail(start_rail, 2.0 * args.duration_scale)

        node.get_logger().info("Leaf pick imitation finished.")
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
