#!/usr/bin/env python3
"""Bounded joint-space motion example for mock or real arm controllers."""

from __future__ import annotations

import argparse
import math
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
ARM_JOINTS = ["1", "2", "3", "4", "5", "6"]
JOINT_LIMITS = {
    "1": (-2.06780610, 2.06780610),
    "2": (-1.85304879, 1.85304879),
    "3": (-1.68737887, 1.68891285),
    "4": (-1.80702937, 1.80702937),
    "5": (-2.89308777, 2.89308777),
    "6": (-0.18561168, 2.03252454),
}
# Conservative default deltas (radians).
DEFAULT_DELTAS = {
    "1": 0.15,
    "2": 0.10,
    "3": -0.10,
    "4": 0.08,
    "5": 0.0,
    "6": 0.10,
}


class SmallArmMotion(Node):
    def __init__(self):
        super().__init__("small_arm_motion")
        self._positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._client = ActionClient(self, FollowJointTrajectory, ARM_ACTION)

    def _on_js(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in ARM_JOINTS:
                self._positions[name] = float(position)

    def wait_for_state(self, timeout_sec: float = 20.0):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(name in self._positions for name in ARM_JOINTS):
                return
        missing = [name for name in ARM_JOINTS if name not in self._positions]
        raise RuntimeError(f"Missing joint states for: {missing}")

    def send(self, targets: dict[str, float], duration_sec: float):
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Action server {ARM_ACTION} is not available")

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

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("Trajectory goal was rejected")

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


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Move the arm by small bounded joint deltas."
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
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument(
        "--return-home",
        action="store_true",
        help="Return to the starting joint positions after the move.",
    )
    parser.add_argument(
        "--delta-deg",
        type=float,
        default=None,
        help="Optional uniform delta in degrees applied to joints 1-4 and 6.",
    )
    args = parser.parse_args(argv)

    if args.hardware and not args.allow_hardware:
        print(
            "Refusing to command hardware without --allow-hardware.",
            file=sys.stderr,
        )
        return 2

    deltas = dict(DEFAULT_DELTAS)
    if args.delta_deg is not None:
        delta = math.radians(args.delta_deg)
        if abs(delta) > math.radians(20.0):
            print("delta-deg must be within ±20 degrees.", file=sys.stderr)
            return 2
        deltas = {name: (delta if name != "5" else 0.0) for name in ARM_JOINTS}

    rclpy.init()
    node = SmallArmMotion()
    try:
        node.get_logger().info("Waiting for /joint_states …")
        node.wait_for_state()
        start = {name: node._positions[name] for name in ARM_JOINTS}
        targets = {
            name: clamp(name, start[name] + deltas[name]) for name in ARM_JOINTS
        }
        node.get_logger().info(f"Start: {start}")
        node.get_logger().info(f"Target: {targets}")
        node.send(targets, args.duration)
        if args.return_home:
            node.get_logger().info("Returning to start positions …")
            node.send(start, args.duration)
        node.get_logger().info("Small arm motion finished.")
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
