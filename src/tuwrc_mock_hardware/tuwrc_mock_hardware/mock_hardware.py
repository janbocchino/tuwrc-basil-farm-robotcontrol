#!/usr/bin/env python3
"""Mock FollowJointTrajectory servers for view mode (no Gazebo, no USB)."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


ARM_JOINTS = ["1", "2", "3", "4", "5", "6"]
RAIL_JOINTS = ["rail_joint"]
ALL_JOINTS = RAIL_JOINTS + ARM_JOINTS

# Measured URDF limits from Max's so101_base.xacro (+ provisional rail).
JOINT_LIMITS = {
    "rail_joint": (-0.5, 0.5),
    "1": (-2.06780610, 2.06780610),
    "2": (-1.85304879, 1.85304879),
    "3": (-1.68737887, 1.68891285),
    "4": (-1.80702937, 1.80702937),
    "5": (-2.89308777, 2.89308777),
    "6": (-0.18561168, 2.03252454),
}


class MockHardwareNode(Node):
    def __init__(self):
        super().__init__("tuwrc_mock_hardware")
        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._positions = {name: 0.0 for name in ALL_JOINTS}
        self._active = False

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter(
            "arm_action_name",
            "/six_motor_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "rail_action_name",
            "/rail_controller/follow_joint_trajectory",
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / max(rate, 1.0), self._publish_joint_states)

        self._arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("arm_action_name").value),
            execute_callback=self._execute_arm,
            goal_callback=self._accept_arm_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb_group,
        )
        self._rail_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("rail_action_name").value),
            execute_callback=self._execute_rail,
            goal_callback=self._accept_rail_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            "Mock hardware ready: arm + rail FollowJointTrajectory servers active"
        )

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            msg.name = list(ALL_JOINTS)
            msg.position = [self._positions[name] for name in ALL_JOINTS]
        self._js_pub.publish(msg)

    def _accept_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _accept_arm_goal(self, goal_request):
        names = list(goal_request.trajectory.joint_names)
        if not names or any(name not in ARM_JOINTS for name in names):
            self.get_logger().warn(f"Rejecting arm goal with joints: {names}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _accept_rail_goal(self, goal_request):
        names = list(goal_request.trajectory.joint_names)
        if names != RAIL_JOINTS:
            self.get_logger().warn(f"Rejecting rail goal with joints: {names}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_arm(self, goal_handle):
        return self._execute_trajectory(goal_handle, ARM_JOINTS)

    def _execute_rail(self, goal_handle):
        return self._execute_trajectory(goal_handle, RAIL_JOINTS)

    def _execute_trajectory(self, goal_handle, allowed_joints):
        result = FollowJointTrajectory.Result()
        trajectory: JointTrajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)

        try:
            if not trajectory.points:
                raise ValueError("Trajectory has no points")
            for name in names:
                if name not in allowed_joints:
                    raise ValueError(f"Joint {name} not allowed on this controller")

            with self._lock:
                start = {name: self._positions[name] for name in names}

            previous_t = 0.0
            previous_targets = start
            for point in trajectory.points:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result

                target_t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                duration = max(target_t - previous_t, 0.0)
                targets = {
                    name: self._clamp(name, float(value))
                    for name, value in zip(names, point.positions)
                }
                self._interpolate(previous_targets, targets, duration, goal_handle)
                previous_targets = targets
                previous_t = target_t

            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result
        except Exception as exc:
            self.get_logger().error(f"Mock trajectory failed: {exc}")
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            goal_handle.abort()
            return result

    def _interpolate(self, start, targets, duration, goal_handle):
        steps = max(int(duration * 50), 1)
        for step in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                return
            alpha = step / steps
            with self._lock:
                for name, end_value in targets.items():
                    self._positions[name] = (
                        start[name] + (end_value - start[name]) * alpha
                    )
            time.sleep(duration / steps if duration > 0 else 0.02)

    @staticmethod
    def _clamp(name: str, value: float) -> float:
        lower, upper = JOINT_LIMITS[name]
        if math.isnan(value):
            raise ValueError(f"NaN target for {name}")
        return max(lower, min(upper, value))


def main():
    rclpy.init()
    node = MockHardwareNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
