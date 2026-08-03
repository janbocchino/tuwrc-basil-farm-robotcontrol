#!/usr/bin/env python3
"""Publish a fixed rail_joint=0 and reject motion while arm hardware is live."""

from __future__ import annotations

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RailHoldNode(Node):
    def __init__(self):
        super().__init__("tuwrc_rail_hold")
        self.declare_parameter("position", 0.0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter(
            "rail_action_name",
            "/rail_controller/follow_joint_trajectory",
        )

        self._position = float(self.get_parameter("position").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / max(rate, 1.0), self._publish)

        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("rail_action_name").value),
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().warn(
            "Rail hold active: rail_joint is fixed at "
            f"{self._position:.3f} m. Physical rail hardware is not supported yet."
        )

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["rail_joint"]
        msg.position = [self._position]
        self._js_pub.publish(msg)

    def _accept_goal(self, goal_request):
        names = list(goal_request.trajectory.joint_names)
        if names != ["rail_joint"]:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        for point in trajectory.points:
            if not point.positions:
                continue
            target = float(point.positions[0])
            if abs(target - self._position) > 1e-4:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = (
                    "Physical rail motion is disabled. Rail stays at "
                    f"{self._position:.3f} m until a real rail driver is added."
                )
                self.get_logger().error(result.error_string)
                goal_handle.abort()
                return result

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main():
    rclpy.init()
    node = RailHoldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
