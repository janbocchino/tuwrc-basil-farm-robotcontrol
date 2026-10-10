#!/usr/bin/env python3
"""Real rail driver: FollowJointTrajectory server over serial to Arduino stepper."""

from __future__ import annotations

import threading
import time

import rclpy
import serial
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

RAIL_LIMITS_M = (-0.5, 0.5)


class RailDriverNode(Node):
    def __init__(self):
        super().__init__("tuwrc_rail_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 9600)
        # Steps per meter — unknown until calibrated. Set via parameter or
        # run the calibration procedure (move a known distance, count steps).
        self.declare_parameter("steps_per_meter", 10000.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter(
            "rail_action_name", "/rail_controller/follow_joint_trajectory"
        )

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        self._steps_per_meter = float(self.get_parameter("steps_per_meter").value)
        rate = float(self.get_parameter("publish_rate_hz").value)

        self._lock = threading.Lock()
        self._pos_m = 0.0  # position in meters, 0 = center

        self._serial = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2.0)  # Arduino resets on connect — wait for it
        self._serial.reset_input_buffer()
        self.get_logger().info(f"Connected to Arduino rail on {port} @ {baud} baud")
        self._drain()

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
        self.create_service(Trigger, "~/set_home", self._set_home)
        self.get_logger().warn(
            f"Rail driver ready. steps_per_meter={self._steps_per_meter:.0f}. "
            "Move rail to physical home, then call ~/set_home to zero position."
        )

    # ------------------------------------------------------------------ helpers

    def _set_home(self, _request, response):
        with self._lock:
            self._pos_m = 0.0
        self.get_logger().info("Rail home set: current position is now 0.0 m")
        response.success = True
        response.message = "Rail position reset to 0.0 m"
        return response

    def _drain(self):
        time.sleep(0.1)
        while self._serial.in_waiting:
            self._serial.readline()

    def _send(self, cmd: str):
        self._serial.write((cmd + "\n").encode())

    def _wait_for_done(self, timeout_sec: float, goal_handle) -> bool:
        """Read serial until Arduino says done. Returns True on success."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            if self._serial.in_waiting:
                line = self._serial.readline().decode(errors="replace").strip()
                self.get_logger().debug(f"Arduino: {line}")
                if "Target reached" in line or "Stopped" in line:
                    return True
            time.sleep(0.02)
        self.get_logger().warn("Timeout waiting for Arduino — assuming done")
        return True

    # ------------------------------------------------------------------ ROS

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["rail_joint"]
        with self._lock:
            msg.position = [self._pos_m]
        self._js_pub.publish(msg)

    def _accept_goal(self, goal_request):
        if list(goal_request.trajectory.joint_names) != ["rail_joint"]:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory

        for point in trajectory.points:
            if not point.positions:
                continue

            if goal_handle.is_cancel_requested:
                self._send("s")
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            target_m = float(point.positions[0])
            target_m = max(RAIL_LIMITS_M[0], min(RAIL_LIMITS_M[1], target_m))

            with self._lock:
                delta_m = target_m - self._pos_m

            if abs(delta_m) < 5e-4:  # less than 0.5 mm — skip
                continue

            steps = int(round(abs(delta_m) * self._steps_per_meter))
            if steps == 0:
                continue

            direction = "r" if delta_m > 0 else "f"
            self._serial.reset_input_buffer()
            self._send(f"{direction}{steps}")
            self.get_logger().info(
                f"Rail → {direction}{steps} steps  ({delta_m * 100:+.1f} cm)"
            )

            # Arduino default: pd=50 µs → 10 000 steps/s; add generous margin
            duration_estimate = steps / max(self._steps_per_meter * 0.5, 1.0) + 5.0
            success = self._wait_for_done(duration_estimate, goal_handle)

            if not success:
                self._send("s")
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            with self._lock:
                self._pos_m = target_m

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def destroy_node(self):
        try:
            self._send("s")
            self._serial.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RailDriverNode()
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
