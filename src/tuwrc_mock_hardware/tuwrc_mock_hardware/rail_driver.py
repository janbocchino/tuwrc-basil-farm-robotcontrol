#!/usr/bin/env python3
"""Real rail driver: FollowJointTrajectory server over serial to Arduino stepper.

Talks to firmware/rail_stepper. Position is measured from the left end
(lm_left = 0 m, increasing towards lm_right) and only valid after calling
~/calibrate, which the Arduino forgets on every reconnect.
"""

from __future__ import annotations

import threading
import time

import rclpy
import serial
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger


class RailDriverNode(Node):
    def __init__(self):
        super().__init__("tuwrc_rail_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        # Measured: 1000 steps = 1 cm.
        self.declare_parameter("steps_per_meter", 100000.0)
        # Arduino default pd=50 µs high + 50 µs low → 10 000 steps/s.
        self.declare_parameter("step_rate_hz", 10000.0)
        self.declare_parameter("calibration_timeout_sec", 300.0)
        # Measured travel between lm_left and lm_right; must match the URDF
        # rail_upper limit. Calibration warns if it measures something else.
        self.declare_parameter("expected_length_m", 0.715)
        self.declare_parameter("length_tolerance_m", 0.01)
        self.declare_parameter(
            "rail_action_name", "/rail_controller/follow_joint_trajectory"
        )
        self.declare_parameter("lm_left_topic", "/rail/lm_left")
        self.declare_parameter("lm_right_topic", "/rail/lm_right")
        self.declare_parameter("position_cm_topic", "/rail/position_cm")

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        self._steps_per_meter = float(self.get_parameter("steps_per_meter").value)
        self._step_rate = float(self.get_parameter("step_rate_hz").value)
        self._calibration_timeout = float(
            self.get_parameter("calibration_timeout_sec").value
        )
        self._expected_length_m = float(self.get_parameter("expected_length_m").value)
        self._length_tolerance_m = float(self.get_parameter("length_tolerance_m").value)

        self._lock = threading.Lock()
        self._pos_steps = 0  # from the Arduino's ST lines
        self._homed = False
        self._length_steps: int | None = None  # set by a successful ~/calibrate
        self._move_done = threading.Event()
        self._calibration_done = threading.Event()
        self._calibration_result = ""

        self._serial = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2.0)  # Arduino resets on connect — wait for it
        self._serial.reset_input_buffer()
        self.get_logger().info(f"Connected to Arduino rail on {port} @ {baud} baud")

        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._position_cm_pub = self.create_publisher(
            Float64, str(self.get_parameter("position_cm_topic").value), 10
        )
        # Switches lm_left (D5) and lm_right (D4): True = closed, False = open.
        self._lm_left_pub = self.create_publisher(
            Bool, str(self.get_parameter("lm_left_topic").value), 10
        )
        self._lm_right_pub = self.create_publisher(
            Bool, str(self.get_parameter("lm_right_topic").value), 10
        )
        # The Arduino reports status at 50 Hz; the reader thread republishes
        # it, so publishing continues while a move or calibration blocks.
        self._reader_running = True
        self._reader = threading.Thread(target=self._read_serial, daemon=True)
        self._reader.start()

        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("rail_action_name").value),
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.create_service(Trigger, "~/calibrate", self._calibrate)
        self.get_logger().warn(
            f"Rail driver ready. steps_per_meter={self._steps_per_meter:.0f}. "
            "Rail is NOT calibrated: call ~/calibrate before moving it."
        )

    # ------------------------------------------------------------------ serial

    def _read_serial(self):
        """Read every Arduino line: ST status reports and command replies."""
        while self._reader_running:
            try:
                raw = self._serial.readline()
            except serial.SerialException as error:
                self.get_logger().error(f"Serial read failed: {error}")
                return
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("ST "):
                self._on_status(line[3:].split())
                continue
            self.get_logger().info(f"Arduino: {line}")
            if line.startswith("Calibrated") or line.startswith("Calibration failed"):
                self._calibration_result = line
                self._calibration_done.set()
            elif "Target reached" in line or "Stopped" in line:
                self._move_done.set()

    def _on_status(self, values: list[str]):
        if len(values) != 4:
            return
        try:
            pos_steps = int(values[2])
        except ValueError:
            return
        with self._lock:
            self._pos_steps = pos_steps
            self._homed = values[3] == "1"
        pos_m = pos_steps / self._steps_per_meter

        self._lm_left_pub.publish(Bool(data=values[0] == "1"))
        self._lm_right_pub.publish(Bool(data=values[1] == "1"))
        self._position_cm_pub.publish(Float64(data=pos_m * 100.0))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["rail_joint"]
        msg.position = [pos_m]
        self._js_pub.publish(msg)

    def _send(self, cmd: str):
        self._serial.write((cmd + "\n").encode())

    def _is_calibrated(self) -> bool:
        with self._lock:
            return self._homed and self._length_steps is not None

    # ------------------------------------------------------------------ ROS

    def _calibrate(self, _request, response):
        self.get_logger().info(
            "Calibrating rail: lm_left → lm_right → middle → 0 (lm_left) …"
        )
        with self._lock:
            self._length_steps = None
        self._calibration_done.clear()
        self._send("h")

        if not self._calibration_done.wait(timeout=self._calibration_timeout):
            self._send("s")
            response.success = False
            response.message = (
                f"Calibration timed out after {self._calibration_timeout:.0f} s"
            )
            self.get_logger().error(response.message)
            return response

        result = self._calibration_result
        if not result.startswith("Calibrated"):
            response.success = False
            response.message = result
            self.get_logger().error(result)
            return response

        length_steps = int(result.split()[1])
        with self._lock:
            self._length_steps = length_steps
        length_cm = length_steps / self._steps_per_meter * 100.0
        response.success = True
        response.message = (
            f"Rail calibrated: length {length_cm:.1f} cm ({length_steps} steps), "
            "now at 0 cm (lm_left)"
        )
        self.get_logger().info(response.message)
        expected_cm = self._expected_length_m * 100.0
        if abs(length_cm - expected_cm) > self._length_tolerance_m * 100.0:
            warning = (
                f"Measured length {length_cm:.1f} cm differs from the expected "
                f"{expected_cm:.1f} cm (URDF rail_upper). Check the switches and "
                "steps_per_meter."
            )
            self.get_logger().warn(warning)
            response.message += f". WARNING: {warning}"
        return response

    def _accept_goal(self, goal_request):
        if list(goal_request.trajectory.joint_names) != ["rail_joint"]:
            return GoalResponse.REJECT
        if not self._is_calibrated():
            self.get_logger().error(
                "Rail goal rejected: not calibrated. Call "
                "/tuwrc_rail_driver/calibrate first."
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        with self._lock:
            length_m = self._length_steps / self._steps_per_meter

        for point in trajectory.points:
            if not point.positions:
                continue

            if goal_handle.is_cancel_requested:
                self._send("s")
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            target_m = max(0.0, min(length_m, float(point.positions[0])))
            target_steps = int(round(target_m * self._steps_per_meter))
            with self._lock:
                distance_steps = abs(target_steps - self._pos_steps)
            if distance_steps == 0:
                continue

            self._move_done.clear()
            self._send(f"a{target_steps}")
            self.get_logger().info(f"Rail → {target_m * 100:.1f} cm ({target_steps} steps)")

            timeout = distance_steps / max(self._step_rate, 1.0) * 1.5 + 5.0
            deadline = time.time() + timeout
            while not self._move_done.wait(timeout=0.02):
                if goal_handle.is_cancel_requested:
                    self._send("s")
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
                if time.time() > deadline:
                    self._send("s")
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = f"Rail move timed out after {timeout:.1f} s"
                    self.get_logger().error(result.error_string)
                    return result

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def destroy_node(self):
        self._reader_running = False
        try:
            self._send("s")
            self._reader.join(timeout=2.0)
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
