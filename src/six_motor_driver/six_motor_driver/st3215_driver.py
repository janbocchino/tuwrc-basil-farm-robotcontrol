#!/usr/bin/env python3
"""ROS 2 FollowJointTrajectory server for three ST3215 servos on one bus."""

from __future__ import annotations

import glob
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


STEPS_PER_RADIAN = 4096.0 / (2.0 * math.pi)
DEFAULT_JOINT_NAMES = [
    "motor_joint_1",
    "motor_joint_2",
    "motor_joint_3",
]


def import_st3215():
    try:
        from st3215 import ST3215
        return ST3215
    except ImportError:
        local_library = Path.home() / "libs" / "python-st3215"
        if local_library.is_dir():
            sys.path.insert(0, str(local_library))
            from st3215 import ST3215
            return ST3215
        raise


ST3215 = import_st3215()


class ThreeMotorDriver(Node):
    """Translate three ROS joint angles to three ST3215 servo positions."""

    def __init__(self):
        super().__init__("three_motor_driver")

        self.declare_parameter("port", "")
        self.declare_parameter("servo_id_1", 1)
        self.declare_parameter("servo_id_2", 2)
        self.declare_parameter("servo_id_3", 3)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter(
            "action_name", "/real_motor_controller/follow_joint_trajectory"
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("max_relative_angle", 1.0)
        self.declare_parameter("max_speed_steps_s", 800)
        self.declare_parameter("acceleration", 30)
        self.declare_parameter("center_on_start", True)
        self.declare_parameter("center_position_steps", 2048)
        self.declare_parameter("center_speed_steps_s", 250)
        self.declare_parameter("center_timeout_s", 12.0)
        self.declare_parameter("center_tolerance_steps", 20)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("disable_torque_on_shutdown", True)

        self.port = self._find_port(str(self.get_parameter("port").value))
        self.servo_ids = [
            int(self.get_parameter("servo_id_1").value),
            int(self.get_parameter("servo_id_2").value),
            int(self.get_parameter("servo_id_3").value),
        ]
        self.joint_names = list(self.get_parameter("joint_names").value)
        if len(self.joint_names) != 3:
            raise RuntimeError("Parameter joint_names muss genau 3 Namen enthalten")
        if len(set(self.servo_ids)) != 3:
            raise RuntimeError("Die drei Servo-IDs müssen unterschiedlich sein")

        self.action_name = str(self.get_parameter("action_name").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.requested_limit = float(
            self.get_parameter("max_relative_angle").value
        )
        self.max_speed = int(self.get_parameter("max_speed_steps_s").value)
        self.acceleration = int(self.get_parameter("acceleration").value)
        self.center_on_start = bool(
            self.get_parameter("center_on_start").value
        )
        self.center_position = int(
            self.get_parameter("center_position_steps").value
        )
        self.center_speed = int(
            self.get_parameter("center_speed_steps_s").value
        )
        self.center_timeout = float(
            self.get_parameter("center_timeout_s").value
        )
        self.center_tolerance = int(
            self.get_parameter("center_tolerance_steps").value
        )
        self.disable_on_shutdown = bool(
            self.get_parameter("disable_torque_on_shutdown").value
        )
        if not 0 <= self.center_position <= 4095:
            raise RuntimeError(
                "center_position_steps muss zwischen 0 und 4095 liegen"
            )
        if not 1 <= self.center_speed <= 3400:
            raise RuntimeError(
                "center_speed_steps_s muss zwischen 1 und 3400 liegen"
            )
        if self.center_timeout <= 0.0:
            raise RuntimeError("center_timeout_s muss größer als 0 sein")
        if self.center_tolerance < 0:
            raise RuntimeError(
                "center_tolerance_steps darf nicht negativ sein"
            )

        self.io_lock = threading.Lock()
        self.servo = ST3215(self.port)
        self.zero_steps: list[int] = []
        self.lower_limits: list[float] = []
        self.upper_limits: list[float] = []
        self.last_angles = [0.0, 0.0, 0.0]

        initial_positions: list[int] = []
        with self.io_lock:
            for servo_id in self.servo_ids:
                if not self.servo.PingServo(servo_id):
                    raise RuntimeError(
                        f"Servo ID {servo_id} antwortet nicht auf {self.port}"
                    )
                position = self.servo.ReadPosition(servo_id)
                if position is None:
                    raise RuntimeError(
                        f"Startposition von Servo ID {servo_id} nicht lesbar"
                    )
                initial_positions.append(int(position))

        if self.center_on_start:
            self.zero_steps = self._center_servos(initial_positions)
        else:
            self.zero_steps = initial_positions
            self.get_logger().warning(
                "Automatische Mittelstellung ist deaktiviert; "
                "aktuelle Positionen werden als 0 rad verwendet"
            )

        for servo_id, zero in zip(self.servo_ids, self.zero_steps):
            negative_room = zero / STEPS_PER_RADIAN
            positive_room = (4095 - zero) / STEPS_PER_RADIAN
            lower = max(-self.requested_limit, -negative_room)
            upper = min(self.requested_limit, positive_room)
            if upper - lower < 0.30:
                direction = 800 if zero < 2048 else -800
                raise RuntimeError(
                    f"Servo ID {servo_id} steht bei {zero} nahe einer Grenze. "
                    "Richtung Mitte bewegen: "
                    f"ros2 run six_motor_driver configure_servo_id --ros-args -- --port {self.port} "
                    f"--id {servo_id} --jog {direction}"
                )
            self.lower_limits.append(lower)
            self.upper_limits.append(upper)

        with self.io_lock:
            for servo_id in self.servo_ids:
                self.servo.StartServo(servo_id)
                self.servo.SetMode(servo_id, 0)
                self.servo.SetAcceleration(servo_id, self.acceleration)
                self.servo.SetSpeed(servo_id, min(300, self.max_speed))

        self.joint_publisher = self.create_publisher(
            JointState, joint_states_topic, 10
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / publish_rate, self.publish_joint_state)

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            goal_callback=self.accept_goal,
            cancel_callback=self.accept_cancel,
            execute_callback=self.execute_trajectory,
        )

        details = ", ".join(
            f"{joint}=ID {servo_id}, zero={zero}, "
            f"[{lower:.2f},{upper:.2f}] rad"
            for joint, servo_id, zero, lower, upper in zip(
                self.joint_names,
                self.servo_ids,
                self.zero_steps,
                self.lower_limits,
                self.upper_limits,
            )
        )
        self.get_logger().info(
            f"Drei ST3215 bereit auf {self.port}: {details}; "
            f"Action={self.action_name}"
        )

    @staticmethod
    def _find_port(configured_port: str) -> str:
        if configured_port:
            return configured_port
        candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        if not candidates:
            raise RuntimeError("Kein /dev/ttyACM* oder /dev/ttyUSB* gefunden")
        return candidates[0]

    @staticmethod
    def _write_succeeded(result) -> bool:
        if result is None:
            return False
        if isinstance(result, tuple):
            return all(value == 0 for value in result)
        return bool(result)

    def _center_servos(self, initial_positions: list[int]) -> list[int]:
        start_text = ", ".join(
            f"ID {servo_id}: {position}"
            for servo_id, position in zip(
                self.servo_ids, initial_positions
            )
        )
        self.get_logger().warning(
            "Automatische Startbewegung: alle Servos fahren langsam auf "
            f"{self.center_position} Schritte. Ausgangspositionen: {start_text}"
        )

        try:
            with self.io_lock:
                for servo_id in self.servo_ids:
                    if not self._write_succeeded(
                        self.servo.StartServo(servo_id)
                    ):
                        raise RuntimeError(
                            f"Drehmoment für Servo ID {servo_id} "
                            "nicht aktiviert"
                        )
                    if not self._write_succeeded(
                        self.servo.SetMode(servo_id, 0)
                    ):
                        raise RuntimeError(
                            f"Positionsmodus für Servo ID {servo_id} "
                            "nicht gesetzt"
                        )
                    if (
                        self.servo.SetAcceleration(
                            servo_id, self.acceleration
                        )
                        is None
                    ):
                        raise RuntimeError(
                            f"Beschleunigung für Servo ID {servo_id} "
                            "nicht gesetzt"
                        )
                    if self.servo.SetSpeed(
                        servo_id, self.center_speed
                    ) is None:
                        raise RuntimeError(
                            f"Startgeschwindigkeit für Servo ID {servo_id} "
                            "nicht gesetzt"
                        )

                for servo_id in self.servo_ids:
                    if self.servo.WritePosition(
                        servo_id, self.center_position
                    ) is None:
                        raise RuntimeError(
                            f"Mittelstellung für Servo ID {servo_id} "
                            "nicht gesendet"
                        )

            deadline = time.monotonic() + self.center_timeout
            measured = list(initial_positions)
            while time.monotonic() < deadline:
                with self.io_lock:
                    for index, servo_id in enumerate(self.servo_ids):
                        position = self.servo.ReadPosition(servo_id)
                        if position is not None:
                            measured[index] = int(position)

                if all(
                    abs(position - self.center_position)
                    <= self.center_tolerance
                    for position in measured
                ):
                    self.get_logger().info(
                        "Alle Servos in Mittelstellung: "
                        + ", ".join(
                            f"ID {servo_id}={position}"
                            for servo_id, position in zip(
                                self.servo_ids, measured
                            )
                        )
                    )
                    return measured
                time.sleep(0.05)

            raise RuntimeError(
                "Mittelstellung nicht rechtzeitig erreicht: "
                + ", ".join(
                    f"ID {servo_id}={position}"
                    for servo_id, position in zip(
                        self.servo_ids, measured
                    )
                )
            )
        except Exception:
            with self.io_lock:
                for servo_id in self.servo_ids:
                    self.servo.StopServo(servo_id)
            raise

    def steps_to_angle(self, index: int, steps: int) -> float:
        return (steps - self.zero_steps[index]) / STEPS_PER_RADIAN

    def angle_to_steps(self, index: int, angle: float) -> int:
        bounded = min(
            self.upper_limits[index],
            max(self.lower_limits[index], angle),
        )
        return int(round(self.zero_steps[index] + bounded * STEPS_PER_RADIAN))

    def read_angles(self) -> list[float]:
        with self.io_lock:
            for index, servo_id in enumerate(self.servo_ids):
                steps = self.servo.ReadPosition(servo_id)
                if steps is not None:
                    self.last_angles[index] = self.steps_to_angle(
                        index, int(steps)
                    )
        return list(self.last_angles)

    def publish_joint_state(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.joint_names)
        message.position = self.read_angles()
        self.joint_publisher.publish(message)

    def accept_goal(self, request):
        if list(request.trajectory.joint_names) != self.joint_names:
            self.get_logger().error(
                f"Erwartete Gelenke {self.joint_names}, "
                f"erhalten {list(request.trajectory.joint_names)}"
            )
            return GoalResponse.REJECT
        if not request.trajectory.points:
            self.get_logger().error("Leere Trajektorie wurde abgelehnt")
            return GoalResponse.REJECT

        for point in request.trajectory.points:
            if len(point.positions) != 3:
                self.get_logger().error(
                    "Jeder Trajektorienpunkt muss 3 Positionen enthalten"
                )
                return GoalResponse.REJECT
            for index, target in enumerate(point.positions):
                if not (
                    self.lower_limits[index]
                    <= float(target)
                    <= self.upper_limits[index]
                ):
                    self.get_logger().error(
                        f"{self.joint_names[index]}: Ziel {target:.3f} "
                        f"außerhalb [{self.lower_limits[index]:.3f}, "
                        f"{self.upper_limits[index]:.3f}]"
                    )
                    return GoalResponse.REJECT

        self.get_logger().info(
            f"Trajektorie angenommen: {len(request.trajectory.points)} Punkte, "
            f"Endziel={list(request.trajectory.points[-1].positions)}"
        )
        return GoalResponse.ACCEPT

    @staticmethod
    def accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    @staticmethod
    def duration_seconds(duration) -> float:
        return duration.sec + duration.nanosec / 1e9

    def stop_all(self):
        with self.io_lock:
            for servo_id in self.servo_ids:
                self.servo.StopServo(servo_id)

    def execute_trajectory(self, goal_handle):
        result = FollowJointTrajectory.Result()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(self.joint_names)
        start = time.monotonic()
        previous_time = 0.0
        previous_angles = self.read_angles()

        try:
            with self.io_lock:
                for servo_id in self.servo_ids:
                    self.servo.StartServo(servo_id)
                    self.servo.SetMode(servo_id, 0)

            for point in goal_handle.request.trajectory.points:
                if goal_handle.is_cancel_requested:
                    self.stop_all()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result

                targets = [float(value) for value in point.positions]
                target_time = self.duration_seconds(point.time_from_start)
                segment_time = max(0.05, target_time - previous_time)
                target_steps = [
                    self.angle_to_steps(index, target)
                    for index, target in enumerate(targets)
                ]

                with self.io_lock:
                    for index, servo_id in enumerate(self.servo_ids):
                        previous_steps = self.angle_to_steps(
                            index, previous_angles[index]
                        )
                        distance_steps = abs(
                            target_steps[index] - previous_steps
                        )
                        speed = max(
                            20,
                            min(
                                self.max_speed,
                                int(distance_steps / segment_time),
                            ),
                        )
                        self.servo.SetAcceleration(
                            servo_id, self.acceleration
                        )
                        self.servo.SetSpeed(servo_id, speed)
                        written = self.servo.WritePosition(
                            servo_id, target_steps[index]
                        )
                        if written is None:
                            raise RuntimeError(
                                f"Ziel für Servo ID {servo_id} nicht gesendet"
                            )

                self.get_logger().info(
                    f"Motorbefehl: rad={targets}, Schritte={target_steps}"
                )

                while time.monotonic() - start < target_time:
                    if goal_handle.is_cancel_requested:
                        self.stop_all()
                        goal_handle.canceled()
                        result.error_code = (
                            FollowJointTrajectory.Result.SUCCESSFUL
                        )
                        return result

                    actual = self.read_angles()
                    feedback.header.stamp = self.get_clock().now().to_msg()
                    feedback.desired.positions = targets
                    feedback.actual.positions = actual
                    feedback.error.positions = [
                        target - measured
                        for target, measured in zip(targets, actual)
                    ]
                    goal_handle.publish_feedback(feedback)
                    time.sleep(0.02)

                previous_time = target_time
                previous_angles = targets

            final_targets = [
                float(value)
                for value in goal_handle.request.trajectory.points[
                    -1
                ].positions
            ]
            final_actual = self.read_angles()
            largest_error = max(
                abs(target - actual)
                for target, actual in zip(final_targets, final_actual)
            )
            if largest_error > 0.08:
                result.error_code = (
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                )
                result.error_string = (
                    f"Ziel={final_targets}, Ist={final_actual}"
                )
                goal_handle.abort()
                return result

            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            self.get_logger().info(
                f"Trajektorie beendet: Ziel={final_targets}, "
                f"Ist={final_actual}"
            )
            goal_handle.succeed()
            return result
        except Exception as error:
            self.stop_all()
            self.get_logger().error(f"Trajektorie fehlgeschlagen: {error}")
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            goal_handle.abort()
            return result

    def destroy_node(self):
        try:
            with self.io_lock:
                if self.disable_on_shutdown:
                    for servo_id in self.servo_ids:
                        self.servo.StopServo(servo_id)
                self.servo.portHandler.closePort()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ThreeMotorDriver()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
