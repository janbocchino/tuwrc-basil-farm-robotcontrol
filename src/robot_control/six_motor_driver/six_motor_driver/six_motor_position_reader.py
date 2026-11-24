#!/usr/bin/env python3
"""Read six ST3215 positions continuously without commanding any motion."""

from __future__ import annotations

import glob
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

from .cyclic_calibration import CyclicJointCalibration
from .gripper_opening import GRIPPER_JOINT, GripperOpeningCalibration
from .st3215_import import load_st3215


DEFAULT_SERVO_IDS = [1, 2, 3, 4, 5, 6]
DEFAULT_NAMES = [f"servo_{servo_id}" for servo_id in DEFAULT_SERVO_IDS]


ST3215 = load_st3215()


class SixMotorPositionReader(Node):
    """Publish raw and midpoint-relative positions for six bus servos."""

    def __init__(self):
        super().__init__("six_motor_position_reader")
        self.declare_parameter("port", "")
        self.declare_parameter("servo_ids", DEFAULT_SERVO_IDS)
        self.declare_parameter("names", DEFAULT_NAMES)
        self.declare_parameter("read_rate_hz", 10.0)
        self.declare_parameter("print_rate_hz", 2.0)
        self.declare_parameter("lower_limits_steps", [0] * 6)
        self.declare_parameter("upper_limits_steps", [4095] * 6)
        self.declare_parameter("zero_positions_steps", [2048] * 6)
        self.declare_parameter("directions", [1] * 6)
        self.declare_parameter("gripper_closed_steps", -1)
        self.declare_parameter("gripper_open_steps", -1)
        self.declare_parameter("gripper_travel_m", 0.017)
        self.declare_parameter("warn_outside_limits", True)
        self.declare_parameter("require_all_servos", True)
        self.declare_parameter(
            "steps_topic", "/six_motor/positions_steps"
        )
        self.declare_parameter(
            "joint_states_topic", "/six_motor/joint_states"
        )

        self.port = self._find_port(str(self.get_parameter("port").value))
        self.servo_ids = [
            int(value) for value in self.get_parameter("servo_ids").value
        ]
        self.names = [
            str(value) for value in self.get_parameter("names").value
        ]
        lower_limits = list(
            self.get_parameter("lower_limits_steps").value
        )
        upper_limits = list(
            self.get_parameter("upper_limits_steps").value
        )
        zero_positions = list(
            self.get_parameter("zero_positions_steps").value
        )
        directions = list(self.get_parameter("directions").value)
        self.warn_outside = bool(
            self.get_parameter("warn_outside_limits").value
        )
        self.require_all = bool(
            self.get_parameter("require_all_servos").value
        )
        read_rate = float(self.get_parameter("read_rate_hz").value)
        print_rate = float(self.get_parameter("print_rate_hz").value)

        if len(self.servo_ids) != 6:
            raise RuntimeError("servo_ids muss genau 6 IDs enthalten")
        if len(set(self.servo_ids)) != 6:
            raise RuntimeError("Alle sechs Servo-IDs müssen eindeutig sein")
        if len(self.names) != 6:
            raise RuntimeError("names muss genau 6 Einträge enthalten")
        for parameter_name, values in (
            ("lower_limits_steps", lower_limits),
            ("upper_limits_steps", upper_limits),
            ("zero_positions_steps", zero_positions),
            ("directions", directions),
        ):
            if len(values) != 6:
                raise RuntimeError(
                    f"{parameter_name} muss genau 6 Einträge enthalten"
                )
        if read_rate <= 0.0 or print_rate < 0.0:
            raise RuntimeError("Ungültige Lese- oder Ausgaberate")

        self.calibrations = [
            CyclicJointCalibration(lower, upper, zero, direction)
            for lower, upper, zero, direction in zip(
                lower_limits,
                upper_limits,
                zero_positions,
                directions,
            )
        ]

        self.servo = ST3215(self.port)
        missing = [
            servo_id
            for servo_id in self.servo_ids
            if not self.servo.PingServo(servo_id)
        ]
        if missing and self.require_all:
            self.servo.portHandler.closePort()
            raise RuntimeError(
                "Diese Servo-IDs antworten nicht: "
                + ", ".join(str(value) for value in missing)
            )
        if missing:
            self.get_logger().warning(
                "Nicht erreichbare Servo-IDs werden als -1/NaN publiziert: "
                + ", ".join(str(value) for value in missing)
            )

        self.steps_publisher = self.create_publisher(
            Int32MultiArray,
            str(self.get_parameter("steps_topic").value),
            10,
        )
        self.joint_publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            10,
        )
        self.last_steps = [-1] * 6
        self.read_count = 0
        self.print_every = (
            max(1, int(round(read_rate / print_rate)))
            if print_rate > 0.0
            else 0
        )
        self.create_timer(1.0 / read_rate, self.read_and_publish)

        self.get_logger().info(
            f"Passiver Positionsleser auf {self.port}; IDs={self.servo_ids}; "
            "es werden keine Motorbefehle gesendet"
        )
        self.gripper = GripperOpeningCalibration(
            int(self.get_parameter("gripper_closed_steps").value),
            int(self.get_parameter("gripper_open_steps").value),
            float(self.get_parameter("gripper_travel_m").value),
        )
        for name, servo_id, calibration in zip(
            self.names, self.servo_ids, self.calibrations
        ):
            if name == GRIPPER_JOINT:
                if self.gripper.calibrated:
                    self.get_logger().info(
                        f"{name}/ID {servo_id}: jaw opening 0–"
                        f"{self.gripper.travel_m:.3f} m, steps "
                        f"{self.gripper.closed_steps} → {self.gripper.open_steps}"
                    )
                else:
                    self.get_logger().error(
                        f"{name}/ID {servo_id}: jaw opening is metres, but "
                        "gripper endpoints are not calibrated"
                    )
                continue
            lower_angle, upper_angle = calibration.angle_limits()
            self.get_logger().info(
                f"{name}/ID {servo_id}: raw "
                f"{calibration.lower_steps} -> {calibration.upper_steps}, "
                f"zero={calibration.zero_steps}, "
                f"direction={calibration.direction}, ROS "
                f"[{lower_angle:.4f}, {upper_angle:.4f}] rad"
            )

    @staticmethod
    def _find_port(configured_port: str) -> str:
        if configured_port:
            return configured_port
        candidates = sorted(
            glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        )
        if not candidates:
            raise RuntimeError(
                "Kein /dev/ttyACM* oder /dev/ttyUSB* gefunden"
            )
        return candidates[0]

    def read_and_publish(self):
        positions = []
        for servo_id in self.servo_ids:
            value = self.servo.ReadPosition(servo_id)
            positions.append(-1 if value is None else int(value))
        self.last_steps = positions

        raw_message = Int32MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = "servo_ids"
        dimension.size = 6
        dimension.stride = 6
        raw_message.layout.dim = [dimension]
        raw_message.data = positions
        self.steps_publisher.publish(raw_message)

        joint_message = JointState()
        joint_message.header.stamp = self.get_clock().now().to_msg()
        joint_message.name = []
        joint_message.position = []
        for name, value, calibration in zip(
            self.names, positions, self.calibrations
        ):
            if name == GRIPPER_JOINT and not self.gripper.calibrated:
                continue
            if value < 0:
                position = math.nan
            elif name == GRIPPER_JOINT:
                try:
                    position = self.gripper.raw_to_meters(value)
                except ValueError:
                    position = math.nan
            else:
                position = calibration.raw_to_angle(value)
            joint_message.name.append(name)
            joint_message.position.append(position)
        self.joint_publisher.publish(joint_message)

        self.read_count += 1
        if self.print_every and self.read_count % self.print_every == 0:
            outside = [
                servo_id
                for name, servo_id, value, calibration in zip(
                    self.names, self.servo_ids, positions, self.calibrations
                )
                if self._outside_angle_window(name, value, calibration)
            ]
            self.get_logger().info(
                " | ".join(
                    f"ID {servo_id}: {value:4d}"
                    + (
                        " AUSSERHALB"
                        if self._outside_angle_window(name, value, calibration)
                        else ""
                    )
                    if value >= 0
                    else f"ID {servo_id}: FEHLER"
                    for name, servo_id, value, calibration in zip(
                        self.names,
                        self.servo_ids,
                        positions,
                        self.calibrations,
                    )
                )
            )
            if outside and self.warn_outside:
                self.get_logger().warning(
                    "Außerhalb der kalibrierten Grenzen: IDs "
                    + ", ".join(str(value) for value in outside)
                )

    @staticmethod
    def _outside_angle_window(name: str, value: int, calibration) -> bool:
        if name == GRIPPER_JOINT or value < 0:
            return False
        return not calibration.contains(value)

    def destroy_node(self):
        try:
            self.servo.portHandler.closePort()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SixMotorPositionReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
