#!/usr/bin/env python3
"""Read six ST3215 positions continuously without commanding any motion."""

from __future__ import annotations

import glob
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

from .cyclic_calibration import CyclicJointCalibration


DEFAULT_SERVO_IDS = [1, 2, 3, 4, 5, 6]
DEFAULT_NAMES = [f"servo_{servo_id}" for servo_id in DEFAULT_SERVO_IDS]


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
        for name, servo_id, calibration in zip(
            self.names, self.servo_ids, self.calibrations
        ):
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
        joint_message.name = list(self.names)
        joint_message.position = [
            (
                calibration.raw_to_angle(value)
                if value >= 0
                else math.nan
            )
            for value, calibration in zip(
                positions, self.calibrations
            )
        ]
        self.joint_publisher.publish(joint_message)

        self.read_count += 1
        if self.print_every and self.read_count % self.print_every == 0:
            outside = [
                servo_id
                for servo_id, value, calibration in zip(
                    self.servo_ids, positions, self.calibrations
                )
                if value >= 0 and not calibration.contains(value)
            ]
            self.get_logger().info(
                " | ".join(
                    f"ID {servo_id}: {value:4d}"
                    + (
                        " AUSSERHALB"
                        if value >= 0
                        and not calibration.contains(value)
                        else ""
                    )
                    if value >= 0
                    else f"ID {servo_id}: FEHLER"
                    for servo_id, value, calibration in zip(
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
