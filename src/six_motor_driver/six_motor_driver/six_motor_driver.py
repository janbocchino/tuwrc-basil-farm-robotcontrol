#!/usr/bin/env python3
"""FollowJointTrajectory server for six cyclically calibrated ST3215 servos."""

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
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

from .cyclic_calibration import (
    STEPS_PER_REVOLUTION,
    CyclicJointCalibration,
)


def import_st3215():
    try:
        from st3215 import ST3215
        return ST3215
    except ImportError:
        library = Path.home() / "libs" / "python-st3215"
        if library.is_dir():
            sys.path.insert(0, str(library))
            from st3215 import ST3215
            return ST3215
        raise


ST3215 = import_st3215()


class SixMotorDriver(Node):
    COUNT = 6
    WRAP_BRIDGE_MARGIN_STEPS = 8

    def __init__(self):
        super().__init__("six_motor_driver")
        self.declare_parameter("port", "")
        self.declare_parameter("servo_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter(
            "names", [f"servo_{index}" for index in range(1, 7)]
        )
        self.declare_parameter("lower_limits_steps", [0] * 6)
        self.declare_parameter("upper_limits_steps", [4095] * 6)
        self.declare_parameter("zero_positions_steps", [2048] * 6)
        self.declare_parameter("directions", [1] * 6)
        self.declare_parameter(
            "action_name",
            "/six_motor_controller/follow_joint_trajectory",
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("center_on_start", True)
        self.declare_parameter("center_speed_steps_s", 180)
        self.declare_parameter("max_speed_steps_s", 600)
        self.declare_parameter("acceleration", 20)
        self.declare_parameter("position_tolerance_steps", 25)
        self.declare_parameter("movement_timeout_s", 20.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("disable_torque_on_shutdown", True)
        self.declare_parameter("bridge_sensor_wrap", True)
        self.declare_parameter("wrap_chunk_steps", 10)
        self.declare_parameter("wrap_rotate_speed_steps_s", 120)

        self.port = self._find_port(str(self.get_parameter("port").value))
        self.servo_ids = self._six_ints("servo_ids")
        self.names = [
            str(value) for value in self.get_parameter("names").value
        ]
        lower = self._six_ints("lower_limits_steps")
        upper = self._six_ints("upper_limits_steps")
        zero = self._six_ints("zero_positions_steps")
        directions = self._six_ints("directions")
        if len(self.names) != self.COUNT:
            raise RuntimeError("names muss genau 6 Einträge enthalten")
        if len(set(self.servo_ids)) != self.COUNT:
            raise RuntimeError("Die sechs Servo-IDs müssen eindeutig sein")

        self.calibrations = [
            CyclicJointCalibration(a, b, c, d)
            for a, b, c, d in zip(lower, upper, zero, directions)
        ]
        self.angle_limits = [
            calibration.angle_limits()
            for calibration in self.calibrations
        ]
        self.action_name = str(self.get_parameter("action_name").value)
        self.center_on_start = bool(
            self.get_parameter("center_on_start").value
        )
        self.center_speed = int(
            self.get_parameter("center_speed_steps_s").value
        )
        self.max_speed = int(
            self.get_parameter("max_speed_steps_s").value
        )
        self.acceleration = int(
            self.get_parameter("acceleration").value
        )
        self.tolerance = int(
            self.get_parameter("position_tolerance_steps").value
        )
        self.timeout = float(
            self.get_parameter("movement_timeout_s").value
        )
        self.disable_on_shutdown = bool(
            self.get_parameter("disable_torque_on_shutdown").value
        )
        self.bridge_sensor_wrap = bool(
            self.get_parameter("bridge_sensor_wrap").value
        )
        self.wrap_chunk_steps = max(
            1,
            int(self.get_parameter("wrap_chunk_steps").value),
        )
        self.wrap_rotate_speed = max(
            20,
            int(self.get_parameter("wrap_rotate_speed_steps_s").value),
        )
        self.io_lock = threading.Lock()
        self.servo = ST3215(self.port)
        self.last_raw = [-1] * self.COUNT
        self.active = [True] * self.COUNT
        self.holdable = [False] * self.COUNT
        self.disabled_reasons = [""] * self.COUNT

        with self.io_lock:
            for index, servo_id in enumerate(self.servo_ids):
                if not self.servo.PingServo(servo_id):
                    self._disable_servo(index, "antwortet nicht")
                    continue
                raw = self.servo.ReadPosition(servo_id)
                if raw is None:
                    self._disable_servo(index, "Position nicht lesbar")
                    continue
                self.last_raw[index] = int(raw)
                self.holdable[index] = True
                if not self.calibrations[index].contains(int(raw)):
                    self._disable_servo(
                        index,
                        f"steht mit Rohwert {raw} außerhalb "
                        "seiner kalibrierten Grenzen",
                    )

        if self.center_on_start:
            self.stop_all()
            self.get_logger().warning(
                "Alle aktiven Motoren fahren jetzt der Reihe nach in ihre "
                "kalibrierten Mitten. Deaktivierte Motoren werden übersprungen."
            )
            self._center_sequentially()
        else:
            self._hold_active_servos_at_current_positions(self.center_speed)

        self.joint_publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            10,
        )
        self.steps_publisher = self.create_publisher(
            Int32MultiArray,
            "/six_motor/positions_steps",
            10,
        )
        self.active_publisher = self.create_publisher(
            Int32MultiArray,
            "/six_motor/active_mask",
            10,
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / publish_rate, self.publish_joint_state)
        self.create_timer(1.0, self.publish_active_mask)
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            goal_callback=self.accept_goal,
            cancel_callback=self.accept_cancel,
            execute_callback=self.execute,
        )
        self.get_logger().info(
            f"Sechs-Motor-Controller bereit: {self.action_name}"
        )
        self._log_active_summary()

    def _disable_servo(self, index: int, reason: str):
        self.active[index] = False
        self.disabled_reasons[index] = reason
        self.get_logger().error(
            f"Servo {index + 1} / ID {self.servo_ids[index]} deaktiviert: "
            f"{reason}"
        )

    def _log_active_summary(self):
        active_names = [
            f"{name}(ID {servo_id})"
            for name, servo_id, active in zip(
                self.names,
                self.servo_ids,
                self.active,
            )
            if active
        ]
        disabled = [
            f"{name}(ID {servo_id}): {reason}"
            for name, servo_id, active, reason in zip(
                self.names,
                self.servo_ids,
                self.active,
                self.disabled_reasons,
            )
            if not active
        ]
        self.get_logger().info(
            "Aktive Servos: "
            + (", ".join(active_names) if active_names else "keine")
        )
        if disabled:
            self.get_logger().warning(
                "Deaktivierte Servos: " + "; ".join(disabled)
            )

    def _six_ints(self, parameter_name: str) -> list[int]:
        values = [
            int(value)
            for value in self.get_parameter(parameter_name).value
        ]
        if len(values) != self.COUNT:
            raise RuntimeError(
                f"{parameter_name} muss genau 6 Werte enthalten"
            )
        return values

    @staticmethod
    def _find_port(configured: str) -> str:
        if configured:
            return configured
        candidates = sorted(
            glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        )
        if not candidates:
            raise RuntimeError("Kein serieller ST3215-Port gefunden")
        return candidates[0]

    def _enable_all(self, speed: int):
        self._enable_indices(
            [
                index
                for index, active in enumerate(self.active)
                if active
            ],
            speed,
        )

    def _hold_active_servos_at_current_positions(self, speed: int):
        raw_values = self._read_raw()
        indices = [
            index
            for index, holdable in enumerate(self.holdable)
            if holdable and raw_values[index] >= 0
        ]
        if not indices:
            return
        self.get_logger().info(
            "Erreichbare Servos halten aktuelle Positionen: "
            + ", ".join(
                f"ID {self.servo_ids[index]}={raw_values[index]}"
                for index in indices
            )
        )
        with self.io_lock:
            for index in indices:
                servo_id = self.servo_ids[index]
                current = raw_values[index]
                if self.servo.SetMode(servo_id, 0) is None:
                    raise RuntimeError(
                        f"Servo ID {servo_id}: Positionsmodus fehlgeschlagen"
                    )
                if self.servo.SetAcceleration(
                    servo_id, self.acceleration
                ) is None:
                    raise RuntimeError(
                        f"Servo ID {servo_id}: Beschleunigung fehlgeschlagen"
                    )
                if self.servo.SetSpeed(servo_id, speed) is None:
                    raise RuntimeError(
                        f"Servo ID {servo_id}: Geschwindigkeit fehlgeschlagen"
                    )
                if self.servo.WritePosition(servo_id, current) is None:
                    raise RuntimeError(
                        f"Servo ID {servo_id}: Halteposition nicht gesetzt"
                    )
                if self.servo.StartServo(servo_id) is None:
                    raise RuntimeError(
                        f"Servo ID {servo_id} konnte nicht aktiviert werden"
                    )

    def _enable_indices(self, indices: list[int], speed: int):
        with self.io_lock:
            for index in indices:
                servo_id = self.servo_ids[index]
                if self.servo.StartServo(servo_id) is None:
                    raise RuntimeError(
                        f"Servo ID {servo_id} konnte nicht aktiviert werden"
                    )
                if self.servo.SetMode(servo_id, 0) is None:
                    raise RuntimeError(
                        f"Servo ID {servo_id}: Positionsmodus fehlgeschlagen"
                    )
                if self.servo.SetAcceleration(
                    servo_id, self.acceleration
                ) is None:
                    raise RuntimeError(
                        f"Servo ID {servo_id}: Beschleunigung fehlgeschlagen"
                    )
                if self.servo.SetSpeed(servo_id, speed) is None:
                    raise RuntimeError(
                        f"Servo ID {servo_id}: Geschwindigkeit fehlgeschlagen"
                    )

    def _enable_one_at_current_position(self, index: int, speed: int):
        servo_id = self.servo_ids[index]
        if not self.active[index]:
            return
        current = self._read_one_raw(index)
        with self.io_lock:
            if self.servo.SetMode(servo_id, 0) is None:
                raise RuntimeError(
                    f"Servo ID {servo_id}: Positionsmodus fehlgeschlagen"
                )
            if self.servo.SetAcceleration(
                servo_id, self.acceleration
            ) is None:
                raise RuntimeError(
                    f"Servo ID {servo_id}: Beschleunigung fehlgeschlagen"
                )
            if self.servo.SetSpeed(servo_id, speed) is None:
                raise RuntimeError(
                    f"Servo ID {servo_id}: Geschwindigkeit fehlgeschlagen"
                )
            if self.servo.WritePosition(servo_id, current) is None:
                raise RuntimeError(
                    f"Servo ID {servo_id}: Halteposition nicht gesetzt"
                )
            if self.servo.StartServo(servo_id) is None:
                raise RuntimeError(
                    f"Servo ID {servo_id} konnte nicht aktiviert werden"
                )

    def _return_to_position_mode_holding_current(
        self,
        index: int,
        speed: int,
    ) -> int:
        servo_id = self.servo_ids[index]
        current = self._read_one_raw(index)
        with self.io_lock:
            self.servo.StopServo(servo_id)
            if self.servo.SetMode(servo_id, 0) is None:
                raise RuntimeError(
                    f"Servo ID {servo_id}: Positionsmodus fehlgeschlagen"
                )
            if self.servo.SetAcceleration(
                servo_id, self.acceleration
            ) is None:
                raise RuntimeError(
                    f"Servo ID {servo_id}: Beschleunigung fehlgeschlagen"
                )
            if self.servo.SetSpeed(servo_id, speed) is None:
                raise RuntimeError(
                    f"Servo ID {servo_id}: Geschwindigkeit fehlgeschlagen"
                )
            if self.servo.WritePosition(servo_id, current) is None:
                raise RuntimeError(
                    f"Servo ID {servo_id}: Halteposition nicht gesetzt"
                )
            if self.servo.StartServo(servo_id) is None:
                raise RuntimeError(
                    f"Servo ID {servo_id} konnte nicht aktiviert werden"
                )
        self.last_raw[index] = current
        return current

    def _read_raw(self) -> list[int]:
        with self.io_lock:
            for index, servo_id in enumerate(self.servo_ids):
                if not self.holdable[index]:
                    continue
                value = self.servo.ReadPosition(servo_id)
                if value is not None:
                    self.last_raw[index] = int(value)
        return list(self.last_raw)

    def _read_one_raw(self, index: int) -> int:
        servo_id = self.servo_ids[index]
        if not self.active[index]:
            raise RuntimeError(
                f"Servo ID {servo_id} ist deaktiviert: "
                f"{self.disabled_reasons[index]}"
            )
        with self.io_lock:
            value = self.servo.ReadPosition(servo_id)
        if value is None:
            raise RuntimeError(
                f"Position von Servo ID {servo_id} nicht lesbar"
            )
        self.last_raw[index] = int(value)
        return int(value)

    @staticmethod
    def _cyclic_error(actual: int, target: int) -> int:
        return min(
            (actual - target) % 4096,
            (target - actual) % 4096,
        )

    @staticmethod
    def _raw_for_progress(
        calibration: CyclicJointCalibration,
        progress: int,
    ) -> int:
        return (calibration.lower_steps + int(progress)) % STEPS_PER_REVOLUTION

    @staticmethod
    def _crosses_sensor_wrap(
        calibration: CyclicJointCalibration,
        start_progress: int,
        target_progress: int,
    ) -> bool:
        if calibration.lower_steps <= calibration.upper_steps:
            return False
        wrap_progress = STEPS_PER_REVOLUTION - calibration.lower_steps
        low = min(start_progress, target_progress)
        high = max(start_progress, target_progress)
        return low < wrap_progress < high

    def _bridge_sensor_wrap_if_needed(
        self,
        index: int,
        target_progress: int,
        speed: int,
        timeout: float,
    ) -> int:
        calibration = self.calibrations[index]
        servo_id = self.servo_ids[index]
        start_raw = self._read_one_raw(index)
        start_progress = calibration.progress_steps(start_raw)
        if not self.bridge_sensor_wrap:
            return start_raw
        if not self._crosses_sensor_wrap(
            calibration,
            start_progress,
            target_progress,
        ):
            return start_raw

        wrap_progress = STEPS_PER_REVOLUTION - calibration.lower_steps
        direction = 1 if target_progress > start_progress else -1
        margin = self.WRAP_BRIDGE_MARGIN_STEPS
        if direction > 0:
            bridge_progress = min(target_progress, wrap_progress + margin)
        else:
            bridge_progress = max(target_progress, wrap_progress - margin)
        bridge_raw = self._raw_for_progress(calibration, bridge_progress)
        rotate_speed = max(60, min(abs(speed), self.max_speed))
        rotate_command = rotate_speed if direction > 0 else -rotate_speed

        self.get_logger().warning(
            f"Servo ID {servo_id}: Sensor-Nullübergang wird per "
            f"Rotationsmodus überbrückt ({start_raw} -> {bridge_raw})"
        )
        self._rotate_across_wrap_with_retry(
            index,
            bridge_progress,
            direction,
            rotate_command,
            min(timeout, 4.0),
        )

        actual = self._return_to_position_mode_holding_current(index, speed)
        self.get_logger().info(
            f"Servo ID {servo_id}: Übergang passiert, aktuelle Position "
            f"{actual} als neues Halteziel gesetzt"
        )
        return actual

    def _rotate_to_raw_across_sensor_wrap(
        self,
        index: int,
        target_raw: int,
        speed: int,
        timeout: float,
    ):
        calibration = self.calibrations[index]
        servo_id = self.servo_ids[index]
        start_raw = self._read_one_raw(index)
        start_progress = calibration.progress_steps(start_raw)
        target_progress = calibration.progress_steps(target_raw)
        direction = 1 if target_progress > start_progress else -1
        rotate_speed = max(
            20,
            min(abs(speed), self.max_speed, self.wrap_rotate_speed),
        )
        rotate_command = rotate_speed if direction > 0 else -rotate_speed

        self.get_logger().warning(
            f"Servo ID {servo_id}: Zielweg kreuzt 4095/0; fahre "
            f"kontrolliert im Rotationsmodus von {start_raw} nach "
            f"{target_raw}"
        )
        try:
            self._rotate_to_progress_with_retry(
                index,
                target_progress,
                direction,
                rotate_command,
                timeout,
            )
        finally:
            actual = self._return_to_position_mode_holding_current(
                index,
                speed,
            )
            self.get_logger().info(
                f"Servo ID {servo_id}: Rotationsfahrt beendet, "
                f"Halteposition {actual}"
            )

    def _rotate_across_wrap_with_retry(
        self,
        index: int,
        bridge_progress: int,
        direction: int,
        rotate_command: int,
        timeout: float,
    ):
        try:
            self._rotate_until_progress(
                index,
                bridge_progress,
                direction,
                rotate_command,
                timeout,
            )
            return
        except RuntimeError as error:
            servo_id = self.servo_ids[index]
            self.get_logger().warning(
                f"Servo ID {servo_id}: erster Rotationsversuch am "
                f"Sensorübergang fehlgeschlagen ({error}); versuche "
                "Gegenrichtung"
            )
            with self.io_lock:
                self.servo.StopServo(servo_id)
            time.sleep(0.05)
            self._rotate_until_progress(
                index,
                bridge_progress,
                direction,
                -rotate_command,
                timeout,
            )

    def _rotate_to_progress_with_retry(
        self,
        index: int,
        target_progress: int,
        direction: int,
        rotate_command: int,
        timeout: float,
    ):
        try:
            self._rotate_until_target_progress(
                index,
                target_progress,
                direction,
                rotate_command,
                timeout,
            )
            return
        except RuntimeError as error:
            servo_id = self.servo_ids[index]
            self.get_logger().warning(
                f"Servo ID {servo_id}: Rotationsfahrt zum Ziel "
                f"fehlgeschlagen ({error}); versuche Gegenrichtung"
            )
            with self.io_lock:
                self.servo.StopServo(servo_id)
            time.sleep(0.05)
            self._rotate_until_target_progress(
                index,
                target_progress,
                direction,
                -rotate_command,
                timeout,
            )

    def _rotate_until_progress(
        self,
        index: int,
        bridge_progress: int,
        direction: int,
        rotate_command: int,
        timeout: float,
    ):
        calibration = self.calibrations[index]
        servo_id = self.servo_ids[index]
        with self.io_lock:
            if self.servo.SetMode(servo_id, 1) is None:
                raise RuntimeError("Rotationsmodus fehlgeschlagen")
            if self.servo.Rotate(servo_id, rotate_command) is None:
                raise RuntimeError("Rotationsbefehl fehlgeschlagen")

        deadline = time.monotonic() + timeout
        last_progress = calibration.progress_steps(self.last_raw[index])
        wrong_direction_count = 0
        while time.monotonic() < deadline:
            actual = self._read_one_raw(index)
            progress = calibration.progress_steps(actual)
            if direction > 0 and progress >= bridge_progress:
                return
            if direction < 0 and progress <= bridge_progress:
                return
            if direction > 0 and progress + 2 < last_progress:
                wrong_direction_count += 1
            elif direction < 0 and progress - 2 > last_progress:
                wrong_direction_count += 1
            else:
                wrong_direction_count = 0
            if wrong_direction_count >= 5:
                raise RuntimeError("Rotationsrichtung wirkt falsch")
            last_progress = progress
            time.sleep(0.02)
        raise RuntimeError("Sensor-Nullübergang nicht erreicht")

    def _rotate_until_target_progress(
        self,
        index: int,
        target_progress: int,
        direction: int,
        rotate_command: int,
        timeout: float,
    ):
        calibration = self.calibrations[index]
        servo_id = self.servo_ids[index]
        with self.io_lock:
            if self.servo.SetAcceleration(
                servo_id, self.acceleration
            ) is None:
                raise RuntimeError("Beschleunigung fehlgeschlagen")
            if self.servo.SetMode(servo_id, 1) is None:
                raise RuntimeError("Rotationsmodus fehlgeschlagen")
            if self.servo.Rotate(servo_id, rotate_command) is None:
                raise RuntimeError("Rotationsbefehl fehlgeschlagen")

        deadline = time.monotonic() + timeout
        last_progress = calibration.progress_steps(self.last_raw[index])
        wrong_direction_count = 0
        last_display = 0.0
        while time.monotonic() < deadline:
            actual = self._read_one_raw(index)
            progress = calibration.progress_steps(actual)
            now = time.monotonic()
            if now - last_display >= 0.20:
                self.get_logger().info(
                    f"Servo ID {servo_id}: Rotation Position {actual}, "
                    f"Fortschritt {progress}/{target_progress}"
                )
                last_display = now

            if abs(progress - target_progress) <= self.tolerance:
                return
            if direction > 0 and progress >= target_progress:
                return
            if direction < 0 and progress <= target_progress:
                return

            if direction > 0 and progress + 2 < last_progress:
                wrong_direction_count += 1
            elif direction < 0 and progress - 2 > last_progress:
                wrong_direction_count += 1
            else:
                wrong_direction_count = 0
            if wrong_direction_count >= 5:
                raise RuntimeError("Rotationsrichtung wirkt falsch")
            last_progress = progress
            time.sleep(0.02)
        raise RuntimeError("Rotationsziel nicht erreicht")

    def _center_sequentially(self):
        for index in range(self.COUNT):
            servo_id = self.servo_ids[index]
            if not self.active[index]:
                self.get_logger().warning(
                    f"[{index + 1}/{self.COUNT}] Servo ID {servo_id} "
                    f"wird übersprungen: {self.disabled_reasons[index]}"
                )
                continue
            target = self.calibrations[index].zero_steps
            self.get_logger().warning(
                f"[{index + 1}/{self.COUNT}] Servo ID {servo_id} "
                f"fährt zur Mitte {target}"
            )
            self._enable_one_at_current_position(index, self.center_speed)
            self._move_one_to_raw(
                index,
                target,
                self.center_speed,
                self.timeout,
            )
            self.get_logger().info(
                f"[{index + 1}/{self.COUNT}] Servo ID {servo_id} "
                f"hat die Mitte erreicht: {self.last_raw[index]}"
            )

    def _write_one_position(self, index: int, raw: int, speed: int):
        servo_id = self.servo_ids[index]
        with self.io_lock:
            if self.servo.SetSpeed(servo_id, speed) is None:
                raise RuntimeError(
                    f"Servo ID {servo_id}: Geschwindigkeit fehlgeschlagen"
                )
            if self.servo.WritePosition(servo_id, raw) is None:
                raise RuntimeError(
                    f"Ziel für Servo ID {servo_id} nicht gesendet"
                )

    def _wait_one_position(
        self,
        index: int,
        target_raw: int,
        timeout: float,
        label: str,
    ):
        servo_id = self.servo_ids[index]
        deadline = time.monotonic() + timeout
        last_display = 0.0
        while time.monotonic() < deadline:
            actual = self._read_one_raw(index)
            now = time.monotonic()
            if now - last_display >= 0.20:
                self.get_logger().info(
                    f"Servo ID {servo_id}: Position {actual} / "
                    f"{label} {target_raw}"
                )
                last_display = now
            if self._cyclic_error(actual, target_raw) <= self.tolerance:
                return
            time.sleep(0.03)
        raise RuntimeError(
            f"Servo ID {servo_id}: {label} {target_raw} nicht erreicht; "
            f"letzte Position {self.last_raw[index]}"
        )

    def _progress_targets(
        self,
        start_progress: int,
        target_progress: int,
        step: int,
    ) -> list[int]:
        distance = target_progress - start_progress
        if distance == 0:
            return [target_progress]
        direction = 1 if distance > 0 else -1
        count = max(1, math.ceil(abs(distance) / max(1, step)))
        return [
            start_progress
            + direction * min(abs(distance), chunk * step)
            for chunk in range(1, count + 1)
        ]

    def _move_one_to_raw(
        self,
        index: int,
        target_raw: int,
        speed: int,
        timeout: float,
    ):
        calibration = self.calibrations[index]
        servo_id = self.servo_ids[index]
        target_progress = calibration.progress_steps(target_raw)
        start_raw = self._read_one_raw(index)
        start_progress = calibration.progress_steps(start_raw)
        distance = abs(target_progress - start_progress)
        crosses_wrap = self._crosses_sensor_wrap(
            calibration,
            start_progress,
            target_progress,
        )
        if crosses_wrap and self.bridge_sensor_wrap:
            self._rotate_to_raw_across_sensor_wrap(
                index,
                target_raw,
                speed,
                timeout,
            )
            return
        if not crosses_wrap:
            self.get_logger().info(
                f"Servo ID {servo_id}: direktes Ziel {target_raw} "
                f"({distance} Schritte)"
            )
            self._write_one_position(index, target_raw, speed)
            self._wait_one_position(
                index,
                target_raw,
                timeout,
                "Ziel",
            )
            return

        progress_targets = self._progress_targets(
            start_progress,
            target_progress,
            self.wrap_chunk_steps,
        )
        chunk_timeout = max(0.4, timeout / len(progress_targets))
        self.get_logger().warning(
            f"Servo ID {servo_id}: Sensor-Nullübergang erkannt; "
            f"{len(progress_targets)} Zwischenziele à ca. "
            f"{self.wrap_chunk_steps} Schritte"
        )
        for progress in progress_targets:
            raw = self._raw_for_progress(calibration, progress)
            self._write_one_position(index, raw, speed)
            self._wait_one_position(
                index,
                raw,
                chunk_timeout,
                "Zwischenziel",
            )

    def _move_to_raw(
        self,
        indices: list[int],
        targets: list[int],
        speed: int,
        timeout: float,
    ):
        self._read_raw()
        starts = [self.last_raw[index] for index in indices]
        calibrations = [self.calibrations[index] for index in indices]
        servo_ids = [self.servo_ids[index] for index in indices]
        start_progress = [
            calibration.progress_steps(raw)
            for calibration, raw in zip(calibrations, starts)
        ]
        target_progress = [
            calibration.progress_steps(raw)
            for calibration, raw in zip(calibrations, targets)
        ]
        crosses_wrap = [
            self._crosses_sensor_wrap(calibration, start, target)
            for calibration, start, target in zip(
                calibrations,
                start_progress,
                target_progress,
            )
        ]
        largest_distance = max(
            abs(target - start)
            for start, target in zip(start_progress, target_progress)
        )
        if not any(crosses_wrap):
            self.get_logger().info(
                f"Direkte Mehrgelenk-Ziele: {targets}"
            )
            with self.io_lock:
                for servo_id in servo_ids:
                    self.servo.SetSpeed(servo_id, speed)
                for servo_id, raw in zip(servo_ids, targets):
                    if self.servo.WritePosition(servo_id, raw) is None:
                        raise RuntimeError(
                            f"Ziel für Servo ID {servo_id} nicht gesendet"
                        )
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self._read_raw()
                measured = [self.last_raw[index] for index in indices]
                if all(
                    self._cyclic_error(actual, target) <= self.tolerance
                    for actual, target in zip(measured, targets)
                ):
                    return
                time.sleep(0.03)
            raise RuntimeError(
                f"Ziel nicht erreicht: {targets}, Ist={self.last_raw}"
            )

        if self.bridge_sensor_wrap:
            self.get_logger().warning(
                "Mindestens ein Ziel kreuzt 4095/0; fahre angeforderte "
                "Gelenke nacheinander im sicheren Einzelmodus"
            )
            for index, target in zip(indices, targets):
                self._move_one_to_raw(index, target, speed, timeout)
            return

        chunks = max(1, math.ceil(largest_distance / self.wrap_chunk_steps))
        chunk_timeout = max(1.0, timeout / chunks)

        for chunk in range(1, chunks + 1):
            fraction = chunk / chunks
            chunk_targets = [
                (
                    calibration.lower_steps
                    + int(round(start + (target - start) * fraction))
                )
                % 4096
                for calibration, start, target in zip(
                    calibrations,
                    start_progress,
                    target_progress,
                )
            ]
            with self.io_lock:
                for servo_id in servo_ids:
                    self.servo.SetSpeed(servo_id, speed)
                for servo_id, raw in zip(servo_ids, chunk_targets):
                    if self.servo.WritePosition(servo_id, raw) is None:
                        raise RuntimeError(
                            f"Ziel für Servo ID {servo_id} nicht gesendet"
                        )

            deadline = time.monotonic() + chunk_timeout
            while time.monotonic() < deadline:
                self._read_raw()
                measured = [self.last_raw[index] for index in indices]
                if all(
                    self._cyclic_error(actual, target) <= self.tolerance
                    for actual, target in zip(measured, chunk_targets)
                ):
                    break
                time.sleep(0.03)
            else:
                raise RuntimeError(
                    f"Zwischenziel nicht erreicht: {chunk_targets}, "
                    f"Ist={self.last_raw}"
                )

    def publish_joint_state(self):
        raw_values = self._read_raw()
        raw_message = Int32MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = "servo_ids"
        dimension.size = self.COUNT
        dimension.stride = self.COUNT
        raw_message.layout.dim = [dimension]
        raw_message.data = list(raw_values)
        self.steps_publisher.publish(raw_message)

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = []
        message.position = []
        for name, calibration, raw in zip(
            self.names,
            self.calibrations,
            raw_values,
        ):
            if raw < 0:
                continue
            message.name.append(name)
            message.position.append(calibration.raw_to_angle(raw))
        self.joint_publisher.publish(message)

    def publish_active_mask(self):
        message = Int32MultiArray()
        message.data = [1 if active else 0 for active in self.active]
        self.active_publisher.publish(message)

    def accept_goal(self, request):
        requested_names = list(request.trajectory.joint_names)
        requested_indices: list[int] = []
        for name in requested_names:
            if name not in self.names:
                self.get_logger().error(f"Unbekanntes Gelenk: {name}")
                return GoalResponse.REJECT
            index = self.names.index(name)
            if not self.active[index]:
                self.get_logger().error(
                    f"Gelenk {name} ist deaktiviert: "
                    f"{self.disabled_reasons[index]}"
                )
                return GoalResponse.REJECT
            requested_indices.append(index)

        if len(set(requested_indices)) != len(requested_indices):
            self.get_logger().error(
                f"Doppelte Gelenknamen erhalten: {requested_names}"
            )
            return GoalResponse.REJECT
        if not requested_indices:
            self.get_logger().error("Keine Gelenke im Ziel enthalten")
            return GoalResponse.REJECT

        active_names = [
            name
            for name, active in zip(self.names, self.active)
            if active
        ]
        self.get_logger().info(
            f"Ziel für Gelenke {requested_names}; aktive Gelenke sind "
            f"{active_names}"
        )
        if any(index not in range(self.COUNT) for index in requested_indices):
            self.get_logger().error(
                f"Ungültige Gelenk-Indizes aus {requested_names}"
            )
            return GoalResponse.REJECT
        if not request.trajectory.points:
            return GoalResponse.REJECT
        requested_limits = [
            self.angle_limits[index] for index in requested_indices
        ]
        for point in request.trajectory.points:
            if len(point.positions) != len(requested_indices):
                return GoalResponse.REJECT
            for angle, limits in zip(point.positions, requested_limits):
                if not limits[0] <= float(angle) <= limits[1]:
                    return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        try:
            requested_indices = [
                self.names.index(name)
                for name in goal_handle.request.trajectory.joint_names
            ]
            self._hold_active_servos_at_current_positions(self.center_speed)
            self._enable_indices(requested_indices, self.max_speed)
            requested_calibrations = [
                self.calibrations[index] for index in requested_indices
            ]
            for point in goal_handle.request.trajectory.points:
                if goal_handle.is_cancel_requested:
                    self._hold_active_servos_at_current_positions(
                        self.center_speed
                    )
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
                targets = [
                    calibration.angle_to_raw(float(angle))
                    for calibration, angle in zip(
                        requested_calibrations,
                        point.positions,
                    )
                ]
                duration = (
                    point.time_from_start.sec
                    + point.time_from_start.nanosec / 1e9
                )
                movement_timeout = max(self.timeout, duration + 2.0)
                if len(requested_indices) == 1:
                    self._move_one_to_raw(
                        requested_indices[0],
                        targets[0],
                        self.max_speed,
                        movement_timeout,
                    )
                else:
                    self._move_to_raw(
                        requested_indices,
                        targets,
                        self.max_speed,
                        movement_timeout,
                    )
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            return result
        except Exception as error:
            try:
                self._hold_active_servos_at_current_positions(
                    self.center_speed
                )
            except Exception as hold_error:
                self.get_logger().error(
                    f"Halteposition nach Fehler fehlgeschlagen: {hold_error}"
                )
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            goal_handle.abort()
            return result

    def stop_all(self):
        with self.io_lock:
            for servo_id in self.servo_ids:
                self.servo.StopServo(servo_id)

    def destroy_node(self):
        try:
            if self.disable_on_shutdown:
                self.stop_all()
            self.servo.portHandler.closePort()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SixMotorDriver()
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
