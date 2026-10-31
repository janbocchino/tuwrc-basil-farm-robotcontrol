#!/usr/bin/env python3
"""Step the rail from 0 to the far end in 10 cm steps, call the combined point
cloud services at every stop and check the end switches to estimate the
position error.

Sequence: clear -> (move, settle, capture) x N -> save -> end switch checks.

Prerequisites: rail driver running and calibrated (carriage at 0, lm_left
pressed), camera + robot_state_publisher running, and the point cloud node
offering the Trigger services capture / save / clear.

    python3 rail_scan.py --ros-args -p camera_frame:=camera_depth_optical_frame

Optional: -p ask_ruler:=true asks for a ruler measurement at every stop, which
is the only way to see the REAL error (there is no encoder).
"""

import csv
import os
import time

import rclpy
import tf2_ros
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


class RailScan(Node):
    def __init__(self):
        super().__init__("rail_scan")
        declare = self.declare_parameter
        declare("step_m", 0.10)
        declare("length_m", 0.715)  # expected travel between the switches
        declare("capture_service", "/capture")
        declare("save_service", "/save")
        declare("clear_service", "/clear")
        declare("service_timeout_sec", 100.0)
        declare("world_frame", "world")
        declare("camera_frame", "camera_depth_optical_frame")
        declare("output_dir", "rail_scan")
        declare("settle_sec", 4.5)
        declare("return_home", True)
        declare("ask_ruler", False)
        declare("action_name", "/rail_controller/follow_joint_trajectory")

        def get(name):
            return self.get_parameter(name).value

        self.step = float(get("step_m"))
        self.length = float(get("length_m"))
        self.world = str(get("world_frame"))
        self.camera_frame = str(get("camera_frame"))
        self.out_dir = str(get("output_dir"))
        self.settle = float(get("settle_sec"))
        self.service_timeout = float(get("service_timeout_sec"))
        self.return_home = bool(get("return_home"))
        self.ask_ruler = bool(get("ask_ruler"))

        self.rail_pos = None
        self.lm = {"left": None, "right": None}
        self.pending = []  # switches that just closed -> position taken from next joint state
        self.edges = []  # (switch, counter position in m when it closed)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=True
        )
        self.client = ActionClient(
            self, FollowJointTrajectory, str(get("action_name"))
        )
        self.capture_cli = self.create_client(Trigger, str(get("capture_service")))
        self.save_cli = self.create_client(Trigger, str(get("save_service")))
        self.clear_cli = self.create_client(Trigger, str(get("clear_service")))

        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.create_subscription(
            Bool, "/rail/lm_left", lambda m: self.on_lm("left", m.data), 10
        )
        self.create_subscription(
            Bool, "/rail/lm_right", lambda m: self.on_lm("right", m.data), 10
        )

    # ---------------------------------------------------------------- callbacks

    def on_js(self, msg):
        if "rail_joint" not in msg.name:
            return
        self.rail_pos = msg.position[msg.name.index("rail_joint")]
        for name in self.pending:
            self.edges.append((name, self.rail_pos))
            self.get_logger().info(
                f"lm_{name} closed at counter position {self.rail_pos * 100:.2f} cm"
            )
        self.pending.clear()

    def on_lm(self, name, closed):
        if closed and self.lm[name] is False:
            self.pending.append(name)
        self.lm[name] = closed

    # ------------------------------------------------------------------ helpers

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, cond, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and not cond():
            if time.monotonic() > end:
                return False
            rclpy.spin_once(self, timeout_sec=0.02)
        return True

    def call(self, client, label):
        """Call a Trigger service, return (success, message)."""
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.service_timeout
        )
        if not future.done():
            self.get_logger().error(
                f"{label}: no response within {self.service_timeout:.0f} s"
            )
            return False, "timeout"
        res = future.result()
        if res.success:
            self.get_logger().info(f"{label}: {res.message}")
        else:
            self.get_logger().error(f"{label}: {res.message}")
        return res.success, res.message

    def move_to(self, target):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["rail_joint"]
        point = JointTrajectoryPoint()
        point.positions = [float(target)]
        point.time_from_start = DurationMsg(sec=1)
        goal.trajectory.points = [point]
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Rail goal rejected (not calibrated?)")
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        code = result.result().result.error_code
        return code == FollowJointTrajectory.Result.SUCCESSFUL

    def lookup_x(self):
        """Camera X in the world frame (latest TF; the carriage is stationary)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world, self.camera_frame, Time(), timeout=Duration(seconds=1.0)
            )
            return tf.transform.translation.x
        except Exception as err:
            self.get_logger().warn(f"TF lookup failed: {err}")
            return float("nan")

    def stop(self, index, cmd, prev_reported):
        self.spin_for(self.settle)
        reported = self.rail_pos
        tf_x = self.lookup_x()
        ok, msg = self.call(self.capture_cli, f"capture {index}")
        row = {
            "index": index,
            "commanded_m": cmd,
            "reported_m": reported,
            "moved_m": reported - prev_reported,
            "tf_x_m": tf_x,
            "capture_ok": ok,
            "capture_msg": msg,
            "ruler_cm": "",
        }
        if self.ask_ruler:
            row["ruler_cm"] = input(
                f"[{cmd * 100:.1f} cm] measured distance from the left end in cm "
                "(Enter to skip): "
            ).strip()
        self.get_logger().info(
            f"stop {index}: commanded {cmd * 100:.2f} cm | counter {reported * 100:.2f} cm "
            f"| moved {row['moved_m'] * 100:.2f} cm | camera TF x {tf_x:.4f} m"
        )
        return row

    # --------------------------------------------------------------------- main

    def run(self):
        log = self.get_logger()
        os.makedirs(self.out_dir, exist_ok=True)
        if not self.wait_for(
            lambda: self.rail_pos is not None and None not in self.lm.values(), 10.0
        ):
            log.error("No rail status (/joint_states, /rail/lm_left, /rail/lm_right)")
            return
        if not self.client.wait_for_server(timeout_sec=5.0):
            log.error("Rail action server not available")
            return
        for cli, label in (
            (self.capture_cli, "capture"),
            (self.save_cli, "save"),
            (self.clear_cli, "clear"),
        ):
            if not cli.wait_for_service(timeout_sec=5.0):
                log.error(
                    f"Service '{label}' not available "
                    f"({cli.srv_name}); set the *_service parameters"
                )
                return
        if not self.lm["left"] or abs(self.rail_pos) > 0.005:
            log.error(
                "Rail must start at 0 with lm_left pressed. Call "
                "/tuwrc_rail_driver/calibrate first."
            )
            return

        n = int(self.length / self.step + 1e-9)
        targets = [round(i * self.step, 4) for i in range(n + 1)]
        if self.length - targets[-1] > 1e-6:
            targets.append(self.length)

        # ok, _ = self.call(self.clear_cli, "clear")
        # if not ok:
        #     log.error("Could not clear the point cloud buffer, aborting")
        #     return

        rows = []
        prev = self.rail_pos
        for i, target in enumerate(targets):
            if i > 0 and not self.move_to(target):
                log.error(f"Move to {target * 100:.1f} cm failed, stopping")
                break
            row = self.stop(i, target, prev)
            prev = row["reported_m"]
            rows.append(row)

        self.call(self.save_cli, "save")

        with open(os.path.join(self.out_dir, "summary.csv"), "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

        # ---- end of travel: did lm_right close where it should have?
        self.spin_for(0.5)
        log.info("=============== END OF TRAVEL ===============")
        log.info(
            f"commanded {targets[-1] * 100:.2f} cm | counter {self.rail_pos * 100:.2f} cm "
            f"| lm_right pressed: {self.lm['right']}"
        )
        right = [p for name, p in self.edges if name == "right"]
        if right:
            err = self.length - right[0]
            log.info(
                f"lm_right closed at counter {right[0] * 100:.2f} cm, expected "
                f"{self.length * 100:.2f} cm -> counter error {err * 1000:+.1f} mm "
                "(positive: carriage is physically further than the counter says)"
            )
        if not self.lm["right"]:
            log.warn(
                "lm_right is NOT pressed at the end: the carriage is short of the "
                "switch (lost steps or too-short calibrated length). Measure the "
                "gap with a ruler - the size is unknown without an encoder."
            )

        # ---- return to 0: did lm_left close at counter 0?
        if self.return_home:
            self.move_to(0.0)
            self.spin_for(self.settle)
            left = [p for name, p in self.edges if name == "left"]
            log.info("=============== BACK AT 0 ===============")
            log.info(
                f"counter {self.rail_pos * 100:.2f} cm | lm_left pressed: {self.lm['left']}"
            )
            if left:
                log.info(
                    f"lm_left closed at counter {left[0] * 100:.2f} cm, expected 0.00 cm "
                    f"-> round-trip error {left[0] * 1000:+.1f} mm"
                )
            if not self.lm["left"]:
                log.warn(
                    "lm_left is NOT pressed back at 0: steps were lost on the "
                    "round trip."
                )
        log.info(f"Summary in {os.path.abspath(self.out_dir)}/summary.csv")


def main():
    rclpy.init()
    node = RailScan()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()