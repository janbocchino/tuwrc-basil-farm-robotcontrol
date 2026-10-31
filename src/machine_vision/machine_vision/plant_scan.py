#!/usr/bin/env python3
"""Visit 6 camera views around the basil plant (standalone, no local imports).

Views (rail positions are measured from the left end, 0 = lm_left):
  1.1 left of start, joint 1 panned toward the plant
  1.2 left, camera raised a few cm, joint 4 tilted so the plant stays in view
  2.1 start rail position, base arm pose
  2.2 start rail position, raised + tilted (looking from above)
  3.1 / 3.2 same as 1.x but to the right of the start position

The arm pose at launch (camera looking at the plant at --start-rail-m) is the
base pose. Every view = base pose + joint deltas, so put the arm in your good
pose before running. Optional TF guard aborts if the EEF y goes below -0.1859 m.

Hardware: the rail must be calibrated first (/tuwrc_rail_driver/calibrate).
"""

from __future__ import annotations

import argparse
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
RAIL_ACTION = "/rail_controller/follow_joint_trajectory"
ARM_JOINTS = ["1", "2", "3", "4", "5", "6"]
RAIL_JOINT = "rail_joint"

JOINT_LIMITS = {
    "rail_joint": (0.0, 0.715),  # 0 = left end (lm_left)
    "1": (-2.06780610, 2.06780610),
    "2": (-1.85304879, 1.85304879),
    "3": (-1.68737887, 1.68891285),
    "4": (-1.80702937, 1.80702937),
    "5": (-2.89308777, 2.89308777),
    "6": (-0.18561168, 2.03252454),
}

Y_MIN = -0.1859  # m, world frame: EEF must stay above this (pipe)

# --------------------------------------------------------------------------
# Tunables. Check the signs in view mode first!
# --------------------------------------------------------------------------
# Joint "1" = base pan. Sign that turns the camera toward the plant when the
# arm is on the LEFT of the plant (rail < start). Flip to +1.0 if it turns away.
PAN_SIGN_FROM_LEFT = 1.0
PAN_DELTA = 0.35  # rad (~20 deg), roughly atan(side_offset / camera-plant distance)

# "Camera up a couple of cm" via joints 2/3, joint 4 re-tilts to keep the plant
# in view. Signs/magnitudes depend on your kinematics: tune in view mode.
UP_SMALL: dict[str, float] = {"2": -0.10, "3": 0.10, "4": 0.20}
# Looking down from higher up: more lift and more tilt.
UP_LARGE: dict[str, float] = {"2": -0.20, "3": 0.20, "4": 0.50}

_LEFT_PAN = {"1": PAN_SIGN_FROM_LEFT * PAN_DELTA}
_RIGHT_PAN = {"1": -PAN_SIGN_FROM_LEFT * PAN_DELTA}

# (label, rail offset in units of --side-offset, joint deltas, arm duration s)
VIEWS: list[tuple[str, float, dict[str, float], float]] = [
    ("1.1 left", -1.0, _LEFT_PAN, 2.5),
    ("1.2 left, up+tilt", -1.0, {**_LEFT_PAN, **UP_SMALL}, 5.5),
    ("2.1 centre", 0.0, {}, 2.5),
    ("2.2 centre, up+tilt", 0.0, UP_LARGE, 5.5),
    ("3.1 right", +1.0, _RIGHT_PAN, 5.5),
    ("3.2 right, up+tilt", +1.0, {**_RIGHT_PAN, **UP_SMALL}, 5.5),
]


class ViewScan(Node):
    def __init__(self):
        super().__init__("rail_view_scan")
        self._positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._arm_client = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
        self._rail_client = ActionClient(self, FollowJointTrajectory, RAIL_ACTION)

    def _on_js(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in ARM_JOINTS or name == RAIL_JOINT:
                self._positions[name] = float(position)

    def wait_for_state(self, timeout_sec: float = 20.0):
        needed = list(ARM_JOINTS) + [RAIL_JOINT]
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(name in self._positions for name in needed):
                return
        missing = [name for name in needed if name not in self._positions]
        raise RuntimeError(f"Missing joint states for: {missing}")

    def send_arm(self, targets: dict[str, float], duration_sec: float):
        self._send(self._arm_client, ARM_ACTION, targets, duration_sec)

    def send_rail(self, position_m: float, duration_sec: float):
        self._send(
            self._rail_client, RAIL_ACTION, {RAIL_JOINT: position_m}, duration_sec
        )

    arm_tol = 0.08   # rad (~4.5 deg)
    rail_tol = 0.005  # m

    def _errors(self, targets):
        return {n: self._positions[n] - v for n, v in targets.items() if n in self._positions}

    def _within(self, targets, tol):
        errs = self._errors(targets)
        return bool(errs) and max(abs(e) for e in errs.values()) <= tol

    def wait_settled(self, targets, tol, stable_sec=0.5, timeout=10.0):
        """Block until joints are within tol of targets AND have stopped moving."""
        t0 = time.time()
        last = None
        stable_since = None
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            cur = {n: self._positions[n] for n in targets if n in self._positions}
            moving = last is not None and any(
                abs(cur[n] - last[n]) > 1e-3 for n in cur
            )
            last = cur
            if self._within(targets, tol) and not moving:
                stable_since = stable_since or time.time()
                if time.time() - stable_since >= stable_sec:
                    return True
            else:
                stable_since = None
        self.get_logger().warn(f"Not settled after {timeout:.0f}s, errors: {self._errors(targets)}")
        return False

    def _send(self, client, action_name, targets, duration_sec):
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Action server {action_name} is not available")

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(targets.keys())
        point = JointTrajectoryPoint()
        point.positions = [float(targets[name]) for name in traj.joint_names]
        point.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec % 1.0) * 1e9)
        )
        traj.points = [point]
        goal.trajectory = traj

        # The six_motor_driver ignores new goals for a few seconds after a move: retry.
        handle = None
        retry_deadline = time.time() + 50.0
        while True:
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            handle = future.result()
            if handle is not None and handle.accepted:
                break
            if time.time() > retry_deadline:
                raise RuntimeError(f"Trajectory goal rejected on {action_name}")
            self.get_logger().warn(f"{action_name}: goal rejected, retrying in 0.5 s")
            end = time.time() + 5.0
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.05)


        result_future = handle.get_result_async()
        tol = self.rail_tol if RAIL_JOINT in targets else self.arm_tol
        t0 = time.time()
        deadline = t0 + duration_sec + 25.0  # driver gives up after ~20 s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if result_future.done():
                res = result_future.result().result
                if res.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                    return
                break  # driver reported failure: check the pose below
            if time.time() - t0 > duration_sec + 1.0 and self._within(targets, tol):
                handle.cancel_goal_async()  # close enough, stop waiting
                return

        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        errs = self._errors(targets)
        if self._within(targets, tol):
            self.get_logger().warn(f"{action_name}: driver reported no success, but pose is within tolerance.")
            return
        raise RuntimeError(f"{action_name}: target not reached, joint errors: {errs}")

class EefGuard:
    """Optional TF check that the EEF stays at y > Y_MIN (+ margin)."""

    def __init__(self, node: Node, world: str, eef: str, margin: float):
        from tf2_ros import Buffer, TransformListener

        self.node, self.world, self.eef, self.margin = node, world, eef, margin
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, node)

    def check(self, label: str) -> bool:
        from rclpy.duration import Duration as RclDuration
        from rclpy.time import Time

        try:
            tf = self.buf.lookup_transform(
                self.world, self.eef, Time(), timeout=RclDuration(seconds=1.0)
            )
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"TF lookup failed at '{label}': {exc}")
            return True  # warn only; don't block on a missing TF
        t = tf.transform.translation
        self.node.get_logger().info(
            f"[{label}] EEF xyz = ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})"
        )
        if t.y < Y_MIN + self.margin:
            self.node.get_logger().error(
                f"EEF y={t.y:.3f} < {Y_MIN + self.margin:.3f} at '{label}', aborting."
            )
            return False
        return True


def clamp(name: str, value: float) -> float:
    lower, upper = JOINT_LIMITS[name]
    return max(lower, min(upper, value))


def main(argv=None):
    p = argparse.ArgumentParser(description="Scan the basil plant from 6 views.")
    p.add_argument("--start-rail-m", type=float, default=0.21,
                   help="Rail position of the centre view (default 0.36).")
    p.add_argument("--side-offset", type=float, default=0.12,
                   help="Rail distance left/right of the start position in m (default 0.12).")
    p.add_argument("--allow-hardware", action="store_true",
                   help="Required confirmation when commanding the real arm and rail.")
    p.add_argument("--hardware", action="store_true",
                   help="Declare that this run targets real hardware (requires --allow-hardware).")
    p.add_argument("--duration-scale", type=float, default=1.0,
                   help="Multiply each arm waypoint duration (default 1.0).")
    p.add_argument("--dwell", type=float, default=1.5,
                   help="Seconds to wait at each view (e.g. for the image / cloud capture).")
    p.add_argument("--capture-srv", default="/capture",
                   help="Optional std_srvs/Trigger called at each view, "
                        "e.g. /pointcloud_fuser/capture.")
    p.add_argument("--eef-frame", default="",
                   help="If set, check via TF after each move that EEF y > -0.1859 m.")
    p.add_argument("--world-frame", default="world")
    p.add_argument("--y-margin", type=float, default=0.01,
                   help="Extra safety margin on the y limit in m (default 0.01).")
    p.add_argument("--return-home", action="store_true",
                   help="Return arm and rail to their launch positions at the end.")
    args = p.parse_args(argv)

    if args.hardware and not args.allow_hardware:
        print("Refusing to command hardware without --allow-hardware.", file=sys.stderr)
        return 2
    if args.duration_scale <= 0.0:
        print("--duration-scale must be > 0.", file=sys.stderr)
        return 2
    lo, hi = JOINT_LIMITS[RAIL_JOINT]
    rails = [args.start_rail_m + off * args.side_offset for _, off, _, _ in VIEWS]
    if not all(lo <= r <= hi for r in rails):
        print(f"All rail targets must be within {lo}-{hi} m, got {rails}", file=sys.stderr)
        return 2

    rclpy.init()
    node = ViewScan()
    try:
        node.get_logger().info("Waiting for /joint_states …")
        node.wait_for_state()
        base_arm = {name: node._positions[name] for name in ARM_JOINTS}
        start_rail = node._positions[RAIL_JOINT]
        node.get_logger().info(f"Base arm pose (current): {base_arm}")

        guard = (
            EefGuard(node, args.world_frame, args.eef_frame, args.y_margin)
            if args.eef_frame
            else None
        )

        capture = None
        Trigger = None
        if args.capture_srv:
            from std_srvs.srv import Trigger

            capture = node.create_client(Trigger, args.capture_srv)
            if not capture.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"Service {args.capture_srv} is not available")

        node.get_logger().info(f"Moving rail to start {args.start_rail_m:.3f} m …")
        node.send_rail(clamp(RAIL_JOINT, args.start_rail_m), 3.0)

        for (label, _off, deltas, dur), rail_m in zip(VIEWS, rails):
            rail_target = clamp(RAIL_JOINT, rail_m)
            node.get_logger().info(f"== View {label}: rail {rail_target:.3f} m")
            # The real rail runs at Arduino speed; time_from_start only paces the mock.
            node.send_rail(rail_target, 3.0)

            targets = {
                name: clamp(name, base_arm[name] + deltas.get(name, 0.0))
                for name in ARM_JOINTS
            }
            d = dur * args.duration_scale
            node.get_logger().info(f"Arm targets: {targets} ({d:.1f}s)")
            node.send_arm(targets, d)

            if guard is not None and not guard.check(label):
                return 1

            time.sleep(args.dwell)
            if capture is not None:
                fut = capture.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
                res = fut.result()
                node.get_logger().info(
                    f"capture '{label}': {res.message if res else 'no response'}"
                )

        if args.return_home:
            node.get_logger().info("Returning to start positions …")
            node.send_arm(base_arm, 2.5 * args.duration_scale)
            node.send_rail(start_rail, 3.0)

        node.get_logger().info("6-view scan finished.")
        return 0
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())