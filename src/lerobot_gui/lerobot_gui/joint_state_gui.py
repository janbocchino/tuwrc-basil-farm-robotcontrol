import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = {"1", "2", "3", "4", "5"}
GRIPPER_JOINTS = {"6"}
LINEAR_JOINTS = {"rail_joint"}  # prismatic, values in meters (not radians)
ARM_ACTION_JOINTS = {"1", "2", "3", "4", "5", "6"}
# Pose reference frame: must be a FIXED frame. "base" moves with the rail
# carriage now, so targets/display are expressed in "world" instead.
BASE_FRAME = "world"
END_EFFECTOR_FRAME = "tcp"  # tool frame: Z = approach direction, avoids gimbal lock
MOVEIT_IK_SERVICE = "/compute_ik"
MOVEIT_GROUP = "arm"
ARM_ACTION_NAME = "/six_motor_controller/follow_joint_trajectory"
RAIL_ACTION_NAME = "/rail_controller/follow_joint_trajectory"

# TCP pose with every joint at 0 (matches the "home" group state in the SRDF).
# Position in meters, orientation in degrees, expressed in BASE_FRAME (world).
# Used by the "Home" button in the task-space section - resetting that section
# to 0/0/0 would be meaningless (that point sits inside the base, and an
# orientation of 0/0/0 would mean the gripper points straight up).
HOME_POSE = {
    "x": -0.0094, "y": -0.3675, "z": 0.2819,
    "roll": 90.0, "pitch": 0.0, "yaw": 0.0,
}

# Measured URDF limits from Max's so101_base.xacro (degrees for revolute joints).
JOINT_LIMITS_DEG = {
    "1": (-118.48, 118.48),
    "2": (-106.17, 106.17),
    "3": (-96.68, 96.77),
    "4": (-103.53, 103.53),
    "5": (-165.76, 165.76),
    "6": (-10.64, 116.45),
}

HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>TUWRC Robot Control</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Arial, Helvetica, sans-serif;
      background: #181a1f;
      color: #f4f4f5;
    }
    body { margin: 0; min-height: 100vh; background: #181a1f; }
    main { max-width: 1040px; margin: 0 auto; padding: 28px; }

    header {
      display: flex; align-items: flex-end;
      justify-content: space-between; gap: 16px; margin-bottom: 18px;
    }
    h1 { margin: 0; font-size: 28px; font-weight: 700; }

    .status {
      min-width: 150px; padding: 8px 10px;
      border: 1px solid #343842; border-radius: 6px;
      text-align: center; color: #cbd5e1; background: #20232b;
    }
    .status.live { color: #b9f6ca; border-color: #2e6f45; }

    table {
      width: 100%; border-collapse: collapse; overflow: hidden;
      border: 1px solid #343842; border-radius: 8px; background: #20232b;
    }
    th, td {
      padding: 12px 14px; border-bottom: 1px solid #343842;
      text-align: right; font-variant-numeric: tabular-nums;
    }
    th:first-child, td:first-child { text-align: left; }
    th { color: #aab3c2; font-size: 13px; font-weight: 700; text-transform: uppercase; }
    tr:last-child td { border-bottom: 0; }
    tr:hover td { background: #252830; }

    /* divider between read-only and editable columns */
    th.divider, td.divider { border-left: 1px solid #444a58; }

    .target-cell { display: flex; align-items: center; gap: 8px; justify-content: flex-end; }

    .target-input {
      width: 88px; padding: 6px 8px;
      background: #2a2d38; border: 1px solid #444a58;
      border-radius: 5px; color: #f4f4f5;
      font-size: 14px; text-align: right;
      font-variant-numeric: tabular-nums;
      transition: border-color 0.15s;
    }
    .target-input:focus { outline: none; border-color: #6b8cff; background: #1e2130; }

    .limit-hint { font-size: 11px; color: #555d6e; white-space: nowrap; }
    .section-title { margin: 24px 0 10px; font-size: 17px; font-weight: 700; }
    .section-meta { color: #7d8797; font-size: 12px; font-weight: 400; margin-left: 8px; }

    /* control bar */
    .control-bar {
      margin-top: 14px; display: flex; align-items: center;
      gap: 10px; flex-wrap: wrap;
    }
    .control-label {
      display: flex; align-items: center; gap: 7px;
      color: #aab3c2; font-size: 14px;
    }
    .duration-input {
      width: 68px; padding: 8px 10px;
      background: #20232b; border: 1px solid #343842;
      border-radius: 6px; color: #f4f4f5; font-size: 14px; text-align: right;
    }
    .duration-input:focus { outline: none; border-color: #6b8cff; }

    .btn {
      padding: 9px 18px; border-radius: 6px;
      font-size: 14px; font-weight: 600; cursor: pointer;
      border: 1px solid transparent; transition: background 0.15s, opacity 0.15s;
    }
    .btn-ghost {
      background: transparent; border-color: #444a58; color: #aab3c2;
    }
    .btn-ghost:hover { background: #2a2d38; }
    .btn-secondary {
      background: #2a2d38; border-color: #444a58; color: #cbd5e1;
    }
    .btn-secondary:hover { background: #343842; }
    .btn-primary {
      background: #3b5bdb; border-color: #3b5bdb; color: #fff;
    }
    .btn-primary:hover { background: #4c6ef5; }
    .btn-primary:disabled { background: #2a2d38; border-color: #444a58; color: #6b7280; cursor: default; }

    .feedback {
      margin-left: auto; font-size: 13px; padding: 7px 14px;
      border-radius: 5px; opacity: 0; transition: opacity 0.2s;
    }
    .feedback.show-ok  { background: #1a3a2a; color: #b9f6ca; opacity: 1; }
    .feedback.show-err { background: #3a1a1a; color: #fca5a5; opacity: 1; }

    .empty {
      padding: 28px; border: 1px solid #343842;
      border-radius: 8px; color: #aab3c2; background: #20232b;
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>TUWRC Robot Control</h1>
      <div id="status" class="status">waiting</div>
    </header>
    <p id="mode-line" style="margin-top:-8px;color:#94a3b8;font-size:14px;">Mode: detecting…</p>

    <div id="empty" class="empty">Waiting for /joint_states…</div>

    <table id="table" hidden>
      <thead>
        <tr>
          <th>Joint</th>
          <th>Position rad/m</th>
          <th>Position deg</th>
          <th>Velocity</th>
          <th class="divider">Target deg/m</th>
          <th>Range</th>
        </tr>
      </thead>
      <tbody id="rows"></tbody>
    </table>

    <div id="controls" class="control-bar" hidden>
      <label class="control-label">
        Duration&nbsp;(s)
        <input type="number" id="duration" class="duration-input"
               value="2.0" min="0.1" max="10" step="0.1">
      </label>
      <button class="btn btn-ghost"      id="zero-btn">Reset to 0</button>
      <button class="btn btn-secondary"  id="fill-btn">Fill current</button>
      <button class="btn btn-primary"    id="send-btn">Send</button>
      <span class="feedback" id="feedback"></span>
    </div>

    <section id="pose-section" hidden>
      <h2 class="section-title">
        End Effector Pose
        <span class="section-meta" id="pose-frame">world → tcp</span>
      </h2>
      <table id="pose-table">
        <thead>
          <tr>
            <th>Coordinate</th>
            <th>Current</th>
            <th class="divider">Target</th>
            <th>Unit</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td><strong>X</strong></td><td id="pose-x">—</td>
            <td class="divider"><input type="number" class="target-input" id="pose-target-x" value="0.0000" step="0.001"></td><td>m</td>
          </tr>
          <tr>
            <td><strong>Y</strong></td><td id="pose-y">—</td>
            <td class="divider"><input type="number" class="target-input" id="pose-target-y" value="0.0000" step="0.001"></td><td>m</td>
          </tr>
          <tr>
            <td><strong>Z</strong></td><td id="pose-z">—</td>
            <td class="divider"><input type="number" class="target-input" id="pose-target-z" value="0.0000" step="0.001"></td><td>m</td>
          </tr>
          <tr>
            <td><strong>Roll</strong> <span class="limit-hint">rotation around X axis</span></td><td id="pose-roll">—</td>
            <td class="divider"><input type="number" class="target-input" id="pose-target-roll" value="0.0000" step="1"></td><td>deg</td>
          </tr>
          <tr>
            <td><strong>Pitch</strong> <span class="limit-hint">rotation around Y axis</span></td><td id="pose-pitch">—</td>
            <td class="divider"><input type="number" class="target-input" id="pose-target-pitch" value="0.0000" step="1"></td><td>deg</td>
          </tr>
          <tr>
            <td><strong>Yaw</strong> <span class="limit-hint">rotation around Z axis</span></td><td id="pose-yaw">—</td>
            <td class="divider"><input type="number" class="target-input" id="pose-target-yaw" value="0.0000" step="1"></td><td>deg</td>
          </tr>
        </tbody>
      </table>
      <div id="pose-controls" class="control-bar">
        <label class="control-label">
          Duration&nbsp;(s)
          <input type="number" id="pose-duration" class="duration-input"
                 value="2.0" min="0.1" max="10" step="0.1">
        </label>
        <button class="btn btn-ghost"      id="pose-home-btn">Home</button>
        <button class="btn btn-secondary"  id="pose-fill-btn">Fill current</button>
        <button class="btn btn-primary"    id="pose-send-btn">Send</button>
        <span class="feedback" id="pose-feedback"></span>
      </div>
    </section>
  </main>

  <script>
    const JOINT_ORDER  = ["rail_joint", "1", "2", "3", "4", "5", "6"];
    const LINEAR_JOINTS = new Set(["rail_joint"]);   // prismatic, values in meters
    const JOINT_LIMITS = {
      "rail_joint": [-0.5, 0.5],
      "1": [-118.48, 118.48], "2": [-106.17, 106.17], "3": [-96.68, 96.77],
      "4": [-103.53, 103.53], "5": [-165.76, 165.76], "6": [-10.64, 116.45],
    };
    const modeLineEl = document.getElementById("mode-line");

    const statusEl   = document.getElementById("status");
    const emptyEl    = document.getElementById("empty");
    const tableEl    = document.getElementById("table");
    const rowsEl     = document.getElementById("rows");
    const controlsEl = document.getElementById("controls");
    const fillBtn    = document.getElementById("fill-btn");
    const zeroBtn    = document.getElementById("zero-btn");
    const sendBtn    = document.getElementById("send-btn");
    const feedbackEl = document.getElementById("feedback");
    const poseSectionEl = document.getElementById("pose-section");
    const poseFrameEl = document.getElementById("pose-frame");
    const poseFillBtn = document.getElementById("pose-fill-btn");
    const poseHomeBtn = document.getElementById("pose-home-btn");
    const HOME_POSE   = __HOME_POSE__;   // injected by the server
    const poseSendBtn = document.getElementById("pose-send-btn");
    const poseFeedbackEl = document.getElementById("pose-feedback");

    let currentDeg = {};
    let currentPose = null;
    let poseTargetsInitialized = false;
    let initialized = false;

    function fmt(v, d = 4) {
      return typeof v === "number" && isFinite(v) ? v.toFixed(d) : "—";
    }

    function initRows(joints) {
      const names = JOINT_ORDER.filter(n => joints.some(j => j.name === n));
      rowsEl.innerHTML = "";
      for (const name of names) {
        const linear = LINEAR_JOINTS.has(name);
        const [lo, hi] = JOINT_LIMITS[name] || [-180, 180];
        const unit = linear ? "m" : "°";
        const step = linear ? 0.01 : 0.5;
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td><strong>${name}</strong></td>
          <td id="r-${name}">—</td>
          <td id="d-${name}">—</td>
          <td id="v-${name}">—</td>
          <td class="divider">
            <div class="target-cell">
              <input type="number" class="target-input" id="t-${name}"
                     value="0.00" step="${step}" min="${lo}" max="${hi}">
            </div>
          </td>
          <td><span class="limit-hint">${lo}${unit} … ${hi}${unit}</span></td>
        `;
        rowsEl.appendChild(tr);
      }
      initialized = true;
    }

    function updateRows(joints) {
      for (const j of joints) {
        const linear = LINEAR_JOINTS.has(j.name);
        currentDeg[j.name] = linear ? j.position : j.position_deg;
        const r = document.getElementById(`r-${j.name}`);
        const d = document.getElementById(`d-${j.name}`);
        const v = document.getElementById(`v-${j.name}`);
        if (r) r.textContent = fmt(j.position);
        if (d) d.textContent = linear ? "—" : fmt(j.position_deg, 2);
        if (v) v.textContent = fmt(j.velocity);
      }
    }

    function updatePose(pose) {
      poseSectionEl.hidden = false;
      if (!pose || !pose.available) {
        poseFrameEl.textContent = pose && pose.error ? pose.error : "waiting for TF";
        ["x", "y", "z", "roll", "pitch", "yaw"].forEach(name => {
          document.getElementById(`pose-${name}`).textContent = "—";
        });
        return;
      }
      currentPose = pose;
      poseFrameEl.textContent = `${pose.parent_frame} → ${pose.child_frame}`;
      document.getElementById("pose-x").textContent = fmt(pose.x, 4);
      document.getElementById("pose-y").textContent = fmt(pose.y, 4);
      document.getElementById("pose-z").textContent = fmt(pose.z, 4);
      document.getElementById("pose-roll").textContent = fmt(pose.roll_deg, 2);
      document.getElementById("pose-pitch").textContent = fmt(pose.pitch_deg, 2);
      document.getElementById("pose-yaw").textContent = fmt(pose.yaw_deg, 2);
      if (!poseTargetsInitialized) {
        fillPoseTargetsFromCurrent();
        poseTargetsInitialized = true;
      }
    }

    const events = new EventSource("/events");
    events.onopen  = () => { statusEl.textContent = "connected"; statusEl.classList.add("live"); };
    events.onerror = () => { statusEl.textContent = "reconnecting"; statusEl.classList.remove("live"); };
    events.onmessage = (e) => {
      const state = JSON.parse(e.data);
      statusEl.textContent = `live ${state.age_ms} ms`;
      statusEl.classList.add("live");
      emptyEl.hidden    = true;
      tableEl.hidden    = false;
      controlsEl.hidden = false;
      if (!initialized) initRows(state.joints);
      updateRows(state.joints);
      updatePose(state.end_effector_pose);
      if (state.status && modeLineEl) {
        const ready = state.status.controllers_ready ? "controllers ready" : "waiting for controllers";
        modeLineEl.textContent = `Mode: ${state.status.mode} · ${ready}. ${state.status.note || ""}`;
      }
    };

    fillBtn.addEventListener("click", () => {
      for (const [name, deg] of Object.entries(currentDeg)) {
        const inp = document.getElementById(`t-${name}`);
        if (inp) inp.value = deg.toFixed(2);
      }
    });

    zeroBtn.addEventListener("click", () => {
      JOINT_ORDER.forEach(name => {
        const inp = document.getElementById(`t-${name}`);
        if (inp) inp.value = "0.00";
      });
    });

    let feedbackTimer = null;
    let poseFeedbackTimer = null;
    function showFeedback(el, timerName, msg, ok) {
      el.textContent = msg;
      el.className = "feedback " + (ok ? "show-ok" : "show-err");
      const oldTimer = timerName === "pose" ? poseFeedbackTimer : feedbackTimer;
      clearTimeout(oldTimer);
      const newTimer = setTimeout(() => { el.className = "feedback"; }, 3000);
      if (timerName === "pose") poseFeedbackTimer = newTimer;
      else feedbackTimer = newTimer;
    }

    function fillPoseTargetsFromCurrent() {
      if (!currentPose || !currentPose.available) return;
      document.getElementById("pose-target-x").value = currentPose.x.toFixed(4);
      document.getElementById("pose-target-y").value = currentPose.y.toFixed(4);
      document.getElementById("pose-target-z").value = currentPose.z.toFixed(4);
      document.getElementById("pose-target-roll").value  = currentPose.roll_deg.toFixed(2);
      document.getElementById("pose-target-pitch").value = currentPose.pitch_deg.toFixed(2);
      document.getElementById("pose-target-yaw").value   = currentPose.yaw_deg.toFixed(2);
    }

    poseFillBtn.addEventListener("click", fillPoseTargetsFromCurrent);

    // Fill the task-space targets with the pose the robot has when every joint
    // is at 0. Sending it moves the arm back to its home posture.
    poseHomeBtn.addEventListener("click", () => {
      document.getElementById("pose-target-x").value = HOME_POSE.x.toFixed(4);
      document.getElementById("pose-target-y").value = HOME_POSE.y.toFixed(4);
      document.getElementById("pose-target-z").value = HOME_POSE.z.toFixed(4);
      document.getElementById("pose-target-roll").value  = HOME_POSE.roll.toFixed(2);
      document.getElementById("pose-target-pitch").value = HOME_POSE.pitch.toFixed(2);
      document.getElementById("pose-target-yaw").value   = HOME_POSE.yaw.toFixed(2);
    });

    poseSendBtn.addEventListener("click", async () => {
      const pose = {
        x: parseFloat(document.getElementById("pose-target-x").value),
        y: parseFloat(document.getElementById("pose-target-y").value),
        z: parseFloat(document.getElementById("pose-target-z").value),
        roll:  parseFloat(document.getElementById("pose-target-roll").value),
        pitch: parseFloat(document.getElementById("pose-target-pitch").value),
        yaw:   parseFloat(document.getElementById("pose-target-yaw").value),
      };
      const duration = parseFloat(document.getElementById("pose-duration").value) || 2.0;

      poseSendBtn.disabled = true;
      try {
        const resp = await fetch("/send_pose", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ pose, duration }),
        });
        const result = await resp.json().catch(() => ({}));
        showFeedback(
          poseFeedbackEl,
          "pose",
          resp.ok ? "Position trajectory sent!" : (result.error || `Error ${resp.status}`),
          resp.ok
        );
      } catch {
        showFeedback(poseFeedbackEl, "pose", "Connection error", false);
      }
      poseSendBtn.disabled = false;
    });

    function showJointFeedback(msg, ok) {
      showFeedback(feedbackEl, "joint", msg, ok);
    }

    sendBtn.addEventListener("click", async () => {
      const joints = {};
      for (const name of JOINT_ORDER) {
        const inp = document.getElementById(`t-${name}`);
        if (inp) joints[name] = parseFloat(inp.value);
      }
      const duration = parseFloat(document.getElementById("duration").value) || 2.0;

      sendBtn.disabled = true;
      try {
        const resp = await fetch("/send", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ joints, duration }),
        });
        showJointFeedback(resp.ok ? "Trajectory sent!" : `Error ${resp.status}`, resp.ok);
      } catch {
        showJointFeedback("Connection error", false);
      }
      sendBtn.disabled = false;
    });
  </script>
</body>
</html>
"""


class JointStateStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._latest = None

    def update(self, message: JointState):
        joints = []
        for index, name in enumerate(message.name):
            position = self._value_at(message.position, index)
            velocity = self._value_at(message.velocity, index)
            effort   = self._value_at(message.effort,   index)
            joints.append({
                "name":         name,
                "position":     position,
                "position_deg": math.degrees(position) if position is not None else None,
                "velocity":     velocity,
                "effort":       effort,
            })
        with self._lock:
            self._latest = {"received_at": time.time(), "joints": joints}

    def snapshot(self):
        with self._lock:
            if self._latest is None:
                return None
            s = dict(self._latest)
            s["age_ms"] = int((time.time() - s["received_at"]) * 1000)
            return s

    def joint_positions(self):
        with self._lock:
            if self._latest is None:
                return {}
            return {
                joint["name"]: joint["position"]
                for joint in self._latest["joints"]
                if joint["position"] is not None
            }

    @staticmethod
    def _value_at(values, index):
        if index >= len(values):
            return None
        v = float(values[index])
        return None if math.isnan(v) else v


class JointStateNode(Node):
    def __init__(self, store: JointStateStore, mode: str = "view"):
        super().__init__("joint_state_gui")
        self._store = store
        self._mode = mode
        self._last_error = ""
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, ARM_ACTION_NAME
        )
        self._rail_client = ActionClient(
            self, FollowJointTrajectory, RAIL_ACTION_NAME
        )
        self._ik_client = self.create_client(GetPositionIK, MOVEIT_IK_SERVICE)
        self.get_logger().info(
            f"Listening to /joint_states; mode={mode}; using action clients"
        )

    def _on_joint_state(self, message: JointState):
        self._store.update(message)

    def status_info(self):
        arm_ready = self._arm_client.server_is_ready()
        rail_ready = self._rail_client.server_is_ready()
        return {
            "mode": self._mode,
            "arm_action_ready": arm_ready,
            "rail_action_ready": rail_ready,
            "controllers_ready": arm_ready and rail_ready,
            "last_error": self._last_error,
            "note": (
                "Rail motion is disabled in hardware mode."
                if self._mode == "hardware"
                else "View mode uses mock controllers."
            ),
        }

    def end_effector_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                BASE_FRAME, END_EFFECTOR_FRAME, Time()
            )
        except TransformException as exc:
            return {
                "available": False,
                "parent_frame": BASE_FRAME,
                "child_frame": END_EFFECTOR_FRAME,
                "error": f"waiting for {BASE_FRAME} → {END_EFFECTOR_FRAME}",
                "details": str(exc),
            }

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        roll, pitch, yaw = quaternion_to_euler(rotation.x, rotation.y, rotation.z, rotation.w)
        return {
            "available": True,
            "parent_frame": BASE_FRAME,
            "child_frame": END_EFFECTOR_FRAME,
            "x": translation.x,
            "y": translation.y,
            "z": translation.z,
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
        }

    def send_trajectory(self, targets_rad: dict, duration_sec: float):
        arm = {
            name: value
            for name, value in targets_rad.items()
            if name in ARM_ACTION_JOINTS
        }
        rail = {
            name: value
            for name, value in targets_rad.items()
            if name in LINEAR_JOINTS
        }
        if rail and self._mode == "hardware":
            raise RuntimeError(
                "Physical rail motion is disabled. Arm joints can still move."
            )
        if arm:
            self._send_action(self._arm_client, ARM_ACTION_NAME, arm, duration_sec)
        if rail:
            self._send_action(self._rail_client, RAIL_ACTION_NAME, rail, duration_sec)
        self._last_error = ""
        self.get_logger().info(f"Sent trajectory: {targets_rad} in {duration_sec}s")

    def _send_action(self, client, action_name, joints, duration_sec):
        if not client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(
                f"Action server {action_name} is not available. "
                "Start view or hardware bringup first."
            )
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(joints.keys())
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in joints.values()]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1_000_000_000),
        )
        traj.points = [point]
        goal.trajectory = traj

        send_future = client.send_goal_async(goal)
        done = threading.Event()
        send_future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=5.0):
            raise RuntimeError(f"Timed out sending goal to {action_name}.")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Goal rejected by {action_name}.")

        result_future = goal_handle.get_result_async()
        result_done = threading.Event()
        result_future.add_done_callback(lambda _: result_done.set())
        if not result_done.wait(timeout=max(duration_sec + 10.0, 15.0)):
            raise RuntimeError(f"Timed out waiting for {action_name} result.")
        wrapped = result_future.result()
        result = wrapped.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            message = result.error_string or f"error_code={result.error_code}"
            self._last_error = message
            raise RuntimeError(message)

    def send_pose_trajectory(self, pose_target: dict, duration_sec: float):
        if not self._ik_client.wait_for_service(timeout_sec=0.2):
            raise RuntimeError(
                "MoveIt IK service /compute_ik is not available. Start MoveIt first."
            )

        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(pose_target["x"])
        pose.pose.position.y = float(pose_target["y"])
        pose.pose.position.z = float(pose_target["z"])
        roll  = math.radians(float(pose_target.get("roll", 0.0)))
        pitch = math.radians(float(pose_target.get("pitch", 0.0)))
        yaw   = math.radians(float(pose_target.get("yaw", 0.0)))
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        request = GetPositionIK.Request()
        request.ik_request.group_name = MOVEIT_GROUP
        request.ik_request.ik_link_name = END_EFFECTOR_FRAME
        request.ik_request.pose_stamped = pose
        request.ik_request.timeout.sec = 2
        current_positions = self._store.joint_positions()
        request.ik_request.robot_state.joint_state.name = list(current_positions.keys())
        request.ik_request.robot_state.joint_state.position = list(current_positions.values())

        future = self._ik_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=5.0):
            raise RuntimeError("Timed out waiting for MoveIt IK response.")

        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt IK returned no response.")
        if response.error_code.val != 1:
            raise RuntimeError(moveit_error_message(response.error_code.val))

        solution = {
            name: position
            for name, position in zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
            if name in ARM_JOINTS or name in LINEAR_JOINTS
        }
        if not solution:
            raise RuntimeError("MoveIt IK returned no arm joint solution.")
        if self._mode == "hardware" and "rail_joint" in solution:
            # Keep the physical rail fixed; still apply the arm portion.
            solution.pop("rail_joint", None)
        self.send_trajectory(solution, duration_sec)
        self.get_logger().info(f"Sent pose target via IK: {pose_target}")


def make_request_handler(store: JointStateStore, node: JointStateNode):
    class RequestHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                self._send_html()
            elif self.path == "/events":
                self._send_events()
            elif self.path == "/status":
                self._json(node.status_info())
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path == "/send":
                try:
                    length = int(self.headers.get("Content-Length", 0))
                    body = json.loads(self.rfile.read(length))
                    joints_in  = body.get("joints", {})
                    duration   = float(body.get("duration", 2.0))
                    joints_rad = {
                        k: (float(v) if k in LINEAR_JOINTS else math.radians(float(v)))
                        for k, v in joints_in.items()
                    }
                    node.send_trajectory(joints_rad, duration)
                    self._json({"ok": True})
                except Exception as exc:
                    self._json({"ok": False, "error": str(exc)}, 400)
            elif self.path == "/send_pose":
                try:
                    length = int(self.headers.get("Content-Length", 0))
                    body = json.loads(self.rfile.read(length))
                    pose = body.get("pose", {})
                    duration = float(body.get("duration", 2.0))
                    node.send_pose_trajectory(pose, duration)
                    self._json({"ok": True})
                except Exception as exc:
                    self._json({"ok": False, "error": str(exc)}, 400)
            else:
                self.send_error(404)

        def log_message(self, format, *args):
            return

        def _send_html(self):
            page = HTML_PAGE.replace("__HOME_POSE__", json.dumps(HOME_POSE))
            body = page.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            while True:
                snapshot = store.snapshot()
                if snapshot is not None:
                    snapshot["end_effector_pose"] = node.end_effector_pose()
                    snapshot["status"] = node.status_info()
                    self.wfile.write(f"data: {json.dumps(snapshot)}\n\n".encode("utf-8"))
                    self.wfile.flush()
                time.sleep(0.1)

        def _json(self, data, status=200):
            body = json.dumps(data).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return RequestHandler


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


def moveit_error_message(code):
    messages = {
        -16: "MoveIt IK failed: invalid planning group name.",
        -19: "MoveIt IK failed: invalid end-effector link name.",
        -21: "MoveIt IK failed: frame transform failed.",
        -23: "MoveIt IK failed: robot state is stale.",
        -31: (
            "MoveIt IK found no solution for this position. Try Fill current first, "
            "then change x/y/z in small steps."
        ),
    }
    return messages.get(code, f"MoveIt IK failed with error code {code}.")


def main():
    parser = argparse.ArgumentParser(description="TUWRC joint monitor + control GUI.")
    parser.add_argument("--port", type=int, default=3000)
    parser.add_argument(
        "--mode",
        choices=["view", "hardware"],
        default="view",
        help="view: mock controllers; hardware: real arm + fixed rail",
    )
    args = parser.parse_args()

    rclpy.init()
    store = JointStateStore()
    node  = JointStateNode(store, mode=args.mode)

    try:
        server = ThreadingHTTPServer(
            ("0.0.0.0", args.port), make_request_handler(store, node)
        )
    except OSError as exc:
        if exc.errno == 98:
            node.get_logger().error(f"Port {args.port} already in use.")
            node.destroy_node()
            rclpy.shutdown()
            return
        raise

    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"GUI available at http://localhost:{args.port}")

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
