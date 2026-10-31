#!/usr/bin/env python3
"""
Eye-in-hand calibration for the Gemini 2 mounted on the SO-101 `wrist` link.

What it does
  1. You move the arm to a pose and press ENTER. The script waits for the arm to
     settle, grabs a fresh colour image, finds the ChArUco board, and stores
     T_base_wrist (from TF) and T_camera_board (from solvePnP).
  2. After 15-25 poses type `s` to solve. It prints the six camera_mount_* values
     for the xacro (camera_link expressed in the `wrist` frame) and a residual
     report that tells you whether the calibration can be trusted.

Rules for good data
  - Keep the rail still. Board fixed rigidly (anywhere, position does not matter).
  - Vary the camera ORIENTATION (+-20-30 deg about every axis), not only position.
  - Board 25-40 cm from the camera, fully visible, no motion blur.

Usage (inside your ROS 2 container, with the camera driver and robot_state_publisher running)
  python3 handeye_wrist.py --ros-args \
      -p squares_x:=5 -p squares_y:=7 -p square_length:=0.035 -p marker_length:=0.026 \
      -p dictionary:=DICT_5X5_250
  Commands at the prompt:  ENTER = capture,  d = drop last,  s = solve,  q = quit
"""
import sys
import threading
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

# ----------------------------------------------------------------------------
# Pure functions (no ROS) -- these are unit-tested offline
# ----------------------------------------------------------------------------


def make_board(squares_x, squares_y, square_length, marker_length, dictionary_name):
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
    if hasattr(aruco, "CharucoBoard_create"):          # OpenCV <= 4.6
        board = aruco.CharucoBoard_create(squares_x, squares_y, square_length,
                                          marker_length, dictionary)
    else:                                              # OpenCV >= 4.7
        board = aruco.CharucoBoard((squares_x, squares_y), square_length,
                                   marker_length, dictionary)
    return board, dictionary


def board_object_points(board):
    return board.getChessboardCorners() if hasattr(board, "getChessboardCorners") \
        else board.chessboardCorners


def detect_board_pose(gray, board, dictionary, K, dist, min_corners=8):
    """Return (T_cam_board 4x4, reprojection_rms_px, n_corners) or None."""
    aruco = cv2.aruco
    if hasattr(aruco, "CharucoDetector"):              # OpenCV >= 4.7
        det = aruco.CharucoDetector(board)
        ch_corners, ch_ids, _, _ = det.detectBoard(gray)
    else:                                              # OpenCV <= 4.6
        corners, ids, _ = aruco.detectMarkers(gray, dictionary)
        if ids is None or len(ids) == 0:
            return None
        n, ch_corners, ch_ids = aruco.interpolateCornersCharuco(corners, ids, gray, board)
        if n is None or n < 1:
            return None
    if ch_ids is None or len(ch_ids) < min_corners:
        return None
    obj_all = board_object_points(board)
    obj = obj_all[ch_ids.flatten()].reshape(-1, 3).astype(np.float64)
    img = ch_corners.reshape(-1, 2).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.flatten()
    return T, rms, len(ch_ids)


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def solve_handeye(T_base_ee_list, T_cam_board_list):
    """Eye-in-hand. Returns {method: (T_ee_cam, trans_std_mm, rot_std_deg)}."""
    Rg = [T[:3, :3] for T in T_base_ee_list]
    tg = [T[:3, 3].reshape(3, 1) for T in T_base_ee_list]
    Rt = [T[:3, :3] for T in T_cam_board_list]
    tt = [T[:3, 3].reshape(3, 1) for T in T_cam_board_list]
    out = {}
    for name, m in METHODS.items():
        try:
            R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=m)
        except cv2.error:
            continue
        X = np.eye(4)
        X[:3, :3] = R
        X[:3, 3] = t.flatten()
        if not np.all(np.isfinite(X)):
            continue
        # Consistency: the board is rigid, so T_base_board must be identical for all samples
        boards = [Tb @ X @ Tc for Tb, Tc in zip(T_base_ee_list, T_cam_board_list)]
        P = np.array([b[:3, 3] for b in boards])
        mean_R = Rot.from_matrix([b[:3, :3] for b in boards]).mean()
        ang = [np.degrees((Rot.from_matrix(b[:3, :3]) * mean_R.inv()).magnitude()) for b in boards]
        out[name] = (X, float(np.sqrt(np.mean(np.sum((P - P.mean(0)) ** 2, axis=1))) * 1000),
                     float(np.sqrt(np.mean(np.square(ang)))))
    return out


def optical_to_camera_link(T_ee_opt, T_cl_opt):
    """T_wrist_camera_link = T_wrist_optical * inv(T_camera_link_optical)."""
    return T_ee_opt @ inv(T_cl_opt)


def xacro_args(T):
    r, p, y = Rot.from_matrix(T[:3, :3]).as_euler("xyz")   # URDF rpy convention
    x_, y_, z_ = T[:3, 3]
    return (f'<xacro:arg name="camera_mount_x"     default="{x_:.4f}" />\n'
            f'<xacro:arg name="camera_mount_y"     default="{y_:.4f}" />\n'
            f'<xacro:arg name="camera_mount_z"     default="{z_:.4f}" />\n'
            f'<xacro:arg name="camera_mount_roll"  default="{r:.4f}" />\n'
            f'<xacro:arg name="camera_mount_pitch" default="{p:.4f}" />\n'
            f'<xacro:arg name="camera_mount_yaw"   default="{y:.4f}" />')


def report(samples_base_ee, samples_cam_board, T_cl_opt):
    res = solve_handeye(samples_base_ee, samples_cam_board)
    if not res:
        print("Solver failed: need more / more varied poses.")
        return
    print(f"\n{len(samples_base_ee)} samples. Consistency of the (fixed) board pose in `base`:")
    print("  method      pos scatter [mm]   rot scatter [deg]")
    for k, (_, s_t, s_r) in res.items():
        print(f"  {k:<10}  {s_t:10.2f}        {s_r:8.2f}")
    best = min(res, key=lambda k: res[k][1])
    X, s_t, s_r = res[best]
    print(f"\nBest: {best}  (position scatter {s_t:.1f} mm, rotation scatter {s_r:.2f} deg)")
    if s_t < 4 and s_r < 0.7:
        print("=> GOOD. The mount was the problem and these values should fix it.")
    elif s_t < 10:
        print("=> OK but noisy: capture more, better-spread poses, check board size entered.")
    else:
        print("=> POOR (cm-level scatter). A constant mount transform cannot explain this:\n"
              "   the arm's joint states/kinematics do not match the real arm. Check joint\n"
              "   offsets/signs, stale /joint_states, servo not reaching targets, then redo.")
    T_wrist_cl = optical_to_camera_link(X, T_cl_opt)
    print("\nPaste into the xacro (camera_link in the `wrist` frame):\n")
    print(xacro_args(T_wrist_cl))
    print()


# ----------------------------------------------------------------------------
# ROS 2 node
# ----------------------------------------------------------------------------


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.time import Time
    from sensor_msgs.msg import Image, CameraInfo
    import tf2_ros

    rclpy.init(args=sys.argv)

    class Calib(Node):
        def __init__(self):
            super().__init__("handeye_wrist")
            d = self.declare_parameter
            self.sx = d("squares_x", 7).value
            self.sy = d("squares_y", 5).value
            self.sl = d("square_length", 0.025).value
            self.ml = d("marker_length", 0.018).value
            self.dict_name = d("dictionary", "DICT_6X6_250").value
            self.base = d("base_frame", "base").value
            self.ee = d("ee_frame", "wrist").value
            self.cam_link = d("camera_link_frame", "camera_link").value
            self.optical = d("optical_frame", "camera_color_optical_frame").value
            img_topic = d("image_topic", "/camera/color/image_raw").value
            info_topic = d("info_topic", "/camera/color/camera_info").value
            self.settle = d("settle_s", 1.5).value
            self.board, self.dictionary = make_board(self.sx, self.sy, self.sl, self.ml, self.dict_name)
            self.buf = tf2_ros.Buffer()
            self.lis = tf2_ros.TransformListener(self.buf, self)
            self.K = self.dist = None
            self.latest = None
            self.lock = threading.Lock()
            self.create_subscription(CameraInfo, info_topic, self.on_info, 1)
            self.create_subscription(Image, img_topic, self.on_img, 1)
            self.T_base_ee, self.T_cam_board = [], []

        def on_info(self, m):
            self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)
            self.dist = np.array(m.d, dtype=np.float64) if len(m.d) else np.zeros(5)

        def on_img(self, m):
            enc = m.encoding
            a = np.frombuffer(m.data, np.uint8)
            if enc in ("rgb8", "bgr8"):
                im = a.reshape(m.height, m.width, 3)
                gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY if enc == "rgb8" else cv2.COLOR_BGR2GRAY)
            elif enc == "mono8":
                gray = a.reshape(m.height, m.width)
            else:
                return
            with self.lock:
                self.latest = (Time.from_msg(m.header.stamp), gray)

        def tf(self, target, source, stamp=None):
            t = Time() if stamp is None else stamp
            T = self.buf.lookup_transform(target, source, t, timeout=Duration(seconds=1.0))
            q, p = T.transform.rotation, T.transform.translation
            M = np.eye(4)
            M[:3, :3] = Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            M[:3, 3] = [p.x, p.y, p.z]
            return M

        def capture(self):
            if self.K is None:
                print("No camera_info yet."); return
            t_req = self.get_clock().now() + Duration(seconds=self.settle)
            print(f"  settling {self.settle:.1f}s ...", end="", flush=True)
            time.sleep(self.settle)
            deadline = time.time() + 3
            while time.time() < deadline:                     # wait for an image newer than the settle time
                with self.lock:
                    cur = self.latest
                if cur is not None and cur[0] > t_req:
                    break
                time.sleep(0.05)
            else:
                print(" no fresh image."); return
            stamp, gray = cur
            r = detect_board_pose(gray, self.board, self.dictionary, self.K, self.dist)
            if r is None:
                print(" board not found / too few corners."); return
            T_cb, rms, n = r
            if rms > 1.0:
                print(f" rejected: reprojection {rms:.2f}px"); return
            try:
                T_be = self.tf(self.base, self.ee, stamp)
            except Exception as e:                            # image stamp not on ROS clock -> latest TF
                print(f" (TF at image stamp failed: {e.__class__.__name__}; using latest TF)", end="")
                T_be = self.tf(self.base, self.ee)
            self.T_base_ee.append(T_be)
            self.T_cam_board.append(T_cb)
            d = np.linalg.norm(T_cb[:3, 3])
            print(f" OK #{len(self.T_base_ee)}  ({n} corners, {rms:.2f}px, board {d*100:.0f} cm away)")

        def solve(self):
            if len(self.T_base_ee) < 6:
                print("Need at least 6 samples (15-25 recommended)."); return
            T_cl_opt = self.tf(self.cam_link, self.optical)
            report(self.T_base_ee, self.T_cam_board, T_cl_opt)
            np.savez("handeye_samples.npz", T_base_ee=self.T_base_ee, T_cam_board=self.T_cam_board,
                     T_cl_opt=T_cl_opt)
            print("Samples saved to handeye_samples.npz")

    node = Calib()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(__doc__.split("Usage")[0])
    print("Commands: ENTER = capture, d = drop last, s = solve, q = quit")
    while True:
        cmd = input("> ").strip().lower()
        if cmd == "":
            node.capture()
        elif cmd == "d" and node.T_base_ee:
            node.T_base_ee.pop(); node.T_cam_board.pop(); print(f"  {len(node.T_base_ee)} samples left")
        elif cmd == "s":
            node.solve()
        elif cmd == "q":
            break
    rclpy.shutdown()


if __name__ == "__main__":
    main()