#!/usr/bin/env python3
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

def quat_to_matrix(x, y, z, w):
    n = x*x + y*y + z*z + w*w
    s = 2.0 / n
    xx, yy, zz = x*x*s, y*y*s, z*z*s
    xy, xz, yz = x*y*s, x*z*s, y*z*s
    wx, wy, wz = w*x*s, w*y*s, w*z*s
    return np.array([
        [1 - (yy + zz), xy - wz,       xz + wy],
        [xy + wz,       1 - (xx + zz), yz - wx],
        [xz - wy,       yz + wx,       1 - (xx + yy)],
    ])


def tf_to_matrix(t):
    q = t.transform.rotation
    p = t.transform.translation
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def rot_angle_deg(T):
    c = np.clip((np.trace(T[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def cloud_to_np(msg):
    names = [f.name for f in msg.fields]
    pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
    rgb = None
    if "rgb" in names:
        raw = point_cloud2.read_points(msg, field_names=("rgb",), skip_nans=False)["rgb"]
        raw = np.ascontiguousarray(raw).view(np.uint32)
        full = np.column_stack([(raw >> 16) & 255, (raw >> 8) & 255, raw & 255]) / 255.0
        # only usable if the cloud is organised and no points were dropped
        if len(full) == len(xyz):
            rgb = full
    return xyz, rgb


class Fuse(Node):
    def __init__(self):
        super().__init__("pointcloud_fuser")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("topic", "/camera/depth_registered/points")
        self.declare_parameter("out", "/workspace/scan/fused.ply")
        self.declare_parameter("voxel", 0.002)          # final downsample (m)
        self.declare_parameter("min_range", 0.1)
        self.declare_parameter("max_range", 0.6)
        # ROI in target frame [xmin, ymin, zmin, xmax, ymax, zmax]; recommended
        self.declare_parameter("use_roi", False)
        self.declare_parameter("roi", [-0.5, -0.5, 0.0, 0.5, 0.5, 0.6])
        # ICP
        self.declare_parameter("use_icp", True)
        self.declare_parameter("icp_voxel", 0.003)      # downsample used for ICP
        self.declare_parameter("icp_max_dist", 0.01)    # max correspondence dist (m)
        self.declare_parameter("icp_min_fitness", 0.3)  # reject if overlap lower
        self.declare_parameter("icp_max_shift", 0.03)   # reject correction > 3 cm
        self.declare_parameter("icp_max_rot_deg", 5.0)  # reject correction > 5 deg

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.last = None
        self.parts_icp = []   # clouds after ICP refinement
        self.parts_tf = []    # clouds with TF only (for comparison)
        self.ref = None       # downsampled reference with normals

        self.create_subscription(PointCloud2, self.get_parameter("topic").value,
                                 lambda m: setattr(self, "last", m), 1)
        self.create_service(Trigger, "capture", self.capture)
        self.create_service(Trigger, "save", self.save)
        self.create_service(Trigger, "clear", self.clear)

    def _prep_ref(self, pc):
        v = self.get_parameter("icp_voxel").value
        ref = pc.voxel_down_sample(v)
        ref.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=v * 4, max_nn=30))
        return ref

    def capture(self, _, res):
        msg = self.last
        if msg is None:
            res.success, res.message = False, "no cloud received yet"
            return res
        target = self.get_parameter("target_frame").value
        try:
            tf = self.buf.lookup_transform(
                target, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=1.0))
        except Exception as e:
            res.success, res.message = False, f"tf failed: {e}"
            return res

        xyz, rgb = cloud_to_np(msg)
        d = np.linalg.norm(xyz, axis=1)
        keep = (d > self.get_parameter("min_range").value) & \
               (d < self.get_parameter("max_range").value)
        xyz = xyz[keep]
        rgb = rgb[keep] if rgb is not None else None

        T = tf_to_matrix(tf)
        xyz = xyz @ T[:3, :3].T + T[:3, 3]

        if self.get_parameter("use_roi").value:
            r = self.get_parameter("roi").value
            m = np.all((xyz >= r[:3]) & (xyz <= r[3:]), axis=1)
            xyz = xyz[m]
            rgb = rgb[m] if rgb is not None else None

        if len(xyz) < 100:
            res.success, res.message = False, f"only {len(xyz)} points after filtering"
            return res

        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
        if rgb is not None:
            pc.colors = o3d.utility.Vector3dVector(rgb)

        self.parts_tf.append(o3d.geometry.PointCloud(pc))   # copy, TF only
        info = "TF only"

        if self.get_parameter("use_icp").value:
            v = self.get_parameter("icp_voxel").value
            src = pc.voxel_down_sample(v)
            if self.ref is None:
                info = "reference scan"
            else:
                reg = o3d.pipelines.registration.registration_icp(
                    src, self.ref,
                    self.get_parameter("icp_max_dist").value,
                    np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
                Tc = reg.transformation
                shift = float(np.linalg.norm(Tc[:3, 3]))
                ang = rot_angle_deg(Tc)
                ok = (reg.fitness >= self.get_parameter("icp_min_fitness").value and
                      shift <= self.get_parameter("icp_max_shift").value and
                      ang <= self.get_parameter("icp_max_rot_deg").value)
                info = (f"ICP {'applied' if ok else 'REJECTED'}: fitness={reg.fitness:.2f} "
                        f"rmse={reg.inlier_rmse * 1000:.1f}mm "
                        f"correction={shift * 1000:.1f}mm/{ang:.2f}deg")
                if ok:
                    pc.transform(Tc)

            # grow the reference with this (possibly corrected) scan
            if self.ref is None:
                self.ref = self._prep_ref(pc)
            else:
                combined = o3d.geometry.PointCloud(self.ref)
                combined += pc
                self.ref = self._prep_ref(combined)

        self.parts_icp.append(pc)
        res.success, res.message = True, f"scan {len(self.parts_icp)}: {len(xyz)} pts, {info}"
        self.get_logger().info(res.message)
        return res

    def _merge(self, parts):
        merged = o3d.geometry.PointCloud()
        for p in parts:
            merged += p
        merged = merged.voxel_down_sample(self.get_parameter("voxel").value)
        merged, _ = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        return merged

    def save(self, _, res):
        if not self.parts_icp:
            res.success, res.message = False, "nothing captured"
            return res
        out = self.get_parameter("out").value
        out_tf = out.replace(".ply", "_tf_only.ply")
        a = self._merge(self.parts_icp)
        b = self._merge(self.parts_tf)
        o3d.io.write_point_cloud(out, a)
        o3d.io.write_point_cloud(out_tf, b)
        res.success = True
        res.message = f"wrote {len(a.points)} pts to {out} and {len(b.points)} pts to {out_tf}"
        return res

    def clear(self, _, res):
        self.parts_icp.clear()
        self.parts_tf.clear()
        self.ref = None
        res.success, res.message = True, "cleared"
        return res


def main():
    rclpy.init()
    rclpy.spin(Fuse())


if __name__ == "__main__":
    main()