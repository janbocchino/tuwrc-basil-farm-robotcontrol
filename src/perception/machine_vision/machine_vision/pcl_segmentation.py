#!/usr/bin/env python3
"""
basil_stem_leaf_segmenter.py

Subscribes to a PointCloud2 (e.g. /camera/depth/points), isolates the basil plant
and labels every point as STEM or LEAF using local geometry (PCA features):

  * stem  -> locally LINEAR (one dominant eigenvalue) and oriented along "up"
  * leaf  -> locally PLANAR / scattered

Publishes
  ~/labeled  PointCloud2  all plant points, fields: x y z rgb label   (label 0=leaf, 1=stem)
  ~/stem     PointCloud2  stem points only (same fields)
  ~/leaves   PointCloud2  leaf points only (same fields)

Dependencies: open3d (>=0.17), numpy, sensor_msgs_py, tf2_ros, tf2_sensor_msgs
"""
import numpy as np
import open3d as o3d
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import tf2_ros
from tf2_ros import TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

LEAF, STEM = 0, 1
COLOR = {LEAF: (60, 200, 60), STEM: (230, 40, 40)}

CLOUD_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                        ("rgb", "<f4"), ("label", "<u4")])


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    arr = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    if getattr(arr, "dtype", None) is not None and arr.dtype.names:
        xyz = np.column_stack([arr["x"], arr["y"], arr["z"]])
    else:  # older sensor_msgs_py returns a generator of tuples
        xyz = np.asarray(list(arr), dtype=np.float64).reshape(-1, 3)
    xyz = xyz.astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def make_cloud(header, xyz: np.ndarray, labels: np.ndarray) -> PointCloud2:
    n = len(xyz)
    data = np.zeros(n, dtype=CLOUD_DTYPE)
    data["x"], data["y"], data["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rgb = np.zeros(n, dtype=np.uint32)
    for lab, (r, g, b) in COLOR.items():
        rgb[labels == lab] = (r << 16) | (g << 8) | b
    data["rgb"] = rgb.view(np.float32)
    data["label"] = labels.astype(np.uint32)

    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = 1, n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="label", offset=16, datatype=PointField.UINT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = CLOUD_DTYPE.itemsize
    msg.row_step = msg.point_step * n
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


class BasilStemLeafSegmenter(Node):
    def __init__(self):
        super().__init__("basil_stem_leaf_segmenter")

        # --- I/O -----------------------------------------------------------
        self.declare_parameter("input_topic", "/camera/depth/points")
        # If set, clouds are transformed into this frame first (recommended, e.g. "base_link"),
        # so ROI and "up" are fixed in the world rather than moving with the camera.
        self.declare_parameter("target_frame", "")
        self.declare_parameter("accumulate_frames", 5)   # merge N frames (static scene only)
        self.declare_parameter("min_period", 1.0)        # s, throttle processing

        # --- Region of interest (in target_frame, or camera frame if empty) --
        # Camera optical frame: x right, y down, z forward.
        self.declare_parameter("roi_min", [-0.1, -0.25, 0.15])
        self.declare_parameter("roi_max", [0.2, 0.1 , 0.40])
        # Direction the stem grows. Optical frame w/ horizontal camera: [0,-1,0].
        # In base_link/world: [0,0,1].
        self.declare_parameter("up_axis", [0.0, -1.0, 0.0])

        # --- Preprocessing ----------------------------------------------------
        self.declare_parameter("voxel_size", 0.002)
        self.declare_parameter("plant_cluster_eps", 0.01)
        self.declare_parameter("plant_cluster_min_points", 20)
        self.declare_parameter("min_plant_points", 200)

        # --- Stem detection ---------------------------------------------------
        self.declare_parameter("feature_radius", 0.05)     # PCA neighbourhood
        self.declare_parameter("linearity_thresh", 0.6)
        self.declare_parameter("verticality_thresh", 0.6)
        self.declare_parameter("stem_cluster_eps", 0.012)
        self.declare_parameter("stem_cluster_min_points", 5)

        p = self.get_parameter
        self.target_frame = p("target_frame").value
        self.n_accum = int(p("accumulate_frames").value)
        self.min_period = float(p("min_period").value)
        self.buffer = []
        self.last_run = self.get_clock().now()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            PointCloud2, p("input_topic").value, self.on_cloud, qos)
        self.pub_all = self.create_publisher(PointCloud2, "~/labeled", 1)
        self.pub_stem = self.create_publisher(PointCloud2, "~/stem", 1)
        self.pub_leaves = self.create_publisher(PointCloud2, "~/leaves", 1)
        self.get_logger().info(f"Listening on {p('input_topic').value}")

    # ------------------------------------------------------------------ cb
    def on_cloud(self, msg: PointCloud2):
        now = self.get_clock().now()
        if (now - self.last_run).nanoseconds * 1e-9 < self.min_period and not self.buffer:
            return

        if self.target_frame and msg.header.frame_id != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, msg.header.frame_id, rclpy.time.Time(),
                    timeout=Duration(seconds=0.5))
                msg = do_transform_cloud(msg, tf)
            except TransformException as e:
                self.get_logger().warn(f"TF failed: {e}")
                return

        self.buffer.append(cloud_to_xyz(msg))
        if len(self.buffer) < self.n_accum:
            return
        xyz = np.vstack(self.buffer)
        self.buffer = []
        self.last_run = now

        result = self.segment(xyz)
        if result is None:
            self.get_logger().warn("No plant found in ROI (check roi_min/roi_max).")
            return
        pts, labels = result

        # out = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        # colors = np.where((labels == STEM)[:, None], [0.9, 0.15, 0.15], [0.25, 0.8, 0.25])
        # out.colors = o3d.utility.Vector3dVector(colors)
        # o3d.io.write_point_cloud(f"/workspace/scans/basil_{self.get_clock().now().nanoseconds}.ply", out)

        header = msg.header
        self.pub_all.publish(make_cloud(header, pts, labels))
        self.pub_stem.publish(make_cloud(header, pts[labels == STEM], labels[labels == STEM]))
        self.pub_leaves.publish(make_cloud(header, pts[labels == LEAF], labels[labels == LEAF]))
        self.get_logger().info(
            f"plant: {len(pts)} pts | stem: {(labels == STEM).sum()} | leaves: {(labels == LEAF).sum()}")

    # ------------------------------------------------------------ pipeline
    def segment(self, xyz: np.ndarray):
        p = lambda k: self.get_parameter(k).value  # noqa: E731

        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
        pcd = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(
            np.array(p("roi_min")), np.array(p("roi_max"))))
        pcd = pcd.voxel_down_sample(p("voxel_size"))
        if len(pcd.points) < p("min_plant_points"):
            return None
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # Keep the largest connected blob = the plant
        lab = np.array(pcd.cluster_dbscan(
            eps=p("plant_cluster_eps"), min_points=p("plant_cluster_min_points")))
        if lab.size == 0 or (lab >= 0).sum() == 0:
            return None
        biggest = np.bincount(lab[lab >= 0]).argmax()
        pcd = pcd.select_by_index(np.where(lab == biggest)[0])
        if len(pcd.points) < p("min_plant_points"):
            return None
        pts = np.asarray(pcd.points)

        # Local PCA features
        pcd.estimate_covariances(o3d.geometry.KDTreeSearchParamHybrid(
            radius=p("feature_radius"), max_nn=60))
        cov = np.asarray(pcd.covariances)                 # (N,3,3)
        w, v = np.linalg.eigh(cov)                        # ascending eigenvalues
        l1 = np.maximum(w[:, 2], 1e-12)
        l2 = w[:, 1]
        linearity = (l1 - l2) / l1                        # ~1 for line-like neighbourhoods
        principal = v[:, :, 2]                            # direction of largest variance
        up = np.array(p("up_axis"), dtype=float)
        up /= np.linalg.norm(up)
        verticality = np.abs(principal @ up)

        cand = (linearity > p("linearity_thresh")) & (verticality > p("verticality_thresh"))
        stem_mask = self.largest_cluster(pts, cand)

        labels = np.where(stem_mask, STEM, LEAF).astype(np.uint32)
        return pts, labels

    def largest_cluster(self, pts, mask):
        """Keep only the biggest connected group among masked points (drops speckle)."""
        idx = np.where(mask)[0]
        out = np.zeros(len(pts), dtype=bool)
        if len(idx) < self.get_parameter("stem_cluster_min_points").value:
            return out
        sub = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts[idx]))
        lab = np.array(sub.cluster_dbscan(
            eps=self.get_parameter("stem_cluster_eps").value,
            min_points=self.get_parameter("stem_cluster_min_points").value))
        if lab.size == 0 or (lab >= 0).sum() == 0:
            return out
        out[idx[lab == np.bincount(lab[lab >= 0]).argmax()]] = True
        return out


def main():
    rclpy.init()
    node = BasilStemLeafSegmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()