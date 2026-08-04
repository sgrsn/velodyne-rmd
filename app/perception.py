#!/usr/bin/env python3
"""知覚ノード — 背景ボクセル差分によるドローン検出とKF追跡。

パイプライン(設計方針アーティファクト準拠):
  PointCloud2 → base_link変換(TF) → 背景ボクセル差分 → グリッドクラスタリング
  → 等速モデルKF(6状態) → 目標状態を /perception/target (Odometry) で配信

状態:
  MAP  — 起動から map_duration_s の間、点群をボクセル集合に蓄積(掃引1周期分)
  RUN  — 背景を凍結し、前景抽出+追跡を実行

背景照合は「点のボクセルの26近傍いずれかが背景に占有されていれば背景」
(=背景側を1格子膨張するのと等価)。背景はint64パックキーのソート済み
np配列で保持し、np.searchsortedで27オフセット分を一括照合する。

トラック管理: 単一目標前提。最近傍ゲート+N-of-M確定(鳥・ノイズ棄却)。
確定トラックのみOdometryを配信し、共分散も載せる(LOST時の探索幅決定用)。

オフライン開発: bag再生に use_sim_time:=true で追従する。
  ros2 bag play <bag> --clock
  python3 perception.py --ros-args -p use_sim_time:=true
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, Header, String
from tf2_ros import Buffer, TransformListener

# int64パック用のオフセット(±2^20格子 ≒ ±260km@0.25m、十分)
_B = 1 << 20
_S = 1 << 21


def pack(idx):
    """(N,3) int32格子インデックス → int64キー"""
    return ((idx[:, 0].astype(np.int64) + _B) * _S + (idx[:, 1] + _B)) * _S + (idx[:, 2] + _B)


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Track:
    """等速モデルKF(状態: x y z vx vy vz)。"""

    def __init__(self, pos, t, r_meas, sigma_a):
        self.x = np.array([*pos, 0.0, 0.0, 0.0])
        self.P = np.diag([r_meas**2] * 3 + [4.0] * 3)
        self.t = t
        self.r2 = r_meas**2
        self.sa2 = sigma_a**2
        self.hits = 1
        self.age = 1
        self.misses = 0
        self.confirmed = False

    def predict(self, t):
        dt = max(t - self.t, 0.0)
        self.t = t
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        q = self.sa2
        dt2, dt3, dt4 = dt * dt, dt**3, dt**4
        Q = np.zeros((6, 6))
        Q[:3, :3] = np.eye(3) * (dt4 / 4 * q)
        Q[:3, 3:] = Q[3:, :3] = np.eye(3) * (dt3 / 2 * q)
        Q[3:, 3:] = np.eye(3) * (dt2 * q)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        S = self.P[:3, :3] + np.eye(3) * self.r2
        K = self.P[:, :3] @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.x[:3])
        self.P = self.P - K @ self.P[:3, :]

    def gate_radius(self):
        """位置共分散に応じた許容ゲート半径[m]"""
        return 3.0 * math.sqrt(max(np.trace(self.P[:3, :3]) / 3.0, 1e-6)) + 0.5


class Perception(Node):
    def __init__(self):
        super().__init__("perception")
        self.declare_parameter("cloud_topic", "/livox/lidar")
        self.declare_parameter("voxel_m", 0.25)
        self.declare_parameter("cluster_cell_m", 0.35)
        self.declare_parameter("min_points", 2)
        self.declare_parameter("max_cluster_span_m", 2.0)   # ドローンサイズゲート
        self.declare_parameter("map_duration_s", 55.0)      # 掃引1周期+余裕
        self.declare_parameter("range_min_m", 0.5)
        self.declare_parameter("range_max_m", 40.0)
        self.declare_parameter("r_meas_m", 0.15)
        self.declare_parameter("sigma_a", 8.0)
        self.declare_parameter("confirm_hits", 5)           # N-of-M確定
        self.declare_parameter("max_misses", 8)             # 連続未検出で破棄

        self.voxel = self.get_parameter("voxel_m").value
        self.cell = self.get_parameter("cluster_cell_m").value
        self.min_pts = int(self.get_parameter("min_points").value)
        self.max_span = self.get_parameter("max_cluster_span_m").value
        self.map_dur = self.get_parameter("map_duration_s").value
        self.rmin = self.get_parameter("range_min_m").value
        self.rmax = self.get_parameter("range_max_m").value
        self.r_meas = self.get_parameter("r_meas_m").value
        self.sigma_a = self.get_parameter("sigma_a").value
        self.confirm_hits = int(self.get_parameter("confirm_hits").value)
        self.max_misses = int(self.get_parameter("max_misses").value)

        # spin_thread=True: 点群コールバック内でTF到着をブロック待ちするため、
        # TF購読は専用スレッドで回す(単一executorだと待ち中にTFが入らずデッドロック)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.mode = "MAP"
        self.map_keys = set()          # MAP中の一時集合
        self.bg = None                 # RUN中: ソート済みint64キー配列
        self.map_t0 = None
        self.tracks = []
        self._neighbor_offsets = None  # 27近傍のパックキーオフセット

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, self.get_parameter("cloud_topic").value,
                                 self._on_cloud, qos)
        self.create_subscription(Empty, "perception/remap", self._on_remap, 1)
        self.pub_target = self.create_publisher(Odometry, "perception/target", 10)
        self.pub_mode = self.create_publisher(String, "perception/mode", 5)
        self.create_timer(0.5, lambda: self.pub_mode.publish(String(data=self.mode)))
        self.pub_fg = self.create_publisher(PointCloud2, "perception/foreground", 5)
        self.pub_clusters = self.create_publisher(PoseArray, "perception/clusters", 5)
        self.get_logger().info(
            f"MAP mode: accumulating background for {self.map_dur:.0f}s "
            f"(voxel {self.voxel}m, topic {self.get_parameter('cloud_topic').value})")

    def _on_remap(self, _):
        self.mode = "MAP"
        self.map_keys = set()
        self.bg = None
        self.map_t0 = None
        self.tracks = []
        self.get_logger().info("re-mapping background")

    # ---- 点群処理 ---------------------------------------------------------
    def _cloud_to_base(self, msg):
        """PointCloud2をbase_link系のNx3 float32へ。TF未解決ならNone。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                rclpy.duration.Duration(seconds=0.15))
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
            return None
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"),
                                             skip_nans=True)
        if pts.shape[0] == 0:
            return None
        r2 = np.einsum("ij,ij->i", pts, pts)
        m = (r2 > self.rmin**2) & (r2 < self.rmax**2)
        pts = pts[m]
        if pts.shape[0] == 0:
            return None
        q = tf.transform.rotation
        tr = tf.transform.translation
        R = quat_to_mat((q.x, q.y, q.z, q.w))
        return pts @ R.T + np.array([tr.x, tr.y, tr.z])

    def _on_cloud(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pts = self._cloud_to_base(msg)
        if pts is None:
            return

        if self.mode == "MAP":
            if self.map_t0 is None:
                self.map_t0 = t
            keys = pack(np.floor(pts / self.voxel).astype(np.int32))
            self.map_keys.update(np.unique(keys).tolist())
            if t - self.map_t0 >= self.map_dur:
                self.bg = np.array(sorted(self.map_keys), dtype=np.int64)
                self.map_keys = set()
                offs = np.arange(-1, 2, dtype=np.int64)
                self._neighbor_offsets = np.array(
                    [(dx * _S + dy) * _S + dz for dx in offs for dy in offs for dz in offs],
                    dtype=np.int64)
                self.mode = "RUN"
                self.get_logger().info(f"background frozen: {len(self.bg)} voxels — RUN mode")
            return

        # ---- RUN: 前景抽出(26近傍照合=背景1格子膨張と等価) ----
        keys = pack(np.floor(pts / self.voxel).astype(np.int32))
        is_bg = np.zeros(keys.shape[0], dtype=bool)
        for off in self._neighbor_offsets:
            k = keys + off
            i = np.searchsorted(self.bg, k)
            i[i >= self.bg.shape[0]] = 0
            is_bg |= self.bg[i] == k
        fg = pts[~is_bg]

        clusters = self._cluster(fg)
        self._track_update(clusters, t)
        if clusters:
            self.get_logger().info(
                f"t={t:.2f} clusters: " + "; ".join(
                    f"({c[0]:.2f},{c[1]:.2f},{c[2]:.2f}) n={n} span={s:.2f}"
                    for c, n, s in clusters))
        self.get_logger().info(
            f"fg={fg.shape[0]} clusters={len(clusters)} tracks={len(self.tracks)} "
            f"confirmed={sum(tr.confirmed for tr in self.tracks)}",
            throttle_duration_sec=2.0)
        self._publish_debug(msg.header.stamp, fg, clusters)

    def _cluster(self, fg):
        """グリッド連結成分クラスタリング → [(centroid, npts, span)]"""
        if fg.shape[0] == 0:
            return []
        cells = np.floor(fg / self.cell).astype(np.int32)
        cell_map = {}
        for i, c in enumerate(map(tuple, cells)):
            cell_map.setdefault(c, []).append(i)
        seen = set()
        out = []
        for start in cell_map:
            if start in seen:
                continue
            comp = []
            stack = [start]
            seen.add(start)
            while stack:
                c = stack.pop()
                comp.extend(cell_map[c])
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            n = (c[0] + dx, c[1] + dy, c[2] + dz)
                            if n in cell_map and n not in seen:
                                seen.add(n)
                                stack.append(n)
            p = fg[comp]
            if p.shape[0] < self.min_pts:
                continue
            span = float(np.max(np.ptp(p, axis=0)))
            if span > self.max_span:
                continue  # 人・ドア等の大きな前景を棄却
            out.append((p.mean(axis=0), p.shape[0], span))
        return out

    # ---- 追跡 -------------------------------------------------------------
    def _track_update(self, clusters, t):
        for tr in self.tracks:
            tr.predict(t)

        used = set()
        for tr in sorted(self.tracks, key=lambda x: -x.hits):
            best, best_d = None, tr.gate_radius()
            for i, (c, n, _) in enumerate(clusters):
                if i in used:
                    continue
                d = float(np.linalg.norm(c - tr.x[:3]))
                if d < best_d:
                    best, best_d = i, d
            if best is not None:
                used.add(best)
                tr.update(clusters[best][0])
                tr.hits += 1
                tr.misses = 0
                if not tr.confirmed and tr.hits >= self.confirm_hits:
                    tr.confirmed = True
                    self.get_logger().info(
                        f"track CONFIRMED at {tr.x[:3].round(2).tolist()}")
            else:
                tr.misses += 1
            tr.age += 1

        self.tracks = [tr for tr in self.tracks if tr.misses <= self.max_misses]
        for i, (c, n, _) in enumerate(clusters):
            if i not in used:
                self.tracks.append(Track(c, t, self.r_meas, self.sigma_a))

        best = max((tr for tr in self.tracks if tr.confirmed),
                   key=lambda x: x.hits, default=None)
        if best is not None:
            self._publish_target(best, t)

    def _publish_target(self, tr, t):
        od = Odometry()
        od.header.stamp.sec = int(t)
        od.header.stamp.nanosec = int((t % 1.0) * 1e9)
        od.header.frame_id = "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = tr.x[:3]
        od.pose.pose.orientation.w = 1.0
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = tr.x[3:]
        cov = np.zeros((6, 6))
        cov[:3, :3] = tr.P[:3, :3]
        od.pose.covariance = cov.flatten().tolist()
        tcov = np.zeros((6, 6))
        tcov[:3, :3] = tr.P[3:, 3:]
        od.twist.covariance = tcov.flatten().tolist()
        self.pub_target.publish(od)

    def _publish_debug(self, stamp, fg, clusters):
        if self.pub_fg.get_subscription_count() > 0 and fg.shape[0] > 0:
            h = Header()
            h.stamp = stamp
            h.frame_id = "base_link"
            self.pub_fg.publish(point_cloud2.create_cloud_xyz32(h, fg))
        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = "base_link"
        for c, n, _ in clusters:
            p = Pose()
            p.position.x, p.position.y, p.position.z = map(float, c)
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_clusters.publish(pa)


def main():
    rclpy.init()
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
