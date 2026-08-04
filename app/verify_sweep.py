#!/usr/bin/env python3
"""掃引補償の定量検証。

/livox/lidar (または任意のPointCloud2) をヘッダ時刻のTFでbase_link系へ変換し、
チルト角ビンごとに床/天井の推定高さを集計する。TF補償が正しければ
静止構造物の高さはチルト角に依存しない(std が数cm以内)。
使い方: python3 verify_sweep.py [--topic /livox/lidar] [--duration 60]
"""
import argparse
import math
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64
import tf2_ros


class Verifier(Node):
    def __init__(self, topic, duration):
        super().__init__("sweep_verifier")
        self.buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30))
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.tilt_deg = None
        self.samples = defaultdict(list)  # angle_bin -> [(floor_z, ceil_z, n_points)]
        self.n_clouds = 0
        self.deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=duration)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float64, "tilt/angle_deg", self.on_angle, 10)
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos)

    def on_angle(self, msg):
        self.tilt_deg = msg.data

    def on_cloud(self, msg):
        if self.tilt_deg is None:
            return
        try:
            t = self.buf.lookup_transform("base_link", msg.header.frame_id,
                                          msg.header.stamp,
                                          rclpy.duration.Duration(seconds=0.2))
        except tf2_ros.TransformException:
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r > 0.3) & (r < 8.0)]
        if len(pts) < 100:
            return
        q = t.transform.rotation
        R = quat_to_mat(q.x, q.y, q.z, q.w)
        trans = np.array([t.transform.translation.x,
                          t.transform.translation.y,
                          t.transform.translation.z])
        world = pts @ R.T + trans
        z = world[:, 2]
        # 床/天井: zヒストグラムの下端・上端のピーク(2cm分解能)
        hist, edges = np.histogram(z, bins=np.arange(z.min(), z.max() + 0.02, 0.02))
        floor_z = ceil_z = None
        peak = hist.max() * 0.1
        idx = np.where(hist > max(peak, 50))[0]
        if len(idx):
            floor_z = edges[idx[0]] + 0.01
            ceil_z = edges[idx[-1]] + 0.01
        abin = 5 * round((self.tilt_deg or 0) / 5)  # ロータ角5°ビン
        self.samples[abin].append((floor_z, ceil_z, len(pts)))
        self.n_clouds += 1

    def done(self):
        return self.get_clock().now() > self.deadline


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/livox/lidar")
    ap.add_argument("--duration", type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = Verifier(args.topic, args.duration)
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.2)

    floors, ceils = [], []
    print(f"\nclouds processed: {node.n_clouds}")
    print("rotor_bin  n   floor_z[m]  ceil_z[m]")
    for abin in sorted(node.samples):
        rows = node.samples[abin]
        f = [r[0] for r in rows if r[0] is not None]
        c = [r[1] for r in rows if r[1] is not None]
        fm = np.median(f) if f else float("nan")
        cm = np.median(c) if c else float("nan")
        if f: floors.append(fm)
        if c: ceils.append(cm)
        print(f"{abin:+9d} {len(rows):3d}  {fm:+.3f}      {cm:+.3f}")
    if len(floors) >= 3:
        print(f"\nfloor spread across bins: std={np.std(floors)*100:.1f}cm "
              f"range={(max(floors)-min(floors))*100:.1f}cm")
    if len(ceils) >= 3:
        print(f"ceil  spread across bins: std={np.std(ceils)*100:.1f}cm "
              f"range={(max(ceils)-min(ceils))*100:.1f}cm")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
