#!/usr/bin/env python3
"""追跡コントローラ — 状態機械とlook-at変換。

perception/target(Odometry, base_link系)を受け、チルト目標仰角+仰角速度FFを
tilt/cmd へ配信する。状態機械:

  MAP    — perception/mode が MAP の間、全可動域をゆっくり掃引(背景取得用)
  SEARCH — 中速の三角波掃引で検出機会を最大化
  TRACK  — 確定目標をlook-at追従: pitch = -atan2(z, √(x²+y²)) + 速度FF
  LOST   — 最終仰角まわりを拡大掃引、lost_max_s超過でSEARCHへ

チルト指令の規約(tilt_servo.py):
  data[0] = ピッチ角 [rad] REP-103(+Yまわり、正=下向き)
  data[1] = ピッチ角速度FF [rad/s]
20Hzのタイマで駆動。TRACK中は目標状態を等速外挿して毎tick再計算する。
"""
import math

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


class TrackingController(Node):
    def __init__(self):
        super().__init__("tracking_controller")
        self.declare_parameter("limits_file", "/workspaces/velodyne-rmd/config/limits.yaml")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("map_speed_dps", 4.0)      # 出力軸、背景取得掃引
        self.declare_parameter("search_speed_dps", 15.0)  # 出力軸、探索掃引
        self.declare_parameter("lost_timeout_s", 0.8)     # 目標更新途絶→LOST
        self.declare_parameter("lost_max_s", 5.0)         # LOST継続→SEARCH
        self.declare_parameter("lost_amp0_deg", 3.0)      # LOST掃引の初期振幅
        self.declare_parameter("lost_amp_growth_dps", 4.0)

        with open(self.get_parameter("limits_file").value) as f:
            lim = yaml.safe_load(f)
        gear = float(lim.get("gear_ratio", 6.2))
        sign = float(lim.get("sign", 1.0))
        margin = float(lim.get("margin_deg", 2.0))
        a = sign * math.radians(float(lim["upper_rel_deg"]) - margin) / gear
        b = sign * math.radians(float(lim["lower_rel_deg"]) + margin) / gear
        self.pitch_lo, self.pitch_hi = min(a, b), max(a, b)

        self.rate = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / self.rate
        self.map_v = math.radians(self.get_parameter("map_speed_dps").value)
        self.search_v = math.radians(self.get_parameter("search_speed_dps").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout_s").value)
        self.lost_max = float(self.get_parameter("lost_max_s").value)
        self.lost_amp0 = math.radians(self.get_parameter("lost_amp0_deg").value)
        self.lost_growth = math.radians(self.get_parameter("lost_amp_growth_dps").value)

        self.state = "MAP"
        self.perception_mode = "MAP"
        self.sweep_pitch = 0.0
        self.sweep_dir = 1.0
        self.target = None          # (t, pos(3), vel(3))
        self.lost_since = None
        self.lost_center = 0.0

        self.pub_cmd = self.create_publisher(Float64MultiArray, "tilt/cmd", 10)
        self.pub_state = self.create_publisher(String, "tracker/state", 5)
        self.create_subscription(Odometry, "perception/target", self._on_target, 10)
        self.create_subscription(String, "perception/mode", self._on_mode, 5)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"pitch range [{math.degrees(self.pitch_lo):+.1f}, "
            f"{math.degrees(self.pitch_hi):+.1f}] deg — MAP state")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_mode(self, msg):
        self.perception_mode = msg.data

    def _on_target(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.target = (self._now(), (p.x, p.y, p.z), (v.x, v.y, v.z))
        if self.state in ("SEARCH", "LOST"):
            self._set_state("TRACK")

    def _set_state(self, s):
        if s != self.state:
            self.get_logger().info(f"{self.state} -> {s}")
            self.state = s
            if s == "LOST":
                self.lost_since = self._now()

    # ---- look-at ----------------------------------------------------------
    @staticmethod
    def look_at(pos, vel):
        """目標位置・速度 → (pitch, pitch_rate)。REP-103: 正=下向き。"""
        x, y, z = pos
        vx, vy, vz = vel
        rxy = math.hypot(x, y)
        pitch = -math.atan2(z, max(rxy, 1e-6))
        d = rxy * rxy + z * z
        if d < 1e-9 or rxy < 1e-6:
            return pitch, 0.0
        drxy = (x * vx + y * vy) / rxy
        pitch_rate = -(vz * rxy - z * drxy) / d
        return pitch, pitch_rate

    # ---- 掃引生成 ----------------------------------------------------------
    def _sweep(self, speed, lo=None, hi=None):
        lo = self.pitch_lo if lo is None else lo
        hi = self.pitch_hi if hi is None else hi
        self.sweep_pitch += self.sweep_dir * speed * self.dt
        if self.sweep_pitch >= hi:
            self.sweep_pitch = hi
            self.sweep_dir = -1.0
        elif self.sweep_pitch <= lo:
            self.sweep_pitch = lo
            self.sweep_dir = 1.0
        return self.sweep_pitch, self.sweep_dir * speed

    # ---- メインループ ------------------------------------------------------
    def _tick(self):
        now = self._now()

        if self.state == "MAP":
            if self.perception_mode == "RUN":
                self._set_state("SEARCH")
            pitch, rate = self._sweep(self.map_v)
        elif self.state == "SEARCH":
            pitch, rate = self._sweep(self.search_v)
        elif self.state == "TRACK":
            t0, pos, vel = self.target
            age = now - t0
            if age > self.lost_timeout:
                p, _ = self.look_at(pos, vel)
                self.lost_center = p
                self.sweep_pitch = p
                self._set_state("LOST")
                return
            # 等速外挿してlook-at
            pos = tuple(p + v * age for p, v in zip(pos, vel))
            pitch, rate = self.look_at(pos, vel)
        else:  # LOST
            dur = now - self.lost_since
            if dur > self.lost_max:
                self._set_state("SEARCH")
                return
            amp = self.lost_amp0 + self.lost_growth * dur
            lo = max(self.lost_center - amp, self.pitch_lo)
            hi = min(self.lost_center + amp, self.pitch_hi)
            pitch, rate = self._sweep(self.search_v, lo, hi)

        pitch = min(max(pitch, self.pitch_lo), self.pitch_hi)
        m = Float64MultiArray()
        m.data = [pitch, rate]
        self.pub_cmd.publish(m)
        self.pub_state.publish(String(data=self.state))


def main():
    rclpy.init()
    node = TrackingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
