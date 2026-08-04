#!/usr/bin/env python3
"""tilt_servoの追従帯域実測 — ステップ応答と正弦波追従誤差。

tilt_servo.py起動中に実行する。tilt/cmdへプロファイルを50Hzで配信し、
joint_states(出力軸rad)で実測角を記録して評価する。

プロファイル(出力軸角、水平=0基準):
  0-2s   : 0°へ整定
  2-4s   : +10°ステップ → t90/オーバーシュート測定
  4-6s   : 0°へ戻し
  6-18s  : 正弦波 A=15° f=0.25Hz(ピーク角速度23.6°/s = 想定最大要求)+速度FF
  18-19s : 0°へ戻して終了
出力は1行JSON。
"""
import json
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

STEP_DEG = 10.0
SINE_A_DEG = 15.0
SINE_F_HZ = 0.25
T_END = 19.0


class TrackingTest(Node):
    def __init__(self):
        super().__init__("servo_tracking_test")
        self.pub = self.create_publisher(Float64MultiArray, "tilt/cmd", 10)
        self.create_subscription(JointState, "joint_states", self._on_joint, 50)
        self.samples = []      # (t, cmd_rad, meas_rad)
        self.t0 = None
        self.cmd = (0.0, 0.0)
        self.done = False
        self.create_timer(0.02, self._tick)  # 50Hz指令

    def _profile(self, t):
        """(target_rad, ff_rad_s)"""
        a = math.radians(SINE_A_DEG)
        w = 2.0 * math.pi * SINE_F_HZ
        if t < 2.0:
            return 0.0, 0.0
        if t < 4.0:
            return math.radians(STEP_DEG), 0.0
        if t < 6.0:
            return 0.0, 0.0
        if t < 18.0:
            ts = t - 6.0
            return a * math.sin(w * ts), a * w * math.cos(w * ts)
        return 0.0, 0.0

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        if t > T_END:
            self.done = True
            return
        tgt, ff = self._profile(t)
        self.cmd = (tgt, ff)
        m = Float64MultiArray()
        m.data = [tgt, ff]
        self.pub.publish(m)

    def _on_joint(self, msg):
        if self.t0 is None or not msg.position:
            return
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        self.samples.append((t, self.cmd[0], msg.position[0]))

    # ---- 評価 -------------------------------------------------------------
    def report(self):
        deg = math.degrees
        step = [(t, m) for t, _, m in self.samples if 2.0 <= t < 4.0]
        base = next((m for t, _, m in self.samples if t >= 1.8), 0.0)
        tgt = math.radians(STEP_DEG)
        t90 = next((t - 2.0 for t, m in step
                    if (m - base) >= 0.9 * (tgt - base)), None)
        tail = [m for t, m in step if t >= 3.0]
        overshoot = deg(max(m for _, m in step) - tgt) if step else None
        settled_err = deg(sum(tail) / len(tail) - tgt) if tail else None

        sine = [(c, m) for t, c, m in self.samples if 7.0 <= t < 18.0]
        errs = [deg(c - m) for c, m in sine]
        rms = math.sqrt(sum(e * e for e in errs) / len(errs)) if errs else None
        emax = max(abs(e) for e in errs) if errs else None

        return {
            "ok": True,
            "n_samples": len(self.samples),
            "step": {"t90_ms": round(1000 * t90, 1) if t90 is not None else None,
                     "overshoot_deg": round(overshoot, 3) if overshoot is not None else None,
                     "settled_err_deg": round(settled_err, 3) if settled_err is not None else None},
            "sine_23dps": {"rms_err_deg": round(rms, 3) if rms is not None else None,
                           "max_err_deg": round(emax, 3) if emax is not None else None},
        }


def main():
    rclpy.init()
    node = TrackingTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.samples:
            print(json.dumps({"ok": False, "error": "no joint_states received — is tilt_servo running?"}))
            return 1
        print(json.dumps(node.report()))
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
