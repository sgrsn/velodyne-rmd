#!/usr/bin/env python3
"""tracking_controllerの状態機械・look-at変換のロジックテスト(ハードウェア不要)。

同一プロセスにコントローラと試験ノードを載せ、シナリオを駆動する:
  1. 起動直後: MAP状態で掃引指令が出ている
  2. perception/mode=RUN → SEARCH遷移
  3. perception/target配信(x=5,z=2,vz=1) → TRACK遷移、look-at角の検証
  4. 配信停止 → LOST遷移 → lost_max_s後にSEARCH復帰

他インスタンスと干渉しないよう ROS_DOMAIN_ID を変えて実行すること:
  ROS_DOMAIN_ID=42 python3 test_controller_logic.py
出力は1行JSON。
"""
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from tracking_controller import TrackingController


class TestDriver(Node):
    def __init__(self):
        super().__init__("ctrl_test")
        self.pub_mode = self.create_publisher(String, "perception/mode", 5)
        self.pub_target = self.create_publisher(Odometry, "perception/target", 10)
        self.state = None
        self.cmds = []
        self.create_subscription(String, "tracker/state", self._on_state, 5)
        self.create_subscription(Float64MultiArray, "tilt/cmd", self._on_cmd, 10)

    def _on_state(self, msg):
        self.state = msg.data

    def _on_cmd(self, msg):
        self.cmds.append(tuple(msg.data))

    def send_target(self, x, y, z, vx=0.0, vy=0.0, vz=0.0):
        od = Odometry()
        od.header.frame_id = "base_link"
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.position.z = z
        od.twist.twist.linear.x = vx
        od.twist.twist.linear.y = vy
        od.twist.twist.linear.z = vz
        self.pub_target.publish(od)


def spin_for(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.02)


def main():
    rclpy.init()
    # テスト短縮のためlost系パラメータを小さく
    import rclpy.parameter as rp
    ctrl = TrackingController()
    ctrl.lost_timeout = 0.4
    ctrl.lost_max = 1.0
    drv = TestDriver()
    ex = SingleThreadedExecutor()
    ex.add_node(ctrl)
    ex.add_node(drv)
    results = {}

    # 1) MAP状態で掃引指令が出る
    spin_for(ex, 1.0)
    results["map_state"] = drv.state
    results["map_cmds_flowing"] = len(drv.cmds) > 10
    sweep_moving = len({round(c[0], 4) for c in drv.cmds[-10:]}) > 1
    results["map_sweeping"] = sweep_moving

    # 2) RUN通知 → SEARCH
    drv.pub_mode.publish(String(data="RUN"))
    spin_for(ex, 0.5)
    results["after_run_state"] = drv.state

    # 3) 目標配信 → TRACK、look-at検証 (x=5,z=2: pitch=-atan2(2,5)=-0.3805)
    drv.cmds.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        drv.send_target(5.0, 0.0, 2.0, vz=1.0)
        spin_for(ex, 0.1)
    results["track_state"] = drv.state
    if drv.cmds:
        pitch, rate = drv.cmds[-1]
        # 1秒間でzが+1進む外挿込み: -atan2(2〜3, 5) の範囲
        results["track_pitch_deg"] = round(math.degrees(pitch), 2)
        results["track_rate_dps"] = round(math.degrees(rate), 2)
        results["track_pitch_ok"] = -32.0 < math.degrees(pitch) < -21.0
        results["track_rate_ok"] = -12.0 < math.degrees(rate) < -6.0

    # 4) 配信停止 → LOST → SEARCH
    spin_for(ex, 0.7)
    results["lost_state"] = drv.state
    spin_for(ex, 1.2)
    results["recovered_state"] = drv.state

    ok = (results.get("map_state") == "MAP" and results.get("map_sweeping")
          and results.get("after_run_state") == "SEARCH"
          and results.get("track_state") == "TRACK"
          and results.get("track_pitch_ok") and results.get("track_rate_ok")
          and results.get("lost_state") == "LOST"
          and results.get("recovered_state") == "SEARCH")
    print(json.dumps({"ok": ok, **results}))
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
