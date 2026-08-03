#!/usr/bin/env python3
"""RMDチルト機構ノード。

モータを上限〜下限の間で往復掃引しつつ、実測角度からTFを配信する:

  base_link --(動的: モータ角度による回転)--> tilt_link --(静的: 取付オフセット)--> velodyne

velodyneドライバの点群(frame_id: velodyne)はこのTFでbase_link系に正しく変換され、
rviz2(Fixed Frame: base_link)でモータが回転しても静止環境が静止して見える。

limits.yaml(ジョグ手順で生成)から上限/下限/水平角度を読む。
パラメータ:
  limits_file  : limits.yamlのパス
  sweep        : true=往復掃引 / false=保持のみ(TF配信は継続)
  speed_dps    : 掃引速度
  current_limit_a, margin_deg, tf_rate_hz
"""
import math
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from rmd_can import RmdComError, RmdMotor

AXES = {
    "x": (1.0, 0.0, 0.0),
    "y": (0.0, 1.0, 0.0),
    "z": (0.0, 0.0, 1.0),
}


class TiltNode(Node):
    def __init__(self):
        super().__init__("rmd_tilt")
        self.declare_parameter("limits_file", "/workspaces/velodyne-rmd/config/limits.yaml")
        self.declare_parameter("sweep", True)
        self.declare_parameter("speed_dps", 20.0)
        self.declare_parameter("current_limit_a", 4.0)
        self.declare_parameter("margin_deg", 2.0)
        self.declare_parameter("tf_rate_hz", 50.0)
        self.declare_parameter("iface", "can0")
        self.declare_parameter("motor_id", 1)

        limits_file = self.get_parameter("limits_file").value
        with open(limits_file) as f:
            lim = yaml.safe_load(f)
        # 限界は水平からの相対角(ロータ角)で保持。水平の絶対基準は単回転
        # エンコーダ値から毎起動時に復元する(電源断で多回転基準が失われるため)。
        self.level_encoder = int(lim["level_encoder"])
        upper_rel = float(lim["upper_rel_deg"])
        lower_rel = float(lim["lower_rel_deg"])
        max_offset = float(lim.get("max_level_offset_deg", 120.0))
        # 0x92/0xA4は減速機入力側(ロータ)の角度。出力軸角 = ロータ角/gear_ratio
        self.gear = float(lim.get("gear_ratio", 6.2))
        self.axis = AXES[str(lim.get("axis", "y")).lower()]
        self.sign = float(lim.get("sign", 1.0))
        # マージンはlimits.yaml優先(較正とセットで管理)、無ければノードパラメータ
        margin = float(lim.get("margin_deg", self.get_parameter("margin_deg").value))

        self.speed = float(self.get_parameter("speed_dps").value)
        self.i_limit = float(self.get_parameter("current_limit_a").value)
        self.sweep = bool(self.get_parameter("sweep").value)

        self.motor = RmdMotor(self.get_parameter("iface").value,
                              int(self.get_parameter("motor_id").value))
        self.lock = threading.Lock()  # CANソケットはタイマ間で共有

        with self.lock:
            self.motor.enable()
            st = self.motor.status()
            self.level, delta = self.motor.level_multiturn_from_encoder(self.level_encoder)
        if st["error"]:
            raise RuntimeError(f"motor error flag 0x{st['error']:04X} at startup")
        if abs(delta) > max_offset:
            raise RuntimeError(
                f"current position is {delta:+.1f} rotor-deg from calibrated level "
                f"(limit {max_offset}). センサをだいたい水平にしてから起動するか、再較正してください")

        self.lower = self.level + lower_rel
        self.upper = self.level + upper_rel
        self.sw_lower = self.lower + margin
        self.sw_upper = self.upper - margin
        if self.sw_lower >= self.sw_upper:
            raise RuntimeError(f"limits too narrow: [{self.lower}, {self.upper}] margin {margin}")

        self.tf_b = TransformBroadcaster(self)
        self.static_b = StaticTransformBroadcaster(self)
        self.pub_angle = self.create_publisher(Float64, "tilt/angle_deg", 10)
        self.pub_joint = self.create_publisher(JointState, "joint_states", 10)

        self._publish_static_mount()

        self.get_logger().info(
            f"motor ok: {st['temp_c']}C, level recovered at {self.level:+.2f} "
            f"(current offset {delta:+.2f}), sweep "
            f"[{self.sw_lower:+.1f}, {self.sw_upper:+.1f}] @ {self.speed} dps")

        self.target = self.sw_upper
        self.faulted = False
        if self.sweep:
            with self.lock:
                self.motor.command_position(self.target, self.speed)

        # CANアクセスは単一タイマに集約(複数タイマの競合で読み値が遅延するため)。
        # 毎tickで角度読み+TF配信、_GUARD_DIVIDE回に1回だけ状態確認と折り返し判定。
        self._GUARD_DIVIDE = 5
        self._tick_count = 0
        rate = float(self.get_parameter("tf_rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        self._tf_tick()
        self._tick_count += 1
        if self._tick_count % self._GUARD_DIVIDE == 0:
            self._control_tick()

    def _publish_static_mount(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "tilt_link"
        t.child_frame_id = "velodyne"
        t.transform.rotation.w = 1.0
        self.static_b.sendTransform(t)

    def _tf_tick(self):
        with self.lock:
            try:
                pos = self.motor.angle_deg()
            except RmdComError:
                self.get_logger().warn("angle read failed", throttle_duration_sec=1.0)
                return
        rad = math.radians(self.sign * (pos - self.level) / self.gear)
        ax, ay, az = self.axis
        s = math.sin(rad / 2.0)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "tilt_link"
        t.transform.rotation.x = ax * s
        t.transform.rotation.y = ay * s
        t.transform.rotation.z = az * s
        t.transform.rotation.w = math.cos(rad / 2.0)
        self.tf_b.sendTransform(t)

        self.pub_angle.publish(Float64(data=pos))
        js = JointState()
        js.header.stamp = t.header.stamp
        js.name = ["tilt_joint"]
        js.position = [rad]
        self.pub_joint.publish(js)
        self._last_pos = pos

    def _control_tick(self):
        if self.faulted or not self.sweep:
            return
        with self.lock:
            try:
                st = self.motor.status()
                pos = self.motor.angle_deg()
            except RmdComError:
                self.get_logger().warn("status read failed", throttle_duration_sec=1.0)
                return
            if st["error"] or abs(st["current_a"]) > self.i_limit:
                self.motor.stop()
                self.faulted = True
                self.get_logger().error(
                    f"GUARD TRIP: error=0x{st['error']:04X} current={st['current_a']:.2f}A "
                    f"at {pos:+.2f} deg — stopped (holding). sweep disabled.")
                return
            if abs(pos - self.target) < 2.0:
                self.target = self.sw_lower if self.target == self.sw_upper else self.sw_upper
                self.motor.command_position(self.target, self.speed)
                self.get_logger().info(f"sweep toggle -> {self.target:+.1f} deg (rotor)")

    def shutdown(self):
        with self.lock:
            try:
                self.motor.stop()
            except Exception:
                pass
            self.motor.close()


def main():
    rclpy.init()
    node = TiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("stopping motor (holding position)")
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
