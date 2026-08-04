#!/usr/bin/env python3
"""RMDチルトサーボノード — 速度制御(0xA2)による目標仰角追従。

追跡パイプラインの駆動層。tracking_controllerから目標仰角+仰角速度FFを受け、
100Hzの速度制御ループで追従する。安全ガード(可動域・過電流・通信断)を内蔵し、
tilt_node.pyと同じTF/トピック(base_link→tilt_link、tilt/angle_deg、joint_states)
を配信するので、verify_sweep.py等の既存ツールがそのまま使える。

前提(probe_speed.py 2026-08-04実測):
  - 0xA2は本ファームで動作。指令・応答ともロータ角単位(0.01dps/LSB)
  - ステップ応答 t90≈20ms、±31dps反転 t90≈90ms
  - 0xA2応答(電流/速度/単回転エンコーダ)+0x92を毎tickでも100Hz成立

指令トピック tilt/cmd (std_msgs/Float64MultiArray):
  data[0] = 目標ピッチ角 [rad] (出力軸、REP-103: 上向き正)
  data[1] = 目標ピッチ角速度FF [rad/s] (省略可)
更新間はFFで目標を外挿する(10Hz知覚→100Hz駆動の補間)。cmd_timeout_s以上
指令が来なければその場で保持。

角度推定: 0xA2応答の単回転エンコーダ差分を積算(1 tickあたり最大でも
数百カウント≪32768なので巻き込み判定は単純)。1秒ごとに0x92でドリフト補正。
"""
import math
import threading

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from rmd_can import RmdComError, RmdMotor

AXES = {
    "x": (1.0, 0.0, 0.0),
    "y": (0.0, 1.0, 0.0),
    "z": (0.0, 0.0, 1.0),
}


class TiltServo(Node):
    def __init__(self):
        super().__init__("tilt_servo")
        self.declare_parameter("limits_file", "/workspaces/velodyne-rmd/config/limits.yaml")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("kp", 4.0)               # [1/s] 位置誤差→速度
        self.declare_parameter("max_speed_dps", 250.0)  # ロータ角速度上限(出力軸≈40dps)
        self.declare_parameter("wall_gain", 5.0)        # [1/s] 可動域端の減速包絡
        self.declare_parameter("current_limit_a", 4.0)
        self.declare_parameter("cmd_timeout_s", 0.3)
        self.declare_parameter("iface", "can0")
        self.declare_parameter("motor_id", 1)

        with open(self.get_parameter("limits_file").value) as f:
            lim = yaml.safe_load(f)
        self.level_encoder = int(lim["level_encoder"])
        max_offset = float(lim.get("max_level_offset_deg", 120.0))
        self.gear = float(lim.get("gear_ratio", 6.2))
        self.axis = AXES[str(lim.get("axis", "y")).lower()]
        self.sign = float(lim.get("sign", 1.0))
        self.mounts = lim.get("mounts", {"velodyne": {}})
        margin = float(lim.get("margin_deg", 2.0))

        self.kp = float(self.get_parameter("kp").value)
        self.vmax = float(self.get_parameter("max_speed_dps").value)
        self.wall_gain = float(self.get_parameter("wall_gain").value)
        self.i_limit = float(self.get_parameter("current_limit_a").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)
        self.rate = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / self.rate

        self.motor = RmdMotor(self.get_parameter("iface").value,
                              int(self.get_parameter("motor_id").value))
        self.lock = threading.Lock()

        with self.lock:
            self.motor.enable()
            st = self.motor.status()
            self.level, delta = self.motor.level_multiturn_from_encoder(self.level_encoder)
            self.pos = self.motor.angle_deg()   # ロータ多回転角の推定値
        if st["error"]:
            raise RuntimeError(f"motor error flag 0x{st['error']:04X} at startup")
        if abs(delta) > max_offset:
            raise RuntimeError(
                f"current position is {delta:+.1f} rotor-deg from calibrated level "
                f"(limit {max_offset}). センサをだいたい水平にしてから起動するか、再較正してください")

        self.lower = self.level + float(lim["lower_rel_deg"])
        self.upper = self.level + float(lim["upper_rel_deg"])
        self.sw_lower = self.lower + margin
        self.sw_upper = self.upper - margin
        self.prev_enc = st["encoder"]

        # 指令状態(subscriptionとtimerは同一executorなので排他不要)
        self.target = self.pos          # ロータ角の目標(FFで毎tick外挿)
        self.ff_dps = 0.0               # ロータ角速度FF
        self.last_cmd_time = None       # 未受信=起動位置保持
        self.faulted = False
        self._i_over = 0                # 過電流連続カウント
        self._com_fail = 0              # 0xA2無応答連続カウント
        self._tick_count = 0

        self.tf_b = TransformBroadcaster(self)
        self.static_b = StaticTransformBroadcaster(self)
        self.pub_angle = self.create_publisher(Float64, "tilt/angle_deg", 10)
        self.pub_joint = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(Float64MultiArray, "tilt/cmd", self._on_cmd, 10)

        self._publish_static_mount()
        self.get_logger().info(
            f"servo ready: {st['temp_c']}C, level {self.level:+.2f}, "
            f"window [{self.sw_lower:+.1f}, {self.sw_upper:+.1f}] rotor-deg, "
            f"kp={self.kp}/s vmax={self.vmax}dps @ {self.rate:.0f}Hz")

        self.create_timer(self.dt, self._tick)

    # ---- 指令受信 ---------------------------------------------------------
    def _on_cmd(self, msg):
        if not msg.data:
            return
        cmd_rad = float(msg.data[0])
        cmd_vel = float(msg.data[1]) if len(msg.data) > 1 else 0.0
        # 出力軸REP-103 → ロータ角 (tilt_node._tf_tickの逆変換)
        self.target = self.level + self.sign * self.gear * math.degrees(cmd_rad)
        self.ff_dps = self.sign * self.gear * math.degrees(cmd_vel)
        self.last_cmd_time = self.get_clock().now()

    # ---- メインループ(全CANアクセスをここに集約) -------------------------
    def _tick(self):
        self._tick_count += 1
        if self.faulted:
            self._tf_only_tick()
            return

        # 指令の鮮度: タイムアウトでその場保持へ移行
        if self.last_cmd_time is not None:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                self.target = self.pos
                self.ff_dps = 0.0
                self.last_cmd_time = None
                self.get_logger().warn("cmd timeout — holding position",
                                       throttle_duration_sec=5.0)

        # FFによる目標外挿(知覚更新の合間を補間)
        if self.ff_dps:
            self.target += self.ff_dps * self.dt
        target = min(max(self.target, self.sw_lower), self.sw_upper)

        # 速度指令 = P項 + FF、上限と可動域端の減速包絡でクランプ
        v = self.kp * (target - self.pos) + self.ff_dps
        v = min(max(v, -self.vmax), self.vmax)
        v = min(v, self.wall_gain * (self.sw_upper - self.pos))
        v = max(v, -self.wall_gain * (self.pos - self.sw_lower))

        with self.lock:
            reply = self.motor.command_speed(v)

        if reply is None:
            self._com_fail += 1
            if self._com_fail >= 5:
                self._trip("0xA2 no reply x5 — communication lost")
            return
        self._com_fail = 0

        # エンコーダ差分で角度推定を更新(単回転65536/rev、ロータ)
        d = (reply["encoder"] - self.prev_enc) % 65536
        if d > 32768:
            d -= 65536
        self.prev_enc = reply["encoder"]
        self.pos += d * 360.0 / 65536.0

        # ガード: ハード可動域逸脱・過電流(5tick連続)
        if not (self.lower <= self.pos <= self.upper):
            self._trip(f"hard limit exceeded at {self.pos:+.2f} rotor-deg")
            return
        if abs(reply["current_a"]) > self.i_limit:
            self._i_over += 1
            if self._i_over >= 5:
                self._trip(f"over-current {reply['current_a']:.2f} A sustained")
                return
        else:
            self._i_over = 0

        # 1秒ごと: 0x92でドリフト補正、0x9Aでエラーフラグ確認
        if self._tick_count % int(self.rate) == 0:
            with self.lock:
                try:
                    true_pos = self.motor.angle_deg()
                    st = self.motor.status()
                except RmdComError:
                    st = None
                    true_pos = None
            if true_pos is not None:
                drift = true_pos - self.pos
                if abs(drift) > 0.5:
                    self.get_logger().warn(f"encoder drift {drift:+.2f} deg — corrected")
                self.pos = true_pos
            if st and st["error"]:
                self._trip(f"error flag 0x{st['error']:04X}")
                return

        self._publish_state(reply)

    def _trip(self, reason):
        with self.lock:
            try:
                self.motor.command_speed(0.0)
                self.motor.stop()
            except Exception:
                pass
        self.faulted = True
        self.get_logger().error(f"GUARD TRIP: {reason} — stopped (holding). servo disabled.")

    def _tf_only_tick(self):
        """フォールト後もTF配信は続ける(知覚側の座標変換を止めない)。"""
        if self._tick_count % 10 != 0:   # 10Hzに落とす
            return
        with self.lock:
            try:
                self.pos = self.motor.angle_deg()
            except RmdComError:
                return
        self._publish_state(None)

    # ---- 配信(tilt_node.pyと同一のフレーム構成) --------------------------
    def _publish_static_mount(self):
        transforms = []
        for frame, ofs in self.mounts.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "tilt_link"
            t.child_frame_id = frame
            t.transform.translation.x = float(ofs.get("x", 0.0))
            t.transform.translation.y = float(ofs.get("y", 0.0))
            t.transform.translation.z = float(ofs.get("z", 0.0))
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self.static_b.sendTransform(transforms)

    def _publish_state(self, reply):
        rad = math.radians(self.sign * (self.pos - self.level) / self.gear)
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

        self.pub_angle.publish(Float64(data=self.pos))
        js = JointState()
        js.header.stamp = t.header.stamp
        js.name = ["tilt_joint"]
        js.position = [rad]
        if reply is not None:
            js.velocity = [math.radians(self.sign * reply["speed_dps"] / self.gear)]
            js.effort = [reply["current_a"]]
        self.pub_joint.publish(js)

    def shutdown(self):
        with self.lock:
            try:
                self.motor.command_speed(0.0)
                self.motor.stop()
            except Exception:
                pass
            self.motor.close()


def main():
    rclpy.init()
    node = TiltServo()
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
