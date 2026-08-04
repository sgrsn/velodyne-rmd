#!/usr/bin/env python3
"""追跡結果の可視化ノード — 目標軌跡Path + 現在位置マーカー + 状態テキスト。

`ros2 bag record -a` で記録した追跡セッションの再生に重ねて使う。
/perception/target (Odometry, base_link系) を購読し、rviz2で見やすい形で配信:
  trajectory_viz/path    — nav_msgs/Path   目標の軌跡(緑のライン表示用)
  trajectory_viz/markers — MarkerArray     現在位置の球 + 状態/速度テキスト

bag再生への追従:
  ros2 bag play <bag> --clock
  python3 trajectory_viz.py --ros-args -p use_sim_time:=true
再生をやり直すとスタンプが過去に飛ぶので、それを検知して軌跡をリセットする。
動画化は scripts/bag2video.sh がこのノードごと面倒を見る。
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

MAX_POSES = 20000  # 10Hzで30分強。X投稿用の数分なら十分


class TrajectoryViz(Node):
    def __init__(self):
        super().__init__("trajectory_viz")
        self.path = Path()
        self.path.header.frame_id = "base_link"
        self.state = ""
        self.last_t = None

        self.create_subscription(Odometry, "perception/target", self._on_target, 10)
        self.create_subscription(String, "tracker/state", self._on_state, 5)
        self.pub_path = self.create_publisher(Path, "trajectory_viz/path", 5)
        self.pub_markers = self.create_publisher(MarkerArray, "trajectory_viz/markers", 5)

    def _on_state(self, msg):
        self.state = msg.data

    def _on_target(self, od):
        t = od.header.stamp.sec + od.header.stamp.nanosec * 1e-9
        if self.last_t is not None and t < self.last_t - 1.0:
            self.get_logger().info("stamp jumped backwards — clearing trajectory (bag restart)")
            self.path.poses.clear()
        self.last_t = t

        ps = PoseStamped()
        ps.header = od.header
        ps.pose = od.pose.pose
        self.path.poses.append(ps)
        if len(self.path.poses) > MAX_POSES:
            del self.path.poses[: len(self.path.poses) - MAX_POSES]
        self.path.header.stamp = od.header.stamp
        self.pub_path.publish(self.path)

        p = od.pose.pose.position
        v = od.twist.twist.linear
        speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)

        sphere = Marker()
        sphere.header = od.header
        sphere.ns = "target"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.pose = od.pose.pose
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.45, 0.0, 0.9

        text = Marker()
        text.header = od.header
        text.ns = "target"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.x, text.pose.position.y = p.x, p.y
        text.pose.position.z = p.z + 0.7
        text.pose.orientation.w = 1.0
        text.scale.z = 0.35
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        label = f"{speed:.1f} m/s  d={dist:.1f} m"
        text.text = f"{self.state}  {label}" if self.state else label

        self.pub_markers.publish(MarkerArray(markers=[sphere, text]))


def main():
    rclpy.init()
    node = TrajectoryViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
