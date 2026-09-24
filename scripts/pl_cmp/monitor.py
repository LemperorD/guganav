#!/usr/bin/env python3
"""轻量监视器: 只记录话题的时间戳与规模, 不保存点云数据。

用法: monitor.py <输出目录>

产出:
  odom.csv    stamp_ns,x,y,z,qx,qy,qz,qw
  cloud.csv   stamp_ns,header_stamp_ns,points
  path.csv    stamp_ns,pose_count

重构版会以约 500 Hz 重复发布整幅点云, 用 rosbag2 录制会迅速占满磁盘,
因此这里改为逐条记录规模指标。
"""

import os
import sys
import threading

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class Monitor(Node):
    def __init__(self, out_dir, watch=("odom", "cloud", "path")):
        super().__init__("pl_cmp_monitor")
        self._watch = set(watch)
        self._lock = threading.Lock()
        self._odom = open(os.path.join(out_dir, "odom.csv"), "w", buffering=1)
        self._cloud = open(os.path.join(out_dir, "cloud.csv"), "w", buffering=1)
        self._path = open(os.path.join(out_dir, "path.csv"), "w", buffering=1)
        self._odom.write("stamp_ns,x,y,z,qx,qy,qz,qw\n")
        self._cloud.write("stamp_ns,header_stamp_ns,points\n")
        self._path.write("stamp_ns,pose_count\n")

        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        if "odom" in self._watch:
            self.create_subscription(Odometry, "/aft_mapped_to_init", self._on_odom, qos)
        if "cloud" in self._watch:
            self.create_subscription(PointCloud2, "/cloud_registered", self._on_cloud, qos)
        if "path" in self._watch:
            self.create_subscription(Path, "/path", self._on_path, qos)
        self.get_logger().info("monitor 已启动")

    @staticmethod
    def _ns(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _on_odom(self, msg):
        p = msg.pose.pose
        with self._lock:
            self._odom.write(
                f"{self._ns(msg)},{p.position.x},{p.position.y},{p.position.z},"
                f"{p.orientation.x},{p.orientation.y},{p.orientation.z},"
                f"{p.orientation.w}\n")

    def _on_cloud(self, msg):
        with self._lock:
            self._cloud.write(
                f"{self._ns(msg)},{self._ns(msg)},{int(msg.width) * int(msg.height)}\n")

    def _on_path(self, msg):
        with self._lock:
            self._path.write(f"{self._ns(msg)},{len(msg.poses)}\n")

    def close(self):
        for f in (self._odom, self._cloud, self._path):
            f.close()


def main():
    out_dir = sys.argv[1]
    watch = tuple(sys.argv[2].split("+")) if len(sys.argv) > 2 else ("odom", "cloud", "path")
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = Monitor(out_dir, watch)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
