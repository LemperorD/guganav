#!/usr/bin/env python3
"""合并验证用监视器: 记录 /registered_scan 与 /lidar_odometry 的内容摘要。

用法: monitor_project.py <输出目录>

产出:
  lidar_odometry.csv  stamp_ns,frame_id,child_frame_id,x,y,z,qx,qy,qz,qw
  registered_scan.csv stamp_ns,frame_id,points,sum_x,sum_y,sum_z

点云按键名解析字段偏移, 因此不受 point_step 布局变化影响。
"""

import os
import sys
import threading

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def cloud_summary(msg):
    fields = {f.name: f.offset for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return 0, 0.0, 0.0, 0.0
    n = int(msg.width) * int(msg.height)
    if n == 0:
        return 0, 0.0, 0.0, 0.0
    step = int(msg.point_step)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)
    vals = []
    for axis in ("x", "y", "z"):
        off = fields[axis]
        col = buf[:, off:off + 4].copy().view(np.float32).reshape(-1).astype(np.float64)
        vals.append(col)
    return n, float(vals[0].sum()), float(vals[1].sum()), float(vals[2].sum())


class Monitor(Node):
    def __init__(self, out_dir):
        super().__init__("merge_verify_monitor")
        self._lock = threading.Lock()
        self._odom = open(os.path.join(out_dir, "lidar_odometry.csv"), "w", buffering=1)
        self._scan = open(os.path.join(out_dir, "registered_scan.csv"), "w", buffering=1)
        self._odom.write("stamp_ns,frame_id,child_frame_id,x,y,z,qx,qy,qz,qw\n")
        self._scan.write("stamp_ns,frame_id,points,sum_x,sum_y,sum_z\n")
        self.create_subscription(Odometry, "/lidar_odometry", self._on_odom, 10)
        self.create_subscription(PointCloud2, "/registered_scan", self._on_scan, 10)
        # 上游原始话题, 用于对照首帧是否为零云
        self._raw_odom = open(os.path.join(out_dir, "aft_mapped_to_init.csv"), "w", buffering=1)
        self._raw_scan = open(os.path.join(out_dir, "cloud_registered.csv"), "w", buffering=1)
        self._raw_odom.write("stamp_ns,frame_id,child_frame_id,x,y,z\n")
        self._raw_scan.write("stamp_ns,frame_id,points,sum_x,sum_y,sum_z\n")
        self.create_subscription(Odometry, "/aft_mapped_to_init", self._on_raw_odom, 10)
        self.create_subscription(PointCloud2, "/cloud_registered", self._on_raw_scan, 10)
        self.get_logger().info("merge 验证监视器已启动")

    @staticmethod
    def _ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self._lock:
            self._odom.write(
                f"{self._ns(msg.header.stamp)},{msg.header.frame_id},{msg.child_frame_id},"
                f"{p.x},{p.y},{p.z},{q.x},{q.y},{q.z},{q.w}\n")

    def _on_scan(self, msg):
        n, sx, sy, sz = cloud_summary(msg)
        with self._lock:
            self._scan.write(f"{self._ns(msg.header.stamp)},{msg.header.frame_id},"
                             f"{n},{sx:.6f},{sy:.6f},{sz:.6f}\n")

    def _on_raw_odom(self, msg):
        p = msg.pose.pose.position
        with self._lock:
            self._raw_odom.write(f"{self._ns(msg.header.stamp)},{msg.header.frame_id},"
                                 f"{msg.child_frame_id},{p.x},{p.y},{p.z}\n")

    def _on_raw_scan(self, msg):
        n, sx, sy, sz = cloud_summary(msg)
        with self._lock:
            self._raw_scan.write(f"{self._ns(msg.header.stamp)},{msg.header.frame_id},"
                                 f"{n},{sx:.6f},{sy:.6f},{sz:.6f}\n")

    def close(self):
        for f in (self._odom, self._scan, self._raw_odom, self._raw_scan):
            f.close()


def main():
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = Monitor(out_dir)
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
