#!/usr/bin/env python3
"""直接从 rosbag2 的 sqlite 库里用 SQL 取里程计与各话题计数, 避免读取巨大的点云 blob。"""

import os
import sqlite3
import sys

import numpy as np
import rclpy.serialization
from nav_msgs.msg import Odometry

PL_DIR = os.environ.get("PL_CMP_DIR", os.path.join(WS, ".pl_cmp_bench"))


def main():
    label = sys.argv[1]
    db = os.path.join(PL_DIR, "out", label, "bag", "bag_0.db3")
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    cur = con.cursor()
    print(f"===== {label} ({db}) =====")
    for name, n in cur.execute(
            "select t.name, count(m.id) from topics t "
            "left join messages m on m.topic_id = t.id group by t.name"):
        print(f"  {name}: {n} 条")

    rows = cur.execute(
        "select m.timestamp, m.data from messages m join topics t on t.id = m.topic_id "
        "where t.name = '/aft_mapped_to_init' order by m.timestamp").fetchall()
    out = os.path.join(PL_DIR, "out", label, "odom.csv")
    with open(out, "w") as f:
        f.write("stamp_ns,header_stamp_ns,x,y,z,qx,qy,qz,qw\n")
        for stamp, data in rows:
            msg = rclpy.serialization.deserialize_message(data, Odometry)
            p = msg.pose.pose
            hdr = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            f.write(f"{stamp},{hdr},{p.position.x},{p.position.y},{p.position.z},"
                    f"{p.orientation.x},{p.orientation.y},{p.orientation.z},"
                    f"{p.orientation.w}\n")
    print(f"  里程计已导出: {out} ({len(rows)} 条)")

    # 点云与 path 的头部时间统计: 只取头部那几十字节不足以判断, 这里改为统计
    # 消息总时长与条数, 用于判断发布频率
    for topic in ("/cloud_registered", "/path"):
        r = cur.execute(
            "select min(m.timestamp), max(m.timestamp) from messages m "
            "join topics t on t.id = m.topic_id where t.name = ?", (topic,)).fetchone()
        if r and r[0]:
            print(f"  {topic}: 录制时间跨度 {(r[1] - r[0]) / 1e9:.2f} s")
    con.close()


if __name__ == "__main__":
    main()
