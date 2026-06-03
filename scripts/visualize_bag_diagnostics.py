#!/usr/bin/env python3
import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@dataclass
class Series3:
    t: List[float]
    x: List[float]
    y: List[float]
    z: List[float]

    def append(self, t: float, x: float, y: float, z: float) -> None:
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)


@dataclass
class ScalarSeries:
    t: List[float]
    v: List[float]

    def append(self, t: float, v: float) -> None:
        self.t.append(t)
        self.v.append(v)


def nearest_index(sorted_t: List[float], target: float) -> Optional[int]:
    if not sorted_t:
        return None
    i = bisect.bisect_left(sorted_t, target)
    if i == 0:
        return 0
    if i >= len(sorted_t):
        return len(sorted_t) - 1
    left = i - 1
    if abs(sorted_t[i] - target) < abs(sorted_t[left] - target):
        return i
    return left


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Visualize rosbag diagnostics for collision/speed analysis")
    p.add_argument("bag", help="Path to rosbag file or rosbag dir (mcap)")
    p.add_argument("--storage", default="mcap", help="Storage id, default mcap")
    p.add_argument("--start-offset", type=float, default=0.0, help="Start time offset from bag start (s)")
    p.add_argument("--end-offset", type=float, default=None, help="End time offset from bag start (s)")
    p.add_argument("--output-prefix", default="bag_diag", help="Output prefix for png/csv")
    p.add_argument("--max-costmap-points", type=int, default=2000, help="Max local costmap samples to decode")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    bag_path = str(Path(args.bag).expanduser().resolve())

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=args.storage),
        rosbag2_py.ConverterOptions("", ""),
    )

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}

    wanted = {
        "/cmd_vel_nav2_result",
        "/cmd_vel_controller",
        "/odometry",
        "/serial/TES_speed",
        "/serial/Yaw",
        "/chassis_mode",
        "/rosout",
        "/local_costmap/costmap_raw",
        "/referee/game_status",
    }

    type_map: Dict[str, object] = {}
    for name in wanted:
        if name in topics:
            type_map[name] = get_message(topics[name])

    cmd_nav = Series3([], [], [], [])
    cmd_ctrl = Series3([], [], [], [])
    odom = Series3([], [], [], [])
    tes = Series3([], [], [], [])
    serial_yaw = ScalarSeries([], [])
    chassis_mode = ScalarSeries([], [])
    coll_ts: List[float] = []
    cost_occupied = ScalarSeries([], [])
    game_status = ScalarSeries([], [])

    bag_start_ns: Optional[int] = None
    cost_samples = 0

    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        if bag_start_ns is None:
            bag_start_ns = ts_ns
        rel_t = (ts_ns - bag_start_ns) / 1e9

        if rel_t < args.start_offset:
            continue
        if args.end_offset is not None and rel_t > args.end_offset:
            break
        if topic not in type_map:
            continue

        msg = deserialize_message(data, type_map[topic])

        if topic == "/cmd_vel_nav2_result":
            cmd_nav.append(rel_t, msg.linear.x, msg.linear.y, msg.angular.z)
        elif topic == "/cmd_vel_controller":
            cmd_ctrl.append(rel_t, msg.linear.x, msg.linear.y, msg.angular.z)
        elif topic == "/odometry":
            tw = msg.twist.twist
            odom.append(rel_t, tw.linear.x, tw.linear.y, tw.angular.z)
        elif topic == "/serial/TES_speed":
            tes.append(rel_t, msg.linear.x, msg.linear.y, msg.angular.z)
        elif topic == "/serial/Yaw":
            serial_yaw.append(rel_t, float(msg.data))
        elif topic == "/chassis_mode":
            chassis_mode.append(rel_t, float(msg.data))
        elif topic == "/rosout":
            if msg.name == "controller_server" and "Collision detected in the trajectory" in msg.msg:
                coll_ts.append(rel_t)
        elif topic == "/local_costmap/costmap_raw":
            if cost_samples < args.max_costmap_points:
                arr = np.array(msg.data, dtype=np.uint8)
                if arr.size > 0:
                    occ_ratio = float(np.count_nonzero(arr >= 254)) / float(arr.size)
                    cost_occupied.append(rel_t, occ_ratio)
                    cost_samples += 1
        elif topic == "/referee/game_status":
            v = None
            for fn in ("game_progress", "stage", "status", "game_status"):
                if hasattr(msg, fn):
                    try:
                        v = float(getattr(msg, fn))
                        break
                    except Exception:
                        pass
            if v is None:
                # fallback: first numeric-like field
                for fn in dir(msg):
                    if fn.startswith('_'):
                        continue
                    try:
                        val = getattr(msg, fn)
                        if isinstance(val, (int, float, bool)):
                            v = float(val)
                            break
                    except Exception:
                        pass
            if v is not None:
                game_status.append(rel_t, v)

    # compute errors: controller cmd vs odom
    ev_t: List[float] = []
    ev_lin: List[float] = []
    ev_ang: List[float] = []

    for i, t in enumerate(cmd_ctrl.t):
        j = nearest_index(odom.t, t)
        if j is None:
            continue
        dvx = cmd_ctrl.x[i] - odom.x[j]
        dvy = cmd_ctrl.y[i] - odom.y[j]
        dw = cmd_ctrl.z[i] - odom.z[j]
        ev_t.append(t)
        ev_lin.append(math.hypot(dvx, dvy))
        ev_ang.append(abs(dw))

    # write csv
    csv_path = Path(f"{args.output_prefix}_errors.csv").resolve()
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "e_lin", "e_ang"])
        for t, e1, e2 in zip(ev_t, ev_lin, ev_ang):
            w.writerow([f"{t:.6f}", f"{e1:.6f}", f"{e2:.6f}"])

    # plot
    fig, axes = plt.subplots(7, 1, figsize=(16, 23), sharex=True)

    def draw_collision_lines(ax):
        for t in coll_ts:
            ax.axvline(t, color="red", alpha=0.15, linewidth=1)

    # 1 nav2 vs controller vx
    axes[0].plot(cmd_nav.t, cmd_nav.x, label="nav2 vx", linewidth=1)
    axes[0].plot(cmd_ctrl.t, cmd_ctrl.x, label="ctrl vx", linewidth=1)
    axes[0].plot(odom.t, odom.x, label="odom vx", linewidth=1)
    if tes.t:
        axes[0].plot(tes.t, tes.x, label="tes vx", linewidth=1)
    axes[0].set_ylabel("vx (m/s)")
    axes[0].legend(loc="upper right")
    draw_collision_lines(axes[0])

    # 2 vy
    axes[1].plot(cmd_nav.t, cmd_nav.y, label="nav2 vy", linewidth=1)
    axes[1].plot(cmd_ctrl.t, cmd_ctrl.y, label="ctrl vy", linewidth=1)
    axes[1].plot(odom.t, odom.y, label="odom vy", linewidth=1)
    if tes.t:
        axes[1].plot(tes.t, tes.y, label="tes vy", linewidth=1)
    axes[1].set_ylabel("vy (m/s)")
    axes[1].legend(loc="upper right")
    draw_collision_lines(axes[1])

    # 3 wz
    axes[2].plot(cmd_nav.t, cmd_nav.z, label="nav2 wz", linewidth=1)
    axes[2].plot(cmd_ctrl.t, cmd_ctrl.z, label="ctrl wz", linewidth=1)
    axes[2].plot(odom.t, odom.z, label="odom wz", linewidth=1)
    if tes.t:
        axes[2].plot(tes.t, tes.z, label="tes wz", linewidth=1)
    axes[2].set_ylabel("wz (rad/s)")
    axes[2].legend(loc="upper right")
    draw_collision_lines(axes[2])

    # 4 errors
    axes[3].plot(ev_t, ev_lin, label="|v_ctrl - v_odom|", linewidth=1)
    axes[3].plot(ev_t, ev_ang, label="|w_ctrl - w_odom|", linewidth=1)
    axes[3].set_ylabel("error")
    axes[3].legend(loc="upper right")
    draw_collision_lines(axes[3])

    # 5 mode & yaw
    if chassis_mode.t:
        axes[4].step(chassis_mode.t, chassis_mode.v, where="post", label="chassis_mode", linewidth=1)
    if serial_yaw.t:
        axes[4].plot(serial_yaw.t, serial_yaw.v, label="serial yaw", linewidth=1, alpha=0.7)
    axes[4].set_ylabel("mode/yaw")
    axes[4].legend(loc="upper right")
    draw_collision_lines(axes[4])

    # 6 game status
    if game_status.t:
        axes[5].step(game_status.t, game_status.v, where="post", label="referee game_status", linewidth=1)
    axes[5].set_ylabel("game status")
    axes[5].legend(loc="upper right")
    draw_collision_lines(axes[5])

    # 7 local costmap occupancy
    if cost_occupied.t:
        axes[6].plot(cost_occupied.t, cost_occupied.v, label="local_costmap occupied_ratio(>=254)", linewidth=1)
    axes[6].set_ylabel("occ ratio")
    axes[6].set_xlabel("time from bag start (s)")
    axes[6].legend(loc="upper right")
    draw_collision_lines(axes[6])

    fig.suptitle(f"Bag Diagnostics: {Path(bag_path).name} [{args.start_offset}s, {args.end_offset if args.end_offset is not None else 'end'}s]")
    fig.tight_layout(rect=(0, 0, 1, 0.98))

    png_path = Path(f"{args.output_prefix}_overview.png").resolve()
    fig.savefig(png_path, dpi=140)

    # print quick summary
    def p95(vals: List[float]) -> float:
        if not vals:
            return 0.0
        arr = sorted(vals)
        return arr[min(len(arr)-1, int(0.95*len(arr)))]

    print("=== Summary ===")
    print(f"range: {args.start_offset} -> {args.end_offset if args.end_offset is not None else 'end'} s")
    print(f"collision_count: {len(coll_ts)}")
    print(f"samples cmd_nav/cmd_ctrl/odom/tes: {len(cmd_nav.t)}/{len(cmd_ctrl.t)}/{len(odom.t)}/{len(tes.t)}")
    print(f"error_lin_mean: {np.mean(ev_lin) if ev_lin else 0:.6f}, error_lin_p95: {p95(ev_lin):.6f}")
    print(f"error_ang_mean: {np.mean(ev_ang) if ev_ang else 0:.6f}, error_ang_p95: {p95(ev_ang):.6f}")
    print(f"output_png: {png_path}")
    print(f"output_csv: {csv_path}")


if __name__ == "__main__":
    main()
