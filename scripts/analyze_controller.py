#!/usr/bin/env python3
"""Analyze a rosbag for controller performance issues.

Usage:
  python3 scripts/analyze_controller.py <bag_path> [--storage mcap]
  python3 scripts/analyze_controller.py rosbag/rosbag2_2026_04_04-15_24_33_0.mcap
"""

import sys
import math
import argparse

sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
sys.path.insert(0, '/opt/ros/humble/local/lib/python3.10/dist-packages')

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from pathlib import Path as FsPath


def analyze(bag_uri, storage_id="mcap"):
    storage = StorageOptions(uri=bag_uri, storage_id=storage_id)
    converter = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr")

    reader = SequentialReader()
    reader.open(storage, converter)

    cmd_vel = []        # (ts, speed)
    local_plan = []     # (ts, poses_count)
    collisions = []     # (ts, msg)
    tf_errors = []      # (ts, msg)
    plan = []           # (ts, poses_count)

    print("Reading bag...", file=sys.stderr, end="", flush=True)
    count = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        count += 1
        if count % 500000 == 0:
            print(f".", file=sys.stderr, end="", flush=True)

        if topic == "/cmd_vel_controller":
            msg = deserialize_message(data, Twist)
            cmd_vel.append((ts, math.hypot(msg.linear.x, msg.linear.y)))
        elif topic == "/local_plan":
            msg = deserialize_message(data, NavPath)
            local_plan.append((ts, len(msg.poses)))
        elif topic == "/plan":
            msg = deserialize_message(data, NavPath)
            plan.append((ts, len(msg.poses)))
        elif topic == "/rosout":
            msg = deserialize_message(data, Log)
            if "controller" not in msg.name.lower():
                continue
            if "collision" in msg.msg.lower():
                collisions.append((ts, msg.msg))
            elif "extrapolation" in msg.msg.lower():
                tf_errors.append((ts, msg.msg))

    print(f"\nDone: {count} messages", file=sys.stderr)

    if not cmd_vel:
        print("ERROR: No /cmd_vel_controller messages found!")
        return

    # ---- Filter active period ----
    first_move = 0
    for i in range(10, len(cmd_vel)):
        if all(s > 0.1 for _, s in cmd_vel[i-9:i+1]):
            first_move = cmd_vel[i-9][0]
            break
    t0 = first_move if first_move else cmd_vel[0][0]

    def active(seq):
        return [(ts, v) for ts, v in seq if ts >= t0]

    cv = active(cmd_vel)
    lp = active(local_plan)
    pl = active(plan)
    cols = active(collisions)
    tf_errs = active(tf_errors)

    t_end = cv[-1][0]
    dur = (t_end - t0) / 1e9

    # ---- Speed metrics ----
    speeds = [s for _, s in cv]
    speeds_sorted = sorted(speeds)
    avg_speed = sum(speeds) / len(speeds)
    var = sum((s - avg_speed)**2 for s in speeds) / len(speeds)
    stopped = sum(1 for s in speeds if s < 0.05)
    drops = sum(1 for i in range(1, len(speeds))
                if speeds[i-1] - speeds[i] > 1.0 and speeds[i] < 0.1)

    # ---- Timing ----
    dts = [(cv[i][0] - cv[i-1][0]) / 1e9 for i in range(1, len(cv))]
    dts_sorted = sorted(dts)
    stalls_500 = sum(1 for d in dts if d > 0.5)

    # ---- Local plan ----
    lp_lens = [n for _, n in lp]
    lp_empty = sum(1 for n in lp_lens if n <= 1)
    lp_median_len = sorted(lp_lens)[len(lp_lens)//2] if lp_lens else 0

    # ---- Plan ----
    pl_dts = [(pl[i][0] - pl[i-1][0]) / 1e9 for i in range(1, len(pl))]
    pl_median_dt = sorted(pl_dts)[len(pl_dts)//2] if pl_dts else 0
    pl_short = sum(1 for _, n in pl if n < 5)

    # ---- Output ----
    print(f"""
{'='*60}
  BAG: {bag_uri}
  Duration (active): {dur:.0f}s

  --- cmd_vel ---
  Messages:         {len(cv):>6}  ({len(cv)/dur:.1f} Hz)
  Speed mean:       {avg_speed:>6.2f} m/s
  Speed std:        {math.sqrt(var):>6.2f} m/s
  p50 speed:        {speeds_sorted[len(speeds_sorted)//2]:>6.2f} m/s
  p10 speed:        {speeds_sorted[len(speeds_sorted)//10]:>6.2f} m/s
  p90 speed:        {speeds_sorted[len(speeds_sorted)*9//10]:>6.2f} m/s
  Stopped (<0.05):  {stopped:>6}  ({100*stopped/len(speeds):.1f}%)
  Speed drops (>1):  {drops:>6}  ({drops/(dur/60):.1f}/min)
  Stalls (>500ms):  {stalls_500:>6}
  Max dt:           {max(dts)*1000 if dts else 0:>7.0f} ms
  Median dt:        {dts_sorted[len(dts_sorted)//2]*1000 if dts_sorted else 0:>7.1f} ms
                     (~{1000/(dts_sorted[len(dts_sorted)//2]*1000) if dts_sorted and dts_sorted[len(dts_sorted)//2] > 0 else 0:.0f} Hz)

  --- /local_plan ---
  Messages:         {len(lp):>6}  ({len(lp)/dur:.1f} Hz)
  Poses median:     {lp_median_len:>6}
  Empty (<=1):      {lp_empty:>6}  ({100*lp_empty/max(len(lp_lens),1):.1f}%)

  --- /plan ---
  Messages:         {len(pl):>6}  ({len(pl)/(dur/60):.1f}/min)
  Interval median:  {pl_median_dt:>6.1f}s
  Short (<5 poses): {pl_short:>6}  ({100*pl_short/max(len(pl),1):.1f}%)

  --- Errors ---
  Collision:        {len(cols):>6}  ({len(cols)/(dur/60):.1f}/min)
  TF extrapolation: {len(tf_errs):>6}  ({len(tf_errs)/(dur/60):.1f}/min)
{'='*60}
""")


def compare(bag1, bag2):
    """Placeholder for before/after comparison."""
    pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze controller performance from rosbag")
    parser.add_argument("bag", help="Path to rosbag")
    parser.add_argument("--storage", default="mcap", help="Storage ID (mcap, sqlite3)")
    parser.add_argument("--compare", help="Path to second bag for before/after comparison")
    args = parser.parse_args()

    if not FsPath(args.bag).exists():
        print(f"ERROR: bag not found: {args.bag}")
        sys.exit(1)

    analyze(args.bag, args.storage)

    if args.compare:
        if not FsPath(args.compare).exists():
            print(f"ERROR: comparison bag not found: {args.compare}")
            sys.exit(1)
        print("\n========== COMPARISON BAG ==========")
        analyze(args.compare, args.storage)
