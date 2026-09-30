#!/usr/bin/env python3
"""对比两个 point_lio 版本的里程计输出 (基于已导出的 CSV)。

- 原始版: out/original/odom.csv (从 bag 导出, 含录制时间与 header 时间)
- 重构版: out/refactored/odom.csv (监视器导出, header 时间)
对齐基准统一用 header_stamp_ns (雷达帧结束时刻)。
"""

import math
import os
import sqlite3
import sys

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
PL_DIR = os.environ.get("PL_CMP_DIR", os.path.join(WS, ".pl_cmp_bench"))

RADIUS = 2.0
OMEGA = 0.2
HEIGHT = 0.5
BOB_A = 0.05
BOB_F = 1.0
T_START_LIDAR = 1.0


def first_lidar_stamp():
    """从输入 bag 里读首帧雷达时间戳, 作为把 header 时间换算成真值时刻的基准。"""
    db = os.path.join(PL_DIR, "bags", "mid360_synth", "mid360_synth_0.db3")
    if not os.path.exists(db):
        raise SystemExit(f"输入 bag 不存在: {db}, 请先运行 gen_bag.py")
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    row = con.execute(
        "select min(m.timestamp) from messages m join topics t on t.id = m.topic_id "
        "where t.name = '/livox/lidar'").fetchone()
    con.close()
    return row[0]


FIRST_LIDAR_STAMP = None  # 在 main() 中初始化


def load(label):
    path = os.path.join(PL_DIR, "out", label, "odom.csv")
    raw = np.genfromtxt(path, delimiter=",", names=True)
    return raw


def yaw(q):
    x, y, z, w = q["qx"], q["qy"], q["qz"], q["qw"]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def gt(t):
    return np.stack([
        RADIUS * np.sin(OMEGA * t),
        RADIUS - RADIUS * np.cos(OMEGA * t),
        HEIGHT + BOB_A * np.sin(2.0 * math.pi * BOB_F * t),
    ], axis=1)


def align_2d(run_xy, gt_xy):
    c_run, c_gt = run_xy.mean(axis=0), gt_xy.mean(axis=0)
    a, b = run_xy - c_run, gt_xy - c_gt
    u, _, vt = np.linalg.svd(a.T @ b)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ np.diag([1.0, d]) @ u.T
    trans = c_gt - rot @ c_run
    return (rot @ run_xy.T).T + trans - gt_xy, rot, trans


def describe(label, raw):
    names = raw.dtype.names
    hdr = raw["header_stamp_ns"] if "header_stamp_ns" in names else raw["stamp_ns"]
    pos = np.stack([raw["x"], raw["y"], raw["z"]], axis=1)
    t = (hdr - FIRST_LIDAR_STAMP) / 1e9 + T_START_LIDAR
    print(f"\n[{label}]")
    print(f"  里程计条数: {len(hdr)}   雷达时间跨度: {t[-1] - t[0]:.3f} s"
          f"   时间步中位数: {np.median(np.diff(t)) * 1000:.1f} ms")
    print(f"  起止雷达时刻: {t[0]:.3f} s -> {t[-1]:.3f} s (输入共 20 s)")
    print(f"  轨迹长度: {np.linalg.norm(np.diff(pos, axis=0), axis=1).sum():.3f} m"
          f"   终点: ({pos[-1, 0]:.3f}, {pos[-1, 1]:.3f}, {pos[-1, 2]:.3f})")
    g = gt(t)
    resid, rot, trans = align_2d(pos[:, :2], g[:, :2])
    print(f"  相对真值(2D 刚体对齐后) RMSE: {np.sqrt((resid ** 2).sum(axis=1).mean()):.4f} m"
          f"   最大偏差: {np.abs(resid).max():.4f} m")
    print(f"  对齐角度: {math.degrees(math.atan2(rot[1, 0], rot[0, 0])):.2f} deg"
          f"   对齐平移: ({trans[0]:.3f}, {trans[1]:.3f}) m")
    y = yaw(raw)
    zb = np.arctan2(np.sin(y - (OMEGA * t)), np.cos(y - (OMEGA * t)))
    # 去掉常量偏置后的 yaw 误差才反映真实漂移
    zc = np.arctan2(np.sin(zb - zb.mean()), np.cos(zb - zb.mean()))
    print(f"  yaw 与真值差: 均值 {math.degrees(zb.mean()):.2f} deg,"
          f" 去偏置后 RMSE {math.degrees(np.sqrt((zc ** 2).mean())):.3f} deg")
    return hdr, pos, y


def main():
    global FIRST_LIDAR_STAMP
    FIRST_LIDAR_STAMP = first_lidar_stamp()
    labels = sys.argv[1:3] or ["original", "refactored_after_fix"]
    o = load(labels[0])
    r = load(labels[1])
    ho, po, yo = describe(f"{labels[0]} (main 分支)", o)
    hr, pr, yr = describe(f"{labels[1]} (重构版)", r)

    common = np.intersect1d(ho, hr)
    print(f"\n=== 两版本对比 ===")
    print(f"  共同雷达帧: {len(common)} 帧"
          f" (原始版 {len(ho)} 帧的 {100.0 * len(common) / len(ho):.1f}%,"
          f" 重构版 {len(hr)} 帧的 {100.0 * len(common) / len(hr):.1f}%)")
    if len(common):
        io = {s: i for i, s in enumerate(ho)}
        ir = {s: i for i, s in enumerate(hr)}
        ao = np.array([po[io[s]] for s in common])
        ar = np.array([pr[ir[s]] for s in common])
        diff = ao - ar
        print(f"  共同帧位置差 RMSE: {np.sqrt((diff ** 2).sum(axis=1).mean()):.4f} m"
              f"   最大: {np.abs(diff).max():.4f} m")
        print(f"  终点位置差: {np.linalg.norm(diff[-1]):.4f} m")
        dyo = np.array([yo[io[s]] for s in common])
        dyr = np.array([yr[ir[s]] for s in common])
        dy = np.arctan2(np.sin(dyo - dyr), np.cos(dyo - dyr))
        print(f"  共同帧 yaw 差 RMSE: {math.degrees(np.sqrt((dy ** 2).mean())):.4f} deg"
              f"   最大: {math.degrees(np.abs(dy).max()):.4f} deg")

    # 未对齐的帧: 重构版处理了哪些、漏了哪些
    only_o = np.setdiff1d(ho, hr)
    print(f"  仅原始版处理过的帧: {len(only_o)} 帧")
    if len(only_o):
        gaps = np.diff(np.sort(hr))
        print(f"  重构版相邻处理帧的时间间隔: 中位数 {np.median(gaps) / 1e6:.1f} ms"
              f"   最大 {gaps.max() / 1e6:.1f} ms (输入帧间隔 100 ms)")


if __name__ == "__main__":
    main()
