#!/usr/bin/env python3
"""
可视化 MPC 调试日志。

用法:
    # 交互查看最后一帧 + 概览
    python3 visualize_mpc_log.py

    # 保存图片
    python3 visualize_mpc_log.py --save results/

    # 生成视频 (并行渲染 + ffmpeg)
    python3 visualize_mpc_log.py --video output.mp4
    python3 visualize_mpc_log.py --video output.mp4 --fps 15 --workers 8
"""

import argparse
import atexit
import os
import shutil
import subprocess
import sys
import tempfile
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from multiprocessing import cpu_count

import numpy as np


def _setup_agg():
    """在 worker 进程中设置 Agg 后端 (无 GUI, 更快的 PNG 渲染)."""
    import matplotlib
    matplotlib.use("Agg")


def parse_log(filepath):
    """解析 MPC 调试日志文件, 返回帧列表."""
    frames = []
    current = None

    with open(filepath, "r") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue

            parts = line.split()
            tag = parts[0]

            if tag == "FRAME":
                if current is not None:
                    frames.append(current)
                current = {"frame_id": int(parts[1])}

            elif tag == "X0":
                current["x0"] = [float(x) for x in parts[1:]]
            elif tag == "YAW":
                current["yaw"] = float(parts[1])
            elif tag == "LOCAL_PLAN":
                current["local_plan_n"] = int(parts[1])
                current["local_plan"] = []
            elif tag == "LP":
                current["local_plan"].append([float(x) for x in parts[1:]])
            elif tag == "REF_TRAJ":
                current["ref_traj_n"] = int(parts[1])
                current["ref_traj"] = []
            elif tag == "RT":
                current["ref_traj"].append([float(x) for x in parts[1:]])
            elif tag == "END_REF":
                current["end_ref"] = [float(x) for x in parts[1:]]
            elif tag == "PRED_STATES":
                current["pred_states_n"] = int(parts[1])
                current["pred_states"] = []
            elif tag == "PS":
                current["pred_states"].append([float(x) for x in parts[1:]])
            elif tag == "U_OPT_MAP":
                current["u_opt_map"] = [float(x) for x in parts[1:]]
            elif tag == "U_OPT_BODY":
                current["u_opt_body"] = [float(x) for x in parts[1:]]
            elif tag == "SOLVE_TIME":
                current["solve_time"] = float(parts[1])

    if current is not None:
        frames.append(current)

    return frames


# ──────────────────────────────────────────────
#  单帧渲染 (worker 进程入口)
# ──────────────────────────────────────────────

def _draw_one_frame(frame_data):
    """
    在子进程中渲染单帧并保存为 PNG.
    frame_data: (frame_dict, out_path) 元组.
    返回: out_path 或 None.
    """
    _setup_agg()
    import matplotlib.pyplot as plt

    frame, out_path = frame_data
    fid = frame.get("frame_id", 0)

    try:
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(f"MPC Debug — Frame {fid}", fontsize=14)

        # ---- (0,0) XY 轨迹 ----
        ax = axes[0, 0]
        ax.set_title("XY Trajectory Comparison")
        ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
        ax.set_aspect("equal"); ax.grid(True)

        if frame.get("local_plan"):
            lp = np.array(frame["local_plan"])
            ax.plot(lp[:, 0], lp[:, 1], "k.-", linewidth=2, markersize=4,
                    label="Local Plan")
            ax.scatter(lp[0, 0], lp[0, 1], c="k", marker="o", s=60, zorder=5)

        if frame.get("ref_traj"):
            rt = np.array(frame["ref_traj"])
            ax.plot(rt[:, 0], rt[:, 1], "orange", linestyle="--", linewidth=2,
                    marker="s", markersize=5, label="Ref Trajectory")
            for i in range(0, len(rt), 2):
                ax.arrow(rt[i, 0], rt[i, 1],
                         0.05 * np.cos(rt[i, 2]), 0.05 * np.sin(rt[i, 2]),
                         head_width=0.03, head_length=0.03,
                         fc="orange", ec="orange", alpha=0.7)

        if frame.get("end_ref"):
            er = frame["end_ref"]
            ax.scatter(er[0], er[1], c="red", marker="*", s=120, zorder=6,
                       label="End Ref")

        if frame.get("pred_states"):
            ps = np.array(frame["pred_states"])
            ax.plot(ps[:, 0], ps[:, 1], "b-", linewidth=2, marker="^",
                    markersize=4, label="Predicted States")

        if frame.get("x0"):
            x0 = frame["x0"]
            ax.scatter(x0[0], x0[1], c="green", marker="D", s=100, zorder=7,
                       label="Robot (x0)")
            ax.arrow(x0[0], x0[1],
                     0.08 * np.cos(x0[2]), 0.08 * np.sin(x0[2]),
                     head_width=0.04, head_length=0.04, fc="green", ec="green")
        ax.legend(fontsize=8)

        # ---- (0,1) Theta ----
        ax = axes[0, 1]
        ax.set_title("Theta over Stages")
        ax.set_xlabel("Stage"); ax.set_ylabel("Theta [rad]"); ax.grid(True)
        if frame.get("ref_traj"):
            rt = np.array(frame["ref_traj"])
            ax.plot(np.arange(len(rt)), rt[:, 2], "orange", linestyle="--",
                    marker="s", markersize=5, label="Ref Theta")
        if frame.get("pred_states"):
            ps = np.array(frame["pred_states"])
            ax.plot(np.arange(len(ps)), ps[:, 2], "b-", marker="^",
                    markersize=4, label="Predicted Theta")
        if frame.get("x0"):
            ax.axhline(y=frame["x0"][2], color="green", linestyle=":",
                       linewidth=1.5, label="Robot Theta (x0)")
        ax.legend(fontsize=8)

        # ---- (1,0) 参考速度 ----
        ax = axes[1, 0]
        ax.set_title("Reference Velocities over Stages")
        ax.set_xlabel("Stage"); ax.set_ylabel("Velocity [m/s or rad/s]")
        ax.grid(True)
        if frame.get("ref_traj"):
            rt = np.array(frame["ref_traj"])
            stages = np.arange(len(rt))
            ax.plot(stages, rt[:, 3], "r-", marker=".", label="vx_ref")
            ax.plot(stages, rt[:, 4], "g-", marker=".", label="vy_ref")
            ax.plot(stages, rt[:, 5], "b-", marker=".", label="ω_ref")
        ax.legend(fontsize=8)

        # ---- (1,1) 信息面板 ----
        ax = axes[1, 1]
        ax.set_title("Control Output"); ax.axis("off")

        lines = [f"Frame: {fid}"]
        if frame.get("x0"):
            x0 = frame["x0"]
            lines.append(f"Robot Pose: ({x0[0]:.3f}, {x0[1]:.3f}, {x0[2]:.3f})")
        if frame.get("yaw") is not None:
            lines.append(f"Yaw: {frame['yaw']:.3f} rad ({np.degrees(frame['yaw']):.1f}°)")
        if frame.get("u_opt_map"):
            um = frame["u_opt_map"]
            lines.append(f"U (map):  vx={um[0]:.3f}  vy={um[1]:.3f}  ω={um[2]:.3f}")
        if frame.get("u_opt_body"):
            ub = frame["u_opt_body"]
            lines.append(f"U (body): vx={ub[0]:.3f}  vy={ub[1]:.3f}  ω={ub[2]:.3f}")
        if frame.get("solve_time") is not None:
            lines.append(f"Solve Time: {frame['solve_time'] * 1000:.2f} ms")
        if frame.get("local_plan_n"):
            lines.append(f"Local Plan Points: {frame['local_plan_n']}")
        if frame.get("ref_traj_n"):
            lines.append(f"Ref Traj Steps: {frame['ref_traj_n']}")
        if frame.get("pred_states_n"):
            lines.append(f"Pred States: {frame['pred_states_n']}")

        ax.text(0.05, 0.95, "\n".join(lines), transform=ax.transAxes,
                fontsize=10, fontfamily="monospace", verticalalignment="top")

        fig.tight_layout()
        fig.savefig(out_path, dpi=120)        # dpi=120 比 150 快, 视频足够清晰
        plt.close(fig)
        return out_path

    except Exception as exc:
        print(f"\n[错误] 渲染帧 {fid} 失败: {exc}")
        return None


# ──────────────────────────────────────────────
#  并行渲染器
# ──────────────────────────────────────────────

def render_frames_parallel(frames, out_dir, max_workers=None, batch_size=50):
    """
    并行渲染所有帧为 PNG.

    参数:
        frames:     帧数据列表.
        out_dir:    输出目录.
        max_workers: 并行进程数 (默认: CPU 核心数 - 1, 至少 1).
        batch_size: 每批提交的任务数 (避免一次性占用太多内存).
    """
    os.makedirs(out_dir, exist_ok=True)
    total = len(frames)

    if max_workers is None:
        max_workers = max(1, cpu_count() - 1)

    # 构建任务列表: (frame, out_path)
    tasks = []
    for idx, frame in enumerate(frames):
        fid = frame.get("frame_id", idx)
        out_path = os.path.join(out_dir, f"frame_{fid:06d}.png")
        tasks.append((frame, out_path))

    completed = 0
    failed = 0

    print(f"  并行渲染 {total} 帧 (workers={max_workers}) ...")

    # 分批提交, 避免 ProcessPoolExecutor 一次性序列化全部数据
    for batch_start in range(0, total, batch_size):
        batch = tasks[batch_start: batch_start + batch_size]

        with ProcessPoolExecutor(
            max_workers=max_workers,
            initializer=_setup_agg,
        ) as executor:
            futures = {executor.submit(_draw_one_frame, t): t for t in batch}

            for future in as_completed(futures):
                result = future.result()
                if result is not None:
                    completed += 1
                else:
                    failed += 1

                pct = (completed + failed) / total * 100
                print(f"\r    进度: {completed + failed}/{total} "
                      f"({pct:.0f}%)  ✓{completed} ✗{failed}",
                      end="", flush=True)

    print()
    return completed, failed


# ──────────────────────────────────────────────
#  视频合成
# ──────────────────────────────────────────────

def frames_to_video(frames_dir, output_path, fps=10, nvenc=False):
    """
    将 frames_dir 下的 PNG 序列合成为 MP4 视频.
    """
    input_pattern = os.path.join(frames_dir, "frame_%06d.png")

    if nvenc and shutil.which("ffmpeg"):
        # 检查 nvenc 是否可用
        check = subprocess.run(
            ["ffmpeg", "-hide_banner", "-encoders"],
            capture_output=True, text=True)
        if "h264_nvenc" not in check.stdout:
            print("  [警告] h264_nvenc 不可用, 回退到 libx264")
            nvenc = False

    if nvenc:
        vcodec = "h264_nvenc"
        extra = ["-preset", "p1", "-rc", "vbr", "-cq", "23"]
    else:
        vcodec = "libx264"
        extra = ["-preset", "medium", "-crf", "23"]

    first_frame = os.path.join(frames_dir, "frame_000000.png")
    if not os.path.exists(first_frame):
        existing = sorted(os.listdir(frames_dir))
        if not existing:
            print("[错误] 没有渲染出任何帧")
            return False

    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(fps),
        "-i", input_pattern,
        "-c:v", vcodec,
        *extra,
        "-pix_fmt", "yuv420p",
        "-threads", str(cpu_count()),
        "-loglevel", "error",
        "-stats",
        output_path,
    ]

    print(f"  合成视频 (this may take a while) ...")
    # stdin=DEVNULL 防止 ffmpeg 等待终端输入导致卡死
    result = subprocess.run(cmd, stdin=subprocess.DEVNULL)
    if result.returncode != 0:
        print(f"\n[错误] ffmpeg 退出码: {result.returncode}")
        return False
    return True


# ──────────────────────────────────────────────
#  交互模式: 单帧详情 + 概览
# ──────────────────────────────────────────────

def plot_single_frame(frame, out_dir=None):
    """为单帧数据绘制详细图表 (交互/保存)."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f"MPC Debug — Frame {frame['frame_id']}", fontsize=14)

    # ---- (0,0) XY ----
    ax = axes[0, 0]
    ax.set_title("XY Trajectory Comparison")
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
    ax.set_aspect("equal"); ax.grid(True)

    if frame.get("local_plan"):
        lp = np.array(frame["local_plan"])
        ax.plot(lp[:, 0], lp[:, 1], "k.-", linewidth=2, markersize=4,
                label="Local Plan")
        ax.scatter(lp[0, 0], lp[0, 1], c="k", marker="o", s=60, zorder=5)
        ax.scatter(lp[-1, 0], lp[-1, 1], c="k", marker="x", s=60, zorder=5)

    if frame.get("ref_traj"):
        rt = np.array(frame["ref_traj"])
        ax.plot(rt[:, 0], rt[:, 1], "orange", linestyle="--", linewidth=2,
                marker="s", markersize=5, label="Ref Trajectory")
        for i in range(0, len(rt), 2):
            ax.arrow(rt[i, 0], rt[i, 1],
                     0.05 * np.cos(rt[i, 2]), 0.05 * np.sin(rt[i, 2]),
                     head_width=0.03, head_length=0.03,
                     fc="orange", ec="orange", alpha=0.7)

    if frame.get("end_ref"):
        er = frame["end_ref"]
        ax.scatter(er[0], er[1], c="red", marker="*", s=120, zorder=6,
                   label="End Ref")

    if frame.get("pred_states"):
        ps = np.array(frame["pred_states"])
        ax.plot(ps[:, 0], ps[:, 1], "b-", linewidth=2, marker="^",
                markersize=4, label="Predicted States")
        ax.scatter(ps[0, 0], ps[0, 1], c="b", marker="o", s=50, zorder=5)

    if frame.get("x0"):
        x0 = frame["x0"]
        ax.scatter(x0[0], x0[1], c="green", marker="D", s=100, zorder=7,
                   label="Robot (x0)")
        ax.arrow(x0[0], x0[1],
                 0.08 * np.cos(x0[2]), 0.08 * np.sin(x0[2]),
                 head_width=0.04, head_length=0.04, fc="green", ec="green")
    ax.legend(fontsize=8)

    # ---- (0,1) Theta ----
    ax = axes[0, 1]
    ax.set_title("Theta over Stages")
    ax.set_xlabel("Stage"); ax.set_ylabel("Theta [rad]"); ax.grid(True)

    if frame.get("ref_traj"):
        rt = np.array(frame["ref_traj"])
        ax.plot(np.arange(len(rt)), rt[:, 2], "orange", linestyle="--",
                marker="s", markersize=5, label="Ref Theta")
    if frame.get("pred_states"):
        ps = np.array(frame["pred_states"])
        ax.plot(np.arange(len(ps)), ps[:, 2], "b-", marker="^",
                markersize=4, label="Predicted Theta")
    if frame.get("x0"):
        ax.axhline(y=frame["x0"][2], color="green", linestyle=":",
                   linewidth=1.5, label="Robot Theta (x0)")
    ax.legend(fontsize=8)

    # ---- (1,0) 参考速度 ----
    ax = axes[1, 0]
    ax.set_title("Reference Velocities over Stages")
    ax.set_xlabel("Stage"); ax.set_ylabel("Velocity [m/s or rad/s]")
    ax.grid(True)
    if frame.get("ref_traj"):
        rt = np.array(frame["ref_traj"])
        stages = np.arange(len(rt))
        ax.plot(stages, rt[:, 3], "r-", marker=".", label="vx_ref")
        ax.plot(stages, rt[:, 4], "g-", marker=".", label="vy_ref")
        ax.plot(stages, rt[:, 5], "b-", marker=".", label="ω_ref")
    ax.legend(fontsize=8)

    # ---- (1,1) 信息面板 ----
    ax = axes[1, 1]
    ax.set_title("Control Output"); ax.axis("off")

    lines = [f"Frame: {frame.get('frame_id', '?')}"]
    if frame.get("x0"):
        x0 = frame["x0"]
        lines.append(f"Robot Pose: ({x0[0]:.3f}, {x0[1]:.3f}, {x0[2]:.3f})")
    if frame.get("yaw") is not None:
        lines.append(f"Yaw: {frame['yaw']:.3f} rad ({np.degrees(frame['yaw']):.1f}°)")
    if frame.get("u_opt_map"):
        um = frame["u_opt_map"]
        lines.append(f"U (map):  vx={um[0]:.3f}  vy={um[1]:.3f}  ω={um[2]:.3f}")
    if frame.get("u_opt_body"):
        ub = frame["u_opt_body"]
        lines.append(f"U (body): vx={ub[0]:.3f}  vy={ub[1]:.3f}  ω={ub[2]:.3f}")
    if frame.get("solve_time") is not None:
        lines.append(f"Solve Time: {frame['solve_time'] * 1000:.2f} ms")
    if frame.get("local_plan_n"):
        lines.append(f"Local Plan Points: {frame['local_plan_n']}")
    if frame.get("ref_traj_n"):
        lines.append(f"Ref Traj Steps: {frame['ref_traj_n']}")
    if frame.get("pred_states_n"):
        lines.append(f"Pred States: {frame['pred_states_n']}")

    ax.text(0.05, 0.95, "\n".join(lines), transform=ax.transAxes,
            fontsize=10, fontfamily="monospace", verticalalignment="top")

    fig.tight_layout()

    if out_dir:
        out_path = os.path.join(out_dir, f"frame_{frame['frame_id']:04d}.png")
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        fig.savefig(out_path, dpi=150)
        print(f"  已保存: {out_path}")
    else:
        plt.show()
    plt.close(fig)


def plot_state_tracking(frames, out_dir=None):
    """
    状态跟踪对比图: 参考轨迹 vs 机器人实际状态 (x, y, θ 分开画).

    从每帧提取:
      - 实际状态 x0 = [x, y, θ]
      - 参考状态 ref0 = ref_traj 的第一个 stage (最近参考点) [x, y, θ]
    """
    import matplotlib.pyplot as plt

    # 提取数据
    x0_arr = np.array([f["x0"]            for f in frames if "x0"       in f])
    ref0    = np.array([f["ref_traj"][0]  for f in frames if "ref_traj" in f])

    n_robot = len(x0_arr)
    n_ref   = len(ref0)
    if n_robot == 0 and n_ref == 0:
        return
    n_max = max(n_robot, n_ref)

    time_axis = np.arange(n_max)

    # 跟踪误差 (仅当两者帧数一致时对齐; 否则各画各的)
    if n_robot == n_ref:
        err = x0_arr - ref0[:, :3]   # [Δx, Δy, Δθ]
        # 角度误差归一化到 [-π, π]
        err[:, 2] = np.arctan2(np.sin(err[:, 2]), np.cos(err[:, 2]))
        has_error = True
    else:
        has_error = False

    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle("MPC State Tracking: Reference vs Actual", fontsize=14)

    labels = ["X [m]", "Y [m]", "θ [rad]"]
    colors_ref = ["orange", "orange", "orange"]
    colors_act = ["blue", "blue", "blue"]

    for row, (label, cref, cact) in enumerate(zip(labels, colors_ref, colors_act)):
        # 左列: 参考 vs 实际
        ax = axes[row, 0]
        ax.set_title(f"{label} — Reference vs Actual")
        ax.set_xlabel("Frame"); ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)

        if n_ref > 0:
            ax.plot(time_axis[:n_ref], ref0[:, row],
                    color=cref, linestyle="--", linewidth=2,
                    marker=".", markersize=3, label=f"{label} ref")
        if n_robot > 0:
            ax.plot(time_axis[:n_robot], x0_arr[:, row],
                    color=cact, linestyle="-", linewidth=1.5,
                    label=f"{label} actual")

        # theta 图加 ±π 参考线
        if row == 2:
            ax.axhline(y=np.pi,  color="gray", linestyle=":", alpha=0.4)
            ax.axhline(y=-np.pi, color="gray", linestyle=":", alpha=0.4)

        ax.legend(fontsize=8)

        # 右列: 跟踪误差
        ax = axes[row, 1]
        ax.set_title(f"{label} — Tracking Error")
        ax.set_xlabel("Frame"); ax.set_ylabel(f"Δ{label}")
        ax.grid(True, alpha=0.3)

        if has_error:
            ax.plot(time_axis, err[:, row], "r-", linewidth=1.5,
                    marker=".", markersize=3, label=f"Δ{label}")
            ax.axhline(y=0, color="gray", linestyle="--", alpha=0.5)

            # 统计信息
            rmse = np.sqrt(np.mean(err[:, row] ** 2))
            max_e = np.max(np.abs(err[:, row]))
            ax.text(0.02, 0.95,
                    f"RMSE: {rmse:.4f}\nMax: {max_e:.4f}",
                    transform=ax.transAxes, fontsize=9,
                    fontfamily="monospace", verticalalignment="top",
                    bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.6))
        else:
            ax.text(0.5, 0.5, "帧数不匹配, 无法计算误差",
                    transform=ax.transAxes, ha="center", fontsize=10)
        ax.legend(fontsize=8)

    fig.tight_layout()

    if out_dir:
        out_path = os.path.join(out_dir, "state_tracking.png")
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        fig.savefig(out_path, dpi=150)
        print(f"  已保存: {out_path}")
    else:
        plt.show()
    plt.close(fig)


def plot_overview(frames, out_dir=None):
    """所有帧的概览图."""
    import matplotlib.pyplot as plt

    n = len(frames)
    if n < 1:
        return

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(f"MPC Overview — {n} Frames", fontsize=14)

    x0_arr = np.array([f["x0"] for f in frames if "x0" in f])

    # (0,0) 机器人 XY 轨迹
    ax = axes[0, 0]
    ax.set_title("Robot Trajectory (x0)")
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
    ax.set_aspect("equal"); ax.grid(True)
    if len(x0_arr) > 0:
        ax.plot(x0_arr[:, 0], x0_arr[:, 1], "g-", linewidth=1.5, alpha=0.8)
        ax.scatter(x0_arr[0, 0], x0_arr[0, 1], c="blue", marker="o", s=60,
                   label="Start")
        ax.scatter(x0_arr[-1, 0], x0_arr[-1, 1], c="red", marker="x", s=60,
                   label="End")
        step = max(1, len(x0_arr) // 20)
        for i in range(0, len(x0_arr), step):
            ax.arrow(x0_arr[i, 0], x0_arr[i, 1],
                     0.05 * np.cos(x0_arr[i, 2]),
                     0.05 * np.sin(x0_arr[i, 2]),
                     head_width=0.02, head_length=0.02,
                     fc="green", ec="green", alpha=0.5)
        ax.legend(fontsize=8)

    # (0,1) θ 跟踪对比 (ref vs actual)
    ax = axes[0, 1]
    ax.set_title("Theta: Ref vs Actual")
    ax.set_xlabel("Frame"); ax.set_ylabel("Theta [rad]"); ax.grid(True)
    if len(x0_arr) > 0:
        ax.plot(x0_arr[:, 2], "b-", linewidth=1.5, alpha=0.8, label="Robot θ")
    # 从 ref_traj 取第一个 stage 的 θ
    ref_theta = np.array([f["ref_traj"][0][2]
                          for f in frames if "ref_traj" in f])
    if len(ref_theta) > 0:
        ax.plot(ref_theta, "orange", linestyle="--", linewidth=1.5,
                alpha=0.8, label="Ref θ")
    ax.axhline(y=np.pi, color="gray", linestyle=":", alpha=0.3)
    ax.axhline(y=-np.pi, color="gray", linestyle=":", alpha=0.3)
    ax.legend(fontsize=8)

    # (0,2) 控制 (body)
    ax = axes[0, 2]
    ax.set_title("Control (Body Frame)")
    ax.set_xlabel("Frame"); ax.set_ylabel("Velocity"); ax.grid(True)
    ub_arr = np.array([f["u_opt_body"] for f in frames if "u_opt_body" in f])
    if len(ub_arr) > 0:
        ax.plot(ub_arr[:, 0], "r-", linewidth=1, alpha=0.8, label="vx_body")
        ax.plot(ub_arr[:, 1], "g-", linewidth=1, alpha=0.8, label="vy_body")
        ax.plot(ub_arr[:, 2], "b-", linewidth=1, alpha=0.8, label="ω_body")
        ax.legend(fontsize=8)

    # (1,0) 控制 (map)
    ax = axes[1, 0]
    ax.set_title("Control (Map Frame)")
    ax.set_xlabel("Frame"); ax.set_ylabel("Velocity"); ax.grid(True)
    um_arr = np.array([f["u_opt_map"] for f in frames if "u_opt_map" in f])
    if len(um_arr) > 0:
        ax.plot(um_arr[:, 0], "r-", linewidth=1, alpha=0.8, label="vx_map")
        ax.plot(um_arr[:, 1], "g-", linewidth=1, alpha=0.8, label="vy_map")
        ax.plot(um_arr[:, 2], "b-", linewidth=1, alpha=0.8, label="ω_map")
        ax.legend(fontsize=8)

    # (1,1) 求解时间
    ax = axes[1, 1]
    ax.set_title("Solve Time")
    ax.set_xlabel("Frame"); ax.set_ylabel("Time [ms]"); ax.grid(True)
    st_arr = [f["solve_time"] * 1000 for f in frames if "solve_time" in f]
    if len(st_arr) > 0:
        ax.plot(st_arr, "m-", linewidth=1, alpha=0.8)
        ax.axhline(y=np.mean(st_arr), color="r", linestyle="--",
                   label=f"mean={np.mean(st_arr):.2f} ms")
        ax.legend(fontsize=8)

    # (1,2) 路径点数
    ax = axes[1, 2]
    ax.set_title("Path Point Counts")
    ax.set_xlabel("Frame"); ax.set_ylabel("Count"); ax.grid(True)
    lp_counts = [f.get("local_plan_n", 0) for f in frames]
    rt_counts = [f.get("ref_traj_n", 0) for f in frames]
    ax.plot(lp_counts, "k-", linewidth=1, alpha=0.8, label="Local Plan pts")
    ax.plot(rt_counts, "orange", linestyle="--", linewidth=1, alpha=0.8,
            label="Ref Traj pts")
    ax.legend(fontsize=8)

    fig.tight_layout()

    if out_dir:
        out_path = os.path.join(out_dir, "overview.png")
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        fig.savefig(out_path, dpi=150)
        print(f"  已保存: {out_path}")
    else:
        plt.show()
    plt.close(fig)


# ──────────────────────────────────────────────
#  Main
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="可视化 MPC 调试日志")
    parser.add_argument("log_file", nargs="?", type=str,
                        default=os.path.join(
                            os.path.dirname(__file__),
                            "/home/ld/guganav/src/guga_controller/"
                            "mpc_controller/tmp/mpc_debug_log.txt"),
                        help="调试日志文件路径")
    parser.add_argument("--frame", "-f", type=int, default=None,
                        help="指定要详细显示的帧编号 (默认: 最后一帧)")
    parser.add_argument("--save", "-s", type=str, default=None,
                        help="保存图表到指定目录 (默认: 显示窗口)")
    parser.add_argument("--video", "-v", type=str, default=None,
                        help="将所有帧合成为 MP4 视频, 输出到指定路径")
    parser.add_argument("--fps", type=int, default=10,
                        help="视频帧率 (默认: 10)")
    parser.add_argument("--workers", "-w", type=int,
                        default=max(1, cpu_count() - 1),
                        help=f"并行 worker 数 (默认: CPU 核数-1={max(1, cpu_count() - 1)})")
    parser.add_argument("--nvenc", action="store_true",
                        help="使用 NVIDIA GPU 硬件编码 (需要 nvidia 驱动)")
    args = parser.parse_args()

    if not os.path.exists(args.log_file):
        print(f"[错误] 日志文件不存在: {args.log_file}")
        sys.exit(1)

    print(f"正在解析: {args.log_file}")
    frames = parse_log(args.log_file)
    print(f"  共解析 {len(frames)} 帧")

    if len(frames) == 0:
        print("[错误] 没有解析到任何数据")
        sys.exit(1)

    # ── 视频模式 ──
    if args.video:
        if not shutil.which("ffmpeg"):
            print("[错误] 未找到 ffmpeg, 请先安装: sudo apt install ffmpeg")
            sys.exit(1)

        video_path = args.video
        video_dir = os.path.dirname(video_path) or "."
        os.makedirs(video_dir, exist_ok=True)

        # 持久化临时目录 (让 ffmpeg 可直接读取已渲染的 PNG)
        tmpdir = tempfile.mkdtemp(prefix="mpc_frames_")
        # 注册退出清理
        def _cleanup():
            if os.path.isdir(tmpdir):
                shutil.rmtree(tmpdir, ignore_errors=True)
        atexit.register(_cleanup)

        try:
            print(f"帧缓存目录: {tmpdir}")

            # 并行渲染
            completed, failed = render_frames_parallel(
                frames, tmpdir, max_workers=args.workers)

            if completed == 0:
                print("[错误] 没有成功渲染任何帧")
                sys.exit(1)

            print(f"  渲染完成: {completed} 成功, {failed} 失败")

            # 追加概览 + 状态跟踪图到视频末尾
            last_fid = max(f.get("frame_id", i)
                           for i, f in enumerate(frames))
            extra_frames = [
                ("overview",         lambda d: plot_overview(frames, out_dir=d)),
                ("state_tracking",   lambda d: plot_state_tracking(frames, out_dir=d)),
            ]
            for name, render_fn in extra_frames:
                last_fid += 1
                out_path = os.path.join(tmpdir,
                                        f"frame_{last_fid:06d}.png")
                try:
                    render_fn(tmpdir)
                    # 重命名生成的文件到统一命名序列
                    generated = os.path.join(tmpdir, f"{name}.png")
                    if os.path.exists(generated):
                        os.rename(generated, out_path)
                        print(f"  追加总结帧: {out_path}")
                except Exception as exc:
                    print(f"  总结帧 {name} 渲染失败: {exc}")

            # ffmpeg 合成
            print("合成视频...")
            if frames_to_video(tmpdir, video_path, args.fps, nvenc=args.nvenc):
                size_mb = os.path.getsize(video_path) / (1024 * 1024)
                print(f"✓ 视频已生成: {video_path} ({size_mb:.1f} MB)")
            else:
                sys.exit(1)
        finally:
            _cleanup()
        return

    # ── 交互 / 保存模式 ──
    if args.frame is not None:
        matches = [f for f in frames if f["frame_id"] == args.frame]
        if not matches:
            print(f"[错误] 帧 {args.frame} 不存在 "
                  f"(范围: 0-{frames[-1]['frame_id']})")
            sys.exit(1)
        target = matches[0]
    else:
        target = frames[-1]

    save_dir = args.save

    print(f"\n绘制单帧详情 — Frame {target['frame_id']} ...")
    plot_single_frame(target, out_dir=save_dir)

    if len(frames) > 1:
        print(f"\n绘制概览 — 共 {len(frames)} 帧 ...")
        plot_overview(frames, out_dir=save_dir)
        print(f"\n绘制状态跟踪对比 — 共 {len(frames)} 帧 ...")
        plot_state_tracking(frames, out_dir=save_dir)

    if save_dir is None:
        print("\n完成. 关闭图表窗口退出.")
        # matplotlib.use("TkAgg") 已在文件开头, 这里直接 show
        import matplotlib.pyplot as plt
        plt.show()
    else:
        print(f"\n完成. 图表已保存至: {save_dir}")


if __name__ == "__main__":
    main()
