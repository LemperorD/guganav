#!/usr/bin/env python3
"""
plot_results.py — 读取 test_unicycle_solver 输出的 CSV 数据并绘制图表。

用法:
    cd test_model_cc/build && ./test_unicycle_solver
    python3 ../plot_results.py          # 默认读取 build/data/
    python3 ../plot_results.py data/    # 显式指定数据目录
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")  # 无头模式
import matplotlib.pyplot as plt
import os
import sys
import glob


def load_csv(path):
    """读取 CSV 并返回 (header_names, data as 2D numpy array)."""
    with open(path, "r") as f:
        header = f.readline().strip().split(",")
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if data.ndim == 1:
        data = data.reshape(-1, 1)
    return header, data


def plot_open_loop(data_dir, out_dir):
    """开环模型验证图: unicycle 在固定控制输入下的轨迹."""
    traj_file = os.path.join(data_dir, "open_loop_traj.csv")
    params_file = os.path.join(data_dir, "open_loop_params.csv")

    if not os.path.exists(traj_file):
        print(f"  [SKIP] {traj_file} not found")
        return
    if not os.path.exists(params_file):
        print(f"  [SKIP] {params_file} not found")
        return

    header, data = load_csv(traj_file)
    params_h, params_d = load_csv(params_file)

    # 提取参数
    params = {}
    for row in params_d:
        params[str(row[0])] = float(row[1])

    dt = params.get("dt", 0.05)
    n_steps = int(params.get("n_steps", 20))
    t = np.arange(n_steps + 1) * dt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Open-Loop Model Verification: Unicycle RK4", fontsize=13)

    # XY 轨迹
    ax = axes[0, 0]
    ax.plot(data[:, 0], data[:, 1], "b-o", ms=4, label="RK4")
    ax.plot(data[0, 0], data[0, 1], "ro", ms=8, label="Start")
    ax.plot(data[-1, 0], data[-1, 1], "r*", ms=12, label="End")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("XY Trajectory (v=%.1f, ω=%.1f)" % (params.get("u_v", 0), params.get("u_omega", 0)))
    ax.legend()
    ax.axis("equal")
    ax.grid(True)

    # 状态
    labels = ["px [m]", "py [m]", "theta [rad]"]
    for i in range(3):
        ax = axes[0, 1] if i == 0 else (axes[1, 0] if i == 1 else axes[1, 1])
        ax.plot(t, data[:, i], "b-", lw=1.5)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(labels[i])
        ax.set_title(labels[i])
        ax.grid(True)

    plt.tight_layout()
    path = os.path.join(out_dir, "open_loop_trajectory.png")
    plt.savefig(path, dpi=150)
    print(f"  Saved: {path}")
    plt.close()


def plot_closed_loop(data_dir, out_dir, tag):
    """闭环 MPC 仿真综合图: 6 合 1."""
    x_file  = os.path.join(data_dir, f"x_history_{tag}.csv")
    u_file  = os.path.join(data_dir, f"u_history_{tag}.csv")
    ref_file = os.path.join(data_dir, f"ref_history_{tag}.csv")
    time_file = os.path.join(data_dir, f"solve_times_{tag}.csv")
    params_file = os.path.join(data_dir, f"sim_params_{tag}.csv")

    for fpath in [x_file, u_file, ref_file, time_file, params_file]:
        if not os.path.exists(fpath):
            print(f"  [SKIP] {fpath} not found")
            return

    x_head, x_data   = load_csv(x_file)
    u_head, u_data   = load_csv(u_file)
    ref_head, ref_data = load_csv(ref_file)
    t_head, t_data   = load_csv(time_file)
    p_head, p_data   = load_csv(params_file)

    params = {}
    for row in p_data:
        params[str(row[0])] = float(row[1])

    dt = params.get("dt", 0.05)
    n_sim = int(params.get("n_sim_steps", 180))

    t_sim = np.arange(n_sim + 1) * dt
    t_ctrl = np.arange(n_sim) * dt

    # ---- 绘图 ----
    fig, axes = plt.subplots(2, 3, figsize=(17, 9))
    title_map = {"circle": "Circle (r=2.0m)", "figure8": "Figure-8 (2×1.5m)"}
    fig.suptitle("MPC Closed-Loop: " + title_map.get(tag, tag), fontsize=13)

    # [0,0] XY 轨迹
    ax = axes[0, 0]
    # 生成完整参考轨迹用于绘制
    t_full = np.arange(0, 10.0, 0.01)
    ref_full = np.zeros((len(t_full), 3))
    for i, ti in enumerate(t_full):
        # 重用 C 代码的轨迹生成逻辑 — 这里用 Python 复现
        if tag == "circle":
            r, w = 2.0, 0.8
            ref_full[i, 0] = r * np.cos(w * ti)
            ref_full[i, 1] = r * np.sin(w * ti)
            ref_full[i, 2] = w * ti + np.pi / 2
        else:
            ax_f8, ay, w = 2.0, 1.5, 0.6
            ref_full[i, 0] = ax_f8 * np.sin(w * ti)
            ref_full[i, 1] = ay * np.sin(2.0 * w * ti)
            eps = 0.001
            ref_full[i, 2] = np.arctan2(
                ay * np.sin(2.0*w*(ti+eps)) - ay * np.sin(2.0*w*(ti-eps)),
                ax_f8 * np.sin(w*(ti+eps)) - ax_f8 * np.sin(w*(ti-eps)))

    ax.plot(ref_full[:, 0], ref_full[:, 1], "g-", alpha=0.35, lw=1.5, label="Reference")
    ax.plot(x_data[:, 0], x_data[:, 1], "b-", lw=1.2, label="MPC")
    ax.plot(x_data[0, 0], x_data[0, 1], "ro", ms=6, label="Start")
    ax.plot(x_data[-1, 0], x_data[-1, 1], "r*", ms=10, label="End")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("XY Trajectory")
    ax.legend(fontsize=8)
    ax.axis("equal")
    ax.grid(True)

    # [0,1] px & [0,2] py & [1,0] theta
    state_labels = ["px [m]", "py [m]", "theta [rad]"]
    for i in range(3):
        ax = axes[0, 1] if i == 0 else (axes[0, 2] if i == 1 else axes[1, 0])
        ax.plot(t_sim, x_data[:, i], "b-", lw=1.2, label="MPC")
        ax.plot(t_ctrl, ref_data[:, i], "r--", lw=1.0, label="Ref")
        ax.set_xlabel("time [s]")
        ax.set_ylabel(state_labels[i])
        ax.set_title(state_labels[i])
        ax.legend(fontsize=8)
        ax.grid(True)

    # [1,1] v
    ax = axes[1, 1]
    ax.plot(t_ctrl, u_data[:, 0], "b-", lw=1.0, label="v")
    ax.axhline(y=2.5, color="r", ls="--", alpha=0.4, lw=0.8)
    ax.axhline(y=-2.5, color="r", ls="--", alpha=0.4, lw=0.8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("v [m/s]")
    ax.set_title("Control: v")
    ax.legend(fontsize=8)
    ax.grid(True)

    # [1,2] omega
    ax = axes[1, 2]
    ax.plot(t_ctrl, u_data[:, 1], "b-", lw=1.0, label="ω")
    ax.axhline(y=4.0, color="r", ls="--", alpha=0.4, lw=0.8)
    ax.axhline(y=-4.0, color="r", ls="--", alpha=0.4, lw=0.8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("ω [rad/s]")
    ax.set_title("Control: ω")
    ax.legend(fontsize=8)
    ax.grid(True)

    plt.tight_layout()
    path = os.path.join(out_dir, f"closed_loop_{tag}.png")
    plt.savefig(path, dpi=150)
    print(f"  Saved: {path}")
    plt.close()


def plot_solve_times(data_dir, out_dir):
    """合并两条轨迹的求解时间直方图."""
    tags = ["circle", "figure8"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    fig.suptitle("MPC Solve Time Distribution", fontsize=13)

    for idx, tag in enumerate(tags):
        time_file = os.path.join(data_dir, f"solve_times_{tag}.csv")
        if not os.path.exists(time_file):
            continue
        _, t_data = load_csv(time_file)
        times = t_data.flatten()

        ax = axes[idx]
        ax.hist(times, bins=30, color="steelblue", edgecolor="white", alpha=0.85)
        ax.axvline(x=np.mean(times), color="red", ls="--", lw=1.5,
                   label=f"mean={np.mean(times):.3f} ms")
        ax.set_xlabel("solve time [ms]")
        ax.set_ylabel("count")
        ax.set_title(f"Solve Times: {tag}")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, "solve_times_hist.png")
    plt.savefig(path, dpi=150)
    print(f"  Saved: {path}")
    plt.close()


def plot_error_over_time(data_dir, out_dir):
    """位置误差随时间的演化."""
    tags = ["circle", "figure8"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    fig.suptitle("Tracking Error Over Time", fontsize=13)

    for idx, tag in enumerate(tags):
        x_file = os.path.join(data_dir, f"x_history_{tag}.csv")
        ref_file = os.path.join(data_dir, f"ref_history_{tag}.csv")
        params_file = os.path.join(data_dir, f"sim_params_{tag}.csv")
        if not os.path.exists(x_file) or not os.path.exists(ref_file):
            continue

        _, x_data = load_csv(x_file)
        _, ref_data = load_csv(ref_file)
        _, p_data = load_csv(params_file)
        dt = 0.05
        for row in p_data:
            if str(row[0]) == "dt":
                dt = float(row[1])

        n_sim = ref_data.shape[0]
        t_ctrl = np.arange(n_sim) * dt

        pos_err = np.sqrt(
            (x_data[1:, 0] - ref_data[:, 0])**2 +
            (x_data[1:, 1] - ref_data[:, 1])**2
        )

        ax = axes[idx]
        ax.plot(t_ctrl, pos_err, "b-", lw=1.0)
        ax.axhline(y=np.mean(pos_err), color="r", ls="--", lw=1.2,
                   label=f"mean={np.mean(pos_err):.4f} m")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("position error [m]")
        ax.set_title(f"Tracking Error: {tag}")
        ax.legend(fontsize=9)
        ax.grid(True)

    plt.tight_layout()
    path = os.path.join(out_dir, "tracking_error.png")
    plt.savefig(path, dpi=150)
    print(f"  Saved: {path}")
    plt.close()


def main():
    # 解析数据目录
    if len(sys.argv) > 1:
        data_dir = sys.argv[1]
    else:
        # 默认: 脚本所在目录下的 build/data/
        script_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(script_dir, "build", "data")

    out_dir = data_dir  # 图片输出到同一目录

    if not os.path.isdir(data_dir):
        print(f"Error: data directory not found: {data_dir}")
        print("Usage: python3 plot_results.py [data_dir]")
        sys.exit(1)

    print("=" * 60)
    print("Plotting acados MPC Simulation Results")
    print(f"  Data: {data_dir}")
    print(f"  Output: {out_dir}")
    print("=" * 60)

    csv_files = glob.glob(os.path.join(data_dir, "*.csv"))
    if not csv_files:
        print(f"  No CSV files found in {data_dir}")
        print("  Run ./test_unicycle_solver first.")
        sys.exit(0)

    plot_open_loop(data_dir, out_dir)
    plot_closed_loop(data_dir, out_dir, "circle")
    plot_closed_loop(data_dir, out_dir, "figure8")
    plot_solve_times(data_dir, out_dir)
    plot_error_over_time(data_dir, out_dir)

    print("\nDone! All plots saved to: " + out_dir)


if __name__ == "__main__":
    main()
