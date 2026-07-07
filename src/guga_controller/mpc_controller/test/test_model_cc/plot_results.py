#!/usr/bin/env python3
"""
plot_results.py — 读取 test_unicycle_solver 输出的 CSV 数据并绘制图表。

用法:
    cd test_model_cc/build && ./test_unicycle_solver
    python3 ../plot_results.py
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os
import sys
import glob


def load_csv(path):
    """读取 CSV 返回 (header_list, data_2d_array)."""
    with open(path, "r") as f:
        header = f.readline().strip().split(",")
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if data.ndim == 1:
        data = data.reshape(-1, 1)
    return header, data


def load_params(path):
    """读取 param,value CSV 返回 dict."""
    p = {}
    with open(path, "r") as f:
        f.readline()
        for line in f:
            line = line.strip()
            if not line:
                continue
            kv = line.split(",", 1)
            if len(kv) < 2:
                continue
            try:
                p[kv[0]] = float(kv[1])
            except ValueError:
                p[kv[0]] = kv[1]
    return p


# ==============================================================================
def plot_open_loop(data_dir, out_dir):
    """开环模型验证：固定控制下的 unicycle 轨迹."""
    tfile = os.path.join(data_dir, "open_loop_traj.csv")
    pfile = os.path.join(data_dir, "open_loop_params.csv")
    if not os.path.exists(tfile) or not os.path.exists(pfile):
        print("  [SKIP] open_loop files not found")
        return

    _, d = load_csv(tfile)
    p = load_params(pfile)
    dt = p.get("dt", 0.05)
    n = d.shape[0]
    t = np.arange(n) * dt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Open-Loop Model: Unicycle RK4 (v=%.1f, ω=%.1f)" %
                 (p.get("u_v", 0), p.get("u_omega", 0)), fontsize=13)

    ax = axes[0, 0]
    ax.plot(d[:, 0], d[:, 1], "b-o", ms=4, label="RK4")
    ax.plot(d[0, 0], d[0, 1], "ro", ms=8, label="Start")
    ax.plot(d[-1, 0], d[-1, 1], "r*", ms=12, label="End")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("XY Trajectory")
    ax.legend(); ax.axis("equal"); ax.grid(True)

    for i, lab in enumerate(["px [m]", "py [m]", "theta [rad]"]):
        ax = axes[0, 1] if i == 0 else (axes[1, 0] if i == 1 else axes[1, 1])
        ax.plot(t, d[:, i], "b-", lw=1.5)
        ax.set_xlabel("time [s]"); ax.set_ylabel(lab)
        ax.set_title(lab); ax.grid(True)

    plt.tight_layout()
    path = os.path.join(out_dir, "open_loop_trajectory.png")
    plt.savefig(path, dpi=150)
    print(f"  Saved {path}")
    plt.close()


# ==============================================================================
def plot_closed_loop(data_dir, out_dir, tag):
    """闭环 MPC 仿真 6 合 1 图."""
    xf = os.path.join(data_dir, f"x_history_{tag}.csv")
    uf = os.path.join(data_dir, f"u_history_{tag}.csv")
    rf = os.path.join(data_dir, f"ref_history_{tag}.csv")
    pf = os.path.join(data_dir, f"sim_params_{tag}.csv")
    if not all(os.path.exists(f) for f in [xf, uf, rf, pf]):
        print(f"  [SKIP] {tag} files incomplete")
        return

    _, xd = load_csv(xf)
    _, ud = load_csv(uf)
    _, rd = load_csv(rf)
    p = load_params(pf)
    dt = p.get("dt", 0.05)
    n_sim = int(p.get("n_sim_steps", rd.shape[0]))
    n = xd.shape[0]
    t_state = np.arange(n) * dt
    t_ctrl = np.arange(n_sim) * dt

    fig, axes = plt.subplots(2, 3, figsize=(17, 9))
    titles = {"circle": "Circle (r=2.0m)", "figure8": "Figure-8 (2×1.5m)"}
    fig.suptitle("MPC Closed-Loop: " + titles.get(tag, tag), fontsize=13)

    # XY
    ax = axes[0, 0]
    # 生成完整参考
    t_full = np.arange(0, 10.0, 0.01)
    rf_full = np.zeros((len(t_full), 2))
    if tag == "circle":
        r, w = 2.0, 0.8
        rf_full[:, 0] = r * np.cos(w * t_full)
        rf_full[:, 1] = r * np.sin(w * t_full)
    else:
        ax_f8, ay, w = 2.0, 1.5, 0.6
        rf_full[:, 0] = ax_f8 * np.sin(w * t_full)
        rf_full[:, 1] = ay * np.sin(2.0 * w * t_full)
    ax.plot(rf_full[:, 0], rf_full[:, 1], "g-", alpha=0.35, lw=1.5, label="Reference")
    ax.plot(xd[:, 0], xd[:, 1], "b-", lw=1.2, label="MPC")
    ax.plot(xd[0, 0], xd[0, 1], "ro", ms=6, label="Start")
    ax.plot(xd[-1, 0], xd[-1, 1], "r*", ms=10, label="End")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("XY Trajectory"); ax.legend(fontsize=8)
    ax.axis("equal"); ax.grid(True)

    # 状态
    for i, lab in enumerate(["px [m]", "py [m]", "theta [rad]"]):
        ax = axes[0, 1] if i == 0 else (axes[0, 2] if i == 1 else axes[1, 0])
        ax.plot(t_state, xd[:, i], "b-", lw=1.2, label="MPC")
        ax.plot(t_ctrl, rd[:, i], "r--", lw=1.0, label="Ref")
        ax.set_xlabel("time [s]"); ax.set_ylabel(lab)
        ax.set_title(lab); ax.legend(fontsize=8); ax.grid(True)

    # v
    ax = axes[1, 1]
    ax.plot(t_ctrl, ud[:, 0], "b-", lw=1.0, label="v")
    ax.axhline(2.5, color="r", ls="--", alpha=0.4, lw=0.8)
    ax.axhline(-2.5, color="r", ls="--", alpha=0.4, lw=0.8)
    ax.set_xlabel("time [s]"); ax.set_ylabel("v [m/s]")
    ax.set_title("Control: v"); ax.legend(fontsize=8); ax.grid(True)

    # omega
    ax = axes[1, 2]
    ax.plot(t_ctrl, ud[:, 1], "b-", lw=1.0, label="ω")
    ax.axhline(4.0, color="r", ls="--", alpha=0.4, lw=0.8)
    ax.axhline(-4.0, color="r", ls="--", alpha=0.4, lw=0.8)
    ax.set_xlabel("time [s]"); ax.set_ylabel("ω [rad/s]")
    ax.set_title("Control: ω"); ax.legend(fontsize=8); ax.grid(True)

    plt.tight_layout()
    path = os.path.join(out_dir, f"closed_loop_{tag}.png")
    plt.savefig(path, dpi=150)
    print(f"  Saved {path}")
    plt.close()


# ==============================================================================
def plot_solve_times(data_dir, out_dir):
    """求解时间直方图."""
    tags = ["circle", "figure8"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    fig.suptitle("MPC Solve Time Distribution", fontsize=13)

    for idx, tag in enumerate(tags):
        tf = os.path.join(data_dir, f"solve_times_{tag}.csv")
        if not os.path.exists(tf):
            continue
        _, td = load_csv(tf)
        tms = td.flatten()
        ax = axes[idx]
        ax.hist(tms, bins=30, color="steelblue", edgecolor="white", alpha=0.85)
        ax.axvline(np.mean(tms), color="r", ls="--", lw=1.5,
                   label=f"mean={np.mean(tms):.3f} ms")
        ax.set_xlabel("solve time [ms]"); ax.set_ylabel("count")
        ax.set_title(f"Solve Times: {tag}")
        ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, "solve_times_hist.png")
    plt.savefig(path, dpi=150)
    print(f"  Saved {path}")
    plt.close()


# ==============================================================================
def plot_error_over_time(data_dir, out_dir):
    """位置跟踪误差随时间变化."""
    tags = ["circle", "figure8"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    fig.suptitle("Tracking Error Over Time", fontsize=13)

    for idx, tag in enumerate(tags):
        xf = os.path.join(data_dir, f"x_history_{tag}.csv")
        rf = os.path.join(data_dir, f"ref_history_{tag}.csv")
        pf = os.path.join(data_dir, f"sim_params_{tag}.csv")
        if not os.path.exists(xf) or not os.path.exists(rf):
            continue
        _, xd = load_csv(xf)
        _, rd = load_csv(rf)
        p = load_params(pf)
        dt = p.get("dt", 0.05)
        n_sim = rd.shape[0]
        t_ctrl = np.arange(n_sim) * dt
        err = np.sqrt((xd[1:, 0] - rd[:, 0])**2 + (xd[1:, 1] - rd[:, 1])**2)

        ax = axes[idx]
        ax.plot(t_ctrl, err, "b-", lw=1.0)
        ax.axhline(np.mean(err), color="r", ls="--", lw=1.2,
                   label=f"mean={np.mean(err):.4f} m")
        ax.set_xlabel("time [s]"); ax.set_ylabel("position error [m]")
        ax.set_title(f"Tracking Error: {tag}")
        ax.legend(fontsize=9); ax.grid(True)

    plt.tight_layout()
    path = os.path.join(out_dir, "tracking_error.png")
    plt.savefig(path, dpi=150)
    print(f"  Saved {path}")
    plt.close()


# ==============================================================================
def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if len(sys.argv) > 1:
        data_dir = sys.argv[1]
    else:
        data_dir = os.path.join(script_dir, "data")

    out_dir = data_dir

    if not os.path.isdir(data_dir):
        print(f"Error: data directory not found: {data_dir}")
        print(f"Usage: python3 {sys.argv[0]} [data_dir]")
        sys.exit(1)

    csvs = glob.glob(os.path.join(data_dir, "*.csv"))
    if not csvs:
        print(f"No CSV files in {data_dir}. Run ./test_unicycle_solver first.")
        sys.exit(0)

    print("=" * 60)
    print("Plotting acados MPC Simulation Results")
    print(f"  Data:   {data_dir}")
    print(f"  Output: {out_dir}")
    print("=" * 60)

    plot_open_loop(data_dir, out_dir)
    plot_closed_loop(data_dir, out_dir, "circle")
    plot_closed_loop(data_dir, out_dir, "figure8")
    plot_solve_times(data_dir, out_dir)
    plot_error_over_time(data_dir, out_dir)

    print("\nDone! All plots saved to: " + out_dir)


if __name__ == "__main__":
    main()
