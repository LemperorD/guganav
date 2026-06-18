#!/usr/bin/env python3
"""
MPC 路径跟踪 — Model A vs Model B 对比仿真。

在圆形/八字/S 形三条轨迹上同时运行两个模型，生成对比图。

Model A: 世界系逐点跟踪 (现有, 3D 状态 + 6D 增广)
Model B: 7D 增广误差状态 + 终端代价 (新增)
"""

import os
import sys
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# CJK font
import matplotlib.font_manager as fm
_cjk_font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
fm.fontManager.addfont(_cjk_font_path)
_fp = fm.FontProperties(fname=_cjk_font_path)
plt.rcParams["font.family"] = _fp.get_name()
plt.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(__file__))
from mpc_solver import MpcSolver as ModelASolver
from mpc_solver_augmented import AugmentedMpcSolver as ModelBSolver


# ==========================================================================
# 运动学仿真（RK4）
# ==========================================================================
def omni_rk4(x, u, dt):
    def f(xk):
        th = xk[2]
        ct, st = np.cos(th), np.sin(th)
        return np.array([u[0]*ct - u[1]*st, u[0]*st + u[1]*ct, u[2]])
    k1 = f(x); k2 = f(x+0.5*dt*k1); k3 = f(x+0.5*dt*k2); k4 = f(x+dt*k3)
    return x + (dt/6.0)*(k1+2*k2+2*k3+k4)


# ==========================================================================
# 参考轨迹生成
# ==========================================================================
def make_circle(t, radius=2.0, speed=0.8):
    wr = speed / radius
    x = radius * np.cos(wr * t)
    y = radius * np.sin(wr * t)
    th = wr * t + np.pi / 2.0
    return (np.column_stack([x, y, th]),
            np.column_stack([np.full_like(t, speed), np.zeros_like(t), np.full_like(t, wr)]))


def make_figure8(t, a=2.0, w=0.35):
    s = w * t
    x, y = a * np.sin(s), a * 0.5 * np.sin(2.0 * s)
    dx, dy = a * w * np.cos(s), a * w * np.cos(2.0 * s)
    ddx, ddy = -a * w**2 * np.sin(s), -2.0 * a * w**2 * np.sin(2.0 * s)
    spd = np.sqrt(dx**2 + dy**2)
    curv = (dx * ddy - dy * ddx) / (spd**3 + 1e-9)
    th = np.arctan2(dy, dx)
    vx = np.clip(spd, -3.0, 3.0)
    w_ref = np.clip(spd * curv, -6.0, 6.0)
    return (np.column_stack([x, y, th]),
            np.column_stack([vx, np.zeros_like(s), w_ref]))


def make_s_curve(t, duration=15.0):
    T = duration
    v0, y_lc = 0.8, 2.0
    x = v0 * t
    t1, t2 = T * 0.3, T * 0.7
    y = np.zeros_like(t)
    for i, ti in enumerate(t):
        if ti < t1: y[i] = 0.0
        elif ti < t2:
            tau = (ti - t1) / (t2 - t1)
            y[i] = y_lc * (10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5)
        else: y[i] = y_lc
    dt_avg = np.mean(np.diff(t))
    dx, dy = np.gradient(x, dt_avg), np.gradient(y, dt_avg)
    ddx, ddy = np.gradient(dx, dt_avg), np.gradient(dy, dt_avg)
    spd = np.sqrt(dx**2 + dy**2)
    curv = (dx * ddy - dy * ddx) / (spd**3 + 1e-9)
    th = np.arctan2(dy, dx)
    w_ref = np.clip(v0 * curv, -6.0, 6.0)
    return (np.column_stack([x, y, th]),
            np.column_stack([np.full_like(t, v0), np.zeros_like(t), w_ref]))


# ==========================================================================
# 仿真循环
# ==========================================================================
def run_model_a(solver, ref_s, ref_u, ts, x0):
    """Model A 闭环仿真。"""
    Nt = len(ts); Nh = solver.N
    x = np.array(x0, dtype=float)
    st = np.zeros((Nt, 3))
    ct = np.zeros((Nt, 3))
    ce = np.zeros(Nt)
    he = np.zeros(Nt)
    tm = np.zeros(Nt)
    solver.set_initial_control([0.0, 0.0, 0.0])
    for i in range(Nt):
        st[i] = x.copy()
        ri, re = min(i, Nt-1), min(i+Nh, Nt)
        rs, ru = ref_s[ri:re], ref_u[ri:re]
        if rs.shape[0] < Nh:
            pad = Nh - rs.shape[0]
            rs = np.vstack([rs, np.tile(rs[-1:], (pad, 1))])
            ru = np.vstack([ru, np.tile(ru[-1:], (pad, 1))])
        t0 = time.perf_counter()
        u = solver.solve(x, rs, ru)
        tm[i] = (time.perf_counter() - t0) * 1000.0
        ct[i] = u
        rsi = ref_s[min(i, Nt-1)]
        dx, dy = x[0]-rsi[0], x[1]-rsi[1]
        cs, sn = np.cos(rsi[2]), np.sin(rsi[2])
        ce[i] = -sn*dx + cs*dy
        he[i] = (x[2]-rsi[2] + np.pi) % (2*np.pi) - np.pi
        if i < Nt-1: x = omni_rk4(x, u, ts[i+1]-ts[i])
    return dict(states=st, ctls=ct, cte=ce, he=he, tm=tm, ref_s=ref_s, ref_u=ref_u, t=ts)


def run_model_b(solver, ref_s, ref_u, ts, x0, s0=0.0):
    """Model B 闭环仿真（Frenet 误差系）。"""
    Nt = len(ts); Nh = solver.N
    x = np.array(x0, dtype=float)
    s_cur = s0
    st = np.zeros((Nt, 3))
    ct = np.zeros((Nt, 3))
    ce = np.zeros(Nt)
    he = np.zeros(Nt)
    tm = np.zeros(Nt)
    solver.set_initial_control([0.0, 0.0, 0.0])
    # 预计算弧长数组
    arc = np.zeros(len(ts))
    for i in range(1, len(ts)):
        dx = ref_s[i, 0] - ref_s[i-1, 0]
        dy = ref_s[i, 1] - ref_s[i-1, 1]
        arc[i] = arc[i-1] + np.hypot(dx, dy)

    for i in range(Nt):
        st[i] = x.copy()
        s_cur = arc[i]  # 使用当前时刻的弧长
        ri, re = min(i, Nt-1), min(i+Nh, Nt)
        rs, ru = ref_s[ri:re], ref_u[ri:re]
        if rs.shape[0] < Nh:
            pad = Nh - rs.shape[0]
            rs = np.vstack([rs, np.tile(rs[-1:], (pad, 1))])
            ru = np.vstack([ru, np.tile(ru[-1:], (pad, 1))])
        t0 = time.perf_counter()
        u = solver.solve(x, rs, ru)
        tm[i] = (time.perf_counter() - t0) * 1000.0
        ct[i] = u
        rsi = ref_s[min(i, Nt-1)]
        dx, dy = x[0]-rsi[0], x[1]-rsi[1]
        cs, sn = np.cos(rsi[2]), np.sin(rsi[2])
        ce[i] = -sn*dx + cs*dy
        he[i] = (x[2]-rsi[2] + np.pi) % (2*np.pi) - np.pi
        if i < Nt-1:
            dt_actual = ts[i+1]-ts[i]
            x = omni_rk4(x, u, dt_actual)
            s_cur += u[0] * dt_actual  # 弧长由 vx 推进
    return dict(states=st, ctls=ct, cte=ce, he=he, tm=tm, ref_s=ref_s, ref_u=ref_u, t=ts)


# ==========================================================================
# 可视化
# ==========================================================================
def plot_comparison(res_a, res_b, title, save_path):
    """绘制 Model A vs B 的 2x3 对比图。"""
    fig, axes = plt.subplots(3, 2, figsize=(14, 14))
    fig.suptitle(title + " — Model A (左) vs Model B (右)", fontsize=14, fontweight="bold")

    colors = ["C0", "C1", "C2"]

    def plot_one(ax, res, label_prefix):
        t_ = res["t"]
        ax.plot(res["ref_s"][:, 0], res["ref_s"][:, 1], "k--", lw=1.0, alpha=0.5, label="参考")
        ax.plot(res["states"][:, 0], res["states"][:, 1], "C0-", lw=1.2, label="实际")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.set_aspect("equal"); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
        rms = np.sqrt(np.mean(res["cte"]**2)); mx = np.max(np.abs(res["cte"]))
        ax.set_title(f"{label_prefix} | RMS={rms:.4f}m, Max={mx:.4f}m", fontsize=9)

    plot_one(axes[0, 0], res_a, "Model A"); plot_one(axes[0, 1], res_b, "Model B")

    # Row 1: lateral error
    for j, (res, lbl) in enumerate([(res_a, "Model A"), (res_b, "Model B")]):
        ax = axes[1, j]
        ax.plot(res["t"], res["cte"], "C0-", lw=1.5, label="横向误差")
        ax.plot(res["t"], res["he"], "C1-", lw=1.0, alpha=0.7, label="航向误差")
        ax.axhline(0, color="k", lw=0.5)
        ax.set_xlabel("t [s]"); ax.set_ylabel("误差 [m / rad]")
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
        ax.set_title(f"{lbl} 跟踪误差", fontsize=9)

    # Row 2: controls
    for j, (res, lbl) in enumerate([(res_a, "Model A"), (res_b, "Model B")]):
        ax = axes[2, j]
        ax.plot(res["t"], res["ctls"][:, 0], label="vx", lw=1.2)
        ax.plot(res["t"], res["ctls"][:, 1], label="vy", lw=1.2)
        ax.plot(res["t"], res["ctls"][:, 2], label="ω", lw=1.2)
        ax.set_xlabel("t [s]"); ax.set_ylabel("控制量")
        ax.legend(fontsize=7, ncol=3); ax.grid(True, alpha=0.3)
        ax.set_title(f"{lbl} 控制量", fontsize=9)

    fig.tight_layout()
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_summary_compare(all_results, labels, save_path):
    """汇总对比：每个场景一行，Model A vs B 的 CTE + 求解时间。"""
    n_scenes = len(all_results)
    fig, axes = plt.subplots(2, n_scenes, figsize=(6 * n_scenes, 8), squeeze=False)
    fig.suptitle("Model A vs Model B — 横向误差 + 求解时间汇总", fontsize=14, fontweight="bold")

    for i, (res_pair, lbl) in enumerate(zip(all_results, labels)):
        res_a, res_b = res_pair

        for j, (res, model_name, color, ls) in enumerate(
            [(res_a, "Model A", "C0", "-"), (res_b, "Model B", "C1", "--")]):

            # 横向误差
            ax = axes[0, i]
            ax.plot(res["t"], res["cte"], color=color, linestyle=ls, lw=1.5,
                    label=f"{model_name} (RMS={np.sqrt(np.mean(res['cte']**2)):.4f})")
            ax.axhline(0, color="k", lw=0.5)
            ax.set_title(f"{lbl}", fontsize=11)
            ax.set_xlabel("t [s]"); ax.set_ylabel("横向误差 [m]")
            ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

            # 求解时间
            ax = axes[1, i]
            ax.plot(res["t"], res["tm"], color=color, linestyle=ls, lw=1.2,
                    label=f"{model_name} (mean={np.mean(res['tm']):.2f}ms)")
            ax.set_xlabel("t [s]"); ax.set_ylabel("求解时间 [ms]")
            ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
            ax.set_ylim(bottom=0)

    fig.tight_layout()
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"汇总图已保存: {save_path}")


# ==========================================================================
# 主函数
# ==========================================================================
def main():
    print("=" * 60)
    print("MPC 路径跟踪 — Model A vs Model B 对比仿真")
    print("=" * 60)

    # ---- 公共参数 ----
    N = 15; dt = 0.05
    w_err = [10.0, 2.0]          # [w_elat, w_etheta]
    w_ctrl = [0.1, 0.1, 0.05]    # [w_vx, w_vy, w_omega]
    w_dctrl = [0.5, 0.5, 0.3]    # [w_dvx, w_dvy, w_domega]
    w_term = 20.0                 # 终端放大
    u_min = [-3.0, -3.0, -6.0]
    u_max = [3.0, 3.0, 6.0]

    # Model A 权重（复用 3D 状态 + 控制平滑 6D 参数化）
    Qa = [10.0, 10.0, 2.0]
    Ra = [0.1, 0.1, 0.05]
    Rda = [0.5, 0.5, 0.3]

    sim_dir = os.path.dirname(__file__) or "."
    all_results, labels = [], []

    # ---- 场景 1: 圆形 ----
    print("\n[场景 1: 圆形] 创建求解器...")
    solver_a = ModelASolver(N, dt, Qa, Ra, Rda, u_min, u_max)
    solver_b = ModelBSolver(N, dt, w_err, w_ctrl, w_dctrl, w_term, u_min, u_max)

    T, dt_sim = 20.0, 0.02
    ts = np.arange(0, T, dt_sim)
    rs, ru = make_circle(ts, radius=2.0, speed=0.8)
    x0 = np.array([2.2, 0.0, np.pi / 2.0])

    print("  运行 Model A...", end=" ", flush=True)
    ra = run_model_a(solver_a, rs, ru, ts, x0)
    print(f"RMS={np.sqrt(np.mean(ra['cte']**2)):.4f}m")

    print("  运行 Model B...", end=" ", flush=True)
    rb = run_model_b(solver_b, rs, ru, ts, x0)
    print(f"RMS={np.sqrt(np.mean(rb['cte']**2)):.4f}m")

    plot_comparison(ra, rb, "圆形轨迹 (R=2m, v=0.8m/s)",
                    os.path.join(sim_dir, "compare_circle.png"))
    all_results.append((ra, rb)); labels.append("圆形 (R=2m)")

    # 清理
    del solver_a, solver_b
    import gc; gc.collect()

    # ---- 场景 2: 八字形 ----
    print("\n[场景 2: 八字形] 创建求解器...")
    solver_a = ModelASolver(N, dt, Qa, Ra, Rda, u_min, u_max)
    solver_b = ModelBSolver(N, dt, w_err, w_ctrl, w_dctrl, w_term, u_min, u_max)

    T, dt_sim = 30.0, 0.02
    ts = np.arange(0, T, dt_sim)
    rs, ru = make_figure8(ts, a=2.0, w=0.35)
    x0 = np.array([0.3, 0.2, 0.0])

    print("  运行 Model A...", end=" ", flush=True)
    ra = run_model_a(solver_a, rs, ru, ts, x0)
    print(f"RMS={np.sqrt(np.mean(ra['cte']**2)):.4f}m")

    print("  运行 Model B...", end=" ", flush=True)
    rb = run_model_b(solver_b, rs, ru, ts, x0)
    print(f"RMS={np.sqrt(np.mean(rb['cte']**2)):.4f}m")

    plot_comparison(ra, rb, "八字形轨迹 (Lemniscate, a=2m)",
                    os.path.join(sim_dir, "compare_figure8.png"))
    all_results.append((ra, rb)); labels.append("八字形")

    del solver_a, solver_b; gc.collect()

    # ---- 场景 3: S 形换道 ----
    print("\n[场景 3: S形] 创建求解器...")
    solver_a = ModelASolver(N, dt, Qa, Ra, Rda, u_min, u_max)
    solver_b = ModelBSolver(N, dt, w_err, w_ctrl, w_dctrl, w_term, u_min, u_max)

    T, dt_sim = 15.0, 0.02
    ts = np.arange(0, T, dt_sim)
    rs, ru = make_s_curve(ts, duration=T)
    x0 = np.array([0.0, 0.1, 0.0])

    print("  运行 Model A...", end=" ", flush=True)
    ra = run_model_a(solver_a, rs, ru, ts, x0)
    print(f"RMS={np.sqrt(np.mean(ra['cte']**2)):.4f}m")

    print("  运行 Model B...", end=" ", flush=True)
    rb = run_model_b(solver_b, rs, ru, ts, x0)
    print(f"RMS={np.sqrt(np.mean(rb['cte']**2)):.4f}m")

    plot_comparison(ra, rb, "S形换道轨迹 (换道宽2m, v=0.8m/s)",
                    os.path.join(sim_dir, "compare_s_curve.png"))
    all_results.append((ra, rb)); labels.append("S形")

    del solver_a, solver_b; gc.collect()

    # ---- 汇总图 ----
    print("\n生成汇总图...")
    plot_summary_compare(all_results, labels,
                         os.path.join(sim_dir, "compare_summary.png"))

    # ---- 统计 ----
    print("\n" + "=" * 70)
    print("Model A vs Model B — 统计汇总")
    print("=" * 70)
    print(f"{'场景':<16} {'Model A RMS':>12} {'Model B RMS':>12} "
          f"{'RMS改善':>10} {'A求解ms':>10} {'B求解ms':>10}")
    print("-" * 70)
    for (ra, rb), lbl in zip(all_results, labels):
        rms_a = np.sqrt(np.mean(ra["cte"]**2))
        rms_b = np.sqrt(np.mean(rb["cte"]**2))
        impr = (rms_a - rms_b) / (rms_a + 1e-9) * 100
        t_a = np.mean(ra["tm"]); t_b = np.mean(rb["tm"])
        print(f"{lbl:<16} {rms_a:12.4f}m {rms_b:12.4f}m "
              f"{impr:9.1f}% {t_a:8.3f}ms {t_b:8.3f}ms")
    print("-" * 70)

    # 总体对比
    all_cte_a = np.concatenate([r[0]["cte"] for r in all_results])
    all_cte_b = np.concatenate([r[1]["cte"] for r in all_results])
    overall_impr = (np.sqrt(np.mean(all_cte_a**2)) - np.sqrt(np.mean(all_cte_b**2))) \
                   / (np.sqrt(np.mean(all_cte_a**2)) + 1e-9) * 100
    print(f"\n总体 RMS 横向误差: A={np.sqrt(np.mean(all_cte_a**2)):.4f}m "
          f"B={np.sqrt(np.mean(all_cte_b**2)):.4f}m "
          f"({overall_impr:.1f}% 改善)")

    print("\n输出图像:")
    for f in ["compare_circle.png", "compare_figure8.png",
              "compare_s_curve.png", "compare_summary.png"]:
        print(f"  {os.path.join(sim_dir, f)}")
    print("\n完成!")


if __name__ == "__main__":
    main()
