#!/usr/bin/env python3
"""
全向移动机器人 MPC 路径跟踪闭环仿真。

验证场景:
  - 圆形轨迹（定曲率）
  - 八字形轨迹（变曲率、换向）
  - S 形换道轨迹（大曲率、阶跃类参考）

输出图片:
  - sim_results_circle.png    — 圆形轨迹跟踪
  - sim_results_figure8.png   — 八字形轨迹跟踪
  - sim_results_s_curve.png   — S 形换道跟踪
  - sim_summary.png           — 三条轨迹的横向误差 + 求解时间对比
"""

import os
import sys
import time
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import MaxNLocator  # noqa: E402

# 配置中文字体支持
import matplotlib.font_manager as fm
_cjk_font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
fm.fontManager.addfont(_cjk_font_path)
_fp = fm.FontProperties(fname=_cjk_font_path)
plt.rcParams["font.family"] = _fp.get_name()
# 解决负号显示问题
plt.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(__file__))
from mpc_solver import MpcSolver  # noqa: E402


# ==============================================================================
# 运动学仿真（RK4）
# ==============================================================================
def omni_rk4(x, u, dt):
    """全向轮运动学 RK4 一步递推。

    Parameters
    ----------
    x : np.ndarray (3,)  — [x, y, theta]
    u : np.ndarray (3,)  — [vx, vy, omega] (车体系)
    dt : float

    Returns
    -------
    np.ndarray (3,)
    """

    def f(xk):
        theta = xk[2]
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        return np.array(
            [u[0] * cos_t - u[1] * sin_t, u[0] * sin_t + u[1] * cos_t, u[2]]
        )

    k1 = f(x)
    k2 = f(x + 0.5 * dt * k1)
    k3 = f(x + 0.5 * dt * k2)
    k4 = f(x + dt * k3)
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


# ==============================================================================
# 参考轨迹生成
# ==============================================================================
def make_circle(t: np.ndarray, radius: float = 2.0, speed: float = 0.5):
    """圆形参考轨迹。

    (x_ref, y_ref) = R*(cos ωt, sin ωt), ω = v/R.
    """
    omega_r = speed / radius
    x = radius * np.cos(omega_r * t)
    y = radius * np.sin(omega_r * t)
    # 切线方向（逆时针为正）
    theta = omega_r * t + np.pi / 2.0
    vx_ref = np.full_like(t, speed)
    vy_ref = np.zeros_like(t)
    w_ref = np.full_like(t, omega_r)
    return np.column_stack([x, y, theta]), np.column_stack([vx_ref, vy_ref, w_ref])


def make_figure8(t: np.ndarray, a: float = 2.0, omega: float = 0.3):
    """八字形（Lemniscate of Gerono）参考轨迹。

    参数方程:
      x(s) = a * sin(s)
      y(s) = a * sin(s) * cos(s) = a/2 * sin(2s)
    其中 s = ω * t.

    曲率连续变化，最大曲率出现在交叉点处。
    """
    s = omega * t
    x = a * np.sin(s)
    y = a * 0.5 * np.sin(2.0 * s)

    # 一阶导数
    dx = a * omega * np.cos(s)
    dy = a * omega * np.cos(2.0 * s)
    speed = np.sqrt(dx**2 + dy**2)

    # 二阶导数
    ddx = -a * omega**2 * np.sin(s)
    ddy = -2.0 * a * omega**2 * np.sin(2.0 * s)

    # 曲率 κ = (dx*ddy - dy*ddx) / (dx²+dy²)^(3/2)
    curvature = (dx * ddy - dy * ddx) / (speed**3 + 1e-9)

    theta = np.arctan2(dy, dx)
    # 参考速度沿切线方向（车体 x 时刻对准切线）
    vx_ref = speed
    vy_ref = np.zeros_like(speed)
    w_ref = speed * curvature

    # 上限裁剪
    vx_ref = np.clip(vx_ref, -3.0, 3.0)
    w_ref = np.clip(w_ref, -6.0, 6.0)

    return np.column_stack([x, y, theta]), np.column_stack([vx_ref, vy_ref, w_ref])


def make_s_curve(t: np.ndarray, duration: float = 15.0):
    """S 形换道轨迹。

    前 1/3 直线，中 1/3 S 形换道（正弦横向偏移），后 1/3 直线。
    """
    T = duration
    v0 = 0.8
    y_lane_change = 2.0  # 横向偏移 2m
    x_total = v0 * T

    x = v0 * t

    phase = t / T
    # 使用平滑的 sigmoid 类横向偏移
    # y(s) = y_lane_change * (1 / (1 + exp(-a*(s - 0.5))))
    # 但这样不够平滑，改用三次多项式过渡
    t1, t2 = T * 0.3, T * 0.7
    y = np.zeros_like(t)
    for i, ti in enumerate(t):
        if ti < t1:
            y[i] = 0.0
        elif ti < t2:
            # 归一化到 [0,1]
            tau = (ti - t1) / (t2 - t1)
            # 五次多项式（位置、速度、加速度在边界连续）
            y[i] = y_lane_change * (
                10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
            )
        else:
            y[i] = y_lane_change

    # 数值微分求导数和曲率
    dt_avg = np.mean(np.diff(t))
    dx = np.gradient(x, dt_avg)
    dy = np.gradient(y, dt_avg)
    ddx = np.gradient(dx, dt_avg)
    ddy = np.gradient(dy, dt_avg)
    speed = np.sqrt(dx**2 + dy**2)
    curvature = (dx * ddy - dy * ddx) / (speed**3 + 1e-9)

    theta = np.arctan2(dy, dx)
    vx_ref = np.full_like(t, v0)
    vy_ref = np.zeros_like(t)
    w_ref = vx_ref * curvature
    w_ref = np.clip(w_ref, -6.0, 6.0)

    return np.column_stack([x, y, theta]), np.column_stack([vx_ref, vy_ref, w_ref])


# ==============================================================================
# MPC 闭环仿真循环
# ==============================================================================
def run_mpc_simulation(
    solver: MpcSolver,
    ref_states_all,
    ref_controls_all,
    t_sim,
    x0,
    disturbance_std=0.0,
):
    """运行 MPC 闭环仿真。

    Parameters
    ----------
    solver : MpcSolver
    ref_states_all : np.ndarray (N_sim, 3)
    ref_controls_all : np.ndarray (N_sim, 3)
    t_sim : np.ndarray
    x0 : np.ndarray (3,)
    disturbance_std : float
        过程噪声标准差（0 表示无噪声）。

    Returns
    -------
    dict with keys:
      states, controls, cross_track_errors, heading_errors,
      solve_times_ms, ref_states, ref_controls, t
    """
    N_total = len(t_sim)
    N_horizon = solver.N
    dt = solver.dt

    x_cur = np.array(x0, dtype=float)
    states = np.zeros((N_total, 3))
    controls = np.zeros((N_total, 3))
    cross_track_errors = np.zeros(N_total)
    heading_errors = np.zeros(N_total)
    solve_times_ms = np.zeros(N_total)

    # 初始控制猜测设为零
    solver.set_initial_control([0.0, 0.0, 0.0])

    for i in range(N_total):
        # 记录状态
        states[i, :] = x_cur

        # 提取预测时域内的参考轨迹（从当前位置向前看 N 步）
        ref_start = min(i, N_total - 1)
        ref_end = min(ref_start + N_horizon, N_total)
        ref_states_horizon = ref_states_all[ref_start:ref_end, :]
        ref_controls_horizon = ref_controls_all[ref_start:ref_end, :]

        # 如果参考不够 N 步，用最后一步重复填充
        if ref_states_horizon.shape[0] < N_horizon:
            pad = N_horizon - ref_states_horizon.shape[0]
            ref_states_horizon = np.vstack(
                [ref_states_horizon, np.tile(ref_states_horizon[-1:], (pad, 1))]
            )
            ref_controls_horizon = np.vstack(
                [
                    ref_controls_horizon,
                    np.tile(ref_controls_horizon[-1:], (pad, 1)),
                ]
            )

        # 求解 MPC
        t0 = time.perf_counter()
        u_opt = solver.solve(x_cur, ref_states_horizon, ref_controls_horizon)
        solve_times_ms[i] = (time.perf_counter() - t0) * 1000.0

        controls[i, :] = u_opt

        # 计算跟踪误差
        ref_state_i = ref_states_all[min(i, N_total - 1), :]
        dx = x_cur[0] - ref_state_i[0]
        dy = x_cur[1] - ref_state_i[1]
        # 横向误差：投影到参考航向的法向
        cos_t = np.cos(ref_state_i[2])
        sin_t = np.sin(ref_state_i[2])
        cross_track_errors[i] = -sin_t * dx + cos_t * dy
        heading_errors[i] = _wrap_pi(x_cur[2] - ref_state_i[2])

        # 递推动力学
        if i < N_total - 1:
            actual_dt = t_sim[i + 1] - t_sim[i]
            x_cur = omni_rk4(x_cur, u_opt, actual_dt)
            if disturbance_std > 0:
                x_cur[:2] += np.random.randn(2) * disturbance_std

    return dict(
        states=states,
        controls=controls,
        cross_track_errors=cross_track_errors,
        heading_errors=heading_errors,
        solve_times_ms=solve_times_ms,
        ref_states=ref_states_all,
        ref_controls=ref_controls_all,
        t=t_sim,
    )


def _wrap_pi(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


# ==============================================================================
# 可视化
# ==============================================================================
def plot_simulation(result, title, save_path, show_solve_time=True):
    """绘制单条轨迹的仿真结果（2×2 子图）。"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(title, fontsize=14, fontweight="bold")

    # (0,0) 轨迹对比
    ax = axes[0, 0]
    ax.plot(result["ref_states"][:, 0], result["ref_states"][:, 1],
            "k--", linewidth=1.5, alpha=0.7, label="参考轨迹")
    ax.plot(result["states"][:, 0], result["states"][:, 1],
            "C0-", linewidth=1.5, label="实际轨迹")
    ax.scatter(*result["states"][0, :2], c="C2", marker="o",
               s=60, zorder=5, label="起点")
    ax.scatter(*result["states"][-1, :2], c="C3", marker="*",
               s=80, zorder=5, label="终点")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (0,1) 速度曲线
    ax = axes[0, 1]
    t = result["t"]
    ax.plot(t, result["controls"][:, 0], label="vx", linewidth=1.2)
    ax.plot(t, result["controls"][:, 1], label="vy", linewidth=1.2)
    ax.plot(t, result["controls"][:, 2], label="ω", linewidth=1.2)
    if result["ref_controls"] is not None:
        ax.plot(t, result["ref_controls"][:, 0],
                "k:", linewidth=1.0, alpha=0.5, label="vx_ref")
        ax.plot(t, result["ref_controls"][:, 2],
                "k-.", linewidth=1.0, alpha=0.5, label="ω_ref")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("控制量")
    ax.legend(fontsize=7, ncol=3)
    ax.grid(True, alpha=0.3)

    # (1,0) 横向误差 + 航向误差
    ax = axes[1, 0]
    ax.plot(t, result["cross_track_errors"], "C0-", linewidth=1.5,
            label="横向误差")
    ax.plot(t, result["heading_errors"], "C1-", linewidth=1.5,
            alpha=0.7, label="航向误差")
    ax.axhline(0, color="k", linewidth=0.5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("误差 [m / rad]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (1,1) 求解时间
    ax = axes[1, 1]
    if show_solve_time:
        solve_ms = result["solve_times_ms"]
        ax.plot(t, solve_ms, "C2-", linewidth=1.2)
        ax.axhline(np.mean(solve_ms), color="C2",
                   linestyle="--", linewidth=1.0,
                   label=f"均值={np.mean(solve_ms):.2f} ms")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("求解时间 [ms]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)
    else:
        ax.axis("off")

    fig.tight_layout()
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_summary(all_results, labels, save_path):
    """绘制所有轨迹的横向误差 + 求解时间汇总对比图。"""
    n = len(all_results)
    fig, axes = plt.subplots(2, n, figsize=(5 * n, 8),
                             squeeze=False)
    fig.suptitle("MPC 路径跟踪 —— 多场景对比", fontsize=14, fontweight="bold")

    for i, (res, lbl) in enumerate(zip(all_results, labels)):
        t = res["t"]

        # 横向误差
        ax = axes[0, i]
        ax.plot(t, res["cross_track_errors"], "C0-", linewidth=1.5)
        ax.axhline(0, color="k", linewidth=0.5)
        rms = np.sqrt(np.mean(res["cross_track_errors"] ** 2))
        max_e = np.max(np.abs(res["cross_track_errors"]))
        ax.set_title(f"{lbl}\nRMS={rms:.4f}m, max={max_e:.4f}m", fontsize=10)
        ax.set_xlabel("t [s]")
        ax.set_ylabel("横向误差 [m]")
        ax.grid(True, alpha=0.3)

        # 求解时间
        ax = axes[1, i]
        ax.plot(t, res["solve_times_ms"], "C2-", linewidth=1.2)
        mean_ms = np.mean(res["solve_times_ms"])
        max_ms = np.max(res["solve_times_ms"])
        ax.axhline(mean_ms, color="C3", linestyle="--", linewidth=1.0)
        ax.set_title(f"求解时间: mean={mean_ms:.2f}ms, max={max_ms:.2f}ms",
                     fontsize=10)
        ax.set_xlabel("t [s]")
        ax.set_ylabel("求解时间 [ms]")
        ax.set_ylim(bottom=0)
        ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"汇总图已保存: {save_path}")


# ==============================================================================
# 主函数
# ==============================================================================
def main():
    print("=" * 60)
    print("全向移动机器人 MPC 路径跟踪仿真")
    print("=" * 60)

    # ---- 参数 ----
    N = 15
    dt = 0.05  # 20 Hz
    Q = [10.0, 10.0, 2.0]
    R = [0.1, 0.1, 0.05]
    Rd = [0.5, 0.5, 0.3]
    u_min = [-3.0, -3.0, -6.0]
    u_max = [3.0, 3.0, 6.0]

    print(f"\nMPC 参数: N={N}, dt={dt}s, 预测时长={N*dt}s")
    print(f"Q={Q}, R={R}, Rd={Rd}")
    print(f"u_min={u_min}, u_max={u_max}")

    # ---- 创建求解器 ----
    print("\n[1/3] 创建 acados MPC 求解器...")
    solver = MpcSolver(N, dt, Q, R, Rd, u_min, u_max)
    print("  求解器创建成功。")

    sim_dir = os.path.dirname(__file__) or "."
    all_results, labels = [], []

    # ---- 场景 1: 圆形轨迹 ----
    print("\n[2/3] 运行仿真...\n  [圆形] ", end="", flush=True)
    T, dt_sim = 20.0, 0.01
    t_sim = np.arange(0, T, dt_sim)
    ref_s, ref_u = make_circle(t_sim, radius=2.0, speed=0.8)
    x0_circle = np.array([2.2, 0.0, np.pi / 2.0])  # 初始位置有 0.2m 偏差
    solver.set_initial_control([0.0, 0.0, 0.0])
    res_circle = run_mpc_simulation(solver, ref_s, ref_u, t_sim, x0_circle)
    plot_simulation(res_circle, "圆形轨迹跟踪 (R=2m, v=0.8m/s)",
                    os.path.join(sim_dir, "sim_results_circle.png"))
    all_results.append(res_circle)
    labels.append("圆形 (R=2m)")
    print(f"完成。RMS 横向误差: "
          f"{np.sqrt(np.mean(res_circle['cross_track_errors']**2)):.4f}m, "
          f"平均求解时间: {np.mean(res_circle['solve_times_ms']):.3f}ms")

    # ---- 场景 2: 八字形轨迹 ----
    print("  [八字] ", end="", flush=True)
    T, dt_sim = 30.0, 0.01
    t_sim = np.arange(0, T, dt_sim)
    ref_s, ref_u = make_figure8(t_sim, a=2.0, omega=0.35)
    x0_f8 = np.array([0.3, 0.2, 0.0])
    solver.set_initial_control([0.0, 0.0, 0.0])
    res_f8 = run_mpc_simulation(solver, ref_s, ref_u, t_sim, x0_f8)
    plot_simulation(res_f8, "八字形轨迹跟踪 (Lemniscate, a=2m)",
                    os.path.join(sim_dir, "sim_results_figure8.png"))
    all_results.append(res_f8)
    labels.append("八字形 (Lemniscate)")
    print(f"完成。RMS 横向误差: "
          f"{np.sqrt(np.mean(res_f8['cross_track_errors']**2)):.4f}m, "
          f"平均求解时间: {np.mean(res_f8['solve_times_ms']):.3f}ms")

    # ---- 场景 3: S 形换道 ----
    print("  [S 形] ", end="", flush=True)
    T, dt_sim = 15.0, 0.01
    t_sim = np.arange(0, T, dt_sim)
    ref_s, ref_u = make_s_curve(t_sim, duration=T)
    x0_sc = np.array([0.0, 0.1, 0.0])  # 微小横向偏差
    solver.set_initial_control([0.0, 0.0, 0.0])
    res_sc = run_mpc_simulation(solver, ref_s, ref_u, t_sim, x0_sc)
    plot_simulation(res_sc, "S 形换道轨迹跟踪 (换道宽 2m, v=0.8m/s)",
                    os.path.join(sim_dir, "sim_results_s_curve.png"))
    all_results.append(res_sc)
    labels.append("S 形换道")
    print(f"完成。RMS 横向误差: "
          f"{np.sqrt(np.mean(res_sc['cross_track_errors']**2)):.4f}m, "
          f"平均求解时间: {np.mean(res_sc['solve_times_ms']):.3f}ms")

    # ---- 汇总图 ----
    print("\n[3/3] 生成汇总图...")
    plot_summary(all_results, labels, os.path.join(sim_dir, "sim_summary.png"))

    # ---- 统计 ----
    print("\n" + "=" * 60)
    print("仿真统计汇总")
    print("=" * 60)
    print(f"{'场景':<20} {'RMS CTE':>10} {'Max |CTE|':>10} "
          f"{'求解时间均值':>12} {'求解时间最大':>12}")
    print("-" * 64)
    for lbl, res in zip(labels, all_results):
        rms = np.sqrt(np.mean(res["cross_track_errors"] ** 2))
        mx = np.max(np.abs(res["cross_track_errors"]))
        t_mean = np.mean(res["solve_times_ms"])
        t_max = np.max(res["solve_times_ms"])
        print(f"{lbl:<20} {rms:10.4f}m {mx:10.4f}m "
              f"{t_mean:10.3f}ms {t_max:10.3f}ms")
    print("-" * 64)

    # 判断实时性
    all_times = np.concatenate([r["solve_times_ms"] for r in all_results])
    worst = np.max(all_times)
    if worst < 5.0:
        print(f"\n✅ 实时性达标: 最大求解时间 {worst:.2f} ms < 5 ms")
    elif worst < 50.0:
        print(f"\n⚠️ 勉强可用: 最大求解时间 {worst:.2f} ms")
    else:
        print(f"\n❌ 不满足实时性: 最大求解时间 {worst:.2f} ms")

    print("\n输出图像:")
    for f in ["sim_results_circle.png", "sim_results_figure8.png",
              "sim_results_s_curve.png", "sim_summary.png"]:
        print(f"  {os.path.join(sim_dir, f)}")

    print("\n完成!")


if __name__ == "__main__":
    main()
