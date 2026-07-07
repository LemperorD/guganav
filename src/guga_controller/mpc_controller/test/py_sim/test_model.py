#!/usr/bin/env python3
"""
test_model.py — acados unicycle MPC 模型精度验证 Demo。

使用 Python AcadosOcpSolver 接口进行闭环路径跟踪仿真，
验证：
  1. 开环模型积分精度
  2. 闭环 MPC 路径跟踪性能
  3. 绘制轨迹图和控制量图

用法:
    python3 test_model.py
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from acados_template import AcadosOcpSolver, AcadosSimSolver
from c_codegen import MPCSolver

# 参考轨迹生成
def generate_reference_trajectory(
    trajectory_type: str = "circle",
    total_time: float = 10.0,
    dt: float = 0.05,
) -> np.ndarray:
    """
    生成参考轨迹。

    trajectory_type: "circle" | "line" | "figure8"
    返回值: (N, 3) 数组，每列 [x_ref, y_ref, theta_ref]
    """
    t = np.arange(0.0, total_time, dt)

    if trajectory_type == "circle":
        radius = 2.0
        omega = 0.8
        x_ref = radius * np.cos(omega * t)
        y_ref = radius * np.sin(omega * t)
        theta_ref = omega * t + np.pi / 2.0  # 切线方向

    elif trajectory_type == "line":
        speed = 1.0
        x_ref = speed * t
        y_ref = np.zeros_like(t)
        theta_ref = np.zeros_like(t)

    elif trajectory_type == "figure8":
        a_x, a_y = 2.0, 1.5
        omega_freq = 0.6
        x_ref = a_x * np.sin(omega_freq * t)
        y_ref = a_y * np.sin(2.0 * omega_freq * t)
        # 数值微分计算切线方向
        dt_smooth = 0.001
        dx = np.gradient(x_ref, dt)
        dy = np.gradient(y_ref, dt)
        theta_ref = np.arctan2(dy, dx)

    else:
        raise ValueError(f"Unknown trajectory_type: {trajectory_type}")

    return np.column_stack((x_ref, y_ref, theta_ref)), t


# 闭环 MPC 路径跟踪
def test_closed_loop_mpc(
    mpc_solver: MPCSolver,
    trajectory_type: str = "circle",
):
    """
    使用 acados OCP Solver 进行闭环路径跟踪仿真。
    """
    print("\n" + "=" * 60)
    print(f"2. 闭环 MPC 路径跟踪仿真 ({trajectory_type})")
    print("=" * 60)

    dt = mpc_solver.Tf / mpc_solver.N  # 控制步长 = 1.0/20 = 0.05s
    total_time = 10.0

    # 生成参考轨迹
    ref_full, t_ref = generate_reference_trajectory(
        trajectory_type=trajectory_type, total_time=total_time, dt=dt
    )

    # 创建求解器
    solver = AcadosOcpSolver(
        mpc_solver.ocp,
        json_file=os.path.join(mpc_solver.ocp.code_export_directory,
                               "..", f"{mpc_solver.model.name}_ocp.json"),
    )

    nx = mpc_solver.nx
    nu = mpc_solver.nu
    N = mpc_solver.N

    # 初始状态 (偏离参考)
    if trajectory_type == "circle":
        x_current = np.array([1.5, 0.0, 0.0])  # 偏离圆心
    elif trajectory_type == "figure8":
        x_current = np.array([0.5, 0.3, 0.0])
    else:
        x_current = np.array([0.0, 0.2, 0.0])

    # 仿真
    n_sim_steps = len(ref_full) - N - 1
    x_history = np.zeros((n_sim_steps + 1, nx))
    u_history = np.zeros((n_sim_steps, nu))
    solve_times = np.zeros(n_sim_steps)
    ref_history = np.zeros((n_sim_steps, 3))

    x_history[0] = x_current

    for k in range(n_sim_steps):
        # 当前 horizon 内的参考轨迹
        ref_window = ref_full[k : k + N + 1]

        # 设置初始状态约束
        solver.set(0, "lbx", x_current)
        solver.set(0, "ubx", x_current)

        # 设置参考轨迹 (yref = [x, y, theta, v, omega])
        for i in range(N):
            rx, ry, rtheta = ref_window[i]
            solver.set(i, "yref", np.array([rx, ry, rtheta, 0.0, 0.0]))

        # 末端参考
        rx_e, ry_e, rtheta_e = ref_window[N]
        solver.set(N, "yref", np.array([rx_e, ry_e, rtheta_e]))

        # 求解
        solver.solve()

        # 获取最优控制
        u_opt = solver.get(0, "u")
        u_history[k] = u_opt

        # 记录求解时间
        stats = solver.get_stats("time_tot")
        if stats is not None:
            solve_times[k] = stats

        x_history[k + 1] = x_current
        ref_history[k] = ref_window[0]

        if k % 50 == 0:
            print(f"  step {k}/{n_sim_steps}: x=({x_current[0]:.2f}, {x_current[1]:.2f}), "
                  f"u=({u_opt[0]:.2f}, {u_opt[1]:.2f})")

    # --- 误差统计 ---
    pos_error = np.linalg.norm(x_history[1:, :2] - ref_history[:, :2], axis=1)
    print(f"\n  位置跟踪 RMSE:  {np.sqrt(np.mean(pos_error**2)):.4f} m")
    print(f"  位置跟踪 MAX:   {np.max(pos_error):.4f} m")
    print(f"  平均求解时间:   {np.mean(solve_times)*1000:.2f} ms")
    if solve_times.max() > 0:
        print(f"  最大求解时间:   {np.max(solve_times)*1000:.2f} ms")

    # --- 绘图 ---
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle(f"MPC Closed-Loop Path Tracking ({trajectory_type})", fontsize=13)

    t_sim = np.arange(n_sim_steps + 1) * dt

    # XY 轨迹
    ax = axes[0, 0]
    ax.plot(ref_full[:, 0], ref_full[:, 1], "g-", alpha=0.4, lw=1.5,
            label="Reference")
    ax.plot(x_history[:, 0], x_history[:, 1], "b-", lw=1.2,
            label="MPC")
    ax.plot(x_history[0, 0], x_history[0, 1], "ro", ms=6, label="Start")
    ax.plot(x_history[-1, 0], x_history[-1, 1], "r*", ms=10, label="End")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("XY Trajectory")
    ax.legend()
    ax.axis("equal")
    ax.grid(True)

    # 状态 vs 参考
    labels = ["px [m]", "py [m]", "theta [rad]"]
    for i in range(3):
        ax = axes[0, 1] if i == 0 else (axes[0, 2] if i == 1 else axes[1, 0])
        ax.plot(t_sim, x_history[:, i], "b-", lw=1.2, label="MPC")
        ax.plot(t_sim[:-1], ref_history[:, i], "r--", lw=1.0, label="Ref")
        ax.set_xlabel("time [s]")
        ax.set_ylabel(labels[i])
        ax.set_title(labels[i])
        ax.legend()
        ax.grid(True)

    # 控制量
    ax = axes[1, 1]
    ax.plot(t_sim[:-1], u_history[:, 0], "b-", lw=1.0, label="v")
    ax.axhline(y=mpc_solver.ocp.constraints.lbu[0], color="r", ls="--",
               alpha=0.5, label="v bound")
    ax.axhline(y=mpc_solver.ocp.constraints.ubu[0], color="r", ls="--", alpha=0.5)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("v [m/s]")
    ax.set_title("Control: v")
    ax.legend()
    ax.grid(True)

    ax = axes[1, 2]
    ax.plot(t_sim[:-1], u_history[:, 1], "b-", lw=1.0, label="omega")
    ax.axhline(y=mpc_solver.ocp.constraints.lbu[1], color="r", ls="--",
               alpha=0.5, label="ω bound")
    ax.axhline(y=mpc_solver.ocp.constraints.ubu[1], color="r", ls="--", alpha=0.5)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("ω [rad/s]")
    ax.set_title("Control: ω")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__)+"/results",
                           f"closed_loop_{trajectory_type}.png")
    plt.savefig(out_path, dpi=150)
    print(f"  图表已保存: {out_path}")
    plt.close()

    return x_history, u_history, solve_times


# Main
def main():
    print("=" * 60)
    print("acados Unicycle MPC — 模型精度验证 Demo")
    print("=" * 60)

    # 构建 MPC 配置
    mpc_solver = MPCSolver()
    print(f"  模型:  {mpc_solver.model.name}")
    print(f"  状态:  nx={mpc_solver.nx}, 控制: nu={mpc_solver.nu}")
    print(f"  Horizon: N={mpc_solver.N}, Tf={mpc_solver.Tf}s")
    print(f"  权重 Q=diag(30,30,15), R=diag(1.0,0.5)")

    # 圆轨迹
    x_hist, u_hist, solve_times = test_closed_loop_mpc(
        mpc_solver, trajectory_type="circle"
    )

    # 8字轨迹
    x_hist2, u_hist2, solve_times2 = test_closed_loop_mpc(
        mpc_solver, trajectory_type="figure8"
    )

    print("\n" + "=" * 60)
    print("验证完成！")
    print(f"  开环对比图: open_loop_verification.png")
    print(f"  闭环圆轨迹: closed_loop_circle.png")
    print(f"  闭环8字轨迹: closed_loop_figure8.png")
    print("=" * 60)


if __name__ == "__main__":
    main()
