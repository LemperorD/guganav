#!/usr/bin/env python3
"""
MPC 求解器封装 —— 基于 acados 的全向移动机器人路径跟踪MPC。

状态变量 (6D): [x, y, θ, vx_prev, vy_prev, ω_prev]
  - 前三维是机器人位姿
  - 后三维是上一时刻的控制量（用于计算控制变化率代价）

控制输入 (3D): [vx, vy, ω]

代价函数（非线性最小二乘）:
  h(x,u,k) = [x, y, θ, vx, vy, ω, vx-vx_prev, vy-vy_prev, ω-ω_prev]
  y_ref(k) = [x_ref, y_ref, θ_ref, vx_ref, vy_ref, ω_ref, 0, 0, 0]
  J = Σ 0.5 * ||h(x,u,k) - y_ref||²_W + 0.5 * ||h_N(x) - y_ref_N||²_W_e

离散动力学（前向欧拉，步长 dt）:
  x_{k+1}   = x_k     + dt*(vx*cos θ_k - vy*sin θ_k)
  y_{k+1}   = y_k     + dt*(vx*sin θ_k + vy*cos θ_k)
  θ_{k+1}   = θ_k     + dt*ω
  u_prev_{k+1} = u_k（控制量向前传递）
"""

import sys
import numpy as np

sys.path.insert(0, "/home/ld/3rdparty/acados/interfaces/acados_template")

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel  # noqa: E402
import casadi as ca  # noqa: E402


class MpcSolver:
    """全向轮 MPC 路径跟踪求解器。

    Parameters
    ----------
    N : int
        预测时域步数。
    dt : float
        控制步长 [s]。
    Q : list[float]
        状态权重 [qx, qy, qtheta]。
    R : list[float]
        控制权重 [rvx, rvy, romega]。
    Rd : list[float]
        控制变化率权重 [rdvx, rdvy, rdomega]。
    u_min : list[float]
        控制下限 [vx_min, vy_min, omega_min]。
    u_max : list[float]
        控制上限 [vx_max, vy_max, omega_max]。
    """

    nx = 6  # 增广状态维度
    nu = 3  # 控制维度
    ny = 9  # 阶段代价输出维度
    ny_e = 3  # 终端代价输出维度

    def __init__(self, N, dt, Q, R, Rd, u_min, u_max):
        self._N = N
        self._dt = dt
        self._u_prev = np.zeros(self.nu)

        model = self._build_model(dt)
        ocp = self._build_ocp(model, N, dt, Q, R, Rd, u_min, u_max)

        # 就地生成 C 代码并编译为共享库
        self._solver = AcadosOcpSolver(ocp, json_file="mpc_omni_solver.json")

    # ------------------------------------------------------------------
    def _build_model(self, dt):
        """构建增广状态全向轮离散动力学模型。"""
        model = AcadosModel()
        model.name = "omni_mpc"

        # ---- 符号变量 ----
        x_sym = ca.SX.sym("x")
        y_sym = ca.SX.sym("y")
        theta_sym = ca.SX.sym("theta")
        vxp = ca.SX.sym("vx_prev")
        vyp = ca.SX.sym("vy_prev")
        wp = ca.SX.sym("omega_prev")

        vx = ca.SX.sym("vx")
        vy = ca.SX.sym("vy")
        omega = ca.SX.sym("omega")

        X = ca.vertcat(x_sym, y_sym, theta_sym, vxp, vyp, wp)
        U = ca.vertcat(vx, vy, omega)

        model.x = X
        model.u = U

        # ---- 离散动力学（前向欧拉）----
        cos_t = ca.cos(theta_sym)
        sin_t = ca.sin(theta_sym)

        x_next = x_sym + dt * (vx * cos_t - vy * sin_t)
        y_next = y_sym + dt * (vx * sin_t + vy * cos_t)
        theta_next = theta_sym + dt * omega
        # 控制量向前传递（作为下一阶段的 u_prev）
        vxp_next = vx
        vyp_next = vy
        wp_next = omega

        model.disc_dyn_expr = ca.vertcat(
            x_next, y_next, theta_next, vxp_next, vyp_next, wp_next
        )

        # ---- 阶段代价输出 ----
        # h(x,u) = [x, y, θ, vx, vy, ω, Δvx, Δvy, Δω]
        model.cost_y_expr = ca.vertcat(
            x_sym, y_sym, theta_sym,  # 状态跟踪
            vx, vy, omega,  # 控制代价
            vx - vxp, vy - vyp, omega - wp,  # 控制变化率
        )
        # 终端代价（仅状态）
        model.cost_y_expr_e = ca.vertcat(x_sym, y_sym, theta_sym)

        return model

    # ------------------------------------------------------------------
    def _build_ocp(self, model, N, dt, Q, R, Rd, u_min, u_max):
        """构建 acados OCP 问题。"""
        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = N

        # ---- 代价 ----
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"

        # 阶段权重: [x, y, θ, vx, vy, ω, dvx, dvy, dω]
        ocp.cost.W = np.diag(
            [Q[0], Q[1], Q[2], R[0], R[1], R[2], Rd[0], Rd[1], Rd[2]]
        )
        # 终端权重（增大以促进收敛）
        ocp.cost.W_e = np.diag([Q[0] * 10.0, Q[1] * 10.0, Q[2] * 10.0])

        ocp.cost.yref = np.zeros(self.ny)
        ocp.cost.yref_e = np.zeros(self.ny_e)

        # ---- 控制约束 ----
        ocp.constraints.lbu = np.array(u_min)
        ocp.constraints.ubu = np.array(u_max)
        ocp.constraints.idxbu = np.array([0, 1, 2])

        # 初始状态约束（每次求解时动态设置）
        ocp.constraints.x0 = np.zeros(self.nx)

        # ---- 求解器参数 ----
        ocp.solver_options.tf = N * dt
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "DISCRETE"
        ocp.solver_options.nlp_solver_type = "SQP"
        ocp.solver_options.nlp_solver_max_iter = 50
        ocp.solver_options.print_level = 0  # 静默

        return ocp

    # ------------------------------------------------------------------
    @property
    def N(self):
        return self._N

    @property
    def dt(self):
        return self._dt

    # ------------------------------------------------------------------
    def set_initial_control(self, u0):
        """设置初始阶段 u_prev（求解器创建后首次调用前）。"""
        self._u_prev = np.array(u0, dtype=float).flatten()

    # ------------------------------------------------------------------
    def solve(self, x0_vec, ref_states, ref_controls):
        """求解 MPC 并获得最优控制量。

        Parameters
        ----------
        x0_vec : np.ndarray (3,)
            当前机器人状态 [x, y, theta]。
        ref_states : np.ndarray (N, 3)
            预测时域内参考状态 [x_ref, y_ref, theta_ref]（k = 1..N）。
        ref_controls : np.ndarray (N, 3)
            预测时域内参考控制 [vx_ref, vy_ref, omega_ref]（k = 0..N-1）。

        Returns
        -------
        u_opt : np.ndarray (3,)
            最优控制量 [vx, vy, omega]。
        """
        N = self._N

        # 构造增广初始状态: [x, y, theta, vx_prev, vy_prev, omega_prev]
        x0_aug = np.zeros(self.nx)
        x0_aug[:3] = x0_vec
        x0_aug[3:] = self._u_prev

        self._solver.set(0, "lbx", x0_aug)
        self._solver.set(0, "ubx", x0_aug)

        # 设置各阶段参考值
        for k in range(N):
            yref = np.zeros(self.ny)
            if k < ref_states.shape[0]:
                yref[:3] = ref_states[k, :3]
            if k < ref_controls.shape[0]:
                yref[3:6] = ref_controls[k, :3]
            # yref[6:9] = 0（期望控制变化率为零）
            self._solver.set(k, "yref", yref)

        # 终端参考
        yref_e = np.zeros(self.ny_e)
        if ref_states.shape[0] > 0:
            yref_e[:3] = ref_states[-1, :3]
        self._solver.set(N, "yref", yref_e)

        # 求解
        status = self._solver.solve()
        if status != 0:
            print(f"  [MPC] 警告: acados 返回状态码 {status}")

        # 获取最优控制
        u_opt = self._solver.get(0, "u")
        self._u_prev = u_opt.copy()

        return u_opt
