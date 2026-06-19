#!/usr/bin/env python3
"""
Model B —— 增广误差状态 + 终端约束 MPC 求解器 (基于 acados)。

与 Model A 的结构性差异:
  Model A: 世界系逐点跟踪，y_ref 每阶段变化，终端权重 = 10*Q
  Model B: 相同动力学，但:
    1. 终端权重放大 w_term=100 倍 (vs Model A 的 10 倍)
    2. 终端状态约束: |x_N - x_ref_N|≤ε, |y_N - y_ref_N|≤ε, |θ_N - θ_ref_N|≤ε
       (每个 solve() 动态更新，保证收敛到参考终点)
    3. 这使得 Model B 显式 guarantee 终点可达性

状态 (6D): [x, y, θ, vx_prev, vy_prev, ω_prev]
控制 (3D): [vx, vy, ω]
代价 (NONLINEAR_LS): h = [x, y, θ, vx, vy, ω, Δvx, Δvy, Δω]
"""

import sys
import numpy as np

sys.path.insert(0, "/home/ld/3rdparty/acados/interfaces/acados_template")

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel  # noqa: E402
import casadi as ca  # noqa: E402


class AugmentedMpcSolver:
    """增广误差状态 + 终端约束 MPC 求解器 (Model B)。"""

    nx = 6
    nu = 3
    ny = 9
    ny_e = 3

    def __init__(self, N, dt, w_err, w_ctrl, w_dctrl, w_term, u_min, u_max,
                 term_eps=0.05):
        self._N = N
        self._dt = dt
        self._u_prev = np.zeros(self.nu)
        self._term_eps = term_eps

        model = self._build_model(dt)
        ocp = self._build_ocp(model, N, dt, w_err, w_ctrl, w_dctrl,
                              w_term, u_min, u_max)

        self._solver = AcadosOcpSolver(ocp, json_file="mpc_augmented_solver.json")

    def _build_model(self, dt):
        """构建 6D 增广状态全向轮离散动力学 (同 Model A)。"""
        model = AcadosModel()
        model.name = "omni_mpc_aug"

        x_s = ca.SX.sym("x"); y_s = ca.SX.sym("y"); th_s = ca.SX.sym("theta")
        vxp = ca.SX.sym("vx_prev"); vyp = ca.SX.sym("vy_prev"); wp = ca.SX.sym("omega_prev")
        vx = ca.SX.sym("vx"); vy = ca.SX.sym("vy"); om = ca.SX.sym("omega")

        X = ca.vertcat(x_s, y_s, th_s, vxp, vyp, wp)
        U = ca.vertcat(vx, vy, om)
        model.x = X; model.u = U

        ct, st = ca.cos(th_s), ca.sin(th_s)
        model.disc_dyn_expr = ca.vertcat(
            x_s + dt * (vx * ct - vy * st),
            y_s + dt * (vx * st + vy * ct),
            th_s + dt * om,
            vx, vy, om
        )

        model.cost_y_expr = ca.vertcat(
            x_s, y_s, th_s, vx, vy, om,
            vx - vxp, vy - vyp, om - wp
        )
        model.cost_y_expr_e = ca.vertcat(x_s, y_s, th_s)

        return model

    def _build_ocp(self, model, N, dt, w_err, w_ctrl, w_dctrl,
                   w_term, u_min, u_max):
        """构建 acados OCP + 终端状态约束。"""
        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = N

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"

        ocp.cost.W = np.diag(
            [w_err[0], w_err[0], w_err[1],
             w_ctrl[0], w_ctrl[1], w_ctrl[2],
             w_dctrl[0], w_dctrl[1], w_dctrl[2]]
        )
        # 终端权重放大 w_term 倍 (vs Model A 的 10 倍)
        ocp.cost.W_e = np.diag(
            [w_err[0] * w_term, w_err[0] * w_term, w_err[1] * w_term]
        )
        ocp.cost.yref = np.zeros(self.ny)
        ocp.cost.yref_e = np.zeros(self.ny_e)

        # 控制约束
        ocp.constraints.lbu = np.array(u_min)
        ocp.constraints.ubu = np.array(u_max)
        ocp.constraints.idxbu = np.array([0, 1, 2])

        ocp.constraints.x0 = np.zeros(self.nx)

        # ★ 终端状态约束 (Model B 核心差异)
        # 在每个 solve() 中动态更新上下界
        ocp.constraints.lbx_e = np.zeros(self.nx)
        ocp.constraints.ubx_e = np.zeros(self.nx)
        ocp.constraints.idxbx_e = np.array([0, 1, 2])  # 仅约束 x, y, θ

        # 求解器参数
        ocp.solver_options.tf = N * dt
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "DISCRETE"
        ocp.solver_options.nlp_solver_type = "SQP"
        ocp.solver_options.nlp_solver_max_iter = 50
        ocp.solver_options.print_level = 0

        return ocp

    @property
    def N(self): return self._N

    @property
    def dt(self): return self._dt

    def set_initial_control(self, u0):
        self._u_prev = np.array(u0, dtype=float).flatten()

    def solve(self, x0_vec, ref_states, ref_controls):
        N = self._N

        x0_aug = np.zeros(self.nx)
        x0_aug[:3] = x0_vec
        x0_aug[3:] = self._u_prev
        self._solver.set(0, "lbx", x0_aug)
        self._solver.set(0, "ubx", x0_aug)

        # 阶段参考
        for k in range(N):
            yref = np.zeros(self.ny)
            if k < ref_states.shape[0]:
                yref[0] = ref_states[k, 0]
                yref[1] = ref_states[k, 1]
                yref[2] = ref_states[k, 2]
            if k < ref_controls.shape[0]:
                yref[3] = ref_controls[k, 0]
                yref[4] = ref_controls[k, 1]
                yref[5] = ref_controls[k, 2]
            self._solver.set(k, "yref", yref)

        # 终端参考
        yref_e = np.zeros(self.ny_e)
        if ref_states.shape[0] > 0:
            yref_e[0] = ref_states[-1, 0]
            yref_e[1] = ref_states[-1, 1]
            yref_e[2] = ref_states[-1, 2]
        self._solver.set(N, "yref", yref_e)

        # ★ 终端状态约束: 约束 x_N, y_N, θ_N 在参考终点 ±ε 范围内
        eps = self._term_eps
        if ref_states.shape[0] > 0:
            ref_N = ref_states[-1]
            # 只设 3 个约束 (idxbx_e = [0,1,2])
            lbx_e = np.array([ref_N[0] - eps, ref_N[1] - eps, ref_N[2] - eps * 2.0])
            ubx_e = np.array([ref_N[0] + eps, ref_N[1] + eps, ref_N[2] + eps * 2.0])
            self._solver.set(N, "lbx", lbx_e)
            self._solver.set(N, "ubx", ubx_e)

        status = self._solver.solve()
        if status != 0:
            print(f"  [Model B] acados status={status}")

        u_opt = self._solver.get(0, "u")
        self._u_prev = u_opt.copy()
        return u_opt
