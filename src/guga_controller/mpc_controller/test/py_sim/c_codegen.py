from acados_template import AcadosOcp, AcadosOcpSolver
from casadi import SX, vertcat
import scipy.linalg
import numpy as np
import os

# 导入动力学模型
from model.unicycle_model import export_cycle_model
# from model.omni_model import export_omni_model

class MPCSolver:
    def __init__(self):
        self.ocp = AcadosOcp()
        
        self.model = export_cycle_model()
        # self.model = export_omni_model()
        self.ocp.model = self.model
        
        # Horizon 设置
        self.N = 20                 # 预测步数
        self.Tf = 1.0               # 预测时间(s)
        self.ocp.solver_options.N_horizon = self.N
        self.ocp.solver_options.tf = self.Tf

        # 状态维度
        self.nx = self.model.x.size()[0]
        self.nu = self.model.u.size()[0]

        self.ny = self.nx + self.nu
        self.ny_e = self.nx

        # Cost 类型
        self.ocp.cost.cost_type = "NONLINEAR_LS"
        self.ocp.cost.cost_type_e = "NONLINEAR_LS"

        # Cost Expression
        self.ocp.model.cost_y_expr = vertcat(self.model.x, self.model.u)
        self.ocp.model.cost_y_expr_e = self.model.x

        # 状态权重
        Q = np.diag([
            30.0,      # x
            30.0,      # y
            15.0       # theta
        ])

        # 控制权重
        R = np.diag([
            1.0,       # v
            0.5        # omega
        ])

        # Cost Matrix
        self.ocp.cost.W = scipy.linalg.block_diag(Q, R)
        self.ocp.cost.W_e = Q

        # Reference
        self.ocp.cost.yref = np.zeros(self.ny)
        self.ocp.cost.yref_e = np.zeros(self.ny_e)

        # 输入约束
        v_max = 2.5
        omega_max = 4.0
        self.ocp.constraints.lbu = np.array([
            -v_max,
            -omega_max
        ])

        self.ocp.constraints.ubu = np.array([
            v_max,
            omega_max
        ])

        self.ocp.constraints.idxbu = np.array([
            0,
            1
        ])

        # 初始状态
        self.ocp.constraints.x0 = np.zeros(self.nx)

        # Solver
        self.ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        self.ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        self.ocp.solver_options.integrator_type = "ERK"
        self.ocp.solver_options.nlp_solver_type = "SQP_RTI"

        # acados 安装目录
        acados_path = os.environ["ACADOS_SOURCE_DIR"]
        print(f"acados_path: {acados_path}")
        self.ocp.acados_include_path = acados_path + "/include"
        self.ocp.acados_lib_path = acados_path + "/lib"

        # C代码导出目录，根据自身情况修改
        self.ocp.code_export_directory = "/home/ld/guganav/src/guga_controller/mpc_controller/generated/c_code"
        
if __name__ == "__main__":
    mpc_solver = MPCSolver()
	
    solver = AcadosOcpSolver(
        mpc_solver.ocp,
        json_file=f"{mpc_solver.model.name}_ocp.json"
    )

    print("=======================================")
    print("acados Solver 已生成")
    print("生成目录：generated/c_code/")
    print("JSON文件：generated/unicycle_ocp.json")
    print("=======================================")