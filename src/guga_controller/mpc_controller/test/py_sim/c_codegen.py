from acados_template import AcadosOcp, AcadosOcpSolver
from model.unicycle_model import export_cycle_model
from model.omni_model import export_omni_model
import numpy as np
import casadi as ca

import os # 提取acados安装位置的环境变量

class MPC_Solver:
  def __init__(self, cost_params=None):
    self.ocp = AcadosOcp()       # OCP 
    self.model = export_cycle_model()
    self.ocp.model = self.model  # model set

    # parameters
    default_cost_params = {
        'N': 21,
        'Tf': 1.05
    }
    self.cost_params = {**default_cost_params, **(cost_params or {})}

    # set environment variables
    # 请在安装acados时设置环境变量ACADOS_SOURCE_DIR，指向acados的安装目录
    self.ocp.acados_include_path = os.environ.get('ACADOS_SOURCE_DIR') + '/include'
    self.ocp.acados_lib_path = os.environ.get('ACADOS_SOURCE_DIR') + '/lib'

    # set dynamic q_c
    self.cost_error_contour_param = ca.SX.sym('cost_error_contour')
    self.cost_error_contour_x_param = ca.SX.sym('cost_error_contour_x')
    self.cost_error_contour_y_param = ca.SX.sym('cost_error_contour_y')
    self.cost_error_contour_z_param = ca.SX.sym('cost_error_contour_z')
    self.cost_error_lag_param = ca.SX.sym('cost_error_lag')
    self.cost_error_w_param = ca.SX.sym('cost_error_w')
    self.cost_error_df_param = ca.SX.sym('cost_error_df')
    self.cost_error_dv_theta_param = ca.SX.sym('cost_error_dv_theta')
    self.cost_error_v_param = ca.SX.sym('cost_error_v')
    self.poly_coef_x = ca.SX.sym('poly_coef_x', 4)
    self.poly_coef_y = ca.SX.sym('poly_coef_y', 4)
    self.poly_coef_z = ca.SX.sym('poly_coef_z', 4)
    self.ocp.model.p = ca.vertcat(self.cost_error_contour_x_param,
                                  self.cost_error_contour_y_param,
                                  self.cost_error_contour_z_param,
                                  self.cost_error_lag_param,
                                  self.cost_error_w_param,
                                  self.cost_error_df_param,
                                  self.cost_error_dv_theta_param,
                                  self.cost_error_v_param,
                                  self.poly_coef_x,
                                  self.poly_coef_y,
                                  self.poly_coef_z)
    self.ocp.parameter_values = np.array([200.0, 200.0, 200.0, 1000.0, 1.0, 1.0, 10.0, 500.0, 
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    self.nx = self.model.x.size()[0]    # dimension of state
    self.nu = self.model.u.size()[0]    # dimension of control
    self.ny = self.nx + self.nu         # dimension of control + state
    self.ny_e = self.nx                 # end dimension of control + state
    # prediction steps and total prediction time
    self.ocp.solver_options.N_horizon = self.cost_params['N']
    # self.ocp.dims.N = 21
    self.ocp.solver_options.tf = self.cost_params['Tf']

if __name__ == "__main__":
  mpc_solver = MPC_Solver()
  solver = AcadosOcpSolver(mpc_solver.ocp, json_file=f"{mpc_solver.model.name}_ocp.json")

  print("✅ C 代码已生成，位于 build 文件夹，可直接在 C 项目中调用。")