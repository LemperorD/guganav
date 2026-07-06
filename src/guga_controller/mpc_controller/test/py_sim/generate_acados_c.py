from acados_template import AcadosModel # 引入acados模型建立模块
from casadi import SX, vertcat, sin, cos # 引入casadi模块
import numpy as np

def export_robot_model() -> AcadosModel:
    model_name = "omni"

    # set up states & controls
    x = SX.sym("x")
    y = SX.sym("y")
    v = SX.sym("v")
    theta = SX.sym("theta")
    theta_d = SX.sym("theta_d")

    x = vertcat(x, y, v, theta, theta_d)

    F = SX.sym("F")
    T = SX.sym("T")
    u = vertcat(F, T)

    # xdot
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    v_dot = SX.sym("v_dot")
    theta_dot = SX.sym("theta_dot")
    theta_ddot = SX.sym("theta_ddot")

    xdot = vertcat(x_dot, y_dot, v_dot, theta_dot, theta_ddot)

    # dynamics
    f_expl = vertcat(v * cos(theta), v * sin(theta), F, theta_d, T)

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    model.t_label = "$t$ [s]"
    model.x_labels = ["$x$", "$y$", "$v$", "$\\theta$", "$\\dot{\\theta}$"]
    model.u_labels = ["$F$", "$T$"]

    return model