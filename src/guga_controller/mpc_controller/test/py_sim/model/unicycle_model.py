from acados_template import AcadosModel # 引入acados模型建立模块
from casadi import SX, vertcat, sin, cos # 引入casadi模块
import numpy as np

def export_cycle_model() -> AcadosModel:
    model_name = "unicycle"

    # set up states & controls
    px = SX.sym("px")
    py = SX.sym("py")
    v = SX.sym("v")
    theta = SX.sym("theta")
    omega = SX.sym("omega")

    x = vertcat(px, py, theta)
    u = vertcat(v, omega)

    # xdot
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    theta_dot = SX.sym("theta_dot")

    xdot = vertcat(x_dot, y_dot, theta_dot)

    # dynamics
    f_expl = vertcat(
        v * cos(theta),
        v * sin(theta),
        omega
    )

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    model.t_label = "$t$ [s]"
    model.x_labels = ["$p_x$", "$p_y$", "$\\theta$"]
    model.u_labels = ["$v$", "$\\omega$"]

    return model