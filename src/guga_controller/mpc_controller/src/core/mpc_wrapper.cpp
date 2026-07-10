#include "mpc_controller/core/mpc_wrapper.hpp"

namespace mpc_controller
{

MpcWrapper::MpcWrapper()
{
  // 创建求解器实例
  capsule_ = omni_acados_create_capsule();
  int status = omni_acados_create_with_discretization(capsule_, N, nullptr);
  if (status) { std::printf("  FAIL: create\n"); return false; }

  // 获取求解器配置和维度信息
  ocp_nlp_config *cfg_ = omni_acados_get_nlp_config(capsule_);
  ocp_nlp_dims  *dims_ = omni_acados_get_nlp_dims(capsule_);
  ocp_nlp_in      *in_ = omni_acados_get_nlp_in(capsule_);
  ocp_nlp_out    *out_ = omni_acados_get_nlp_out(capsule_);
  ocp_nlp_solver *slv_ = omni_acados_get_nlp_solver(capsule_);
}

MpcWrapper::~MpcWrapper()
{
}

void MpcWrapper::setup_solver(
  ocp_nlp_config *cfg, ocp_nlp_dims *dims,
  ocp_nlp_in *in, ocp_nlp_out *out,
  const double x_cur[3], TrajectoryType type, double t_start, double dt) {
  double lbx[NBX0]={x_cur[0],x_cur[1],x_cur[2]};
  double ubx[NBX0]={x_cur[0],x_cur[1],x_cur[2]};
  ocp_nlp_constraints_model_set(cfg,dims,in,out,0,"lbx",void_ptr(lbx));
  ocp_nlp_constraints_model_set(cfg,dims,in,out,0,"ubx",void_ptr(ubx));

  for (int i=0;i<N;++i) {
      double ref[3];
      generate_reference(type,t_start+i*dt,ref);
      double yref[NY]={ref[0],ref[1],ref[2],0.0,0.0};
      ocp_nlp_cost_model_set(cfg,dims,in,i,"y_ref",void_ptr(yref));
  }
  double ref[3];
  generate_reference(type,t_start+N*dt,ref);
  double yref_e[NYN]={ref[0],ref[1],ref[2]};
  ocp_nlp_cost_model_set(cfg,dims,in,N,"y_ref",void_ptr(yref_e));
}

}  // namespace mpc_controller
