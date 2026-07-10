#include "mpc_controller/core/mpc_wrapper.hpp"

namespace mpc_controller
{

MpcWrapper::MpcWrapper()
{
  // 创建求解器实例
  capsule_ = omni_acados_create_capsule();
  int status = omni_acados_create_with_discretization(capsule_, N, nullptr);
  if (status) {
    printf("omni_acados_create() returned status %d. Exiting.\n", status);
    exit(1);
  }

  // 获取求解器配置和维度信息
  ocp_nlp_config *cfg_ = omni_acados_get_nlp_config(capsule_);
  ocp_nlp_dims  *dims_ = omni_acados_get_nlp_dims(capsule_);
  ocp_nlp_in      *in_ = omni_acados_get_nlp_in(capsule_);
  ocp_nlp_out    *out_ = omni_acados_get_nlp_out(capsule_);
  ocp_nlp_solver *slv_ = omni_acados_get_nlp_solver(capsule_);
}

MpcWrapper::~MpcWrapper()
{
  // free solver
  status = omni_acados_free(capsule_);
  if (status) {
      printf("omni_acados_free() returned status %d. \n", status);
  }

  // free solver capsule
  status = omni_acados_free_capsule(capsule_);
  if (status) {
      printf("omni_ode_acados_free_capsule() returned status %d. \n", status);
  }
}

void MpcWrapper::setup_solver(const double x_cur[3], TrajectoryType type, double t_start, double dt)
{
  // 设置初始状态约束
  double lbx[NBX0]={x_cur[0],x_cur[1],x_cur[2]};
  double ubx[NBX0]={x_cur[0],x_cur[1],x_cur[2]};
  ocp_nlp_constraints_model_set(cfg_,dims_,in_,out_,0,"lbx",void_ptr(lbx));
  ocp_nlp_constraints_model_set(cfg_,dims_,in_,out_,0,"ubx",void_ptr(ubx));

  // 设置参考轨迹
  for (int i=0;i<N;++i) {
      double ref[3];
      generate_reference(type,t_start+i*dt,ref);
      double yref[NY]={ref[0],ref[1],ref[2],0.0,0.0};
      ocp_nlp_cost_model_set(cfg_,dims_,in_,out_,i,"y_ref",void_ptr(yref));
  }
  double ref[3];
  generate_reference(type,t_start+N*dt,ref);
  double yref_e[NYN]={ref[0],ref[1],ref[2]};
  ocp_nlp_cost_model_set(cfg_,dims_,in_,out_,N,"y_ref",void_ptr(yref_e));
}

const std::vector<double>& MpcWrapper::solve(){
    // solve ocp in loop
    for (int ii = 0; ii < NTIMINGS_; ii++)
    {
        // initialize solution
        for (int i = 0; i < N_; i++)
        {
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x_init_.data());
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", u_init_.data());
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", y_ref_.data());
        }
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, N_, "x", x_init_.data());
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "yref", y_ref_.data());
        status_ = quadrotor_acados_solve(acados_ocp_capsule_);
        ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time_);
        min_time_= MIN(elapsed_time_, min_time_);
    }
        if (status_ != ACADOS_SUCCESS)
    {
        printf("quadrotor_acados_solve() failed with _ %d.\n", status_);
    }
        /* print solution and statistics */
    /*for (int ii = 0; ii <= nlp_dims_->N; ii++)
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj_[ii*NX]);
    for (int ii = 0; ii < nlp_dims_->N; ii++)
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj_[ii*NU]);
    for (int ii = 0 ; ii < NU; ++ii){
        u_opt_[ii] = utraj_[ii];
    }*/
    
    // Get optimal control at first stage
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u_opt_.data());
    
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf_);
    ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter_);

    return u_opt_;
}

}  // namespace mpc_controller
