#include "mpc_controller/core/mpc_wrapper.hpp"

namespace mpc_controller
{

MpcWrapper::MpcWrapper()
{
  // 创建求解器实例
  capsule_ = omni_acados_create_capsule();
  status_ = omni_acados_create_with_discretization(capsule_, N, nullptr);
  if (status_) {
    std::printf("omni_acados_create() returned status %d. Exiting.\n", status_);
    exit(1);
  }

  // 获取求解器配置和维度信息
  ocp_nlp_config *cfg_ = omni_acados_get_nlp_config(capsule_);
  ocp_nlp_dims  *dims_ = omni_acados_get_nlp_dims(capsule_);
  ocp_nlp_in      *in_ = omni_acados_get_nlp_in(capsule_);
  ocp_nlp_out    *out_ = omni_acados_get_nlp_out(capsule_);
  ocp_nlp_solver *slv_ = omni_acados_get_nlp_solver(capsule_);

  std::printf("MPC Wrapper initialized successfully.\n");
}

MpcWrapper::~MpcWrapper()
{
  // free solver
  status_ = omni_acados_free(capsule_);
  if (status_) {
    std::printf("omni_acados_free() returned status %d. \n", status_);
  }

  // free solver capsule
  status_ = omni_acados_free_capsule(capsule_);
  if (status_) {
    std::printf("omni_acados_free_capsule() returned status %d. \n", status_);
  }
}

const std::vector<double>& MpcWrapper::solve(){
  // solve ocp in loop
  for (int ii = 0; ii < NTIMINGS_; ii++)
  {
    // initialize solution
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_out_set(cfg_, dims_, out_, in_, i, "x", x_init_.data());
        ocp_nlp_out_set(cfg_, dims_, out_, in_, i, "u", u_init_.data());
        ocp_nlp_cost_model_set(cfg_, dims_, in_, i, "yref", y_ref_.data());
    }
    ocp_nlp_out_set(cfg_, dims_, out_, in_, N, "x", x_init_.data());
    ocp_nlp_cost_model_set(cfg_, dims_, in_, N, "yref", y_ref_.data());
    status_ = omni_acados_solve(acados_ocp_capsule_);
    ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time_);
    min_time_= MIN(elapsed_time_, min_time_);
  }

  // 判断是否求解成功
  if (status_ != ACADOS_SUCCESS) {
    std::printf("omni_acados_solve() failed with %d.\n", status_);
  }

  // print solution and statistics
  // for (int ii = 0; ii <= nlp_dims_->N; ii++)
  //     ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &x_traj_[ii*NX]);
  // for (int ii = 0; ii < nlp_dims_->N; ii++)
  //     ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &u_traj_[ii*NU]);
  // for (int ii = 0 ; ii < NU; ++ii){
  //     u_opt_[ii] = u_traj_[ii];
  // }
  
  // Get optimal control at first stage
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u_opt_.data());
  
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf_);
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter_);

  return u_opt_;
}

// uses previous u_opt as u_ref
const std::vector<double>& MpcWrapper::solve(std::vector<double>& x0, const std::vector<double>& x_des){
    set_uinit(u_opt_);
    set_xinit(x0);
    set_x0(x0);
    set_yref(x_des, u_opt_);
    return solve();
}

void MpcWrapper::set_x0(std::vector<double>& init_state){
  if (init_state.size() != NBX0){
    throw std::runtime_error("Expected initstate to have "+ std::to_string(NBX0)+ " elements");
  }
  
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", init_state.data());
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", init_state.data());
}

void MpcWrapper::set_xinit(const std::vector<double>& xinit){
  if (xinit.size() != NX) {
    throw std::runtime_error("Expected "+ std::to_string(NX)+" elements");
  }
  x_init_ = xinit;
}
void MpcWrapper::set_uinit(const std::vector<double>& uinit){
  if (uinit.size() != NU) {
    throw std::runtime_error("Expected "+ std::to_string(NU)+" elements");
  }
  u_init_ = uinit;
}

void MpcWrapper::set_yref(const std::vector<double>& xref, const std::vector<double>& uref){
  if (xref.size() != NX) {
    throw std::runtime_error("Expected "+ std::to_string(NX)+" elements");
  }
  if (uref.size() != NU) {
    throw std::runtime_error("Expected "+ std::to_string(NU)+" elements");
  }
  for (size_t i = 0; i< NX; ++i){
    y_ref_[i] = xref[i];
    x_ref_[i] = xref[i];
  }
  for (size_t i = 0; i<NU ;++i){
    y_ref_[NX+i] = uref[i];
    u_ref_[i] = uref[i];
  }
}

}  // namespace mpc_controller
