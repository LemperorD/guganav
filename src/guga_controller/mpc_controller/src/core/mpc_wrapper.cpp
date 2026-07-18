#include "mpc_controller/core/mpc_wrapper.hpp"

namespace mpc_controller
{

MpcWrapper::MpcWrapper()
{
  // 创建求解器实例
  capsule_ = omni_acados_create_capsule();
  status_ = omni_acados_create_with_discretization(capsule_, kN, nullptr);
  if (status_) {
    std::printf("omni_acados_create() returned status %d. Exiting.\n", status_);
    exit(1);
  }

  // 获取求解器配置和维度信息
  cfg_  = omni_acados_get_nlp_config(capsule_);
  dims_ = omni_acados_get_nlp_dims(capsule_);
  in_   = omni_acados_get_nlp_in(capsule_);
  out_  = omni_acados_get_nlp_out(capsule_);
  slv_  = omni_acados_get_nlp_solver(capsule_);

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
    for (int i = 0; i < kN; i++)
    {
      ocp_nlp_out_set(cfg_, dims_, out_, in_, i, "x", x_init_.data());
      ocp_nlp_out_set(cfg_, dims_, out_, in_, i, "u", u_init_.data());
      ocp_nlp_cost_model_set(cfg_, dims_, in_, i, "yref", y_ref_.data());
    }
    ocp_nlp_out_set(cfg_, dims_, out_, in_, kN, "x", x_init_.data());
    ocp_nlp_cost_model_set(cfg_, dims_, in_, kN, "yref", y_ref_.data());
    status_ = omni_acados_solve(capsule_);
    ocp_nlp_get(slv_, "time_tot", &elapsed_time_);
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
  ocp_nlp_out_get(cfg_, dims_, out_, 0, "u", u_opt_.data());
  
  ocp_nlp_out_get(cfg_, dims_, out_, 0, "kkt_norm_inf", &kkt_norm_inf_);
  ocp_nlp_get(slv_, "sqp_iter", &sqp_iter_);

  return u_opt_;
}

const std::vector<double>& MpcWrapper::solve(std::vector<double>& x0, const std::vector<double>& x_des){
    set_uinit(u_opt_);
    set_xinit(x0);
    set_x0(x0);
    set_yref(x_des, u_opt_);
    return solve();
}

void MpcWrapper::set_x0(const std::vector<double>& x0){
  if (x0.size() != kNBX0){
    throw std::runtime_error("Expected initstate to have "+ std::to_string(kNBX0)+ " elements");
  }
  
  ocp_nlp_constraints_model_set(cfg_, dims_, in_, out_, 0, "lbx", const_cast<double*>(x0.data()));
  ocp_nlp_constraints_model_set(cfg_, dims_, in_, out_, 0, "ubx", const_cast<double*>(x0.data()));
}

void MpcWrapper::set_xinit(const std::vector<double>& xinit){
  if (xinit.size() != kNX) {
    throw std::runtime_error("Expected "+ std::to_string(kNX)+" elements");
  }
  x_init_ = xinit;
}
void MpcWrapper::set_uinit(const std::vector<double>& uinit){
  if (uinit.size() != kNU) {
    throw std::runtime_error("Expected "+ std::to_string(kNU)+" elements");
  }
  u_init_ = uinit;
}

void MpcWrapper::set_yref(const std::vector<double>& xref, const std::vector<double>& uref){
  if (xref.size() != kNX) {
    throw std::runtime_error("Expected "+ std::to_string(kNX)+" elements");
  }
  if (uref.size() != kNU) {
    throw std::runtime_error("Expected "+ std::to_string(kNU)+" elements");
  }
  for (size_t i = 0; i< kNX; ++i){
    y_ref_[i] = xref[i];
    x_ref_[i] = xref[i];
  }
  for (size_t i = 0; i<kNU ;++i){
    y_ref_[kNX+i] = uref[i];
    u_ref_[i] = uref[i];
  }
}

void MpcWrapper::setCosts(
  const Eigen::Ref<const Eigen::Matrix<double, kNX, kNX>> Q,
  const Eigen::Ref<const Eigen::Matrix<double, kNU, kNU>> R,
  const Eigen::Ref<const Eigen::Matrix<double, kNX, kNX>> QE
) {
  Eigen::Matrix<double,kNX+kNU,kNX+kNU> W;
  W.setZero();

  W.block<kNX,kNX>(0,0) = Q;
  W.block<kNU,kNU>(kNX,kNX) = R;

  for(int stage=0;stage<kN;++stage) {
    ocp_nlp_cost_model_set(cfg_, dims_, in_, stage, "W", W.data());
  }

  // ocp_nlp_cost_model_set expects a void* (non-const), but QE.data() is const double*
  // cast away const to match the API. QE is not modified by this call.
  ocp_nlp_cost_model_set(cfg_, dims_, in_, kN, "W", const_cast<double*>(QE.data()));
}

void MpcWrapper::setControlLimits(
  double vx_min, double vx_max, double vy_min, double vy_max, double omega_min, double omega_max
) {
  double lbu[kNU] = {vx_min, vy_min, omega_min};
  double ubu[kNU] = {vx_max, vy_max, omega_max};

  for(int stage=0; stage<kN; ++stage)
  {
    ocp_nlp_constraints_model_set(cfg_, dims_, in_, out_, stage, "lbu", lbu);
    ocp_nlp_constraints_model_set(cfg_, dims_, in_, out_, stage, "ubu", ubu);
  }
}

}  // namespace mpc_controller
