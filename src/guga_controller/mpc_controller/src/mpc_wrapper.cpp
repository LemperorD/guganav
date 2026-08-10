#include "mpc_controller/mpc_wrapper.hpp"

#include <algorithm>

namespace mpc_controller {

// 构造
MpcWrapper::MpcWrapper()
{
  capsule_ = omni_acados_create_capsule();
  status_  = omni_acados_create_with_discretization(capsule_, kHorizonSteps, nullptr);
  if (status_) {
    std::fprintf(stderr, "[MpcWrapper] omni_acados_create() failed: %d\n", status_);
    std::exit(1);
  }

  cfg_  = omni_acados_get_nlp_config(capsule_);
  dims_ = omni_acados_get_nlp_dims(capsule_);
  in_   = omni_acados_get_nlp_in(capsule_);
  out_  = omni_acados_get_nlp_out(capsule_);
  slv_  = omni_acados_get_nlp_solver(capsule_);

  std::printf("[MpcWrapper] solver created. N=%d, nx=%d, nu=%d\n", kHorizonSteps, kStateSize, kInputSize);
}

// 析构
MpcWrapper::~MpcWrapper()
{
  if (capsule_) {
    status_ = omni_acados_free(capsule_);
    if (status_) std::fprintf(stderr, "[MpcWrapper] omni_acados_free() returned %d\n", status_);
    status_ = omni_acados_free_capsule(capsule_);
    if (status_) std::fprintf(stderr, "[MpcWrapper] omni_acados_free_capsule() returned %d\n", status_);
  }
}

// 移动语义
MpcWrapper::MpcWrapper(MpcWrapper &&other) noexcept
  : capsule_(other.capsule_),
    cfg_(other.cfg_),
    dims_(other.dims_),
    in_(other.in_),
    out_(other.out_),
    slv_(other.slv_),
    x0_(other.x0_),
    x_pred_(other.x_pred_),
    u_pred_(other.u_pred_),
    u_opt_(other.u_opt_),
    status_(other.status_),
    elapsed_time_(other.elapsed_time_),
    min_time_(other.min_time_),
    kkt_norm_inf_(other.kkt_norm_inf_),
    sqp_iter_(other.sqp_iter_)
{
  other.capsule_ = nullptr;
  other.cfg_    = nullptr;
  other.dims_   = nullptr;
  other.in_     = nullptr;
  other.out_    = nullptr;
  other.slv_    = nullptr;
}

MpcWrapper &MpcWrapper::operator=(MpcWrapper &&other) noexcept
{
  if (this != &other) {
    if (capsule_) {
      omni_acados_free(capsule_);
      omni_acados_free_capsule(capsule_);
    }
    capsule_ = other.capsule_;  other.capsule_ = nullptr;
    cfg_     = other.cfg_;      other.cfg_     = nullptr;
    dims_    = other.dims_;     other.dims_    = nullptr;
    in_      = other.in_;       other.in_      = nullptr;
    out_     = other.out_;      other.out_     = nullptr;
    slv_     = other.slv_;      other.slv_     = nullptr;

    x0_      = other.x0_;
    x_pred_  = other.x_pred_;
    u_pred_  = other.u_pred_;
    u_opt_   = other.u_opt_;
  }
  return *this;
}

// 设置代价矩阵
void MpcWrapper::setCostWeights(const Eigen::Matrix<double, kStateSize, kStateSize> &Q,
                                const Eigen::Matrix<double, kInputSize, kInputSize> &R,
                                const Eigen::Matrix<double, kEndRefSize, kEndRefSize> &QE)
{
  // 路径 cost: W = blkdiag(Q, R) ∈ R^{6×6}
  Eigen::Matrix<double, kRefSize, kRefSize> W = Eigen::Matrix<double, kRefSize, kRefSize>::Zero();
  W.block<3, 3>(0, 0) = Q;
  W.block<3, 3>(3, 3) = R;

  for (int stage = 0; stage < kHorizonSteps; ++stage) {
    ocp_nlp_cost_model_set(cfg_, dims_, in_, stage, "W", W.data());
  }

  // 终端 cost: W_e = QE ∈ R^{3×3}
  ocp_nlp_cost_model_set(cfg_, dims_, in_, kHorizonSteps, "W", const_cast<Eigen::Matrix<double, kEndRefSize, kEndRefSize> &>(QE).data());
}

void MpcWrapper::setControlLimits(double vx_min, double vx_max,
                                  double vy_min, double vy_max,
                                  double omega_min, double omega_max)
{
  InputBound lower_bu = {vx_min, vy_min, omega_min};
  InputBound upper_bu = {vx_max, vy_max, omega_max};

  for (int stage = 0; stage < kHorizonSteps; ++stage) {
    ocp_nlp_constraints_model_set(cfg_, dims_, in_, out_, stage, "lbu", lower_bu.data());
    ocp_nlp_constraints_model_set(cfg_, dims_, in_, out_, stage, "ubu", upper_bu.data());
  }
}

// 设置初始状态约束
void MpcWrapper::setInitialState(const State &x0)
{
  x0_ = x0;
  ocp_nlp_constraints_model_set(cfg_, dims_, in_, out_, 0, "lbx", x0_.data());
  ocp_nlp_constraints_model_set(cfg_, dims_, in_, out_, 0, "ubx", x0_.data());
}

// 设置参考轨迹
void MpcWrapper::setReferenceTrajectory(const RefHorizon &ref, const TerminalRef &ref_e)
{
  // 轨迹
  for (int stage = 0; stage < kHorizonSteps; ++stage) {
    RefVector yref = ref.col(stage);
    ocp_nlp_cost_model_set(cfg_, dims_, in_, stage, "yref", yref.data());
  }
  // 终端
  ocp_nlp_cost_model_set(cfg_, dims_, in_, kHorizonSteps, "yref", const_cast<TerminalRef &>(ref_e).data());
}

// 求解
Input MpcWrapper::solve()
{
  // 求解
  status_ = omni_acados_solve(capsule_);
  ocp_nlp_get(slv_, "time_tot", &solve_time_);
  // 保存预测状态序列
  for(int i=0;i<=kHorizonSteps;i++)
  {
    ocp_nlp_out_get(cfg_, dims_, out_, i, "x", x_pred_.col(i).data());
  }
  // 保存预测控制序列
  for(int i=0;i<=kHorizonSteps;i++)
  {
    ocp_nlp_out_get(cfg_, dims_, out_, i, "u", u_pred_.col(i).data());
  }

  if (status_ != ACADOS_SUCCESS) {
    std::fprintf(stderr, "[MpcWrapper] solve failed: status=%d\n", status_);
    return Input::Zero();
  }

  // 取 stage 0 的最优控制量
  ocp_nlp_out_get(cfg_, dims_, out_, 0, "u", u_opt_.data());
  return u_opt_;
}

}  // namespace mpc_controller
