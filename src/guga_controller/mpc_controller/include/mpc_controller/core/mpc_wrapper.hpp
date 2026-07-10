#ifndef MPC_WRAPPER_HPP_
#define MPC_WRAPPER_HPP_

// acados
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/utils/math.h"
#include "acados/utils/print.h"

// import omni model
extern "C" {
  #include "acados_solver_omni.h"
}

// import omni model dimensions
#define NX   OMNI_NX
#define NU   OMNI_NU
#define N    OMNI_N
#define NY   OMNI_NY
#define NYN  OMNI_NYN
#define NBX0 OMNI_NBX0

#include <Eigen/Dense>

namespace mpc_controller
{

class MpcWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐宏，确保在STL容器中安全使用

  explicit MpcWrapper(); // 构造函数
  ~MpcWrapper(); // 析构函数

  // 不允许复制
  MpcWrapper(const MpcWrapper &) = delete;
  MpcWrapper & operator=(const MpcWrapper &) = delete;

  // 设置代价权重
  void setCosts(
    const Eigen::Ref<const Eigen::Matrix<double, NX, NX>> Q,
    const Eigen::Ref<const Eigen::Matrix<double, NU, NU>> R,
    const Eigen::Ref<const Eigen::Matrix<double, NX, NX>> QE
  );

  // 设置控制量边界条件
  void setControlLimits(
    double vx_min, double vx_max, double vy_min, double vy_max,
    double omega_min, double omega_max
  );

  // 设置初始条件与参考点
  void setup_solver(const double x_cur[3], TrajectoryType type, double t_start, double dt);

  const std::vector<double>& solve();

  // uses previous u_opt as u_ref
  const std::vector<double>& solve(std::vector<double>& x0, const std::vector<double>& x_des);

private:
  omni_solver_capsule *capsule_;
  ocp_nlp_config *cfg_;
  ocp_nlp_dims *dims_;
  ocp_nlp_in *in_;
  ocp_nlp_out *out_;
  ocp_nlp_solver *slv_;

  // solver metrics
  int NTIMINGS_ = 1;
  double min_time_ = 1e12;
  double kkt_norm_inf_;
  double elapsed_time_;
  int sqp_iter_;

  std::vector<double> xtraj_=std::vector<double>(NX * (N_+1), 0.0);
  std::vector<double> utraj_=std::vector<double>(NU * N_,  0.0);
  std::vector<double> x_init_=std::vector<double>(NX,  0.0);
  std::vector<double> u_init_=std::vector<double>(NU,  0.0);
  std::vector<double> x_ref_=std::vector<double>(NX,  0.0);
  std::vector<double> u_ref_=std::vector<double>(NU,  0.0);
  std::vector<double> y_ref_=std::vector<double>(NX + NU,  0.0);
  std::vector<double> u_opt_=std::vector<double>(NU,  0.0);

};

}  // namespace mpc_controller

#endif  // MPC_WRAPPER_HPP_
