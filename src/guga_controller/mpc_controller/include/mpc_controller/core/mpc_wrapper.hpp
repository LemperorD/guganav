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
  bool setCosts(
    const Eigen::Ref<const Eigen::Matrix<double, NX, NX>> Q,
    const Eigen::Ref<const Eigen::Matrix<double, NU, NU>> R,
    const Eigen::Ref<const Eigen::Matrix<double, NX, NX>> QE
  );

  // 设置边界条件
  bool setLimits(
    double vx_min, double vx_max, double vy_min, double vy_max,
    double omega_min, double omega_max
  );

  void setup_solver(
    ocp_nlp_config *cfg, ocp_nlp_dims *dims,
    ocp_nlp_in *in, ocp_nlp_out *out,
    const double x_cur[3], TrajectoryType type, double t_start, double dt);

private:
  omni_solver_capsule *capsule_;
  ocp_nlp_config *cfg_;
  ocp_nlp_dims *dims_;
  ocp_nlp_in *in_;
  ocp_nlp_out *out_;
  ocp_nlp_solver *slv_;

};

}  // namespace mpc_controller

#endif  // MPC_WRAPPER_HPP_
