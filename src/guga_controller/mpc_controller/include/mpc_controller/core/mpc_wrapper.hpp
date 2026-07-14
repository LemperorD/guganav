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

namespace mpc_controller {

/**
 * @brief MPC 控制器配置结构体。
 */
struct MpcWrapperConfig
{
  int horizon_n;
  double control_dt;
  double vx_min, vx_max;
  double vy_min, vy_max;
  double omega_min, omega_max;
  CostWeights cost_weights;
} typedef MpcConfig;

struct CostWeights
{
  // 状态权重
  double qx; double qy; double qtheta;

  // 控制权重
  double rvx; double rvy; double romega;

  // 终端状态权重
  double qx_e; double qy_e; double qtheta_e;
} typedef CostWeights;

/**
 * @brief MPC 求解器封装类。
 */
class MpcWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐宏，确保在STL容器中安全使用

  explicit MpcWrapper(); // 构造函数
  ~MpcWrapper(); // 析构函数

  // 禁止复制
  MpcWrapper(const MpcWrapper &) = delete;
  MpcWrapper & operator=(const MpcWrapper &) = delete;

  /**
   * @brief 设置代价函数权重矩阵。
   * @param Q 状态权重矩阵 (NX x NX)
   * @param R 控制权重矩阵 (NU x NU)
   * @param QE 终端状态权重矩阵 (NX x NX)
   */
  void setCosts(
    const Eigen::Ref<const Eigen::Matrix<double, NX, NX>> Q,
    const Eigen::Ref<const Eigen::Matrix<double, NU, NU>> R,
    const Eigen::Ref<const Eigen::Matrix<double, NX, NX>> QE
  );

  /**
   * @brief 设置控制输入约束。
   * @param vx_min 最小线速度 x @param vx_max 最大线速度 x
   * @param vy_min 最小线速度 y @param vy_max 最大线速度 y
   * @param omega_min 最小角速度 @param omega_max 最大角速度
   */
  void setControlLimits(
    double vx_min, double vx_max, double vy_min, double vy_max,
    double omega_min, double omega_max
  );

  /**
   * @brief 设置初始状态约束。
   * @param init_state 初始状态向量 (NX)
   */
  void set_xinit(const std::vector<double>& x0);

  /**
   * @brief 设置初始控制量。
   * @param u0 初始控制量向量 (NU)
   */
  void set_uinit(const std::vector<double>& u0);

  /**
   * @brief 设置参考轨迹。
   * @param x_ref 参考状态轨迹 (NX)
   * @param u_ref 参考控制轨迹 (NU)
   */
  void set_yref(const std::vector<double>& x_ref, const std::vector<double>& u_ref);

  /**
   * @brief 设置当前状态。
   * @param x0 当前状态向量 (NX)
   */
  void set_x0(const std::vector<double>& x0);

  /**
   * @brief 求解MPC问题。
   * @return 最优控制量向量 (NU)
   */
  const std::vector<double>& solve();

  /**
   * @brief 求解MPC问题（使用之前的最优控制量作为参考）。
   * @param x0 当前状态向量 (NX)
   * @param x_des 目标状态向量 (NX)
   * @return 最优控制量向量 (NU)
   */
  const std::vector<double>& solve(std::vector<double>& x0, const std::vector<double>& x_des);

private:
  // 求解器相关
  int status_;
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

  // solver data
  // std::vector<double> x_traj_=std::vector<double>(NX * (N+1), 0.0);
  // std::vector<double> u_traj_=std::vector<double>(NU * N,  0.0);
  std::vector<double> x_init_=std::vector<double>(NX,  0.0);
  std::vector<double> u_init_=std::vector<double>(NU,  0.0);
  std::vector<double> x_ref_=std::vector<double>(NX,  0.0);
  std::vector<double> u_ref_=std::vector<double>(NU,  0.0);
  std::vector<double> y_ref_=std::vector<double>(NX + NU,  0.0);
  std::vector<double> u_opt_=std::vector<double>(NU,  0.0);

};

}  // namespace mpc_controller

#endif  // MPC_WRAPPER_HPP_
