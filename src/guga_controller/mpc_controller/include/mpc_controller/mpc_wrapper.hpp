#ifndef MPC_WRAPPER_HPP_
#define MPC_WRAPPER_HPP_

#include <cstdio>

// acados
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/utils/math.h"

// import omni model
extern "C" {
  #include "acados_solver_omni.h"
}

#include <Eigen/Dense>

// ─── 编译期维度常量 ───
static constexpr int kHorizonSteps   = OMNI_N;      // 预测步数 N
static constexpr int kStateSize      = OMNI_NX;     // 状态维度 nx = 3  [p_x, p_y, ψ]
static constexpr int kInputSize      = OMNI_NU;     // 控制维度 nu = 3  [v_x, v_y, ω]
static constexpr int kStateBoundSize = OMNI_NBX0;   // 初态约束维度
static constexpr int kInputBoundSize = OMNI_NBU;    // 控制约束维度
static constexpr int kRefSize        = OMNI_NY;     // 路径参考维度 ny = nx+nu = 6
static constexpr int kEndRefSize     = OMNI_NYN;    // 终端参考维度 ny_e = nx = 3

// ─── 类型别名 ───

/// 状态向量 [p_x, p_y, ψ]^T
using State  = Eigen::Vector<double, kStateSize>;
/// 控制向量 [v_x, v_y, ω]^T
using Input  = Eigen::Vector<double, kInputSize>;
/// 初始状态约束向量 (与状态同维, 但语义上用于等式约束的 bound)
using StateBound  = Eigen::Vector<double, kStateBoundSize>;
/// 控制约束向量
using InputBound  = Eigen::Vector<double, kInputBoundSize>;

/// 状态轨迹: 3 × (N+1), 每列是一个 stage 的状态
using StateHorizon  = Eigen::Matrix<double, kStateSize,   kHorizonSteps + 1>;
/// 控制轨迹: 3 × N, 每列是一个 stage 的控制
using InputHorizon  = Eigen::Matrix<double, kInputSize,   kHorizonSteps>;

/// 轨迹点向量
using RefVector     = Eigen::Vector<double, kRefSize>;
/// 路径参考矩阵: 6 × N, 每列为 [p_x, p_y, ψ, v_x, v_y, ω]^T
using RefHorizon    = Eigen::Matrix<double, kRefSize,     kHorizonSteps>;
/// 终端参考向量: 3 × 1, [p_x, p_y, ψ]^T
using TerminalRef   = Eigen::Vector<double, kEndRefSize>;

namespace mpc_controller {

struct CostWeights {
  double qx{10.0}, qy{10.0}, qtheta{2.0};                // 状态权重
  double rvx{0.1}, rvy{0.1}, romega{0.05};               // 控制权重
  double qx_e{10.0}, qy_e{10.0}, qtheta_e{2.0};          // 终端权重
};

struct MpcConfig {
  int    horizon_n{15};
  double control_dt{0.05};
  double vx_min{-3.0},   vx_max{3.0};
  double vy_min{-3.0},   vy_max{3.0};
  double omega_min{-6.0}, omega_max{6.0};
  CostWeights cost_weights;
};

/**
 * @brief acados MPC 求解器封装。
 *
 * 生命周期:
 *   1. 构造 → configure 阶段调 setCostWeights / setControlLimits
 *   2. 每帧: setInitialState → setReferenceTrajectory → solve → 取 Control
 *
 * 移动安全 (不可复制); Eigen 内存对齐已处理。
 */
class MpcWrapper {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcWrapper();
  ~MpcWrapper();

  // ── 禁止复制, 允许移动 ──
  MpcWrapper(const MpcWrapper &)            = delete;
  MpcWrapper &operator=(const MpcWrapper &) = delete;
  MpcWrapper(MpcWrapper &&other) noexcept;
  MpcWrapper &operator=(MpcWrapper &&other) noexcept;

  /// 设置代价函数权重 Q (3×3), R (3×3), QE (3×3)
  void setCostWeights(const Eigen::Matrix<double, kStateSize, kStateSize> &Q,
                      const Eigen::Matrix<double, kInputSize, kInputSize> &R,
                      const Eigen::Matrix<double, kEndRefSize, kEndRefSize> &QE);

  /// 设置控制输入约束
  void setControlLimits(double vx_min, double vx_max,
                        double vy_min, double vy_max,
                        double omega_min, double omega_max);

  // ────────────────────────────────────────────────
  // 每帧调用
  // ────────────────────────────────────────────────

  /// 设置 stage 0 的初始状态等式约束 (lbx = ubx = x0)
  void setInitialState(const StateBound &x0);

  /// 设置全 horizon 参考轨迹 (同时作为 warm-start)
  /// @param ref   6×N, 每列为 [px, py, ψ, vx, vy, ω]^T (stage 0..N-1)
  /// @param ref_e 3×1, 终端 [px, py, ψ]^T (stage N)
  void setReferenceTrajectory(const RefHorizon &ref,
                              const TerminalRef &ref_e);

  /// 求解 OCP, 返回最优控制量
  Input solve();

  /// 获取求解时间
  double solve_time() const { return solve_time_; }

  /// 获取预测状态轨迹
  const StateHorizon &predictedStates() const { return x_pred_; }

  /// 获取预测控制轨迹
  const InputHorizon &predictedInputs() const { return u_pred_; }

 private:
  // ── acados 内部句柄 ──
  omni_solver_capsule *capsule_{nullptr};
  ocp_nlp_config      *cfg_{nullptr};
  ocp_nlp_dims        *dims_{nullptr};
  ocp_nlp_in          *in_{nullptr};
  ocp_nlp_out         *out_{nullptr};
  ocp_nlp_solver      *slv_{nullptr};

  // ── 求解器状态 (每帧更新) ──
  State        x0_{State::Zero()};            // 当前初始状态约束
  StateHorizon x_pred_{StateHorizon::Zero()}; // 预测状态轨迹
  InputHorizon u_pred_{InputHorizon::Zero()}; // 预测控制轨迹

  // ── 输出 ──
  Input u_opt_{Input::Zero()};

  // ── 诊断 ──
  int    status_{0};
  double elapsed_time_{0.0};
  double solve_time_{0.0};
  double min_time_{1e12};
  double kkt_norm_inf_{0.0};
  int    sqp_iter_{0};
};

}  // namespace mpc_controller

#endif  // MPC_WRAPPER_HPP_
