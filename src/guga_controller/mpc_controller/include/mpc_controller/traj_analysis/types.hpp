#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mpc_controller
{

// ==========================================================================
// 轨迹生成模式
// ==========================================================================
enum class TrajectoryMode : uint8_t {
  DISCRETE = 0,   // Nav2 Path 航点线性插值
  B_SPLINE = 1,   // Eigen B 样条全局拟合
  MINCO = 2,      // MINCO 能量最优轨迹优化
};

// ==========================================================================
// 参考轨迹点（统一内部表示）
// ==========================================================================
struct ReferencePoint {
  double x{};
  double y{};
  double theta{};
  double vx{};
  double vy{};
  double omega{};
  double curvature{};
  double s{};         // 弧长参数
};

using ReferenceTrajectory = std::vector<ReferencePoint>;

// ==========================================================================
// MPC 配置 (≡ Mode A `Config` struct, 全 public)
// ==========================================================================
struct MpcConfig {
  // ---- 时域 ----
  int horizon_n{15};                  // 预测时域步数 N
  double control_dt{0.05};           // 控制步长 [s]

  // ---- 状态跟踪权重 ----
  double qx{10.0};                    // x 位置权重
  double qy{10.0};                    // y 位置权重
  double qtheta{2.0};                 // theta 角度权重

  // ---- 控制代价权重 ----
  double rvx{0.1};                    // vx 代价权重
  double rvy{0.1};                    // vy 代价权重
  double romega{0.05};                // omega 代价权重

  // ---- 控制平滑权重 ----
  double rdvx{0.5};                   // Δvx 变化率权重
  double rdvy{0.5};                   // Δvy 变化率权重
  double rdomega{0.3};                // Δomega 变化率权重

  // ---- 控制约束 ----
  double vx_min{-3.0};                // [m/s]
  double vx_max{3.0};
  double vy_min{-3.0};
  double vy_max{3.0};
  double omega_min{-6.0};             // [rad/s]
  double omega_max{6.0};

  // ---- 轨迹生成 ----
  TrajectoryMode trajectory_mode{TrajectoryMode::DISCRETE};
  double lookahead_min{0.2};          // [m] 最小前瞻距离
  double lookahead_velocity_gain{1.0};// [s] 速度比例系数
  double path_max_length{5.0};        // [m] 路径截断长度
  double transform_tolerance{0.5};    // [s] TF 变换超时
};

// ==========================================================================
// MPC 运行时状态
// ==========================================================================
struct MpcState {
  double last_vx{};
  double last_vy{};
  double last_omega{};
};

// ==========================================================================
// MPC QP 矩阵容器
// ==========================================================================
struct MpcMatrices {
  int n_vars{0};               // 优化变量数 = horizon_n * 3
  int n_constraints{0};        // 约束数 = horizon_n * 6 (上下界各3)
  std::vector<double> H;       // Hessian (n_vars × n_vars, row-major)
  std::vector<double> g;       // gradient (n_vars)
  std::vector<double> lb;      // 变量下界
  std::vector<double> ub;      // 变量上界
  std::vector<double> solution; // ΔU 最优解
};

}  // namespace mpc_controller
