#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

namespace bspline_opt
{

/**
 * @brief B-spline 轨迹优化的不可变配置。
 *
 * @note  默认值已调校: fit() 生成精确插值样条,
 *        optimize() 仅执行障碍物避让后投射。
 *        梯度下降默认关闭 (精确插值样条无需平滑)。
 */
struct BSplineConfig
{
  int degree{7};                   // 阶数, 与 Eigen Spline 类型固定匹配

  // ── 优化权重 (仅在启用梯度下降时使用) ──
  double smoothness_weight{0.1};    // 低: 不主导插值精度
  double distance_weight{10.0};     // 低: 样条已精确插值
  double obstacle_weight{50000.0};  // 高: 严格障碍物避让

  // ── 调校参数 ──
  int max_iterations{200};          // 梯度下降最大迭代次数
  bool enable_gradient_descent{false}; // 默认关闭 — 插值已是最优
  double corridor_halfwidth{2.5};   // 可移动走廊半宽度 (格元)
  int max_control_points{200};      // 默认使用全部插值控制点
};

/**
 * @brief fit() 构造、optimize() 修改的内部状态。
 */
struct BSplineState
{
  /** 控制点 (2 × M): 第 i 列存储 (x_i, y_i)。 */
  Eigen::Matrix2Xd control_points{};

  /** 节点向量 (长度 = M + 阶数 + 1)。必须为 RowVectorXd 类型。 */
  Eigen::RowVectorXd knots{};

  /** 每个原始路径点的 chord-length 参数, 值域 [0, 1]。 */
  Eigen::VectorXd parameters{};

  /** 优化前的初始控制点位置 (用于梯度下降时的走廊约束)。 */
  std::vector<double> initial_params{};

  /** chord-length 参数化得到的总弧长 (单位: 格元)。 */
  double total_arc_length{};

  /** 原始路径点 (地图坐标), 用于距离代价。 */
  std::vector<Eigen::Vector2d> original_points{};

  /** 拟合时实际使用的阶数 (对短路径会自动降低)。 */
  int effective_degree{};

  /** @brief 可选的代价地图网格, 用于障碍物避让 (cost ≥ 253 = 障碍物)。
   *  指针非拥有 — 调用者必须保持其生命周期。nullptr = 不检查障碍物。 */
  const unsigned char * costmap_data{nullptr};
  int costmap_w{};
  int costmap_h{};
};

/**
 * @brief optimize() 的输出, 包含平滑路径和各项指标。
 */
struct BSplineResult
{
  std::vector<std::pair<double, double>> smoothed_path{};
  std::vector<double> curvature_profile{};
  std::vector<std::pair<double, double>> control_points_xy{};
  double total_curvature_energy{};
  int iterations{};
  double cost_initial{};
  double cost_final{};
  bool converged{};
};

/**
 * @class BSplineOptimizer
 * @brief B-spline 拟合, 支持可选的梯度下降优化。
 *
 * ## 默认算法 (不启用梯度下降)
 *
 * ```
 *   输入: JPS 航点 (N 个)
 *        │
 *        ▼
 *   SplineFitting::Interpolate  ──→  精确穿过所有航点的密集 B-spline
 *        │
 *        ▼ (提取 N 个控制点)
 *   BSplineState::control_points
 *        │
 *        ▼ (控制点 + 采样点的障碍物避让后投射)
 *   rebuildSpline() → sample(N_out) → BSplineResult
 * ```
 *
 * ## 代价函数 (仅在启用梯度下降时)
 *   J(P) = w_s·∫‖C''‖² du + w_d·Σ‖C(τᵢ)–qᵢ‖² + w_o·Σ penalty(障碍物)
 *
 * @see docs/DESIGN.md 查看完整设计原理。
 */
class BSplineOptimizer
{
public:
  explicit BSplineOptimizer(const BSplineConfig & config = BSplineConfig{});

  /**
   * @brief 通过 SplineFitting::Interpolate 将 7 阶 B-spline 拟合到 JPS 航点。
   *
   * 所有航点被精确插值 (控制点数 = N)。不做降采样 —
   * 调用 optimize() 进行障碍物避让处理。
   *
   * @param path  地图坐标下的 JPS 航点 (≥8 个方可使用完整 B-spline)。
   * @return 拟合成功返回 true。
   */
  [[nodiscard]] bool fit(
    const std::vector<std::pair<double, double>> & path);

  /**
   * @brief 对样条进行后处理, 确保避障。
   *
   * 梯度下降关闭时 (默认):
   *   — 将落在障碍物格元内的控制点投射到最近空闲格元
   *   — 对采样路径点逐一检查并修正障碍物冲突
   *   — 返回包含曲率曲线和统计信息的 BSplineResult
   *
   * 梯度下降开启时:
   *   — 先用数值梯度下降优化内部控制点位置
   *   — 再执行同上后处理
   *
   * @param num_samples  输出航点数量。
   * @return 优化结果。
   */
  [[nodiscard]] BSplineResult optimize(int num_samples = 200);

  /** @brief 在 N 个均匀参数 u ∈ [0,1] 处采样当前样条。 */
  [[nodiscard]] std::vector<std::pair<double, double>> sample(int N) const;

  /** @brief 计算参数 u ∈ [0, 1] 处的曲率 κ。 */
  [[nodiscard]] double curvatureAt(double u) const;

  /** @brief fit() 是否成功。 */
  [[nodiscard]] bool isFitted() const { return fitted_; }

  /** @brief 访问内部状态 (只读 / 可变)。 */
  [[nodiscard]] const BSplineState & state() const { return state_; }
  [[nodiscard]] BSplineState & state() { return state_; }
  [[nodiscard]] const BSplineConfig & config() const { return config_; }

  /** @brief 计算积分曲率能量 ∫₀¹ ‖C''(u)‖² du (数值求积)。 */
  [[nodiscard]] double computeCurvatureEnergy() const;

private:
  BSplineConfig config_{};
  BSplineState state_{};
  bool fitted_{false};

  using Spline2D = Eigen::Spline<double, 2, 7>;

  void rebuildSpline();

  std::unique_ptr<Spline2D> spline_{};
};

}  // namespace bspline_opt
