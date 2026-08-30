#include "bspline_opt/bspline_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <unsupported/Eigen/Splines>

#include "bspline_opt/detail/cost_cache.hpp"
#include "bspline_opt/detail/cost_function.hpp"
#include "bspline_opt/detail/gradient_descent.hpp"
#include "bspline_opt/detail/grid_utils.hpp"

namespace bspline_opt
{

using namespace bspline_opt::detail;

BSplineOptimizer::BSplineOptimizer(const BSplineConfig & config)
: config_(config)
{
}

bool BSplineOptimizer::fit(
  const std::vector<std::pair<double, double>> & path)
{
  fitted_ = false;
  state_ = {};
  spline_.reset();

  const int n_pts = static_cast<int>(path.size());
  if (n_pts < 2) {return false;}

  // ── 短路径回退 (线性) ──
  if (n_pts < 8) {
    state_.effective_degree = 1;
    state_.original_points.reserve(n_pts);
    for (const auto & [x, y] : path) {
      state_.original_points.emplace_back(x, y);
    }
    state_.control_points.resize(2, 2);
    state_.control_points(0, 0) = path.front().first;
    state_.control_points(1, 0) = path.front().second;
    state_.control_points(0, 1) = path.back().first;
    state_.control_points(1, 1) = path.back().second;
    fitted_ = true;
    return true;
  }

  const int eff_deg = 7;
  state_.effective_degree = eff_deg;

  state_.original_points.reserve(n_pts);
  for (const auto & [x, y] : path) {
    state_.original_points.emplace_back(x, y);
  }

  // Chord-length 参数化
  std::vector<double> arc_lengths(n_pts);
  arc_lengths[0] = 0.0;
  for (int i = 1; i < n_pts; ++i) {
    double dx = path[i].first - path[i - 1].first;
    double dy = path[i].second - path[i - 1].second;
    arc_lengths[i] = arc_lengths[i - 1] + std::hypot(dx, dy);
  }
  state_.total_arc_length = arc_lengths.back();
  if (state_.total_arc_length < 1e-9) {return false;}

  state_.parameters.resize(n_pts);
  for (int i = 0; i < n_pts; ++i) {
    state_.parameters[i] = arc_lengths[i] / state_.total_arc_length;
  }
  state_.parameters[n_pts - 1] = 1.0;

  // 点矩阵 (2 × N)
  Eigen::MatrixXd pts(2, n_pts);
  for (int i = 0; i < n_pts; ++i) {
    pts(0, i) = path[i].first;
    pts(1, i) = path[i].second;
  }

  // 节点平均
  Eigen::RowVectorXd chord_vec(n_pts);
  for (int i = 0; i < n_pts; ++i) {
    chord_vec(i) = state_.parameters(i);
  }
  Eigen::RowVectorXd knot_vec;
  Eigen::KnotAveraging(chord_vec, eff_deg, knot_vec);
  state_.knots = knot_vec;

  // ── 第1步: 通过 SplineFitting 插值全部 N 个航点 ──
  using SplineFitter =
    Eigen::SplineFitting<Eigen::Spline<double, 2, 7>>;
  auto dense_spline = SplineFitter::Interpolate(pts, eff_deg);

  // ── 第2步: 确定控制点数量 ──
  int M = std::min(config_.max_control_points, n_pts);
  if (M < 8) {M = 8;}
  if (M > n_pts) {M = n_pts;}

  // ── 第3步: 在 chord-length 参数处重新采样 M 个控制点 ──
  // 不使用均匀间距, 而是沿 ARC LENGTH 均匀选取原始航点子集对应的参数。
  // 这样可保留路径的几何形状。
  state_.control_points.resize(2, M);
  int best_idx = 0;
  for (int i = 0; i < M; ++i) {
    // 将索引 i (0..M-1) 映射到弧长比例, 再找到最近的 chord 参数
    double target_arc_frac = static_cast<double>(i) / static_cast<double>(M - 1);
    // parameters 单调递增, 用双指针线性扫描代替 O(M·N) 全量搜索。
    while (best_idx + 1 < n_pts &&
      std::abs(state_.parameters(best_idx + 1) - target_arc_frac) <
      std::abs(state_.parameters(best_idx) - target_arc_frac))
    {
      ++best_idx;
    }
    // 在此航点的 chord-length 参数处对密集样条求值
    double u = state_.parameters(best_idx);
    Eigen::Vector2d p = dense_spline(u);
    state_.control_points(0, i) = p.x();
    state_.control_points(1, i) = p.y();
  }

  // ── 第4步: 为 M 个控制点创建节点向量 ──
  // 使用从 M 个航点子集中均匀间隔的 chord-length
  Eigen::RowVectorXd sub_chord(M);
  for (int i = 0; i < M; ++i) {
    sub_chord(i) = static_cast<double>(i) / static_cast<double>(M - 1);
  }
  Eigen::RowVectorXd new_knots;
  Eigen::KnotAveraging(sub_chord, eff_deg, new_knots);
  state_.knots = new_knots;

  // 保存初始控制点位置 (flat doubles), 用于走廊约束
  state_.initial_params.clear();
  state_.initial_params.reserve(2 * (M - 2));
  for (int i = 1; i < M - 1; ++i) {
    state_.initial_params.push_back(state_.control_points(0, i));
    state_.initial_params.push_back(state_.control_points(1, i));
  }

  rebuildSpline();
  fitted_ = true;
  return true;
}

void BSplineOptimizer::rebuildSpline()
{
  if (state_.control_points.cols() < state_.effective_degree + 1) {return;}
  spline_ = std::make_unique<Spline2D>(state_.knots, state_.control_points);
}

std::vector<std::pair<double, double>> BSplineOptimizer::sample(int N) const
{
  std::vector<std::pair<double, double>> out{};
  if (!fitted_) {return out;}
  if (!spline_) {
    // Linear fallback: 端点间线性插值
    if (state_.original_points.empty()) {return out;}
    out.reserve(N);
    double x0 = state_.original_points.front().x();
    double y0 = state_.original_points.front().y();
    double x1 = state_.original_points.back().x();
    double y1 = state_.original_points.back().y();
    for (int i = 0; i < N; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(N - 1);
      out.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
    }
    return out;
  }
  out.reserve(N);
  for (int i = 0; i < N; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(N - 1);
    Eigen::Vector2d p = (*spline_)(u);
    out.emplace_back(p.x(), p.y());
  }
  return out;
}

double BSplineOptimizer::curvatureAt(double u) const
{
  if (!fitted_ || !spline_) {return 0.0;}
  Eigen::Matrix<double, 2, 3> derivs = spline_->derivatives<2>(u);
  double dx = derivs(0, 1);
  double dy = derivs(1, 1);
  double ddx = derivs(0, 2);
  double ddy = derivs(1, 2);
  double denom = dx * dx + dy * dy;
  if (denom < 1e-12) {return 0.0;}
  return std::abs(dx * ddy - dy * ddx) / (denom * std::sqrt(denom));
}

double BSplineOptimizer::computeCurvatureEnergy() const
{
  if (!fitted_ || !spline_) {return 0.0;}
  double energy{};
  constexpr int K = 200;
  for (int i = 0; i <= K; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(K);
    auto derivs = spline_->derivatives<2>(u);
    double ddx = derivs(0, 2);
    double ddy = derivs(1, 2);
    energy += (ddx * ddx + ddy * ddy);
  }
  return energy / static_cast<double>(K + 1);
}

BSplineResult BSplineOptimizer::optimize(int num_samples)
{
  BSplineResult result{};
  // spline_ 可能为空 (短路径线性兜底分支): 由 sample() 的线性回退处理
  if (!fitted_) {return result;}

  const int M = static_cast<int>(state_.control_points.cols());
  if (M < 3) {
    result.smoothed_path = sample(num_samples);
    result.curvature_profile.resize(num_samples);
    for (int i = 0; i < num_samples; ++i) {
      double u = static_cast<double>(i) / static_cast<double>(num_samples - 1);
      result.curvature_profile[i] = curvatureAt(u);
    }
    result.total_curvature_energy = computeCurvatureEnergy();
    result.converged = true;
    return result;
  }

  result.cost_initial = computeCurvatureEnergy();

  // ── 第1步: 可选梯度下降 ──
  if (config_.enable_gradient_descent) {
    const int n_interior = M - 2;
    const int param_size = 2 * n_interior;
    std::vector<double> params(param_size);
    for (int i = 0; i < n_interior; ++i) {
      params[2 * i] = state_.control_points(0, i + 1);
      params[2 * i + 1] = state_.control_points(1, i + 1);
    }

    double fx = state_.control_points(0, 0);
    double fy = state_.control_points(1, 0);
    double lx = state_.control_points(0, M - 1);
    double ly = state_.control_points(1, M - 1);

    bool converged{};
    int iters_out{};
    const auto cc = buildCostCache(
      state_.knots, M, state_.original_points, state_.parameters);
    // 归一化标度: 各代价项除以其初始值, 使权重无量纲且与路径长度/格网无关
    double js0{};
    double jd0{};
    double je0{};
    evalTerms(
      params, cc, fx, fy, lx, ly,
      state_.esdf_distance, state_.esdf_w, state_.esdf_h,
      state_.esdf_resolution, state_.esdf_origin_x, state_.esdf_origin_y,
      state_.esdf_max_distance, config_.esdf_safe_distance,
      js0, jd0, je0);
    if (js0 < 1e-12) {js0 = 1.0;}
    if (jd0 < 1e-12) {jd0 = 1.0;}
    if (je0 < 1e-12) {je0 = 1.0;}
    auto opt = gradientDescent(
      params, cc,
      fx, fy, lx, ly,
      config_.smoothness_weight, config_.distance_weight,
      state_.esdf_distance, state_.esdf_gradient_x, state_.esdf_gradient_y,
      state_.esdf_w, state_.esdf_h,
      state_.esdf_resolution, state_.esdf_origin_x, state_.esdf_origin_y,
      state_.esdf_max_distance,
      config_.esdf_safe_distance, config_.esdf_weight,
      js0, jd0, je0,
      config_.max_iterations, config_.corridor_halfwidth, converged, iters_out);
    result.iterations = iters_out;

    for (int i = 0; i < n_interior; ++i) {
      state_.control_points(0, i + 1) = opt[2 * i];
      state_.control_points(1, i + 1) = opt[2 * i + 1];
    }
  }

  // ── 第2步: 控制点的障碍物避让投射 (螺旋搜索) ──
  if (state_.costmap_data != nullptr) {
    for (int i = 1; i < M - 1; ++i) {
      double & cx = state_.control_points(0, i);
      double & cy = state_.control_points(1, i);
      projectPointToFree(
        state_.costmap_data, state_.costmap_w,
        state_.costmap_h, cx, cy);
    }
  }

  rebuildSpline();

  // ── 第3步: 采样输出路径 ──
  auto path = sample(num_samples);

  // ── 第4步: 采样点障碍物投射 (与第2步的控制点投射互补) ──
  // 控制点投影只保证控制点不落入障碍格元, 采样曲线仍可能擦边切角。
  // 先把落在障碍格元内的采样点投射到最近空闲格元; 再按 ≤0.5 格步长
  // 检查相邻采样点之间的线段, 把仍穿过障碍的加密点也投射出去。
  // 这样输出路径可通过 planner 的碰撞检查, 不再轻易回退到线性折线。
  if (state_.costmap_data != nullptr) {
    for (auto & p : path) {
      projectPointToFree(
        state_.costmap_data, state_.costmap_w, state_.costmap_h,
        p.first, p.second);
    }

    // 对投影产生的长线段统一按 ≤0.5 格加密, 落入障碍的加密点一并投射,
    // 保证相邻输出点间距足够小且均位于空闲格元, 下游碰撞检查不再回退。
    constexpr double kRefineStepCells = 0.5;
    for (int pass = 0; pass < 8; ++pass) {
      bool projected_any = false;
      bool has_long = false;
      std::vector<std::pair<double, double>> refined;
      refined.reserve(path.size() * 2);
      for (size_t i = 0; i < path.size(); ++i) {
        refined.push_back(path[i]);
        if (i + 1 >= path.size()) {break;}
        const double x0 = path[i].first;
        const double y0 = path[i].second;
        const double x1 = path[i + 1].first;
        const double y1 = path[i + 1].second;
        const double len = std::hypot(x1 - x0, y1 - y0);
        const int steps = std::max(
          1, static_cast<int>(std::ceil(len / kRefineStepCells)));
        if (steps > 1) {has_long = true;}
        for (int s = 1; s < steps; ++s) {
          const double t = static_cast<double>(s) / steps;
          auto pt = std::make_pair(
            x0 + t * (x1 - x0), y0 + t * (y1 - y0));
          if (inObstacle(
              state_.costmap_data, state_.costmap_w, state_.costmap_h,
              pt.first, pt.second))
          {
            projectPointToFree(
              state_.costmap_data, state_.costmap_w, state_.costmap_h,
              pt.first, pt.second);
            projected_any = true;
          }
          refined.push_back(pt);
        }
      }
      path = std::move(refined);
      if (!projected_any && !has_long) {break;}
    }
  }

  result.smoothed_path = std::move(path);
  result.curvature_profile.resize(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(num_samples - 1);
    result.curvature_profile[i] = curvatureAt(u);
  }
  result.total_curvature_energy = computeCurvatureEnergy();
  result.cost_final = result.total_curvature_energy;
  result.converged = true;

  result.control_points_xy.reserve(M);
  for (int i = 0; i < M; ++i) {
    result.control_points_xy.emplace_back(
      state_.control_points(0, i), state_.control_points(1, i));
  }
  return result;
}


}  // namespace bspline_opt
