#include "mpc_controller/trajectory/bspline_generator.hpp"
#include "mpc_controller/trajectory/discrete_generator.hpp"

#include <unsupported/Eigen/Splines>
#include <algorithm>
#include <cmath>

namespace mpc_controller
{

// ==========================================================================
ReferenceTrajectory BSplineGenerator::generate(
  const nav_msgs::msg::Path & path,
  int horizon_n,
  double dt,
  double lookahead)
{
  ctrl_points_.clear();
  knots_.clear();
  chord_params_.clear();
  chord_arc_lengths_.clear();
  total_arc_length_ = 0.0;

  const int n_pts = static_cast<int>(path.poses.size());
  if (n_pts < kDegree + 1) {
    DiscreteGenerator dg;
    return dg.generate(path, horizon_n, dt, lookahead);
  }

  // ---- 1. Chord-length 参数化 ----
  chord_params_.resize(n_pts);
  chord_arc_lengths_.resize(n_pts);
  chord_params_[0] = 0.0;
  chord_arc_lengths_[0] = 0.0;

  for (int i = 1; i < n_pts; ++i) {
    const double dx = path.poses[i].pose.position.x -
                      path.poses[i - 1].pose.position.x;
    const double dy = path.poses[i].pose.position.y -
                      path.poses[i - 1].pose.position.y;
    const double seg = std::hypot(dx, dy);
    total_arc_length_ += seg;
    chord_arc_lengths_[i] = total_arc_length_;
  }

  if (total_arc_length_ < 1e-6) { return {}; }

  for (int i = 1; i < n_pts; ++i) {
    chord_params_[i] = chord_arc_lengths_[i] / total_arc_length_;
  }
  chord_params_.back() = 1.0;

  // ---- 2. 构建 Eigen 插值点矩阵 ----
  Eigen::MatrixXd pts(2, n_pts);
  for (int j = 0; j < n_pts; ++j) {
    pts(0, j) = path.poses[j].pose.position.x;
    pts(1, j) = path.poses[j].pose.position.y;
  }

  // knot averaging（Eigen 自由函数）
  Eigen::RowVectorXd chord_vec = Eigen::RowVectorXd::Map(
    chord_params_.data(), n_pts);
  Eigen::RowVectorXd knot_vec;
  Eigen::KnotAveraging(chord_vec, kDegree, knot_vec);

  knots_.assign(knot_vec.data(), knot_vec.data() + knot_vec.size());

  // ---- 3. 拟合 B 样条 ----
  using Spline2D = Eigen::Spline<double, 2, kDegree>;
  Spline2D spline = Eigen::SplineFitting<Spline2D>::Interpolate(
    pts, kDegree, knot_vec);

  // 提取控制点
  const auto & ctrl = spline.ctrls();
  ctrl_points_.resize(ctrl.cols());
  for (int i = 0; i < static_cast<int>(ctrl.cols()); ++i) {
    ctrl_points_[i] = ctrl.col(i);
  }

  // ---- 4. 重建 spline 用于求值 ----
  Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> ctrl_mat(
    ctrl_points_[0].data(), 2, static_cast<int>(ctrl_points_.size()));
  Eigen::Map<const Eigen::VectorXd> knot_map(
    knots_.data(), static_cast<int>(knots_.size()));

  // ---- 5. 等距采样 ----
  ReferenceTrajectory ref_traj;
  ref_traj.reserve(horizon_n);

  const double v_typical = 1.0;
  const double window_len = lookahead + horizon_n * dt * v_typical;
  const double step_len = window_len / static_cast<double>(horizon_n);

  for (int k = 0; k < horizon_n; ++k) {
    const double s_k = lookahead + k * step_len;
    const double u_k = arcToParam(std::min(s_k, total_arc_length_));

    Spline2D eval_spline(knot_map, ctrl_mat);
    const Eigen::Vector2d pos = eval_spline(u_k);

    ReferencePoint pt;
    pt.s = s_k;
    pt.x = pos(0);
    pt.y = pos(1);

    // 切线 = 数值微分
    constexpr double h = 1e-3;
    const double u_p = std::min(u_k + h, 1.0);
    const double u_m = std::max(u_k - h, 0.0);
    const Eigen::Vector2d pp = eval_spline(u_p);
    const Eigen::Vector2d pm = eval_spline(u_m);
    const Eigen::Vector2d tan = (pp - pm) / (2.0 * h);

    pt.theta = std::atan2(tan(1), tan(0));

    // 曲率
    const double spd2 = tan.squaredNorm();
    if (spd2 > 1e-9) {
      const double dx = tan(0), dy = tan(1);
      const double u_pp = std::min(u_k + h, 1.0);
      const double u_mm = std::max(u_k - h, 0.0);
      const Eigen::Vector2d t_p = eval_spline(u_pp);
      const Eigen::Vector2d t_m = eval_spline(u_mm);
      const double ddx = (t_p(0) - 2.0 * pos(0) + t_m(0)) / (h * h);
      const double ddy = (t_p(1) - 2.0 * pos(1) + t_m(1)) / (h * h);
      pt.curvature = std::abs(dx * ddy - dy * ddx) / (spd2 * std::sqrt(spd2));
    } else {
      pt.curvature = 0.0;
    }

    // 参考速度
    constexpr double kappa_scale = 2.0;
    const double v_ref = 1.0 / (1.0 + kappa_scale * std::abs(pt.curvature));
    pt.vx = v_ref * std::cos(pt.theta);
    pt.vy = v_ref * std::sin(pt.theta);
    pt.omega = v_ref * pt.curvature;

    ref_traj.push_back(pt);
  }

  return ref_traj;
}

// ==========================================================================
double BSplineGenerator::curvature(double s) const
{
  if (ctrl_points_.empty() || total_arc_length_ < 1e-9) { return 0.0; }

  const double u = arcToParam(std::min(s, total_arc_length_));
  using Spline2D = Eigen::Spline<double, 2, kDegree>;
  Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> ctrl_mat(
    ctrl_points_[0].data(), 2, static_cast<int>(ctrl_points_.size()));
  Eigen::Map<const Eigen::VectorXd> knot_map(
    knots_.data(), static_cast<int>(knots_.size()));
  Spline2D spline(knot_map, ctrl_mat);

  constexpr double h = 1e-3;
  const double u_p = std::min(u + h, 1.0);
  const double u_m = std::max(u - h, 0.0);
  const Eigen::Vector2d pp = spline(u_p);
  const Eigen::Vector2d pm = spline(u_m);
  const double dx = pp(0) - pm(0);
  const double dy = pp(1) - pm(1);
  const double spd2 = dx * dx + dy * dy;

  if (spd2 < 1e-9) { return 0.0; }

  const Eigen::Vector2d p0 = spline(u);
  const double ddx = (pp(0) - 2.0 * p0(0) + pm(0));
  const double ddy = (pp(1) - 2.0 * p0(1) + pm(1));

  return std::abs(dx * ddy - dy * ddx) / (spd2 * std::sqrt(spd2));
}

// ==========================================================================
Eigen::Vector2d BSplineGenerator::tangent(double s) const
{
  if (ctrl_points_.empty()) { return {1.0, 0.0}; }
  constexpr double h = 1e-3;
  const double u = arcToParam(std::min(s, total_arc_length_));
  const double u_p = std::min(u + h, 1.0);
  const double u_m = std::max(u - h, 0.0);

  using Spline2D = Eigen::Spline<double, 2, kDegree>;
  Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> ctrl_mat(
    ctrl_points_[0].data(), 2, static_cast<int>(ctrl_points_.size()));
  Eigen::Map<const Eigen::VectorXd> knot_map(
    knots_.data(), static_cast<int>(knots_.size()));
  Spline2D spline(knot_map, ctrl_mat);
  return (spline(u_p) - spline(u_m)) / (2.0 * h);
}

// ==========================================================================
Eigen::Vector2d BSplineGenerator::secondDeriv(double s) const
{
  (void)s;
  return {0.0, 0.0};
}

}  // namespace mpc_controller
