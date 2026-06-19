#include "mpc_controller/trajectory/discrete_generator.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_controller
{

// ==========================================================================
ReferenceTrajectory DiscreteGenerator::generate(
  const nav_msgs::msg::Path & path,
  int horizon_n,
  double dt,
  double lookahead)
{
  waypoints_.clear();
  total_arc_length_ = 0.0;

  if (path.poses.size() < 2) { return {}; }

  // ---- 1. 构建弧长参数化航点序列 ----
  waypoints_.push_back({
    path.poses[0].pose.position.x,
    path.poses[0].pose.position.y,
    0.0, 0.0});

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x -
                      path.poses[i - 1].pose.position.x;
    const double dy = path.poses[i].pose.position.y -
                      path.poses[i - 1].pose.position.y;
    const double seg = std::hypot(dx, dy);
    total_arc_length_ += seg;

    const double theta = std::atan2(dy, dx);
    waypoints_.push_back({
      path.poses[i].pose.position.x,
      path.poses[i].pose.position.y,
      theta,
      total_arc_length_});
  }

  if (total_arc_length_ < 1e-6) { return {}; }

  // ---- 2. 计算预测时域内的采样步长 ----
  // 预测窗口总长度 = lookahead + N * dt * v_max
  const double v_typical = 1.0;  // 典型速度 [m/s]
  const double window_len = lookahead + horizon_n * dt * v_typical;
  const double step_len = window_len / static_cast<double>(horizon_n);

  // ---- 3. 在弧长上等距采样 ----
  ReferenceTrajectory ref_traj;
  ref_traj.reserve(horizon_n);

  for (int k = 0; k < horizon_n; ++k) {
    const double s_k = lookahead + k * step_len;
    const size_t idx = findIndexAtArcLength(s_k);

    ReferencePoint pt;
    if (idx >= waypoints_.size() - 1) {
      // 超出路径范围 — 用最后一点
      const auto & last = waypoints_.back();
      pt.x = last.x;
      pt.y = last.y;
      pt.theta = last.theta;
      pt.s = s_k;
    } else {
      // 在两个航点之间线性插值
      const auto & w0 = waypoints_[idx];
      const auto & w1 = waypoints_[idx + 1];
      const double ds = w1.s - w0.s;
      const double alpha = (ds > 1e-9) ? (s_k - w0.s) / ds : 0.0;
      alpha < 0.0 ? 0.0 : (alpha > 1.0 ? 1.0 : alpha);

      pt.x = w0.x + alpha * (w1.x - w0.x);
      pt.y = w0.y + alpha * (w1.y - w0.y);
      pt.theta = w0.theta + alpha *
        std::atan2(std::sin(w1.theta - w0.theta),
                   std::cos(w1.theta - w0.theta));
      pt.s = s_k;
    }

    // 曲率
    pt.curvature = estimateCurvature(
      findIndexAtArcLength(s_k), 0.3);

    // 参考速度（曲率衰减）
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
double DiscreteGenerator::curvature(double s) const
{
  const size_t idx = findIndexAtArcLength(s);
  return estimateCurvature(idx, 0.3);
}

// ==========================================================================
size_t DiscreteGenerator::findIndexAtArcLength(double s) const
{
  if (s <= 0.0) { return 0; }
  if (s >= total_arc_length_) { return waypoints_.size() > 1 ? waypoints_.size() - 2 : 0; }

  // 二分查找
  auto it = std::lower_bound(
    waypoints_.begin(), waypoints_.end(), s,
    [](const Waypoint & w, double val) { return w.s < val; });

  if (it == waypoints_.begin()) { return 0; }
  return static_cast<size_t>(it - waypoints_.begin()) - 1;
}

// ==========================================================================
double DiscreteGenerator::estimateCurvature(size_t idx,
                                             double forward_dist) const
{
  if (waypoints_.size() < 3) { return 0.0; }

  const size_t n = waypoints_.size();
  // 向前/向后采样点
  const double s0 = waypoints_[idx].s;
  const double s_back = std::max(0.0, s0 - forward_dist);
  const double s_fwd = std::min(total_arc_length_, s0 + forward_dist);

  const size_t i0 = findIndexAtArcLength(s_back);
  const size_t i2 = findIndexAtArcLength(s_fwd);

  const auto & p0 = waypoints_[i0];
  const auto & p1 = waypoints_[idx];
  const auto & p2 = waypoints_[i2];

  // Menger curvature: κ = 4Δ / (ab·bc·ca)
  const double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
  const double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
  const double c = std::hypot(p2.x - p0.x, p2.y - p0.y);

  if (a < 1e-9 || b < 1e-9 || c < 1e-9) { return 0.0; }

  const double s_tri = 0.5 * (a + b + c);
  const double area2 = std::abs(
    (p0.x - p1.x) * (p2.y - p1.y) - (p0.y - p1.y) * (p2.x - p1.x));

  const double denom = a * b * c;
  if (denom < 1e-9) { return 0.0; }

  return 4.0 * area2 / denom;
}

}  // namespace mpc_controller
