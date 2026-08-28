#include "bspline_opt/detail/cost_function.hpp"
#include "bspline_opt/detail/esdf_utils.hpp"

namespace bspline_opt
{
namespace detail
{

void evalTerms(
  const std::vector<double> & params, const CostCache & cc,
  double first_x, double first_y, double last_x, double last_y,
  const float * esdf_dist, int esdf_w, int esdf_h,
  double esdf_res, double esdf_ox, double esdf_oy, double esdf_max,
  double esdf_safe_dist,
  double & js, double & jd, double & je)
{
  const int M = cc.M;
  Eigen::MatrixXd ctrl;
  fillCtrl(params, first_x, first_y, last_x, last_y, M, ctrl);
  js = 0.0; jd = 0.0; je = 0.0;

  for (const auto & row : cc.d2_smooth) {
    double ddx = bandedDot(row, ctrl, 0);
    double ddy = bandedDot(row, ctrl, 1);
    js += ddx * ddx + ddy * ddy;
  }
  js /= static_cast<double>(cc.d2_smooth.size());

  for (size_t i = 0; i < cc.b_dist.size(); ++i) {
    double px = bandedDot(cc.b_dist[i], ctrl, 0);
    double py = bandedDot(cc.b_dist[i], ctrl, 1);
    double dx = px - cc.dist_q[i].x();
    double dy = py - cc.dist_q[i].y();
    jd += dx * dx + dy * dy;
  }

  if (esdf_dist) {
    for (const auto & row : cc.b_esdf) {
      double px = bandedDot(row, ctrl, 0);
      double py = bandedDot(row, ctrl, 1);
      double wx = px * esdf_res + esdf_ox;
      double wy = py * esdf_res + esdf_oy;
      float dist = esdfDistanceAt(
        esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox, esdf_oy, wx, wy, esdf_max);
      if (dist < static_cast<float>(esdf_safe_dist)) {
        double violation = esdf_safe_dist - static_cast<double>(dist);
        je += violation * violation;
      }
    }
  }
}


double evalCost(
  const std::vector<double> & params, const CostCache & cc,
  double first_x, double first_y, double last_x, double last_y,
  double w_smooth, double w_dist,
  const float * esdf_dist, int esdf_w, int esdf_h,
  double esdf_res, double esdf_ox, double esdf_oy, double esdf_max,
  double esdf_safe_dist, double w_esdf,
  double js0, double jd0, double je0)
{
  double js, jd, je;
  evalTerms(
    params, cc, first_x, first_y, last_x, last_y,
    esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox, esdf_oy, esdf_max,
    esdf_safe_dist, js, jd, je);
  double cost = 0.0;
  if (w_smooth > 0.0 && js0 > 1e-12) {cost += w_smooth * (js / js0);}
  if (w_dist > 0.0 && jd0 > 1e-12) {cost += w_dist * (jd / jd0);}
  if (w_esdf > 0.0 && je0 > 1e-12) {cost += w_esdf * (je / je0);}
  return cost;
}


void computeGradient(
  const std::vector<double> & params, const CostCache & cc,
  double first_x, double first_y, double last_x, double last_y,
  double w_smooth, double w_dist,
  const float * esdf_dist, const float * esdf_gx, const float * esdf_gy,
  int esdf_w, int esdf_h, double esdf_res, double esdf_ox, double esdf_oy,
  double esdf_max, double esdf_safe_dist, double w_esdf,
  double js0, double jd0, double je0,
  std::vector<double> & grad)
{
  const int M = cc.M;
  grad.assign(static_cast<size_t>(2 * (M - 2)), 0.0);
  Eigen::MatrixXd ctrl;
  fillCtrl(params, first_x, first_y, last_x, last_y, M, ctrl);

  auto accum = [&](int j, double gx, double gy) {
      if (j >= 1 && j <= M - 2) {
        grad[static_cast<size_t>(2 * (j - 1))] += gx;
        grad[static_cast<size_t>(2 * (j - 1) + 1)] += gy;
      }
    };

  if (w_smooth > 0.0 && js0 > 1e-12) {
    const double fac = 2.0 * w_smooth / (js0 * static_cast<double>(cc.d2_smooth.size()));
    for (const auto & row : cc.d2_smooth) {
      double ddx = bandedDot(row, ctrl, 0);
      double ddy = bandedDot(row, ctrl, 1);
      for (int k = 0; k < row.count; ++k) {
        accum(row.start + k, fac * ddx * row.val[k], fac * ddy * row.val[k]);
      }
    }
  }

  if (w_dist > 0.0 && jd0 > 1e-12) {
    const double fac = 2.0 * w_dist / jd0;
    for (size_t i = 0; i < cc.b_dist.size(); ++i) {
      const auto & row = cc.b_dist[i];
      double px = bandedDot(row, ctrl, 0);
      double py = bandedDot(row, ctrl, 1);
      double ex = px - cc.dist_q[i].x();
      double ey = py - cc.dist_q[i].y();
      for (int k = 0; k < row.count; ++k) {
        accum(
          row.start + k, fac * ex * row.val[k], fac * ey * row.val[k]);
      }
    }
  }

  if (w_esdf > 0.0 && je0 > 1e-12 && esdf_dist) {
    // 梯度下降沿 -∇J 移动; ∇J_esdf = -2·w·(d_safe-d)·∇d,
    // 因此系数必须为负, 使曲线朝远离障碍物的方向移动。
    const double fac = -2.0 * w_esdf / je0;
    for (const auto & row : cc.b_esdf) {
      double px = bandedDot(row, ctrl, 0);
      double py = bandedDot(row, ctrl, 1);
      double wx = px * esdf_res + esdf_ox;
      double wy = py * esdf_res + esdf_oy;
      float dist = esdfDistanceAt(
        esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox, esdf_oy, wx, wy, esdf_max);
      if (dist < static_cast<float>(esdf_safe_dist)) {
        double gd_x{};
        double gd_y{};
        if (esdf_gx && esdf_gy) {
          esdfGradientAt(
            esdf_gx, esdf_gy, esdf_w, esdf_h, esdf_res, esdf_ox, esdf_oy,
            wx, wy, gd_x, gd_y);
        } else {
          gd_x = 0.5 * static_cast<double>(
            esdfDistanceAt(
              esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox,
              esdf_oy, wx + esdf_res, wy, esdf_max) -
            esdfDistanceAt(
              esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox,
              esdf_oy, wx - esdf_res, wy, esdf_max));
          gd_y = 0.5 * static_cast<double>(
            esdfDistanceAt(
              esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox,
              esdf_oy, wx, wy + esdf_res, esdf_max) -
            esdfDistanceAt(
              esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox,
              esdf_oy, wx, wy - esdf_res, esdf_max));
        }
        double violation = esdf_safe_dist - static_cast<double>(dist);
        for (int k = 0; k < row.count; ++k) {
          accum(
            row.start + k, fac * violation * gd_x * row.val[k],
            fac * violation * gd_y * row.val[k]);
        }
      }
    }
  }
}

}  // namespace detail
}  // namespace bspline_opt
