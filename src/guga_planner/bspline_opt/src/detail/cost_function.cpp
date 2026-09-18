#include "bspline_opt/detail/cost_function.hpp"
#include "bspline_opt/detail/esdf_utils.hpp"


namespace bspline_opt::detail
{

void evalTerms(
  const std::vector<double> & params, const CostCache & cc,
  EndPoints & endpoints,
  EsdfData & esdfdata,
   double & js, double & jd, double & je)
{
  const int M = cc.M;
  Eigen::MatrixXd ctrl;
  fillCtrl(params, endpoints.first_x, endpoints.first_y, endpoints.last_x, endpoints.last_y, M, ctrl);
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

  if (esdfdata.esdf_dist) {
    for (const auto & row : cc.b_esdf) {
      double px = bandedDot(row, ctrl, 0);
      double py = bandedDot(row, ctrl, 1);
      double wx = (px * esdfdata.esdf_res) + esdfdata.esdf_ox;
      double wy = (py * esdfdata.esdf_res) + esdfdata.esdf_oy;
      float dist = esdfDistanceAt(
        esdfdata, wx, wy);
      if (dist < static_cast<float>(esdfdata.esdf_safe_dist)) {
        double violation = esdfdata.esdf_safe_dist - static_cast<double>(dist);
        je += violation * violation;
      }
    }
  }
}


double evalCost(
  const std::vector<double> & params, const CostCache & cc,
  EndPoints & endpoints,
  Weight & weight,
  EsdfData & esdfdata, 
  CostNormal & costnormal)
{
  double js, jd, je;
  evalTerms(
    params,cc, 
    endpoints,esdfdata,js, jd, je);
  double cost = 0.0;
  if (weight.w_smooth > 0.0 && costnormal.js0 > 1e-12) {cost += weight.w_smooth * (js / costnormal.js0);}
  if (weight.w_dist > 0.0 && costnormal.jd0 > 1e-12) {cost += weight.w_dist * (jd / costnormal.jd0);}
  if (weight.w_esdf > 0.0 && costnormal.je0 > 1e-12) {cost += weight.w_esdf * (je / costnormal.je0);}
  return cost;
}


void computeGradient(
  const std::vector<double> & params, 
  const CostCache & cc,
  EndPoints & endpoints, 
  Weight & weight,
  EsdfData & esdfdata, 
  CostNormal & costnormal,
  std::vector<double> & grad,
  const float * esdf_gx,const float * esdf_gy)
{
  const int M = cc.M;
  grad.assign(static_cast<size_t>(2 * (M - 2)), 0.0);
  Eigen::MatrixXd ctrl;
  fillCtrl(params, endpoints.first_x, endpoints.first_y, endpoints.last_x, endpoints.last_y, M, ctrl);

  auto accum = [&](int j, double gx, double gy) {
      if (j >= 1 && j <= M - 2) {
        grad[static_cast<size_t>(2 * (j - 1))] += gx;
        grad[static_cast<size_t>(2 * (j - 1) + 1)] += gy;
      }
    };

  if (weight.w_smooth > 0.0 && costnormal.js0 > 1e-12) {
    const double fac = 2.0 * weight.w_smooth / (costnormal.js0 * static_cast<double>(cc.d2_smooth.size()));
    for (const auto & row : cc.d2_smooth) {
      double ddx = bandedDot(row, ctrl, 0);
      double ddy = bandedDot(row, ctrl, 1);
      for (int k = 0; k < row.count; ++k) {
        accum(row.start + k, fac * ddx * row.val[k], fac * ddy * row.val[k]);
      }
    }
  }

  if (weight.w_dist > 0.0 && costnormal.jd0 > 1e-12) {
    const double fac = 2.0 * weight.w_dist / costnormal.jd0;
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

  if (weight.w_esdf > 0.0 && costnormal.je0 > 1e-12 && esdfdata.esdf_dist) {
    // 梯度下降沿 -∇J 移动; ∇J_esdf = -2·w·(d_safe-d)·∇d,
    // 因此系数必须为负, 使曲线朝远离障碍物的方向移动。
    const double fac = -2.0 * weight.w_esdf / costnormal.je0;
    for (const auto & row : cc.b_esdf) {
      double px = bandedDot(row, ctrl, 0);
      double py = bandedDot(row, ctrl, 1);
      double wx = px * esdfdata.esdf_res + esdfdata.esdf_ox;
      double wy = py * esdfdata.esdf_res + esdfdata.esdf_oy;
      float dist = esdfDistanceAt(
        esdfdata, wx, wy);
      if (dist < static_cast<float>(esdfdata.esdf_safe_dist)) {
        double gd_x{};
        double gd_y{};
        if (esdf_gx && esdf_gy) {
          esdfGradientAt(
            esdf_gx, esdf_gy, 
            esdfdata,
            wx, wy, 
            gd_x, gd_y);
        } else {
          gd_x = 0.5 * static_cast<double>(
            esdfDistanceAt(
              esdfdata, wx + esdfdata.esdf_res, wy) -
            esdfDistanceAt(
              esdfdata, wx - esdfdata.esdf_res, wy));
          gd_y = 0.5 * static_cast<double>(
            esdfDistanceAt(
              esdfdata, wx, wy + esdfdata.esdf_res) -
            esdfDistanceAt(
              esdfdata, wx, wy - esdfdata.esdf_res));
        }
        double violation = esdfdata.esdf_safe_dist - static_cast<double>(dist);
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

