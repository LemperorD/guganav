#pragma once

#include "bspline_opt/detail/cost_cache.hpp"
#include <vector>

namespace bspline_opt
{
namespace detail
{

void evalTerms(
  const std::vector<double> & params, const CostCache & cc, double first_x,
  double first_y, double last_x, double last_y, const float * esdf_dist, int esdf_w,
  int esdf_h, double esdf_res, double esdf_ox, double esdf_oy, double esdf_max,
  double esdf_safe_dist, double & js, double & jd, double & je);

double evalCost(
  const std::vector<double> & params, const CostCache & cc, double first_x,
  double first_y, double last_x, double last_y, double w_smooth, double w_dist,
  const float * esdf_dist, int esdf_w, int esdf_h, double esdf_res, double esdf_ox,
  double esdf_oy, double esdf_max, double esdf_safe_dist, double w_esdf, double js0,
  double jd0, double je0);

void computeGradient(
  const std::vector<double> & params, const CostCache & cc, double first_x,
  double first_y, double last_x, double last_y, double w_smooth, double w_dist,
  const float * esdf_dist, const float * esdf_gx, const float * esdf_gy,
  int esdf_w, int esdf_h, double esdf_res, double esdf_ox, double esdf_oy,
  double esdf_max, double esdf_safe_dist, double w_esdf, double js0, double jd0,
  double je0, std::vector<double> & grad);

}  // namespace detail
}  // namespace bspline_opt
