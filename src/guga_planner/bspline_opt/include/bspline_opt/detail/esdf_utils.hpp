#pragma once
#include"bspline_opt/detail/cost_function.hpp"

namespace bspline_opt::detail
{

float esdfDistanceAt(
  EsdfData & esdf, 
  double wx, double wy);

void esdfGradientAt(
  const float * esdf_gx, const float * esdf_gy, 
  EsdfData & esdfdata,
  double wx, double wy,
  double & gx, double & gy);

}  // namespace detail

