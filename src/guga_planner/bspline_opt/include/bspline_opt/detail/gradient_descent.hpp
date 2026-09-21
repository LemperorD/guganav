#pragma once

#include "bspline_opt/detail/cost_cache.hpp"
#include <vector>
#include"bspline_opt/detail/cost_function.hpp"
namespace bspline_opt::detail
{


std::vector<double> gradientDescent(
  const std::vector<double> & init_params, 
  const CostCache & cc,
  EndPoints & enpoints,
  Weight & weight, 
  const float * esdf_gx, const float * esdf_gy,
  EsdfData & esdfdata,
  CostNormal & costnormal, 
  int max_iters,
  double corridor_hw, 
  bool & converged, 
  int & iters_out);

}  // namespace detail
