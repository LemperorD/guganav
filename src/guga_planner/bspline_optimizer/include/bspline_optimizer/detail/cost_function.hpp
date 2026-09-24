#pragma once

#include "bspline_optimizer/detail/cost_cache.hpp"
#include <vector>

namespace bspline_optimizer::detail {
  struct EndPoints {
    double first_x, first_y;
    double last_x, last_y;
  };

  struct EsdfData {
    const float* esdf_dist;
    double esdf_w;
    double esdf_h;
    double esdf_res;
    double esdf_ox;
    double esdf_oy;
    double esdf_max;
    double esdf_safe_dist;
  };

  struct CostNormal {
    double js0;
    double jd0;
    double je0;
  };

  struct Weight {
    double w_smooth;
    double w_dist;
    double w_esdf;
  };

  void evalTerms(const std::vector<double>& params, const CostCache& cc,
                 EndPoints& endpoints, EsdfData& esdf_data, double& js,
                 double& jd, double& je);

  double evalCost(const std::vector<double>& params, const CostCache& cc,
                  EndPoints& endpoints, Weight& weight, EsdfData& esdfdata,
                  CostNormal& costnormal);

  void computeGradient(const std::vector<double>& params, const CostCache& cc,
                       EndPoints& endpoints, Weight& weight, EsdfData& esdfdata,
                       CostNormal& costnormal, std::vector<double>& grad,
                       const float* esdf_gx, const float* esdf_gy);

}  // namespace bspline_optimizer::detail
