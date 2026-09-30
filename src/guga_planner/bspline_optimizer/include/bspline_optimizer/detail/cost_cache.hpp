#pragma once

#include <Eigen/Core>
#include <vector>

namespace bspline_optimizer::detail {

  struct BandRow {
    int start{};
    int count{};
    double value[8]{};
  };

  struct CostCache {
    int M{};
    int smooth_sample{50};
    int esdf_sample{200};
    std::vector<BandRow> d2_smooth{};
    std::vector<BandRow> basis_dist{};
    std::vector<Eigen::Vector2d> dist_q{};
    std::vector<BandRow> basis_esdf{};
  };
}  // namespace bspline_optimizer::detail
