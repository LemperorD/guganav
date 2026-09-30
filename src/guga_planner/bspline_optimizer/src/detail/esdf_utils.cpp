#include "bspline_optimizer/detail/cost_function.hpp"
#include "bspline_optimizer/detail/esdf_utils.hpp"
#include <cstddef>

namespace bspline_optimizer::detail {

  float esdfDistanceAt(EsdfData& esdfdata, double wx, double wy) {
    if (!esdfdata.esdf_dist) {
      return static_cast<float>(esdfdata.esdf_max);
    }

    double fx = (wx - esdfdata.esdf_ox) / esdfdata.esdf_res;
    double fy = (wy - esdfdata.esdf_oy) / esdfdata.esdf_res;

    int ix = static_cast<int>(fx);
    int iy = static_cast<int>(fy);

    if (ix < 0 || ix + 1 >= esdfdata.esdf_w || iy < 0
        || iy + 1 >= esdfdata.esdf_h) {
      return static_cast<float>(esdfdata.esdf_max);
    }

    double dx = fx - static_cast<double>(ix);
    double dy = fy - static_cast<double>(iy);

    size_t idx00 = static_cast<size_t>(iy)
                       * static_cast<size_t>(esdfdata.esdf_w)
                   + static_cast<size_t>(ix);
    size_t idx10 = idx00 + 1;
    size_t idx01 = idx00 + static_cast<size_t>(esdfdata.esdf_w);
    size_t idx11 = idx01 + 1;

    float d00 = esdfdata.esdf_dist[idx00];
    float d10 = esdfdata.esdf_dist[idx10];
    float d01 = esdfdata.esdf_dist[idx01];
    float d11 = esdfdata.esdf_dist[idx11];

    auto d0 = static_cast<float>(static_cast<double>(d00) * (1.0 - dx)
                                 + static_cast<double>(d10) * dx);
    auto d1 = static_cast<float>(static_cast<double>(d01) * (1.0 - dx)
                                 + static_cast<double>(d11) * dx);

    return d0 * static_cast<float>(1.0 - dy) + d1 * static_cast<float>(dy);
  }

  void esdfGradientAt(const float* esdf_gx, const float* esdf_gy,
                      EsdfData& esdfdata, double wx, double wy, double& gx,
                      double& gy) {
    gx = 0.0;
    gy = 0.0;
    if (!esdf_gx || !esdf_gy) {
      return;
    }

    double fx = (wx - esdfdata.esdf_ox) / esdfdata.esdf_res;
    double fy = (wy - esdfdata.esdf_oy) / esdfdata.esdf_res;

    int ix = static_cast<int>(fx);
    int iy = static_cast<int>(fy);

    if (ix < 0 || ix + 1 >= esdfdata.esdf_w || iy < 0
        || iy + 1 >= esdfdata.esdf_h) {
      return;
    }

    double dx = fx - static_cast<double>(ix);
    double dy = fy - static_cast<double>(iy);

    size_t idx00 = static_cast<size_t>(iy)
                       * static_cast<size_t>(esdfdata.esdf_w)
                   + static_cast<size_t>(ix);
    size_t idx10 = idx00 + 1;
    size_t idx01 = idx00 + static_cast<size_t>(esdfdata.esdf_w);
    size_t idx11 = idx01 + 1;

    float gx00 = esdf_gx[idx00];
    float gx10 = esdf_gx[idx10];
    float gx01 = esdf_gx[idx01];
    float gx11 = esdf_gx[idx11];
    auto gx0 = static_cast<float>(static_cast<double>(gx00) * (1.0 - dx)
                                  + static_cast<double>(gx10) * dx);
    auto g1x = static_cast<float>(static_cast<double>(gx01) * (1.0 - dx)
                                  + static_cast<double>(gx11) * dx);
    gx = static_cast<double>(gx0 * static_cast<float>(1.0 - dy)
                             + g1x * static_cast<float>(dy));

    float gy00 = esdf_gy[idx00];
    float gy10 = esdf_gy[idx10];
    float gy01 = esdf_gy[idx01];
    float gy11 = esdf_gy[idx11];
    auto gy0 = static_cast<float>(static_cast<double>(gy00) * (1.0 - dx)
                                  + static_cast<double>(gy10) * dx);
    auto g1y = static_cast<float>(static_cast<double>(gy01) * (1.0 - dx)
                                  + static_cast<double>(gy11) * dx);
    gy = static_cast<double>(gy0 * static_cast<float>(1.0 - dy)
                             + g1y * static_cast<float>(dy));
  }

}  // namespace bspline_optimizer::detail
