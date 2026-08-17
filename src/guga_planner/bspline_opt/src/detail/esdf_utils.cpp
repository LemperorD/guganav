#include "bspline_opt/detail/esdf_utils.hpp"
#include <cstddef>

namespace bspline_opt
{
namespace detail
{

float esdfDistanceAt(
  const float * esdf_dist, int esdf_w, int esdf_h,
  double resolution, double origin_x, double origin_y,
  double wx, double wy, double max_dist)
{
  if (!esdf_dist) {return static_cast<float>(max_dist);}

  double fx = (wx - origin_x) / resolution;
  double fy = (wy - origin_y) / resolution;

  int ix = static_cast<int>(fx);
  int iy = static_cast<int>(fy);

  if (ix < 0 || ix + 1 >= esdf_w || iy < 0 || iy + 1 >= esdf_h) {
    return static_cast<float>(max_dist);
  }

  double dx = fx - static_cast<double>(ix);
  double dy = fy - static_cast<double>(iy);

  size_t idx00 = static_cast<size_t>(iy) * static_cast<size_t>(esdf_w) +
    static_cast<size_t>(ix);
  size_t idx10 = idx00 + 1;
  size_t idx01 = idx00 + static_cast<size_t>(esdf_w);
  size_t idx11 = idx01 + 1;

  float d00 = esdf_dist[idx00];
  float d10 = esdf_dist[idx10];
  float d01 = esdf_dist[idx01];
  float d11 = esdf_dist[idx11];

  float d0 = static_cast<float>(static_cast<double>(d00) * (1.0 - dx) +
    static_cast<double>(d10) * dx);
  float d1 = static_cast<float>(static_cast<double>(d01) * (1.0 - dx) +
    static_cast<double>(d11) * dx);

  return d0 * static_cast<float>(1.0 - dy) + d1 * static_cast<float>(dy);
}


void esdfGradientAt(
  const float * esdf_gx, const float * esdf_gy,
  int esdf_w, int esdf_h, double resolution, double origin_x, double origin_y,
  double wx, double wy, double & gx, double & gy)
{
  gx = 0.0; gy = 0.0;
  if (!esdf_gx || !esdf_gy) {return;}

  double fx = (wx - origin_x) / resolution;
  double fy = (wy - origin_y) / resolution;

  int ix = static_cast<int>(fx);
  int iy = static_cast<int>(fy);

  if (ix < 0 || ix + 1 >= esdf_w || iy < 0 || iy + 1 >= esdf_h) {return;}

  double dx = fx - static_cast<double>(ix);
  double dy = fy - static_cast<double>(iy);

  size_t idx00 = static_cast<size_t>(iy) * static_cast<size_t>(esdf_w) +
    static_cast<size_t>(ix);
  size_t idx10 = idx00 + 1;
  size_t idx01 = idx00 + static_cast<size_t>(esdf_w);
  size_t idx11 = idx01 + 1;

  float g00x = esdf_gx[idx00]; float g10x = esdf_gx[idx10];
  float g01x = esdf_gx[idx01]; float g11x = esdf_gx[idx11];
  float g0x = static_cast<float>(static_cast<double>(g00x) * (1.0 - dx) +
    static_cast<double>(g10x) * dx);
  float g1x = static_cast<float>(static_cast<double>(g01x) * (1.0 - dx) +
    static_cast<double>(g11x) * dx);
  gx = static_cast<double>(g0x * static_cast<float>(1.0 - dy) +
    g1x * static_cast<float>(dy));

  float g00y = esdf_gy[idx00]; float g10y = esdf_gy[idx10];
  float g01y = esdf_gy[idx01]; float g11y = esdf_gy[idx11];
  float g0y = static_cast<float>(static_cast<double>(g00y) * (1.0 - dx) +
    static_cast<double>(g10y) * dx);
  float g1y = static_cast<float>(static_cast<double>(g01y) * (1.0 - dx) +
    static_cast<double>(g11y) * dx);
  gy = static_cast<double>(g0y * static_cast<float>(1.0 - dy) +
    g1y * static_cast<float>(dy));
}

}  // namespace detail
}  // namespace bspline_opt
