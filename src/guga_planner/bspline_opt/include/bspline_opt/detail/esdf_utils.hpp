#pragma once


namespace bspline_opt
{
namespace detail
{

float esdfDistanceAt(
  const float * esdf_dist, int esdf_w, int esdf_h, double resolution,
  double origin_x, double origin_y, double wx, double wy, double max_dist);

void esdfGradientAt(
  const float * esdf_gx, const float * esdf_gy, int esdf_w, int esdf_h,
  double resolution, double origin_x, double origin_y, double wx, double wy,
  double & gx, double & gy);

}  // namespace detail
}  // namespace bspline_opt
