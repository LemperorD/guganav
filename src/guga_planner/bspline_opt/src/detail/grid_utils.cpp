#include "bspline_opt/detail/grid_utils.hpp"
#include <cmath>

namespace bspline_opt
{
namespace detail
{

unsigned char cellCost(
  const unsigned char * cmap, int w, int h, int cx, int cy)
{
  if (!cmap || cx < 0 || cx >= w || cy < 0 || cy >= h) {return 255;}
  return cmap[static_cast<size_t>(cy * w + cx)];
}


bool inObstacle(
  const unsigned char * cmap, int w, int h, double px, double py)
{
  int cx = static_cast<int>(px);
  int cy = static_cast<int>(py);
  return cellCost(cmap, w, h, cx, cy) >= 253;
}


bool projectPointToFree(
  const unsigned char * cmap, int w, int h, double & px, double & py)
{
  int ix = static_cast<int>(px);
  int iy = static_cast<int>(py);
  if (!inObstacle(cmap, w, h, px, py)) {return true;}

  for (int r = 1; r <= 8; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::abs(dx) < r && std::abs(dy) < r) {continue;}
        int tx = ix + dx;
        int ty = iy + dy;
        if (cellCost(cmap, w, h, tx, ty) < 253) {
          px = static_cast<double>(tx) + 0.5;
          py = static_cast<double>(ty) + 0.5;
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace detail
}  // namespace bspline_opt
