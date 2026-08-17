#pragma once


namespace bspline_opt
{
namespace detail
{

unsigned char cellCost(const unsigned char * cmap, int w, int h, int cx, int cy);

bool inObstacle(const unsigned char * cmap, int w, int h, double px, double py);

bool projectPointToFree(const unsigned char * cmap, int w, int h, double & px, double & py);

}  // namespace detail
}  // namespace bspline_opt
