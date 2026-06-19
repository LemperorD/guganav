#pragma once

#include "jps_planner/jps_algorithm.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace jps_planner
{

/** @brief Default test configuration with known-unknown disabled. */
inline JPSConfig defaultTestConfig()
{
  JPSConfig c{};
  c.allow_unknown = false;
  return c;
}

/**
 * @brief Create an empty (all free) costmap.
 * @param w Grid width in cells.
 * @param h Grid height in cells.
 */
inline std::vector<unsigned char> makeEmptyGrid(int w, int h)
{
  return std::vector<unsigned char>(static_cast<size_t>(w * h), 0);
}

/**
 * @brief Create a grid from ASCII art (bottom row = y=0).
 *
 * '#' = LETHAL_OBSTACLE (254)
 * '.' = FREE_SPACE (0)
 * 'X' = INSCRIBED_INFLATED_OBSTACLE (253)
 * '?' = NO_INFORMATION (255)
 * digit = raw cost (0-9 maps to 0-90, *10 for readability)
 *
 * @param pattern  Vector of strings, each the same width.
 *                 pattern[0] is the TOP row (highest y).
 */
inline std::vector<unsigned char> makeGrid(
  const std::vector<std::string> & pattern)
{
  int h = static_cast<int>(pattern.size());
  int w = static_cast<int>(pattern[0].size());
  std::vector<unsigned char> grid(static_cast<size_t>(w * h), 0);
  for (int row = 0; row < h; ++row) {
    int y = h - 1 - row;  // pattern[0] is top → highest y
    for (int x = 0; x < w; ++x) {
      char ch = pattern[static_cast<size_t>(row)]
                      [static_cast<size_t>(x)];
      unsigned char val{0};
      switch (ch) {
        case '#': val = 254; break;  // LETHAL_OBSTACLE
        case '.': val = 0;   break;  // FREE_SPACE
        case 'X': val = 253; break;  // INSCRIBED_INFLATED
        case '?': val = 255; break;  // NO_INFORMATION
        default:
          if (ch >= '0' && ch <= '9') {
            val = static_cast<unsigned char>((ch - '0') * 10);
          }
          break;
      }
      grid[static_cast<size_t>(y * w + x)] = val;
    }
  }
  return grid;
}

/**
 * @brief Initialise a JPSState from a raw grid buffer.
 * Clears any previous search state.
 */
inline void initState(
  JPSState & s, const std::vector<unsigned char> & grid, int w, int h)
{
  s = {};
  s.costmap_data = grid.empty() ? nullptr : grid.data();
  s.size_x = w;
  s.size_y = h;
}

}  // namespace jps_planner
