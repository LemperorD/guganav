#include "jps_planner/jps_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace jps_planner;

static void buildGrid(
    std::vector<unsigned char>& grid, int W, int H,
    int robot_r_cells, int infl_r_cells) {
  grid.assign(static_cast<size_t>(W * H), 0);
  const double res = 0.05;
  const double robot_r = robot_r_cells * res;
  const double infl_r = infl_r_cells * res;
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const double dx = std::max(0.0, std::max(48.0 * res - (x + 0.5) * res,
                                               (x + 0.5) * res - 53.0 * res));
      const double dy = std::max(0.0, std::max(30.0 * res - (y + 0.5) * res,
                                               (y + 0.5) * res - 71.0 * res));
      const double d = std::hypot(dx, dy);
      unsigned char c = 0;
      if (d <= robot_r) {
        c = 253;
      } else if (d <= infl_r) {
        const double t = (d - robot_r) / (infl_r - robot_r);
        c = static_cast<unsigned char>(
          std::max(1, std::min(252, static_cast<int>(252.0 * (1.0 - t)))));
      }
      grid[static_cast<size_t>(y * W + x)] = c;
    }
  }
}

static void run(
    const char* name, std::vector<unsigned char>& grid, int W, int H,
    double w_traversal) {
  JPSConfig cfg;
  cfg.w_traversal_cost = w_traversal;
  JPSState state;
  state.costmap_data = grid.data();
  state.size_x = W;
  state.size_y = H;

  std::vector<std::pair<double, double>> path;
  const bool ok = JPSAlgorithm::generatePath(cfg, state, 10, 50, 90, 50, path);
  if (!ok) { std::printf("[%s] no path\n", name); return; }

  double d_min = 1e9;
  for (const auto& [px, py] : path) {
    const double dx = std::max(0.0, std::max(48.0 - px, px - 53.0));
    const double dy = std::max(0.0, std::max(30.0 - py, py - 71.0));
    d_min = std::min(d_min, std::hypot(dx, dy));
  }
  std::printf("[%s] n=%zu d_min=%.2f\n", name, path.size(), d_min);
}

int main() {
  constexpr int W = 100;
  constexpr int H = 100;
  std::vector<unsigned char> grid;
  buildGrid(grid, W, H, 8, 14);
  run("baseline wt10", grid, W, H, 10.0);
  run("baseline wt1000", grid, W, H, 1000.0);
  return 0;
}
