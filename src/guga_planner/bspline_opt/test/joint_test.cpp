/**
 * @brief Joint test: JPS front-end → B-spline back-end.
 *
 * Reads JPS path data files, runs BSplineOptimizer on each, computes
 * before/after metrics, and writes visualization data files.
 */

#include "bspline_opt/bspline_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <sys/stat.h>

namespace
{

std::vector<std::pair<double, double>> readPath(const std::string & fname)
{
  std::vector<std::pair<double, double>> out{};
  std::ifstream f(fname);
  if (!f) { return out; }
  double x{}, y{};
  while (f >> x >> y) { out.emplace_back(x, y); }
  return out;
}

void writePath(
  const std::vector<std::pair<double, double>> & path,
  const std::string & fname)
{
  std::ofstream f(fname);
  for (auto [x, y] : path) { f << x << " " << y << "\n"; }
}

void writeCurvature(
  const std::vector<double> & curv, const std::string & fname)
{
  std::ofstream f(fname);
  f << "arc_param,curvature\n";
  for (size_t i = 0; i < curv.size(); ++i) {
    f << static_cast<double>(i) / static_cast<double>(curv.size() - 1)
      << "," << curv[i] << "\n";
  }
}

double pathMaxCurvature(
  const std::vector<std::pair<double, double>> & path)
{
  if (path.size() < 3) { return 0.0; }
  double max_k{};
  for (size_t i = 1; i < path.size() - 1; ++i) {
    double dx1 = path[i].first - path[i - 1].first;
    double dy1 = path[i].second - path[i - 1].second;
    double dx2 = path[i + 1].first - path[i].first;
    double dy2 = path[i + 1].second - path[i].second;
    double l1 = std::hypot(dx1, dy1);
    double l2 = std::hypot(dx2, dy2);
    if (l1 < 1e-9 || l2 < 1e-9) { continue; }
    double dot = (dx1 * dx2 + dy1 * dy2) / (l1 * l2);
    double angle = std::acos(std::clamp(dot, -1.0, 1.0));
    double k = angle / (0.5 * (l1 + l2));
    max_k = std::max(max_k, k);
  }
  return max_k;
}

double pathLength(const std::vector<std::pair<double, double>> & path)
{
  double len{};
  for (size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(
      path[i].first - path[i - 1].first,
      path[i].second - path[i - 1].second);
  }
  return len;
}

}  // namespace

int main(int argc, char ** argv)
{
  using namespace bspline_opt;

  std::string data_dir = "../../jps_planner/test/jps_viz_data";
  std::string out_dir = "bspline_viz_data";
  if (argc > 1) { data_dir = argv[1]; }
  if (argc > 2) { out_dir = argv[2]; }
  mkdir(out_dir.c_str(), 0755);

  const char * scenarios[] = {
    "s1_empty_diag", "s2_wall_gap", "s3_maze",
    "s4_obstaclefield", "s5_large_empty", "s6_costband"};

  BSplineConfig cfg{};
  cfg.smoothness_weight = 1.0;
  cfg.distance_weight = 500.0;
  cfg.obstacle_weight = 50000.0;
  cfg.max_control_points = 15;  // Balanced: fewer pts = less corner-cutting
  cfg.ceres_max_iterations = 200;

  std::ofstream bench(out_dir + "/benchmark.csv");
  bench << "scenario,orig_points,smooth_points,orig_max_curv,"
        << "smooth_max_curv,orig_len,smooth_len,"
        << "curv_energy_init,curv_energy_final,converged\n";

  std::cout << "=== JPS → B-spline Joint Test ===\n";
  std::cout << "Input: " << data_dir << "\n";
  std::cout << "Output: " << out_dir << "\n\n";

  for (const auto * sc : scenarios) {
    std::string in_path = data_dir + "/" + sc + "_path.dat";
    auto path = readPath(in_path);
    if (path.empty()) {
      std::cout << "  " << sc << ": no data, skipping\n";
      continue;
    }

    std::cout << "  " << sc;
    std::cout << " (" << path.size() << " pts)... ";

    BSplineOptimizer opt(cfg);

    // Load costmap grid into state for obstacle avoidance
    std::string grid_in = data_dir + "/" + sc + "_grid.dat";
    std::vector<unsigned char> grid_data{};
    int gw{}, gh{};
    {
      std::ifstream gf(grid_in);
      if (gf) {
        double x{}, y{};
        int c{};
        // Two-pass: first determine size, then load
        std::vector<std::tuple<int,int,int>> cells{};
        while (gf >> x >> y >> c) {
          int cx = static_cast<int>(x);
          int cy = static_cast<int>(y);
          gw = std::max(gw, cx + 1);
          gh = std::max(gh, cy + 1);
          cells.emplace_back(cx, cy, c);
        }
        grid_data.assign(static_cast<size_t>(gw * gh), 0);
        for (auto [cx, cy, c] : cells) {
          grid_data[static_cast<size_t>(cy * gw + cx)] = static_cast<unsigned char>(c);
        }
      }
    }

    if (!opt.fit(path)) {
      std::cout << "FIT FAILED\n";
      continue;
    }

    // Inject costmap into state before optimize()
    if (!grid_data.empty()) {
      opt.state().costmap_data = grid_data.data();
      opt.state().costmap_w = gw;
      opt.state().costmap_h = gh;
    }

    auto result = opt.optimize(200);

    writePath(path, out_dir + "/" + sc + "_orig_path.dat");
    writePath(result.smoothed_path, out_dir + "/" + sc + "_smooth_path.dat");
    writePath(result.control_points_xy, out_dir + "/" + sc + "_ctrl_pts.dat");
    writeCurvature(result.curvature_profile, out_dir + "/" + sc + "_curvature.csv");

    std::ofstream sg(out_dir + "/" + sc + "_startgoal.dat");
    sg << path.front().first << " " << path.front().second << " start\n";
    sg << path.back().first << " " << path.back().second << " goal\n";

    // Copy grid file (already loaded above)
    {
      std::ifstream gf2(grid_in, std::ios::binary);
      if (gf2) {
        std::ofstream go(out_dir + "/" + sc + "_grid.dat", std::ios::binary);
        go << gf2.rdbuf();
      }
    }

    double orig_max_k = pathMaxCurvature(path);
    double smooth_max_k{};
    for (double k : result.curvature_profile) { smooth_max_k = std::max(smooth_max_k, k); }
    double orig_len = pathLength(path);
    double smooth_len = pathLength(result.smoothed_path);

    bench << sc << "," << path.size() << ","
          << result.smoothed_path.size() << ","
          << orig_max_k << "," << smooth_max_k << ","
          << orig_len << "," << smooth_len << ","
          << result.cost_initial << "," << result.cost_final << ","
          << (result.converged ? 1 : 0) << "\n";

    std::cout << "OK  curv: " << orig_max_k << "→" << smooth_max_k
              << "  len: " << orig_len << "→" << smooth_len << "\n";
  }

  bench.close();
  std::cout << "\nDone. Data in " << out_dir << "/\n";
  return 0;
}
