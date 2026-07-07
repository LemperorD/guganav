/**
 * @brief JPS visualization data exporter (standalone, no ROS).
 *
 * Runs JPS on representative scenarios and writes per-scenario data files
 * (grid, path, jump points, expanded nodes) + a benchmark CSV.
 */

#include "jps_planner/jps_algorithm.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <utility>
#include <vector>

namespace jps_planner
{

// ── Grid builders ────────────────────────────────────────

std::vector<unsigned char> makeEmptyGrid(int w, int h)
{
  return std::vector<unsigned char>(static_cast<size_t>(w * h), 0);
}

std::vector<unsigned char> makeGrid(
  int w, int h, const std::vector<std::string> & pattern)
{
  std::vector<unsigned char> grid(static_cast<size_t>(w * h), 0);
  for (int row = 0; row < h; ++row) {
    int y = h - 1 - row;
    for (int x = 0; x < w; ++x) {
      char ch = pattern[static_cast<size_t>(row)][static_cast<size_t>(x)];
      unsigned char val{0};
      if (ch == '#')       val = 254;
      else if (ch == '.')  val = 0;
      else if (ch == 'X')  val = 253;
      else if (ch == '?')  val = 255;
      else if (ch >= '0' && ch <= '9')
        val = static_cast<unsigned char>((ch - '0') * 10);
      grid[static_cast<size_t>(y * w + x)] = val;
    }
  }
  return grid;
}

void initState(JPSState & s, const std::vector<unsigned char> & grid,
               int w, int h)
{
  s = {};
  s.costmap_data = grid.data();
  s.size_x = w;
  s.size_y = h;
  s.debug_.enabled = true;
}

// ── File writers ─────────────────────────────────────────

void writeGrid(const std::vector<unsigned char> & grid, int w, int h,
               const std::string & fname)
{
  std::ofstream f(fname);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      f << x << " " << y << " " << static_cast<int>(grid[static_cast<size_t>(y * w + x)]) << "\n";
}

void writePath(const std::vector<std::pair<double, double>> & path,
               const std::string & fname)
{
  std::ofstream f(fname);
  for (auto [px, py] : path) f << px << " " << py << "\n";
}

void writeIntList(const std::vector<int> & xs, const std::vector<int> & ys,
                  const std::string & fname)
{
  std::ofstream f(fname);
  for (size_t i = 0; i < xs.size(); ++i) f << xs[i] << " " << ys[i] << "\n";
}

// ── Scenario runner ──────────────────────────────────────

struct ScenarioResult
{
  std::string name;
  bool found{};
  size_t nodes_created{};
  size_t expanded_nodes{};
  size_t path_len{};
  double time_ms{};
};

ScenarioResult runScenario(
  const std::string & name, const JPSConfig & cfg,
  const std::vector<unsigned char> & grid, int w, int h,
  int sx, int sy, int gx, int gy, const std::string & out_dir)
{
  JPSState st{};
  initState(st, grid, w, h);

  auto t0 = std::chrono::high_resolution_clock::now();
  std::vector<std::pair<double, double>> path{};
  bool found = JPSAlgorithm::generatePath(cfg, st, sx, sy, gx, gy, path);
  auto t1 = std::chrono::high_resolution_clock::now();

  ScenarioResult r{};
  r.name = name;
  r.found = found;
  r.nodes_created = st.nodes_.size();
  r.path_len = path.size();
  r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  for (const auto & np : st.nodes_) {
    if (np && np->closed) { r.expanded_nodes++; }
  }

  std::string pfx = out_dir + "/" + name;
  writeGrid(grid, w, h, pfx + "_grid.dat");
  if (found) writePath(path, pfx + "_path.dat");
  writeIntList(st.debug_.expanded_x, st.debug_.expanded_y,
               pfx + "_expanded.dat");
  writeIntList(st.debug_.jumppoint_x, st.debug_.jumppoint_y,
               pfx + "_jumppoints.dat");

  std::ofstream f(pfx + "_startgoal.dat");
  f << sx + 0.5 << " " << sy + 0.5 << " start\n";
  f << gx + 0.5 << " " << gy + 0.5 << " goal\n";

  std::cout << "  " << name << ": " << (found ? "OK" : "FAIL")
            << "  t=" << r.time_ms << "ms"
            << "  expanded=" << r.expanded_nodes
            << "  path=" << r.path_len << "\n";
  return r;
}

}  // namespace jps_planner

// ──────────────────────────────────────────────────────────
// Main
// ──────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  using namespace jps_planner;

  std::string out_dir = "jps_viz_data";
  if (argc > 1) out_dir = argv[1];

  // Create output directory
  mkdir(out_dir.c_str(), 0755);

  JPSConfig cfg{};

  std::ofstream bench(out_dir + "/benchmark.csv");
  bench << "scenario,w,h,time_ms,nodes_created,expanded,path_len,found\n";

  auto saveBench = [&](const std::string & n, int w, int h,
                       const ScenarioResult & r) {
    bench << n << "," << w << "," << h << ","
          << r.time_ms << "," << r.nodes_created << ","
          << r.expanded_nodes << "," << r.path_len << ","
          << (r.found ? 1 : 0) << "\n";
  };

  std::cout << "=== JPS Visualization Data Export ===\n";
  std::cout << "Output: " << out_dir << "\n\n";

  // ── S1: Empty grid diagonal ──
  {
    constexpr int W = 80, H = 80;
    auto g = makeEmptyGrid(W, H);
    auto r = runScenario("s1_empty_diag", cfg, g, W, H, 5, 5, 75, 75, out_dir);
    saveBench("s1_empty_diag", W, H, r);
  }

  // ── S2: Vertical wall with gap ──
  {
    auto g = makeGrid(40, 25, {
      "........................................",
      "........................................",
      "........................................",
      "........................................",
      "........................................",
      "........................................",
      "........................................",
      "....................#...................",
      "....................#...................",
      "....................#...................",
      "....................#...................",
      "....................#...................",
      "....................#...................",
      "....................#...................",
      "....................#...................",
      "....................#...................",
      "....................#...................",
      "........................................",
      "........................................",
      "........................................",
      "........................................",
      "........................................",
      "........................................",
      "........................................",
      "........................................",
    });
    auto r = runScenario("s2_wall_gap", cfg, g, 40, 25, 5, 12, 35, 12, out_dir);
    saveBench("s2_wall_gap", 40, 25, r);
  }

  // ── S3: Maze corridor ──
  {
    auto g = makeGrid(40, 32, {
      "########################################",
      "#......................................#",
      "#.####################################.#",
      "#.#..................................#.#",
      "#.#.################################.#.#",
      "#.#.#............................#.#.#.#",
      "#.#.#.##########################.#.#.#.#",
      "#.#.#.#........................#.#.#.#.#",
      "#.#.#.#.######################.#.#.#.#.#",
      "#.#.#.#.#..................#.#.#.#.#.#.#",
      "#.#.#.#.#.################.#.#.#.#.#.#.#",
      "#.#.#.#.#.#............#.#.#.#.#.#.#.#.#",
      "#.#.#.#.#.#.##########.#.#.#.#.#.#.#.#.#",
      "#.#.#.#.#.#.#......#.#.#.#.#.#.#.#.#.#.#",
      "#.#.#.#.#.#.#.####.#.#.#.#.#.#.#.#.#.#.#",
      "#.#.#.#.#.#.#.#..#.#.#.#.#.#.#.#.#.#.#.#",
      "#.#.#.#.#.#.#.#..#.#.#.#.#.#.#.#.#.#.#.#",
      "#.#.#.#.#.#.#.####.#.#.#.#.#.#.#.#.#.#.#",
      "#.#.#.#.#.#.#......#.#.#.#.#.#.#.#.#.#.#",
      "#.#.#.#.#.#.##########.#.#.#.#.#.#.#.#.#",
      "#.#.#.#.#.#............#.#.#.#.#.#.#.#.#",
      "#.#.#.#.#.################.#.#.#.#.#.#.#",
      "#.#.#.#.#..................#.#.#.#.#.#.#",
      "#.#.#.#.######################.#.#.#.#.#",
      "#.#.#.#........................#.#.#.#.#",
      "#.#.#.##########################.#.#.#.#",
      "#.#.#............................#.#.#.#",
      "#.#.################################.#.#",
      "#.#..................................#.#",
      "#.####################################.#",
      "#......................................#",
      "########################################",
    });
    auto r = runScenario("s3_maze", cfg, g, 40, 32, 1, 1, 38, 30, out_dir);
    saveBench("s3_maze", 40, 32, r);
  }

  // ── S4: Obstacle field ──
  {
    constexpr int W = 50, H = 50;
    std::vector<unsigned char> g(static_cast<size_t>(W * H), 0);
    for (int y = 5; y < 45; y += 10)
      for (int x = 5; x < 45; x += 11)
        for (int dy = 0; dy < 4; ++dy)
          for (int dx = 0; dx < 4; ++dx) {
            int py = y + dy + ((x / 11) % 2 == 0 ? 3 : -3);
            int px = x + dx;
            if (px >= 0 && px < W && py >= 0 && py < H)
              g[static_cast<size_t>(py * W + px)] = 254;
          }
    auto r = runScenario("s4_obstaclefield", cfg, g, W, H, 2, 2, 47, 47, out_dir);
    saveBench("s4_obstaclefield", W, H, r);
  }

  // ── S5: Large empty grid ──
  {
    constexpr int W = 200, H = 200;
    auto g = makeEmptyGrid(W, H);
    auto r = runScenario("s5_large_empty", cfg, g, W, H, 10, 10, 190, 190, out_dir);
    saveBench("s5_large_empty", W, H, r);
  }

  // ── S6: High-cost diagonal band ──
  {
    constexpr int W = 60, H = 60;
    std::vector<unsigned char> g(static_cast<size_t>(W * H), 0);
    for (int x = 0; x < W; ++x)
      for (int y = 0; y < H; ++y)
        if (std::abs(x - y) < 6)
          g[static_cast<size_t>(y * W + x)] = static_cast<unsigned char>(200 + std::abs(x - y) * 10);
    auto r = runScenario("s6_costband", cfg, g, W, H, 5, 5, 55, 55, out_dir);
    saveBench("s6_costband", W, H, r);
  }

  // ── Scaling benchmark ──
  {
    for (int sz : {50, 100, 150, 200, 250}) {
      auto g = makeEmptyGrid(sz, sz);
      JPSState st{};
      initState(st, g, sz, sz);
      auto t0 = std::chrono::high_resolution_clock::now();
      std::vector<std::pair<double, double>> p{};
      bool found = JPSAlgorithm::generatePath(cfg, st, 1, 1, sz - 2, sz - 2, p);
      auto t1 = std::chrono::high_resolution_clock::now();
      double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      size_t closed = 0;
      for (const auto & np : st.nodes_) {
        if (np && np->closed) closed++;
      }
      bench << "scale_" << sz << "," << sz << "," << sz << ","
            << ms << "," << st.nodes_.size() << "," << closed << ","
            << p.size() << "," << (found ? 1 : 0) << "\n";
      std::cout << "  scale " << sz << "x" << sz << ": t=" << ms << "ms"
                << " nodes=" << st.nodes_.size()
                << " expanded=" << closed << "\n";
    }
  }

  bench.close();
  std::cout << "\nDone! Data files in " << out_dir << "/\n";
  return 0;
}
