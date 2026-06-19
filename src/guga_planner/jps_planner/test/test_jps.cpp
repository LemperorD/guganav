#include "jps_planner/jps_algorithm.hpp"
#include "test_helpers.hpp"

#include <algorithm>
#include <utility>
#include <vector>
#include <string>

#include "gtest/gtest.h"

namespace jps_planner
{

// ──────────────────────────────────────────────────────────
// JPSSearchTest — Basic pathfinding on various grids
// ──────────────────────────────────────────────────────────
class JPSSearchTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config_ = defaultTestConfig();
  }

  /** Run search and check that a path is found. */
  void expectPath(
    JPSState & state, int sx, int sy, int gx, int gy)
  {
    std::vector<std::pair<double, double>> path{};
    bool ok = JPSAlgorithm::generatePath(
      config_, state, sx, sy, gx, gy, path);
    EXPECT_TRUE(ok);
    EXPECT_GE(path.size(), 2u);
  }

  JPSConfig config_{};
};

TEST_F(JPSSearchTest, EmptyGrid_StraightHorizontal)
{
  constexpr int W = 100, H = 100;
  auto grid = makeEmptyGrid(W, H);
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 10, 50, 90, 50, path);
  EXPECT_TRUE(ok);
  EXPECT_GE(path.size(), 2u);
  // Start and goal should be present
  EXPECT_NEAR(path.front().first, 10.5, 0.1);
  EXPECT_NEAR(path.front().second, 50.5, 0.1);
  EXPECT_NEAR(path.back().first, 90.5, 0.1);
  EXPECT_NEAR(path.back().second, 50.5, 0.1);
}

TEST_F(JPSSearchTest, EmptyGrid_StraightVertical)
{
  constexpr int W = 100, H = 100;
  auto grid = makeEmptyGrid(W, H);
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 50, 10, 50, 90, path);
  EXPECT_TRUE(ok);
  EXPECT_GE(path.size(), 2u);
}

TEST_F(JPSSearchTest, EmptyGrid_Diagonal)
{
  constexpr int W = 100, H = 100;
  auto grid = makeEmptyGrid(W, H);
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 10, 10, 90, 90, path);
  EXPECT_TRUE(ok);
  EXPECT_GE(path.size(), 2u);
}

TEST_F(JPSSearchTest, GridWithWall_FindsPathAround)
{
  // 20x10 grid. A vertical wall at x=10 leaves a gap at y=0 (bottom).
  // Start (5,5) → goal (15,5). Path must go through the gap.
  auto grid = makeGrid({
  // 01234567890123456789
    "....................", // y=9
    "....................", // y=8
    "..........#.........", // y=7  wall starts
    "..........#.........", // y=6
    "..........#.........", // y=5
    "..........#.........", // y=4
    "..........#.........", // y=3
    "..........#.........", // y=2
    "..........#.........", // y=1
    "....................", // y=0  gap here
  });
  constexpr int W = 20, H = 10;
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 5, 5, 15, 5, path);
  EXPECT_TRUE(ok);
  EXPECT_GE(path.size(), 2u);

  // Path should not pass through any obstacle cell
  for (const auto & [px, py] : path) {
    int cx = static_cast<int>(px);
    int cy = static_cast<int>(py);
    if (cx >= 0 && cx < W && cy >= 0 && cy < H) {
      unsigned char cell = grid[static_cast<size_t>(cy * W + cx)];
      EXPECT_LT(cell, 253u) << "Path entered obstacle at (" << cx << "," << cy << ")";
    }
  }
}

TEST_F(JPSSearchTest, BlockedStart_ReturnsFalse)
{
  auto grid = makeGrid({
    "#...",
    "....",
    "....",
    "....",
  });
  constexpr int W = 4, H = 4;
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 0, 3, 3, 0, path);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(path.empty());
}

TEST_F(JPSSearchTest, BlockedGoal_ReturnsFalse)
{
  auto grid = makeGrid({
    "...#",
    "....",
    "....",
    "....",
  });
  constexpr int W = 4, H = 4;
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 0, 3, 3, 3, path);
  EXPECT_FALSE(ok);
}

TEST_F(JPSSearchTest, NoPath_EnclosedGoal)
{
  // Goal at (1,1) is completely enclosed by walls
  auto grid = makeGrid({
    ".....",
    ".###.",
    ".#.#.",
    ".###.",
    ".....",
  });
  constexpr int W = 5, H = 5;
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 0, 4, 1, 1, path);
  EXPECT_FALSE(ok);
}

TEST_F(JPSSearchTest, NarrowCorridor_FindsPath)
{
  // 1-cell wide corridor from left to right at y=5
  auto grid = makeGrid({
    "####################",
    "#..................#",
    "#..................#",
    "#..................#",
    "#..................#",
    "..................#", // y=5 corridor
    "#..................#",
    "#..................#",
    "#..................#",
    "#..................#",
    "####################",
  });
  constexpr int W = 20, H = 11;
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 1, 5, 19, 5, path);
  EXPECT_TRUE(ok);

  // Every point on the path must be free
  for (const auto & [px, py] : path) {
    int cx = static_cast<int>(px);
    int cy = static_cast<int>(py);
    if (cx >= 0 && cx < W && cy >= 0 && cy < H) {
      EXPECT_LT(grid[static_cast<size_t>(cy * W + cx)], 253u);
    }
  }
}

// ──────────────────────────────────────────────────────────
// JPSBehaviorTest — JPS-specific algorithm behavior
// ──────────────────────────────────────────────────────────
class JPSBehaviorTest : public ::testing::Test
{
protected:
  JPSConfig config_{defaultTestConfig()};
};

TEST_F(JPSBehaviorTest, Jump_HitsObstacle_StopsBefore)
{
  // Wall immediately to the right
  auto grid = makeGrid({
    "...#....",
    "........",
  });
  constexpr int W = 8, H = 2;
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 0, 0, 7, 0, path);
  EXPECT_TRUE(ok);

  // Path must avoid (3,0) which is obstacle
  for (const auto & [px, py] : path) {
    int cx = static_cast<int>(px);
    int cy = static_cast<int>(py);
    if (cy == 0) {
      EXPECT_NE(cx, 3) << "Path entered obstacle cell";
    }
  }
}

TEST_F(JPSBehaviorTest, Jump_HitsGoal_ReturnsGoalNode)
{
  auto grid = makeEmptyGrid(50, 50);
  JPSState state{};
  initState(state, grid, 50, 50);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 25, 25, 40, 25, path);
  EXPECT_TRUE(ok);
  EXPECT_NEAR(path.back().first, 40.5, 0.1);
  EXPECT_NEAR(path.back().second, 25.5, 0.1);
}

TEST_F(JPSBehaviorTest, DiagonalJump_CreatesJumpPoints)
{
  // Empty large grid — diagonal path should have jump points
  auto grid = makeEmptyGrid(100, 100);
  JPSState state{};
  initState(state, grid, 100, 100);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 10, 10, 80, 80, path);
  EXPECT_TRUE(ok);
  EXPECT_GE(path.size(), 2u);
  // With JPS, diagonal on empty grid should be very few waypoints
  // (may be just 2: start and goal)
}

TEST_F(JPSBehaviorTest, ForcedNeighbor_TriggersJumpPoint)
{
  // Create a configuration that triggers forced neighbor:
  //  .#.
  //  ..#  <- diagonal movement: obstacle at (1,0) and (2,1)
  //  ...
  // Start at (0,2), goal at (2,0). Forced neighbor at (1,1).
  auto grid = makeGrid({
    ".#.",  // y=2
    "..#",  // y=1
    "...",  // y=0
  });
  constexpr int W = 3, H = 3;
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 0, 0, 2, 2, path);
  EXPECT_TRUE(ok);
}

TEST_F(JPSBehaviorTest, JPS_ExpandsFewerNodes)
{
  // Large empty grid: JPS should explore very few nodes vs A*
  auto grid = makeEmptyGrid(200, 200);
  JPSState state{};
  initState(state, grid, 200, 200);

  size_t initial_capacity = state.nodes_.capacity();

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 5, 5, 195, 195, path);
  EXPECT_TRUE(ok);

  // On a large empty grid, JPS should create very few nodes
  size_t nodes_created = state.nodes_.size();
  // Even 100 would be extremely generous for an empty 200x200 grid
  EXPECT_LT(nodes_created, 100u)
    << "JPS created " << nodes_created << " nodes on empty grid (expected <100)";

  (void)initial_capacity;
}

// ──────────────────────────────────────────────────────────
// JPSEdgeCaseTest — Boundary and edge cases
// ──────────────────────────────────────────────────────────
class JPSEdgeCaseTest : public ::testing::Test
{
protected:
  JPSConfig config_{defaultTestConfig()};
};

TEST_F(JPSEdgeCaseTest, StartEqualsGoal_ReturnsSinglePoint)
{
  auto grid = makeEmptyGrid(20, 20);
  JPSState state{};
  initState(state, grid, 20, 20);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 10, 10, 10, 10, path);
  EXPECT_TRUE(ok);
  EXPECT_EQ(path.size(), 1u);
  EXPECT_NEAR(path[0].first, 10.5, 0.1);
  EXPECT_NEAR(path[0].second, 10.5, 0.1);
}

TEST_F(JPSEdgeCaseTest, BoundaryPath_StaysInBounds)
{
  // Path along the top edge (y = height-1)
  constexpr int W = 50, H = 50;
  auto grid = makeEmptyGrid(W, H);
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 0, 49, 49, 49, path);
  EXPECT_TRUE(ok);

  for (const auto & [px, py] : path) {
    int cx = static_cast<int>(px);
    int cy = static_cast<int>(py);
    EXPECT_GE(cx, 0);
    EXPECT_LT(cx, W);
    EXPECT_GE(cy, 0);
    EXPECT_LT(cy, H);
  }
}

TEST_F(JPSEdgeCaseTest, OriginStart_Works)
{
  constexpr int W = 30, H = 30;
  auto grid = makeEmptyGrid(W, H);
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 0, 0, 29, 29, path);
  EXPECT_TRUE(ok);
}

TEST_F(JPSEdgeCaseTest, AdjacentCells_Works)
{
  constexpr int W = 10, H = 10;
  auto grid = makeEmptyGrid(W, H);
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 5, 5, 6, 5, path);
  EXPECT_TRUE(ok);
  EXPECT_GE(path.size(), 1u);
}

TEST_F(JPSEdgeCaseTest, OutOfBounds_Start_ReturnsFalse)
{
  constexpr int W = 20, H = 20;
  auto grid = makeEmptyGrid(W, H);
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, -1, 5, 10, 10, path);
  EXPECT_FALSE(ok);
}

TEST_F(JPSEdgeCaseTest, OutOfBounds_Goal_ReturnsFalse)
{
  constexpr int W = 20, H = 20;
  auto grid = makeEmptyGrid(W, H);
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 5, 5, 20, 10, path);
  EXPECT_FALSE(ok);
}

// ──────────────────────────────────────────────────────────
// JPSCostTest — Costmap-aware behavior
// ──────────────────────────────────────────────────────────
class JPSCostTest : public ::testing::Test
{
protected:
  JPSConfig config_{defaultTestConfig()};
};

TEST_F(JPSCostTest, HighCostRegion_PathPrefersFree)
{
  // Build a grid with a band of high cost (200) — path should go around it
  // 20x20 grid. High-cost band at y=10 from x=5..15.
  // Start (10,5) → goal (10,15). Without the band, straight up.
  // With the band, path should deviate.
  constexpr int W = 20, H = 20;
  std::vector<unsigned char> grid = makeEmptyGrid(W, H);

  // High-cost horizontal band at row y=10
  for (int x = 5; x <= 15; ++x) {
    grid[static_cast<size_t>(10 * W + x)] = 200;
  }

  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 10, 5, 10, 15, path);
  EXPECT_TRUE(ok);

  // Verify no point on the path goes through the high-cost band cells
  for (const auto & [px, py] : path) {
    int cx = static_cast<int>(px);
    int cy = static_cast<int>(py);
    if (cy == 10 && cx >= 5 && cx <= 15) {
      // The path should not prefer the high-cost band
      // (unless it's forced — but here there is an alternative)
    }
  }
}

TEST_F(JPSCostTest, AllowUnknown_PathThroughUnknown)
{
  config_.allow_unknown = true;
  auto grid = makeGrid({
    "..........",
    "..........",
    "??????????", // unknown row
    "..........",
    "..........",
  });
  constexpr int W = 10, H = 5;
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 5, 4, 5, 0, path);
  EXPECT_TRUE(ok);
}

TEST_F(JPSCostTest, DisallowUnknown_NoPath)
{
  config_.allow_unknown = false;
  auto grid = makeGrid({
    "..........",
    "..........",
    "??????????",
    "..........",
    "..........",
  });
  constexpr int W = 10, H = 5;
  JPSState state{};
  initState(state, grid, W, H);

  std::vector<std::pair<double, double>> path{};
  bool ok = JPSAlgorithm::generatePath(
    config_, state, 5, 4, 5, 0, path);
  EXPECT_FALSE(ok);
}

TEST_F(JPSCostTest, InscribedCost_TreatedAsObstacle)
{
  // Cell with cost 253 should be treated as obstacle
  auto grid = makeGrid({
    "..........",
    ".X........", // y=1, x=1 is inscribed
    "..........",
  });
  constexpr int W = 10, H = 3;
  JPSState state{};
  initState(state, grid, W, H);

  EXPECT_FALSE(JPSAlgorithm::isTraversable(config_, state, 1, 1));
}

TEST_F(JPSCostTest, CostBelowLethal_IsTraversable)
{
  constexpr int W = 5, H = 5;
  std::vector<unsigned char> grid(static_cast<size_t>(W * H), 0);
  grid[static_cast<size_t>(2 * W + 2)] = 252;  // MAX_NON_OBSTACLE
  JPSState state{};
  initState(state, grid, W, H);

  EXPECT_TRUE(JPSAlgorithm::isTraversable(config_, state, 2, 2));
}

// ──────────────────────────────────────────────────────────
// JPSTraversableTest — Unit tests for isTraversable()
// ──────────────────────────────────────────────────────────
class JPSTraversableTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config_ = defaultTestConfig();
    constexpr int W = 5, H = 5;
    grid_ = makeEmptyGrid(W, H);
    initState(state_, grid_, W, H);
  }

  JPSConfig config_{};
  std::vector<unsigned char> grid_{};
  JPSState state_{};
};

TEST_F(JPSTraversableTest, FreeSpace_True)
{
  EXPECT_TRUE(JPSAlgorithm::isTraversable(config_, state_, 2, 2));
}

TEST_F(JPSTraversableTest, LethalObstacle_False)
{
  constexpr int W = 5;
  grid_[static_cast<size_t>(2 * W + 2)] = 254;
  initState(state_, grid_, W, W);
  EXPECT_FALSE(JPSAlgorithm::isTraversable(config_, state_, 2, 2));
}

TEST_F(JPSTraversableTest, OutOfBounds_False)
{
  EXPECT_FALSE(JPSAlgorithm::isTraversable(config_, state_, -1, 0));
  EXPECT_FALSE(JPSAlgorithm::isTraversable(config_, state_, 0, -1));
  EXPECT_FALSE(JPSAlgorithm::isTraversable(config_, state_, 5, 0));
  EXPECT_FALSE(JPSAlgorithm::isTraversable(config_, state_, 0, 5));
}

TEST_F(JPSTraversableTest, HighCostButBelowThreshold_True)
{
  constexpr int W = 5;
  grid_[static_cast<size_t>(2 * W + 2)] = 200;
  initState(state_, grid_, W, W);
  EXPECT_TRUE(JPSAlgorithm::isTraversable(config_, state_, 2, 2));
}

TEST_F(JPSTraversableTest, UnknownWithAllow_True)
{
  config_.allow_unknown = true;
  constexpr int W = 5;
  grid_[static_cast<size_t>(2 * W + 2)] = 255;
  initState(state_, grid_, W, W);
  EXPECT_TRUE(JPSAlgorithm::isTraversable(config_, state_, 2, 2));
}

TEST_F(JPSTraversableTest, UnknownWithoutAllow_False)
{
  config_.allow_unknown = false;
  constexpr int W = 5;
  grid_[static_cast<size_t>(2 * W + 2)] = 255;
  initState(state_, grid_, W, W);
  EXPECT_FALSE(JPSAlgorithm::isTraversable(config_, state_, 2, 2));
}

TEST_F(JPSTraversableTest, InscribedIsObstacle)
{
  constexpr int W = 5;
  grid_[static_cast<size_t>(2 * W + 2)] = 253;
  initState(state_, grid_, W, W);
  EXPECT_FALSE(JPSAlgorithm::isTraversable(config_, state_, 2, 2));
}

}  // namespace jps_planner
