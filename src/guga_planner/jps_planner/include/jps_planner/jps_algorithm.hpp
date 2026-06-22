#pragma once

#include <cmath>
#include <cstddef>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

namespace jps_planner
{

constexpr double INF_COST = 1e308;

/** @brief Grid coordinate (cell index). */
struct Node
{
  int x{};
  int y{};
};

/**
 * @brief Search node used during JPS planning.
 * Stored in heap-allocated unique_ptr for pointer stability.
 */
struct SearchNode
{
  int x{};
  int y{};
  double g{INF_COST};
  double h{INF_COST};
  double f{INF_COST};
  const SearchNode * parent{nullptr};
  bool closed{false};
};

/** @brief Immutable configuration for the JPS algorithm. */
struct JPSConfig
{
  double w_euc_cost{1.0};
  double w_traversal_cost{10.0};
  double w_heuristic_cost{1.0};
  bool allow_unknown{false};
};

/** @brief Debug data collected during a JPS search (when debug_enabled=true). */
struct JPSDebug
{
  bool enabled{false};
  /** Grid cells expanded (jump-point successors identified). */
  std::vector<int> expanded_x{};
  std::vector<int> expanded_y{};
  /** All jump points discovered (including those never expanded). */
  std::vector<int> jumppoint_x{};
  std::vector<int> jumppoint_y{};
};

/** @brief Mutable state re-created for each planning request. */
struct JPSState
{
  const unsigned char * costmap_data{nullptr};
  int size_x{};
  int size_y{};

  /** Heap-allocated node storage: indices remain valid across resizes. */
  std::vector<std::unique_ptr<SearchNode>> nodes_{};

  /**
   * @brief O(1) map from grid coords to SearchNode*.
   * Indexed as [size_x_ * y + x], nullptr = unvisited.
   */
  std::vector<SearchNode *> node_position_{};

  /** Priority queue (open list) for A* expansion. */
  struct Comp
  {
    bool operator()(const SearchNode * a, const SearchNode * b) const
    {
      return a->f > b->f;
    }
  };
  std::priority_queue<SearchNode *, std::vector<SearchNode *>, Comp> open_list_{};

  /** @brief Debug data populated during search when enabled. */
  JPSDebug debug_{};
};

/**
 * @class JPSAlgorithm
 * @brief Pure static methods implementing Jump Point Search on a costmap grid.
 *
 * Follows Pattern A (函数式数据流): stateless, explicit (const Config&, State&)
 * parameter passing.
 */
class JPSAlgorithm
{
public:
  /**
   * @brief Run JPS from (sx, sy) to (gx, gy) on the grid described by state.
   * @param config  Immutable algorithm parameters.
   * @param state   Mutable working state (must have costmap_data/size populated).
   * @param sx, sy  Start cell coordinates.
   * @param gx, gy  Goal cell coordinates.
   * @param path    Output path in map coordinates (will be cleared first).
   * @return true if a path was found, false otherwise.
   */
  [[nodiscard]] static bool generatePath(
    const JPSConfig & config, JPSState & state, int sx, int sy, int gx, int gy,
    std::vector<std::pair<double, double>> & path);

  /**
   * @brief Check whether cell (x, y) is traversable.
   * @param config  Algorithm parameters (controls unknown-space policy).
   * @param state   State with costmap data pointer.
   * @param x, y    Cell coordinates.
   * @return true if the cell can be entered.
   */
  [[nodiscard]] static bool isTraversable(
    const JPSConfig & config, const JPSState & state, int x, int y);
};

}  // namespace jps_planner
