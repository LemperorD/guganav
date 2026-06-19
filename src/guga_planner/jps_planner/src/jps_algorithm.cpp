#include "jps_planner/jps_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace jps_planner
{

// ──────────────────────────────────────────────────────────
// Anonymous namespace — helper free functions (Pattern A)
// ──────────────────────────────────────────────────────────
namespace
{

constexpr unsigned char UNKNOWN_COST = 255;
constexpr unsigned char LETHAL_COST = 254;
constexpr unsigned char INSCRIBED_COST = 253;
constexpr unsigned char MAX_NON_OBSTACLE = 252;

/** @brief Raw bounds-checked costmap lookup. */
[[nodiscard]] inline unsigned char getCost(
  const JPSState & s, int x, int y)
{
  if (x < 0 || x >= s.size_x || y < 0 || y >= s.size_y) { return UNKNOWN_COST; }
  return s.costmap_data[static_cast<size_t>(y * s.size_x + x)];
}

/** @brief Cell is blocked (obstacle or inflated). */
[[nodiscard]] inline bool isObstacle(
  const JPSConfig & c, const JPSState & s, int x, int y)
{
  auto cost = getCost(s, x, y);
  if (cost == UNKNOWN_COST && c.allow_unknown) { return false; }
  return cost >= INSCRIBED_COST;
}

/** @brief Within grid bounds (inclusive). */
[[nodiscard]] inline bool withinLimits(
  const JPSState & s, int x, int y)
{
  return x >= 0 && x < s.size_x && y >= 0 && y < s.size_y;
}

/** @brief Scaled traversal cost from raw costmap value (ThetaStar formula). */
[[nodiscard]] inline double scaledCost(unsigned char raw)
{
  double sc = 26.0 + 0.9 * static_cast<double>(raw);
  return sc * sc / (MAX_NON_OBSTACLE * MAX_NON_OBSTACLE);
}

/** @brief Weighted traversal cost for a single cell. */
[[nodiscard]] inline double traversalCost(
  const JPSConfig & c, unsigned char raw)
{
  return c.w_traversal_cost * scaledCost(raw);
}

/** @brief Weighted Euclidean distance heuristic. */
[[nodiscard]] inline double heuristic(
  const JPSConfig & c, int x1, int y1, int x2, int y2)
{
  return c.w_heuristic_cost * std::hypot(
    static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));
}

/** @brief Weighted Euclidean distance between two cells. */
[[nodiscard]] inline double euclideanCost(
  const JPSConfig & c, int ax, int ay, int bx, int by)
{
  return c.w_euc_cost * std::hypot(
    static_cast<double>(ax - bx), static_cast<double>(ay - by));
}

/**
 * @brief Check forced neighbors for straight horizontal move (dx = ±1, dy = 0).
 *
 * Forced neighbor exists when a cell adjacent to the jump direction
 * is blocked but the cell diagonally beyond it is free — meaning that
 * the blocked cell would not be reachable optimally if we skipped this node.
 */
[[nodiscard]] bool hasForcedNeighborHoriz(
  const JPSConfig & c, const JPSState & s, int x, int y, int dx)
{
  return (isObstacle(c, s, x, y + 1) && !isObstacle(c, s, x + dx, y + 1)) ||
         (isObstacle(c, s, x, y - 1) && !isObstacle(c, s, x + dx, y - 1));
}

/**
 * @brief Check forced neighbors for straight vertical move (dx = 0, dy = ±1).
 */
[[nodiscard]] bool hasForcedNeighborVert(
  const JPSConfig & c, const JPSState & s, int x, int y, int dy)
{
  return (isObstacle(c, s, x + 1, y) && !isObstacle(c, s, x + 1, y + dy)) ||
         (isObstacle(c, s, x - 1, y) && !isObstacle(c, s, x - 1, y + dy));
}

/**
 * @brief Check forced neighbors for diagonal move (dx = ±1, dy = ±1).
 */
[[nodiscard]] bool hasForcedNeighborDiag(
  const JPSConfig & c, const JPSState & s, int x, int y, int dx, int dy)
{
  // Cells just behind the diagonal direction are blocked but
  // beyond-diagonal cells are free
  return (isObstacle(c, s, x - dx, y) && !isObstacle(c, s, x - dx, y + dy)) ||
         (isObstacle(c, s, x, y - dy) && !isObstacle(c, s, x + dx, y - dy));
}

/** @brief Unified forced-neighbor check dispatching on direction. */
[[nodiscard]] bool hasForcedNeighbor(
  const JPSConfig & c, const JPSState & s, int x, int y, int dx, int dy)
{
  if (dx != 0 && dy == 0) { return hasForcedNeighborHoriz(c, s, x, y, dx); }
  if (dx == 0 && dy != 0) { return hasForcedNeighborVert(c, s, x, y, dy); }
  if (dx != 0 && dy != 0) { return hasForcedNeighborDiag(c, s, x, y, dx, dy); }
  return false;
}

// ──────────────────────────────────────────────────────────
// JPS successor identification
// ──────────────────────────────────────────────────────────

/**
 * @brief Prune non-natural neighbors given a parent direction.
 *
 * Returns the set of direction vectors that are "natural" from
 * (x, y) when arriving from parent in direction (dx, dy).
 * Additionally checks for forced neighbors in the pruned directions.
 *
 * @param dx        Parent-to-current x direction.
 * @param dy        Parent-to-current y direction.
 * @param c, s, x, y  Current state.
 * @param directions Output set of natural + forced-successor directions.
 */
void pruneNeighbors(
  const JPSConfig & c, const JPSState & s, int x, int y, int dx, int dy,
  std::vector<std::pair<int, int>> & directions)
{
  directions.clear();

  // ── Straight horizontal parent direction ──
  if (dx != 0 && dy == 0) {
    // Natural: keep going straight
    directions.emplace_back(dx, 0);

    // Forced: check above/below if blocked diagonal would create forced neighbor
    if (isObstacle(c, s, x, y + 1) && !isObstacle(c, s, x + dx, y + 1)) {
      directions.emplace_back(dx, 1);
    }
    if (isObstacle(c, s, x, y - 1) && !isObstacle(c, s, x + dx, y - 1)) {
      directions.emplace_back(dx, -1);
    }
    return;
  }

  // ── Straight vertical parent direction ──
  if (dx == 0 && dy != 0) {
    directions.emplace_back(0, dy);

    if (isObstacle(c, s, x + 1, y) && !isObstacle(c, s, x + 1, y + dy)) {
      directions.emplace_back(1, dy);
    }
    if (isObstacle(c, s, x - 1, y) && !isObstacle(c, s, x - 1, y + dy)) {
      directions.emplace_back(-1, dy);
    }
    return;
  }

  // ── Diagonal parent direction ──
  if (dx != 0 && dy != 0) {
    // Natural: keep going in the same diagonal + the two cardinal components
    directions.emplace_back(dx, dy);
    directions.emplace_back(dx, 0);
    directions.emplace_back(0, dy);

    // Forced neighbors from diagonal pruning
    if (isObstacle(c, s, x - dx, y) && !isObstacle(c, s, x - dx, y + dy)) {
      directions.emplace_back(-dx, dy);
    }
    if (isObstacle(c, s, x, y - dy) && !isObstacle(c, s, x + dx, y - dy)) {
      directions.emplace_back(dx, -dy);
    }
    return;
  }
}

/**
 * @brief Recursive JPS jump.
 *
 * Moves in direction (dx, dy) from (x, y), accumulating traversal costs.
 * Stops at obstacles, forced neighbors, or the goal.
 *
 * @param x, y    Current cell (NOT yet evaluated — start of jump).
 * @param dx, dy  Jump direction.
 * @param gx, gy  Goal coordinates.
 * @param acc     [in/out] Accumulated traversal cost from the calling jump point.
 * @return        Pointer to the destination jump-point SearchNode, or nullptr
 *                if jumping hits an obstacle immediately.
 */
SearchNode * jump(
  const JPSConfig & c, JPSState & s, int x, int y, int dx, int dy,
  int gx, int gy, double & acc)
{
  int nx = x + dx;
  int ny = y + dy;

  // First step — immediate block check
  if (!withinLimits(s, nx, ny) || isObstacle(c, s, nx, ny)) { return nullptr; }

  // Accumulate cost for this step
  acc += traversalCost(c, getCost(s, nx, ny));

  // Goal reached
  if (nx == gx && ny == gy) {
    // Retrieve or create a node for this cell
    auto idx = static_cast<size_t>(ny * s.size_x + nx);
    if (s.node_position_[idx] != nullptr) { return s.node_position_[idx]; }
    auto & ptr = s.nodes_.emplace_back(
      std::make_unique<SearchNode>(SearchNode{nx, ny}));
    s.node_position_[idx] = ptr.get();
    return ptr.get();
  }

  // Forced neighbor → this cell is a jump point
  if (hasForcedNeighbor(c, s, nx, ny, dx, dy)) {
    auto idx = static_cast<size_t>(ny * s.size_x + nx);
    if (s.node_position_[idx] != nullptr) { return s.node_position_[idx]; }
    auto & ptr = s.nodes_.emplace_back(
      std::make_unique<SearchNode>(SearchNode{nx, ny}));
    s.node_position_[idx] = ptr.get();
    return ptr.get();
  }

  // ── Diagonal jump: also check cardinal components ──
  if (dx != 0 && dy != 0) {
    double dummy_h{};
    double dummy_v{};
    if (jump(c, s, nx, ny, dx, 0, gx, gy, dummy_h) != nullptr ||
        jump(c, s, nx, ny, 0, dy, gx, gy, dummy_v) != nullptr) {
      auto idx = static_cast<size_t>(ny * s.size_x + nx);
      if (s.node_position_[idx] != nullptr) { return s.node_position_[idx]; }
      auto & ptr = s.nodes_.emplace_back(
        std::make_unique<SearchNode>(SearchNode{nx, ny}));
      s.node_position_[idx] = ptr.get();
      return ptr.get();
    }
  }

  // Continue jumping
  return jump(c, s, nx, ny, dx, dy, gx, gy, acc);
}

/**
 * @brief Identify all jump-point successors of the current node.
 *
 * For a node with no parent (start), explores all 8 directions.
 * For a node with a parent, uses neighbor pruning rules.
 */
void identifySuccessors(
  const JPSConfig & c, JPSState & s, const SearchNode * current,
  int gx, int gy, std::vector<std::pair<SearchNode *, double>> & successors)
{
  successors.clear();

  int cx = current->x;
  int cy = current->y;
  std::vector<std::pair<int, int>> directions{};

  if (current->parent == nullptr) {
    // Start node — check all 8 directions
    static constexpr int all_dirs[8][2] = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};
    for (auto [dx, dy] : all_dirs) {
      double acc_cost{0.0};
      auto * succ = jump(c, s, cx, cy, dx, dy, gx, gy, acc_cost);
      if (succ != nullptr) {
        successors.emplace_back(succ, acc_cost);
      }
    }
    return;
  }

  // Compute parent direction (normalised to -1/0/1)
  int pdx = cx - current->parent->x;
  int pdy = cy - current->parent->y;
  if (pdx != 0) { pdx = pdx > 0 ? 1 : -1; }
  if (pdy != 0) { pdy = pdy > 0 ? 1 : -1; }

  pruneNeighbors(c, s, cx, cy, pdx, pdy, directions);

  for (auto [dx, dy] : directions) {
    double acc_cost{0.0};
    auto * succ = jump(c, s, cx, cy, dx, dy, gx, gy, acc_cost);
    if (succ != nullptr) {
      successors.emplace_back(succ, acc_cost);
    }
  }
}

/**
 * @brief Backtrace from goal to start via parent pointers to build the path.
 * @param goal  Goal search node.
 * @param path  Output path in map coordinates (cell centers).
 */
void backtracePath(
  const SearchNode * goal, std::vector<std::pair<double, double>> & path)
{
  path.clear();
  const SearchNode * n = goal;
  while (n != nullptr) {
    // Cell center coordinates
    path.emplace_back(
      static_cast<double>(n->x) + 0.5, static_cast<double>(n->y) + 0.5);
    n = n->parent;
  }
  // Reverse to go start → goal
  std::reverse(path.begin(), path.end());
}

}  // namespace

// ──────────────────────────────────────────────────────────
// Public API
// ──────────────────────────────────────────────────────────

bool JPSAlgorithm::generatePath(
  const JPSConfig & c, JPSState & s, int sx, int sy, int gx, int gy,
  std::vector<std::pair<double, double>> & path)
{
  path.clear();

  // Bounds + traversability check on start and goal
  if (!withinLimits(s, sx, sy) || !withinLimits(s, gx, gy)) { return false; }
  if (!isTraversable(c, s, sx, sy) || !isTraversable(c, s, gx, gy)) {
    return false;
  }

  // Trivial case: start == goal
  if (sx == gx && sy == gy) {
    path.emplace_back(
      static_cast<double>(sx) + 0.5, static_cast<double>(sy) + 0.5);
    return true;
  }

  // Reset state for a fresh search
  s.nodes_.clear();
  s.open_list_ = {};
  s.node_position_.assign(
    static_cast<size_t>(s.size_x * s.size_y), nullptr);

  const size_t total_cells = static_cast<size_t>(s.size_x * s.size_y);
  s.nodes_.reserve(total_cells / 4);  // Heuristic: ~25% cells visited

  // Create start node
  auto & start_ptr = s.nodes_.emplace_back(
    std::make_unique<SearchNode>(SearchNode{sx, sy}));
  SearchNode * start = start_ptr.get();
  start->g = 0.0;
  start->h = heuristic(c, sx, sy, gx, gy);
  start->f = start->h;
  s.node_position_[static_cast<size_t>(sy * s.size_x + sx)] = start;
  s.open_list_.push(start);

  std::vector<std::pair<SearchNode *, double>> successors{};
  successors.reserve(8);

  while (!s.open_list_.empty()) {
    SearchNode * current = s.open_list_.top();
    s.open_list_.pop();

    if (current->closed) { continue; }
    current->closed = true;

    // Goal check
    if (current->x == gx && current->y == gy) {
      backtracePath(current, path);
      return true;
    }

    // Expand
    identifySuccessors(c, s, current, gx, gy, successors);

    for (auto [succ, jump_cost] : successors) {
      if (succ->closed) { continue; }

      double tentative_g = current->g + jump_cost +
        euclideanCost(c, current->x, current->y, succ->x, succ->y);

      if (tentative_g < succ->g) {
        succ->g = tentative_g;
        succ->h = heuristic(c, succ->x, succ->y, gx, gy);
        succ->f = succ->g + succ->h;
        succ->parent = current;
        s.open_list_.push(succ);
      }
    }
  }

  // Open list exhausted — no path
  return false;
}

bool JPSAlgorithm::isTraversable(
  const JPSConfig & c, const JPSState & s, int x, int y)
{
  if (x < 0 || x >= s.size_x || y < 0 || y >= s.size_y) { return false; }
  auto cost = getCost(s, x, y);
  if (cost == UNKNOWN_COST) { return c.allow_unknown; }
  return cost < INSCRIBED_COST;
}

}  // namespace jps_planner
