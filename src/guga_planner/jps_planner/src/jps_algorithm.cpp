#include "jps_planner/jps_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace jps_planner
{

// ══════════════════════════════════════════════════════════
// 匿名 namespace — 辅助自由函数 (模式 A)
// ══════════════════════════════════════════════════════════
namespace
{

constexpr unsigned char UNKNOWN_COST = 255;
constexpr unsigned char LETHAL_COST = 254;
constexpr unsigned char INSCRIBED_COST = 253;
constexpr unsigned char MAX_NON_OBSTACLE = 252;

/** @brief 带边界检查的代价地图原始查询。 */
[[nodiscard]] inline unsigned char getCost(
  const JPSState & s, int x, int y)
{
  if (x < 0 || x >= s.size_x || y < 0 || y >= s.size_y) { return UNKNOWN_COST; }
  return s.costmap_data[static_cast<size_t>(y * s.size_x + x)];
}

/** @brief 该格元是否被阻塞 (障碍物或膨胀区域)。 */
[[nodiscard]] inline bool isObstacle(
  const JPSConfig & c, const JPSState & s, int x, int y)
{
  auto cost = getCost(s, x, y);
  if (cost == UNKNOWN_COST && c.allow_unknown) { return false; }
  return cost >= INSCRIBED_COST;
}

/** @brief 是否在网格边界内 (含边界)。 */
[[nodiscard]] inline bool withinLimits(
  const JPSState & s, int x, int y)
{
  return x >= 0 && x < s.size_x && y >= 0 && y < s.size_y;
}

/** @brief 原始 costmap 值映射到缩放代价值 (ThetaStar 公式)。 */
[[nodiscard]] inline double scaledCost(unsigned char raw)
{
  double sc = 26.0 + 0.9 * static_cast<double>(raw);
  return sc * sc / (MAX_NON_OBSTACLE * MAX_NON_OBSTACLE);
}

/** @brief 单个格元的加权通行代价。 */
[[nodiscard]] inline double traversalCost(
  const JPSConfig & c, unsigned char raw)
{
  return c.w_traversal_cost * scaledCost(raw);
}

/** @brief 加权欧几里得距离启发函数。 */
[[nodiscard]] inline double heuristic(
  const JPSConfig & c, int x1, int y1, int x2, int y2)
{
  return c.w_heuristic_cost * std::hypot(
    static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));
}

/** @brief 两格元间的加权欧几里得距离。 */
[[nodiscard]] inline double euclideanCost(
  const JPSConfig & c, int ax, int ay, int bx, int by)
{
  return c.w_euc_cost * std::hypot(
    static_cast<double>(ax - bx), static_cast<double>(ay - by));
}

/**
 * @brief 检测水平直行方向 (dx = ±1, dy = 0) 的强制邻居。
 *
 * 当跳跃方向旁边的格元被阻塞、但其对角线远处格元空闲时,
 * 该节点成为强制邻居 —— 因为跳过它会导致被阻塞格元不可达。
 */
[[nodiscard]] bool hasForcedNeighborHoriz(
  const JPSConfig & c, const JPSState & s, int x, int y, int dx)
{
  return (isObstacle(c, s, x, y + 1) && !isObstacle(c, s, x + dx, y + 1)) ||
         (isObstacle(c, s, x, y - 1) && !isObstacle(c, s, x + dx, y - 1));
}

/** @brief 检测垂直直行方向 (dx = 0, dy = ±1) 的强制邻居。 */
[[nodiscard]] bool hasForcedNeighborVert(
  const JPSConfig & c, const JPSState & s, int x, int y, int dy)
{
  return (isObstacle(c, s, x + 1, y) && !isObstacle(c, s, x + 1, y + dy)) ||
         (isObstacle(c, s, x - 1, y) && !isObstacle(c, s, x - 1, y + dy));
}

/**
 * @brief 检测对角线方向 (dx = ±1, dy = ±1) 的强制邻居。
 *
 * 对角线背后的两个格元中, 其中一个被阻塞而远处格元空闲时触发。
 */
[[nodiscard]] bool hasForcedNeighborDiag(
  const JPSConfig & c, const JPSState & s, int x, int y, int dx, int dy)
{
  return (isObstacle(c, s, x - dx, y) && !isObstacle(c, s, x - dx, y + dy)) ||
         (isObstacle(c, s, x, y - dy) && !isObstacle(c, s, x + dx, y - dy));
}

/** @brief 统一的强制邻居检测, 根据方向分量分发到对应检测函数。 */
[[nodiscard]] bool hasForcedNeighbor(
  const JPSConfig & c, const JPSState & s, int x, int y, int dx, int dy)
{
  if (dx != 0 && dy == 0) { return hasForcedNeighborHoriz(c, s, x, y, dx); }
  if (dx == 0 && dy != 0) { return hasForcedNeighborVert(c, s, x, y, dy); }
  if (dx != 0 && dy != 0) { return hasForcedNeighborDiag(c, s, x, y, dx, dy); }
  return false;
}

// ══════════════════════════════════════════════════════════
// JPS 后继节点识别
// ══════════════════════════════════════════════════════════

/**
 * @brief 根据父节点方向裁剪非自然邻居。
 *
 * 返回从 (x, y) 出发、父节点方向为 (dx, dy) 时的"自然"方向集合。
 * 同时检查被裁剪方向上是否存在强制邻居。
 *
 * @param dx        父节点到当前节点的 x 方向分量。
 * @param dy        父节点到当前节点的 y 方向分量。
 * @param c, s, x, y  当前位置与状态。
 * @param directions 输出: 自然方向 + 强制邻接方向集合。
 */
void pruneNeighbors(
  const JPSConfig & c, const JPSState & s, int x, int y, int dx, int dy,
  std::vector<std::pair<int, int>> & directions)
{
  directions.clear();

  // ── 水平直行父方向 ──
  if (dx != 0 && dy == 0) {
    directions.emplace_back(dx, 0);  // 自然方向: 继续直行

    if (isObstacle(c, s, x, y + 1) && !isObstacle(c, s, x + dx, y + 1)) {
      directions.emplace_back(dx, 1);
    }
    if (isObstacle(c, s, x, y - 1) && !isObstacle(c, s, x + dx, y - 1)) {
      directions.emplace_back(dx, -1);
    }
    return;
  }

  // ── 垂直直行父方向 ──
  if (dx == 0 && dy != 0) {
    directions.emplace_back(0, dy);  // 自然方向: 继续直行

    if (isObstacle(c, s, x + 1, y) && !isObstacle(c, s, x + 1, y + dy)) {
      directions.emplace_back(1, dy);
    }
    if (isObstacle(c, s, x - 1, y) && !isObstacle(c, s, x - 1, y + dy)) {
      directions.emplace_back(-1, dy);
    }
    return;
  }

  // ── 对角线父方向 ──
  if (dx != 0 && dy != 0) {
    // 自然方向: 继续对角线 + 两个分量方向
    directions.emplace_back(dx, dy);
    directions.emplace_back(dx, 0);
    directions.emplace_back(0, dy);

    // 对角线裁剪产生的强制邻居
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
 * @brief 递归 JPS 跳跃。
 *
 * 从 (x, y) 沿方向 (dx, dy) 移动, 累计通行代价。
 * 遇到障碍物 / 强制邻居 / 终点时停止。
 *
 * @param x, y    当前格元 (尚未评估, 跳跃的起点)。
 * @param dx, dy  跳跃方向。
 * @param gx, gy  终点坐标。
 * @param acc     [入/出] 从调用跳转点开始的累计通行代价。
 * @return        目的地跳转点 SearchNode 的指针; 如果跳跃立即撞到障碍物则返回 nullptr。
 */
SearchNode * jump(
  const JPSConfig & c, JPSState & s, int x, int y, int dx, int dy,
  int gx, int gy, double & acc)
{
  int nx = x + dx;
  int ny = y + dy;

  // 第一步 — 立即检查是否阻塞
  if (!withinLimits(s, nx, ny) || isObstacle(c, s, nx, ny)) { return nullptr; }

  // 累计此步的通行代价
  acc += traversalCost(c, getCost(s, nx, ny));

  // 到达终点
  if (nx == gx && ny == gy) {
    auto idx = static_cast<size_t>(ny * s.size_x + nx);
    if (s.node_position_[idx] != nullptr) { return s.node_position_[idx]; }
    auto & ptr = s.nodes_.emplace_back(
      std::make_unique<SearchNode>(SearchNode{nx, ny}));
    s.node_position_[idx] = ptr.get();
    if (s.debug_.enabled) {
      s.debug_.jumppoint_x.push_back(nx);
      s.debug_.jumppoint_y.push_back(ny);
    }
    return ptr.get();
  }

  // 强制邻居 → 此格元为跳转点
  if (hasForcedNeighbor(c, s, nx, ny, dx, dy)) {
    auto idx = static_cast<size_t>(ny * s.size_x + nx);
    if (s.node_position_[idx] != nullptr) { return s.node_position_[idx]; }
    auto & ptr = s.nodes_.emplace_back(
      std::make_unique<SearchNode>(SearchNode{nx, ny}));
    s.node_position_[idx] = ptr.get();
    if (s.debug_.enabled) {
      s.debug_.jumppoint_x.push_back(nx);
      s.debug_.jumppoint_y.push_back(ny);
    }
    return ptr.get();
  }

  // ── 对角跳跃: 还须检查两个分量方向 ──
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
      if (s.debug_.enabled) {
        s.debug_.jumppoint_x.push_back(nx);
        s.debug_.jumppoint_y.push_back(ny);
      }
      return ptr.get();
    }
  }

  // 继续跳跃
  return jump(c, s, nx, ny, dx, dy, gx, gy, acc);
}

/**
 * @brief 识别当前节点的所有跳转点后继。
 *
 * 无父节点 (起点) 时, 探索全部 8 个方向。
 * 有父节点时, 使用邻居裁剪规则。
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
    // 起点 — 检查全部 8 个方向
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

  // 计算父节点方向 (归一化到 -1/0/1)
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
 * @brief 从终点沿父指针回溯到起点, 构建路径。
 * @param goal  终点搜索节点。
 * @param path  输出路径 (地图坐标, 格元中心)。
 */
void backtracePath(
  const SearchNode * goal, std::vector<std::pair<double, double>> & path)
{
  path.clear();
  const SearchNode * n = goal;
  while (n != nullptr) {
    path.emplace_back(
      static_cast<double>(n->x) + 0.5, static_cast<double>(n->y) + 0.5);
    n = n->parent;
  }
  std::reverse(path.begin(), path.end());
}

}  // namespace

// ══════════════════════════════════════════════════════════
// 公开 API
// ══════════════════════════════════════════════════════════

bool JPSAlgorithm::generatePath(
  const JPSConfig & c, JPSState & s, int sx, int sy, int gx, int gy,
  std::vector<std::pair<double, double>> & path)
{
  path.clear();

  // 边界 + 起终点可通行性检查
  if (!withinLimits(s, sx, sy) || !withinLimits(s, gx, gy)) { return false; }
  if (!isTraversable(c, s, sx, sy) || !isTraversable(c, s, gx, gy)) {
    return false;
  }

  // 平凡情况: 起点 == 终点
  if (sx == gx && sy == gy) {
    path.emplace_back(
      static_cast<double>(sx) + 0.5, static_cast<double>(sy) + 0.5);
    return true;
  }

  // 重置状态, 准备一次新的搜索
  s.nodes_.clear();
  s.open_list_ = {};
  s.node_position_.assign(
    static_cast<size_t>(s.size_x * s.size_y), nullptr);

  const size_t total_cells = static_cast<size_t>(s.size_x * s.size_y);
  s.nodes_.reserve(total_cells / 4);  // 经验值: 约 25% 格元被访问

  // 创建起始节点
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

    // 终点检查
    if (current->x == gx && current->y == gy) {
      backtracePath(current, path);
      return true;
    }

    // 展开节点
    identifySuccessors(c, s, current, gx, gy, successors);

    // 调试: 记录已展开节点
    if (s.debug_.enabled) {
      s.debug_.expanded_x.push_back(current->x);
      s.debug_.expanded_y.push_back(current->y);
    }

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

  // 开放列表耗尽 — 无路径
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
