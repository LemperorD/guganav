#include "jps_planner/jps_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace jps_planner{
     // ── 代价地图查询 ──
  /** @brief 判断格元是否被阻塞 (cost ≥ 253 = 障碍物或膨胀区域)。
   *  allow_unknown=true 时, 未知空间 (255) 视为可通行。 */
      // ══════════════════════════════════════════════════════════════════════════════
// isTraversable — 单格元可通行性检查
// ══════════════════════════════════════════════════════════════════════════════

  bool JPSAlgorithm::isTraversable(const JPSConfig& c, const JPSState& s,
                                   int x,int y) {
     if (x < 0 || x >= s.size_x || y < 0 || y >= s.size_y) {
        return false;
      }
      auto cost = getCost(s, x, y);
      // 未知空间: 由 allow_unknown 决定
      if (cost == UNKNOWN_COST) {
        return c.allow_unknown;
      }
      // 代价值 < 253 的格元可通行
      return cost < INSCRIBED_COST;
  }
  bool JPSAlgorithm::isObstacle(const JPSConfig& c,
                                const JPSState& s, int x, int y) {
    auto cost = getCost(s, x, y); 
    if (cost == UNKNOWN_COST && c.allow_unknown) {
      return false;
    }
    return cost >= INSCRIBED_COST;
  }

  /** @brief 判断坐标是否在网格边界内 (含边界)。 */

  /** @brief 判断格元是否可作为可穿越落点。越界始终不可通行。 */

  /** @brief 判断格元是否阻断移动。越界视为阻断, 不受 allow_unknown 影响。 */
  bool JPSAlgorithm::isBlockedCell(const JPSConfig& c,
                                                 const JPSState& s, int x,
                                                 int y) {
    return !isTraversableCell(c, s, x, y);
  }

  /** @brief 判断从 (x,y) 沿 (dx,dy) 前进一步是否合法。 */
  bool JPSAlgorithm::canStep(const JPSConfig& c,
                                           const JPSState& s, int x, int y,
                                           int dx, int dy) {
    int nx = x + dx;
    int ny = y + dy;
    if (!isTraversableCell(c, s, nx, ny)) {
      return false;
    }
    if (dx == 0 || dy == 0) {
      return true;
    }

    // 禁止从两个阻断格之间斜穿；允许贴着单个阻断格绕角。
    // 注意: 对角裁剪/强制邻居规则与该行为配套, 不可单独收紧为
    // "两侧都需可通行"——那会导致绕墙缺口等场景搜不到路径。
    return isTraversableCell(c, s, x + dx, y)
           || isTraversableCell(c, s, x, y + dy);
  }

  // ── 代价函数 ──

  /** @brief 将原始 costmap 值映射为缩放代价值。
   *  使用 Theta* 论文中的公式: s(c) = (26 + 0.9c)² / 252²
   *  此映射使代价差异在高值区域更平滑, 避免障碍物附近代价突变过大。 */
  double JPSAlgorithm::scaledCost(unsigned char raw) {
    double sc = 26.0 + 0.9 * static_cast<double>(raw);
    return sc * sc / (MAX_NON_OBSTACLE * MAX_NON_OBSTACLE);
  }

  /** @brief 单个格元的加权通行代价: t(c) = w_t · s(c)。
   *  以跳转点为单位的跳跃路径累计此代价。 */
  double JPSAlgorithm::traversalCost(const JPSConfig& c,
                                            unsigned char raw) {
    return c.w_traversal_cost * scaledCost(raw);
  }

  /** @brief 加权欧几里得距离启发函数: h = w_h · √((Δx)² + (Δy)²)。
   *  使用欧几里得距离 (而非曼哈顿), 在 8 连通网格上保证 admissible。 */
  double JPSAlgorithm::heuristic(const JPSConfig& c, int x1, int y1,
                                        int x2, int y2) {
    return c.w_heuristic_cost
           * std::hypot(static_cast<double>(x2 - x1),
                        static_cast<double>(y2 - y1));
  }

  /** @brief 两格元间的加权欧几里得距离: d = w_e · √((Δx)² + (Δy)²)。
   *  在 g 值更新时作为跳转点间的候选距离代价。 */
  double JPSAlgorithm::euclideanCost(const JPSConfig& c, int ax, int ay,
                                            int bx, int by) {
    return c.w_euc_cost
           * std::hypot(static_cast<double>(ax - bx),
                        static_cast<double>(ay - by));
  }
}