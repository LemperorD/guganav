#include "jps_planner/jps_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace jps_planner{
// ══════════════════════════════════════════════════════════════════════════════
  // 强制邻居检测 (Forced Neighbour Detection)
  //
  // JPS 的核心创新之一。当跳跃方向旁边的格元被阻塞、但其对角位置
  // 空闲时, 当前格元成为"强制邻居"——跳过它会导致一条更优路径被遗漏。
  // 三个方向的检测函数分别处理水平、垂直和对角线情况。
  //
  // 形式化条件 (详见 DESIGN.md §1.4):
  //   水平: obs(x, y±1) ∧ ¬obs(x+dx, y±1)
  //   垂直: obs(x±1, y) ∧ ¬obs(x±1, y+dy)
  //   对角: obs(x-dx, y) ∧ ¬obs(x-dx, y+dy) ∨ obs(x, y-dy) ∧ ¬obs(x+dx, y-dy)
  // ══════════════════════════════════════════════════════════════════════════════
  // ══════════════════════════════════════════════════════════════════════════════
  // 递归跳跃 (Recursive Jump)
  //
  // JPS 的核心: 沿 (dx, dy) 方向递归前进, 跳过中间节点,
  // 只在以下位置停止:
  //   1. 越界或遇到障碍物 → 返回 nullptr
  //   2. 到达终点 → 返回跳转点
  //   3. 检测到强制邻居 → 返回跳转点
  //   4. 对角线跳跃时, 分量方向存在跳转点 → 返回跳转点
  //
  // 跳跃路径上的通行代价通过 acc 累计传出。
  // ══════════════════════════════════════════════════════════════════════════════
     /**
   * @brief 检测水平直行方向 (dx = ±1, dy = 0) 的强制邻居。
   * 当跳跃方向旁边的格元被阻塞、但其对角线远处格元空闲时触发。 */
  bool JPSAlgorithm::hasForcedNeighborHoriz(const JPSConfig& c,
                                            const JPSState& s, int x, int y,
                                            int dx) {
    return (isBlockedCell(c, s, x, y + 1)
            && isTraversableCell(c, s, x + dx, y + 1))
           || (isBlockedCell(c, s, x, y - 1)
               && isTraversableCell(c, s, x + dx, y - 1));
  }

  /** @brief 检测垂直直行方向 (dx = 0, dy = ±1) 的强制邻居。 */
  bool JPSAlgorithm::hasForcedNeighborVert(const JPSConfig& c,
                                           const JPSState& s, int x, int y,
                                           int dy) {
    return (isBlockedCell(c, s, x + 1, y)
            && isTraversableCell(c, s, x + 1, y + dy))
           || (isBlockedCell(c, s, x - 1, y)
               && isTraversableCell(c, s, x - 1, y + dy));
  }

  /**
   * @brief 检测对角线方向 (dx = ±1, dy = ±1) 的强制邻居。
   * 对角线背后的两个格元中, 其中一个被阻塞而远处格元空闲时触发。 */
  bool JPSAlgorithm::hasForcedNeighborDiag(const JPSConfig& c,
                                           const JPSState& s, int x, int y,
                                           int dx, int dy) {
    return (isBlockedCell(c, s, x - dx, y)
            && isTraversableCell(c, s, x - dx, y + dy))
           || (isBlockedCell(c, s, x, y - dy)
               && isTraversableCell(c, s, x + dx, y - dy));
  }

  /** @brief 统一的强制邻居检测, 根据方向分量分发到对应检测函数。 */
  bool JPSAlgorithm::hasForcedNeighbor(const JPSConfig& c, const JPSState& s,
                                       int x, int y, int dx, int dy) {
    if (dx != 0 && dy == 0) {
      return hasForcedNeighborHoriz(c, s, x, y, dx);
    }
    if (dx == 0 && dy != 0) {
      return hasForcedNeighborVert(c, s, x, y, dy);
    }
    if (dx != 0 && dy != 0) {
      return hasForcedNeighborDiag(c, s, x, y, dx, dy);
    }
    return false;
  }

  // ══════════════════════════════════════════════════════════════════════════════
  // JPS 邻居裁剪 (Neighbour Pruning)
  //
  // 给定父节点方向, 裁剪非自然邻居, 只保留可能产生更优路径的方向。
  //
  // 自然邻居规则:
  //   水平(±1,0):  {(±1, 0)}
  //   垂直(0,±1):  {(0, ±1)}
  //   对角(±1,±1): {(±1, ±1), (±1, 0), (0, ±1)}
  //
  // 同时检查被裁剪方向上是否存在强制邻居, 有则加入方向集合。
  // ══════════════════════════════════════════════════════════════════════════════

  void JPSAlgorithm::pruneNeighbors(const JPSConfig& c, const JPSState& s, int x, int y,
                      int dx, int dy,
                      std::vector<std::pair<int, int>>& directions) {
    directions.clear();

    // 水平直行父方向 (dx = ±1, dy = 0)
    if (dx != 0 && dy == 0) {
      directions.emplace_back(dx, 0);  // 自然方向: 继续直行

      // 检查上方/下方是否存在强制邻居
      if (isBlockedCell(c, s, x, y + 1)
          && isTraversableCell(c, s, x + dx, y + 1)) {
        directions.emplace_back(dx, 1);
      }
      if (isBlockedCell(c, s, x, y - 1)
          && isTraversableCell(c, s, x + dx, y - 1)) {
        directions.emplace_back(dx, -1);
      }
      return;
    }

    // 垂直直行父方向 (dx = 0, dy = ±1)
    if (dx == 0 && dy != 0) {
      directions.emplace_back(0, dy);  // 自然方向: 继续直行

      // 检查左方/右方是否存在强制邻居
      if (isBlockedCell(c, s, x + 1, y)
          && isTraversableCell(c, s, x + 1, y + dy)) {
        directions.emplace_back(1, dy);
      }
      if (isBlockedCell(c, s, x - 1, y)
          && isTraversableCell(c, s, x - 1, y + dy)) {
        directions.emplace_back(-1, dy);
      }
      return;
    }

    // 对角线父方向 (dx = ±1, dy = ±1)
    if (dx != 0 && dy != 0) {
      // 自然方向: 对角线 + 两个轴分量方向
      directions.emplace_back(dx, dy);
      directions.emplace_back(dx, 0);
      directions.emplace_back(0, dy);

      // 对角线裁剪产生的强制邻居
      if (isBlockedCell(c, s, x - dx, y)
          && isTraversableCell(c, s, x - dx, y + dy)) {
        directions.emplace_back(-dx, dy);
      }
      if (isBlockedCell(c, s, x, y - dy)
          && isTraversableCell(c, s, x + dx, y - dy)) {
        directions.emplace_back(dx, -dy);
      }
      return;
    }
  }
}