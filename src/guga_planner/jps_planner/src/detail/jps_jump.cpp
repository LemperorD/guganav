#include "jps_planner/jps_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
namespace jps_planner{
    SearchNode* JPSAlgorithm::jump(const JPSConfig& c, JPSState& s, int x, int y, int dx,
                   int dy, int gx, int gy, double& acc) {
    int nx = x + dx;
    int ny = y + dy;

    // 终止条件 1: 越界、撞墙或非法对角切角
    if (!canStep(c, s, x, y, dx, dy)) {
      return nullptr;
    }

    // 累计当前格元的通行代价
    acc += traversalCost(c, getCost(s, nx, ny));

    // 终止条件 2: 到达终点 → 此格元为跳转点
    if (nx == gx && ny == gy) {
      auto idx = static_cast<size_t>(ny * s.size_x + nx);
      if (s.node_position_[idx] != nullptr) {
        return s.node_position_[idx];
      }
      auto& ptr = s.nodes_.emplace_back(
          std::make_unique<SearchNode>(SearchNode{nx, ny}));
      s.node_position_[idx] = ptr.get();
      if (s.debug_.enabled) {
        s.debug_.jumppoint_x.push_back(nx);
        s.debug_.jumppoint_y.push_back(ny);
      }
      return ptr.get();
    }

    // 终止条件 3: 检测到强制邻居 → 此格元为跳转点
    if (hasForcedNeighbor(c, s, nx, ny, dx, dy)) {
      auto idx = static_cast<size_t>(ny * s.size_x + nx);
      if (s.node_position_[idx] != nullptr) {
        return s.node_position_[idx];
      }
      auto& ptr = s.nodes_.emplace_back(
          std::make_unique<SearchNode>(SearchNode{nx, ny}));
      s.node_position_[idx] = ptr.get();
      if (s.debug_.enabled) {
        s.debug_.jumppoint_x.push_back(nx);
        s.debug_.jumppoint_y.push_back(ny);
      }
      return ptr.get();
    }

    // 终止条件 4 (仅对角线): 分量方向存在跳转点 → 当前格元为跳转点
    if (dx != 0 && dy != 0) {
      double dummy_h{};
      double dummy_v{};
      if (jump(c, s, nx, ny, dx, 0, gx, gy, dummy_h) != nullptr
          || jump(c, s, nx, ny, 0, dy, gx, gy, dummy_v) != nullptr) {
        auto idx = static_cast<size_t>(ny * s.size_x + nx);
        if (s.node_position_[idx] != nullptr) {
          return s.node_position_[idx];
        }
        auto& ptr = s.nodes_.emplace_back(
            std::make_unique<SearchNode>(SearchNode{nx, ny}));
        s.node_position_[idx] = ptr.get();
        if (s.debug_.enabled) {
          s.debug_.jumppoint_x.push_back(nx);
          s.debug_.jumppoint_y.push_back(ny);
        }
        return ptr.get();
      }
    }

    // 不满足任何终止条件 → 继续沿方向跳跃
    return jump(c, s, nx, ny, dx, dy, gx, gy, acc);
  }

  // ══════════════════════════════════════════════════════════════════════════════
  // 后继节点识别 (Successor Identification)
  //
  // 对当前节点, 找出所有可能的跳转点后继:
  //   - 起点 (无父节点): 探索全部 8 个方向
  //   - 有父节点: 使用 pruneNeighbors 裁剪后的方向集合
  //
  // 每个后继附带从当前节点到该后继的跳跃累计代价 acc_cost。
  // ══════════════════════════════════════════════════════════════════════════════

  void JPSAlgorithm::identifySuccessors(
      const JPSConfig& c, JPSState& s, const SearchNode* current, int gx,
      int gy, std::vector<std::pair<SearchNode*, double>>& successors) {
    successors.clear();

    int cx = current->x;
    int cy = current->y;
    std::vector<std::pair<int, int>> directions{};

    if (current->parent == nullptr) {
      // 起点: 探索全部 8 个方向
      static constexpr int all_dirs[8][2] = {
          { 1,  0},
          {-1,  0},
          { 0,  1},
          { 0, -1},
          { 1,  1},
          {-1, -1},
          { 1, -1},
          {-1,  1}
      };
      for (auto [dx, dy] : all_dirs) {
        double acc_cost{0.0};
        auto* succ = jump(c, s, cx, cy, dx, dy, gx, gy, acc_cost);
        if (succ != nullptr) {
          successors.emplace_back(succ, acc_cost);
        }
      }
      return;
    }

    // 有父节点: 计算归一化父方向, 应用邻居裁剪规则
    int pdx = cx - current->parent->x;
    int pdy = cy - current->parent->y;
    if (pdx != 0) {
      pdx = pdx > 0 ? 1 : -1;
    }
    if (pdy != 0) {
      pdy = pdy > 0 ? 1 : -1;
    }

    pruneNeighbors(c, s, cx, cy, pdx, pdy, directions);

    for (auto [dx, dy] : directions) {
      double acc_cost{0.0};
      auto* succ = jump(c, s, cx, cy, dx, dy, gx, gy, acc_cost);
      if (succ != nullptr) {
        successors.emplace_back(succ, acc_cost);
      }
    }
  }

  /** @brief 从终点沿父指针回溯到起点, 构建路径 (格元中心坐标)。 */
  void JPSAlgorithm::backtracePath(const SearchNode* goal,
                     std::vector<std::pair<double, double>>& path) {
    path.clear();
    const SearchNode* n = goal;
    while (n != nullptr) {
      // 输出格元中心: (x + 0.5, y + 0.5)
      path.emplace_back(static_cast<double>(n->x) + 0.5,
                        static_cast<double>(n->y) + 0.5);
      n = n->parent;
    }
    // 反转: 从起点到终点
    std::reverse(path.begin(), path.end());
  }
}