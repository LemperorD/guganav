#include "jps_planner/jps_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace jps_planner {

  // ══════════════════════════════════════════════════════════════════════════════
  // 公开 API — generatePath
  //
  // 主 A* + JPS 搜索循环:
  //   1. 验证起终点合法
  //   2. 重置 state (nodes_, open_list_, node_position_)
  //   3. 创建起点, 初始化 f = g + h = 0 + heuristic(start, goal)
  //   4. 主循环: 从 open_list_ 弹出最优 f 值节点
  //      - 到达终点 → backtracePath 构建路径
  //      - 展开节点 → identifySuccessors 查找跳转点后继
  //      - 对每个后继: 计算 tentative_g, 更新最优路径
  //   5. 若 open_list_ 耗尽 → 无路径 (返回 false)
  // ══════════════════════════════════════════════════════════════════════════════
  bool JPSAlgorithm::generatePath(
      const JPSConfig& c, JPSState& s, int sx, int sy, int gx, int gy,
      std::vector<std::pair<double, double>>& path) {
    path.clear();
    JPSAlgorithm planner;
    // Step 1: 边界和可通行性验证
    if (!withinLimits(s, sx, sy) || !withinLimits(s, gx, gy)) {
      return false;
    }
    if (!isTraversable(c, s, sx, sy) || !isTraversable(c, s, gx, gy)) {
      return false;
    }

    // Step 2: 平凡情况 (起点即终点)
    if (sx == gx && sy == gy) {
      path.emplace_back(static_cast<double>(sx) + 0.5,
                        static_cast<double>(sy) + 0.5);
      return true;
    }

    // Step 3: 初始化搜索状态
    s.nodes_.clear();
    s.open_list_ = {};
    s.node_position_.assign(static_cast<size_t>(s.size_x * s.size_y), nullptr);

    const size_t total_cells = static_cast<size_t>(s.size_x * s.size_y);
    s.nodes_.reserve(total_cells / 4);  // 经验: 约 25% 格元会被访问

    // Step 4: 创建起始节点
    // start.g = 0, start.h = heuristic(start, goal), start.f = start.h
    auto& start_ptr = s.nodes_.emplace_back(
        std::make_unique<SearchNode>(SearchNode{sx, sy}));
    SearchNode* start = start_ptr.get();
    start->g = 0.0;
    start->h = planner.heuristic(c, sx, sy, gx, gy);
    start->f = start->h;
    s.node_position_[static_cast<size_t>(sy * s.size_x + sx)] = start;
    s.open_list_.push(start);

    std::vector<std::pair<SearchNode*, double>> successors{};
    successors.reserve(8);  // 最多 8 个后继 (8 连通)

    // Step 5: 主 A* + JPS 循环
    while (!s.open_list_.empty()) {
      SearchNode* current = s.open_list_.top();
      s.open_list_.pop();

      if (current->closed) {
        continue;
      }
      current->closed = true;

      // 终点检查
      if (current->x == gx && current->y == gy) {
        planner.backtracePath(current, path);
        return true;
      }

      // 找出所有跳转点后继
      planner.identifySuccessors(c, s, current, gx, gy, successors);

      // 记录展开节点用于调试
      if (s.debug_.enabled) {
        s.debug_.expanded_x.push_back(current->x);
        s.debug_.expanded_y.push_back(current->y);
      }

      // 对每个后继节点尝试更新 g 值
      // tentative_g = current.g + jump_cost + euclideanCost(current, succ)
      for (auto [succ, jump_cost] : successors) {
        if (succ->closed) {
          continue;
        }

        double tentative_g = current->g + jump_cost
                             + planner.euclideanCost(c, current->x, current->y,
                                                     succ->x, succ->y);

        if (tentative_g < succ->g) {
          succ->g = tentative_g;
          succ->h = planner.heuristic(c, succ->x, succ->y, gx, gy);
          succ->f = succ->g + succ->h;
          succ->parent = current;
          s.open_list_.push(succ);
        }
      }
    }

    // 开放列表耗尽 — 无路径
    return false;
  }

  // ══════════════════════════════════════════════════════════════════════════════
  // detourCornerHuggingDiagonals — 路径层去对角贴障碍
  //
  // JPS 对角规则允许单侧贴障碍 (canStep: 两个相邻格之一可通行即可),
  // 这类对角段经过 B-spline 平滑时会在转角内侧切角, 产生锯齿并触发
  // isPathCollisionFree 回退。此处不修改搜索 (搜索的裁剪/强制邻居规则
  // 与该对角行为配套), 而是在路径层把贴障碍的对角段改写为正交移动。
  // ══════════════════════════════════════════════════════════════════════════════

  std::vector<std::pair<double, double>>
  JPSAlgorithm::detourCornerHuggingDiagonals(
      const std::vector<std::pair<double, double>>& path,
      const unsigned char* costmap_data, int cm_w, int cm_h,
      bool allow_unknown) {
    if (path.size() < 2 || costmap_data == nullptr) {
      return path;
    }

    auto blocked = [&](double cx, double cy) {
      const int ix = static_cast<int>(cx);
      const int iy = static_cast<int>(cy);
      if (ix < 0 || ix >= cm_w || iy < 0 || iy >= cm_h) {
        return true;
      }
      const unsigned char cost =
          costmap_data[static_cast<size_t>(iy * cm_w + ix)];
      if (cost == UNKNOWN_COST) {
        return !allow_unknown;
      }
      return cost >= INSCRIBED_COST;
    };

    std::vector<std::pair<double, double>> out{};
    out.reserve(path.size() * 2);
    out.push_back(path.front());

    for (size_t i = 1; i < path.size(); ++i) {
      const double x0 = path[i - 1].first;
      const double y0 = path[i - 1].second;
      const double x1 = path[i].first;
      const double y1 = path[i].second;
      const int adx = static_cast<int>(std::lround(x1 - x0));
      const int ady = static_cast<int>(std::lround(y1 - y0));

      if (adx == 0 || ady == 0) {
        out.push_back(path[i]);
        continue;
      }

      const int steps = std::max(std::abs(adx), std::abs(ady));
      const double sx = (adx > 0) ? 1.0 : -1.0;
      const double sy = (ady > 0) ? 1.0 : -1.0;
      double cx = x0;
      double cy = y0;

      for (int s = 0; s < steps; ++s) {
        const double hx = cx + sx;  // 水平相邻格 (x+dx, y)
        const double hy = cy;
        const double vx = cx;  // 垂直相邻格 (x, y+dy)
        const double vy = cy + sy;
        const bool h_free = !blocked(hx, hy);
        const bool v_free = !blocked(vx, vy);

        if (h_free != v_free) {
          // 恰好一侧被阻塞: 先走空闲侧正交格, 再水平/垂直到对角格
          if (h_free) {
            out.emplace_back(hx, hy);
          } else {
            out.emplace_back(vx, vy);
          }
        }

        cx += sx;
        cy += sy;
        out.emplace_back(cx, cy);
      }
    }
    return out;

  }  // namespace jps_planner
}  // namespace jps_planner