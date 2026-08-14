#include "jps_planner/jps_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace jps_planner {

  // ══════════════════════════════════════════════════════════════════════════════
  // 匿名 namespace — 辅助自由函数 (模式 A: 函数式数据流)
  //
  // 所有 JPS 核心算法都在匿名 namespace 中作为纯函数实现,
  // 参数通过 (const Config&, State&) 显式传递, 无全局状态。
  // ══════════════════════════════════════════════════════════════════════════════
  namespace {

    // ── Costmap 常量 ──
    constexpr unsigned char UNKNOWN_COST = 255;    // 未知空间
    constexpr unsigned char INSCRIBED_COST = 253;  // 膨胀后的内切障碍物
    constexpr unsigned char MAX_NON_OBSTACLE = 252;  // 最高非障碍物代价值

    // ── 代价地图查询 ──

    /** @brief 带边界检查的代价地图原始查询。越界返回 UNKNOWN_COST。 */
    [[nodiscard]] inline unsigned char getCost(const JPSState& s, int x,
                                               int y) {
      if (x < 0 || x >= s.size_x || y < 0 || y >= s.size_y) {
        return UNKNOWN_COST;
      }
      return s.costmap_data[static_cast<size_t>((y * s.size_x) + x)];
    }

    /** @brief 判断格元是否被阻塞 (cost ≥ 253 = 障碍物或膨胀区域)。
     *  allow_unknown=true 时, 未知空间 (255) 视为可通行。 */
    [[nodiscard]] inline bool isObstacle(const JPSConfig& c, const JPSState& s,
                                         int x, int y) {
      auto cost = getCost(s, x, y);
      if (cost == UNKNOWN_COST && c.allow_unknown) {
        return false;
      }
      return cost >= INSCRIBED_COST;
    }

    /** @brief 判断坐标是否在网格边界内 (含边界)。 */
    [[nodiscard]] inline bool withinLimits(const JPSState& s, int x, int y) {
      return x >= 0 && x < s.size_x && y >= 0 && y < s.size_y;
    }

    /** @brief 判断格元是否可作为移动落点。越界始终不可通行。 */
    [[nodiscard]] inline bool isTraversableCell(const JPSConfig& c,
                                                const JPSState& s, int x,
                                                int y) {
      return withinLimits(s, x, y) && !isObstacle(c, s, x, y);
    }

    /** @brief 判断格元是否阻断移动。越界视为阻断, 不受 allow_unknown 影响。 */
    [[nodiscard]] inline bool isBlockedCell(const JPSConfig& c,
                                            const JPSState& s, int x, int y) {
      return !isTraversableCell(c, s, x, y);
    }

    /** @brief 判断从 (x,y) 沿 (dx,dy) 前进一步是否合法。 */
    [[nodiscard]] inline bool canStep(const JPSConfig& c, const JPSState& s,
                                      int x, int y, int dx, int dy) {
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
    [[nodiscard]] inline double scaledCost(unsigned char raw) {
      double sc = 26.0 + 0.9 * static_cast<double>(raw);
      return sc * sc / (MAX_NON_OBSTACLE * MAX_NON_OBSTACLE);
    }

    /** @brief 单个格元的加权通行代价: t(c) = w_t · s(c)。
     *  以跳转点为单位的跳跃路径累计此代价。 */
    [[nodiscard]] inline double traversalCost(const JPSConfig& c,
                                              unsigned char raw) {
      return c.w_traversal_cost * scaledCost(raw);
    }

    /** @brief 加权欧几里得距离启发函数: h = w_h · √((Δx)² + (Δy)²)。
     *  使用欧几里得距离 (而非曼哈顿), 在 8 连通网格上保证 admissible。 */
    [[nodiscard]] inline double heuristic(const JPSConfig& c, int x1, int y1,
                                          int x2, int y2) {
      return c.w_heuristic_cost
             * std::hypot(static_cast<double>(x2 - x1),
                          static_cast<double>(y2 - y1));
    }

    /** @brief 两格元间的加权欧几里得距离: d = w_e · √((Δx)² + (Δy)²)。
     *  在 g 值更新时作为跳转点间的候选距离代价。 */
    [[nodiscard]] inline double euclideanCost(const JPSConfig& c, int ax,
                                              int ay, int bx, int by) {
      return c.w_euc_cost
             * std::hypot(static_cast<double>(ax - bx),
                          static_cast<double>(ay - by));
    }

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

    /**
     * @brief 检测水平直行方向 (dx = ±1, dy = 0) 的强制邻居。
     * 当跳跃方向旁边的格元被阻塞、但其对角线远处格元空闲时触发。 */
    [[nodiscard]] bool hasForcedNeighborHoriz(const JPSConfig& c,
                                              const JPSState& s, int x, int y,
                                              int dx) {
      return (isBlockedCell(c, s, x, y + 1)
              && isTraversableCell(c, s, x + dx, y + 1))
             || (isBlockedCell(c, s, x, y - 1)
                 && isTraversableCell(c, s, x + dx, y - 1));
    }

    /** @brief 检测垂直直行方向 (dx = 0, dy = ±1) 的强制邻居。 */
    [[nodiscard]] bool hasForcedNeighborVert(const JPSConfig& c,
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
    [[nodiscard]] bool hasForcedNeighborDiag(const JPSConfig& c,
                                             const JPSState& s, int x, int y,
                                             int dx, int dy) {
      return (isBlockedCell(c, s, x - dx, y)
              && isTraversableCell(c, s, x - dx, y + dy))
             || (isBlockedCell(c, s, x, y - dy)
                 && isTraversableCell(c, s, x + dx, y - dy));
    }

    /** @brief 统一的强制邻居检测, 根据方向分量分发到对应检测函数。 */
    [[nodiscard]] bool hasForcedNeighbor(const JPSConfig& c, const JPSState& s,
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

    void pruneNeighbors(const JPSConfig& c, const JPSState& s, int x, int y,
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

    SearchNode* jump(const JPSConfig& c, JPSState& s, int x, int y, int dx,
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

    void identifySuccessors(
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
    void backtracePath(const SearchNode* goal,
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

  }  // namespace

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

    // Step 3: 重置搜索状态
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
    start->h = heuristic(c, sx, sy, gx, gy);
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
        backtracePath(current, path);
        return true;
      }

      // 找出所有跳转点后继
      identifySuccessors(c, s, current, gx, gy, successors);

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
                             + euclideanCost(c, current->x, current->y, succ->x,
                                             succ->y);

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

  // ══════════════════════════════════════════════════════════════════════════════
  // isTraversable — 单格元可通行性检查
  // ══════════════════════════════════════════════════════════════════════════════

  bool JPSAlgorithm::isTraversable(const JPSConfig& c, const JPSState& s, int x,
                                   int y) {
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

  // ══════════════════════════════════════════════════════════════════════════════
  // detourCornerHuggingDiagonals — 路径层去对角贴障碍
  //
  // JPS 对角规则允许单侧贴障碍 (canStep: 两个相邻格之一可通行即可),
  // 这类对角段经过 B-spline 平滑时会在转角内侧切角, 产生锯齿并触发
  // isPathCollisionFree 回退。此处不修改搜索 (搜索的裁剪/强制邻居规则
  // 与该对角行为配套), 而是在路径层把贴障碍的对角段改写为正交移动。
  // ══════════════════════════════════════════════════════════════════════════════

  std::vector<std::pair<double, double>> detourCornerHuggingDiagonals(
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
        const double vx = cx;       // 垂直相邻格 (x, y+dy)
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
  }

}  // namespace jps_planner
