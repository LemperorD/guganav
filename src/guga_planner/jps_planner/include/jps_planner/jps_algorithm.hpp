#pragma once

#include <cmath>
#include <cstddef>
#include <memory>
#include <queue>
#include <utility>
#include <vector>
#include <limits>
#include "jps_planner/jps_node.hpp"
namespace jps_planner {
  // ────────────────────────────────────────────────────────────
  // JPS 算法类 (纯静态方法, 遵循模式 A 函数式数据流)
  // ────────────────────────────────────────────────────────────

  /**
   * @class JPSAlgorithm
   * @brief 纯静态方法, 在代价地图网格上执行 Jump Point Search。
   *
   * 遵循模式 A (函数式数据流):
   *   无状态 — 所有算法内状态通过 State 参数显式传递。
   *   参数 — (const Config&, State&) 的不可变 + 可变模式。
   *   两层抽象 — Config+State struct → Algorithm static func。
   */
  class JPSAlgorithm {
  public:
    /**
     * @brief 在 state 描述的代价地图上, 从 (sx, sy) 到 (gx, gy) 执行 JPS 搜索。
     *
     * @param config  不可变算法参数 (权重, 未知空间策略)。
     * @param state   可变工作状态 (必须预填充 costmap_data, size_x, size_y)。
     * @param sx, sy  起点格元坐标 (整数索引)。
     * @param gx, gy  终点格元坐标 (整数索引)。
     * @param path    输出路径: 地图坐标 (格元中心), 调用前被清空。
     * @return        找到路径返回 true; 终点不可达或无路径返回 false。
     */
    [[nodiscard]] static bool generatePath(
        const JPSConfig& config, JPSState& state, int sx, int sy, int gx,
        int gy, std::vector<std::pair<double, double>>& path);

    /**
     * @brief 检查格元 (x, y) 是否可通行。
     *
     * 可通行条件: 在边界内, 且代价值 < INSCRIBED_COST (253)。
     * allow_unknown=true 时, 未知空间 (255) 视为可通行。
     *
     * @param config  算法参数 (控制未知空间策略)。
     * @param state   包含代价地图指针的状态。
     * @param x, y    格元坐标。
     * @return        可通行返回 true。
     */
    [[nodiscard]] static bool isTraversable(const JPSConfig& config,
                                            const JPSState& state, int x,
                                            int y);

  private:
    // ── Costmap 常量 ──
    static constexpr unsigned char UNKNOWN_COST = 255;  // 未知空间
    static constexpr unsigned char INSCRIBED_COST = 253;  // 膨胀后的内切障碍物
    static constexpr unsigned char MAX_NON_OBSTACLE =
        252;  // 最高非障碍物代价值

    static unsigned char getCost(const JPSState& s, int x, int y) {
      if (x < 0 || x >= s.size_x || y < 0 || y >= s.size_y) {
        return UNKNOWN_COST;
      }
      return s.costmap_data[static_cast<size_t>((y * s.size_x) + x)];
    }
    static bool isObstacle(const JPSConfig& c, const JPSState& s, int x, int y);
    bool canStep(const JPSConfig& c, const JPSState& s, int x, int y, int dx,
                 int dy);
    static bool withinLimits(const JPSState& s, int x, int y) {
      return x >= 0 && x < s.size_x && y >= 0 && y < s.size_y;
    };
    static bool isTraversableCell(const JPSConfig& c, const JPSState& s, int x,
                                  int y) {
      return withinLimits(s, x, y) && !isObstacle(c, s, x, y);
    };

    double traversalCost(const JPSConfig& c, unsigned char raw);

    bool isBlockedCell(const JPSConfig& c, const JPSState& s, int x, int y);

    double scaledCost(unsigned char raw);

    double heuristic(const JPSConfig& c, int x1, int y1, int x2, int y2);

    double euclideanCost(const JPSConfig& c, int ax, int ay, int bx, int by);

    bool hasForcedNeighborHoriz(const JPSConfig& c, const JPSState& s, int x,
                                int y, int dx);
    bool hasForcedNeighborVert(const JPSConfig& c, const JPSState& s, int x,
                               int y, int dy);
    bool hasForcedNeighborDiag(const JPSConfig& c, const JPSState& s, int x,
                               int y, int dx, int dy);
    bool hasForcedNeighbor(const JPSConfig& c, const JPSState& s, int x, int y,
                           int dx, int dy);
    void pruneNeighbors(const JPSConfig& c, const JPSState& s, int x, int y,
                        int dx, int dy,
                        std::vector<std::pair<int, int>>& directions);

    SearchNode* jump(const JPSConfig& c, JPSState& s, int x, int y, int dx,
                     int dy, int gx, int gy, double& acc);
    void identifySuccessors(
        const JPSConfig& c, JPSState& s, const SearchNode* current, int gx,
        int gy, std::vector<std::pair<SearchNode*, double>>& successors);
    void backtracePath(const SearchNode* goal,
                       std::vector<std::pair<double, double>>& path);

  public:
    // ────────────────────────────────────────────────────────────
    // 路径后处理
    // ────────────────────────────────────────────────────────────

    /**
     * @brief 把"贴障碍的对角段"改写为两段正交移动 (水平/垂直各一格)。
     *
     * JPS 对角规则允许单侧贴障碍 (两个相邻格之一可通行即可), 这类对角段
     * 经过 B-spline 平滑时会在转角内侧切角, 产生锯齿并触发下游碰撞检查回退。
     * 本函数在路径层处理:
     *   对角单位步若恰好一侧相邻格被阻塞 → 改走空闲侧正交格再到对角格
     *   (纯水平+垂直, 不产生贴障碍对角段);
     *   两侧都空闲 → 保持对角; 两侧都阻塞 (对角夹缝) → 保持对角。
     *
     * @param path          地图坐标路径 (格元中心)。
     * @param costmap_data  代价地图原始数据 (非拥有, 可为 nullptr)。
     * @param cm_w, cm_h    代价地图尺寸 (格元)。
     * @param allow_unknown 未知空间 (255) 是否视为可通行。
     * @return 处理后的路径。
     */
    std::vector<std::pair<double, double>> static detourCornerHuggingDiagonals(
        const std::vector<std::pair<double, double>>& path,
        const unsigned char* costmap_data, int cm_w, int cm_h,
        bool allow_unknown);
  };
}  // namespace jps_planner
