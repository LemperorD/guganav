#pragma once

#include <cmath>
#include <cstddef>
#include <memory>
#include <queue>
#include <utility>
#include <vector>
#include <limits>

namespace jps_planner {

  /// 无穷大代价 (初始值, 用于 A* 的 g/h/f)
  constexpr double INF_COST = std::numeric_limits<double>::infinity();

  // ────────────────────────────────────────────────────────────
  // 数据结构
  // ────────────────────────────────────────────────────────────

  /** @brief 网格坐标 (格元索引, 整数)。 */
  struct Node {
    int x{};
    int y{};
  };

  /**
   * @brief JPS 搜索中使用的搜索节点。
   *
   * 通过 unique_ptr 在堆上分配 (存储在 JPSState::nodes_ 中),
   * 保证指针在 vector 扩容时不失效 — parent 指针永远有效。
   */
  struct SearchNode {
    int x{};
    int y{};
    double g{INF_COST};  // 从起点到此节点的实际代价
    double h{INF_COST};  // 从此节点到终点的启发式估计
    double f{INF_COST};  // f = g + h (A* 排序键)
    const SearchNode* parent{nullptr};  // 指向最优路径上的前驱节点
    bool closed{false};                 // 是否已从开放列表中关闭
  };

  /** @brief JPS 算法的不可变配置 (遵循模式 A: 公开字段)。 */
  struct JPSConfig {
    double w_euc_cost{1.0};         // 欧几里得距离代价权重
    double w_traversal_cost{10.0};  // 通行代价权重 (Theta* 缩放后)
    double w_heuristic_cost{1.0};  // 启发式代价权重 (控制贪心程度)
    bool allow_unknown{false};     // 是否允许穿越未知空间 (cost=255)
  };

  /** @brief JPS 搜索过程中收集的调试数据 (仅 debug_enabled=true 时启用)。 */
  struct JPSDebug {
    bool enabled{false};
    std::vector<int> expanded_x{};   // 搜索过程中展开的格子 x 坐标
    std::vector<int> expanded_y{};   // 搜索过程中展开的格子 y 坐标
    std::vector<int> jumppoint_x{};  // 发现的所有跳转点 x 坐标
    std::vector<int> jumppoint_y{};  // 发现的所有跳转点 y 坐标
  };

  /**
   * @brief 每次规划请求重新创建的可变状态 (遵循模式 A: 公开字段)。
   *
   * 包含代价地图指针、A* 展开所需的节点存储和优先队列。
   * 调用方负责在每次 generatePath() 调用前填充 costmap_data/size_x/size_y。
   */
  struct JPSState {
    const unsigned char* costmap_data{
        nullptr};  // Nav2 costmap 原始数据 (非拥有)
    int size_x{};  // 代价地图宽度 (格元数)
    int size_y{};  // 代价地图高度 (格元数)

    /** 节点存储池: 所有 SearchNode 在堆上分配 (unique_ptr),
     * 保证 vector 扩容时指针不失效。 */
    std::vector<std::unique_ptr<SearchNode>> nodes_{};

    /**
     * @brief 从网格坐标到 SearchNode* 的 O(1) 映射。
     *
     * 索引方式: node_position_[size_x * y + x] = SearchNode*,
     * nullptr 表示该格元尚未被访问。
     */
    std::vector<SearchNode*> node_position_{};

    /** A* 优先队列 (开放列表), 按 f 值升序排列。 */
    struct Comp {
      bool operator()(const SearchNode* a, const SearchNode* b) const {
        return a->f > b->f;  // std::priority_queue 默认是大顶堆, 反转实现小顶堆
      }
    };
    std::priority_queue<SearchNode*, std::vector<SearchNode*>, Comp>
        open_list_{};

    JPSDebug debug_{};  // 可选的调试数据收集
  };

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
  };

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
  [[nodiscard]] std::vector<std::pair<double, double>>
  detourCornerHuggingDiagonals(
      const std::vector<std::pair<double, double>>& path,
      const unsigned char* costmap_data, int cm_w, int cm_h,
      bool allow_unknown);

}  // namespace jps_planner
