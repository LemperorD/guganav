#pragma once

#include <cmath>
#include <cstddef>
#include <memory>
#include <queue>
#include <utility>
#include <vector>
#include <limits>
namespace jps_planner{
    
  /// 无穷大代价 (初始值, 用于 A* 的 g/h/f)
  constexpr double INF_COST = std::numeric_limits<double>::infinity();

  // ────────────────────────────────────────────────────────────
  // 数据结构
  // ────────────────────────────────────────────────────────────

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
    bool closed{false};                 // 是否已从开放列表中删去
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
}