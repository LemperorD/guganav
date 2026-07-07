#pragma once

#include <cmath>
#include <cstddef>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

namespace jps_planner
{

constexpr double INF_COST = 1e308;

/** @brief 网格坐标 (格元索引)。 */
struct Node
{
  int x{};
  int y{};
};

/**
 * @brief JPS 搜索中使用的搜索节点。
 * 通过 unique_ptr 在堆上分配, 保证指针在 vector 扩容时不失效。
 */
struct SearchNode
{
  int x{};
  int y{};
  double g{INF_COST};
  double h{INF_COST};
  double f{INF_COST};
  const SearchNode * parent{nullptr};
  bool closed{false};
};

/** @brief JPS 算法的不可变配置。 */
struct JPSConfig
{
  double w_euc_cost{1.0};
  double w_traversal_cost{10.0};
  double w_heuristic_cost{1.0};
  bool allow_unknown{false};
};

/** @brief JPS 搜索过程中收集的调试数据 (debug_enabled=true 时启用)。 */
struct JPSDebug
{
  bool enabled{false};
  std::vector<int> expanded_x{};   // 被展开的格子 x 坐标
  std::vector<int> expanded_y{};   // 被展开的格子 y 坐标
  std::vector<int> jumppoint_x{};  // 发现的所有跳转点 x 坐标
  std::vector<int> jumppoint_y{};  // 发现的所有跳转点 y 坐标
};

/** @brief 每次规划请求重新创建的可变状态。 */
struct JPSState
{
  const unsigned char * costmap_data{nullptr};
  int size_x{};
  int size_y{};

  /** 节点存储: 在堆上分配, 保证指针不变性。 */
  std::vector<std::unique_ptr<SearchNode>> nodes_{};

  /**
   * @brief 从网格坐标到 SearchNode* 的 O(1) 映射。
   * 索引方式: [size_x_ * y + x], nullptr = 未访问。
   */
  std::vector<SearchNode *> node_position_{};

  /** A* 优先队列 (开放列表)。 */
  struct Comp
  {
    bool operator()(const SearchNode * a, const SearchNode * b) const
    {
      return a->f > b->f;
    }
  };
  std::priority_queue<SearchNode *, std::vector<SearchNode *>, Comp> open_list_{};

  JPSDebug debug_{};
};

/**
 * @class JPSAlgorithm
 * @brief 纯静态方法, 在代价地图网格上执行 Jump Point Search。
 *
 * 遵循模式 A (函数式数据流): 无状态, 参数通过 (const Config&, State&) 显式传递。
 */
class JPSAlgorithm
{
public:
  /**
   * @brief 在 state 描述的代价地图上从 (sx, sy) 到 (gx, gy) 执行 JPS 搜索。
   * @param config  不可变算法参数。
   * @param state   可变工作状态 (必须预先填充 costmap_data/size)。
   * @param sx, sy  起点格元坐标。
   * @param gx, gy  终点格元坐标。
   * @param path    输出路径 (地图坐标, 格元中心), 会被先清空。
   * @return 找到路径返回 true, 否则返回 false。
   */
  [[nodiscard]] static bool generatePath(
    const JPSConfig & config, JPSState & state, int sx, int sy, int gx, int gy,
    std::vector<std::pair<double, double>> & path);

  /**
   * @brief 检查格元 (x, y) 是否可通行。
   * @param config  算法参数 (控制未知空间策略)。
   * @param state   包含代价地图指针的状态。
   * @param x, y    格元坐标。
   * @return 可通行返回 true。
   */
  [[nodiscard]] static bool isTraversable(
    const JPSConfig & config, const JPSState & state, int x, int y);
};

}  // namespace jps_planner
