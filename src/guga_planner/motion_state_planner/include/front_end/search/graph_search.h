/**
 * @file graph_search.h
 * @brief 图搜索后端 — A* 算法与 Jump Point Search (JPS) 跳点搜索的实现
 */

#ifndef JPS_GRAPH_SEARCH_H
#define JPS_GRAPH_SEARCH_H

#include <boost/heap/d_ary_heap.hpp>      // boost 二叉堆，支持 mutable 操作
#include <memory>                         // std::shared_ptr 智能指针
#include <limits>                         // std::numeric_limits 无穷大常量
#include <vector>                         // std::vector 动态数组
#include <unordered_map>                  // std::unordered_map 哈希表
#include <plan_env/sdf_map.h>             // 符号距离场地图

namespace JPS
{

/**
 * @struct compare_state
 * @brief 优先队列的比较函数对象
 *
 * A* 搜索使用 f = g + h 作为优先级键值，优先扩展 f 值最小的节点。
 * 当两个节点的 f 值在容差范围内相等时（浮点比较），优先比较 g 值:
 * g 值更大者更优先（即 tie-breaking 策略，倾向于先扩展已行进更远的节点）
 *
 * 注意: 该比较器返回 true 表示 a1 优先级低于 a2（即 a1 后弹出），
 * 因此 f1 > f2 时返回 true（更大的 f 后处理）。
 */
template <class T>
struct compare_state
{
  bool operator()(T a1, T a2) const
  {
    double f1 = a1->g + a1->h;  // 节点 a1 的总估计代价
    double f2 = a2->g + a2->h;  // 节点 a2 的总估计代价
    // 浮点 f 值在 1e-6 容差内视为相等
    if( ( f1 >= f2 - 0.000001) && (f1 <= f2 +0.000001) )
      return a1->g < a2->g;     // f 相等时，g 更小者优先级更高（打破平局）
    return f1 > f2;             // f 更大的在后（优先级更低）
  }
};


/// 定义优先队列类型
struct State; // 前向声明
/// State 的共享指针类型别名
using StatePtr = std::shared_ptr<State>;
/**
 * @brief 优先队列类型定义
 *
 * 使用 boost::d_ary_heap 实现二叉堆（arity=2），支持:
 * - mutable_: 允许在堆中更新已存在元素的键值（用于 decrease-key 操作）
 * - 当发现到某个节点的更优路径时，通过 heapkey 句柄直接更新其在堆中的位置
 * - 这避免了对已关闭节点的重新插入，是 A* 高效实现的关键
 */
using priorityQueue = boost::heap::d_ary_heap<StatePtr, boost::heap::mutable_<true>,
                      boost::heap::arity<2>, boost::heap::compare< compare_state<StatePtr> >>;

/**
 * @struct State
 * @brief 图搜索中的节点（栅格单元）
 *
 * 每个 State 对应栅格地图中的一个离散点，存储:
 * - 空间位置: 整数栅格坐标 (x, y, z)
 * - 运动方向: 单位方向向量 (dx, dy, dz)，用于 JPS 确定搜索方向
 * - A* 代价: g（已行进距离）和 h（到目标的启发式估计）
 * - 状态标记: opened/closed，用于跟踪 A* 搜索进度
 *
 * JPS 特有的设计:
 * - 方向向量 (dx,dy,dz) 不是简单的邻接关系，而是记录当前搜索的"动量方向"
 * - parentId 用于路径回溯，而非直接存储父节点指针（节省内存）
 */
struct State
{
  /// 一维哈希 ID（由 x + y*xDim 计算得出，用于 O(1) 查找）
  int id;
  /// 栅格坐标 (x, y, z)，z 默认为 0 以兼容 2D
  int x, y, z = 0;
  /// 当前搜索方向（归一化的单位方向分量，取值为 {-1, 0, 1}）
  int dx, dy, dz;                   // 该节点的离散坐标和方向
  /// 父节点 ID（用于在搜索完成后反向回溯重建路径）
  int parentId = -1;

  /// 指向优先队列堆中位置的句柄（支持 O(log n) 的 decrease-key 操作）
  priorityQueue::handle_type heapkey;

  /// g 代价: 从起点到当前节点的实际路径代价，初始化为无穷大
  double g = std::numeric_limits<double>::infinity();
  /// h 代价: 到目标节点的启发式估计（欧几里得距离 * 启发式权重 eps_）
  double h;
  /// 是否已被加入开放集（opened = true 表示已入队至少一次）
  bool opened = false;
  /// 是否已被加入关闭集（closed = true 表示已扩展完毕）
  bool closed = false;

  /// 2D 构造函数: 初始化坐标和方向（无 z 分量）
  State(int id, int x, int y, int dx, int dy )
    : id(id), x(x), y(y), dx(dx), dy(dy)
  {}

  /// 3D 构造函数: 初始化坐标和三维方向
  State(int id, int x, int y, int z, int dx, int dy, int dz )
    : id(id), x(x), y(y), z(z), dx(dx), dy(dy), dz(dz)
  {}

};

/**
 * @struct JPS2DNeib
 * @brief 二维 JPS 邻居查找表（Look-Up Table）
 *
 * 核心数据结构，预计算并存储每种搜索方向 (dx, dy) 对应的邻居信息。
 * 通过预计算避免在搜索过程中重复推断邻居关系，大幅提升效率。
 *
 * 数据结构解释:
 * - 第一维 [9] = 3x3 = 9 种方向组合 (dx, dy ∈ {-1, 0, 1})
 * - ns[dir][0..1][k]: 第 k 个自然邻居（始终需要检查的邻居）的 dx/dy
 * - f1[dir][0..1][k]: 第 k 个强制邻居检查点的 dx/dy（需要检查其是否被占用）
 * - f2[dir][0..1][k]: 如果 f1 被占用，需要跳向的邻居方向
 *
 * nsz[norm][0]: 该范数类型下的自然邻居数量
 * nsz[norm][1]: 该范数类型下的强制邻居数量
 *
 * 移动类型分类 (by L1范数 norm1 = |dx|+|dy|):
 *   norm1=0 (起点特例): 8个自然邻居，0个强制邻居
 *   norm1=1 (直线移动): 1个自然邻居（沿方向），2个强制邻居
 *   norm1=2 (对角线移动): 3个自然邻居（沿各分量+对角线），2个强制邻居
 */
struct JPS2DNeib {
  // 对每种 (dx,dy) 方向:
  //    ns: 始终需要添加的邻居（自然邻居）
  //    f1: 需要检查的强制邻居位置
  //    f2: 如果 f1 位置被占用则需要添加的邻居
  int ns[9][2][8];
  int f1[9][2][2];
  int f2[9][2][2];
  // nsz 存储不同类型移动的邻居数量:
  // 静止 (norm=0):              8个始终添加的邻居
  //                               0个强制邻居（永远不会出现）
  //                               0个被强制时添加的邻居（永远不会出现）
  // 直线 (norm=1):              1个始终添加的邻居
  //                               2个需要检查的强制邻居
  //                               2个被强制时添加的邻居
  // 对角线 (norm=sqrt(2)):      3个始终添加的邻居
  //                               2个需要检查的强制邻居
  //                               2个被强制时添加的邻居
  static constexpr int nsz[3][2] = {{8, 0}, {1, 2}, {3, 2}};

  void print();          // 调试输出函数
  JPS2DNeib();           // 构造函数: 预计算所有方向的邻居查找表
  private:
  /// 计算方向 (dx,dy) 下第 dev 个自然邻居的坐标 (tx, ty)
  void Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty);
  /// 计算方向 (dx,dy) 下第 dev 个强制邻居 (fx,fy) 及对应跳跃方向 (nx,ny)
  void FNeib(int dx, int dy, int norm1, int dev,
      int& fx, int& fy, int& nx, int& ny);
};


/**
 * @struct JPS3DNeib
 * @brief 三维 JPS 邻居查找表
 *
 * 与 JPS2DNeib 类似，但扩展到三维:
 * - 第一维 [27] = 3x3x3 = 27 种方向组合 (dx, dy, dz ∈ {-1, 0, 1})
 *
 * 移动类型分类 (by L1范数 norm1 = |dx|+|dy|+|dz|):
 *   norm1=0 (起点):         26个自然邻居，0个强制邻居
 *   norm1=1 (直线):          1个自然邻居，8个强制邻居
 *   norm1=2 (面对角线):      3个自然邻居，8个强制邻居，12个强制时添加
 *   norm1=3 (空间对角线):    7个自然邻居，6个强制邻居，12个强制时添加
 */
struct JPS3DNeib {
  // 对每种 (dx,dy,dz) 方向:
  //    ns: 始终需要添加的邻居（自然邻居）
  //    f1: 需要检查的强制邻居位置
  //    f2: 如果 f1 位置被占用则需要添加的邻居
  int ns[27][3][26];
  int f1[27][3][12];
  int f2[27][3][12];
  // nsz 存储不同类型移动的邻居数量:
  // 静止 (norm=0):              26个始终添加的邻居
  //                               0个强制邻居（永远不会出现）
  //                               0个被强制时添加的邻居（永远不会出现）
  // 直线 (norm=1):               1个始终添加的邻居
  //                               8个需要检查的强制邻居
  //                               8个被强制时添加的邻居
  // 对角线 (norm=sqrt(2)):       3个始终添加的邻居
  //                               8个需要检查的强制邻居
  //                              12个被强制时添加的邻居
  // 对角线 (norm=sqrt(3)):       7个始终添加的邻居
  //                               6个需要检查的强制邻居
  //                              12个被强制时添加的邻居
  static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
  JPS3DNeib();           // 构造函数: 预计算所有方向的邻居查找表
  private:
  /// 计算方向 (dx,dy,dz) 下第 dev 个自然邻居的坐标 (tx, ty, tz)
  void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
  /// 计算方向 (dx,dy,dz) 下第 dev 个强制邻居及对应跳跃方向
  void FNeib( int dx, int dy, int dz, int norm1, int dev,
      int& fx, int& fy, int& fz,
      int& nx, int& ny, int& nz);
};

/**
 * @class GraphSearch
 * @brief 图搜索算法的核心类，实现 A* 和 JPS（Jump Point Search）算法
 *
 * 设计意图:
 * - 提供统一的搜索接口，通过 useJps 标志在 A* 和 JPS 之间切换
 * - 对上层屏蔽搜索细节，返回路径点序列
 * - 同时维护 2D 和 3D 搜索能力（当前 3D 为注释掉的状态）
 *
 * 算法流程:
 * 1. plan() 初始化起止点和启发式函数
 * 2. 主循环中反复从优先队列取出 f 值最小的节点进行扩展
 * 3. A* 模式: getSucc() 扩展所有8邻域
 * 4. JPS 模式: getJpsSucc() 根据方向查找表跳向跳点
 * 5. 到达目标后通过 recoverPath() 反向回溯构建路径
 *
 * 关键设计决策:
 * - 使用 SDFmap 而非原始栅格地图，通过 SDF 距离进行碰撞检测
 * - safe_dis_ 参数提供可配置的安全距离膨胀
 * - head map（hm_）同时充当 closed 集合和父节点索引（减少额外数据结构）
 */
class GraphSearch
{
  public:
  // 原始栅格地图版本的构造函数（已废弃，改用 SDFmap 版本）
  //   GraphSearch(const char* cMap, int xDim, int yDim, double eps = 1, bool verbose = false);
  //   GraphSearch(const char* cMap, int xDim, int yDim, int zDim, double eps = 1, bool verbose = false);

    /**
     * @brief 图搜索构造函数（使用 SDF 地图）
     * @param Map 共享的 SDF 地图指针（SDF = Signed Distance Field，符号距离场）
     * @param safe_dis 安全距离：在此距离内有障碍物则视为不可通行
     */
    GraphSearch(std::shared_ptr<SDFmap> Map, const double &safe_dis);

    /**
     * @brief 启动 2D 路径规划
     *
     * @param xStart 起点 x 坐标（栅格索引）
     * @param yStart 起点 y 坐标（栅格索引）
     * @param xGoal  目标 x 坐标（栅格索引）
     * @param yGoal  目标 y 坐标（栅格索引）
     * @param useJps true 时使用 JPS 跳点搜索，false 时使用标准 A*
     * @param maxExpand 最大扩展节点数，默认 -1 表示无限制
     * @return true 找到路径，false 搜索失败
     */
    bool plan(int xStart, int yStart, int xGoal, int yGoal, bool useJps, int maxExpand = -1);
    /**
     * @brief 启动 3D 路径规划（当前未启用）
     */
    bool plan(int xStart, int yStart, int zStart, int xGoal, int yGoal, int zGoal, bool useJps, int maxExpand = -1);

    /// 获取最优路径（从起点到目标的有序节点序列）
    std::vector<StatePtr> getPath() const;

    /// 获取开放集（opened=true, closed=false，尚未扩展的边界节点）
    std::vector<StatePtr> getOpenSet() const;

    /// 获取关闭集（closed=true，已扩展完毕的节点）
    std::vector<StatePtr> getCloseSet() const;

    /// 获取哈希表中所有已发现的节点
    std::vector<StatePtr> getAllSet() const;

    /// 设置/获取安全距离（可动态调整碰撞检测阈值）
    void SetSafeDis(const double &safe_dis);
    double GetSafeDis();

  private:
    /**
     * @brief 主规划循环
     *
     * 实现 A*/JPS 的标准主循环:
      * while(open集非空) {
      *   取出f值最小的节点 -> 标记为closed
      *   如果是目标 -> 成功退出
      *   扩展后继节点 -> 更新g值和父节点
      * }
      *
      * @param currNode_ptr 起始节点（入参/出参，循环结束后指向目标节点）
      * @param max_expand 最大扩展次数限制
      * @param start_id 起点的哈希ID
      * @param goal_id 目标的哈希ID
      */
    bool plan(StatePtr& currNode_ptr, int max_expand, int start_id, int goal_id);
    /// A* 后继生成: 遍历8邻域，过滤不可通行区域
    void getSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
    /// JPS 后继生成: 按方向查找表进行跳点搜索，跳过冗余节点
    void getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
    /// 从目标节点回溯到起点，重建完整路径
    std::vector<StatePtr> recoverPath(StatePtr node, int id);

    /// 将 (x, y) 栅格坐标转换为一维哈希ID
    int coordToId(int x, int y) const;

    /// 检查 (x, y) 是否可通行（在地图范围内且 SDF 距离 >= safe_dis_）
    bool isFree(int x, int y) const;

    /// 检查 (x, y) 是否未被占用（用于 JPS 中检查更宽松的条件）
    bool isUnoccupied(int x, int y) const;

    /// 检查 (x, y) 是否被障碍物占用（边界外也视为被占用）
    bool isOccupied(int x, int y) const;

    /// 计算启发式代价 h = eps_ * sqrt((x-xgoal)^2 + (y-ygoal)^2)
    /// eps_ > 1 时变为加权A*（有界次优，搜索更快但路径可能非最优）
    double getHeur(int x, int y) const;

    /**
     * @brief 检查 (x,y) 沿 (dx,dy) 方向是否存在强制邻居
     *
     * 强制邻居的定义（JPS算法的核心概念）:
     * 对于直线移动方向 (dx,dy)，如果侧方的某个栅格被障碍物占用，
     * 且从父节点的更短路径无法到达该侧方附近的空白栅格，
     * 则该空白栅格就是一个"跳点"（因为经过它才能到达被阻塞的区域）
     *
     * @return true 如果存在强制邻居（即当前点是一个跳点）
     */
    bool hasForced(int x, int y, int dx, int dy);

    /**
     * @brief JPS 核心: 沿 (dx,dy) 方向递归跳跃
     *
     * 跳点搜索的递归规则:
     * 1. 前进一格: new = current + (dx,dy)
     * 2. 如果 new 不可通行 -> 返回 false（跳跃失败）
     * 3. 如果 new 是目标 -> 返回 true（找到目标）
     * 4. 如果 new 有强制邻居 -> 返回 true（new 是一个跳点）
     * 5. 对角线移动时: 递归检查两个轴向分量 -> 如果任一找到跳点则 new 是跳点
     * 6. 继续沿原方向递归跳跃
     *
     * @param[out] new_x, new_y 输出找到的跳点坐标
     * @return true 如果找到目标或跳点
     */
    bool jump(int x, int y, int dx, int dy, int& new_x, int& new_y);

    /// 初始化 2D JPS 邻居查找表（当前在构造时通过 JPS2DNeib 构造函数完成）
    void init2DJps();

    // --- 数据成员 ---
    std::shared_ptr<SDFmap> map_;        // SDF 地图（符号距离场）
    int xDim_, yDim_, zDim_;             // 地图尺寸（栅格数）
    double eps_;                         // 启发式权重（1.0=最优A*，>1=加权A*）
    bool verbose_;                       // 详细输出开关

    double safe_dis_;                    // 安全距离阈值（碰撞检测时膨胀障碍物）

    const char val_free_ = 0;            // 空闲栅格值（用于原始栅格地图，当前废弃）
    int xGoal_, yGoal_, zGoal_;          // 目标点坐标
    bool use_2d_;                        // 2D/3D 模式标志
    bool use_jps_ = false;               // A*/JPS 模式标志

    priorityQueue pq_;                   // 优先队列（开放集），按 f=g+h 排序
    std::vector<StatePtr> hm_;           // 哈希映射（大小 = xDim*yDim），id->StatePtr
    std::vector<bool> seen_;             // 标记向量，记录某ID是否已被访问过

    std::vector<StatePtr> path_;         // 最终路径（从起点到目标）

    std::vector<std::vector<int>> ns_;   // A*的邻居偏移量（8邻域）
    std::shared_ptr<JPS2DNeib> jn2d_;    // 2D JPS 邻居查找表
    std::shared_ptr<JPS3DNeib> jn3d_;    // 3D JPS 邻居查找表
};

} // namespace JPS

#endif // JPS_GRAPH_SEARCH_H
