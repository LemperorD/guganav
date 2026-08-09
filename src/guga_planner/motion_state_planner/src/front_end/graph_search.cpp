/**
 * @file graph_search.cpp
 * @brief A* 和 JPS (Jump Point Search) 跳点搜索算法的实现
 */

#include <front_end/search/graph_search.h>
#include <cmath>

using namespace JPS;

GraphSearch::GraphSearch(std::shared_ptr<EsdfMap> Map, const double &safe_dis):map_(Map), safe_dis_(safe_dis)
{
  verbose_ = false;                               // 默认不输出调试信息
  xDim_ = map_->sizeX();                        // 地图 X 方向栅格数
  yDim_ = map_->sizeY();                        // 地图 Y 方向栅格数
  eps_ = 1;                                       // 启发式权重 = 1.0 (标准A*)

  hm_.resize(xDim_ * yDim_);                      // 哈希映射: id -> StatePtr
  seen_.resize(xDim_ * yDim_, false);             // 访问标记: 标记某 id 是否已被生成

  // 初始化 A* 的 8 邻域偏移量（Moore邻域，不包括自身）
  for(int x = -1; x <= 1; x ++) {
    for(int y = -1; y <= 1; y ++) {
      if(x == 0 && y == 0) continue;              // 跳过自身
      ns_.push_back(std::vector<int>{x, y});
    }
  }

  jps_neib_2d_ = std::make_shared<JPS2DNeib>();          // 预计算 JPS 2D 邻居查找表
}

// 坐标哈希函数, 将 (x,y) 栅格坐标转换为唯一的一维哈希 ID
inline int GraphSearch::coordToId(int x, int y) const {
  return map_->Index2Vectornum(x,y);               // 委托给 SDFmap 的索引转换函数
}

// inline int GraphSearch::coordToId(int x, int y, int z) const {
//   return x + y*xDim_ + z*xDim_*yDim_;
// }

// 碰撞检测函数
inline bool GraphSearch::isFree(int x, int y) const {
  if(x < 0 || x >= xDim_ || y < 0 || y >= yDim_)  // 边界检查
    return false;
  return !map_->isOccWithSafeDis(x,y,safe_dis_);   // SDF 安全距离检查
}

// 检查栅格是否未被占用（使用地图原始占用信息，不考虑安全距离膨胀）
inline bool GraphSearch::isUnoccupied(int x, int y) const{
  if(x < 0 || x >= xDim_ || y < 0 || y >= yDim_)
    return false;
  return map_->isUnOccupied(x,y);
}

// 检查栅格是否被占用（边界外视为占用）
inline bool GraphSearch::isOccupied(int x, int y) const {
  // return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ &&
  //   cMap_[coordToId(x, y)] > val_free_;
  if(x < 0 || x >= xDim_ || y < 0 || y >= yDim_)
    return true;                                   // 边界外视为被占用
  return map_->isOccupied(x,y);
}

// 启发式函数
inline double GraphSearch::getHeur(int x, int y) const {
  return eps_ * std::sqrt((x - xGoal_) * (x - xGoal_) + (y - yGoal_) * (y - yGoal_));
}

// 2D 规划入口
bool GraphSearch::plan(int xStart, int yStart, int xGoal, int yGoal, bool useJps, int maxExpand)
{
  use_2d_ = true;                                  // 标记为 2D 模式
  pq_.clear();                                     // 清空优先队列
  path_.clear();                                   // 清空路径
  // hm_.resize(xDim_ * yDim_);
  // seen_.resize(xDim_ * yDim_, false);
  std::fill(seen_.begin(), seen_.end(), false);    // 重置所有访问标记

  // 设置 JPS 或 A* 模式
  use_jps_ = useJps;

  // 设置目标
  int goal_id = coordToId(xGoal, yGoal);
  xGoal_ = xGoal; yGoal_ = yGoal;

  // 设置起始节点: 方向 (0,0) 表示起点无惯性方向
  int start_id = coordToId(xStart, yStart);
  StatePtr currNode_ptr = std::make_shared<State>(State(start_id, xStart, yStart, 0, 0));
  currNode_ptr->g = 0;                             // 起点 g 代价 = 0
  currNode_ptr->h = getHeur(xStart, yStart);       // 计算到目标的启发式估计

  return plan(currNode_ptr, maxExpand, start_id, goal_id);
}

// A*/JPS 主循环
bool GraphSearch::plan(StatePtr& currNode_ptr, int maxExpand, int start_id, int goal_id) {
  // 将起始节点插入优先队列
  currNode_ptr->heapkey = pq_.push(currNode_ptr);  // 入队并保存堆位置句柄
  currNode_ptr->opened = true;                     // 标记为已入队
  hm_[currNode_ptr->id] = currNode_ptr;            // 注册到哈希映射
  seen_[currNode_ptr->id] = true;                  // 标记为已访问

  int expand_iteration = 0;                        // 扩展计数器
  while(true)
  {
    expand_iteration++;
    // 取出 f 值最小的节点并弹出（closed）
    currNode_ptr = pq_.top(); pq_.pop();
    currNode_ptr->closed = true;                   // 加入关闭集

    // 目标检测
    if(currNode_ptr->id == goal_id) {
      if(verbose_)
        printf("Goal Reached!!!!!!\n\n");
      break;
    }

    std::vector<int> succ_ids;                     // 后继节点 ID 列表
    std::vector<double> succ_costs;                // 后继节点代价列表
    // 根据模式选择后继生成策略
    if(!use_jps_)
      getSucc(currNode_ptr, succ_ids, succ_costs);  // A*: 8邻域
    else
      getJpsSucc(currNode_ptr, succ_ids, succ_costs); // JPS: 跳点搜索

    if(verbose_)
      printf("size of succs: %zu\n", succ_ids.size());
    // 处理所有后继节点
    for( int s = 0; s < (int) succ_ids.size(); s++ )
    {
      // 检查是否能改善后继节点的 g 值
      StatePtr& child_ptr = hm_[succ_ids[s]];
      double tentative_gval = currNode_ptr->g + succ_costs[s];

      if( tentative_gval < child_ptr->g )
      {
        child_ptr->parentId = currNode_ptr->id;    // 更新父节点
        child_ptr->g = tentative_gval;             // 更新 g 代价

        //double fval = child_ptr->g + child_ptr->h;

        // 情况1: 节点已在开放集中（opened 且未 closed）
        //   执行 decrease-key 操作更新堆位置
        if( child_ptr->opened && !child_ptr->closed) {
          pq_.increase( child_ptr->heapkey );       // 更新堆（boost heap 中 increase 即重新评估键值）
          // 更新方向信息（指向当前节点，归一化为单位方向）
          child_ptr->dx = (child_ptr->x - currNode_ptr->x);
          child_ptr->dy = (child_ptr->y - currNode_ptr->y);
          child_ptr->dz = (child_ptr->z - currNode_ptr->z);
          if(child_ptr->dx != 0)
            child_ptr->dx /= std::abs(child_ptr->dx);
          if(child_ptr->dy != 0)
            child_ptr->dy /= std::abs(child_ptr->dy);
           if(child_ptr->dz != 0)
            child_ptr->dz /= std::abs(child_ptr->dz);
        }
        // 情况2: 节点在关闭集中（opened 且 closed）
        //   这在标准 A* 中理论上不应发生（但加权A*中可能出现）
        else if( child_ptr->opened && child_ptr->closed)
        {
          printf("ASTAR ERROR!\n");
        }
        else // 情况3: 新节点，首次发现
        {
          //printf("add to open set: %d, %d\n", child_ptr->x, child_ptr->y);
          child_ptr->heapkey = pq_.push(child_ptr);  // 加入优先队列
          child_ptr->opened = true;                  // 标记为已入队
        }
      } //
    } // 处理后继节点

    // 最大扩展次数限制检查
    if(maxExpand > 0 && expand_iteration >= maxExpand) {
      if(verbose_)
        printf("MaxExpandStep [%d] Reached!!!!!!\n\n", maxExpand);
      return false;
    }

    // 开放集为空 = 无路可达
    if( pq_.empty()) {
      if(verbose_)
        printf("Priority queue is empty!!!!!!\n\n");
      return false;
    }
  }

  if(verbose_) {
    printf("goal g: %f, h: %f!\n", currNode_ptr->g, currNode_ptr->h);
    printf("Expand [%d] nodes!\n", expand_iteration);
  }

  // 从目标节点反向回溯构建路径
  path_ = recoverPath(currNode_ptr, start_id);

  return true;
}


/**
 * @brief 从目标节点回溯到起点，重建完整路径
 *
 * 利用每个 State 中存储的 parentId，从目标节点沿父链回溯到起点。
 * 返回的路径是反向的（目标在前，起点在后），调用者需要 reverse。
 *
 * 时间复杂度: O(path_length)
 */
std::vector<StatePtr> GraphSearch::recoverPath(StatePtr node, int start_id) {
  std::vector<StatePtr> path;
  path.push_back(node);                            // 先加入目标节点
  while(node && node->id != start_id) {            // 沿父链回溯
    node = hm_[node->parentId];                    // 通过哈希映射查找父节点
    path.push_back(node);
  }

  return path;                                     // 返回反向路径 (goal -> start)
}

// A* 后继生成 (8邻域)
void GraphSearch::getSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs) {
  if(use_2d_) {
    for(const auto& d: ns_) {                      // 遍历 8 邻域
      int new_x = curr->x + d[0];
      int new_y = curr->y + d[1];
      if(!isFree(new_x, new_y))                    // 碰撞检测
        continue;

      int new_id = coordToId(new_x, new_y);
      if(!seen_[new_id]) {                         // 首次发现: 创建新 State
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, d[0], d[1]);
        hm_[new_id]->h = getHeur(new_x, new_y);    // 预计算启发式代价
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt(d[0]*d[0]+d[1]*d[1]));  // 欧几里得距离代价
    }
  }
  // 3D 版本（已废弃）
  // else {
  //   for(const auto& d: ns_) {
  //     int new_x = curr->x + d[0];
  //     int new_y = curr->y + d[1];
  //     int new_z = curr->z + d[2];
  //     if(!isFree(new_x, new_y, new_z))
  //       continue;

  //     int new_id = coordToId(new_x, new_y, new_z);
  //     if(!seen_[new_id]) {
  //       seen_[new_id] = true;
  //       hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, new_z, d[0], d[1], d[2]);
  //       hm_[new_id]->h = getHeur(new_x, new_y, new_z);
  //     }

  //     succ_ids.push_back(new_id);
  //     succ_costs.push_back(std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]));
  //   }
  // }
}

// JPS 后继生成 (跳点搜索)
void GraphSearch::getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs) {
  if(use_2d_) {
    // L1 范数: |dx|+|dy| 用于分类移动类型
    const int norm1 = std::abs(curr->dx)+std::abs(curr->dy);
    int num_neib = jn2d_->nsz[norm1][0];           // 自然邻居数量
    int num_fneib = jn2d_->nsz[norm1][1];          // 强制邻居数量
    // 将 (dx,dy) ∈ {-1,0,1}^2 映射到一维 ID [0..8]
    // id = (dx+1) + 3*(dy+1)
    int id = (curr->dx+1)+3*(curr->dy+1);

    // 遍历所有需要检查的邻居
    for( int dev = 0; dev < num_neib+num_fneib; ++dev) {
      int new_x, new_y;
      int dx, dy;
      // 阶段1: 处理自然邻居（始终需要检查）
      if(dev < num_neib) {
        dx = jn2d_->ns[id][0][dev];                // 查找表中的自然邻居方向
        dy = jn2d_->ns[id][1][dev];
        // 沿 (dx,dy) 执行跳跃，搜索跳点
        if(!jump(curr->x, curr->y, dx, dy, new_x, new_y)) continue;
      }
      // 阶段2: 处理强制邻居（仅在 f1 被占用时添加）
      else {
        // 检查 f1 位置是否被占用
        int nx = curr->x + jn2d_->f1[id][0][dev-num_neib];
        int ny = curr->y + jn2d_->f1[id][1][dev-num_neib];
        if(!isFree(nx,ny)) {                       // f1 被占用 -> 强制邻居触发
          dx = jn2d_->f2[id][0][dev-num_neib];     // 向 f2 方向跳跃
          dy = jn2d_->f2[id][1][dev-num_neib];
          if(!jump(curr->x, curr->y, dx, dy, new_x, new_y)) continue;
        }
        else
          continue;                                // f1 空闲 -> 不需要此方向
      }

      // 跳点或目标已找到，注册为后继节点
      int new_id = coordToId(new_x, new_y);
      if(!seen_[new_id]) {                         // 首次发现
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, dx, dy);
        hm_[new_id]->h = getHeur(new_x, new_y);
      }
      succ_ids.push_back(new_id);
      // 后继代价 = 直接从当前节点到跳点的欧几里得距离
      // 这与 A* 的逐格代价不同，JPS 的后继可以跨越多个栅格
      succ_costs.push_back(std::sqrt((new_x - curr->x) * (new_x - curr->x) +
            (new_y - curr->y) * (new_y - curr->y)));
    }
  }
  // 3D 版本（已废弃，逻辑与2D相同但包含z分量）
  // else {
  //   const int norm1 = std::abs(curr->dx)+std::abs(curr->dy)+std::abs(curr->dz);
  //   int num_neib = jn3d_->nsz[norm1][0];
  //   int num_fneib = jn3d_->nsz[norm1][1];
  //   int id = (curr->dx+1)+3*(curr->dy+1)+9*(curr->dz+1);

  //   for( int dev = 0; dev < num_neib+num_fneib; ++dev) {
  //     int new_x, new_y, new_z;
  //     int dx, dy, dz;
  //     if(dev < num_neib) {
  //       dx = jn3d_->ns[id][0][dev];
  //       dy = jn3d_->ns[id][1][dev];
  //       dz = jn3d_->ns[id][2][dev];
  //       if(!jump(curr->x, curr->y, curr->z,
  //             dx, dy, dz, new_x, new_y, new_z)) continue;
  //     }
  //     else {
  //       int nx = curr->x + jn3d_->f1[id][0][dev-num_neib];
  //       int ny = curr->y + jn3d_->f1[id][1][dev-num_neib];
  //       int nz = curr->z + jn3d_->f1[id][2][dev-num_neib];
  //       if(isOccupied(nx,ny,nz)) {
  //         dx = jn3d_->f2[id][0][dev-num_neib];
  //         dy = jn3d_->f2[id][1][dev-num_neib];
  //         dz = jn3d_->f2[id][2][dev-num_neib];
  //         if(!jump(curr->x, curr->y, curr->z,
  //               dx, dy, dz, new_x, new_y, new_z)) continue;
  //       }
  //       else
  //         continue;
  //     }

  //     int new_id = coordToId(new_x, new_y, new_z);
  //     if(!seen_[new_id]) {
  //       seen_[new_id] = true;
  //       hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, new_z, dx, dy, dz);
  //       hm_[new_id]->h = getHeur(new_x, new_y, new_z);
  //     }

  //     succ_ids.push_back(new_id);
  //     succ_costs.push_back(std::sqrt((new_x - curr->x) * (new_x - curr->x) +
  //           (new_y - curr->y) * (new_y - curr->y) +
  //           (new_z - curr->z) * (new_z - curr->z)));
  //   }

  // }
}


/**
 * @brief JPS 核心: 沿方向 (dx,dy) 递归跳跃
 *
 * 这是 JPS 算法的核心函数。它沿给定方向一直前进，跳过所有"非跳点"的中间节点，
 * 直到遇到: 目标、障碍物、或跳点。
 *
 * 递归规则（2D）:
 *
 * jump(x, y, dx, dy):
 *   new_x = x + dx, new_y = y + dy
 *
 *   // 规则1: 撞墙
 *   if not isFree(new_x, new_y): return false
 *
 *   // 规则2: 到达目标
 *   if (new_x, new_y) == goal: return true, (new_x, new_y)
 *
 *   // 规则3: 强制邻居 -> 这是一个跳点
 *   if hasForced(new_x, new_y, dx, dy): return true, (new_x, new_y)
 *
 *   // 规则4: 对角线移动时检查轴向分量
 *   if dx != 0 and dy != 0:  // 对角线移动
 *     if jump(new_x, new_y, dx, 0, ...) or jump(new_x, new_y, 0, dy, ...):
 *       return true, (new_x, new_y)  // 轴向能跳到跳点，则对角线方向也是跳点
 *
 *   // 规则5: 继续沿原方向跳跃
 *   return jump(new_x, new_y, dx, dy, new_x, new_y)
 *
 * 图示说明（向右搜索 dx=1, dy=0）:
 *   @ . . . X . . G
 *   起点向右跳跃，经过若干空节点，遇到:
 *   - X: 如果 X 下方的格子是障碍物而 X 的下方-右下是空的，X 就是跳点
 *   - 否则继续右跳直到 G 或地图边界
 *
 * 关键性能优化:
 * jump() 是尾递归的，编译器可将其优化为迭代，避免递归栈溢出。
 * 在开阔区域一次 jump 可跨越数十上百个节点，这是 JPS 高效的原因。
 *
 * @param[in] x, y 当前节点坐标
 * @param[in] dx, dy 跳跃方向（归一化到 {-1, 0, 1}）
 * @param[out] new_x, new_y 输出的跳点坐标（仅在返回 true 时有意义）
 * @return true 找到目标或跳点; false 遇到障碍物
 */
bool GraphSearch::jump(int x, int y, int dx, int dy, int& new_x, int& new_y ) {
  new_x = x + dx;                                  // 沿方向前进一格
  new_y = y + dy;

  // 规则1: 撞墙检查
  if (!isFree(new_x, new_y))
    return false;

  // 规则2: 目标检查
  if (new_x ==  xGoal_ && new_y == yGoal_)
    return true;

  // 规则3: 强制邻居检查 -> 这是一个跳点
  if (hasForced(new_x, new_y, dx, dy))
    return true;

  // 规则4: 对角线移动 -> 检查轴向分量
  // 只有当 dx 和 dy 都非零时才需要此检查（即对角线移动）
  const int id = (dx+1)+3*(dy+1);
  const int norm1 = std::abs(dx) + std::abs(dy);
  int num_neib = jn2d_->nsz[norm1][0];             // 自然邻居数
  // 遍历自然邻居中的前 (num_neib-1) 个（即轴向分量，不包括对角线自身）
  for( int k = 0; k < num_neib-1; ++k )
  {
    int new_new_x, new_new_y;
    // 递归检查轴向分量是否可跳
    if(jump(new_x, new_y, jn2d_->ns[id][0][k], jn2d_->ns[id][1][k],
        new_new_x, new_new_y)) return true;         // 轴向能跳到跳点 -> 对角线也是跳点
  }

  // 规则5: 继续沿原方向递归跳跃
  return jump(new_x, new_y, dx, dy, new_x, new_y);
}


// bool GraphSearch::jump(int x, int y, int z, int dx, int dy, int dz, int& new_x, int& new_y, int& new_z) {
//   new_x = x + dx;
//   new_y = y + dy;
//   new_z = z + dz;
//   if (!isFree(new_x, new_y, new_z))
//     return false;

//   if (new_x ==  xGoal_ && new_y == yGoal_ && new_z == zGoal_)
//     return true;

//   if (hasForced(new_x, new_y, new_z, dx, dy, dz))
//     return true;

//   const int id = (dx+1)+3*(dy+1)+9*(dz+1);
//   const int norm1 = std::abs(dx) + std::abs(dy) +std::abs(dz);
//   int num_neib = jn3d_->nsz[norm1][0];
//   for( int k = 0; k < num_neib-1; ++k )
//   {
//     int new_new_x, new_new_y, new_new_z;
//     if(jump(new_x,new_y,new_z,
//           jn3d_->ns[id][0][k], jn3d_->ns[id][1][k], jn3d_->ns[id][2][k],
//         new_new_x, new_new_y, new_new_z)) return true;
//   }


//   return jump(new_x, new_y, new_z, dx, dy, dz, new_x, new_y, new_z);
// }

// 强制邻居检测
inline bool GraphSearch::hasForced(int x, int y, int dx, int dy) {
  const int id = (dx+1)+3*(dy+1);                  // 方向->查找表索引
  // 检查两组强制邻居（如直线移动的上下两格）
  for( int fn = 0; fn < 2; ++fn )
  {
    int nx = x + jn2d_->f1[id][0][fn];             // 强制邻居检查点坐标
    int ny = y + jn2d_->f1[id][1][fn];
    if( !isFree(nx,ny) )                           // 如果该位置不可通行
      return true;                                 // -> 存在强制邻居，当前点是跳点
  }
  return false;                                    // 无强制邻居
}


// inline bool GraphSearch::hasForced(int x, int y, int z, int dx, int dy, int dz) {
//   int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
//   int id = (dx+1)+3*(dy+1)+9*(dz+1);
//   switch(norm1)
//   {
//     case 1:
//       // 1-d move, check 8 neighbors
//       for( int fn = 0; fn < 8; ++fn )
//       {
//         int nx = x + jn3d_->f1[id][0][fn];
//         int ny = y + jn3d_->f1[id][1][fn];
//         int nz = z + jn3d_->f1[id][2][fn];
//         if( isOccupied(nx,ny,nz) )
//           return true;
//       }
//       return false;
//     case 2:
//       // 2-d move, check 8 neighbors
//       for( int fn = 0; fn < 8; ++fn )
//       {
//         int nx = x + jn3d_->f1[id][0][fn];
//         int ny = y + jn3d_->f1[id][1][fn];
//         int nz = z + jn3d_->f1[id][2][fn];
//         if( isOccupied(nx,ny,nz) )
//           return true;
//       }
//       return false;
//     case 3:
//       // 3-d move, check 6 neighbors
//       for( int fn = 0; fn < 6; ++fn )
//       {
//         int nx = x + jn3d_->f1[id][0][fn];
//         int ny = y + jn3d_->f1[id][1][fn];
//         int nz = z + jn3d_->f1[id][2][fn];
//         if( isOccupied(nx,ny,nz) )
//           return true;
//       }
//       return false;
//     default:
//       return false;
//   }
// }

// 公共接口函数

std::vector<StatePtr> GraphSearch::getPath() const {
  return path_;
}

/// 获取开放集中的节点（opened=true, closed=false）
std::vector<StatePtr> GraphSearch::getOpenSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it && it->opened && !it->closed)            // 已入队但未扩展
      ss.push_back(it);
  }
  return ss;
}

/// 获取关闭集中的节点（closed=true）
std::vector<StatePtr> GraphSearch::getCloseSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it && it->closed)                           // 已扩展
      ss.push_back(it);
  }
  return ss;
}

/// 获取所有已发现的节点
std::vector<StatePtr> GraphSearch::getAllSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it)                                         // 非空状态指针
      ss.push_back(it);
  }
  return ss;
}

void GraphSearch::SetSafeDis(const double &safe_dis){
  safe_dis_ = safe_dis;
}

double GraphSearch::GetSafeDis(){
  return safe_dis_;
}

// ==================== JPS 邻居查找表实现 ====================

/**
 * @brief 静态常量定义（必须在命名空间作用域中定义）
 *
 * JPS2DNeib::nsz:
 *   nsz[norm1][0] = 自然邻居数量
 *   nsz[norm1][1] = 强制邻居数量
 *   norm1=0 (起点):     8 自然,  0 强制
 *   norm1=1 (直线):     1 自然,  2 强制
 *   norm1=2 (对角线):   3 自然,  2 强制
 */
constexpr int JPS2DNeib::nsz[3][2];

/**
 * @brief 构造函数: 预计算 JPS 2D 邻居查找表
 *
 * 对所有 9 种方向 (dx,dy) ∈ {-1,0,1}^2（排除 (0,0) 起点状态）:
 * 1. 计算 L1 范数 norm1 = |dx|+|dy|
 * 2. 查找 nsz[norm1] 获取自然邻居和强制邻居的数量
 * 3. 对每个自然邻居调用 Neib() 计算坐标
 * 4. 对每个强制邻居调用 FNeib() 计算检查坐标和跳跃方向
 *
 * 结果存储在:
 * - ns[id][0..1][k]: 自然邻居的 dx/dy（始终需要跳跃的方向）
 * - f1[id][0..1][k]: 强制邻居的检查坐标
 * - f2[id][0..1][k]: 如果 f1 被占用，向 f2 方向跳跃
 *
 * 时间复杂度: O(1)（固定大小查找表的初始化）
 */
JPS2DNeib::JPS2DNeib() {
  int id = 0;
  for(int dy = -1; dy <= 1; ++dy) {                // 遍历 y 方向分量
    for(int dx = -1; dx <= 1; ++dx) {              // 遍历 x 方向分量
      int norm1 = std::abs(dx) + std::abs(dy);      // L1 范数 = 移动类型
      // 填充自然邻居
      for(int dev = 0; dev < nsz[norm1][0]; ++dev)
        Neib(dx,dy,norm1,dev, ns[id][0][dev], ns[id][1][dev]);
      // 填充强制邻居
      for(int dev = 0; dev < nsz[norm1][1]; ++dev)
      {
        FNeib(dx,dy,norm1,dev,
            f1[id][0][dev],f1[id][1][dev],
            f2[id][0][dev],f2[id][1][dev]);
      }
      id ++;                                       // 增量为下一个方向枚举
    }
  }
}

/// 调试打印函数: 输出每种方向的强制邻居信息
void JPS2DNeib::print() {
  for(int dx = -1; dx <= 1; dx++) {
    for(int dy = -1; dy <= 1; dy++) {
      int id = (dx+1)+3*(dy+1);
      printf("[dx: %d, dy: %d]-->id: %d:\n", dx, dy, id);
      for(unsigned int i = 0; i < sizeof(f1[id][0])/sizeof(f1[id][0][0]); i++)
        printf("                f1: [%d, %d]\n", f1[id][0][i], f1[id][1][i]);
    }
  }
}

/**
 * @brief 计算 2D 自然邻居
 *
 * 自然邻居: 沿移动方向 (dx,dy) 行进时，需要通过 JPS 跳跃检查的邻居方向。
 *
 * 规则:
 * - norm1=0 (起点): 8 个方向各一个（全部邻域），dev 依次遍历 8 个方向
 * - norm1=1 (直线移动): 只有沿 (dx,dy) 正前方一个方向
 * - norm1=2 (对角线移动): 3 个方向: dx分量, dy分量, 对角线方向 (dx,dy)
 *
 * @param dx, dy 移动方向分量
 * @param norm1  移动类型的 L1 范数
 * @param dev    邻居索引（0..nsz[norm1][0]-1）
 * @param[out] tx, ty 自然邻居的坐标偏移量
 */
void JPS2DNeib::Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty)
{
  switch(norm1)
  {
    case 0:  // 起点: 8个方向依次枚举
      switch(dev)
      {
        case 0: tx=1; ty=0; return;                // 右
        case 1: tx=-1; ty=0; return;               // 左
        case 2: tx=0; ty=1; return;                // 上
        case 3: tx=1; ty=1; return;                // 右上
        case 4: tx=-1; ty=1; return;               // 左上
        case 5: tx=0; ty=-1; return;               // 下
        case 6: tx=1; ty=-1; return;               // 右下
        case 7: tx=-1; ty=-1; return;              // 左下
     }
    case 1:  // 直线移动: 唯一方向 = (dx, dy)
      tx = dx; ty = dy; return;
    case 2:  // 对角线移动: 3个分量方向
      switch(dev)
      {
        case 0: tx = dx; ty = 0; return;            // 水平分量
        case 1: tx = 0; ty = dy; return;            // 垂直分量
        case 2: tx = dx; ty = dy; return;           // 对角线方向本身
      }
  }
}

/**
 * @brief 计算 2D 强制邻居
 *
 * 强制邻居原理（以直线向右移动 dx=1, dy=0 为例）:
 *
 *   场景: 当前向 (1,0) 移动
 *   f1[0] = (0, 1)  -> 检查上方格子
 *   f1[1] = (0,-1)  -> 检查下方格子
 *
 *   如果 f1 处的格子被占用:
 *     f2 = (dx+f1x, dy+f1y) 是强制邻居对应的跳跃方向
 *     例如上方被占用时 f2 = (1, 1) 表示向右上跳跃
 *
 *   这保证了 JPS 能发现所有必经的"拐角"节点。
 *
 * @param dx, dy 移动方向
 * @param norm1  移动类型
 * @param dev    强制邻居索引
 * @param[out] fx, fy  需要检查的强制邻居位置
 * @param[out] nx, ny  如果 fx,fy 被占用的跳跃方向
 */
void JPS2DNeib::FNeib( int dx, int dy, int norm1, int dev,
                       int& fx, int& fy, int& nx, int& ny)
{
  switch(norm1)
  {
    case 1:  // 直线移动: 需要检查与移动方向垂直的两个侧方
      switch(dev)
      {
        case 0: fx= 0; fy= 1; break;               // 上方检查点
        case 1: fx= 0; fy= -1;  break;             // 下方检查点
      }

      // 如果方向是上下移动（dx=0），需要交换 f 和 n 的坐标
      // switch order if different direction
      if(dx == 0)
        fx = fy, fy = 0;                           // 交换, 使 fx 沿移动垂直方向

      nx = dx + fx; ny = dy + fy;                  // 强制跳跃方向 = 前进+侧向
      return;
    case 2:  // 对角线移动
      switch(dev)
      {
        case 0:
          fx = -dx; fy = 0;                        // 检查侧后方水平方向
          nx = -dx; ny = dy;                       // 被占用时跳向: 侧后方水平 + 垂直
          return;
        case 1:
          fx = 0; fy = -dy;                        // 检查侧后方垂直方向
          nx = dx; ny = -dy;                       // 被占用时跳向: 水平 + 侧后方垂直
          return;
      }
  }
}

// ==================== JPS 3D 邻居查找表实现 ====================

/**
 * @brief 静态常量定义
 *
 * JPS3DNeib::nsz:
 *   norm1=0 (起点):          26 自然,  0 强制
 *   norm1=1 (直线):           1 自然,  8 强制
 *   norm1=2 (面对角线, sqrt2): 3 自然, 12 强制
 *   norm1=3 (体对角线, sqrt3): 7 自然, 12 强制
 */
constexpr int JPS3DNeib::nsz[4][2];

/**
 * @brief 构造函数: 预计算 JPS 3D 邻居查找表
 *
 * 对所有 27 种三维方向 (dx,dy,dz) ∈ {-1,0,1}^3:
 * 初始化和填充自然邻居和强制邻居查找表。
 *
 * 注意: 3D JPS 扩展了 2D 的规则，但原理相同:
 * - 自然邻居数量更多（因为3D有更多分量组合）
 * - 强制邻居检查更复杂（考虑空间中的障碍物阻塞）
 */
JPS3DNeib::JPS3DNeib() {
  int id = 0;
  for(int dz = -1; dz <= 1; ++dz) {
    for(int dy = -1; dy <= 1; ++dy) {
      for(int dx = -1; dx <= 1; ++dx) {
        int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
        for(int dev = 0; dev < nsz[norm1][0]; ++dev)
          Neib(dx,dy,dz,norm1,dev,
              ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
        for(int dev = 0; dev < nsz[norm1][1]; ++dev)
        {
          FNeib(dx,dy,dz,norm1,dev,
              f1[id][0][dev],f1[id][1][dev], f1[id][2][dev],
              f2[id][0][dev],f2[id][1][dev], f2[id][2][dev]);
        }
        id ++;
      }
    }
  }
}

/**
 * @brief 计算 3D 自然邻居
 *
 * 与 2D 版本的逻辑类似，但考虑了三维方向的分量组合:
 * - norm1=0: 26 个方向（体心立方邻域，3^3-1=26）
 * - norm1=1: 沿方向本身（直线移动）
 * - norm1=2: 3个分量方向（各非零分量 + 组合对角线）
 * - norm1=3: 7个方向（3个轴向分量 + 3个面对角线 + 体对角线本身）
 */
void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev,
    int& tx, int& ty, int& tz)
{
  switch(norm1)
  {
    case 0:  // 起点: 枚举 26 个邻域方向
      switch(dev)
      {
        case 0: tx=1; ty=0; tz=0; return;
        case 1: tx=-1; ty=0; tz=0; return;
        case 2: tx=0; ty=1; tz=0; return;
        case 3: tx=1; ty=1; tz=0; return;
        case 4: tx=-1; ty=1; tz=0; return;
        case 5: tx=0; ty=-1; tz=0; return;
        case 6: tx=1; ty=-1; tz=0; return;
        case 7: tx=-1; ty=-1; tz=0; return;
        case 8: tx=0; ty=0; tz=1; return;
        case 9: tx=1; ty=0; tz=1; return;
        case 10: tx=-1; ty=0; tz=1; return;
        case 11: tx=0; ty=1; tz=1; return;
        case 12: tx=1; ty=1; tz=1; return;
        case 13: tx=-1; ty=1; tz=1; return;
        case 14: tx=0; ty=-1; tz=1; return;
        case 15: tx=1; ty=-1; tz=1; return;
        case 16: tx=-1; ty=-1; tz=1; return;
        case 17: tx=0; ty=0; tz=-1; return;
        case 18: tx=1; ty=0; tz=-1; return;
        case 19: tx=-1; ty=0; tz=-1; return;
        case 20: tx=0; ty=1; tz=-1; return;
        case 21: tx=1; ty=1; tz=-1; return;
        case 22: tx=-1; ty=1; tz=-1; return;
        case 23: tx=0; ty=-1; tz=-1; return;
        case 24: tx=1; ty=-1; tz=-1; return;
        case 25: tx=-1; ty=-1; tz=-1; return;
      }
    case 1:  // 直线移动: 方向本身
      tx = dx; ty = dy; tz = dz; return;
    case 2:  // 面对角线移动 (|dx|+|dy|+|dz|=2): 3个分量
      switch(dev)
      {
        case 0:
          if(dz == 0){
            tx = 0; ty = dy; tz = 0; return;        // 纯y分量
          }else{
            tx = 0; ty = 0; tz = dz; return;        // 纯z分量
          }
        case 1:
          if(dx == 0){
            tx = 0; ty = dy; tz = 0; return;        // 纯y分量
          }else{
            tx = dx; ty = 0; tz = 0; return;        // 纯x分量
          }
        case 2:
          tx = dx; ty = dy; tz = dz; return;        // 对角线本身
      }
    case 3:  // 体对角线移动 (|dx|+|dy|+|dz|=3): 7个分量
      switch(dev)
      {
        case 0: tx = dx; ty =  0; tz =  0; return;  // 轴向x
        case 1: tx =  0; ty = dy; tz =  0; return;  // 轴向y
        case 2: tx =  0; ty =  0; tz = dz; return;  // 轴向z
        case 3: tx = dx; ty = dy; tz =  0; return;  // 面对角线xy
        case 4: tx = dx; ty =  0; tz = dz; return;  // 面对角线xz
        case 5: tx =  0; ty = dy; tz = dz; return;  // 面对角线yz
        case 6: tx = dx; ty = dy; tz = dz; return;  // 体对角线
      }
  }
}

/**
 * @brief 计算 3D 强制邻居
 *
 * 3D 强制邻居的复杂度来自:
 * - 直线移动 (norm1=1): 8 个强制邻居（与方向垂直的平面上的 8 个格子）
 * - 面对角线 (norm1=2): 最多 12 个强制邻居检查 + 跳跃方向
 * - 体对角线 (norm1=3): 最多 12 个强制邻居检查 + 跳跃方向
 *
 * 强制邻居的触发条件:
 * 当某个 f1 检查点被障碍物占用时，对应对角线方向的格子无法通过
 * 更短路径到达，因此必须通过当前节点。这使得当前节点成为跳点。
 */
void JPS3DNeib::FNeib( int dx, int dy, int dz, int norm1, int dev,
                          int& fx, int& fy, int& fz,
                          int& nx, int& ny, int& nz)
{
  switch(norm1)
  {
    case 1:  // 直线移动: 8个强制邻居
      switch(dev)
      {
        case 0: fx= 0; fy= 1; fz = 0; break;
        case 1: fx= 0; fy=-1; fz = 0; break;
        case 2: fx= 1; fy= 0; fz = 0; break;
        case 3: fx= 1; fy= 1; fz = 0; break;
        case 4: fx= 1; fy=-1; fz = 0; break;
        case 5: fx=-1; fy= 0; fz = 0; break;
        case 6: fx=-1; fy= 1; fz = 0; break;
        case 7: fx=-1; fy=-1; fz = 0; break;
      }
      nx = fx; ny = fy; nz = dz;
      // 根据移动的轴向调整坐标顺序
      if(dx != 0){
        fz = fx; fx = 0;
        nz = fz; nx = dx;
      }if(dy != 0){
        fz = fy; fy = 0;
        nz = fz; ny = dy;
      }
      return;
    case 2:  // 面对角线移动
      if(dx == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = 0; ny = dy; nz = -dz;
            return;
          case 1:
            fx = 0; fy = -dy; fz = 0;
            nx = 0; ny = -dy; nz = dz;
            return;
          case 2:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = dz;
            return;
          case 3:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = dz;
            return;
          case 4:
            fx = 1; fy = 0; fz = -dz;
            nx = 1; ny = dy; nz = -dz;
            return;
          case 5:
            fx = 1; fy = -dy; fz = 0;
            nx = 1; ny = -dy; nz = dz;
            return;
          case 6:
            fx = -1; fy = 0; fz = -dz;
            nx = -1; ny = dy; nz = -dz;
            return;
          case 7:
            fx = -1; fy = -dy; fz = 0;
            nx = -1; ny = -dy; nz = dz;
            return;
          // 额外检查（确保所有可能的强制邻居都被覆盖）
          case 8:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = 0;
            return;
          case 9:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = 0; nz = dz;
            return;
          case 10:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = 0;
            return;
          case 11:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = 0; nz = dz;
            return;
        }
      }else if(dy == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = dx; ny = 0; nz = -dz;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = 0; nz = dz;
            return;
          case 2:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = dz;
            return;
          case 3:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1;nz = dz;
            return;
          case 4:
            fx = 0; fy = 1; fz = -dz;
            nx = dx; ny = 1; nz = -dz;
            return;
          case 5:
            fx = -dx; fy = 1; fz = 0;
            nx = -dx; ny = 1; nz = dz;
            return;
          case 6:
            fx = 0; fy = -1; fz = -dz;
            nx = dx; ny = -1; nz = -dz;
            return;
          case 7:
            fx = -dx; fy = -1; fz = 0;
            nx = -dx; ny = -1; nz = dz;
            return;
          // 额外检查
          case 8:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = 0;
            return;
          case 9:
            fx = 0; fy = 1; fz = 0;
            nx = 0; ny = 1; nz = dz;
            return;
          case 10:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1; nz = 0;
            return;
          case 11:
            fx = 0; fy = -1; fz = 0;
            nx = 0; ny = -1; nz = dz;
            return;
        }
      }else{// dz==0 面对角线在 xy 平面
        switch(dev)
        {
          case 0:
            fx = 0; fy = -dy; fz = 0;
            nx = dx; ny = -dy; nz = 0;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = dy; nz = 0;
            return;
          case 2:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = dy; nz = 1;
            return;
          case 3:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = dy; nz = -1;
            return;
          case 4:
            fx = 0; fy = -dy; fz = 1;
            nx = dx; ny = -dy; nz = 1;
            return;
          case 5:
            fx = -dx; fy = 0; fz = 1;
            nx = -dx; ny = dy; nz = 1;
            return;
          case 6:
            fx = 0; fy = -dy; fz = -1;
            nx = dx; ny = -dy; nz = -1;
            return;
          case 7:
            fx = -dx; fy = 0; fz = -1;
            nx = -dx; ny = dy; nz = -1;
            return;
          // 额外检查
          case 8:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = 0; nz = 1;
            return;
          case 9:
            fx = 0; fy = 0; fz = 1;
            nx = 0; ny = dy; nz = 1;
            return;
          case 10:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = 0; nz = -1;
            return;
          case 11:
            fx = 0; fy = 0; fz = -1;
            nx = 0; ny = dy; nz = -1;
            return;
        }
      }
    case 3:  // 体对角线移动
      switch(dev)
      {
        case 0:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = dz;
          return;
        case 1:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = dz;
          return;
        case 2:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = dy; nz = -dz;
          return;
        // 前3个是核心强制邻居检查
        case 3:
          fx = 0; fy = -dy; fz = -dz;
          nx = dx; ny = -dy; nz = -dz;
          return;
        case 4:
          fx = -dx; fy = 0; fz = -dz;
          nx = -dx; ny = dy; nz = -dz;
          return;
        case 5:
          fx = -dx; fy = -dy; fz = 0;
          nx = -dx; ny = -dy; nz = dz;
          return;
        // 额外检查
        case 6:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = 0; nz = dz;
          return;
        case 7:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = 0;
          return;
        case 8:
          fx = 0; fy = -dy; fz = 0;
          nx = 0; ny = -dy; nz = dz;
          return;
        case 9:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = 0;
          return;
        case 10:
          fx = 0; fy = 0; fz = -dz;
          nx = 0; ny = dy; nz = -dz;
          return;
        case 11:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = 0; nz = -dz;
          return;
      }
  }
}





