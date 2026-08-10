/**
 * @file jps_planner.cpp
 * @brief 前端 JPS 规划器实现 — 路径搜索 + 路径简化 + 梯形速度剖面时间参数化
 *
 * 本文件实现了 JPSPlanner 类的全部成员函数，是前端轨迹规划的完整流水线。
 *
 * 整体流程:
 * 1. 构造函数: 从 ROS 参数服务器加载配置参数，初始化 ROS 发布器和图搜索引擎
 * 2. plan(): 调用 JPS 图搜索生成从起点到目标的几何路径
 * 3. getKinoNodeWithStartPath(): 路径后处理主入口
 *    a. removeCornerPts(): 贪心路径简化（去除之字形）
 *    b. getSampleTraj(): 生成带角度信息的5D采样轨迹
 *    c. getTrajsWithTime(): 梯形速度剖面时间参数化
 *
 * 梯形速度剖面的设计意图:
 * 由于 JPS 输出的是分段直线路径（由跳点连接），路径经过简化后变为一系列直线段
 * 连接的关键点。在每个线段上，使用一维梯形速度剖面:
 *   - 加速段: v(t) = v_start + a_max * t,    s(t) = v_start*t + 0.5*a_max*t^2
 *   - 匀速段: v(t) = v_max,                   s(t) = s_acc + v_max*(t - t_acc)
 *   - 减速段: v(t) = v_max - a_max * t,      s(t) = s_cruise + v_max*(t-t_cruise) - 0.5*a_max*(t-t_cruise)^2
 *
 * 如果线段长度不足以加速到最大速度，退化为三角形速度剖面（仅加速+减速）。
 *
 * 加权路径长度的设计:
 * 将路径的"代价"定义为弧长和角度变化的加权和:
 *   L_weighted = sum( distance_weight * |ds| + yaw_weight * |dtheta| )
 * 这使得速度剖面同时考虑了空间位移（distance_weight）和转向角度（yaw_weight），
 * 在需要大角度转向的地方自动减速，产生更合理的速度规划。
 */

#include "front_end/jps_planner/jps_planner.h"

using namespace JPS;

// ======================== 构造函数 ========================

/**
 * @brief JPSPlanner 构造函数
 *
 * 初始化流程:
 * 1. 创建 ROS 发布器（路径可视化、法向量可视化）
 * 2. 从 ROS 参数服务器加载所有配置参数
 * 3. 创建 GraphSearch 图搜索引擎实例
 *
 * ROS 参数列表:
 * - jps_safe_dis: 安全距离（米），JPS搜索时障碍物膨胀半径
 * - max_jps_dis: JPS 最大搜索距离
 * - jps_distance_weight: 弧长权重
 * - jps_yaw_weight: 偏航角变化权重
 * - trajCutLength: 输出轨迹的最大弧长（米）
 * - max_vel: 最大线速度（米/秒）
 * - max_acc: 最大线加速度（米/秒^2）
 * - max_omega: 最大角速度（弧度/秒）
 * - max_domega: 最大角加速度（弧度/秒^2）
 * - timeResolution: 轨迹时间采样间隔（秒）
 * - mintrajNum: 最小轨迹采样点数
 * - jps_truncation_time: JPS 截断时间
 */
JPSPlanner::JPSPlanner(std::shared_ptr<SDFmap> map, const ros::NodeHandle &nh): map_util_(map), nh_(nh) {
    // graph_search_ = std::make_shared<GraphSearch>(map_util_);
    // 创建 ROS 发布器: 简化后的路径、初始JPS原始路径、法向量
    path_pub_ = ros::Publisher(nh_.advertise<nav_msgs::Path>("jps_path", 1));
    init_path_pub_ = ros::Publisher(nh_.advertise<nav_msgs::Path>("init_jps_path", 1));
    normal_vector_pub_ = ros::Publisher(nh_.advertise<visualization_msgs::Marker>("normal_vector", 1));

    // 从 ROS 参数服务器加载配置参数
    nh_.getParam(ros::this_node::getName()+ "/jps_safe_dis", safe_dis_);
    nh_.getParam(ros::this_node::getName()+ "/max_jps_dis", max_jps_dis_);
    nh_.getParam(ros::this_node::getName()+ "/jps_distance_weight", distance_weight_);
    nh_.getParam(ros::this_node::getName()+ "/jps_yaw_weight", yaw_weight_);
    nh_.getParam(ros::this_node::getName()+ "/trajCutLength", trajCutLength_);

    nh_.getParam(ros::this_node::getName()+ "/max_vel",max_vel_);
    nh_.getParam(ros::this_node::getName()+ "/max_acc",max_acc_);
    nh_.getParam(ros::this_node::getName()+ "/max_omega",max_omega_);
    nh_.getParam(ros::this_node::getName()+ "/max_domega",max_domega_);

    nh_.getParam(ros::this_node::getName()+ "/timeResolution",sampletime_);
    nh_.getParam(ros::this_node::getName()+ "/mintrajNum", mintrajNum_);

    nh_.getParam(ros::this_node::getName()+ "/jps_truncation_time", jps_truncation_time_);

    // 创建图搜索引擎（使用 SDF 地图和安全距离）
    graph_search_ = std::make_shared<GraphSearch>(map_util_, safe_dis_);

}

// ======================== JPS 路径规划 ========================

/**
 * @brief 执行 JPS 全局路径搜索
 *
 * 流程:
 * 1. 保存起止状态
 * 2. 将世界坐标转为栅格索引
 * 3. 动态调整安全距离: 取 (safe_dis_, start_sdf*0.8, goal_sdf*0.8) 的最小值
 *    这确保起止点如果太靠近障碍物也能被搜索到
 *    （强制在起点和目标处放宽碰撞检测标准）
 * 4. 调用 graph_search_->plan() 执行 JPS 搜索（useJps=true, maxExpand=1e10）
 * 5. 将搜索结果的栅格路径转为世界坐标
 * 6. 发布初始路径用于可视化
 *
 * @return true 找到路径，false 搜索失败
 */
bool JPSPlanner::plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal){

    start_state_ = start;                                    // 保存起点状态 [x, y, yaw]
    end_state_ = goal;                                       // 保存目标状态 [x, y, yaw]

    // 世界坐标 -> 栅格索引
    Eigen::Vector2i start_idx = map_util_->coord2gridIndex(start.head(2));
    Eigen::Vector2i goal_idx = map_util_->coord2gridIndex(goal.head(2));

    // 动态调整安全距离: 取 SDF 距离的 80% 与参数安全距离的较小值
    // 这确保起点和目标点即使贴墙也能被通过
    double start_dis = map_util_->getDistanceReal(map_util_->gridIndex2coordd(start_idx)) * 0.8;
    double goal_dis = map_util_->getDistanceReal(map_util_->gridIndex2coordd(goal_idx)) * 0.8;
    double safe_dis = std::max(std::min(safe_dis_, start_dis), 0.0);
    safe_dis = std::max(std::min(safe_dis, goal_dis), 0.0);

    // 执行 JPS 搜索: useJps=true, 最大扩展节点 1e10（近似无限制）
    graph_search_->plan(start_idx(0), start_idx(1), goal_idx(0), goal_idx(1), true, 1e10);

    // 获取搜索路径并检查有效性
    const auto path = graph_search_->getPath();
    if (path.size() < 1) {
        std::cout << "Cannot find a path from " << start.transpose() <<" to " << goal.transpose() << " Abort!" << std::endl;
        status_ = -1;
        return false;
    }

    // 将栅格路径转为世界坐标
    std::vector<Eigen::Vector2d> ps;
    for (const auto &it : path) {
        ps.push_back(map_util_->gridIndex2coordd(Eigen::Vector2i(it->x, it->y)));
    }
    pubPath(ps, init_path_pub_);                           // 发布初始路径

    raw_path_ = ps;
    // JPS 从目标开始回溯，路径是反向的（goal->start），需要反转
    std::reverse(std::begin(raw_path_), std::end(raw_path_));

    // 确保路径起点和终点精确匹配（避免因栅格离散化产生的微小偏差）
    raw_path_.front() = start.head(2);
    raw_path_.back() = goal.head(2);


    return true;
}

// ======================== 高分辨率栅格路径生成 ========================

/**
 * @brief 生成高分辨率栅格路径
 *
 * 对简化后的路径 path_ 中每对相邻点，使用 Bresenham 直线光栅化算法填充
 * 两者之间的所有栅格坐标。这用于精确的碰撞检测。
 *
 * 例如 path_ = [A, B, C]，则生成:
 *   Bresenham(A,B) + Bresenham(B,C) 的所有栅格点
 *
 * 注意: 路径点的插入顺序保证了 small_resolution_path_ 是连续的。
 */
void JPSPlanner::get_small_resolution_path_(){
    small_resolution_path_.clear();

    int path_size = path_.size();
    for(int i = 0; i < path_size - 1; i++){
        // 将世界坐标转为栅格索引
        Eigen::Vector2i start = map_util_->coord2gridIndex(path_[i]);
        Eigen::Vector2i end = map_util_->coord2gridIndex(path_[i+1]);
        // Bresenham 直线光栅化
        std::vector<Eigen::Vector2i> line = getGridsBetweenPoints2D(start, end);
        // 将新线段追加到高分辨率路径末尾
        small_resolution_path_.insert(small_resolution_path_.end(), line.begin(), line.end());
    }

}

// ======================== 路径发布 ========================

/**
 * @brief 将路径发布为 ROS nav_msgs::Path 消息
 *
 * 转换: Eigen::Vector2d -> geometry_msgs::PoseStamped
 * 所有路径点发布的 z 坐标为 0（2D 路径）
 */
void JPSPlanner::pubPath(const std::vector<Eigen::Vector2d> &path, const ros::Publisher &pub){
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "world";                   // 使用 world 坐标系
    for (const auto &it : path) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = it(0);
        pose.pose.position.y = it(1);
        pose.pose.position.z = 0;                          // 2D 路径，z = 0
        path_msg.poses.push_back(pose);
    }
    pub.publish(path_msg);
}

// ======================== 路径简化（去除冗余拐角点） ========================

/**
 * @brief 贪心路径简化: 移除可被直线连接替代的中间拐角点
 *
 * 算法详细说明:
 *
 * 给定路径点序列 p[0], p[1], ..., p[n]:
 *
 * 初始化:
 *   optimized_path = [p[0]]
 *   prev_pose = p[0]
 *   cost1 = dist(p[0], p[1]) if collision_free(p[0], p[1]) else INF
 *
 * 对 i = 1 到 n-2:
 *   cost2 = dist(p[i], p[i+1]) if collision_free(p[i], p[i+1]) else INF
 *   cost3 = dist(prev_pose, p[i+1]) if collision_free(prev_pose, p[i+1]) else INF
 *
 *   if cost3 < cost1 + cost2:
 *     // 跳过 p[i]: 从 prev_pose 直接到 p[i+1] 更短
 *     cost1 = cost3
 *   else:
 *     // 保留 p[i]: 这是必要的转折点
 *     optimized_path.push_back(p[i])
 *     cost1 = dist(p[i], p[i+1]) if collision_free else INF
 *     prev_pose = p[i]
 *
 * 最后:
 *   optimized_path.push_back(p[n])  // 总是保留终点
 *
 * 时间复杂度: O(n * m) 其中 n 是路径点数，m 是每个线段检查的栅格数
 *
 * 该算法与 Ramer-Douglas-Peucker 类似但方向相反:
 * RDP 从整体到局部递归细化，而本算法从局部到全局贪心前进。
 * 贪心方法的优势是不需要递归，内存效率高，适合在线规划。
 */
std::vector<Eigen::Vector2d> JPSPlanner::removeCornerPts(const std::vector<Eigen::Vector2d> &path) {
    if (path.size() < 2)                                 // 路径太短，无需简化
        return path;

    // 去除之字形路径段 (cut zigzag segment)
    std::vector<Eigen::Vector2d> optimized_path;
    Eigen::Vector2d pose1 = path[0];
    Eigen::Vector2d pose2 = path[1];
    Eigen::Vector2d prev_pose = pose1;                   // 上一个保留点
    optimized_path.push_back(pose1);                     // 始终保留起点
    double cost1, cost2, cost3;

    // 初始化: 计算首段的代价
    if (!checkLineCollision(pose1, pose2))
        cost1 = (pose1 - pose2).norm();                  // 无碰撞: 线段长度
    else
        cost1 = std::numeric_limits<double>::infinity(); // 有碰撞: 无穷大（不可行）

    // 遍历中间点
    for (unsigned int i = 1; i < path.size() - 1; i++) {
        pose1 = path[i];
        pose2 = path[i + 1];
        // 计算经 p[i] 到 p[i+1] 的代价
        if (!checkLineCollision(pose1, pose2))
            cost2 = (pose1 - pose2).norm();
        else
            cost2 = std::numeric_limits<double>::infinity();

        // 计算跳过 p[i] 直接连接的代价
        if (!checkLineCollision(prev_pose, pose2))
            cost3 = (prev_pose - pose2).norm();
        else
            cost3 = std::numeric_limits<double>::infinity();

        if (cost3 < cost1 + cost2)
            cost1 = cost3;                               // 跳过 p[i]（暂不确认，继续检查）
        else {
            // p[i] 是必要的转折点，保留之
            optimized_path.push_back(path[i]);
            cost1 = (pose1 - pose2).norm();              // 更新 cost1 为从 p[i] 出发的代价
            prev_pose = pose1;                           // 更新参考点
        }
    }

    optimized_path.push_back(path.back());               // 始终保留终点
    return optimized_path;
}

// ======================== 直线碰撞检测 ========================

/**
 * @brief 检查从 start 到 end 的线段是否与障碍物碰撞
 *
 * 方法: 使用 Bresenham 算法光栅化线段，对线段经过的每个栅格进行碰撞检测。
 * 碰撞检测使用 SDF 距离（isOccWithSafeDis），考虑安全距离膨胀。
 *
 * @return true 有碰撞; false 无碰撞
 */
bool JPSPlanner::checkLineCollision(const Eigen::Vector2d &start, const Eigen::Vector2d &end){
    std::vector<Eigen::Vector2i> line = getGridsBetweenPoints2D(
        map_util_->coord2gridIndex(start),
        map_util_->coord2gridIndex(end)
    );
    for(auto line_pt:line){
        if(map_util_->isOccWithSafeDis(line_pt, graph_search_->GetSafeDis())){
            return true;                                 // 检测到碰撞
        }
    }
    return false;                                        // 无碰撞
}

// ======================== Bresenham 直线光栅化算法 ========================

/**
 * @brief Bresenham 直线光栅化算法
 *
 * 经典 Bresenham 算法的简化版，用于在栅格地图中生成从 start 到 end
 * 的直线所经过的所有整数栅格坐标。
 *
 * 算法原理（仅整数运算，无浮点）:
 *   dx = |x1-x0|, dy = |y1-y0|
 *   sx = sign(x1-x0), sy = sign(y1-y0)
 *   err = dx - dy  // 初始误差
 *
 *   while 当前位置 != 终点:
 *     输出当前位置
 *     e2 = 2*err
 *     if e2 > -dy:  err -= dy; x += sx  // 水平步进
 *     if e2 < dx:   err += dx; y += sy  // 垂直步进
 *
 * 误差项 err 的数学意义:
 *   err = dx - dy 表示在水平和垂直方向上的累积步数差异。
 *   当 e2 > -dy 时，再水平走一步更接近理想直线
 *   当 e2 < dx 时，再垂直走一步更接近理想直线
 *   两者可同时满足（对角线步进）
 *
 * 时间复杂度: O(max(|dx|, |dy|))
 *
 * @param start 起点栅格索引
 * @param end   终点栅格索引
 * @return 直线经过的所有栅格坐标序列
 */
std::vector<Eigen::Vector2i> JPSPlanner::getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end){
    std::vector<Eigen::Vector2i> line;

    int dx = abs(end.x() - start.x());                   // x 方向绝对距离
    int dy = abs(end.y() - start.y());                   // y 方向绝对距离
    int sx = (start.x() < end.x()) ? 1 : -1;              // x 步进方向
    int sy = (start.y() < end.y()) ? 1 : -1;              // y 步进方向
    int err = dx - dy;                                    // 初始误差项

    double x0 = start.x();
    double y0 = start.y();

    while (true) {
        line.emplace_back(x0, y0);                        // 记录当前栅格
        if (x0 == end.x() && y0 == end.y()) break;        // 到达终点
        int e2 = 2 * err;                                 // 误差项的两倍（避免浮点）
        if (e2 > -dy) {                                   // 水平步进
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {                                    // 垂直步进
            err += dx;
            y0 += sy;
        }
    }

    return line;
}

// ======================== 运动学节点生成 (路径后处理主入口) ========================

/**
 * @brief 路径后处理主入口: 合并起始路径 + 简化 + 时间参数化
 *
 * 这是从外部调用的主要接口，接收 odometry 提供的当前状态和前段路径，
 * 将其与 JPS 搜索路径合并，然后生成完整的时空轨迹。
 *
 * 流程:
 * 1. 如果没有起始路径 (start_path 为空):
 *    - 直接使用 raw_path_ 进行简化和参数化
 * 2. 如果有起始路径 (从 odometry 或 ICM 预测得来):
 *    - 将 start_path 的点转为 2D 并插入到 raw_path_ 开头
 *    - 更新 start_state_ 为起始路径的第一点
 * 3. removeCornerPts() 简化合并后的路径
 * 4. getSampleTraj() 生成5D采样轨迹
 * 5. getTrajsWithTime() 时间参数化
 *
 * 合并起始路径的设计目的:
 * 由于规划不是瞬间完成的，在规划期间机器人已经沿着之前的轨迹
 * 前进了一段距离。start_path 包含了这段时间内机器人所处的位置，
 * 将此合并到新规划路径中可保证轨迹的连续性。
 *
 * @param start_path 外部起始路径段（3D点列，含z坐标通常为yaw）
 * @param if_forward 前进/后退标志
 * @param current_state_VAJ 当前 VAJ 状态
 * @param current_state_OAJ 当前 OAJ 状态
 */
void JPSPlanner::getKinoNodeWithStartPath(const std::vector<Eigen::Vector3d> &start_path, const bool if_forward, const Eigen::Vector3d &current_state_VAJ, const Eigen::Vector3d &current_state_OAJ){
    current_state_VAJ_ = current_state_VAJ;              // 保存当前 VAJ 状态
    current_state_OAJ_ = current_state_OAJ;              // 保存当前 OAJ 状态

    if(start_path.size() > 0){
      std::vector<Eigen::Vector2d> start_path_2d;
      for(auto pt:start_path){
        start_path_2d.push_back(pt.head(2));             // 提取 (x,y) 分量
        ROS_INFO_STREAM("start_path_3d: " << pt.transpose());
        start_path_2d.pop_back();                        // 注意: 这里 pop_back 会移除刚加入的元素
                                                         // 可能是一个 bug: 应删除此行或移动到循环外
      }
      // 将起始路径插入到 raw_path_ 开头
      raw_path_.insert(raw_path_.begin(), start_path_2d.begin(), start_path_2d.end());
      start_state_ = start_path.front();                 // 更新起始状态
    }

    // get_small_resolution_path_();
    path_ = removeCornerPts(raw_path_);                  // 贪心路径简化
    Unoccupied_path_ = path_;                            // 无障碍路径 = 简化后路径

    pubPath(path_, path_pub_);                           // 发布简化后路径
    getSampleTraj();                                     // 生成5D采样轨迹
    getTrajsWithTime();                                  // 梯形速度剖面时间参数化
}

// ======================== 5D 采样轨迹生成 ========================

/**
 * @brief 将几何路径转化为 5D 采样轨迹 [x, y, theta, dtheta, ds]
 *
 * 算法的核心思想:
 * 在路径上的每个关键点处，同时生成"到达点"（到达该位置）和"转向点"（在该位置转身），
 * 这两类点交替排列，使得位置移动和角度旋转在时间上分离。
 *
 * 5D 状态空间定义:
 *   x, y:    世界坐标位置
 *   theta:   偏航角（朝向）
 *   dtheta:  相对上一采样点的角度变化量（用于计算转向代价）
 *   ds:      相对上一采样点的弧长增量（用于计算前进代价）
 *
 * 详细流程:
 * 1. 起点处生成 3 个候选状态:
 *    - 原始朝向 start_state_.z()
 *    - 从起点指向第一个路径点的朝向
 *    - 从起点指向第一个路径点的反方向（+PI，用于允许倒车）
 *    这样为后端优化器提供了多种初始朝向选择
 * 2. 对路径中每个中间点 p[i] (i=1..n-2):
 *    a. 插入"到达点": 位置 = p[i]，朝向 = 前一点的朝向，ds = 累计弧长
 *    b. 插入"转向点": 位置 = p[i]，朝向 = 指向 p[i+1] 的方向，
 *       dtheta = 新朝向 - 旧朝向
 * 3. 路径末点: 插入到达点和目标朝向转向点
 *
 * 这种交替插入策略使得速度剖面可以独立处理转向（原地旋转时速度为0）
 * 和前进（保持恒定朝向时角度变化为0）。
 */
void JPSPlanner::getSampleTraj(){
    Unoccupied_sample_trajs_.clear();
    double cur_theta;

    // 构造 5D 状态向量 [x, y, theta, dtheta, ds]
    Eigen::VectorXd state5d;
    state5d.resize(5);

    // [0] 起点状态: theta = start yaw, dtheta = 0, ds = 0
    state5d << start_state_.x(), start_state_.y(), start_state_.z(), 0, 0;
    Unoccupied_sample_trajs_.push_back(state5d);

    // [1] 起点转向候选1: theta = 从起点指向第一个路径点的方向
    cur_theta = atan2(Unoccupied_path_[1].y() - Unoccupied_path_[0].y(),
                       Unoccupied_path_[1].x() - Unoccupied_path_[0].x());
    normalizeAngle(start_state_.z(), cur_theta);         // 规范化到参考角度附近
    state5d << start_state_.x(), start_state_.y(), cur_theta , cur_theta - start_state_.z(), 0;
    Unoccupied_sample_trajs_.push_back(state5d);

    // [2] 起点转向候选2: theta = 从起点指向第一个路径点的反方向（向后行驶）
    cur_theta = atan2(Unoccupied_path_[0].y() - Unoccupied_path_[1].y(),
                       Unoccupied_path_[0].x() - Unoccupied_path_[1].x()) + M_PI;
    normalizeAngle(start_state_.z(), cur_theta);
    state5d << start_state_.x(), start_state_.y(), cur_theta , cur_theta - start_state_.z(), 0;
    Unoccupied_sample_trajs_.push_back(state5d);         // 下标 2

    // 遍历路径中间点，交替插入"到达点"和"转向点"
    int path_size = Unoccupied_path_.size();
    Eigen::VectorXd pt;
    for(int i = 1; i<path_size-1; i++){
        pt = Unoccupied_path_[i];

        // A. 插入"到达点": 到达路径点 p[i]，朝向不变
        // ds = 从前一点到此点的欧几里得距离
        state5d << pt.x(), pt.y(), Unoccupied_sample_trajs_.back()[2], 0,
            sqrt(pow(pt.x() - Unoccupied_sample_trajs_.back()[0], 2) +
                 pow(pt.y() - Unoccupied_sample_trajs_.back()[1], 2));
        Unoccupied_sample_trajs_.push_back(state5d);

        // B. 插入"转向点": 在 p[i] 处转向，朝向指向下一个路径点
        cur_theta = atan2(Unoccupied_path_[i+1].y() - Unoccupied_path_[i].y(),
                           Unoccupied_path_[i+1].x() - Unoccupied_path_[i].x());
        normalizeAngle(Unoccupied_sample_trajs_.back()[2], cur_theta);
        state5d << pt.x(), pt.y(), cur_theta,
            cur_theta - Unoccupied_sample_trajs_.back()[2], 0;  // dtheta = 角度变化量
        Unoccupied_sample_trajs_.push_back(state5d);

    }

    // 路径末点: 到达点
    pt = Unoccupied_path_.back();
    state5d << pt.x(), pt.y(), Unoccupied_sample_trajs_.back()[2], 0,
        sqrt(pow(pt.x() - Unoccupied_sample_trajs_.back()[0], 2) +
             pow(pt.y() - Unoccupied_sample_trajs_.back()[1], 2));
    Unoccupied_sample_trajs_.push_back(state5d);

    // 路径末点: 转向到目标朝向
    cur_theta = end_state_.z();
    normalizeAngle(Unoccupied_sample_trajs_.back()[2], cur_theta);
    state5d << pt.x(), pt.y(), cur_theta,
        cur_theta - Unoccupied_sample_trajs_.back()[2], 0;
    Unoccupied_sample_trajs_.push_back(state5d);
}

// ======================== 梯形速度剖面时间参数化 ========================

/**
 * @brief 使用梯形速度剖面对采样轨迹进行时间参数化
 *
 * 核心算法:
 *
 * 第一步: 计算加权路径长度
 *   遍历 Unoccupied_sample_trajs_ 中的每个采样点:
 *     weighted_length = yaw_weight * |dtheta| + distance_weight * |ds|
 *   累计所有采样点的加权长度得到总加权路径长度 AllWeightingPathLength_
 *   累计未加权的弧长得到真实路径长度 AllPathLength
 *
 *   加权长度的物理含义:
 *   将角度变化转换为等效的空间长度，使得速度剖面在需要转向时自动"拉长"路径，
 *   从而在转向段产生更低的规划速度。
 *
 * 第二步: 轨迹截断
 *   如果累计弧长超过 trajCutLength_:
 *     在最后一个完整的采样点区间内线性插值截断点
 *     将截断点状态存入 cut_Unoccupied_sample_trajs_
 *
 * 第三步: 计算总时间
 *   evaluateDuration(AllWeightingPathLength_, current_v, 0, max_vel_, max_acc_)
 *
 * 第四步: 时间均匀采样
 *   delta_t = max(total_time / mintrajNum_, sampletime_)
 *   for t = delta_t, 2*delta_t, ... , total_time - epsilon:
 *     1. evaluateLength(t, ...) 计算当前时间对应的加权弧长
 *     2. 在加权弧长空间中查找对应的路径段
 *     3. 在该段内线性插值位置 (x, y) 和朝向 (yaw)
 *     4. 将 [yaw, arc_length, t] 存入 UnOccupied_traj_pts
 *     5. 将 [x, y, yaw] 存入 UnOccupied_positions
 *
 * 第五步: 生成 FlatTrajData 输出
 *   将起止状态、采样轨迹、边界条件封装到 flat_traj_ 中，
 *   供后端 NMPC 或轨迹优化器使用。
 */
void JPSPlanner::getTrajsWithTime(){
    cut_Unoccupied_sample_trajs_.clear();

    // 存储每个采样点的偏航角和弧长信息
    std::vector<double> Unoccupied_thetas;
    std::vector<double> Unoccupied_pathlengths;          // 未加权弧长累计
    std::vector<double> Unoccupied_Weightpathlengths;    // 加权弧长累计

    double Unoccupied_AllWeightingPathLength_ = 0;       // 累积加权路径长度
    double Unoccupied_AllPathLength = 0;                 // 累积真实路径长度

    bool if_cut = false;                                  // 轨迹截断标志
    Eigen::Vector3d cut_state = Unoccupied_sample_trajs_.back().head(3);  // 截断点状态

    // 初始化: 第一个采样点
    int PathNodeNum = Unoccupied_sample_trajs_.size();
    cut_Unoccupied_sample_trajs_.push_back(Unoccupied_sample_trajs_[0]);
    Unoccupied_thetas.push_back(Unoccupied_sample_trajs_[0][2]);       // theta
    Unoccupied_pathlengths.push_back(0);                               // s = 0
    Unoccupied_Weightpathlengths.push_back(0);                         // weighted s = 0

    // 遍历所有采样点，计算加权弧长并检查是否需要截断
    int pathnodeindex = 1;
    for(; pathnodeindex<PathNodeNum&&!if_cut; pathnodeindex++){
        Eigen::VectorXd pathnode = Unoccupied_sample_trajs_[pathnodeindex];
        // 检查是否超过截断长度（且当前点有实际弧长增量，非纯旋转点）
        if(Unoccupied_AllPathLength + fabs(pathnode[4]) >= trajCutLength_ && pathnode[4] != 0){
            if_cut = true;

            // 在最后一段内线性插值截断点
            Eigen::Vector3d former_state = Unoccupied_sample_trajs_[pathnodeindex-1].head(3);
            // 截断点位置 = 前一点 + (前一点->当前点) * (剩余长度 / 当前段长度)
            cut_state = former_state + (pathnode.head(3) - former_state) *
                (trajCutLength_ - Unoccupied_AllPathLength) / fabs(pathnode[4]);

            // 构造截断点 5D 状态
            Eigen::VectorXd state5d; state5d.resize(5);
            state5d<<cut_state.x(), cut_state.y(), cut_state.z(),
                (trajCutLength_ - Unoccupied_AllPathLength)/fabs(pathnode[4]) * pathnode[3],
                trajCutLength_ - Unoccupied_AllPathLength;
            cut_Unoccupied_sample_trajs_.push_back(state5d);
            Unoccupied_thetas.push_back(state5d[2]);
            Unoccupied_AllPathLength += state5d[4];                  // 更新累计弧长
            Unoccupied_pathlengths.push_back(Unoccupied_AllPathLength);
            // 计算加权路径长度: yaw_cost + distance_cost
            Unoccupied_AllWeightingPathLength_ += yaw_weight_ * abs(state5d[3]) + distance_weight_ * abs(state5d[4]);
            Unoccupied_Weightpathlengths.push_back(Unoccupied_AllWeightingPathLength_);

            PathNodeNum = cut_Unoccupied_sample_trajs_.size();        // 更新节点数
            break;
        }
        // 正常点: 加入截断列表
        cut_Unoccupied_sample_trajs_.push_back(pathnode);
        Unoccupied_thetas.push_back(pathnode[2]);
        Unoccupied_AllPathLength += pathnode[4];                     // 累加弧长
        Unoccupied_pathlengths.push_back(Unoccupied_AllPathLength);
        // 加权路径长度 = yaw_weight * |dtheta| + distance_weight * |ds|
        Unoccupied_AllWeightingPathLength_ += yaw_weight_ * abs(pathnode[3]) + distance_weight_ * abs(pathnode[4]);
        Unoccupied_Weightpathlengths.push_back(Unoccupied_AllWeightingPathLength_);
    }

    // 梯形速度剖面: 计算总耗时
    // 从当前速度 current_state_VAJ_.x() 开始，到 0 速度结束
    double totalTrajTime_ = evaluateDuration(Unoccupied_AllWeightingPathLength_,
        current_state_VAJ_.x(), 0.0, max_vel_, max_acc_);

    // 存储采样结果
    std::vector<Eigen::Vector3d> Unoccupied_traj_pts;       // [yaw, s, t] 采样轨迹点
    std::vector<Eigen::Vector3d> Unoccupied_positions;      // [x, y, yaw] 采样位置

    double Unoccupied_totalTrajTime_ = totalTrajTime_;
    double Unoccupied_sampletime;
    int Unoccupied_PathNodeIndex = 1;                       // 当前处理到的路径节点索引

    // 计算采样时间步长: 保证至少 mintrajNum_ 个采样点
    Unoccupied_sampletime = Unoccupied_totalTrajTime_ /
        std::max(int(Unoccupied_totalTrajTime_ / sampletime_ + 0.5), mintrajNum_);

    PathNodeNum = cut_Unoccupied_sample_trajs_.size();
    double tmparc = 0;

    // 时间均匀采样循环
    for(double samplet = Unoccupied_sampletime;
        samplet<Unoccupied_totalTrajTime_-1e-3;
        samplet+=Unoccupied_sampletime){

        // 使用梯形速度剖面反算: 时刻 t 对应的加权弧长
        double arc = evaluateLength(samplet, Unoccupied_AllWeightingPathLength_,
            Unoccupied_totalTrajTime_, current_state_VAJ_.x(), 0.0, max_vel_, max_acc_);

        // 在加权弧长空间中查找对应的路径段
        for (int k = Unoccupied_PathNodeIndex; k<PathNodeNum; k++){
            Eigen::VectorXd pathnode = cut_Unoccupied_sample_trajs_[k];
            Eigen::VectorXd prepathnode = cut_Unoccupied_sample_trajs_[k-1];
            tmparc = Unoccupied_Weightpathlengths[k];

            if(tmparc >= arc){                               // 找到所在段
                Unoccupied_PathNodeIndex = k;
                double l1 = tmparc-arc;                      // 当前节点之后的剩余加权弧长
                double l = Unoccupied_Weightpathlengths[k]-Unoccupied_Weightpathlengths[k-1];
                                                             // 该段的加权弧长总量
                // 线性插值: 弧长 (s) 和朝向 (yaw)
                // interp_s: 在真实弧长空间的插值
                double interp_s = Unoccupied_pathlengths[k-1] + (l-l1)/l*(pathnode[4]);
                // interp_yaw: 在朝向空间的插值
                double interp_yaw = cut_Unoccupied_sample_trajs_[k-1][2] + (l-l1)/l*(pathnode[3]);

                // 记录采样点: [yaw, 弧长, 时间]
                Unoccupied_traj_pts.emplace_back(interp_yaw, interp_s, samplet);

                // 线性插值世界坐标 (x, y)
                double interp_x = l1/l*prepathnode[0] + (l-l1)/l*(pathnode[0]);
                double interp_y = l1/l*prepathnode[1] + (l-l1)/l*(pathnode[1]);
                Unoccupied_positions.emplace_back(interp_x, interp_y, interp_yaw);
                break;
            }
        }
    }

    // 构造输出数据结构 FlatTrajData
    Eigen::MatrixXd startS;
    Eigen::MatrixXd endS;
    startS.resize(2,3);                                    // 2 行 (位置行 + 速度行) x 3 列
    endS.resize(2,3);

    // 起始状态: 第0列 = [yaw, 弧长]
    Eigen::Vector2d startP(cut_Unoccupied_sample_trajs_[0][2], 0);
    Eigen::Vector2d finalP(cut_Unoccupied_sample_trajs_[PathNodeNum-1][2],
        Unoccupied_pathlengths[PathNodeNum-1]);

    startS.col(0) = startP;                                // 起始 pva 位置列
    startS.block(0,1,1,2) = current_state_OAJ_.transpose().head(2);  // OAJ: 角速度, 角加速度
    startS.block(1,1,1,2) = current_state_VAJ_.transpose().head(2);  // VAJ: 速度, 加速度

    endS.col(0) = finalP;                                  // 终止 pva 位置列
    endS.col(1).setZero();                                 // 终止速度设为 0（停止）
    endS.col(2).setZero();                                 // 终止加速度设为 0

    // 填充 FlatTrajData 输出
    flat_traj_.UnOccupied_traj_pts = Unoccupied_traj_pts;
    flat_traj_.UnOccupied_initT = Unoccupied_sampletime;   // 采样时间步长
    flat_traj_.UnOccupied_positions = Unoccupied_positions;

    flat_traj_.start_state = startS;
    flat_traj_.final_state = endS;
    flat_traj_.start_state_XYTheta = start_state_;
    // flat_traj_.final_state_XYTheta = end_state_;
    flat_traj_.if_cut = if_cut;
    flat_traj_.final_state_XYTheta = cut_state;            // 可能被截断后的终点

    // flat_traj_.printFlatTrajData();
}

// ======================== 角度规范化 ========================

/**
 * @brief 规范化角度到参考角度的 [-PI, +PI] 范围内
 *
 * 确保 (ref_angle - angle) 落在 [-PI, PI] 区间内，
 * 通过反复加减 2*PI 实现。这避免了由于角度周期性
 * （如 359 度和 1 度仅差 2 度而非 358 度）导致的歧义问题。
 *
 * @param ref_angle 参考角度（不变）
 * @param angle 要规范化的角度（引用传递，原地修改）
 */
void JPSPlanner::normalizeAngle(const double &ref_angle, double &angle){
    while(ref_angle - angle > M_PI){                      // 角度太小，向右旋
        angle += 2*M_PI;
    }
    while(ref_angle - angle < -M_PI){                     // 角度太大，向左旋
        angle -= 2*M_PI;
    }
}

// ======================== 梯形速度剖面: 总耗时计算 ========================

/**
 * @brief 梯形速度剖面: 根据路径长度计算总耗时
 *
 * 数学推导:
 *
 * 设起始速度 v_s, 终止速度 v_e, 最大速度 v_max, 最大加速度 a_max
 *
 * 临界长度 L_crit:
 *   L_crit = (v_max^2 - v_s^2)/(2*a_max) + (v_max^2 - v_e^2)/(2*a_max)
 *   这是"加速到 v_max 立刻开始减速"所需的最小长度
 *
 * 情况1: L >= L_crit (可达到最大速度)
 *   加速段时间: t_a = (v_max - v_s)/a_max
 *   减速段时间: t_d = (v_max - v_e)/a_max
 *   匀速段长度: L_const = L - (v_max^2-v_s^2)/(2a) - (v_max^2-v_e^2)/(2a)
 *   匀速段时间: t_c = L_const/v_max
 *   总时间: T = t_a + t_c + t_d
 *
 * 情况2: L < L_crit (达不到最大速度，三角形剖面)
 *   峰值速度 v_m = sqrt(0.5*(v_s^2 + v_e^2 + 2*a_max*L))
 *   加速时间: t_a = (v_m - v_s)/a_max
 *   减速时间: t_d = (v_m - v_e)/a_max
 *   总时间: T = t_a + t_d
 *
 * @return 总耗时（秒）
 */
double JPSPlanner::evaluateDuration(const double &length, const double &startV, const double &endV, const double &maxV, const double &maxA){
  double critical_len;                                   // 临界长度
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(maxV,2);
  // 安全检查: 如果当前速度超过最大速度，限制为最大速度
  if(startV>maxV){
    startv2 = maxv2;
  }
  if(endV>max_vel_){
    endv2 = maxv2;
  }

  // 临界长度 = 加速到v_max的距离 + 从v_max减速到v_end的距离
  critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);

  if(length>=critical_len){                              // 可达到最大速度
    return (maxV-startV)/maxA+(maxV-endV)/maxA+(length-critical_len)/maxV;
  }
  else{                                                  // 达不到最大速度，三角形剖面
    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*length));  // 峰值速度
    return (tmpv-startV)/maxA + (tmpv-endV)/maxA;
  }
}



/**
 * @brief 梯形速度剖面: 根据当前时间计算行进的弧长
 *
 * 这是 evaluateDuration 的逆函数（之一），给定时刻 t 计算对应的行进距离 s(t)。
 *
 * 情况1: L >= L_crit (三段式: 加速-匀速-减速)
 *   t_a = (v_max - v_start)/a_max
 *   t_v = t_a + (L - L_crit)/v_max                     // 匀速结束时刻
 *   if t <= t_a:           s = v_start*t + 0.5*a*t^2               // 加速段
 *   else if t <= t_v:      s = s_a + v_max*(t - t_a)               // 匀速段
 *   else:                  s = s_v + v_max*(t-t_v) - 0.5*a*(t-t_v)^2  // 减速段
 *
 * 情况2: L < L_crit (三角形: 加速-减速)
 *   v_peak = sqrt(0.5*(v_s^2+v_e^2+2*a*L))
 *   t_a = (v_peak - v_start)/a_max
 *   if t <= t_a:           s = v_start*t + 0.5*a*t^2               // 加速段
 *   else:                  s = s_a + v_peak*(t-t_a) - 0.5*a*(t-t_a)^2 // 减速段
 *
 * @return 从起点到时刻 t 的行进距离
 */
double JPSPlanner::evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA){
  // std::cout<<"curt: "<<curt<<"  locallength: "<<locallength<<"  localtime: "<<localtime<<"  startV: "<<startV<<"  endV: "<<endV<<"  maxV: "<<maxV<<"  maxA: "<<maxA<<std::endl;
  double critical_len;
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(maxV,2);
  if(startV>maxV){
    startv2 = maxv2;
  }
  if(endV>max_vel_){
    endv2 = maxv2;
  }

  critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);

  if(locallength>=critical_len){                         // 可达到最大速度 -> 三段式
    double t1 = (maxV-startV)/maxA;                      // 加速结束时刻
    double t2 = t1+(locallength-critical_len)/maxV;       // 匀速结束时刻（即减速开始时刻）
    if(curt<=t1){                                        // 加速段
      return startV*curt + 0.5*maxA*pow(curt,2);
    }
    else if(curt<=t2){                                   // 匀速段
      return startV*t1 + 0.5*maxA*pow(t1,2)+(curt-t1)*maxV;
    }
    else{                                                // 减速段
      return startV*t1 + 0.5*maxA*pow(t1,2) + (t2-t1)*maxV + maxV*(curt-t2)-0.5*maxA*pow(curt-t2,2);
    }
  }
  else{                                                  // 达不到最大速度 -> 三角形
    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*locallength));
    double tmpt = (tmpv-startV)/maxA;                    // 加速结束时刻（= 减速开始时刻）
    if(curt<=tmpt){                                      // 加速段
      return startV*curt+0.5*maxA*pow(curt,2);
    }
    else{                                                // 减速段
      return startV*tmpt+0.5*maxA*pow(tmpt,2) + tmpv*(curt-tmpt)-0.5*maxA*pow(curt-tmpt,2);
    }
  }
}

/**
 * @brief 梯形速度剖面: 根据当前时间计算速度
 *
 * v(t) 曲线:
 * 加速段: v(t) = v_start + a_max * t
 * 匀速段: v(t) = v_max
 * 减速段: v(t) = v_max - a_max * (t - t_decel_start)
 *
 * 三角形剖面:
 * 加速段: v(t) = v_start + a_max * t
 * 减速段: v(t) = v_peak - a_max * (t - t_peak)
 */
double JPSPlanner::evaluateVel(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA){
  double critical_len;
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(maxV,2);
  if(startV>maxV){
    startv2 = maxv2;
  }
  if(endV>max_vel_){
    endv2 = maxv2;
  }

  critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);

  if(locallength>=critical_len){                         // 三段式
    double t1 = (maxV-startV)/maxA;                      // 加速结束时刻
    double t2 = t1+(locallength-critical_len)/maxV;       // 减速开始时刻
    if(curt<=t1){
      return startV + maxA*curt;                         // v = v0 + a*t
    }
    else if(curt<=t2){
      return maxV;                                       // v = v_max（恒定）
    }
    else{
      return maxV - maxA*(curt-t2);                      // v = v_max - a*(t-t2)
    }
  }
  else{                                                  // 三角形
    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*locallength));
    double tmpt = (tmpv-startV)/maxA;
    if(curt<=tmpt){
      return startV + maxA*curt;
    }
    else{
      return tmpv - maxA * (curt - tmpt);
    }
  }
}

/**
 * @brief 梯形速度剖面: 根据位置计算到达时间
 *
 * 这是 evaluateLength 的完全逆函数: 给定位置 s，计算到达该位置的时刻 t(s)。
 *
 * 加速段: t = (sqrt(v0^2 + 2*a*s) - v0) / a
 *   (由 s = v0*t + 0.5*a*t^2 解得)
 *
 * 匀速段: t = t_a + (s - s_a) / v_max
 *
 * 减速段: t = t_dec_start + (v_max - sqrt(v_max^2 - 2*a*(s-s_dec_start))) / a
 *   (由 s = s_dec_start + v_max*(t-t_dec_start) - 0.5*a*(t-t_dec_start)^2 解得)
 */
double JPSPlanner::evaluteTimeOfPos(const double &pos, const double &locallength, const double &startV, const double &endV, const double &maxV, const double &maxA){
  double critical_len;
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(maxV,2);
  double localpos = pos;
  if(startV>maxV){
    startv2 = maxv2;
  }
  if(endV>max_vel_){
    endv2 = maxv2;
  }
  if(pos>locallength){                                   // 边界保护: 位置不能超过总长度
    localpos = locallength;
  }

  critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
  if(locallength>=critical_len){
    // 加速段终点位置
    double s1 = (maxv2-startv2)/maxA/2.0;
    if(localpos < s1){                                   // 在加速段
      return (sqrt(startV*startV + 2*maxA*localpos)-startV)/maxA;
    }
    // 减速段起点位置
    double s2 = locallength - (maxv2-endv2)/maxA/2.0;
    if(localpos < s2){                                   // 在匀速段
      return (maxV - startV)/maxA + (localpos-s1)/maxV;
    }
    else{                                                // 在减速段
        return (maxV - startV)/maxA + (s2-s1)/maxV +
            (maxV - sqrt(maxv2-2.0*maxA*(localpos-s2)))/maxA;
    }
  }
  else{                                                  // 三角形剖面
    double v_m = sqrt(0.5*(startv2+endv2+2*maxA*locallength));
    if(localpos < (v_m*v_m - startv2)/2.0/maxA){          // 在加速段
      return (sqrt(startV*startV + 2*maxA*localpos)-startV)/maxA;
    }
    else{                                                // 在减速段
      double rest_s = pos - (v_m*v_m - startv2)/2.0/maxA;
      return (v_m - startV)/maxA + (v_m - sqrt(v_m*v_m - 2*maxA*rest_s))/maxA;
    }
  }
}

// ======================== 单点碰撞检测 ========================

/**
 * @brief 检查单个位置是否安全（用于 JPS 规划过程中的快速碰撞检测）
 *
 * @return true 如果 SDF 距离 > safe_dis_（安全），false 如果太接近障碍物
 */
bool JPSPlanner::JPS_check_if_collision(const Eigen::Vector2d &pos){
  return map_util_->getDistanceReal(pos) > safe_dis_;
}