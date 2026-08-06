/**
 * @file jps_planner.h
 * @brief 前端规划器 — 基于 JPS 路径搜索 + 梯形速度剖面时间参数化的完整路径规划
 *
 * 本模块实现了一个完整的前端轨迹规划流水线:
 * 1. JPS 跳点搜索: 在栅格地图中快速生成几何路径（稀疏的关键点序列）
 * 2. 路径简化: removeCornerPts() 通过直线连接减少冗余的之字形转折点
 * 3. 时间参数化: 使用梯形速度剖面（trapezoidal velocity profile）对路径进行时间分配
 *    - 梯形速度剖面假设: 加速段（恒定加速度）-> 匀速段（最大速度）-> 减速段（恒定减速度）
 *    - 当路径长度不足以达到最大速度时，退化为三角形速度剖面（仅加速-减速）
 * 4. 均匀采样: 以固定时间间隔沿时间参数化后的轨迹采样，生成平坦输出轨迹
 *
 * 关键公式（梯形速度剖面）:
 *   临界长度 L_crit = (v_max^2 - v_start^2)/(2*a_max) + (v_max^2 - v_end^2)/(2*a_max)
 *   如果 L >= L_crit: 可达到最大速度，走加速-匀速-减速三段
 *   如果 L < L_crit:  达不到最大速度，走加速-减速两段（三角形）
 *
 * 输出数据类型 FlatTrajData:
 *   包含等时间采样的 [yaw, arc_length, timestamp] 三元组，传递给后端 NMPC 优化器
 */

#ifndef JPS_PLANNER_H
#define JPS_PLANNER_H

#include <plan_env/sdf_map.h>
#include <front_end/jps_planner/graph_search.h>
#include <front_end/traj_representation.h>

#include <nav_msgs/Path.h>                // ROS 标准路径消息
#include <visualization_msgs/Marker.h>    // ROS 可视化标记消息
#include <tf/tf.h>                        // TF 坐标变换库

namespace JPS
{

    /**
     * @class JPSPlanner
     * @brief 前端规划器: JPS 搜索 + 路径后处理 + 时间参数化
     *
     * 主要功能:
     * - plan(): 执行 JPS 全局路径搜索，生成从起点到目标的几何路径
     * - getKinoNodeWithStartPath(): 对原始路径进行后处理（简化、采样）生成运动学可用轨迹
     * - evaluateDuration/evaluateLength/evaluateVel: 梯形速度剖面的时间/位置/速度求解
     *
     * 工作流程:
     *   plan() -> getKinoNodeWithStartPath() -> getSampleTraj() -> getTrajsWithTime()
     *    ^搜索          ^简化+拼接                ^采样5D状态         ^梯形速度剖面
     */
    class JPSPlanner
    {
        private:

            // --- 参数配置（从 ROS 参数服务器加载） ---
            double safe_dis_;           // 安全距离：JPS 搜索时障碍物膨胀半径（单位: 米）
            double max_jps_dis_;        // JPS 搜索最大距离限制
            double distance_weight_;    // 弧长权重：用于将路径弧长映射到代价（加权路径长度）
            double yaw_weight_;         // 偏航角变化权重：用于惩罚频繁转向
            double trajCutLength_;      // 轨迹截断长度：输出轨迹的最大弧长（单位: 米）
            double max_vel_;            // 最大线速度 v_max（单位: m/s）
            double max_acc_;            // 最大线加速度 a_max（单位: m/s^2）
            double max_omega_;          // 最大角速度 omega_max（单位: rad/s）
            double max_domega_;         // 最大角加速度（单位: rad/s^2）
            double sampletime_;         // 轨迹采样时间间隔（单位: 秒）
            int mintrajNum_;            // 最小轨迹采样点数（保证输出轨迹有足够的分辨率）

            // --- 运行状态 ---
            Eigen::Vector3d start_state_;    // 起点状态 [x, y, yaw]（world坐标系）
            Eigen::Vector3d current_state_VAJ_;  // 当前 VAJ 状态 [速度, 加速度, 加加速度]
            Eigen::Vector3d current_state_OAJ_;  // 当前 OAJ 状态 [角速度, 角加速度, 角加加速度]
            Eigen::Vector3d end_state_;      // 目标状态 [x, y, yaw]

            bool if_first_point_cut_;        // 标记首点是否被截断

            std::shared_ptr<SDFmap> map_util_;      // SDF 地图工具类
            std::shared_ptr<GraphSearch> graph_search_;  // JPS/A* 图搜索引擎

            int status_;                         // 规划状态（-1 = 失败）
            std::vector<Eigen::Vector2d> raw_path_;   // JPS 搜索的原始路径（栅格坐标转世界坐标）
            std::vector<Eigen::Vector2d> path_;        // 简化后的路径（去除之字形转折）

            std::vector<Eigen::Vector2d> Unoccupied_path_;   // 无障碍路径（简化后的最终几何路径）

            /**
             * @brief 5D 采样轨迹状态: [x, y, theta, dtheta, ds]
             * x, y: 位置坐标
             * theta: 偏航角（朝向）
             * dtheta: 相邻采样点之间的角度变化量（用于计算转向代价）
             * ds: 相邻采样点之间的弧长增量（用于计算前进代价）
             */
            std::vector<Eigen::VectorXd> Unoccupied_sample_trajs_;     // 完整采样轨迹
            std::vector<Eigen::VectorXd> cut_Unoccupied_sample_trajs_; // 截断后的采样轨迹

            std::vector<Eigen::Vector2i> small_resolution_path_;  // 高分辨率栅格路径（用于碰撞检测）

            // --- ROS 发布器 ---

        public:

            FlatTrajData flat_traj_;            // 输出的平坦轨迹数据

            double jps_truncation_time_;        // JPS 截断时间

            /**
             * @brief JPS 规划器构造函数
             * @param map 共享的 SDF 地图
             * @param nh ROS 节点句柄（用于加载参数和创建发布器）
             */
            JPSPlanner(std::shared_ptr<SDFmap> map, const ros::NodeHandle &nh);

            /**
             * @brief 执行 JPS 路径规划
             *
             * 流程:
             * 1. 将世界坐标起止点转为栅格索引
             * 2. 根据起点和目标点的 SDF 距离动态调整安全距离
             *    （避免起止点因为太靠近障碍物而被标记为不可达）
             * 3. 调用 GraphSearch::plan() 进行 JPS 搜索
             * 4. 将搜索结果的栅格路径转回世界坐标，得到 raw_path_
             *
             * @param start 起点 [x, y, yaw]
             * @param goal  目标 [x, y, yaw]
             * @return true 找到路径; false 搜索失败
             */
            bool plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);

            /// 获取高分辨率栅格路径（将简化后的路径插值为连续的栅格序列）
            void get_small_resolution_path_();

            /// 发布路径为 ROS nav_msgs::Path 消息
            void pubPath(const std::vector<Eigen::Vector2d> &path, const ros::Publisher &pub);

            /**
             * @brief 移除路径中的冗余转折点（路径简化）
             *
             * 算法原理:
             * 对于路径点序列 p[0], p[1], ..., p[n]：
             * 1. 初始化: prev_pose = p[0], 将 p[0] 加入优化路径
             * 2. 对每个中间点 p[i] (i=1..n-1):
             *    - cost1 = 如果 p[i-1]到p[i] 无碰撞则取欧几里得距离，否则为无穷
             *    - cost2 = 如果 p[i]到p[i+1] 无碰撞则取欧几里得距离，否则为无穷
             *    - cost3 = 如果 prev_pose到p[i+1] 无碰撞则取欧几里得距离，否则为无穷
             * 3. 如果 cost3 < cost1+cost2: 说明 p[i] 可以被跳过（直线连接更优）
             *    否则: p[i] 是必要的转折点，保留之
             *
             * 这本质上是一种贪心的路径简化，类似 Ramer-Douglas-Peucker 算法的变体。
             *
             * @param path 原始路径（JPS 输出的之字形路径）
             * @return 优化后的简化路径
             */
            std::vector<Eigen::Vector2d> removeCornerPts(const std::vector<Eigen::Vector2d> &path);

            /// 检查从 start 到 end 的线段是否与障碍物碰撞
            bool checkLineCollision(const Eigen::Vector2d &start, const Eigen::Vector2d &end);

            /**
             * @brief Bresenham 直线光栅化算法
             *
             * 使用经典 Bresenham 算法在栅格地图中生成从 start 到 end 的
             * 直线所经过的所有栅格坐标。用于线段碰撞检测。
             *
             * 算法特点: 仅使用整数加减法和比较，无需浮点运算，效率极高。
             *
             * @param start 起点栅格索引
             * @param end   终点栅格索引
             * @return 直线经过的所有栅格点序列
             */
            std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end);

            /**
             * @brief 从外部接收起始路径并生成运动学可用轨迹
             *
             * 完整流水线:
             * 1. 合并外部起始路径 (start_path) 和 JPS 搜索路径 (raw_path_)
             * 2. removeCornerPts() 简化路径去除冗余点
             * 3. getSampleTraj() 将几何路径转化为5D采样状态 [x,y,theta,dtheta,ds]
             * 4. getTrajsWithTime() 使用梯形速度剖面进行时间参数化
             *
             * @param start_path 外部提供的起始路径段（用于拼接，例如来自 odometry 的预测路径）
             * @param if_forward 前进/后退标志
             * @param current_state_VAJ 当前 VAJ 状态 [v, a, j]
             * @param current_state_OAJ 当前 OAJ 状态 [omega, alpha, j_omega]
             */
            void getKinoNodeWithStartPath(const std::vector<Eigen::Vector3d> &start_path, const bool if_forward,
                                          const Eigen::Vector3d &current_state_VAJ, const Eigen::Vector3d &current_state_OAJ);

            /**
             * @brief 将几何路径采样为 5D 状态序列 [x, y, theta, dtheta, ds]
             *
             * 流程:
             * 1. 在路径首点处生成三个候选朝向（原朝向 + 两种可能的初始方向）
             * 2. 对路径中每个中间点:
             *    - 插入"到达点"状态: 位置 = 路径点坐标，朝向 = 前一朝向，ds = 距上一点弧长
             *    - 插入"转向点"状态: 朝向更新为指向下一路径点的方向，dtheta = 朝向变化量
             * 3. 在路径末点使用目标朝向
             *
             * 这样交替插入"到达点"和"转向点"可以分离位置移动和角度旋转，
             * 使后端优化器分别处理平移和转向的代价。
             */
            void getSampleTraj();

            /**
             * @brief 使用梯形速度剖面对采样轨迹进行时间参数化
             *
             * 算法流程:
             * 1. 计算加权路径长度: L_weighted = sum(yaw_weight * |dtheta| + distance_weight * |ds|)
             *    - 同时考虑空间距离和角度变化，将转向代价和前进代价统一为标量
             * 2. 如果总长度超过 trajCutLength_，截断轨迹
             * 3. evaluateDuration() 计算总时间 T_total
             * 4. 以时间步长 delta_t 均匀采样，对每个采样时刻 t:
             *    - evaluateLength(t) 计算从起点到时刻 t 的行进弧长
             *    - 在加权弧长空间中找到对应的路径段，线性插值位置和朝向
             * 5. 将结果存入 flat_traj_
             */
            void getTrajsWithTime();

            /**
             * @brief 规范化角度到参考角度的 [-PI, +PI] 范围内
             *
             * 通过加减 2*PI 使得 (ref_angle - angle) 落在 [-PI, PI] 区间内。
             * 这保证了角度差不会因周期性而产生歧义。
             *
             * @param ref_angle 参考角度
             * @param angle 要规范化的角度（引用传递，原地修改）
             */
            void normalizeAngle(const double &ref_angle, double &angle);

            /**
             * @brief 梯形速度剖面: 根据路径长度计算总耗时
             *
             * 物理模型:
             *   加速段: v(t) = v_start + a_max * t,   s(t) = v_start*t + 0.5*a_max*t^2
             *   匀速段: v(t) = v_max,                  s(t) = v_max * t
             *   减速段: v(t) = v_max - a_max * t,     s(t) = v_max*t - 0.5*a_max*t^2
             *
             * @param length  路径总长度（加权弧长）
             * @param startV  起始速度
             * @param endV    终止速度
             * @param maxV    最大允许速度
             * @param maxA    最大允许加速度
             * @return 总耗时（秒）
             */
            double evaluateDuration(const double &length, const double &startV, const double &endV, const double &maxV, const double &maxA);

            /**
             * @brief 梯形速度剖面: 根据当前时间计算行进的弧长
             *
             * @param curt       当前时间 t
             * @param locallength 路径段总长度
             * @param localtime   路径段总耗时
             * @param startV     起始速度
             * @param endV       终止速度
             * @param maxV       最大速度
             * @param maxA       最大加速度
             * @return 从起点到时刻 t 的行进距离
             */
            double evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA);

            /// 梯形速度剖面: 根据当前时间计算速度
            double evaluateVel(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA);

            /// 梯形速度剖面: 根据位置计算到达时间（速度剖面的逆函数）
            double evaluteTimeOfPos(const double &pos, const double &locallength, const double &startV, const double &endV, const double &maxV, const double &maxA);

            /// 检查某个位置是否碰撞（使用 SDF 距离与 safe_dis_ 比较）
            bool JPS_check_if_collision(const Eigen::Vector2d &pos);
    };

};



#endif // JPS_PLANNER_H