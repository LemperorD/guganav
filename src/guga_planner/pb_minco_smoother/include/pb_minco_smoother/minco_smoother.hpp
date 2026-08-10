/**
 * @file minco_smoother.hpp
 * @brief MINCO 轨迹平滑器 - Nav2 Smoother 插件
 *
 * 基于 MINCO (Minimum Control) 算法的轨迹平滑器插件，
 * 继承自 nav2_core::Smoother 接口，用于 Nav2 导航框架。
 *
 * 功能特性：
 * 1. 将 A* 规划路径转换为平滑的多项式轨迹
 * 2. 集成 ESDF 避障优化
 * 3. 速度和加速度软约束
 * 4. 时间最优优化
 *
 * MIT License
 * Created by Jinbo Liu, 2025
 */

#ifndef PB_MINCO_SMOOTHER__MINCO_SMOOTHER_HPP_
#define PB_MINCO_SMOOTHER__MINCO_SMOOTHER_HPP_

#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <memory>
#include <nav2_core/smoother.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_subscriber.hpp>
#include <nav2_costmap_2d/footprint_subscriber.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

// 自定义消息
#include <interfaces/msg/minco_trajectory.hpp>

#include "gcopter/trajectory.hpp"
#include "pb_minco_smoother/costmap_esdf_adapter.hpp"
#include "pb_minco_smoother/minco_optimizer.hpp"

namespace pb_minco {

/**
 * @brief MINCO 轨迹平滑器参数
 */
struct MincoSmootherParams {
    // ========== 优化权重 ==========
    double weight_smooth = 1.0;        ///< 平滑性权重
    double weight_obstacle = 100.0;    ///< 避障权重
    double weight_feasibility = 10.0;  ///< 动力学可行性权重
    double weight_time = 10.0;         ///< 时间正则化权重

    // ========== 动力学约束 ==========
    double max_vel = 2.0;        ///< 最大速度 (m/s)
    double max_acc = 2.0;        ///< 最大加速度 (m/s²)
    double safe_distance = 0.5;  ///< 安全距离 (m)

    // ========== 优化器参数 ==========
    int max_iterations = 200;     ///< 最大迭代次数
    double g_epsilon = 1e-4;      ///< 梯度收敛阈值
    int integral_resolution = 8;  ///< 积分采样分辨率

    // ========== 路径处理参数 ==========
    int min_waypoints = 3;  ///< 最少路点数
    int max_waypoints = 50;  ///< 最多路点数，防止 MINCO 优化段数过多
    double initial_time_per_meter = 0.5;  ///< 初始每米时间估计 (s/m) [已废弃]

    // ========== 梯形速度规划与均匀时间重采样参数 ==========
    double resample_time_resolution =
        1.0;  ///< 重采样时间分辨率 (s)，MINCO 每段的初始时间
    double rotation_penalty_weight =
        1.0;  ///< 转弯惩罚权重 k2，将角度变化折算为等效距离
    double dense_sample_resolution =
        0.10;  ///< 高密度采样分辨率 (m)，用于梯形规划积分

    // ========== 滚动规划参数 ==========
    double local_planning_horizon =
        6.0;  ///< 局部规划距离 (m)，只优化前方这段路径
    double replan_prediction_time = 0.05;  ///< 重规划预测时间 (s)，补偿规划耗时
    double trajectory_validity_duration =
        2.0;  ///< 轨迹有效期 (s)，超时则视为无效

    // ========== 轨迹连续性参数（平滑重规划） ==========
    double trajectory_continuity_threshold =
        0.3;  ///< 轨迹连续性距离阈值
              ///< (m)，机器人距轨迹投影点小于此值时使用投影点边界
    double projection_search_resolution =
        0.02;  ///< 投影点搜索时间分辨率 (s)，越小越精确但计算量越大

    // ========== 输出参数 ==========
    double output_dt = 0.1;            ///< 输出轨迹采样间隔 (s)
    bool publish_trajectory = true;    ///< 是否发布轨迹可视化
    std::string odom_topic = "/odom";  ///< 里程计话题

    // ========== 两阶段优化参数 ==========
    // --- 第一阶段（Shape Optimization）权重 ---
    double stage1_weight_smooth = 1.0;  ///< 第一阶段平滑性权重
    double stage1_weight_obstacle = 50.0;  ///< 第一阶段避障权重（放松）
    double stage1_weight_feasibility =
        5.0;  ///< 第一阶段动力学可行性权重（放松）
    double stage1_weight_time = 10.0;  ///< 第一阶段时间正则化权重
    double stage1_weight_mean_time = 10.0;  ///< 第一阶段平均时间约束权重
    int stage1_max_iterations = 2000;       ///< 第一阶段最大迭代次数

    // --- 平均时间约束参数（两阶段共用） ---
    double mean_time_lower_bound = 0.9;  ///< 段时间下界 = avg_T * lower_bound
    double mean_time_upper_bound = 1.1;  ///< 段时间上界 = avg_T * upper_bound
    double weight_mean_time = 10.0;  ///< 第二阶段平均时间约束权重
};

/**
 * @class MincoSmoother
 * @brief Nav2 Smoother 插件 - 基于 MINCO 算法的轨迹平滑
 *
 * 将 A* 规划器输出的离散路径点转换为时间参数化的平滑多项式轨迹。
 * 优化目标包括：平滑性（最小化 Jerk）、避障（ESDF）、动力学可行性。
 */
class MincoSmoother : public nav2_core::Smoother {
   public:
    /**
     * @brief 构造函数
     */
    MincoSmoother() = default;

    /**
     * @brief 析构函数
     */
    ~MincoSmoother() override = default;

    /**
     * @brief 配置平滑器
     *
     * @param parent 父生命周期节点
     * @param name 插件名称
     * @param tf TF2 缓冲区
     * @param costmap_sub Costmap 订阅器
     * @param footprint_sub 足迹订阅器
     */
    void configure(
        const nav2_util::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
        std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub)
        override;

    /**
     * @brief 清理资源
     */
    void cleanup() override;

    /**
     * @brief 激活插件
     */
    void activate() override;

    /**
     * @brief 停用插件
     */
    void deactivate() override;

    /**
     * @brief 平滑路径
     *
     * @param path 输入/输出路径
     * @param max_time 最大允许时间
     * @return 是否成功完成（true）或被时间限制中断（false）
     */
    bool smooth(nav_msgs::msg::Path &path,
                const rclcpp::Duration &max_time) override;

   protected:
    /**
     * @brief 从 ROS 参数加载配置
     */
    void loadParameters();

    /**
     * @brief 路径预处理结果结构体
     *
     * 包含经过梯形速度规划和均匀时间重采样后的路点和时间分配
     */
    struct PathProcessingResult {
        Eigen::Matrix2Xd waypoints;  ///< 均匀时间重采样后的路点
        Eigen::VectorXd times;  ///< 每段的初始时间分配（均匀）
        double total_time;      ///< 总时间估计
        bool success;           ///< 处理结果是否有效

        PathProcessingResult() : total_time(0.0), success(false) {}
    };

    /**
     * @brief 基于梯形速度规划和均匀时间重采样的路径预处理
     *
     * 核心流程：
     * 1. 高密度采样原始路径，计算每个点的累积等效距离（考虑转弯惩罚）
     * 2. 使用前向-后向传播构建梯形速度分布
     * 3. 按照均匀时间间隔重采样路点
     *
     * @param path 原始路径
     * @param start_velocity 起点速度向量（用于梯形规划）
     * @return 处理结果，包含路点和时间分配
     */
    PathProcessingResult processPath(const nav_msgs::msg::Path &path,
                                     const Eigen::Vector2d &start_velocity,
                                     const Eigen::Vector2d &start_pose);

    /**
     * @brief 使用梯形速度规划计算走完指定距离所需的时间
     *
     * 基于 jps_planner.cpp 中的 evaluateDuration 实现
     *
     * @param length 路径长度
     * @param startV 起始速度
     * @param endV 终止速度
     * @param maxV 最大速度
     * @param maxA 最大加速度
     * @return 所需时间
     */
    double evaluateDuration(const double &length, const double &startV,
                            const double &endV, const double &maxV,
                            const double &maxA);

    /**
     * @brief 使用梯形速度规划计算给定时间点对应的路程
     *
     * 基于 jps_planner.cpp 中的 evaluateLength 实现
     *
     * @param curt 当前时间
     * @param locallength 总路径长度
     * @param localtime 总时间
     * @param startV 起始速度
     * @param endV 终止速度
     * @param maxV 最大速度
     * @param maxA 最大加速度
     * @return 对应的路程
     */
    double evaluateLength(const double &curt, const double &locallength,
                          const double &localtime, const double &startV,
                          const double &endV, const double &maxV,
                          const double &maxA);

    /**
     * @brief 从 MINCO 轨迹生成 Nav2 路径
     *
     * @param traj MINCO 轨迹 (S=3, 5次多项式, 2D)
     * @param header 路径消息头
     * @return 采样后的 Nav2 路径
     */
    nav_msgs::msg::Path trajectoryToPath(const Trajectory<5, 2> &traj,
                                         const std_msgs::msg::Header &header);

    /**
     * @brief 计算路径上两点之间的欧几里得距离
     */
    double computePathLength(const Eigen::Matrix2Xd &points);

    /**
     * @brief 发布轨迹可视化
     */
    void publishTrajectoryVisualization(const Trajectory<5, 2> &traj,
                                        const std_msgs::msg::Header &header);

    /**
     * @brief 发布中间路点可视化（球形 Marker）
     *
     * 使用 MarkerArray 发布均匀时间采样后的中间路点，
     * 便于观察 MINCO 优化器的输入路点分布。
     *
     * @param waypoints 路点矩阵 (2 x N)
     * @param header 消息头
     */
    void publishWaypointsVisualization(const Eigen::Matrix2Xd &waypoints,
                                       const std_msgs::msg::Header &header);

    /**
     * @brief 发布最终轨迹路点可视化（MarkerArray）
     *
     * 使用 MarkerArray 发布 MINCO 优化后的路点位置，
     * 用醒目颜色区分，方便与优化前路点对比。
     *
     * @param waypoints 优化后的路点矩阵 (2 x N)
     * @param header 消息头
     */
    void publishFinalWaypointsVisualization(
        const Eigen::Matrix2Xd &waypoints, const std_msgs::msg::Header &header);

    /**
     * @brief 将 MINCO 轨迹转换为 ROS 消息
     *
     * @param traj MINCO 轨迹对象 (S=3, 5次多项式, 2D)
     * @param header 消息头（时间戳和坐标系）
     * @param start_time 轨迹起始绝对时间
     * @return MincoTrajectory 消息
     */
    interfaces::msg::MincoTrajectory trajectoryToMsg(
        const Trajectory<5, 2> &traj, const std_msgs::msg::Header &header,
        const rclcpp::Time &start_time);

    /**
     * @brief 轨迹投影结果结构体
     *
     * 存储在历史轨迹上查找投影点的结果，包括投影时间、距离、
     * 以及投影点处的位置、速度和加速度信息。
     */
    struct TrajectoryProjectionResult {
        bool valid = false;            ///< 投影是否有效
        double projection_time = 0.0;  ///< 投影点在轨迹上的时间参数
        double distance = 0.0;  ///< 机器人当前位置到投影点的距离
        Eigen::Vector2d position;      ///< 投影点位置
        Eigen::Vector2d velocity;      ///< 投影点速度
        Eigen::Vector2d acceleration;  ///< 投影点加速度 (S=3 热启动)

        TrajectoryProjectionResult()
            : position(Eigen::Vector2d::Zero()),
              velocity(Eigen::Vector2d::Zero()),
              acceleration(Eigen::Vector2d::Zero()) {}
    };

    /**
     * @brief 在历史轨迹上查找距离当前机器人位置最近的投影点
     *
     * 通过在轨迹时间参数上进行搜索，找到距离机器人当前位置最近的点。
     * 该方法用于实现平滑重规划：当机器人偏离轨迹不大时，使用投影点
     * 的位置和速度作为新轨迹的初始边界条件，而非零速假设。
     *
     * @param traj 要搜索的轨迹
     * @param robot_pos 机器人当前2D位置
     * @param traj_start_time 轨迹的起始绝对时间
     * @param current_time 当前时间
     * @return 投影结果，包含投影点的位置、速度和距离信息
     *
     * @note 搜索范围限制在 [当前相对时间, 轨迹总时长] 内，
     *       因为已经经过的轨迹部分没有参考意义。
     */
    TrajectoryProjectionResult findProjectionOnTrajectory(
        const Trajectory<5, 2> &traj, const Eigen::Vector2d &robot_pos,
        const rclcpp::Time &traj_start_time, const rclcpp::Time &current_time);

   private:
    // 节点相关
    nav2_util::LifecycleNode::WeakPtr node_;
    rclcpp::Logger logger_{rclcpp::get_logger("MincoSmoother")};
    rclcpp::Clock::SharedPtr clock_;
    std::string plugin_name_;

    // TF 和 Costmap
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;

    // ESDF 适配器（用于连接 costmap 层的 ESDF）
    std::shared_ptr<CostmapESDFAdapter> esdf_adapter_;

    // 更新订阅器
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    /// 当前机器人位姿（用于 ESDF 查询高度）
    std::mutex robot_state_mutex_;

    // ========== 速度缓存（用于重规划的边界条件） ==========
    /**
     * @brief 当前机器人速度（2D，在世界坐标系下）
     *
     * 用于在重规划时设置轨迹的初始速度边界条件，
     * 避免生成"从静止开始"的轨迹导致的抖动。
     */
    Eigen::Vector2d current_velocity_{0.0, 0.0};
    double velocity_timestamp_ = 0.0;  ///< 速度更新时间戳
    std::mutex velocity_mutex_;        ///< 保护速度数据的互斥锁

    /// 速度有效性超时时间（秒），超过此时间认为速度无效，使用零速度
    static constexpr double kVelocityTimeout = 0.5;

    // ========== 2D 位置缓存（用于轨迹投影计算） ==========
    /**
     * @brief 当前机器人 2D 位置（世界坐标系）
     *
     * 从里程计中提取，用于在历史轨迹上查找投影点。
     * 与 current_robot_pos_ (3D) 不同，这里只存储 x, y。
     */
    Eigen::Vector2d current_robot_pos_2d_{0.0, 0.0};
    double robot_pos_timestamp_ = 0.0;  ///< 位置更新时间戳
    std::mutex robot_pos_2d_mutex_;     ///< 保护 2D 位置数据的互斥锁

    /// 位置有效性超时时间（秒）
    static constexpr double kPositionTimeout = 0.5;

    /**
     * @brief 里程计回调（更新机器人状态）
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // MINCO 优化器
    std::unique_ptr<MincoOptimizer> optimizer_;

    // 第一阶段优化器（Shape Optimization）
    std::unique_ptr<MincoOptimizer> stage1_optimizer_;

    // 参数
    MincoSmootherParams params_;

    // 发布器 - 可视化路径
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
        trajectory_pub_;

    // 发布器 - 第一阶段轨迹可视化（调试用）
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
        stage1_trajectory_pub_;

    // 发布器 - 中间路点可视化（球形 Marker）
    rclcpp_lifecycle::LifecyclePublisher<
        visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_marker_pub_;

    // 发布器 - 最终轨迹路点 MarkerArray 可视化
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::
        SharedPtr final_waypoints_marker_pub_;

    // 发布器 - MINCO 多项式轨迹（用于 Controller 订阅）
    rclcpp_lifecycle::LifecyclePublisher<
        interfaces::msg::MincoTrajectory>::SharedPtr minco_traj_pub_;

    // 轨迹 ID 计数器（用于生成唯一标识）
    uint32_t trajectory_id_counter_ = 0;

    double dyn_obs_timestamp_ = 0.0;  ///< 最近一次更新时间戳（秒）
    std::mutex dyn_obs_cache_mutex_;  ///< 保护缓存的互斥锁

    // ========== 滚动规划：轨迹缓存 ==========
    Trajectory<5, 2> last_trajectory_;  ///< 上一条优化后的轨迹 (S=3, Quintic)
    rclcpp::Time last_trajectory_time_;  ///< 上一条轨迹的发布时间戳
    bool has_valid_trajectory_ = false;  ///< 是否有有效的历史轨迹

    // 动态参数回调
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        dyn_params_handler_;

    /**
     * @brief 动态参数回调
     */
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters);
};

}  // namespace pb_minco

#endif  // PB_MINCO_SMOOTHER__MINCO_SMOOTHER_HPP_
