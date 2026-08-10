/**
 * @file minco_smoother.cpp
 * @brief MINCO 轨迹平滑器实现
 *
 * MIT License
 * Created by Jinbo Liu, 2025
 */

#include "pb_minco_smoother/minco_smoother.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pb_minco {

using nav2_util::declare_parameter_if_not_declared;
using std::chrono::steady_clock;

void MincoSmoother::configure(
    const nav2_util::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) {
    auto node = parent.lock();
    if (!node) {
        throw std::runtime_error("Unable to lock node!");
    }

    node_ = parent;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    tf_ = tf;
    costmap_sub_ = costmap_sub;
    footprint_sub_ = footprint_sub;

    // 加载参数
    loadParameters();

    // 创建 MINCO 优化器（第二阶段，Kinematic & Feasibility）
    optimizer_ = std::make_unique<MincoOptimizer>();

    // 创建第一阶段优化器（Shape Optimization）
    stage1_optimizer_ = std::make_unique<MincoOptimizer>();

    // 创建轨迹可视化发布器
    if (params_.publish_trajectory) {
        trajectory_pub_ =
            node->create_publisher<nav_msgs::msg::Path>("minco_trajectory", 1);
        // 第一阶段轨迹可视化（调试用）
        stage1_trajectory_pub_ = node->create_publisher<nav_msgs::msg::Path>(
            "~/debug/stage1_trajectory", 1);
    }

    // 创建 MINCO 多项式轨迹发布器（供 Controller 订阅）
    minco_traj_pub_ = node->create_publisher<interfaces::msg::MincoTrajectory>(
        "~/minco_polynomial_trajectory", rclcpp::QoS(1).reliable());

    // 创建中间路点可视化发布器（MarkerArray）
    waypoints_marker_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/minco_waypoints_marker", rclcpp::QoS(1));

    // 创建最终轨迹路点可视化发布器（MarkerArray）
    final_waypoints_marker_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/minco_final_waypoints_marker", rclcpp::QoS(1));

    // 创建 Costmap ESDF 适配器
    // 参数说明：
    // - max_distance: ESDF 最大距离，设为 safe_distance 的 10
    // 倍以提供足够的梯度场
    // - roi_margin: ROI 外扩距离，设为 4.0m 确保覆盖优化可能的区域
    esdf_adapter_ = std::make_shared<CostmapESDFAdapter>(
        logger_,
        params_.safe_distance * 10.0,  // max_distance = 10 * safe_distance
        4.0);                          // roi_margin = 4.0m

    // 创建里程计订阅器（用于获取当前速度，实现平滑重规划）
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        params_.odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&MincoSmoother::odomCallback, this, std::placeholders::_1));

    // 注册动态参数回调
    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(&MincoSmoother::dynamicParametersCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(logger_, "Configured MINCO smoother plugin: %s",
                plugin_name_.c_str());
}

void MincoSmoother::loadParameters() {
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("Unable to lock node!");
    }

    // 优化权重
    declare_parameter_if_not_declared(node, plugin_name_ + ".weight_smooth",
                                      rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".weight_obstacle",
                                      rclcpp::ParameterValue(100.0));
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".weight_feasibility",
                                      rclcpp::ParameterValue(10.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".weight_time",
                                      rclcpp::ParameterValue(10.0));

    // 动力学约束
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_vel",
                                      rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_acc",
                                      rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".safe_distance",
                                      rclcpp::ParameterValue(0.5));

    // 优化器参数
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_iterations",
                                      rclcpp::ParameterValue(200));
    declare_parameter_if_not_declared(node, plugin_name_ + ".g_epsilon",
                                      rclcpp::ParameterValue(1e-4));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".integral_resolution", rclcpp::ParameterValue(8));

    // 路径处理参数
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_waypoints",
                                      rclcpp::ParameterValue(3));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_waypoints",
                                      rclcpp::ParameterValue(50));
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".initial_time_per_meter",
                                      rclcpp::ParameterValue(0.5));

    // 梯形速度规划与均匀时间重采样参数
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".resample_time_resolution",
        rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".rotation_penalty_weight",
                                      rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".dense_sample_resolution",
                                      rclcpp::ParameterValue(0.10));

    // 输出参数
    declare_parameter_if_not_declared(node, plugin_name_ + ".output_dt",
                                      rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".publish_trajectory",
                                      rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".odom_topic",
                                      rclcpp::ParameterValue("/odom"));
    // 轨迹连续性参数（平滑重规划）
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".trajectory_continuity_threshold",
        rclcpp::ParameterValue(0.25));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".projection_search_resolution",
        rclcpp::ParameterValue(0.02));

    // 两阶段优化参数
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".stage1_weight_smooth",
                                      rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".stage1_weight_obstacle",
                                      rclcpp::ParameterValue(50.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".stage1_weight_feasibility",
        rclcpp::ParameterValue(5.0));
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".stage1_weight_time",
                                      rclcpp::ParameterValue(10.0));
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".stage1_weight_mean_time",
                                      rclcpp::ParameterValue(10.0));
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".stage1_max_iterations",
                                      rclcpp::ParameterValue(2000));

    // 平均时间约束参数
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".mean_time_lower_bound",
                                      rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".mean_time_upper_bound",
                                      rclcpp::ParameterValue(1.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".weight_mean_time",
                                      rclcpp::ParameterValue(10.0));

    // 读取参数
    node->get_parameter(plugin_name_ + ".weight_smooth", params_.weight_smooth);
    node->get_parameter(plugin_name_ + ".weight_obstacle",
                        params_.weight_obstacle);
    node->get_parameter(plugin_name_ + ".weight_feasibility",
                        params_.weight_feasibility);
    node->get_parameter(plugin_name_ + ".weight_time", params_.weight_time);
    node->get_parameter(plugin_name_ + ".max_vel", params_.max_vel);
    node->get_parameter(plugin_name_ + ".max_acc", params_.max_acc);
    node->get_parameter(plugin_name_ + ".safe_distance", params_.safe_distance);
    node->get_parameter(plugin_name_ + ".max_iterations",
                        params_.max_iterations);
    node->get_parameter(plugin_name_ + ".g_epsilon", params_.g_epsilon);
    node->get_parameter(plugin_name_ + ".integral_resolution",
                        params_.integral_resolution);
    node->get_parameter(plugin_name_ + ".min_waypoints", params_.min_waypoints);
    node->get_parameter(plugin_name_ + ".max_waypoints", params_.max_waypoints);
    node->get_parameter(plugin_name_ + ".initial_time_per_meter",
                        params_.initial_time_per_meter);

    // 读取梯形速度规划与均匀时间重采样参数
    node->get_parameter(plugin_name_ + ".resample_time_resolution",
                        params_.resample_time_resolution);
    node->get_parameter(plugin_name_ + ".rotation_penalty_weight",
                        params_.rotation_penalty_weight);
    node->get_parameter(plugin_name_ + ".dense_sample_resolution",
                        params_.dense_sample_resolution);

    node->get_parameter(plugin_name_ + ".output_dt", params_.output_dt);
    node->get_parameter(plugin_name_ + ".publish_trajectory",
                        params_.publish_trajectory);
    node->get_parameter(plugin_name_ + ".odom_topic", params_.odom_topic);
    // 读取轨迹连续性参数
    node->get_parameter(plugin_name_ + ".trajectory_continuity_threshold",
                        params_.trajectory_continuity_threshold);
    node->get_parameter(plugin_name_ + ".projection_search_resolution",
                        params_.projection_search_resolution);

    // 读取两阶段优化参数
    node->get_parameter(plugin_name_ + ".stage1_weight_smooth",
                        params_.stage1_weight_smooth);
    node->get_parameter(plugin_name_ + ".stage1_weight_obstacle",
                        params_.stage1_weight_obstacle);
    node->get_parameter(plugin_name_ + ".stage1_weight_feasibility",
                        params_.stage1_weight_feasibility);
    node->get_parameter(plugin_name_ + ".stage1_weight_time",
                        params_.stage1_weight_time);
    node->get_parameter(plugin_name_ + ".stage1_weight_mean_time",
                        params_.stage1_weight_mean_time);
    node->get_parameter(plugin_name_ + ".stage1_max_iterations",
                        params_.stage1_max_iterations);

    // 读取平均时间约束参数
    node->get_parameter(plugin_name_ + ".mean_time_lower_bound",
                        params_.mean_time_lower_bound);
    node->get_parameter(plugin_name_ + ".mean_time_upper_bound",
                        params_.mean_time_upper_bound);
    node->get_parameter(plugin_name_ + ".weight_mean_time",
                        params_.weight_mean_time);
}

void MincoSmoother::cleanup() {
    RCLCPP_INFO(logger_, "Cleaning up MINCO smoother plugin: %s",
                plugin_name_.c_str());

    // 清理订阅器
    odom_sub_.reset();
    esdf_adapter_.reset();
    optimizer_.reset();
    stage1_optimizer_.reset();
    trajectory_pub_.reset();
    stage1_trajectory_pub_.reset();
    minco_traj_pub_.reset();
    final_waypoints_marker_pub_.reset();
}

void MincoSmoother::activate() {
    RCLCPP_INFO(logger_, "Activating MINCO smoother plugin: %s",
                plugin_name_.c_str());
    if (trajectory_pub_) {
        trajectory_pub_->on_activate();
    }
    if (stage1_trajectory_pub_) {
        stage1_trajectory_pub_->on_activate();
    }
    if (minco_traj_pub_) {
        minco_traj_pub_->on_activate();
    }
    if (waypoints_marker_pub_) {
        waypoints_marker_pub_->on_activate();
    }
    if (final_waypoints_marker_pub_) {
        final_waypoints_marker_pub_->on_activate();
    }
}

void MincoSmoother::deactivate() {
    RCLCPP_INFO(logger_, "Deactivating MINCO smoother plugin: %s",
                plugin_name_.c_str());
    if (trajectory_pub_) {
        trajectory_pub_->on_deactivate();
    }
    if (stage1_trajectory_pub_) {
        stage1_trajectory_pub_->on_deactivate();
    }
    if (minco_traj_pub_) {
        minco_traj_pub_->on_deactivate();
    }
    if (waypoints_marker_pub_) {
        waypoints_marker_pub_->on_deactivate();
    }
    if (final_waypoints_marker_pub_) {
        final_waypoints_marker_pub_->on_deactivate();
    }
}

bool MincoSmoother::smooth(nav_msgs::msg::Path &path,
                           const rclcpp::Duration &max_time) {
    steady_clock::time_point start_time = steady_clock::now();

    // 检查路径有效性
    if (path.poses.size() < 2) {
        RCLCPP_WARN(logger_,
                    "Path too short for smoothing, need at least 2 poses");
        return true;  // 路径太短，直接返回
    }

    RCLCPP_DEBUG(logger_, "Smoothing path with %zu poses", path.poses.size());

    try {
        // 1. 确定初始状态 (Head State)
        Eigen::Vector2d head_pos;
        Eigen::Vector2d head_vel;
        Eigen::Vector2d head_acc;  // S=3 热启动：加速度边界条件
        bool use_projection = false;

        // 1.1 获取当前机器人位置
        Eigen::Vector2d robot_pos;
        double pos_timestamp;
        {
            std::lock_guard<std::mutex> lock(robot_pos_2d_mutex_);
            robot_pos = current_robot_pos_2d_;
            pos_timestamp = robot_pos_timestamp_;
        }

        // 检查位置数据是否有效（是否超时）
        double current_time_sec = clock_->now().seconds();
        bool position_valid =
            (current_time_sec - pos_timestamp) < kPositionTimeout;

        // 1.2 尝试从历史轨迹获取投影点
        if (has_valid_trajectory_ && position_valid) {
            rclcpp::Time now = clock_->now();
            double traj_age = (now - last_trajectory_time_).seconds();

            if (traj_age < params_.trajectory_validity_duration) {
                TrajectoryProjectionResult proj = findProjectionOnTrajectory(
                    last_trajectory_, robot_pos, last_trajectory_time_, now);

                if (proj.valid &&
                    proj.distance < params_.trajectory_continuity_threshold) {
                    head_pos = proj.position;
                    head_vel = proj.velocity;
                    head_acc = proj.acceleration;  // S=3 热启动：继承加速度
                    use_projection = true;
                    RCLCPP_DEBUG(logger_,
                                 "使用投影点作为起点: pos=(%.2f, %.2f), "
                                 "vel=(%.2f, %.2f), acc=(%.2f, %.2f)",
                                 head_pos.x(), head_pos.y(), head_vel.x(),
                                 head_vel.y(), head_acc.x(), head_acc.y());
                }
            }
        }

        // 如果没有有效投影，退化为使用路径第一个点和零速、零加速
        if (!use_projection) {
            const auto &start_pose = path.poses.front().pose.position;
            head_pos = Eigen::Vector2d(start_pose.x, start_pose.y);
            head_vel = Eigen::Vector2d::Zero();
            head_acc = Eigen::Vector2d::Zero();  // 静止启动：加速度为零
            RCLCPP_DEBUG(logger_, "使用零速边界条件: pos=(%.2f, %.2f)",
                         head_pos.x(), head_pos.y());
        }

        // 2. 路径预处理：梯形速度规划 + 均匀时间重采样
        // 直接使用确定的 head_vel 和 head_pos
        PathProcessingResult processed = processPath(path, head_vel, head_pos);

        if (!processed.success) {
            RCLCPP_WARN(logger_, "路径预处理失败，返回原始路径");
            return true;
        }

        Eigen::Matrix2Xd waypoints = processed.waypoints;
        Eigen::VectorXd initialTimes = processed.times;
        int numWaypoints = waypoints.cols();

        if (numWaypoints < params_.min_waypoints) {
            RCLCPP_DEBUG(logger_, "路点数不足 (%d < %d)，返回原始路径",
                         numWaypoints, params_.min_waypoints);
            return true;
        }

        int pieceNum = numWaypoints - 1;

        // 3. 构建边界条件（S=3 需要 PVA: 位置、速度、加速度）
        Eigen::Matrix<double, 2, 3> headPVA, tailPVA;

        // 终点边界：位置为路径终点，速度和加速度为零（停止状态）
        tailPVA.col(0) = waypoints.col(numWaypoints - 1);  // 位置
        tailPVA.col(1) = Eigen::Vector2d::Zero();          // 速度
        tailPVA.col(2) = Eigen::Vector2d::Zero();          // 加速度

        // 起点边界：使用热启动的 PVA
        headPVA.col(0) = head_pos;  // 位置
        headPVA.col(1) = head_vel;  // 速度
        headPVA.col(2) = head_acc;  // 加速度（投影继承或静止启动）

        // 4. 提取中间路点（不包含首尾）
        Eigen::Matrix2Xd innerPoints(2, pieceNum - 1);
        for (int i = 0; i < pieceNum - 1; ++i) {
            innerPoints.col(i) = waypoints.col(i + 1);
        }

        // 4.1 发布中间路点可视化（便于调试观察均匀时间分配后的路点位置）
        if (waypoints_marker_pub_) {
            publishWaypointsVisualization(waypoints, path.header);
        }

        // 5. 更新 ESDF（两阶段共用）
        // Step 5.1: 从下采样后的路点构建路径点向量
        std::vector<Eigen::Vector2d> path_points;
        path_points.reserve(waypoints.cols());
        for (int i = 0; i < waypoints.cols(); ++i) {
            path_points.emplace_back(waypoints.col(i));
        }

        // Step 5.2: 获取 Costmap 指针并更新局部 ESDF
        auto costmap = costmap_sub_->getCostmap();
        bool esdf_available = false;
        if (costmap && esdf_adapter_) {
            bool esdf_ok =
                esdf_adapter_->updateSnapshot(costmap.get(), path_points);
            if (esdf_ok) {
                esdf_available = true;
                RCLCPP_DEBUG(logger_, "使用 Costmap ESDF 快照进行优化");
            } else {
                RCLCPP_WARN(logger_,
                            "Costmap ESDF 快照更新失败，优化可能无避障");
            }
        } else {
            RCLCPP_WARN(logger_, "无法获取 Costmap，优化将无避障功能");
        }

        // ====================================================================
        // ========== 第一阶段：Shape Optimization（形状优化） ===========
        // ====================================================================
        // 目标：快速获得合理的几何形状和初步的时间分配
        // 使用放松的约束权重，主要优化轨迹形状
        {
            MincoOptimizerConfig stage1_config;
            stage1_config.weight_smooth = params_.stage1_weight_smooth;
            stage1_config.weight_obstacle = params_.stage1_weight_obstacle;
            stage1_config.weight_feasibility =
                params_.stage1_weight_feasibility;
            stage1_config.weight_time = params_.stage1_weight_time;
            stage1_config.weight_mean_time = params_.stage1_weight_mean_time;
            stage1_config.mean_time_lower_bound = params_.mean_time_lower_bound;
            stage1_config.mean_time_upper_bound = params_.mean_time_upper_bound;
            stage1_config.max_vel = params_.max_vel;
            stage1_config.max_acc = params_.max_acc;
            stage1_config.safe_distance = params_.safe_distance;
            stage1_config.max_iterations = params_.stage1_max_iterations;
            stage1_config.g_epsilon = params_.g_epsilon;
            stage1_config.integral_resolution = params_.integral_resolution;

            stage1_optimizer_->setConfig(stage1_config);
            if (esdf_available) {
                stage1_optimizer_->setESDFInterface(esdf_adapter_);
            }

            stage1_optimizer_->initialize(headPVA, tailPVA, pieceNum);

            // 检查时间限制
            auto elapsed = steady_clock::now() - start_time;
            if (elapsed >
                std::chrono::duration<double>(max_time.seconds() * 0.4)) {
                RCLCPP_WARN(logger_,
                            "MINCO smoother timed out during stage1 init");
                return false;
            }

            bool stage1_success =
                stage1_optimizer_->optimize(innerPoints, initialTimes);

            if (!stage1_success) {
                RCLCPP_WARN(logger_,
                            "Stage 1 optimization failed, using initial "
                            "points for stage 2");
            } else {
                // 使用第一阶段的优化结果作为第二阶段的初值
                innerPoints = stage1_optimizer_->getOptimizedPoints();
                initialTimes = stage1_optimizer_->getOptimizedTimes();

                // RCLCPP_INFO(logger_,
                //             "Stage 1 (Shape) 完成: pieces=%d, total_T=%.2f
                //             s", pieceNum, initialTimes.sum());

                // 发布第一阶段轨迹可视化
                if (params_.publish_trajectory && stage1_trajectory_pub_) {
                    Trajectory<5, 2> stage1Traj;
                    stage1_optimizer_->getTrajectory(stage1Traj);
                    auto stage1_path =
                        trajectoryToPath(stage1Traj, path.header);
                    stage1_trajectory_pub_->publish(stage1_path);
                }
            }
        }

        // ====================================================================
        // ========== 第二阶段：Kinematic & Feasibility Optimization ====
        // ====================================================================
        // 目标：满足严格的动力学约束和安全距离
        // 输入：使用第一阶段优化后的路点和时间作为初值
        {
            MincoOptimizerConfig stage2_config;
            stage2_config.weight_smooth = params_.weight_smooth;
            stage2_config.weight_obstacle = params_.weight_obstacle;
            stage2_config.weight_feasibility = params_.weight_feasibility;
            stage2_config.weight_time = params_.weight_time;
            stage2_config.weight_mean_time = params_.weight_mean_time;
            stage2_config.mean_time_lower_bound = params_.mean_time_lower_bound;
            stage2_config.mean_time_upper_bound = params_.mean_time_upper_bound;
            stage2_config.max_vel = params_.max_vel;
            stage2_config.max_acc = params_.max_acc;
            stage2_config.safe_distance = params_.safe_distance;
            stage2_config.max_iterations = params_.max_iterations;
            stage2_config.g_epsilon = params_.g_epsilon;
            stage2_config.integral_resolution = params_.integral_resolution;

            optimizer_->setConfig(stage2_config);
            if (esdf_available) {
                optimizer_->setESDFInterface(esdf_adapter_);
            }

            optimizer_->initialize(headPVA, tailPVA, pieceNum);

            // 检查时间限制
            auto elapsed = steady_clock::now() - start_time;
            if (elapsed > std::chrono::duration<double>(max_time.seconds())) {
                RCLCPP_WARN(logger_,
                            "MINCO smoother timed out during stage2 init");
                return false;
            }

            bool success = optimizer_->optimize(innerPoints, initialTimes);

            if (!success) {
                RCLCPP_WARN(logger_,
                            "Stage 2 optimization failed, returning "
                            "original path");
                return true;
            }
        }

        // 10. 获取优化后的轨迹 (S=3, Quintic, 2D)
        Trajectory<5, 2> optimizedTraj;
        optimizer_->getTrajectory(optimizedTraj);

        // 11. 缓存优化后的轨迹（供下次重规划时查找投影点）
        last_trajectory_ = optimizedTraj;
        last_trajectory_time_ = clock_->now();
        has_valid_trajectory_ = true;

        RCLCPP_DEBUG(logger_, "已缓存优化轨迹: duration=%.2f s, pieces=%d",
                     optimizedTraj.getTotalDuration(),
                     optimizedTraj.getPieceNum());

        // 12. 将轨迹转换为 Nav2 路径
        nav_msgs::msg::Path smoothedPath =
            trajectoryToPath(optimizedTraj, path.header);

        // 13. 更新输出路径（保留给 Nav2 框架）
        path = smoothedPath;

        // 14. 发布轨迹可视化
        if (params_.publish_trajectory && trajectory_pub_) {
            publishTrajectoryVisualization(optimizedTraj, path.header);
        }

        // 14.1 发布最终优化路点可视化（MarkerArray）
        if (final_waypoints_marker_pub_) {
            // 构建包含首尾的完整路点矩阵
            Eigen::Matrix2Xd finalWaypoints(2, pieceNum + 1);
            finalWaypoints.col(0) = headPVA.col(0);
            const auto &optPts = optimizer_->getOptimizedPoints();
            for (int i = 0; i < pieceNum - 1; ++i) {
                finalWaypoints.col(i + 1) = optPts.col(i);
            }
            finalWaypoints.col(pieceNum) = tailPVA.col(0);
            publishFinalWaypointsVisualization(finalWaypoints, path.header);
        }

        // 15. 发布 MINCO 多项式轨迹消息（供 Controller 订阅）
        if (minco_traj_pub_) {
            // 使用缓存的轨迹时间作为起始时间
            auto minco_msg = trajectoryToMsg(optimizedTraj, path.header,
                                             last_trajectory_time_);
            minco_traj_pub_->publish(minco_msg);
            RCLCPP_DEBUG(logger_,
                         "Published MINCO trajectory with %d pieces, id=%u",
                         optimizedTraj.getPieceNum(), minco_msg.trajectory_id);
        }
        return true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "Exception during path smoothing: %s", e.what());
        return false;
    }
}

nav_msgs::msg::Path MincoSmoother::trajectoryToPath(
    const Trajectory<5, 2> &traj, const std_msgs::msg::Header &header) {
    nav_msgs::msg::Path path;
    path.header = header;

    double totalDuration = traj.getTotalDuration();
    int numSamples = static_cast<int>(totalDuration / params_.output_dt) + 1;

    path.poses.reserve(numSamples);

    for (int i = 0; i < numSamples; ++i) {
        double t = i * params_.output_dt;
        if (t > totalDuration) {
            t = totalDuration;
        }

        Eigen::Vector2d pos = traj.getPos(t);
        Eigen::Vector2d vel = traj.getVel(t);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = 0.0;

        // 从速度方向计算朝向
        double yaw = std::atan2(vel.y(), vel.x());
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        path.poses.push_back(pose);
    }

    return path;
}

void MincoSmoother::publishTrajectoryVisualization(
    const Trajectory<5, 2> &traj, const std_msgs::msg::Header &header) {
    if (!trajectory_pub_) {
        return;
    }

    auto path = trajectoryToPath(traj, header);
    trajectory_pub_->publish(path);
}

void MincoSmoother::publishWaypointsVisualization(
    const Eigen::Matrix2Xd &waypoints, const std_msgs::msg::Header &header) {
    if (!waypoints_marker_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    // 首先发布一个 DELETEALL marker 来清除旧的 markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header = header;
    delete_marker.ns = "minco_waypoints";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int num_waypoints = waypoints.cols();

    // 为每个路点创建球形 Marker
    for (int i = 0; i < num_waypoints; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "minco_waypoints";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 位置
        marker.pose.position.x = waypoints(0, i);
        marker.pose.position.y = waypoints(1, i);
        marker.pose.position.z = 0.1;  // 稍微抬高以便可见
        marker.pose.orientation.w = 1.0;

        // 尺寸
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;

        // 颜色：使用渐变色，起点绿色，终点红色
        double ratio = static_cast<double>(i) / std::max(1, num_waypoints - 1);
        marker.color.r = static_cast<float>(ratio);
        marker.color.g = static_cast<float>(1.0 - ratio);
        marker.color.b = 0.2f;
        marker.color.a = 0.9f;

        // 生命周期（设为 0 表示永久，直到被删除）
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        marker_array.markers.push_back(marker);
    }

    // 添加起点和终点的特殊标记
    if (num_waypoints >= 2) {
        // 起点 - 较大的绿色球
        visualization_msgs::msg::Marker start_marker;
        start_marker.header = header;
        start_marker.ns = "minco_waypoints_endpoints";
        start_marker.id = 0;
        start_marker.type = visualization_msgs::msg::Marker::SPHERE;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        start_marker.pose.position.x = waypoints(0, 0);
        start_marker.pose.position.y = waypoints(1, 0);
        start_marker.pose.position.z = 0.15;
        start_marker.pose.orientation.w = 1.0;
        start_marker.scale.x = 0.25;
        start_marker.scale.y = 0.25;
        start_marker.scale.z = 0.25;
        start_marker.color.r = 0.0f;
        start_marker.color.g = 1.0f;
        start_marker.color.b = 0.0f;
        start_marker.color.a = 1.0f;
        marker_array.markers.push_back(start_marker);

        // 终点 - 较大的红色球
        visualization_msgs::msg::Marker end_marker;
        end_marker.header = header;
        end_marker.ns = "minco_waypoints_endpoints";
        end_marker.id = 1;
        end_marker.type = visualization_msgs::msg::Marker::SPHERE;
        end_marker.action = visualization_msgs::msg::Marker::ADD;
        end_marker.pose.position.x = waypoints(0, num_waypoints - 1);
        end_marker.pose.position.y = waypoints(1, num_waypoints - 1);
        end_marker.pose.position.z = 0.15;
        end_marker.pose.orientation.w = 1.0;
        end_marker.scale.x = 0.25;
        end_marker.scale.y = 0.25;
        end_marker.scale.z = 0.25;
        end_marker.color.r = 1.0f;
        end_marker.color.g = 0.0f;
        end_marker.color.b = 0.0f;
        end_marker.color.a = 1.0f;
        marker_array.markers.push_back(end_marker);
    }

    // 添加连接线（LINE_STRIP）展示路点顺序
    visualization_msgs::msg::Marker line_marker;
    line_marker.header = header;
    line_marker.ns = "minco_waypoints_line";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.03;  // 线宽
    line_marker.color.r = 0.3f;
    line_marker.color.g = 0.3f;
    line_marker.color.b = 1.0f;
    line_marker.color.a = 0.7f;

    for (int i = 0; i < num_waypoints; ++i) {
        geometry_msgs::msg::Point p;
        p.x = waypoints(0, i);
        p.y = waypoints(1, i);
        p.z = 0.05;
        line_marker.points.push_back(p);
    }
    marker_array.markers.push_back(line_marker);

    waypoints_marker_pub_->publish(marker_array);

    RCLCPP_DEBUG(logger_, "发布了 %d 个中间路点的可视化 Marker", num_waypoints);
}

void MincoSmoother::publishFinalWaypointsVisualization(
    const Eigen::Matrix2Xd &waypoints, const std_msgs::msg::Header &header) {
    if (!final_waypoints_marker_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    // 首先发布一个 DELETEALL marker 来清除旧的 markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header = header;
    delete_marker.ns = "minco_final_waypoints";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int num_waypoints = waypoints.cols();

    // 为每个优化后的路点创建菱形 Marker（区别于优化前的球形）
    for (int i = 0; i < num_waypoints; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "minco_final_waypoints";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 位置
        marker.pose.position.x = waypoints(0, i);
        marker.pose.position.y = waypoints(1, i);
        marker.pose.position.z = 0.2;  // 稍高于输入路点
        // 旋转 45° 形成菱形视觉效果
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, M_PI / 4.0);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        // 尺寸
        marker.scale.x = 0.12;
        marker.scale.y = 0.12;
        marker.scale.z = 0.12;

        // 颜色：使用醒目的青色-洋红渐变，区别于输入路点的绿-红渐变
        double ratio = static_cast<double>(i) / std::max(1, num_waypoints - 1);
        marker.color.r = static_cast<float>(ratio);
        marker.color.g = static_cast<float>(0.8 * (1.0 - ratio));
        marker.color.b = 1.0f;
        marker.color.a = 0.95f;

        marker.lifetime = rclcpp::Duration::from_seconds(0);

        marker_array.markers.push_back(marker);
    }

    // 添加连接线（LINE_STRIP），使用不同颜色区分
    visualization_msgs::msg::Marker line_marker;
    line_marker.header = header;
    line_marker.ns = "minco_final_waypoints_line";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.04;  // 线宽
    line_marker.color.r = 1.0f;
    line_marker.color.g = 0.5f;
    line_marker.color.b = 0.0f;
    line_marker.color.a = 0.8f;

    for (int i = 0; i < num_waypoints; ++i) {
        geometry_msgs::msg::Point p;
        p.x = waypoints(0, i);
        p.y = waypoints(1, i);
        p.z = 0.15;
        line_marker.points.push_back(p);
    }
    marker_array.markers.push_back(line_marker);

    final_waypoints_marker_pub_->publish(marker_array);

    RCLCPP_DEBUG(logger_, "发布了 %d 个最终优化路点的可视化 Marker",
                 num_waypoints);
}

rcl_interfaces::msg::SetParametersResult
MincoSmoother::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        const std::string &name = param.get_name();

        if (name == plugin_name_ + ".weight_smooth") {
            params_.weight_smooth = param.as_double();
        } else if (name == plugin_name_ + ".weight_obstacle") {
            params_.weight_obstacle = param.as_double();
        } else if (name == plugin_name_ + ".weight_feasibility") {
            params_.weight_feasibility = param.as_double();
        } else if (name == plugin_name_ + ".weight_time") {
            params_.weight_time = param.as_double();
        } else if (name == plugin_name_ + ".max_vel") {
            params_.max_vel = param.as_double();
        } else if (name == plugin_name_ + ".max_acc") {
            params_.max_acc = param.as_double();
        } else if (name == plugin_name_ + ".safe_distance") {
            params_.safe_distance = param.as_double();
        } else if (name == plugin_name_ + ".max_iterations") {
            params_.max_iterations = param.as_int();
        } else if (name == plugin_name_ + ".output_dt") {
            params_.output_dt = param.as_double();
        } else if (name == plugin_name_ + ".mean_time_lower_bound") {
            params_.mean_time_lower_bound = param.as_double();
        } else if (name == plugin_name_ + ".mean_time_upper_bound") {
            params_.mean_time_upper_bound = param.as_double();
        } else if (name == plugin_name_ + ".weight_mean_time") {
            params_.weight_mean_time = param.as_double();
        } else if (name == plugin_name_ + ".stage1_weight_smooth") {
            params_.stage1_weight_smooth = param.as_double();
        } else if (name == plugin_name_ + ".stage1_weight_obstacle") {
            params_.stage1_weight_obstacle = param.as_double();
        } else if (name == plugin_name_ + ".stage1_weight_feasibility") {
            params_.stage1_weight_feasibility = param.as_double();
        } else if (name == plugin_name_ + ".stage1_weight_time") {
            params_.stage1_weight_time = param.as_double();
        } else if (name == plugin_name_ + ".stage1_weight_mean_time") {
            params_.stage1_weight_mean_time = param.as_double();
        } else if (name == plugin_name_ + ".stage1_max_iterations") {
            params_.stage1_max_iterations = param.as_int();
        }
    }

    return result;
}

interfaces::msg::MincoTrajectory MincoSmoother::trajectoryToMsg(
    const Trajectory<5, 2> &traj, const std_msgs::msg::Header &header,
    const rclcpp::Time &start_time) {
    interfaces::msg::MincoTrajectory msg;

    // 设置消息头
    msg.header = header;
    msg.trajectory_id = trajectory_id_counter_++;
    msg.start_time = start_time;

    int pieceNum = traj.getPieceNum();

    // 设置每段的持续时间
    msg.durations.resize(pieceNum);
    for (int i = 0; i < pieceNum; ++i) {
        msg.durations[i] = traj[i].getDuration();
    }

    // 展平系数矩阵
    // 每段: 2 维 (x, y) × 6 系数 (c5, c4, c3, c2, c1, c0) = 12 个 double
    // 总计: pieceNum × 12
    constexpr int DEGREE = 5;                   // 五次多项式
    constexpr int DIM = 2;                      // 2D 轨迹
    constexpr int COEFFS_PER_DIM = DEGREE + 1;  // 6 个系数

    msg.coefficients.resize(pieceNum * DIM * COEFFS_PER_DIM);

    for (int i = 0; i < pieceNum; ++i) {
        const auto &coeffMat = traj[i].getCoeffMat();  // 2 x 6 矩阵
        int base_idx = i * DIM * COEFFS_PER_DIM;

        // x 维度的系数 (从高次到低次: c5, c4, c3, c2, c1, c0)
        for (int j = 0; j < COEFFS_PER_DIM; ++j) {
            msg.coefficients[base_idx + j] = coeffMat(0, j);
        }

        // y 维度的系数 (从高次到低次: c5, c4, c3, c2, c1, c0)
        for (int j = 0; j < COEFFS_PER_DIM; ++j) {
            msg.coefficients[base_idx + COEFFS_PER_DIM + j] = coeffMat(1, j);
        }
    }

    return msg;
}

MincoSmoother::TrajectoryProjectionResult
MincoSmoother::findProjectionOnTrajectory(const Trajectory<5, 2> &traj,
                                          const Eigen::Vector2d &robot_pos,
                                          const rclcpp::Time &traj_start_time,
                                          const rclcpp::Time &current_time) {
    TrajectoryProjectionResult result;

    // 1. 计算当前相对时间（相对于轨迹起始时间）
    double elapsed_time = (current_time - traj_start_time).seconds();
    double total_duration = traj.getTotalDuration();

    // 如果轨迹已经完全过期，返回无效结果
    if (elapsed_time >= total_duration) {
        RCLCPP_WARN(logger_, "轨迹已过期: elapsed=%.2f s, duration=%.2f s",
                    elapsed_time, total_duration);
        return result;  // valid = false
    }

    // 2. 确定搜索范围
    //    - 起始时间: max(0, elapsed_time) - 已经经过的部分没有参考意义
    //    - 结束时间: total_duration
    //    - 考虑到重规划预测时间，稍微向前看一点
    double search_start = 0.0;
    double search_end = total_duration;

    // 如果可搜索范围太小，返回无效
    if (search_end - search_start < params_.projection_search_resolution) {
        RCLCPP_WARN(logger_, "轨迹剩余时间太短，无法进行投影搜索");
        return result;  // valid = false
    }

    // 3. 在轨迹上搜索距离机器人最近的投影点
    //    采用均匀采样搜索，找到最小距离点
    double min_distance_sq = std::numeric_limits<double>::max();
    double best_time = search_start;

    // 计算搜索步数
    int num_samples = static_cast<int>((search_end - search_start) /
                                       params_.projection_search_resolution) +
                      1;
    num_samples = std::max(num_samples, 10);  // 至少采样 10 个点

    for (int i = 0; i < num_samples; ++i) {
        // 计算当前采样时间
        double t =
            search_start + (search_end - search_start) * i / (num_samples - 1);

        // 获取轨迹上该时间点的位置
        Eigen::Vector2d traj_pos = traj.getPos(t);

        // 计算到机器人的距离平方
        double dist_sq = (traj_pos - robot_pos).squaredNorm();

        // 更新最小距离
        if (dist_sq < min_distance_sq) {
            min_distance_sq = dist_sq;
            best_time = t;
        }
    }

    // 4. 局部精细搜索（可选，提高精度）
    //    在粗搜索结果附近进行更精细的搜索
    double refine_range = params_.projection_search_resolution * 2.0;
    double refine_start = std::max(search_start, best_time - refine_range);
    double refine_end = std::min(search_end, best_time + refine_range);
    double refine_step = params_.projection_search_resolution * 0.1;

    for (double t = refine_start; t <= refine_end; t += refine_step) {
        Eigen::Vector2d traj_pos = traj.getPos(t);
        double dist_sq = (traj_pos - robot_pos).squaredNorm();

        if (dist_sq < min_distance_sq) {
            min_distance_sq = dist_sq;
            best_time = t;
        }
    }

    // 5. 填充结果
    result.valid = true;
    result.projection_time = best_time;
    result.distance = std::sqrt(min_distance_sq);
    result.position = traj.getPos(best_time);
    result.velocity = traj.getVel(best_time);
    result.acceleration = traj.getAcc(best_time);  // S=3 热启动：获取加速度

    RCLCPP_DEBUG(logger_,
                 "轨迹投影结果: t=%.3f s, dist=%.3f m, pos=(%.2f, %.2f), "
                 "vel=(%.2f, %.2f), acc=(%.2f, %.2f)",
                 result.projection_time, result.distance, result.position.x(),
                 result.position.y(), result.velocity.x(), result.velocity.y(),
                 result.acceleration.x(), result.acceleration.y());

    return result;
}

// 梯形速度规划辅助函数
double MincoSmoother::evaluateDuration(const double &length,
                                       const double &startV, const double &endV,
                                       const double &maxV, const double &maxA) {
    // 计算梯形速度规划的总时间
    // 考虑两种情况：
    // 1. 能够加速到最大速度再减速
    // 2. 只能加速到某个中间速度再减速（三角形速度剖面）

    double startv2 = startV * startV;
    double endv2 = endV * endV;
    double maxv2 = maxV * maxV;

    // 临界距离：从 startV 加速到 maxV 所需距离 + 从 maxV 减速到 endV
    // 所需距离
    double critical_len =
        (maxv2 - startv2) / (2.0 * maxA) + (maxv2 - endv2) / (2.0 * maxA);

    if (length >= critical_len) {
        // 能够达到最大速度：梯形速度剖面
        // 时间 = 加速时间 + 匀速时间 + 减速时间
        double t_acc = (maxV - startV) / maxA;
        double t_dec = (maxV - endV) / maxA;
        double t_const = (length - critical_len) / maxV;
        return t_acc + t_const + t_dec;
    } else {
        // 无法达到最大速度：三角形速度剖面
        // 计算能达到的峰值速度
        double peak_v =
            std::sqrt(0.5 * (startv2 + endv2 + 2.0 * maxA * length));
        double t_acc = (peak_v - startV) / maxA;
        double t_dec = (peak_v - endV) / maxA;
        return t_acc + t_dec;
    }
}

double MincoSmoother::evaluateLength(const double &curt,
                                     const double &locallength,
                                     const double &localtime,
                                     const double &startV, const double &endV,
                                     const double &maxV, const double &maxA) {
    // 给定时间 curt，返回在该时刻走过的距离
    // 用于均匀时间重采样

    double startv2 = startV * startV;
    double endv2 = endV * endV;
    double maxv2 = maxV * maxV;

    double critical_len =
        (maxv2 - startv2) / (2.0 * maxA) + (maxv2 - endv2) / (2.0 * maxA);

    if (locallength >= critical_len) {
        // 梯形速度剖面
        double t1 = (maxV - startV) / maxA;  // 加速结束时刻
        double t2 = t1 + (locallength - critical_len) / maxV;  // 匀速结束时刻

        if (curt <= t1) {
            // 加速阶段: s = v0*t + 0.5*a*t^2
            return startV * curt + 0.5 * maxA * curt * curt;
        } else if (curt <= t2) {
            // 匀速阶段: s = s1 + maxV * (t - t1)
            double s1 = startV * t1 + 0.5 * maxA * t1 * t1;
            return s1 + maxV * (curt - t1);
        } else {
            // 减速阶段: s = s2 + maxV*(t-t2) - 0.5*a*(t-t2)^2
            double s1 = startV * t1 + 0.5 * maxA * t1 * t1;
            double s2 = s1 + maxV * (t2 - t1);
            double dt = curt - t2;
            return s2 + maxV * dt - 0.5 * maxA * dt * dt;
        }
    } else {
        // 三角形速度剖面
        double peak_v =
            std::sqrt(0.5 * (startv2 + endv2 + 2.0 * maxA * locallength));
        double t_peak = (peak_v - startV) / maxA;  // 峰值时刻

        if (curt <= t_peak) {
            // 加速阶段
            return startV * curt + 0.5 * maxA * curt * curt;
        } else {
            // 减速阶段
            double s_peak = startV * t_peak + 0.5 * maxA * t_peak * t_peak;
            double dt = curt - t_peak;
            return s_peak + peak_v * dt - 0.5 * maxA * dt * dt;
        }
    }
}

// 路径预处理函数（梯形速度规划 + 均匀时间重采样）
MincoSmoother::PathProcessingResult MincoSmoother::processPath(
    const nav_msgs::msg::Path &path, const Eigen::Vector2d &start_vel,
    const Eigen::Vector2d &start_pose) {
    PathProcessingResult result;
    result.success = false;

    if (path.poses.size() < 2) {
        RCLCPP_WARN(logger_, "processPath: 路径点数太少 (%zu < 2)",
                    path.poses.size());
        return result;
    }

    // Step 1: 高密度采样 + 等效距离计算
    struct DenseSample {
        Eigen::Vector2d position;
        double yaw;
        double equiv_distance;   // 从起点的等效累积距离
        double linear_distance;  // 从起点的线性累积距离
    };
    std::vector<DenseSample> dense_samples;

    // 提取第一个点
    // const auto &first_pose = start_pose;
    // Eigen::Vector2d first_pos(first_pose.position.x,
    // first_pose.position.y);
    double first_yaw = std::atan2(start_vel.y(), start_vel.x());

    DenseSample first_sample;
    first_sample.position = start_pose;
    first_sample.yaw = first_yaw;
    first_sample.equiv_distance = 0.0;
    first_sample.linear_distance = 0.0;
    dense_samples.push_back(first_sample);

    double accumulated_linear_dist = 0.0;
    double accumulated_equiv_dist = 0.0;
    Eigen::Vector2d last_pos = start_pose;
    double last_yaw = first_yaw;

    // 对原始路径进行高密度重采样
    for (size_t i = 1; i < path.poses.size(); ++i) {
        const auto &pose = path.poses[i].pose;
        Eigen::Vector2d current_pos(pose.position.x, pose.position.y);
        double current_yaw = tf2::getYaw(pose.orientation);

        double segment_dist = (current_pos - last_pos).norm();

        // 如果段距离过大，进行细分插值
        int num_subdivisions = std::max(
            1, static_cast<int>(
                   std::ceil(segment_dist / params_.dense_sample_resolution)));

        for (int j = 1; j <= num_subdivisions; ++j) {
            double t = static_cast<double>(j) / num_subdivisions;
            Eigen::Vector2d interp_pos =
                last_pos + t * (current_pos - last_pos);

            // 角度插值（考虑2π周期性）
            double delta_yaw = current_yaw - last_yaw;
            // 归一化到 [-π, π]
            while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
            while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;
            double interp_yaw = last_yaw + t * delta_yaw;

            // 计算这一小段的距离增量
            Eigen::Vector2d prev_sample_pos = dense_samples.back().position;
            double prev_sample_yaw = dense_samples.back().yaw;

            double ds_linear = (interp_pos - prev_sample_pos).norm();
            double d_theta = std::abs(interp_yaw - prev_sample_yaw);
            while (d_theta > M_PI) d_theta = 2.0 * M_PI - d_theta;

            // 等效距离 = 线性距离 + k2 * |Δθ|
            double ds_equiv =
                ds_linear + params_.rotation_penalty_weight * d_theta;

            accumulated_linear_dist += ds_linear;
            accumulated_equiv_dist += ds_equiv;

            DenseSample sample;
            sample.position = interp_pos;
            sample.yaw = interp_yaw;
            sample.linear_distance = accumulated_linear_dist;
            sample.equiv_distance = accumulated_equiv_dist;
            dense_samples.push_back(sample);
        }

        last_pos = current_pos;
        last_yaw = current_yaw;
    }

    if (dense_samples.size() < 2) {
        RCLCPP_WARN(logger_, "processPath: 高密度采样后点数太少");
        return result;
    }

    double total_equiv_length = dense_samples.back().equiv_distance;
    RCLCPP_DEBUG(logger_,
                 "processPath: 高密度采样完成，%zu 个点，总等效距离: %.2f m",
                 dense_samples.size(), total_equiv_length);

    // Step 2 前向-后向速度传播
    // 2.0 构筑允许超速缓冲的 "基准限速线"
    double start_speed = start_vel.norm();
    std::vector<double> speed_limits(dense_samples.size(), params_.max_vel);

    if (start_speed > params_.max_vel) {
        speed_limits[0] = start_speed;
        for (size_t i = 1; i < dense_samples.size(); ++i) {
            double ds = dense_samples[i].equiv_distance -
                        dense_samples[i - 1].equiv_distance;
            double v_prev_limit = speed_limits[i - 1];
            // 只在仍处于超速减速缓冲期时放宽限速
            if (v_prev_limit > params_.max_vel) {
                // 允许以最大的合理减速度降到 max_vel
                double relaxed_v =
                    std::sqrt(std::max(0.0, v_prev_limit * v_prev_limit -
                                                2.0 * params_.max_acc * ds));
                speed_limits[i] = std::max(params_.max_vel, relaxed_v);
            }
        }
    }

    // 为每个 dense sample 分配数组，默认继承刚计算好的基准限速线
    std::vector<double> max_velocities = speed_limits;

    // 2.1 前向传播：从起点开始，将物理加速限制叠加到限速线上
    max_velocities[0] = start_speed;

    for (size_t i = 1; i < dense_samples.size(); ++i) {
        double ds = dense_samples[i].equiv_distance -
                    dense_samples[i - 1].equiv_distance;
        // v^2 = v0^2 + 2*a*s => v_max = sqrt(v0^2 + 2*a*s)
        double v_prev = max_velocities[i - 1];
        // 当前允许到达的最高速度
        double v_forward =
            std::sqrt(v_prev * v_prev + 2.0 * params_.max_acc * ds);
        // 合并前向限制与自身基准限制
        max_velocities[i] = std::min(speed_limits[i], v_forward);
    }

    // 2.2 后向传播：从终点开始，考虑减速限制（终点速度为0）
    max_velocities.back() = 0.0;

    for (int i = static_cast<int>(dense_samples.size()) - 2; i >= 0; --i) {
        double ds = dense_samples[i + 1].equiv_distance -
                    dense_samples[i].equiv_distance;
        double v_next = max_velocities[i + 1];
        double v_backward =
            std::sqrt(v_next * v_next + 2.0 * params_.max_acc * ds);
        max_velocities[i] = std::min(max_velocities[i], v_backward);
    }

    // Step 3 计算每个采样点的到达时间
    std::vector<double> arrival_times(dense_samples.size(), 0.0);

    for (size_t i = 1; i < dense_samples.size(); ++i) {
        double ds = dense_samples[i].equiv_distance -
                    dense_samples[i - 1].equiv_distance;
        double v_start = max_velocities[i - 1];
        double v_end = max_velocities[i];

        // 在致密的微元点间，由于速度已经过前后向加速度修整，它必定是合法的匀变速运动。
        // 直接使用匀变速直线运动平均速度计算 t = ds /
        // avg_v，摒弃冗长且有漏洞的宏观梯形逻辑评估。
        double avg_v = 0.5 * (v_start + v_end);
        double dt;

        // 防除零：如果速度过小，说明是刚起步或刚停下，用零初始速度的匀加速运动公式求解
        if (avg_v > 1e-4) {
            dt = ds / avg_v;
        } else {
            dt = std::sqrt(2.0 * ds / params_.max_acc);
        }

        // 兜底保护数值
        dt = std::max(dt, 1e-6);

        arrival_times[i] = arrival_times[i - 1] + dt;
    }

    double total_time = arrival_times.back();
    RCLCPP_DEBUG(logger_, "processPath: 运动学微元积分总时间: %.2f s",
                 total_time);

    double current_real_vel = start_vel.norm();
    // 物理上刹死所需的最小时间 T = V / A_max
    double min_physical_time = current_real_vel / params_.max_acc;

    // 检查1: 是否发生了“隐式降速” (Backward pass修改了起点速度)
    // 检查2: 或者算出的几何时间太短
    if (total_time < min_physical_time) {
        RCLCPP_WARN(logger_,
                    "检测到物理不可行约束: 真实速度 %.2f m/s 需要 %.2f s "
                    "刹停，但几何规划仅分配 %.2f s",
                    current_real_vel, min_physical_time, total_time);

        // 强制使用物理时间，并给予 10% 的余量让优化器更从容
        double safe_total_time = min_physical_time * 1.5;

        // 拉伸时间：这等价于告诉后续步骤“慢慢刹，别急”
        double ratio = safe_total_time / total_time;
        for (auto &t : arrival_times) {  // 虽然 arrival_times
                                         // 后面没用了，但逻辑上保持一致
            t *= ratio;
        }
        total_time = safe_total_time;
    }

    // 兜底：防止时间极小
    total_time = std::max(total_time, 0.1);

    if (total_time < 1e-6) {
        RCLCPP_WARN(logger_, "processPath: 总时间过小，路径无效");
        return result;
    }

    // Step 4: 均匀时间重采样
    double time_resolution = params_.resample_time_resolution;
    int num_waypoints =
        std::max(params_.min_waypoints,
                 static_cast<int>(std::ceil(total_time / time_resolution)) + 1);

    // 确保路点数不超过最大限制
    num_waypoints = std::min(num_waypoints, params_.max_waypoints);

    // 重新计算实际的时间间隔
    double actual_time_interval = total_time / (num_waypoints - 1);

    std::vector<Eigen::Vector2d> resampled_points;
    std::vector<double> resampled_times;
    resampled_points.reserve(num_waypoints);
    resampled_times.reserve(num_waypoints);

    // 使用二分查找 + 线性插值进行时间重采样
    for (int k = 0; k < num_waypoints; ++k) {
        double target_time = k * actual_time_interval;

        // 边界处理
        if (k == 0) {
            resampled_points.push_back(dense_samples.front().position);
            resampled_times.push_back(0.0);
            continue;
        }
        if (k == num_waypoints - 1) {
            resampled_points.push_back(dense_samples.back().position);
            resampled_times.push_back(actual_time_interval);
            continue;
        }

        // 二分查找目标时间所在的段
        auto it = std::lower_bound(arrival_times.begin(), arrival_times.end(),
                                   target_time);
        size_t idx = std::distance(arrival_times.begin(), it);

        if (idx == 0) idx = 1;
        if (idx >= arrival_times.size()) idx = arrival_times.size() - 1;

        // 线性插值
        double t0 = arrival_times[idx - 1];
        double t1 = arrival_times[idx];
        double alpha = (t1 > t0) ? (target_time - t0) / (t1 - t0) : 0.0;
        alpha = std::clamp(alpha, 0.0, 1.0);

        Eigen::Vector2d interp_pos =
            (1.0 - alpha) * dense_samples[idx - 1].position +
            alpha * dense_samples[idx].position;

        resampled_points.push_back(interp_pos);
        resampled_times.push_back(actual_time_interval);  // 均匀时间间隔
    }

    RCLCPP_DEBUG(
        logger_,
        "processPath: 均匀时间重采样完成，%zu 个路点，时间间隔: %.3f s",
        resampled_points.size(), actual_time_interval);

    // Step 5: 构建输出结果
    result.waypoints.resize(2, resampled_points.size());
    for (size_t i = 0; i < resampled_points.size(); ++i) {
        result.waypoints.col(i) = resampled_points[i];
    }

    // 时间分配：每段时间相同（均匀时间间隔）
    result.times.resize(resampled_points.size() - 1);
    for (int i = 0; i < result.times.size(); ++i) {
        result.times(i) = actual_time_interval;
    }

    result.total_time = total_time;
    result.success = true;

    // RCLCPP_INFO(logger_,
    //             "processPath: 路径预处理完成 - %ld 个路点, 总时间 %.2f s, "
    //             "段时间 %.3f s",
    //             result.waypoints.cols(), result.total_time,
    //             actual_time_interval);

    return result;
}

void MincoSmoother::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 构造 PoseStamped
    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;

    geometry_msgs::msg::PoseStamped map_pose;

    // 尝试转换到 map 坐标系
    // 我们需要确保与轨迹（通常在 map 系）一致
    // 采用 TimePointZero 避免阻塞
    if (odom_pose.header.frame_id != "map" && tf_) {
        try {
            auto transform = tf_->lookupTransform(
                "map", odom_pose.header.frame_id, tf2::TimePointZero);
            tf2::doTransform(odom_pose, map_pose, transform);
        } catch (const tf2::TransformException &ex) {
            // 降级处理
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000,
                "[MincoSmoother] 无法将机器人姿态转换到 map 坐标系: %s",
                ex.what());
            map_pose = odom_pose;
        }
    } else {
        map_pose = odom_pose;
    }

    // 更新 2D 位置缓存（供轨迹投影计算使用）
    {
        std::lock_guard<std::mutex> lock(robot_pos_2d_mutex_);
        current_robot_pos_2d_.x() = map_pose.pose.position.x;
        current_robot_pos_2d_.y() = map_pose.pose.position.y;
        robot_pos_timestamp_ = clock_->now().seconds();
    }
}

}  // namespace pb_minco

// 注册 Nav2 插件
PLUGINLIB_EXPORT_CLASS(pb_minco::MincoSmoother, nav2_core::Smoother)
