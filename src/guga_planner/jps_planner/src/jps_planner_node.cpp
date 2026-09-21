#include "jps_planner/jps_planner.hpp"

#include <algorithm>
#include <cmath>
#include <guga_ui_common/ui_types.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "bspline_opt/bspline_optimizer.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rog_map_layer/esdf_layer.hpp"
#include "rog_map_layer/esdf_map.hpp"

namespace jps_planner {
  // ══════════════════════════════════════════════════════════════════════════════
  // configure — 读取 ROS 参数, 初始化状态
  // ══════════════════════════════════════════════════════════════════════════════

  void JPSPlanner::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("JPSPlanner: failed to lock parent node");
    }

    // 保存 ROS 基础设施的引用
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros->getCostmap();  // 获取底层 Costmap2D 原始指针
    global_frame_ = costmap_ros_->getGlobalFrameID();
    clock_ = node->get_clock();
    logger_ = node->get_logger();

    // ── JPS 搜索参数 ──
    // w_traversal_cost: 单格元通行代价权重 (Theta* 缩放后)
    // w_euc_cost:       跳转点间欧几里得距离代价权重
    // w_heuristic_cost: A* 启发式权重 (控制贪心程度)
    // allow_unknown:    是否允许穿越未知空间 (255 = NO_INFORMATION)
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".w_traversal_cost", rclcpp::ParameterValue(10.0));
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".w_euc_cost",
                                                 rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".w_heuristic_cost", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".allow_unknown",
                                                 rclcpp::ParameterValue(false));

    // B-spline 平滑开关
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".enable_bspline", rclcpp::ParameterValue(true));

    node->get_parameter(name_ + ".w_traversal_cost", config_.w_traversal_cost);
    node->get_parameter(name_ + ".w_euc_cost", config_.w_euc_cost);
    node->get_parameter(name_ + ".w_heuristic_cost", config_.w_heuristic_cost);
    node->get_parameter(name_ + ".allow_unknown", config_.allow_unknown);
    node->get_parameter(name_ + ".enable_bspline", enable_bspline_);

    // ── ESDF 梯度优化参数 ──
    // enable_esdf:        是否启用 ESDF 辅助优化
    // esdf_weight:        ESDF 距离代价权重 w_e
    // esdf_safe_distance: 安全距离 d_safe (米),
    //                     路径离障碍物小于此距离时开始产生惩罚
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".enable_esdf",
                                                 rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".esdf_weight",
                                                 rclcpp::ParameterValue(100.0));
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".esdf_safe_distance", rclcpp::ParameterValue(0.6));
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".corridor_halfwidth", rclcpp::ParameterValue(8.0));
    // ── B-spline 优化权重 (归一化代价: 各分量除以其初始值) ──
    // smoothness_weight: 曲率能量代价权重, 越大拐弯越圆滑
    // distance_weight:   偏离原始 JPS 航点的代价权重, 越大越贴原路径
    // max_control_points: B-spline 控制点数, 越小近似越强、拐弯越明显
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".smoothness_weight", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".distance_weight", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".max_control_points", rclcpp::ParameterValue(200));
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".max_iterations", rclcpp::ParameterValue(200));
    // B-spline 平滑路径碰撞判定阈值 (253 严格 / 254 宽松)
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".collision_cost_threshold", rclcpp::ParameterValue(253));
    node->get_parameter(name_ + ".enable_esdf", enable_esdf_);
    node->get_parameter(name_ + ".esdf_weight", esdf_weight_);
    node->get_parameter(name_ + ".esdf_safe_distance", esdf_safe_distance_);
    node->get_parameter(name_ + ".corridor_halfwidth", corridor_halfwidth_);
    bspline_config_.corridor_halfwidth = corridor_halfwidth_;
    node->get_parameter(name_ + ".smoothness_weight",
                        bspline_config_.smoothness_weight);
    node->get_parameter(name_ + ".distance_weight",
                        bspline_config_.distance_weight);
    node->get_parameter(name_ + ".max_control_points",
                        bspline_config_.max_control_points);
    node->get_parameter(name_ + ".max_iterations",
                        bspline_config_.max_iterations);
    node->get_parameter(name_ + ".collision_cost_threshold",
                        collision_cost_threshold_);

    RCLCPP_INFO(
        logger_,
        "JPSPlanner configured: w_traversal=%.2f w_euc=%.2f "
        "w_heuristic=%.2f allow_unknown=%d enable_bspline=%d enable_esdf=%d "
        "esdf_safe_distance=%.2f corridor_halfwidth=%.1f "
        "collision_threshold=%d smoothness_w=%.2f distance_w=%.2f "
        "max_ctrl_pts=%d max_iter=%d",
        config_.w_traversal_cost, config_.w_euc_cost, config_.w_heuristic_cost,
        config_.allow_unknown, enable_bspline_, enable_esdf_,
        esdf_safe_distance_, corridor_halfwidth_, collision_cost_threshold_,
        bspline_config_.smoothness_weight, bspline_config_.distance_weight,
        bspline_config_.max_control_points, bspline_config_.max_iterations);

    // 初始化共享内存写入端 — 将规划结果推送给 Pangolin UI 渲染
    shm_ready_ = shm_writer_.init("guga_shm", guga_ui::UiSlotId::PATH);
    if (!shm_ready_) {
      RCLCPP_ERROR(logger_,
                   "ShmWriter init failed, UI path display unavailable");
    } else {
      RCLCPP_INFO(logger_, "ShmWriter initialized for UI path display");
    }
  }

  void JPSPlanner::cleanup() {
    RCLCPP_INFO(logger_, "JPSPlanner: cleaning up");
    costmap_ = nullptr;
    costmap_ros_.reset();
    tf_.reset();
    is_active_ = false;
  }

  void JPSPlanner::activate() {
    RCLCPP_INFO(logger_, "JPSPlanner: activating");
    is_active_ = true;
  }

  void JPSPlanner::deactivate() {
    RCLCPP_INFO(logger_, "JPSPlanner: deactivating");
    is_active_ = false;
  }

  nav_msgs::msg::Path JPSPlanner::linearInterpolation(
      const std::vector<std::pair<double, double>>& raw_path,
      double resolution) {
    nav_msgs::msg::Path plan;
    if (raw_path.empty()) {
      return plan;
    }
    if (raw_path.size() == 1) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = raw_path[0].first;
      pose.pose.position.y = raw_path[0].second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.poses.push_back(pose);
      return plan;
    }

    plan.poses.reserve(raw_path.size() * 2);

    for (size_t i = 0; i < raw_path.size() - 1; ++i) {
      double x0 = raw_path[i].first;
      double y0 = raw_path[i].second;
      double x1 = raw_path[i + 1].first;
      double y1 = raw_path[i + 1].second;

      double dist = std::hypot(x1 - x0, y1 - y0);
      int steps = std::max(1, static_cast<int>(std::ceil(dist / resolution)));

      for (int s = 0; s < steps; ++s) {
        double t = static_cast<double>(s) / static_cast<double>(steps);
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = x0 + (t * (x1 - x0));
        pose.pose.position.y = y0 + (t * (y1 - y0));
        pose.pose.position.z = 0.0;
        // 简单的朝向: 指向下一个航点
        double yaw = std::atan2(y1 - y0, x1 - x0);
        pose.pose.orientation =
            nav2_util::geometry_utils::orientationAroundZAxis(yaw);
        plan.poses.push_back(pose);
      }
    }

    // 添加最后一个点
    geometry_msgs::msg::PoseStamped final_pose;
    final_pose.pose.position.x = raw_path.back().first;
    final_pose.pose.position.y = raw_path.back().second;
    final_pose.pose.position.z = 0.0;
    final_pose.pose.orientation = plan.poses.back().pose.orientation;
    plan.poses.push_back(final_pose);

    return plan;
  }

  void JPSPlanner::writePathToShm(const nav_msgs::msg::Path& plan) {
    if (!shm_ready_) {
      return;
    }

    const size_t n = plan.poses.size();
    if (n == 0) {
      return;
    }

    guga_ui::UiPath ui_path{};
    ui_path.stamp_sec = clock_->now().seconds();

    // 降采样: 每 stride 个点取 1 个 (目标 ≤ UI_PATH_MAX_POINTS)
    size_t stride = (n <= guga_ui::UI_PATH_MAX_POINTS)
                        ? 1
                        : ((n / guga_ui::UI_PATH_MAX_POINTS) + 1);
    ui_path.count = std::min((n + stride - 1) / stride,
                             guga_ui::UI_PATH_MAX_POINTS);

    for (uint32_t i = 0; i < ui_path.count; ++i) {
      size_t src = i * stride;
      ui_path.x[i] = plan.poses[src].pose.position.x;
      ui_path.y[i] = plan.poses[src].pose.position.y;
    }

    shm_writer_.write(&ui_path, sizeof(ui_path));
  }

}  // namespace jps_planner

PLUGINLIB_EXPORT_CLASS(jps_planner::JPSPlanner, nav2_core::GlobalPlanner)
