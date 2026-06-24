#include "jps_planner/jps_planner.hpp"

#include <cmath>
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

namespace jps_planner
{

void JPSPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("JPSPlanner: failed to lock parent node");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  // 以规划器名称为前缀声明 JPS 参数
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_traversal_cost",
    rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_euc_cost",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_heuristic_cost",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown",
    rclcpp::ParameterValue(false));

  // B-spline 平滑参数
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enable_bspline",
    rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".w_traversal_cost", config_.w_traversal_cost);
  node->get_parameter(name_ + ".w_euc_cost", config_.w_euc_cost);
  node->get_parameter(name_ + ".w_heuristic_cost", config_.w_heuristic_cost);
  node->get_parameter(name_ + ".allow_unknown", config_.allow_unknown);
  node->get_parameter(name_ + ".enable_bspline", enable_bspline_);

  RCLCPP_INFO(
    logger_, "JPSPlanner configured: w_traversal=%.2f w_euc=%.2f "
    "w_heuristic=%.2f allow_unknown=%d enable_bspline=%d",
    config_.w_traversal_cost, config_.w_euc_cost,
    config_.w_heuristic_cost, config_.allow_unknown, enable_bspline_);

  // 构建共享内存结构体来传输路径信息
  bool ok = shm_writer_.init("guga_shm", guga_ui::UiSlotId::PATH);
  if (!ok) {
    RCLCPP_ERROR(logger_, "ShmWriter init failed, UI path display unavailable");
}

}

void JPSPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "JPSPlanner: cleaning up");
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
  is_active_ = false;
}

void JPSPlanner::activate()
{
  RCLCPP_INFO(logger_, "JPSPlanner: activating");
  is_active_ = true;
}

void JPSPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "JPSPlanner: deactivating");
  is_active_ = false;
}

nav_msgs::msg::Path JPSPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path plan;
  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  if (!is_active_) {
    RCLCPP_WARN(logger_, "JPSPlanner: not active, returning empty plan");
    return plan;
  }

  // 验证代价地图
  if (costmap_ == nullptr) {
    RCLCPP_ERROR(logger_, "JPSPlanner: costmap is null");
    return plan;
  }

  // 将起点转换为地图坐标
  unsigned int mx_start{}, my_start{};
  if (!costmap_->worldToMap(
        start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
    RCLCPP_ERROR(
      logger_, "JPSPlanner: start pose (%.2f, %.2f) out of costmap bounds",
      start.pose.position.x, start.pose.position.y);
    return plan;
  }

  // 将终点转换为地图坐标
  unsigned int mx_goal{}, my_goal{};
  if (!costmap_->worldToMap(
        goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    RCLCPP_ERROR(
      logger_, "JPSPlanner: goal pose (%.2f, %.2f) out of costmap bounds",
      goal.pose.position.x, goal.pose.position.y);
    return plan;
  }

  int sx = static_cast<int>(mx_start);
  int sy = static_cast<int>(my_start);
  int gx = static_cast<int>(mx_goal);
  int gy = static_cast<int>(my_goal);

  RCLCPP_INFO(
    logger_, "JPSPlanner: planning from (%d, %d) to (%d, %d) [cells]", sx, sy,
    gx, gy);

  // 准备算法状态
  JPSState state{};
  state.costmap_data = costmap_->getCharMap();
  state.size_x = static_cast<int>(costmap_->getSizeInCellsX());
  state.size_y = static_cast<int>(costmap_->getSizeInCellsY());

  // 运行 JPS
  std::vector<std::pair<double, double>> map_path{};
  bool found = JPSAlgorithm::generatePath(config_, state, sx, sy, gx, gy, map_path);

  if (!found) {
    RCLCPP_WARN(logger_, "JPSPlanner: no path found");
    return plan;
  }

  RCLCPP_INFO(
    logger_, "JPSPlanner: path found with %zu waypoints", map_path.size());

  // ── B-spline 平滑 (默认启用) ──
  if (enable_bspline_ && map_path.size() >= 8) {
    plan = bsplineSmooth(map_path, costmap_->getCharMap(),
                         static_cast<int>(costmap_->getSizeInCellsX()),
                         static_cast<int>(costmap_->getSizeInCellsY()),
                         costmap_->getResolution());
    RCLCPP_INFO(logger_, "JPSPlanner: B-spline smooth applied, %zu poses",
                plan.poses.size());
  } else {
    // 回退到线性插值 (路径点太少或 B-spline 被禁用)
    if (enable_bspline_) {
      RCLCPP_INFO(
        logger_, "JPSPlanner: too few waypoints (%zu) for B-spline, linear fallback",
        map_path.size());
    }
    std::vector<std::pair<double, double>> world_path{};
    world_path.reserve(map_path.size());
    for (const auto & [mx, my] : map_path) {
      double wx{}, wy{};
      costmap_->mapToWorld(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
      world_path.emplace_back(wx, wy);
    }
    plan = linearInterpolation(world_path, costmap_->getResolution());
  }

  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  return plan;
}

nav_msgs::msg::Path JPSPlanner::bsplineSmooth(
  const std::vector<std::pair<double, double>> & map_path,
  const unsigned char * costmap_data, int cm_w, int cm_h,
  double resolution)
{
  nav_msgs::msg::Path plan;

  // 创建 B-spline 优化器并拟合原始 JPS 路径
  bspline_opt::BSplineOptimizer opt(bspline_config_);
  if (!opt.fit(map_path)) {
    RCLCPP_WARN(logger_, "JPSPlanner: B-spline fit failed, using linear interpolation");
    std::vector<std::pair<double, double>> world_path{};
    world_path.reserve(map_path.size());
    for (const auto & [mx, my] : map_path) {
      double wx{}, wy{};
      costmap_->mapToWorld(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
      world_path.emplace_back(wx, wy);
    }
    return linearInterpolation(world_path, resolution);
  }

  // 注入代价地图数据用于障碍物避让
  opt.state().costmap_data = costmap_data;
  opt.state().costmap_w = cm_w;
  opt.state().costmap_h = cm_h;

  // 运行优化 (默认仅做障碍物投射, 不做梯度下降)
  int num_samples = std::max(100, static_cast<int>(map_path.size()) * 5);
  auto result = opt.optimize(num_samples);

  // 将地图坐标平滑路径转换为世界坐标 nav_msgs::Path
  plan.poses.reserve(result.smoothed_path.size());
  for (size_t i = 0; i < result.smoothed_path.size(); ++i) {
    double wx{}, wy{};
    costmap_->mapToWorld(
      static_cast<unsigned int>(result.smoothed_path[i].first),
      static_cast<unsigned int>(result.smoothed_path[i].second),
      wx, wy);
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    // 朝向: 指向下一个航点 (绕 Z 轴的偏航角)
    double yaw{};
    if (i + 1 < result.smoothed_path.size()) {
      double dx = result.smoothed_path[i + 1].first -
                  result.smoothed_path[i].first;
      double dy = result.smoothed_path[i + 1].second -
                  result.smoothed_path[i].second;
      yaw = std::atan2(dy, dx);
    }
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    plan.poses.push_back(pose);
  }

  // 将 smoothed_path 写入共享内存, 供 UI 显示
  path_data_.stamp_sec = clock_->now().seconds();
  path_data_.count = std::min(plan.poses.size(), guga_ui::UI_PATH_MAX_POINTS);
  for (size_t i = 0; i < path_data_.count; ++i) {
    path_data_.x[i] = plan.poses[i].pose.position.x;
    path_data_.y[i] = plan.poses[i].pose.position.y;
  }
  shm_writer_.write(&path_data_, sizeof(path_data_));

  return plan;
}

nav_msgs::msg::Path JPSPlanner::linearInterpolation(
  const std::vector<std::pair<double, double>> & raw_path, double resolution)
{
  nav_msgs::msg::Path plan;
  if (raw_path.empty()) { return plan; }
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
      pose.pose.position.x = x0 + t * (x1 - x0);
      pose.pose.position.y = y0 + t * (y1 - y0);
      pose.pose.position.z = 0.0;
      // 简单的朝向: 指向下一个航点
      double yaw = std::atan2(y1 - y0, x1 - x0);
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
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

}  // namespace jps_planner

PLUGINLIB_EXPORT_CLASS(jps_planner::JPSPlanner, nav2_core::GlobalPlanner)
