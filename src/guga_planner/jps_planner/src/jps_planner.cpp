#include "jps_planner/jps_planner.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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

  // Declare parameters with the planner name as prefix
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

  node->get_parameter(name_ + ".w_traversal_cost", config_.w_traversal_cost);
  node->get_parameter(name_ + ".w_euc_cost", config_.w_euc_cost);
  node->get_parameter(name_ + ".w_heuristic_cost", config_.w_heuristic_cost);
  node->get_parameter(name_ + ".allow_unknown", config_.allow_unknown);

  RCLCPP_INFO(
    logger_, "JPSPlanner configured: w_traversal=%.2f w_euc=%.2f w_heuristic=%.2f allow_unknown=%d",
    config_.w_traversal_cost, config_.w_euc_cost,
    config_.w_heuristic_cost, config_.allow_unknown);
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

  // Validate costmap
  if (costmap_ == nullptr) {
    RCLCPP_ERROR(logger_, "JPSPlanner: costmap is null");
    return plan;
  }

  // Convert start to map coordinates
  unsigned int mx_start{}, my_start{};
  if (!costmap_->worldToMap(
        start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
    RCLCPP_ERROR(
      logger_, "JPSPlanner: start pose (%.2f, %.2f) out of costmap bounds",
      start.pose.position.x, start.pose.position.y);
    return plan;
  }

  // Convert goal to map coordinates
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

  // Prepare algorithm state
  JPSState state{};
  state.costmap_data = costmap_->getCharMap();
  state.size_x = static_cast<int>(costmap_->getSizeInCellsX());
  state.size_y = static_cast<int>(costmap_->getSizeInCellsY());

  // Run JPS
  std::vector<std::pair<double, double>> map_path{};
  bool found = JPSAlgorithm::generatePath(config_, state, sx, sy, gx, gy, map_path);

  if (!found) {
    RCLCPP_WARN(logger_, "JPSPlanner: no path found");
    return plan;
  }

  RCLCPP_INFO(
    logger_, "JPSPlanner: path found with %zu waypoints", map_path.size());

  // Convert map coordinates to world coordinates
  std::vector<std::pair<double, double>> world_path{};
  world_path.reserve(map_path.size());
  for (const auto & [mx, my] : map_path) {
    double wx{}, wy{};
    costmap_->mapToWorld(
      static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
    world_path.emplace_back(wx, wy);
  }

  // Interpolate to get a dense path at costmap resolution
  double resolution = costmap_->getResolution();
  plan = linearInterpolation(world_path, resolution);
  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

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
      // Simple orientation: point toward next waypoint
      double yaw = std::atan2(y1 - y0, x1 - x0);
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      plan.poses.push_back(pose);
    }
  }

  // Add the final point
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
