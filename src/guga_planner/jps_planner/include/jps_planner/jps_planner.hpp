#pragma once

#include <memory>
#include <string>
#include <vector>

#include "jps_planner/jps_algorithm.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace jps_planner
{

/**
 * @class JPSPlanner
 * @brief Nav2 global planner plugin using Jump Point Search.
 *
 * Implements the nav2_core::GlobalPlanner interface. Wraps the stateless
 * JPSAlgorithm with ROS2 lifecycle management and costmap access.
 */
class JPSPlanner : public nav2_core::GlobalPlanner
{
public:
  JPSPlanner() = default;
  ~JPSPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  /**
   * @brief Interpolate between jump-point waypoints to match costmap resolution.
   * @param raw_path  Sparse path in world coordinates.
   * @param resolution  Desired spacing between waypoints (meters).
   * @return Densely sampled nav_msgs::Path.
   */
  static nav_msgs::msg::Path linearInterpolation(
    const std::vector<std::pair<double, double>> & raw_path,
    double resolution);

  std::shared_ptr<tf2_ros::Buffer> tf_{};
  rclcpp::Clock::SharedPtr clock_{};
  rclcpp::Logger logger_{rclcpp::get_logger("JPSPlanner")};

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_{};
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  std::string global_frame_{};
  std::string name_{};
  bool is_active_{false};

  JPSConfig config_{};
};

}  // namespace jps_planner
