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
 * @brief 使用 Jump Point Search 的 Nav2 全局规划器插件。
 *
 * 实现 nav2_core::GlobalPlanner 接口, 用 ROS2 生命周期管理
 * 和代价地图访问封装无状态 JPSAlgorithm。
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
   * @brief 在跳转点航点之间线性插值, 使输出路径间距匹配代价地图分辨率。
   * @param raw_path  地图坐标下的稀疏路径。
   * @param resolution  期望的航点间距 (米)。
   * @return 密集采样的 nav_msgs::Path。
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
