#ifndef NAV2_WRAPPER_HPP_
#define NAV2_WRAPPER_HPP_

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <memory>
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include <vector>
#include <iterator>
#include <optional>

namespace mpc_controller {

class NavWrapper
{
public:
  explicit NavWrapper(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub,
    double transform_tolerance = 1.0);

  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose, nav_msgs::msg::Path& global_plan);

  [[nodiscard]] double getCostmapMaxExtent() const;

  [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> transformPose(
    const std::string& frame,
    const geometry_msgs::msg::PoseStamped& in_pose) const;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  tf2::Duration transform_tolerance_;
  rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitControllerNode")};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
      local_path_pub_;
  double max_robot_pose_search_dist_;
};

}  // namespace mpc_controller

#endif  // NAV2_WRAPPER_HPP_