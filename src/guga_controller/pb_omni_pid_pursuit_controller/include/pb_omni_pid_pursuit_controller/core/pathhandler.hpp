#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <memory>
#include <tf2_ros/buffer.h>
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <vector>
#include <iterator>

class PathHandler
{
public:

  void initialize(
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
    const rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr & pub);

  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped& pose,
    nav_msgs::msg::Path& global_plan);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  tf2::Duration transform_tolerance_;

  rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitControllerNode")};

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    local_path_pub_;

  double max_robot_pose_search_dist_;

  bool transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose)const;

  double getCostmapMaxExtent() const;
};