#include <memory>  
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/publisher.hpp"
namespace visualization_helper
{
  std::unique_ptr<geometry_msgs::msg::PointStamped>
  createCarrotMsg(const geometry_msgs::msg::PoseStamped & carrot_pose);
  
  void visualizeCurvaturePoints(
  const geometry_msgs::msg::PoseStamped & backward_pose,
  const geometry_msgs::msg::PoseStamped & forward_pose,
  const rclcpp_lifecycle::LifecyclePublisher<
  visualization_msgs::msg::MarkerArray>::SharedPtr & pub);

};