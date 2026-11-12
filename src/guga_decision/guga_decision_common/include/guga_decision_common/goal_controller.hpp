#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "guga_decision_common/waypoint_navigator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace guga_decision_common
{

// 面向 goal_pose 话题的最小导航目标发布器。
//
// WaypointNavigator 只负责航点顺序，本类负责把 Waypoint2D 转成 PoseStamped
// 并发布。以后改用 NavigateToPose action 时，可以新增一个相同接口的实现，
// 不需要修改航点状态机。
class GoalController
{
public:
  GoalController(
    rclcpp::Node& node,
    const std::string& topic = "goal_pose",
    const std::string& frame_id = "map");

  // 返回 true 表示该目标已经处于有效状态。相同目标被去重时也返回 true，
  // 因为 Nav2 此时已经在执行这个目标。
  bool sendGoal(const Waypoint2D& goal, bool force = false);

  const std::string& frameId() const { return frame_id_; }

private:
  rclcpp::Node* node_;
  std::string frame_id_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    marker_publisher_;
  Waypoint2D last_goal_;
  bool has_last_goal_{false};
};

}  // namespace guga_decision_common
