#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "guga_decision_common/waypoint_navigator.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace guga_decision_common
{

enum class NavigationResult
{
  NONE,
  PENDING,
  SUCCEEDED,
  FAILED,
  CANCELED,
};

// 面向 Nav2 NavigateToPose action 的导航目标客户端。
//
// WaypointNavigator 只负责航点顺序，本类负责发送 action、接收 result 并维护
// RViz Marker。与 goal_pose 话题不同，action result 可以明确区分导航成功、
// 失败和取消，因此不需要额外查询 TF 来猜测是否到点。
class GoalController
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GoalController(
    rclcpp::Node& node,
    const std::string& action_name = "navigate_to_pose",
    const std::string& frame_id = "map",
    const std::string& marker_topic = "goal_pose_marker");

  // 返回 true 表示 action goal 已发送；最终结果通过 pollResult() 获取。
  // 只有轮询到 SUCCEEDED 才表示 Nav2 确认导航完成。
  bool sendGoal(const Waypoint2D& goal, bool force = false);

  // PENDING 不会被消费；终态只返回一次，避免重复推进航点。
  NavigationResult pollResult();
  void cancelGoal();

  const std::string& frameId() const { return frame_id_; }

private:
  void publishMarker(const Waypoint2D& goal);

  rclcpp::Node* node_;
  std::string action_name_;
  std::string frame_id_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    marker_publisher_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  std::mutex mutex_;
  NavigationResult result_{NavigationResult::NONE};
  GoalHandle::SharedPtr current_goal_;
  std::uint64_t generation_{0};

  Waypoint2D last_goal_;
  bool has_last_goal_{false};
};

}  // namespace guga_decision_common
