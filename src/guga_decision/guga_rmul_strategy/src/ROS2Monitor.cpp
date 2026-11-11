#include "ROS2Monitor.hpp"

#include <string>

ROS2Monitor::ROS2Monitor(const rclcpp::NodeOptions& options)
    : guga_decision_common::DecisionInputHub("ros2_monitor", options) {
  const std::string goal_topic = declare_parameter<std::string>(
      "goal_pose_topic", "goal_pose");

  // 与 Nav2 bt_navigator 的订阅 QoS 保持一致。
  goal_pub_ = create_publisher<PoseStamped>(goal_topic,
                                            rclcpp::SensorDataQoS());

  const std::string stop_topic = declare_parameter<std::string>(
      "stop_topic", "/chassis_stop");
  stop_pub_ = create_publisher<std_msgs::msg::Bool>(stop_topic,
                                                    rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(), "决策输出话题: goal=%s(坐标系 %s),stop=%s",
              goal_topic.c_str(), frameId().c_str(), stop_topic.c_str());
}

void ROS2Monitor::publishStop(bool stop) {
  std_msgs::msg::Bool msg;
  msg.data = stop;
  stop_pub_->publish(msg);

  if (!has_last_stop_ || stop != last_stop_) {
    RCLCPP_INFO(get_logger(), "停车指令: %s", stop ? "要求停" : "解除");
    last_stop_ = stop;
    has_last_stop_ = true;
  }
}

bool ROS2Monitor::sendGoalPose(double x, double y, bool force) {
  if (!force && has_last_goal_ && x == last_goal_x_ && y == last_goal_y_) {
    return false;
  }

  PoseStamped goal;
  goal.header.frame_id = frameId();
  goal.header.stamp = now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;

  // 默认单位四元数对应 yaw=0。当前控制器把 yaw 容差设成 2π，因此这里只表达
  // 合法朝向；以后需要约束朝向时再给目标 yaw 增加参数。
  goal.pose.orientation.w = 1.0;
  goal_pub_->publish(goal);

  last_goal_x_ = x;
  last_goal_y_ = y;
  has_last_goal_ = true;

  RCLCPP_INFO(get_logger(), "发布目标点 (%.2f, %.2f)，坐标系 %s", x, y,
              frameId().c_str());
  return true;
}
