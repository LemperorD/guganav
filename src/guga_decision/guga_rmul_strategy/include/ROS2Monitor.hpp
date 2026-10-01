#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "guga_interfaces/msg/robot_status.hpp"
#include "guga_interfaces/msg/vision_info.hpp"
#include "rclcpp/rclcpp.hpp"

// 决策节点的 ROS 2 侧接口，集中持有本包用到的全部 ROS 资源：
// 订阅裁判数据，以及向导航发布目标点。
//
// 之所以都收在这一个类里：它在 main 里只创建一次，而行为树节点
// （ROS2Wrapper、SetGoalPose）在树上可以有多个实例。publisher 与
// declare_parameter 若放在行为树节点里，会因为树上存在同类实例而重复声明，
// 加载时抛 ParameterAlreadyDeclaredException。
//
// 它不认识行为树，不碰黑板，便于单独测试。
class ROS2Monitor : public rclcpp::Node
{
public:
  explicit ROS2Monitor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // 裁判数据：返回最近一次收到的值。
  std::uint16_t currentHp() const { return current_hp_.load(); }
  std::uint16_t maximumHp() const { return maximum_hp_.load(); }

  // 允许发弹量。它在同一条裁判消息里，所以 hasData() 为真时这个值也是有效的。
  std::uint16_t projectileAllowance() const { return projectile_allowance_.load(); }

  // 是否已经收到过至少一条消息。没收到时血量是 0，判断前应当先看这个标志。
  bool hasData() const { return has_data_.load(); }

  // 视觉汇总：视野内的敌方机器人数量，> 0 表示有敌人。
  std::int32_t enemyCount() const { return enemy_count_.load(); }

  // 发布导航目标点。目标与上次相同时不重复发布，返回 false 表示这次没有发。
  bool sendGoalPose(double x, double y);

private:
  using RobotStatusMsg = guga_interfaces::msg::RobotStatus;
  using VisionInfo = guga_interfaces::msg::VisionInfo;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  rclcpp::Subscription<RobotStatusMsg>::SharedPtr sub_state_;
  rclcpp::Subscription<VisionInfo>::SharedPtr sub_vision_;
  
  std::atomic<std::uint16_t> current_hp_{0};
  std::atomic<std::uint16_t> maximum_hp_{0};
  std::atomic<std::uint16_t> projectile_allowance_{0};
  std::atomic<std::int32_t> enemy_count_{0};
  std::atomic<bool> has_data_{false};

  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pub_;
  std::string frame_id_;

  // 上次发布过的目标点，用来判断是否真的变了。
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  bool has_last_goal_{false};
};
