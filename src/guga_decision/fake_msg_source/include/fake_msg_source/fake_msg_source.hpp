#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "guga_interfaces/msg/rfid_status.hpp"
#include "guga_interfaces/msg/robot_status.hpp"
#include "guga_interfaces/msg/vision_info.hpp"

// 假数据源：把决策需要的外部输入一次性凑齐，便于不开仿真、不接实车时联调。
//
// 四路：
//   /referee/robot_status   本机裁判状态（血量、发弹量、热量…）
//   /referee/rfid_status    RFID 增益点状态（"到家/占点"是否真的到了）
//   vision/info             视觉汇总（看见几个敌人）
//   odometry + TF           假位姿：odom -> base_footprint 随动，map -> odom 静态单位变换
//
// 裁判两路用绝对话题名，与实车 serial_driver_node 发布的名称一致；视觉与位姿不是
// 裁判数据，实车侧用的是相对名，所以这里也保持相对名。
//
// 位姿是决策判断"是否到达"的输入（ROS2Monitor 查 map -> base_footprint）。没有它
// 时 robot_x/robot_y 一直是 NaN，到达判定永远为假，占点分支一辈子走不进去，所以
// 假数据源自己补上这一段，不必再手工起 static_transform_publisher。
// 只在不接仿真、不接实车时使用：真机上 odom -> base_footprint 由驱动发、map -> odom
// 由定位发，同时跑会争同一个变换。
//
// 裁判字段的取值依据 U26 表 3-4 与 P26 表 1-12：哨兵 ID 7、血量上限 400、
// 枪管热量上限 260、冷却 30/秒。文档确认不了的三项（等级、允许发弹量、金币）
// 默认值保守，运行时可改。
//
// 所有字段都是 ROS 2 参数且逐帧读取，所以 ros2 param set 改完下一帧即生效。
class FakeMsgSource : public rclcpp::Node
{
public:
  FakeMsgSource();

private:
  using RobotStatusMsg = guga_interfaces::msg::RobotStatus;
  using RfidStatusMsg = guga_interfaces::msg::RfidStatus;
  using VisionInfoMsg = guga_interfaces::msg::VisionInfo;
  using OdometryMsg = nav_msgs::msg::Odometry;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

  void publish_robot_status();
  void publish_rfid_status();
  void publish_vision_info();
  void publish_pose();
  // 朝最近一次收到的 goal_pose 走一步；速度 0 表示不动，位置完全由参数决定。
  void step_pose(double dt);
  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
    const std::vector<rclcpp::Parameter>& params);

  // 每帧都重新读参数，所以运行中改动立刻生效，不需要额外缓存。
  std::int64_t param_int(const std::string& name) const
  {
    return get_parameter(name).as_int();
  }
  double param_double(const std::string& name) const
  {
    return get_parameter(name).as_double();
  }
  bool param_bool(const std::string& name) const
  {
    return get_parameter(name).as_bool();
  }
  std::string param_str(const std::string& name) const
  {
    return get_parameter(name).as_string();
  }

  rclcpp::Publisher<RobotStatusMsg>::SharedPtr status_pub_;
  rclcpp::Publisher<RfidStatusMsg>::SharedPtr rfid_pub_;
  rclcpp::Publisher<VisionInfoMsg>::SharedPtr vision_pub_;
  rclcpp::Publisher<OdometryMsg>::SharedPtr odom_pub_;
  rclcpp::Subscription<PoseStampedMsg>::SharedPtr goal_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr vision_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  // 用来判断血量是否下降，进而填 is_hp_deduced
  std::uint16_t last_hp_{0};

  // 最近一次收到的目标点。速度非 0 时机器人朝它走，走到就不再动。
  double goal_x_{0.0};
  double goal_y_{0.0};
  bool has_goal_{false};
};
