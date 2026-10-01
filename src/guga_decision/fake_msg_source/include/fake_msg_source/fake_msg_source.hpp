#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "guga_interfaces/msg/robot_status.hpp"
#include "guga_interfaces/msg/vision_info.hpp"

// 假数据源：把决策需要的外部输入一次性凑齐，便于不开仿真、不接实车时联调。
//
// 目前两路：
//   referee/robot_status   本机裁判状态（血量、发弹量、热量…）
//   vision/info            视觉汇总（看见几个敌人）
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
  using VisionInfoMsg = guga_interfaces::msg::VisionInfo;

  void publish_robot_status();
  void publish_vision_info();
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

  rclcpp::Publisher<RobotStatusMsg>::SharedPtr status_pub_;
  rclcpp::Publisher<VisionInfoMsg>::SharedPtr vision_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr vision_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  // 用来判断血量是否下降，进而填 is_hp_deduced
  std::uint16_t last_hp_{0};
};
