#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "guga_interfaces/msg/robot_status.hpp"

// 假裁判系统：按固定频率发布本机 RobotStatus，字段用 ROS 2 参数动态调整。
//
// 取值依据 U26 表 3-4 与 P26 表 1-12（本机性能体系 0x0201）：
// 哨兵 ID = 7，血量上限 400，枪管热量上限 260、冷却 30/秒。
// 文档无法确认的字段（等级、允许发弹量、金币）默认值保守，运行时可改。
class FakeRefereeNode : public rclcpp::Node
{
public:
  FakeRefereeNode();

private:
  using RobotStatusMsg = guga_interfaces::msg::RobotStatus;

  void publish_status();
  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
    const std::vector<rclcpp::Parameter>& params);

  // 每帧都重新读参数，所以 ros2 param set 立即生效，不需要额外缓存。
  std::int64_t param_int(const std::string& name) const
  {
    return get_parameter(name).as_int();
  }

  rclcpp::Publisher<RobotStatusMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  // 用来判断血量是否下降，进而填 is_hp_deduced
  std::uint16_t last_hp_{0};
};
