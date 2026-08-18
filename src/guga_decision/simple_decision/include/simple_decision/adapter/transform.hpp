#pragma once

#include "guga_interfaces/msg/armors.hpp"
#include "guga_interfaces/msg/target.hpp"
#include "guga_interfaces/msg/game_status.hpp"
#include "guga_interfaces/msg/robot_status.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "simple_decision/core/types.hpp"

namespace simple_decision {
  using RobotStatusMsg = guga_interfaces::msg::RobotStatus;
  using GameStatusMsg = guga_interfaces::msg::GameStatus;
  using ArmorsMsg = guga_interfaces::msg::Armors;
  using ArmorMsg = guga_interfaces::msg::Armor;
  using TargetMsg = guga_interfaces::msg::Target;

  /// Convert ROS Quaternion message to domain Quaternion
  Quaternion ConvertQuaternion(const geometry_msgs::msg::Quaternion& ros_quat);//四元数转换

  /// Convert ROS Point message to domain Position
  Position ConvertPoint(const geometry_msgs::msg::Point& ros_point);//点坐标转换

  Pose3D ConvertPose(const geometry_msgs::msg::Pose& ros_pose);//位姿转换

  /// Convert ROS RobotStatus message to domain RobotStatus
  /// @throws std::invalid_argument if msg is nullptr
  RobotStatus ConvertRobotStatus(
      guga_interfaces::msg::RobotStatus::SharedPtr msg);//机器人状态转换（核心）

  /// Convert ROS GameStatus message to domain GameStatus
  /// @throws std::invalid_argument if msg is nullptr
  GameStatus ConvertGameStatus(guga_interfaces::msg::GameStatus::SharedPtr msg);//比赛状态转换

  /// Convert ROS Armors message to domain Armors
  /// @throws std::invalid_argument if msg is nullptr
  Armors ConvertArmors(
      const guga_interfaces::msg::Armors::SharedPtr& ros_armorsmsg);//装甲板列表转换

  Armor ConvertArmor(const guga_interfaces::msg::Armor& ros_armormsg);//单个装甲板转换

  /// Convert ROS Target message to domain Target
  /// @throws std::invalid_argument if msg is nullptr
  Target ConvertTarget(guga_interfaces::msg::Target::SharedPtr ros_targetmsg);//目标信息转换

}  // namespace simple_decision
