#include "simple_decision/adapter/transform.hpp"

#include <stdexcept>

namespace simple_decision {

  Quaternion ConvertQuaternion(const geometry_msgs::msg::Quaternion& ros_quat) {
    Quaternion quat;
    quat.x = ros_quat.x;
    quat.y = ros_quat.y;
    quat.z = ros_quat.z;
    quat.w = ros_quat.w;
    return quat;
  }

  Position ConvertPoint(const geometry_msgs::msg::Point& ros_point) {
    Position position;
    position.x = ros_point.x;
    position.y = ros_point.y;
    position.z = ros_point.z;
    return position;
  }

  Pose3D ConvertPose(const geometry_msgs::msg::Pose& ros_pose) {
    Pose3D pose;
    pose.position = ConvertPoint(ros_pose.position);
    pose.orientation = ConvertQuaternion(ros_pose.orientation);
    return pose;
  }

  RobotStatus ConvertRobotStatus(
      guga_interfaces::msg::RobotStatus::SharedPtr msg) {
    RobotStatus robotstatus;
    if (!msg) {
      throw std::invalid_argument(
          "ConvertRobotStatus received nullptr RobotStatus message");
    }

    robotstatus.robot_id = msg->robot_id;
    robotstatus.robot_level = msg->robot_level;
    robotstatus.current_hp = msg->current_hp;
    robotstatus.maximum_hp = msg->maximum_hp;
    robotstatus.shooter_barrel_cooling_value =
        msg->shooter_barrel_cooling_value;
    robotstatus.shooter_barrel_heat_limit = msg->shooter_barrel_heat_limit;
    robotstatus.shooter_17mm_1_barrel_heat = msg->shooter_17mm_1_barrel_heat;
    robotstatus.robot_pos = ConvertPose(msg->robot_pos);
    robotstatus.armor_id = msg->armor_id;
    robotstatus.hp_deduction_reason = msg->hp_deduction_reason;
    robotstatus.projectile_allowance_17mm = msg->projectile_allowance_17mm;
    robotstatus.remaining_gold_coin = msg->remaining_gold_coin;
    robotstatus.is_hp_deduced = msg->is_hp_deduced;

    return robotstatus;
  }

  GameStatus ConvertGameStatus(
      const guga_interfaces::msg::GameStatus::SharedPtr msg) {
    GameStatus gamestatus;

    gamestatus.game_progress = msg->game_progress;

    return gamestatus;
  }

  Armors ConvertArmors(
      const guga_interfaces::msg::Armors::SharedPtr& ros_armorsmsg) {
    Armors armors;

    if (!ros_armorsmsg) {
      throw std::invalid_argument(
          "ConvertRobotStatus received nullptr Armors message");
    }
    armors.header.stamp.nanosec = ros_armorsmsg->header.stamp.nanosec;
    armors.header.stamp.sec = ros_armorsmsg->header.stamp.sec;
    armors.header.frame_id = ros_armorsmsg->header.frame_id;

    // 调整目标容器大小
    armors.armors.resize(ros_armorsmsg->armors.size());

    // 使用 transform 进行批量转换映射
    std::transform(ros_armorsmsg->armors.begin(), ros_armorsmsg->armors.end(),
                   armors.armors.begin(), [](const auto& ros_armor) {
                     return ConvertArmor(ros_armor);  // 调用单体转换逻辑
                   });

    return armors;
  }

  Armor ConvertArmor(const guga_interfaces::msg::Armor& ros_armormsg) {
    Armor armor;
    armor.pose = ConvertPose(ros_armormsg.pose);
    return armor;
  }

  Target ConvertTarget(
      const guga_interfaces::msg::Target::SharedPtr ros_targetmsg) {
    Target target;
    target.header.stamp.nanosec = ros_targetmsg->header.stamp.nanosec;
    target.header.stamp.sec = ros_targetmsg->header.stamp.sec;
    target.header.frame_id = ros_targetmsg->header.frame_id;

    target.position = ConvertPoint(ros_targetmsg->position);
    target.yaw = ros_targetmsg->yaw;
    target.tracking = ros_targetmsg->tracking;
    return target;
  }
}  // namespace simple_decision
