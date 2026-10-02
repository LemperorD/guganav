#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "guga_interfaces/msg/rfid_status.hpp"
#include "guga_interfaces/msg/robot_status.hpp"
#include "guga_interfaces/msg/vision_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// 决策节点的 ROS 2 侧接口，集中持有本包用到的全部 ROS 资源：
// 订阅裁判与视觉数据、查询定位、以及向导航发布目标点。
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

  // RFID 增益点：机器人是否压到了对应的卡上。base 是己方基地增益点（"家"），
  // center 是中心增益点（RMUL 的"占点"）。未触发时为 false。
  bool baseGainPoint() const { return base_gain_point_.load(); }
  bool centerGainPoint() const { return center_gain_point_.load(); }

  // 是否收到过 RFID 状态。没收到时上面两位恒为 false，判断"没触发"之前要先看
  // 这个标志，否则会把"话题没通"当成"没压到卡上"，进而在场上一直找不到点。
  bool hasRfidData() const { return has_rfid_data_.load(); }

  // 发布"要求停"给下游（controller / 底盘侧自己订阅处理）。
  //
  // 是电平不是事件：true 表示现在要求停，false 表示不要求。行为树每个 tick 都会
  // 发一条（不做去重），订阅端因此可以拿"超时没收到"当失效信号——上游挂了就停，
  // 比保持最后一帧速度安全。
  void publishStop(bool stop);

  // 发布导航目标点。目标与上次相同时不重复发布，返回 false 表示这次没有发。
  //
  // force 为 true 时跳过去重，即使与上次相同也重新发一条。目标点是即发即忘的
  // 话题，订阅端丢包不会重传，所以需要用重发来兜底；但订阅端 bt_navigator 把
  // 每条 goal_pose 都当成一个新目标，重发会中止正在执行的目标，所以只该按
  // 远低于 tick 频率的周期调用，不能每个 tick 都发。
  bool sendGoalPose(double x, double y, bool force = false);

  // 机器人在 map 系下的位置，用于判断是否到达目标点。
  //
  // 里程计话题（nav_msgs/Odometry）里的坐标是 odom 系的，而目标点是 map 系的，
  // 两者相差一个 map -> odom 变换（由 SLAM 或点云重定位发布）。这里不订阅 odom
  // 自己做变换，而是让 tf2 直接查 base_frame_id 到 frame_id 的链，少维护一处。
  //
  // 定位未就绪时返回 false，x、y 不被修改（调用方传入的值保持原样）。
  bool lookupRobotPose(double& x, double& y);

private:
  using RobotStatusMsg = guga_interfaces::msg::RobotStatus;
  using RfidStatusMsg = guga_interfaces::msg::RfidStatus;
  using VisionInfo = guga_interfaces::msg::VisionInfo;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  rclcpp::Subscription<RobotStatusMsg>::SharedPtr sub_state_;
  rclcpp::Subscription<RfidStatusMsg>::SharedPtr sub_rfid_;
  rclcpp::Subscription<VisionInfo>::SharedPtr sub_vision_;
  
  std::atomic<std::uint16_t> current_hp_{0};
  std::atomic<std::uint16_t> maximum_hp_{0};
  std::atomic<std::uint16_t> projectile_allowance_{0};
  std::atomic<std::int32_t> enemy_count_{0};
  std::atomic<bool> has_data_{false};
  std::atomic<bool> base_gain_point_{false};
  std::atomic<bool> center_gain_point_{false};
  std::atomic<bool> has_rfid_data_{false};

  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  bool last_stop_{false};
  bool has_last_stop_{false};
  std::string frame_id_;

  // 定位：查 map -> base_frame_id 的变换。缓冲区由 TransformListener 填充，
  // 它把 /tf 订阅放到自己的线程上，不依赖别处有没有人 spin 本节点。
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string base_frame_id_;

  // 上次发布过的目标点，用来判断是否真的变了。
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  bool has_last_goal_{false};
};
