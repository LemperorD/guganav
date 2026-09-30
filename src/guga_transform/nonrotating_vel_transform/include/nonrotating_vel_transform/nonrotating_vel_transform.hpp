#ifndef NONROTATING_VEL_TRANSFORM__NONROTATING_VEL_TRANSFORM_HPP_
#define NONROTATING_VEL_TRANSFORM__NONROTATING_VEL_TRANSFORM_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

// 与 simple_decision 的 ChassisMode 枚举对齐（0=CHASSIS_FOLLOWED, 1=LITTLE_TES, 2=GO_HOME）
typedef enum {
  chassisFollowed = 0,
  littleTES,
  goHome,
} ChassisMode;

namespace nonrotating_vel_transform
{

class NonrotatingVelTransform : public rclcpp::Node
{
public:
  explicit NonrotatingVelTransform(const rclcpp::NodeOptions & options);

  static double selectVelocityYawDiff(
    uint8_t chassis_mode, double chassis_followed_yaw, double robot_base_angle);
  static geometry_msgs::msg::Twist rotateVelocity(
    const geometry_msgs::msg::Twist & twist, double yaw_diff);

private:
  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const nav_msgs::msg::Path::ConstSharedPtr & local_plan);
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdSpinCallback(example_interfaces::msg::Float32::SharedPtr msg);
  void chassisModeCallback(std_msgs::msg::UInt8::SharedPtr msg);
  void updateGimbalYaw();
  void publishTransform();
  /** @brief littleTES/goHome 下用 spin 外推底盘 yaw，避免 odom 低频导致补偿跳变。 */
  double estimateRobotBaseAngle() const;
  geometry_msgs::msg::Twist transformVelocity(
    const geometry_msgs::msg::Twist::SharedPtr & twist, double yaw_diff);
  void visualizeVelocity(const geometry_msgs::msg::Twist & vel);
  void onConfigure();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_spin_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr chassis_mode_sub_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_filter_;
  message_filters::Subscriber<nav_msgs::msg::Path> local_plan_sub_filter_;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Path>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_marker_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr tf_sub_timer_;
  rclcpp::TimerBase::SharedPtr tf_pub_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string robot_base_frame_;
  std::string nonrotating_robot_base_frame_;
  std::string chassis_frame_;
  std::string odom_topic_;
  std::string local_plan_topic_;
  std::string cmd_spin_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string vis_cmd_vel_topic_;
  std::string vis_frame_id_;
  double vis_scale_{1.0};
  std::string chassis_mode_topic_;
  float spin_speed_;
  bool output_in_chassis_frame_{false};
  uint8_t chassis_mode_{littleTES};
  double chassis_followed_yaw_ = 0.0;

  std::mutex cmd_vel_mutex_;
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
  double current_robot_base_angle_{0.0};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_controller_activate_time_;
};

}  // namespace nonrotating_vel_transform

#endif  // NONROTATING_VEL_TRANSFORM__NONROTATING_VEL_TRANSFORM_HPP_
