// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// #define ODEMETRY_DEBUG

#include "nonrotating_vel_transform/nonrotating_vel_transform.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nonrotating_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.5;

NonrotatingVelTransform::NonrotatingVelTransform(const rclcpp::NodeOptions & options)
: Node("nonrotating_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start NonrotatingVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "base_footprint");
  this->declare_parameter<std::string>("nonrotating_robot_base_frame", "base_footprint_nonrotating");
  this->declare_parameter<std::string>("chassis_frame", "chassis");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("vis_cmd_vel_topic", "cmd_vel_marker");
  this->declare_parameter<std::string>("vis_frame_id", "base_link");
  this->declare_parameter<double>("vis_scale", 1.0);
  this->declare_parameter<std::string>("chassis_mode_topic", "chassis_mode");
  this->declare_parameter<float>("init_spin_speed", 0.0);
  this->declare_parameter<bool>("output_in_chassis_frame", false);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("nonrotating_robot_base_frame", nonrotating_robot_base_frame_);
  this->get_parameter("chassis_frame", chassis_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("vis_cmd_vel_topic", vis_cmd_vel_topic_);
  this->get_parameter("vis_scale", vis_scale_);
  this->get_parameter("chassis_mode_topic", chassis_mode_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);
  this->get_parameter("output_in_chassis_frame", output_in_chassis_frame_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  vis_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(vis_cmd_vel_topic_, 10);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&NonrotatingVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&NonrotatingVelTransform::cmdVelCallback, this, std::placeholders::_1));

  chassis_mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    chassis_mode_topic_, 1,
    std::bind(&NonrotatingVelTransform::chassisModeCallback, this, std::placeholders::_1));

  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&NonrotatingVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&NonrotatingVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, the velocity is published by the controller without timestamped.
  // We consider the velocity is published at the same time as local_plan.
  // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&NonrotatingVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  tf_sub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&NonrotatingVelTransform::updateGimbalYaw, this));

  // 50Hz Timer to send transform from `robot_base_frame` to `nonrotating_robot_base_frame`
  tf_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&NonrotatingVelTransform::publishTransform, this));
}

void NonrotatingVelTransform::chassisModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  chassis_mode_ = msg->data;
}

void NonrotatingVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_ = msg->data;
}

void NonrotatingVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // NOTE: Haven't synced with local_plan
  if ((rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
#ifdef ODEMETRY_DEBUG
    std::cout << "odom parent frame: " << msg->header.frame_id << std::endl;
    std::cout << "odom child frame: " << msg->child_frame_id << std::endl;
#endif
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
  }
}

void NonrotatingVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (
    is_zero_vel ||
    (rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    // Recovery behaviors do not publish a local plan, but their velocity is
    // still expressed in nonrotating_robot_base_frame_ and must be transformed.
    const double yaw_diff =
      selectVelocityYawDiff(chassis_mode_, chassis_followed_yaw_, current_robot_base_angle_);
    auto aft_tf_vel = transformVelocity(msg, yaw_diff);
    cmd_vel_chassis_pub_->publish(aft_tf_vel);
    visualizeVelocity(aft_tf_vel);
  } else {
    latest_cmd_vel_ = msg;
  }
}

void NonrotatingVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Consider nav2_controller_server is activated when receiving local_plan
  last_controller_activate_time_ = rclcpp::Clock().now();
}

void NonrotatingVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  geometry_msgs::msg::Twist::SharedPtr current_cmd_vel;
  {
    if (!latest_cmd_vel_) {
      return;
    }
    current_cmd_vel = latest_cmd_vel_;
  }

  current_robot_base_angle_ = tf2::getYaw(odom_msg->pose.pose.orientation);
  const double yaw_diff =
    selectVelocityYawDiff(chassis_mode_, chassis_followed_yaw_, current_robot_base_angle_);

  geometry_msgs::msg::Twist aft_tf_vel = transformVelocity(current_cmd_vel, yaw_diff);
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
  visualizeVelocity(aft_tf_vel);
}

void NonrotatingVelTransform::updateGimbalYaw()
{
  try {
    auto tf = tf_buffer_->lookupTransform(chassis_frame_, robot_base_frame_, tf2::TimePointZero);

    tf2::Quaternion q(
      tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
      tf.transform.rotation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    chassis_followed_yaw_ = yaw;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Failed to lookup TF %s->%s: %s", chassis_frame_.c_str(),
      robot_base_frame_.c_str(), ex.what());
  }
}

void NonrotatingVelTransform::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = nonrotating_robot_base_frame_;

  double tf_yaw = 0.0;
  if (chassis_mode_ == chassisFollowed)
    tf_yaw = -chassis_followed_yaw_;
  else
    tf_yaw = -current_robot_base_angle_;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, tf_yaw);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

double NonrotatingVelTransform::selectVelocityYawDiff(
  uint8_t chassis_mode, double chassis_followed_yaw, double robot_base_angle)
{
  return chassis_mode == chassisFollowed ? chassis_followed_yaw : robot_base_angle;
}

geometry_msgs::msg::Twist NonrotatingVelTransform::rotateVelocity(
  const geometry_msgs::msg::Twist & twist, double yaw_diff)
{
  geometry_msgs::msg::Twist out = twist;
  out.linear.x = twist.linear.x * std::cos(yaw_diff) + twist.linear.y * std::sin(yaw_diff);
  out.linear.y = -twist.linear.x * std::sin(yaw_diff) + twist.linear.y * std::cos(yaw_diff);
  return out;
}

geometry_msgs::msg::Twist NonrotatingVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, double yaw_diff)
{
  const double nonrotating_to_chassis_yaw = chassis_followed_yaw_ - yaw_diff;
  auto out = output_in_chassis_frame_ ? rotateVelocity(*twist, nonrotating_to_chassis_yaw)
                                      : rotateVelocity(*twist, yaw_diff);
  if (chassis_mode_ == chassisFollowed) {
    out.angular.z = twist->angular.z;
  } else {
    out.angular.z = twist->angular.z + spin_speed_;
  }
  return out;
}

void NonrotatingVelTransform::visualizeVelocity(const geometry_msgs::msg::Twist & vel)
{
  auto now = this->get_clock()->now();
  double scale = vis_scale_;

  visualization_msgs::msg::Marker linear_marker;
  linear_marker.header.frame_id = output_in_chassis_frame_ ? chassis_frame_ : robot_base_frame_;
  linear_marker.header.stamp = now;
  linear_marker.ns = "cmd_vel";
  linear_marker.id = 0;
  linear_marker.type = visualization_msgs::msg::Marker::ARROW;
  linear_marker.action = visualization_msgs::msg::Marker::ADD;
  linear_marker.pose.orientation.w = 1.0;
  linear_marker.scale.x = 0.06;
  linear_marker.scale.y = 0.12;
  linear_marker.scale.z = 0.0;
  linear_marker.color.r = 0.0;
  linear_marker.color.g = 1.0;
  linear_marker.color.b = 0.0;
  linear_marker.color.a = 0.8;
  linear_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;

  geometry_msgs::msg::Point end;
  end.x = vel.linear.x * scale;
  end.y = vel.linear.y * scale;
  end.z = 0.0;

  linear_marker.points.push_back(start);
  linear_marker.points.push_back(end);

  vis_marker_pub_->publish(linear_marker);

  visualization_msgs::msg::Marker angular_marker;
  angular_marker.header.frame_id = output_in_chassis_frame_ ? chassis_frame_ : robot_base_frame_;
  angular_marker.header.stamp = now;
  angular_marker.ns = "cmd_vel";
  angular_marker.id = 1;
  angular_marker.type = visualization_msgs::msg::Marker::ARROW;
  angular_marker.action = visualization_msgs::msg::Marker::ADD;
  angular_marker.pose.position.z = 0.05;
  angular_marker.pose.orientation.w = 1.0;
  angular_marker.scale.x = 0.04;
  angular_marker.scale.y = 0.08;
  angular_marker.scale.z = 0.0;
  angular_marker.color.r = 1.0;
  angular_marker.color.g = 0.5;
  angular_marker.color.b = 0.0;
  angular_marker.color.a = 0.8;
  angular_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  geometry_msgs::msg::Point z_start;
  z_start.x = 0.0;
  z_start.y = 0.0;
  z_start.z = 0.0;

  geometry_msgs::msg::Point z_end;
  z_end.x = 0.0;
  z_end.y = 0.0;
  z_end.z = vel.angular.z * scale * 0.5;

  angular_marker.points.push_back(z_start);
  angular_marker.points.push_back(z_end);

  vis_marker_pub_->publish(angular_marker);
}

}  // namespace nonrotating_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nonrotating_vel_transform::NonrotatingVelTransform)
