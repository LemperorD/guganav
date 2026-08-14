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

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <cmath>

namespace sensor_scan_generation
{

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");
  this->declare_parameter<double>("min_odometry_dt", 1e-3);
  this->declare_parameter<double>("max_linear_velocity", 10.0);
  this->declare_parameter<double>("max_angular_velocity", 20.0);

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("min_odometry_dt", min_odometry_dt_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_angular_velocity", max_angular_velocity_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  laser_cloud_sub_.subscribe(this, "registered_scan", qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, laser_cloud_sub_);
  sync_->registerCallback(std::bind(
    &SensorScanGenerationNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
    std::placeholders::_2));

  // 启动四个线程执行本功能包的四个并行任务
  sensor_scan_thread_ = std::thread(std::bind(&SensorScanGenerationNode::updateSensorScan, this));
  chassis_odom_thread_ = std::thread(std::bind(&SensorScanGenerationNode::updateChassisOdometry, this));
  robot_base_odom_thread_ = std::thread(std::bind(&SensorScanGenerationNode::updateRobotBaseOdometry, this));
  chassis_tf_thread_ = std::thread(std::bind(&SensorScanGenerationNode::updateChassisTF, this));
}

// 析构函数,释放线程资源
SensorScanGenerationNode::~SensorScanGenerationNode() {
  if (chassis_tf_thread_.joinable()) {
    chassis_tf_thread_.join();
  }
  if (chassis_odom_thread_.joinable()) {
    chassis_odom_thread_.join();
  }
  if (robot_base_odom_thread_.joinable()) {
    robot_base_odom_thread_.join();
  }
  if (sensor_scan_thread_.joinable()) {
    sensor_scan_thread_.join();
  }
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  tf2::Transform tf_lidar_to_chassis;

  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar_);
  tf_lidar_to_robot_base_ = getTransform(lidar_frame_, robot_base_frame_, pcd_msg->header.stamp);
  tf_lidar_to_chassis = getTransform(lidar_frame_, base_frame_, pcd_msg->header.stamp);

  tf_odom_to_chassis_ = tf_odom_to_lidar_ * tf_lidar_to_chassis;
  tf_odom_to_robot_base_ = tf_odom_to_lidar_ * tf_lidar_to_robot_base_;

  publishTransform(
    tf_odom_to_chassis_, odometry_msg->header.frame_id, base_frame_, pcd_msg->header.stamp);
  publishOdometry(
    tf_odom_to_robot_base_, odometry_msg->header.frame_id, robot_base_frame_, pcd_msg->header.stamp, pub_chassis_odometry_);

  // sensor_msgs::msg::PointCloud2 out;
  // pcl_ros::transformPointCloud(lidar_frame_, tf_odom_to_lidar.inverse(), *pcd_msg, out);
  // pub_laser_cloud_->publish(out);

  {
    std::lock_guard<std::mutex> lock(sensor_scan_mutex_);
    sensor_scan_ready_ = true;
    in_ = *pcd_msg;
  }
  sensor_scan_cv_.notify_one();
}

tf2::Transform SensorScanGenerationNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Returning identity.", ex.what());
    return tf2::Transform::getIdentity();
  }
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ptr)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  if (has_previous_odometry_) {
    const double dt = (stamp - previous_odometry_stamp_).seconds();
    if (dt > min_odometry_dt_) {
      const auto linear_velocity =
        (transform.getOrigin() - previous_odometry_transform_.getOrigin()) / dt;

      tf2::Quaternion q_diff =
        transform.getRotation() * previous_odometry_transform_.getRotation().inverse();
      q_diff.normalize();
      const double angle = std::remainder(q_diff.getAngle(), 2.0 * M_PI);
      const auto angular_velocity = q_diff.getAxis() * angle / dt;

      const double linear_speed = linear_velocity.length();
      const double angular_speed = angular_velocity.length();
      if (
        std::isfinite(linear_speed) && std::isfinite(angular_speed) &&
        linear_speed <= max_linear_velocity_ && angular_speed <= max_angular_velocity_) {
        out.twist.twist.linear.x = linear_velocity.x();
        out.twist.twist.linear.y = linear_velocity.y();
        out.twist.twist.linear.z = linear_velocity.z();
        out.twist.twist.angular.x = angular_velocity.x();
        out.twist.twist.angular.y = angular_velocity.y();
        out.twist.twist.angular.z = angular_velocity.z();
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Discard odometry twist spike: dt=%.6f, linear=%.3f, angular=%.3f",
          dt, linear_speed, angular_speed);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip odometry twist update because dt is too small or non-positive: %.6f", dt);
    }
  }

  previous_odometry_transform_ = transform;
  previous_odometry_stamp_ = stamp;
  has_previous_odometry_ = true;

  odom_pub_ptr->publish(out);
}

void SensorScanGenerationNode::updateChassisTF() {
  rclcpp::Rate rate(100); // 100 Hz
  while (rclcpp::ok()) {
    std::cout << "Updating chassis TF..." << std::endl;
    // publishTransform(
    //   tf_odom_to_chassis_, "odom", base_frame_, this->now());
    rate.sleep();
  }
}

void SensorScanGenerationNode::updateChassisOdometry() {
  rclcpp::Rate rate(100); // 100 Hz
  while (rclcpp::ok()) {
    std::cout << "Updating chassis odometry..." << std::endl;
    // publishOdometry(
    //   tf_odom_to_chassis_, "odom", base_frame_, this->now(), pub_chassis_odometry_);
    rate.sleep();
  }
}

void SensorScanGenerationNode::updateRobotBaseOdometry() {
  rclcpp::Rate rate(100); // 100 Hz
  while (rclcpp::ok()) {
    std::cout << "Updating robot base odometry..." << std::endl;
    // publishOdometry(
    //   tf_odom_to_robot_base_, "odom", robot_base_frame_, this->now(), pub_robot_base_odometry_);
    rate.sleep();
  }
}

void SensorScanGenerationNode::updateSensorScan() {
  while (rclcpp::ok()) {
    {
      sensor_msgs::msg::PointCloud2 input;
      {
        std::unique_lock<std::mutex> lock(sensor_scan_mutex_);
        sensor_scan_cv_.wait(lock, [this]() {
          return sensor_scan_ready_ || !rclcpp::ok();
        });
      }
      input = in_;
      sensor_scan_ready_ = false;
      if (!rclcpp::ok()) { return; }
      pcl_ros::transformPointCloud(lidar_frame_, tf_odom_to_lidar_.inverse(), in_, out_);
    }
    pub_laser_cloud_->publish(out_);
  }
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
