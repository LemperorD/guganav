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

#ifndef SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
#define SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_

#include <memory>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace sensor_scan_generation
{

class SensorScanGenerationNode : public rclcpp::Node
{
public:
  explicit SensorScanGenerationNode(const rclcpp::NodeOptions & options);
  ~SensorScanGenerationNode();

private:
  void laserCloudAndOdometryHandler(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & laserCloud2);

  tf2::Transform getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time);

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  void publishOdometry(
    const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
    const rclcpp::Time & stamp, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ptr);

  std::string lidar_frame_;
  std::string base_frame_;
  std::string robot_base_frame_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_chassis_odometry_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_robot_base_odometry_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> laser_cloud_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  tf2::Transform tf_lidar_to_robot_base_;
  tf2::Transform previous_odometry_transform_;
  rclcpp::Time previous_odometry_stamp_;
  bool has_previous_odometry_{false};
  double min_odometry_dt_{1e-3};
  double max_linear_velocity_{10.0};
  double max_angular_velocity_{20.0};
  
private: // 优化: 使用单独的四个线程执行本功能包的四个并行任务
  // 线程内调用的成员变量
  tf2::Transform tf_odom_to_lidar_;
  tf2::Transform tf_odom_to_robot_base_;
  tf2::Transform tf_odom_to_chassis_;
  sensor_msgs::msg::PointCloud2 in_;
  sensor_msgs::msg::PointCloud2 out_;

  // 线程对象
  std::thread chassis_tf_thread_;
  std::thread chassis_odom_thread_;
  std::thread robot_base_odom_thread_;
  std::thread sensor_scan_thread_;

  // 线程函数
  void updateChassisTF();
  void updateChassisOdometry();
  void updateRobotBaseOdometry();
  void updateSensorScan();

  // 线程同步机制
  std::mutex sensor_scan_mutex_;
  std::mutex chassis_odom_mutex_;
  std::mutex robot_base_odom_mutex_;
  std::mutex chassis_tf_mutex_;
  std::condition_variable sensor_scan_cv_;
  std::condition_variable chassis_odom_cv_;
  std::condition_variable robot_base_odom_cv_;
  std::condition_variable chassis_tf_cv_;

  // 线程间共享的标志变量
  bool sensor_scan_ready_{false};
  bool chassis_odom_ready_{false};
  bool robot_base_odom_ready_{false};
};

}  // namespace sensor_scan_generation

#endif  // SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
