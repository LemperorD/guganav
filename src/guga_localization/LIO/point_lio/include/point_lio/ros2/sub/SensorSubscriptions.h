/**
 * @file SensorSubscriptions.h
 * @brief 传感器订阅与消息转换.
 *
 * 接收 ROS2 点云与 IMU 消息, 转成纯数据后交给处理器, 消息类型只在此层出现.
 */

#pragma once

#include <deque>
#include <memory>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "point_lio/core/FrameProcessor.h"
#include "point_lio/core/parameters.h"
#include "point_lio/preprocess.h"

/// ROS2 传感器订阅: 把消息转成纯数据后交给处理器.
class SensorSubscriptions {
public:
  void configure(rclcpp::Node* node, const PointLioParams& config,
                 FrameProcessor* processor, Preprocess* preprocess,
                 rclcpp::CallbackGroup::SharedPtr group);

private:
  void onLivox(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

  rclcpp::Node* node_{nullptr};
  const PointLioParams* config_{nullptr};
  FrameProcessor* processor_{nullptr};
  Preprocess* preprocess_{nullptr};
  int scan_count_{0};

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};
