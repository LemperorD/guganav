/**
 * @file ProjectOutputPublisher.h
 * @brief odom 系输出发布.
 *
 * 把 camera_init 系点云与里程计经投影变换后, 以 odom 坐标系发布.
 */

#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "point_lio/OdomProcessor.h"
#include "point_lio/core/common_lib.h"
#include "point_lio/core/parameters.h"

/// 把 camera_init 系的结果转到 odom 系后发布.
class ProjectOutputPublisher {
public:
  void configure(rclcpp::Node* node, const OutputFrameParams& out,
                 OdomProcessor* projector);

  void publish(const PointCloudXYZI& world_cloud,
               const nav_msgs::msg::Odometry& odometry, bool odometry_valid,
               const rclcpp::Time& stamp);

private:
  [[nodiscard]] bool passesZFilter(const Eigen::Vector3d& point) const;

  const OutputFrameParams* out_{nullptr};
  OdomProcessor* projector_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};
