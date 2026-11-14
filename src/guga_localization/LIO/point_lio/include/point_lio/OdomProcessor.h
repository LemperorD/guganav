/**
 * @file OdomProcessor.h
 * @brief camera_init 到 odom 的固定变换.
 *
 * 由 base<-lidar 与 lidar->imu 外参合成, 支持参数直接给出或从 TF 查询.
 */

#pragma once

#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "point_lio/core/parameters.h"

/// camera_init 到 odom 的固定变换, 由 base<-lidar 与 lidar->imu 合成.
class OdomProcessor {
public:
  /// 解析安装变换: 优先用参数, 否则查一次 TF.
  bool resolve(const OutputFrameParams& out, const tf2_ros::Buffer* buffer,
               const rclcpp::Logger& logger, rclcpp::Clock& clock,
               const std::vector<double>& lidar_r,
               const std::vector<double>& lidar_t);

  [[nodiscard]] bool ready() const {
    return ready_;
  }

  [[nodiscard]] Eigen::Vector3d apply(const Eigen::Vector3d& point) const {
    return rotation_ * point + translation_;
  }

  [[nodiscard]] Eigen::Vector3d rotate(const Eigen::Vector3d& vector) const {
    return rotation_ * vector;
  }

  [[nodiscard]] const Eigen::Matrix3d& rotation() const {
    return rotation_;
  }

private:
  bool ready_{false};
  Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation_{Eigen::Vector3d::Zero()};
};
