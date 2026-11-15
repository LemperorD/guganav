/**
 * @file OdomProcessor.h
 * @brief camera_init 到 odom 的固定变换.
 *
 * 由 output_frame.body_to_base_r/t 直接给出.
 */

#pragma once

#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "point_lio/core/parameters.h"

/// camera_init 到 odom 的固定变换, 由 body_to_base_r/t 直接给出.
class OdomProcessor {
public:
  /// 解析安装变换: 只读 output_frame.body_to_base_r/t.
  bool resolve(const OutputFrameParams& out, const rclcpp::Logger& logger);

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
