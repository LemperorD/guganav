#include "point_lio/OdomProcessor.h"

#include <tf2/exceptions.h>

#include "point_lio/core/common_lib.h"

bool OdomProcessor::resolve(const OutputFrameParams& out,
                            const tf2_ros::Buffer* buffer,
                            const rclcpp::Logger& logger, rclcpp::Clock& clock,
                            const std::vector<double>& lidar_r,
                            const std::vector<double>& lidar_t) {
  if (ready_) {
    return true;
  }
  if (out.body_to_base_from_params) {
    rotation_ << out.body_to_base_r[0], out.body_to_base_r[1],
        out.body_to_base_r[2], out.body_to_base_r[3], out.body_to_base_r[4],
        out.body_to_base_r[5], out.body_to_base_r[6], out.body_to_base_r[7],
        out.body_to_base_r[8];
    translation_ << out.body_to_base_t[0], out.body_to_base_t[1],
        out.body_to_base_t[2];
    ready_ = true;
    RCLCPP_INFO(logger, "odom 输出使用显式参数 %s <- body",
                out.base_frame.c_str());
    return true;
  }

  Eigen::Matrix3d base_lidar_rotation;
  Eigen::Vector3d base_lidar_translation;
  if (out.extrinsic_from_params) {
    base_lidar_rotation << out.lidar_to_base_r[0], out.lidar_to_base_r[1],
        out.lidar_to_base_r[2], out.lidar_to_base_r[3], out.lidar_to_base_r[4],
        out.lidar_to_base_r[5], out.lidar_to_base_r[6], out.lidar_to_base_r[7],
        out.lidar_to_base_r[8];
    base_lidar_translation << out.lidar_to_base_t[0], out.lidar_to_base_t[1],
        out.lidar_to_base_t[2];
  } else {
    if (buffer == nullptr) {
      return false;
    }
    try {
      // 静态变换, 用 TimePointZero 取最新可用值, 不依赖雷达时间戳
      const auto tf_stamped = buffer->lookupTransform(
          out.base_frame, out.lidar_frame, tf2::TimePointZero);
      const auto& q = tf_stamped.transform.rotation;
      const Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
      base_lidar_rotation = quat.toRotationMatrix();
      const auto& t = tf_stamped.transform.translation;
      base_lidar_translation << t.x, t.y, t.z;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
          logger, clock, 2000,
          "查不到安装变换 %s <- %s: %s, 暂不发布 %s/%s 或保存 PCD",
          out.base_frame.c_str(), out.lidar_frame.c_str(), ex.what(),
          out.registered_scan_topic.c_str(), out.lidar_odometry_topic.c_str());
      return false;
    }
  }

  const Eigen::Matrix3d lidar_imu_rotation = to_mat3d(lidar_r);
  const Eigen::Vector3d lidar_imu_translation = to_vec3d(lidar_t);
  const Eigen::Matrix3d imu_lidar_rotation = lidar_imu_rotation.transpose();
  const Eigen::Vector3d imu_lidar_translation = -imu_lidar_rotation
                                                * lidar_imu_translation;
  rotation_ = base_lidar_rotation * imu_lidar_rotation;
  translation_ = base_lidar_rotation * imu_lidar_translation
                 + base_lidar_translation;
  ready_ = true;
  RCLCPP_INFO(logger,
              "odom 输出使用 %s <- body（由 base<-lidar 与 lidar->imu 合成）",
              out.base_frame.c_str());
  return true;
}
