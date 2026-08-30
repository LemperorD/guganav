/**
 * @file common_lib.h
 * @brief Point-LIO 公共类型定义和工具函数
 * @author HKU-MARS (original), LihanChen2004 (fork)
 *
 * 本文件定义了 Point-LIO 系统中所有模块共享的：
 * - 流形类型别名 (基于 IKFoM/MTK 模板库)
 * - 状态流形定义 (input/output 两种模式)
 * - 过程噪声流形定义
 * - 常用宏、Eigen 类型别名
 * - MeasureGroup 数据结构 (雷达-IMU帧同步)
 * - 平面估计等模板工具函数
 */

#pragma once

#include <deque>
#include <memory>
#include <stdexcept>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <point_lio/core/StateTypes.h>
#include <Eigen/Eigen>
#include <sensor_msgs/msg/imu.hpp>
#include <point_lio/core/PointTypes.h>

using namespace std;
using namespace Eigen;

#define NUM_MATCH_POINTS (5)  ///< 点到面匹配所需的最小近邻点数

// ==================== std::vector → Eigen 转换 ====================
/** @brief std::vector<double> → Eigen::Vector3d (带长度校验) */
inline V3D to_vec3d(const std::vector<double>& v) {
  if (v.size() < 3) {
    throw std::runtime_error("to_vec3d: 输入长度不足 3");
  }
  return Eigen::Map<const V3D>(v.data());
}

/** @brief std::vector<double> → Eigen::Matrix3d (带长度校验) */
inline M3D to_mat3d(const std::vector<double>& v) {
  if (v.size() < 9) {
    throw std::runtime_error("to_mat3d: 输入长度不足 9");
  }
  return Eigen::Map<const M3D>(v.data());
}

#define VF(a) Matrix<float, (a), 1>        ///< 动态大小单精度列向量

/** @brief 预定义的常用常量矩阵 */
const M3D Eye3d(M3D::Identity());  ///< 3x3 单位阵 (double)
const V3D Zero3d(0, 0, 0);         ///< 3维零向量 (double)

// ==================== 工具函数 ====================

/**
 * @brief 时间压缩: 按时间戳分组点云 (用于逐组点 Kalman 更新)
 *
 * 点云的 PCL curvature 存储字段用于保存该点相对帧首的时间偏移 (ms)。
 * 此函数根据时间偏移的单调性将点云分成多个组，
 * 返回每个组的大小序列。
 *
 * 分组逻辑:
 * - 遍历点云，当点时间偏移递增时 (同组内时间递增)，累加计数
 * - 当点时间偏移回跳时 (新的一组开始)，记录当前组大小并重置计数
 *
 * @param point_cloud 输入点云 (point time offset = 时间偏移，单位 ms)
 * @return 每组包含的点数序列
 */
inline std::vector<int> time_compressing(
    const PointCloudXYZI::Ptr& point_cloud) {
  int points_size = point_cloud->points.size();
  int j = 0;
  std::vector<int> time_seq;
  time_seq.reserve(points_size);
  for (int i = 0; i < points_size - 1; i++) {
    j++;
    // 当点时间偏移回跳时，开始新的分组
    if (point_time_offset_ms(point_cloud->points[i + 1])
        > point_time_offset_ms(point_cloud->points[i])) {
      time_seq.emplace_back(j);
      j = 0;
    }
  }
  // 最后一组
  {
    time_seq.emplace_back(j + 1);
  }
  return time_seq;
}

/**
 * @brief 用 5 个最近邻点估计局部平面 (固定点数版本)
 *
 * 平面方程: n·p + d = 0, 其中 n 为单位法向量, d 为截距
 * 返回 pca_result = [nx, ny, nz, d]^T
 *
 * @tparam T 浮点类型
 * @param[out] pca_result 4维平面参数 (nx, ny, nz, d)
 * @param point 5个最近邻点
 * @param threshold 平面一致性阈值
 * @return true 如果5个点都在阈值内(平面有效), false 否则
 */
template <typename T>
bool esti_plane(Matrix<T, 4, 1>& pca_result, const PointVector& point,
                const T& threshold) {
  Matrix<T, NUM_MATCH_POINTS, 3> A;
  Matrix<T, NUM_MATCH_POINTS, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }

  Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

  // 归一化法向量并计算截距 d = 1/|x0|
  T n = normvec.norm();
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0 / n;

  // 一致性检查: 所有5个点到平面的距离必须在阈值内
  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y
             + pca_result(2) * point[j].z + pca_result(3))
        > threshold) {
      return false;
    }
  }
  return true;
}

/**
 * @brief ROS2 Time 消息 → 秒 (double)
 */
inline double get_time_sec(const builtin_interfaces::msg::Time& time) {
  return rclcpp::Time(time).seconds();
}

/**
 * @brief 秒 (double) → ROS2 Time 消息
 */
inline rclcpp::Time get_ros_time(double timestamp) {
  int32_t sec = std::floor(timestamp);
  auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
  uint32_t nanosec = nanosec_d;
  return rclcpp::Time(sec, nanosec);
}
