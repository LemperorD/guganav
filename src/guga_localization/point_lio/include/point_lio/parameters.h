/**
 * @file parameters.h
 * @brief Point-LIO 参数结构声明和工具函数
 *
 * 配置参数从 YAML 文件通过 ROS2 参数系统读取，写入 PointLioParams。
 * 参数通过 readParameters() 函数解析，涵盖:
 * - LiDAR 配置 (类型、线数、扫描频率)
 * - IMU 配置 (协方差、饱和值)
 * - 建图配置 (滤波分辨率、iVox 网格、FOV)
 * - 外参标定 (LiDAR→IMU 变换)
 * - 发布配置 (里程计、点云、TF)
 * - 先验 PCD 地图
 *
 * 该文件遵循编码规范 **模式 B** (对象式封装，参见 CODING_STANDARD.md)
 */

#pragma once
#include <ivox/ivox3d.h>
#include <cmath>
#include <omp.h>
#include <pcl/common/transforms.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "point_lio/IMU_Processing.h"
#include "point_lio/preprocess.h"

namespace rclcpp_lifecycle {
class LifecycleNode;
}

// 选择 iVox 节点类型: PHC (Plane-Histogram-Coplanarity) 或 DEFAULT
// #define IVOX_NODE_TYPE_PHC

#ifdef IVOX_NODE_TYPE_PHC
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PointType>;
#else
/// @brief 使用默认 iVox 节点类型的增量体素地图类型
using IVoxType =
    faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
#endif

struct MappingParams {
  bool space_down_sample{true};
  bool propagate_at_imu_frequency{true};
  bool use_imu_as_input{false};
  bool publish_odometry_without_downsample{false};
  int init_map_size{100};
  double filter_size_surf{0.5};
  double filter_size_map{0.5};
  double fov_deg{180.0};
  float det_range{300.0F};
  IVoxType::Options ivox_options;
};

struct CommonParams {
  double gravity_magnitude{9.81};
};

struct FilterParams {
  double acc_cov_input{0.1};
  double gyr_cov_input{0.1};
  double vel_cov{20.0};
  double gyr_cov_output{0.1};
  double acc_cov_output{0.1};
  double b_gyr_cov{0.0001};
  double b_acc_cov{0.0001};
};

struct ImuParams {
  ImuProcessor::Params processor;
  double integration_interval{0.005};
  double timestamp_offset{0.0};
  bool check_saturation{true};
  double saturation_acc{3.0};
  double saturation_gyro{35.0};
  double acc_norm{1.0};
  double measurement_acc_cov{0.1};
  double measurement_gyro_cov{0.1};
};

struct PublishParams {
  bool path_enabled{true};
  bool scan_enabled{true};
  bool scan_body_enabled{true};
  bool tf_enabled{true};
  bool pcd_save_enabled{false};
  int pcd_save_interval{-1};
};

struct SensorParams {
  std::string lidar_topic{"/livox/lidar"};
  std::string imu_topic{"/livox/imu"};
  std::vector<double> extrinsic_t;
  std::vector<double> extrinsic_r;
  double lidar_to_imu_time{0.0};
  bool enable_prior_map{false};
  std::string prior_map_path;
  std::vector<double> initial_pose;
};

struct LidarParams {
  PreprocessParams preprocess;
  int lidar_type{AVIA};
  bool con_frame{false};
  int con_frame_num{1};
  bool cut_frame{false};
  int cut_frame_num{1};
  double cut_frame_interval{0.1};
  double lidar_time_interval{0.1};
  bool extrinsic_estimation{true};
  double point_covariance{0.1};
  double match_threshold{81.0};
  float plane_threshold{0.05F};
};

struct PointLioParams {
  CommonParams common;
  MappingParams mapping;
  FilterParams filter;
  PublishParams publish;
  SensorParams sensor;
  LidarParams lidar;
  ImuParams imu;
};

// ==================== 函数声明 ====================

/**
 * @brief 解析所有 ROS2 节点参数
 *
 * 从 YAML 配置文件读取参数，若参数不存在则使用默认值。
 * 参数直接写入各模块的参数结构。
 *
 * @param n ROS2 节点共享指针
 * @return 完整的只读配置值
 */
[[nodiscard]] PointLioParams readParameters(
    rclcpp_lifecycle::LifecycleNode* n);

/**
 * @brief SO(3) → ZYX 欧拉角
 * @param orient SO(3) 旋转
 * @return 3维欧拉角 [roll, pitch, yaw] (rad)
 */
