/**
 * @file li_initialization.h
 * @brief LiDAR-IMU 初始化和传感器数据同步
 *
 * 本模块负责:
 * - **传感器数据接收**: 三种回调函数 (标准点云 / Livox 点云 / IMU)
 * - **帧预处理**: 切帧 (cut_frame) / 合帧 (con_frame) 逻辑
 * - **时间同步**: IMU 时间戳校准 (timediff_imu_wrt_lidar)
 * - **数据打包**: sync_packages() 将雷达帧与对应时间的 IMU 队列组合成
 * MeasureGroup
 *
 * 数据流:
 *   传感器 → 回调 (cbk) → 预处理 → lidar_buffer / imu_deque → sync_packages →
 * MeasureGroup
 */

#pragma once

#include <memory>
#include "preprocess.h"
#include "common_lib.h"
#include "Imu.h"

#include "Estimator.h"

/// @brief 最大缓冲区大小
#define MAXN (720000)

class Lidar {
public:
  Lidar() = default;
  ~Lidar() = default;

  struct Params {
    PreprocessParams preprocess;
    bool imu_enabled{true};
    bool con_frame{false};
    int con_frame_num{1};
    bool cut_frame{false};
    int cut_frame_num{1};
    double lidar_time_interval{0.1};
  };

  void configure(const Params& params);
  [[nodiscard]] int mergeFrameCount() const {
    return params_.con_frame_num;
  }

  void onStandardPcl(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
  void onLivoxPcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);
  bool syncPackages(Imu& imu, MeasureGroup& meas);

  double T1[MAXN]{};
  double s_plot[MAXN]{};
  double s_plot2[MAXN]{};
  double s_plot3[MAXN]{};
  double s_plot11[MAXN]{};  ///< 预处理耗时

  // ==================== 线程同步 ====================

  PointCloudXYZI::Ptr ptr_con =
      std::make_shared<PointCloudXYZI>();  ///< 合帧累积点云

  // ==================== 调试数组 ====================

  int scan_count = 0;         ///< 接收帧数
  int frame_ct = 0;           ///< 合帧计数
  bool lidar_pushed = false;  ///< 雷达帧已推入 (IMU模式防重复取帧)
  bool imu_pushed = false;    ///< IMU 已推入
  std::deque<PointCloudXYZI::Ptr> lidar_buffer;  ///< 雷达帧缓冲队列
  std::deque<double> time_buffer;                ///< 雷达时间戳缓冲队列

  double last_timestamp_lidar = -1.0;
  double time_con = 0.0;
  bool lose_lid = false;

private:
  Params params_;
  Preprocess preprocess_;
};
