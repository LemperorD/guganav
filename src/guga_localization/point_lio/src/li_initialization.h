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
#include "common_lib.h"

#include "Estimator.h"

/// @brief 最大缓冲区大小
#define MAXN (720000)

// ==================== 数据缓冲区 ====================

/// @brief IMU 数据队列 (按时间序排列)
extern std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

// ==================== IMU 帧间数据 ====================

extern sensor_msgs::msg::Imu imu_last;  ///< 上一帧 IMU (用于插值/传播)
extern sensor_msgs::msg::Imu imu_next;  ///< 下一帧 IMU (用于插值/传播)

// ==================== 调试数据数组 ====================

extern double s_plot11[MAXN];  ///< 预处理耗时 (秒)

// ==================== 回调函数声明 ====================

/**
 * @brief 标准 pointcloud2 点云回调 (Velodyne/Ouster/Hesai)
 *
 * 处理流程:
 * 1. 时间戳回环检测 → 丢弃乱序帧
 * 2. 如果 cut_frame_init 启用: process_cut_frame_pcl2() 切分帧
 * 3. 否则: process() 常规处理
 * 4. 如果 con_frame 启用: 累积多帧合并
 * 5. 最终推入 lidar_buffer
 *
 * @param msg 标准 ROS2 点云消息
 */
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

/**
 * @brief Livox 自定义点云回调 (avia_handler)
 *
 * 处理流程与 standard_pcl_cbk 类似，
 * 但使用 Livox CustomMsg 格式和 avia_handler。
 *
 * @param msg Livox 自定义点云消息
 */
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);

/**
 * @brief IMU 数据回调
 *
 * 处理流程:
 * 1. 时间戳校准 (timediff_imu_wrt_lidar)
 * 2. 时间戳回环检测 → 丢弃乱序数据
 * 3. 推入 imu_deque 队列
 *
 * @param msg_in IMU 消息 (角速度 + 线加速度)
 */
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr& msg_in);

/**
 * @brief LiDAR-IMU 数据同步和打包
 *
 * 该函数是 Point-LIO 数据管线的核心调度函数，在主循环中按 500Hz 调用:
 *
 * **IMU 禁用模式**:
 *   - 直接从 lidar_buffer 取一帧，打包到 meas
 *
 * **IMU 启用模式**:
 *   - 等待 lidar_buffer 和 imu_deque 都有数据
 *   - 取一帧雷达: 计算 lidar_end_time (最远点时间 + 帧起始时间)
 *   - 等待 IMU 数据到达: last_timestamp_imu >= lidar_end_time
 *   - 将 [lidar_beg_time, lidar_end_time] 范围内的 IMU 数据打包到 meas.imu
 *
 * @param[out] meas 输出的 MeasureGroup (雷达帧 + IMU 队列)
 * @return true 成功打包一组数据, false 数据不足需继续等待
 */
bool sync_packages(MeasureGroup& meas);

class Lidar {
public:
  Lidar() = default;
  ~Lidar() = default;

  void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
  void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);
  void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr& msg_in);
  bool sync_packages(MeasureGroup& meas);

  // ==================== 线程同步 ====================
  sensor_msgs::msg::Imu imu_last;  ///< 上帧 IMU
  sensor_msgs::msg::Imu imu_next;  ///< 下帧 IMU
  PointCloudXYZI::Ptr ptr_con =
      std::make_shared<PointCloudXYZI>();  ///< 合帧累积点云

  // ==================== 调试数组 ====================
  double T1[MAXN]{};        ///< 时间戳数组
  double s_plot[MAXN]{};    ///< 总耗时
  double s_plot2[MAXN]{};   ///< 特征点数
  double s_plot3[MAXN]{};   ///< 平均耗时
  double s_plot11[MAXN]{};  ///< 预处理耗时

  int scan_count = 0;         ///< 接收帧数
  int frame_ct = 0;           ///< 合帧计数
  bool lidar_pushed = false;  ///< 雷达帧已推入 (IMU模式防重复取帧)
  bool imu_pushed = false;    ///< IMU 已推入
  std::deque<PointCloudXYZI::Ptr> lidar_buffer;  ///< 雷达帧缓冲队列
  std::deque<double> time_buffer;                ///< 雷达时间戳缓冲队列
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr>
      imu_deque;  ///< IMU 数据缓冲队列
};
