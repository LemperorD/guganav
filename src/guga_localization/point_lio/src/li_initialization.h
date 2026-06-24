/**
 * @file li_initialization.h
 * @brief LiDAR-IMU 初始化和传感器数据同步
 *
 * 本模块负责:
 * - **传感器数据接收**: 三种回调函数 (标准点云 / Livox 点云 / IMU)
 * - **帧预处理**: 切帧 (cut_frame) / 合帧 (con_frame) 逻辑
 * - **时间同步**: IMU 时间戳校准 (timediff_imu_wrt_lidar)
 * - **数据打包**: sync_packages() 将雷达帧与对应时间的 IMU 队列组合成 MeasureGroup
 *
 * 数据流:
 *   传感器 → 回调 (cbk) → 预处理 → lidar_buffer / imu_deque → sync_packages → MeasureGroup
 */

#pragma once

#include <common_lib.h>

#include "Estimator.h"

/// @brief 最大缓冲区大小
#define MAXN (720000)

// ==================== 初始化状态标志 ====================

extern bool data_accum_finished;     ///< 数据累积完成 (初始化用)
extern bool data_accum_start;        ///< 数据累积开始
extern bool online_calib_finish;     ///< 在线标定完成
extern bool refine_print;            ///< 精标定打印标志
extern int frame_num_init;           ///< 初始化帧计数

// ==================== 时间标定参数 ====================

extern double time_lag_IMU_wtr_lidar;      ///< IMU→LiDAR 时间延迟 (估计值)
extern double move_start_time;             ///< 运动开始时间
extern double online_calib_starts_time;    ///< 在线标定开始时间

extern double timediff_imu_wrt_lidar;      ///< IMU→LiDAR 固有时差 (用于同步对齐)
extern bool timediff_set_flg;              ///< 时差是否已设置

// ==================== 重力 ====================

extern V3D gravity_lio;                   ///< LIO 估计的重力向量

// ==================== 线程同步 ====================

extern mutex mtx_buffer;                  ///< 缓冲区互斥锁
extern condition_variable sig_buffer;     ///< 缓冲区条件变量 (通知有数据可用)

// ==================== 帧计数器 ====================

extern int scan_count;                    ///< 接收到的总帧数
extern int frame_ct;                      ///< 合帧计数器
extern int wait_num;                      ///< 等待次数

// ==================== 数据缓冲区 ====================

/// @brief 预处理后的 LiDAR 帧队列 (每个元素为一帧点云)
extern std::deque<PointCloudXYZI::Ptr> lidar_buffer;

/// @brief LiDAR 帧时间戳队列 (与 lidar_buffer 一一对应)
extern std::deque<double> time_buffer;

/// @brief IMU 数据队列 (按时间序排列)
extern std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

// ==================== 互斥与时序 ====================

extern std::mutex m_time;
extern bool lidar_pushed;                 ///< lidar 帧是否已推入 MeasureGroup
extern bool imu_pushed;                   ///< IMU 数据是否已推入 MeasureGroup
extern double imu_first_time;             ///< 首帧 IMU 时间戳
extern bool lose_lid;                     ///< 是否丢失了当前激光帧

// ==================== IMU 帧间数据 ====================

extern sensor_msgs::msg::Imu imu_last;    ///< 上一帧 IMU (用于插值/传播)
extern sensor_msgs::msg::Imu imu_next;    ///< 下一帧 IMU (用于插值/传播)

// ==================== 合帧模式 ====================

extern PointCloudXYZI::Ptr ptr_con;       ///< 合帧临时点云 (累积多帧)

// ==================== 调试数据数组 ====================

extern double T1[MAXN];         ///< 时序数据 (主循环记录的时间戳)
extern double s_plot[MAXN];     ///< 总耗时 (秒)
extern double s_plot2[MAXN];    ///< 特征点数
extern double s_plot3[MAXN];    ///< 平均耗时 (秒)
extern double s_plot11[MAXN];   ///< 预处理耗时 (秒)

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
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr & msg);

/**
 * @brief Livox 自定义点云回调 (avia_handler)
 *
 * 处理流程与 standard_pcl_cbk 类似，
 * 但使用 Livox CustomMsg 格式和 avia_handler。
 *
 * @param msg Livox 自定义点云消息
 */
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg);

/**
 * @brief IMU 数据回调
 *
 * 处理流程:
 * 1. 时间戳校准 (timediff_imu_wrt_lidar + time_lag_IMU_wtr_lidar)
 * 2. 时间戳回环检测 → 丢弃乱序数据
 * 3. 推入 imu_deque 队列
 *
 * @param msg_in IMU 消息 (角速度 + 线加速度)
 */
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr & msg_in);

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
bool sync_packages(MeasureGroup & meas);

// #endif