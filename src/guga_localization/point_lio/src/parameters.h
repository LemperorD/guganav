/**
 * @file parameters.h
 * @brief Point-LIO 全局参数声明和工具函数
 *
 * 所有配置参数从 YAML 文件通过 ROS2 参数系统读取，存储为全局变量。
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
#include <Python.h>
#include <ivox/ivox3d.h>
#include <math.h>
#include <omp.h>
#include <pcl/common/transforms.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <fstream>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "IMU_Processing.h"
#include "preprocess.h"

// 选择 iVox 节点类型: PHC (Plane-Histogram-Coplanarity) 或 DEFAULT
// #define IVOX_NODE_TYPE_PHC

#ifdef IVOX_NODE_TYPE_PHC
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PointType>;
#else
/// @brief 使用默认 iVox 节点类型的增量体素地图类型
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
#endif

// ==================== 帧控制标志 ====================
extern bool is_first_frame;       ///< 首帧标志
extern double lidar_end_time;     ///< 当前帧结束时间戳 (秒)
extern double first_lidar_time;   ///< 首帧激光雷达时间戳
extern double time_con;           ///< 合帧模式下的起始时间
extern double last_timestamp_lidar;  ///< 上一帧激光雷达时间戳 (用于回环检测)
extern double last_timestamp_imu;    ///< 上一帧 IMU 时间戳 (用于回环检测)
extern int pcd_index;                ///< PCD 文件保存序号

// ==================== iVox 地图参数 ====================
extern IVoxType::Options ivox_options_;  ///< iVox 体素地图配置
extern int ivox_nearby_type;             ///< 近邻搜索类型 (0=中心 6=6邻域 18=18邻域 26=26邻域)

// ==================== EKF 状态 ====================
extern state_input state_in;   ///< IMU-as-input 模式的状态
extern state_output state_out; ///< IMU-as-output 模式的状态

// ==================== ROS2 话题名 ====================
extern std::string lid_topic;  ///< LiDAR 点云话题名 (默认: /livox/lidar)
extern std::string imu_topic;  ///< IMU 数据话题名 (默认: /livox/imu)

// ==================== 模式开关 ====================
extern bool prop_at_freq_of_imu;        ///< 是否按 IMU 频率传播 (vs 按 LiDAR 点频率)
extern bool check_satu;                 ///< 是否检查 IMU 饱和
extern bool con_frame;                  ///< 是否启用合帧模式 (多帧合并为一帧)
extern bool cut_frame;                  ///< 是否启用切帧模式 (单帧切分为多子帧)
extern bool use_imu_as_input;           ///< true=IMU作为输入[24维], false=IMU作为输出[30维]
extern bool space_down_sample;          ///< 是否对降采样后点云再做空间降采样

// ==================== 外参标定 ====================
extern bool extrinsic_est_en;           ///< 是否在线估计 LiDAR-IMU 外参
extern bool publish_odometry_without_downsample;  ///< 是否逐点发布里程计 (vs 逐帧发布)

// ==================== 地图初始化 ====================
extern int init_map_size;               ///< 初始地图所需的最小点数
extern int con_frame_num;               ///< 合帧数 (con_frame 模式)

// ==================== 匹配阈值 ====================
extern double match_s;                  ///< 点面匹配的 Mahalanobis 距离阈值

// ==================== IMU 饱和阈值 ====================
extern double satu_acc;                 ///< 加速度计饱和值 (m/s²)
extern double satu_gyro;                ///< 陀螺仪饱和值 (rad/s)
extern double cut_frame_time_interval;  ///< 切帧时间间隔 (秒)

// ==================== 平面提取阈值 ====================
extern float plane_thr;                 ///< 平面一致性判断的残差阈值
extern double filter_size_surf_min;     ///< 曲面点降采样体素边长 (米)
extern double filter_size_map_min;      ///< 地图点降采样体素边长 (米)
extern double fov_deg;                  ///< LiDAR 视场角 (度)

// ==================== 检测范围 ====================
extern float DET_RANGE;                ///< 最大检测距离 (米)

// ==================== IMU 参数 ====================
extern bool imu_en;                     ///< 是否启用 IMU
extern double imu_time_inte;            ///< IMU 预积分时间步长 (秒)
extern double laser_point_cov;          ///< 激光雷达点的量测噪声协方差
extern double acc_norm;                 ///< 加速度归一化系数
extern double acc_cov_input;            ///< input 模式加速度过程噪声
extern double gyr_cov_input;            ///< input 模式陀螺仪过程噪声
extern double vel_cov;                  ///< output 模式速度过程噪声
extern double gyr_cov_output;           ///< output 模式陀螺仪过程噪声
extern double acc_cov_output;           ///< output 模式加速度过程噪声
extern double b_gyr_cov;                ///< 陀螺仪零偏随机游走协方差
extern double b_acc_cov;                ///< 加速度计零偏随机游走协方差
extern double imu_meas_acc_cov;         ///< IMU 加速度量测噪声协方差
extern double imu_meas_omg_cov;         ///< IMU 角速度量测噪声协方差

// ==================== LiDAR 参数 ====================
extern int lidar_type;                  ///< LiDAR 类型: 1=AVIA 2=VELO16 3=OUST64 4=HESAIxt32
extern int pcd_save_interval;           ///< PCD 保存间隔 (帧数)

// ==================== 重力参数 ====================
extern std::vector<double> gravity_init;  ///< 初始重力向量 (世界坐标系)
extern std::vector<double> gravity;       ///< 当前重力向量

// ==================== 输出开关 ====================
extern bool runtime_pos_log;            ///< 是否输出运行时位姿日志到文件
extern bool pcd_save_en;                ///< 是否启用 PCD 保存
extern bool path_en;                    ///< 是否发布路径话题
extern bool scan_pub_en;                ///< 是否发布配准后点云
extern bool scan_body_pub_en;           ///< 是否发布 IMU 坐标系下的点云
extern bool tf_send_en;                 ///< 是否发布 TF 变换

// ==================== 处理器实例 ====================
extern shared_ptr<Preprocess> p_pre;    ///< 点云预处理模块
extern shared_ptr<ImuProcess> p_imu;    ///< IMU 处理模块

// ==================== 外参 ====================
extern std::vector<double> extrinT;     ///< LiDAR→IMU 平移外参 (3维)
extern std::vector<double> extrinR;     ///< LiDAR→IMU 旋转外参 (9维 行主序)
extern double time_diff_lidar_to_imu;   ///< LiDAR→IMU 时间偏移 (秒)

// ==================== 时间与帧参数 ====================
extern double lidar_time_inte;          ///< LiDAR 帧时间间隔 (秒)
extern double first_imu_time;           ///< 首帧 IMU 时间戳
extern int cut_frame_num;               ///< 切帧数
extern int orig_odom_freq;              ///< 原始里程计发布频率
extern double online_refine_time;       ///< 在线精标定时间 (秒)
extern bool cut_frame_init;             ///< 是否启用初始化切帧

// ==================== EKF 时间戳管理 ====================
extern double time_update_last;          ///< 上次协方差更新时间戳
extern double time_current;              ///< 当前处理时间戳
extern double time_predict_last_const;   ///< 上次状态预测时间戳
extern double t_last;                    ///< input 模式的上一时间戳

// ==================== 先验 PCD 地图 ====================
extern bool enable_prior_pcd;           ///< 是否启用先验 PCD 地图 (重定位模式)
extern string prior_pcd_map_path;       ///< 先验 PCD 地图文件路径
extern std::vector<double> init_pose;   ///< 在地图中的初始位姿 [x, y, z]

// ==================== 数据同步 ====================
extern MeasureGroup Measures;           ///< 当前处理的雷达+IMU 数据组合

// ==================== 调试日志 ====================
extern ofstream fout_out;               ///< 位姿输出文件流
extern ofstream fout_imu_pbp;           ///< IMU 逐点输出文件流

// ==================== 函数声明 ====================

/**
 * @brief 解析所有 ROS2 节点参数
 *
 * 从 YAML 配置文件读取参数，若参数不存在则使用默认值。
 * 解析完成后初始化 Preprocess 和 ImuProcess 实例，
 * 并根据 ivox_nearby_type 设置 iVox 近邻类型。
 *
 * @param n ROS2 节点共享指针
 */
void readParameters(std::shared_ptr<rclcpp::Node> & n);

/** @brief 打开调试日志文件 */
void open_file();

/**
 * @brief SO(3) → ZYX 欧拉角
 * @param orient SO(3) 旋转
 * @return 3维欧拉角 [roll, pitch, yaw] (rad)
 */
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 & orient);

/**
 * @brief 重置 IMU-as-input 模式的卡尔曼滤波协方差矩阵
 * @param[out] P_init 24x24 协方差矩阵
 *
 * 初始协方差设置:
 * - 位置/姿态/速度/外参: 0.1 * I
 * - 重力方向: 0.0001 * I
 * - 零偏: 0.001 * I
 */
void reset_cov(Eigen::Matrix<double, 24, 24> & P_init);

/**
 * @brief 重置 IMU-as-output 模式的卡尔曼滤波协方差矩阵
 * @param[out] P_init_output 30x30 协方差矩阵
 */
void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output);