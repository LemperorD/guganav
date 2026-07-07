/**
 * @file IMU_Processing.h
 * @brief IMU 预处理和运动补偿 (去畸变) 模块
 *
 * ImuProcess 类负责:
 * - **IMU 初始化**: 静态阶段累积加速度/角速度数据估计初始重力和陀螺仪零偏
 * - **重力对齐**: Set_init() 计算初始姿态旋转使估计重力与先验重力对齐
 * - **运动畸变矫正**: Process() 在 IMU 初始化完成后复制原始点云 (去畸变未实现)
 *
 * @note 当前实现中 Process() 仅做初始化判断和点云复制，
 *       实际的 IMU 预积分和去畸变在 laserMapping.cpp 主循环中进行。
 */

#pragma once
#include <math.h>

#include <cmath>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ==================== 预配置 ====================

/// @brief IMU 初始化最大累积帧数
#define MAX_INI_COUNT (100)

/**
 * @brief 按 curvature (时间偏移) 排序的谓词
 *
 * 点云的 curvature 域存储该点的时间偏移 (ms)，
 * 按时间升序排列以便做时间分组处理。
 */
const bool time_list(PointType & x, PointType & y);

/// *************IMU Process and undistortion
/**
 * @class ImuProcess
 * @brief IMU 初始化和预积分处理类
 *
 * 主要职责:
 * 1. 静态初始化: 累积 MAX_INI_COUNT 帧的 IMU 数据，估计:
 *    - 陀螺仪零偏 (平均角速度)
 *    - 重力方向 (平均加速度方向)
 * 2. 重力对齐: 计算初始姿态，使估计重力与 YAML 配置的先验重力对齐
 * 3. 点云去畸变: (预留) IMU 反向传播校正点云运动畸变
 */
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  /**
   * @brief 重置 IMU 处理状态 (用于 rosbag 回放重定位)
   */
  void Reset();

  /**
   * @brief 处理当前帧 (IMU 初始化或点云去畸变)
   * @param meas 当前帧的测量组 (点云 + IMU 数据)
   * @param[out] pcl_un_ 处理后/去畸变后的点云
   */
  void Process(const MeasureGroup & meas, PointCloudXYZI::Ptr pcl_un_);

  /** @brief 设置陀螺仪协方差缩放因子 */
  void set_gyr_cov(const V3D & scaler);

  /** @brief 设置加速度计协方差缩放因子 */
  void set_acc_cov(const V3D & scaler);

  /**
   * @brief 计算初始旋转矩阵，使估计重力与先验重力对齐
   *
   * 对齐方法:
   * - 如果两个重力方向共线 (align_norm ≈ 0): 根据方向判正负
   * - 否则: 通过叉积求旋转轴，点积求旋转角，构造 SO(3) 元素
   *
   * @param tmp_gravity 估计的重力方向 (如 -mean_acc/|mean_acc|)
   * @param[out] rot 输出初始姿态旋转矩阵
   */
  void Set_init(Eigen::Vector3d & tmp_gravity, Eigen::Matrix3d & rot);

  // ==================== 公有成员变量 ====================

  MD(12, 12) state_cov = MD(12, 12)::Identity();  ///< IMU 状态协方差 (12维: 姿态/速度/位置/零偏)
  int lidar_type;                                   ///< LiDAR 类型 (从参数复制)

  V3D gravity_;                                     ///< 先验重力向量 (世界坐标系, 从 YAML 读取)
  bool imu_en;                                      ///< 是否启用 IMU

  V3D mean_acc;                                     ///< 平均加速度 (累积, 用于重力估计)
  bool imu_need_init_ = true;                       ///< 标志: 是否需要 IMU 初始化
  bool after_imu_init_ = false;                     ///< 标志: IMU 初始化是否已完成
  bool b_first_frame_ = true;                       ///< 标志: 是否第一帧

  double time_last_scan = 0.0;                      ///< 上帧扫描时间
  V3D cov_gyr_scale = V3D(0.0001, 0.0001, 0.0001); ///< 陀螺仪协方差缩放
  V3D cov_vel_scale = V3D(0.0001, 0.0001, 0.0001); ///< 速度协方差缩放

private:
  /**
   * @brief IMU 初始化子函数: 累积加速度和角速度的滑动平均
   *
   * 使用滑动平均公式: mean += (new - old_mean) / N
   * 累积 MAX_INI_COUNT 帧后 imu_need_init_ 置 false
   *
   * @param meas 当前帧的测量组 (包含 IMU 数据)
   * @param N 累积计数 (输入输出)
   */
  void IMU_init(const MeasureGroup & meas, int & N);

  V3D mean_gyr;                                     ///< 平均角速度 (用于陀螺零偏估计)
  int init_iter_num = 1;                            ///< 初始化迭代计数 (当前累积帧数)
  rclcpp::Logger logger;                            ///< ROS2 日志器
};