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

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <point_lio/common_lib.h>

// ==================== 预配置 ====================

/// @brief IMU 初始化最大累积帧数
#define MAX_INI_COUNT (100)

/**
 * @brief 按点时间偏移排序的谓词
 *
 * 点云的 PCL curvature 字段存储该点的时间偏移 (ms)，
 * 按时间升序排列以便做时间分组处理。
 */
bool time_list(PointType& x, PointType& y);

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
class ImuProcessor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcessor();

  struct Params {
    bool enabled{true};
    V3D gravity{V3D::Zero()};
    V3D gravity_init{0.0, 0.0, -9.81};
    double gravity_magnitude{9.81};
  };

  enum class Stage { Initializing, Ready };

  void configure(const Params& params);

  /**
   * @brief 重置 IMU 处理状态 (用于 rosbag 回放重定位)
   */
  void reset();

  /**
   * @brief 处理当前帧 (IMU 初始化或点云去畸变)
   * @param meas 当前帧的测量组 (点云 + IMU 数据)
   * @param[out] pcl_un_ 处理后/去畸变后的点云
   */
  void process(const MeasureGroup& meas, PointCloudXYZI::Ptr pcl_un_,
               state_input& input_state, state_output& output_state);

  [[nodiscard]] bool needInit() const;

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
  void Set_init(Eigen::Vector3d& tmp_gravity, Eigen::Matrix3d& rot);

  /**
   * @brief 状态级初始化 (只执行一次): 重力对齐 → 初始姿态 → KF 状态赋值
   *
   * 数据级累积完成后调用:
   * - IMU 模式: 用累积的 mean_acc 估计重力方向
   * - 无 IMU 模式: 用配置的先验重力 gravity_init
   */
  void initState(state_input& input_state, state_output& output_state);

  V3D gravity_;             ///< 先验重力向量 (世界坐标系, 从 YAML 读取)
  V3D gravity_init_{0.0, 0.0, -9.81};
  double gravity_magnitude_{9.81};

  V3D mean_acc{V3D::Zero()};  ///< 平均加速度 (累积, 用于重力估计)

private:
  /**
   * @brief IMU 初始化子函数: 累积加速度和角速度的滑动平均
   *
   * 使用滑动平均公式: mean += (new - old_mean) / N
   * 累积 MAX_INI_COUNT 帧后进入 Ready 阶段
   *
   * @param meas 当前帧的测量组 (包含 IMU 数据)
   * @param N 累积计数 (输入输出)
   */
  void IMU_init(const MeasureGroup& meas, int& N);

  int init_iter_num = 1;      ///< 初始化迭代计数 (当前累积帧数)
  bool imu_en{true};          ///< 是否启用 IMU
  Stage stage_{Stage::Initializing};
  rclcpp::Logger logger;      ///< ROS2 日志器
};
