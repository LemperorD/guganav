/**
 * @file IMU_Processing.cpp
 * @brief IMU 预处理实现: 初始化、重力对齐、点云去畸变
 *
 * 实现:
 * - time_list 排序谓词
 * - IMU_init: 累积加速度/角速度的滑动平均 (在线零偏估计)
 * - Set_init: 估计重力与先验重力的旋转对齐
 * - Process: 主入口 (初始化分发)
 */

#include "IMU_Processing.h"

/**
 * @brief 按 curvature (时间偏移) 升序排序
 *
 * curvature 域存储每个点的扫描偏移时间 (ms),
 * 排序后配合 time_compressing() 进行分组处理。
 */
const bool time_list(PointType & x, PointType & y) { return (x.curvature < y.curvature); };

void ImuProcess::set_gyr_cov(const V3D & scaler) { cov_gyr_scale = scaler; }

void ImuProcess::set_acc_cov(const V3D & scaler) { cov_vel_scale = scaler; }

ImuProcess::ImuProcess()
: b_first_frame_(true), imu_need_init_(true), logger(rclcpp::get_logger("ImuProcess"))
{
  imu_en = true;
  init_iter_num = 1;
  mean_acc = V3D(0, 0, 0.0);     // 平均加速度 (m/s²)
  mean_gyr = V3D(0, 0, 0);       // 平均角速度 (rad/s)
  after_imu_init_ = false;
  state_cov.setIdentity();        // 12维协方差初始化为单位阵
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  RCLCPP_WARN(logger, "reset ImuProcess");
  mean_acc = V3D(0, 0, 0.0);
  mean_gyr = V3D(0, 0, 0);
  imu_need_init_ = true;          // 重新触发初始化
  init_iter_num = 1;
  after_imu_init_ = false;
  time_last_scan = 0.0;
}

/**
 * @brief 重力对齐: 计算初始姿态使估计重力与先验重力一致
 *
 * 算法:
 *   1. 构造 hat_grav = [gravity_]× (先验重力的反对称矩阵)
 *   2. 如果 hat_grav * tmp_gravity ≈ 0 (共线):
 *      - align_cos > 0: 方向相同, rot = I
 *      - align_cos < 0: 方向相反, rot = -I
 *   3. 否则:
 *      - 旋转轴: (hat_grav * tmp_gravity) / |hat_grav * tmp_gravity|
 *      - 旋转角: acos(gravity_·tmp_gravity / (|gravity_| * |tmp_gravity|))
 *      - rot = Exp(axis * angle)  (Rodrigues 公式)
 *
 * @param tmp_gravity 估计的重力方向 (如通过加速度均值估计)
 * @param[out] rot    输出的初始旋转矩阵 (使 rot*tmp_gravity ≈ gravity_)
 */
void ImuProcess::Set_init(Eigen::Vector3d & tmp_gravity, Eigen::Matrix3d & rot)
{
  // 构造先验重力的反对称矩阵 [g_prior]×
  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1), -gravity_(2), 0.0, gravity_(0), gravity_(1),
    -gravity_(0), 0.0;

  // |[g_prior]× * g_est| / (|g_prior||g_est|) 用于判断是否共线
  double align_norm = (hat_grav * tmp_gravity).norm() / gravity_.norm() / tmp_gravity.norm();

  // cosθ = g_prior·g_est / (|g_prior||g_est|)
  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();

  if (align_norm < 1e-6) {
    // -- 共线情况: 两个重力方向平行 ----
    if (align_cos > 1e-6) {
      rot = Eye3d;       // 同向: 无需旋转
    } else {
      rot = -Eye3d;      // 反向: 180° 旋转
    }
  } else {
    // -- 非共线情况: 通过叉积求旋转轴, 点积求旋转角 ----
    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos);
    // Rodrigues 公式: axis * angle → SO(3)
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}

/**
 * @brief IMU 在线初始化: 累积加速度和角速度的滑动平均
 *
 * 累积 MAX_INI_COUNT 帧后:
 *   - mean_acc: 平均加速度 (≈ 重力方向，静止时)
 *   - mean_gyr: 平均角速度 (≈ 陀螺仪零偏)
 *
 * 滑动平均公式 (Welford 增量更新):
 *   mean_new = mean_old + (new_value - mean_old) / N
 *
 * @param meas 当前帧测量组
 * @param N    累积计数 (输入输出, 第一次传入1)
 */
void ImuProcess::IMU_init(const MeasureGroup & meas, int & N)
{
  RCLCPP_INFO(logger, "IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;

  if (b_first_frame_) {
    Reset();
    N = 1;
    b_first_frame_ = false;
    // 第一帧: 直接用第一组 IMU 数据初始化均值
    const auto & imu_acc = meas.imu.front()->linear_acceleration;
    const auto & gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  // 遍历帧内所有 IMU 数据，更新滑动平均
  for (const auto & imu : meas.imu) {
    const auto & imu_acc = imu->linear_acceleration;
    const auto & gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    // 增量更新: mean += (cur - mean) / N
    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    N++;
  }
}

/**
 * @brief IMU 处理主入口
 *
 * 流程:
 *   if IMU 启用:
 *     if 需要初始化:
 *       IMU_init() 累积数据
 *       累积满 MAX_INI_COUNT 帧 → imu_need_init_ = false
 *       返回 (初始化阶段不输出去畸变点云)
 *     else:
 *       初始化完成 → 直接复制原始点云 (去畸变未实现)
 *   else (IMU 禁用):
 *     直接复制原始点云
 *
 * @param meas 当前帧测量组
 * @param[out] cur_pcl_un_ 输出点云 (当前版本为原始点云副本)
 */
void ImuProcess::Process(const MeasureGroup & meas, PointCloudXYZI::Ptr cur_pcl_un_)
{
  if (imu_en) {
    if (meas.imu.empty()) return;  // 无 IMU 数据则跳过

    if (imu_need_init_) {
      {
        // ---- IMU 初始化阶段 ----
        IMU_init(meas, init_iter_num);

        imu_need_init_ = true;

        if (init_iter_num > MAX_INI_COUNT) {
          // 初始化完成
          RCLCPP_INFO(logger, "IMU Initializing: %.1f %%", 100.0);
          imu_need_init_ = false;
          *cur_pcl_un_ = *(meas.lidar);  // 复制原始点云
        }
      }
      return;
    }

    // ---- 初始化已完成 ----
    if (!after_imu_init_) after_imu_init_ = true;
    *cur_pcl_un_ = *(meas.lidar);  // 直接使用原始点云 (去畸变预留)

    // @todo: 实现 IMU 反向传播去畸变
    // 可利用 IMU 预积分在 curvature 时间戳上反向插值校正点坐标

    return;
  } else {
    // ---- IMU 禁用: 直接使用原始点云 ----
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}