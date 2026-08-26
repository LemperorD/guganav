/**
 * @file Estimator.h
 * @brief Point-LIO 核心状态估计器 (ESKF on Manifold)
 *
 * 本模块实现了流形上的误差状态卡尔曼滤波器 (Error-State Kalman Filter on
 * Manifold)。 包含:
 * - 过程噪声协方差矩阵构造
 * - 连续时间系统动态模型 (状态转移函数)
 * - 状态转移雅可比矩阵
 * - 量测模型 (点面距离残差 Jacobian + IMU 伪量测)
 * - 雷达坐标系 → 世界坐标系 点变换
 *
 * - input 模式:  IMU 数据驱动状态传播，激光点云用于量测更新 (24维)
 * - output 模式: IMU 数据作为额外量测，角速度和加速度本身被估计 (30维)
 *
 * 核心技术: IKFoM (Invariant Kalman Filter on Manifolds)
 * 量测模型: 点到隐式移动最小二乘 (IMLS) 平面的距离
 * 地图管理: iVox 增量体素 (Faster-LIO)
 */

#pragma once

#include "point_lio/common_lib.h"
#include "point_lio/parameters.h"

#include <bitset>

class Estimator {
public:
  void configure(const EstimatorParams& params);
  [[nodiscard]] const EstimatorParams& params() const;
  [[nodiscard]] Eigen::Matrix<double, 24, 24> processNoiseCovInput() const;
  [[nodiscard]] Eigen::Matrix<double, 30, 30> processNoiseCovOutput() const;
  [[nodiscard]] Eigen::Matrix<double, 24, 1> getFInput(
      state_input& state, const input_ikfom& input) const;
  [[nodiscard]] Eigen::Matrix<double, 30, 1> getFOutput(
      state_output& state, const input_ikfom& input) const;
  [[nodiscard]] Eigen::Matrix<double, 24, 24> dfDxInput(
      state_input& state, const input_ikfom& input) const;
  [[nodiscard]] Eigen::Matrix<double, 30, 30> dfDxOutput(
      state_output& state, const input_ikfom& input) const;
  void hModelInput(state_input& state, Eigen::Matrix3d cov_p,
                   Eigen::Matrix3d cov_R,
                   esekfom::dyn_share_modified<double>& ekfom_data) const;
  void hModelOutput(state_output& state, Eigen::Matrix3d cov_p,
                    Eigen::Matrix3d cov_R,
                    esekfom::dyn_share_modified<double>& ekfom_data) const;
  void hModelImuOutput(state_output& state,
                       esekfom::dyn_share_modified<double>& ekfom_data) const;
  void pointBodyToWorld(PointType const* input, PointType* output,
                        const state_input& state) const;
  void pointBodyToWorld(PointType const* input, PointType* output,
                        const state_output& state) const;

private:
  EstimatorParams params_;
};

// ==================== 量测相关状态 ====================

struct EstimatorState {
  PointCloudXYZI::Ptr normvec{
      new PointCloudXYZI(100000, 1)};  // 每个有效点的平面法向量
  std::vector<int> time_seq;           // 时间分组序列
  PointCloudXYZI::Ptr feats_down_body{
      new PointCloudXYZI(10000, 1)};  // IMU 坐标系下的降采样特征点云
  PointCloudXYZI::Ptr feats_down_world{
      new PointCloudXYZI(10000, 1)};        // 世界坐标系下的降采样特征点云
  std::vector<V3D> pbody_list;              // IMU 坐标系下的点位置列表
  std::vector<PointVector> Nearest_Points;  // 每个特征点的最近邻点列表
  IVoxType::Ptr ivox_{nullptr};             // iVox 增量体素局部地图
  std::vector<float> pointSearchSqDis = std::vector<float>(
      NUM_MATCH_POINTS);                    // 最近邻搜索距离平方
  std::bitset<100000> point_selected_surf;  // 有效曲面点标记
  std::vector<M3D> crossmat_list;           // 反对称矩阵列表
  int effct_feat_num{0};                    // 当前帧有效特征点总数
  int k{0};                                 // 当前处理的时间分组索引
  int idx{-1};                              // 当前处理的点偏移索引
  input_ikfom input_in;                     // IMU 输入数据
  V3D angvel_avr, acc_avr,
      acc_avr_norm;             // IMU 平均角速度、平均加速度、平均加速度范数
  size_t feats_down_size{0};    // feats_down_body 中的点数
  V3D Lidar_T_wrt_IMU{Zero3d};  // LiDAR → IMU 外参平移
  M3D Lidar_R_wrt_IMU{Eye3d};   // LiDAR → IMU 外参旋转
};

extern EstimatorState estimator_state;
