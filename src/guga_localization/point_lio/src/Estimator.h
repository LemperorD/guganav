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
 * 支持两种 EKF 模式 (通过 use_imu_as_input 切换):
 * - input 模式:  IMU 数据驱动状态传播，激光点云用于量测更新 (24维)
 * - output 模式: IMU 数据作为额外量测，角速度和加速度本身被估计 (30维)
 *
 * 核心技术: IKFoM (Invariant Kalman Filter on Manifolds)
 * 量测模型: 点到隐式移动最小二乘 (IMLS) 平面的距离
 * 地图管理: iVox 增量体素 (Faster-LIO)
 */

#pragma once

#include "common_lib.h"
#include "parameters.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <bitset>
#include <unordered_set>

class Estimator {
public:
  void configureEstimatorParams(const EstimatorParams& params);
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
  void hModelImuOutput(
      state_output& state,
      esekfom::dyn_share_modified<double>& ekfom_data) const;
  void pointBodyToWorld(PointType const* input, PointType* output) const;

private:
  EstimatorParams params_;
};

void configureEstimatorParams(const EstimatorParams& params);

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
      NUM_MATCH_POINTS);                      // 最近邻搜索距离平方
  std::bitset<100000> point_selected_surf{};  // 有效曲面点标记
  std::vector<M3D> crossmat_list;             // 反对称矩阵列表
  int effct_feat_num{0};                      // 当前帧有效特征点总数
  int k{0};                                   // 当前处理的时间分组索引
  int idx{-1};                                // 当前处理的点偏移索引
  input_ikfom input_in;                       // IMU 输入数据
  V3D angvel_avr, acc_avr,
      acc_avr_norm;             // IMU 平均角速度、平均加速度、平均加速度范数
  size_t feats_down_size{0};    // feats_down_body 中的点数
  V3D Lidar_T_wrt_IMU{Zero3d};  // LiDAR → IMU 外参平移
  M3D Lidar_R_wrt_IMU{Eye3d};   // LiDAR → IMU 外参旋转
};

extern EstimatorState estimator_state;

// ==================== 函数声明 ====================

/**
 * @brief 构造 IMU-as-input 模式的过程噪声协方差矩阵 Q (24×24)
 *
 * 噪声分量:
 *   - gyr_cov_input: 陀螺仪白噪声 (索引 3-5)
 *   - acc_cov_input: 加速度计白噪声 (索引 12-14)
 *   - b_gyr_cov: 陀螺仪零偏随机游走 (索引 15-17)
 *   - b_acc_cov: 加速度计零偏随机游走 (索引 18-20)
 *
 * @return 24×24 对角块协方差矩阵
 */
Eigen::Matrix<double, 24, 24> process_noise_cov_input();

/**
 * @brief 构造 IMU-as-output 模式的过程噪声协方差矩阵 Q (30×30)
 *
 * 额外包含速度噪声 vel_cov (索引 12-14)
 *
 * @return 30×30 对角块协方差矩阵
 */
Eigen::Matrix<double, 30, 30> process_noise_cov_output();

/**
 * @brief 连续时间状态转移函数 f(x, u) — IMU-as-input 模式
 *
 * 动力学模型 (简化):
 *   d(pos)/dt = vel
 *   d(rot)/dt = rot * [ω_m - bg]×         (SO(3) 动力学)
 *   d(vel)/dt = rot * (a_m - ba) + gravity
 *   d(bg)/dt = 0,  d(ba)/dt = 0
 *   d(gravity)/dt = 0
 *   外参不变: d(offset_R_L_I)/dt = 0, d(offset_T_L_I)/dt = 0
 *
 * @param s 当前状态
 * @param in IMU 输入 (加速度+角速度原始测量)
 * @return 24维状态导数向量 \dot{x}
 */
Eigen::Matrix<double, 24, 1> get_f_input(state_input& s, const input_ikfom& in);

/**
 * @brief 连续时间状态转移函数 f(x) — IMU-as-output 模式
 *
 * 与 input 模式的关键区别:
 * - 角速度和加速度是状态变量 (而非从 in 读取)
 * - 状态动力学: d(rot)/dt = rot * [omg]×, d(vel)/dt = rot * acc + gravity
 *
 * @param s 当前状态 (包含 omg/acc 估计)
 * @param in IMU 输入 (在此模式下用于量测而非驱动)
 * @return 30维状态导数向量 \dot{x}
 */
Eigen::Matrix<double, 30, 1> get_f_output(state_output& s,
                                          const input_ikfom& in);

/**
 * @brief 状态转移雅可比矩阵 df/dx — IMU-as-input 模式 (24×24)
 *
 * 用于 EKF 预测步骤的协方差传播:
 *   d(dx)/dt = F * dx + G * dw
 * 其中 F = df/dx 是系统矩阵
 *
 * 非零块:
 *   F(0:3, 12:15) = I            (位置对速度的偏导)
 *   F(12:15, 3:6) = -R*[a]×     (速度对姿态的偏导)
 *   F(12:15, 18:21) = -R         (速度对加速度零偏的偏导)
 *   F(12:15, 21:24) = I          (速度对重力的偏导)
 *   F(3:6, 15:18) = -I           (姿态对陀螺仪零偏的偏导)
 *
 * @return 24×24 雅可比矩阵
 */
Eigen::Matrix<double, 24, 24> df_dx_input(state_input& s,
                                          const input_ikfom& in);

/**
 * @brief 状态转移雅可比矩阵 df/dx — IMU-as-output 模式 (30×30)
 *
 * 与 input 模式的区别:
 *   F(12:15, 18:21) = +R (加速度估计的符号相反)
 *   F(3:6, 15:18) = +I  (角速度估计, 符号相反)
 *
 * @return 30×30 雅可比矩阵
 */
Eigen::Matrix<double, 30, 30> df_dx_output(state_output& s,
                                           const input_ikfom& in);

/**
 * @brief 量测模型 h(x) — IMU-as-input 模式
 *
 * 对 time_seq[k] 组内的每个点:
 * 1. 将点从 IMU 坐标系变换到世界坐标系
 * 2. 在 iVox 地图中搜索 5 个最近邻
 * 3. 用 esti_plane() 估计局部平面参数 [nx, ny, nz, d]
 * 4. 计算点面距离和量测雅可比 H (12维块状结构)
 *
 * 量测残差: z = -n·p_world - d
 * 量测雅可比 H = [n^T, A^T, B^T, C^T]
 *   其中:
 *     n: 平面法向量 (对位置的偏导)
 *     A = [p_imu]× * C (对姿态的偏导)
 *     B = [p_body]× * R_L_I^T * C (对外参旋转的偏导)
 *     C = R^T * n  (对重力?) (原版: 对外参平移的偏导依赖于具体雅可比定义)
 *
 * @param s 当前状态
 * @param cov_p 位置协方差 (未使用)
 * @param cov_R 姿态协方差 (未使用)
 * @param[out] ekfom_data 量测数据结构 (h_x, z, M_Noise, valid)
 */
void h_model_input(state_input& s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
                   esekfom::dyn_share_modified<double>& ekfom_data);

/**
 * @brief 量测模型 h(x) — IMU-as-output 模式
 *
 * 与 input 模式相同的点面距离残差模型，
 * 但使用 kf_output 的状态进行点变换。
 */
void h_model_output(state_output& s, Eigen::Matrix3d cov_p,
                    Eigen::Matrix3d cov_R,
                    esekfom::dyn_share_modified<double>& ekfom_data);

/**
 * @brief IMU 伪量测模型 — IMU-as-output 模式专用
 *
 * 在 output 模式下，IMU 数据不驱动状态传播，而是作为量测:
 *   z_IMU = [ω_meas - (omg + bg); a_meas * G/acc_norm - (acc + ba)]
 *
 * 当 check_satu 启用时，对饱和的 IMU 轴进行零化处理 (satu_check 标记)
 *
 * @param s 当前状态
 * @param[out] ekfom_data 量测数据结构 (z_IMU, R_IMU, satu_check)
 */
void h_model_IMU_output(state_output& s,
                        esekfom::dyn_share_modified<double>& ekfom_data);

/**
 * @brief 将点从雷达/IMU 坐标系变换到世界坐标系
 *
 * 根据 extrinsic_est_en 标志选择:
 * - 在线估计外参: p_world = R_wb * (R_LI * p_body + T_LI) + p_wb
 * - 固定外参:     p_world = R_wb * (R_LI_fixed * p_body + T_LI_fixed) + p_wb
 *
 * @param pi 输入: IMU/雷达坐标系下的点
 * @param po 输出: 世界坐标系下的点
 */
void pointBodyToWorld(PointType const* const pi, PointType* const po);
