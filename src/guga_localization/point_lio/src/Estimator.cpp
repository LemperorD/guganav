/**
 * @file Estimator.cpp
 * @brief Point-LIO 核心状态估计器的实现
 *
 * 实现内容:
 * - 过程噪声协方差构造 (input/output 两种模式)
 * - 连续时间状态转移函数 f(x, u)
 * - 系统矩阵/雅可比矩阵 F = df/dx
 * - 量测模型: 点面距离残差及其雅可比
 * - IMU 伪量测模型 (output 模式)
 * - 坐标变换: Body → World
 *
 * 核心数学:
 * - 运动学: d(pos)/dt = vel, d(rot)/dt = R·[ω]×, d(vel)/dt = R·a + g
 * - 量测: z = n^T·(R·(R_LI·p_L + T_LI) + t) + d (点面距离)
 * - 优化: 迭代卡尔曼更新 (Iterated Kalman Update)
 */

#include "Estimator.h"

// ==================== 全局变量定义 ====================

/** @brief 每个点的平面法向量和截距: (nx, ny, nz, d=截距) */
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));

/** @brief 时间分组序列 (根据 curvature 时间戳变化分组) */
std::vector<int> time_seq;

/** @brief 降采样后的 IMU 坐标系特征点云 */
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI(10000, 1));

/** @brief 降采样后的世界坐标系特征点云 */
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI(10000, 1));

/** @brief IMU 坐标系下的位置缓存 */
std::vector<V3D> pbody_list;

/** @brief 每个特征点的 5 个最近邻 */
std::vector<PointVector> Nearest_Points;

/** @brief iVox 增量体素局部地图 (Faster-LIO 数据结构) */
std::shared_ptr<IVoxType> ivox_ = nullptr;

/** @brief 最近邻搜索距离平方 */
std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

/** @brief 有效曲面点标记数组 */
bool point_selected_surf[100000] = {0};

/** @brief 反对称矩阵缓存: crossmat[i] = [p_body_i]_IMU× */
std::vector<M3D> crossmat_list;

/** @brief 当前帧有效特征点计数 */
int effct_feat_num = 0;

/** @brief 当前处理的时间分组索引 */
int k = 0;

/** @brief 点偏移索引 (指向当前组前一个点) */
int idx = -1;

/** @brief 卡尔曼滤波器实例 — IMU-as-input 模式 (状态24维, 输入6维) */
esekfom::esekf<state_input, 24, input_ikfom> kf_input;

/** @brief 卡尔曼滤波器实例 — IMU-as-output 模式 (状态30维, 输入6维) */
esekfom::esekf<state_output, 30, input_ikfom> kf_output;

/** @brief IMU 输入数据缓存 */
input_ikfom input_in;

/** @brief 平均角速度、加速度、加速度范数 (用于 IMU 伪量测) */
V3D angvel_avr, acc_avr, acc_avr_norm;

/** @brief 降采样后的点数 */
int feats_down_size = 0;

/** @brief LiDAR→IMU 外参 (固定外参模式, 从 YAML 读取) */
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/** @brief 重力加速度大小 (m/s²), 默认 9.81 */
double G_m_s2 = 9.81;

// ==================== 过程噪声协方差 ====================

/**
 * @brief 构造 IMU-as-input 模式的 Q 矩阵
 *
 * 噪声结构 (索引从0):
 *   [3:6]   = gyr_cov_input * I3   (陀螺仪白噪声, 影响姿态)
 *   [12:15] = acc_cov_input * I3   (加速度计白噪声, 影响速度)
 *   [15:18] = b_gyr_cov * I3       (陀螺仪零偏随机游走)
 *   [18:21] = b_acc_cov * I3       (加速度计零偏随机游走)
 *
 * 注释中的 MTK 版本是类型安全但不在此使用的替代方式。
 */
Eigen::Matrix<double, 24, 24> process_noise_cov_input()
{
  Eigen::Matrix<double, 24, 24> cov;
  cov.setZero();
  cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, gyr_cov_input, gyr_cov_input;
  cov.block<3, 3>(12, 12).diagonal() << acc_cov_input, acc_cov_input, acc_cov_input;
  cov.block<3, 3>(15, 15).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  cov.block<3, 3>(18, 18).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  return cov;
}

/**
 * @brief 构造 IMU-as-output 模式的 Q 矩阵
 *
 * 额外包含 vel_cov (索引 12:15), 因为速度本身带有过程噪声。
 */
Eigen::Matrix<double, 30, 30> process_noise_cov_output()
{
  Eigen::Matrix<double, 30, 30> cov;
  cov.setZero();
  cov.block<3, 3>(12, 12).diagonal() << vel_cov, vel_cov, vel_cov;
  cov.block<3, 3>(15, 15).diagonal() << gyr_cov_output, gyr_cov_output, gyr_cov_output;
  cov.block<3, 3>(18, 18).diagonal() << acc_cov_output, acc_cov_output, acc_cov_output;
  cov.block<3, 3>(24, 24).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  cov.block<3, 3>(27, 27).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  return cov;
}

// ==================== 状态转移函数 ====================

/**
 * @brief f(x, u) — IMU-as-input 模式状态导数
 *
 * 输入: IMU 原始测量 (acc, gyro)
 * 状态导数:
 *   \dot{pos} = vel
 *   \dot{rot} = [ω_m - bg]× (用 SO(3) boxminus 实现)
 *   \dot{vel} = R_wb * (a_m - ba) + gravity
 *   零偏和重力不可观，导数为零
 *
 * @return 24维状态导数向量 \dot{x}
 */
Eigen::Matrix<double, 24, 1> get_f_input(state_input & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
  vect3 omega;
  // 去零偏后的角速度: ω = ω_meas - bg (在流形上做 boxminus)
  in.gyro.boxminus(omega, s.bg);

  // 世界坐标系下的比力: a_world = R_wb * (a_meas - ba)
  vect3 a_inertial = s.rot * (in.acc - s.ba);

  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];           ///< d(pos)/dt = vel
    res(i + 3) = omega[i];       ///< d(rot)/dt ~ ω (SO(3) 上的扰动形式)
    res(i + 12) = a_inertial[i] + s.gravity[i];  ///< d(vel)/dt = R·a + g
  }
  return res;
}

/**
 * @brief f(x) — IMU-as-output 模式状态导数
 *
 * 关键区别:
 * - omg 和 acc 是状态变量 (而非从 IMU 输入计算)
 * - \dot{vel} = R_wb * acc + gravity
 * - 重力和零偏不变
 *
 * @return 30维状态导数向量 \dot{x}
 */
Eigen::Matrix<double, 30, 1> get_f_output(state_output & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 30, 1> res = Eigen::Matrix<double, 30, 1>::Zero();
  // 世界坐标系下的加速度: R_wb * acc (acc 是当前估计的加速度)
  vect3 a_inertial = s.rot * s.acc;

  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];           ///< d(pos)/dt = vel
    res(i + 3) = s.omg[i];       ///< d(rot)/dt ~ omg (omg 是状态)
    res(i + 12) = a_inertial[i] + s.gravity[i];  ///< d(vel)/dt = R·acc + g
  }
  return res;
}

// ==================== 状态转移雅可比矩阵 ====================

/**
 * @brief F = \partial f / \partial x — IMU-as-input 模式
 *
 * 线性化点: 当前状态估计 \hat{x}
 * 非零块 (索引从0):
 *   F(0:3, 12:15) = I3    → 位置对速度: ∂pos_dot/∂vel = I
 *   F(12:15, 3:6) = -R·[a_hat]×  → 速度对姿态: ∂vel_dot/∂δθ = -R·[a]×
 *     其中 a_hat = a_meas - ba (去零偏加速度估计)
 *   F(12:15, 18:21) = -R   → 速度对加速度零偏: ∂vel_dot/∂ba = -R
 *   F(12:15, 21:24) = I3   → 速度对重力扰动: ∂vel_dot/∂g = I
 *   F(3:6, 15:18) = -I3    → 姿态对陀螺零偏: ∂ω/∂bg = -I
 *
 * @return 24×24 雅可比矩阵 F
 */
Eigen::Matrix<double, 24, 24> df_dx_input(state_input & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
  // d(pos)/d(vel) = I3
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();

  // 加速度残差: a_hat = a_meas - ba (去零偏)
  vect3 acc_;
  in.acc.boxminus(acc_, s.ba);

  // 角速度残差: ω_hat = ω_meas - bg (去零偏)
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);

  // d(vel_dot)/d(δθ) = -R * [a_hat]× (姿态误差对速度导数的影响)
  cov.template block<3, 3>(12, 3) = -s.rot * MTK::hat(acc_);

  // d(vel_dot)/d(ba) = -R (加速度零偏对速度导数的影响)
  cov.template block<3, 3>(12, 18) = -s.rot;

  // d(vel_dot)/d(gravity) = I3 (重力方向扰动对速度导数的影响)
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();

  // d(rot_dot)/d(bg) = -I3 (陀螺零偏对姿态导数的直接影响)
  cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();

  return cov;
}

/**
 * @brief F = \partial f / \partial x — IMU-as-output 模式
 *
 * 与 input 模式的关键差异:
 *   F(12:15, 18:21) = +R (符号：直接使用状态 acc)
 *   F(3:6, 15:18) = +I3 (符号：角速度是状态而非输入)
 *
 * @return 30×30 雅可比矩阵 F
 */
Eigen::Matrix<double, 30, 30> df_dx_output(state_output & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 30, 30> cov = Eigen::Matrix<double, 30, 30>::Zero();
  // d(pos)/d(vel) = I3
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();

  // d(vel_dot)/d(δθ) = -R * [acc]× (acc 是状态)
  cov.template block<3, 3>(12, 3) = -s.rot * MTK::hat(s.acc);

  // d(vel_dot)/d(acc) = +R (直接使用状态 acc 而非从 in 去零偏)
  cov.template block<3, 3>(12, 18) = s.rot;

  // d(vel_dot)/d(gravity) = I3
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();

  // d(rot_dot)/d(omg) = +I3 (角速度是状态)
  cov.template block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();

  return cov;
}

// ==================== 量测模型 ====================

/**
 * @brief 点到平面距离量测模型 — IMU-as-input 模式
 *
 * 对 time_seq[k] 组内每个点依次处理:
 *
 * **匹配阶段 (match)**:
 *   1. 将 IMU 系下的点转换到世界系 (pointBodyToWorld)
 *   2. 在 iVox 局部地图中搜索 5 个最近邻
 *   3. 用 esti_plane() 拟合局部平面 n·p + d = 0
 *   4. 验证: pd2 = n·p_world + d 的绝对值需满足 |p_body|² > match_s * pd2²
 *      (即 Mahalanobis 距离检验，match_s 为马氏距离)
 *
 * **量测雅可比构造**:
 *   量测残差: z = -n·p_world - d
 *   雅可比 H 的第 j 行块 (1×12):
 *     [n^T  |  A^T  |  B^T  |  C^T]
 *   其中:
 *     n = [nx, ny, nz] (平面法向量, 对 position 的偏导)
 *     C = R^T · n       (世界系下的法向量转到 IMU 系)
 *     A = [p_imu]× · C  (对 SO(3) 姿态误差的偏导)
 *     B = [p_body]× · R_LI^T · C  (对 LiDAR→IMU 外参旋转误差的偏导)
 *
 * @param[out] ekfom_data.h_x  量测雅可比矩阵 (effect_num_k × 12)
 * @param[out] ekfom_data.z    量测残差向量 (effect_num_k × 1)
 * @param[out] ekfom_data.M_Noise 量测噪声标量
 * @param[out] ekfom_data.valid 是否有效 (至少一个特征点)
 */
void h_model_input(
  state_input & s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
  esekfom::dyn_share_modified<double> & ekfom_data)
{
  bool match_in_map = false;
  VF(4) pabcd;         ///< 平面系数: (nx, ny, nz, d)
  pabcd.setZero();
  normvec->resize(time_seq[k]);  ///< 分配该组点数的法向量缓存
  int effect_num_k = 0;  ///< 当前组中的有效特征点数

  // ---- Step 1: 遍历当前时间分组中的每个点，进行特征匹配 ----
  for (int j = 0; j < time_seq[k]; j++) {
    PointType & point_body_j = feats_down_body->points[idx + j + 1];
    PointType & point_world_j = feats_down_world->points[idx + j + 1];

    // 坐标变换: Body → World
    pointBodyToWorld(&point_body_j, &point_world_j);

    V3D p_body = pbody_list[idx + j + 1];  // IMU系下的坐标
    double p_norm = p_body.norm();          // 距 IMU 原点的距离
    V3D p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;

    {
      auto & points_near = Nearest_Points[idx + j + 1];

      // 在 iVox 地图中搜索 5 个最近邻
      ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS);

      if ((points_near.size() < NUM_MATCH_POINTS)) {
        // 近邻不足5个 → 该点无效
        point_selected_surf[idx + j + 1] = false;
      } else {
        point_selected_surf[idx + j + 1] = false;

        // 用 5 个近邻拟合局部平面 (IMLS)
        if (esti_plane(pabcd, points_near, plane_thr)) {
          // 点面距离
          float pd2 = fabs(
            pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z +
            pabcd(3));

          // Mahalanobis 距离检验: |p_body|² > match_s · pd2²
          // match_s 为马氏距离阈值 (典型值 81, 即 9σ)
          if (p_norm > match_s * pd2 * pd2) {
            point_selected_surf[idx + j + 1] = true;
            // 存储平面系数: (nx, ny, nz) 存于 xyz, d 存于 intensity
            normvec->points[j].x = pabcd(0);
            normvec->points[j].y = pabcd(1);
            normvec->points[j].z = pabcd(2);
            normvec->points[j].intensity = pabcd(3);
            effect_num_k++;
          }
        }
      }
    }
  }

  // ---- Step 2: 无有效特征点 → 跳过本组更新 ----
  if (effect_num_k == 0) {
    ekfom_data.valid = false;
    return;
  }

  // ---- Step 3: 构造量测雅可比矩阵 H 和残差向量 z ----
  ekfom_data.M_Noise = laser_point_cov;  // 每个残差的噪声方差
  ekfom_data.h_x.resize(effect_num_k, 12);
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;  // 有效点计数器 (H 的行索引)

  for (int j = 0; j < time_seq[k]; j++) {
    if (point_selected_surf[idx + j + 1]) {
      // 提取平面法向量
      V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);

      if (extrinsic_est_en) {
        // ------ 在线外参估计模式: 需要 B 分量 ------
        V3D p_body = pbody_list[idx + j + 1];
        M3D p_crossmat, p_imu_crossmat;
        // [p_body]× — 点在 IMU 系下的反对称矩阵
        p_crossmat << SKEW_SYM_MATRX(p_body);

        // 经过外参变换后的 IMU 系坐标: p_imu = R_LI * p_body + T_LI
        V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
        // [p_imu]× — 变换后点的反对称矩阵
        p_imu_crossmat << SKEW_SYM_MATRX(point_imu);

        V3D C(s.rot.transpose() * norm_vec);           // C = R^T · n
        V3D A(p_imu_crossmat * C);                     // A = [p_imu]× · C
        V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);  // B = [p_body]× · R_LI^T · C

        // H_j = [n^T | A^T | B^T | C^T]
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
      } else {
        // ------ 固定外参模式: B 部分为零 ------
        M3D point_crossmat = crossmat_list[idx + j + 1];  // [R_LI_fixed * p_body + T_LI_fixed]×
        V3D C(s.rot.transpose() * norm_vec);              // C = R^T · n
        V3D A(point_crossmat * C);                        // A = [p_imu]× · C

        // H_j = [n^T | A^T | 0^T | C^T]
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }

      // 残差: z_j = -n · p_world - d (点面有向距离)
      ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx + j + 1].x -
                        norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                        norm_vec(2) * feats_down_world->points[idx + j + 1].z -
                        normvec->points[j].intensity;

      m++;
    }
  }
  effct_feat_num += effect_num_k;
}

/**
 * @brief 点到平面距离量测模型 — IMU-as-output 模式
 *
 * 与 h_model_input 逻辑完全相同，唯一的区别:
 * - 使用 kf_output 的状态 (通过 pointBodyToWorld 内部的判断)
 * - 量测雅可比结构不变 (同样依赖时间分组)
 *
 * @see h_model_input
 */
void h_model_output(
  state_output & s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
  esekfom::dyn_share_modified<double> & ekfom_data)
{
  bool match_in_map = false;
  VF(4) pabcd;
  pabcd.setZero();
  normvec->resize(time_seq[k]);
  int effect_num_k = 0;

  // ---- Step 1: 遍历点，搜索近邻，拟合平面，马氏距离检验 ----
  for (int j = 0; j < time_seq[k]; j++) {
    PointType & point_body_j = feats_down_body->points[idx + j + 1];
    PointType & point_world_j = feats_down_world->points[idx + j + 1];
    pointBodyToWorld(&point_body_j, &point_world_j);
    V3D p_body = pbody_list[idx + j + 1];
    double p_norm = p_body.norm();
    V3D p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;
    {
      auto & points_near = Nearest_Points[idx + j + 1];

      ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS);

      if ((points_near.size() < NUM_MATCH_POINTS)) {
        point_selected_surf[idx + j + 1] = false;
      } else {
        point_selected_surf[idx + j + 1] = false;
        if (esti_plane(pabcd, points_near, plane_thr)) {
          float pd2 = fabs(
            pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z +
            pabcd(3));

          // Mahalanobis 距离检验 (注释掉的代码是自适应加权的备选方案)
          if (p_norm > match_s * pd2 * pd2) {
            point_selected_surf[idx + j + 1] = true;
            normvec->points[j].x = pabcd(0);
            normvec->points[j].y = pabcd(1);
            normvec->points[j].z = pabcd(2);
            normvec->points[j].intensity = pabcd(3);
            effect_num_k++;
          }
        }
      }
    }
  }

  if (effect_num_k == 0) {
    ekfom_data.valid = false;
    return;
  }

  // ---- Step 2: 构造雅可比和残差 ----
  ekfom_data.M_Noise = laser_point_cov;
  ekfom_data.h_x.resize(effect_num_k, 12);
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;

  for (int j = 0; j < time_seq[k]; j++) {
    if (point_selected_surf[idx + j + 1]) {
      V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);
      if (extrinsic_est_en) {
        V3D p_body = pbody_list[idx + j + 1];
        M3D p_crossmat, p_imu_crossmat;
        p_crossmat << SKEW_SYM_MATRX(p_body);
        V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
        p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(p_imu_crossmat * C);
        V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
      } else {
        M3D point_crossmat = crossmat_list[idx + j + 1];
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(point_crossmat * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }
      ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx + j + 1].x -
                        norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                        norm_vec(2) * feats_down_world->points[idx + j + 1].z -
                        normvec->points[j].intensity;

      m++;
    }
  }
  effct_feat_num += effect_num_k;
}

/**
 * @brief IMU 伪量测模型 — IMU-as-output 模式专用
 *
 * 在 output 模式下，状态包含角速度和加速度 (omg, acc)。
 * IMU 数据不作为系统输入，而是作为观测:
 *
 *   量测残差:
 *     z_IMU[0:3] = ω_meas - (omg + bg)   (角速度误差)
 *     z_IMU[3:6] = a_meas * G/acc_norm - (acc + ba)  (加速度误差)
 *
 *   量测噪声: R_IMU = diag(imu_meas_omg_cov, imu_meas_omg_cov, ..., imu_meas_acc_cov)
 *
 * **IMU 饱和处理**: 当 check_satu 启用时，如果某轴测量值超过饱和阈值的 99%，
 *   则将该轴的量测残差置零 (相当于该轴无信息)，并标记 satu_check。
 *
 * @param[out] ekfom_data.z_IMU   6维 IMU 残差向量
 * @param[out] ekfom_data.R_IMU   6维权值 (噪声标准差的倒数平方?)
 * @param[out] ekfom_data.satu_check 6维饱和标记 (true=该轴饱和)
 */
void h_model_IMU_output(state_output & s, esekfom::dyn_share_modified<double> & ekfom_data)
{
  // 重置饱和标记
  std::memset(ekfom_data.satu_check, false, 6);

  // 角速度残差: ω_meas - (omg + bg)
  ekfom_data.z_IMU.block<3, 1>(0, 0) = angvel_avr - s.omg - s.bg;

  // 加速度残差: a_meas * G/acc_norm - (acc + ba)
  // 注: IMU 测量值已归一化，需要乘以 G/acc_norm 恢复
  ekfom_data.z_IMU.block<3, 1>(3, 0) = acc_avr * G_m_s2 / acc_norm - s.acc - s.ba;

  // 量测噪声权重 (方差倒数)
  ekfom_data.R_IMU << imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_acc_cov,
    imu_meas_acc_cov, imu_meas_acc_cov;

  // ---- IMU 饱和检测与处理 ----
  if (check_satu) {
    // 陀螺仪 x 轴饱和
    if (fabs(angvel_avr(0)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[0] = true;  // 标记饱和
      ekfom_data.z_IMU(0) = 0.0;        // 残差置零 (该轴无信息)
    }
    if (fabs(angvel_avr(1)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[1] = true;
      ekfom_data.z_IMU(1) = 0.0;
    }
    if (fabs(angvel_avr(2)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[2] = true;
      ekfom_data.z_IMU(2) = 0.0;
    }
    // 加速度计饱和
    if (fabs(acc_avr(0)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[3] = true;
      ekfom_data.z_IMU(3) = 0.0;
    }
    if (fabs(acc_avr(1)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[4] = true;
      ekfom_data.z_IMU(4) = 0.0;
    }
    if (fabs(acc_avr(2)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[5] = true;
      ekfom_data.z_IMU(5) = 0.0;
    }
  }
}

/**
 * @brief 雷达/IMU 坐标系点 → 世界坐标系点
 *
 * 变换链:
 *   p_world = R_wb * p_IMU + t_wb
 *   其中 p_IMU = R_LI * p_body + T_LI  (雷达系 → IMU系)
 *
 * 外参选择:
 *   - 在线估计 (extrinsic_est_en=true): 使用 EKF 状态中的 offset_R_L_I 和 offset_T_L_I
 *   - 固定外参 (extrinsic_est_en=false): 使用 YAML 中的 Lidar_R_wrt_IMU 和 Lidar_T_wrt_IMU
 *
 * 模式选择: 根据 use_imu_as_input 选择 kf_input 或 kf_output 的状态
 *
 * @param pi 输入: 雷达/IMU 坐标系下的点
 * @param po 输出: 世界坐标系下的点 (保持 intensity)
 */
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
  V3D p_body(pi->x, pi->y, pi->z);

  V3D p_global;
  if (extrinsic_est_en) {
    // 在线外参估计: 使用 EKF 状态中的外参
    if (!use_imu_as_input) {
      p_global =
        kf_output.x_.rot * (kf_output.x_.offset_R_L_I * p_body + kf_output.x_.offset_T_L_I) +
        kf_output.x_.pos;
    } else {
      p_global = kf_input.x_.rot * (kf_input.x_.offset_R_L_I * p_body + kf_input.x_.offset_T_L_I) +
                 kf_input.x_.pos;
    }
  } else {
    // 固定外参: 使用 YAML 配置
    if (!use_imu_as_input) {
      p_global = kf_output.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                 kf_output.x_.pos;
    } else {
      p_global = kf_input.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                 kf_input.x_.pos;
    }
  }

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;  // 保留强度信息
}