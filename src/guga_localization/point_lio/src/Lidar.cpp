/**
 * @file Lidar.cpp
 * @brief LiDAR 点面量测模型实现
 *
 * 包含点云接入、切帧/合帧、LiDAR-IMU 同步、点面残差和坐标变换。
 */

#include "point_lio/Lidar.h"

#include <algorithm>

LioWorkspace lio_workspace;

void LidarMeasurementModel::configure(const LidarParams& params) {
  params_ = params;
}

// ==================== 量测模型 ====================

/**
 * @brief 点到平面距离量测模型 — IMU-as-input 模式
 *
 * 对 lio_workspace.time_seq[lio_workspace.k] 组内每个点依次处理:
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
void LidarMeasurementModel::hModelInput(
    state_input& s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
    esekfom::dyn_share_modified<double>& ekfom_data) const {
  VF(4) pabcd;  ///< 平面系数: (nx, ny, nz, d)
  pabcd.setZero();
  lio_workspace.normvec->resize(
      lio_workspace.time_seq[lio_workspace.k]);  ///< 分配该组点数的法向量缓存
  int effect_num_k = 0;                          ///< 当前组中的有效特征点数

  // ---- Step 1: 遍历当前时间分组中的每个点，进行特征匹配 ----
  for (int j = 0; j < lio_workspace.time_seq[lio_workspace.k]; j++) {
    PointType& point_body_j =
        lio_workspace.feats_down_body->points[lio_workspace.idx + j + 1];
    PointType& point_world_j =
        lio_workspace.feats_down_world->points[lio_workspace.idx + j + 1];

    // 坐标变换: Body → World
    this->pointBodyToWorld(&point_body_j, &point_world_j, s);

    V3D p_body =
        lio_workspace.pbody_list[lio_workspace.idx + j + 1];  // IMU系下的坐标
    double p_norm = p_body.norm();  // 距 IMU 原点的距离
    V3D p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;

    {
      auto& points_near =
          lio_workspace.Nearest_Points[lio_workspace.idx + j + 1];

      // 在 iVox 地图中搜索 5 个最近邻
      lio_workspace.ivox_->GetClosestPoint(point_world_j, points_near,
                                           NUM_MATCH_POINTS);

      if ((points_near.size() < NUM_MATCH_POINTS)) {
        // 近邻不足5个 → 该点无效
        lio_workspace.point_selected_surf[lio_workspace.idx + j + 1] = false;
      } else {
        lio_workspace.point_selected_surf[lio_workspace.idx + j + 1] = false;

        // 用 5 个近邻拟合局部平面 (IMLS)
        if (esti_plane(pabcd, points_near, params_.plane_threshold)) {
          // 点面距离
          float pd2 = fabs(pabcd(0) * point_world_j.x
                           + pabcd(1) * point_world_j.y
                           + pabcd(2) * point_world_j.z + pabcd(3));

          // Mahalanobis 距离检验: |p_body|² > match_s · pd2²
          // match_s 为马氏距离阈值 (典型值 81, 即 9σ)
          if (p_norm > params_.match_threshold * pd2 * pd2) {
            lio_workspace.point_selected_surf[lio_workspace.idx + j + 1] = true;
            // 存储平面系数: (nx, ny, nz) 存于 xyz, d 存于 intensity
            lio_workspace.normvec->points[j].x = pabcd(0);
            lio_workspace.normvec->points[j].y = pabcd(1);
            lio_workspace.normvec->points[j].z = pabcd(2);
            lio_workspace.normvec->points[j].intensity = pabcd(3);
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
  ekfom_data.M_Noise = params_.point_covariance;
  ekfom_data.h_x.resize(effect_num_k, 12);
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;  // 有效点计数器 (H 的行索引)

  for (int j = 0; j < lio_workspace.time_seq[lio_workspace.k]; j++) {
    if (lio_workspace.point_selected_surf[lio_workspace.idx + j + 1]) {
      // 提取平面法向量
      V3D norm_vec(lio_workspace.normvec->points[j].x,
                   lio_workspace.normvec->points[j].y,
                   lio_workspace.normvec->points[j].z);

      if (params_.extrinsic_estimation) {
        // ------ 在线外参估计模式: 需要 B 分量 ------
        V3D p_body = lio_workspace.pbody_list[lio_workspace.idx + j + 1];
        M3D p_crossmat, p_imu_crossmat;
        // [p_body]× — 点在 IMU 系下的反对称矩阵
        p_crossmat << SKEW_SYM_MATRX(p_body);

        // 经过外参变换后的 IMU 系坐标: p_imu = R_LI * p_body + T_LI
        V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
        // [p_imu]× — 变换后点的反对称矩阵
        p_imu_crossmat << SKEW_SYM_MATRX(point_imu);

        V3D C(s.rot.transpose() * norm_vec);  // C = R^T · n
        V3D A(p_imu_crossmat * C);            // A = [p_imu]× · C
        V3D B(p_crossmat * s.offset_R_L_I.transpose()
              * C);  // B = [p_body]× · R_LI^T · C

        // H_j = [n^T | A^T | B^T | C^T]
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec.transpose(),
            A.transpose(), B.transpose(), C.transpose();
      } else {
        // ------ 固定外参模式: B 部分为零 ------
        M3D point_crossmat =
            lio_workspace
                .crossmat_list[lio_workspace.idx + j
                               + 1];  // [R_LI_fixed * p_body + T_LI_fixed]×
        V3D C(s.rot.transpose() * norm_vec);  // C = R^T · n
        V3D A(point_crossmat * C);            // A = [p_imu]× · C

        // H_j = [n^T | A^T | 0^T | C^T]
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec.transpose(),
            A.transpose(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }

      // 残差: z_j = -n · p_world - d (点面有向距离)
      ekfom_data.z(m) =
          (-norm_vec(0)
           * lio_workspace.feats_down_world->points[lio_workspace.idx + j + 1]
                 .x)
          - (norm_vec(1)
             * lio_workspace.feats_down_world->points[lio_workspace.idx + j + 1]
                   .y)
          - (norm_vec(2)
             * lio_workspace.feats_down_world->points[lio_workspace.idx + j + 1]
                   .z)
          - lio_workspace.normvec->points[j].intensity;
      m++;
    }
  }
  lio_workspace.effct_feat_num += effect_num_k;
}

/**
 * @brief 点到平面距离量测模型 — IMU-as-output 模式
 *
 * 与 hModelInput 逻辑完全相同，唯一的区别:
 * - 使用回调传入的 output 状态进行坐标变换
 * - 量测雅可比结构不变 (同样依赖时间分组)
 *
 * @see hModelInput
 */
void LidarMeasurementModel::hModelOutput(
    state_output& s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
    esekfom::dyn_share_modified<double>& ekfom_data) const {
  VF(4) pabcd;
  pabcd.setZero();
  lio_workspace.normvec->resize(lio_workspace.time_seq[lio_workspace.k]);
  int effect_num_k = 0;

  // ---- Step 1: 遍历点，搜索近邻，拟合平面，马氏距离检验 ----
  for (int j = 0; j < lio_workspace.time_seq[lio_workspace.k]; j++) {
    PointType& point_body_j =
        lio_workspace.feats_down_body->points[lio_workspace.idx + j + 1];
    PointType& point_world_j =
        lio_workspace.feats_down_world->points[lio_workspace.idx + j + 1];
    this->pointBodyToWorld(&point_body_j, &point_world_j, s);
    V3D p_body = lio_workspace.pbody_list[lio_workspace.idx + j + 1];
    double p_norm = p_body.norm();
    V3D p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;
    {
      auto& points_near =
          lio_workspace.Nearest_Points[lio_workspace.idx + j + 1];

      lio_workspace.ivox_->GetClosestPoint(point_world_j, points_near,
                                           NUM_MATCH_POINTS);

      if ((points_near.size() < NUM_MATCH_POINTS)) {
        lio_workspace.point_selected_surf[lio_workspace.idx + j + 1] = false;
      } else {
        lio_workspace.point_selected_surf[lio_workspace.idx + j + 1] = false;
        if (esti_plane(pabcd, points_near, params_.plane_threshold)) {
          float pd2 = fabs(pabcd(0) * point_world_j.x
                           + pabcd(1) * point_world_j.y
                           + pabcd(2) * point_world_j.z + pabcd(3));

          // Mahalanobis 距离检验 (注释掉的代码是自适应加权的备选方案)
          if (p_norm > params_.match_threshold * pd2 * pd2) {
            lio_workspace.point_selected_surf[lio_workspace.idx + j + 1] = true;
            lio_workspace.normvec->points[j].x = pabcd(0);
            lio_workspace.normvec->points[j].y = pabcd(1);
            lio_workspace.normvec->points[j].z = pabcd(2);
            lio_workspace.normvec->points[j].intensity = pabcd(3);
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
  ekfom_data.M_Noise = params_.point_covariance;
  ekfom_data.h_x.resize(effect_num_k, 12);
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;

  for (int j = 0; j < lio_workspace.time_seq[lio_workspace.k]; j++) {
    if (lio_workspace.point_selected_surf[lio_workspace.idx + j + 1]) {
      V3D norm_vec(lio_workspace.normvec->points[j].x,
                   lio_workspace.normvec->points[j].y,
                   lio_workspace.normvec->points[j].z);
      if (params_.extrinsic_estimation) {
        V3D p_body = lio_workspace.pbody_list[lio_workspace.idx + j + 1];
        M3D p_crossmat, p_imu_crossmat;
        p_crossmat << SKEW_SYM_MATRX(p_body);
        V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
        p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(p_imu_crossmat * C);
        V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec.transpose(),
            A.transpose(), B.transpose(), C.transpose();
      } else {
        M3D point_crossmat =
            lio_workspace.crossmat_list[lio_workspace.idx + j + 1];
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(point_crossmat * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec.transpose(),
            A.transpose(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }
      ekfom_data.z(m) = -norm_vec(0)
                            * lio_workspace.feats_down_world
                                  ->points[lio_workspace.idx + j + 1]
                                  .x
                        - norm_vec(1)
                              * lio_workspace.feats_down_world
                                    ->points[lio_workspace.idx + j + 1]
                                    .y
                        - norm_vec(2)
                              * lio_workspace.feats_down_world
                                    ->points[lio_workspace.idx + j + 1]
                                    .z
                        - lio_workspace.normvec->points[j].intensity;

      m++;
    }
  }
  lio_workspace.effct_feat_num += effect_num_k;
}

/**
 * @brief 雷达/IMU 坐标系点 → 世界坐标系点
 *
 * 变换链:
 *   p_world = R_wb * p_IMU + t_wb
 *   其中 p_IMU = R_LI * p_body + T_LI  (雷达系 → IMU系)
 *
 * 外参选择:
 *   - 在线估计 (extrinsic_est_en=true): 使用 EKF 状态中的 offset_R_L_I 和
 * offset_T_L_I
 *   - 固定外参 (extrinsic_est_en=false): 使用 YAML 中的
 * lio_workspace.Lidar_R_wrt_IMU 和 lio_workspace.Lidar_T_wrt_IMU
 *
 *
 * @param pi 输入: 雷达/IMU 坐标系下的点
 * @param po 输出: 世界坐标系下的点 (保持 intensity)
 */
void LidarMeasurementModel::pointBodyToWorld(PointType const* pi, PointType* po,
                                             const state_input& state) const {
  V3D p_body(pi->x, pi->y, pi->z);

  V3D p_global;
  if (params_.extrinsic_estimation) {
    // 在线外参估计: 使用 EKF 状态中的外参

    p_global = state.rot * (state.offset_R_L_I * p_body + state.offset_T_L_I)
               + state.pos;

  } else {
    // 固定外参: 使用 YAML 配置

    p_global = state.rot
                   * (lio_workspace.Lidar_R_wrt_IMU * p_body
                      + lio_workspace.Lidar_T_wrt_IMU)
               + state.pos;
  }

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;  // 保留强度信息
}

void LidarMeasurementModel::pointBodyToWorld(PointType const* pi, PointType* po,
                                             const state_output& state) const {
  const V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global;
  if (params_.extrinsic_estimation) {
    p_global = state.rot * (state.offset_R_L_I * p_body + state.offset_T_L_I)
               + state.pos;
  } else {
    p_global = state.rot
                   * (lio_workspace.Lidar_R_wrt_IMU * p_body
                      + lio_workspace.Lidar_T_wrt_IMU)
               + state.pos;
  }
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void Lidar::configure(const Params& params) {
  params_ = params;
  params_.con_frame_num = std::max(1, params_.con_frame_num);
  params_.cut_frame_num = std::max(1, params_.cut_frame_num);
  preprocess_.configure(params_.preprocess);
  measurement_model_.configure(params_);
}

void Lidar::appendCutFrames(std::deque<PointCloudXYZI::Ptr>& frames,
                            std::deque<double>& timestamps) {
  while (!frames.empty() && !timestamps.empty()) {
    lidar_buffer_.emplace_back(frames.front());
    frames.pop_front();
    time_buffer_.emplace_back(timestamps.front() / 1000.0);
    timestamps.pop_front();
  }
}

void Lidar::appendFrame(PointCloudXYZI::Ptr points, double timestamp) {
  lidar_buffer_.emplace_back(std::move(points));
  time_buffer_.emplace_back(timestamp);
}

void Lidar::appendMergedFrame(const PointCloudXYZI::Ptr& points,
                              double timestamp) {
  if (frame_ct_ == 0) {
    time_con_ = timestamp;
  }
  if (frame_ct_ < mergeFrameCount()) {
    for (auto point : points->points) {
      set_point_time_offset_ms(
          point, point_time_offset_ms(point)
                     + static_cast<float>((timestamp - time_con_) * 1000.0));
      ptr_con_->push_back(point);
    }
    ++frame_ct_;
    return;
  }
  lidar_buffer_.emplace_back(std::make_shared<PointCloudXYZI>(*ptr_con_));
  time_buffer_.emplace_back(time_con_);
  ptr_con_->clear();
  frame_ct_ = 0;
}

void Lidar::onStandardPcl(
    const sensor_msgs::msg::PointCloud2::SharedPtr& message) {
  ++scan_count_;
  const double timestamp = rclcpp::Time(message->header.stamp).seconds();
  if (timestamp < last_timestamp_lidar_) {
    RCLCPP_ERROR(rclcpp::get_logger("Lidar"), "lidar loop back");
    return;
  }
  last_timestamp_lidar_ = timestamp;

  const int type = params_.preprocess.lidar_type;
  if ((type == VELO16 || type == OUST64 || type == HESA_IXT32)
      && params_.cut_frame) {
    std::deque<PointCloudXYZI::Ptr> frames;
    std::deque<double> timestamps;
    preprocess_.processCutFramePCL2(message, frames, timestamps,
                                    params_.cut_frame_num, scan_count_);
    appendCutFrames(frames, timestamps);
    return;
  }
  auto points = std::make_shared<PointCloudXYZI>(20000, 1);
  preprocess_.process(message, points);
  if (params_.con_frame) {
    appendMergedFrame(points, timestamp);
  } else if (!points->empty()) {
    appendFrame(std::move(points), timestamp);
  }
}

void Lidar::onLivoxPcl(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& message) {
  ++scan_count_;
  const double timestamp = rclcpp::Time(message->header.stamp).seconds();
  if (timestamp < last_timestamp_lidar_) {
    RCLCPP_ERROR(rclcpp::get_logger("Lidar"), "lidar loop back");
    return;
  }
  last_timestamp_lidar_ = timestamp;

  if (params_.cut_frame) {
    std::deque<PointCloudXYZI::Ptr> frames;
    std::deque<double> timestamps;
    preprocess_.processCutFrameLivox(message, frames, timestamps,
                                     params_.cut_frame_num, scan_count_);
    appendCutFrames(frames, timestamps);
    return;
  }
  auto points = std::make_shared<PointCloudXYZI>(10000, 1);
  preprocess_.process(message, points);
  if (params_.con_frame) {
    appendMergedFrame(points, timestamp);
  } else if (!points->empty()) {
    appendFrame(std::move(points), timestamp);
  }
}
