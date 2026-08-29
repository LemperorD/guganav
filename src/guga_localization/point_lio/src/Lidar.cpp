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




void LidarMeasurementModel::hModelInput(
    state_input& s,
    esekfom::dyn_share_modified<double>& ekfom_data) const {
  VF(4) pabcd;
  pabcd.setZero();
  lio_workspace.normvec->resize(
      lio_workspace.time_seq[lio_workspace.k]);
  int effect_num_k = 0;


  for (int j = 0; j < lio_workspace.time_seq[lio_workspace.k]; j++) {
    PointType& point_body_j =
        lio_workspace.feats_down_body->points[lio_workspace.idx + j + 1];
    PointType& point_world_j =
        lio_workspace.feats_down_world->points[lio_workspace.idx + j + 1];


    this->pointBodyToWorld(&point_body_j, &point_world_j, s);

    V3D p_body =
        lio_workspace.pbody_list[lio_workspace.idx + j + 1];
    double p_norm = p_body.norm();
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
        V3D B(p_crossmat * s.offset_R_L_I.transpose()
              * C);


        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec.transpose(),
            A.transpose(), B.transpose(), C.transpose();
      } else {

        M3D point_crossmat =
            lio_workspace
                .crossmat_list[lio_workspace.idx + j
                               + 1];
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(point_crossmat * C);


        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec.transpose(),
            A.transpose(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }


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
}


void LidarMeasurementModel::hModelOutput(
    state_output& s,
    esekfom::dyn_share_modified<double>& ekfom_data) const {
  VF(4) pabcd;
  pabcd.setZero();
  lio_workspace.normvec->resize(lio_workspace.time_seq[lio_workspace.k]);
  int effect_num_k = 0;


  for (int j = 0; j < lio_workspace.time_seq[lio_workspace.k]; j++) {
    PointType& point_body_j =
        lio_workspace.feats_down_body->points[lio_workspace.idx + j + 1];
    PointType& point_world_j =
        lio_workspace.feats_down_world->points[lio_workspace.idx + j + 1];
    this->pointBodyToWorld(&point_body_j, &point_world_j, s);
    V3D p_body = lio_workspace.pbody_list[lio_workspace.idx + j + 1];
    double p_norm = p_body.norm();
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
}


void LidarMeasurementModel::pointBodyToWorld(PointType const* pi, PointType* po,
                                             const state_input& state) const {
  V3D p_body(pi->x, pi->y, pi->z);

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
  if (params.cut_frame_interval > 0.0) {
    params_.cut_frame_num = std::max(
        1, static_cast<int>(std::lround(params_.lidar_time_interval
                                        / params_.cut_frame_interval)));
  }
  params_.con_frame_num = std::max(1, params_.con_frame_num);
  params_.cut_frame_num = std::max(1, params_.cut_frame_num);
  preprocess_.configure(params_.preprocess);
  measurement_model_.configure(params_);
}

void Lidar::reset() {
  ptr_con_->clear();
  lidar_buffer_.clear();
  time_buffer_.clear();
  scan_count_ = 0;
  frame_ct_ = 0;
  last_timestamp_lidar_ = -1.0;
  time_con_ = 0.0;
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
