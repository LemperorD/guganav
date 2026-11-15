#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "point_lio/core/FrameProcessor.h"
#include "point_lio/core/Synchronizer.h"
#include "point_lio/core/ManifoldEskf.h"

FrameProcessor::FrameProcessor(PointLioStage& stage, DataHandler& data_handler,
                               MainLoopState& state)
    : stage_(stage),
      data_handler_(data_handler),
      state_(state),
      manifold_(std::make_unique<ManifoldEskf>()) {
}

FrameProcessor::~FrameProcessor() = default;

const PointCloudXYZI::Ptr& FrameProcessor::currentLidarOrigin() const {
  return measures_.lidar_measure_origin;
}

const PointCloudXYZI::Ptr& FrameProcessor::currentDownsampledWorld() const {
  return output_.feats_down_world;
}

void FrameProcessor::onImu(const ImuMeasurement& measurement,
                           double timestamp) {
  imu_.append(measurement, timestamp);
}

void FrameProcessor::onLidarFrame(PointCloudXYZI::Ptr points,
                                  double timestamp) {
  lidar_.append(std::move(points), timestamp);
}

void FrameProcessor::initialize() {
  lidar_.configure(config_.lidar);
  imu_.configure(config_.imu);
  synchronizer_.configure(config_.lidar.lidar_time_interval);
  lidar_.ivox() = std::make_shared<IVoxType>(config_.mapping.ivox_options);
  lidar_.setExtrinsics(to_vec3d(config_.sensor.extrinsic_t),
                       to_mat3d(config_.sensor.extrinsic_r));
  state_.downsize_filter_surf.setLeafSize(
      static_cast<float>(config_.mapping.filter_size_surf),
      static_cast<float>(config_.mapping.filter_size_surf),
      static_cast<float>(config_.mapping.filter_size_surf));
  manifold_->initialize(config_, lidar_.ivox(), lidar_.RotationLidarToImu(),
                        lidar_.TranslationLidarToImu());
}

void FrameProcessor::initScan() {
  time_current_ = 0.0;

  manifold_->set_gravity(config_.imu.processor.gravity);

  // 首帧的 IMU 队列在 syncOneFrame 里已按帧结束时间收集, 这里再去掉
  // 帧起点之前的样本, 避免它们进入首帧的滤波.
  auto& imu_measure = measures_.imu_measure;
  imu_measure.erase(
      std::remove_if(imu_measure.begin(), imu_measure.end(),
                     [this](const ImuSample& sample) {
                       return sample.timestamp < measures_.lidar_start_time;
                     }),
      imu_measure.end());

  imu_.discardBefore(measures_.lidar_start_time);
}

void FrameProcessor::pointBodyLidarToIMU(const PointType* pi,
                                         PointType* po) const {
  const V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  p_body_imu = lidar_.RotationLidarToImu() * p_body_lidar
               + lidar_.TranslationLidarToImu();
  po->x = static_cast<float>(p_body_imu(0));
  po->y = static_cast<float>(p_body_imu(1));
  po->z = static_cast<float>(p_body_imu(2));
  po->intensity = pi->intensity;
}

void FrameProcessor::mapIncremental() const {
  PointVector points_to_add;
  const auto cur_pts = output_.processed_size;
  const auto& nearest = manifold_->nearestPoints();
  points_to_add.reserve(cur_pts);
  for (std::size_t i = 0; i < cur_pts; ++i) {
    const PointType& point_world = output_.feats_down_world->points[i];
    if (nearest.empty() || nearest[i].empty()) {
      points_to_add.emplace_back(point_world);
      continue;
    }
    const auto& points_near = nearest[i];
    const Eigen::Vector3f center =
        ((point_world.getVector3fMap() / config_.mapping.filter_size_map)
             .array()
             .floor()
         + 0.5f)
        * config_.mapping.filter_size_map;
    bool need_add = true;
    for (const auto& near : points_near) {
      const Eigen::Vector3f delta = near.getVector3fMap() - center;
      if (std::abs(delta.x()) < 0.5 * config_.mapping.filter_size_map
          && std::abs(delta.y()) < 0.5 * config_.mapping.filter_size_map
          && std::abs(delta.z()) < 0.5 * config_.mapping.filter_size_map) {
        need_add = false;
        break;
      }
    }
    if (need_add) {
      points_to_add.emplace_back(point_world);
    }
  }
  lidar_.ivox()->AddPoints(points_to_add);
}

void FrameProcessor::publishOdometry() {
  data_handler_.odom_state = manifold_->toStateOutput();
  data_handler_.odom_stamp =
      config_.mapping.publish_odometry_without_downsample ? time_current_
                                                          : lidar_end_time_;
  data_handler_.odometry_valid = true;
}

bool FrameProcessor::calibrateLidar() {
  const size_t old_size = state_.init_feats_world->size();
  const size_t add_size = measures_.lidar_measure_origin->size();
  state_.init_feats_world->resize(old_size + add_size);

  for (size_t i = 0; i < add_size; i++) {
    lidar_.measurementModel().pointBodyToWorld(
        &(measures_.lidar_measure_origin->points[i]),
        &(state_.init_feats_world->points[old_size + i]),
        manifold_->toStateOutput());
  }

  if (state_.init_feats_world->size()
      >= (size_t)config_.mapping.init_map_size) {
    lidar_.ivox()->AddPoints(state_.init_feats_world->points);

    data_handler_.init_map_cloud = state_.init_feats_world;
    state_.init_feats_world = std::make_shared<PointCloudXYZI>();
    stage_ = PointLioStage::TRACKING;

    return true;
  }
  return false;
}

bool FrameProcessor::calibrateImu() {
  if (const auto estimate = imu_.process(measures_)) {
    manifold_->initState(*estimate, config_.imu);
  }
  return !imu_.needInit();
}

void FrameProcessor::prepareFrameData() {
  state_.downsize_filter_surf.setInputCloud(measures_.lidar_measure_origin);
  state_.downsize_filter_surf.filter(*measures_.feats_down_body);
  sort(measures_.feats_down_body->points.begin(),
       measures_.feats_down_body->points.end(), time_list);

  measures_.time_seq = time_compressing(measures_.feats_down_body);
  measures_.feats_down_size = measures_.feats_down_body->points.size();
  output_.feats_down_world->resize(measures_.feats_down_size);
  // 新一帧的处理进度从零开始, 避免传播失败时把上一帧的进度带给
  // mapIncremental, 造成对 feats_down_world 的越界读.
  output_.processed_size = 0;
}

bool FrameProcessor::syncOneFrame() {
  if (!Synchronizer::syncPackages(lidar_, imu_, measures_)) {
    return false;
  }
  lidar_end_time_ = measures_.lidar_last_time;
  return true;
}

bool FrameProcessor::calibrate() {
  if (!syncOneFrame()) {
    return false;
  }

  if (stage_ == PointLioStage::WAITINGFORDATA) {
    initScan();
    stage_ = PointLioStage::INITIALIZINGIMU;
  }

  if (stage_ == PointLioStage::INITIALIZINGIMU) {
    if (!calibrateImu()) {
      return false;
    }
    stage_ = PointLioStage::INITIALIZINGMAP;
  }

  if (stage_ == PointLioStage::INITIALIZINGMAP) {
    if (!calibrateLidar()) {
      return false;
    }
  }

  return true;
}

bool FrameProcessor::processIteration() {
  if (!syncOneFrame()) {
    return false;
  }

  prepareFrameData();
  processFramePoints(measures_);

  if (!config_.mapping.publish_odometry_without_downsample) {
    publishOdometry();
  }

  if (measures_.feats_down_size > 4) {
    mapIncremental();
  }
  return true;
}

void FrameProcessor::processFramePoints(MeasureGroup& meas) {
  const size_t imu_count = meas.imu_measure.size();
  const size_t group_count = meas.time_seq.size();

  if (imu_count == 0 && is_first_frame_) {
    return;
  }

  if (is_first_frame_) {
    meas.last_time = meas.lidar_start_time;
    manifold_->setStartTime(meas.lidar_start_time);
    is_first_frame_ = false;
  }

  size_t imu_idx = 0;
  size_t group_idx = 0;
  int point_offset = 0;

  while (imu_idx < imu_count || group_idx < group_count) {
    const double imu_time = imu_idx < imu_count
                                ? meas.imu_measure[imu_idx].timestamp
                                : std::numeric_limits<double>::infinity();
    const double lidar_time =
        group_idx < group_count
            ? meas.lidar_start_time
                  + point_time_offset_ms(
                        meas.feats_down_body->points[point_offset])
                        / 1000.0
            : std::numeric_limits<double>::infinity();

    if (imu_time <= lidar_time) {
      time_current_ = imu_time;
      if (!manifold_->imuUpdate(meas.imu_measure[imu_idx], config_.imu)) {
        ++imu_idx;
        continue;
      }
      meas.last_time = imu_time;
      ++imu_idx;
    } else {
      time_current_ = lidar_time;
      const bool startup = startup_frames_ < 20;
      const int effect_num = manifold_->lidarUpdate(
          meas, output_, config_.lidar, group_idx, point_offset, !startup);
      if (startup) {
        ++startup_frames_;
      }
      if (effect_num > 0) {
        meas.last_time = time_current_;
        if (config_.mapping.publish_odometry_without_downsample) {
          publishOdometry();
        }
      }
      point_offset += meas.time_seq[group_idx];
      ++group_idx;
    }
  }

  data_handler_.filter_state = manifold_->get_x();
}
