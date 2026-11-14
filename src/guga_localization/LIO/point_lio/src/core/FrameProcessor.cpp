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

void FrameProcessor::onLidarFrames(std::deque<PointCloudXYZI::Ptr>& frames,
                                   std::deque<double>& timestamps) {
  lidar_.appendFrames(frames, timestamps);
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
  initializeFilter();
}

void FrameProcessor::initializeFilter() {
  manifold_->initialize(config_, lidar_.ivox(), lidar_.RotationLidarToImu(),
                        lidar_.TranslationLidarToImu());
}

void FrameProcessor::setPose(geometry_msgs::msg::Pose& pose) const {
  const auto& state = manifold_->get_x();
  pose.position.x = state.pos(0);
  pose.position.y = state.pos(1);
  pose.position.z = state.pos(2);
  Eigen::Quaterniond q(state.rot);
  pose.orientation.x = q.coeffs()[0];
  pose.orientation.y = q.coeffs()[1];
  pose.orientation.z = q.coeffs()[2];
  pose.orientation.w = q.coeffs()[3];
}

PointCloudXYZI::Ptr FrameProcessor::loadPointcloudFromPcd(
    const std::string& file_path) {
  auto pcd_ptr = std::make_shared<PointCloudXYZI>();

  if (pcl::io::loadPCDFile(file_path, *pcd_ptr) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("laserMapping"),
                 "Couldn't read pcd file %s", file_path.c_str());
    return nullptr;
  }

  RCLCPP_INFO(rclcpp::get_logger("laserMapping"), "Loaded %zu points from %s",
              pcd_ptr->size(), file_path.c_str());
  return pcd_ptr;
}

void FrameProcessor::initScan() {
  const auto& imu_next = imu_.next();
  std::cout << "first imu time: " << imu_next.timestamp << '\n';
  time_current_ = 0.0;

  manifold_->mutable_x().gravity = config_.imu.processor.gravity;

  imu_.discardBefore(measures_.lidar_start_time);
}

bool FrameProcessor::syncPackages() {
  return Synchronizer::syncPackages(lidar_, imu_, measures_);
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
  state_.odom_aft_mapped.header.frame_id = "camera_init";
  state_.odom_aft_mapped.child_frame_id = "body";
  state_.odom_aft_mapped.header.stamp = get_ros_time(
      config_.mapping.publish_odometry_without_downsample ? time_current_
                                                          : lidar_end_time_);

  auto set_pose = [&](const State& state) {
    auto& pose = state_.odom_aft_mapped.pose.pose;
    pose.position.x = state.pos(0);
    pose.position.y = state.pos(1);
    pose.position.z = state.pos(2);
    Eigen::Quaterniond q(state.rot);
    pose.orientation.x = q.coeffs()[0];
    pose.orientation.y = q.coeffs()[1];
    pose.orientation.z = q.coeffs()[2];
    pose.orientation.w = q.coeffs()[3];
  };

  set_pose(manifold_->toStateOutput());

  data_handler_.odometry = state_.odom_aft_mapped;
  data_handler_.odometry_valid = true;
}

bool FrameProcessor::initMapState() {
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
    if (config_.sensor.enable_prior_map) {
      auto map_cloud = loadPointcloudFromPcd(config_.sensor.prior_map_path);
      lidar_.ivox()->AddPoints(map_cloud->points);
    } else {
      lidar_.ivox()->AddPoints(state_.init_feats_world->points);
    }

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*state_.init_feats_world, map_msg);
    map_msg.header.stamp = get_ros_time(measures_.lidar_last_time);
    map_msg.header.frame_id = "camera_init";
    data_handler_.map_cloud = map_msg;
    state_.init_feats_world = std::make_shared<PointCloudXYZI>();
    stage_ = PointLioStage::TRACKING;

    return true;
  }
  return false;
}

bool FrameProcessor::prepareFrame() {
  imu_.process(measures_, manifold_->mutable_x());

  if (imu_.needInit()) {
    stage_ = PointLioStage::INITIALIZINGIMU;
    return false;
  }

  switch (stage_) {
    case PointLioStage::INITIALIZINGIMU:
      stage_ = PointLioStage::INITIALIZINGMAP;
      [[fallthrough]];

    case PointLioStage::INITIALIZINGMAP:
      if (!initMapState()) {
        return false;
      }
      break;

    default:
      break;
  }

  if (config_.mapping.space_down_sample) {
    state_.downsize_filter_surf.setInputCloud(measures_.lidar_measure_origin);
    state_.downsize_filter_surf.filter(*measures_.feats_down_body);
    sort(measures_.feats_down_body->points.begin(),
         measures_.feats_down_body->points.end(), time_list);
  } else {
    measures_.feats_down_body = measures_.lidar_measure_origin;
    sort(measures_.feats_down_body->points.begin(),
         measures_.feats_down_body->points.end(), time_list);
  }

  measures_.time_seq = time_compressing(measures_.feats_down_body);
  measures_.feats_down_size = measures_.feats_down_body->points.size();
  output_.feats_down_world->resize(measures_.feats_down_size);

  return true;
}

bool FrameProcessor::initializeIteration() {
  if (!syncPackages()) {
    return false;
  }

  lidar_end_time_ = measures_.lidar_last_time;

  if (stage_ == PointLioStage::WAITINGFORDATA) {
    initScan();
    stage_ = PointLioStage::INITIALIZINGIMU;
  }

  return prepareFrame();
}

bool FrameProcessor::processIteration() {
  if (!initializeIteration()) {
    return false;
  }

  processFramePoints(measures_, last_time_output_);

  if (!config_.mapping.publish_odometry_without_downsample) {
    publishOdometry();
  }
  if (measures_.feats_down_size > 4
      && (!config_.sensor.enable_prior_map || ++state_.sleep_time > 200)) {
    mapIncremental();
  }
  return true;
}

void FrameProcessor::processFramePoints(MeasureGroup& meas, double& last_time) {
  const double acc_scale = config_.imu.processor.gravity_magnitude
                           / config_.imu.acc_norm;
  const size_t imu_count = meas.imu_measure.size();
  const size_t group_count = measures_.time_seq.size();

  if (imu_count == 0 && is_first_frame_) {
    return;
  }

  if (is_first_frame_) {
    last_time = meas.lidar_start_time;
    time_update_last_ = meas.lidar_start_time;
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
                        measures_.feats_down_body->points[point_offset])
                        / 1000.0
            : std::numeric_limits<double>::infinity();

    if (imu_time <= lidar_time) {
      time_current_ = imu_time;
      const auto& imu_msg = meas.imu_measure[imu_idx].measurement;
      imu_.setCurrentMeasurement(imu_msg);
      const V3D omega = imu_msg.angular_velocity;
      const V3D acc_meas = imu_msg.linear_acceleration * acc_scale;

      if (!manifold_->stateTransform(imu_time, omega, acc_meas)) {
        ++imu_idx;
        continue;
      }

      const auto& state = manifold_->get_x_predict();
      Matrix<double, 6, 1> r_I;
      r_I.segment<3>(0) = imu_msg.angular_velocity - state.omega;
      r_I.segment<3>(3) = imu_msg.linear_acceleration * acc_scale - state.acc;

      std::array<bool, 6> saturated{};
      if (config_.imu.check_saturation) {
        const double gyro[3] = {imu_msg.angular_velocity(0),
                                imu_msg.angular_velocity(1),
                                imu_msg.angular_velocity(2)};
        const double acc[3] = {imu_msg.linear_acceleration(0),
                               imu_msg.linear_acceleration(1),
                               imu_msg.linear_acceleration(2)};
        for (int axis = 0; axis < 3; ++axis) {
          if (std::abs(gyro[axis]) >= 0.99 * config_.imu.saturation_gyro) {
            saturated[axis] = true;
            r_I(axis) = 0.0;
          }
          if (std::abs(acc[axis]) >= 0.99 * config_.imu.saturation_acc) {
            saturated[axis + 3] = true;
            r_I(axis + 3) = 0.0;
          }
        }
      }

      manifold_->setImuResidual(r_I);
      manifold_->applyImuUpdate(saturated);
      static int dbg_count = 0;
      if (++dbg_count % 50 == 0) {
        data_handler_.filter_state = manifold_->get_x();
      }
      last_time = imu_time;
      ++imu_idx;
    } else {
      time_current_ = lidar_time;
      if (!manifold_->stateTransform(time_current_, manifold_->get_x().omega,
                                     manifold_->get_x().acc)) {
        point_offset += measures_.time_seq[group_idx];
        ++group_idx;
        continue;
      }
      // 启动阶段先只传播不更新, 等 acc 收敛后再接入雷达观测.
      if (startup_frames_ < 20) {
        ++startup_frames_;
        manifold_->commitPrediction();
        point_offset += measures_.time_seq[group_idx];
        ++group_idx;
        continue;
      }
      const int effect_num = manifold_->updateLidar(
          measures_, output_, config_.lidar, group_idx, point_offset);
      if (effect_num == 0) {
        manifold_->commitPrediction();
        output_.processed_size = point_offset + measures_.time_seq[group_idx];
        point_offset += measures_.time_seq[group_idx];
        ++group_idx;
        continue;
      }
      manifold_->applyLidarUpdate();
      manifold_->updateWorldPoints(measures_, output_, config_.lidar, group_idx,
                                   point_offset);
      output_.processed_size = point_offset + measures_.time_seq[group_idx];
      last_time = time_current_;
      if (config_.mapping.publish_odometry_without_downsample) {
        publishOdometry();
      }
      point_offset += measures_.time_seq[group_idx];
      ++group_idx;
    }
  }
}
