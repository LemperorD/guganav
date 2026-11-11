#include <array>
#include <cmath>
#include <limits>

#include "point_lio/core/FrameProcessor.h"
#include "point_lio/core/Synchronizer.h"
#include "point_lio/core/ManifoldEskf.h"

FrameProcessor::FrameProcessor(Imu& imu, PointLioStage& stage, Lidar& lidar,
                               const PointLioParams& config,
                               MainLoopState& state)
    : imu_(imu),
      stage_(stage),
      lidar_(lidar),
      config_(config),
      state_(state),
      manifold_(std::make_unique<ManifoldEskf>()) {
}

FrameProcessor::~FrameProcessor() = default;

void FrameProcessor::initialize() {
  initializeFilter();
}

void FrameProcessor::initializeFilter() {
  manifold_->configureNoise(config_.lidar.point_covariance,
                            config_.imu.measurement_acc_cov,
                            config_.imu.measurement_gyro_cov);
  manifold_->configureProcessNoise(
      config_.filter.gyr_cov_output, config_.filter.acc_cov_output,
      config_.filter.b_gyr_cov, config_.filter.b_acc_cov);
  Matrix<double, 30, 30> P = Matrix<double, 30, 30>::Identity() * 0.01;
  P.block<3, 3>(18, 18) = config_.filter.gyr_cov_output * Matrix3d::Identity();
  P.block<3, 3>(21, 21) = config_.filter.acc_cov_output * Matrix3d::Identity();
  P.block<3, 3>(9, 9) = config_.filter.b_gyr_cov * Matrix3d::Identity();
  P.block<3, 3>(12, 12) = config_.filter.b_acc_cov * Matrix3d::Identity();
  manifold_->setInitialCovariance(P);

  manifold_->configureFixedExtrinsic(lidar_.RotationLidarToImu(),
                                     lidar_.TranslationLidarToImu());
  if (config_.lidar.extrinsic_estimation) {
    manifold_->mutable_x().offset_R_L_I = lidar_.RotationLidarToImu();
    manifold_->mutable_x().offset_T_L_I = lidar_.TranslationLidarToImu();
  }
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
  std::cout << "first imu time: " << get_time_sec(imu_next.header.stamp)
            << '\n';
  time_current_ = 0.0;

  manifold_->mutable_x().gravity = config_.imu.processor.gravity;

  imu_.discardBefore(measures_.lidar_start_time);
}

bool FrameProcessor::syncPackages() {
  return Synchronizer::syncPackages(lidar_, imu_, measures_);
}

void FrameProcessor::configureSynchronizer(double lidar_time_interval) {
  synchronizer_.configure(lidar_time_interval);
}

void FrameProcessor::pointBodyLidarToIMU(const PointType* pi,
                                         PointType* po) const {
  const V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  if (config_.lidar.extrinsic_estimation) {
    const auto& state = manifold_->get_x();
    p_body_imu = state.offset_R_L_I * p_body_lidar + state.offset_T_L_I;

  } else {
    p_body_imu = lidar_.RotationLidarToImu() * p_body_lidar
                 + lidar_.TranslationLidarToImu();
  }
  po->x = static_cast<float>(p_body_imu(0));
  po->y = static_cast<float>(p_body_imu(1));
  po->z = static_cast<float>(p_body_imu(2));
  po->intensity = pi->intensity;
}

void FrameProcessor::mapIncremental() const {
  PointVector points_to_add;
  const auto cur_pts = lidar_.workspace().feats_down_world->size();
  points_to_add.reserve(cur_pts);
  for (std::size_t i = 0; i < cur_pts; ++i) {
    const PointType& point_world =
        lidar_.workspace().feats_down_world->points[i];
    if (lidar_.workspace().Nearest_Points[i].empty()) {
      points_to_add.emplace_back(point_world);
      continue;
    }
    const auto& points_near = lidar_.workspace().Nearest_Points[i];
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
  lidar_.workspace().ivox_->AddPoints(points_to_add);
}

void FrameProcessor::publishOdometry(
    const std::function<void(const nav_msgs::msg::Odometry&)>& publish,
    const std::function<void(const geometry_msgs::msg::TransformStamped&)>&
        publish_tf) {
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

  publish(state_.odom_aft_mapped);

  if (config_.publish.tf_enabled && publish_tf) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = state_.odom_aft_mapped.header;
    transform.header.frame_id = "camera_init";
    transform.child_frame_id = "aft_mapped";
    const auto& pose = state_.odom_aft_mapped.pose.pose;
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation = pose.orientation;
    publish_tf(transform);
  }
}

bool FrameProcessor::initMapState(
    std::function<void(const sensor_msgs::msg::PointCloud2&)> publish) {
  const size_t old_size = state_.init_feats_world->size();
  const size_t add_size = state_.feats_origin_lidar->size();
  state_.init_feats_world->resize(old_size + add_size);

  for (size_t i = 0; i < add_size; i++) {
    lidar_.measurementModel().pointBodyToWorld(
        &(state_.feats_origin_lidar->points[i]),
        &(state_.init_feats_world->points[old_size + i]),
        manifold_->toStateOutput());
  }

  if (state_.init_feats_world->size()
      >= (size_t)config_.mapping.init_map_size) {
    if (config_.sensor.enable_prior_map) {
      auto map_cloud = loadPointcloudFromPcd(config_.sensor.prior_map_path);
      lidar_.workspace().ivox_->AddPoints(map_cloud->points);
    } else {
      lidar_.workspace().ivox_->AddPoints(state_.init_feats_world->points);
    }

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*state_.init_feats_world, map_msg);
    map_msg.header.stamp = get_ros_time(measures_.lidar_last_time);
    map_msg.header.frame_id = "camera_init";
    publish(map_msg);
    state_.init_feats_world = std::make_shared<PointCloudXYZI>();
    stage_ = PointLioStage::TRACKING;

    return true;
  }
  return false;
}

bool FrameProcessor::prepareFrame(
    std::function<void(const sensor_msgs::msg::PointCloud2&)> publish) {
  State state = manifold_->toStateOutput();
  imu_.process(measures_, state_.feats_origin_lidar, state);
  manifold_->setFromStateOutput(state);

  if (imu_.needInit()) {
    stage_ = PointLioStage::INITIALIZINGIMU;
    return false;
  }

  switch (stage_) {
    case PointLioStage::INITIALIZINGIMU:
      stage_ = PointLioStage::INITIALIZINGMAP;
      [[fallthrough]];

    case PointLioStage::INITIALIZINGMAP:
      if (!initMapState(publish)) {
        return false;
      }
      break;

    default:
      break;
  }

  if (config_.mapping.space_down_sample) {
    state_.downsize_filter_surf.setInputCloud(state_.feats_origin_lidar);
    state_.downsize_filter_surf.filter(*lidar_.workspace().feats_down_body);
    sort(lidar_.workspace().feats_down_body->points.begin(),
         lidar_.workspace().feats_down_body->points.end(), time_list);
  } else {
    lidar_.workspace().feats_down_body = measures_.lidar_measure;
    sort(lidar_.workspace().feats_down_body->points.begin(),
         lidar_.workspace().feats_down_body->points.end(), time_list);
  }

  lidar_.workspace().time_seq = time_compressing(
      lidar_.workspace().feats_down_body);
  lidar_.workspace().feats_down_size =
      lidar_.workspace().feats_down_body->points.size();

  lidar_.workspace().normvec->resize(lidar_.workspace().feats_down_size);
  lidar_.workspace().feats_down_world->resize(
      lidar_.workspace().feats_down_size);
  lidar_.workspace().Nearest_Points.resize(lidar_.workspace().feats_down_size);
  lidar_.workspace().hat_list.resize(lidar_.workspace().feats_down_size);
  lidar_.workspace().pbody_list.resize(lidar_.workspace().feats_down_size);

  preparePointMeasurements();

  return true;
}

void FrameProcessor::preparePointMeasurements() const {
  for (size_t i = 0; i < lidar_.workspace().feats_down_body->size(); i++) {
    V3D point(lidar_.workspace().feats_down_body->points[i].x,
              lidar_.workspace().feats_down_body->points[i].y,
              lidar_.workspace().feats_down_body->points[i].z);

    lidar_.workspace().pbody_list[i] = point;

    if (!config_.lidar.extrinsic_estimation) {
      point = lidar_.RotationLidarToImu() * point
              + lidar_.TranslationLidarToImu();  // p_I = ^IR_L * p_L + ^IT_L

      M3D point_hat = Hat(point);
      lidar_.workspace().hat_list[i] = point_hat;
    }
  }
}

bool FrameProcessor::initializeIteration(
    std::function<void(const sensor_msgs::msg::PointCloud2&)> publish) {
  if (!syncPackages()) {
    return false;
  }

  lidar_end_time_ = measures_.lidar_last_time;

  if (stage_ == PointLioStage::WAITINGFORDATA) {
    initScan();
    stage_ = PointLioStage::INITIALIZINGIMU;
  }

  return prepareFrame(publish);
}

bool FrameProcessor::processIteration(
    const std::function<void(const sensor_msgs::msg::PointCloud2&)>&
        publish_map,
    const std::function<void(const nav_msgs::msg::Odometry&)>& publish_odom,
    const std::function<void(const geometry_msgs::msg::TransformStamped&)>&
        publish_tf) {
  if (!initializeIteration(publish_map)) {
    return false;
  }

  processFramePoints2(measures_, last_time_output_, publish_odom, publish_tf);

  if (!config_.mapping.publish_odometry_without_downsample) {
    publishOdometry(publish_odom, publish_tf);
  }
  if (lidar_.workspace().feats_down_size > 4
      && (!config_.sensor.enable_prior_map || ++state_.sleep_time > 200)) {
    mapIncremental();
  }
  return true;
}

void FrameProcessor::processFramePoints2(
    MeasureGroup& meas, double& last_time,
    const std::function<void(const nav_msgs::msg::Odometry&)>& publish,
    const std::function<void(const geometry_msgs::msg::TransformStamped&)>&
        publish_tf) {
  const double acc_scale = config_.imu.processor.gravity_magnitude
                           / config_.imu.acc_norm;
  const size_t imu_count = meas.imu_measure.size();
  const size_t group_count = lidar_.workspace().time_seq.size();

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
                                ? get_time_sec(
                                      meas.imu_measure[imu_idx]->header.stamp)
                                : std::numeric_limits<double>::infinity();
    const double lidar_time =
        group_idx < group_count
            ? meas.lidar_start_time
                  + point_time_offset_ms(
                        lidar_.workspace()
                            .feats_down_body->points[point_offset])
                        / 1000.0
            : std::numeric_limits<double>::infinity();

    if (imu_time <= lidar_time) {
      time_current_ = imu_time;
      const auto& imu_msg = *meas.imu_measure[imu_idx];
      imu_.setCurrentMeasurement(makeMeasurement(imu_msg));

      if (!manifold_->propagateImu(imu_time)) {
        ++imu_idx;
        continue;
      }
      const auto& state = manifold_->get_x_predict();
      Matrix<double, 6, 1> r_I;
      r_I.segment<3>(0) = Eigen::Vector3d(imu_msg.angular_velocity.x,
                                          imu_msg.angular_velocity.y,
                                          imu_msg.angular_velocity.z)
                          - (state.omg + state.bg);
      r_I.segment<3>(3) = Eigen::Vector3d(imu_msg.linear_acceleration.x,
                                          imu_msg.linear_acceleration.y,
                                          imu_msg.linear_acceleration.z)
                              * acc_scale
                          - (state.acc + state.ba);

      std::array<bool, 6> saturated{};
      if (config_.imu.check_saturation) {
        const double gyro[3] = {imu_msg.angular_velocity.x,
                                imu_msg.angular_velocity.y,
                                imu_msg.angular_velocity.z};
        const double acc[3] = {imu_msg.linear_acceleration.x,
                               imu_msg.linear_acceleration.y,
                               imu_msg.linear_acceleration.z};
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
      last_time = imu_time;
      ++imu_idx;
    } else {
      time_current_ = lidar_time;
      if (!manifold_->propagateLidar(time_current_)) {
        point_offset += lidar_.workspace().time_seq[group_idx];
        ++group_idx;
        continue;
      }
      const int effect_num = manifold_->updateLidar(
          lidar_.workspace(), config_.lidar, group_idx, point_offset);
      if (effect_num == 0) {
        manifold_->commitPrediction();
        point_offset += lidar_.workspace().time_seq[group_idx];
        ++group_idx;
        continue;
      }
      manifold_->applyLidarUpdate();
      manifold_->updateWorldPoints(lidar_.workspace(), config_.lidar, group_idx,
                                   point_offset);
      last_time = time_current_;
      if (config_.mapping.publish_odometry_without_downsample) {
        publishOdometry(publish, publish_tf);
      }
      point_offset += lidar_.workspace().time_seq[group_idx];
      ++group_idx;
    }
  }
}
