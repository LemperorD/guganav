#include "point_lio/core/FrameProcessor.h"
#include "point_lio/core/Synchronizer.h"

FrameProcessor::FrameProcessor(Imu& imu, PointLioStage& stage, Lidar& lidar,
                               const PointLioParams& config,
                               MainLoopState& state)
    : imu_(imu), stage_(stage), lidar_(lidar), config_(config), state_(state) {
}

void FrameProcessor::initialize() {
  initializeFilter();
}

void FrameProcessor::initializeFilter() {
  filter_.configure(config_.filter);
  filter_.initialize(lidar_.measurementModel(), imu_.measurementModel());
  if (config_.lidar.extrinsic_estimation) {
    filter_.input().x_.offset_R_L_I = lidar_.lidarRotation();
    filter_.input().x_.offset_T_L_I = lidar_.lidarTranslation();
    filter_.output().x_.offset_R_L_I = lidar_.lidarRotation();
    filter_.output().x_.offset_T_L_I = lidar_.lidarTranslation();
  }
}

void FrameProcessor::setPose(geometry_msgs::msg::Pose& pose) const {
  const auto set_from_filter = [&](const auto& kf) {
    pose.position.x = kf.x_.pos(0);
    pose.position.y = kf.x_.pos(1);
    pose.position.z = kf.x_.pos(2);
    Eigen::Quaterniond q(kf.x_.rot);
    pose.orientation.x = q.coeffs()[0];
    pose.orientation.y = q.coeffs()[1];
    pose.orientation.z = q.coeffs()[2];
    pose.orientation.w = q.coeffs()[3];
  };
  if (config_.mapping.use_imu_as_input)
    set_from_filter(filter_.input());
  else
    set_from_filter(filter_.output());
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

  if (config_.imu.processor.enabled) {
    filter_.input().x_.gravity = config_.imu.processor.gravity;
    filter_.output().x_.gravity = config_.imu.processor.gravity;
    imu_.discardBefore(measures_.lidar_start_time);
  } else {
    filter_.input().x_.gravity = config_.imu.processor.gravity;
    filter_.output().x_.gravity = config_.imu.processor.gravity;
    filter_.output().x_.acc = config_.imu.processor.gravity;
    filter_.output().x_.acc *= -1;
  }
}

bool FrameProcessor::syncPackages() {
  return synchronizer_.syncPackages(lidar_, imu_, measures_);
}

void FrameProcessor::configureSynchronizer(double lidar_time_interval) {
  synchronizer_.configure(lidar_time_interval);
}

void FrameProcessor::pointBodyLidarToIMU(const PointType* pi,
                                         PointType* po) const {
  const V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  if (config_.lidar.extrinsic_estimation) {
    if (config_.mapping.use_imu_as_input) {
      p_body_imu = filter_.input().x_.offset_R_L_I * p_body_lidar
                   + filter_.input().x_.offset_T_L_I;
    } else {
      p_body_imu = filter_.output().x_.offset_R_L_I * p_body_lidar
                   + filter_.output().x_.offset_T_L_I;
    }
  } else {
    p_body_imu = lidar_.lidarRotation() * p_body_lidar
                 + lidar_.lidarTranslation();
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

  auto set_pose = [&](const auto& kf) {
    auto& pose = state_.odom_aft_mapped.pose.pose;
    pose.position.x = kf.x_.pos(0);
    pose.position.y = kf.x_.pos(1);
    pose.position.z = kf.x_.pos(2);
    Eigen::Quaterniond q(kf.x_.rot);
    pose.orientation.x = q.coeffs()[0];
    pose.orientation.y = q.coeffs()[1];
    pose.orientation.z = q.coeffs()[2];
    pose.orientation.w = q.coeffs()[3];
  };
  if (config_.mapping.use_imu_as_input) {
    set_pose(filter_.input());
  } else {
    set_pose(filter_.output());
  }
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
  if (stage_ == PointLioStage::TRACKING) {
    return true;
  }
  lidar_.workspace().feats_down_world->resize(state_.feats_undistort->size());
  for (int i = 0; i < (int)state_.feats_undistort->size(); i++) {
    if (config_.mapping.use_imu_as_input) {
      lidar_.measurementModel().pointBodyToWorld(
          &(state_.feats_undistort->points[i]),
          &(lidar_.workspace().feats_down_world->points[i]),
          filter_.input().x_);
    } else {
      lidar_.measurementModel().pointBodyToWorld(
          &(state_.feats_undistort->points[i]),
          &(lidar_.workspace().feats_down_world->points[i]),
          filter_.output().x_);
    }
  }
  for (const auto& point : *lidar_.workspace().feats_down_world) {
    state_.init_feats_world->points.emplace_back(point);
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
    state_.init_feats_world.reset(new PointCloudXYZI());
    stage_ = PointLioStage::TRACKING;
    return true;
  }
  return false;
}

bool FrameProcessor::prepareFrame(
    std::function<void(const sensor_msgs::msg::PointCloud2&)> publish) {
  imu_.process(measures_, state_.feats_undistort, filter_.input().x_,
               filter_.output().x_);

  if (imu_.needInit()) {
    stage_ = PointLioStage::INITIALIZINGIMU;
    return false;
  }
  if (stage_ == PointLioStage::INITIALIZINGIMU) {
    stage_ = PointLioStage::INITIALIZINGMAP;
  }

  if (!initMapState(publish)) {
    return false;
  }

  if (config_.mapping.space_down_sample) {
    state_.downsize_filter_surf.setInputCloud(state_.feats_undistort);
    state_.downsize_filter_surf.filter(*lidar_.workspace().feats_down_body);
    sort(lidar_.workspace().feats_down_body->points.begin(),
         lidar_.workspace().feats_down_body->points.end(), time_list);
  } else {
    lidar_.workspace().feats_down_body = measures_.lidar;
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
  lidar_.workspace().crossmat_list.resize(lidar_.workspace().feats_down_size);
  lidar_.workspace().pbody_list.resize(lidar_.workspace().feats_down_size);

  preparePointMeasurements();

  return true;
}

void FrameProcessor::preparePointMeasurements() const {
  for (size_t i = 0; i < lidar_.workspace().feats_down_body->size(); i++) {
    V3D point_this(lidar_.workspace().feats_down_body->points[i].x,
                   lidar_.workspace().feats_down_body->points[i].y,
                   lidar_.workspace().feats_down_body->points[i].z);

    lidar_.workspace().pbody_list[i] = point_this;
    if (!config_.lidar.extrinsic_estimation) {
      point_this = lidar_.lidarRotation() * point_this
                   + lidar_.lidarTranslation();
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);
      lidar_.workspace().crossmat_list[i] = point_crossmat;
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
    stage_ = imu_.needInit() ? PointLioStage::INITIALIZINGIMU
                             : PointLioStage::INITIALIZINGMAP;
  }

  return prepareFrame(publish);
}

bool FrameProcessor::processIteration(
    const std::function<void(const sensor_msgs::msg::PointCloud2&)>&
        publish_map,
    const std::function<void(const nav_msgs::msg::Odometry&)>& publish_odom,
    const std::function<void(const geometry_msgs::msg::TransformStamped&)>&
        publish_tf) {
  if (!initializeIteration(publish_map))
    return false;
  if (config_.mapping.use_imu_as_input) {
    processFramePoints<true>(filter_.input(), last_time_input_,
                             filter_.inputNoise(), publish_odom, publish_tf);
  } else {
    processFramePoints<false>(filter_.output(), last_time_output_,
                              filter_.outputNoise(), publish_odom, publish_tf);
  }
  if (!config_.mapping.publish_odometry_without_downsample) {
    publishOdometry(publish_odom, publish_tf);
  }
  if (lidar_.workspace().feats_down_size > 4
      && (!config_.sensor.enable_prior_map || ++state_.sleep_time > 200)) {
    mapIncremental();
  }
  return true;
}

template <bool ImuAsInput, typename KF>
void FrameProcessor::processFramePoints(
    KF& kf, double& last_time, auto& q,
    const std::function<void(const nav_msgs::msg::Odometry&)>& publish,
    const std::function<void(const geometry_msgs::msg::TransformStamped&)>&
        publish_tf) {
  const auto& imu_last = imu_.last();
  const auto& imu_next = imu_.next();
  if (lidar_.workspace().time_seq.empty()) {
    if (!imu_.empty()) {
      imu_.advanceCursor();

      while (get_time_sec(imu_next.header.stamp) > time_current_
             && (get_time_sec(imu_next.header.stamp)
                 < measures_.lidar_start_time
                       + config_.lidar.lidar_time_interval)) {
        if (is_first_frame_) {
          while (get_time_sec(imu_next.header.stamp)
                 < measures_.lidar_start_time
                       + config_.lidar.lidar_time_interval) {
            imu_.popAndAdvance();
            if (imu_.empty()) {
              break;
            }
          }

          if constexpr (ImuAsInput) {
            input_in_ = imu_.lastInput(config_.imu.processor.gravity_magnitude
                                       / config_.imu.acc_norm);
          } else {
            const auto measurement = imu_.lastMeasurement();
            imu_.setCurrentMeasurement(measurement);
          }

          last_time = time_current_;
          time_update_last_ = time_current_;
          is_first_frame_ = false;
          break;
        }
        time_current_ = get_time_sec(imu_next.header.stamp);

        if constexpr (ImuAsInput) {
          double dt_cov = time_current_ - time_update_last_;
          if (dt_cov > 0.0) {
            time_update_last_ = get_time_sec(imu_next.header.stamp);
          }
          last_time = get_time_sec(imu_next.header.stamp);
          input_in_ = imu_.nextInput(config_.imu.processor.gravity_magnitude
                                     / config_.imu.acc_norm);
        } else {
          double dt = time_current_ - last_time;
          double dt_cov = time_current_ - time_update_last_;
          if (dt_cov > 0.0) {
            filter_.predict(kf, dt_cov, q, input_in_, false, true);
            time_update_last_ = time_current_;
          }
          filter_.predict(kf, dt, q, input_in_, true, false);
          last_time = time_current_;
          const auto measurement = imu_.nextMeasurement();
          imu_.setCurrentMeasurement(measurement);
          filter_.updateOutputImu();
        }

        imu_.popAndAdvance();
        if (imu_.empty()) {
          break;
        }
      }
    }
    return;
  }

  double pcl_beg_time = measures_.lidar_start_time;
  lidar_.workspace().idx = -1;
  for (lidar_.workspace().k = 0;
       lidar_.workspace().k < (int)lidar_.workspace().time_seq.size();
       lidar_.workspace().k++) {
    PointType& point_body =
        lidar_.workspace()
            .feats_down_body
            ->points[lidar_.workspace().idx
                     + lidar_.workspace().time_seq[lidar_.workspace().k]];
    const double point_offset_ms = point_time_offset_ms(point_body);
    time_current_ = (point_offset_ms / 1000.0) + pcl_beg_time;
    if (is_first_frame_) {
      while (time_current_ > get_time_sec(imu_next.header.stamp)) {
        imu_.popAndAdvance();
        if (imu_.empty()) {
          break;
        }
      }
      if constexpr (ImuAsInput) {
        input_in_ = imu_.lastInput(config_.imu.processor.gravity_magnitude
                                   / config_.imu.acc_norm);
      } else if (config_.imu.processor.enabled) {
        const auto measurement = imu_.lastMeasurement();
        imu_.setCurrentMeasurement(measurement);
      }

      is_first_frame_ = false;
      last_time = time_current_;
      time_update_last_ = time_current_;
    }

    if constexpr (ImuAsInput) {
      while (time_current_ > get_time_sec(imu_next.header.stamp)) {
        imu_.popBuffer();
        input_in_ = imu_.lastInput(config_.imu.processor.gravity_magnitude
                                   / config_.imu.acc_norm);
        double dt = get_time_sec(imu_last.header.stamp) - last_time;
        double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last_;

        if (dt_cov > 0.0) {
          filter_.predict(kf, dt_cov, q, input_in_, false, true);
          time_update_last_ = get_time_sec(imu_last.header.stamp);
        }

        filter_.predict(kf, dt, q, input_in_, true, false);
        last_time = get_time_sec(imu_last.header.stamp);

        if (imu_.empty()) {
          break;
        }
        imu_.advanceCursor();
      }
    } else if (config_.imu.processor.enabled && !imu_.empty()) {
      const bool last_imu = imu_.isSameStamp();
      while (!imu_.empty() && get_time_sec(imu_next.header.stamp) < last_time) {
        if (!last_imu) {
          imu_.advanceCursor();
          break;
        }
        imu_.popAndAdvance();
        if (imu_.empty()) {
          break;
        }
      }

      while (!imu_.empty()
             && time_current_ > get_time_sec(imu_next.header.stamp)) {
        const auto measurement = imu_.nextMeasurement();
        imu_.setCurrentMeasurement(measurement);

        const double imu_time = get_time_sec(imu_next.header.stamp);
        double dt = imu_time - last_time;
        filter_.predict(kf, dt, q, input_in_, true, false);
        last_time = imu_time;

        double dt_cov = imu_time - time_update_last_;
        if (dt_cov > 0.0) {
          filter_.predict(kf, dt_cov, q, input_in_, false, true);
          time_update_last_ = imu_time;
          filter_.updateOutputImu();
        }
        imu_.popAndAdvance();
      }
    }

    double dt = time_current_ - last_time;
    if (!config_.mapping.propagate_at_imu_frequency) {
      double dt_cov = time_current_ - time_update_last_;
      if (dt_cov > 0.0) {
        filter_.predict(kf, dt_cov, q, input_in_, false, true);
        time_update_last_ = time_current_;
      }
    }
    filter_.predict(kf, dt, q, input_in_, true, false);
    last_time = time_current_;

    if (lidar_.workspace().feats_down_size < 1) {
      RCLCPP_WARN(rclcpp::get_logger("laserMapping"),
                  "No point, skip this scan!\n");
      lidar_.workspace().idx +=
          lidar_.workspace().time_seq[lidar_.workspace().k];
      continue;
    }

    if (!filter_.updateLidar(kf)) {
      lidar_.workspace().idx =
          lidar_.workspace().idx
          + lidar_.workspace().time_seq[lidar_.workspace().k];
      continue;
    }
    if (config_.mapping.publish_odometry_without_downsample) {
      publishOdometry(publish, publish_tf);
    }

    for (int j = 0; j < lidar_.workspace().time_seq[lidar_.workspace().k];
         j++) {
      PointType& point_body_j =
          lidar_.workspace()
              .feats_down_body->points[lidar_.workspace().idx + j + 1];
      PointType& point_world_j =
          lidar_.workspace()
              .feats_down_world->points[lidar_.workspace().idx + j + 1];
      lidar_.measurementModel().pointBodyToWorld(&point_body_j, &point_world_j,
                                                 kf.x_);
    }

    lidar_.workspace().idx += lidar_.workspace().time_seq[lidar_.workspace().k];
  }
}

using InputKf = esekfom::esekf<state_input, 24, input_ikfom, state_input, 0>;
using OutputKf = esekfom::esekf<state_output, 30, input_ikfom, state_output, 0>;
using InputNoise = Eigen::Matrix<double, 24, 24>;
using OutputNoise = Eigen::Matrix<double, 30, 30>;
using OdomCallback = std::function<void(const nav_msgs::msg::Odometry&)>;
using TfCallback =
    std::function<void(const geometry_msgs::msg::TransformStamped&)>;

template void FrameProcessor::processFramePoints<true, InputKf, InputNoise>(
    InputKf&, double&, InputNoise&, const OdomCallback&, const TfCallback&);
template void FrameProcessor::processFramePoints<false, OutputKf, OutputNoise>(
    OutputKf&, double&, OutputNoise&, const OdomCallback&, const TfCallback&);
