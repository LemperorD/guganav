#include "point_lio/FrameProcessor.h"
#include "point_lio/Synchronizer.h"

FrameProcessor::FrameProcessor(Imu& imu, Filter& filter, PointLioStage& stage,
                               Synchronizer& synchronizer, Lidar& lidar,
                               MeasureGroup& measurement,
                               PointLioParams& config, MainLoopState& state)
    : imu_(imu),
      filter_(filter),
      stage_(stage),
      synchronizer_(synchronizer),
      lidar_(lidar),
      measures_(measurement),
      config_(config),
      state_(state) {
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
  lio_workspace.feats_down_world->resize(state_.feats_undistort->size());
  for (int i = 0; i < (int)state_.feats_undistort->size(); i++) {
    if (config_.mapping.use_imu_as_input) {
      lidar_.measurementModel().pointBodyToWorld(
          &(state_.feats_undistort->points[i]),
          &(lio_workspace.feats_down_world->points[i]), filter_.input().x_);
    } else {
      lidar_.measurementModel().pointBodyToWorld(
          &(state_.feats_undistort->points[i]),
          &(lio_workspace.feats_down_world->points[i]), filter_.output().x_);
    }
  }
  for (const auto& point : *lio_workspace.feats_down_world) {
    state_.init_feats_world->points.emplace_back(point);
  }

  if (state_.init_feats_world->size()
      >= (size_t)config_.mapping.init_map_size) {
    if (config_.sensor.enable_prior_map) {
      auto map_cloud = loadPointcloudFromPcd(config_.sensor.prior_map_path);
      lio_workspace.ivox_->AddPoints(map_cloud->points);
    } else {
      lio_workspace.ivox_->AddPoints(state_.init_feats_world->points);
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
    state_.downsize_filter_surf.filter(*lio_workspace.feats_down_body);
    sort(lio_workspace.feats_down_body->points.begin(),
         lio_workspace.feats_down_body->points.end(), time_list);
  } else {
    lio_workspace.feats_down_body = measures_.lidar;
    sort(lio_workspace.feats_down_body->points.begin(),
         lio_workspace.feats_down_body->points.end(), time_list);
  }
  lio_workspace.time_seq = time_compressing(lio_workspace.feats_down_body);
  lio_workspace.feats_down_size = lio_workspace.feats_down_body->points.size();

  lio_workspace.normvec->resize(lio_workspace.feats_down_size);
  lio_workspace.feats_down_world->resize(lio_workspace.feats_down_size);
  lio_workspace.Nearest_Points.resize(lio_workspace.feats_down_size);
  lio_workspace.crossmat_list.resize(lio_workspace.feats_down_size);
  lio_workspace.pbody_list.resize(lio_workspace.feats_down_size);

  preparePointMeasurements();

  return true;
}

void FrameProcessor::preparePointMeasurements() const {
  for (size_t i = 0; i < lio_workspace.feats_down_body->size(); i++) {
    V3D point_this(lio_workspace.feats_down_body->points[i].x,
                   lio_workspace.feats_down_body->points[i].y,
                   lio_workspace.feats_down_body->points[i].z);

    lio_workspace.pbody_list[i] = point_this;
    if (!config_.lidar.extrinsic_estimation) {
      point_this = lio_workspace.Lidar_R_wrt_IMU * point_this
                   + lio_workspace.Lidar_T_wrt_IMU;
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);
      lio_workspace.crossmat_list[i] = point_crossmat;
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

template <bool ImuAsInput, typename KF>
void FrameProcessor::processFramePoints(
    KF& kf, double& last_time, auto& q,
    const std::function<void(const nav_msgs::msg::Odometry&)>& publish,
    const std::function<void(const geometry_msgs::msg::TransformStamped&)>&
        publish_tf) {
  const auto& imu_last = imu_.last();
  const auto& imu_next = imu_.next();
  if (lio_workspace.time_seq.empty()) {
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
            lio_workspace.input_in = imu_.lastInput(
                config_.imu.processor.gravity_magnitude / config_.imu.acc_norm);
          } else {
            const auto measurement = imu_.lastMeasurement();
            lio_workspace.angvel_avr = measurement.angular_velocity;
            lio_workspace.acc_avr = measurement.linear_acceleration;
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
          lio_workspace.input_in = imu_.nextInput(
              config_.imu.processor.gravity_magnitude / config_.imu.acc_norm);
        } else {
          double dt = time_current_ - last_time;
          double dt_cov = time_current_ - time_update_last_;
          if (dt_cov > 0.0) {
            kf.predict(dt_cov, q, lio_workspace.input_in, false, true);
            time_update_last_ = time_current_;
          }
          kf.predict(dt, q, lio_workspace.input_in, true, false);
          last_time = time_current_;
          const auto measurement = imu_.nextMeasurement();
          lio_workspace.angvel_avr = measurement.angular_velocity;
          lio_workspace.acc_avr = measurement.linear_acceleration;
          kf.update_iterated_dyn_share_IMU();
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
  lio_workspace.idx = -1;
  for (lio_workspace.k = 0;
       lio_workspace.k < (int)lio_workspace.time_seq.size();
       lio_workspace.k++) {
    PointType& point_body =
        lio_workspace.feats_down_body
            ->points[lio_workspace.idx
                     + lio_workspace.time_seq[lio_workspace.k]];
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
        lio_workspace.input_in = imu_.lastInput(
            config_.imu.processor.gravity_magnitude / config_.imu.acc_norm);
      } else if (config_.imu.processor.enabled) {
        const auto measurement = imu_.lastMeasurement();
        lio_workspace.angvel_avr = measurement.angular_velocity;
        lio_workspace.acc_avr = measurement.linear_acceleration;
      }

      is_first_frame_ = false;
      last_time = time_current_;
      time_update_last_ = time_current_;
    }

    if constexpr (ImuAsInput) {
      while (time_current_ > get_time_sec(imu_next.header.stamp)) {
        imu_.popBuffer();
        lio_workspace.input_in = imu_.lastInput(
            config_.imu.processor.gravity_magnitude / config_.imu.acc_norm);
        double dt = get_time_sec(imu_last.header.stamp) - last_time;
        double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last_;

        if (dt_cov > 0.0) {
          kf.predict(dt_cov, q, lio_workspace.input_in, false, true);
          time_update_last_ = get_time_sec(imu_last.header.stamp);
        }

        kf.predict(dt, q, lio_workspace.input_in, true, false);
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
        lio_workspace.angvel_avr = measurement.angular_velocity;
        lio_workspace.acc_avr = measurement.linear_acceleration;

        const double imu_time = get_time_sec(imu_next.header.stamp);
        double dt = imu_time - last_time;
        kf.predict(dt, q, lio_workspace.input_in, true, false);
        last_time = imu_time;

        double dt_cov = imu_time - time_update_last_;
        if (dt_cov > 0.0) {
          kf.predict(dt_cov, q, lio_workspace.input_in, false, true);
          time_update_last_ = imu_time;
          kf.update_iterated_dyn_share_IMU();
        }
        imu_.popAndAdvance();
      }
    }

    double dt = time_current_ - last_time;
    if (!config_.mapping.propagate_at_imu_frequency) {
      double dt_cov = time_current_ - time_update_last_;
      if (dt_cov > 0.0) {
        kf.predict(dt_cov, q, lio_workspace.input_in, false, true);
        time_update_last_ = time_current_;
      }
    }
    kf.predict(dt, q, lio_workspace.input_in, true, false);
    last_time = time_current_;

    if (lio_workspace.feats_down_size < 1) {
      RCLCPP_WARN(rclcpp::get_logger("laserMapping"),
                  "No point, skip this scan!\n");
      lio_workspace.idx += lio_workspace.time_seq[lio_workspace.k];
      continue;
    }

    if (!kf.update_iterated_dyn_share_modified()) {
      lio_workspace.idx = lio_workspace.idx
                          + lio_workspace.time_seq[lio_workspace.k];
      continue;
    }
    if (config_.mapping.publish_odometry_without_downsample) {
      publishOdometry(publish, publish_tf);
    }

    for (int j = 0; j < lio_workspace.time_seq[lio_workspace.k]; j++) {
      PointType& point_body_j =
          lio_workspace.feats_down_body->points[lio_workspace.idx + j + 1];
      PointType& point_world_j =
          lio_workspace.feats_down_world->points[lio_workspace.idx + j + 1];
      lidar_.measurementModel().pointBodyToWorld(&point_body_j, &point_world_j,
                                                 kf.x_);
    }

    lio_workspace.idx += lio_workspace.time_seq[lio_workspace.k];
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
