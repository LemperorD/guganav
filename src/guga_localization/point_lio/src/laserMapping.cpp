/**
 * @file laserMapping.cpp
 * @brief Point-LIO 主处理流程 (laserMapping 节点)
 *
 * 这是 Point-LIO 的核心主循环, 负责:
 * - **节点初始化**: ROS2 订阅/发布/参数解析
 * - **主循环** (500Hz): 同步→预测→更新→建图→发布
 *
 * 主循环流水线:
 * @code
 *   sync_packages()            // 1. LiDAR-IMU 时间同步
 *   ↓
 *   p_imu->Process()           // 2. IMU 预处理 (初始化/重力对齐; 去畸变预留)
 *   ↓
 *   downSizeFilterSurf         // 3. 体素降采样
 *   ↓
 *   EKF Predict + Update       // 4. 迭代卡尔曼 (逐点)
 *   ↓
 *   MapIncremental             // 5. 增量地图更新 (iVox)
 *   ↓
 *   publish_odometry/path等    // 6. 发布里程计/路径/点云/TF
 * @endcode
 *
 * 两种 EKF 模式:
 * IMU 驱动预测, 激光做量测更新
 *   角速度和加速度本身被估计, 每帧同时做激光量测和 IMU 量测更新
 */

#include "point_lio/laserMapping.h"
#include "point_lio/FrameProcessor.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserMappingNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

LaserMappingNode::CallbackReturn LaserMappingNode::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!parameters_loaded_) {
    config_ = readParameters(this);
    parameters_loaded_ = true;
  }
  initializeSensors();
  initializeMappingState();
  initializeFilter();
  initializeRos2Interfaces();
  stage_ = PointLioStage::WAITINGFORDATA;
  return CallbackReturn::SUCCESS;
}

void LaserMappingNode::processIteration() {
  if (!initializeIteration()) {
    return;
  }
  if (config_.mapping.use_imu_as_input) {
    processFramePoints<true>(filter_.input(), t_last_, filter_.inputNoise());
  } else {
    processFramePoints<false>(filter_.output(), time_predict_last_const_,
                              filter_.outputNoise());
  }
  if (!config_.mapping.publish_odometry_without_downsample) {
    publishOdometry();
  }
  if (lio_workspace.feats_down_size > 4) {
    if (config_.sensor.enable_prior_map) {
      state_.sleep_time++;
    }
    if (!config_.sensor.enable_prior_map || state_.sleep_time > 200) {
      mapIncremental();
    }
  }
  publishFrameOutputs();
}

LaserMappingNode::CallbackReturn LaserMappingNode::on_activate(
    const rclcpp_lifecycle::State&) {
  pub_laser_cloud_full_res_->on_activate();
  pub_laser_cloud_full_res_body_->on_activate();
  pub_laser_cloud_map_->on_activate();
  pub_odom_aft_mapped_->on_activate();
  pub_path_->on_activate();
  createSensorSubscriptions();
  processing_timer_ = create_wall_timer(
      std::chrono::milliseconds(2), [this]() { this->processIteration(); },
      callback_group_);
  return CallbackReturn::SUCCESS;
}

LaserMappingNode::CallbackReturn LaserMappingNode::on_deactivate(
    const rclcpp_lifecycle::State&) {
  processing_timer_.reset();
  destroySensorSubscriptions();
  pub_laser_cloud_full_res_->on_deactivate();
  pub_laser_cloud_full_res_body_->on_deactivate();
  pub_laser_cloud_map_->on_deactivate();
  pub_odom_aft_mapped_->on_deactivate();
  pub_path_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

LaserMappingNode::CallbackReturn LaserMappingNode::on_cleanup(
    const rclcpp_lifecycle::State&) {
  processing_timer_.reset();
  destroySensorSubscriptions();
  savePendingPcd();
  lidar_.reset();
  imu_.reset();
  measures_ = MeasureGroup{};
  lio_workspace = LioWorkspace{};
  state_.sleep_time = 0;
  state_.init_feats_world->clear();
  state_.feats_undistort->clear();
  state_.pcl_wait_save->clear();
  state_.path = nav_msgs::msg::Path{};
  state_.odom_aft_mapped = nav_msgs::msg::Odometry{};
  state_.msg_body_pose = geometry_msgs::msg::PoseStamped{};
  stage_ = PointLioStage::WAITINGFORDATA;
  is_first_frame_ = true;
  lidar_end_time_ = 0.0;
  pcd_index_ = 0;
  pcd_scan_count_ = 0;
  time_update_last_ = 0.0;
  processor_.time_current_ = 0.0;
  time_predict_last_const_ = 0.0;
  t_last_ = 0.0;
  pub_laser_cloud_full_res_.reset();
  pub_laser_cloud_full_res_body_.reset();
  pub_laser_cloud_map_.reset();
  pub_odom_aft_mapped_.reset();
  pub_path_.reset();
  tf_broadcaster_.reset();
  return CallbackReturn::SUCCESS;
}

LaserMappingNode::CallbackReturn LaserMappingNode::on_shutdown(
    const rclcpp_lifecycle::State&) {
  processing_timer_.reset();
  destroySensorSubscriptions();
  savePendingPcd();
  return CallbackReturn::SUCCESS;
}

void LaserMappingNode::initializeSensors() {
  lidar_.configure(config_.lidar);
  synchronizer_.configure(config_.lidar.lidar_time_interval);

  auto imu_params = config_.imu;
  imu_params.timestamp_offset = config_.sensor.lidar_to_imu_time;
  imu_.configure(imu_params);

  RCLCPP_INFO(get_logger(), "lidar_type: %d.", config_.lidar.lidar_type);
}
void LaserMappingNode::initializeMappingState() {
  lio_workspace.ivox_ = std::make_shared<IVoxType>(
      config_.mapping.ivox_options);
  lio_workspace.point_selected_surf.set();

  lio_workspace.Lidar_T_wrt_IMU = to_vec3d(config_.sensor.extrinsic_t);
  lio_workspace.Lidar_R_wrt_IMU = to_mat3d(config_.sensor.extrinsic_r);

  state_.downsize_filter_surf.setLeafSize(
      static_cast<float>(config_.mapping.filter_size_surf),
      static_cast<float>(config_.mapping.filter_size_surf),
      static_cast<float>(config_.mapping.filter_size_surf));

  state_.path.header.stamp = get_ros_time(lidar_end_time_);
  state_.path.header.frame_id = "camera_init";
}
void LaserMappingNode::initializeFilter() {
  filter_.configure(config_.filter);
  filter_.initialize(lidar_.measurementModel(), imu_.measurementModel());
  if (config_.lidar.extrinsic_estimation) {
    filter_.input().x_.offset_R_L_I = lio_workspace.Lidar_R_wrt_IMU;
    filter_.input().x_.offset_T_L_I = lio_workspace.Lidar_T_wrt_IMU;
    filter_.output().x_.offset_R_L_I = lio_workspace.Lidar_R_wrt_IMU;
    filter_.output().x_.offset_T_L_I = lio_workspace.Lidar_T_wrt_IMU;
  }
}
void LaserMappingNode::initializeRos2Interfaces() {
  pub_laser_cloud_full_res_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_registered", 20);
  pub_laser_cloud_full_res_body_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body",
                                                      20);
  pub_laser_cloud_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "Laser_map", 20);
  pub_odom_aft_mapped_ = create_publisher<nav_msgs::msg::Odometry>(
      "aft_mapped_to_init", 20);
  pub_path_ = create_publisher<nav_msgs::msg::Path>("path", 20);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void LaserMappingNode::createSensorSubscriptions() {
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  if (config_.lidar.lidar_type == AVIA) {
    sub_pcl_livox_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        config_.sensor.lidar_topic, rclcpp::SensorDataQoS(),
        [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
          lidar_.onLivoxPcl(msg);
        },
        options);
  } else {
    sub_pcl_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.sensor.lidar_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          lidar_.onStandardPcl(msg);
        },
        options);
  }
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      config_.sensor.imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        imu_.onMessage(msg);
      },
      options);
}

void LaserMappingNode::destroySensorSubscriptions() {
  sub_pcl_pc_.reset();
  sub_pcl_livox_.reset();
  sub_imu_.reset();
}

bool LaserMappingNode::initializeIteration() {
  if (!processor_.syncPackages()) {
    return false;
  }
  lidar_end_time_ = measures_.lidar_last_time;

  if (stage_ == PointLioStage::WAITINGFORDATA) {
    processor_.initScan();
    stage_ = imu_.needInit() ? PointLioStage::INITIALIZINGIMU
                             : PointLioStage::INITIALIZINGMAP;
  }

  return prepareFrame();
}

bool LaserMappingNode::prepareFrame() {
  imu_.process(measures_, state_.feats_undistort, filter_.input().x_,
               filter_.output().x_);

  if (imu_.needInit()) {
    stage_ = PointLioStage::INITIALIZINGIMU;
    return false;
  }
  if (stage_ == PointLioStage::INITIALIZINGIMU) {
    stage_ = PointLioStage::INITIALIZINGMAP;
  }

  if (!initMapState()) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Initializing map: %zu/%d points",
        state_.init_feats_world->size(), config_.mapping.init_map_size);

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

LaserMappingNode::LaserMappingNode()
    : rclcpp_lifecycle::LifecycleNode("laserMapping"),
      processor_(imu_, filter_, stage_, synchronizer_, lidar_, measures_,
                 config_) {
  callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
}

LaserMappingNode::~LaserMappingNode() {
  try {
    savePendingPcd();
  } catch (...) {
  }
}

PointCloudXYZI::Ptr LaserMappingNode::loadPointcloudFromPcd(
    const std::string& file_path) {
  return FrameProcessor::loadPointcloudFromPcd(file_path);
}

void LaserMappingNode::pointBodyLidarToIMU(PointType const* const pi,
                                           PointType* const po) const {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
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
    p_body_imu = lio_workspace.Lidar_R_wrt_IMU * p_body_lidar
                 + lio_workspace.Lidar_T_wrt_IMU;
  }
  po->x = (float)p_body_imu(0);
  po->y = (float)p_body_imu(1);
  po->z = (float)p_body_imu(2);
  po->intensity = pi->intensity;
}

void LaserMappingNode::mapIncremental() const {
  PointVector points_to_add;
  auto cur_pts = lio_workspace.feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (std::size_t i = 0; i < cur_pts; ++i) {
    PointType& point_world = lio_workspace.feats_down_world->points[i];
    if (!lio_workspace.Nearest_Points[i].empty()) {
      const PointVector& points_near = lio_workspace.Nearest_Points[i];

      Eigen::Vector3f center =
          ((point_world.getVector3fMap() / config_.mapping.filter_size_map)
               .array()
               .floor()
           + 0.5)
          * config_.mapping.filter_size_map;
      bool need_add = true;
      for (const auto x : points_near) {
        Eigen::Vector3f dis_2_center = x.getVector3fMap() - center;
        if (fabs(dis_2_center.x()) < 0.5 * config_.mapping.filter_size_map
            && fabs(dis_2_center.y()) < 0.5 * config_.mapping.filter_size_map
            && fabs(dis_2_center.z()) < 0.5 * config_.mapping.filter_size_map) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  lio_workspace.ivox_->AddPoints(points_to_add);
}

void LaserMappingNode::publishInitMap() {
  sensor_msgs::msg::PointCloud2 laser_cloudmsg;

  pcl::toROSMsg(*state_.init_feats_world, laser_cloudmsg);

  laser_cloudmsg.header.stamp = get_ros_time(lidar_end_time_);
  laser_cloudmsg.header.frame_id = "camera_init";
  pub_laser_cloud_map_->publish(laser_cloudmsg);
}

void LaserMappingNode::publishFrameWorld() {
  if (config_.publish.scan_enabled) {
    sensor_msgs::msg::PointCloud2 laser_cloud_msg;
    pcl::toROSMsg(*lio_workspace.feats_down_world, laser_cloud_msg);

    laser_cloud_msg.header.stamp = get_ros_time(lidar_end_time_);
    laser_cloud_msg.header.frame_id = "camera_init";
    pub_laser_cloud_full_res_->publish(laser_cloud_msg);

    if (config_.publish.pcd_save_enabled) {
      *state_.pcl_wait_save += *lio_workspace.feats_down_world;

      pcd_scan_count_++;
      if (!state_.pcl_wait_save->empty()
          && config_.publish.pcd_save_interval > 0
          && pcd_scan_count_ >= config_.publish.pcd_save_interval) {
        pcd_index_++;
        string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_")
                              + to_string(pcd_index_) + string(".pcd"));
        pcl::PCDWriter pcd_writer;
        std::cout << "current scan saved to /PCD/" << all_points_dir << '\n';
        pcd_writer.writeBinary(all_points_dir, *state_.pcl_wait_save);
        state_.pcl_wait_save->clear();
        pcd_scan_count_ = 0;
      }
    }
  }
}

void LaserMappingNode::publishFrameBody() {
  size_t size = state_.feats_undistort->points.size();
  PointCloudXYZI::Ptr lasercloud_imu_body(new PointCloudXYZI(size, 1));

  for (std::size_t i = 0; i < size; i++) {
    pointBodyLidarToIMU(&state_.feats_undistort->points[i],
                        &lasercloud_imu_body->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laser_cloud_msg;
  pcl::toROSMsg(*lasercloud_imu_body, laser_cloud_msg);
  laser_cloud_msg.header.stamp = get_ros_time(lidar_end_time_);
  laser_cloud_msg.header.frame_id = "body";
  pub_laser_cloud_full_res_body_->publish(laser_cloud_msg);
}

template <typename T>
void LaserMappingNode::setPosestamp(T& out) {
  auto set_output_from_kf = [&](const auto& kf) {
    out.position.x = kf.x_.pos(0);
    out.position.y = kf.x_.pos(1);
    out.position.z = kf.x_.pos(2);
    Eigen::Quaterniond q(kf.x_.rot);
    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];
  };

  if (config_.mapping.use_imu_as_input) {
    set_output_from_kf(filter_.input());
  } else {
    set_output_from_kf(filter_.output());
  }
}

void LaserMappingNode::publishOdometry() {
  state_.odom_aft_mapped.header.frame_id = "camera_init";
  state_.odom_aft_mapped.child_frame_id = "body";
  if (config_.mapping.publish_odometry_without_downsample) {
    state_.odom_aft_mapped.header.stamp = get_ros_time(
        processor_.time_current_);
  } else {
    state_.odom_aft_mapped.header.stamp = get_ros_time(lidar_end_time_);
  }
  setPosestamp(state_.odom_aft_mapped.pose.pose);

  pub_odom_aft_mapped_->publish(state_.odom_aft_mapped);

  if (config_.publish.tf_enabled) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "camera_init";
    transform.child_frame_id = "aft_mapped";
    transform.transform.translation.x =
        state_.odom_aft_mapped.pose.pose.position.x;
    transform.transform.translation.y =
        state_.odom_aft_mapped.pose.pose.position.y;
    transform.transform.translation.z =
        state_.odom_aft_mapped.pose.pose.position.z;
    transform.transform.rotation.w =
        state_.odom_aft_mapped.pose.pose.orientation.w;
    transform.transform.rotation.x =
        state_.odom_aft_mapped.pose.pose.orientation.x;
    transform.transform.rotation.y =
        state_.odom_aft_mapped.pose.pose.orientation.y;
    transform.transform.rotation.z =
        state_.odom_aft_mapped.pose.pose.orientation.z;
    transform.header.stamp = state_.odom_aft_mapped.header.stamp;
    tf_broadcaster_->sendTransform(transform);
  }
}

void LaserMappingNode::publishPath() {
  setPosestamp(state_.msg_body_pose.pose);

  state_.msg_body_pose.header.stamp = get_ros_time(lidar_end_time_);
  state_.msg_body_pose.header.frame_id = "camera_init";
  state_.path.poses.emplace_back(state_.msg_body_pose);
  pub_path_->publish(state_.path);
}

bool LaserMappingNode::initMapState() {
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
    publishInitMap();
    state_.init_feats_world.reset(new PointCloudXYZI());
    stage_ = PointLioStage::TRACKING;
    return true;
  }
  return false;
}

void LaserMappingNode::preparePointMeasurements() const {
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

void LaserMappingNode::publishFrameOutputs() {
  if (config_.publish.path_enabled) {
    publishPath();
  }
  if (config_.publish.scan_enabled || config_.publish.pcd_save_enabled) {
    publishFrameWorld();
  }
  if (config_.publish.scan_enabled && config_.publish.scan_body_enabled) {
    publishFrameBody();
  }
}

template <bool ImuAsInput, typename KF>
void LaserMappingNode::processFramePoints(KF& kf, double& last_time, auto& q) {
  const auto& imu_last = imu_.last();
  const auto& imu_next = imu_.next();
  if (lio_workspace.time_seq.empty()) {
    if (!imu_.empty()) {
      imu_.advanceCursor();

      while (get_time_sec(imu_next.header.stamp) > processor_.time_current_
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

          last_time = processor_.time_current_;
          time_update_last_ = processor_.time_current_;
          is_first_frame_ = false;
          break;
        }
        processor_.time_current_ = get_time_sec(imu_next.header.stamp);

        if constexpr (ImuAsInput) {
          double dt_cov = processor_.time_current_ - time_update_last_;
          if (dt_cov > 0.0) {
            time_update_last_ = get_time_sec(imu_next.header.stamp);
          }
          last_time = get_time_sec(imu_next.header.stamp);
          lio_workspace.input_in = imu_.nextInput(
              config_.imu.processor.gravity_magnitude / config_.imu.acc_norm);
        } else {
          double dt = processor_.time_current_ - last_time;
          double dt_cov = processor_.time_current_ - time_update_last_;
          if (dt_cov > 0.0) {
            kf.predict(dt_cov, q, lio_workspace.input_in, false, true);
            time_update_last_ = processor_.time_current_;
          }
          kf.predict(dt, q, lio_workspace.input_in, true, false);
          last_time = processor_.time_current_;
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
    processor_.time_current_ = (point_offset_ms / 1000.0) + pcl_beg_time;
    if (is_first_frame_) {
      while (processor_.time_current_ > get_time_sec(imu_next.header.stamp)) {
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
      last_time = processor_.time_current_;
      time_update_last_ = processor_.time_current_;
    }

    if constexpr (ImuAsInput) {
      while (processor_.time_current_ > get_time_sec(imu_next.header.stamp)) {
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
             && processor_.time_current_
                    > get_time_sec(imu_next.header.stamp)) {
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

    double dt = processor_.time_current_ - last_time;
    if (!config_.mapping.propagate_at_imu_frequency) {
      double dt_cov = processor_.time_current_ - time_update_last_;
      if (dt_cov > 0.0) {
        kf.predict(dt_cov, q, lio_workspace.input_in, false, true);
        time_update_last_ = processor_.time_current_;
      }
    }
    kf.predict(dt, q, lio_workspace.input_in, true, false);
    last_time = processor_.time_current_;

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
      publishOdometry();
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

void LaserMappingNode::savePendingPcd() {
  if (state_.pcl_wait_save->empty() || !config_.publish.pcd_save_enabled) {
    return;
  }
  savePcd();
  state_.pcl_wait_save->clear();
  pcd_scan_count_ = 0;
}

void LaserMappingNode::savePcd() {
  auto t = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
  std::tm tm{};
  localtime_r(&t, &tm);
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y_%m_%d-%H_%M_%S");
  std::string str_time = ss.str();

  string file_name = string("scans_" + str_time + ".pcd");
  string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
  pcl::PCDWriter pcd_writer;
  pcd_writer.writeBinary(all_points_dir, *state_.pcl_wait_save);
}
