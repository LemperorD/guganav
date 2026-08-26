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

namespace {
  bool flg_exit = false;  // NOLINT
  std::condition_variable
      sig_buffer;  // NOLINT< 缓冲区条件变量 (通知有数据可用)

  /** @brief Ctrl+C 信号处理: 设置退出标志并通知条件变量 */
  void SigHandle(int sig) {
    flg_exit = true;
    RCLCPP_WARN(rclcpp::get_logger("laserMapping"), "catch sig %d", sig);
    sig_buffer.notify_all();
  }
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserMappingNode>();
  return node->run();
}

/** @brief 节点入口: 调用 initialize() 后进入主循环 (原 main) */
int LaserMappingNode::run() {
  totalInitialize();

  while (rclcpp::ok() && !flg_exit) {
    executor_.spin_some();

    if (!initializeIteration()) {
      continue;
    }

    processFramePoints(kf_input, t_last_, state_.q_input);

    if (!config_.mapping.publish_odometry_without_downsample) {
      publishOdometry();
    }

    // [Workflow 8][Workflow 10] 增量地图更新: 将已更新/未匹配的点加入 iVox
    // 地图。
    if (estimator_state.feats_down_size > 4) {
      if (config_.sensor.enable_prior_map) {
        state_.sleep_time++;
      }
      if (!config_.sensor.enable_prior_map || state_.sleep_time > 200) {
        mapIncremental();
      }
    }

    publishAndLogFrame();

    rate_.sleep();
  }

  if (!state_.pcl_wait_save->empty() && config_.publish.pcd_save_enabled) {
    savePcd();
  }

  fout_out_.close();
  fout_imu_pbp_.close();

  return 0;
}

/** @brief 节点初始化: 参数 / 滤波器 / 日志 / 订阅发布 (原 run 前半段) */
void LaserMappingNode::totalInitialize() {
  // 本类已继承 rclcpp::Node, 直接以自身加入执行器。
  executor_.add_node(this->get_node_base_interface());

  // shared_from_this() 要求对象由 shared_ptr 管理 (main 中以 make_shared
  // 创建)。
  rclcpp::Node::SharedPtr self = this->shared_from_this();
  const PointLioParams params = readParameters(self);
  config_ = params;
  configureEstimatorParams(config_.estimator);

  Lidar::Params lidar_params;
  lidar_params.preprocess = config_.laser_mapping.preprocess;
  lidar_params.imu_enabled = config_.laser_mapping.imu_enabled;
  lidar_params.con_frame = config_.laser_mapping.con_frame;
  lidar_params.con_frame_num = config_.laser_mapping.con_frame_num;
  lidar_params.cut_frame = config_.laser_mapping.cut_frame;
  if (config_.laser_mapping.cut_frame_interval > 0.0) {
    lidar_params.cut_frame_num = std::max(
        1, static_cast<int>(
               std::lround(config_.laser_mapping.lidar_time_interval
                           / config_.laser_mapping.cut_frame_interval)));
  }
  lidar_params.lidar_time_interval = config_.laser_mapping.lidar_time_interval;
  lidar_.configure(lidar_params);

  RCLCPP_INFO(rclcpp::get_logger("laserMapping"), "lidar_type: %d.\n",
              config_.laser_mapping.lidar_type);

  estimator_state.ivox_ = std::make_shared<IVoxType>(
      config_.mapping.ivox_options);

  estimator_state.point_selected_surf.set();
  state_.downsize_filter_surf.setLeafSize(
      static_cast<float>(config_.mapping.filter_size_surf),
      static_cast<float>(config_.mapping.filter_size_surf),
      static_cast<float>(config_.mapping.filter_size_surf));

  state_.downsize_filter_map.setLeafSize(
      static_cast<float>(config_.mapping.filter_size_map),
      static_cast<float>(config_.mapping.filter_size_map),
      static_cast<float>(config_.mapping.filter_size_map));

  state_.path.header.stamp = get_ros_time(lidar_end_time_);
  state_.path.header.frame_id = "camera_init";

  estimator_state.Lidar_T_wrt_IMU = to_vec3d(config_.sensor.extrinsic_t);
  estimator_state.Lidar_R_wrt_IMU = to_mat3d(config_.sensor.extrinsic_r);

  if (config_.mapping.extrinsic_estimation) {
    kf_input.x_.offset_R_L_I = estimator_state.Lidar_R_wrt_IMU;
    kf_input.x_.offset_T_L_I = estimator_state.Lidar_T_wrt_IMU;
  }

  Imu::Params imu_params;
  imu_params.processor.enabled = config_.laser_mapping.imu_enabled;
  imu_params.processor.gravity =
      to_vec3d(config_.laser_mapping.gravity);
  imu_params.processor.gravity_init =
      to_vec3d(config_.laser_mapping.gravity_init);
  imu_params.processor.gravity_magnitude =
      config_.estimator.gravity_magnitude;
  imu_params.integration_interval = config_.laser_mapping.imu_time_interval;
  imu_params.timestamp_offset = config_.sensor.lidar_to_imu_time;
  imu_.configure(imu_params);

  kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
  kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output,
                                       h_model_output, h_model_IMU_output);

  reset_cov(state_.p_init);
  kf_input.change_P(state_.p_init);

  reset_cov_output(state_.p_init_output);
  kf_output.change_P(state_.p_init_output);

  state_.q_input = process_noise_cov_input();
  state_.q_output = process_noise_cov_output();

  state_.pos_log_dir = std::string(ROOT_DIR) + "/Log/pos_log.txt";
  state_.fp.open(state_.pos_log_dir);
  if (!state_.fp.is_open()) {
    RCLCPP_WARN(rclcpp::get_logger("laserMapping"), "无法打开 pos_log 文件: %s",
                state_.pos_log_dir.c_str());
  }
  fout_out_.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
  fout_imu_pbp_.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);
  if (fout_out_ && fout_imu_pbp_) {
    std::cout << "~~~~" << ROOT_DIR << " file opened" << '\n';
  } else {
    std::cout << "~~~~" << ROOT_DIR << " doesn't exist" << '\n';
  }

  if (config_.laser_mapping.lidar_type == AVIA) {
    sub_pcl_livox_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        config_.sensor.lidar_topic, rclcpp::SensorDataQoS(),
        [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
          lidar_.onLivoxPcl(msg);
        });
  } else {
    sub_pcl_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.sensor.lidar_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          lidar_.onStandardPcl(msg);
        });
  }

  pub_laser_cloud_full_res_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_registered", 20);
  pub_laser_cloud_full_res_body_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body",
                                                      20);
  pub_laser_cloud_effect_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_effected", 20);
  pub_laser_cloud_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "Laser_map", 20);
  pub_odom_aft_mapped_ = create_publisher<nav_msgs::msg::Odometry>(
      "aft_mapped_to_init", 20);
  pub_path_ = create_publisher<nav_msgs::msg::Path>("path", 20);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      config_.sensor.imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        imu_.onMessage(msg);
      });

  while (state_.flg_first_scan && rclcpp::ok()) {
    executor_.spin_some();
    if (!lidar_.syncPackages(
            imu_, measures_)) {  // 存在同步的一帧,更新了最后时间(副作用)
      rate_.sleep();
      continue;
    }
    lidar_end_time_ = measures_.lidar_last_time;
    initScan();
  }

  signal(SIGINT, SigHandle);  // NOLINT
}
bool LaserMappingNode::initializeIteration() {
  if (!lidar_.syncPackages(
          imu_, measures_)) {  // 存在同步的一帧,更新了最后时间(副作用)
    rate_.sleep();
    return false;
  }
  lidar_end_time_ = measures_.lidar_last_time;

  // 帧级初始化: 计时归零 / IMU 预处理 / 降采样分组 / 就绪检查 / 量测准备。
  return prepareFrame();
}
/** @brief 帧级初始化 (每轮主循环): 见类声明 */
bool LaserMappingNode::prepareFrame() {
  // IMU 预处理: 在线初始化 / 重力对齐 / 点云拷贝 (去畸变预留;
  // 饱和检查不在本函数, 见 [Workflow 13] IMU 量测更新处)。
  imu_.process(measures_, state_.feats_undistort);

  // ---- 就绪检查: IMU 初始化 / 地图初始化 ----

  if (imu_.needInit()) {
    return false;  // IMU 初始化中
  }

  if (!initMapState()) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Initializing map: %zu/%d points",
        state_.init_feats_world->size(), config_.mapping.init_map_size);

    return false;  // 地图初始化中: 跳过本帧
  }

  // ---- 降采样 + 按时间戳排序 + 分组 ----
  if (config_.mapping.space_down_sample) {
    state_.downsize_filter_surf.setInputCloud(state_.feats_undistort);
    state_.downsize_filter_surf.filter(*estimator_state.feats_down_body);
    sort(estimator_state.feats_down_body->points.begin(),
         estimator_state.feats_down_body->points.end(), time_list);
  } else {
    estimator_state.feats_down_body = measures_.lidar;
    sort(estimator_state.feats_down_body->points.begin(),
         estimator_state.feats_down_body->points.end(), time_list);
  }
  estimator_state.time_seq = time_compressing<int>(
      estimator_state.feats_down_body);
  estimator_state.feats_down_size =
      estimator_state.feats_down_body->points.size();

  // ---- 量测准备 ----
  estimator_state.normvec->resize(estimator_state.feats_down_size);
  estimator_state.feats_down_world->resize(estimator_state.feats_down_size);
  estimator_state.Nearest_Points.resize(estimator_state.feats_down_size);
  estimator_state.crossmat_list.reserve(estimator_state.feats_down_size);
  estimator_state.pbody_list.reserve(estimator_state.feats_down_size);
  // [Workflow 3] 准备点量测: 将 LiDAR 点变换到 IMU 系并缓存反对称矩阵
  // (量测雅可比 A 块用)。点量测噪声为标量 laser_point_cov;
  // 伪代码步骤③的逐点协方差变换 (C_P) 本实现未启用 (cov_p/cov_R 传入未用)。
  preparePointMeasurements();

  return true;
}
/** @brief 节点构造: 初始化 rclcpp::Node 基类 ("laserMapping") */
LaserMappingNode::LaserMappingNode() : rclcpp::Node("laserMapping") {
}

PointCloudXYZI::Ptr LaserMappingNode::loadPointcloudFromPcd(
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

void LaserMappingNode::dumpLioStatetoLog() {
  V3D rot_ang;

  rot_ang = SO3ToEuler(kf_input.x_.rot);

  state_.fp << std::fixed << std::setprecision(6);
  state_.fp << measures_.lidar_start_time - first_lidar_time_ << ' ';
  state_.fp << rot_ang(0) << ' ' << rot_ang(1) << ' ' << rot_ang(2)
            << ' ';  // Angle

  state_.fp << kf_input.x_.pos(0) << ' ' << kf_input.x_.pos(1) << ' '
            << kf_input.x_.pos(2) << ' ';               // Pos
  state_.fp << 0.0 << ' ' << 0.0 << ' ' << 0.0 << ' ';  // omega
  state_.fp << kf_input.x_.vel(0) << ' ' << kf_input.x_.vel(1) << ' '
            << kf_input.x_.vel(2) << ' ';               // Vel
  state_.fp << 0.0 << ' ' << 0.0 << ' ' << 0.0 << ' ';  // Acc
  state_.fp << kf_input.x_.bg(0) << ' ' << kf_input.x_.bg(1) << ' '
            << kf_input.x_.bg(2) << ' ';  // Bias_g
  state_.fp << kf_input.x_.ba(0) << ' ' << kf_input.x_.ba(1) << ' '
            << kf_input.x_.ba(2) << ' ';  // Bias_a
  state_.fp << kf_input.x_.gravity(0) << ' ' << kf_input.x_.gravity(1) << ' '
            << kf_input.x_.gravity(2) << ' ';  // Gravity

  state_.fp << "\r\n";
  state_.fp.flush();
}

void LaserMappingNode::pointBodyLidarToIMU(PointType const* const pi,
                                           PointType* const po) const {
  V3D p_body_lidar(pi->x, pi->y, pi->z);  // NOLINT
  V3D p_body_imu;
  if (config_.mapping.extrinsic_estimation) {
    p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar
                 + kf_input.x_.offset_T_L_I;

  } else {
    p_body_imu = estimator_state.Lidar_R_wrt_IMU * p_body_lidar
                 + estimator_state.Lidar_T_wrt_IMU;
  }
  po->x = (float)p_body_imu(0);   // NOLINT
  po->y = (float)p_body_imu(1);   // NOLINT
  po->z = (float)p_body_imu(2);   // NOLINT
  po->intensity = pi->intensity;  // NOLINT
}

void LaserMappingNode::mapIncremental() const {
  PointVector points_to_add;
  auto cur_pts = estimator_state.feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (int i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType& point_world = estimator_state.feats_down_world->points[i];
    if (!estimator_state.Nearest_Points[i].empty()) {
      const PointVector& points_near = estimator_state.Nearest_Points[i];

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
      // [Workflow 10] 无有效平面对应: 将点直接加入地图。
      points_to_add.emplace_back(point_world);
    }
  }
  estimator_state.ivox_->AddPoints(points_to_add);
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
    pcl::toROSMsg(*estimator_state.feats_down_world, laser_cloud_msg);

    laser_cloud_msg.header.stamp = get_ros_time(lidar_end_time_);
    laser_cloud_msg.header.frame_id = "camera_init";
    pub_laser_cloud_full_res_->publish(laser_cloud_msg);

    //--------------------------save map-----------------------------------
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performances
    if (config_.publish.pcd_save_enabled) {
      *state_.pcl_wait_save += *estimator_state.feats_down_world;

      static int scan_wait_num = 0;
      scan_wait_num++;
      if (!state_.pcl_wait_save->empty()
          && config_.publish.pcd_save_interval > 0
          && scan_wait_num >= config_.publish.pcd_save_interval) {
        pcd_index_++;
        string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_")
                              + to_string(pcd_index_) + string(".pcd"));
        pcl::PCDWriter pcd_writer;
        std::cout << "current scan saved to /PCD/" << all_points_dir << '\n';
        pcd_writer.writeBinary(all_points_dir, *state_.pcl_wait_save);
        state_.pcl_wait_save->clear();
        scan_wait_num = 0;
      }
    }
  }
}

void LaserMappingNode::publishFrameBody() {
  size_t size = state_.feats_undistort->points.size();
  PointCloudXYZI::Ptr lasercloud_imu_body(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
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
  // Static variable, initialized to true, only effective on the first call
  static bool is_first_kf = true;

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

  set_output_from_kf(kf_input);
}

void LaserMappingNode::publishOdometry() {
  state_.odom_aft_mapped.header.frame_id = "camera_init";
  state_.odom_aft_mapped.child_frame_id = "body";
  if (config_.mapping.publish_odometry_without_downsample) {
    state_.odom_aft_mapped.header.stamp = get_ros_time(time_current_);
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
  // state_.msg_body_pose.header.stamp = ros::Time::now();
  state_.msg_body_pose.header.stamp = get_ros_time(lidar_end_time_);
  state_.msg_body_pose.header.frame_id = "camera_init";
  state_.path.poses.emplace_back(state_.msg_body_pose);
  pub_path_->publish(state_.path);
}

/** @brief 初始化地图: 累积世界系点云, 达到 init_map_size 后建图
 * (iVox/先验PCD)
 * @return true  地图已就绪, 本帧可继续正常处理
 *         false 初始化阶段 (本帧用于累积/建图, 调用方应跳过)
 */
bool LaserMappingNode::initMapState() {
  if (state_.init_map) {
    return true;  // 已完成
  }
  estimator_state.feats_down_world->resize(state_.feats_undistort->size());
  for (int i = 0; i < (int)state_.feats_undistort->size(); i++) {
    pointBodyToWorld(&(state_.feats_undistort->points[i]),
                     &(estimator_state.feats_down_world->points[i]));
  }
  for (const auto& point : *estimator_state.feats_down_world) {
    state_.init_feats_world->points.emplace_back(point);
  }

  if (state_.init_feats_world->size()
      >= (size_t)config_.mapping.init_map_size) {
    if (config_.sensor.enable_prior_map) {
      auto map_cloud = loadPointcloudFromPcd(config_.sensor.prior_map_path);
      estimator_state.ivox_->AddPoints(map_cloud->points);
    } else {
      estimator_state.ivox_->AddPoints(state_.init_feats_world->points);
    }
    publishInitMap();
    state_.init_feats_world.reset(new PointCloudXYZI());
    state_.init_map = true;
    return true;
  }
  return false;  // 仍在累积
}

void LaserMappingNode::preparePointMeasurements() const {
  for (size_t i = 0; i < estimator_state.feats_down_body->size(); i++) {
    V3D point_this(estimator_state.feats_down_body->points[i].x,   // NOLINT
                   estimator_state.feats_down_body->points[i].y,   // NOLINT
                   estimator_state.feats_down_body->points[i].z);  // NOLINT

    estimator_state.pbody_list[i] = point_this;
    if (!config_.mapping.extrinsic_estimation) {
      point_this = estimator_state.Lidar_R_wrt_IMU * point_this
                   + estimator_state.Lidar_T_wrt_IMU;
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);
      estimator_state.crossmat_list[i] = point_crossmat;
    }
  }
}

/** @brief 帧尾: 计时收尾 + 发布输出 + 运行时位姿/耗时日志 */
void LaserMappingNode::publishAndLogFrame() {
  if (config_.publish.path_enabled) {
    publishPath();
  }
  if (config_.publish.scan_enabled || config_.publish.pcd_save_enabled) {
    publishFrameWorld();
  }
  if (config_.publish.scan_enabled && config_.publish.scan_body_enabled) {
    publishFrameBody();
  }

  if (config_.publish.runtime_log_enabled) {
    if (!config_.mapping.publish_odometry_without_downsample) {
      state_.euler_cur = SO3ToEuler(kf_input.x_.rot);
      fout_out_ << setw(20) << measures_.lidar_start_time - first_lidar_time_
                << " " << state_.euler_cur.transpose() << " "
                << kf_input.x_.pos.transpose() << " "
                << kf_input.x_.vel.transpose() << " "
                << kf_input.x_.bg.transpose() << " "
                << kf_input.x_.ba.transpose() << " "
                << kf_input.x_.gravity.transpose() << " "
                << state_.feats_undistort->points.size() << '\n';
    }
    dumpLioStatetoLog();
  }
}

/** @brief 帧内点处理 (2×2: 行=IMU 模式, 列=有无 LiDAR 点)
 * @tparam ImuAsInput true = kf_input (24维, IMU-as-input) / false =
 * kf_output (30维)
 * @param kf        对应滤波器 (kf_input / kf_output)
 * @param last_time 传播时间基准 (t_last_ / time_predict_last_const_)
 * @param q         过程噪声 (state_.q_input / state_.q_output)
 */
template <typename KF>
void LaserMappingNode::processFramePoints(KF& kf, double& last_time, auto& q) {
  const auto& imu_last = imu_.last();
  const auto& imu_next = imu_.next();
  estimator_state.effct_feat_num = 0;

  if (estimator_state.time_seq.empty()) {
    // [Workflow 12] 否则如果 IMU 测量: 当前时间段没有 LiDAR 点, 仅处理 IMU。
    if (!imu_.empty()) {
      imu_.advanceCursor();

      while (get_time_sec(imu_next.header.stamp) > time_current_
             && (get_time_sec(imu_next.header.stamp)
                 < measures_.lidar_start_time
                       + config_.laser_mapping.lidar_time_interval)) {
        if (is_first_frame_) {
          while (get_time_sec(imu_next.header.stamp)
                 < measures_.lidar_start_time
                       + config_.laser_mapping.lidar_time_interval) {
            imu_.popAndAdvance();
            if (imu_.empty()) {
              break;
            }
          }

          estimator_state.input_in = imu_.lastInput(
              config_.estimator.gravity_magnitude / config_.estimator.acc_norm);

          last_time = time_current_;
          time_update_last_ = time_current_;
          is_first_frame_ = false;
          break;
        }
        time_current_ = get_time_sec(imu_next.header.stamp);

        // 原 B2: 仅推进 IMU 指针/填充输入 (无传播)
        double dt_cov = time_current_ - time_update_last_;
        if (dt_cov > 0.0) {
          time_update_last_ = get_time_sec(imu_next.header.stamp);
        }
        last_time = get_time_sec(imu_next.header.stamp);
        estimator_state.input_in = imu_.nextInput(
            config_.estimator.gravity_magnitude / config_.estimator.acc_norm);

        imu_.popAndAdvance();
        if (imu_.empty()) {
          break;
        }
      }
    }
    return;
  }

  // [Workflow 2] LiDAR 点输入分支: 逐时间组处理 (传播 → 平面更新 →
  // 世界变换)。
  double pcl_beg_time = measures_.lidar_start_time;
  estimator_state.idx = -1;
  for (estimator_state.k = 0;
       estimator_state.k < (int)estimator_state.time_seq.size();
       estimator_state.k++) {
    PointType& point_body =
        estimator_state.feats_down_body
            ->points[estimator_state.idx
                     + estimator_state.time_seq[estimator_state.k]];
    const double point_offset_ms = point_time_offset_ms(point_body);
    time_current_ = (point_offset_ms / 1000.0) + pcl_beg_time;
    if (is_first_frame_) {
      while (time_current_ > get_time_sec(imu_next.header.stamp)) {
        imu_.popAndAdvance();
        if (imu_.empty()) {
          break;
        }
      }
      estimator_state.input_in = imu_.lastInput(
          config_.estimator.gravity_magnitude / config_.estimator.acc_norm);

      is_first_frame_ = false;
      last_time = time_current_;
      time_update_last_ = time_current_;
    }

    // [Workflow 1] 将状态传播到当前 LiDAR 点时间。

    while (time_current_ > get_time_sec(imu_next.header.stamp)) {
      imu_.popBuffer();
      estimator_state.input_in = imu_.lastInput(
          config_.estimator.gravity_magnitude / config_.estimator.acc_norm);
      double dt = get_time_sec(imu_last.header.stamp) - last_time;
      double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last_;

      if (dt_cov > 0.0) {
        kf.predict(dt_cov, q, estimator_state.input_in, false, true);
        time_update_last_ = get_time_sec(imu_last.header.stamp);
      }

      kf.predict(dt, q, estimator_state.input_in, true, false);
      last_time = get_time_sec(imu_last.header.stamp);

      if (imu_.empty()) {
        break;
      }
      imu_.advanceCursor();
    }

    if (state_.flg_reset) {
      break;
    }

    double dt = time_current_ - last_time;
    if (!config_.mapping.propagate_at_imu_frequency) {
      double dt_cov = time_current_ - time_update_last_;
      if (dt_cov > 0.0) {
        kf.predict(dt_cov, q, estimator_state.input_in, false, true);
        time_update_last_ = time_current_;
      }
    }
    kf.predict(dt, q, estimator_state.input_in, true, false);
    last_time = time_current_;

    if (estimator_state.feats_down_size < 1) {
      RCLCPP_WARN(rclcpp::get_logger("laserMapping"),
                  "No point, skip this scan!\n");
      estimator_state.idx += estimator_state.time_seq[estimator_state.k];
      continue;
    }
    // [Workflow 4] 判断是否存在有效平面对应。
    // [Workflow 5] 残差、雅可比和量测协方差在 EKF 更新函数内计算。
    // [Workflow 6] 状态更新在 EKF 更新函数内完成。
    // [Workflow 7] 协方差更新在 EKF 更新函数内完成。
    if (!kf.update_iterated_dyn_share_modified()) {
      // [Workflow 9] 无有效平面对应时跳过当前点更新。
      estimator_state.idx = estimator_state.idx
                            + estimator_state.time_seq[estimator_state.k];
      continue;
    }
    if (config_.mapping.publish_odometry_without_downsample) {
      publishOdometry();
      if (config_.publish.runtime_log_enabled) {
        state_.euler_cur = SO3ToEuler(kf.x_.rot);

        fout_out_ << setw(20) << measures_.lidar_start_time - first_lidar_time_
                  << " " << state_.euler_cur.transpose() << " "
                  << kf.x_.pos.transpose() << " " << kf.x_.vel.transpose()
                  << " " << kf.x_.bg.transpose() << " " << kf.x_.ba.transpose()
                  << " " << kf.x_.gravity.transpose() << " "
                  << state_.feats_undistort->points.size() << '\n';
      }
    }

    // [Workflow 8] 将更新后的点变换到世界系。
    for (int j = 0; j < estimator_state.time_seq[estimator_state.k]; j++) {
      PointType& point_body_j =
          estimator_state.feats_down_body->points[estimator_state.idx + j + 1];
      PointType& point_world_j =
          estimator_state.feats_down_world->points[estimator_state.idx + j + 1];
      pointBodyToWorld(&point_body_j, &point_world_j);
    }

    estimator_state.idx += estimator_state.time_seq[estimator_state.k];
  }
}

void LaserMappingNode::initScan() {
  const auto& imu_next = imu_.next();
  first_lidar_time_ = measures_.lidar_start_time;
  state_.flg_first_scan = false;
  std::cout << "first imu time: " << get_time_sec(imu_next.header.stamp)
            << '\n';
  time_current_ = 0.0;

  if (config_.laser_mapping.imu_enabled) {
    kf_input.x_.gravity = to_vec3d(config_.laser_mapping.gravity);
    kf_output.x_.gravity = to_vec3d(config_.laser_mapping.gravity);
    imu_.discardBefore(measures_.lidar_start_time);  // 去除起始时间之前的帧
  } else {
    kf_input.x_.gravity = to_vec3d(config_.laser_mapping.gravity);
    kf_output.x_.gravity = to_vec3d(config_.laser_mapping.gravity);
    kf_output.x_.acc = to_vec3d(config_.laser_mapping.gravity);
    kf_output.x_.acc *= -1;
    imu_.setNeedInit(false);
  }
  if (config_.laser_mapping.gravity.size() >= 3) {  // TODO 重力不是四维向量
    config_.estimator.gravity_magnitude = std::hypot(
        config_.laser_mapping.gravity[0], config_.laser_mapping.gravity[1],
        config_.laser_mapping.gravity[2]);
    configureEstimatorParams(config_.estimator);
  }
}

void LaserMappingNode::savePcd() {
  auto t = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
  std::tm tm{};  // 栈上自己的 tm, 不碰静态缓冲
  localtime_r(&t, &tm);
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y_%m_%d-%H_%M_%S");
  std::string str_time = ss.str();

  string file_name = string("scans_" + str_time + ".pcd");
  string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
  pcl::PCDWriter pcd_writer;
  pcd_writer.writeBinary(all_points_dir, *state_.pcl_wait_save);
}
