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
 * - **IMU-as-input** (use_imu_as_input=true): IMU 驱动预测, 激光做量测更新
 * - **IMU-as-output** (use_imu_as_input=false, default): IMU 也作为量测,
 *   角速度和加速度本身被估计, 每帧同时做激光量测和 IMU 量测更新
 */

#include "laserMapping.h"

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
  initialize();

  while (rclcpp::ok() && !flg_exit) {
    executor_.spin_some();

    if (!lidar_.syncPackages(imu_, Measures)) {
      rate_.sleep();
      continue;
    }

    if (state_.flg_reset) {  // 需要reset
      resetSystem();
    }

    if (state_.flg_first_scan) {  // (reset后)首次扫描
      initScan();
    }

    // 帧级初始化: 计时归零 / IMU 预处理 / 降采样分组 / 就绪检查 / 量测准备。
    double t0 = omp_get_wtime();
    bool frame_ready = prepareFrame();
    double t1 = omp_get_wtime();

    if (!frame_ready) {
      continue;  // IMU 初始化中或地图未就绪: 跳过本帧
    }

    // 按 use_imu_as_input 选择滤波器 (input: IMU 驱动预测 / output: IMU
    // 亦作量测)。 LiDAR/IMU 分支在 processFramePoints 内部 ([Workflow 2] /
    // [Workflow 12])。
    if (use_imu_as_input) {
      processFramePoints<true>(kf_input, t_last, state_.q_input);
    } else {
      processFramePoints<false>(kf_output, time_predict_last_const,
                                state_.q_output);
    }

    // [Workflow 18] 输出阶段 (算法末尾): 发布更新后的里程计 x_{k+1} /
    // P_{k+1}。
    if (!publish_odometry_without_downsample) {
      publishOdometry();
    }

    double t2 = omp_get_wtime();
    // [Workflow 8][Workflow 10] 增量地图更新: 将已更新/未匹配的点加入 iVox
    // 地图。
    if (feats_down_size > 4) {
      if (enable_prior_pcd) {
        state_.sleep_time++;
      }
      if (!enable_prior_pcd || state_.sleep_time > 200) {
        mapIncremental();
      }
    }

    publishAndLogFrame(t0, t1, t2);

    rate_.sleep();
  }

  if (!state_.pcl_wait_save->empty() && pcd_save_en) {
    savePcd();
  }

  fout_out.close();
  fout_imu_pbp.close();

  return 0;
}

/** @brief 节点构造: 初始化 rclcpp::Node 基类 ("laserMapping") */
LaserMappingNode::LaserMappingNode() : rclcpp::Node("laserMapping") {
}

/** @brief 节点初始化: 参数 / 滤波器 / 日志 / 订阅发布 (原 run 前半段) */
void LaserMappingNode::initialize() {
  // 本类已继承 rclcpp::Node, 直接以自身加入执行器。
  executor_.add_node(this->get_node_base_interface());

  // shared_from_this() 要求对象由 shared_ptr 管理 (main 中以 make_shared
  // 创建)。
  rclcpp::Node::SharedPtr self = this->shared_from_this();
  readParameters(self);

  RCLCPP_INFO(rclcpp::get_logger("laserMapping"), "lidar_type: %d.\n",
              lidar_type);

  ivox_ = std::make_shared<IVoxType>(ivox_options_);

  point_selected_surf.set();
  state_.downsize_filter_surf.setLeafSize(
      static_cast<float>(filter_size_surf_min),
      static_cast<float>(filter_size_surf_min),
      static_cast<float>(filter_size_surf_min));

  state_.downsize_filter_map.setLeafSize(
      static_cast<float>(filter_size_map_min),
      static_cast<float>(filter_size_map_min),
      static_cast<float>(filter_size_map_min));

  state_.path.header.stamp = get_ros_time(lidar_end_time);
  state_.path.header.frame_id = "camera_init";

  Lidar_T_wrt_IMU = to_vec3d(extrinT);
  Lidar_R_wrt_IMU = to_mat3d(extrinR);

  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    } else {
      kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    }
  }

  p_pre->lidar_type = lidar_type;
  imu_.configure(lidar_type, imu_enabled, gravity);

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
  open_file();

  if (p_pre->lidar_type == AVIA) {
    sub_pcl_livox_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lid_topic, rclcpp::SensorDataQoS(),
        [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
          lidar_.onLivoxPcl(msg);
        });
  } else {
    sub_pcl_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, rclcpp::SensorDataQoS(),
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
      imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        imu_.onMessage(msg);
      });

  signal(SIGINT, SigHandle);  // NOLINT
}

/** @brief 帧级初始化 (每轮主循环): 见类声明 */
bool LaserMappingNode::prepareFrame() {
  // ---- 耗时统计归零 (本帧内由 processFramePoints 累加) ----
  state_.solve_time = 0;
  state_.propag_time = 0;
  state_.update_time = 0;

  // IMU 预处理: 在线初始化 / 重力对齐 / 点云拷贝 (去畸变预留;
  // 饱和检查不在本函数, 见 [Workflow 13] IMU 量测更新处)。
  imu_.process(Measures, state_.feats_undistort);

  // ---- 降采样 + 按时间戳排序 + 分组 ----
  if (space_down_sample) {
    state_.downsize_filter_surf.setInputCloud(state_.feats_undistort);
    state_.downsize_filter_surf.filter(*feats_down_body);
    sort(feats_down_body->points.begin(), feats_down_body->points.end(),
         time_list);
  } else {
    feats_down_body = Measures.lidar;
    sort(feats_down_body->points.begin(), feats_down_body->points.end(),
         time_list);
  }
  time_seq = time_compressing<int>(feats_down_body);
  feats_down_size = feats_down_body->points.size();

  // ---- 就绪检查: IMU 初始化 / 地图初始化 ----
  if (imu_.needInit()) {
    return false;  // IMU 初始化中
  }
  if (!initMapState()) {
    return false;  // 地图初始化中: 跳过本帧
  }

  // ---- 量测准备 ----
  normvec->resize(feats_down_size);
  feats_down_world->resize(feats_down_size);
  Nearest_Points.resize(feats_down_size);
  crossmat_list.reserve(feats_down_size);
  pbody_list.reserve(feats_down_size);
  // [Workflow 3] 准备点量测: 将 LiDAR 点变换到 IMU 系并缓存反对称矩阵
  // (量测雅可比 A 块用)。点量测噪声为标量 laser_point_cov;
  // 伪代码步骤③的逐点协方差变换 (C_P) 本实现未启用 (cov_p/cov_R 传入未用)。
  preparePointMeasurements();

  return true;
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
  if (!use_imu_as_input) {
    rot_ang = SO3ToEuler(kf_output.x_.rot);
  } else {
    rot_ang = SO3ToEuler(kf_input.x_.rot);
  }

  state_.fp << std::fixed << std::setprecision(6);
  state_.fp << Measures.lidar_beg_time - first_lidar_time << ' ';
  state_.fp << rot_ang(0) << ' ' << rot_ang(1) << ' ' << rot_ang(2)
            << ' ';  // Angle
  if (use_imu_as_input) {
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
  } else {
    state_.fp << kf_output.x_.pos(0) << ' ' << kf_output.x_.pos(1) << ' '
              << kf_output.x_.pos(2) << ' ';              // Pos
    state_.fp << 0.0 << ' ' << 0.0 << ' ' << 0.0 << ' ';  // omega
    state_.fp << kf_output.x_.vel(0) << ' ' << kf_output.x_.vel(1) << ' '
              << kf_output.x_.vel(2) << ' ';              // Vel
    state_.fp << 0.0 << ' ' << 0.0 << ' ' << 0.0 << ' ';  // Acc
    state_.fp << kf_output.x_.bg(0) << ' ' << kf_output.x_.bg(1) << ' '
              << kf_output.x_.bg(2) << ' ';  // Bias_g
    state_.fp << kf_output.x_.ba(0) << ' ' << kf_output.x_.ba(1) << ' '
              << kf_output.x_.ba(2) << ' ';  // Bias_a
    state_.fp << kf_output.x_.gravity(0) << ' ' << kf_output.x_.gravity(1)
              << ' ' << kf_output.x_.gravity(2) << ' ';  // Gravity
  }
  state_.fp << "\r\n";
  state_.fp.flush();
}

void LaserMappingNode::pointBodyLidarToIMU(PointType const* const pi,
                                           PointType* const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);  // NOLINT
  V3D p_body_imu;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar
                   + kf_output.x_.offset_T_L_I;
    } else {
      p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar
                   + kf_input.x_.offset_T_L_I;
    }
  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  }
  po->x = (float)p_body_imu(0);   // NOLINT
  po->y = (float)p_body_imu(1);   // NOLINT
  po->z = (float)p_body_imu(2);   // NOLINT
  po->intensity = pi->intensity;  // NOLINT
}

void LaserMappingNode::mapIncremental() {
  PointVector points_to_add;
  auto cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (int i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType& point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) {
      const PointVector& points_near = Nearest_Points[i];

      Eigen::Vector3f center =
          ((point_world.getVector3fMap() / filter_size_map_min).array().floor()
           + 0.5)
          * filter_size_map_min;
      bool need_add = true;
      for (const auto x : points_near) {
        Eigen::Vector3f dis_2_center = x.getVector3fMap() - center;
        if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min
            && fabs(dis_2_center.y()) < 0.5 * filter_size_map_min
            && fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
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
  ivox_->AddPoints(points_to_add);
}

void LaserMappingNode::publishInitMap() {
  sensor_msgs::msg::PointCloud2 laser_cloudmsg;

  pcl::toROSMsg(*state_.init_feats_world, laser_cloudmsg);

  laser_cloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laser_cloudmsg.header.frame_id = "camera_init";
  pub_laser_cloud_map_->publish(laser_cloudmsg);
}

void LaserMappingNode::publishFrameWorld() {
  if (scan_pub_en) {
    sensor_msgs::msg::PointCloud2 laser_cloud_msg;
    pcl::toROSMsg(*feats_down_world, laser_cloud_msg);

    laser_cloud_msg.header.stamp = get_ros_time(lidar_end_time);
    laser_cloud_msg.header.frame_id = "camera_init";
    pub_laser_cloud_full_res_->publish(laser_cloud_msg);

    //--------------------------save map-----------------------------------
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performances
    if (pcd_save_en) {
      *state_.pcl_wait_save += *feats_down_world;

      static int scan_wait_num = 0;
      scan_wait_num++;
      if (!state_.pcl_wait_save->empty() && pcd_save_interval > 0
          && scan_wait_num >= pcd_save_interval) {
        pcd_index++;
        string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_")
                              + to_string(pcd_index) + string(".pcd"));
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
  laser_cloud_msg.header.stamp = get_ros_time(lidar_end_time);
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

  if (!use_imu_as_input) {
    if (enable_prior_pcd && is_first_kf) {
      // Execute only on the first call
      kf_output.x_.pos(0) = init_pose[0];
      kf_output.x_.pos(1) = init_pose[1];
      kf_output.x_.pos(2) = init_pose[2];
      set_output_from_kf(kf_output);
      is_first_kf = false;  // Set is_first_kf to false after the first call
    } else {
      set_output_from_kf(kf_output);
    }
  } else {
    set_output_from_kf(kf_input);
  }
}

void LaserMappingNode::publishOdometry() {
  state_.odom_aft_mapped.header.frame_id = "camera_init";
  state_.odom_aft_mapped.child_frame_id = "body";
  if (publish_odometry_without_downsample) {
    state_.odom_aft_mapped.header.stamp = get_ros_time(time_current);
  } else {
    state_.odom_aft_mapped.header.stamp = get_ros_time(lidar_end_time);
  }
  setPosestamp(state_.odom_aft_mapped.pose.pose);

  pub_odom_aft_mapped_->publish(state_.odom_aft_mapped);

  if (tf_send_en) {
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
  state_.msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
  state_.msg_body_pose.header.frame_id = "camera_init";
  state_.path.poses.emplace_back(state_.msg_body_pose);
  pub_path_->publish(state_.path);
}

/** @brief 系统复位: 重置滤波器/里程计状态/地图 (bag 回放等场景) */
void LaserMappingNode::resetSystem() {
  RCLCPP_WARN(rclcpp::get_logger("laserMapping"),
              "reset when rosbag play back");
  imu_.reset();
  lidar_.reset();
  state_.feats_undistort = std::make_shared<PointCloudXYZI>();
  if (use_imu_as_input) {
    state_in = state_input();
    kf_input.change_P(state_.p_init);
  } else {
    state_out = state_output();
    kf_output.change_P(state_.p_init_output);
  }
  state_.flg_first_scan = true;
  is_first_frame = true;
  state_.flg_reset = false;
  state_.init_map = false;
  ivox_ = std::make_shared<IVoxType>(ivox_options_);
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
  feats_down_world->resize(state_.feats_undistort->size());
  for (int i = 0; i < (int)state_.feats_undistort->size(); i++) {
    pointBodyToWorld(&(state_.feats_undistort->points[i]),
                     &(feats_down_world->points[i]));
  }
  for (const auto& point : *feats_down_world) {
    state_.init_feats_world->points.emplace_back(point);
  }

  if (state_.init_feats_world->size() >= (size_t)init_map_size) {
    if (enable_prior_pcd) {
      auto map_cloud = loadPointcloudFromPcd(prior_pcd_map_path);
      ivox_->AddPoints(map_cloud->points);
    } else {
      ivox_->AddPoints(state_.init_feats_world->points);
    }
    publishInitMap();
    state_.init_feats_world.reset(new PointCloudXYZI());
    state_.init_map = true;
    return true;
  }
  return false;  // 仍在累积
}

void LaserMappingNode::preparePointMeasurements() {
  for (size_t i = 0; i < feats_down_body->size(); i++) {
    V3D point_this(feats_down_body->points[i].x,   // NOLINT
                   feats_down_body->points[i].y,   // NOLINT
                   feats_down_body->points[i].z);  // NOLINT

    pbody_list[i] = point_this;
    if (!extrinsic_est_en) {
      point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);
      crossmat_list[i] = point_crossmat;
    }
  }
}

/** @brief 帧尾: 计时收尾 + 发布输出 + 运行时位姿/耗时日志 */
void LaserMappingNode::publishAndLogFrame(double t0, double t1, double t2) {
  double t3 = omp_get_wtime();
  if (path_en) {
    publishPath();
  }
  if (scan_pub_en || pcd_save_en) {
    publishFrameWorld();
  }
  if (scan_pub_en && scan_body_pub_en) {
    publishFrameBody();
  }

  if (runtime_pos_log) {
    state_.frame_num++;
    state_.aver_time_consu = (state_.aver_time_consu * (state_.frame_num - 1)
                              / state_.frame_num)
                             + ((t3 - t0) / state_.frame_num);
    state_.aver_time_icp = (state_.aver_time_icp * (state_.frame_num - 1)
                            / state_.frame_num)
                           + (state_.update_time / state_.frame_num);
    state_.aver_time_solve = (state_.aver_time_solve * (state_.frame_num - 1)
                              / state_.frame_num)
                             + (state_.solve_time / state_.frame_num);
    state_.aver_time_propag = (state_.aver_time_propag * (state_.frame_num - 1)
                               / state_.frame_num)
                              + (state_.propag_time / state_.frame_num);
    lidar_.T1[state_.time_log_counter] = Measures.lidar_beg_time;
    lidar_.s_plot[state_.time_log_counter] = t3 - t0;
    lidar_.s_plot2[state_.time_log_counter] =
        (double)state_.feats_undistort->points.size();
    lidar_.s_plot3[state_.time_log_counter] = state_.aver_time_consu;
    state_.time_log_counter++;

    std::cout << std::fixed << std::setprecision(6)
              << "[ mapping ]: time: IMU + Map + Input Downsample: " << t1 - t0
              << " ave solve: " << state_.aver_time_solve
              << "  ave ICP: " << t2 - t1 << "  map incre: " << t3 - t2
              << " ave total: " << state_.aver_time_consu
              << " icp: " << state_.aver_time_icp
              << " propogate: " << state_.aver_time_propag << '\n';

    if (!publish_odometry_without_downsample) {
      if (!use_imu_as_input) {
        state_.euler_cur = SO3ToEuler(kf_output.x_.rot);
        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time
                 << " " << state_.euler_cur.transpose() << " "
                 << kf_output.x_.pos.transpose() << " "
                 << kf_output.x_.vel.transpose() << " "
                 << kf_output.x_.omg.transpose() << " "
                 << kf_output.x_.acc.transpose() << " "
                 << kf_output.x_.gravity.transpose() << " "
                 << kf_output.x_.bg.transpose() << " "
                 << kf_output.x_.ba.transpose() << " "
                 << state_.feats_undistort->points.size() << '\n';
      } else {
        state_.euler_cur = SO3ToEuler(kf_input.x_.rot);
        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time
                 << " " << state_.euler_cur.transpose() << " "
                 << kf_input.x_.pos.transpose() << " "
                 << kf_input.x_.vel.transpose() << " "
                 << kf_input.x_.bg.transpose() << " "
                 << kf_input.x_.ba.transpose() << " "
                 << kf_input.x_.gravity.transpose() << " "
                 << state_.feats_undistort->points.size() << '\n';
      }
    }
    dumpLioStatetoLog();
  }
}

/** @brief 帧内点处理 (2×2: 行=IMU 模式, 列=有无 LiDAR 点)
 * @tparam ImuAsInput true = kf_input (24维, IMU-as-input) / false =
 * kf_output (30维)
 * @param kf        对应滤波器 (kf_input / kf_output)
 * @param last_time 传播时间基准 (t_last / time_predict_last_const)
 * @param q         过程噪声 (state_.q_input / state_.q_output)
 */
template <bool ImuAsInput, typename KF>
void LaserMappingNode::processFramePoints(KF& kf, double& last_time, auto& q) {
  auto& imu_deque = imu_.buffer();
  auto& imu_last = imu_.lastMutable();
  auto& imu_next = imu_.nextMutable();
  effct_feat_num = 0;
  if (time_seq.empty()) {
    // [Workflow 12] 否则如果 IMU 测量: 当前时间段没有 LiDAR 点, 仅处理 IMU。
    if (!imu_deque.empty()) {
      imu_.advanceCursor();

      while (get_time_sec(imu_next.header.stamp) > time_current
             && (get_time_sec(imu_next.header.stamp)
                 < Measures.lidar_beg_time + lidar_time_inte)) {
        if (is_first_frame) {
          while (get_time_sec(imu_next.header.stamp)
                 < Measures.lidar_beg_time + lidar_time_inte) {
            imu_.popAndAdvance();
            if (imu_deque.empty()) {
              break;
            }
          }
          if constexpr (ImuAsInput) {
            input_in.gyro << imu_last.angular_velocity.x,
                imu_last.angular_velocity.y, imu_last.angular_velocity.z;
            input_in.acc << imu_last.linear_acceleration.x,
                imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
          } else {
            angvel_avr << imu_last.angular_velocity.x,
                imu_last.angular_velocity.y, imu_last.angular_velocity.z;
            acc_avr << imu_last.linear_acceleration.x,
                imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
          }
          last_time = time_current;
          time_update_last = time_current;
          is_first_frame = false;
          break;
        }
        time_current = get_time_sec(imu_next.header.stamp);

        if constexpr (ImuAsInput) {
          // 原 B2: 仅推进 IMU 指针/填充输入 (无传播)
          double dt_cov = time_current - time_update_last;
          if (dt_cov > 0.0) {
            time_update_last = get_time_sec(imu_next.header.stamp);
          }
          last_time = get_time_sec(imu_next.header.stamp);
          input_in.gyro << imu_next.angular_velocity.x,
              imu_next.angular_velocity.y, imu_next.angular_velocity.z;
          input_in.acc << imu_next.linear_acceleration.x,
              imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;
          input_in.acc = input_in.acc * G_m_s2 / acc_norm;
        } else {
          // [Workflow 1] 状态传播 (9)(10): 以下 kf.predict
          // 分别完成协方差传播与状态预测。 原 A2: 传播 + IMU 更新
          double dt = time_current - last_time;
          {
            double dt_cov = time_current - time_update_last;
            if (dt_cov > 0.0) {
              kf.predict(dt_cov, q, input_in, false, true);
              time_update_last = time_current;
            }
            kf.predict(dt, q, input_in, true, false);
          }
          last_time = time_current;
          angvel_avr << imu_next.angular_velocity.x,
              imu_next.angular_velocity.y, imu_next.angular_velocity.z;
          acc_avr << imu_next.linear_acceleration.x,
              imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;
          // [Workflow 13]-[Workflow 17] IMU 量测更新 (output 模式):
          //   [Workflow 13] 无饱和度检查 (satu_check) 在 h_model_IMU_output
          //   内完成; [Workflow 14] 残差/雅可比/量测噪声 (14)(15) 在
          //   h_model_IMU_output 内计算; [Workflow 15] 状态更新 (20)(21) 在
          //   EKF 更新函数内完成; [Workflow 16] 协方差更新 (23)(24) 在 EKF
          //   更新函数内完成; [Workflow 17] 否则 (饱和):
          //   饱和轴被排除在状态/协方差更新之外。
          kf.update_iterated_dyn_share_IMU();
        }
        imu_.popAndAdvance();
        if (imu_deque.empty()) {
          break;
        }
      }
    }
    return;
  }

  // [Workflow 2] LiDAR 点输入分支: 逐时间组处理 (传播 → 平面更新 →
  // 世界变换)。
  double pcl_beg_time = Measures.lidar_beg_time;
  idx = -1;
  for (k = 0; k < (int)time_seq.size(); k++) {
    PointType& point_body = feats_down_body->points[idx + time_seq[k]];
    time_current = (point_body.curvature / 1000.0)  // NOLINT
                   + pcl_beg_time;
    if (is_first_frame) {
      if constexpr (ImuAsInput) {
        while (time_current > get_time_sec(imu_next.header.stamp)) {
          imu_.popAndAdvance();
          if (imu_deque.empty()) {
            break;
          }
        }
        input_in.gyro << imu_last.angular_velocity.x,
            imu_last.angular_velocity.y, imu_last.angular_velocity.z;
        input_in.acc << imu_last.linear_acceleration.x,
            imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
      } else {
        if (imu_enabled) {
          while (time_current > get_time_sec(imu_next.header.stamp)) {
            imu_.popAndAdvance();
            if (imu_deque.empty()) {
              break;
            }
          }
          angvel_avr << imu_last.angular_velocity.x,
              imu_last.angular_velocity.y, imu_last.angular_velocity.z;
          acc_avr << imu_last.linear_acceleration.x,
              imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
        }
      }
      is_first_frame = false;
      last_time = time_current;
      time_update_last = time_current;
    }

    // [Workflow 1] 将状态传播到当前 LiDAR 点时间。
    if constexpr (ImuAsInput) {
      while (time_current > get_time_sec(imu_next.header.stamp)) {
        imu_deque.pop_front();
        input_in.gyro << imu_last.angular_velocity.x,
            imu_last.angular_velocity.y, imu_last.angular_velocity.z;
        input_in.acc << imu_last.linear_acceleration.x,
            imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
        double dt = get_time_sec(imu_last.header.stamp) - last_time;

        double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
        if (dt_cov > 0.0) {
          kf.predict(dt_cov, q, input_in, false, true);
          time_update_last = get_time_sec(imu_last.header.stamp);
        }
        kf.predict(dt, q, input_in, true, false);
        last_time = get_time_sec(imu_last.header.stamp);

        if (imu_deque.empty()) {
          break;
        }
        imu_.advanceCursor();
      }
    } else {
      if (imu_enabled && !imu_deque.empty()) {
        bool last_imu = get_time_sec(imu_next.header.stamp)
                        == get_time_sec(imu_deque.front()->header.stamp);
        while (get_time_sec(imu_next.header.stamp) < last_time
               && !imu_deque.empty()) {
          if (!last_imu) {
            imu_.advanceCursor();
            break;
          }

          imu_.popAndAdvance();
          if (imu_deque.empty()) {
            break;
          }
        }
        bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
        while (imu_comes) {
          angvel_avr << imu_next.angular_velocity.x,
              imu_next.angular_velocity.y, imu_next.angular_velocity.z;
          acc_avr << imu_next.linear_acceleration.x,
              imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

          double dt = get_time_sec(imu_next.header.stamp) - last_time;
          kf.predict(dt, q, input_in, true, false);
          last_time = get_time_sec(imu_next.header.stamp);

          {
            double dt_cov = get_time_sec(imu_next.header.stamp)
                            - time_update_last;

            if (dt_cov > 0.0) {
              time_update_last = get_time_sec(imu_next.header.stamp);
              double propag_imu_start = omp_get_wtime();

              kf.predict(dt_cov, q, input_in, false, true);

              state_.propag_time += omp_get_wtime() - propag_imu_start;
              double solve_imu_start = omp_get_wtime();
              // [Workflow 13]-[Workflow 17] IMU 量测更新 (同列2):
              //   饱和检查与残差 (13)(14) 在 h_model_IMU_output 内完成;
              //   状态/协方差更新 (15)(16) 在 EKF 更新函数内完成;
              //   饱和轴被排除在更新之外 (17)。
              kf.update_iterated_dyn_share_IMU();
              state_.solve_time += omp_get_wtime() - solve_imu_start;
            }
          }
          imu_.popAndAdvance();
          if (imu_deque.empty()) {
            break;
          }
          imu_comes = time_current > get_time_sec(imu_next.header.stamp);
        }
      }
    }
    if (state_.flg_reset) {
      break;
    }

    double dt = time_current - last_time;
    double propag_state_start = omp_get_wtime();
    if (!prop_at_freq_of_imu) {
      double dt_cov = time_current - time_update_last;
      if (dt_cov > 0.0) {
        kf.predict(dt_cov, q, input_in, false, true);
        time_update_last = time_current;
      }
    }
    kf.predict(dt, q, input_in, true, false);
    state_.propag_time += omp_get_wtime() - propag_state_start;
    last_time = time_current;
    double t_update_start = omp_get_wtime();

    if (feats_down_size < 1) {
      RCLCPP_WARN(rclcpp::get_logger("laserMapping"),
                  "No point, skip this scan!\n");
      idx += time_seq[k];
      continue;
    }
    // [Workflow 4] 判断是否存在有效平面对应。
    // [Workflow 5] 残差、雅可比和量测协方差在 EKF 更新函数内计算。
    // [Workflow 6] 状态更新在 EKF 更新函数内完成。
    // [Workflow 7] 协方差更新在 EKF 更新函数内完成。
    if (!kf.update_iterated_dyn_share_modified()) {
      // [Workflow 9] 无有效平面对应时跳过当前点更新。
      idx = idx + time_seq[k];
      continue;
    }
    double solve_start = omp_get_wtime();

    if (publish_odometry_without_downsample) {
      publishOdometry();
      if (runtime_pos_log) {
        state_.euler_cur = SO3ToEuler(kf.x_.rot);
        if constexpr (ImuAsInput) {
          fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time
                   << " " << state_.euler_cur.transpose() << " "
                   << kf.x_.pos.transpose() << " " << kf.x_.vel.transpose()
                   << " " << kf.x_.bg.transpose() << " " << kf.x_.ba.transpose()
                   << " " << kf.x_.gravity.transpose() << " "
                   << state_.feats_undistort->points.size() << '\n';
        } else {
          fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time
                   << " " << state_.euler_cur.transpose() << " "
                   << kf.x_.pos.transpose() << " " << kf.x_.vel.transpose()
                   << " " << kf.x_.omg.transpose() << " "
                   << kf.x_.acc.transpose() << " " << kf.x_.gravity.transpose()
                   << " " << kf.x_.bg.transpose() << " " << kf.x_.ba.transpose()
                   << " " << state_.feats_undistort->points.size() << '\n';
        }
      }
    }

    // [Workflow 8] 将更新后的点变换到世界系。
    for (int j = 0; j < time_seq[k]; j++) {
      PointType& point_body_j = feats_down_body->points[idx + j + 1];
      PointType& point_world_j = feats_down_world->points[idx + j + 1];
      pointBodyToWorld(&point_body_j, &point_world_j);
    }

    state_.solve_time += omp_get_wtime() - solve_start;

    state_.update_time += omp_get_wtime() - t_update_start;
    idx += time_seq[k];
  }
}

void LaserMappingNode::initScan() {
  auto& imu_next = imu_.nextMutable();
  first_lidar_time = Measures.lidar_beg_time;
  state_.flg_first_scan = false;
  std::cout << "first imu time: " << get_time_sec(imu_next.header.stamp)
            << '\n';
  time_current = 0.0;

  if (imu_enabled) {
    kf_input.x_.gravity = to_vec3d(gravity);
    kf_output.x_.gravity = to_vec3d(gravity);
    imu_.discardBefore(Measures.lidar_beg_time);
  } else {
    kf_input.x_.gravity = to_vec3d(gravity);
    kf_output.x_.gravity = to_vec3d(gravity);
    kf_output.x_.acc = to_vec3d(gravity);
    kf_output.x_.acc *= -1;
    imu_.setNeedInit(false);
  }
  G_m_s2 = std::sqrt((gravity[0] * gravity[0]) + (gravity[1] * gravity[1])
                     + (gravity[2] * gravity[2]));
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
