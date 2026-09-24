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
#include "point_lio/core/FrameProcessor.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserMappingNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

LaserMappingNode::LaserMappingNode()
    : rclcpp::Node("laserMapping"),
      processor_(imu_, stage_, lidar_, config_, state_) {
  callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  config_ = readParameters(this);
  initializeSensors();
  initializeMappingState();
  processor_.initialize();
  initializeRos2Interfaces();
  processing_timer_ = create_wall_timer(
      std::chrono::milliseconds(2), [this]() { processIteration(); },
      callback_group_);
}
void LaserMappingNode::initializeSensors() {
  lidar_.configure(config_.lidar);
  processor_.configureSynchronizer(config_.lidar.lidar_time_interval);

  auto imu_params = config_.imu;
  imu_params.timestamp_offset = config_.sensor.lidar_to_imu_time;
  imu_.configure(imu_params);

  RCLCPP_INFO(get_logger(), "lidar_type: %d.", config_.lidar.lidar_type);
}
void LaserMappingNode::initializeMappingState() {
  lidar_.workspace().ivox_ = std::make_shared<IVoxType>(
      config_.mapping.ivox_options);
  lidar_.workspace().point_selected_surf.set();

  lidar_.setExtrinsics(to_vec3d(config_.sensor.extrinsic_t),
                       to_mat3d(config_.sensor.extrinsic_r));

  state_.downsize_filter_surf.setLeafSize(
      static_cast<float>(config_.mapping.filter_size_surf),
      static_cast<float>(config_.mapping.filter_size_surf),
      static_cast<float>(config_.mapping.filter_size_surf));

  state_.path.header.stamp = get_ros_time(processor_.lidarEndTime());
  state_.path.header.frame_id = "camera_init";
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

  // 项目约定输出 (原 loam_interface 的职责): 同一个 odom 系下的点云与里程计
  if (config_.output_frame.enabled) {
    pub_registered_scan_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.output_frame.registered_scan_topic, 5);
    pub_lidar_odometry_ = create_publisher<nav_msgs::msg::Odometry>(
        config_.output_frame.lidar_odometry_topic, 5);
    if (!config_.output_frame.extrinsic_from_params) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
  }
  createSensorSubscriptions();
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
void LaserMappingNode::processIteration() {
  // 定时器以 500 Hz 轮询, 但只有真正处理了新帧才发布输出。
  // 否则没有新帧时也会反复重发同一幅点云与同一条 path:
  // 既占满 CPU (每 2 ms 转换一次全分辨率点云),
  // 又让 path 消息不断累积重复位姿。
  const bool frame_processed = processor_.processIteration(
      [this](const sensor_msgs::msg::PointCloud2& msg) {
        if (pub_laser_cloud_map_) {
          pub_laser_cloud_map_->publish(msg);
        }
      },
      [this](const nav_msgs::msg::Odometry& msg) {
        if (pub_odom_aft_mapped_) {
          pub_odom_aft_mapped_->publish(msg);
        }
        // 缓存一份, 换到 odom 系后由项目约定输出发布
        last_odometry_ = msg;
        has_last_odometry_ = true;
      },
      [this](const geometry_msgs::msg::TransformStamped& msg) {
        if (tf_broadcaster_) {
          tf_broadcaster_->sendTransform(msg);
        }
      });
  if (!frame_processed) {
    return;
  }
  publishFrameOutputs();
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
  publishProjectOutputs();
}

bool LaserMappingNode::resolveOdomExtrinsic() {
  if (odom_extrinsic_ready_) {
    return true;
  }
  const auto& out = config_.output_frame;
  if (out.extrinsic_from_params) {
    odom_rotation_ << out.lidar_to_base_r[0], out.lidar_to_base_r[1],
        out.lidar_to_base_r[2], out.lidar_to_base_r[3], out.lidar_to_base_r[4],
        out.lidar_to_base_r[5], out.lidar_to_base_r[6], out.lidar_to_base_r[7],
        out.lidar_to_base_r[8];
    odom_translation_ << out.lidar_to_base_t[0], out.lidar_to_base_t[1],
        out.lidar_to_base_t[2];
    odom_extrinsic_ready_ = true;
    RCLCPP_INFO(get_logger(), "odom 输出使用的安装变换来自参数 %s <- %s",
                out.base_frame.c_str(), out.lidar_frame.c_str());
    return true;
  }
  if (!tf_buffer_) {
    return false;
  }
  try {
    // 静态变换, 用 TimePointZero 取最新可用值, 不依赖雷达时间戳
    const auto tf_stamped = tf_buffer_->lookupTransform(
        out.base_frame, out.lidar_frame, tf2::TimePointZero);
    const auto& q = tf_stamped.transform.rotation;
    const Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    odom_rotation_ = quat.toRotationMatrix();
    const auto& t = tf_stamped.transform.translation;
    odom_translation_ << t.x, t.y, t.z;
    odom_extrinsic_ready_ = true;
    RCLCPP_INFO(get_logger(), "已从 TF 取得安装变换 %s <- %s",
                out.base_frame.c_str(), out.lidar_frame.c_str());
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "查不到安装变换 %s <- %s: %s, 暂不发布 %s/%s",
                         out.base_frame.c_str(), out.lidar_frame.c_str(),
                         ex.what(), out.registered_scan_topic.c_str(),
                         out.lidar_odometry_topic.c_str());
    return false;
  }
  return true;
}

void LaserMappingNode::publishProjectOutputs() {
  if (!pub_registered_scan_ && !pub_lidar_odometry_) {
    return;
  }
  if (!resolveOdomExtrinsic()) {
    return;
  }
  const auto& out = config_.output_frame;
  const rclcpp::Time stamp = get_ros_time(processor_.lidarEndTime());

  if (pub_registered_scan_ && config_.publish.scan_enabled) {
    // 与 loam_interface 一致: 整幅世界系点云左乘安装变换后标 odom
    PointCloudXYZI::Ptr cloud_odom(new PointCloudXYZI);
    cloud_odom->reserve(lidar_.workspace().feats_down_world->size());
    for (const auto& p : lidar_.workspace().feats_down_world->points) {
      const Eigen::Vector3d pw(p.x, p.y, p.z);
      const Eigen::Vector3d po = odom_rotation_ * pw + odom_translation_;
      PointType q = p;
      q.x = static_cast<float>(po.x());
      q.y = static_cast<float>(po.y());
      q.z = static_cast<float>(po.z());
      cloud_odom->points.emplace_back(q);
    }
    cloud_odom->width = cloud_odom->points.size();
    cloud_odom->height = 1;
    cloud_odom->is_dense = false;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud_odom, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = out.odom_frame;
    pub_registered_scan_->publish(msg);
  }

  if (pub_lidar_odometry_ && has_last_odometry_) {
    nav_msgs::msg::Odometry msg = last_odometry_;
    const auto& pose = last_odometry_.pose.pose;
    const Eigen::Vector3d p_lio(pose.position.x, pose.position.y,
                                pose.position.z);
    const Eigen::Quaterniond q_lio(pose.orientation.w, pose.orientation.x,
                                   pose.orientation.y, pose.orientation.z);
    const Eigen::Vector3d p_odom = odom_rotation_ * p_lio + odom_translation_;
    const Eigen::Quaterniond q_odom(odom_rotation_ * q_lio.toRotationMatrix());
    msg.pose.pose.position.x = p_odom.x();
    msg.pose.pose.position.y = p_odom.y();
    msg.pose.pose.position.z = p_odom.z();
    msg.pose.pose.orientation.x = q_odom.x();
    msg.pose.pose.orientation.y = q_odom.y();
    msg.pose.pose.orientation.z = q_odom.z();
    msg.pose.pose.orientation.w = q_odom.w();
    msg.header.frame_id = out.odom_frame;
    msg.child_frame_id = out.lidar_frame;
    pub_lidar_odometry_->publish(msg);
  }
}
void LaserMappingNode::publishPath() {
  setPosestamp(state_.msg_body_pose.pose);

  state_.msg_body_pose.header.stamp = get_ros_time(processor_.lidarEndTime());
  state_.msg_body_pose.header.frame_id = "camera_init";
  state_.path.poses.emplace_back(state_.msg_body_pose);
  pub_path_->publish(state_.path);
}
template <typename T>
void LaserMappingNode::setPosestamp(T& out) {
  processor_.setPose(out);
}
void LaserMappingNode::publishFrameWorld() {
  if (config_.publish.scan_enabled) {
    sensor_msgs::msg::PointCloud2 laser_cloud_msg;
    pcl::toROSMsg(*lidar_.workspace().feats_down_world, laser_cloud_msg);

    laser_cloud_msg.header.stamp = get_ros_time(processor_.lidarEndTime());
    laser_cloud_msg.header.frame_id = "camera_init";
    pub_laser_cloud_full_res_->publish(laser_cloud_msg);

    if (config_.publish.pcd_save_enabled) {
      *state_.pcl_wait_save += *lidar_.workspace().feats_down_world;

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
    processor_.pointBodyLidarToIMU(&state_.feats_undistort->points[i],
                                   &lasercloud_imu_body->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laser_cloud_msg;
  pcl::toROSMsg(*lasercloud_imu_body, laser_cloud_msg);
  laser_cloud_msg.header.stamp = get_ros_time(processor_.lidarEndTime());
  laser_cloud_msg.header.frame_id = "body";
  pub_laser_cloud_full_res_body_->publish(laser_cloud_msg);
}

LaserMappingNode::~LaserMappingNode() {
  try {
    savePendingPcd();
  } catch (...) {
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
