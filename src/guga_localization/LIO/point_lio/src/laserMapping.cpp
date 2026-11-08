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

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

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
  // PCD 保存也依赖 odom 系 Z 滤波, 因此 scan 未发布时也可能需要 TF。
  const bool filter_pcd = config_.publish.pcd_save_enabled
                          && config_.output_frame.z_filter.enabled;
  if (config_.output_frame.enabled || filter_pcd) {
    if (!config_.output_frame.body_to_base_from_params
        && !config_.output_frame.extrinsic_from_params) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
  }
  if (config_.output_frame.enabled) {
    pub_registered_scan_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.output_frame.registered_scan_topic, 5);
    pub_lidar_odometry_ = create_publisher<nav_msgs::msg::Odometry>(
        config_.output_frame.lidar_odometry_topic, 5);
  }

  if (!config_.publish.pcd_save_state_topic.empty()) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    sub_gicp_state_ = create_subscription<std_msgs::msg::UInt8>(
        config_.publish.pcd_save_state_topic, rclcpp::QoS(1),
        [this](const std_msgs::msg::UInt8::SharedPtr msg) {
          gicpStateCallback(msg);
        },
        options);
    RCLCPP_INFO(get_logger(), "PCD 保存按定位状态驱动: %s",
                config_.publish.pcd_save_state_topic.c_str());
  } else if (config_.publish.pcd_save_require_lost) {
    if (config_.publish.pcd_save_trigger_topic.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "pcd_save.require_lost 为 true 但 trigger_topic 为空, "
                   "PCD 将不会保存");
    } else {
      rclcpp::SubscriptionOptions options;
      options.callback_group = callback_group_;
      sub_pcd_save_trigger_ = create_subscription<std_msgs::msg::Bool>(
          config_.publish.pcd_save_trigger_topic, rclcpp::QoS(1),
          [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data && !pcd_save_armed_) {
              pcd_save_armed_ = true;
              RCLCPP_WARN(get_logger(),
                          "收到 GICP LOST 信号, 开始累积并保存 PCD");
            }
          },
          options);
      RCLCPP_INFO(get_logger(), "PCD 保存等待 LOST 触发: %s",
                  config_.publish.pcd_save_trigger_topic.c_str());
    }
  }

  if (!config_.publish.pcd_save_reload_service.empty()) {
    reload_client_ = create_client<std_srvs::srv::Trigger>(
        config_.publish.pcd_save_reload_service);
    RCLCPP_INFO(get_logger(), "先验地图重载服务: %s",
                config_.publish.pcd_save_reload_service.c_str());
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
  if (config_.publish.scan_enabled || isPcdSaveActive()) {
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
  if (out.body_to_base_from_params) {
    odom_rotation_ << out.body_to_base_r[0], out.body_to_base_r[1],
        out.body_to_base_r[2], out.body_to_base_r[3], out.body_to_base_r[4],
        out.body_to_base_r[5], out.body_to_base_r[6], out.body_to_base_r[7],
        out.body_to_base_r[8];
    odom_translation_ << out.body_to_base_t[0], out.body_to_base_t[1],
        out.body_to_base_t[2];
    odom_extrinsic_ready_ = true;
    RCLCPP_INFO(get_logger(), "odom 输出使用显式参数 %s <- body",
                out.base_frame.c_str());
    return true;
  }

  Eigen::Matrix3d base_lidar_rotation;
  Eigen::Vector3d base_lidar_translation;
  if (out.extrinsic_from_params) {
    base_lidar_rotation << out.lidar_to_base_r[0], out.lidar_to_base_r[1],
        out.lidar_to_base_r[2], out.lidar_to_base_r[3], out.lidar_to_base_r[4],
        out.lidar_to_base_r[5], out.lidar_to_base_r[6], out.lidar_to_base_r[7],
        out.lidar_to_base_r[8];
    base_lidar_translation << out.lidar_to_base_t[0], out.lidar_to_base_t[1],
        out.lidar_to_base_t[2];
  } else {
    if (!tf_buffer_) {
      return false;
    }
    try {
      // 静态变换, 用 TimePointZero 取最新可用值, 不依赖雷达时间戳
      const auto tf_stamped = tf_buffer_->lookupTransform(
          out.base_frame, out.lidar_frame, tf2::TimePointZero);
      const auto& q = tf_stamped.transform.rotation;
      const Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
      base_lidar_rotation = quat.toRotationMatrix();
      const auto& t = tf_stamped.transform.translation;
      base_lidar_translation << t.x, t.y, t.z;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "查不到安装变换 %s <- %s: %s, 暂不发布 %s/%s 或保存 PCD",
          out.base_frame.c_str(), out.lidar_frame.c_str(), ex.what(),
          out.registered_scan_topic.c_str(), out.lidar_odometry_topic.c_str());
      return false;
    }
  }

  const Eigen::Matrix3d lidar_imu_rotation =
      to_mat3d(config_.sensor.extrinsic_r);
  const Eigen::Vector3d lidar_imu_translation =
      to_vec3d(config_.sensor.extrinsic_t);
  const Eigen::Matrix3d imu_lidar_rotation =
      lidar_imu_rotation.transpose();
  const Eigen::Vector3d imu_lidar_translation =
      -imu_lidar_rotation * lidar_imu_translation;
  odom_rotation_ = base_lidar_rotation * imu_lidar_rotation;
  odom_translation_ = base_lidar_rotation * imu_lidar_translation
                      + base_lidar_translation;
  odom_extrinsic_ready_ = true;
  RCLCPP_INFO(get_logger(),
              "odom 输出使用 %s <- body（由 base<-lidar 与 lidar->imu 合成）",
              out.base_frame.c_str());
  return true;
}

bool LaserMappingNode::pointPassesOdomZFilter(
    const Eigen::Vector3d& point_world) const {
  const auto& z_filter = config_.output_frame.z_filter;
  if (!z_filter.enabled) {
    return true;
  }
  const double z_odom = (odom_rotation_ * point_world + odom_translation_).z();
  return z_odom >= z_filter.min_z && z_odom <= z_filter.max_z;
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
      if (!pointPassesOdomZFilter(pw)) {
        continue;
      }
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
  }

  // PCD 保存与 scan 发布相互独立: 导航模式可能关闭 scan_publish_en 以省 CPU,
  // 但仍需要累积点云。
  // 维护 SUSPECT/LOST 累计计时, FINE 帧也要执行, 用于结算上一段可疑时长。
  maybeUpdateMapOnSuspect();

  if (!isPcdSaveActive()) {
    return;
  }

  const auto& z_filter = config_.output_frame.z_filter;
  if (z_filter.enabled) {
    if (!resolveOdomExtrinsic()) {
      return;
    }
    for (const auto& point : lidar_.workspace().feats_down_world->points) {
      const Eigen::Vector3d point_world(point.x, point.y, point.z);
      if (pointPassesOdomZFilter(point_world)) {
        state_.pcl_wait_save->push_back(point);
      }
    }
  } else {
    *state_.pcl_wait_save += *lidar_.workspace().feats_down_world;
  }

  pcd_scan_count_++;
  if (!state_.pcl_wait_save->empty() && config_.publish.pcd_save_interval > 0
      && pcd_scan_count_ >= config_.publish.pcd_save_interval) {
    pcd_index_++;
    const std::string path = pcdOutputPath(pcd_index_);
    if (writePcd(path, *state_.pcl_wait_save)) {
      std::cout << "current scan saved to " << path << '\n';
    }
    state_.pcl_wait_save->clear();
    pcd_scan_count_ = 0;
  }

  // LOST 由回调置位, 在主循环里写盘, 避免跨线程操作点云。
  if (pcd_save_requested_.exchange(false)) {
    savePcdOnLost();
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
  if (state_.pcl_wait_save->empty() || !isPcdSaveActive()) {
    return;
  }
  savePcd();
  state_.pcl_wait_save->clear();
  pcd_scan_count_ = 0;
}
bool LaserMappingNode::isPcdSaveActive() const {
  if (!config_.publish.pcd_save_enabled) {
    return false;
  }
  if (!config_.publish.pcd_save_state_topic.empty()) {
    // 2=SUSPECT 3=LOST: 怀疑丢失后开始累积, 真正丢失时才写盘
    return gicp_state_ == 2 || gicp_state_ == 3;
  }
  return !config_.publish.pcd_save_require_lost || pcd_save_armed_;
}

void LaserMappingNode::gicpStateCallback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  gicp_state_ = msg->data;
  switch (msg->data) {
    case 1:  // FINE: 暂停累积, 不清空缓存, 下次 SUSPECT 继续累加
      if (pcd_save_armed_) {
        pcd_save_armed_ = false;
        RCLCPP_INFO(get_logger(), "定位恢复 FINE, 暂停点云累积");
      }
      break;
    case 2:  // SUSPECT: 开始累积
      if (!pcd_save_armed_) {
        pcd_save_armed_ = true;
        RCLCPP_WARN(get_logger(), "定位进入 SUSPECT, 开始累积点云");
      }
      break;
    case 3:  // LOST: 首次立即覆盖写盘, 之后交给累计计时做增量
      if (!first_rebuild_done_ && !pcd_save_on_lost_done_) {
        pcd_save_on_lost_done_ = true;
        pcd_save_requested_ = true;
        RCLCPP_WARN(get_logger(), "定位进入 LOST, 首次触发覆盖写盘");
      }
      break;
    default:
      break;
  }
}

void LaserMappingNode::backupPriorMap() {
  const std::string path = pcdOutputPath(0);
  std::error_code ec;
  const std::filesystem::path target(path);
  if (!std::filesystem::exists(target, ec)) {
    return;
  }
  const auto stamp = std::to_string(get_clock()->now().nanoseconds());
  const auto backup = target.parent_path() /
      (target.stem().string() + "_" + stamp + ".bak.pcd");
  std::filesystem::copy_file(
      target, backup, std::filesystem::copy_options::overwrite_existing, ec);
  if (ec) {
    RCLCPP_WARN(get_logger(), "备份先验地图失败: %s", ec.message().c_str());
  } else {
    RCLCPP_INFO(get_logger(), "先验地图已备份到 %s", backup.c_str());
  }
}

bool LaserMappingNode::mergeMapAndWrite() {
  // 缓冲里的点云同属本次 odom 系, 直接写快照即可, 不读旧 PCD:
  // 旧地图可能来自上一次会话, 原点与本次 camera_init 不同, 合并会错层。
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.setInputCloud(state_.pcl_wait_save);
  PointCloudXYZI::Ptr filtered(new PointCloudXYZI());
  voxel.filter(*filtered);
  if (filtered->empty()) {
    return false;
  }

  const std::string path = pcdOutputPath(0);
  backupPriorMap();
  return writePcd(path, *filtered);
}

void LaserMappingNode::maybeUpdateMapOnSuspect() {
  const auto now = get_clock()->now();
  const uint8_t state = gicp_state_.load();
  const bool suspect_now = (state == 2 || state == 3);

  if (suspect_now && !suspect_tracking_) {
    suspect_tracking_ = true;
    suspect_mark_ = now;
  } else if (!suspect_now && suspect_tracking_) {
    suspect_accum_sec_ += (now - suspect_mark_).seconds();
    suspect_tracking_ = false;
  }
  if (!suspect_now) {
    // 离开可疑状态, 允许下一段 SUSPECT 再更新一次
    updated_this_suspect_ = false;
  }

  // 首次丢失由 pcd_save_requested_ 走覆盖写盘, 增量只用于之后的累计触发
  if (!first_rebuild_done_ || updated_this_suspect_) {
    return;
  }

  const double delay = config_.publish.pcd_save_suspect_build_delay_sec;
  const double total =
      suspect_accum_sec_ +
      (suspect_tracking_ ? (now - suspect_mark_).seconds() : 0.0);
  if (delay <= 0.0 || total < delay || state_.pcl_wait_save->empty()) {
    return;
  }

  const bool ok = mergeMapAndWrite();
  if (ok) {
    RCLCPP_INFO(get_logger(), "可疑状态累计 %.1f s, 已更新地图 (%zu 点)",
                total, state_.pcl_wait_save->size());
  } else {
    RCLCPP_ERROR(get_logger(), "更新地图失败, 保留旧文件");
  }
  updated_this_suspect_ = true;
  suspect_accum_sec_ = 0.0;
  suspect_mark_ = now;
  if (ok) {
    requestPriorMapReload();
  }
}

void LaserMappingNode::savePcdOnLost() {
  if (state_.pcl_wait_save->empty()) {
    RCLCPP_WARN(get_logger(), "定位丢失但累积点云为空, 跳过 PCD 保存");
    return;
  }

  const std::string path = pcdOutputPath(0);
  backupPriorMap();
  if (writePcd(path, *state_.pcl_wait_save)) {
    RCLCPP_INFO(get_logger(), "定位丢失, 累积点云已写入 %s (%zu 点)",
                path.c_str(), state_.pcl_wait_save->size());
  } else {
    RCLCPP_ERROR(get_logger(), "定位丢失, 写入 %s 失败", path.c_str());
  }
  first_rebuild_done_ = true;
  suspect_accum_sec_ = 0.0;
  suspect_mark_ = get_clock()->now();
  // 不清空缓冲: 后续快照要包含首次写盘之前的点云, 否则每次更新都会丢掉
  // 更早的地图。缓冲最长按 12 min 运行时长估算, 内存可接受。
  pcd_scan_count_ = 0;
  requestPriorMapReload();
}

void LaserMappingNode::requestPriorMapReload() {
  if (!reload_client_) {
    RCLCPP_WARN(get_logger(), "未配置重载服务, 先验地图保持旧文件");
    return;
  }
  if (!reload_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "重载服务不可用, 先验地图保持旧文件");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reload_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(get_logger(), "先验地图重载成功: %s",
                      response->message.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "先验地图重载失败: %s",
                       response->message.c_str());
        }
      });
}
std::string LaserMappingNode::pcdOutputPath(int index) const {
  const std::string& configured = config_.publish.pcd_save_path;
  if (!configured.empty()) {
    if (index <= 0) {
      return configured;
    }
    const std::filesystem::path path(configured);
    return (path.parent_path()
            / (path.stem().string() + "_" + std::to_string(index)
               + path.extension().string()))
        .string();
  }

  if (index > 0) {
    return std::string(ROOT_DIR) + "PCD/scans_" + std::to_string(index)
           + ".pcd";
  }

  auto t = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
  std::tm tm{};
  localtime_r(&t, &tm);
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y_%m_%d-%H_%M_%S");
  return std::string(ROOT_DIR) + "PCD/scans_" + ss.str() + ".pcd";
}
bool LaserMappingNode::writePcd(const std::string& path,
                                const PointCloudXYZI& cloud) {
  // 先解析软链接: install 目录中的 PCD 通常是指向源码的符号链接,
  // 直接 rename 会把符号链接本身替换掉。
  std::error_code ec;
  std::filesystem::path target(path);
  const auto resolved = std::filesystem::weakly_canonical(target, ec);
  if (!ec) {
    target = resolved;
  } else {
    ec.clear();
  }

  if (target.has_parent_path()) {
    std::filesystem::create_directories(target.parent_path(), ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "无法创建 PCD 目录 %s: %s",
                   target.parent_path().c_str(), ec.message().c_str());
      return false;
    }
  }

  // 先写临时文件再原子替换, 避免中途失败留下半个 PCD, 也能覆盖只读的
  // 先验地图文件 (目录可写即可)。
  const std::string tmp_path = target.string() + ".tmp";
  pcl::PCDWriter writer;
  if (writer.writeBinary(tmp_path, cloud) != 0) {
    RCLCPP_ERROR(get_logger(), "写入 PCD 临时文件失败: %s", tmp_path.c_str());
    return false;
  }
  std::filesystem::rename(tmp_path, target, ec);
  if (ec) {
    RCLCPP_ERROR(get_logger(), "替换 PCD 文件 %s 失败: %s", target.c_str(),
                 ec.message().c_str());
    std::error_code remove_ec;
    std::filesystem::remove(tmp_path, remove_ec);
    return false;
  }
  return true;
}
void LaserMappingNode::savePcd() {
  const std::string path = pcdOutputPath(0);
  if (writePcd(path, *state_.pcl_wait_save)) {
    RCLCPP_INFO(get_logger(), "PCD 已保存到 %s", path.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "保存 PCD 失败: %s", path.c_str());
  }
}
