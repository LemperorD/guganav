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

#include "point_lio/ros2/laserMapping.h"
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
    : rclcpp::Node("laserMapping"), processor_(stage_, data_handler_, state_) {
  callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);  // TODO:记得考虑回调组方法
  initializeParameters();
  frame_publisher_.configure(this, config_.publish);
  processor_.initialize();

  RCLCPP_INFO(get_logger(), "lidar_type: %d.", config_.lidar.lidar_type);
  initializeRos2Interfaces();

  processing_timer_ = create_wall_timer(
      std::chrono::milliseconds(2), [this]() { processIteration(); },
      callback_group_);
}

void LaserMappingNode::initializeParameters() {
  try {
    config_.mapping.propagate_at_imu_frequency = this->declare_parameter<bool>(
        "prop_at_freq_of_imu", true);
    config_.imu.check_saturation = this->declare_parameter<bool>("check_satu",
                                                                 true);
    config_.mapping.init_map_size = static_cast<int>(
        this->declare_parameter<long>("init_map_size", 100));
    config_.mapping.space_down_sample = this->declare_parameter<bool>(
        "space_down_sample", true);

    config_.imu.saturation_acc = this->declare_parameter<double>(
        "mapping.satu_acc", 3.0);
    config_.imu.saturation_gyro = this->declare_parameter<double>(
        "mapping.satu_gyro", 35.0);
    config_.imu.acc_norm = this->declare_parameter<double>("mapping.acc_norm",
                                                           1.0);

    config_.lidar.plane_threshold = static_cast<float>(
        this->declare_parameter<double>("mapping.plane_thr", 0.05F));
    config_.lidar.preprocess.point_filter_num = static_cast<int>(
        this->declare_parameter<long>("point_filter_num", 2));

    config_.sensor.lidar_topic = this->declare_parameter<std::string>(
        "common.lid_topic", ".livox.lidar");
    config_.sensor.imu_topic = this->declare_parameter<std::string>(
        "common.imu_topic", ".livox.imu");

    config_.lidar.frame_mode = static_cast<int>(
        this->declare_parameter<long>("common.frame_mode", 0));
    if (config_.lidar.frame_mode < 0 || config_.lidar.frame_mode > 2) {
      RCLCPP_WARN(get_logger(),
                  "common.frame_mode 取值无效: %d, 已回退为 0 (单帧直通)",
                  config_.lidar.frame_mode);
      config_.lidar.frame_mode = 0;
    }
    config_.lidar.con_frame_num = static_cast<int>(
        this->declare_parameter<long>("common.con_frame_num", 1));
    config_.lidar.cut_frame_interval = this->declare_parameter<double>(
        "common.cut_frame_time_interval", 0.1);
    config_.sensor.lidar_to_imu_time = this->declare_parameter<double>(
        "common.time_diff_lidar_to_imu", 0.0);

    config_.sensor.enable_prior_map = this->declare_parameter<bool>(
        "prior_pcd.enable", false);
    config_.sensor.prior_map_path = this->declare_parameter<string>(
        "prior_pcd.prior_pcd_map_path", "");
    config_.sensor.initial_pose = this->declare_parameter<std::vector<double>>(
        "prior_pcd.init_pose", std::vector<double>());

    config_.mapping.filter_size_surf = this->declare_parameter<double>(
        "filter_size_surf", 0.5);
    config_.mapping.filter_size_map = this->declare_parameter<double>(
        "filter_size_map", 0.5);
    config_.mapping.det_range = static_cast<float>(
        this->declare_parameter<double>("mapping.det_range", 300.F));
    config_.lidar.preprocess.det_range = config_.mapping.det_range;


    config_.lidar.point_covariance = this->declare_parameter<double>(
        "mapping.lidar_meas_cov", 0.1);
    config_.filter.acc_cov_input = this->declare_parameter<double>(
        "mapping.acc_cov_input", 0.1);
    config_.filter.vel_cov = this->declare_parameter<double>("mapping.vel_cov",
                                                             20);
    config_.filter.gyr_cov_input = this->declare_parameter<double>(
        "mapping.gyr_cov_input", 0.1);
    config_.filter.gyr_cov_output = this->declare_parameter<double>(
        "mapping.gyr_cov_output", 0.1);
    config_.filter.acc_cov_output = this->declare_parameter<double>(
        "mapping.acc_cov_output", 0.1);
    config_.filter.b_gyr_cov = this->declare_parameter<double>(
        "mapping.b_gyr_cov", 0.0001);
    config_.filter.b_acc_cov = this->declare_parameter<double>(
        "mapping.b_acc_cov", 0.0001);
    config_.imu.measurement_acc_cov = this->declare_parameter<double>(
        "mapping.imu_meas_acc_cov", 0.1);
    config_.imu.measurement_gyro_cov = this->declare_parameter<double>(
        "mapping.imu_meas_omg_cov", 0.1);

    config_.lidar.preprocess.blind = this->declare_parameter<double>(
        "preprocess.blind", 1.0);
    config_.lidar.lidar_type = static_cast<int>(
        this->declare_parameter<long>("preprocess.lidar_type", 1));
    config_.lidar.preprocess.lidar_type = config_.lidar.lidar_type;
    config_.lidar.preprocess.scan_lines = static_cast<int>(
        this->declare_parameter<long>("preprocess.scan_line", 16));
    config_.lidar.preprocess.scan_rate = static_cast<int>(
        this->declare_parameter<long>("preprocess.scan_rate", 10));
    config_.lidar.preprocess.timestamp_unit = static_cast<int>(
        this->declare_parameter<long>("preprocess.timestamp_unit", 1));
    config_.lidar.match_threshold = this->declare_parameter<double>(
        "mapping.match_s", 81);

    const auto gravity = this->declare_parameter<std::vector<double>>(
        "mapping.gravity", std::vector<double>());
    const auto gravity_init = this->declare_parameter<std::vector<double>>(
        "mapping.gravity_init", std::vector<double>());
    if (gravity.size() >= 3) {
      config_.imu.processor.gravity = V3D(gravity[0], gravity[1], gravity[2]);
    }
    if (gravity_init.size() >= 3) {
      config_.imu.processor.gravity_init = V3D(gravity_init[0], gravity_init[1],
                                               gravity_init[2]);
    }

    config_.sensor.extrinsic_t = this->declare_parameter<std::vector<double>>(
        "mapping.extrinsic_T", std::vector<double>());
    config_.sensor.extrinsic_r = this->declare_parameter<std::vector<double>>(
        "mapping.extrinsic_R", std::vector<double>());

    config_.mapping.publish_odometry_without_downsample =
        this->declare_parameter<bool>(
            "odometry.publish_odometry_without_downsample", false);
    config_.publish.scan_enabled = this->declare_parameter<bool>(
        "publish.scan_publish_en", true);
    config_.publish.debug_state_enabled = this->declare_parameter<bool>(
        "publish.debug_state_en", false);

    config_.publish.pcd_save_enabled = this->declare_parameter<bool>(
        "pcd_save.pcd_save_en", false);
    config_.publish.pcd_save_interval = static_cast<int>(
        this->declare_parameter<long>("pcd_save.interval", -1));
    config_.publish.pcd_save_path = this->declare_parameter<std::string>(
        "pcd_save.path", "");
    config_.publish.pcd_save_require_lost = this->declare_parameter<bool>(
        "pcd_save.require_lost", false);
    config_.publish.pcd_save_trigger_topic =
        this->declare_parameter<std::string>("pcd_save.trigger_topic", "");
    config_.publish.pcd_save_state_topic = this->declare_parameter<std::string>(
        "pcd_save.state_topic", "");
    config_.publish.pcd_save_reload_service =
        this->declare_parameter<std::string>("pcd_save.reload_service", "");
    config_.publish.pcd_save_suspect_build_delay_sec =
        this->declare_parameter<double>("pcd_save.suspect_build_delay_sec",
                                        3.0);
    config_.lidar.lidar_time_interval = this->declare_parameter<double>(
        "mapping.lidar_time_inte", 0.1);

    // 项目约定输出: 把点云与位姿换算到与底盘对齐的 odom 系后另行发布。
    // 这是原先 loam_interface 的职责。安装变换留空时在启动阶段从 TF 查询
    // base_frame <- lidar_frame, 与 loam_interface 的行为一致。
    config_.output_frame.odom_frame = this->declare_parameter<std::string>(
        "output_frame.odom_frame", "odom");
    config_.output_frame.lidar_frame = this->declare_parameter<std::string>(
        "output_frame.lidar_frame", "front_mid360");
    config_.output_frame.base_frame = this->declare_parameter<std::string>(
        "output_frame.base_frame", "base_footprint");
    config_.output_frame.registered_scan_topic =
        this->declare_parameter<std::string>(
            "output_frame.registered_scan_topic", "registered_scan");
    config_.output_frame.lidar_odometry_topic =
        this->declare_parameter<std::string>(
            "output_frame.lidar_odometry_topic", "lidar_odometry");
    config_.output_frame.z_filter.enabled = this->declare_parameter<bool>(
        "output_frame.z_filter.enable", false);
    config_.output_frame.z_filter.min_z = this->declare_parameter<double>(
        "output_frame.z_filter.min_z", -0.2);
    config_.output_frame.z_filter.max_z = this->declare_parameter<double>(
        "output_frame.z_filter.max_z", 1.0);
    if (config_.output_frame.z_filter.min_z
        > config_.output_frame.z_filter.max_z) {
      RCLCPP_ERROR(get_logger(),
                   "output_frame.z_filter 无效: min_z(%.3f) > max_z(%.3f), "
                   "已禁用 Z 滤波",
                   config_.output_frame.z_filter.min_z,
                   config_.output_frame.z_filter.max_z);
      config_.output_frame.z_filter.enabled = false;
    }
    config_.output_frame.lidar_to_base_t =
        this->declare_parameter<std::vector<double>>(
            "output_frame.lidar_to_base_t", std::vector<double>());
    config_.output_frame.lidar_to_base_r =
        this->declare_parameter<std::vector<double>>(
            "output_frame.lidar_to_base_r", std::vector<double>());
    config_.output_frame.body_to_base_t =
        this->declare_parameter<std::vector<double>>(
            "output_frame.body_to_base_t", std::vector<double>());
    config_.output_frame.body_to_base_r =
        this->declare_parameter<std::vector<double>>(
            "output_frame.body_to_base_r", std::vector<double>());
    config_.output_frame.body_to_base_from_params =
        config_.output_frame.body_to_base_t.size() == 3
        && config_.output_frame.body_to_base_r.size() == 9;
    config_.output_frame.extrinsic_from_params =
        config_.output_frame.lidar_to_base_t.size() == 3
        && config_.output_frame.lidar_to_base_r.size() == 9;

    config_.mapping.ivox_options.resolution_ = static_cast<float>(
        this->declare_parameter<double>("mapping.ivox_grid_resolution", 0.2));
    const int ivox_nearby_type = static_cast<int>(
        this->declare_parameter<long>("ivox_nearby_type", 18));
    if (ivox_nearby_type == 0) {
      config_.mapping.ivox_options.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
      config_.mapping.ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 26) {
      config_.mapping.ivox_options.nearby_type_ =
          IVoxType::NearbyType::NEARBY26;
    } else {
      config_.mapping.ivox_options.nearby_type_ =
          IVoxType::NearbyType::NEARBY18;
    }
  } catch (const rclcpp::ParameterTypeException& e) {
    RCLCPP_ERROR(get_logger(), "Parameter type exception: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception: %s", e.what());
  }

  if (config_.imu.processor.gravity.norm() > 0.0) {
    const auto& gravity = config_.imu.processor.gravity;
    config_.common.gravity_magnitude = std::hypot(gravity[0], gravity[1],
                                                  gravity[2]);
  }
  config_.imu.processor.gravity_magnitude = config_.common.gravity_magnitude;
  config_.lidar.cut_frame_num = std::max(1, config_.lidar.cut_frame_num);
  if (config_.lidar.cut_frame_interval > 0.0) {
    config_.lidar.cut_frame_num = std::max(
        1, static_cast<int>(std::lround(config_.lidar.lidar_time_interval
                                        / config_.lidar.cut_frame_interval)));
  }
  config_.lidar.con_frame_num = std::max(1, config_.lidar.con_frame_num);
  config_.imu.timestamp_offset = config_.sensor.lidar_to_imu_time;
}

void LaserMappingNode::initializeRos2Interfaces() {
  if (config_.publish.debug_state_enabled) {
    debug_state_pub_ = create_publisher<std_msgs::msg::String>(
        "/point_lio/debug_state", 10);
  }

  // 项目约定输出 (原 loam_interface 的职责): 同一个 odom 系下的点云与里程计
  // PCD 保存也依赖 odom 系 Z 滤波, 因此 scan 未发布时也可能需要 TF。
  if (!config_.output_frame.body_to_base_from_params
      && !config_.output_frame.extrinsic_from_params) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  project_publisher_.configure(this, config_.output_frame, &odom_projector_);

  if (!config_.publish.pcd_save_state_topic.empty()) {
    RCLCPP_INFO(get_logger(), "PCD 保存按定位状态驱动: %s",
                config_.publish.pcd_save_state_topic.c_str());
  } else if (config_.publish.pcd_save_require_lost) {
    if (config_.publish.pcd_save_trigger_topic.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "pcd_save.require_lost 为 true 但 trigger_topic 为空, "
                   "PCD 将不会保存");
    } else {
      RCLCPP_INFO(get_logger(), "PCD 保存等待 LOST 触发: %s",
                  config_.publish.pcd_save_trigger_topic.c_str());
    }
  }

  prior_map_.configure(config_.publish, config_.output_frame, &odom_projector_,
                       this, callback_group_);
  if (!config_.publish.pcd_save_reload_service.empty()) {
    RCLCPP_INFO(get_logger(), "先验地图重载服务: %s",
                config_.publish.pcd_save_reload_service.c_str());
  }

  preprocess_.configure(config_.lidar.preprocess);
  sensors_.configure(this, config_, &processor_, &preprocess_, callback_group_);
}

void LaserMappingNode::processIteration() {
  // 定时器以 500 Hz 轮询, 但只有真正处理了新帧才发布输出。
  // 否则没有新帧时也会反复重发同一幅点云与同一条 path:
  // 既占满 CPU (每 2 ms 转换一次全分辨率点云),
  // 又让 path 消息不断累积重复位姿。
  const bool is_frame_processed = processor_.processIteration();

  if (prev_stage_ == PointLioStage::INITIALIZINGMAP
      && stage_ == PointLioStage::TRACKING) {
    frame_publisher_.publishMap(data_handler_.map_cloud);
  }
  prev_stage_ = stage_;

  if (!is_frame_processed) {
    return;
  }

  publishFrameOutputs();

  if (debug_state_pub_) {
    const auto& s = data_handler_.filter_state;
    std::ostringstream out;
    out << "omg: " << s.omega.transpose() << '\n'
        << "bg:  " << s.bg.transpose() << '\n'
        << "acc: " << s.acc.transpose() << '\n'
        << "ba:  " << s.ba.transpose() << '\n'
        << "vel: " << s.vel.transpose() << '\n'
        << "pos: " << s.pos.transpose() << '\n'
        << "g:   " << s.gravity.transpose();
    std_msgs::msg::String msg;
    msg.data = out.str();
    debug_state_pub_->publish(msg);
  }
}

void LaserMappingNode::publishFrameOutputs() {
  if (config_.output_frame.z_filter.enabled && prior_map_.active()) {
    odom_projector_.resolve(
        config_.output_frame, tf_buffer_.get(), get_logger(), *get_clock(),
        config_.sensor.extrinsic_r, config_.sensor.extrinsic_t);
  }
  if (config_.publish.scan_enabled || prior_map_.active()) {
    publishFrameWorld();
  }
  if (!odom_projector_.ready()
      && !odom_projector_.resolve(
          config_.output_frame, tf_buffer_.get(), get_logger(), *get_clock(),
          config_.sensor.extrinsic_r, config_.sensor.extrinsic_t)) {
    return;
  }
  if (config_.publish.scan_enabled || data_handler_.odometry_valid) {
    project_publisher_.publish(
        *processor_.currentDownsampledWorld(), data_handler_.odometry,
        data_handler_.odometry_valid, get_ros_time(processor_.lidarEndTime()));
  }
}

void LaserMappingNode::publishFrameWorld() {
  if (config_.publish.scan_enabled) {
    frame_publisher_.publishWorld(*processor_.currentDownsampledWorld(),
                                  get_ros_time(processor_.lidarEndTime()));
  }

  prior_map_.update(*processor_.currentDownsampledWorld());
}

LaserMappingNode::~LaserMappingNode() {
  try {
    prior_map_.savePending();
  } catch (...) {
  }
}
