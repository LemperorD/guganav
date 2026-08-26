/**
 * @file parameters.cpp
 * @brief 运行期共享状态定义和 readParameters() 的实现
 *
 * 该文件定义少量运行期共享状态，并提供从 ROS2 参数服务器读取 YAML 配置的功能。
 * 参数读取使用 try-catch 保护，确保在参数类型不匹配时打印错误信息。
 *
 * 参数分三大类:
 * - **common**: 话题名、合帧/切帧设置、时间偏移
 * - **mapping**: 建图相关 (IMU协方差、饱和值、滤波、重力、外参)
 * - **preprocess**: 预处理 (LiDAR类型、线数、扫描频率、盲区)
 * - **odometry/publish/pcd_save**: 输出相关 (发布开关、路径、PCD)
 */

#include "point_lio/parameters.h"
#include <memory>

PointLioParams readParameters(const std::shared_ptr<rclcpp::Node>& nh) {
  PointLioParams params;
  try {
    // ==================== 模式开关 ====================
    params.mapping.propagate_at_imu_frequency = nh->declare_parameter<bool>(
        "prop_at_freq_of_imu", true);
    params.estimator.check_saturation = nh->declare_parameter<bool>(
        "check_satu", true);
    params.mapping.init_map_size = static_cast<int>(
        nh->declare_parameter<long>("init_map_size", 100));
    params.mapping.space_down_sample = nh->declare_parameter<bool>(
        "space_down_sample", true);

    // ==================== mapping 参数 — IMU 饱和 ====================
    params.estimator.saturation_acc = nh->declare_parameter<double>(
        "mapping.satu_acc", 3.0);
    params.estimator.saturation_gyro = nh->declare_parameter<double>(
        "mapping.satu_gyro", 35.0);
    params.estimator.acc_norm = nh->declare_parameter<double>(
        "mapping.acc_norm", 1.0);

    // ==================== mapping 参数 — 平面提取 ====================
    params.mapping.plane_thr = static_cast<float>(
        nh->declare_parameter<double>("mapping.plane_thr", 0.05F));
    params.estimator.plane_thr = params.mapping.plane_thr;
    params.laser_mapping.preprocess.point_filter_num = static_cast<int>(
        nh->declare_parameter<long>("point_filter_num", 2));

    // ==================== common 参数 — 话题名 ====================
    params.sensor.lidar_topic = nh->declare_parameter<std::string>(
        "common.lid_topic", ".livox.lidar");
    params.sensor.imu_topic = nh->declare_parameter<std::string>(
        "common.imu_topic", ".livox.imu");

    // ==================== common 参数 — 合帧/切帧 ====================
    params.laser_mapping.con_frame = nh->declare_parameter<bool>(
        "common.con_frame", false);
    params.laser_mapping.con_frame_num = static_cast<int>(
        nh->declare_parameter<long>("common.con_frame_num", 1));
    params.laser_mapping.cut_frame = nh->declare_parameter<bool>(
        "common.cut_frame", false);
    params.laser_mapping.cut_frame_interval = nh->declare_parameter<double>(
        "common.cut_frame_time_interval", 0.1);
    params.sensor.lidar_to_imu_time = nh->declare_parameter<double>(
        "common.time_diff_lidar_to_imu", 0.0);

    // ==================== prior_pcd 参数 — 先验地图 ====================
    params.sensor.enable_prior_map = nh->declare_parameter<bool>(
        "prior_pcd.enable", false);
    params.sensor.prior_map_path = nh->declare_parameter<string>(
        "prior_pcd.prior_pcd_map_path", "");
    params.sensor.initial_pose = nh->declare_parameter<std::vector<double>>(
        "prior_pcd.init_pose", std::vector<double>());

    // ==================== 滤波参数 ====================
    params.mapping.filter_size_surf = nh->declare_parameter<double>(
        "filter_size_surf", 0.5);
    params.mapping.filter_size_map = nh->declare_parameter<double>(
        "filter_size_map", 0.5);
    params.mapping.det_range = static_cast<float>(
        nh->declare_parameter<double>("mapping.det_range", 300.F));
    params.laser_mapping.preprocess.det_range = params.mapping.det_range;
    params.mapping.fov_deg = nh->declare_parameter<double>("mapping.fov_degree",
                                                           180);

    // ==================== mapping 参数 — IMU 功能开关 ====================
    params.laser_mapping.imu_enabled = nh->declare_parameter<bool>(
        "mapping.imu_en", true);
    params.mapping.extrinsic_estimation = nh->declare_parameter<bool>(
        "mapping.extrinsic_est_en", true);
    params.estimator.extrinsic_estimation = params.mapping.extrinsic_estimation;
    params.laser_mapping.imu_time_interval = nh->declare_parameter<double>(
        "mapping.imu_time_inte", 0.005);

    // ==================== mapping 参数 — 噪声协方差 ====================
    params.estimator.laser_point_cov = nh->declare_parameter<double>(
        "mapping.lidar_meas_cov", 0.1);
    params.estimator.acc_cov_input = nh->declare_parameter<double>(
        "mapping.acc_cov_input", 0.1);
    params.estimator.vel_cov = nh->declare_parameter<double>("mapping.vel_cov",
                                                             20);
    params.estimator.gyr_cov_input = nh->declare_parameter<double>(
        "mapping.gyr_cov_input", 0.1);
    params.estimator.gyr_cov_output = nh->declare_parameter<double>(
        "mapping.gyr_cov_output", 0.1);
    params.estimator.acc_cov_output = nh->declare_parameter<double>(
        "mapping.acc_cov_output", 0.1);
    params.estimator.b_gyr_cov = nh->declare_parameter<double>(
        "mapping.b_gyr_cov", 0.0001);
    params.estimator.b_acc_cov = nh->declare_parameter<double>(
        "mapping.b_acc_cov", 0.0001);
    params.estimator.imu_meas_acc_cov = nh->declare_parameter<double>(
        "mapping.imu_meas_acc_cov", 0.1);
    params.estimator.imu_meas_omg_cov = nh->declare_parameter<double>(
        "mapping.imu_meas_omg_cov", 0.1);

    // ==================== preprocess 参数 — 雷达配置 ====================
    params.laser_mapping.preprocess.blind = nh->declare_parameter<double>(
        "preprocess.blind", 1.0);
    params.laser_mapping.lidar_type = static_cast<int>(
        nh->declare_parameter<long>("preprocess.lidar_type", 1));
    params.laser_mapping.preprocess.lidar_type =
        params.laser_mapping.lidar_type;
    params.laser_mapping.preprocess.scan_lines = static_cast<int>(
        nh->declare_parameter<long>("preprocess.scan_line", 16));
    params.laser_mapping.preprocess.scan_rate = static_cast<int>(
        nh->declare_parameter<long>("preprocess.scan_rate", 10));
    params.laser_mapping.preprocess.timestamp_unit = static_cast<int>(
        nh->declare_parameter<long>("preprocess.timestamp_unit", 1));
    params.mapping.match_s = nh->declare_parameter<double>("mapping.match_s",
                                                           81);
    params.estimator.match_s = params.mapping.match_s;

    // ==================== mapping 参数 — 重力 ====================
    params.laser_mapping.gravity = nh->declare_parameter<std::vector<double>>(
        "mapping.gravity", std::vector<double>());
    params.laser_mapping.gravity_init =
        nh->declare_parameter<std::vector<double>>("mapping.gravity_init",
                                                   std::vector<double>());

    // ==================== mapping 参数 — 外参 ====================
    params.sensor.extrinsic_t = nh->declare_parameter<std::vector<double>>(
        "mapping.extrinsic_T", std::vector<double>());
    params.sensor.extrinsic_r = nh->declare_parameter<std::vector<double>>(
        "mapping.extrinsic_R", std::vector<double>());

    // ==================== odometry/publish 参数 ====================
    params.mapping.publish_odometry_without_downsample =
        nh->declare_parameter<bool>(
            "odometry.publish_odometry_without_downsample", false);
    params.publish.path_enabled = nh->declare_parameter<bool>("publish.path_en",
                                                              true);
    params.publish.scan_enabled = nh->declare_parameter<bool>(
        "publish.scan_publish_en", true);
    params.publish.scan_body_enabled = nh->declare_parameter<bool>(
        "publish.scan_bodyframe_pub_en", true);
    params.publish.tf_enabled = nh->declare_parameter<bool>(
        "publish.tf_send_en", true);
    params.publish.runtime_log_enabled = nh->declare_parameter<bool>(
        "runtime_pos_log_enable", false);

    // ==================== pcd_save 参数 ====================
    params.publish.pcd_save_enabled = nh->declare_parameter<bool>(
        "pcd_save.pcd_save_en", false);
    params.publish.pcd_save_interval = static_cast<int>(
        nh->declare_parameter<long>("pcd_save.interval", -1));
    params.laser_mapping.lidar_time_interval = nh->declare_parameter<double>(
        "mapping.lidar_time_inte", 0.1);

    // ==================== iVox 网格参数 ====================
    params.mapping.ivox_options.resolution_ = static_cast<float>(
        nh->declare_parameter<double>("mapping.ivox_grid_resolution", 0.2));
    const int ivox_nearby_type = static_cast<int>(
        nh->declare_parameter<long>("ivox_nearby_type", 18));
    if (ivox_nearby_type == 0) {
      params.mapping.ivox_options.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
      params.mapping.ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 26) {
      params.mapping.ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
      params.mapping.ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }
  } catch (const rclcpp::ParameterTypeException& e) {
    RCLCPP_ERROR(nh->get_logger(), "Parameter type exception: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(nh->get_logger(), "Exception: %s", e.what());
  }

  if (params.laser_mapping.gravity.size() >= 3) {
    const auto& gravity = params.laser_mapping.gravity;
    params.estimator.gravity_magnitude = std::hypot(gravity[0], gravity[1],
                                                    gravity[2]);
  }
  return params;
}

/**
 * @brief SO(3) 流形 → ZYX 欧拉角转换
 *
 * 利用 MTK::SO3 类型的 operator() 提取旋转矩阵元素，
 * 然后按照 ZYX 顺序解算欧拉角。
 * 处理万向节死锁情况 (sy≈0 时 yaw=0)。
 */
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3& rot) {
  double sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < 1e-6;  ///< 万向节死锁阈值
  double x, y, z;
  if (!singular) {
    x = atan2(rot(2, 1), rot(2, 2));  ///< roll  = atan2(R32, R33)
    y = atan2(-rot(2, 0), sy);        ///< pitch = atan2(-R31, √(R11²+R21²))
    z = atan2(rot(1, 0), rot(0, 0));  ///< yaw   = atan2(R21, R11)
  } else {
    // 万向节死锁: 设 yaw = 0, 通过 R12,R13 计算 roll
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }
  Eigen::Matrix<double, 3, 1> ang(x, y, z);
  return ang;
}

/**
 * @brief 重置 IMU-as-input 模式的卡尔曼协方差矩阵
 *
 * 协方差分块:
 * - [0:15]:   位置/姿态/速度/外参 = 0.1*I
 * - [15:21]:  速度部分 = 0.001*I
 * - [21:24]:  重力方向 = 0.0001*I
 */
void reset_cov(Eigen::Matrix<double, 24, 24>& P_init) {
  P_init = MD(24, 24)::Identity() * 0.1;
  P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
  P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
}

/**
 * @brief 重置 IMU-as-output 模式的卡尔曼协方差矩阵
 *
 * 协方差分块:
 * - [0:24]:   位置/姿态/速度/外参/角速度/加速度 = 0.01*I
 * - [21:24]:  重力方向 = 0.0001*I
 * - [24:30]:  零偏 = 0.001*I
 */
void reset_cov_output(Eigen::Matrix<double, 30, 30>& P_init_output) {
  P_init_output = MD(30, 30)::Identity() * 0.01;
  P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
  P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
}
