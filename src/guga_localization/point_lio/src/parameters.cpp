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
    params.imu.check_saturation = nh->declare_parameter<bool>(
        "check_satu", true);
    params.mapping.init_map_size = static_cast<int>(
        nh->declare_parameter<long>("init_map_size", 100));
    params.mapping.space_down_sample = nh->declare_parameter<bool>(
        "space_down_sample", true);

    // ==================== mapping 参数 — IMU 饱和 ====================
    params.imu.saturation_acc = nh->declare_parameter<double>(
        "mapping.satu_acc", 3.0);
    params.imu.saturation_gyro = nh->declare_parameter<double>(
        "mapping.satu_gyro", 35.0);
    params.imu.acc_norm = nh->declare_parameter<double>(
        "mapping.acc_norm", 1.0);

    // ==================== mapping 参数 — 平面提取 ====================
    params.lidar.plane_threshold = static_cast<float>(
        nh->declare_parameter<double>("mapping.plane_thr", 0.05F));
    params.lidar.preprocess.point_filter_num = static_cast<int>(
        nh->declare_parameter<long>("point_filter_num", 2));

    // ==================== common 参数 — 话题名 ====================
    params.sensor.lidar_topic = nh->declare_parameter<std::string>(
        "common.lid_topic", ".livox.lidar");
    params.sensor.imu_topic = nh->declare_parameter<std::string>(
        "common.imu_topic", ".livox.imu");

    // ==================== common 参数 — 合帧/切帧 ====================
    params.lidar.con_frame = nh->declare_parameter<bool>(
        "common.con_frame", false);
    params.lidar.con_frame_num = static_cast<int>(
        nh->declare_parameter<long>("common.con_frame_num", 1));
    params.lidar.cut_frame = nh->declare_parameter<bool>(
        "common.cut_frame", false);
    params.lidar.cut_frame_interval = nh->declare_parameter<double>(
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
    params.lidar.preprocess.det_range = params.mapping.det_range;
    params.mapping.fov_deg = nh->declare_parameter<double>("mapping.fov_degree",
                                                           180);

    // ==================== mapping 参数 — IMU 功能开关 ====================
    params.imu.processor.enabled = nh->declare_parameter<bool>(
        "mapping.imu_en", true);
    params.lidar.extrinsic_estimation = nh->declare_parameter<bool>(
        "mapping.extrinsic_est_en", true);
    params.imu.integration_interval = nh->declare_parameter<double>(
        "mapping.imu_time_inte", 0.005);

    // ==================== mapping 参数 — 噪声协方差 ====================
    params.lidar.point_covariance = nh->declare_parameter<double>(
        "mapping.lidar_meas_cov", 0.1);
    params.filter.acc_cov_input = nh->declare_parameter<double>(
        "mapping.acc_cov_input", 0.1);
    params.filter.vel_cov = nh->declare_parameter<double>("mapping.vel_cov",
                                                             20);
    params.filter.gyr_cov_input = nh->declare_parameter<double>(
        "mapping.gyr_cov_input", 0.1);
    params.filter.gyr_cov_output = nh->declare_parameter<double>(
        "mapping.gyr_cov_output", 0.1);
    params.filter.acc_cov_output = nh->declare_parameter<double>(
        "mapping.acc_cov_output", 0.1);
    params.filter.b_gyr_cov = nh->declare_parameter<double>(
        "mapping.b_gyr_cov", 0.0001);
    params.filter.b_acc_cov = nh->declare_parameter<double>(
        "mapping.b_acc_cov", 0.0001);
    params.imu.measurement_acc_cov = nh->declare_parameter<double>(
        "mapping.imu_meas_acc_cov", 0.1);
    params.imu.measurement_gyro_cov = nh->declare_parameter<double>(
        "mapping.imu_meas_omg_cov", 0.1);

    // ==================== preprocess 参数 — 雷达配置 ====================
    params.lidar.preprocess.blind = nh->declare_parameter<double>(
        "preprocess.blind", 1.0);
    params.lidar.lidar_type = static_cast<int>(
        nh->declare_parameter<long>("preprocess.lidar_type", 1));
    params.lidar.preprocess.lidar_type =
        params.lidar.lidar_type;
    params.lidar.preprocess.scan_lines = static_cast<int>(
        nh->declare_parameter<long>("preprocess.scan_line", 16));
    params.lidar.preprocess.scan_rate = static_cast<int>(
        nh->declare_parameter<long>("preprocess.scan_rate", 10));
    params.lidar.preprocess.timestamp_unit = static_cast<int>(
        nh->declare_parameter<long>("preprocess.timestamp_unit", 1));
    params.lidar.match_threshold = nh->declare_parameter<double>("mapping.match_s",
                                                             81);

    // ==================== mapping 参数 — 重力 ====================
    const auto gravity = nh->declare_parameter<std::vector<double>>(
        "mapping.gravity", std::vector<double>());
    const auto gravity_init = nh->declare_parameter<std::vector<double>>(
        "mapping.gravity_init", std::vector<double>());
    if (gravity.size() >= 3) {
      params.imu.processor.gravity =
          V3D(gravity[0], gravity[1], gravity[2]);
    }
    if (gravity_init.size() >= 3) {
      params.imu.processor.gravity_init =
          V3D(gravity_init[0], gravity_init[1], gravity_init[2]);
    }

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
    // ==================== pcd_save 参数 ====================
    params.publish.pcd_save_enabled = nh->declare_parameter<bool>(
        "pcd_save.pcd_save_en", false);
    params.publish.pcd_save_interval = static_cast<int>(
        nh->declare_parameter<long>("pcd_save.interval", -1));
    params.lidar.lidar_time_interval = nh->declare_parameter<double>(
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

  if (params.imu.processor.gravity.norm() > 0.0) {
    const auto& gravity = params.imu.processor.gravity;
    params.common.gravity_magnitude = std::hypot(gravity[0], gravity[1],
                                                 gravity[2]);
  }
  params.imu.processor.gravity_magnitude = params.common.gravity_magnitude;
  return params;
}
