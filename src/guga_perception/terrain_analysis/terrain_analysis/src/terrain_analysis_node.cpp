// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#include "terrain_analysis/terrain_analysis_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace terrain_analysis {

  namespace {

    /**
     * @brief 校验高度参数之间被代码隐含依赖的关系，不满足时提示用户。
     *
     * computeHeightMap 的障碍输出先按 ceilingClearance 裁掉高点，再用
     * vehicleHeight 判"是否算障碍"。若 ceilingClearance < vehicleHeight，
     * 实际生效的上限是前者，vehicleHeight 永远轮不到——表现为"高于某高度
     * 的障碍全部漏检"。二者无编译期约束，故在启动时校验。
     */
    void warnOnHeightParams(const TerrainConfig& config) {
      if (config.ceiling_clearance < config.vehicle_height) {
        RCLCPP_WARN(
            rclcpp::get_logger("terrain_analysis"),
            "ceilingClearance(%.3f) < vehicleHeight(%.3f)：障碍输出的实际上界"
            "是 ceilingClearance，vehicleHeight 不会生效，高于 %.3f m 的障碍将"
            "全部漏检。建议按车体高度 + 100mm 设定 ceilingClearance。",
            config.ceiling_clearance, config.vehicle_height,
            config.ceiling_clearance);
      }
    }

  }  // namespace

  TerrainAnalysis::TerrainAnalysis(const rclcpp::NodeOptions& options)
      : Node("terrain_analysis", options) {
    TerrainConfig& config = processor_.config();
    config.scan_voxel_size = declare_parameter("scanVoxelSize",
                                               config.scan_voxel_size);
    config.decay_time = declare_parameter("decayTime", config.decay_time);
    config.no_decay_distance = declare_parameter("noDecayDis",
                                                 config.no_decay_distance);
    config.use_sorting = declare_parameter("useSorting", config.use_sorting);
    config.quantile_z = declare_parameter("quantileZ", config.quantile_z);
    config.consider_drop = declare_parameter("considerDrop",
                                             config.consider_drop);
    config.limit_ground_lift = declare_parameter("limitGroundLift",
                                                 config.limit_ground_lift);
    config.max_ground_lift = declare_parameter("maxGroundLift",
                                               config.max_ground_lift);
    config.min_dy_obs_distance = declare_parameter("minDyObsDis",
                                                   config.min_dy_obs_distance);
    config.min_dy_obs_angle = declare_parameter("minDyObsAngle",
                                                config.min_dy_obs_angle);
    config.min_dy_obs_relative_z = declare_parameter(
        "minDyObsRelZ", config.min_dy_obs_relative_z);
    config.abs_dy_obs_relative_z_threshold = declare_parameter(
        "absDyObsRelZThre", config.abs_dy_obs_relative_z_threshold);
    config.min_dy_obs_vfov = declare_parameter("minDyObsVFOV",
                                               config.min_dy_obs_vfov);
    config.max_dy_obs_vfov = declare_parameter("maxDyObsVFOV",
                                               config.max_dy_obs_vfov);
    config.min_dy_obs_point_num = declare_parameter(
        "minDyObsPointNum", config.min_dy_obs_point_num);
    config.min_block_point_num = declare_parameter("minBlockPointNum",
                                                   config.min_block_point_num);
    config.vehicle_height = declare_parameter("vehicleHeight",
                                              config.vehicle_height);
    config.ceiling_clearance = declare_parameter("ceilingClearance",
                                                 config.ceiling_clearance);
    config.voxel_point_update_thre = declare_parameter(
        "voxelPointUpdateThre", config.voxel_point_update_thre);
    config.voxel_time_update_thre = declare_parameter(
        "voxelTimeUpdateThre", config.voxel_time_update_thre);
    config.min_relative_z = declare_parameter("minRelZ", config.min_relative_z);
    config.max_relative_z = declare_parameter("maxRelZ", config.max_relative_z);
    config.ground_floor_z = declare_parameter("groundFloorZ",
                                              config.ground_floor_z);
    config.distance_ratio_z = declare_parameter("disRatioZ",
                                                config.distance_ratio_z);

    config.min_dy_obs_angle *= M_PI / 180.0;
    config.min_dy_obs_vfov *= M_PI / 180.0;
    config.max_dy_obs_vfov *= M_PI / 180.0;

    warnOnHeightParams(config);

    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "lidar_odometry", 5,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          double roll{};
          double pitch{};
          double yaw{};
          const auto& q = msg->pose.pose.orientation;
          tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w))
              .getRPY(roll, pitch, yaw);
          processor_.ingestOdometry(
              msg->pose.pose.position.x, msg->pose.pose.position.y,
              msg->pose.pose.position.z, roll, pitch, yaw);
        });

    sub_laser_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "registered_scan", 5,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
          pcl::fromROSMsg(*msg, *cloud);
          processor_.ingestLaserCloud(
              cloud, rclcpp::Time(msg->header.stamp).seconds());
        });

    pub_terrain_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "terrain_map", 2);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     [this]() { processOnce(); });
  }

  bool TerrainAnalysis::processOnce() {
    if (!processor_.hasPendingCloud()) {
      return rclcpp::ok();
    }

    processor_.run();
    publishPointCloud();
    return rclcpp::ok();
  }

  void TerrainAnalysis::publishPointCloud() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(processor_.terrainCloudElev(), message);
    message.header.stamp = rclcpp::Time(
        static_cast<int64_t>(processor_.laserCloudTime() * 1e9));
    message.header.frame_id = "odom";
    pub_terrain_map_->publish(message);
  }

}  // namespace terrain_analysis
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis::TerrainAnalysis)