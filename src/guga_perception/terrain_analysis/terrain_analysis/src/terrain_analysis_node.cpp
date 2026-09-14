// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#include "terrain_analysis/terrain_analysis_node.hpp"
#include "terrain_analysis/core/algorithm.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace terrain_analysis {

  TerrainAnalysis::TerrainAnalysis(const rclcpp::NodeOptions& options)
      : Node("terrain_analysis", options) {
    config_.scan_voxel_size = declare_parameter("scanVoxelSize",
                                                config_.scan_voxel_size);
    config_.decay_time = declare_parameter("decayTime", config_.decay_time);
    config_.no_decay_distance = declare_parameter("noDecayDis",
                                                  config_.no_decay_distance);
    config_.use_sorting = declare_parameter("useSorting", config_.use_sorting);
    config_.quantile_z = declare_parameter("quantileZ", config_.quantile_z);
    config_.consider_drop = declare_parameter("considerDrop",
                                              config_.consider_drop);
    config_.limit_ground_lift = declare_parameter("limitGroundLift",
                                                  config_.limit_ground_lift);
    config_.max_ground_lift = declare_parameter("maxGroundLift",
                                                config_.max_ground_lift);
    config_.clear_dy_obs = declare_parameter("clearDyObs",
                                             config_.clear_dy_obs);
    config_.min_dy_obs_distance = declare_parameter(
        "minDyObsDis", config_.min_dy_obs_distance);
    config_.min_dy_obs_angle = declare_parameter("minDyObsAngle",
                                                 config_.min_dy_obs_angle);
    config_.min_dy_obs_relative_z = declare_parameter(
        "minDyObsRelZ", config_.min_dy_obs_relative_z);
    config_.abs_dy_obs_relative_z_threshold = declare_parameter(
        "absDyObsRelZThre", config_.abs_dy_obs_relative_z_threshold);
    config_.min_dy_obs_vfov = declare_parameter("minDyObsVFOV",
                                                config_.min_dy_obs_vfov);
    config_.max_dy_obs_vfov = declare_parameter("maxDyObsVFOV",
                                                config_.max_dy_obs_vfov);
    config_.min_dy_obs_point_num = declare_parameter(
        "minDyObsPointNum", config_.min_dy_obs_point_num);
    config_.min_block_point_num = declare_parameter(
        "minBlockPointNum", config_.min_block_point_num);
    config_.vehicle_height = declare_parameter("vehicleHeight",
                                               config_.vehicle_height);
    config_.ceiling_clearance = declare_parameter("ceilingClearance",
                                                  config_.ceiling_clearance);
    config_.voxel_point_update_thre = declare_parameter(
        "voxelPointUpdateThre", config_.voxel_point_update_thre);
    config_.voxel_time_update_thre = declare_parameter(
        "voxelTimeUpdateThre", config_.voxel_time_update_thre);
    config_.min_relative_z = declare_parameter("minRelZ",
                                               config_.min_relative_z);
    config_.max_relative_z = declare_parameter("maxRelZ",
                                               config_.max_relative_z);
    config_.distance_ratio_z = declare_parameter("disRatioZ",
                                                 config_.distance_ratio_z);

    config_.min_dy_obs_angle *= M_PI / 180.0;
    config_.min_dy_obs_vfov *= M_PI / 180.0;
    config_.max_dy_obs_vfov *= M_PI / 180.0;
    state_.down_size_filter.setLeafSize(
        static_cast<float>(config_.scan_voxel_size),
        static_cast<float>(config_.scan_voxel_size),
        static_cast<float>(config_.scan_voxel_size));

    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "lidar_odometry", 5,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          double roll{};
          double pitch{};
          double yaw{};
          const auto& q = msg->pose.pose.orientation;
          tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w))
              .getRPY(roll, pitch, yaw);
          terrain_analysis::algorithm::ingestOdometry(
              config_, state_, msg->pose.pose.position.x,
              msg->pose.pose.position.y, msg->pose.pose.position.z, roll, pitch,
              yaw);
        });

    sub_laser_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "registered_scan", 5,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
          pcl::fromROSMsg(*msg, *cloud);
          terrain_analysis::algorithm::ingestLaserCloud(
              config_, state_, cloud,
              rclcpp::Time(msg->header.stamp).seconds());
        });

    pub_terrain_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "terrain_map", 2);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     [this]() { processOnce(); });
  }

  bool TerrainAnalysis::processOnce() {
    if (!state_.new_laser_cloud) {
      return rclcpp::ok();
    }

    terrain_analysis::algorithm::run(config_, state_);
    publishPointCloud();
    return rclcpp::ok();
  }

  void TerrainAnalysis::publishPointCloud() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*state_.terrain_cloud_elev, message);
    message.header.stamp = rclcpp::Time(
        static_cast<int64_t>(state_.laser_cloud_time * 1e9));
    message.header.frame_id = "odom";
    pub_terrain_map_->publish(message);
  }

}  // namespace terrain_analysis
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis::TerrainAnalysis)