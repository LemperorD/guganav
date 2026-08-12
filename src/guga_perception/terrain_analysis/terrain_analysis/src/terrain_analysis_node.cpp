// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#include "terrain_analysis/terrain_analysis_node.hpp"
#include "terrain_analysis/core/algorithm.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace {

  template <typename T>
  void declareAndGet(rclcpp::Node& node, const char* name, T& value) {
    node.declare_parameter<T>(name, value);
    node.get_parameter(name, value);
  }

}  // namespace

TerrainAnalysis::TerrainAnalysis(rclcpp::Node* node) : node_(node) {
  declareAndGet(*node_, "scanVoxelSize", config_.scan_voxel_size);
  declareAndGet(*node_, "decayTime", config_.decay_time);
  declareAndGet(*node_, "noDecayDis", config_.no_decay_distance);
  declareAndGet(*node_, "clearingDis", config_.clearing_distance);
  declareAndGet(*node_, "useSorting", config_.use_sorting);
  declareAndGet(*node_, "quantileZ", config_.quantile_z);
  declareAndGet(*node_, "considerDrop", config_.consider_drop);
  declareAndGet(*node_, "limitGroundLift", config_.limit_ground_lift);
  declareAndGet(*node_, "maxGroundLift", config_.max_ground_lift);
  declareAndGet(*node_, "clearDyObs", config_.clear_dy_obs);
  declareAndGet(*node_, "minDyObsDis", config_.min_dy_obs_distance);
  declareAndGet(*node_, "minDyObsAngle", config_.min_dy_obs_angle);
  declareAndGet(*node_, "minDyObsRelZ", config_.min_dy_obs_relative_z);
  declareAndGet(*node_, "absDyObsRelZThre",
                config_.abs_dy_obs_relative_z_threshold);
  declareAndGet(*node_, "minDyObsVFOV", config_.min_dy_obs_vfov);
  declareAndGet(*node_, "maxDyObsVFOV", config_.max_dy_obs_vfov);
  declareAndGet(*node_, "minDyObsPointNum", config_.min_dy_obs_point_num);
  declareAndGet(*node_, "noDataObstacle", config_.no_data_obstacle);
  declareAndGet(*node_, "noDataBlockSkipNum", config_.no_data_block_skip_num);
  declareAndGet(*node_, "minBlockPointNum", config_.min_block_point_num);
  declareAndGet(*node_, "vehicleHeight", config_.vehicle_height);
  declareAndGet(*node_, "voxelPointUpdateThre",
                config_.voxel_point_update_thre);
  declareAndGet(*node_, "voxelTimeUpdateThre", config_.voxel_time_update_thre);
  declareAndGet(*node_, "minRelZ", config_.min_relative_z);
  declareAndGet(*node_, "maxRelZ", config_.max_relative_z);
  declareAndGet(*node_, "disRatioZ", config_.distance_ratio_z);

  config_.min_dy_obs_angle *= M_PI / 180.0;
  config_.min_dy_obs_vfov *= M_PI / 180.0;
  config_.max_dy_obs_vfov *= M_PI / 180.0;
  state_.down_size_filter.setLeafSize(
      static_cast<float>(config_.scan_voxel_size),
      static_cast<float>(config_.scan_voxel_size),
      static_cast<float>(config_.scan_voxel_size));

  sub_odometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "lidar_odometry", 5, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
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

  sub_laser_cloud_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "registered_scan", 5,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*msg, *cloud);
        terrain_analysis::algorithm::ingestLaserCloud(
            config_, state_, cloud, rclcpp::Time(msg->header.stamp).seconds());
      });

  sub_clearing_ = node_->create_subscription<std_msgs::msg::Float32>(
      "map_clearing", 5, [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
        terrain_analysis::algorithm::ingestClearing(state_, msg->data);
      });

  pub_terrain_map_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "terrain_map", 2);

  timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
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
