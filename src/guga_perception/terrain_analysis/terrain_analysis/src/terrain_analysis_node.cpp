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
  namespace {

    template <typename T>
    void declareAndGet(rclcpp::Node* node, const char* name, T& value) {
      node->declare_parameter<T>(name, value);
      node->get_parameter(name, value);
    }

  }  // namespace

  TerrainAnalysis::TerrainAnalysis(const rclcpp::NodeOptions& options)
      : Node("terrain_analysis", options) {
    declareAndGet(this, "scanVoxelSize", config_.scan_voxel_size);
    declareAndGet(this, "decayTime", config_.decay_time);
    declareAndGet(this, "noDecayDis", config_.no_decay_distance);
    declareAndGet(this, "clearingDis", config_.clearing_distance);
    declareAndGet(this, "useSorting", config_.use_sorting);
    declareAndGet(this, "quantileZ", config_.quantile_z);
    declareAndGet(this, "considerDrop", config_.consider_drop);
    declareAndGet(this, "limitGroundLift", config_.limit_ground_lift);
    declareAndGet(this, "maxGroundLift", config_.max_ground_lift);
    declareAndGet(this, "clearDyObs", config_.clear_dy_obs);
    declareAndGet(this, "minDyObsDis", config_.min_dy_obs_distance);
    declareAndGet(this, "minDyObsAngle", config_.min_dy_obs_angle);
    declareAndGet(this, "minDyObsRelZ", config_.min_dy_obs_relative_z);
    declareAndGet(this, "absDyObsRelZThre",
                  config_.abs_dy_obs_relative_z_threshold);
    declareAndGet(this, "minDyObsVFOV", config_.min_dy_obs_vfov);
    declareAndGet(this, "maxDyObsVFOV", config_.max_dy_obs_vfov);
    declareAndGet(this, "minDyObsPointNum", config_.min_dy_obs_point_num);
    declareAndGet(this, "noDataObstacle", config_.no_data_obstacle);
    declareAndGet(this, "noDataBlockSkipNum", config_.no_data_block_skip_num);
    declareAndGet(this, "minBlockPointNum", config_.min_block_point_num);
    declareAndGet(this, "vehicleHeight", config_.vehicle_height);
    declareAndGet(this, "ceilingClearance", config_.ceiling_clearance);
    declareAndGet(this, "voxelPointUpdateThre",
                  config_.voxel_point_update_thre);
    declareAndGet(this, "voxelTimeUpdateThre", config_.voxel_time_update_thre);
    declareAndGet(this, "minRelZ", config_.min_relative_z);
    declareAndGet(this, "maxRelZ", config_.max_relative_z);
    declareAndGet(this, "disRatioZ", config_.distance_ratio_z);

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

    sub_clearing_ = this->create_subscription<std_msgs::msg::Float32>(
        "map_clearing", 5, [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
          terrain_analysis::algorithm::ingestClearing(state_, msg->data);
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