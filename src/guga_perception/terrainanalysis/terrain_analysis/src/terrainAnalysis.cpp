// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#include "terrain_analysis/terrain_analysis.hpp"
#include "terrain_analysis/core/algorithm.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>

TerrainAnalysis::TerrainAnalysis(rclcpp::Node* node) : node_(node) {
}

TerrainAnalysis::~TerrainAnalysis() = default;

void TerrainAnalysis::initialize(const std::string& output_topic) {
  node_->declare_parameter<double>("scanVoxelSize", context_.cfg.scan_voxel_size);
  node_->declare_parameter<double>("decayTime", context_.cfg.decay_time);
  node_->declare_parameter<double>("noDecayDis", context_.cfg.no_decay_distance);
  node_->declare_parameter<double>("clearingDis", context_.cfg.clearing_distance);
  node_->declare_parameter<bool>("useSorting", context_.cfg.use_sorting);
  node_->declare_parameter<double>("quantileZ", context_.cfg.quantile_z);
  node_->declare_parameter<bool>("considerDrop", context_.cfg.consider_drop);
  node_->declare_parameter<bool>("limitGroundLift",
                                 context_.cfg.limit_ground_lift);
  node_->declare_parameter<double>("maxGroundLift", context_.cfg.max_ground_lift);
  node_->declare_parameter<bool>("clearDyObs", context_.cfg.clear_dy_obs);
  node_->declare_parameter<double>("minDyObsDis", context_.cfg.min_dy_obs_distance);
  node_->declare_parameter<double>("minDyObsAngle", context_.cfg.min_dy_obs_angle);
  node_->declare_parameter<double>("minDyObsRelZ", context_.cfg.min_dy_obs_relative_z);
  node_->declare_parameter<double>("absDyObsRelZThre",
                                   context_.cfg.abs_dy_obs_relative_z_threshold);
  node_->declare_parameter<double>("minDyObsVFOV", context_.cfg.min_dy_obs_vfov);
  node_->declare_parameter<double>("maxDyObsVFOV", context_.cfg.max_dy_obs_vfov);
  node_->declare_parameter<int>("minDyObsPointNum",
                                context_.cfg.min_dy_obs_point_num);
  node_->declare_parameter<bool>("noDataObstacle", context_.cfg.no_data_obstacle);
  node_->declare_parameter<int>("noDataBlockSkipNum",
                                context_.cfg.no_data_block_skip_num);
  node_->declare_parameter<int>("minBlockPointNum",
                                context_.cfg.min_block_point_num);
  node_->declare_parameter<double>("vehicleHeight", context_.cfg.vehicle_height);
  node_->declare_parameter<int>("voxelPointUpdateThre",
                                context_.cfg.voxel_point_update_thre);
  node_->declare_parameter<double>("voxelTimeUpdateThre",
                                   context_.cfg.voxel_time_update_thre);
  node_->declare_parameter<double>("minRelZ", context_.cfg.min_relative_z);
  node_->declare_parameter<double>("maxRelZ", context_.cfg.max_relative_z);
  node_->declare_parameter<double>("disRatioZ", context_.cfg.distance_ratio_z);

  node_->get_parameter("scanVoxelSize", context_.cfg.scan_voxel_size);
  node_->get_parameter("decayTime", context_.cfg.decay_time);
  node_->get_parameter("noDecayDis", context_.cfg.no_decay_distance);
  node_->get_parameter("clearingDis", context_.cfg.clearing_distance);
  node_->get_parameter("useSorting", context_.cfg.use_sorting);
  node_->get_parameter("quantileZ", context_.cfg.quantile_z);
  node_->get_parameter("considerDrop", context_.cfg.consider_drop);
  node_->get_parameter("limitGroundLift", context_.cfg.limit_ground_lift);
  node_->get_parameter("maxGroundLift", context_.cfg.max_ground_lift);
  node_->get_parameter("clearDyObs", context_.cfg.clear_dy_obs);
  node_->get_parameter("minDyObsDis", context_.cfg.min_dy_obs_distance);
  node_->get_parameter("minDyObsAngle", context_.cfg.min_dy_obs_angle);
  context_.cfg.min_dy_obs_angle *= M_PI / 180.0;
  node_->get_parameter("minDyObsRelZ", context_.cfg.min_dy_obs_relative_z);
  node_->get_parameter("absDyObsRelZThre", context_.cfg.abs_dy_obs_relative_z_threshold);
  node_->get_parameter("minDyObsVFOV", context_.cfg.min_dy_obs_vfov);
  context_.cfg.min_dy_obs_vfov *= M_PI / 180.0;
  node_->get_parameter("maxDyObsVFOV", context_.cfg.max_dy_obs_vfov);
  context_.cfg.max_dy_obs_vfov *= M_PI / 180.0;
  node_->get_parameter("minDyObsPointNum", context_.cfg.min_dy_obs_point_num);
  node_->get_parameter("noDataObstacle", context_.cfg.no_data_obstacle);
  node_->get_parameter("noDataBlockSkipNum", context_.cfg.no_data_block_skip_num);
  node_->get_parameter("minBlockPointNum", context_.cfg.min_block_point_num);
  node_->get_parameter("vehicleHeight", context_.cfg.vehicle_height);
  node_->get_parameter("voxelPointUpdateThre",
                       context_.cfg.voxel_point_update_thre);
  node_->get_parameter("voxelTimeUpdateThre", context_.cfg.voxel_time_update_thre);
  node_->get_parameter("minRelZ", context_.cfg.min_relative_z);
  node_->get_parameter("maxRelZ", context_.cfg.max_relative_z);
  node_->get_parameter("disRatioZ", context_.cfg.distance_ratio_z);

  sub_odometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "lidar_odometry", 5, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        double roll{};
        double pitch{};
        double yaw{};
        const auto& q = msg->pose.pose.orientation;
        tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w))
            .getRPY(roll, pitch, yaw);
        context_.onOdometry(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z, roll, pitch, yaw);
      });

  sub_laser_cloud_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "registered_scan", 5,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*msg, *pcl_cloud);
        double t = rclcpp::Time(msg->header.stamp).seconds();
        context_.onLaserCloud(pcl_cloud, t);
      });

  sub_joystick_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 5, [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        context_.onJoystick(msg->buttons[5] > 0.5);
      });

  sub_clearing_ = node_->create_subscription<std_msgs::msg::Float32>(
      "map_clearing", 5, [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
        context_.onClearing(msg->data);
      });

  pub_terrain_map_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, 2);

  context_.state.down_size_filter.setLeafSize(context_.cfg.scan_voxel_size,
                                         context_.cfg.scan_voxel_size,
                                         context_.cfg.scan_voxel_size);
}

bool TerrainAnalysis::processOnce() {
  rclcpp::spin_some(node_->get_node_base_interface());
  if (!context_.state.hasNewCloud()) {
    return rclcpp::ok();
  }
  TerrainAlgorithm::run(context_.cfg, context_.state);
  publishPointCloud();
  return rclcpp::ok();
}

void TerrainAnalysis::publishPointCloud() {
  sensor_msgs::msg::PointCloud2 terraincloud2;
  pcl::toROSMsg(context_.state.terrainCloudElev(), terraincloud2);

  terraincloud2.header.stamp = rclcpp::Time(
      static_cast<int64_t>(context_.state.laser_cloud_time * 1e9));
  terraincloud2.header.frame_id = "odom";

  pub_terrain_map_->publish(terraincloud2);
}
