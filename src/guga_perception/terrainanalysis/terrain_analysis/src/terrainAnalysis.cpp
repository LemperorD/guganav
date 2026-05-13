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

TerrainAnalysis::TerrainAnalysis(rclcpp::Node* node) : node_(node) {}

TerrainAnalysis::~TerrainAnalysis() = default;

void TerrainAnalysis::initialize() {
  node_->declare_parameter<double>("scanVoxelSize", ctx_.scan_voxel_size_);
  node_->declare_parameter<double>("decayTime", ctx_.decay_time_);
  node_->declare_parameter<double>("noDecayDis", ctx_.no_decay_dis_);
  node_->declare_parameter<double>("clearingDis", ctx_.clearing_dis_);
  node_->declare_parameter<bool>("useSorting", ctx_.use_sorting_);
  node_->declare_parameter<double>("quantileZ", ctx_.quantile_z_);
  node_->declare_parameter<bool>("considerDrop", ctx_.consider_drop_);
  node_->declare_parameter<bool>("limitGroundLift", ctx_.limit_ground_lift_);
  node_->declare_parameter<double>("maxGroundLift", ctx_.max_ground_lift_);
  node_->declare_parameter<bool>("clearDyObs", ctx_.clear_dy_obs_);
  node_->declare_parameter<double>("minDyObsDis", ctx_.min_dy_obs_dis_);
  node_->declare_parameter<double>("minDyObsAngle", ctx_.min_dy_obs_angle_);
  node_->declare_parameter<double>("minDyObsRelZ", ctx_.min_dy_obs_rel_z_);
  node_->declare_parameter<double>("absDyObsRelZThre", ctx_.abs_dy_obs_rel_z_thre_);
  node_->declare_parameter<double>("minDyObsVFOV", ctx_.min_dy_obs_vfov_);
  node_->declare_parameter<double>("maxDyObsVFOV", ctx_.max_dy_obs_vfov_);
  node_->declare_parameter<int>("minDyObsPointNum", ctx_.min_dy_obs_point_num_);
  node_->declare_parameter<bool>("noDataObstacle", ctx_.no_data_obstacle_);
  node_->declare_parameter<int>("noDataBlockSkipNum", ctx_.no_data_block_skip_num_);
  node_->declare_parameter<int>("minBlockPointNum", ctx_.min_block_point_num_);
  node_->declare_parameter<double>("vehicleHeight", ctx_.vehicle_height_);
  node_->declare_parameter<int>("voxelPointUpdateThre", ctx_.voxel_point_update_thre_);
  node_->declare_parameter<double>("voxelTimeUpdateThre", ctx_.voxel_time_update_thre_);
  node_->declare_parameter<double>("minRelZ", ctx_.min_rel_z_);
  node_->declare_parameter<double>("maxRelZ", ctx_.max_rel_z_);
  node_->declare_parameter<double>("disRatioZ", ctx_.dis_ratio_z_);

  node_->get_parameter("scanVoxelSize", ctx_.scan_voxel_size_);
  node_->get_parameter("decayTime", ctx_.decay_time_);
  node_->get_parameter("noDecayDis", ctx_.no_decay_dis_);
  node_->get_parameter("clearingDis", ctx_.clearing_dis_);
  node_->get_parameter("useSorting", ctx_.use_sorting_);
  node_->get_parameter("quantileZ", ctx_.quantile_z_);
  node_->get_parameter("considerDrop", ctx_.consider_drop_);
  node_->get_parameter("limitGroundLift", ctx_.limit_ground_lift_);
  node_->get_parameter("maxGroundLift", ctx_.max_ground_lift_);
  node_->get_parameter("clearDyObs", ctx_.clear_dy_obs_);
  node_->get_parameter("minDyObsDis", ctx_.min_dy_obs_dis_);
  node_->get_parameter("minDyObsAngle", ctx_.min_dy_obs_angle_);
  node_->get_parameter("minDyObsRelZ", ctx_.min_dy_obs_rel_z_);
  node_->get_parameter("absDyObsRelZThre", ctx_.abs_dy_obs_rel_z_thre_);
  node_->get_parameter("minDyObsVFOV", ctx_.min_dy_obs_vfov_);
  node_->get_parameter("maxDyObsVFOV", ctx_.max_dy_obs_vfov_);
  node_->get_parameter("minDyObsPointNum", ctx_.min_dy_obs_point_num_);
  node_->get_parameter("noDataObstacle", ctx_.no_data_obstacle_);
  node_->get_parameter("noDataBlockSkipNum", ctx_.no_data_block_skip_num_);
  node_->get_parameter("minBlockPointNum", ctx_.min_block_point_num_);
  node_->get_parameter("vehicleHeight", ctx_.vehicle_height_);
  node_->get_parameter("voxelPointUpdateThre", ctx_.voxel_point_update_thre_);
  node_->get_parameter("voxelTimeUpdateThre", ctx_.voxel_time_update_thre_);
  node_->get_parameter("minRelZ", ctx_.min_rel_z_);
  node_->get_parameter("maxRelZ", ctx_.max_rel_z_);
  node_->get_parameter("disRatioZ", ctx_.dis_ratio_z_);

  sub_odometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "lidar_odometry", 5,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        double roll, pitch, yaw;
        auto& q = msg->pose.pose.orientation;
        tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w))
            .getRPY(roll, pitch, yaw);
        ctx_.onOdometry(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z, roll, pitch, yaw);
      });

  sub_laser_cloud_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "registered_scan", 5,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*msg, *pcl_cloud);
        double t = rclcpp::Time(msg->header.stamp).seconds();
        ctx_.onLaserCloud(pcl_cloud, t);
      });

  sub_joystick_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 5,
      [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        ctx_.onJoystick(msg->buttons[5] > 0.5);
      });

  sub_clearing_ = node_->create_subscription<std_msgs::msg::Float32>(
      "map_clearing", 5,
      [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
        ctx_.onClearing(msg->data);
      });

  pub_terrain_map_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map", 2);

  ctx_.down_size_filter_.setLeafSize(
      ctx_.scan_voxel_size_, ctx_.scan_voxel_size_, ctx_.scan_voxel_size_);
}

bool TerrainAnalysis::processOnce() {
  rclcpp::spin_some(node_->get_node_base_interface());
  if (!ctx_.new_laser_cloud_) {
    return rclcpp::ok();
  }
  TerrainAlgorithm::run(ctx_);

  sensor_msgs::msg::PointCloud2 terrainCloud2;
  pcl::toROSMsg(ctx_.terrainCloudElev(), terrainCloud2);
  terrainCloud2.header.stamp =
      rclcpp::Time(static_cast<uint64_t>(ctx_.laser_cloud_time_ * 1e9));
  terrainCloud2.header.frame_id = "odom";
  pub_terrain_map_->publish(terrainCloud2);

  return rclcpp::ok();
}
