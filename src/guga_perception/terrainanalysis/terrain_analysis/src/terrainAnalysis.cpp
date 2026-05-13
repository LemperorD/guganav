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

void TerrainAnalysis::initialize() {
  node_->declare_parameter<double>("scanVoxelSize", context_.scan_voxel_size_);
  node_->declare_parameter<double>("decayTime", context_.decay_time_);
  node_->declare_parameter<double>("noDecayDis", context_.no_decay_dis_);
  node_->declare_parameter<double>("clearingDis", context_.clearing_dis_);
  node_->declare_parameter<bool>("useSorting", context_.use_sorting_);
  node_->declare_parameter<double>("quantileZ", context_.quantile_z_);
  node_->declare_parameter<bool>("considerDrop", context_.consider_drop_);
  node_->declare_parameter<bool>("limitGroundLift",
                                 context_.limit_ground_lift_);
  node_->declare_parameter<double>("maxGroundLift", context_.max_ground_lift_);
  node_->declare_parameter<bool>("clearDyObs", context_.clear_dy_obs_);
  node_->declare_parameter<double>("minDyObsDis", context_.min_dy_obs_dis_);
  node_->declare_parameter<double>("minDyObsAngle", context_.min_dy_obs_angle_);
  node_->declare_parameter<double>("minDyObsRelZ", context_.min_dy_obs_rel_z_);
  node_->declare_parameter<double>("absDyObsRelZThre",
                                   context_.abs_dy_obs_rel_z_thre_);
  node_->declare_parameter<double>("minDyObsVFOV", context_.min_dy_obs_vfov_);
  node_->declare_parameter<double>("maxDyObsVFOV", context_.max_dy_obs_vfov_);
  node_->declare_parameter<int>("minDyObsPointNum",
                                context_.min_dy_obs_point_num_);
  node_->declare_parameter<bool>("noDataObstacle", context_.no_data_obstacle_);
  node_->declare_parameter<int>("noDataBlockSkipNum",
                                context_.no_data_block_skip_num_);
  node_->declare_parameter<int>("minBlockPointNum",
                                context_.min_block_point_num_);
  node_->declare_parameter<double>("vehicleHeight", context_.vehicle_height_);
  node_->declare_parameter<int>("voxelPointUpdateThre",
                                context_.voxel_point_update_thre_);
  node_->declare_parameter<double>("voxelTimeUpdateThre",
                                   context_.voxel_time_update_thre_);
  node_->declare_parameter<double>("minRelZ", context_.min_rel_z_);
  node_->declare_parameter<double>("maxRelZ", context_.max_rel_z_);
  node_->declare_parameter<double>("disRatioZ", context_.dis_ratio_z_);

  node_->get_parameter("scanVoxelSize", context_.scan_voxel_size_);
  node_->get_parameter("decayTime", context_.decay_time_);
  node_->get_parameter("noDecayDis", context_.no_decay_dis_);
  node_->get_parameter("clearingDis", context_.clearing_dis_);
  node_->get_parameter("useSorting", context_.use_sorting_);
  node_->get_parameter("quantileZ", context_.quantile_z_);
  node_->get_parameter("considerDrop", context_.consider_drop_);
  node_->get_parameter("limitGroundLift", context_.limit_ground_lift_);
  node_->get_parameter("maxGroundLift", context_.max_ground_lift_);
  node_->get_parameter("clearDyObs", context_.clear_dy_obs_);
  node_->get_parameter("minDyObsDis", context_.min_dy_obs_dis_);
  node_->get_parameter("minDyObsAngle", context_.min_dy_obs_angle_);
  node_->get_parameter("minDyObsRelZ", context_.min_dy_obs_rel_z_);
  node_->get_parameter("absDyObsRelZThre", context_.abs_dy_obs_rel_z_thre_);
  node_->get_parameter("minDyObsVFOV", context_.min_dy_obs_vfov_);
  node_->get_parameter("maxDyObsVFOV", context_.max_dy_obs_vfov_);
  node_->get_parameter("minDyObsPointNum", context_.min_dy_obs_point_num_);
  node_->get_parameter("noDataObstacle", context_.no_data_obstacle_);
  node_->get_parameter("noDataBlockSkipNum", context_.no_data_block_skip_num_);
  node_->get_parameter("minBlockPointNum", context_.min_block_point_num_);
  node_->get_parameter("vehicleHeight", context_.vehicle_height_);
  node_->get_parameter("voxelPointUpdateThre",
                       context_.voxel_point_update_thre_);
  node_->get_parameter("voxelTimeUpdateThre", context_.voxel_time_update_thre_);
  node_->get_parameter("minRelZ", context_.min_rel_z_);
  node_->get_parameter("maxRelZ", context_.max_rel_z_);
  node_->get_parameter("disRatioZ", context_.dis_ratio_z_);

  sub_odometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "lidar_odometry", 5, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        double roll, pitch, yaw;
        auto& q = msg->pose.pose.orientation;
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
      "terrain_map", 2);

  context_.down_size_filter_.setLeafSize(context_.scan_voxel_size_,
                                         context_.scan_voxel_size_,
                                         context_.scan_voxel_size_);
}

bool TerrainAnalysis::processOnce() {
  rclcpp::spin_some(node_->get_node_base_interface());
  if (!context_.new_laser_cloud_) {
    return rclcpp::ok();
  }
  TerrainAlgorithm::run(context_);

  sensor_msgs::msg::PointCloud2 terrainCloud2;
  pcl::toROSMsg(context_.terrainCloudElev(), terrainCloud2);
  terrainCloud2.header.stamp = rclcpp::Time(
      static_cast<uint64_t>(context_.laser_cloud_time_ * 1e9));
  terrainCloud2.header.frame_id = "odom";
  pub_terrain_map_->publish(terrainCloud2);

  return rclcpp::ok();
}
