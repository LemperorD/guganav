// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#include "terrain_analysis_ext/terrain_analysis.hpp"
#include "terrain_analysis_ext/core/algorithm.hpp"

#include "pcl_conversions/pcl_conversions.h"

TerrainAnalysisExtNode::TerrainAnalysisExtNode(rclcpp::Node* node)
    : node_(node) {
  node_->declare_parameter<double>("localTerrainMapRadius",
                                   local_terrain_map_radius_);
  node_->get_parameter("localTerrainMapRadius", local_terrain_map_radius_);

  pub_terrain_map_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "terrain_map_ext", 2);

  sub_odometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "lidar_odometry", 5, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        state_.vehicle_x = msg->pose.pose.position.x;
        state_.vehicle_y = msg->pose.pose.position.y;
      });

  sub_local_terrain_ =
      node_->create_subscription<sensor_msgs::msg::PointCloud2>(
          "terrain_map", 2,
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pcl::fromROSMsg(*msg, *cloud);
            state_.laser_cloud_time = rclcpp::Time(msg->header.stamp).seconds();
            state_.terrain_cloud_local->clear();
            *state_.terrain_cloud_local = *cloud;
            state_.has_new_laser_cloud = true;
          });
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                    [this]() { processOnce(); });
}

bool TerrainAnalysisExtNode::processOnce() {
  if (!state_.has_new_laser_cloud) {
    return rclcpp::ok();
  }

  terrain_analysis_ext::algorithm::runExt(local_terrain_map_radius_, state_);
  publishPointCloud();
  return rclcpp::ok();
}

void TerrainAnalysisExtNode::publishPointCloud() {
  sensor_msgs::msg::PointCloud2 terraincloud2;
  pcl::toROSMsg(*state_.terrain_cloud_elev, terraincloud2);

  terraincloud2.header.stamp = rclcpp::Time(
      static_cast<int64_t>(state_.laser_cloud_time * 1e9));
  terraincloud2.header.frame_id = "odom";

  pub_terrain_map_->publish(terraincloud2);
}
