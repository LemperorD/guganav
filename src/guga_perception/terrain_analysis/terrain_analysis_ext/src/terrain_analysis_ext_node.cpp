// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#include "terrain_analysis_ext/terrain_analysis.hpp"
#include "terrain_analysis_ext/core/algorithm.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include <algorithm>
#include <cmath>

#include "rclcpp_components/register_node_macro.hpp"

namespace terrain_analysis_ext {

  TerrainAnalysisExtNode::TerrainAnalysisExtNode(
      const rclcpp::NodeOptions& options)
      : Node("terrain_analysis_ext", options) {
    this->declare_parameter<double>("localTerrainMapRadius",
                                    local_terrain_map_radius_);
    this->get_parameter("localTerrainMapRadius", local_terrain_map_radius_);

    pub_terrain_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "terrain_map_ext", 2);

    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "lidar_odometry", 5,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          state_.vehicle_x = msg->pose.pose.position.x;
          state_.vehicle_y = msg->pose.pose.position.y;
        });

    sub_local_terrain_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "terrain_map", 2,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
              // 记录回调到达时刻：含 DDS/intra-process 传输延迟，
              // 与发布时刻的差值即"接收→处理→发布"端到端延迟
              receive_wall_ = std::chrono::steady_clock::now();
              auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
              pcl::fromROSMsg(*msg, *cloud);
              state_.laser_cloud_time =
                  rclcpp::Time(msg->header.stamp).seconds();
              state_.terrain_cloud_local->clear();
              *state_.terrain_cloud_local = *cloud;
              state_.has_new_laser_cloud = true;
            });
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
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

    // ── 延迟插桩：接收（terrain_map 回调）→ 发布（terrain_map_ext）──
    // 组件化前后对比：差值主要来自 DDS 传输 vs intra-process 直传
    if (receive_wall_.time_since_epoch().count() == 0) {
      return;  // 尚未收到过 terrain_map
    }
    const double latency_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now()
                                  - receive_wall_)
                                  .count();
    latency_ms_.push_back(latency_ms);
    if (latency_ms_.size() >= kBenchmarkWindow) {
      printLatencyStats();
      latency_ms_.clear();
    }
  }

  void TerrainAnalysisExtNode::printLatencyStats() {
    std::vector<double> sorted = latency_ms_;
    std::sort(sorted.begin(), sorted.end());
    const size_t n = sorted.size();
    const double p50 = sorted[n / 2];
    const double p95 = sorted[static_cast<size_t>(n * 0.95)];
    const double max = sorted.back();
    RCLCPP_INFO_STREAM(this->get_logger(), "[benchmark] latency_ms count="
                                               << n << " p50=" << p50 << " p95="
                                               << p95 << " max=" << max);
  }

}  // namespace terrain_analysis_ext

RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis_ext::TerrainAnalysisExtNode)
