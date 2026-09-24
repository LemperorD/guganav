// 主线位置：ROS 回调收帧 → processOnce() 跑一帧并发布三条点云。
//
// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#include "terrain_analysis/terrain_analysis_node.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace terrain_analysis {

  namespace {

    // 高度带两个边界都随车而异，启动时打印实际生效的值。
    void logHeightParams(const PerFrameHeightConfig& config) {
      RCLCPP_INFO(rclcpp::get_logger("terrain_analysis"),
                  "障碍输出高度带：%.3f <= h < %.3f m（距局部地面；下界为地面带"
                  "死区，上界按车高 + 100 mm 设定）",
                  config.min_obstacle_height, config.ceiling_clearance);
    }

  }  // namespace

  PersistentVoxelConfig TerrainAnalysis::getVoxelConfig() {
    PersistentVoxelConfig config;
    config.scan_voxel_size = declare_parameter("scanVoxelSize",
                                               config.scan_voxel_size);
    config.scan_voxel_size_z = declare_parameter("scanVoxelSizeZ",
                                                 config.scan_voxel_size_z);
    config.decay_time = declare_parameter("decayTime", config.decay_time);
    config.max_relative_z = declare_parameter("maxRelZ", config.max_relative_z);
    config.distance_ratio_z = declare_parameter("disRatioZ",
                                                config.distance_ratio_z);

    config.min_relative_z = declare_parameter("minRelZ", config.min_relative_z);
    return config;
  }

  PerFrameHeightConfig TerrainAnalysis::getHeightConfig(double min_relative_z) {
    PerFrameHeightConfig config;
    config.use_sorting = declare_parameter("useSorting", config.use_sorting);
    config.quantile_z = declare_parameter("quantileZ", config.quantile_z);
    config.consider_drop = declare_parameter("considerDrop",
                                             config.consider_drop);
    config.limit_ground_lift = declare_parameter("limitGroundLift",
                                                 config.limit_ground_lift);
    config.max_ground_lift = declare_parameter("maxGroundLift",
                                               config.max_ground_lift);
    config.min_block_point_num = declare_parameter("minBlockPointNum",
                                                   config.min_block_point_num);
    config.min_obstacle_height = declare_parameter("minObstacleHeight",
                                                   config.min_obstacle_height);
    config.ceiling_clearance = declare_parameter("ceilingClearance",
                                                 config.ceiling_clearance);
    config.ground_floor_z = declare_parameter("groundFloorZ",
                                              config.ground_floor_z);

    config.min_relative_z = min_relative_z;

    logHeightParams(config);
    return config;
  }

  TerrainAnalysis::TerrainAnalysis(const rclcpp::NodeOptions& options)
      : Node("terrain_analysis", options),
        voxel_config_(getVoxelConfig()),
        height_config_(getHeightConfig(voxel_config_.min_relative_z)),
        persistent_voxel_map_(voxel_config_),
        per_frame_height_map_(height_config_) {
    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "lidar_odometry", 5,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          // 姿态没有任何阶段使用。
          // 位置直接采用，不做坐标变换。
          const auto& position = msg->pose.pose.position;
          lidar_position_.x = position.x;
          lidar_position_.y = position.y;
          lidar_position_.z = position.z;
        });

    sub_laser_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "registered_scan", 5,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
          pcl::fromROSMsg(*msg, *cloud);
          // 每次只接收限定范围内的点云
          receiveFrame(*cloud, lidar_position_,
                       rclcpp::Time(msg->header.stamp).seconds());
        });

    pub_terrain_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "terrain_map", 2);
    pub_terrain_returns_current_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "terrain_returns_current", 2);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     [this]() { processOnce(); });
  }

  void TerrainAnalysis::receiveFrame(
      const pcl::PointCloud<pcl::PointXYZI>& cloud,
      const guga_common::Point3d& lidar_position, double timestamp_sec) {
    lidar_position_ = lidar_position;
    last_stamp_ = timestamp_sec;
    persistent_voxel_map_.receiveFrame(cloud, lidar_position_, last_stamp_);
  }

  bool TerrainAnalysis::processFrame(
      const pcl::PointCloud<pcl::PointXYZI>& cloud,
      const guga_common::Point3d& lidar_position, double timestamp_sec) {
    receiveFrame(cloud, lidar_position, timestamp_sec);
    return processOnce();
  }

  bool TerrainAnalysis::processOnce() {
    if (!persistent_voxel_map_.hasPendingFrame()) {
      return rclcpp::ok();
    }

    persistent_voxel_map_.update();
    persistent_voxel_map_.collectCloud(*collected_cloud_);

    // 使用上文雷达位置,确保同步
    const guga_common::Point3d& lidar_position =
        persistent_voxel_map_.lidarPosition();

    per_frame_height_map_.compute(*collected_cloud_, lidar_position);
    per_frame_height_map_.computeFrameReturns(
        persistent_voxel_map_.frameCloud(), lidar_position);
    publishClouds();
    return rclcpp::ok();
  }

  void TerrainAnalysis::publishClouds() {
    publishCloud(pub_terrain_map_, obstacleCloud());
    publishCloud(pub_terrain_returns_current_, frameReturnCloud());
  }

  void TerrainAnalysis::publishCloud(
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
          publisher,
      const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.stamp = rclcpp::Time(
        static_cast<int64_t>(last_stamp_ * 1e9));
    message.header.frame_id = "odom";
    publisher->publish(message);
  }

}  // namespace terrain_analysis
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis::TerrainAnalysis)