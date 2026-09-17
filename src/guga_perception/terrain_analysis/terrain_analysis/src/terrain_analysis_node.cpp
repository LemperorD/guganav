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
     * @brief 启动时提示障碍输出的现行高度上界。
     *
     * computeHeightMap 只保留 minObstacleHeight <= h < ceilingClearance 的点，
     * 其中 h 是距局部地面的高度。两个边界都随车而异、也没有编译期约束，
     * 所以在启动时打印，便于确认节点实际加载的值。
     */
    void logHeightParams(const TerrainConfig& config) {
      RCLCPP_INFO(rclcpp::get_logger("terrain_analysis"),
                  "障碍输出高度带：%.3f <= h < %.3f m（距局部地面；下界为地面带"
                  "死区，上界按车高 + 100 mm 设定）",
                  config.min_obstacle_height, config.ceiling_clearance);
    }

  }  // namespace

  TerrainAnalysis::TerrainAnalysis(const rclcpp::NodeOptions& options)
      : Node("terrain_analysis", options) {
    TerrainConfig& config = pipeline_.config();
    config.scan_voxel_size = declare_parameter("scanVoxelSize",
                                               config.scan_voxel_size);
    config.scan_voxel_size_z = declare_parameter("scanVoxelSizeZ",
                                                 config.scan_voxel_size_z);
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
    config.min_block_point_num = declare_parameter("minBlockPointNum",
                                                   config.min_block_point_num);
    config.min_obstacle_height = declare_parameter("minObstacleHeight",
                                                   config.min_obstacle_height);
    config.ceiling_clearance = declare_parameter("ceilingClearance",
                                                 config.ceiling_clearance);
    config.min_relative_z = declare_parameter("minRelZ", config.min_relative_z);
    config.max_relative_z = declare_parameter("maxRelZ", config.max_relative_z);
    config.ground_floor_z = declare_parameter("groundFloorZ",
                                              config.ground_floor_z);
    config.distance_ratio_z = declare_parameter("disRatioZ",
                                                config.distance_ratio_z);

    logHeightParams(config);

    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "lidar_odometry", 5,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          double roll{};
          double pitch{};
          double yaw{};
          const auto& q = msg->pose.pose.orientation;
          tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w))
              .getRPY(roll, pitch, yaw);
          pipeline_.ingestOdometry(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z, roll, pitch, yaw);
        });

    sub_laser_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "registered_scan", 5,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
          pcl::fromROSMsg(*msg, *cloud);
          pipeline_.ingestLaserCloud(cloud,
                                     rclcpp::Time(msg->header.stamp).seconds());
        });

    pub_terrain_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "terrain_map", 2);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     [this]() { processOnce(); });
  }

  bool TerrainAnalysis::processOnce() {
    if (!pipeline_.hasPendingCloud()) {
      return rclcpp::ok();
    }

    // 先把本帧数据分发给体素地图（跨帧持久），再跑逐帧阶段。
    voxel_map_.update(pipeline_.croppedCloud(), pipeline_.lidarPosition(),
                      pipeline_.elapsedSeconds(), pipeline_.config());
    voxel_map_.collectCloud(pipeline_.collectedCloud());
    pipeline_.runStages();
    publishPointCloud();
    return rclcpp::ok();
  }

  void TerrainAnalysis::publishPointCloud() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(pipeline_.terrainCloudElev(), message);
    message.header.stamp = rclcpp::Time(
        static_cast<int64_t>(pipeline_.laserCloudTime() * 1e9));
    message.header.frame_id = "odom";
    pub_terrain_map_->publish(message);
  }

}  // namespace terrain_analysis
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis::TerrainAnalysis)