// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#include "terrain_analysis/terrain_analysis_node.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace terrain_analysis {

  namespace {

    /**
     * @brief 启动时提示障碍输出的现行高度带。
     *
     * computeHeightMap 只保留 minObstacleHeight <= h < ceilingClearance 的点，
     * 其中 h 是距局部地面的高度。两个边界都随车而异、也没有编译期约束，
     * 所以在启动时打印，便于确认节点实际加载的值。
     */
    void logHeightParams(const PerFrameHeightConfig& config) {
      RCLCPP_INFO(rclcpp::get_logger("terrain_analysis"),
                  "障碍输出高度带：%.3f <= h < %.3f m（距局部地面；下界为地面带"
                  "死区，上界按车高 + 100 mm 设定）",
                  config.min_obstacle_height, config.ceiling_clearance);
    }

  }  // namespace

  TerrainAnalysis::TerrainAnalysis(const rclcpp::NodeOptions& options)
      : Node("terrain_analysis", options) {
    // 参数按两半各自的读取范围分发：前半段要叶尺寸、衰减与接收带，后半段要地面
    // 估计、输出带与平面网格。minRelZ 两半都用（用途不同），故填两次。
    PersistentVoxelConfig& voxel_config = persistent_voxel_map_.config();
    PerFrameHeightConfig& height_config = per_frame_height_map_.config();

    voxel_config.scan_voxel_size = declare_parameter(
        "scanVoxelSize", voxel_config.scan_voxel_size);
    voxel_config.scan_voxel_size_z = declare_parameter(
        "scanVoxelSizeZ", voxel_config.scan_voxel_size_z);
    voxel_config.decay_time = declare_parameter("decayTime",
                                                voxel_config.decay_time);
    voxel_config.no_decay_distance = declare_parameter(
        "noDecayDis", voxel_config.no_decay_distance);
    voxel_config.max_relative_z = declare_parameter(
        "maxRelZ", voxel_config.max_relative_z);
    voxel_config.distance_ratio_z = declare_parameter(
        "disRatioZ", voxel_config.distance_ratio_z);

    height_config.use_sorting = declare_parameter("useSorting",
                                                  height_config.use_sorting);
    height_config.quantile_z = declare_parameter("quantileZ",
                                                 height_config.quantile_z);
    height_config.consider_drop = declare_parameter(
        "considerDrop", height_config.consider_drop);
    height_config.limit_ground_lift = declare_parameter(
        "limitGroundLift", height_config.limit_ground_lift);
    height_config.max_ground_lift = declare_parameter(
        "maxGroundLift", height_config.max_ground_lift);
    height_config.min_block_point_num = declare_parameter(
        "minBlockPointNum", height_config.min_block_point_num);
    height_config.min_obstacle_height = declare_parameter(
        "minObstacleHeight", height_config.min_obstacle_height);
    height_config.ceiling_clearance = declare_parameter(
        "ceilingClearance", height_config.ceiling_clearance);
    height_config.ground_floor_z = declare_parameter(
        "groundFloorZ", height_config.ground_floor_z);

    // 同一个参数进入两半：前半段用它定义接收带下沿，后半段用它挡穿透点。
    const double min_relative_z = declare_parameter(
        "minRelZ", voxel_config.min_relative_z);
    voxel_config.min_relative_z = min_relative_z;
    height_config.min_relative_z = min_relative_z;

    logHeightParams(height_config);

    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "lidar_odometry", 5,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          // terrain 订阅的是雷达里程计，位置直接采用，不在这里做坐标变换。
          // 姿态没有被任何阶段使用（地面估计与障碍判定只用位置与点云），
          // 因此不再解析四元数。
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
          last_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
          persistent_voxel_map_.ingest(*cloud, lidar_position_, last_stamp_);
        });

    pub_terrain_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "terrain_map", 2);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     [this]() { processOnce(); });
  }

  bool TerrainAnalysis::processOnce() {
    if (!persistent_voxel_map_.hasPendingFrame()) {
      return rclcpp::ok();
    }

    // 前半段：累积本帧观测并维护体素地图；采集窗口内的累积点云交给后半段。
    persistent_voxel_map_.update();
    persistent_voxel_map_.collectCloud(*collected_cloud_);
    // 锚点用前半段记下的那份，避免节点再存一份、两处不同步。
    per_frame_height_map_.compute(*collected_cloud_,
                                  persistent_voxel_map_.lidarPosition());
    publishPointCloud();
    return rclcpp::ok();
  }

  void TerrainAnalysis::publishPointCloud() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(obstacleCloud(), message);
    message.header.stamp = rclcpp::Time(
        static_cast<int64_t>(last_stamp_ * 1e9));
    message.header.frame_id = "odom";
    pub_terrain_map_->publish(message);
  }

}  // namespace terrain_analysis
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis::TerrainAnalysis)
