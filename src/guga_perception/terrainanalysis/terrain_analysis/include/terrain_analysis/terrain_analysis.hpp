#pragma once

#include "terrain_analysis/core/context.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

class TerrainAnalysis {
public:
  explicit TerrainAnalysis(rclcpp::Node* node);
  ~TerrainAnalysis();

  TerrainAnalysis(const TerrainAnalysis&) = delete;
  TerrainAnalysis& operator=(const TerrainAnalysis&) = delete;
  TerrainAnalysis(TerrainAnalysis&&) = delete;
  TerrainAnalysis& operator=(TerrainAnalysis&&) = delete;

  void initialize();
  bool processOnce();

  const pcl::PointCloud<pcl::PointXYZI>& terrainCloudElev() const {
    return ctx_.terrainCloudElev();
  }

  TerrainAnalysisContext ctx_;

private:
  rclcpp::Node* node_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_laser_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joystick_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_clearing_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_terrain_map_;
};
