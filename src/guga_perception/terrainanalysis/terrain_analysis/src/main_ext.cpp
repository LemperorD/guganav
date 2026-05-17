#include "terrain_analysis/terrain_analysis.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nodeptr = rclcpp::Node::make_shared("terrainAnalysisExt");

  TerrainAnalysis ext(nodeptr.get());

  auto& cfg = ext.context_.cfg;
  cfg.terrain_voxel_size = 2.0;
  cfg.planar_voxel_size = 0.4;
  cfg.scan_voxel_size = 0.1;
  cfg.decay_time = 10.0;
  cfg.no_decay_distance = 0.0;
  cfg.use_sorting = false;
  cfg.max_relative_z = 1.0;
  cfg.local_terrain_map_radius = 4.0;

  ext.initialize("terrain_map_ext", true);

  auto sub_odometry = nodeptr->create_subscription<nav_msgs::msg::Odometry>(
      "lidar_odometry", 5,
      [&ext](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        double roll{};
        double pitch{};
        double yaw{};
        const auto& q = msg->pose.pose.orientation;
        tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w))
            .getRPY(roll, pitch, yaw);
        ext.context_.onOdometry(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z, roll, pitch, yaw);
      });

  auto sub_local_terrain =
      nodeptr->create_subscription<sensor_msgs::msg::PointCloud2>(
          "terrain_map", 2,
          [&ext](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            auto cloud =
                std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pcl::fromROSMsg(*msg, *cloud);
            ext.context_.onLocalTerrainCloud(cloud);
          });

  rclcpp::Rate rate(100);
  while (ext.processOnce()) {
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
