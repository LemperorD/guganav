#include "terrain_analysis/terrain_analysis.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nodeptr = rclcpp::Node::make_shared("terrainAnalysisExt");

  TerrainAnalysis ext(nodeptr.get());

  // ext-specific defaults (overridden by ROS params if declared)
  auto& cfg = ext.context_.cfg;
  cfg.terrain_voxel_size = 2.0;
  cfg.planar_voxel_size = 0.4;
  cfg.scan_voxel_size = 0.1;
  cfg.decay_time = 10.0;
  cfg.no_decay_distance = 0.0;
  cfg.use_sorting = false;
  cfg.max_relative_z = 1.0;
  cfg.check_terrain_connectivity = true;
  cfg.ceiling_filter_threshold = 2.0;
  cfg.terrain_connectivity_threshold = 0.5;
  cfg.terrain_under_vehicle = -0.75;
  cfg.local_terrain_map_radius = 4.0;

  ext.initialize("terrain_map_ext");

  // subscribe to terrain_map (from terrain_analysis) for local merge
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
