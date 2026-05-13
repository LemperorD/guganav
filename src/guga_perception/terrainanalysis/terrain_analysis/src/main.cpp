#include "terrain_analysis/terrain_analysis.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("terrainAnalysis");

  TerrainAnalysis terrain(nh.get());
  terrain.initialize();

  rclcpp::Rate rate(100);
  while (terrain.processOnce()) {
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
