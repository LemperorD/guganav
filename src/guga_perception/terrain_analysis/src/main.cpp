#include "terrain_analysis/terrain_analysis.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nodeptr = rclcpp::Node::make_shared("terrainAnalysis");

  TerrainAnalysis terrain(nodeptr.get());
  terrain.initialize();

  rclcpp::Rate rate(100);
  while (terrain.processOnce()) {
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
