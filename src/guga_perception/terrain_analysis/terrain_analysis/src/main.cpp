#include "terrain_analysis/terrain_analysis_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nodeptr = rclcpp::Node::make_shared("terrainAnalysis");

  TerrainAnalysis terrain(nodeptr.get());
  rclcpp::spin(nodeptr);

  rclcpp::shutdown();
  return 0;
}
