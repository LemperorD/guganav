#include "terrain_analysis/terrain_analysis.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nodeptr = rclcpp::Node::make_shared("terrainAnalysisExt");

  TerrainAnalysis ext(nodeptr.get());
  ext.initialize("terrain_map_ext", true, TerrainConfig::extDefaults());

  rclcpp::Rate rate(100);
  while (ext.processOnce()) {
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
