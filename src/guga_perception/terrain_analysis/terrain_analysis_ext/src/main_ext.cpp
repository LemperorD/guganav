#include "terrain_analysis_ext/terrain_analysis.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nodeptr = rclcpp::Node::make_shared("terrainAnalysisExt");

  TerrainAnalysisExtNode ext(nodeptr.get());
  rclcpp::spin(nodeptr);
  rclcpp::shutdown();

  return 0;
}
