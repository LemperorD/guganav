#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "guga_decision_common/semantic_map.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<guga_decision_common::SemanticMap>());
  rclcpp::shutdown();
  return 0;
}
