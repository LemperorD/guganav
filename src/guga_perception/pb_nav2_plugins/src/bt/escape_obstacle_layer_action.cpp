// Copyright 2026 HaoZhen Liu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_nav2_plugins/bt/escape_obstacle_layer_action.hpp"

#include <memory>

namespace pb_nav2_behaviors
{

EscapeObstacleLayerAction::EscapeObstacleLayerAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<guga_interfaces::action::EscapeObstacleLayer>(
    xml_tag_name, action_name, conf)
{
}

void EscapeObstacleLayerAction::on_tick()
{
  double max_distance = 1.2;
  double speed = 0.3;
  double time_allowance = 8.0;
  getInput("max_distance", max_distance);
  getInput("speed", speed);
  getInput("time_allowance", time_allowance);

  goal_.max_distance = static_cast<float>(max_distance);
  goal_.speed = static_cast<float>(speed);
  goal_.time_allowance = static_cast<float>(time_allowance);
}

BT::PortsList EscapeObstacleLayerAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("max_distance", 1.2, "Maximum distance to travel while escaping"),
    BT::InputPort<double>("speed", 0.3, "Escape speed"),
    BT::InputPort<double>("time_allowance", 8.0, "Time budget in seconds"),
  });
}

}  // namespace pb_nav2_behaviors

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<pb_nav2_behaviors::EscapeObstacleLayerAction>(
      name, "escape_obstacle_layer", config);
  };

  factory.registerBuilder<pb_nav2_behaviors::EscapeObstacleLayerAction>(
    "EscapeObstacleLayer", builder);
}
