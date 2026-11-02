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

#ifndef PB_NAV2_PLUGINS__BT__ESCAPE_OBSTACLE_LAYER_ACTION_HPP_
#define PB_NAV2_PLUGINS__BT__ESCAPE_OBSTACLE_LAYER_ACTION_HPP_

#include <string>

#include "guga_interfaces/action/escape_obstacle_layer.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace pb_nav2_behaviors
{

/**
 * @brief 行为树节点：调用 behavior_server 上名为 escape_obstacle_layer 的行为。
 *
 * 端口：
 *   max_distance   —— 行走/搜索上限（m），0 表示用行为节点的 default_max_distance
 *   speed          —— 行走速度（m/s），0 表示用 default_speed
 *   time_allowance —— 时间上限（s），0 表示用 default_time_allowance
 */
class EscapeObstacleLayerAction
: public nav2_behavior_tree::BtActionNode<guga_interfaces::action::EscapeObstacleLayer>
{
public:
  EscapeObstacleLayerAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts();
};

}  // namespace pb_nav2_behaviors

#endif  // PB_NAV2_PLUGINS__BT__ESCAPE_OBSTACLE_LAYER_ACTION_HPP_
