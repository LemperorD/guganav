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

#ifndef PB_NAV2_PLUGINS__BEHAVIORS__ESCAPE_OBSTACLE_LAYER_HPP_
#define PB_NAV2_PLUGINS__BEHAVIORS__ESCAPE_OBSTACLE_LAYER_HPP_

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "guga_interfaces/action/escape_obstacle_layer.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb_nav2_behaviors
{

using EscapeObstacleLayerAction = guga_interfaces::action::EscapeObstacleLayer;

/**
 * @class pb_nav2_behaviors::EscapeObstacleLayer
 * @brief 从膨胀层的硬区（cost >= 253）里挪出来，走多远由几何决定而不是固定距离。
 *
 * 与 nav2_behaviors/BackUp 的区别：
 *   - 目标不是"退 X 米"，而是"直到车心所在栅格脱离 253 并再多走 exit_clearance"；
 *   - 距离只在两种情况下出现：搜索上限 max_distance（安全阀）、时间上限 time_allowance；
 *   - 方向在 onRun 里选一次，之后按 rescan_period 闭环重选（带角度迟滞），
 *     因此代价地图在执行中变化（动态障碍、footprint clearing、局部图滚动）也能跟上；
 *   - 起点已经在 253 里时，它不会像 BackUp 那样在第一步就被碰撞检查判死。
 *
 * 结束条件：
 *   - 成功：车心 cost < 253，且自脱离点起又走了 exit_clearance；
 *   - 失败：无可行方向 / 触达 max_distance / 触达 time_allowance / 位姿或代价地图不可用。
 *     失败时不谎报成功，交给行为树里的下一个恢复项。
 */
class EscapeObstacleLayer : public nav2_behaviors::TimedBehavior<EscapeObstacleLayerAction>
{
public:
  EscapeObstacleLayer() = default;

  void onConfigure() override;
  void onCleanup() override;
  nav2_behaviors::Status onRun(
    const std::shared_ptr<const EscapeObstacleLayerAction::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

  /// 方向搜索用的参数，单独抽出来便于单测
  struct EscapeConfig
  {
    double max_distance{1.2};              ///< 射线搜索与行走的硬上限
    double exit_clearance{0.1};            ///< 脱离 253 后再多走的距离
    double lethal_traverse_distance{0.5};  ///< 起点在 254 里时允许穿越真障碍的最大厚度
    double footprint_radius{0.4};          ///< 终点足迹安全检查半径
    double angle_step{0.09817477042468103};  ///< pi / 32
  };

  struct EscapeDirection
  {
    double angle;             ///< 代价地图坐标系下的方向角
    double exit_distance;     ///< 射线第一次离开 253 的距离
    double command_distance;  ///< 建议行走距离（exit + clearance，被 max_distance 截断）
  };

  /**
   * @brief 在 360° 里找一条"最短距离离开 253"且终点足迹不压真障碍的方向。
   * @return 无解时返回 std::nullopt；起点不在 253 里时也返回 std::nullopt（调用方应视为 no-op）。
   */
  static std::optional<EscapeDirection> findEscapeDirection(
    const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
    const EscapeConfig & config);

protected:
  EscapeConfig makeConfig() const;

  /// 从代价地图服务取一帧快照；失败返回 std::nullopt
  std::optional<nav2_msgs::msg::Costmap> fetchCostmap();

private:
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;

  // 参数
  std::string service_name_;
  std::string costmap_frame_;
  double default_max_distance_{1.2};
  double default_speed_{0.3};
  double default_time_allowance_{8.0};
  double exit_clearance_{0.1};
  double lethal_traverse_distance_{0.5};
  double footprint_radius_{0.4};
  double angle_step_{0.09817477042468103};
  double rescan_period_{0.2};
  double angle_hysteresis_{0.3490658503988659};  // 20°

  // 运行时状态
  geometry_msgs::msg::PoseStamped start_pose_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  double max_distance_{1.2};
  double speed_{0.3};
  double time_allowance_{8.0};
  double direction_{0.0};
  double traveled_{0.0};
  double escape_start_distance_{0.0};
  bool escaped_{false};
  bool noop_{false};
};

}  // namespace pb_nav2_behaviors

#endif  // PB_NAV2_PLUGINS__BEHAVIORS__ESCAPE_OBSTACLE_LAYER_HPP_
