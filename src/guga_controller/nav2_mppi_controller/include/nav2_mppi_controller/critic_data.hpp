// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_

#include <memory>
#include <vector>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/motion_models.hpp"


namespace mppi
{

/**
 * @struct mppi::CriticData
 * @brief 传递给 critics 的评分数据，包括状态、轨迹、路径、成本以及需要共享的重要参数
 */
struct CriticData
{
  const models::State & state;  ///< 本轮优化使用的机器人状态和控制样本。
  const models::Trajectories & trajectories;  ///< 待评分的候选轨迹集合。
  const models::Path & path;  ///< 转换到控制坐标系的参考路径。

  xt::xtensor<float, 1> & costs;  ///< 各候选轨迹的累计成本。
  float & model_dt;  ///< 运动模型的积分时间间隔。

  bool fail_flag;  ///< 是否所有候选轨迹均不可用。
  nav2_core::GoalChecker * goal_checker;  ///< 当前导航任务的目标检查器。
  std::shared_ptr<MotionModel> motion_model;  ///< 当前底盘运动模型。
  std::optional<std::vector<bool>> path_pts_valid;  ///< 参考路径点有效性缓存。
  std::optional<size_t> furthest_reached_path_point;  ///< 候选轨迹到达的最远路径点缓存。
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
