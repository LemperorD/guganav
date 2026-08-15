// Copyright (c) 2023 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief 按候选轨迹与参考路径的横向偏差评分
 * 较高权重会提高路径贴合精度，但也会削弱绕开动态障碍的能力，
 * 因此应与障碍 critic 的权重配合调整。
 */
class PathAlignCritic : public CriticFunction
{
public:
  /**
    * @brief 初始化路径对齐 critic 参数
    */
  void initialize() override;

  /**
   * @brief 计算路径对齐代价并累加到总成本
   * @param data: 评分上下文，其中的成本张量将被就地更新
   */
  void score(CriticData & data) override;

protected:
  size_t offset_from_furthest_{0};  ///< 最远到达路径点之后额外考虑的点数。
  int trajectory_point_step_{0};  ///< 候选轨迹参与对齐评分的采样间隔。
  float threshold_to_consider_{0};  ///< 停止计算路径对齐代价的近目标距离。
  float max_path_occupancy_ratio_{0};  ///< 允许局部路径被障碍占用的最大比例。
  bool use_path_orientations_{false};  ///< 是否在对齐距离中加入路径航向误差。
  unsigned int power_{0};  ///< 路径对齐代价的幂次。
  float weight_{0};  ///< 路径对齐代价的权重。
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
