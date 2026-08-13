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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_LEGACY_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_LEGACY_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::PathAlignLegacyCritic
 * @brief 路径对齐 critic 的旧版实现
 * 较高权重会提高路径贴合精度，但也会削弱绕开动态障碍的能力，
 * 因此应与障碍 critic 的权重配合调整。本类保留 2023 年 10 月前的算法行为。
 */
class PathAlignLegacyCritic : public CriticFunction
{
public:
  /**
    * @brief 初始化旧版路径对齐 critic 参数
    */
  void initialize() override;

  /**
   * @brief 使用旧版算法计算路径对齐代价并累加到总成本
   * @param data: 评分上下文，其中的成本张量将被就地更新
   */
  void score(CriticData & data) override;

protected:
  size_t offset_from_furthest_{0};
  int trajectory_point_step_{0};
  float threshold_to_consider_{0};
  float max_path_occupancy_ratio_{0};
  bool use_path_orientations_{false};
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_LEGACY_CRITIC_HPP_
