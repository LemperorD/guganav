// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief 按轨迹终点到目标位置的距离评分
 */
class GoalCritic : public CriticFunction
{
public:
  /**
    * @brief 初始化目标位置 critic 参数
    */
  void initialize() override;

  /**
   * @brief 计算目标位置代价并累加到总成本
   * @param data: 评分上下文，其中的成本张量将被就地更新
   */
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float threshold_to_consider_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_
