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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief 在大角度偏离或转弯场景下按轨迹朝向与路径方向评分
 */
class PathAngleCritic : public CriticFunction
{
public:
  /**
    * @brief 初始化路径角度 critic 参数
    */
  void initialize() override;

  /**
   * @brief 计算路径角度代价并累加到总成本
   * @param data: 评分上下文，其中的成本张量将被就地更新
   */
  void score(CriticData & data) override;

protected:
  float max_angle_to_furthest_{0};
  float threshold_to_consider_{0};

  size_t offset_from_furthest_{0};
  bool reversing_allowed_{true};
  bool forward_preference_{true};

  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_
