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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief 按轨迹末端到前视路径点的距离评分，实现近似路径跟随
 * 该 critic 允许轨迹为绕障偏离路径；精确贴合由 PathAlignCritic 控制。
 * 当前视偏移大于 1 时，提高权重会推动采样更快达到全速并产生一定切弯效果。
 */
class PathFollowCritic : public CriticFunction
{
public:
  /**
    * @brief 初始化路径跟随 critic 参数
    */
  void initialize() override;

  /**
   * @brief 计算前视路径点跟随代价并累加到总成本
   * @param data: 评分上下文，其中的成本张量将被就地更新
   */
  void score(CriticData & data) override;

protected:
  float threshold_to_consider_{0};  ///< 停止计算路径跟随代价的近目标距离。
  size_t offset_from_furthest_{0};  ///< 最远到达路径点之后的前视点偏移量。

  unsigned int power_{0};  ///< 路径跟随代价的幂次。
  float weight_{0};  ///< 路径跟随代价的权重。
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_
