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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief 对倒车运动施加代价以偏好向前行驶
 */
class PreferForwardCritic : public CriticFunction
{
public:
  /**
    * @brief 初始化前向偏好 critic 参数
    */
  void initialize() override;

  /**
   * @brief 计算倒车惩罚并累加到总成本
   * @param data: 评分上下文，其中的成本张量将被就地更新
   */
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};  ///< 倒车惩罚代价的幂次。
  float weight_{0};  ///< 倒车惩罚代价的权重。
  float threshold_to_consider_{0};  ///< 停止施加前向偏好代价的近目标距离。
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_
