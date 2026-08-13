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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__VELOCITY_DEADBAND_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__VELOCITY_DEADBAND_CRITIC_HPP_

#include <vector>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::VelocityDeadbandCritic
 * @brief 对落入底盘不可执行速度死区的控制量施加代价
 */
class VelocityDeadbandCritic : public CriticFunction
{
public:
  /**
    * @brief 初始化速度死区 critic 参数
   */
  void initialize() override;

  /**
   * @brief 计算速度死区代价并累加到总成本
   * @param data: 评分上下文，其中的成本张量将被就地更新
   */
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};  ///< 速度死区代价的幂次。
  float weight_{0};  ///< 速度死区代价的权重。
  std::vector<float> deadband_velocities_{0.0f, 0.0f, 0.0f};  ///< 三个控制维度的死区阈值。
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__VELOCITY_DEADBAND_CRITIC_HPP_
