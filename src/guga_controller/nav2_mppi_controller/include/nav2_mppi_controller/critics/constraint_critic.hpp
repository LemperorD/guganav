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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief 对违反底盘可行速度约束的轨迹施加代价
 */
class ConstraintCritic : public CriticFunction
{
public:
  /**
    * @brief 初始化约束 critic 参数
    */
  void initialize() override;

  /**
   * @brief 计算速度约束代价并累加到总成本
   * @param data: 评分上下文，其中的成本张量将被就地更新
   */
  void score(CriticData & data) override;

  /**
   * @brief 获取当前最大纵向速度约束
   * @return 返回值: 最大速度约束，供单元测试验证参数加载结果
   */
  float getMaxVelConstraint() {return max_vel_;}

  /**
   * @brief 获取当前最小纵向速度约束
   * @return 返回值: 最小速度约束，供单元测试验证参数加载结果
   */
  float getMinVelConstraint() {return min_vel_;}

protected:
  unsigned int power_{0};  ///< 速度约束代价的幂次。
  float weight_{0};  ///< 速度约束代价的权重。
  float min_vel_;  ///< 合成平移速度的允许下限。
  float max_vel_;  ///< 合成平移速度的允许上限。
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
