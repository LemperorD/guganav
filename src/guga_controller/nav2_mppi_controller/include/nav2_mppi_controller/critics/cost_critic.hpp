// Copyright (c) 2023 Robocc Brice Renaudeau
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CostCritic
 * @brief 使用代价地图膨胀代价值评价候选轨迹的避障安全性
 */
class CostCritic : public CriticFunction
{
public:
  /**
    * @brief 初始化代价地图 critic 参数和碰撞检查器
    */
  void initialize() override;

  /**
   * @brief 计算障碍代价并累加到总成本
   * @param data: 评分上下文，其中的成本和碰撞标志将被就地更新
   */
  void score(CriticData & data) override;

protected:
  /**
    * @brief 判断给定位姿是否发生碰撞
    * @param cost: 位姿中心对应的代价值
    * @param x: 位姿 X 坐标
    * @param y: 位姿 Y 坐标
    * @param theta: 位姿航向角
    * @return 返回值: 碰撞时为 true，评分器据此标记整条候选轨迹无效
    */
  bool inCollision(float cost, float x, float y, float theta);

  /**
    * @brief 查询机器人位姿中心的代价值
    * @param x: 位姿 X 坐标
    * @param y: 位姿 Y 坐标
    * @return 返回值: 代价值，供碰撞判定和障碍惩罚计算使用
    */
  float costAtPose(float x, float y);

  /**
    * @brief 计算可能需要完整 footprint 碰撞检查的最小膨胀代价值
    * @param costmap: 提供膨胀层参数和机器人半径的代价地图对象
    * @return 返回值: 外接圆临界代价；高于该值时评分器会执行完整 footprint 碰撞检查
    */
  float findCircumscribedCost(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap);

protected:
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  collision_checker_{nullptr};
  float possibly_inscribed_cost_;

  bool consider_footprint_{true};
  float circumscribed_radius_{0};
  float circumscribed_cost_{0};
  float collision_cost_{0};
  float critical_cost_{0};
  float weight_{0};

  float near_goal_distance_;
  std::string inflation_layer_name_;

  unsigned int power_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
