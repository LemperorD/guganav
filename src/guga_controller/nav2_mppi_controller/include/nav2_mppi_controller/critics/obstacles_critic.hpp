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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_

#include <memory>
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief 按障碍距离评价轨迹并允许为绕障偏离参考路径
 * 该 critic 应与 PathAlignCritic 配合调参，以平衡路径跟踪精度和动态绕障能力。
 */
class ObstaclesCritic : public CriticFunction
{
public:
  /**
    * @brief 初始化障碍距离 critic 参数和碰撞检查器
    */
  void initialize() override;

  /**
   * @brief 计算障碍距离代价并累加到总成本
   * @param data: 评分上下文，其中的成本和碰撞标志将被就地更新
   */
  void score(CriticData & data) override;

protected:
  /**
    * @brief 判断代价值是否表示碰撞
    * @param cost: 代价地图中的栅格代价值
    * @return 返回值: 碰撞时为 true，评分器据此标记轨迹无效
    */
  inline bool inCollision(float cost) const;

  /**
    * @brief 查询给定机器人位姿的障碍代价和碰撞状态
    * @param x: 位姿 X 坐标
    * @param y: 位姿 Y 坐标
    * @param theta: 位姿航向角
    * @return 返回值: 碰撞代价结构，供距离反算和轨迹碰撞标记使用
    */
  inline CollisionCost costAtPose(float x, float y, float theta);

  /**
    * @brief 根据膨胀代价反算到最近障碍的距离
    * @param cost: 位姿碰撞代价结构
    * @return 返回值: 障碍距离，供障碍接近代价累计使用
    */
  inline float distanceToObstacle(const CollisionCost & cost);

  /**
    * @brief 计算可能需要完整 footprint 碰撞检查的最小膨胀代价值
    * @param costmap: 提供膨胀层参数和机器人半径的代价地图对象
    * @return 返回值: 外接圆临界代价；高于该值时评分器会执行完整 footprint 碰撞检查
    */
  float findCircumscribedCost(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap);

protected:
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  collision_checker_{nullptr};

  bool consider_footprint_{true};
  float collision_cost_{0};
  float inflation_scale_factor_{0}, inflation_radius_{0};

  float possibly_inscribed_cost_;
  float collision_margin_distance_;
  float near_goal_distance_;
  float circumscribed_cost_{0}, circumscribed_radius_{0};

  unsigned int power_{0};
  float repulsion_weight_, critical_weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_
