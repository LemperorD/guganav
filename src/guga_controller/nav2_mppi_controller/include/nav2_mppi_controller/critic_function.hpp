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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/critic_data.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CollisionCost
 * @brief 保存单个位姿碰撞代价及 footprint 检查状态
 */
struct CollisionCost
{
  float cost{0};  ///< 位姿对应的代价地图值。
  bool using_footprint{false};  ///< 该代价是否通过机器人轮廓计算。
};

/**
 * @class mppi::critics::CriticFunction
 * @brief 对候选轨迹评分的 critic 抽象接口
 */
class CriticFunction
{
public:
  /**
    * @brief 构造 critic 基类
    */
  CriticFunction() = default;

  /**
    * @brief 析构 critic 基类
    */
  virtual ~CriticFunction() = default;

  /**
    * @brief 配置 critic 并读取公共参数
    * @param parent: 控制器服务器生命周期节点的弱引用
    * @param parent_name: 所属控制器名称
    * @param name: critic 插件实例名称
    * @param costmap_ros: 提供环境代价地图的对象
    * @param param_handler: 动态参数处理器
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & parent_name,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * param_handler)
  {
    parent_ = parent;
    logger_ = parent_.lock()->get_logger();
    name_ = name;
    parent_name_ = parent_name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    parameters_handler_ = param_handler;

    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(enabled_, "enabled", true);

    initialize();
  }

  /**
    * @brief 计算本 critic 的轨迹代价并累加到总成本
    * @param data: 包含状态、轨迹、路径和累计成本的评分上下文
    */
  virtual void score(CriticData & data) = 0;

  /**
    * @brief 初始化具体 critic 的参数和缓存
    */
  virtual void initialize() = 0;

  /**
    * @brief 获取 critic 插件实例名称
    * @return 返回值: critic 名称，供日志、诊断和插件管理使用
    */
  std::string getName()
  {
    return name_;
  }

protected:
  bool enabled_;  ///< 是否启用当前评分器。
  std::string name_, parent_name_;  ///< 评分器实例名称和所属控制器名称。
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;  ///< 控制器服务器生命周期节点。
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  ///< 代价地图接口。
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};  ///< 代价地图数据的非持有指针。

  ParametersHandler * parameters_handler_;  ///< 动态参数处理器的非持有指针。
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};  ///< 评分器日志记录器。
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
